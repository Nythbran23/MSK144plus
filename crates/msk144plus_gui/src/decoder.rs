// crates/msk144plus_gui/src/decoder.rs
//
// Slot accumulator and decoder driver.
//
// MSK144 transmits in slot-aligned bursts. Slot length is configurable:
//  - 15 s — WSJT-X / US convention (slot boundaries at UTC seconds
//    0, 15, 30, 45)
//  - 30 s — IARU Region 1 specification for 144 MHz (boundaries at
//    UTC seconds 0, 30)
// Both ends of a QSO must use the same period. We accumulate audio
// into a slot-length buffer (15 × 12 kHz = 180 000 samples for 15 s
// or 30 × 12 kHz = 360 000 samples for 30 s) and at slot boundaries
// hand the buffer to engine::decode_slot. Decodes are sent over a
// channel back to the UI thread.

use std::sync::mpsc::{Receiver, Sender};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;
use msk144plus_engine::{Depth, DecodeEvent, ShortMessageConfig};
use dx_runtime::{Database, DecodeRecord, Recorder, Paths};
use crate::audio::AudioChunk;

pub const SAMPLE_RATE: usize = 12000;
/// Default slot capacity in samples — matches the IARU R1 30s slot.
/// Used only as the initial buffer reserve; the actual slot length
/// is taken from `DecoderConfig::slot_period_secs` at runtime so it
/// can switch between 15s and 30s without restart.
pub const DEFAULT_SLOT_SAMPLES: usize = 30 * SAMPLE_RATE; // 360000

/// Compute the slot length in samples for a given slot period in seconds.
/// Used by the framer to know how big a slot to drain.
pub fn slot_samples_for_period(period_secs: u32) -> usize {
    period_secs as usize * SAMPLE_RATE
}

/// One decode produced by the engine, plus the slot timestamp it came from.
#[derive(Debug, Clone)]
pub struct UiDecodeEvent {
    pub slot_utc: String,        // "HHMMSS"
    pub event: DecodeEvent,
    pub fc_hz: f32,              // recorded so the UI can show absolute freq
}

/// Live signal level estimate (RMS in int16 amplitude units), updated as
/// audio chunks arrive.
#[derive(Debug, Clone, Copy)]
pub struct LevelStats {
    pub rms: f32,
    pub peak: f32,
}

/// Decoder configuration shared between UI and worker via mutex/atomic.
#[derive(Debug, Clone)]
pub struct DecoderConfig {
    pub fc_hz: f32,
    pub ntol_hz: f32,
    pub depth: Depth,
    pub mycall: String,
    pub hiscall: String,
    pub msk40_enabled: bool,
    /// True while we are actively transmitting. Historically used by
    /// the framer to mark slots as TX-tainted via
    /// `tx_active_seen_in_buf`; now that the audio pipeline is fully
    /// torn down at TX start (see app.rs::tear_down_audio_pipeline),
    /// the framer thread isn't even running during TX, so this flag
    /// is informational only.
    pub tx_active: bool,
    /// If Some, each audio chunk is also written to this WAV file.
    /// The UI sets this to Some(path) when "Record" is on, None when off.
    /// The decoder thread opens/closes the WavWriter based on transitions.
    pub record_path: Option<std::path::PathBuf>,
    /// Enable the soft-bit accumulator decoder path. Runs in parallel
    /// with the standard SPD/avg-N pipeline; emits decode events with
    /// `is_accumulated: true` so the UI can mark them with ` [A]`. Off
    /// by default — when off, accumulator code is skipped entirely.
    pub accumulator_enabled: bool,
    /// Tighter ±frequency tolerance (Hz) used by the accumulator when
    /// `freq_lock_hz` is Some. Suppresses non-partner traffic from
    /// polluting soft-bit sums during an active QSO. When `freq_lock_hz`
    /// is None, accumulator uses the wider `ntol_hz`.
    pub accumulator_ntol_hz: f32,
    /// Partner's measured audio-frequency offset from `fc_hz` during an
    /// active QSO, EMA-smoothed across successive partner decodes.
    /// `None` means no lock (use `ntol_hz` as the wide window);
    /// `Some(df)` means accumulator filters fragments to ±accumulator_ntol_hz
    /// around `df`. Set/cleared from the UI thread; the decoder reads it
    /// when feeding candidates to the accumulator.
    pub freq_lock_hz: Option<f32>,

    /// Slot length in seconds. 15 (WSJT-X / US default) or 30 (IARU
    /// Region 1 specification). Determines how many audio samples we
    /// accumulate before handing a slot to the decoder worker, AND
    /// the boundary alignment for slot-end timestamps. Both ends of
    /// a QSO must use the same value.
    pub slot_period_secs: u32,
}

impl Default for DecoderConfig {
    fn default() -> Self {
        Self {
            fc_hz: 1500.0,
            ntol_hz: 200.0,
            depth: Depth::Deep,
            mycall: String::new(),
            hiscall: String::new(),
            msk40_enabled: false,
            tx_active: false,
            record_path: None,
            accumulator_enabled: false,
            accumulator_ntol_hz: 50.0,
            freq_lock_hz: None,
            slot_period_secs: 30,
        }
    }
}

/// Job sent from the slot-framer thread to the decoder-worker thread.
/// Bundles everything `process_slot` needs to run a single decode pass
/// without further locking. The slot timestamp is captured by the
/// framer at the moment the slot fills (`now_ms` snapped to /15000), so
/// a backed-up worker can't misattribute audio to a later wall-clock
/// time when it eventually gets to it.
///
/// Two related timestamps are carried for audit purposes:
///   - `slot_end_ms`     — the snapped UTC slot boundary (slot identity)
///   - `buffer_start_ms` — the raw capture time of the FIRST sample in
///     this slot's audio buffer (pre-snap)
/// Their difference (`snap_offset_ms = buffer_start_ms - slot_start_ms`)
/// should be small (≪ slot length) in steady state. Large or growing
/// values indicate either an audio underrun or a slot-attribution bug.
struct SlotJob {
    slot: Vec<f32>,
    cfg: DecoderConfig,
    utc: String,         // "HHMMSS"
    utc_iso: String,     // "YYYY-MM-DDTHH:MM:SSZ"
    slot_end_ms: i64,    // snapped UTC end of slot — for capture_path_at + ping audit
    buffer_start_ms: i64, // raw capture time of first sample (pre-snap), for audit
}

/// Run the full per-slot decode pipeline: standard SPD decode,
/// accumulator pass, DB writes, auto-WAV save, and forward UI events.
/// Pure function over the captured Arcs — safe to call from a worker
/// thread.
fn process_slot(
    job: SlotJob,
    decode_tx: &Sender<UiDecodeEvent>,
    db: Option<&Arc<Database>>,
    recorder: Option<&Arc<Recorder>>,
    paths: Option<&Paths>,
) {
    let SlotJob {
        slot, cfg, utc, utc_iso, slot_end_ms, buffer_start_ms,
    } = job;

    // Slot-level provenance audit. Greppable as `[SLOT]` per slot:
    //   - slot_start_ms / slot_end_ms : the snapped UTC slot bounds
    //     (this is the slot identity used for all attribution)
    //   - buffer_start_ms             : raw capture time of the FIRST
    //     audio sample in this slot's buffer
    //   - snap_offset_ms              : buffer_start_ms - slot_start_ms.
    //     In steady state this should be tiny (typically < ~50 ms);
    //     persistent or growing values point to an audio underrun
    //     OR a misalignment between the framer's capture-time anchor
    //     and the actual sample rate / chunk arrival cadence.
    // Together these let us verify that every per-ping epoch (logged
    // below in [PING]) falls inside [slot_start_ms, slot_end_ms).
    let slot_len_ms = cfg.slot_period_secs as i64 * 1000;
    let slot_start_ms = slot_end_ms - slot_len_ms;
    let snap_offset_ms = buffer_start_ms - slot_start_ms;
    log::info!(
        "[SLOT] utc={} slot_start_ms={} slot_end_ms={} \
         buffer_start_ms={} snap_offset_ms={} slot_len_ms={} \
         fc={:.0} ntol={:.0}",
        utc,
        slot_start_ms, slot_end_ms,
        buffer_start_ms, snap_offset_ms, slot_len_ms,
        cfg.fc_hz, cfg.ntol_hz,
    );

    // Note: there is no `cfg.tx_active` early-return here. The framer
    // thread doesn't ship slots that overlapped TX (it's torn down
    // before TX starts and rebuilt after — see app.rs), so anything
    // arriving at process_slot is by definition a clean RX slot. An
    // earlier version checked cfg.tx_active here, but the SlotJob's
    // cfg snapshot is taken at framer drain time; if drain races
    // with TX start at a slot boundary, the snapshot can capture
    // tx_active=true even though every chunk that filled the slot
    // was clean RX, leading to RX slots being silently dropped.

    let short_cfg = if cfg.msk40_enabled
        && !cfg.mycall.is_empty() && !cfg.hiscall.is_empty()
    {
        Some(ShortMessageConfig {
            mycall: cfg.mycall.clone(),
            hiscall: cfg.hiscall.clone(),
            enabled: true,
        })
    } else {
        None
    };
    log::info!("[DECODE] running slot {} (fc={} ntol={})",
        utc, cfg.fc_hz, cfg.ntol_hz);
    let (events, diag) = msk144plus_engine::decode_slot_with_diag(
        &slot,
        cfg.ntol_hz,
        cfg.fc_hz,
        cfg.depth,
        short_cfg.as_ref(),
    );
    log::info!("[DECODE] slot {}: {} decode(s)", utc, events.len());
    if events.is_empty() && (diag.n_msk144_cands > 0 || diag.n_msk40_cands > 0) {
        log::info!(
            "[NO-DECODE] slot {}: msk144_cands={} msk40_cands={} \
             strongest_detmet={:.1} best_xmax={:.2} \
             best_nbadsync={} best_nharderror={}",
            utc,
            diag.n_msk144_cands,
            diag.n_msk40_cands,
            diag.strongest_detmet,
            diag.best_xmax,
            diag.best_n_bad_sync.map_or("—".into(), |b| b.to_string()),
            diag.best_n_hard_errors.map_or("—".into(), |e| e.to_string()),
        );
    }

    let process_event = |e: msk144plus_engine::DecodeEvent| {
        // Per-decode provenance audit. For every event we log:
        //   - t_in_slot     : the engine's reported burst position
        //     within this slot (None for averaging/accumulator decodes)
        //   - ping_epoch_ms : ABSOLUTE UTC epoch of the burst,
        //     computed as slot_start_ms + slot_position_secs * 1000
        //   - epoch_slot_utc: the slot HHMMSS that ping_epoch_ms
        //     falls into, computed independently from epoch alone
        //   - match         : epoch_slot_utc == utc (the slot label
        //     we're about to attach). FALSE here means slot
        //     attribution is broken — a ping is being labelled with
        //     the wrong slot. Loud warning fires in that case.
        // For accumulator/averaging decodes, slot_position_secs is
        // None by construction (no single time-position), so the
        // cross-check is skipped and we mark match=n/a.
        let (ping_epoch_str, epoch_slot_str, match_str, attribution_ok) =
            match e.slot_position_secs {
                Some(t) => {
                    let ping_epoch_ms = slot_start_ms + (t * 1000.0) as i64;
                    // Re-derive the slot label from the epoch alone:
                    // floor to slot boundary (= slot start), then
                    // format. Must match the framer's labelling
                    // convention exactly — which is now slot-START
                    // (WSJT-X / MSHV / MSK2K). If we change the
                    // framer's convention, this must change too.
                    let epoch_slot_start =
                        ping_epoch_ms.div_euclid(slot_len_ms) * slot_len_ms;
                    let (epoch_utc, _) = fmt_utc_ms(epoch_slot_start);
                    let ok = epoch_utc == utc;
                    (
                        ping_epoch_ms.to_string(),
                        epoch_utc,
                        ok.to_string(),
                        ok,
                    )
                }
                None => ("—".into(), "—".into(), "n/a".into(), true),
            };
        log::info!(
            "[PING] slot={} t_in_slot={} ping_epoch_ms={} \
             epoch_slot_utc={} match={} df={:+.0}Hz xmax={:.2} \
             method={} accum={} text=\"{}\"",
            utc,
            e.slot_position_secs.map_or("—".into(),
                |t| format!("{:.2}s", t)),
            ping_epoch_str,
            epoch_slot_str,
            match_str,
            e.freq_offset,
            e.xmax,
            e.method,
            e.is_accumulated,
            e.text,
        );
        if !attribution_ok {
            log::warn!(
                "[PING] !!! SLOT ATTRIBUTION MISMATCH: decode labelled \
                 slot={} but ping epoch falls in slot={} \
                 (ping_epoch_ms={}, slot_start_ms={}, slot_end_ms={}, \
                 t_in_slot={:?}). This indicates a slot-boundary or \
                 capture-time bug; PSK Reporter spots will be wrong.",
                utc, epoch_slot_str, ping_epoch_str,
                slot_start_ms, slot_end_ms, e.slot_position_secs,
            );
        }

        if let Some(db_ref) = db {
            let parts: Vec<&str> = e.text.split_whitespace().collect();
            // Extract the SENDER's callsign (the station we actually
            // heard). Used for the DB row, the heard-stations table,
            // and the WAV filename. Three message shapes to handle:
            //
            //   "CQ <call> <grid>"      → <call>
            //   "<to> <from> <report>"  → <from>     (NOT <to> — they
            //                                         were never heard;
            //                                         only the from-
            //                                         station's signal
            //                                         reached us)
            //   "<to> <...> <report>"   → None       (SH format: sender's
            //                                         callsign hashed,
            //                                         we don't know who
            //                                         transmitted)
            //
            // An earlier version of this function used "first non-CQ
            // token" which incorrectly returned <to> for two-call
            // messages, leading to WAVs filed under the recipient's
            // call and PSK Reporter / heard-call tracking spotting
            // a station we never heard. Fixed to mirror app.rs's
            // extract_callsign rules.
            let callsign: Option<String> = if !parts.is_empty()
                && parts[0].eq_ignore_ascii_case("CQ")
            {
                // CQ form — first callsign-shaped token after CQ
                parts.iter().skip(1).find(|t| {
                    let s = t.as_bytes();
                    s.len() <= 10
                        && t.chars().any(|c| c.is_ascii_alphabetic())
                        && t.chars().any(|c| c.is_ascii_digit())
                        && t.chars().all(|c| c.is_ascii_alphanumeric() || c == '/')
                }).map(|s| s.to_uppercase())
            } else if parts.len() >= 2
                // Reject if SENDER position contains an SH hash.
                && !(parts[1].starts_with('<') && parts[1].ends_with('>'))
            {
                // <to> <from> ... form — return <from> if it looks
                // callsign-shaped, else fall through to None.
                let cand = parts[1];
                if cand.len() <= 10
                    && cand.chars().any(|c| c.is_ascii_alphabetic())
                    && cand.chars().any(|c| c.is_ascii_digit())
                    && cand.chars().all(|c| c.is_ascii_alphanumeric() || c == '/')
                {
                    Some(cand.to_uppercase())
                } else {
                    None
                }
            } else {
                None
            };
            let grid: Option<String> = parts.last()
                .filter(|s| s.len() == 4
                    && s.chars().take(2).all(|c| c.is_ascii_uppercase())
                    && s.chars().skip(2).all(|c| c.is_ascii_digit()))
                .map(|s| (*s).to_string());
            let rec = DecodeRecord {
                utc: utc_iso.clone(),
                mode: "MSK144".into(),
                fc_hz: cfg.fc_hz,
                freq_offset_hz: e.freq_offset,
                xmax: e.xmax,
                method: e.method.clone(),
                text: e.text.clone(),
                callsign: callsign.clone(),
                grid,
                wav_path: None,
                band_mhz: None,
            };
            match db_ref.record_decode(&rec) {
                Ok(decode_id) => {
                    if let Some(c) = callsign.as_deref() {
                        let _ = db_ref.record_heard_call(
                            c, rec.grid.as_deref(), &utc_iso);
                    }
                    if let (Some(rec_h), Some(p)) = (recorder, paths) {
                        let label = callsign.as_deref().unwrap_or("decode");
                        if let Ok(wav_path) = p.capture_path_at(&utc, label) {
                            match rec_h.trigger_save(wav_path.clone()) {
                                Ok(_) => {
                                    let _ = db_ref.update_wav_path(
                                        decode_id,
                                        &wav_path.to_string_lossy());
                                }
                                Err(err) => log::warn!(
                                    "[REC] trigger_save failed: {}", err),
                            }
                        }
                    }
                }
                Err(err) => log::warn!("[DB] record_decode failed: {}", err),
            }
        }
        let _ = decode_tx.send(UiDecodeEvent {
            slot_utc: utc.clone(),
            event: e,
            fc_hz: cfg.fc_hz,
        });
    };

    // Capture the set of texts SPD/avg already produced for this slot
    // so the accumulator block below can flag whether each [A] decode
    // is genuinely UNIQUE (SPD missed it — adds value) or a duplicate
    // confirmation. This is the "is the accumulator earning its keep?"
    // diagnostic. Texts are captured BEFORE process_event is called
    // because process_event consumes the DecodeEvent by value.
    let spd_texts: std::collections::HashSet<String> = events.iter()
        .map(|e| e.text.clone()).collect();

    for e in events {
        process_event(e);
    }

    if cfg.accumulator_enabled {
        // ───── ACCUM-TEST GATES ─────────────────────────────────────
        // Strict baseline (Roger ran this for ~7 days): XMIN=1.0,
        // MIN_FRAGS=2 → out of 445 successful accum decodes, only 2
        // were unique vs SPD. To probe whether accum can add value
        // in marginal conditions, we lower these now. Pair the change
        // with the lib.rs ACCUM-TEST gates (envelope=0.10, coverage
        // halved). The [ACCUM] log lines below tag each decode as
        // "UNIQUE" or "dup-of-SPD" so the next session's data tells
        // us definitively whether the relaxed gates produce genuine
        // new decodes or just more duplicates / false positives.
        //
        // To restore strict mode: XMIN=1.0, MIN_FRAGS=2.
        const ACCUM_XMIN: f32 = 0.5;
        const ACCUM_MIN_FRAGS: usize = 1;
        // ────────────────────────────────────────────────────────────

        let fragments = msk144plus_engine::collect_softbit_fragments(
            &slot,
            cfg.ntol_hz,
            cfg.fc_hz,
            cfg.freq_lock_hz,
            cfg.accumulator_ntol_hz,
            ACCUM_XMIN,
        );
        log::info!(
            "[ACCUM] slot={} slot_start_ms={} slot_end_ms={} \
             collected {} fragment(s){}",
            utc, slot_start_ms, slot_end_ms,
            fragments.len(),
            cfg.freq_lock_hz.map_or(String::new(),
                |df| format!(" (lock={:+.0} ±{:.0} Hz)",
                    df, cfg.accumulator_ntol_hz))
        );

        let accum_decodes = msk144plus_engine::accumulator_decode(
            &fragments, ACCUM_MIN_FRAGS);
        if !accum_decodes.is_empty() {
            log::info!(
                "[ACCUM] slot={} jigsaw produced {} decode(s) from \
                 {} fragments",
                utc, accum_decodes.len(), fragments.len());
            for evt in accum_decodes {
                let unique_tag = if spd_texts.contains(&evt.text) {
                    "dup-of-SPD"
                } else {
                    "UNIQUE"
                };
                log::info!(
                    "[ACCUM] slot={} {} → {} (method={} df={:+.0}Hz)",
                    utc, unique_tag, evt.text, evt.method, evt.freq_offset);
                process_event(evt);
            }
        }
    }

    // slot_end_ms / buffer_start_ms / slot_start_ms are all consumed
    // above by the [SLOT] and [PING] audit logs.
}

/// Spawn the decoder worker thread + slot framer thread. Both run
/// for the lifetime of the listening session and exit naturally
/// when their input channels close (audio_rx closes when the
/// app's tee thread drops its sender; slot_rx closes when this
/// function's slot_tx goes out of scope).
///
/// Earlier this function returned the framer's JoinHandle so the
/// caller could synchronously join it during a tear-down at every
/// TX boundary. That pattern hung the UI on macOS because dropping
/// `cpal::Stream` is asynchronous — the callback's tx clone might
/// outlive the drop call by an indeterminate amount, leaving the
/// tee's capture_rx blocked, which left the framer's audio_rx
/// blocked, which made the join hang forever. We've moved to the
/// FSK441+ pattern: keep the audio pipeline alive across TX on
/// macOS / Windows; only Linux's half-duplex ALSA needs explicit
/// stream release. Nothing in this layer joins anymore.
pub fn run_decoder_thread(
    audio_rx: Receiver<AudioChunk>,
    decode_tx: Sender<UiDecodeEvent>,
    level_tx: Sender<LevelStats>,
    config: std::sync::Arc<std::sync::Mutex<DecoderConfig>>,
    db: Option<Arc<Database>>,
    recorder: Option<Arc<Recorder>>,
    paths: Option<Paths>,
    session_active: Arc<AtomicBool>,
) {
    // Bounded slot-job channel from framer → worker. Capacity 4 is
    // ample: a 15-s slot fills in 15 wall-clock seconds, and a worker
    // that can't keep up with that pace has bigger problems than
    // backpressure. If the channel is full when we try to ship a
    // slot, the framer logs a warning and DROPS the slot rather than
    // blocking — protecting real-time audio capture at the cost of
    // a missed decode (which is still better than running multiple
    // slots behind wall clock).
    let (slot_tx, slot_rx) = std::sync::mpsc::sync_channel::<SlotJob>(4);

    // Spawn the worker thread that runs decode + DB + UI dispatch.
    // It owns the heavy work; the framer thread (below) only does
    // light per-chunk operations (level, recorder, slot accumulation)
    // so it always keeps pace with the audio stream.
    {
        let decode_tx_w = decode_tx.clone();
        let db_w = db.clone();
        let recorder_w = recorder.clone();
        let paths_w = paths.clone();
        let active_w = session_active.clone();
        std::thread::Builder::new()
            .name("msk144-decoder-worker".into())
            .spawn(move || {
                // recv_timeout pattern so we can check the session
                // flag periodically. On Stop the framer's flag
                // also goes false, and after the framer exits
                // slot_rx will close and we'd exit on Disconnected
                // anyway — but the timeout-+-flag combo is the
                // belt-and-braces version that exits even if the
                // framer somehow hangs.
                loop {
                    match slot_rx.recv_timeout(Duration::from_millis(200)) {
                        Ok(job) => {
                            process_slot(
                                job,
                                &decode_tx_w,
                                db_w.as_ref(),
                                recorder_w.as_ref(),
                                paths_w.as_ref(),
                            );
                        }
                        Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                            if !active_w.load(Ordering::Acquire) {
                                log::info!(
                                    "[DECODE] worker exiting (session inactive)");
                                return;
                            }
                        }
                        Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {
                            log::info!(
                                "[DECODE] worker exiting (slot_rx closed)");
                            return;
                        }
                    }
                }
            })
            .expect("spawn decoder worker");
    }

    // Slot framer thread. Receives audio chunks from cpal callback,
    // updates levels and recorder, accumulates slots, ships each
    // filled slot to the worker. NEVER blocks on heavy work — even
    // when the worker is mid-decode, this thread keeps draining
    // audio so the OS never has to backpressure the input device.
    //
    // Lives for the entire listening session — exits naturally when
    // its audio_rx returns Err (which happens at process exit when
    // the app's tee thread finally drops its sender, or on Linux at
    // explicit pipeline teardown), OR when the per-session
    // `session_active` flag flips false (the cpal callback drops
    // chunks at source, and the recv_timeout below catches the
    // flag transition within ~200 ms).
    let active_framer = session_active.clone();
    std::thread::Builder::new()
        .name("msk144-decoder".into())
        .spawn(move || {
            let mut slot_buf: Vec<f32> = Vec::with_capacity(DEFAULT_SLOT_SAMPLES + 4096);
            // Slot identity comes from the AUDIO CAPTURE time of the
            // FIRST sample in the current slot buffer — NOT the
            // wall-clock time at ship time. The decoder runs LDPC
            // for several seconds per pass and can shift slots' real-
            // time alignment, so snapping `now()` at ship time was
            // attributing some slots to the wrong window. Anchoring
            // on capture time is the correct invariant: the slot a
            // burst belongs to is determined by when the audio came
            // in, irrespective of when we got around to decoding it.
            //
            // `slot_capture_start_ms` is the UTC ms when the FIRST
            // sample of the slot currently in `slot_buf` was captured.
            // None when the buffer is empty (next chunk arrival sets
            // it). On every drain we reset to None so the next chunk
            // re-anchors.
            let mut slot_capture_start_ms: Option<i64> = None;

            // WAV recorder state. We track the path that's currently open and
            // close/reopen on changes.
            let mut current_writer:
                Option<hound::WavWriter<std::io::BufWriter<std::fs::File>>> = None;
            let mut current_path: Option<std::path::PathBuf> = None;

            // Helper that determines whether we should treat the
            // current state as "audio source has gone away" — used
            // by both the timeout-with-flag path and the
            // disconnected-channel path. Sharing the flush logic
            // between them is what would otherwise need a helper
            // function; this just sets a flag and the existing
            // Err-arm code reads it.
            #[derive(PartialEq)]
            enum Source { Live, Stopped, Disconnected }

            loop {
                // recv_timeout so we can periodically check the
                // session flag. On Stop the cpal callback's flag
                // also goes false, so chunks stop flowing — but we
                // can't rely on audio_rx ever returning Err on
                // macOS (the cpal Stream isn't synchronously
                // dropped), so the timeout path is the canonical
                // way to notice Stop and exit cleanly.
                let recv = audio_rx.recv_timeout(Duration::from_millis(200));
                let source = match &recv {
                    Ok(_) => Source::Live,
                    Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                        if active_framer.load(Ordering::Acquire) {
                            // Live session, just no chunk this
                            // tick. Loop and try again.
                            continue;
                        }
                        Source::Stopped
                    }
                    Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {
                        Source::Disconnected
                    }
                };

                let (chunk, capture_unix_ms) = match recv {
                    Ok(AudioChunk { samples, capture_unix_ms }) => {
                        (samples, capture_unix_ms)
                    }
                    Err(_) => {
                        // Audio channel closed. Two cases bring us here:
                        //   (a) STOP — full session shutdown. The user
                        //       clicked Stop; nothing more to do.
                        //   (b) TX teardown — the cpal input was torn
                        //       down so the device can be re-claimed
                        //       by the output side. Coming with this
                        //       close is whatever audio was in
                        //       slot_buf for the slot that just ended.
                        //       MSK2K's `FlushAndStop` pattern: drain
                        //       what we have, ship it to the worker,
                        //       then exit. The worker thread is
                        //       independent and keeps running across
                        //       framer restarts; its output shows up
                        //       on the UI whenever the decode finishes
                        //       (typically during the TX that follows).
                        //
                        // For the partial-slot flush to produce a
                        // valid SlotJob, we need:
                        //   - slot_capture_start_ms (already aligned)
                        //   - slot_buf with at least 1 second of audio
                        //     (anything shorter is too brief to
                        //     produce a useful decode and isn't worth
                        //     the worker round-trip)
                        //   - pad slot_buf up to a full slot's length
                        //     with trailing silence so process_slot's
                        //     fixed-length DSP doesn't panic on a
                        //     short Vec
                        if let Some(capture_start) = slot_capture_start_ms {
                            let cfg = config.lock().unwrap().clone();
                            let cur_slot_samples =
                                slot_samples_for_period(cfg.slot_period_secs);
                            let min_samples = SAMPLE_RATE; // 1 second
                            if slot_buf.len() >= min_samples {
                                let real_samples = slot_buf.len();
                                if slot_buf.len() < cur_slot_samples {
                                    let pad = cur_slot_samples - slot_buf.len();
                                    slot_buf.resize(slot_buf.len() + pad, 0.0);
                                }
                                let slot: Vec<f32> = slot_buf
                                    .drain(..cur_slot_samples).collect();
                                let slot_len_ms =
                                    cfg.slot_period_secs as i64 * 1000;
                                let slot_start_aligned =
                                    (capture_start / slot_len_ms) * slot_len_ms;
                                let slot_end_ms = slot_start_aligned + slot_len_ms;
                                let (utc, utc_iso) = fmt_utc_ms(slot_start_aligned);
                                log::info!(
                                    "[FRAMER] flush-on-close: shipping \
                                     partial slot {} ({} real samples, \
                                     padded to {}) to worker before exit",
                                    utc, real_samples, cur_slot_samples,
                                );
                                let job = SlotJob {
                                    slot,
                                    cfg,
                                    utc,
                                    utc_iso,
                                    slot_end_ms,
                                    buffer_start_ms: capture_start,
                                };
                                // try_send only — worker may be
                                // backlogged, in which case we lose
                                // this one slot. Acceptable trade.
                                let _ = slot_tx.try_send(job);
                            }
                        }

                        match source {
                            Source::Stopped =>
                                log::info!("[FRAMER] exiting (session stopped)"),
                            Source::Disconnected =>
                                log::info!("[FRAMER] exiting (audio_rx closed)"),
                            Source::Live => {} // unreachable
                        }
                        // Finalise any open WAV
                        if let Some(w) = current_writer.take() {
                            let _ = w.finalize();
                        }
                        return;
                    }
                };

                // ── Capture-clock gap detection ─────────────────────
                // Defensive — under the current Option B architecture
                // (audio pipeline torn down at every TX boundary)
                // this should never fire because the framer thread
                // exits cleanly when audio_rx closes, taking its
                // slot_buf with it. But if a future change ever
                // enables continuous-capture-during-TX, or if cpal
                // ever drops chunks mid-stream for any other reason,
                // this catches the "stale anchor + post-gap chunks"
                // failure mode where slot identity would otherwise
                // drift perpetually.
                //
                // Measurement uses cpal's CAPTURE timestamp (not
                // chrono::Utc::now() at recv time) so it's immune
                // to framer-thread scheduling jitter. 500 ms threshold
                // sits well above normal cpal jitter (~100 ms) and
                // well below any real audio gap.
                const GAP_THRESHOLD_MS: i64 = 500;
                if let Some(anchor) = slot_capture_start_ms {
                    let buffered_ms = (slot_buf.len() as i64 * 1000)
                        / SAMPLE_RATE as i64;
                    let expected_ms = anchor + buffered_ms;
                    let gap_ms = capture_unix_ms - expected_ms;
                    if gap_ms > GAP_THRESHOLD_MS {
                        log::warn!(
                            "[FRAMER] capture-clock gap detected: {} ms \
                             (anchor={} buf_len={} expected={} actual={}); \
                             discarding stale buffer and re-anchoring",
                            gap_ms, anchor, slot_buf.len(),
                            expected_ms, capture_unix_ms);
                        slot_buf.clear();
                        slot_capture_start_ms = None;
                    }
                }

                // ── Slot-boundary alignment ─────────────────────────
                // At startup (and after any drain that empties the
                // buffer, OR a gap-detection clear above),
                // `slot_capture_start_ms` is None. The very first
                // sample we accumulate into `slot_buf` MUST land
                // exactly at a wall-clock slot boundary, otherwise the
                // slot label and the audio it contains drift apart by
                // a fixed, lifelong offset.
                //
                // Background: the framer drains exactly one slot's
                // worth (e.g. 360_000 samples for 30 s) per pass.
                // Whatever offset the FIRST aligned sample has
                // relative to the wall-clock slot grid propagates to
                // every subsequent slot — there is no self-correcting
                // mechanism. So we MUST get the first sample's
                // alignment right at startup.
                //
                // The wall-clock anchor is `capture_unix_ms` from the
                // AudioChunk: cpal's authoritative claim about when
                // the FIRST sample of this chunk was captured by the
                // hardware. This is robust against OS scheduling
                // jitter and any audio-pipeline buffering between
                // the device and our handler — accuracy depends only
                // on the one-time calibration done in audio.rs at
                // first callback, plus the cpal monotonic clock.
                let chunk_to_buffer: &[f32] = if slot_capture_start_ms.is_some() {
                    // Already aligned, accumulate full chunk.
                    &chunk[..]
                } else {
                    let first_sample_ms = capture_unix_ms;
                    let slot_len_ms_align =
                        config.lock().unwrap().slot_period_secs as i64 * 1000;
                    let into_slot_ms = first_sample_ms.rem_euclid(slot_len_ms_align);

                    // Snap DOWN to the slot boundary that's already
                    // passed. The current slot started `into_slot_ms`
                    // ago. Pad slot_buf with leading silence so its
                    // drain timing matches wall-clock; real audio
                    // appended after the pad decodes normally.
                    //
                    // Earlier strategy was to wait for the NEXT slot
                    // boundary before accepting samples — but on the
                    // half-duplex CODEC each TX cycle restarts cpal
                    // mid-RX-slot, so waiting for the next boundary
                    // (which coincides with the next TX) means we
                    // throw away the RX slot entirely. Snap-down
                    // recovers the partial slot.
                    let snapped_start_ms = first_sample_ms - into_slot_ms;
                    slot_capture_start_ms = Some(snapped_start_ms);

                    let target_pad_samples = ((into_slot_ms
                        * SAMPLE_RATE as i64) / 1000) as usize;
                    if target_pad_samples > 0 {
                        slot_buf.resize(target_pad_samples, 0.0);
                    }

                    log::info!(
                        "[FRAMER] aligned via snap-down to slot {} \
                         ({}); padded {} sample(s) of silence \
                         (into-slot offset {}ms)",
                        snapped_start_ms,
                        fmt_utc_ms(snapped_start_ms).0,
                        target_pad_samples,
                        into_slot_ms,
                    );
                    &chunk[..]
                };

                // Update level stats
                if !chunk.is_empty() {
                    let sum_sq: f32 = chunk.iter().map(|x| x * x).sum();
                    let rms = (sum_sq / chunk.len() as f32).sqrt();
                    let peak = chunk.iter().map(|x| x.abs())
                        .fold(0.0f32, f32::max);
                    let _ = level_tx.send(LevelStats { rms, peak });
                }

                // Push every chunk into the rolling pre-roll buffer (drives
                // auto-WAV-on-decode). Recorder also writes any pending saves
                // whose post-roll has filled.
                if let Some(rec) = recorder.as_ref() {
                    rec.push_audio(&chunk);
                }

                // Check recorder state
                let want_path = config.lock().unwrap().record_path.clone();
                match (&current_path, &want_path) {
                    (None, Some(p)) => {
                        // Start recording
                        match open_wav_writer(p) {
                            Ok(w) => {
                                current_writer = Some(w);
                                current_path = Some(p.clone());
                                log::info!("[REC] started → {}", p.display());
                            }
                            Err(e) => {
                                log::error!("[REC] failed to open {}: {}", p.display(), e);
                            }
                        }
                    }
                    (Some(_), None) => {
                        // Stop recording
                        if let Some(w) = current_writer.take() {
                            let _ = w.finalize();
                            log::info!("[REC] stopped");
                        }
                        current_path = None;
                    }
                    (Some(curr), Some(p)) if curr != p => {
                        // Path changed - close and reopen
                        if let Some(w) = current_writer.take() {
                            let _ = w.finalize();
                        }
                        match open_wav_writer(p) {
                            Ok(w) => {
                                current_writer = Some(w);
                                current_path = Some(p.clone());
                                log::info!("[REC] re-opened → {}", p.display());
                            }
                            Err(e) => {
                                log::error!("[REC] reopen failed: {}", e);
                                current_path = None;
                            }
                        }
                    }
                    _ => {} // No change
                }

                // Write chunk to WAV if recording
                if let Some(w) = current_writer.as_mut() {
                    for &sample in &chunk {
                        // Audio is already in int16-equivalent f32 scale
                        let s_i16 = sample.clamp(-32767.0, 32767.0) as i16;
                        let _ = w.write_sample(s_i16);
                    }
                    // Periodic flush (every chunk - it's still buffered by BufWriter)
                    let _ = w.flush();
                }

                slot_buf.extend_from_slice(chunk_to_buffer);

                // Drain any complete slots. Read slot_period_secs each
                // pass — operator may flip 15s ↔ 30s mid-session.
                let cur_slot_samples =
                    slot_samples_for_period(config.lock().unwrap().slot_period_secs);
                while slot_buf.len() >= cur_slot_samples {
                    let slot: Vec<f32> = slot_buf.drain(..cur_slot_samples).collect();
                    let cfg = config.lock().unwrap().clone();

                    // Slot identity from the CAPTURE time of the first
                    // sample of this slot. Snap to the slot boundary
                    // to get the canonical UTC slot start. The slot
                    // covers [slot_start_aligned, slot_end_ms), and
                    // we tag with the START time — matches WSJT-X /
                    // MSHV / MSK2K convention.
                    let slot_len_ms = cfg.slot_period_secs as i64 * 1000;
                    let capture_start = slot_capture_start_ms
                        .unwrap_or_else(|| chrono::Utc::now().timestamp_millis());
                    let slot_start_aligned =
                        (capture_start / slot_len_ms) * slot_len_ms;
                    let slot_end_ms = slot_start_aligned + slot_len_ms;
                    let (utc, utc_iso) = fmt_utc_ms(slot_start_aligned);

                    // Re-anchor for the NEXT slot.
                    slot_capture_start_ms = if slot_buf.is_empty() {
                        None
                    } else {
                        Some(slot_end_ms)
                    };

                

                    // Ship the slot to the decoder-worker thread.
                    // try_send (not send) because we must not block:
                    // the audio stream is still feeding us chunks
                    // and we have to keep draining. If the worker
                    // is more than 4 slots behind we drop the slot
                    // and log.
                    let job = SlotJob {
                        slot,
                        cfg,
                        utc: utc.clone(),
                        utc_iso,
                        slot_end_ms,
                        buffer_start_ms: capture_start,
                    };
                    match slot_tx.try_send(job) {
                        Ok(()) => {}
                        Err(std::sync::mpsc::TrySendError::Full(_)) => {
                            log::warn!(
                                "[DECODE] worker backlog full; dropping slot {} \
                                 (decode falling behind real time)", utc);
                        }
                        Err(std::sync::mpsc::TrySendError::Disconnected(_)) => {
                            log::error!(
                                "[DECODE] worker thread gone; framer exiting");
                            return;
                        }
                    }
                }
            }
        })
        .expect("spawn decoder thread");
}

fn open_wav_writer(path: &std::path::Path)
    -> anyhow::Result<hound::WavWriter<std::io::BufWriter<std::fs::File>>>
{
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let spec = hound::WavSpec {
        channels: 1,
        sample_rate: SAMPLE_RATE as u32,
        bits_per_sample: 16,
        sample_format: hound::SampleFormat::Int,
    };
    Ok(hound::WavWriter::create(path, spec)?)
}

/// Format a unix-epoch-millisecond UTC value into both the compact
/// "HHMMSS" form (used for the user-visible slot label) and the full
/// ISO-8601 "YYYY-MM-DDTHH:MM:SSZ" form (used for SQLite records).
/// Returned as a tuple so callers don't have to format twice.
fn fmt_utc_ms(utc_ms: i64) -> (String, String) {
    let dt = chrono::DateTime::<chrono::Utc>::from_timestamp_millis(utc_ms)
        .unwrap_or_else(chrono::Utc::now);
    let hhmmss = dt.format("%H%M%S").to_string();
    let iso = dt.format("%Y-%m-%dT%H:%M:%SZ").to_string();
    (hhmmss, iso)
}