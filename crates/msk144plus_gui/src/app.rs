// crates/msk144plus_gui/src/app.rs
//
// MSK144+ RX GUI. Visual layout mirrors MSK2K's gui/app.rs so users moving
// between modes see the same chassis with one mode selector flipped.
//
// Panel layout (top-to-bottom):
//   top_bar       — callsign | freq | BAND | MODE | UTC clock | ⚙
//   actions       — 📻 LISTEN | 📢 CALL CQ | TARGET: <call> | CALL | (right) STOP / IN QSO
//   central       — 3 columns: 📥 RX | 📤 TX | 🎯 SPOTS
//   log_footer    — Logbook ▶/▼ expandable QSO table
//   status_strip  — STATE | RMS | Quality
//
// MSK2K vs MSK144+ visual differences:
//   - MODE label = "MSK144"
//   - Reports use MSK144 format (e.g. "+10") rendered straight from decode
//   - PERIOD selector on top bar — 15s or 30s (default 30s for IARU R1)
//
// Compared to MSK2K source we currently:
//   - Have RX working via the decoder thread
//   - Show LISTEN as functional, other action buttons present but stubbed
//   - Logbook table renders empty until QSO state machine implemented
//   - Settings dialog minimal (audio device + decoder params; hamlib later)

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{channel, Receiver, Sender};

use eframe::egui;
use msk144plus_engine::Depth;
use dx_runtime::{Paths, Settings, Database, Recorder};
use dx_runtime::adif::AdifLogger;
use dx_runtime::qso::{QsoEngine, QsoState, Intent, EngineEvent, Action};
use dx_runtime::proto::{self, RxEnvelope, TxEnvelope, render_payload_with_sh, Rendered};

use crate::audio::{
    list_input_devices, default_input_device_name, list_all_devices_diagnostic,
    start_capture, AudioChunk, CaptureHandle,
};
use crate::decoder::{
    DecoderConfig, LevelStats, UiDecodeEvent, run_decoder_thread,
};

#[derive(Clone)]
struct LogEntry {
    text: String,
    colored: bool,
    timestamp: String,
    #[allow(dead_code)]
    rx_slot: Option<u8>,
    /// Signal-to-noise ratio (dB) for the decode this row represents,
    /// computed from the engine's xmax via WSJT-X's formula. Used when
    /// the user clicks a CQ row to answer — we send back this value as
    /// the +NN report quantised to MSK144's 2-dB-step protocol range.
    /// `None` for rows that aren't decodes (e.g. TX entries, info messages).
    snr_db: Option<i16>,
}

pub struct App {
    cfg: Arc<Mutex<DecoderConfig>>,

    // Runtime infrastructure (paths, persisted settings, DB, recorder)
    paths: Paths,
    settings: Settings,
    settings_dirty: bool,
    db: Option<Arc<Database>>,
    recorder: Arc<Recorder>,

    my_call: String,
    their_call: String,
    /// Refresh-generation counter for the TARGET TextEdit's egui id.
    /// Incremented whenever `their_call` is updated programmatically
    /// by the QSO engine (TheirCallChanged event). The widget id is
    /// derived from this counter, so a bump forces egui to treat the
    /// TextEdit as a new widget on the next frame and re-read the
    /// bound string — preventing the stale-cache issue where a
    /// partner cold-calling us wouldn't visibly populate the field.
    target_field_refresh_gen: u32,
    /// Partner's Maidenhead locator (4 or 6 chars). Set automatically
    /// when the user clicks a CQ row that included a grid; cleared
    /// when their_call changes to a different station. Used by the
    /// distance/bearing/scatter display next to the TARGET field.
    their_grid: Option<String>,
    band: String,
    rig_freq_hz: Option<u64>,
    /// Base frequency (Hz) at the time of first CAT connect this
    /// session. Used as the centre of the ±250 kHz clamp on the
    /// click-to-edit kHz field — prevents an accidental triple-tap
    /// from QSY'ing across an entire band. Set lazily; cleared on
    /// disconnect.
    base_freq_hz: Option<u64>,
    /// True while the user has the kHz field active and is typing
    /// a new value. While true, the field renders as an active
    /// TextEdit (amber); otherwise it renders as a click-to-edit
    /// label (matches FSK441Plus's style).
    freq_editing: bool,
    /// The 3-digit kHz value being typed. Pre-populated with the
    /// current kHz on click so the field is never blank.
    freq_edit: String,

    fc_hz: f32,
    ntol_hz: f32,
    depth: Depth,

    /// Operator's typed report value for the manual-TX override
    /// dropdown. Used for Tx2 (sending +rpt) and Tx3 (sending R+rpt).
    /// Range matches protocol: -4..=24, in 2 dB steps. Stored as the
    /// raw integer; the dB string is rendered when the field is edited.
    /// Defaults to 0 (= "+00") on app start; persists across slot
    /// override selections so the operator only types it once for a
    /// given QSO.
    manual_tx_rpt: i16,
    /// Text-edit buffer for the report field on the top bar. Allows
    /// negative values and 1-2 digits. Parsed back into manual_tx_rpt
    /// on focus loss / Enter.
    manual_tx_rpt_str: String,

    /// True when the window is too narrow to render the full top bar
    /// on a single line. Computed at the start of each update from
    /// `ctx.screen_rect().width()`. When true, the TX-control cluster
    /// (parity + Manual TX + Rpt + Sh) skips its inline render on
    /// row 1 and is instead rendered on a dedicated overflow panel
    /// inserted between row 1 and the actions strip.
    compact_top_bar: bool,

    is_listening: bool,
    is_calling_cq: bool,
    in_active_qso: bool,
    is_transmitting: bool,

    available_devices: Vec<String>,
    selected_device: Option<String>,
   capture_handle: Option<CaptureHandle>,
    audio_tx: Option<Sender<AudioChunk>>,

    /// Per-listening-session liveness flag. Set to a fresh
    /// `Arc<AtomicBool>(true)` in `start_listening`, flipped to
    /// `false` in `tear_down_audio_pipeline`. Cloned into:
    ///   - the cpal callback closures (audio.rs) — drops chunks at
    ///     source when false
    ///   - the audio tee thread (this file) — exits its forwarding
    ///     loop when false
    ///   - the framer + decoder-worker threads (decoder.rs) — exit
    ///     cleanly when false
    ///
    /// This is the ONLY reliable way to halt the audio pipeline on
    /// macOS, because `cpal::Stream::drop` is asynchronous: dropping
    /// the stream does not synchronously stop the CoreAudio
    /// callback, and the callback can keep firing for an
    /// indeterminate window after drop. Without this flag, an old
    /// session's callbacks would keep delivering chunks to leaked
    /// tee/framer threads, while the new session spawns its own —
    /// resulting in TWO framer threads decoding the same audio
    /// (the doubled `[SLOT]` lines and `[REC] saved` × 2 we used
    /// to see after Stop+Listen).
    ///
    /// Each new session gets a FRESH Arc — old workers see their
    /// old flag stay false and exit. New workers run with the new
    /// flag. No shared mutable state across sessions.
    session_active: Option<Arc<AtomicBool>>,

    decode_rx: Option<Receiver<UiDecodeEvent>>,
    level_rx: Option<Receiver<LevelStats>>,

    /// Realtime spectrum display state.
    /// Each column is one FFT snapshot; the vector accumulates within
    /// a single 15s slot and resets at slot boundaries. Frozen during
    /// our own TX (we don't want our outgoing audio painted on the
    /// spectrum, even if it leaked through monitor).
    spectrum_columns: Vec<crate::spectrum::SpectrumColumn>,
    /// Largest column count observed in any complete slot this
    /// session. Used as the divisor when laying out the spectrum
    /// width so columns fill the full panel regardless of the
    /// actual audio chunk size produced by the resampler.
    ///
    /// The static-formula estimate (15 s × 12 kHz / 1024 ≈ 176)
    /// only holds when the input device runs at exactly 12 kHz
    /// natively. With resampling from 11.025 kHz on most Mac
    /// CODECs, the rubato resampler produces chunks of ~1115
    /// samples which yields ~161 columns per slot — leaving ~9 %
    /// of the panel width empty if we use the static estimate.
    /// This field tracks the actual observed maximum so the
    /// layout self-calibrates after the first complete slot.
    spectrum_max_cols_seen: usize,
    /// Slot index at which the columns were last cleared. Increment
    /// triggers a clear.
    spectrum_slot_idx: i64,
    /// Receiver from the spectrum worker thread; None when not listening.
    spectrum_rx: Option<Receiver<crate::spectrum::SpectrumColumn>>,
    /// Spectrum-side audio sender — clone into the cpal callback so
    /// audio chunks fan out to both the decoder and the spectrum worker.
    spectrum_audio_tx: Option<Sender<Vec<f32>>>,

    rx_log: Vec<LogEntry>,
    tx_log: Vec<LogEntry>,
    cq_log: Vec<LogEntry>,
    decode_counts: HashMap<String, u32>,
    qso_log_expanded: bool,

    last_level: LevelStats,
    last_corr: f32,
    current_state: String,

    settings_open: bool,
    show_audio_diag: bool,
    audio_diag_lines: Vec<String>,

    cat_connected: bool,

    // Hamlib (rigctld TCP client). Spawned on first use; receives freq
    // updates via dedicated channel. None when disabled.
    hamlib: Option<Arc<dx_runtime::HamlibClient>>,
    hamlib_rx: Option<std::sync::mpsc::Receiver<dx_runtime::HamlibUpdate>>,
    /// RAII guard for the rigctld child process. Dropping it kills rigctld.
    rigctld_guard: Option<dx_runtime::ProcessGuard>,
    /// Cached list of serial ports for the settings UI; refreshed when the
    /// settings dialog opens.
    available_serial_ports: Vec<String>,
    /// Cached list of audio output devices.
    available_output_devices: Vec<String>,

    // TX scheduler thread + event channel
    transmitter: Option<crate::transmitter::Transmitter>,
    tx_event_rx: Option<std::sync::mpsc::Receiver<crate::transmitter::TxEvent>>,

    /// PSK Reporter UDP client. Spawned on listener start when enabled
    /// in settings; dropped on listener stop or when the operator
    /// toggles the setting off. None when disabled or when we can't
    /// spawn (no callsign / no grid).
    psk_reporter: Option<Arc<dx_runtime::pskreporter::PskReporter>>,

    watchdog_enabled: bool,
    qso_started_at: Option<std::time::Instant>,

    /// Partner's measured audio-frequency offset from fc_hz during an
    /// active QSO, EMA-smoothed across successive decodes. None when
    /// QSO is Idle or no decode has been received yet. Pushed into
    /// `DecoderConfig.freq_lock_hz` so the decoder thread's accumulator
    /// can use it for narrow-window fragment filtering.
    qso_freq_lock_hz: Option<f32>,
    /// Wall-clock time of the most recent partner decode. Used to time
    /// out the freq lock when partner falls silent (so a re-emerging
    /// signal at slightly different frequency isn't excluded by stale
    /// lock). Cleared when `qso_freq_lock_hz` is cleared.
    qso_freq_last_partner_at: Option<std::time::Instant>,

    /// Histogram of decode counts keyed by slot parity (0 or 1). Used
    /// by the auto-parity detector: if all decodes consistently land
    /// in one parity, the operator's TX parity should be the OPPOSITE
    /// (so we don't TX over partner). Counts are decayed as they fill
    /// (halved when total ≥ 32) so the heuristic tracks recent
    /// activity rather than session totals.
    decode_parity_counts: [u32; 2],

    /// QSO state machine — pure logic, no UI/audio. Drives auto-call sequencing.
    qso: QsoEngine,
    /// Persistent ADIF logger (writes ~/msk144plus_log.adi on QsoComplete).
    adif: Option<Arc<AdifLogger>>,
    /// QSOs successfully written to ADIF this session (for status display)
    qso_session_count: u32,
    /// In-memory cache of logged QSOs for the Logbook panel. Populated
    /// at startup by parsing the ADIF file (so QSOs from previous
    /// sessions show up too) and appended to on every successful
    /// QsoComplete event. Newest at the front so the panel scrolls
    /// chronologically descending.
    logged_qsos: Vec<dx_runtime::QsoRecord>,

    /// Debounce timer for auto-save: when settings_dirty is true and this
    /// long has passed since the last save, the next update() will save.
    last_save_check: std::time::Instant,
}

impl Default for App {
    fn default() -> Self {
        // Fallback "no runtime injected" path. Used only by older entry
        // points; production uses with_runtime() from main.rs.
        let paths = Paths::new("msk144plus");
        let _ = paths.ensure_dirs();
        let settings = Settings::default();
        let recorder = Arc::new(Recorder::new(dx_runtime::SaveConfig {
            sample_rate: 12000,
            pre_roll_secs: 15,
            post_roll_secs: 15,
            captures_root: paths.captures_dir.clone(),
        }));
        Self::new_with(paths, settings, None, recorder)
    }
}

impl App {
    /// Construct with full runtime injected from main.rs.
    pub fn with_runtime(
        paths: Paths,
        settings: Settings,
        db: Option<Arc<Database>>,
        recorder: Arc<Recorder>,
    ) -> Self {
        // ADIF log path is owned by Paths so --config-dir moves it too.
        let adif = Arc::new(AdifLogger::new(paths.adif_file.clone(), "MSK144PLUS"));
        let mut app = Self::new_with(paths, settings, db, recorder);
        app.adif = Some(adif.clone());

        // Pre-populate the Logbook panel by parsing the existing ADIF
        // file. New QSOs appended in this session (via QsoComplete)
        // will be added to the front of the in-memory list as they
        // happen. Failures here are non-fatal — log a warning and
        // continue with an empty list (the panel will then only show
        // QSOs from the current session).
        match adif.read_all() {
            Ok(records) => {
                let n = records.len();
                // Newest-first ordering for the panel: ADIF is
                // chronologically ascending (oldest first), reverse
                // so the panel shows the most recent at the top.
                app.logged_qsos = records.into_iter().rev().collect();
                log::info!("[LOGBOOK] loaded {} QSO(s) from {}",
                    n, app.adif.as_ref().unwrap().path_display());
            }
            Err(e) => {
                log::warn!("[LOGBOOK] failed to load ADIF: {} (panel will start empty)", e);
            }
        }

        // Set engine's max_repeats to match the configured slot
        // period BEFORE any QSO state is initiated. (The Default::new
        // engine constructor uses 5; for 30s slots we want 3.)
        app.qso.set_slot_period(app.settings.station.slot_period_secs);
        // Auto-start capture on launch — user expects the app to begin
        // listening immediately. They can STOP if they need to reconfigure.
        app.start_listening();
        app
    }

    fn new_with(
        paths: Paths,
        settings: Settings,
        db: Option<Arc<Database>>,
        recorder: Arc<Recorder>,
    ) -> Self {
        // Build a DecoderConfig from settings so the decoder thread sees
        // the persisted values.
        let depth = match settings.decoder.depth.as_str() {
            "Fast" => Depth::Fast,
            "Normal" => Depth::Normal,
            _ => Depth::Deep,
        };
        let cfg = DecoderConfig {
            fc_hz: settings.decoder.fc_hz,
            ntol_hz: settings.decoder.ntol_hz,
            depth,
            mycall: settings.station.callsign.clone(),
            hiscall: String::new(),
            msk40_enabled: settings.decoder.msk40_enabled,
            tx_active: false,
            record_path: None,
            accumulator_enabled: settings.decoder.accumulator_enabled,
            accumulator_ntol_hz: settings.decoder.accumulator_ntol_hz,
            freq_lock_hz: None,
            slot_period_secs: settings.station.slot_period_secs,
        };
        let devs = list_input_devices();
        // Resolve selected_device: prefer settings, else default
        let selected_device = settings.audio.input_device.clone()
            .or_else(default_input_device_name);
        Self {
            fc_hz: cfg.fc_hz,
            ntol_hz: cfg.ntol_hz,
            depth: cfg.depth,
            cfg: Arc::new(Mutex::new(cfg)),
            paths,
            my_call: settings.station.callsign.clone(),
            their_call: String::new(),
            target_field_refresh_gen: 0,
            their_grid: None,
            band: settings.station.band.clone().unwrap_or_else(|| "2M".into()),
            rig_freq_hz: None,
            base_freq_hz: None,
            freq_editing: false,
            freq_edit: String::new(),
            manual_tx_rpt: 0,
            manual_tx_rpt_str: "+00".to_string(),
            compact_top_bar: false,
            is_listening: false,
            is_calling_cq: false,
            in_active_qso: false,
            is_transmitting: false,
            available_devices: devs,
            selected_device,
            capture_handle: None,
            audio_tx: None,
            session_active: None,
            decode_rx: None,
            level_rx: None,
            spectrum_columns: Vec::new(),
            spectrum_max_cols_seen: 0,
            spectrum_slot_idx: -1,
            spectrum_rx: None,
            spectrum_audio_tx: None,
            rx_log: Vec::new(),
            tx_log: Vec::new(),
            cq_log: Vec::new(),
            decode_counts: HashMap::new(),
            qso_log_expanded: settings.ui.logbook_expanded,
            last_level: LevelStats { rms: 0.0, peak: 0.0 },
            last_corr: 0.0,
            current_state: "Idle".into(),
            settings_open: false,
            show_audio_diag: false,
            audio_diag_lines: Vec::new(),
            cat_connected: false,
            hamlib: None,
            hamlib_rx: None,
            rigctld_guard: None,
            available_serial_ports: dx_runtime::list_serial_ports(),
            available_output_devices: dx_runtime::list_output_devices(),
            transmitter: None,
            tx_event_rx: None,
            psk_reporter: None,
            watchdog_enabled: true,
            qso_started_at: None,
            qso_freq_lock_hz: None,
            qso_freq_last_partner_at: None,
            decode_parity_counts: [0, 0],
            qso: {
                let mut q = QsoEngine::new(settings.station.callsign.clone());
                q.set_my_grid(settings.station.grid.clone());
                q.set_band(settings.station.band.clone().unwrap_or_else(|| "2M".into()));
                q
            },
            adif: None,  // resolved in with_runtime once paths are known
            qso_session_count: 0,
            logged_qsos: Vec::new(),
            settings,
            settings_dirty: false,
            db,
            recorder,
            last_save_check: std::time::Instant::now(),
        }
    }

    /// Persist current UI state into Settings + write TOML. Idempotent.
    fn save_settings(&mut self) {
        // Pull current UI state into self.settings
        self.settings.station.callsign = self.my_call.clone();
        self.settings.station.band = Some(self.band.clone());
        self.settings.audio.input_device = self.selected_device.clone();
        self.settings.decoder.fc_hz = self.fc_hz;
        self.settings.decoder.ntol_hz = self.ntol_hz;
        self.settings.decoder.depth = format!("{:?}", self.depth);
        self.settings.ui.logbook_expanded = self.qso_log_expanded;
        if let Err(e) = self.settings.save(&self.paths.config_file) {
            log::warn!("[CFG] save failed: {}", e);
        } else {
            log::info!("[CFG] saved → {}", self.paths.config_file.display());
            self.settings_dirty = false;
        }
    }

    fn start_listening(&mut self) {
        // Two guards. capture_handle catches the normal "already
        // listening" case. is_listening catches the failure path
        // where a previous start_capture failed: capture_handle
        // wasn't set, but a tee + framer were already spawned and
        // are still running. Without the is_listening guard, every
        // subsequent call to start_listening (e.g. from
        // TxEvent::Finished) would spawn ANOTHER tee + framer pair,
        // accumulating one per TX cycle until the system collapses
        // under the parallel decode load.
        if self.capture_handle.is_some() || self.is_listening { return; }

        // Audio capture pipeline:
        //
        //   cpal callback ──► capture_tx ──► tee thread ──┬──► audio_rx ──► decoder
        //                                                 └──► spec_rx   ──► spectrum
        //
        // The tee thread exists because cpal's callback is given a single
        // sender, and we need the same audio chunks to feed both the
        // decoder and the realtime spectrum display. Cloning the chunk
        // (~85ms × 12000Hz × 4B = 4 kB per send, ~12 sends/s = 48 kB/s
        // copy work) is well below noise. Trying to give cpal two
        // senders directly would require modifying audio.rs's API and
        // making the sound-card callback path twice as wide; the tee
        // keeps that callback simple.
        //
        // Channel types:
        //   - capture_tx / audio_tx carry `AudioChunk` (samples + cpal
        //     capture-time in Unix ms). The decoder framer needs the
        //     timestamp for slot-boundary alignment.
        //   - spec_audio_tx carries plain `Vec<f32>` — the spectrum
        //     widget only needs samples; the tee strips the timestamp.
        //
        // Lifecycle: opened once when listening starts, lives until
        // stop() is called. We do NOT tear down at TX boundaries on
        // macOS — the IC-9700 USB CODEC exposes (TX) and (RX) as
        // separate CoreAudio devices, so input and output never
        // collide. Matches FSK441+'s working pattern. On Linux the
        // half-duplex teardown is handled inside audio.rs's
        // spawn_holder via the stop_rx channel.
        //
        // Allocate a fresh per-session liveness flag. Cloned into
        // every worker that needs to know whether THIS session is
        // still alive. On Stop / tear_down we flip this Arc's
        // AtomicBool to false; old workers exit, the next Listen
        // creates a new Arc with a fresh `true` so new workers run
        // independently of the old ones. See the field-level
        // comment on `App.session_active` for the full story.
        let active = Arc::new(AtomicBool::new(true));

        let (capture_tx, capture_rx) = channel::<AudioChunk>();
        let (audio_tx, audio_rx) = channel::<AudioChunk>();
        let (spec_audio_tx, spec_audio_rx) = channel::<Vec<f32>>();
        let (spec_col_tx, spec_col_rx) = channel::<crate::spectrum::SpectrumColumn>();

        // Tee: forward each captured chunk to both the decoder and the
        // spectrum. Failure on either side just drops that downstream
        // (e.g. spectrum thread exits → we keep feeding the decoder).
        //
        // recv_timeout + session-flag check so we exit cleanly on
        // Stop. Without this the tee would wait forever on
        // capture_rx.recv() because the cpal callback's tx clone
        // doesn't drop synchronously on macOS — the channel never
        // closes — so a leaked tee thread would happily forward
        // chunks from the dying old cpal stream into a leaked
        // audio_tx clone, while the next Listen spawns ANOTHER
        // tee/framer pair. That's the doubled-slot bug.
        let active_tee = active.clone();
        std::thread::Builder::new()
            .name("msk144-audio-tee".into())
            .spawn(move || {
                use std::sync::mpsc::RecvTimeoutError;
                use std::time::Duration;
                loop {
                    match capture_rx.recv_timeout(Duration::from_millis(200)) {
                        Ok(chunk) => {
                            // Decoder gets the full timestamped chunk.
                            let samples_for_spec = chunk.samples.clone();
                            let _ = audio_tx.send(chunk);
                            // Spectrum just needs the audio samples.
                            let _ = spec_audio_tx.send(samples_for_spec);
                        }
                        Err(RecvTimeoutError::Timeout) => {
                            if !active_tee.load(Ordering::Acquire) {
                                log::info!(
                                    "[AUDIO-TEE] exiting (session stopped)");
                                return;
                            }
                        }
                        Err(RecvTimeoutError::Disconnected) => {
                            log::info!(
                                "[AUDIO-TEE] exiting (capture_rx closed)");
                            return;
                        }
                    }
                }
            })
            .expect("spawn audio tee");

        let (decode_tx, decode_rx) = channel::<UiDecodeEvent>();
        let (level_tx, level_rx) = channel::<LevelStats>();

        {
            let mut c = self.cfg.lock().unwrap();
            c.fc_hz = self.fc_hz;
            c.ntol_hz = self.ntol_hz;
            c.depth = self.depth;
            c.mycall = self.my_call.clone();
            c.slot_period_secs = self.settings.station.slot_period_secs;
        }
        run_decoder_thread(
            audio_rx, decode_tx, level_tx, self.cfg.clone(),
            self.db.clone(),
            Some(self.recorder.clone()),
            Some(self.paths.clone()),
            active.clone(),
        );
        crate::spectrum::run_spectrum_thread(spec_audio_rx, spec_col_tx);

        match start_capture(self.selected_device.clone(), capture_tx.clone(), active.clone()) {
            Ok(handle) => {
                self.current_state = handle.audio_info.clone();
                self.capture_handle = Some(handle);
                self.audio_tx = Some(capture_tx);
                self.session_active = Some(active);
                self.decode_rx = Some(decode_rx);
                self.level_rx = Some(level_rx);
                self.spectrum_rx = Some(spec_col_rx);
                self.spectrum_audio_tx = None;  // not used downstream; kept for symmetry
                self.is_listening = true;
                log::info!("[UI] Listening started");

                // Put the QSO engine into Listening state so it auto-
                // answers any direct call to us. Without this the engine
                // sits in Idle and ignores even valid <them> <me> messages.
                // We also need a transmitter ready in case auto-answer fires.
                self.ensure_transmitter();
                self.qso.set_my_call(self.my_call.clone());
                self.qso.set_my_grid(self.settings.station.grid.clone());
                self.qso.set_band(self.band.clone());
                self.qso.set_freq_mhz(
                    self.rig_freq_hz.map(|hz| hz as f64 / 1_000_000.0));
                // Only put the engine into Listening state if we're
                // currently idle. If we're already mid-call, mid-CQ,
                // or otherwise in a live exchange, don't disturb that
                // — Intent::Listen calls reset_qso() inside the engine,
                // which wipes their_call and tx_repeat_count and ends
                // up causing the engine's next_tx() to return None on
                // the next slot, so TX silently stops after one slot.
                // Listening is only relevant when we're idle and
                // waiting for someone to call us.
                let live_qso = matches!(
                    self.qso.state,
                    QsoState::CallingCq
                        | QsoState::CallingStn
                        | QsoState::SendingReport
                        | QsoState::SendingRReport
                        | QsoState::SendingRr
                        | QsoState::Sending73,
                );
                if !live_qso {
                    let (action, events) = self.qso.on_intent(Intent::Listen);
                    self.apply_engine_output(action, events);
                }

                // Spawn PSK Reporter client if enabled in settings
                // and we have the prerequisite station info. Skip
                // silently if disabled or under-configured — the
                // operator has full control via the Settings dialog.
                self.maybe_spawn_psk_reporter();
            }
            Err(e) => {
                // start_capture failed — but we already spawned the
                // tee, the decoder framer, and the decoder worker
                // above (all holding clones of `active`). Flip the
                // flag false so they exit cleanly within ~200 ms
                // instead of becoming orphans that consume CPU
                // until process exit. is_listening stays false so
                // the operator can retry via the LISTEN button.
                active.store(false, Ordering::Release);
                self.current_state = format!("Capture error: {}", e);
                log::error!("Capture error: {}", e);
            }
        }
    }

    /// Tear down ONLY the audio pipeline (cpal stream, tee thread,
    /// framer, decoder channels). Used by `stop()` for full session
    /// teardown.
    ///
    /// We do NOT call this at TX boundaries on macOS. The IC-9700
    /// (and other USB CODECs that expose separate `(TX)` / `(RX)`
    /// CoreAudio devices) handle full duplex transparently — the
    /// input stream stays alive across TX with no callback collision.
    /// Matches FSK441+'s working pattern.
    ///
    /// On Linux, where ALSA exposes the CODEC as a single half-duplex
    /// handle, the audio-output side itself orchestrates the input
    /// release via the `stop_rx` channel inside audio.rs::spawn_holder.
    /// That keeps the TX-cycle device juggling out of this layer.
    fn tear_down_audio_pipeline(&mut self) {
        // FIRST: flip the session-active flag. This is the canonical
        // signal that this listening session has ended — the cpal
        // callback closures see it on every chunk and start dropping
        // chunks at source; the tee/framer/decoder threads all
        // recv_timeout on a 200 ms tick and check the flag, exiting
        // cleanly. On macOS this is the ONLY reliable way to halt
        // the pipeline because cpal::Stream::drop is asynchronous —
        // the CoreAudio callback can keep firing for an
        // indeterminate window after we drop the stream below, so
        // relying on channel-close-propagation alone leaks workers.
        //
        // We `take()` the Arc out so the next start_listening
        // allocates a FRESH Arc<AtomicBool>(true) — the old workers
        // continue to see their old flag stay false (and exit), the
        // new workers run with the new flag, no shared state across
        // sessions.
        if let Some(active) = self.session_active.take() {
            active.store(false, Ordering::Release);
        }

        // Drop the cpal capture handle. On Linux this signals
        // spawn_holder's stop_rx to return Err, the holder thread
        // exits, the cpal::Stream drops synchronously, and the
        // ALSA handle is released. On macOS / Windows spawn_holder
        // parks forever (no recv), so dropping the handle here is
        // a no-op for the audio thread — but harmless because the
        // process is exiting or about to call start_listening
        // again. Importantly, on macOS we don't depend on this
        // drop to stop the audio: the session flag flipped above
        // already halted chunk delivery from the cpal callback.
        self.capture_handle = None;

        // Drop our own capture_tx clone. The cpal callback still
        // holds its own clone (especially on macOS where the
        // stream isn't being released), so capture_rx may NOT
        // return Err immediately — but the tee thread now exits
        // on the session_active flag check (200 ms tick) rather
        // than waiting for capture_rx to close. So this drop is
        // bookkeeping rather than load-bearing.
        self.audio_tx = None;

        // Drop the spectrum receiver. Spectrum thread exits on
        // its own when its input channel closes (which happens
        // when the tee exits — guaranteed within ~200 ms now
        // that the flag-driven exit path is in place).
        self.spectrum_rx = None;
        self.spectrum_audio_tx = None;
        self.spectrum_columns.clear();
        self.spectrum_slot_idx = -1;
    }

    fn stop(&mut self) {
        // Tell the engine we're stepping away from any active state —
        // it'll reset partner data and transition to Idle. Also clears
        // the accumulator's freq lock so a future session starts fresh.
        let (action, events) = self.qso.on_intent(Intent::Abort);
        self.apply_engine_output(action, events);
        self.clear_freq_lock();

        self.tear_down_audio_pipeline();
        // Full session teardown also drops the decoder result
        // receivers (kept across TX-cycle teardowns, but a Stop is
        // permanent until the user clicks Listen again). Clearing
        // these here so the worker thread sees its decode_tx channel
        // disconnect and exits cleanly.
        self.decode_rx = None;
        self.level_rx = None;
        self.is_listening = false;
        self.is_calling_cq = false;
        self.in_active_qso = false;
        self.qso_started_at = None;
        // Drop the PSK Reporter client — its worker thread will see
        // the channel closed and exit cleanly after a final flush.
        if let Some(reporter) = self.psk_reporter.take() {
            // We're holding an Arc — get the inner one and stop it.
            // If we can't (other refs exist somehow), just drop the
            // Arc and the worker will exit when it sees disconnect.
            if let Ok(inner) = Arc::try_unwrap(reporter) {
                inner.stop();
            }
        }
        self.current_state = "Idle".into();
        log::info!("[UI] Stopped");
    }

    /// Spawn rigctld (if auto-launch enabled and config is complete) and
    /// connect the hamlib worker. No-op if already running.
    fn start_hamlib(&mut self) {
        if self.hamlib.is_some() { return; }

        // Auto-launch rigctld as a child process if config is complete.
        if self.settings.station.auto_launch_rigctld
            && !self.settings.station.rig_model.is_empty()
            && !self.settings.station.rig_port.is_empty()
        {
            let opts = dx_runtime::RigctldOpts {
                model: self.settings.station.rig_model.clone(),
                port: self.settings.station.rig_port.clone(),
                baud: self.settings.station.rig_baud.parse().unwrap_or(19200),
                listen_port: self.settings.station.rigctld_port,
            };
            match dx_runtime::RigctldLauncher::launch(&opts) {
                Ok(guard) => {
                    log::info!("[UI] rigctld launched (pid={})", guard.pid());
                    self.rigctld_guard = Some(guard);
                    // Give rigctld time to bind its TCP listen port
                    std::thread::sleep(std::time::Duration::from_millis(800));
                }
                Err(e) => {
                    log::error!("[UI] rigctld launch failed: {}", e);
                    self.current_state = format!("rigctld launch failed: {}", e);
                    return;
                }
            }
        }

        let (tx, rx) = std::sync::mpsc::channel::<dx_runtime::HamlibUpdate>();
        let client = dx_runtime::HamlibClient::spawn(
            self.settings.station.rigctld_host.clone(),
            self.settings.station.rigctld_port,
            std::time::Duration::from_secs(5),
            tx,
        );
        self.hamlib = Some(Arc::new(client));
        self.hamlib_rx = Some(rx);
        log::info!("[UI] Hamlib client started → {}:{}",
            self.settings.station.rigctld_host,
            self.settings.station.rigctld_port);

        // Drop the existing transmitter (if any) so the next TX intent
        // rebuilds it with the fresh hamlib client. Without this, a
        // transmitter spawned BEFORE hamlib connected would hold a
        // None snapshot of self.hamlib forever — TX audio plays but
        // PTT is never asserted because the transmitter has no rig
        // handle. This was the root cause of the "CAT connected but
        // PTT not keying" regression. ensure_transmitter() is
        // idempotent and re-spawns lazily on next demand.
        if self.transmitter.is_some() {
            log::info!("[UI] Dropping stale transmitter; will re-spawn with fresh hamlib snapshot");
            self.transmitter = None;
            self.tx_event_rx = None;
        }
    }

    /// Drop the hamlib client and kill rigctld (if we launched it).
    fn stop_hamlib(&mut self) {
        if self.hamlib.is_some() {
            log::info!("[UI] Hamlib client stopped");
        }
        self.hamlib = None;
        self.hamlib_rx = None;
        // Dropping ProcessGuard sends T 0 then kills rigctld
        if self.rigctld_guard.is_some() {
            log::info!("[UI] Killing child rigctld");
            self.rigctld_guard = None;
        }
        self.cat_connected = false;
        self.rig_freq_hz = None;
        self.base_freq_hz = None;
        // Drop the transmitter too — it holds an Arc<HamlibClient>
        // that's now pointing at a worker we've just shut down. Next
        // TX intent will re-spawn the transmitter with whatever
        // hamlib state exists at that point (None, until the user
        // re-enables CAT).
        if self.transmitter.is_some() {
            log::info!("[UI] Dropping transmitter (hamlib stopped)");
            self.transmitter = None;
            self.tx_event_rx = None;
        }
    }

    /// Drain updates from hamlib worker; update rig_freq_hz / band /
    /// cat_connected accordingly.
    /// Spawn the TX scheduler thread (idempotent).
    fn ensure_transmitter(&mut self) {
        if self.transmitter.is_some() { return; }
        let (ev_tx, ev_rx) = std::sync::mpsc::channel::<crate::transmitter::TxEvent>();
        let initial = crate::transmitter::TxState {
            mode: crate::transmitter::TxMode::Idle,
            message: String::new(),
            fc_hz: self.fc_hz,
            slot_period_secs: self.settings.station.slot_period_secs,
            tx_parity: self.settings.station.tx_parity.clone(),
            ptt_delay_ms: self.settings.station.ptt_delay_ms,
            output_device: self.settings.station.tx_output_device.clone(),
            tx_level: self.settings.station.tx_level,
        };
        let tx = crate::transmitter::Transmitter::spawn(
            initial,
            self.hamlib.clone(),
            ev_tx,
        );
        self.transmitter = Some(tx);
        self.tx_event_rx = Some(ev_rx);
        log::info!("[UI] Transmitter scheduler started");
    }

    /// Push current TX-related settings into the running transmitter
    /// AND the QSO engine. Slot period affects both: the transmitter
    /// uses it for slot-boundary scheduling and audio length, the
    /// engine uses it to scale max_repeats so QSO timeouts feel
    /// similar in wall-clock seconds across 15s / 30s modes.
    fn sync_transmitter_config(&mut self) {
        // QSO engine first — adjusts max_repeats based on period.
        // Independent of whether the transmitter is up; even pre-TX
        // listening should have the right repeat budget set.
        self.qso.set_slot_period(self.settings.station.slot_period_secs);

        if let Some(tx) = &self.transmitter {
            tx.set_slot_config(
                self.settings.station.slot_period_secs,
                self.settings.station.tx_parity.clone(),
            );
            tx.set_output_config(
                self.settings.station.tx_output_device.clone(),
                self.settings.station.tx_level,
                self.settings.station.ptt_delay_ms,
            );
            tx.set_fc(self.fc_hz);
        }
    }

    /// Spawn the PSK Reporter client if enabled in settings and the
    /// prerequisite station info (callsign + grid) is configured. If
    /// already spawned, do nothing. Called from start_listening (so
    /// it auto-starts with the audio pipeline) and from the Settings
    /// dialog when the operator toggles the flag (so changes are live).
    ///
    /// Silently skips spawning when:
    ///   - psk_reporter_enabled is false
    ///   - callsign is empty (PSK Reporter requires sender ID)
    ///   - grid is empty or shorter than 4 chars
    /// In those cases the client stays None until the operator fixes
    /// the missing config and toggles the flag again.
    fn maybe_spawn_psk_reporter(&mut self) {
        if !self.settings.station.psk_reporter_enabled { return; }
        if self.psk_reporter.is_some() { return; }

        let call = self.settings.station.callsign.trim().to_string();
        if call.is_empty() {
            log::info!("[PSKR] not spawning: callsign is empty");
            return;
        }
        let grid = self.settings.station.grid.clone().unwrap_or_default();
        let grid = grid.trim().to_string();
        if grid.len() < 4 {
            log::info!("[PSKR] not spawning: grid '{}' too short (need 4 or 6 chars)", grid);
            return;
        }

        let antenna = self.settings.station.psk_reporter_antenna.clone();
        let program = format!("MSK144Plus {}", env!("CARGO_PKG_VERSION"));
        // Resolve the Hamlib model number (e.g. "3081") to its
        // human-readable name (e.g. "Icom IC-9700") for the PSK
        // Reporter rig info field. PSK Reporter shows this in the
        // station info popup; a bare model number isn't useful to
        // human readers. Falls back to the raw value if the model
        // isn't in our common-models lookup table (e.g. operator
        // typed in a custom Hamlib id directly).
        let rig_id = self.settings.station.rig_model.clone();
        let rig_info = if rig_id.is_empty() {
            String::new()
        } else {
            dx_runtime::common_rig_models().iter()
                .find(|(id, _)| *id == rig_id.as_str())
                .map(|(_, name)| name.to_string())
                .unwrap_or_else(|| rig_id.clone())
        };

        match dx_runtime::pskreporter::PskReporter::spawn(
            call.clone(), grid.clone(), antenna, program, rig_info,
        ) {
            Ok(reporter) => {
                self.psk_reporter = Some(Arc::new(reporter));
                log::info!("[PSKR] spawned (call={}, grid={})", call, grid);
            }
            Err(e) => {
                log::warn!("[PSKR] failed to spawn: {}", e);
            }
        }
    }

    /// Drop the PSK Reporter client. Called when the operator toggles
    /// the setting off. Worker thread does a final flush attempt then
    /// exits cleanly.
    fn stop_psk_reporter(&mut self) {
        if let Some(reporter) = self.psk_reporter.take() {
            // We may be the only Arc holder, in which case stop()
            // gives the worker a clean shutdown path. If other refs
            // exist (shouldn't, but defensively) we just drop ours.
            if let Ok(inner) = Arc::try_unwrap(reporter) {
                inner.stop();
                log::info!("[PSKR] client stopped (operator disabled)");
            }
        }
    }

    /// Begin calling CQ via the QSO engine.
    fn start_cq(&mut self) {
        // Ensure the audio pipeline is running. See start_call_target
        // for the reasoning — without RX, the QSO state machine has
        // no inputs, and the call/CQ TX repeats but never advances.
        if !self.is_listening {
            self.start_listening();
        }
        self.ensure_transmitter();
        self.qso.set_my_call(self.my_call.clone());
        self.qso.set_my_grid(self.settings.station.grid.clone());
        self.qso.set_band(self.band.clone());
        self.qso.set_freq_mhz(self.rig_freq_hz.map(|hz| hz as f64 / 1_000_000.0));

        let (action, events) = self.qso.on_intent(Intent::Cq);
        self.apply_engine_output(action, events);
        self.is_calling_cq = true;
        // is_transmitting stays false until TxEvent::Started arrives —
        // that's when we're actually keying/emitting audio.
        self.in_active_qso = false;
        self.their_call.clear();
        self.their_grid = None;
    }

    /// Cold-call a specific station: send "<them> <me>" with no report.
    /// Used for CALL button when the user has typed a callsign manually
    /// (a station they want to call but haven't heard CQ from).
    ///
    /// MSK2K convention: CALL button = cold call. Clicking a SPOT row that
    /// contains a CQ uses the answer_cq() path instead.
    fn start_call_target(&mut self) {
        if self.their_call.is_empty() {
            log::warn!("[UI] Call requested with empty TARGET; ignoring");
            return;
        }
        // Ensure the audio pipeline is running. Without it the
        // transmitter scheduler can fire a TX, but RX is dead — the
        // user can't hear partner replies, the QSO state machine
        // can't advance on RX events, and once the initial Active
        // message exhausts the engine emits None for next_tx() and
        // TX silently stops. Always bring up the listener at the
        // start of a call so RX + TX are both alive.
        if !self.is_listening {
            self.start_listening();
        }
        self.ensure_transmitter();
        self.qso.set_my_call(self.my_call.clone());
        self.qso.set_my_grid(self.settings.station.grid.clone());
        self.qso.set_band(self.band.clone());
        self.qso.set_freq_mhz(self.rig_freq_hz.map(|hz| hz as f64 / 1_000_000.0));

        let (action, events) = self.qso.on_intent(Intent::Call {
            their: self.their_call.clone(),
        });
        self.apply_engine_output(action, events);
        self.is_calling_cq = false;
        self.in_active_qso = true;
        // is_transmitting set later by TxEvent::Started
        self.qso_started_at = Some(std::time::Instant::now());
    }

    /// Operator-driven manual TX override. Forces the QSO engine into
    /// the chosen state and starts transmitting the corresponding
    /// message. Used for picking up a QSO mid-stream when auto-state
    /// can't infer where the partner is — e.g. you broke off, came
    /// back, and are hearing the partner's R-report waiting for your
    /// RR73.
    ///
    /// `state` must be one of: CallingStn, SendingReport,
    /// SendingRReport, SendingRr, Sending73, CallingCq. Other states
    /// are rejected by force_state and we get Action::None back.
    ///
    /// The partner call comes from the TARGET field (self.their_call)
    /// — the GUI's dropdown has already validated it isn't empty for
    /// states that require it. The report comes from the operator's
    /// typed manual_tx_rpt value.
    /// Render the operator's TX-control cluster: parity dropdown +
    /// auto-parity status indicator + Manual TX dropdown + Rpt input
    /// + Sh checkbox. These four widgets are always rendered together
    /// — they're the operator's per-slot controls and travel as a
    /// group between row 1 and the conditional "overflow" row 2 so
    /// the layout doesn't burst on narrow windows.
    ///
    /// Caller decides where to put the cluster — wide windows render
    /// it inline on row 1 (between MSK144 • 15s and the clock); narrow
    /// windows render it on its own row sandwiched between row 1 and
    /// the actions strip below.
    fn render_tx_controls_cluster(&mut self, ui: &mut egui::Ui) {
        // TX SLOT (parity) selector — coloured to match the parity
        // convention used in the RX/SPOTS rows. A small status
        // indicator alongside shows whether observed decode parity
        // is consistent with the configured setting.
        let saved_selection = ui.visuals().selection.bg_fill;
        let slate = egui::Color32::from_rgb(70, 90, 110);
        ui.visuals_mut().selection.bg_fill = slate;

        ui.label("TX:");
        // Display convention: WSJT-X / MSHV / Roger's preference is
        // "1st" / "2nd" period rather than "Even" / "Odd". The
        // INTERNAL canonical strings ("Even" / "Odd") remain — that's
        // what's persisted in Settings, what tx_parity_to_int matches
        // on, and what the auto-parity sync code at line ~2596
        // writes/reads. Only the display labels in this dropdown
        // change. Mapping: Even → "1st", Odd → "2nd" (Even-period
        // starts at the even minute = first period).
        let parity_internal = self.settings.station.tx_parity.clone();
        let cur_tx_par = tx_parity_to_int(&parity_internal);
        let parity_display = parity_display_label(&parity_internal);
        let parity_resp = egui::ComboBox::from_id_source("parity")
            .selected_text(egui::RichText::new(parity_display)
                .color(parity_accent(cur_tx_par))
                .strong())
            .width(70.0)
            .show_ui(ui, |ui| {
                let mut clicked: Option<String> = None;
                if ui.selectable_label(
                    self.settings.station.tx_parity == "Odd",
                    egui::RichText::new("2nd").color(parity_accent(1))).clicked()
                { clicked = Some("Odd".into()); }
                if ui.selectable_label(
                    self.settings.station.tx_parity == "Even",
                    egui::RichText::new("1st").color(parity_accent(0))).clicked()
                { clicked = Some("Even".into()); }
                clicked
            });
        if let Some(Some(p)) = parity_resp.inner {
            if self.settings.station.tx_parity != p {
                self.settings.station.tx_parity = p;
                self.settings_dirty = true;
                self.sync_transmitter_config();
            }
        }

        // Auto-parity status indicator (✓ / ⚠ / ·)
        {
            let total = self.decode_parity_counts.iter().sum::<u32>();
            let (label, color) = if total < 4 {
                ("·", egui::Color32::from_rgb(120, 120, 120))
            } else {
                let dominant: u8 =
                    if self.decode_parity_counts[1] > self.decode_parity_counts[0] {
                        1
                    } else { 0 };
                let dom_count = self.decode_parity_counts[dominant as usize];
                let oth_count = self.decode_parity_counts[(dominant ^ 1) as usize];
                let strong = dom_count >= 3 * oth_count.max(1);
                if !strong {
                    ("·", egui::Color32::from_rgb(120, 120, 120))
                } else if dominant == cur_tx_par {
                    ("⚠", egui::Color32::from_rgb(220, 140, 60))
                } else {
                    ("✓", egui::Color32::from_rgb(80, 200, 120))
                }
            };
            ui.label(egui::RichText::new(label).color(color).strong())
                .on_hover_text(format!(
                    "Slot parity counts (Even/Odd): {}/{}\n\
                     ✓ = TX parity is opposite of dominant decode parity (correct)\n\
                     ⚠ = decodes in same parity as TX (you'd transmit over partner)\n\
                     · = not enough data yet",
                    self.decode_parity_counts[0],
                    self.decode_parity_counts[1]));
        }

        ui.visuals_mut().selection.bg_fill = saved_selection;

        // Manual TX override (MSHV / WSJT-X style). Lets the operator
        // pick up a QSO mid-stream when the auto-driven state machine
        // doesn't know the partner's progress. The dropdown shows
        // fully-rendered text for each Tx[N] message using the
        // current TARGET callsign and the typed Rpt value.
        ui.add_space(12.0);
        ui.label(egui::RichText::new("Manual TX:")
            .color(egui::Color32::from_rgb(180, 180, 180)));

        let me = &self.my_call;
        let them = self.their_call.trim().to_uppercase();
        let have_target = !them.is_empty();
        let rpt = self.manual_tx_rpt;
        let rpt_str_signed = if rpt >= 0 {
            format!("+{:02}", rpt)
        } else {
            format!("-{:02}", -rpt)
        };
        // Tx6 preview shows the on-air form, which uses the 4-char
        // field-square locator regardless of whether the operator's
        // settings hold the extended 6-char value (kept full for PSK
        // Reporter and ADIF). Truncate here so the dropdown preview
        // matches what actually goes out over the air.
        let my_grid_full = self.settings.station.grid.clone()
            .unwrap_or_else(|| "----".into());
        let my_grid_protocol = if my_grid_full.len() >= 4 {
            my_grid_full[..4].to_string()
        } else {
            my_grid_full.clone()
        };

        let txt_tx1 = if have_target {
            format!("Tx1: {} {}", them, me)
        } else { "Tx1: <set TARGET>".to_string() };
        let txt_tx2 = if have_target {
            format!("Tx2: {} {} {}", them, me, rpt_str_signed)
        } else { "Tx2: <set TARGET>".to_string() };
        let txt_tx3 = if have_target {
            format!("Tx3: {} {} R{}", them, me, rpt_str_signed)
        } else { "Tx3: <set TARGET>".to_string() };
        let txt_tx4 = if have_target {
            format!("Tx4: {} {} RR73", them, me)
        } else { "Tx4: <set TARGET>".to_string() };
        let txt_tx5 = if have_target {
            format!("Tx5: {} {} 73", them, me)
        } else { "Tx5: <set TARGET>".to_string() };
        let txt_tx6 = format!("Tx6: CQ {} {}", me, my_grid_protocol);

        let mut chosen: Option<dx_runtime::qso::QsoState> = None;
        egui::ComboBox::from_id_source("manual_tx_dropdown")
            .selected_text(egui::RichText::new("Auto").monospace())
            .width(200.0)
            .show_ui(ui, |ui| {
                ui.set_min_width(220.0);
                let mut item = |ui: &mut egui::Ui,
                                enabled: bool,
                                text: &str,
                                tooltip: &str,
                                on_click: dx_runtime::qso::QsoState| {
                    let resp = ui.add_enabled(enabled,
                        egui::SelectableLabel::new(false, text))
                        .on_hover_text(tooltip);
                    if resp.clicked() { chosen = Some(on_click); }
                };
                item(ui, have_target, &txt_tx1,
                    "Send <them> <me>  (cold call)",
                    dx_runtime::qso::QsoState::CallingStn);
                item(ui, have_target, &txt_tx2,
                    "Send <them> <me> +rpt  (sending report)",
                    dx_runtime::qso::QsoState::SendingReport);
                item(ui, have_target, &txt_tx3,
                    "Send <them> <me> R+rpt  (R-report)",
                    dx_runtime::qso::QsoState::SendingRReport);
                item(ui, have_target, &txt_tx4,
                    "Send <them> <me> RR73  (acknowledge + 73)",
                    dx_runtime::qso::QsoState::SendingRr);
                item(ui, have_target, &txt_tx5,
                    "Send <them> <me> 73  (final)",
                    dx_runtime::qso::QsoState::Sending73);
                item(ui, true, &txt_tx6,
                    "Send CQ <me> <grid>",
                    dx_runtime::qso::QsoState::CallingCq);
            });
        if let Some(state) = chosen {
            self.apply_manual_tx_override(state);
        }

        // Rpt — operator's typed report value used by Tx2 / Tx3.
        ui.add_space(6.0);
        ui.label(egui::RichText::new("Rpt:")
            .color(egui::Color32::from_rgb(180, 180, 180)));
        let resp = ui.add(
            egui::TextEdit::singleline(&mut self.manual_tx_rpt_str)
                .desired_width(46.0)
                .font(egui::TextStyle::Monospace),
        );
        let enter_pressed = ui.input(|i| i.key_pressed(egui::Key::Enter));
        if resp.lost_focus() || (resp.has_focus() && enter_pressed) {
            let s = self.manual_tx_rpt_str.trim();
            let parsed = s.trim_start_matches('+').parse::<i16>();
            if let Ok(n) = parsed {
                let q = (2 * (n / 2)).clamp(-4, 24);
                self.manual_tx_rpt = q;
                self.manual_tx_rpt_str = if q >= 0 {
                    format!("+{:02}", q)
                } else {
                    format!("-{:02}", -q)
                };
            } else {
                let q = self.manual_tx_rpt;
                self.manual_tx_rpt_str = if q >= 0 {
                    format!("+{:02}", q)
                } else {
                    format!("-{:02}", -q)
                };
            }
        }

        // Sh msg toggle.
        ui.add_space(10.0);
        let mut sh = self.settings.decoder.msk40_enabled;
        let sh_resp = ui.checkbox(&mut sh, "Sh")
            .on_hover_text(
                "Short-message mode (MSK40). When enabled, the \
                 late-stage QSO messages (R-report, RRR/RR73, 73) \
                 are sent in the compact bracket-pair form. Both \
                 stations must have this enabled to interoperate. \
                 Off by default — most stations don't run it.");
        if sh_resp.changed() {
            self.settings.decoder.msk40_enabled = sh;
            self.settings_dirty = true;
            if let Ok(mut c) = self.cfg.lock() {
                c.msk40_enabled = sh;
            }
        }
    }

    fn apply_manual_tx_override(&mut self, state: dx_runtime::qso::QsoState) {
        // Make sure the listener is up so we can hear partner's reply
        // — same logic as start_call_target. Without RX, we'd send the
        // override message into the void with no way to advance.
        if !self.is_listening {
            self.start_listening();
        }
        self.ensure_transmitter();
        self.qso.set_my_call(self.my_call.clone());
        self.qso.set_my_grid(self.settings.station.grid.clone());
        self.qso.set_band(self.band.clone());
        self.qso.set_freq_mhz(self.rig_freq_hz.map(|hz| hz as f64 / 1_000_000.0));

        // For Tx6 (CallingCq), the engine doesn't need a partner call.
        // For everything else, pull from the TARGET field.
        let their = if matches!(state, dx_runtime::qso::QsoState::CallingCq) {
            None
        } else {
            let t = self.their_call.trim().to_string();
            if t.is_empty() { return; }
            Some(t)
        };

        let (action, events) = self.qso.force_state(
            state,
            their,
            Some(self.manual_tx_rpt),
        );
        // Drive the engine output through the same path as any other
        // intent — Action::Transmit will spawn/refresh the transmitter
        // and StateChanged will update in_active_qso etc.
        self.apply_engine_output(action, events);
        self.is_calling_cq = matches!(state, dx_runtime::qso::QsoState::CallingCq);
        self.in_active_qso  = !self.is_calling_cq;
        self.qso_started_at = Some(std::time::Instant::now());
        log::info!("[UI] Manual TX override → {:?}", state);
    }

    /// Answer a CQ that the user just clicked on in the SPOTS column.
    /// Sends "<them> <me> +NN" — matches MSK2K's click-to-answer flow.
    fn start_answer_cq(&mut self, their_call: String, their_grid: Option<String>) {
        if their_call.is_empty() { return; }
        self.their_call = their_call.clone();
        self.their_grid = their_grid.clone();
        // Programmatic write to their_call → bump the refresh
        // generation so the TARGET TextEdit re-renders with the
        // new value. Same reasoning as the TheirCallChanged event
        // handler — egui's per-id text cache otherwise hides the
        // update on the next frame.
        self.target_field_refresh_gen =
            self.target_field_refresh_gen.wrapping_add(1);
        // See start_call_target for reasoning. Bring the audio
        // pipeline up if it isn't already, so partner replies can
        // reach the QSO state machine.
        if !self.is_listening {
            self.start_listening();
        }
        self.ensure_transmitter();
        self.qso.set_my_call(self.my_call.clone());
        self.qso.set_my_grid(self.settings.station.grid.clone());
        self.qso.set_band(self.band.clone());
        self.qso.set_freq_mhz(self.rig_freq_hz.map(|hz| hz as f64 / 1_000_000.0));

        let rpt = self.qso.my_report;
        let (action, events) = self.qso.on_intent(Intent::AnswerCq {
            their: their_call,
            rpt,
            grid: their_grid,
        });
        self.apply_engine_output(action, events);
        self.is_calling_cq = false;
        self.in_active_qso = true;
        // is_transmitting set later by TxEvent::Started
        self.qso_started_at = Some(std::time::Instant::now());
    }

    /// Stop transmitting (return to RX-only). Resets QSO engine to Idle.
    fn stop_tx(&mut self) {
        let (action, events) = self.qso.on_intent(Intent::Abort);
        self.apply_engine_output(action, events);
        if let Some(tx) = &self.transmitter {
            tx.set_mode(crate::transmitter::TxMode::Idle);
        }
        self.is_calling_cq = false;
        self.is_transmitting = false;
        self.in_active_qso = false;
        // Clear accumulator freq lock — no QSO in progress.
        self.clear_freq_lock();
    }

    /// Apply the (Action, Vec<EngineEvent>) returned by the QSO engine:
    ///  - Action::Transmit → push the rendered text into the transmitter and
    ///    activate the scheduler so it fires on the next slot.
    ///  - EngineEvent::QsoComplete → write to ADIF, log to UI, clear state.
    ///  - EngineEvent::Tx → push line to TX column.
    ///  - EngineEvent::Info / StateChanged → push line to TX column for visibility.
    /// Update the QSO frequency lock from a partner decode. Uses an EMA
    /// (alpha=0.3) so single-shot outliers don't yank the lock away from
    /// where it's converged. First decode just establishes the lock at
    /// the measured frequency. Pushes the new value into the decoder
    /// thread's config so the accumulator picks it up next slot.
    ///
    /// `df_hz` is the decoded message's freq_offset (relative to fc).
    fn update_freq_lock(&mut self, df_hz: f32) {
        const ALPHA: f32 = 0.3;
        // Outlier rejection: if the new measurement is further from the
        // current lock than 2× accumulator_ntol_hz, ignore for the EMA
        // but still mark the timestamp (so silence-timeout doesn't fire).
        // This matters when, mid-QSO, an interferer briefly decodes at
        // a far-off frequency — we don't want it to yank the lock.
        let outlier_mult = 2.0_f32;
        let ntol = self.settings.decoder.accumulator_ntol_hz.max(1.0);
        let new_lock = match self.qso_freq_lock_hz {
            None => df_hz,
            Some(prev) => {
                if (df_hz - prev).abs() > ntol * outlier_mult {
                    log::debug!(
                        "[FREQ-LOCK] outlier rejected: {:+.1} Hz vs lock {:+.1} Hz",
                        df_hz, prev);
                    prev
                } else {
                    (1.0 - ALPHA) * prev + ALPHA * df_hz
                }
            }
        };
        let changed = self.qso_freq_lock_hz != Some(new_lock);
        self.qso_freq_lock_hz = Some(new_lock);
        self.qso_freq_last_partner_at = Some(std::time::Instant::now());
        if changed {
            log::info!("[FREQ-LOCK] {:+.1} Hz (ntol ±{:.0} Hz)", new_lock, ntol);
        }
        if let Ok(mut c) = self.cfg.lock() {
            c.freq_lock_hz = Some(new_lock);
        }
    }

    /// Clear the QSO frequency lock. Called on QsoComplete, Abort, and
    /// also periodically when partner has been silent for too long
    /// (handled by `check_freq_lock_timeout`).
    fn clear_freq_lock(&mut self) {
        if self.qso_freq_lock_hz.is_some() {
            log::info!("[FREQ-LOCK] cleared");
        }
        self.qso_freq_lock_hz = None;
        self.qso_freq_last_partner_at = None;
        if let Ok(mut c) = self.cfg.lock() {
            c.freq_lock_hz = None;
        }
    }

    /// Drop the freq lock if the partner has been silent for a while —
    /// but ONLY when we're not in an active QSO. The lock exists to
    /// keep the accumulator's narrow ±freq window pointed at the
    /// partner's measured offset; clearing it mid-QSO loses that aim
    /// and lets unrelated traffic pollute the soft-bit fragments.
    ///
    /// During an active QSO the partner is naturally silent for long
    /// stretches (our own TX slots — 30 s each — plus any meteor-
    /// scatter dead time). 60 s of silence is normal and not a signal
    /// to drop the lock.
    ///
    /// The timeout only fires in monitoring mode (no live QSO) where
    /// a stale lock from an old contact would otherwise persist
    /// forever — the next station we hear should re-establish a fresh
    /// lock at their frequency, not be filtered against the previous
    /// partner's offset.
    ///
    /// 60 seconds = roughly 4 slot periods. Called every UI frame from
    /// `update`; cheap (just a time comparison).
    fn check_freq_lock_timeout(&mut self) {
        // Hold the lock for the entire active QSO regardless of how
        // long the partner has been silent. The lock will be cleared
        // explicitly when the QSO completes / aborts (see clear_freq_lock
        // calls in stop(), QsoComplete handlers, etc).
        if self.in_active_qso {
            return;
        }
        const TIMEOUT_SECS: u64 = 60;
        if let Some(at) = self.qso_freq_last_partner_at {
            if at.elapsed().as_secs() >= TIMEOUT_SECS {
                log::info!(
                    "[FREQ-LOCK] timeout after {} s of partner silence (no active QSO)", TIMEOUT_SECS);
                self.clear_freq_lock();
            }
        }
    }

    /// Broadcast a completed QSO over WSJT-X UDP for third-party
    /// loggers. Called from the QsoComplete handler after the local
    /// ADIF write succeeds AND when udp_logging_enabled is true.
    /// Best-effort: any error is logged inside `wsjtx_udp::broadcast`
    /// and swallowed so the QSO completion path is never blocked.
    ///
    /// Frequency: prefer the live rig frequency (CAT-reported), fall
    /// back to the ADIF record's freq field if the rig is offline,
    /// fall back to band default if neither is available. Without a
    /// frequency, loggers can't deduce the band so we always try to
    /// supply something plausible.
    fn broadcast_qso_udp(&self, rec: &dx_runtime::adif::QsoRecord) {
        // Resolve frequency in Hz with the priority above.
        let freq_hz = if let Some(hz) = self.rig_freq_hz {
            hz
        } else if let Some(mhz) = rec.freq {
            (mhz * 1_000_000.0) as u64
        } else {
            // 2 m MS centre as a last-resort default. Loggers will at
            // least correctly identify the band.
            144_360_000
        };

        // Parse the HHMMSS strings the QSO record carries back into
        // full UTC datetimes. The record uses the QSO's date-of-day
        // (qso_date in YYYYMMDD); we combine it with each HHMMSS
        // string. Failures fall back to "now" — better to log a
        // slightly-wrong timestamp than to skip the broadcast.
        let date = &rec.qso_date;
        let parse_dt = |hhmmss: &str| -> chrono::DateTime<chrono::Utc> {
            use chrono::TimeZone;
            if date.len() == 8 && hhmmss.len() == 6 {
                let y: i32 = date[0..4].parse().unwrap_or(2000);
                let mo: u32 = date[4..6].parse().unwrap_or(1);
                let d: u32 = date[6..8].parse().unwrap_or(1);
                let h: u32 = hhmmss[0..2].parse().unwrap_or(0);
                let mi: u32 = hhmmss[2..4].parse().unwrap_or(0);
                let s: u32 = hhmmss[4..6].parse().unwrap_or(0);
                chrono::Utc.with_ymd_and_hms(y, mo, d, h, mi, s)
                    .single()
                    .unwrap_or_else(chrono::Utc::now)
            } else {
                chrono::Utc::now()
            }
        };
        let time_on  = parse_dt(&rec.time_on);
        let time_off = parse_dt(&rec.time_off);

        let log_rec = dx_runtime::wsjtx_udp::LogRecord {
            my_call:   self.settings.station.callsign.clone(),
            my_grid:   self.settings.station.grid.clone().unwrap_or_default(),
            dx_call:   rec.call.clone(),
            dx_grid:   rec.gridsquare.clone().unwrap_or_default(),
            freq_hz,
            mode:      rec.mode.clone(),
            rst_sent:  rec.rst_sent.clone(),
            rst_rcvd:  rec.rst_rcvd.clone(),
            time_on,
            time_off,
            comment:   String::new(),
            // PROP_MODE = "MS" for meteor scatter. Hardcoded here
            // because msk144plus_v2 is exclusively an MS app — the
            // operator isn't going to be running this at HF and
            // misclassifying tropo. If we ever generalise to a
            // multi-mode app, this becomes a settings field.
            prop_mode: "MS".into(),
        };
        dx_runtime::wsjtx_udp::broadcast(
            &self.settings.station.udp_logging_host,
            self.settings.station.udp_logging_port,
            "MSK144Plus",
            &log_rec,
        );
    }

    fn apply_engine_output(&mut self, action: Action, events: Vec<EngineEvent>) {
        // Action: schedule the next outgoing message
        if let Action::Transmit(env) = action {
            // Lazy-spawn the transmitter if it isn't running yet. This
            // is the canonical re-entry point after a hamlib reconnect
            // dropped the old transmitter — by the time we get here,
            // self.hamlib is in its current (correct) state, so the
            // newly-spawned transmitter snapshots a working hamlib
            // handle and PTT works.
            self.ensure_transmitter();
            if let Some(tx) = &self.transmitter {
                tx.set_message(env.raw.clone());
                tx.set_mode(crate::transmitter::TxMode::Active);
            }
        }
        // Events: drive UI
        for ev in events {
            match ev {
                EngineEvent::Tx(_env) => {
                    // The TX line is pushed to tx_log when the slot actually
                    // fires (TxEvent::Started in drain_transmitter), not when
                    // the engine queues it. This gives one entry per real on-
                    // air transmission with the correct slot timestamp.
                }
                EngineEvent::StateChanged(s) => {
                    log::info!("[QSO] state → {}", s);
                    self.current_state = format!("QSO: {}", s);
                    // Mirror is_in_qso for legacy UI fields
                    self.in_active_qso = matches!(
                        s,
                        QsoState::CallingStn
                            | QsoState::SendingReport
                            | QsoState::SendingRReport
                            | QsoState::SendingRr
                            | QsoState::Sending73
                    );
                    self.is_calling_cq = matches!(s, QsoState::CallingCq);
                }
                EngineEvent::TheirCallChanged { callsign, grid } => {
                    log::info!("[QSO] their_call={} grid={:?}", callsign, grid);
                    self.their_call = callsign;
                    // Bump the TARGET TextEdit's refresh generation —
                    // the next render will use a new widget id, so
                    // egui treats it as a fresh instance and re-reads
                    // the bound string. Without this, the field can
                    // show stale (typically empty) cached content
                    // when a partner cold-calls us.
                    self.target_field_refresh_gen =
                        self.target_field_refresh_gen.wrapping_add(1);
                }
                EngineEvent::QsoComplete { their, record } => {
                    log::info!("[QSO] complete with {}", their);
                    self.tx_log.push(LogEntry {
                        text: format!("✓ QSO with {} complete", their),
                        colored: true,
                        timestamp: chrono::Utc::now().format("%H%M%S").to_string(),
                        rx_slot: None,
                    snr_db: None,
                    });
                    let had_record = record.is_some();
                    if let (Some(adif), Some(rec)) = (self.adif.as_ref(), record) {
                        // Write to local ADIF file first — that's the
                        // canonical record. UDP broadcast is best-effort
                        // on top.
                        match adif.log_qso(&rec) {
                            Ok(()) => {
                                self.qso_session_count += 1;
                                log::info!("[QSO] logged to ADIF (session count: {})",
                                    self.qso_session_count);
                                // Append to the in-memory Logbook list
                                // (newest-first) so the bottom panel
                                // updates immediately. The ADIF file
                                // is the source of truth, but the
                                // panel renders this list.
                                self.logged_qsos.insert(0, rec.clone());
                            }
                            Err(e) => {
                                log::error!("[QSO] ADIF write failed: {}", e);
                            }
                        }
                        // WSJT-X UDP broadcast for third-party loggers
                        // (Logger32, N1MM+, JTAlert, HRD, DX Lab Suite,
                        // Cloudlog UDP plugin, etc.). Fire-and-forget
                        // — failures are logged but never block the QSO
                        // completion path. Trigger is identical to the
                        // ADIF write, which is itself triggered by the
                        // QSO state machine on receipt of partner's
                        // final 73 or RR73.
                        if self.settings.station.udp_logging_enabled {
                            self.broadcast_qso_udp(&rec);
                        }
                    } else if !had_record {
                        log::warn!("[QSO] no record produced — not enough info to log");
                    }
                    // Engine has transitioned to Done; we drop back to listening.
                    self.is_calling_cq = false;
                    self.in_active_qso = false;
                    self.is_transmitting = false;
                    if let Some(tx) = &self.transmitter {
                        tx.set_mode(crate::transmitter::TxMode::Idle);
                    }
                    // Clear target so user can start a new QSO
                    self.their_call.clear();
                    self.their_grid = None;
                    // Bump target refresh gen — same reasoning as the
                    // TheirCallChanged handler; clearing the bound
                    // string isn't enough on its own.
                    self.target_field_refresh_gen =
                        self.target_field_refresh_gen.wrapping_add(1);
                    // Clear the accumulator's frequency lock — partner may
                    // have moved off, and a fresh lock will be established
                    // by the next QSO's first decode.
                    self.clear_freq_lock();
                    // Re-arm: put the engine back into Listening so the
                    // next station that calls us is auto-answered. This
                    // means a single user "Listen" click at the start of
                    // a session covers an unlimited sequence of QSOs;
                    // the user only has to click Stop to disarm.
                    if self.is_listening {
                        let (action, events) = self.qso.on_intent(Intent::Listen);
                        self.apply_engine_output(action, events);
                    }
                }
                EngineEvent::Info(m) => {
                    log::info!("[QSO] {}", m);
                }
                EngineEvent::Rx(_) => { /* already logged elsewhere */ }
            }
        }
    }

    /// Drain TxEvents from the transmitter thread.
    fn drain_transmitter(&mut self) {
        // Collect events first, releasing the borrow on self.tx_event_rx
        // before we mutate other fields of self.
        let events: Vec<crate::transmitter::TxEvent> = match &self.tx_event_rx {
            Some(rx) => rx.try_iter().collect(),
            None => return,
        };

        for e in events {
            match e {
                crate::transmitter::TxEvent::Started { slot_index, message } => {
                    self.is_transmitting = true;
                    log::info!("[UI] TX started: slot={} msg={:?}", slot_index, message);
                    // Tell the decoder to skip incoming audio while we
                    // transmit so it doesn't decode the rig's monitor/sidetone.
                    // The framer's tx_active gate (in decoder.rs) drops any
                    // slot whose accumulation period overlapped with TX —
                    // RX slots between TX cycles continue to accumulate
                    // and decode normally because the cpal input stream
                    // stays alive throughout.
                    if let Ok(mut c) = self.cfg.lock() {
                        c.tx_active = true;
                    }
                    // Tell the recorder we're now transmitting. It will:
                    // (a) immediately finalise any pending auto-WAV saves
                    //     (so their post-roll doesn't capture our TX), and
                    // (b) drop incoming audio for the duration of TX (so
                    //     the ring buffer stays clean and the next save's
                    //     pre-roll is the pre-TX RX audio, not our own TX).
                    self.recorder.set_tx_active(true);
                    // One entry per real transmission, logged at slot start
                    // with the actual UTC time the slot fired.
                    self.tx_log.push(LogEntry {
                        text: message,
                        colored: true,
                        timestamp: chrono::Utc::now().format("%H%M%S").to_string(),
                        rx_slot: None,
                        snr_db: None,
                    });
                    if self.tx_log.len() > 200 {
                        let n = self.tx_log.len() - 200;
                        self.tx_log.drain(..n);
                    }
                }
                crate::transmitter::TxEvent::Finished { slot_index } => {
                    self.is_transmitting = false;
                    log::info!("[UI] TX finished: slot={}", slot_index);
                    // Re-enable the decoder for incoming RX audio.
                    if let Ok(mut c) = self.cfg.lock() {
                        c.tx_active = false;
                    }
                    // Recorder can now resume capturing audio for ring/pending.
                    self.recorder.set_tx_active(false);
                    // After a TX slot completes, ask the engine for the
                    // next outgoing message. The engine uses next_tx() to
                    // dictate whether to keep repeating the same line
                    // (CQ, calling-station) or move on (Done after 73 ×N).
                    if let Some(payload) = self.qso.next_tx() {
                        // Use Sh form when the user has it enabled AND
                        // we know the partner call (needed to compute
                        // the hash). Mirrors the gate in QsoEngine::make_tx.
                        let use_sh = self.settings.decoder.msk40_enabled
                            && self.qso.their_call.is_some();
                        let env = match render_payload_with_sh(&payload, use_sh) {
                            Rendered::Text(s) => TxEnvelope {
                                payload: payload.clone(),
                                format: payload.format(),
                                raw: s,
                            },
                        };
                        if let Some(tx) = &self.transmitter {
                            tx.set_message(env.raw.clone());
                            tx.set_mode(crate::transmitter::TxMode::Active);
                        }
                        // Don't push to tx_log here; it'll be pushed when
                        // the slot actually fires (TxEvent::Started below).
                    } else {
                        // Engine says no more TX → stop transmitting.
                        // (e.g. completed 73 × max_repeats without confirm.)
                        if let Some(ev) = self.qso.check_complete() {
                            self.apply_engine_output(Action::None, vec![ev]);
                        } else if !matches!(self.qso.state,
                            QsoState::CallingCq
                                | QsoState::CallingStn
                                | QsoState::SendingReport
                                | QsoState::SendingRReport
                                | QsoState::SendingRr
                                | QsoState::Sending73)
                        {
                            if let Some(tx) = &self.transmitter {
                                tx.set_mode(crate::transmitter::TxMode::Idle);
                            }
                            self.is_calling_cq = false;
                            self.in_active_qso = false;
                        }
                    }
                }
                crate::transmitter::TxEvent::EncodeFailed { reason } => {
                    log::warn!("[UI] TX encode failed: {}", reason);
                    self.tx_log.push(LogEntry {
                        text: format!("encode failed: {}", reason),
                        colored: false,
                        timestamp: chrono::Utc::now().format("%H%M%S").to_string(),
                        rx_slot: None,
                    snr_db: None,
                    });
                }
            }
        }
    }

    fn drain_hamlib(&mut self) {
        if let Some(rx) = &self.hamlib_rx {
            while let Ok(u) = rx.try_recv() {
                self.cat_connected = u.connected;
                if let Some(f) = u.freq_hz {
                    self.rig_freq_hz = Some(f);
                    // Capture the base frequency on the first freq
                    // report after a (re)connect. The kHz click-edit
                    // is clamped to ±250 kHz of this value so the
                    // user can't accidentally QSY across a band edge.
                    // Cleared when CAT disconnects so a re-connect
                    // re-anchors at the rig's then-current setting.
                    if self.base_freq_hz.is_none() {
                        self.base_freq_hz = Some(f);
                    }
                    if let Some(b) = dx_runtime::band_from_freq_hz(f) {
                        if self.band != b {
                            self.band = b.to_string();
                            self.settings_dirty = true;
                        }
                    }
                } else if !u.connected {
                    self.rig_freq_hz = None;
                    self.base_freq_hz = None;
                }
            }
        }
    }

    /// Drain pending spectrum columns from the worker thread and append
    /// them to the display buffer. Resets the buffer at the start of
    /// each new 15-second slot. Drops incoming columns while we are
    /// transmitting (we don't want our own outgoing audio painted on
    /// the spectrum even if it leaks back through monitor / loopback).
    fn drain_spectrum(&mut self) {
        // Slot-boundary reset (aligned to UTC). When the current slot
        // index advances, clear and start a fresh slot. Slot period
        // is configurable (15s WSJT-X / 30s IARU R1) — read it from
        // the live settings each pass so the operator can flip
        // periods at runtime via the top-bar Period selector.
        //
        // Before clearing, record how many columns the just-completed
        // slot had — this self-calibrates the layout's column-width
        // divisor so the panel fills the full panel width regardless
        // of the actual audio chunk size produced by the resampler.
        let slot_period_secs = self.settings.station.slot_period_secs.max(1) as i64;
        let slot_period_ms   = slot_period_secs * 1000;
        let now_ms = chrono::Utc::now().timestamp_millis();
        let slot_idx = now_ms / slot_period_ms;
        if slot_idx != self.spectrum_slot_idx {
            // Only count the slot if it ran approximately to
            // completion. Threshold scales with period so 30s slots
            // get a higher floor (160 cols ≈ 13.3s of 30s = nearly
            // full).
            let completion_threshold = (slot_period_secs as usize * 12) / 2; // ~6 cols/sec
            let n = self.spectrum_columns.len();
            if n >= completion_threshold && n > self.spectrum_max_cols_seen {
                self.spectrum_max_cols_seen = n;
            }
            self.spectrum_columns.clear();
            self.spectrum_slot_idx = slot_idx;
            // If the operator just changed period, the previous max
            // is stale — reset so the new period self-calibrates
            // afresh. Detect by the cap also having moved.
            let expected_max_for_period =
                ((slot_period_secs as f32 * 12000.0 / 1024.0) as usize).max(1);
            if self.spectrum_max_cols_seen > expected_max_for_period * 13 / 10 {
                // Old value was ~2× too high (we changed from 30s → 15s)
                self.spectrum_max_cols_seen = 0;
            }
        }

        // Drain anything the worker has produced. While transmitting,
        // we still drain (otherwise the channel would back up while
        // TX runs) but discard the data — it's our own audio bleeding
        // through and shouldn't appear on the display.
        let rx = match &self.spectrum_rx {
            Some(rx) => rx,
            None => return,
        };
        let tx_active = self.is_transmitting;
        // Cap scales with slot length: 15s ≈ 176 cols → cap 200,
        // 30s ≈ 352 cols → cap 400. Prevents unbounded growth on
        // clock skew / paused process.
        let cap = ((slot_period_secs as f32 * 12000.0 / 1024.0) as usize) + 25;
        for col in rx.try_iter() {
            if tx_active {
                continue;
            }
            if self.spectrum_columns.len() >= cap {
                self.spectrum_columns.remove(0);
            }
            self.spectrum_columns.push(col);
        }
    }

    fn drain_decoder(&mut self) {
        let decodes: Vec<UiDecodeEvent> = match &self.decode_rx {
            Some(rx) => rx.try_iter().collect(),
            None => return,
        };
        for e in decodes {
            {
                // Display timestamp = canonical slot-end time the decoder
                // computed when the audio was drained from the slot buffer.
                // Format: "HHMMSS" → "HH:MM:SS" with colons inserted. Using
                // the slot timestamp (rather than wall-clock at this moment)
                // means the displayed time always matches the audio's on-air
                // moment, even when LDPC processing took several seconds.
                let ts_disp = if e.slot_utc.len() == 6 {
                    format!("{}:{}:{}",
                        &e.slot_utc[0..2], &e.slot_utc[2..4], &e.slot_utc[4..6])
                } else {
                    e.slot_utc.clone()
                };

                // Frequency offset from configured center (DF in Hz, signed)
                let df_hz = e.event.freq_offset;
                let abs_freq = e.fc_hz + df_hz;
                let _ = abs_freq;

                // The engine's SpdCandidate already carries the canonical
                // WSJT-X SNR (12*log10(detmet)/2 - 9) computed off the raw
                // detmet metric. Quantise to MSK144's 2-dB grid and clamp
                // to [-4, +24].
                let snr_db = quantise_snr_db(e.event.snr_db);

                // Compact MSHV-style format: time, T (time-into-slot),
                // freq offset, message. Strength/method dropped — easy
                // to grok at a glance, and anyone debugging can see
                // them in the daily log file.
                // Append " [A]" suffix for soft-bit accumulator decodes
                // so they're visually distinguishable from standard
                // ones during A/B comparison (matches MSK2K's convention).
                let suffix = if e.event.is_accumulated { " [A]" } else { "" };
                // T column: time-into-slot in seconds, from the engine's
                // SPD candidate (or MSK40 candidate). One decimal place.
                // Averaging-pattern and accumulator decodes don't carry
                // a single time-position; show "T=----" for those so the
                // column lines up AND so parse_clicked_row's whitespace-
                // token skip stays at a fixed count of 4.
                let t_disp = match e.event.slot_position_secs {
                    Some(t) => format!("T={:>4.1}", t),
                    None    => "T=----".into(),
                };
                let text = format!(
                    "{}  {}  {:>+5.0} Hz   {}{}",
                    ts_disp,
                    t_disp,
                    df_hz,
                    e.event.text,
                    suffix,
                );

                // Routing rules (mirroring MSK2K):
                //  - CQ-style messages → SPOTS column (always, regardless of QSO state)
                //  - Messages containing MY callsign → RX (someone calling me)
                //  - Messages containing TARGET callsign during a QSO → RX
                //  - Everything else → SPOTS by default (we're "monitoring" them)
                let msg = &e.event.text;

                // First: skip echoes of our own TX (loopback or rig-monitor).
                // The "from" caller is detected via the same parser the QSO
                // engine uses. If from == my_call, ignore — don't pollute
                // RX/SPOTS with our own transmissions.
                let from_is_me = if let Some(p) = proto::parse_decode_text(msg) {
                    !self.my_call.is_empty()
                        && p.from_call().eq_ignore_ascii_case(&self.my_call)
                } else {
                    false
                };
                if from_is_me {
                    log::debug!("[ROUTE] skipping own TX echo: {}", msg);
                    continue;
                }

                let is_cq      = msg.starts_with("CQ ");
                let mentions_me = !self.my_call.is_empty()
                    && msg.contains(&self.my_call);
                let mentions_target = !self.their_call.is_empty()
                    && msg.contains(&self.their_call);

                let goes_to_rx = mentions_me
                    || (self.in_active_qso && mentions_target);

                // Compute the slot parity for this decode (0 or 1) from
                // its slot_utc, so the row can be coloured by parity.
                // Period must match what the framer was using when the
                // slot was timestamped.
                let parity = slot_parity_from_hhmmss(
                    &e.slot_utc, self.settings.station.slot_period_secs);

                if goes_to_rx {
                    self.rx_log.push(LogEntry {
                        text: text.clone(),
                        colored: mentions_me,
                        timestamp: String::new(),
                        rx_slot: parity,
                        snr_db: Some(snr_db),
                    });
                    if self.rx_log.len() > 500 {
                        let n = self.rx_log.len() - 500;
                        self.rx_log.drain(..n);
                    }
                }

                // CQ + everything-not-going-to-RX → SPOTS
                if is_cq || !goes_to_rx {
                    self.cq_log.push(LogEntry {
                        text: text.clone(),
                        colored: is_cq,
                        timestamp: String::new(),
                        rx_slot: parity,
                        snr_db: Some(snr_db),
                    });
                    if self.cq_log.len() > 200 {
                        let n = self.cq_log.len() - 200;
                        self.cq_log.drain(..n);
                    }
                }

                // Update the parity histogram for auto-detect — we only
                // care about decodes-not-from-us (already filtered above
                // via from_is_me) so this counts purely RX traffic.
                if let Some(p) = parity {
                    let idx = (p & 1) as usize;
                    self.decode_parity_counts[idx] =
                        self.decode_parity_counts[idx].saturating_add(1);
                    // Cheap sliding window: when total exceeds threshold,
                    // halve both counts to keep recent history weighted
                    // higher than ancient history. No timestamps needed.
                    let total = self.decode_parity_counts[0]
                        + self.decode_parity_counts[1];
                    if total >= 32 {
                        self.decode_parity_counts[0] /= 2;
                        self.decode_parity_counts[1] /= 2;
                    }
                }

                let count = self.decode_counts.entry(msg.clone()).or_insert(0);
                *count += 1;

                self.last_corr = (e.event.xmax / 5.0).min(1.0);

                // PSK Reporter spot push. Only fire when:
                //   - reporter is spawned (settings + station info OK)
                //   - decode came from a recognisable callsign (not us)
                //   - we have a rig CAT lock so the absolute frequency
                //     is known accurately (otherwise the spot's freq
                //     would just be the audio offset, which would plot
                //     us at ~1500 Hz. Useless.)
                //   - decode has per-burst timing fidelity. Averaging
                //     and accumulator decodes don't carry a single
                //     time-position (T=----), so PSK Reporter's
                //     timestamp would be ambiguous. Skip those.
                //
                // Each path through the gate logs a brief reason at
                // info level so the operator can see why a given
                // decode did or didn't generate a spot.
                if let Some(reporter) = &self.psk_reporter {
                    if from_is_me {
                        // Already filtered above by `continue`, but
                        // keep the branch for clarity.
                    } else if let Some(heard_call) = extract_callsign(&text) {
                        let has_burst_time = e.event.slot_position_secs.is_some();
                        let rig_freq = self.rig_freq_hz;
                        if !has_burst_time {
                            log::info!(
                                "[PSKR] skip spot for {}: no burst timing \
                                 (averaging or accumulator decode)",
                                heard_call);
                        } else if rig_freq.is_none() {
                            log::info!(
                                "[PSKR] skip spot for {}: no rig CAT freq \
                                 — enable hamlib for accurate spotting",
                                heard_call);
                        } else {
                            // PSK Reporter convention: report DIAL
                            // frequency (the rig's set frequency),
                            // NOT dial + audio-offset. Reasons:
                            //   - Other operators tune their rig to
                            //     this number to hear the spotted
                            //     station; audio offset is decoder-
                            //     specific and doesn't translate.
                            //   - WSJT-X / MSHV / FT8 spotters all
                            //     report dial; spotting dial+offset
                            //     puts our reports in the wrong
                            //     bucket on the map (e.g. 144.362
                            //     vs 144.360).
                            // The audio offset (e.fc_hz + df_hz) is
                            // still recorded locally in the DB and
                            // shown in the UI's "[+NN Hz]" column —
                            // operators who care about the exact
                            // tone within the dial pass-band can
                            // see it there.
                            let abs_freq = rig_freq.unwrap();
                            let _audio_offset_hz = (e.fc_hz + df_hz) as i32;  // kept for clarity
                            let heard_grid = extract_grid_from_message(msg);
                            let utc_secs = parse_slot_utc_to_unix(&e.slot_utc)
                                .unwrap_or_else(|| {
                                    std::time::SystemTime::now()
                                        .duration_since(std::time::UNIX_EPOCH)
                                        .map(|d| d.as_secs() as u32)
                                        .unwrap_or(0)
                                });
                            reporter.add_spot(
                                heard_call,
                                heard_grid,
                                abs_freq,
                                "MSK144",
                                snr_db as i32,
                                utc_secs,
                            );
                        }
                    } else {
                        log::debug!(
                            "[PSKR] skip spot: no recognisable callsign in '{}'",
                            text);
                    }
                }

                // Feed the decoded message into the QSO state machine.
                // The engine ignores messages that don't match the current
                // partner / state; it acts on ones that do (e.g. "GW4WND F1ABC +05"
                // when we're CallingCq → triggers SendingRReport).
                if let Some(payload) = proto::parse_decode_text(msg) {
                    // Capture the from-call before moving payload into the
                    // envelope — used for freq-lock matching after on_rx.
                    let from_call = payload.from_call().to_string();
                    let envelope = RxEnvelope {
                        format: payload.format(),
                        payload,
                        // SNR carried in dB so the engine can use it as
                        // the next outgoing report (the R-report we send
                        // after CallingCq → SendingRReport, etc).
                        snr: Some(snr_db as f32),
                        utc_ms: chrono::Utc::now().timestamp_millis(),
                        rx_slot: 0,
                    };
                    let (action, events) = self.qso.on_rx(envelope);
                    self.apply_engine_output(action, events);

                    // Update accumulator's frequency lock if this RX came
                    // from the current QSO partner. The engine sets
                    // `their_call` either before this on_rx (existing QSO)
                    // or as a result of it (new partner identified by their
                    // first message), so we check after apply_engine_output.
                    let their = self.qso.their_call.clone();
                    if let Some(their_call) = their {
                        if !their_call.is_empty()
                            && from_call.eq_ignore_ascii_case(&their_call)
                        {
                            self.update_freq_lock(df_hz);
                        }
                    }
                }
            }
        }
        if let Some(rx) = &self.level_rx {
            while let Ok(level) = rx.try_recv() {
                self.last_level = level;
            }
        }
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Push current QSO partner call into the decoder config so the
        // MSK40 short-message decoder can compute the correct hash. The
        // hash is over "<MYCALL HISCALL>"; we already push mycall at
        // start_listening time, but hiscall changes over the course of
        // a session as QSOs come and go. Cheap (one mutex acquisition
        // per frame; only writes if value changed).
        if let Ok(mut c) = self.cfg.lock() {
            if c.hiscall != self.their_call {
                c.hiscall = self.their_call.clone();
            }
        }

        // Mirror the Sh-msg setting into the QSO engine so its TX
        // rendering knows whether to emit bracketed-pair short form
        // for the late-stage messages (R-report, RRR, 73). Cheap;
        // only writes when value changes.
        let want_sh = self.settings.decoder.msk40_enabled;
        if self.qso.use_short_msg != want_sh {
            self.qso.set_use_short_msg(want_sh);
        }

        // Mirror the QSO engine's their_grid into the app state. The
        // engine populates this when an incoming decode like
        // "GW4WND F1ABC IO82" arrives — the partner volunteered their
        // grid in the call. We may already have set their_grid from a
        // CQ click; this keeps it in sync if the engine learned a more
        // specific value (e.g. CQ had no grid but the answer included
        // one). Only overwrite if the engine has a value AND the calls
        // match — defensive against stale engine state.
        if let Some(eg) = self.qso.their_grid.as_ref() {
            if self.qso.their_call.as_deref() == Some(self.their_call.as_str())
                && self.their_grid.as_deref() != Some(eg.as_str())
            {
                self.their_grid = Some(eg.clone());
            }
        }

        self.drain_decoder();
        self.drain_spectrum();
        self.drain_hamlib();
        self.drain_transmitter();
        // Time out the accumulator freq lock if partner has been silent
        // for more than 60 seconds. Cheap (one timestamp comparison).
        self.check_freq_lock_timeout();

        // Auto-start hamlib if enabled in settings but not yet running.
        if self.settings.station.hamlib_enabled && self.hamlib.is_none() {
            self.start_hamlib();
        }
        // Auto-stop if disabled in settings but still running.
        if !self.settings.station.hamlib_enabled && self.hamlib.is_some() {
            self.stop_hamlib();
        }

        // Debounced auto-save: if settings have been edited and 2 seconds
        // have elapsed since the last check, persist now.
        if self.settings_dirty
            && self.last_save_check.elapsed() >= std::time::Duration::from_secs(2)
        {
            self.save_settings();
            self.last_save_check = std::time::Instant::now();
        }

        // Decide whether to render the TX-control cluster inline on
        // row 1 or on a dedicated overflow row beneath it. Threshold
        // is approximate — the inline cluster is ~600 px wide and
        // the rest of row 1 (callsign + freq + mode/period + clock
        // + cog) is ~520 px, so below ~1180 px the inline form would
        // start to push items off the right side. Set the breakpoint
        // a touch lower so common 1280×… windows still render single-
        // row, but anything ~1100 px wide flips into compact form.
        const TOP_BAR_BREAKPOINT_PX: f32 = 1180.0;
        self.compact_top_bar = ctx.screen_rect().width() < TOP_BAR_BREAKPOINT_PX;

        // ── Top bar ───────────────────────────────────────────────────────────
        egui::TopBottomPanel::top("top_bar").show(ctx, |ui| {
            ui.add_space(5.0);
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new(&self.my_call).strong().size(22.0).color(egui::Color32::GRAY));
                ui.add_space(20.0);

                if self.cat_connected {
                    let freq = self.rig_freq_hz.unwrap_or(0);
                    let mhz = freq / 1_000_000;
                    let khz = (freq % 1_000_000) / 1_000;
                    let hz = (freq % 1_000) / 10;
                    let color = egui::Color32::from_rgb(100, 200, 130);
                    let col_amb = egui::Color32::from_rgb(200, 150, 50);

                    // ±250 kHz clamp from base frequency (set on first
                    // CAT connect this session). Prevents an accidental
                    // triple-tap from QSY'ing across an entire band.
                    let base = self.base_freq_hz.unwrap_or(freq);
                    let lo = base.saturating_sub(250_000);
                    let hi = base + 250_000;

                    let mut new_freq: Option<u64> = None;

                    ui.horizontal(|ui| {
                        ui.spacing_mut().item_spacing.x = 0.0;
                        // "144." prefix
                        ui.label(egui::RichText::new(format!("{}.", mhz))
                            .monospace().size(16.0).color(color));

                        // The kHz field: click-to-edit.
                        // While editing, render as an active TextEdit
                        // (amber). Otherwise, render as a click-to-edit
                        // label using the painter (no underline / focus
                        // visuals from a clickable Label).
                        if self.freq_editing {
                            let resp = ui.add(
                                egui::TextEdit::singleline(&mut self.freq_edit)
                                    .desired_width(34.0)
                                    .font(egui::TextStyle::Monospace)
                                    .text_color(col_amb)
                                    .frame(true),
                            );
                            if !resp.has_focus() { resp.request_focus(); }

                            // Keep digits only, max 3
                            self.freq_edit.retain(|c| c.is_ascii_digit());
                            if self.freq_edit.len() > 3 { self.freq_edit.truncate(3); }

                            // Commit: 3rd digit typed, Enter, or focus lost
                            // (but Escape cancels).
                            let auto_commit = self.freq_edit.len() == 3 && resp.changed();
                            let enter_commit = ui.input(|i| i.key_pressed(egui::Key::Enter));
                            let escape       = ui.input(|i| i.key_pressed(egui::Key::Escape));
                            let focus_lost   = resp.lost_focus() && !escape;

                            if auto_commit || enter_commit || focus_lost {
                                if !escape && self.freq_edit.len() == 3 {
                                    if let Ok(new_k) = self.freq_edit.parse::<u64>() {
                                        let new_f = mhz * 1_000_000 + new_k * 1_000;
                                        if new_f >= lo && new_f <= hi {
                                            new_freq = Some(new_f);
                                        }
                                    }
                                }
                                self.freq_edit.clear();
                                self.freq_editing = false;
                                ui.memory_mut(|m| m.surrender_focus(resp.id));
                            }
                            if escape {
                                self.freq_edit.clear();
                                self.freq_editing = false;
                            }
                        } else {
                            // Idle — clickable. Painter-drawn so there's
                            // no default Label hover-underline.
                            let text = format!("{:03}", khz);
                            let galley = ui.fonts(|f| f.layout_no_wrap(
                                text.clone(),
                                egui::FontId::monospace(16.0),
                                color,
                            ));
                            let (rect, r) = ui.allocate_exact_size(
                                galley.size(), egui::Sense::click(),
                            );
                            if r.hovered() {
                                ui.output_mut(|o| o.cursor_icon = egui::CursorIcon::Text);
                            }
                            let clicked = r.clicked();
                            ui.painter().galley_with_color(rect.min, galley, color);
                            r.on_hover_text(format!(
                                "Click to QSY — type 3 digits (e.g. {}). \
                                 Range ±250 kHz from {}.{:03} MHz.",
                                text, base / 1_000_000, (base % 1_000_000) / 1_000,
                            ));
                            if clicked {
                                // Pre-populate so the field is never
                                // a blank box; matches FSK441Plus UX.
                                self.freq_edit = format!("{:03}", khz);
                                self.freq_editing = true;
                            }
                        }

                        // ".HH MHz" suffix (Hz portion shown small)
                        ui.label(egui::RichText::new(".")
                            .monospace().size(16.0).color(color));
                        ui.vertical(|ui| {
                            ui.add_space(5.0);
                            ui.label(egui::RichText::new(format!("{:02}", hz))
                                .monospace().size(11.0).color(color));
                        });
                        ui.label(egui::RichText::new(" MHz")
                            .monospace().size(16.0).color(color));
                    });

                    if let Some(f) = new_freq {
                        if self.is_transmitting {
                            // Don't QSY mid-TX — the rig is keyed and a
                            // freq change here would split the on-air
                            // transmission across two frequencies, which
                            // is rude and might also trip rig protections.
                            log::info!("[UI] freq change ignored — TX active");
                        } else {
                            // Update local state immediately so the UI reflects
                            // the change without waiting for the next CAT poll.
                            // The rig will confirm via the worker's GetFreq
                            // follow-up read (already triggered by SetFreq).
                            self.rig_freq_hz = Some(f);
                            if let Some(h) = &self.hamlib {
                                h.set_freq(f);
                            }
                        }
                    }
                } else {
                    ui.label(egui::RichText::new("No CAT").monospace().size(16.0).color(egui::Color32::GRAY));
                }

                // Mode + period. Mode is fixed MSK144 (visually muted
                // blue). Period is a small clickable dropdown — 15s
                // (WSJT-X / US default) or 30s (IARU R1 specification
                // for 144 MHz MS). Both ends of a QSO must use the
                // same period to interoperate. Default is 30s on
                // first launch (R1 protocol-correct); the operator
                // can flip to 15s if working US/R2/R3 stations or
                // band conditions warrant.
                ui.add_space(15.0);
                let muted = egui::Color32::from_rgb(140, 140, 140);
                let mode_blue = egui::Color32::from_rgb(100, 180, 220);
                ui.label(egui::RichText::new("MSK144").color(mode_blue));
                ui.label(egui::RichText::new("•").color(muted).weak());

                // Clamp to the two legal values on read so a stale
                // settings file with some other number (a development
                // artefact) gets coerced to 30s on first run.
                let cur_period = if self.settings.station.slot_period_secs == 15 {
                    15
                } else {
                    30
                };
                let cur_period_label = format!("{}s", cur_period);
                let mut new_period: Option<u32> = None;
                egui::ComboBox::from_id_source("slot_period_dropdown")
                    .selected_text(egui::RichText::new(cur_period_label).color(muted).strong())
                    .width(54.0)
                    .show_ui(ui, |ui| {
                        if ui.selectable_label(cur_period == 15,
                            egui::RichText::new("15s").monospace())
                            .on_hover_text(
                                "15-second slots (WSJT-X / US default).\n\
                                 Used by most stations in IARU R2 / R3.\n\
                                 Both ends of a QSO must use the same period.")
                            .clicked()
                        { new_period = Some(15); }
                        if ui.selectable_label(cur_period == 30,
                            egui::RichText::new("30s").monospace())
                            .on_hover_text(
                                "30-second slots (IARU Region 1 specification\n\
                                 for 144 MHz meteor scatter).\n\
                                 Use this in Europe/Africa. Both ends of a\n\
                                 QSO must use the same period.")
                            .clicked()
                        { new_period = Some(30); }
                    });
                if let Some(p) = new_period {
                    if self.settings.station.slot_period_secs != p {
                        self.settings.station.slot_period_secs = p;
                        self.settings_dirty = true;
                        self.sync_transmitter_config();
                        // The decoder framer reads slot_period_secs
                        // from the shared cfg Arc each pass, so the
                        // change takes effect on the next slot drain.
                        // Spectrum buffer self-resets when slot_idx
                        // jumps (which it will since slot_period_ms
                        // just changed).
                        if let Ok(mut c) = self.cfg.lock() {
                            c.slot_period_secs = p;
                        }
                        log::info!("[UI] Slot period changed to {}s", p);
                    }
                }

                // Operator's TX-control cluster (TX parity, Manual TX,
                // Rpt, Sh) — render inline on row 1 if the window is
                // wide enough; otherwise an "overflow" panel below
                // catches it on its own row. Threshold is the
                // approximate sum of the inline cluster widths plus
                // the leading callsign/freq/mode/period and trailing
                // clock+cog. Tweak if fonts or labels change.
                if !self.compact_top_bar {
                    ui.add_space(12.0);
                    self.render_tx_controls_cluster(ui);
                }

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.button("⚙").clicked() {
                        self.settings_open = true;
                        self.available_serial_ports = dx_runtime::list_serial_ports();
                        self.available_output_devices = dx_runtime::list_output_devices();
                    }
                    ui.label(egui::RichText::new(chrono::Utc::now().format("%H:%M:%S Z").to_string()).monospace());
                });
            });
        });

        // ── Top-bar overflow row (only when compact) ──────────────────────────
        // When the window is too narrow for the full row 1, the
        // TX-control cluster (parity + Manual TX + Rpt + Sh) lands
        // here on its own line between row 1 and the actions strip.
        // When the window is wide, the cluster renders inline on
        // row 1 and this panel doesn't appear at all.
        if self.compact_top_bar {
            egui::TopBottomPanel::top("top_bar_overflow").show(ctx, |ui| {
                ui.add_space(3.0);
                ui.horizontal(|ui| {
                    self.render_tx_controls_cluster(ui);
                });
                ui.add_space(3.0);
            });
        }

        // ── Actions strip ─────────────────────────────────────────────────────
        egui::TopBottomPanel::top("actions").show(ctx, |ui| {
            ui.horizontal(|ui| {
                let green = egui::Color32::from_rgb(56, 120, 70);
                let dim = egui::Color32::from_rgb(45, 45, 45);

                if ui.add_sized([90.0, 30.0],
                    egui::Button::new("📻 LISTEN").fill(if self.is_listening { green } else { dim })).clicked()
                {
                    if self.is_listening { self.stop(); } else { self.start_listening(); }
                }

                let cq_btn = ui.add_sized([90.0, 30.0],
                    egui::Button::new("📢 CALL CQ").fill(if self.is_calling_cq { green } else { dim }));
                if cq_btn.clicked() {
                    if self.is_calling_cq {
                        self.stop_tx();
                    } else {
                        self.start_cq();
                    }
                }

                ui.add_space(20.0);
                ui.label("TARGET:");
                ui.horizontal(|ui| {
                    // Width sized for the MSK144 protocol max
                    // callsign length: 11 chars (e.g. "KH6/W1ABC/P"
                    // — country prefix + base call + portable suffix
                    // is the longest legal form). Bare calls are
                    // ≤6 chars. 105 px fits 11 monospace characters
                    // with a little internal padding; the count
                    // indicator alongside flips amber once the operator
                    // hits the limit.
                    //
                    // egui TextEdit caches its displayed text per
                    // widget id across frames. When the QSO engine
                    // updates `self.their_call` programmatically
                    // (e.g. partner cold-called us → TheirCallChanged
                    // event), the cached state from the previous
                    // frame can shadow our update and the field
                    // appears empty even though the binding holds
                    // the new value. The fix: bump a refresh
                    // generation counter when the engine writes,
                    // and feed it into the widget id. A new id makes
                    // egui treat the widget as a fresh instance and
                    // re-read from the binding.
                    let target_id = egui::Id::new(("target_textedit",
                        self.target_field_refresh_gen));
                    let resp = ui.add(
                        egui::TextEdit::singleline(&mut self.their_call)
                            .id(target_id)
                            .desired_width(105.0),
                    );
                    if resp.changed() {
                        self.their_call = self.their_call.to_uppercase();
                        if self.their_call.len() > 11 { self.their_call.truncate(11); }
                        // Manual edit invalidates any stashed grid
                        // (no way to know the user is still pointing at
                        // the same station whose grid we knew about).
                        self.their_grid = None;
                    }
                    if !self.their_call.is_empty() {
                        let count_color = if self.their_call.len() == 11 {
                            egui::Color32::from_rgb(255, 180, 0)
                        } else { egui::Color32::GRAY };
                        ui.label(egui::RichText::new(format!("{}/11", self.their_call.len()))
                            .small().color(count_color));
                    }
                });

                let call_btn = ui.button("CALL");
                if call_btn.clicked() {
                    if self.in_active_qso {
                        self.stop_tx();
                    } else {
                        self.start_call_target();
                    }
                }

                // ── Distance / bearing / scatter / A & B beam headings ─
                // Shown when we know both my grid and their grid (4-char
                // Maidenhead minimum, calculated to centre of the square
                // per Roger's spec). Visual matches FSK441Plus: vertical
                // label-over-value pairs separated by ui.separator().
                //
                // Distance + GC bearing always shown.
                // Scatter arc shown only when compute_scatter_arc returns
                // Some — for very short paths the arc is undefined.
                if !self.their_call.is_empty() {
                    if let (Some(my_grid), Some(their_grid)) = (
                        self.settings.station.grid.as_deref(),
                        self.their_grid.as_deref(),
                    ) {
                        if let (Some(my_qth), Some(their_qth)) = (
                            crate::geo::Qth::from_maidenhead(my_grid),
                            crate::geo::Qth::from_maidenhead(their_grid),
                        ) {
                            let dist = crate::geo::gc_distance_km(
                                my_qth.lat, my_qth.lon,
                                their_qth.lat, their_qth.lon);
                            let bearing = crate::geo::gc_bearing_deg(
                                my_qth.lat, my_qth.lon,
                                their_qth.lat, their_qth.lon);

                            ui.separator();
                            ui.vertical(|ui| {
                                ui.label(egui::RichText::new("Distance")
                                    .small()
                                    .color(egui::Color32::from_gray(180)));
                                ui.label(egui::RichText::new(format!(
                                    "{:.0} km  {:.0}°", dist, bearing))
                                    .monospace()
                                    .color(egui::Color32::from_rgb(180, 180, 255)));
                            });

                            if let Some(arc) = crate::geo::compute_scatter_arc(
                                my_qth.lat, my_qth.lon,
                                their_qth.lat, their_qth.lon,
                                self.settings.station.ant_bw_horiz as f64,
                            ) {
                                ui.separator();
                                ui.vertical(|ui| {
                                    ui.label(egui::RichText::new("Scatter Arc")
                                        .small()
                                        .color(egui::Color32::from_gray(180)));
                                    ui.label(egui::RichText::new(format!(
                                        "{:.0}°–{:.0}°", arc.arc_min, arc.arc_max))
                                        .monospace()
                                        .color(egui::Color32::from_rgb(255, 200, 80)));
                                });

                                ui.vertical(|ui| {
                                    ui.label(egui::RichText::new("Beam A / B")
                                        .small()
                                        .color(egui::Color32::from_gray(180)));
                                    let txt = match (arc.beam_a, arc.beam_b) {
                                        (Some(a), Some(b)) => format!("{:.0}° / {:.0}°", a, b),
                                        (Some(c), None)    => format!("{:.0}°", c),
                                        _                  => "—".into(),
                                    };
                                    ui.label(egui::RichText::new(txt)
                                        .monospace()
                                        .color(egui::Color32::from_rgb(100, 255, 180)));
                                });

                                ui.vertical(|ui| {
                                    ui.label(egui::RichText::new("El")
                                        .small()
                                        .color(egui::Color32::from_gray(180)));
                                    // Red when the midpoint elevation
                                    // sits below the upper half of the
                                    // antenna's vertical lobe (i.e. main
                                    // lobe is pointing too low for this
                                    // path). Otherwise neutral grey.
                                    let half_v = self.settings.station.ant_bw_vert as f64 / 2.0;
                                    let el_col = if arc.midpoint_el <= half_v {
                                        egui::Color32::from_rgb(255, 120, 80)
                                    } else {
                                        egui::Color32::from_gray(200)
                                    };
                                    ui.label(egui::RichText::new(format!(
                                        "{:.0}°", arc.midpoint_el))
                                        .monospace()
                                        .color(el_col));
                                });
                            }
                        }
                    }
                }

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.button("⏹ STOP").clicked() {
                        self.stop_tx();
                        self.their_call.clear();
                        self.their_grid = None;
                        self.in_active_qso = false;
                        self.qso_started_at = None;
                        self.stop();
                    }
                    if self.in_active_qso || !self.their_call.is_empty() {
                        ui.add_space(10.0);
                        let qso_green = egui::Color32::from_rgb(56, 120, 70);
                        if let Some(started) = self.qso_started_at {
                            let elapsed = started.elapsed().as_secs();
                            ui.label(egui::RichText::new(format!("{:02}:{:02}", elapsed / 60, elapsed % 60))
                                .monospace().small().color(egui::Color32::GRAY));
                        }
                        let wd_color = if self.watchdog_enabled { qso_green }
                            else { egui::Color32::from_rgb(100, 40, 40) };
                        let wd_label = if self.watchdog_enabled { "⏰ IN QSO  WD ON" } else { "⏰ IN QSO  WD OFF" };
                        if ui.add_sized([150.0, 30.0],
                            egui::Button::new(egui::RichText::new(wd_label).strong().color(egui::Color32::WHITE))
                                .fill(wd_color)).clicked()
                        {
                            self.watchdog_enabled = !self.watchdog_enabled;
                        }
                    }
                });
            });
        });

        // ── Central: three columns ────────────────────────────────────────────
        egui::CentralPanel::default().show(ctx, |ui| {
            // Click capture — set when user left-clicks a row in RX or SPOTS.
            // Carries (full-text, optional-snr-from-LogEntry, slot-parity).
            // The slot-parity (0=Even, 1=Odd) tells us which slot the
            // partner's burst arrived in. Auto-answer logic flips our
            // TX parity to the OPPOSITE so we transmit when partner is
            // listening (matches MSK2K's auto-slot behaviour).
            let mut clicked_row: Option<(String, Option<i16>, Option<u8>)> = None;
            ui.columns(3, |cols| {
                for i in 0..3 {
                    cols[i].vertical(|ui| {
                        ui.horizontal(|ui| {
                            let label = match i { 0 => "📥 RX", 1 => "📤 TX", _ => "🎯 SPOTS" };
                            if i == 1 && self.is_transmitting {
                                ui.heading(egui::RichText::new(label).color(egui::Color32::from_rgb(255, 50, 50)));
                            } else {
                                ui.heading(label);
                            }
                            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                if ui.small_button("🗑").clicked() {
                                    match i { 0 => self.rx_log.clear(), 1 => self.tx_log.clear(), _ => self.cq_log.clear() };
                                }
                            });
                        });

                        let id = match i { 0 => "rx_sc", 1 => "tx_sc", _ => "cq_sc" };
                        // ScrollArea sizing: auto_shrink([false, false])
                        // tells the area to claim ALL the available
                        // height in the parent column rather than
                        // shrinking to fit content. Without this the
                        // column grows vertically with content, pushing
                        // the central panel beyond the viewport (which
                        // is why the bug looked like "scrolling
                        // disappeared" — content was overflowing the
                        // panel rather than the scroll area handling
                        // it). The previous max_height(available_height)
                        // approach didn't work because available_height
                        // inside cols+vertical doesn't bound a fixed
                        // viewport — it just reports remaining space
                        // which is unbounded as content grows.
                        egui::ScrollArea::vertical()
                            .id_source(id)
                            .auto_shrink([false, false])
                            .show(ui, |ui| {
                            ui.set_min_width(ui.available_width());
                            let log = match i { 0 => &self.rx_log, 1 => &self.tx_log, _ => &self.cq_log };
                            for entry in log.iter().rev() {
                                if i == 1 {
                                    // TX rows: tint by the operator's CURRENT
                                    // TX parity setting (this is the slot we
                                    // transmit IN, by definition). The colour
                                    // mirrors the parity selector in the top
                                    // bar so an operator can confirm at a
                                    // glance which slot their TX lives in.
                                    // Edge bar only (right edge for TX rows,
                                    // mirroring the left-edge bar on RX rows);
                                    // no background tint, default window
                                    // backdrop runs behind the text.
                                    let tx_parity = tx_parity_to_int(
                                        &self.settings.station.tx_parity);
                                    let tx_bar = parity_accent(tx_parity);
                                    ui.allocate_ui(egui::vec2(ui.available_width(), 18.0), |ui| {
                                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                            if entry.colored {
                                                let frame_resp = egui::Frame::none()
                                                    .inner_margin(egui::Margin::symmetric(6.0, 2.0))
                                                    .show(ui, |ui| { ui.monospace(&entry.text); });
                                                let rect = ui.max_rect();
                                                let bar = egui::Rect::from_min_max(
                                                    egui::pos2(rect.max.x - 3.0, rect.min.y),
                                                    egui::pos2(rect.max.x, rect.max.y));
                                                ui.painter().rect_filled(bar, 0.0, tx_bar);
                                                frame_resp.response.context_menu(|ui| {
                                                    if ui.button("📋 Copy TX").clicked() {
                                                        ui.output_mut(|o| o.copied_text = entry.text.clone());
                                                        ui.close_menu();
                                                    }
                                                });
                                            } else {
                                                let lab = ui.monospace(&entry.text);
                                                lab.context_menu(|ui| {
                                                    if ui.button("📋 Copy TX").clicked() {
                                                        ui.output_mut(|o| o.copied_text = entry.text.clone());
                                                        ui.close_menu();
                                                    }
                                                });
                                            }
                                        });
                                    });
                                } else {
                                    ui.allocate_ui(egui::vec2(ui.available_width(), 18.0), |ui| {
                                        let rect = ui.available_rect_before_wrap();
                                        // Slot parity is conveyed by the LEFT-EDGE BAR
                                        // colour only — no background tint. Keeps the
                                        // text legible on the default window backdrop
                                        // while still giving an at-a-glance view of
                                        // which parity each decode came from. Useful
                                        // for diagnosing TX-parity-vs-band-activity
                                        // mismatches without crowding the visual.
                                        //
                                        // Fall-back rules:
                                        //   parity known → coloured bar
                                        //   parity unknown but `entry.colored` true
                                        //     (legacy "highlight this row" flag)
                                        //     → green bar
                                        //   neither → no bar
                                        let parity = entry.rx_slot;
                                        let bar_colour = match (parity, entry.colored) {
                                            (Some(p), _) => parity_accent(p),
                                            (None, true) =>
                                                egui::Color32::from_rgb(31, 143, 58),
                                            (None, false) => egui::Color32::TRANSPARENT,
                                        };
                                        let draw_bar = parity.is_some() || entry.colored;
                                        egui::Frame::none().inner_margin(egui::Margin::symmetric(10.0, 2.0)).show(ui, |ui| {
                                            if draw_bar {
                                                let bar = egui::Rect::from_min_max(
                                                    egui::pos2(rect.min.x, rect.min.y),
                                                    egui::pos2(rect.min.x + 3.0, rect.max.y));
                                                ui.painter().rect_filled(bar, 0.0, bar_colour);
                                            }
                                            ui.horizontal(|ui| {
                                                let label_resp = ui.selectable_label(false, &entry.text);
                                                // Capture click result BEFORE context_menu consumes the response
                                                let clicked_now = label_resp.clicked();
                                                // Right-click → copy decode (matches FSK441+ pattern)
                                                label_resp.context_menu(|ui| {
                                                    if ui.button("📋 Copy decode").clicked() {
                                                        ui.output_mut(|o| o.copied_text = entry.text.clone());
                                                        ui.close_menu();
                                                    }
                                                    if ui.button("📋 Copy with timestamp").clicked() {
                                                        let combined = if entry.timestamp.is_empty() {
                                                            entry.text.clone()
                                                        } else {
                                                            format!("{}  {}", entry.timestamp, entry.text)
                                                        };
                                                        ui.output_mut(|o| o.copied_text = combined);
                                                        ui.close_menu();
                                                    }
                                                    if let Some(call) = extract_callsign(&entry.text) {
                                                        if ui.button(format!("📋 Copy call: {}", call)).clicked() {
                                                            ui.output_mut(|o| o.copied_text = call);
                                                            ui.close_menu();
                                                        }
                                                    }
                                                });
                                                // Left-click: store the full row text so the
                                                // post-loop handler can decide whether it's a
                                                // CQ (→ AnswerCq) or just a TARGET selection.
                                                if clicked_now {
                                                    clicked_row = Some((
                                                        entry.text.clone(),
                                                        entry.snr_db,
                                                        entry.rx_slot,
                                                    ));
                                                }
                                                if !entry.timestamp.is_empty() {
                                                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                                        ui.label(egui::RichText::new(&entry.timestamp).small().color(egui::Color32::GRAY));
                                                    });
                                                }
                                            });
                                        });
                                    });
                                }
                            }
                        });
                    });
                }
            });
            // Post-render: handle a clicked row from RX/SPOTS.
            //
            // If the clicked row was a CQ (e.g. "CQ I5YDI JN54"), we
            // immediately answer it with `Intent::AnswerCq` — matches MSK2K's
            // click-to-answer flow. The grid is parsed from the message body
            // and passed to the engine for ADIF.  The signal report we send
            // back is the SNR we computed when we decoded their CQ
            // (canonical WSJT-X formula, quantised to MSK144's 2-dB grid).
            //
            // If it was anything else (a non-CQ decode), we just put the
            // station's call into TARGET so the user can decide whether to
            // press CALL (cold call) or CALL CQ.
            if let Some((row, snr_opt, rx_parity_opt)) = clicked_row {
                if let Some((call, grid_opt, is_cq)) = parse_clicked_row(&row) {
                    if is_cq {
                        // Auto-slot: flip our TX parity to the OPPOSITE
                        // of the partner's RX slot so we transmit
                        // when they're listening, not when they're
                        // transmitting. Matches MSK2K's auto-slot
                        // behaviour. If the row didn't carry a parity
                        // (e.g. accumulator decode without burst time)
                        // we leave the operator's manual parity setting
                        // alone — they'll have already chosen the right
                        // slot if they were paying attention.
                        if let Some(rx_par) = rx_parity_opt {
                            let want_tx_par = 1 - rx_par;
                            let want_label = if want_tx_par == 1 { "Odd" } else { "Even" };
                            if self.settings.station.tx_parity != want_label {
                                log::info!(
                                    "[UI] Auto-slot: partner heard in parity {} → \
                                     setting our TX to {}",
                                    rx_par, want_label);
                                self.settings.station.tx_parity = want_label.to_string();
                                self.settings_dirty = true;
                                // Push to transmitter + engine so the
                                // change is live for the next slot.
                                self.sync_transmitter_config();
                            }
                        } else {
                            log::info!(
                                "[UI] Auto-slot: clicked row has no parity \
                                 (averaging/accumulator decode); keeping current \
                                 TX parity {}",
                                self.settings.station.tx_parity);
                        }

                        // Use SNR-based report if we have one, otherwise
                        // fall back to the engine's current my_report (default).
                        let rpt = snr_opt.unwrap_or(self.qso.my_report);
                        // Format with explicit sign so negative reports
                        // render cleanly (e.g. "-04" not "+-04").
                        let rpt_str = if rpt >= 0 {
                            format!("+{:02}", rpt)
                        } else {
                            format!("-{:02}", -rpt)
                        };
                        log::info!("[UI] Clicked CQ → answering: {} grid={:?} rpt={}",
                            call, grid_opt, rpt_str);
                        self.qso.set_my_report(rpt);
                        self.start_answer_cq(call, grid_opt);
                    } else {
                        // Non-CQ row clicked. Two sub-cases:
                        //
                        // (a) The decoded message was directed AT US
                        //     ("<my_call> <their_call> [grid]") —
                        //     they've already heard us, so the right
                        //     answer is Tx2 (call+report) NOT Tx1
                        //     (bare call). Engage AnswerCq exactly
                        //     as for a CQ click, using the row's SNR.
                        //     This avoids the "click row, press CALL,
                        //     but rig still sends Tx1 bare call"
                        //     dance — one click should engage the
                        //     QSO and send the right message.
                        //
                        // (b) Third-party exchange ("OTHER1 OTHER2
                        //     [report]") — neither call is ours.
                        //     Just populate TARGET; let the operator
                        //     decide whether to call OTHER2 manually.
                        //
                        // We re-parse the row's tokens here because
                        // parse_clicked_row only returns the partner
                        // call for non-CQ rows; we need msg[0] to
                        // know whether the row was addressed to us,
                        // and msg[2] for any grid.
                        let row_parts: Vec<&str> = row
                            .split_whitespace().collect();
                        let row_msg_start = if !row_parts.is_empty()
                            && row_parts[0].len() >= 8
                            && row_parts[0].as_bytes()[2] == b':'
                        {
                            row_parts.iter().position(|t| *t == "Hz")
                                .map(|i| (i + 1).min(row_parts.len()))
                                .unwrap_or(row_parts.len())
                        } else { 0 };
                        let row_msg: &[&str] = &row_parts[row_msg_start..];
                        let directed_at_me = !self.my_call.is_empty()
                            && row_msg.first()
                                .map(|t| t.eq_ignore_ascii_case(&self.my_call))
                                .unwrap_or(false);

                        if directed_at_me && row_msg.len() >= 2 {
                            // Their call is row_msg[1]. Grid (if
                            // any) is row_msg[2] when shaped like a
                            // 4-char Maidenhead locator (AB12).
                            let their_call = row_msg[1].to_uppercase();
                            let grid_opt: Option<String> = row_msg.get(2)
                                .filter(|s| is_grid_token(s))
                                .map(|s| s.to_uppercase());

                            let rpt = snr_opt.unwrap_or(self.qso.my_report);
                            let rpt_str = if rpt >= 0 {
                                format!("+{:02}", rpt)
                            } else {
                                format!("-{:02}", -rpt)
                            };
                            log::info!(
                                "[UI] Clicked direct call → answering: \
                                 {} grid={:?} rpt={}",
                                their_call, grid_opt, rpt_str);
                            self.qso.set_my_report(rpt);
                            self.start_answer_cq(their_call, grid_opt);
                        } else {
                            // Third-party exchange. Set TARGET only.
                            if self.their_call != call {
                                self.their_grid = None;
                            }
                            self.their_call = call;
                        }
                    }
                }
            }
        });

        // ── Logbook footer ────────────────────────────────────────────────────
        egui::TopBottomPanel::bottom("log_footer").resizable(true).min_height(30.0).show(ctx, |ui| {
            let arrow = if self.qso_log_expanded { "▼" } else { "▶" };
            if ui.add(egui::Button::new(egui::RichText::new(format!("Logbook {}", arrow)).color(egui::Color32::WHITE)).frame(false)).clicked() {
                self.qso_log_expanded = !self.qso_log_expanded;
                self.settings_dirty = true;
            }
            if self.qso_log_expanded {
                ui.add_space(5.0);
                ui.separator();
                egui::Grid::new("log_header_grid").num_columns(8).spacing([8.0, 4.0]).show(ui, |ui| {
                    ui.add_sized([80.0, 20.0], egui::Label::new(egui::RichText::new("DATE").strong().color(egui::Color32::GRAY)));
                    ui.add_sized([65.0, 20.0], egui::Label::new(egui::RichText::new("START (Z)").strong().color(egui::Color32::GRAY)));
                    ui.add_sized([65.0, 20.0], egui::Label::new(egui::RichText::new("END (Z)").strong().color(egui::Color32::GRAY)));
                    ui.add_sized([75.0, 20.0], egui::Label::new(egui::RichText::new("FREQ").strong().color(egui::Color32::GRAY)));
                    ui.add_sized([90.0, 20.0], egui::Label::new(egui::RichText::new("STATION").strong().color(egui::Color32::GRAY)));
                    ui.add_sized([50.0, 20.0], egui::Label::new(egui::RichText::new("GRID").strong().color(egui::Color32::GRAY)));
                    ui.add_sized([40.0, 20.0], egui::Label::new(egui::RichText::new("SENT").strong().color(egui::Color32::GRAY)));
                    ui.add_sized([40.0, 20.0], egui::Label::new(egui::RichText::new("RCVD").strong().color(egui::Color32::GRAY)));
                    ui.end_row();
                });
                ui.add_space(2.0);
                ui.separator();

                if self.logged_qsos.is_empty() {
                    // Empty state — friendlier than the old "TX path
                    // not yet implemented" message. ADIF logging IS
                    // implemented; the logbook just has no QSOs yet
                    // to show.
                    ui.label(egui::RichText::new(
                        "No QSOs logged yet. Completed QSOs will appear here \
                         and in ~/msk144plus_log.adi.")
                        .italics().color(egui::Color32::GRAY));
                } else {
                    // Render the logged QSOs list (newest first). Use a
                    // ScrollArea so we don't blow out the panel height
                    // once the list grows past a few entries. Cap at
                    // displaying the most recent 100 — past that it's
                    // mostly historical, the operator can open the
                    // ADIF file directly for the full archive.
                    let to_show = self.logged_qsos.iter().take(100);
                    egui::ScrollArea::vertical()
                        .max_height(200.0)
                        .show(ui, |ui| {
                            egui::Grid::new("log_rows_grid")
                                .num_columns(8)
                                .spacing([8.0, 2.0])
                                .show(ui, |ui| {
                                    for rec in to_show {
                                        let date  = rec.display_date();
                                        let t_on  = rec.display_time_on();
                                        let t_off = rec.display_time_off();
                                        let freq  = rec.freq
                                            .map(|f| format!("{:.3}", f))
                                            .unwrap_or_else(|| rec.band.clone());
                                        let grid  = rec.gridsquare.clone()
                                            .unwrap_or_default();
                                        ui.add_sized([80.0, 18.0],
                                            egui::Label::new(egui::RichText::new(&date).monospace()));
                                        ui.add_sized([65.0, 18.0],
                                            egui::Label::new(egui::RichText::new(&t_on).monospace()));
                                        ui.add_sized([65.0, 18.0],
                                            egui::Label::new(egui::RichText::new(&t_off).monospace()));
                                        ui.add_sized([75.0, 18.0],
                                            egui::Label::new(egui::RichText::new(&freq).monospace()));
                                        ui.add_sized([90.0, 18.0],
                                            egui::Label::new(egui::RichText::new(&rec.call)
                                                .strong().color(egui::Color32::from_rgb(180, 220, 255))));
                                        ui.add_sized([50.0, 18.0],
                                            egui::Label::new(egui::RichText::new(&grid).monospace()));
                                        ui.add_sized([40.0, 18.0],
                                            egui::Label::new(egui::RichText::new(&rec.rst_sent).monospace()));
                                        ui.add_sized([40.0, 18.0],
                                            egui::Label::new(egui::RichText::new(&rec.rst_rcvd).monospace()));
                                        ui.end_row();
                                    }
                                });
                        });
                }
            }
        });

        // ── Status strip ──────────────────────────────────────────────────────
        // RMS readout removed — replaced by the amplitude trace
        // overlaid along the bottom of the spectrum panel below.
        // This matches MSHV's signal-strength presentation: a moving
        // envelope plot rather than a numerical level meter.
        egui::TopBottomPanel::bottom("status_strip").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.small(format!("STATE: {}", self.current_state));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if self.last_corr > 0.0 {
                        ui.small(format!("Quality: {}%", (self.last_corr * 100.0) as i32));
                    }
                });
            });
        });

        // ── Realtime audio spectrum ───────────────────────────────────────────
        // Sits between the main panes (RX/TX/SPOTS) and the status strip,
        // spanning the full window width. Each FFT column is one ~85 ms
        // snapshot of the input audio; the panel resets on every 15 s
        // slot boundary and freezes during our own TX (see drain_spectrum).
        // Visual style matches FSK441Plus's spectrogram: WSJT-style
        // dark-blue background, blue→cyan→yellow→white heat colour.
        // No tone markers / centre-frequency lines per Roger's spec.
        egui::TopBottomPanel::bottom("spectrum_panel").show(ctx, |ui| {
            let wf_height = 120.0f32;
            let avail_w = ui.available_width();
            let (rect, _) = ui.allocate_exact_size(
                egui::vec2(avail_w, wf_height),
                egui::Sense::hover(),
            );
            let painter = ui.painter_at(rect);
            painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(0, 0, 20));

            let n_cols = self.spectrum_columns.len();
            if n_cols > 0 {
                // Columns per slot: 15 s × 12000 Hz / 1024 ≈ 175.78
                // Rounded up so the layout matches the slot capacity
                // and a fully-populated slot fills almost exactly the
                // panel width.
                // Columns-per-slot divisor for the layout. We start
                // with the static estimate (15 s × 12 kHz / 1024 ≈
                // 176) so the very first slot has a sensible width,
                // then switch to the observed maximum once we've
                // actually seen one or more complete slots. The
                // observed value is correct regardless of native
                // audio rate or rubato chunk sizing — resampling
                // from 11.025 kHz produces ~161 cols/slot, NOT 176,
                // so the static estimate would leave ~9 % of the
                // panel width empty. See drain_spectrum for the
                // calibration logic.
                let slot_period_secs = self.settings.station.slot_period_secs.max(1) as f32;
                let static_estimate =
                    ((slot_period_secs * 12000.0 / 1024.0) as usize).max(1);
                let max_cols = if self.spectrum_max_cols_seen > 0 {
                    // We've seen at least one complete slot — trust it.
                    // Guard with n_cols in case the current slot is
                    // somehow longer (clock skew, etc.).
                    self.spectrum_max_cols_seen.max(n_cols)
                } else {
                    // No complete slot yet — fall back to static
                    // estimate so the first partial slot doesn't render
                    // with absurdly-wide columns.
                    static_estimate.max(n_cols)
                };
                let col_w = avail_w / max_cols as f32;
                let bin_h = wf_height / crate::spectrum::DISPLAY_BINS as f32;

                for (ci, col) in self.spectrum_columns.iter().enumerate() {
                    let x = rect.left() + ci as f32 * col_w;
                    for (bi, &v) in col.bins.iter().enumerate() {
                        // bi=0 is DC (bottom of panel),
                        // bi=DISPLAY_BINS-1 is ~3 kHz (top).
                        let y = rect.bottom() - (bi as f32 + 1.0) * bin_h;
                        if v > 0.05 {
                            painter.rect_filled(
                                egui::Rect::from_min_size(
                                    egui::pos2(x, y),
                                    egui::vec2(col_w.max(1.5), bin_h.max(1.0)),
                                ),
                                0.0,
                                crate::spectrum::heat_color(v),
                            );
                        }
                    }
                }

                // Amplitude trace along the bottom of the panel.
                // Replaces the numerical RMS readout in the status
                // strip — visually equivalent to MSHV's signal-
                // strength view. dB scale: noise floor at the bottom
                // of the trace area, 40 dB dynamic range above.
                //
                // Noise floor = 20th percentile of the RMS values
                // collected so far this slot. Cheap and robust to
                // bursts (which sit at the top end and don't shift
                // the lower percentile much). Re-estimated each
                // frame so the trace adapts as the slot fills.
                if self.spectrum_columns.len() > 1 {
                    let trace_h  = wf_height * 0.28;
                    let baseline = rect.bottom();
                    let db_range = 40.0f32;

                    let mut sorted: Vec<f32> = self.spectrum_columns
                        .iter()
                        .map(|c| c.rms)
                        .collect();
                    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(
                        std::cmp::Ordering::Equal));
                    let noise = sorted[sorted.len() / 5].max(1e-7);

                    let mut pts: Vec<egui::Pos2> = Vec::with_capacity(n_cols);
                    for (ci, col) in self.spectrum_columns.iter().enumerate() {
                        let x  = rect.left() + ci as f32 * col_w + col_w * 0.5;
                        let db = 20.0 * (col.rms / noise).log10();
                        let norm = (db / db_range).clamp(0.0, 1.0);
                        let y  = baseline - norm * trace_h;
                        pts.push(egui::pos2(x, y));
                    }
                    for i in 1..pts.len() {
                        painter.line_segment(
                            [pts[i - 1], pts[i]],
                            egui::Stroke::new(1.5,
                                egui::Color32::from_rgba_unmultiplied(
                                    0, 220, 80, 210)),
                        );
                    }
                }
            }
        });

        // ── Settings dialog ───────────────────────────────────────────────────
        if self.settings_open {
            let mut open = self.settings_open;
            egui::Window::new("Settings")
                .open(&mut open)
                .default_width(540.0)
                .show(ctx, |ui| {
                    ui.heading("Station");
                    ui.horizontal(|ui| {
                        ui.label("My call:");
                        if ui.text_edit_singleline(&mut self.my_call).changed() {
                            self.my_call = self.my_call.to_uppercase();
                            self.settings_dirty = true;
                        }
                    });
                    ui.separator();
                    ui.heading("Audio");
                    ui.horizontal(|ui| {
                        ui.label("Input:");
                        let cur = self.selected_device.clone().unwrap_or_else(|| "(default)".into());
                        let resp = egui::ComboBox::from_id_source("dev_in")
                            .selected_text(truncate(&cur, 32))
                            .width(260.0)
                            .show_ui(ui, |ui| {
                                let mut clicked = None;
                                for d in &self.available_devices {
                                    let sel = self.selected_device.as_deref() == Some(d);
                                    if ui.selectable_label(sel, truncate(d, 50)).clicked() {
                                        clicked = Some(d.clone());
                                    }
                                }
                                clicked
                            });
                        if let Some(Some(new_dev)) = resp.inner {
                            self.selected_device = Some(new_dev);
                            self.settings_dirty = true;
                        }
                        if ui.button("⟳ Refresh").clicked() {
                            self.available_devices = list_input_devices();
                        }
                        if ui.button("Audio devices…").clicked() {
                            self.audio_diag_lines = list_all_devices_diagnostic();
                            self.show_audio_diag = true;
                        }
                    });

                    // TX Output device picker
                    ui.horizontal(|ui| {
                        ui.label("Output:");
                        let cur_out = self.settings.station.tx_output_device.clone()
                            .unwrap_or_else(|| "(default)".into());
                        let resp = egui::ComboBox::from_id_source("dev_out")
                            .selected_text(truncate(&cur_out, 32))
                            .width(260.0)
                            .show_ui(ui, |ui| {
                                let mut clicked: Option<String> = None;
                                for d in &self.available_output_devices {
                                    let sel = self.settings.station.tx_output_device
                                        .as_deref() == Some(d);
                                    if ui.selectable_label(sel, truncate(d, 50)).clicked() {
                                        clicked = Some(d.clone());
                                    }
                                }
                                clicked
                            });
                        if let Some(Some(new_dev)) = resp.inner {
                            self.settings.station.tx_output_device = Some(new_dev);
                            self.settings_dirty = true;
                            self.sync_transmitter_config();
                        }
                        if ui.button("⟳").clicked() {
                            self.available_output_devices = dx_runtime::list_output_devices();
                        }
                    });

                    // TX level + PTT delay
                    ui.horizontal(|ui| {
                        ui.label("TX level:");
                        let mut lvl = self.settings.station.tx_level;
                        if ui.add(egui::Slider::new(&mut lvl, 0.0..=1.0)
                            .fixed_decimals(2)).changed()
                        {
                            self.settings.station.tx_level = lvl;
                            self.settings_dirty = true;
                            self.sync_transmitter_config();
                        }
                        ui.add_space(20.0);
                        ui.label("PTT delay (ms):");
                        let mut d = self.settings.station.ptt_delay_ms as i32;
                        if ui.add(egui::DragValue::new(&mut d)
                            .clamp_range(0..=1000)).changed()
                        {
                            self.settings.station.ptt_delay_ms = d.max(0) as u32;
                            self.settings_dirty = true;
                            self.sync_transmitter_config();
                        }
                    });

                    // Grid for CQ messages
                    ui.horizontal(|ui| {
                        ui.label("Grid:");
                        let mut grid = self.settings.station.grid.clone().unwrap_or_default();
                        if ui.text_edit_singleline(&mut grid).changed() {
                            grid = grid.to_uppercase();
                            self.settings.station.grid = if grid.is_empty() { None } else { Some(grid) };
                            self.settings_dirty = true;
                        }
                        ui.label(egui::RichText::new("(4 or 6 chars; used in CQ messages)")
                            .small().color(egui::Color32::GRAY));
                    });

                    // Antenna beamwidth — drives the scatter-arc inset
                    // for optimal A/B beam headings shown next to TARGET.
                    // Default 50° / 50° matches a typical 4-element 2 m
                    // yagi. Roger can tune for actual antenna later.
                    ui.horizontal(|ui| {
                        ui.label("Antenna H/V beamwidth:");
                        let mut h = self.settings.station.ant_bw_horiz;
                        if ui.add(egui::DragValue::new(&mut h)
                            .speed(1.0)
                            .clamp_range(5.0..=180.0)
                            .suffix("°")).changed()
                        {
                            self.settings.station.ant_bw_horiz = h;
                            self.settings_dirty = true;
                        }
                        let mut v = self.settings.station.ant_bw_vert;
                        if ui.add(egui::DragValue::new(&mut v)
                            .speed(1.0)
                            .clamp_range(5.0..=180.0)
                            .suffix("°")).changed()
                        {
                            self.settings.station.ant_bw_vert = v;
                            self.settings_dirty = true;
                        }
                        ui.label(egui::RichText::new("(used for scatter-arc beam inset)")
                            .small().color(egui::Color32::GRAY));
                    });

                    ui.separator();
                    ui.heading("Logger broadcast");
                    ui.label(egui::RichText::new(
                        "Send each completed QSO over UDP to a third-party logger \
                         (Logger32, N1MM+, JTAlert, HRD, DX Lab Suite, Cloudlog, …). \
                         Uses the WSJT-X UDP format. Trigger is identical to the \
                         local ADIF write — the QSO state machine fires on receipt \
                         of partner's final 73 or RR73.")
                        .small()
                        .color(egui::Color32::GRAY));
                    ui.horizontal(|ui| {
                        let mut enabled = self.settings.station.udp_logging_enabled;
                        if ui.checkbox(&mut enabled, "Send to UDP logger").changed() {
                            self.settings.station.udp_logging_enabled = enabled;
                            self.settings_dirty = true;
                        }
                    });
                    ui.horizontal(|ui| {
                        ui.label("Host:");
                        let mut host = self.settings.station.udp_logging_host.clone();
                        if ui.add(egui::TextEdit::singleline(&mut host)
                            .desired_width(180.0)).changed()
                        {
                            self.settings.station.udp_logging_host = host;
                            self.settings_dirty = true;
                        }
                        ui.add_space(20.0);
                        ui.label("Port:");
                        let mut port = self.settings.station.udp_logging_port;
                        if ui.add(egui::DragValue::new(&mut port)
                            .speed(1.0)
                            .clamp_range(1024..=65535)).changed()
                        {
                            self.settings.station.udp_logging_port = port;
                            self.settings_dirty = true;
                        }
                        ui.label(egui::RichText::new(
                            "(WSJT-X / MSHV default: 127.0.0.1:2237)")
                            .small().color(egui::Color32::GRAY));
                    });

                    ui.separator();
                    ui.heading("PSK Reporter");
                    ui.label(egui::RichText::new(
                        "Submit decode reports to PSK Reporter (the global \
                         propagation-data aggregator). Spots appear on the \
                         pskreporter.info map within ~10 minutes. Same \
                         destination used by WSJT-X and MSHV. Requires a \
                         valid callsign + grid (4 or 6 chars; 6 plots more \
                         accurately) AND a rig CAT connection so the \
                         absolute frequency is known.")
                        .small()
                        .color(egui::Color32::GRAY));
                    ui.horizontal(|ui| {
                        let mut enabled = self.settings.station.psk_reporter_enabled;
                        if ui.checkbox(&mut enabled, "Submit decodes to PSK Reporter")
                            .changed()
                        {
                            self.settings.station.psk_reporter_enabled = enabled;
                            self.settings_dirty = true;
                            // Live toggle — spawn or stop immediately
                            // so the operator doesn't have to restart
                            // the listener for the change to take
                            // effect.
                            if enabled {
                                if self.is_listening {
                                    self.maybe_spawn_psk_reporter();
                                }
                            } else {
                                self.stop_psk_reporter();
                            }
                        }
                    });
                    // Antenna free-text field. Shown to PSK Reporter
                    // viewers in the station-info popup.
                    ui.horizontal(|ui| {
                        ui.label("Antenna:");
                        let mut ant = self.settings.station.psk_reporter_antenna.clone();
                        if ui.add(egui::TextEdit::singleline(&mut ant)
                            .desired_width(360.0)
                            .hint_text("e.g. 4-element 2 m yagi at 12 m"))
                            .changed()
                        {
                            self.settings.station.psk_reporter_antenna = ant;
                            self.settings_dirty = true;
                            // Antenna change requires reporter restart
                            // (it's part of the receiver record sent
                            // once at session start). Kill and let
                            // the next spot trigger respawn — but
                            // for clean live-update we explicitly
                            // restart now.
                            if self.settings.station.psk_reporter_enabled
                                && self.is_listening
                            {
                                self.stop_psk_reporter();
                                self.maybe_spawn_psk_reporter();
                            }
                        }
                    });
                    // Status indicator — useful feedback that the
                    // client actually spawned (vs being silently
                    // skipped due to missing call/grid).
                    ui.horizontal(|ui| {
                        let (label, col) = if self.psk_reporter.is_some() {
                            ("● Active", egui::Color32::from_rgb(80, 200, 120))
                        } else if self.settings.station.psk_reporter_enabled {
                            ("○ Enabled but inactive (need callsign + grid + listener)",
                                egui::Color32::from_rgb(220, 140, 60))
                        } else {
                            ("○ Off", egui::Color32::GRAY)
                        };
                        ui.label(egui::RichText::new("Status:")
                            .small().color(egui::Color32::GRAY));
                        ui.label(egui::RichText::new(label).small().color(col));
                    });
                    // Grid-length hint.
                    if self.settings.station.psk_reporter_enabled {
                        let grid = self.settings.station.grid.clone()
                            .unwrap_or_default();
                        if grid.len() == 4 {
                            ui.label(egui::RichText::new(
                                "Note: 4-char grid plots at the centre of \
                                 the larger square (~150 km wide). For \
                                 accurate plotting set a 6-char grid in \
                                 Station settings.")
                                .small()
                                .color(egui::Color32::from_rgb(220, 180, 60)));
                        }
                    }

                    ui.separator();
                    ui.heading("Decoder");
                    ui.horizontal(|ui| {
                        ui.label("fc:");
                        ui.add(egui::DragValue::new(&mut self.fc_hz).speed(10.0).clamp_range(500.0..=2400.0));
                        ui.label("Hz");
                        ui.add_space(20.0);
                        ui.label("ntol:");
                        ui.add(egui::DragValue::new(&mut self.ntol_hz).speed(10.0).clamp_range(50.0..=500.0));
                        ui.label("Hz");
                        ui.add_space(20.0);
                        ui.label("Depth:");
                        egui::ComboBox::from_id_source("depth")
                            .selected_text(format!("{:?}", self.depth))
                            .show_ui(ui, |ui| {
                                ui.selectable_value(&mut self.depth, Depth::Fast, "Fast");
                                ui.selectable_value(&mut self.depth, Depth::Normal, "Normal");
                                ui.selectable_value(&mut self.depth, Depth::Deep, "Deep");
                            });
                    });

                    // Soft-bit accumulator (experimental — runs in parallel
                    // with the standard decoder; results tagged ` [A]`).
                    ui.horizontal(|ui| {
                        let mut accum_enabled = self.settings.decoder.accumulator_enabled;
                        if ui.checkbox(&mut accum_enabled,
                            "Soft-bit accumulator (experimental)").changed()
                        {
                            self.settings.decoder.accumulator_enabled = accum_enabled;
                            self.settings_dirty = true;
                            if let Ok(mut c) = self.cfg.lock() {
                                c.accumulator_enabled = accum_enabled;
                            }
                        }
                        ui.add_space(15.0);
                        ui.label("accum ntol:");
                        let mut accum_ntol = self.settings.decoder.accumulator_ntol_hz;
                        if ui.add(egui::DragValue::new(&mut accum_ntol)
                                .speed(5.0).clamp_range(20.0..=200.0))
                            .changed()
                        {
                            self.settings.decoder.accumulator_ntol_hz = accum_ntol;
                            self.settings_dirty = true;
                            if let Ok(mut c) = self.cfg.lock() {
                                c.accumulator_ntol_hz = accum_ntol;
                            }
                        }
                        ui.label("Hz (active during QSO)");
                    });

                    // Note: Sh msg (MSK40 short-message format) toggle
                    // moved to the main top bar, immediately after the
                    // TX parity selector. Operator can flip it without
                    // opening Settings.
                    ui.separator();
                    ui.heading("CAT (Hamlib / rigctld)");
                    ui.horizontal(|ui| {
                        let mut enabled = self.settings.station.hamlib_enabled;
                        if ui.checkbox(&mut enabled, "Enable CAT").changed() {
                            self.settings.station.hamlib_enabled = enabled;
                            self.settings_dirty = true;
                        }
                        // Status indicator
                        let (label, color) = if self.cat_connected {
                            ("● connected", egui::Color32::from_rgb(100, 200, 130))
                        } else if self.settings.station.hamlib_enabled {
                            ("○ connecting…", egui::Color32::from_rgb(220, 170, 80))
                        } else {
                            ("○ disabled", egui::Color32::GRAY)
                        };
                        ui.add_space(10.0);
                        ui.label(egui::RichText::new(label).color(color));
                    });

                    ui.horizontal(|ui| {
                        let mut auto = self.settings.station.auto_launch_rigctld;
                        if ui.checkbox(&mut auto,
                            "Auto-launch rigctld (uncheck if running externally)").changed()
                        {
                            self.settings.station.auto_launch_rigctld = auto;
                            self.settings_dirty = true;
                        }
                    });

                    // Rig model dropdown
                    ui.horizontal(|ui| {
                        ui.label("Rig:");
                        let current_label = dx_runtime::common_rig_models().iter()
                            .find(|(id, _)| *id == self.settings.station.rig_model.as_str())
                            .map(|(_, name)| (*name).to_string())
                            .unwrap_or_else(|| {
                                if self.settings.station.rig_model.is_empty() {
                                    "(select)".into()
                                } else {
                                    format!("model {}", self.settings.station.rig_model)
                                }
                            });
                        let resp = egui::ComboBox::from_id_source("rig_model")
                            .selected_text(truncate(&current_label, 28))
                            .width(220.0)
                            .show_ui(ui, |ui| {
                                let mut clicked: Option<String> = None;
                                for (id, name) in dx_runtime::common_rig_models() {
                                    let sel = self.settings.station.rig_model.as_str() == id;
                                    if ui.selectable_label(sel,
                                        format!("{} ({})", name, id)).clicked()
                                    {
                                        clicked = Some(id.to_string());
                                    }
                                }
                                clicked
                            });
                        if let Some(Some(new_model)) = resp.inner {
                            self.settings.station.rig_model = new_model;
                            self.settings_dirty = true;
                        }
                    });

                    // Serial port dropdown + baud + refresh
                    ui.horizontal(|ui| {
                        ui.label("Serial port:");
                        let cur_port = self.settings.station.rig_port.clone();
                        let label = if cur_port.is_empty() { "(select)".to_string() } else { cur_port.clone() };
                        let resp = egui::ComboBox::from_id_source("rig_port")
                            .selected_text(truncate(&label, 28))
                            .width(220.0)
                            .show_ui(ui, |ui| {
                                let mut clicked: Option<String> = None;
                                for p in &self.available_serial_ports {
                                    let sel = self.settings.station.rig_port == *p;
                                    if ui.selectable_label(sel, p).clicked() {
                                        clicked = Some(p.clone());
                                    }
                                }
                                clicked
                            });
                        if let Some(Some(new_port)) = resp.inner {
                            self.settings.station.rig_port = new_port;
                            self.settings_dirty = true;
                        }
                        if ui.button("⟳").clicked() {
                            self.available_serial_ports = dx_runtime::list_serial_ports();
                        }
                        ui.add_space(8.0);
                        ui.label("Baud:");
                        if ui.text_edit_singleline(&mut self.settings.station.rig_baud)
                            .changed()
                        {
                            self.settings_dirty = true;
                        }
                    });

                    ui.collapsing("Advanced (rigctld TCP)", |ui| {
                        ui.horizontal(|ui| {
                            ui.label("Host:");
                            if ui.text_edit_singleline(&mut self.settings.station.rigctld_host)
                                .changed()
                            {
                                self.settings_dirty = true;
                            }
                            ui.label("Port:");
                            let mut port = self.settings.station.rigctld_port as i32;
                            if ui.add(egui::DragValue::new(&mut port)
                                .clamp_range(1..=65535)).changed()
                            {
                                self.settings.station.rigctld_port = port.clamp(1, 65535) as u16;
                                self.settings_dirty = true;
                            }
                        });
                    });

                    if ui.button("Apply").clicked() {
                        {
                            let mut c = self.cfg.lock().unwrap();
                            c.fc_hz = self.fc_hz;
                            c.ntol_hz = self.ntol_hz;
                            c.depth = self.depth;
                        }
                        self.save_settings();
                    }
                });
            self.settings_open = open;
        }

        // ── Audio devices popup ───────────────────────────────────────────────
        if self.show_audio_diag {
            let mut open = self.show_audio_diag;
            egui::Window::new("Audio devices")
                .open(&mut open)
                .default_width(540.0)
                .show(ctx, |ui| {
                    ui.label("All devices visible to cpal on this host. \
                              IN = input-capable, OUT = output-capable. \
                              Asterisk marks the system default.");
                    ui.separator();
                    if ui.button("Refresh").clicked() {
                        self.audio_diag_lines = list_all_devices_diagnostic();
                    }
                    ui.add_space(4.0);
                    egui::ScrollArea::vertical().max_height(400.0).show(ui, |ui| {
                        for line in &self.audio_diag_lines {
                            ui.monospace(line);
                        }
                        if self.audio_diag_lines.is_empty() {
                            ui.label("(no devices)");
                        }
                    });
                });
            self.show_audio_diag = open;
        }

        ctx.request_repaint_after(std::time::Duration::from_millis(100));
    }

    /// Called by eframe when the user closes the window. Last chance to
    /// persist any unsaved state.
    fn on_exit(&mut self, _gl: Option<&eframe::glow::Context>) {
        log::info!("[UI] on_exit: persisting settings + flushing recorder");
        // Always save on exit, even if not dirty - capture window dims, etc.
        self.save_settings();
        // Flush any pending WAV captures whose post-roll never completed
        let _ = self.recorder.flush();
    }
}

fn truncate(s: &str, max: usize) -> String {
    if s.chars().count() > max {
        let mut t: String = s.chars().take(max - 1).collect();
        t.push('…');
        t
    } else {
        s.to_string()
    }
}

/// Parse a clicked decoded-row string. The row format is
///   `HH:MM:SS  T=<value>  <signed_DF> Hz  <MESSAGE>`
/// where `<value>` may be `" 4.2"` (with a leading space when the
/// numeric part is a single digit) or `"----"` (for accumulator /
/// averaging decodes). The leading space inside the T column means
/// `split_whitespace` produces a variable token count for that
/// section, so we can't use a fixed skip count.
///
/// Strategy: find the `"Hz"` token and start the message immediately
/// after it. That's a stable landmark — `"Hz"` only appears once in
/// the metadata prefix and never in a real decoded message body.
///
/// Returns `(callsign, grid_opt, is_cq)` if the row contains a
/// recognisable callsign, or `None` if it doesn't.
///
/// Examples:
///   "12:55:50  T= 4.2  +0 Hz  CQ I5YDI JN54"     → ("I5YDI", Some("JN54"), true)
///   "12:55:50  T=14.7  -10 Hz  GW4WND F1ABC +05" → ("F1ABC", None, false)
///   "12:55:50  T=----  +0 Hz  GW4WND F1ABC [A]"  → ("F1ABC", None, false)
fn parse_clicked_row(text: &str) -> Option<(String, Option<String>, bool)> {
    let parts: Vec<&str> = text.split_whitespace().collect();
    if parts.is_empty() { return None; }

    // If there's a HH:MM:SS prefix, locate the "Hz" landmark token
    // and start the message just after it. Otherwise, treat the
    // whole row as message text (TX log entries take this path —
    // they don't carry the metadata prefix).
    let msg_start = if parts[0].len() >= 8 && parts[0].as_bytes()[2] == b':' {
        match parts.iter().position(|t| *t == "Hz") {
            Some(idx) => (idx + 1).min(parts.len()),
            None      => return None,
        }
    } else {
        0
    };
    let msg = &parts[msg_start..];
    if msg.is_empty() { return None; }

    if msg[0].eq_ignore_ascii_case("CQ") {
        // CQ messages may carry a "modifier" between CQ and the
        // callsign — typically a directional code (DX/EU/NA/SA/AS/AF/
        // OC/AN), a contest area number ("CQ 368 I1DMP JN34" — RSGB
        // UKAC area code 368 zone), or a contest serial. The protocol
        // permits arbitrary tokens here as long as they aren't
        // callsign-shaped. Strategy: scan after CQ for the FIRST
        // callsign-shaped token; everything before it is modifier(s)
        // and gets discarded. Grid (if any) follows the callsign.
        let after = &msg[1..];
        let idx = first_callsign_index(after)?;
        let call = after[idx].to_uppercase();
        let grid = if after.len() > idx + 1 && is_grid_token(after[idx + 1]) {
            Some(after[idx + 1].to_uppercase())
        } else { None };
        return Some((call, grid, true));
    }
    if msg.len() >= 2 && looks_like_callsign(msg[0]) && looks_like_callsign(msg[1]) {
        return Some((msg[1].to_uppercase(), None, false));
    }
    if looks_like_callsign(msg[0]) {
        return Some((msg[0].to_uppercase(), None, false));
    }
    None
}

/// True if `s` is exactly 4 chars: AA##.
fn is_grid_token(s: &str) -> bool {
    let b = s.as_bytes();
    b.len() == 4
        && b[0].is_ascii_alphabetic()
        && b[1].is_ascii_alphabetic()
        && b[2].is_ascii_digit()
        && b[3].is_ascii_digit()
}

/// Find the first callsign-shaped token in a slice. Used to skip CQ
/// modifiers (DX/EU/NA/SA/AS/AF/OC/AN, numeric area codes / contest
/// serials like "368", or any other non-callsign-shaped token that
/// the standard CQ message format permits between the literal "CQ"
/// and the actual sender callsign).
///
/// Returns the index into `toks` of the first callsign-shaped entry,
/// or None if the slice contains no callsign-shaped tokens (corrupt
/// decode, decode-only modifier with no payload, etc.).
fn first_callsign_index(toks: &[&str]) -> Option<usize> {
    toks.iter().position(|t| looks_like_callsign(t))
}

/// Extract a Maidenhead grid (4 chars: AA##) from a decoded message
/// string. Used to populate PSK Reporter spots' `sender_locator`
/// field.
///
/// Tricky bit: the literal grid pattern (2 letters + 2 digits) ALSO
/// matches some MSK144 protocol keywords — most notably `RR73`. A
/// naïve "first token matching AA##" would happily classify `RR73`
/// as a grid in `SP3TLJ GW4WND RR73` and submit a spot claiming the
/// partner is in grid RR73 (which doesn't exist as a real square,
/// since the second letter is bounded A..R in the Maidenhead system,
/// but several real squares like RA00..RR99 do exist — the algorithm
/// can't always tell them apart on shape alone).
///
/// Strategy:
///  1. Tokenise.
///  2. Reject the message outright if its FINAL token is a known
///     protocol keyword that means "this isn't a grid": RR73, RRR,
///     73, or anything matching +/-NN or R+/R-NN.
///  3. Otherwise, the grid (if present) is the LAST token. Validate
///     it as Maidenhead-shaped AND sanity-check the second letter
///     is in the valid Maidenhead range (A..R, since longitude
///     fields go 18× 20° = 360°).
///  4. Return None if no valid grid token found.
///
/// This still over-accepts a few rare collisions (`AB12` could
/// theoretically be a real grid, and we'd accept it) but it
/// correctly rejects RR73 and the report tokens which are the
/// common false positives in real traffic.
fn extract_grid_from_message(msg: &str) -> Option<String> {
    let toks: Vec<&str> = msg.split_whitespace().collect();
    if toks.is_empty() { return None; }

    // Final-token rejection: if the message ends in a known
    // non-grid protocol keyword, there's no grid.
    let last = toks[toks.len() - 1];
    if is_non_grid_keyword(last) { return None; }

    // The grid, if present, sits in the final position. Validate
    // shape AND Maidenhead longitude range (A..R).
    if is_maidenhead_grid(last) {
        return Some(last.to_uppercase());
    }
    None
}

/// True if the token is a protocol keyword that occupies the
/// "grid slot" but isn't a grid: RR73, RRR, 73, +rpt, -rpt,
/// R+rpt, R-rpt. Case-insensitive.
fn is_non_grid_keyword(s: &str) -> bool {
    let up = s.to_ascii_uppercase();
    if matches!(up.as_str(), "RR73" | "RRR" | "73") {
        return true;
    }
    // Reports: +NN, -NN, R+NN, R-NN where NN is 1-2 digits.
    let body = up.trim_start_matches('R');
    if (body.starts_with('+') || body.starts_with('-'))
        && body[1..].len() <= 2
        && body[1..].chars().all(|c| c.is_ascii_digit())
        && !body[1..].is_empty()
    {
        return true;
    }
    false
}

/// True if `s` is a Maidenhead grid square: AA## with the FIRST
/// letter in A..R (longitude field, 18 zones of 20° = 360°) and the
/// SECOND letter also in A..R (latitude field, 18 zones of 10° but
/// only the southern hemisphere uses A..H, and northern uses I..R;
/// both are inside A..R). Excludes oddities like "RR" which fall
/// outside the valid latitude band but still match the lazy
/// is_grid_token shape check.
fn is_maidenhead_grid(s: &str) -> bool {
    let b = s.as_bytes();
    if b.len() != 4 { return false; }
    let a0 = b[0].to_ascii_uppercase();
    let a1 = b[1].to_ascii_uppercase();
    a0.is_ascii_alphabetic()
        && a1.is_ascii_alphabetic()
        && b[2].is_ascii_digit()
        && b[3].is_ascii_digit()
        && (b'A'..=b'R').contains(&a0)
        && (b'A'..=b'R').contains(&a1)
}

/// Parse a "HHMMSS" slot-end timestamp (as produced by
/// `decoder::slot_end_timestamp`) back into a unix-seconds value
/// for today's UTC date. Used by PSK Reporter spot timestamps so
/// the spot's plot timestamp matches the audio's on-air moment
/// rather than the moment our LDPC chain happened to finish.
///
/// Returns None if the input isn't 6 digits or if today's date
/// can't be resolved (only possible on a fundamentally broken
/// system). Wraps 0:00:00 cleanly — UTC midnight.
fn parse_slot_utc_to_unix(hhmmss: &str) -> Option<u32> {
    if hhmmss.len() != 6 { return None; }
    let h: i64 = hhmmss[0..2].parse().ok()?;
    let m: i64 = hhmmss[2..4].parse().ok()?;
    let s: i64 = hhmmss[4..6].parse().ok()?;
    let now = chrono::Utc::now();
    let today_midnight_secs = now.date_naive().and_hms_opt(0, 0, 0)?
        .and_utc().timestamp();
    let secs = today_midnight_secs + h * 3600 + m * 60 + s;
    if secs >= 0 && secs <= u32::MAX as i64 {
        Some(secs as u32)
    } else {
        None
    }
}

/// Pull the "interesting" callsign out of a decoded row.
///
/// The row format is `HH:MM:SS  <DF>Hz  S<n>  <method>  <MESSAGE>`. We skip
/// the 4 metadata tokens and parse the message portion as MSK144 protocol:
///
///   "CQ GW4WND IO82"          → GW4WND       (CQ from this station)
///   "GW4WND F1ABC +05"        → F1ABC        (someone calling me — return their call)
///   "F1ABC GW4WND R+05"       → GW4WND       (we're addressed; ignore — uses parts[0])
///
/// Returns None if no callsign-shaped token is found.
fn extract_callsign(text: &str) -> Option<String> {
    let parts: Vec<&str> = text.split_whitespace().collect();
    if parts.is_empty() { return None; }

    // If a timestamp prefix is present, skip past the metadata by
    // locating the "Hz" landmark token and starting after it. This is
    // robust to the T column rendering as either `"T= 4.2"` (which
    // splits into TWO whitespace tokens) or `"T=----"` (one token).
    // Without the metadata prefix, treat the whole input as message.
    let msg_start = if parts[0].len() >= 8 && parts[0].as_bytes()[2] == b':' {
        match parts.iter().position(|t| *t == "Hz") {
            Some(idx) => (idx + 1).min(parts.len()),
            None      => return None,
        }
    } else {
        0
    };
    let msg = &parts[msg_start..];
    if msg.is_empty() { return None; }

    // CQ <modifier?> <call> <grid?>  → return <call>
    // The modifier slot can be empty, a directional code (DX/EU/NA/etc),
    // a contest area code/serial ("368"), or any other non-callsign-
    // shaped token. We scan forward to the first callsign-shaped
    // token; that's the actual sender.
    if msg[0].eq_ignore_ascii_case("CQ") && msg.len() >= 2 {
        let after = &msg[1..];
        if let Some(idx) = first_callsign_index(after) {
            return Some(after[idx].to_uppercase());
        }
        return None;
    }
    // <to> <from> ...    → return <from>  (the calling station's callsign)
    if msg.len() >= 2 && looks_like_callsign(msg[0]) && looks_like_callsign(msg[1]) {
        return Some(msg[1].to_uppercase());
    }
    // SH-format messages — the sender's callsign is encoded as a hash
    // placeholder (e.g. "<...>") rather than transmitted in the clear.
    // Examples:
    //   "EU1DE/2 <...> RRR"   — someone confirmed an exchange to EU1DE/2
    //   "<...> G4WND -01"     — someone reported -01 to me
    // In all such cases we do NOT know who the sender is, only who they
    // were talking TO (the recipient). Returning the recipient would be
    // wrong: PSK Reporter would log them as "heard" when in fact we
    // only inferred their existence from another station's traffic.
    // Return None so the spotter skips this decode entirely.
    if msg.iter().any(|t| t.starts_with('<') && t.ends_with('>')) {
        return None;
    }
    // <call>             → return <call>
    // Only reachable when the message is a single bare callsign with
    // no recipient and no SH hash. Rare in practice but kept as a
    // defensive last resort.
    if looks_like_callsign(msg[0]) {
        return Some(msg[0].to_uppercase());
    }
    None
}

/// Heuristic: at least one letter and one digit, no spaces, ≤10 chars.
fn looks_like_callsign(s: &str) -> bool {
    if s.is_empty() || s.len() > 10 { return false; }
    let has_letter = s.chars().any(|c| c.is_ascii_alphabetic());
    let has_digit  = s.chars().any(|c| c.is_ascii_digit());
    let valid = s.chars().all(|c| c.is_ascii_alphanumeric() || c == '/');
    has_letter && has_digit && valid
}

/// Quantise an SNR-in-dB to MSK144's 2-dB protocol grid and clamp to the
/// transmittable range [-4, +24].
fn quantise_snr_db(snr_db: f32) -> i16 {
    let quant = 2.0 * (snr_db / 2.0).round();
    let clamped = quant.clamp(-4.0, 24.0);
    clamped as i16
}

/// Colour palette keyed by slot parity. Two distinct hues so the
/// operator can see at a glance which slot half each decode came
/// from. Cool/warm split is the convention used in FSK441+ and
/// MSHV; mirrored here so users moving between apps don't have to
/// re-learn. The selected TX-parity in the top toolbar uses these
/// same hues so the visual link is unambiguous.
///
/// (parity 0 → cool blue, parity 1 → warm orange.)
fn parity_accent(parity: u8) -> egui::Color32 {
    match parity & 1 {
        0 => egui::Color32::from_rgb(80, 160, 220),    // cool blue
        _ => egui::Color32::from_rgb(220, 140, 60),    // warm orange
    }
}

/// Same as `parity_accent` but with low alpha — for row backgrounds
/// where we want a hint of colour without overpowering the text.
/// Currently unused (Roger preferred edge-bar-only colouring with
/// no background tint), but kept around in case the row-tint look
/// is wanted again as an option.
#[allow(dead_code)]
fn parity_tint(parity: u8) -> egui::Color32 {
    match parity & 1 {
        0 => egui::Color32::from_rgba_unmultiplied(80, 160, 220, 12),
        _ => egui::Color32::from_rgba_unmultiplied(220, 140, 60, 12),
    }
}

/// Convert the user-facing TX parity string ("Odd" / "Even") into the
/// 0/1 parity convention used everywhere else (matches the
/// transmitter's `slot_idx % 2` test: "Even" = 0, "Odd" = 1).
fn tx_parity_to_int(s: &str) -> u8 {
    match s {
        "Even" => 0,
        _      => 1,  // "Odd" or unknown defaults to Odd convention
    }
}

/// Map the canonical internal parity string ("Even"/"Odd") to the
/// display label shown in the UI dropdown. Even = "1st" (period
/// starts at the even minute = first period in the pair), Odd = "2nd".
/// Internal strings stay canonical so settings persistence and all
/// matching logic remain unchanged.
fn parity_display_label(s: &str) -> &'static str {
    match s {
        "Even" => "1st",
        _      => "2nd",
    }
}

/// Given a slot-START timestamp string in "HHMMSS" form, return the
/// parity of the slot (0 or 1). Computed via the same convention the
/// transmitter uses: slot_idx = unix_secs_at_slot_start / period_secs;
/// parity = slot_idx % 2.
///
/// Slot HHMMSS labels in this app are slot-START (matching WSJT-X /
/// MSHV / MSK2K convention). For a slot starting at HH:MM:SS, the
/// slot index is simply unix_secs_at_HHMMSS / period_secs — no -1
/// fudge factor (which the old end-time labelling required).
///
/// `period_secs` must match what the decoder framer was using when
/// the slot was timestamped — the caller provides the live setting.
///
/// Returns None if the input isn't a valid HHMMSS, period_secs is
/// zero, or we can't resolve a unix epoch.
fn slot_parity_from_hhmmss(hhmmss: &str, period_secs: u32) -> Option<u8> {
    if hhmmss.len() != 6 || period_secs == 0 { return None; }
    let h: i64 = hhmmss[0..2].parse().ok()?;
    let m: i64 = hhmmss[2..4].parse().ok()?;
    let s: i64 = hhmmss[4..6].parse().ok()?;
    // Compute today's date as a unix-secs anchor, then add the
    // intra-day offset. Parity is invariant under day-shifts for
    // both 15s and 30s periods (86400 / 15 = 5760, 86400 / 30 = 2880,
    // both even), so day-of-week doesn't affect parity.
    let now = chrono::Utc::now();
    let today_midnight_secs = now.date_naive().and_hms_opt(0, 0, 0)?
        .and_utc().timestamp();
    let secs = today_midnight_secs + h * 3600 + m * 60 + s;
    // Slot STARTING at this timestamp has index secs/period_secs.
    let slot_idx = secs / period_secs as i64;
    Some((slot_idx.rem_euclid(2)) as u8)
}

#[cfg(test)]
mod report_tests {
    use super::*;

    #[test]
    fn snr_quantised_to_2db_steps() {
        // 5.5 dB raw → 6 (nearest even integer)
        assert_eq!(quantise_snr_db(5.5), 6);
        // 5.4 dB raw → 4 (rounds down)... wait, round() rounds-half-to-even
        // in Rust: 5.4/2=2.7, round()=3, *2=6. Correct.
        assert_eq!(quantise_snr_db(5.4), 6);
        // 4.5 dB → 4.5/2=2.25, round=2, *2=4
        assert_eq!(quantise_snr_db(4.5), 4);
    }

    #[test]
    fn snr_clamped_low() {
        // Anything below -4 dB clamps to -4
        assert_eq!(quantise_snr_db(-10.0), -4);
        assert_eq!(quantise_snr_db(-4.5), -4);
    }

    #[test]
    fn snr_clamped_high() {
        // Anything above +24 dB clamps to +24
        assert_eq!(quantise_snr_db(30.0), 24);
        assert_eq!(quantise_snr_db(25.0), 24);
    }

    #[test]
    fn snr_typical_meteor_ping() {
        // Real meteor pings give roughly 0–20 dB SNR depending on burst strength
        assert_eq!(quantise_snr_db(10.0), 10);
        assert_eq!(quantise_snr_db(0.0),  0);
        assert_eq!(quantise_snr_db(-2.0), -2);
    }

    #[test]
    fn slot_parity_15s_alternates_each_slot() {
        // Slot STARTING at 12:00:00 has unix-secs = today_midnight + 43200.
        // idx = 43200 / 15 = 2880. Parity 0.
        // Next slot starts at 12:00:15 → idx 2881, parity 1. Etc.
        let p1 = slot_parity_from_hhmmss("120000", 15).unwrap();
        let p2 = slot_parity_from_hhmmss("120015", 15).unwrap();
        let p3 = slot_parity_from_hhmmss("120030", 15).unwrap();
        let p4 = slot_parity_from_hhmmss("120045", 15).unwrap();
        assert_eq!(p1, 0);
        assert_eq!(p2, 1);
        assert_eq!(p3, 0);
        assert_eq!(p4, 1);
    }

    #[test]
    fn slot_parity_30s_alternates_each_slot() {
        // Slot STARTING at 12:00:00 → idx 43200/30 = 1440, parity 0.
        // Next slot starts at 12:00:30 → idx 1441, parity 1. Etc.
        let p1 = slot_parity_from_hhmmss("120000", 30).unwrap();
        let p2 = slot_parity_from_hhmmss("120030", 30).unwrap();
        let p3 = slot_parity_from_hhmmss("120100", 30).unwrap();
        let p4 = slot_parity_from_hhmmss("120130", 30).unwrap();
        assert_eq!(p1, 0);
        assert_eq!(p2, 1);
        assert_eq!(p3, 0);
        assert_eq!(p4, 1);
    }

    #[test]
    fn slot_parity_zero_period_returns_none() {
        // Defensive: division by zero would panic; we return None.
        assert!(slot_parity_from_hhmmss("120015", 0).is_none());
    }
}
