// crates/msk144plus_engine/src/lib.rs
//
// MSK144 decode orchestrator. Faithful port of WSJT-X mskrtd.f90 + the
// back-end portion of msk144spd.f90.

use msk144plus_dsp::{
    analytic, demodulate_frame, demodulate_short_frame, detect_candidates,
    detect_short_candidates, msk144_sync, msk40_sync, short_softbits_to_llr,
    softbits_to_ldpc_llr, AnalyticFilter, NSPM, NSPM_MSK40, NZ_MSKRTD,
    NFFT_ANALYTIC,
};
use msk144plus_fec::{decode_128_90_soft, decode_short, encode_short};
use msk144plus_packjt::{
    jenkins::{format_call_pair, hash12},
    unpack77, Message,
};
use num_complex::Complex32;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Depth {
    Fast = 1,
    Normal = 2,
    Deep = 3,
}

impl Depth {
    pub fn n_patterns(&self) -> usize {
        match self {
            Depth::Fast => 0,
            Depth::Normal => 2,
            Depth::Deep => 4,
        }
    }
}

/// Configuration for the MSK40 short-message decoder. Mirrors WSJT-X's
/// "Sh msg" mode (`bshmsg`).
///
/// MSK40 short messages contain only a 12-bit hash of "<MYCALL HISCALL>"
/// and a 4-bit report code. The receiver MUST know both callsigns in
/// advance to compute the hash and recognise it; otherwise the receiver
/// can't reconstruct the message text.
#[derive(Debug, Clone, Default)]
pub struct ShortMessageConfig {
    /// Operator's own callsign (used as `mycall` in the hash).
    pub mycall: String,
    /// Other station's callsign (used as `hiscall` in the hash).
    pub hiscall: String,
    /// Enable MSK40 decoding. If false, the MSK40 path is not run.
    pub enabled: bool,
}

/// 16-entry MSK40 report table. Source: msk40decodeframe.f90 lines 27-29.
const RPT_TABLE: [&str; 16] = [
    "-03", "+00", "+03", "+06", "+10", "+13", "+16",
    "R-03", "R+00", "R+03", "R+06", "R+10", "R+13", "R+16",
    "RRR", "73",
];

#[derive(Debug, Clone)]
pub struct DecodeEvent {
    pub text: String,
    pub message: Option<Message>,
    pub xmax: f32,
    pub freq_offset: f32,
    pub method: String,
    pub n_hard_errors: u32,
    /// Signal-to-noise ratio in dB, computed by the SPD detector via
    /// WSJT-X's canonical formula:  snr = 12*log10(detmet)/2 - 9.
    /// This is the value to send back to the partner as the +NN report
    /// (after quantising to MSK144's 2-dB grid and clamping to [-4,+24]).
    pub snr_db: f32,
    /// True when this decode came from the soft-bit accumulator path
    /// (multiple sub-threshold fragments combined). False for the
    /// standard SPD/avg-N decode pipeline. The UI appends ` [A]` to
    /// row text when this is true so accumulated decodes are visually
    /// distinguishable from standard ones during A/B comparison.
    pub is_accumulated: bool,
    /// Estimated time within the slot (in seconds, 0..15) at which the
    /// burst was found. Surfaced in the UI's RX log as a `T=N.N` column
    /// so the operator can see when in the slot a burst arrived
    /// (matches MSHV's "T" column).
    ///
    /// Populated by:
    ///   - SPD path: from the candidate's `n_start` sample index plus
    ///     the analysis-window's offset within the slot.
    ///   - avg-N path: weighted mean midpoint of the contributing
    ///     frames (known precisely from the avg pattern's navmask),
    ///     plus the analysis-window offset.
    ///   - Accumulator path: weighted mean of contributing fragments'
    ///     `n_start` values (xmax² weights), already in slot-relative
    ///     samples by construction.
    /// All three express the same thing: when in the slot the burst
    /// was, with precision degrading from spd-pat (≈ tens of ms) to
    /// avg-N (≈ ½ frame, 36 ms) to accumulator (depends on fragment
    /// spread; reported as a single weighted centroid).
    pub slot_position_secs: Option<f32>,
}

#[derive(Debug, Clone)]
pub enum DecodeFailure {
    BufferTooShort,
    LowSignal,
    NoDecode,
}

/// Main entry point. Faithful port of WSJT-X mskrtd.f90.
///
/// Order of operations (matching mskrtd.f90 lines 127-171):
///   1. msk144spd front-end → per-candidate decode loop
///   2. If no MSK144 decode AND short-msg enabled: msk40spd
///   3. If still no decode: MSK144 averaging-pattern loop
/// Diagnostic information about why a slot failed to decode. Built up as
/// the pipeline runs; the decoder thread can log it after each slot to
/// give visibility into "we heard a ping but it didn't decode" cases.
#[derive(Default, Debug, Clone)]
pub struct DecodeDiag {
    /// Number of MSK144 spd candidates found
    pub n_msk144_cands: usize,
    /// Number of MSK40 spd candidates found
    pub n_msk40_cands: usize,
    /// Best (highest) xmax seen across all sync attempts
    pub best_xmax: f32,
    /// Best n_bad_sync seen (lower = better) when xmax >= 1.3
    pub best_n_bad_sync: Option<u8>,
    /// Best n_hard_errors seen when sync passed
    pub best_n_hard_errors: Option<u32>,
    /// Detmet of the strongest candidate (post-normalisation)
    pub strongest_detmet: f32,
}

pub fn mskrtd_decode(
    id2: &[f32],
    ntol_hz: f32,
    fc_hz: f32,
    depth: Depth,
    short_cfg: Option<&ShortMessageConfig>,
) -> Result<DecodeEvent, DecodeFailure> {
    mskrtd_decode_with_diag(id2, ntol_hz, fc_hz, depth, short_cfg).0
}

/// Same as mskrtd_decode but also returns DecodeDiag with per-stage stats
/// useful for "why didn't this decode" debugging.
pub fn mskrtd_decode_with_diag(
    id2: &[f32],
    ntol_hz: f32,
    fc_hz: f32,
    depth: Depth,
    short_cfg: Option<&ShortMessageConfig>,
) -> (Result<DecodeEvent, DecodeFailure>, DecodeDiag) {
    let mut diag = DecodeDiag::default();
    if id2.len() < NZ_MSKRTD {
        return (Err(DecodeFailure::BufferTooShort), diag);
    }
    let nz = NZ_MSKRTD;
    let sum_sq: f32 = id2[..nz].iter().map(|x| x * x).sum();
    let rms = (sum_sq / nz as f32).sqrt();
    if rms < 1.0 {
        return (Err(DecodeFailure::LowSignal), diag);
    }
    let fac = 1.0 / rms;
    let mut d = vec![0.0f32; NFFT_ANALYTIC];
    for i in 0..nz {
        d[i] = fac * id2[i];
    }
    let filter = AnalyticFilter::new(NFFT_ANALYTIC);
    let cdat = analytic(&d, &filter);
    let np = 8 * NSPM;
    if cdat.len() < np {
        return (Err(DecodeFailure::BufferTooShort), diag);
    }
    let cbig = &cdat[..np];

    // Step 1: MSK144 short-ping decoder
    if let Some(evt) = run_spd_decode(cbig, ntol_hz, fc_hz, &mut diag) {
        return (Ok(evt), diag);
    }

    // Step 2: MSK40 short-message decoder (if config has callsigns)
    if let Some(cfg) = short_cfg {
        if cfg.enabled && !cfg.mycall.is_empty() && !cfg.hiscall.is_empty() {
            if let Some(evt) = run_msk40_decode(cbig, ntol_hz, fc_hz, cfg, &mut diag) {
                return (Ok(evt), diag);
            }
        }
    }

    // Step 3: MSK144 averaging-pattern loop
    if let Some(evt) = run_averaging_patterns(cbig, ntol_hz, fc_hz, depth, &mut diag) {
        return (Ok(evt), diag);
    }
    (Err(DecodeFailure::NoDecode), diag)
}

fn run_spd_decode(cbig: &[Complex32], ntol_hz: f32, fc_hz: f32, diag: &mut DecodeDiag) -> Option<DecodeEvent> {
    let navpatterns: [[u8; 3]; 6] = [
        [0, 1, 0], [1, 0, 0], [0, 0, 1],
        [1, 1, 0], [0, 1, 1], [1, 1, 1],
    ];
    let candidates = detect_candidates(cbig, ntol_hz, fc_hz);
    diag.n_msk144_cands = candidates.len();
    if let Some(c) = candidates.first() {
        diag.strongest_detmet = c.detmet.max(diag.strongest_detmet);
    }
    for cand in &candidates {
        let n = cbig.len();
        let n_start_0 = cand.n_start.saturating_sub(1);
        let mut ib = n_start_0.saturating_sub(NSPM);
        let mut ie = ib + 3 * NSPM;
        if ie > n {
            ie = n;
            if ie >= 3 * NSPM { ib = ie - 3 * NSPM; } else { continue; }
        }
        let window = &cbig[ib..ie];
        let fo = fc_hz + cand.freq_err;
        let ntol0 = 8.0;
        let deltaf = 2.0;
        for (pat_idx, navmask) in navpatterns.iter().enumerate() {
            let result = msk144_sync(window, 3, ntol0, deltaf, navmask, 2, fo);
            if result.xmax > diag.best_xmax { diag.best_xmax = result.xmax; }
            if !result.success { continue; }
            for &peak_loc in &result.peak_locations {
                for is in 0..3 {
                    let dither = match is { 0 => 0i32, 1 => -1, _ => 1 };
                    let ic0 = ((peak_loc as i32 + dither).max(0).min(NSPM as i32 - 1)) as usize;
                    let mut ct = [Complex32::new(0.0, 0.0); NSPM];
                    for k in 0..NSPM { ct[k] = result.averaged_frame[(k + ic0) % NSPM]; }
                    let demod = demodulate_frame(&ct);
                    if diag.best_n_bad_sync.map_or(true, |b| demod.n_bad_sync < b) {
                        diag.best_n_bad_sync = Some(demod.n_bad_sync);
                    }
                    if demod.n_bad_sync > 4 { continue; }
                    let llr = softbits_to_ldpc_llr(&demod.softbits);
                    if let Some(r) = decode_128_90_soft(&llr, 10) {
                        if diag.best_n_hard_errors.map_or(true, |e| r.n_hard_errors < e) {
                            diag.best_n_hard_errors = Some(r.n_hard_errors);
                        }
                        if r.n_hard_errors < 18 {
                            let msg = unpack77(&r.message);
                            if !is_valid_message(&msg) { continue; }
                            // In-window position of this burst, seconds.
                            // The slot-level loop in decode_slot_with_diag
                            // adds the window's offset within the slot
                            // to give the final slot-relative T value.
                            let pos_in_window = cand.n_start as f32 / 12000.0;
                            return Some(DecodeEvent {
                                text: msg.to_text(),
                                message: Some(msg),
                                xmax: result.xmax,
                                freq_offset: result.freq_offset + cand.freq_err,
                                method: format!("spd-pat{}", pat_idx),
                                n_hard_errors: r.n_hard_errors,
                            snr_db: cand.snr_db,
                            is_accumulated: false,
                            slot_position_secs: Some(pos_in_window),
                            });
                        }
                    }
                }
            }
        }
    }
    None
}

/// MSK40 short-message decoder. Faithful port of msk40spd.f90 back-end.
fn run_msk40_decode(
    cbig: &[Complex32],
    ntol_hz: f32,
    fc_hz: f32,
    cfg: &ShortMessageConfig,
    diag: &mut DecodeDiag,
) -> Option<DecodeEvent> {
    // Compute the expected hashes for both orderings of the call pair.
    // The MSK40 transmitter always puts ITS OWN call first in the hash:
    //   When we transmit:    hash of "<MYCALL HISCALL>"
    //   When partner transmits: hash of "<HISCALL MYCALL>"
    // To decode partner's traffic during a QSO, we must match against
    // hash_partner. To recognise our own TX echo (rare but possible
    // through monitor leakage), we also accept hash_self. MSHV does
    // exactly this — see SetCalsHash() in decodermsk40.cpp which
    // populates both `prav` (forward) and `obraten` (reversed)
    // entries in the active-QSO hash table.
    let pair_self    = format_call_pair(&cfg.mycall, &cfg.hiscall);
    let pair_partner = format_call_pair(&cfg.hiscall, &cfg.mycall);
    let hash_self    = hash12(&pair_self);
    let hash_partner = hash12(&pair_partner);

    let navpatterns: [[u8; 3]; 6] = [
        [0, 1, 0], [1, 0, 0], [0, 0, 1],
        [1, 1, 0], [0, 1, 1], [1, 1, 1],
    ];

    let candidates = detect_short_candidates(cbig, ntol_hz, fc_hz);
    diag.n_msk40_cands = candidates.len();
    if let Some(c) = candidates.first() {
        diag.strongest_detmet = c.detmet.max(diag.strongest_detmet);
    }
    for cand in &candidates {
        let n = cbig.len();
        let n_start_0 = cand.n_start.saturating_sub(1);
        let mut ib = n_start_0.saturating_sub(NSPM_MSK40);
        let mut ie = ib + 3 * NSPM_MSK40;
        if ie > n {
            ie = n;
            if ie >= 3 * NSPM_MSK40 { ib = ie - 3 * NSPM_MSK40; } else { continue; }
        }
        let window = &cbig[ib..ie];
        let fo = fc_hz + cand.freq_err;
        let xsnr = cand.snr_db;
        // msk40spd.f90: ntol0 = 29, deltaf = 7.2
        let ntol0 = 29.0;
        let deltaf = 7.2;

        for (pat_idx, navmask) in navpatterns.iter().enumerate() {
            let result = msk40_sync(window, 3, ntol0, deltaf, navmask, 2, fo);
            if !result.success { continue; }

            for &peak_loc in &result.peak_locations {
                for is in 0..3 {
                    let dither = match is { 0 => 0i32, 1 => -1, _ => 1 };
                    let ic0 = ((peak_loc as i32 + dither).max(0).min(NSPM_MSK40 as i32 - 1)) as usize;
                    let mut ct = [Complex32::new(0.0, 0.0); NSPM_MSK40];
                    for k in 0..NSPM_MSK40 {
                        ct[k] = result.averaged_frame[(k + ic0) % NSPM_MSK40];
                    }
                    let demod = demodulate_short_frame(&ct);
                    if demod.n_bad_sync > 3 { continue; }
                    let llr = short_softbits_to_llr(&demod.softbits, xsnr);
                    if let Some(r) = decode_short(&llr, 5) {
                        // Compute Hamming distance against re-encoded codeword,
                        // and the soft-correlation residual.
                        let cw = encode_short(&r.message);
                        let mut nhammd = 0;
                        let mut cord = 0.0f32;
                        for k in 0..32 {
                            let hb: u8 = if demod.softbits[8 + k] >= 0.0 { 1 } else { 0 };
                            if cw[k] != hb {
                                nhammd += 1;
                                cord += demod.softbits[8 + k].abs();
                            }
                        }
                        // Decode message bits to get hash and report.
                        // genmsk40.f90 line 40: message(i) = iand(1, ishft(ig, 1-i))
                        //   means message(1)=LSB(ig), message(16)=MSB(ig).
                        // In our 0-indexed array: r.message[0]=LSB, r.message[15]=MSB.
                        // We want imsg = ig, so:
                        //   imsg = (r.message[15] << 15) | (r.message[14] << 14) | ... | r.message[0]
                        let mut imsg = 0u32;
                        for k in 0..16 {
                            imsg |= (r.message[k] as u32) << k;
                        }
                        let nrxrpt = (imsg & 0xF) as usize;
                        let nrxhash = ((imsg >> 4) & 0xFFF) as u16;

                        // msk40decodeframe.f90 line 135-136:
                        //   if nhammd <= 4 .and. cord < 0.65 .and.
                        //      nrxhash == ihash .and. nrxrpt >= 7
                        //
                        // We only accept the PARTNER direction
                        // (hash_partner = "<HISCALL MYCALL>"). The
                        // self direction (our own TX echoing back via
                        // monitor, ionospheric reflection, aircraft
                        // scatter, or any other path) gets rejected
                        // even though the math would decode it. Real-
                        // world testing showed self-decodes appearing
                        // in nearly every RX slot during a QSO with
                        // Sh msg active — annoying noise in the log
                        // and confusing for the QSO state machine if
                        // we ever tried to act on it. MSHV stores
                        // both directions in its hash table for SWL
                        // (overhearing other QSOs), but in active-QSO
                        // mode the only legitimate Sh msg traffic is
                        // from the partner. If genuine self-echo
                        // detection is ever needed (e.g. for a
                        // monitor-mode tuning aid), wire it through
                        // a separate config flag.
                        if nhammd <= 4
                            && cord < 0.65
                            && nrxhash == hash_partner
                            && nrxrpt >= 7
                        {
                            // Partner is the sender, so render with
                            // their call first inside the brackets —
                            // matches MSHV convention and means the
                            // RX column shows what was on-air.
                            let text = format!(
                                "<{} {}> {}",
                                cfg.hiscall.trim(),
                                cfg.mycall.trim(),
                                RPT_TABLE[nrxrpt]
                            );
                            // hash_self is no longer used as a
                            // matching condition but we keep it in
                            // scope so the let binding above doesn't
                            // become dead code; could be wired to
                            // diagnostics if useful.
                            let _ = hash_self;
                            let pos_in_window = cand.n_start as f32 / 12000.0;
                            return Some(DecodeEvent {
                                text,
                                message: None,
                                xmax: result.xmax,
                                freq_offset: result.freq_offset + cand.freq_err,
                                method: format!("msk40-pat{}", pat_idx),
                                n_hard_errors: nhammd as u32,
                            snr_db: cand.snr_db,
                            is_accumulated: false,
                            slot_position_secs: Some(pos_in_window),
                            });
                        }
                    }
                }
            }
        }
    }
    None
}

fn run_averaging_patterns(
    cbig: &[Complex32], ntol_hz: f32, fc_hz: f32, depth: Depth, diag: &mut DecodeDiag,
) -> Option<DecodeEvent> {
    let avg_patterns: [[u8; 8]; 4] = [
        [1, 1, 1, 1, 0, 0, 0, 0],
        [0, 0, 1, 1, 1, 1, 0, 0],
        [1, 1, 1, 1, 1, 0, 0, 0],
        [1, 1, 1, 1, 1, 1, 1, 0],
    ];
    let npat = depth.n_patterns();
    if npat == 0 || cbig.len() < 8 * NSPM { return None; }
    let cdat_8 = &cbig[..8 * NSPM];
    for iavg in 0..npat {
        let navmask = &avg_patterns[iavg];
        let navg: usize = navmask.iter().map(|&x| x as usize).sum();
        if navg == 0 { continue; }
        let deltaf = 10.0 / navg as f32;
        let result = msk144_sync(cdat_8, 8, ntol_hz, deltaf, navmask, 2, fc_hz);
        if result.xmax > diag.best_xmax { diag.best_xmax = result.xmax; }
        if !result.success { continue; }
        for &peak_loc in &result.peak_locations {
            for is in 0..3 {
                let dither = match is { 0 => 0i32, 1 => -1, _ => 1 };
                let ic0 = ((peak_loc as i32 + dither).max(0).min(NSPM as i32 - 1)) as usize;
                let mut ct = [Complex32::new(0.0, 0.0); NSPM];
                for k in 0..NSPM { ct[k] = result.averaged_frame[(k + ic0) % NSPM]; }
                let demod = demodulate_frame(&ct);
                if diag.best_n_bad_sync.map_or(true, |b| demod.n_bad_sync < b) {
                    diag.best_n_bad_sync = Some(demod.n_bad_sync);
                }
                if demod.n_bad_sync > 4 { continue; }
                let llr = softbits_to_ldpc_llr(&demod.softbits);
                if let Some(r) = decode_128_90_soft(&llr, 10) {
                    if diag.best_n_hard_errors.map_or(true, |e| r.n_hard_errors < e) {
                        diag.best_n_hard_errors = Some(r.n_hard_errors);
                    }
                    if r.n_hard_errors < 18 {
                        let msg = unpack77(&r.message);
                        if !is_valid_message(&msg) { continue; }
                        // No SpdCandidate available on the avg path — derive
                        // SNR from result.xmax. xmax is post-normalised by
                        // fac = 1/(48*sqrt(navg)); recover the unscaled
                        // detmet-equivalent and apply the canonical formula.
                        let raw_metric = result.xmax * 48.0 * (navg as f32).sqrt();
                        let snr_db = 12.0 * raw_metric.max(1e-3).log10() / 2.0 - 9.0;
                        // Burst position from the avg pattern's navmask.
                        // avg-N averages a SUBSET of the 8 frames in this
                        // analysis window. The contributing frames are
                        // exactly those with navmask[k] = 1; mean of
                        // their midpoints is the most defensible single-
                        // point estimate. Window-relative; decode_slot
                        // adds the window offset.
                        let frame_secs = NSPM as f32 / 12000.0;
                        let mut sum_pos = 0.0f32;
                        let mut n_contrib: u32 = 0;
                        for (k, &m) in navmask.iter().enumerate() {
                            if m == 1 {
                                sum_pos += (k as f32 + 0.5) * frame_secs;
                                n_contrib += 1;
                            }
                        }
                        let burst_pos_secs = if n_contrib > 0 {
                            sum_pos / n_contrib as f32
                        } else {
                            0.0  // unreachable: navg == 0 filtered above
                        };
                        return Some(DecodeEvent {
                            text: msg.to_text(),
                            message: Some(msg),
                            xmax: result.xmax,
                            freq_offset: result.freq_offset,
                            method: format!("avg-{}", navg),
                            n_hard_errors: r.n_hard_errors,
                            snr_db,
                        is_accumulated: false,
                        slot_position_secs: Some(burst_pos_secs),
                        });
                    }
                }
            }
        }
    }
    None
}

fn is_valid_message(msg: &Message) -> bool {
    matches!(
        msg,
        Message::Standard(_) | Message::FreeText { .. } | Message::Nonstandard(_)
    )
}

/// Process an entire slot. Slides the NZ_MSKRTD window through the audio
/// at half-block increments and collects unique decodes.
pub fn decode_slot(
    audio: &[f32],
    ntol_hz: f32,
    fc_hz: f32,
    depth: Depth,
    short_cfg: Option<&ShortMessageConfig>,
) -> Vec<DecodeEvent> {
    decode_slot_with_diag(audio, ntol_hz, fc_hz, depth, short_cfg).0
}

/// Same as decode_slot but also returns the most "interesting" DecodeDiag
/// from the windows that didn't decode. The returned diag captures the
/// strongest sync and best LDPC partial-success seen across all windows
/// in the slot — useful for understanding why audible pings didn't decode.
pub fn decode_slot_with_diag(
    audio: &[f32],
    ntol_hz: f32,
    fc_hz: f32,
    depth: Depth,
    short_cfg: Option<&ShortMessageConfig>,
) -> (Vec<DecodeEvent>, DecodeDiag) {
    let nz = NZ_MSKRTD;
    let mut events = Vec::new();
    let mut seen_texts = std::collections::HashSet::new();
    let mut best_diag = DecodeDiag::default();
    if audio.len() < nz {
        return (events, best_diag);
    }
    let step = nz / 2;
    let mut k_end = nz;
    while k_end <= audio.len() {
        let window = &audio[k_end - nz..k_end];
        // The window starts this many seconds into the slot. Used to
        // promote a window-relative slot_position_secs in the returned
        // DecodeEvent to slot-relative (= the user-facing T value).
        let window_start_secs = (k_end - nz) as f32 / 12000.0;
        let (res, diag) = mskrtd_decode_with_diag(window, ntol_hz, fc_hz, depth, short_cfg);
        if let Ok(mut evt) = res {
            // Run-time paths (SPD / MSK40) emit slot_position_secs
            // relative to the start of their analysis window. The
            // slot-level loop knows the window's offset within the
            // slot so it adds them here. Averaging and accumulator
            // paths emit None and stay None.
            if let Some(pos) = evt.slot_position_secs {
                evt.slot_position_secs = Some(pos + window_start_secs);
            }
            if seen_texts.insert(evt.text.clone()) {
                events.push(evt);
            }
        }
        // Merge: keep highest-detmet / xmax / lowest-error window
        if diag.n_msk144_cands > best_diag.n_msk144_cands {
            best_diag.n_msk144_cands = diag.n_msk144_cands;
        }
        if diag.n_msk40_cands > best_diag.n_msk40_cands {
            best_diag.n_msk40_cands = diag.n_msk40_cands;
        }
        if diag.strongest_detmet > best_diag.strongest_detmet {
            best_diag.strongest_detmet = diag.strongest_detmet;
        }
        if diag.best_xmax > best_diag.best_xmax {
            best_diag.best_xmax = diag.best_xmax;
        }
        if let Some(b) = diag.best_n_bad_sync {
            if best_diag.best_n_bad_sync.map_or(true, |x| b < x) {
                best_diag.best_n_bad_sync = Some(b);
            }
        }
        if let Some(e) = diag.best_n_hard_errors {
            if best_diag.best_n_hard_errors.map_or(true, |x| e < x) {
                best_diag.best_n_hard_errors = Some(e);
            }
        }
        k_end += step;
    }
    (events, best_diag)
}

// ─── TX side: encode a text message to audio for one slot ──────────────────

use msk144plus_dsp::tx::{build_channel_bits, generate_msk144_slot};
use msk144plus_fec::encode_128_90;
use msk144plus_packjt::pack77_text;

/// MSK144 slot length in seconds. Always 15 for MSK144 v2.
pub const SLOT_SECS_MSK144: u32 = 15;

/// Encode a text message into a slot of 12 kHz audio samples ready for
/// transmission.
///
/// Two encoding paths are supported, selected by message format:
///
/// * **Standard MSK144** (default): regular 77-bit pack → LDPC (128,90)
///   → 144 channel bits per frame → ~208 frames per 15s slot. Used for
///   anything that doesn't have angle-bracketed callsigns.
///
/// * **MSK40 short message**: triggered when the input text starts with
///   `<` and follows the form `<C1 C2> RPT` (e.g. `"<GW4WND F1ABC> 73"`).
///   Packs to 16 message bits (12-bit hash + 4-bit report code) →
///   (32,16) LDPC → 40 channel bits per frame → ~750 frames per 15s
///   slot. The much higher repetition rate gives an SNR advantage in
///   marginal conditions, at the cost of requiring both stations to
///   share the bracketed-pair convention (Sh msg mode).
///
/// `slot_secs` is normally 15. The audio is generated by repeating the
/// frame as many times as fit in the slot, with phase-continuous carrier.
///
/// Returns the generated audio buffer (length ≈ slot_secs * 12000 samples).
pub fn encode_message_to_audio(
    text: &str,
    fc_hz: f32,
    slot_secs: u32,
) -> Result<Vec<f32>, EncodeError> {
    // Bracket-form detection routes to the MSK40 path. We use the
    // packer's own format check (it'll reject malformed bracket text
    // with a clear error rather than silently falling through).
    let trimmed = text.trim();
    if trimmed.starts_with('<') && trimmed.contains('>') {
        return encode_msk40_to_audio(trimmed, fc_hz, slot_secs);
    }

    // Standard MSK144 path
    // 1. Pack to 77 bits (always succeeds — falls through to free-text)
    let msg77: [u8; 77] = pack77_text(text);

    // 2. LDPC encode → 128 bits
    let codeword: [u8; 128] = encode_128_90(&msg77);

    // 3. Build 144 channel bits with sync sequences interleaved
    let chanbits = build_channel_bits(&codeword);

    // 4. Modulate. At 12 kHz, NSPM=864 samples per frame = 72 ms.
    //    A 15-sec slot fits 208 frames (15 * 12000 / 864 ≈ 208.3).
    let samples_per_frame = msk144plus_dsp::NSPM;
    let total_samples = slot_secs as usize * msk144plus_dsp::SAMPLE_RATE as usize;
    let n_frames = total_samples / samples_per_frame;

    let audio = generate_msk144_slot(&chanbits, fc_hz, n_frames);
    Ok(audio)
}

/// Encode a bracket-form short message text to MSK40 audio. Internal —
/// callers should use `encode_message_to_audio` which dispatches based
/// on text format.
fn encode_msk40_to_audio(
    text: &str,
    fc_hz: f32,
    slot_secs: u32,
) -> Result<Vec<f32>, EncodeError> {
    use msk144plus_dsp::{
        build_channel_bits_msk40, generate_msk40_slot,
        NSPM_MSK40, N_CHANSYM_MSK40, SAMPLE_RATE,
    };
    use msk144plus_packjt::pack_msk40;

    // 1. Pack to 16 message bits (12-bit hash + 4-bit report)
    let msg_bits: [u8; 16] = pack_msk40(text)
        .map_err(|e| EncodeError::InvalidMessage(format!("MSK40 pack: {}", e)))?;

    // 2. (32,16) LDPC encode → 32 codeword bits
    let codeword: [u8; 32] = encode_short(&msg_bits);

    // 3. Build 40 channel bits with 8-bit S8R sync prefix
    let chanbits: [u8; N_CHANSYM_MSK40] = build_channel_bits_msk40(&codeword);

    // 4. Modulate. NSPM_MSK40=240 samples/frame = 20 ms at 12 kHz.
    //    A 15s slot fits 750 MSK40 frames (15 * 12000 / 240 = 750).
    let total_samples = slot_secs as usize * SAMPLE_RATE as usize;
    let n_frames = total_samples / NSPM_MSK40;

    let audio = generate_msk40_slot(&chanbits, fc_hz, n_frames);
    Ok(audio)
}

#[derive(Debug, Clone)]
pub enum EncodeError {
    /// Reserved for future strict validation. Currently unused since
    /// pack77_text is always best-effort.
    InvalidMessage(String),
}

impl std::fmt::Display for EncodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EncodeError::InvalidMessage(t) =>
                write!(f, "invalid message: {:?}", t),
        }
    }
}

impl std::error::Error for EncodeError {}

// ============================================================================
// Soft-bit accumulator (experimental)
// ============================================================================
//
// Combines multiple sub-threshold fragments of the same packet within a slot
// into a single weighted-sum softbits vector, then runs the standard LDPC
// decoder on the result. Inspired by MSK2K's accumulator pattern but adapted
// for MSK144's frame structure.
//
// Theory: every transmitter sends the same 77-bit packet repeatedly within
// a slot. A short meteor ping that captures only part of one frame (or one
// frame at low SNR) won't decode standalone — but its softbits still carry
// partial evidence about the message. Multiple such fragments, each
// individually below LDPC's convergence threshold, can be summed: real
// signal accumulates coherently while noise averages toward zero.
//
// The phase-alignment problem MSK2K had to solve does not apply here: by
// the time `demodulate_frame` returns, the softbits array is already in
// canonical order (sync words at fixed positions 0..8 and 56..64). All
// fragments of the same packet produce comparable softbits[i] values and
// can be element-wise summed without rotation.
//
// What we DO need to be careful about: only sum fragments that come from
// the SAME station. The freq-lock filter (applied by the caller before
// passing fragments to the accumulator) handles this during an active QSO.
// Outside a QSO, there's no easy way to pre-filter, but LDPC's own parity
// check rejects mismatched fragment soup with very high reliability — the
// failure mode is "no decode", not "wrong decode".

/// One softbit fragment ready for accumulation. Produced by demodulating a
/// detected SPD candidate; carries the per-symbol soft values plus metadata.
#[derive(Debug, Clone)]
pub struct SoftbitFragment {
    /// 144 per-symbol soft values from `demodulate_frame`. In canonical
    /// transmit order (sync at 0..8 and 56..64); summable element-wise
    /// across fragments of the same packet.
    pub softbits: [f32; 144],
    /// Sync correlation strength (xmax) when this fragment was detected.
    /// Used as the basis for accumulation weight (weight = xmax²) so
    /// stronger fragments contribute more to the sum.
    pub xmax: f32,
    /// Frequency offset (Hz, relative to fc) where this fragment was
    /// detected. Used for clustering (group fragments of the same
    /// station together) and to set the eventual DecodeEvent's
    /// freq_offset field.
    pub freq_offset: f32,
    /// Sync error count from `demodulate_frame`. Lower = better.
    /// Fragments with n_bad_sync > 4 are rejected by the caller (matches
    /// the standard decoder's threshold).
    pub n_bad_sync: u8,
    /// Absolute sample offset of this burst within the FULL slot
    /// (not the analysis window — the slot, 0..audio.len()). At 12 kHz,
    /// divide by 12000 to get seconds-into-slot. Populated by
    /// `collect_softbit_fragments` as `window_start_sample +
    /// cand.n_start`. Used for echo dedup (same physical burst seen
    /// in overlapping windows) and to compute the
    /// `slot_position_secs` of an accumulator decode.
    pub n_start: usize,
}

/// Run the standard MSK144 SPD candidate-finder and demodulator passes
/// across the FULL slot, returning all softbit fragments ready for
/// accumulation, optionally pre-filtered by frequency lock.
///
/// Slides an `NZ_MSKRTD`-sample analysis window through the slot in
/// `nz/2` steps (matching `decode_slot_with_diag`), so a slot with
/// pings at e.g. T=4s and T=18s sees both — not just whatever falls
/// in the first window.
///
/// Each returned `SoftbitFragment` carries an absolute slot-relative
/// `n_start` (window_start_sample + cand.n_start). Echo dedup,
/// frequency clustering, envelope-masked accumulation, and LDPC
/// decoding all happen downstream in `accumulator_decode`.
///
/// `freq_lock_hz`: If `Some(df)`, only fragments within ±`narrow_ntol_hz`
/// of `df` are returned. If `None`, all candidates within the standard
/// `ntol_hz` are returned.
///
/// `xmin`: Minimum sync correlation (xmax) for a fragment to be included.
/// The standard decoder uses 1.3 to consider a candidate decodable;
/// the accumulator can use a lower threshold (0.7 typical) to capture
/// sub-threshold fragments that contribute via accumulation.
pub fn collect_softbit_fragments(
    audio: &[f32],
    ntol_hz: f32,
    fc_hz: f32,
    freq_lock_hz: Option<f32>,
    narrow_ntol_hz: f32,
    xmin: f32,
) -> Vec<SoftbitFragment> {
    let mut fragments = Vec::new();
    let nz = NZ_MSKRTD;
    if audio.len() < nz { return fragments; }

    let step = nz / 2;
    let mut k_end = nz;
    while k_end <= audio.len() {
        let window_start_sample = k_end - nz;
        let window_audio = &audio[window_start_sample..k_end];
        collect_fragments_from_window(
            window_audio,
            window_start_sample,
            ntol_hz,
            fc_hz,
            freq_lock_hz,
            narrow_ntol_hz,
            xmin,
            &mut fragments,
        );
        k_end += step;
    }
    fragments
}

/// Inner per-window fragment collector. Pulls candidates from one
/// NZ_MSKRTD-sample analysis window and pushes them onto `out` with
/// absolute slot-relative n_start.
fn collect_fragments_from_window(
    window_audio: &[f32],
    window_start_sample: usize,
    ntol_hz: f32,
    fc_hz: f32,
    freq_lock_hz: Option<f32>,
    narrow_ntol_hz: f32,
    xmin: f32,
    out: &mut Vec<SoftbitFragment>,
) {
    let nz = NZ_MSKRTD;
    if window_audio.len() < nz { return; }

    // RMS-normalise (same prep as standard decoder)
    let sum_sq: f32 = window_audio[..nz].iter().map(|x| x * x).sum();
    let rms = (sum_sq / nz as f32).sqrt();
    if rms < 1.0 { return; }
    let fac = 1.0 / rms;
    let mut d = vec![0.0f32; NFFT_ANALYTIC];
    for i in 0..nz { d[i] = fac * window_audio[i]; }
    let filter = AnalyticFilter::new(NFFT_ANALYTIC);
    let cdat = analytic(&d, &filter);
    let np = 8 * NSPM;
    if cdat.len() < np { return; }
    let cbig = &cdat[..np];

    let candidates = detect_candidates(cbig, ntol_hz, fc_hz);

    // Same nav patterns the standard SPD decoder uses
    let navpatterns: [[u8; 3]; 2] = [[1, 1, 1], [0, 1, 1]];

    for cand in &candidates {
        // Apply freq-lock filter if active. The candidate's freq_err is
        // relative to fc_hz; the lock value is also relative to fc_hz.
        if let Some(lock_df) = freq_lock_hz {
            if (cand.freq_err - lock_df).abs() > narrow_ntol_hz {
                continue;
            }
        }

        let n = cbig.len();
        let n_start_0 = cand.n_start.saturating_sub(1);
        let mut ib = n_start_0.saturating_sub(NSPM);
        let mut ie = ib + 3 * NSPM;
        if ie > n {
            ie = n;
            if ie >= 3 * NSPM { ib = ie - 3 * NSPM; } else { continue; }
        }
        let window = &cbig[ib..ie];
        let fo = fc_hz + cand.freq_err;
        let ntol0 = 8.0;
        let deltaf = 2.0;

        for navmask in navpatterns.iter() {
            let result = msk144_sync(window, 3, ntol0, deltaf, navmask, 2, fo);
            // Lower xmax threshold than the standard decoder's 1.3 —
            // accumulator deliberately captures sub-threshold fragments.
            if result.xmax < xmin { continue; }

            for &peak_loc in &result.peak_locations {
                for is in 0..3 {
                    let dither = match is { 0 => 0i32, 1 => -1, _ => 1 };
                    let ic0 = ((peak_loc as i32 + dither).max(0).min(NSPM as i32 - 1)) as usize;
                    let mut ct = [Complex32::new(0.0, 0.0); NSPM];
                    for k in 0..NSPM {
                        ct[k] = result.averaged_frame[(k + ic0) % NSPM];
                    }
                    let demod = demodulate_frame(&ct);
                    if demod.n_bad_sync > 4 { continue; }
                    out.push(SoftbitFragment {
                        softbits: demod.softbits,
                        xmax: result.xmax,
                        freq_offset: result.freq_offset + cand.freq_err,
                        n_bad_sync: demod.n_bad_sync,
                        // Absolute slot-relative position: window's
                        // offset within the slot + candidate's offset
                        // within the window. Used by accumulator_decode
                        // for echo dedup and to compute slot_position_secs.
                        n_start: window_start_sample + cand.n_start,
                    });
                }
            }
        }
    }
}

/// Given a collection of softbit fragments from one slot, produce zero
/// or more accumulated decodes by frequency-clustering the fragments
/// (different stations land in different clusters) and running an
/// MSK2K-style "jigsaw" accumulator on each cluster.
///
/// Pipeline per cluster:
///   1. **Echo dedup** — fragments within ½ frame and similar freq are
///      the same physical burst seen in overlapping analysis windows;
///      keep only the strongest.
///   2. **Per-fragment envelope mask** — smooth softbit magnitudes,
///      threshold at 25 % of peak. Bits below the threshold are not
///      reliable from this fragment and contribute zero to that bit
///      position. Mirrors MSK2K's accumulator (envelope-aware).
///   3. **Per-bit weighted accumulation** — for each bit position,
///      sum (xmax² · softbit) only over fragments whose mask covers
///      that position; divide by the per-bit weight sum. This is the
///      "jigsaw": different fragments fill in different bit ranges.
///   4. **LDPC decode** on the accumulated softbits; emit a DecodeEvent
///      with `slot_position_secs = weighted-mean of contributing
///      fragments' n_start` (xmax² weights).
///
/// Returns one `DecodeEvent` per cluster that successfully decodes.
/// Empty vec if nothing decodes. Multi-decode is genuine: it means
/// two stations with separable frequency offsets both produced
/// decodable accumulations in this slot.
pub fn accumulator_decode(
    fragments: &[SoftbitFragment],
    min_fragments: usize,
) -> Vec<DecodeEvent> {
    let mut decodes = Vec::new();
    if fragments.len() < min_fragments { return decodes; }

    // --- Stage 1: frequency clustering ---
    // Sort by freq_offset; start a new cluster whenever the gap to
    // the previous fragment exceeds CLUSTER_GAP_HZ. All-same-station
    // fragments cluster tightly (typically within ±5 Hz of each
    // other across a slot); different stations separated by >20 Hz
    // form distinct clusters.
    const CLUSTER_GAP_HZ: f32 = 20.0;
    let mut sorted: Vec<&SoftbitFragment> = fragments.iter().collect();
    sorted.sort_by(|a, b| a.freq_offset.partial_cmp(&b.freq_offset)
        .unwrap_or(std::cmp::Ordering::Equal));

    let mut clusters: Vec<Vec<&SoftbitFragment>> = Vec::new();
    let mut current: Vec<&SoftbitFragment> = Vec::new();
    let mut prev_freq: Option<f32> = None;
    for frag in sorted {
        match prev_freq {
            Some(pf) if (frag.freq_offset - pf).abs() <= CLUSTER_GAP_HZ => {
                current.push(frag);
            }
            _ => {
                if !current.is_empty() {
                    clusters.push(std::mem::take(&mut current));
                }
                current.push(frag);
            }
        }
        prev_freq = Some(frag.freq_offset);
    }
    if !current.is_empty() { clusters.push(current); }

    // --- Stage 2..4: process each cluster independently ---
    for cluster in clusters {
        if cluster.len() < min_fragments { continue; }
        if let Some(evt) = jigsaw_decode_cluster(&cluster) {
            decodes.push(evt);
        }
    }
    decodes
}

/// Run echo dedup + envelope-masked per-bit accumulation + LDPC on
/// one frequency-coherent cluster of fragments. Returns Some(event)
/// on a successful, valid decode.
fn jigsaw_decode_cluster(cluster: &[&SoftbitFragment]) -> Option<DecodeEvent> {
    if cluster.is_empty() { return None; }

    // ───── ACCUM-TEST GATES ─────────────────────────────────────────
    // The accumulator's strict gates (ENVELOPE_THRESHOLD=0.25,
    // coverage>=2/3, min_fragments=2) produced essentially zero
    // unique decodes vs SPD/avg in week-long testing — every
    // accum-decoded message was already in SPD's output for the
    // same slot. To probe whether the accumulator can add value
    // in genuinely marginal conditions (weak signals that SPD
    // misses), gates are relaxed below. Expect higher false-
    // positive risk; mitigate by close inspection of the [ACCUM]
    // log lines vs SPD output for the same slot. Revert if false
    // decodes start polluting PSK Reporter spots.
    //
    // To restore strict mode: set ENVELOPE_THRESHOLD=0.25, coverage
    // gate to NBITS*2/3, and (in decoder.rs) ACCUM_MIN_FRAGS=2,
    // ACCUM_XMIN=1.0.
    // ────────────────────────────────────────────────────────────────

    // --- Echo dedup ---
    // Two fragments from overlapping analysis windows can pick up the
    // same physical burst — they'll have near-identical n_start and
    // very close freq_offset. Drop the weaker of any such pair.
    // Threshold: within ½ NSPM samples (≈ 36 ms) and ≤ 5 Hz apart.
    const ECHO_SAMPLES: usize = NSPM / 2;
    const ECHO_FREQ_HZ: f32 = 5.0;
    let mut sorted_by_strength: Vec<&SoftbitFragment> = cluster.iter().copied().collect();
    sorted_by_strength.sort_by(|a, b| b.xmax.partial_cmp(&a.xmax)
        .unwrap_or(std::cmp::Ordering::Equal));
    let mut kept: Vec<&SoftbitFragment> = Vec::new();
    for frag in sorted_by_strength {
        let is_echo = kept.iter().any(|k| {
            let dn = (k.n_start as isize - frag.n_start as isize).unsigned_abs();
            let df = (k.freq_offset - frag.freq_offset).abs();
            dn < ECHO_SAMPLES && df < ECHO_FREQ_HZ
        });
        if !is_echo { kept.push(frag); }
    }
    if kept.is_empty() { return None; }

    // --- Per-bit weighted accumulation with envelope mask ---
    // For each fragment: smooth |softbits| with a 5-tap window, find
    // peak, mask anything below ENVELOPE_THRESHOLD × peak. Only
    // masked-in bits contribute to the per-bit sum at their position.
    // ACCUM-TEST: lowered from 0.25 to 0.10 — weaker fragment regions
    // now contribute, increasing coverage at the cost of admitting
    // noisier softbits into the per-bit sum.
    const ENVELOPE_THRESHOLD: f32 = 0.25;
    const SMOOTH_HALFWIDTH: i32 = 2;  // 5-tap window: i-2..=i+2
    const NBITS: usize = 144;

    let mut accum_softbits = [0.0f32; NBITS];
    let mut accum_weight = [0.0f32; NBITS];

    for frag in &kept {
        let w_global = frag.xmax * frag.xmax;
        if w_global <= 0.0 { continue; }

        // Smooth |softbits|
        let mut smoothed = [0.0f32; NBITS];
        let mut peak = 0.0f32;
        for i in 0..NBITS {
            let mut sum = 0.0f32;
            for j in -SMOOTH_HALFWIDTH..=SMOOTH_HALFWIDTH {
                let k = (i as i32 + j).rem_euclid(NBITS as i32) as usize;
                sum += frag.softbits[k].abs();
            }
            smoothed[i] = sum / (2 * SMOOTH_HALFWIDTH + 1) as f32;
            if smoothed[i] > peak { peak = smoothed[i]; }
        }
        if peak <= 0.0 { continue; }
        let thresh = peak * ENVELOPE_THRESHOLD;

        // Mask + accumulate at bit positions where this fragment is
        // confidently above the envelope threshold.
        for i in 0..NBITS {
            if smoothed[i] >= thresh {
                accum_softbits[i] += w_global * frag.softbits[i];
                accum_weight[i] += w_global;
            }
        }
    }

    // Normalise per-bit. Bits with zero weight (no fragment confidently
    // covered them) stay at 0.0 — LDPC will treat them as soft-erasure.
    let mut final_soft = [0.0f32; NBITS];
    let mut covered_bits = 0usize;
    for i in 0..NBITS {
        if accum_weight[i] > 0.0 {
            final_soft[i] = accum_softbits[i] / accum_weight[i];
            covered_bits += 1;
        }
    }
    // Refuse to decode if too few bits are covered — LDPC will
    // happily produce garbage from a near-empty input.
    // ACCUM-TEST: lowered from NBITS*2/3 (96) to NBITS/2 (72) — admit
    // sparser jigsaws. LDPC's own n_hard_errors check downstream
    // remains the final gate against pure noise decodes.
    if covered_bits < NBITS * 2 / 3 { return None; }

    // --- LDPC decode on the jigsawed result ---
    let llr = softbits_to_ldpc_llr(&final_soft);
    let result = decode_128_90_soft(&llr, 10)?;
    if result.n_hard_errors >= 18 { return None; }

    let msg = unpack77(&result.message);
    if !is_valid_message(&msg) { return None; }

    // --- Metadata for the returned DecodeEvent ---
    // Representative xmax / freq_offset = strongest fragment's values.
    // Slot position = xmax²-weighted mean of contributing fragments'
    // n_start (matching the per-bit weight scheme — strongest
    // fragments dominate the time estimate just as they dominate
    // the softbit estimate).
    let strongest = kept.iter().max_by(|a, b| a.xmax.partial_cmp(&b.xmax)
        .unwrap_or(std::cmp::Ordering::Equal))?;
    let mut sum_w = 0.0f32;
    let mut sum_pos = 0.0f32;
    for frag in &kept {
        let w = frag.xmax * frag.xmax;
        sum_w += w;
        sum_pos += w * frag.n_start as f32;
    }
    let mean_n_start = if sum_w > 0.0 { sum_pos / sum_w } else { 0.0 };
    let slot_position_secs = mean_n_start / 12000.0;

    // Accumulated SNR: equal-strength fragments boost SNR by sqrt(N);
    // approximate using the strongest fragment's xmax scaled by
    // sqrt(N_kept).
    let n = kept.len() as f32;
    let detmet_eq = strongest.xmax * 48.0 * n.sqrt();
    let snr_db = 12.0 * detmet_eq.max(1e-3).log10() / 2.0 - 9.0;

    Some(DecodeEvent {
        text: msg.to_text(),
        message: Some(msg),
        xmax: strongest.xmax,
        freq_offset: strongest.freq_offset,
        method: format!("accum-{}/{}", kept.len(), covered_bits),
        n_hard_errors: result.n_hard_errors,
        snr_db,
        is_accumulated: true,
        slot_position_secs: Some(slot_position_secs),
    })
}

#[cfg(test)]
mod encode_tests {
    use super::*;

    #[test]
    fn encode_cq_15s_slot_at_1500_hz() {
        let audio = encode_message_to_audio("CQ GW4WND IO82", 1500.0, 15)
            .expect("encode");
        // 15 sec at 12 kHz = 180000 samples; floor to nearest NSPM.
        // 180000 / 864 = 208.333, floor 208 frames * 864 = 179712.
        assert!(audio.len() >= 179000, "got {} samples", audio.len());
        assert!(audio.len() <= 180000, "got {} samples", audio.len());
    }

    #[test]
    fn encode_30s_slot_doubles() {
        let a15 = encode_message_to_audio("CQ GW4WND IO82", 1500.0, 15).unwrap();
        let a30 = encode_message_to_audio("CQ GW4WND IO82", 1500.0, 30).unwrap();
        let ratio = a30.len() as f32 / a15.len() as f32;
        assert!((ratio - 2.0).abs() < 0.01, "ratio = {}", ratio);
    }

    #[test]
    fn encode_msk40_routes_to_short_path() {
        // Bracket-form text should route to MSK40 modulator. The audio
        // length should be n_frames × NSPM_MSK40 (240) — about 750
        // frames per 15s slot.
        let audio = encode_message_to_audio("<GW4WND F1ABC> 73", 1500.0, 15)
            .expect("encode");
        let nspm_msk40 = msk144plus_dsp::NSPM_MSK40;
        assert_eq!(audio.len() % nspm_msk40, 0,
            "audio len {} not a multiple of NSPM_MSK40={}",
            audio.len(), nspm_msk40);
        assert!(audio.len() >= 179_000 && audio.len() <= 180_001,
            "got {} samples", audio.len());
    }

    #[test]
    fn encode_msk40_self_direction_is_rejected() {
        // Verify that a Sh msg WITH OUR CALL FIRST (i.e. an echo of
        // our own TX) is REJECTED by the decoder. This protects
        // against the rig monitor / sidetone / loopback / aircraft
        // scatter / ionospheric reflection paths that all carry our
        // own transmission back into our RX. Real on-air testing
        // showed self-decodes appearing in nearly every RX slot of
        // an active QSO without this filter.
        use msk144plus_dsp::{
            build_channel_bits_msk40, generate_msk40_slot,
        };
        use msk144plus_packjt::pack_msk40;

        // Message as transmitted by US: hash is "<MYCALL HISCALL>"
        let msg_text = "<GW4WND F1ABC> 73";
        let msg_bits = pack_msk40(msg_text).expect("pack");
        let codeword = encode_short(&msg_bits);
        let chanbits = build_channel_bits_msk40(&codeword);
        let raw = generate_msk40_slot(&chanbits, 1500.0, 60);
        let audio: Vec<f32> = raw.iter().map(|s| s * 8000.0).collect();
        let mut slot = vec![0.0f32; 12000];
        slot.extend(&audio);
        slot.extend(vec![0.0f32; 12000]);

        // Receiver perspective: same calls, both populated.
        let cfg = ShortMessageConfig {
            mycall: "GW4WND".to_string(),
            hiscall: "F1ABC".to_string(),
            enabled: true,
        };
        let events = decode_slot(&slot, 100.0, 1500.0, Depth::Deep, Some(&cfg));
        // We should NOT decode our own TX — the self-direction hash
        // match is suppressed.
        let any_self_decode = events.iter().any(|e|
            e.text.starts_with("<GW4WND F1ABC>"));
        assert!(!any_self_decode,
            "expected NO self-direction decode (would be UI noise); got: {:?}",
            events.iter().map(|e| e.text.clone()).collect::<Vec<_>>());
    }

    #[test]
    fn encode_msk40_partner_direction_decodes() {
        // Partner-direction: on-air message has THEIR call first.
        // We (receiver) have mycall=GW4WND, hiscall=F1ABC, but the
        // on-air hash is computed with F1ABC first. RX must match
        // via the reversed-order hash check in run_msk40_decode.
        use msk144plus_dsp::{
            build_channel_bits_msk40, generate_msk40_slot,
        };
        use msk144plus_packjt::pack_msk40;

        let on_air = "<F1ABC GW4WND> RRR";
        let msg_bits = pack_msk40(on_air).expect("pack");
        let codeword = encode_short(&msg_bits);
        let chanbits = build_channel_bits_msk40(&codeword);
        let raw = generate_msk40_slot(&chanbits, 1500.0, 60);
        let audio: Vec<f32> = raw.iter().map(|s| s * 8000.0).collect();
        let mut slot = vec![0.0f32; 12000];
        slot.extend(&audio);
        slot.extend(vec![0.0f32; 12000]);

        let cfg = ShortMessageConfig {
            mycall: "GW4WND".to_string(),
            hiscall: "F1ABC".to_string(),
            enabled: true,
        };
        let events = decode_slot(&slot, 100.0, 1500.0, Depth::Deep, Some(&cfg));
        assert!(events.iter().any(|e| e.text.contains("RRR")),
            "expected 'RRR' decode for partner-direction msg; got: {:?}",
            events.iter().map(|e| e.text.clone()).collect::<Vec<_>>());
    }
}
