// crates/msk144plus_dsp/src/spd.rs
//
// Faithful port of WSJT-X msk144spd.f90 (197 lines).
//
// "Short-Ping Detector": the front-end that finds candidate MSK144 ping
// locations within a buffer of analytic baseband. For each 18ms step
// across the buffer, it:
//   1. Takes a 72ms (NSPM-sample) window
//   2. Squares the complex signal (collapses MSK FM into pure tones at
//      2*f_data: ~2000 Hz baseband for the lower MSK tone, ~4000 Hz for
//      the upper)
//   3. Applies raised-cosine edge windowing
//   4. FFTs
//   5. Looks for peaks in two windows around 2*(fc-500) and 2*(fc+500)
//   6. Computes detmet (peak amplitude) and detmet2 (peak/avg ratio)
//
// After the sweep, normalises detmet by the 25th-percentile median (noise
// floor → 1.0), then picks up to MAXCAND=5 candidates with detmet >= 3.0
// (fallback to detmet2 >= 12.0 if fewer than 3 found).
//
// This is the FRONT-END only. The back-end decode loop (which calls
// msk144_sync + msk144decodeframe per candidate) is in the engine layer.

use crate::constants::NSPM;
use num_complex::Complex32;
use rustfft::FftPlanner;
use std::f32::consts::PI;

/// Maximum candidates to return.
pub const MAXCAND: usize = 5;
/// Maximum step count we'll consider (Fortran param MAXSTEPS=100).
pub const MAXSTEPS: usize = 100;
/// Step size in samples (Fortran: 216 = 18ms at 12 kHz).
pub const STEP_SAMPLES: usize = 216;
/// FFT size (Fortran: NFFT = NSPM = 864).
pub const NFFT_SPD: usize = 864;

/// Raised-cosine edge window: rcw(i) = (1 - cos((i-1)*pi/12)) / 2 for i=1..12.
/// Used to taper the first/last 12 samples of the 864-sample window before
/// squaring. Mirrors msk144spd.f90 lines 56-59.
pub fn raised_cosine_edge_window() -> [f32; 12] {
    let mut rcw = [0.0f32; 12];
    for i in 0..12 {
        rcw[i] = (1.0 - (i as f32 * PI / 12.0).cos()) / 2.0;
    }
    rcw
}

/// One detection candidate from the spd front-end.
#[derive(Debug, Clone)]
pub struct SpdCandidate {
    /// Sample index within `cbig` where this candidate starts (Fortran:
    /// nstart). The 3-frame analysis window for this candidate is
    /// `cbig[nstart .. nstart + 3*NSPM]`.
    pub n_start: usize,
    /// Estimated frequency offset from fc (Hz). Fortran: ferrs(icand).
    pub freq_err: f32,
    /// Estimated SNR in dB. Fortran: snrs(icand).
    pub snr_db: f32,
    /// detmet at this candidate (post-normalisation).
    pub detmet: f32,
    /// detmet2 at this candidate.
    pub detmet2: f32,
    /// True if found via the primary detmet >= 3.0 path,
    /// false if from the detmet2 >= 12.0 fallback.
    pub primary: bool,
    /// Soft demodulated bits at this candidate's sync position. Populated
    /// by the per-candidate decode pass when the soft-bit accumulator is
    /// enabled; `None` otherwise to avoid allocating during normal decode.
    ///
    /// Order: pre-deinterleave, in transmitted symbol order. Sign carries
    /// the bit decision (negative = '0', positive = '1'); magnitude is
    /// confidence. Compatible with element-wise summation across multiple
    /// candidates of the same packet — the basis of soft-bit accumulation.
    pub soft_bits: Option<Vec<f32>>,
}

/// Run the msk144spd front-end on a buffer of analytic baseband.
///
/// Faithful port of msk144spd.f90 lines 64-153 (everything before the
/// per-candidate decode loop).
///
/// Inputs:
///   `cbig`: analytic baseband buffer
///   `ntol_hz`: frequency tolerance window (Hz)
///   `fc_hz`: nominal carrier frequency (Hz)
///
/// Returns: candidate list (up to MAXCAND), sorted by detection strength.
pub fn detect_candidates(cbig: &[Complex32], ntol_hz: f32, fc_hz: f32) -> Vec<SpdCandidate> {
    let n = cbig.len();
    if n < NSPM {
        return Vec::new();
    }

    // nstep = (n - NSPM) / 216  but cap at MAXSTEPS
    let nstep = ((n - NSPM) / STEP_SAMPLES).min(MAXSTEPS);
    if nstep == 0 {
        return Vec::new();
    }

    let fs = 12000.0_f32;
    let df = fs / NFFT_SPD as f32;

    // Tone window indices (msk144spd.f90 lines 69-76)
    //   nfhi = 2*(fc+500)        ← upper tone in squared spectrum
    //   nflo = 2*(fc-500)        ← lower tone in squared spectrum
    //   ihlo = round((nfhi-2*ntol)/df) + 1
    //   ihhi = round((nfhi+2*ntol)/df) + 1
    //   illo = round((nflo-2*ntol)/df) + 1
    //   ilhi = round((nflo+2*ntol)/df) + 1
    //   i2000 = round(nflo/df) + 1
    //   i4000 = round(nfhi/df) + 1
    //
    // Convert Fortran 1-indexed to Rust 0-indexed: subtract 1 from each.
    let nfhi = 2.0 * (fc_hz + 500.0);
    let nflo = 2.0 * (fc_hz - 500.0);
    let ihlo = ((nfhi - 2.0 * ntol_hz) / df).round() as i32; // 0-indexed already
    let ihhi = ((nfhi + 2.0 * ntol_hz) / df).round() as i32;
    let illo = ((nflo - 2.0 * ntol_hz) / df).round() as i32;
    let ilhi = ((nflo + 2.0 * ntol_hz) / df).round() as i32;
    let i2000 = (nflo / df).round() as i32;
    let i4000 = (nfhi / df).round() as i32;

    let rcw = raised_cosine_edge_window();
    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(NFFT_SPD);

    // Per-step detection results
    let mut detmet = vec![0.0f32; nstep];
    let mut detmet2 = vec![0.0f32; nstep];
    let mut detfer = vec![-999.99f32; nstep];

    let mut ctmp = vec![Complex32::new(0.0, 0.0); NFFT_SPD];

    for istp in 0..nstep {
        let ns = STEP_SAMPLES * istp; // Fortran: ns = 1 + 216*(istp-1) → Rust 0-indexed
        let ne = ns + NSPM;
        if ne > n {
            break;
        }

        // ctmp = cbig[ns..ne], then square, edge window, FFT
        for i in 0..NFFT_SPD {
            ctmp[i] = Complex32::new(0.0, 0.0);
        }
        for i in 0..NSPM {
            let v = cbig[ns + i];
            ctmp[i] = v * v; // square
        }
        // Apply raised-cosine edge window to first/last 12 samples
        // Fortran: ctmp(1:12) = ctmp(1:12) * rcw  (rising edge, rcw(1)=0 at start, rcw(12)≈1 at end)
        //         ctmp(NSPM-11:NSPM) = ctmp(NSPM-11:NSPM) * rcw(12:1:-1)  (falling edge)
        for i in 0..12 {
            ctmp[i].re *= rcw[i];
            ctmp[i].im *= rcw[i];
        }
        for i in 0..12 {
            ctmp[NSPM - 12 + i].re *= rcw[11 - i];
            ctmp[NSPM - 12 + i].im *= rcw[11 - i];
        }

        // FFT (forward)
        fft.process(&mut ctmp);

        // Tone spectrum = |ctmp|²
        let tonespec: Vec<f32> = ctmp.iter().map(|c| c.norm_sqr()).collect();

        // Find peak in upper-tone window (ihlo..=ihhi, clamped)
        let ihlo_c = ihlo.max(1).min(NFFT_SPD as i32 - 2) as usize;
        let ihhi_c = ihhi.max(1).min(NFFT_SPD as i32 - 2) as usize;
        let illo_c = illo.max(1).min(NFFT_SPD as i32 - 2) as usize;
        let ilhi_c = ilhi.max(1).min(NFFT_SPD as i32 - 2) as usize;

        let (ihpk, &ah) = (ihlo_c..=ihhi_c)
            .map(|k| (k, &tonespec[k]))
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
            .unwrap_or((ihlo_c, &0.0));
        let count_h = ihhi_c - ihlo_c + 1;
        let sum_h: f32 = tonespec[ihlo_c..=ihhi_c].iter().sum();
        let ahavp = (sum_h - ah) / count_h.max(1) as f32;
        let trath = ah / (ahavp + 0.01);
        // Parabolic interpolation:
        //   delta = -Re((ctmp(ihpk-1) - ctmp(ihpk+1)) /
        //                (2*ctmp(ihpk) - ctmp(ihpk-1) - ctmp(ihpk+1)))
        let denom_h = ctmp[ihpk] * Complex32::new(2.0, 0.0) - ctmp[ihpk - 1] - ctmp[ihpk + 1];
        let num_h = ctmp[ihpk - 1] - ctmp[ihpk + 1];
        let deltah = if denom_h.norm_sqr() > 1e-30 {
            -(num_h / denom_h).re
        } else {
            0.0
        };

        let (ilpk, &al) = (illo_c..=ilhi_c)
            .map(|k| (k, &tonespec[k]))
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
            .unwrap_or((illo_c, &0.0));
        let count_l = ilhi_c - illo_c + 1;
        let sum_l: f32 = tonespec[illo_c..=ilhi_c].iter().sum();
        let alavp = (sum_l - al) / count_l.max(1) as f32;
        let tratl = al / (alavp + 0.01);
        let denom_l = ctmp[ilpk] * Complex32::new(2.0, 0.0) - ctmp[ilpk - 1] - ctmp[ilpk + 1];
        let num_l = ctmp[ilpk - 1] - ctmp[ilpk + 1];
        let deltal = if denom_l.norm_sqr() > 1e-30 {
            -(num_l / denom_l).re
        } else {
            0.0
        };

        let ferrh = (ihpk as f32 + deltah - i4000 as f32) * df / 2.0;
        let ferrl = (ilpk as f32 + deltal - i2000 as f32) * df / 2.0;
        let ferr = if ah >= al { ferrh } else { ferrl };

        detmet[istp] = ah.max(al);
        detmet2[istp] = trath.max(tratl);
        detfer[istp] = ferr;
    }

    // Median normalisation. Fortran:
    //   call indexx(detmet(1:nstep), nstep, indices)
    //   xmed = detmet(indices(nstep/4))     ← 25th percentile
    //   detmet = detmet / xmed
    let mut sorted = detmet.clone();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let xmed = sorted[nstep / 4].max(1e-12);
    for v in detmet.iter_mut() {
        *v /= xmed;
    }

    // Primary candidate search: detmet >= 3.0
    let mut detmet_work = detmet.clone();
    let mut candidates = Vec::new();
    for _ in 0..MAXCAND {
        let (il, &dm) = detmet_work
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
            .unwrap();
        if dm < 3.0 {
            break;
        }
        if detfer[il].abs() <= ntol_hz {
            // Fortran: nstart(ndet) = 1 + (il-1)*216 + 1 = (il-1)*216 + 2
            // Rust 0-indexed (il is 0-indexed): (il)*216 + 1
            let n_start = il * STEP_SAMPLES + 1;
            // Fortran: snrs(ndet) = 12.0 * log10(detmet(il))/2 - 9.0
            let snr = 12.0 * (dm.log10()) / 2.0 - 9.0;
            candidates.push(SpdCandidate {
                n_start,
                freq_err: detfer[il],
                snr_db: snr,
                detmet: dm,
                detmet2: detmet2[il],
                primary: true,
                soft_bits: None,
            });
        }
        detmet_work[il] = 0.0;
    }

    // Fallback: detmet2 >= 12.0 if fewer than 3 primary candidates
    if candidates.len() < 3 {
        let mut detmet2_work = detmet2.clone();
        // Don't re-pick already-found candidates - clear those positions in detmet2
        for cand in &candidates {
            let il = (cand.n_start - 1) / STEP_SAMPLES;
            if il < detmet2_work.len() {
                detmet2_work[il] = 0.0;
            }
        }
        let need = MAXCAND - candidates.len();
        for _ in 0..need {
            let (il, &dm2) = detmet2_work
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
                .unwrap();
            if dm2 < 12.0 {
                break;
            }
            if detfer[il].abs() <= ntol_hz {
                let n_start = il * STEP_SAMPLES + 1;
                let snr = 12.0 * (dm2.log10()) / 2.0 - 9.0;
                candidates.push(SpdCandidate {
                    n_start,
                    freq_err: detfer[il],
                    snr_db: snr,
                    detmet: detmet[il],
                    detmet2: dm2,
                    primary: false,
                    soft_bits: None,
                });
            }
            detmet2_work[il] = 0.0;
        }
    }

    candidates
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analytic::{analytic, AnalyticFilter};
    use crate::tx::{build_channel_bits, generate_msk144_slot};
    use msk144plus_fec::encode_128_90;
    use msk144plus_packjt::pack77_text;

    #[test]
    fn rcw_endpoints() {
        let rcw = raised_cosine_edge_window();
        // rcw(1) = (1 - cos(0))/2 = 0
        assert!(rcw[0].abs() < 1e-6);
        // rcw(12) = (1 - cos(11π/12))/2 ≈ (1 - (-0.966))/2 ≈ 0.983
        assert!((rcw[11] - (1.0 - (11.0 * PI / 12.0).cos()) / 2.0).abs() < 1e-6);
    }

    #[test]
    fn detect_finds_pulse_in_silence_at_1500() {
        // The msk144spd front-end is designed to find SHORT PINGS in
        // a mostly-silent slot. With a continuous signal the median
        // normalisation collapses everything to ~1.0. To exercise spd
        // properly, we embed a 4-frame burst in 8 frames of silence.
        let payload = pack77_text("CQ K1ABC FN42");
        let codeword = encode_128_90(&payload);
        let bits = build_channel_bits(&codeword);
        let burst = generate_msk144_slot(&bits, 1500.0, 4);

        // Build 12 frames of audio: 4 silence, 4 burst, 4 silence
        let mut audio = vec![0.0f32; 12 * NSPM];
        let burst_start = 4 * NSPM;
        for (i, &s) in burst.iter().enumerate() {
            audio[burst_start + i] = s;
        }

        let nfft = 16384;
        let filter = AnalyticFilter::new(nfft);
        let mut input = vec![0.0f32; nfft];
        input[..audio.len()].copy_from_slice(&audio);
        let baseband = analytic(&input, &filter);
        let cbig: Vec<Complex32> = baseband[..12 * NSPM].to_vec();

        let cands = detect_candidates(&cbig, 100.0, 1500.0);
        eprintln!("burst-in-silence: {} candidates found", cands.len());
        for (i, c) in cands.iter().enumerate() {
            eprintln!("  [{}] n_start={} ferr={:.1} detmet={:.2} primary={}",
                i, c.n_start, c.freq_err, c.detmet, c.primary);
        }
        assert!(!cands.is_empty(), "should detect candidates on burst");
        assert!(
            cands[0].detmet >= 3.0 || cands[0].detmet2 >= 12.0,
            "top candidate should pass primary or fallback gate; got detmet={} detmet2={}",
            cands[0].detmet, cands[0].detmet2
        );
        // The candidate's start should be within the burst region.
        // burst spans samples 4*864..8*864 = 3456..6912. Allow some slack.
        assert!(
            cands[0].n_start >= 3000 && cands[0].n_start <= 7000,
            "candidate position {} should be within burst region",
            cands[0].n_start
        );
        assert!(cands[0].freq_err.abs() < 10.0,
            "freq_err={} expected near 0", cands[0].freq_err);
    }

    #[test]
    fn detect_finds_off_center_pulse() {
        let payload = pack77_text("CQ K1ABC FN42");
        let codeword = encode_128_90(&payload);
        let bits = build_channel_bits(&codeword);
        let burst = generate_msk144_slot(&bits, 1488.0, 4);

        let mut audio = vec![0.0f32; 12 * NSPM];
        let burst_start = 4 * NSPM;
        for (i, &s) in burst.iter().enumerate() {
            audio[burst_start + i] = s;
        }

        let nfft = 16384;
        let filter = AnalyticFilter::new(nfft);
        let mut input = vec![0.0f32; nfft];
        input[..audio.len()].copy_from_slice(&audio);
        let baseband = analytic(&input, &filter);
        let cbig: Vec<Complex32> = baseband[..12 * NSPM].to_vec();

        let cands = detect_candidates(&cbig, 100.0, 1500.0);
        assert!(!cands.is_empty());
        eprintln!("off-center spd: detmet={} ferr={}",
            cands[0].detmet, cands[0].freq_err);
        assert!(
            (cands[0].freq_err - (-12.0)).abs() < 10.0,
            "freq_err={} expected near -12",
            cands[0].freq_err
        );
    }

    #[test]
    fn detect_returns_empty_on_silence() {
        // 8 frames of zero audio
        let cbig = vec![Complex32::new(0.0, 0.0); 7 * NSPM];
        let cands = detect_candidates(&cbig, 100.0, 1500.0);
        assert!(cands.is_empty(), "silence should not produce candidates");
    }
}
