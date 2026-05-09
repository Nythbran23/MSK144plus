// crates/msk144plus_dsp/src/decode_frame.rs
//
// Faithful port of WSJT-X msk144decodeframe.f90 (113 lines).
//
// Input: 864-sample complex frame (NSPM samples) ALIGNED to the sync word.
//   The first 8 channel symbols (samples 1..48 in Fortran, 0..48 in Rust)
//   should contain the sync word; symbols 56..63 contain the second sync word.
//
// Output: Result containing 144 softbits, sync-error count, and (when BP
//   succeeds) the decoded 77-bit message.
//
// This module produces the demodulated softbits + LLR; BP and unpacking
// happen in the engine layer.
//
// Reference: lib/msk144decodeframe.f90.

use crate::constants::{N_CHANSYM, NSPM, S8};
use num_complex::Complex32;
use std::f32::consts::PI;

/// Result of demodulating one MSK144 frame.
pub struct DemodulatedFrame {
    /// 144 softbits. Sign convention: positive = bit 1, negative = bit 0.
    pub softbits: [f32; N_CHANSYM],
    /// Sync error count (0..=8). >4 means the frame is rejected.
    pub n_bad_sync: u8,
    /// Estimated carrier phase that was removed (radians).
    pub phase0: f32,
}

/// Half-sine pulse pp(i) = sin(i*pi/12) for i=0..12. (Fortran 1-indexed:
/// pp(1)=sin(0), pp(12)=sin(11*pi/12). Rust 0-indexed matches.)
pub fn half_sine_pulse() -> [f32; 12] {
    let mut pp = [0.0f32; 12];
    for i in 0..12 {
        pp[i] = (i as f32 * PI / 12.0).sin();
    }
    pp
}

/// Build the 42-sample matched-filter sync waveform `cb` used in
/// msk144decodeframe.f90 (lines 36-44). The Fortran builds it lazily on first
/// call and caches it; we just compute it on demand.
///
/// Construction: 4 of the 8 sync symbols go on the I channel (cbi), 4 on Q
/// (cbq), each shaped by the half-sine pulse pp. The first symbol on Q
/// (s8(1)) uses only the second half of pp (pp(7:12)) because of the
/// half-symbol offset in MSK; the last symbol on I (s8(8)) uses only the
/// first half (pp(1:6)).
pub fn build_sync_waveform() -> [Complex32; 42] {
    let pp = half_sine_pulse();
    // s8 from Fortran data statement: 0,1,1,1,0,0,1,0; then s8 = 2*s8-1
    // → +1/-1 form. S8 const here is the original {0,1} form.
    let mut s = [0.0f32; 8];
    for i in 0..8 {
        s[i] = 2.0 * S8[i] as f32 - 1.0;
    }
    let mut cbq = [0.0f32; 42];
    let mut cbi = [0.0f32; 42];
    // Q channel (Fortran cbq):
    //   cbq(1:6)   = pp(7:12)*s8(1)   ← half-pulse for s8(1), Q half-symbol
    //   cbq(7:18)  = pp     *s8(3)
    //   cbq(19:30) = pp     *s8(5)
    //   cbq(31:42) = pp     *s8(7)
    for i in 0..6 {
        cbq[i] = pp[i + 6] * s[0];
    }
    for i in 0..12 {
        cbq[6 + i] = pp[i] * s[2];
    }
    for i in 0..12 {
        cbq[18 + i] = pp[i] * s[4];
    }
    for i in 0..12 {
        cbq[30 + i] = pp[i] * s[6];
    }
    // I channel:
    //   cbi(1:12)  = pp     *s8(2)
    //   cbi(13:24) = pp     *s8(4)
    //   cbi(25:36) = pp     *s8(6)
    //   cbi(37:42) = pp(1:6)*s8(8)   ← half-pulse for s8(8)
    for i in 0..12 {
        cbi[i] = pp[i] * s[1];
    }
    for i in 0..12 {
        cbi[12 + i] = pp[i] * s[3];
    }
    for i in 0..12 {
        cbi[24 + i] = pp[i] * s[5];
    }
    for i in 0..6 {
        cbi[36 + i] = pp[i] * s[7];
    }
    let mut cb = [Complex32::new(0.0, 0.0); 42];
    for i in 0..42 {
        cb[i] = Complex32::new(cbi[i], cbq[i]);
    }
    cb
}

/// Demodulate a sync-aligned NSPM-sample MSK144 frame to softbits.
///
/// Faithful 1:1 port of msk144decodeframe.f90 lines 48-82 (everything
/// before BP). Returns the demodulated softbits plus diagnostics. The
/// caller applies the n_bad_sync > 4 reject and runs BP.
///
/// The frame `c` is consumed as input; we work on a local copy for the
/// phase rotation.
pub fn demodulate_frame(c_in: &[Complex32; NSPM]) -> DemodulatedFrame {
    let cb = build_sync_waveform();
    let pp = half_sine_pulse();

    // Estimate carrier phase. Fortran:
    //   cca = sum(c(1:1+41) * conjg(cb))         ← 1st sync correlation
    //   ccb = sum(c(1+56*6 : 1+56*6+41) * conjg(cb))  ← 2nd sync correlation
    //   phase0 = atan2(imag(cca+ccb), real(cca+ccb))
    //
    // Fortran 1-indexed, so c(1:42) in Rust is c[0..42].
    // 1+56*6 = 337 in Fortran → c[336] in Rust. Then 42 elements: c[336..378].
    let mut cca = Complex32::new(0.0, 0.0);
    let mut ccb = Complex32::new(0.0, 0.0);
    for i in 0..42 {
        cca += c_in[i] * cb[i].conj();
        ccb += c_in[336 + i] * cb[i].conj();
    }
    let sum = cca + ccb;
    let phase0 = sum.im.atan2(sum.re);

    // Remove phase error. Fortran: cfac = cmplx(cos(phase0), sin(phase0)); c = c * conjg(cfac)
    // i.e. multiply by exp(-j*phase0).
    let cos_p = phase0.cos();
    let sin_p = phase0.sin();
    let cfac_conj = Complex32::new(cos_p, -sin_p);
    let mut c = [Complex32::new(0.0, 0.0); NSPM];
    for i in 0..NSPM {
        c[i] = c_in[i] * cfac_conj;
    }

    // Matched filter. Fortran (1-indexed indices):
    //   softbits(1) = sum(imag(c(1:6))*pp(7:12)) + sum(imag(c(864-5:864))*pp(1:6))
    //   softbits(2) = sum(real(c(1:12))*pp)
    //   do i=2..72:
    //     softbits(2*i-1) = sum(imag(c(1+(i-1)*12-6 : 1+(i-1)*12+5))*pp)
    //     softbits(2*i)   = sum(real(c(7+(i-1)*12-6 : 7+(i-1)*12+5))*pp)
    //
    // Note the half-symbol offset between I and Q in MSK:
    //   - Even-indexed softbits (1, 3, 5, ...) come from Q channel
    //   - Odd-indexed (2, 4, 6, ...) from I channel
    //   - softbits(1) wraps around the frame boundary because of the
    //     half-symbol offset of Q
    let mut softbits = [0.0f32; N_CHANSYM];

    // softbits(1): wrap-around Q. Fortran c(1:6) → c[0..6]. c(859:864) → c[858..864].
    let mut sum_a = 0.0f32;
    for k in 0..6 {
        sum_a += c[k].im * pp[6 + k];
    }
    let mut sum_b = 0.0f32;
    for k in 0..6 {
        sum_b += c[NSPM - 6 + k].im * pp[k];
    }
    softbits[0] = sum_a + sum_b;

    // softbits(2): I channel, c(1:12) → c[0..12]
    let mut s = 0.0f32;
    for k in 0..12 {
        s += c[k].re * pp[k];
    }
    softbits[1] = s;

    // Loop i=2..72 in Fortran → i=1..72 (Rust 0-indexed loops are i=1..72; we
    // use i=2..=72 to match the Fortran var name precisely).
    for i in 2..=72 {
        // Fortran: softbits(2*i-1) = sum(imag(c(1+(i-1)*12-6 : 1+(i-1)*12+5))*pp)
        //   index range: 1+(i-1)*12-6 .. 1+(i-1)*12+5 = (i-1)*12-5 .. (i-1)*12+6
        //   in Fortran 1-indexed = 12 elements
        //   convert to Rust 0-indexed: subtract 1 from both ends
        //   = (i-1)*12-6 .. (i-1)*12+6, i.e. 12 elements starting at (i-1)*12-6
        let start_q = (i - 1) * 12 - 6;
        let mut s = 0.0f32;
        for k in 0..12 {
            s += c[start_q + k].im * pp[k];
        }
        softbits[2 * i - 2] = s; // Fortran softbits(2*i-1) → Rust softbits[2*i-2]

        // Fortran: softbits(2*i) = sum(real(c(7+(i-1)*12-6 : 7+(i-1)*12+5))*pp)
        //   = real(c((i-1)*12+1 : (i-1)*12+12)) = 12 elements starting at (i-1)*12+1 (Fortran)
        //   = (i-1)*12 in Rust 0-indexed
        let start_i = (i - 1) * 12;
        let mut s = 0.0f32;
        for k in 0..12 {
            s += c[start_i + k].re * pp[k];
        }
        softbits[2 * i - 1] = s; // Fortran softbits(2*i) → Rust softbits[2*i-1]
    }

    // Sync error count. Fortran:
    //   hardbits = 1 if softbits >= 0 else 0
    //   nbadsync1 = (8 - sum((2*hardbits(1:8)-1) * s8)) / 2
    //   nbadsync2 = (8 - sum((2*hardbits(57:64)-1) * s8)) / 2
    //   nbadsync = nbadsync1 + nbadsync2
    //
    // s8 here is the +1/-1 form (after data statement transforms it).
    let mut s8_pm = [0i32; 8];
    for i in 0..8 {
        s8_pm[i] = 2 * S8[i] as i32 - 1;
    }
    let mut sum1 = 0i32;
    let mut sum2 = 0i32;
    for i in 0..8 {
        let hb1 = if softbits[i] >= 0.0 { 1 } else { 0 };
        let hb2 = if softbits[56 + i] >= 0.0 { 1 } else { 0 };
        sum1 += (2 * hb1 - 1) * s8_pm[i];
        sum2 += (2 * hb2 - 1) * s8_pm[i];
    }
    let nbadsync1 = (8 - sum1) / 2;
    let nbadsync2 = (8 - sum2) / 2;
    let n_bad_sync = (nbadsync1 + nbadsync2).max(0).min(16) as u8;

    DemodulatedFrame {
        softbits,
        n_bad_sync,
        phase0,
    }
}

/// Convert demodulated softbits to LLR for the (128,90) BP decoder.
///
/// Faithful port of msk144decodeframe.f90 lines 84-93:
///   sav  = mean(softbits)
///   s2av = mean(softbits²)
///   ssig = sqrt(s2av - sav²)        ← stddev of softbits
///   softbits = softbits / ssig
///   sigma = 0.60
///   llr(1:48)   = softbits(9:9+47)    ← skip 8 sync bits at front
///   llr(49:128) = softbits(65:65+80-1) ← skip 8 sync bits at midpoint
///   llr = 2.0 * llr / (sigma*sigma)
pub fn softbits_to_ldpc_llr(softbits: &[f32; N_CHANSYM]) -> [f32; 128] {
    let n = N_CHANSYM as f32;
    let sav: f32 = softbits.iter().sum::<f32>() / n;
    let s2av: f32 = softbits.iter().map(|x| x * x).sum::<f32>() / n;
    let var = (s2av - sav * sav).max(1e-12);
    let ssig = var.sqrt();
    let sigma = 0.60_f32;
    let scale = 2.0 / (sigma * sigma) / ssig;

    let mut llr = [0.0f32; 128];
    // Fortran llr(1:48) = softbits(9:9+47) → llr[0..48] = softbits[8..56]
    for i in 0..48 {
        llr[i] = softbits[8 + i] * scale;
    }
    // Fortran llr(49:128) = softbits(65:65+80-1) → llr[48..128] = softbits[64..144]
    for i in 0..80 {
        llr[48 + i] = softbits[64 + i] * scale;
    }
    llr
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pp_endpoints_zero_middle_max() {
        let pp = half_sine_pulse();
        // pp(1) = sin(0) = 0
        assert!(pp[0].abs() < 1e-6);
        // pp(12) = sin(11π/12) ≈ 0.2588
        // Actually pp[6] = sin(6π/12) = sin(π/2) = 1.0 is the peak
        assert!((pp[6] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn cb_has_42_samples() {
        let cb = build_sync_waveform();
        assert_eq!(cb.len(), 42);
    }

    #[test]
    fn cb_first_six_q_only() {
        // For symbols 0..6: Q channel uses pp[6..12] * s8[0]_signed
        // I channel is zero in samples 0..6 because cbi(1:12) starts at index 1 (Fortran)
        // No wait - cbi(1:12) covers samples 1..12 in Fortran = Rust 0..12.
        // So cbi[0] is non-zero. Let me re-verify by looking at the F90 output...
        // Actually, looking at Fortran:
        //   cbq(1:6)  = pp(7:12)*s8(1)   ← Q only at first 6
        //   cbi(1:12) = pp     *s8(2)    ← I starts from sample 1
        // So at samples 0..6: BOTH Q and I are non-zero (the half-sym offset
        // means Q is the second-half pulse of symbol 0 while I is the first
        // half of symbol 1, overlapping).
        let cb = build_sync_waveform();
        // s8 = [0,1,1,1,0,0,1,0] → s8_pm = [-1,1,1,1,-1,-1,1,-1]
        // cbq[0] = pp[6] * (-1) = -1.0
        let pp = half_sine_pulse();
        assert!((cb[0].im - (pp[6] * -1.0)).abs() < 1e-6);
        // cbi[0] = pp[0] * 1 = 0 (since pp[0]=0)
        assert!(cb[0].re.abs() < 1e-6);
    }

    #[test]
    fn demodulate_zero_frame_gives_high_sync_error() {
        // A zero-input frame: all softbits ~= 0, hard bits = 1 (since softbits >= 0
        // is true for 0.0). With s8 = [0,1,1,1,0,0,1,0] and hardbits = all 1, the
        // sync mismatch is at positions 0,4,5,7 → 4 errors per sync word, 8 total.
        let frame = [Complex32::new(0.0, 0.0); NSPM];
        let result = demodulate_frame(&frame);
        // For all-zero input, hardbits = [1,1,1,1,1,1,1,1] vs s8 = [0,1,1,1,0,0,1,0]
        // Differences at positions 0, 4, 5, 7 = 4 errors each × 2 sync words = 8 total
        assert_eq!(result.n_bad_sync, 8, "expected 8 sync errors for all-zero frame");
    }
}
