// crates/msk144plus_dsp/src/decode_frame_msk40.rs
//
// MSK40 frame demodulator. Faithful port of msk40decodeframe.f90 (DSP portion).

use crate::decode_frame::half_sine_pulse;
use num_complex::Complex32;

pub const NSPM_MSK40: usize = 240;
pub const N_CHANSYM_MSK40: usize = 40;
pub const S8R: [u8; 8] = [1, 0, 1, 1, 0, 0, 0, 1];

pub fn build_sync_waveform_msk40() -> [Complex32; 42] {
    let pp = half_sine_pulse();
    let mut s = [0.0f32; 8];
    for i in 0..8 { s[i] = 2.0 * S8R[i] as f32 - 1.0; }
    let mut cbq = [0.0f32; 42];
    let mut cbi = [0.0f32; 42];
    for i in 0..6 { cbq[i] = pp[i + 6] * s[0]; }
    for i in 0..12 { cbq[6 + i] = pp[i] * s[2]; }
    for i in 0..12 { cbq[18 + i] = pp[i] * s[4]; }
    for i in 0..12 { cbq[30 + i] = pp[i] * s[6]; }
    for i in 0..12 { cbi[i] = pp[i] * s[1]; }
    for i in 0..12 { cbi[12 + i] = pp[i] * s[3]; }
    for i in 0..12 { cbi[24 + i] = pp[i] * s[5]; }
    for i in 0..6 { cbi[36 + i] = pp[i] * s[7]; }
    let mut cb = [Complex32::new(0.0, 0.0); 42];
    for i in 0..42 { cb[i] = Complex32::new(cbi[i], cbq[i]); }
    cb
}

pub struct DemodulatedShortFrame {
    pub softbits: [f32; N_CHANSYM_MSK40],
    pub n_bad_sync: u8,
}

pub fn demodulate_short_frame(c_in: &[Complex32; NSPM_MSK40]) -> DemodulatedShortFrame {
    let cb = build_sync_waveform_msk40();
    let pp = half_sine_pulse();

    let mut cca = Complex32::new(0.0, 0.0);
    for i in 0..42 { cca += c_in[i] * cb[i].conj(); }
    let phase0 = cca.im.atan2(cca.re);

    let cfac_conj = Complex32::new(phase0.cos(), -phase0.sin());
    let mut c = [Complex32::new(0.0, 0.0); NSPM_MSK40];
    for i in 0..NSPM_MSK40 { c[i] = c_in[i] * cfac_conj; }

    let mut softbits = [0.0f32; N_CHANSYM_MSK40];

    let mut sum_a = 0.0f32;
    for k in 0..6 { sum_a += c[k].im * pp[6 + k]; }
    let mut sum_b = 0.0f32;
    for k in 0..6 { sum_b += c[NSPM_MSK40 - 6 + k].im * pp[k]; }
    softbits[0] = sum_a + sum_b;

    let mut s = 0.0f32;
    for k in 0..12 { s += c[k].re * pp[k]; }
    softbits[1] = s;

    for i in 2..=20 {
        let start_q = (i - 1) * 12 - 6;
        let mut sq = 0.0f32;
        for k in 0..12 { sq += c[start_q + k].im * pp[k]; }
        softbits[2 * i - 2] = sq;
        let start_i = (i - 1) * 12;
        let mut si = 0.0f32;
        for k in 0..12 { si += c[start_i + k].re * pp[k]; }
        softbits[2 * i - 1] = si;
    }

    let mut s8r_pm = [0i32; 8];
    for i in 0..8 { s8r_pm[i] = 2 * S8R[i] as i32 - 1; }
    let mut sum1 = 0i32;
    for i in 0..8 {
        let hb = if softbits[i] >= 0.0 { 1 } else { 0 };
        sum1 += (2 * hb - 1) * s8r_pm[i];
    }
    let nbadsync = ((8 - sum1) / 2).max(0).min(8) as u8;

    DemodulatedShortFrame { softbits, n_bad_sync: nbadsync }
}

pub fn short_softbits_to_llr(softbits: &[f32; N_CHANSYM_MSK40], xsnr_db: f32) -> [f32; 32] {
    let n = N_CHANSYM_MSK40 as f32;
    let sav: f32 = softbits.iter().sum::<f32>() / n;
    let s2av: f32 = softbits.iter().map(|x| x * x).sum::<f32>() / n;
    let var = (s2av - sav * sav).max(1e-12);
    let ssig = var.sqrt();
    let sigma = if xsnr_db < 0.0 { 0.75 - 0.11 * xsnr_db } else { 0.75 };
    let scale = 2.0 / (sigma * sigma) / ssig;
    let mut llr = [0.0f32; 32];
    for i in 0..32 { llr[i] = softbits[8 + i] * scale; }
    llr
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cb_msk40_first_sample() {
        let cb = build_sync_waveform_msk40();
        let pp = half_sine_pulse();
        assert!((cb[0].im - pp[6]).abs() < 1e-6);
        assert!(cb[0].re.abs() < 1e-6);
    }

    #[test]
    fn zero_frame_high_sync_error() {
        let frame = [Complex32::new(0.0, 0.0); NSPM_MSK40];
        let result = demodulate_short_frame(&frame);
        assert_eq!(result.n_bad_sync, 4);
    }
}
