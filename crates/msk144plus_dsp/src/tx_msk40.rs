// crates/msk144plus_dsp/src/tx_msk40.rs
//
// MSK40 transmit. Faithful port of genmsk40.f90.

use crate::constants::SAMPLE_RATE;
use crate::decode_frame::half_sine_pulse;
use crate::decode_frame_msk40::{NSPM_MSK40, N_CHANSYM_MSK40, S8R};
use std::f32::consts::PI;

/// Build the 40-bit channel sequence: 8-bit S8R sync + 32-bit codeword.
pub fn build_channel_bits_msk40(codeword: &[u8; 32]) -> [u8; N_CHANSYM_MSK40] {
    let mut bits = [0u8; N_CHANSYM_MSK40];
    bits[0..8].copy_from_slice(&S8R);
    bits[8..40].copy_from_slice(codeword);
    bits
}

/// Generate one 240-sample MSK40 frame at carrier `fc_hz`.
pub fn generate_msk40_frame(bits: &[u8; N_CHANSYM_MSK40], fc_hz: f32) -> [f32; NSPM_MSK40] {
    let pp = half_sine_pulse();
    let mut bitseq = [0.0f32; N_CHANSYM_MSK40];
    for i in 0..N_CHANSYM_MSK40 {
        bitseq[i] = 2.0 * bits[i] as f32 - 1.0;
    }

    // I and Q construction matches MSK144 pattern but with 40 channel bits
    // instead of 144.
    let mut xq = [0.0f32; NSPM_MSK40];
    let mut xi = [0.0f32; NSPM_MSK40];

    // First half-symbol on Q: samples 0..6 use pp[6..12] times bitseq[0]
    for k in 0..6 {
        xq[k] = bitseq[0] * pp[6 + k];
    }
    // Q middle: i = 1..20 covering samples (i-1)*12+6 .. (i-1)*12+18
    // Wait, careful here. NSPM_MSK40 = 240 = 20 symbols * 12 samples/symbol.
    // The pattern for MSK144 was: for i=1..71 (71 inner Q symbols), the i-th
    // Q symbol spans samples (i-1)*12+6 .. (i-1)*12+18.
    // The last half-symbol of Q at samples NSPM-6..NSPM uses bitseq[0] (wraps).
    //
    // For MSK40 with 20 symbols total: 19 inner Q symbols (i=1..19), each
    // spans (i-1)*12+6 .. (i-1)*12+18. The wrap-around happens at the end.
    for i in 1..=19 {
        let start = (i - 1) * 12 + 6;
        let bit_idx = 2 * i; // Q gets even-indexed bits (Fortran 1-based: 3, 5, 7, ...)
        for k in 0..12 {
            xq[start + k] = bitseq[bit_idx] * pp[k];
        }
    }
    // Last half-symbol of Q: samples NSPM-6..NSPM, using bitseq[0] with pp[0..6]
    for k in 0..6 {
        xq[NSPM_MSK40 - 6 + k] = bitseq[0] * pp[k];
    }

    // I channel: 20 full 12-sample symbols at sample 0, 12, 24, ..., 228
    for i in 1..=20 {
        let start = (i - 1) * 12;
        let bit_idx = 2 * i - 1;
        for k in 0..12 {
            xi[start + k] = bitseq[bit_idx] * pp[k];
        }
    }

    let mut audio = [0.0f32; NSPM_MSK40];
    let omega = 2.0 * PI * fc_hz / SAMPLE_RATE as f32;
    for n in 0..NSPM_MSK40 {
        let phase = omega * n as f32;
        audio[n] = xi[n] * phase.cos() - xq[n] * phase.sin();
    }
    audio
}

/// Generate a phase-continuous slot of MSK40 frames.
pub fn generate_msk40_slot(
    bits: &[u8; N_CHANSYM_MSK40],
    fc_hz: f32,
    n_frames: usize,
) -> Vec<f32> {
    let pp = half_sine_pulse();
    let mut bitseq = [0.0f32; N_CHANSYM_MSK40];
    for i in 0..N_CHANSYM_MSK40 {
        bitseq[i] = 2.0 * bits[i] as f32 - 1.0;
    }

    let mut xq = [0.0f32; NSPM_MSK40];
    let mut xi = [0.0f32; NSPM_MSK40];
    for k in 0..6 { xq[k] = bitseq[0] * pp[6 + k]; }
    for i in 1..=19 {
        let start = (i - 1) * 12 + 6;
        let bit_idx = 2 * i;
        for k in 0..12 { xq[start + k] = bitseq[bit_idx] * pp[k]; }
    }
    for k in 0..6 { xq[NSPM_MSK40 - 6 + k] = bitseq[0] * pp[k]; }
    for i in 1..=20 {
        let start = (i - 1) * 12;
        let bit_idx = 2 * i - 1;
        for k in 0..12 { xi[start + k] = bitseq[bit_idx] * pp[k]; }
    }

    let mut out = Vec::with_capacity(n_frames * NSPM_MSK40);
    let omega = 2.0 * PI * fc_hz / SAMPLE_RATE as f32;
    let mut sample_idx: u64 = 0;
    for _ in 0..n_frames {
        for n in 0..NSPM_MSK40 {
            let phase = omega * sample_idx as f32;
            out.push(xi[n] * phase.cos() - xq[n] * phase.sin());
            sample_idx += 1;
        }
    }
    out
}
