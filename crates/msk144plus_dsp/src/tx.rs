// crates/msk144plus_dsp/src/tx.rs
//
// MSK144 transmit-path baseband generation.
//
// Faithful port of WSJT-X genmsk_128_90.f90 (lines 90-115). Builds the
// 144-channel-bit sequence (8-bit sync + 48 bits + 8-bit sync + 80 bits)
// and converts to half-sine-shaped MSK at 12 kHz.

use crate::constants::{N_CHANSYM, NSPM, S8, SAMPLE_RATE};
use crate::decode_frame::half_sine_pulse;
use num_complex::Complex32;
use std::f32::consts::PI;

/// Build the 144-bit channel sequence from a 128-bit codeword.
///
/// Layout (matches genmsk_128_90.f90 lines 92-96):
///   bits[0..8]    = S8                 (sync word 1)
///   bits[8..56]   = codeword[0..48]
///   bits[56..64]  = S8                 (sync word 2)
///   bits[64..144] = codeword[48..128]
pub fn build_channel_bits(codeword: &[u8; 128]) -> [u8; N_CHANSYM] {
    let mut bits = [0u8; N_CHANSYM];
    bits[0..8].copy_from_slice(&S8);
    bits[8..56].copy_from_slice(&codeword[0..48]);
    bits[56..64].copy_from_slice(&S8);
    bits[64..144].copy_from_slice(&codeword[48..128]);
    bits
}

/// Generate one 864-sample MSK144 frame at the given carrier frequency.
///
/// Faithful port of genmsk_128_90.f90 lines 99-114. The bit-stream is mapped
/// to alternating Q (odd-indexed bits) and I (even-indexed bits) tributaries,
/// shaped by the half-sine pulse, then up-converted with a complex sinusoid
/// at fc.
///
/// `bits[0..144]` are the channel bits (sync + codeword) in 0/1 form.
/// Returns 864 real audio samples at 12 kHz.
pub fn generate_msk144_frame(bits: &[u8; N_CHANSYM], fc_hz: f32) -> [f32; NSPM] {
    let pp = half_sine_pulse();

    // Convert bits 0/1 to ±1.
    let mut bitseq = [0.0f32; N_CHANSYM];
    for i in 0..N_CHANSYM {
        bitseq[i] = 2.0 * bits[i] as f32 - 1.0;
    }

    // Build I and Q baseband. Fortran (1-indexed):
    //   xq(1:6)        = bitseq(1)*pp(7:12)  ← first half-symbol on Q
    //   do i=1..71:
    //     is = (i-1)*12 + 7
    //     xq(is:is+11) = bitseq(2*i+1)*pp
    //   xq(864-5:864)  = bitseq(1)*pp(1:6)   ← last half-symbol (Q wraps around!)
    //   do i=1..72:
    //     is = (i-1)*12 + 1
    //     xi(is:is+11) = bitseq(2*i)*pp
    //
    // Note that bitseq(1) on Q wraps from start to end (the half-symbol
    // offset means Q symbol 0 spans the frame boundary). The repeat-frame
    // structure of MSK144 (continuous transmission of repeated frames)
    // makes this wrap consistent.
    let mut xq = [0.0f32; NSPM];
    let mut xi = [0.0f32; NSPM];

    // First half-symbol on Q: samples 0..6 use pp[6..12] times bitseq[0]
    for k in 0..6 {
        xq[k] = bitseq[0] * pp[6 + k];
    }
    // Q middle: i = 1..72 in Fortran maps to i = 1..72 here. Fortran
    // is = (i-1)*12 + 7, span 12 samples → samples [is..is+11] (1-indexed)
    // → Rust 0-indexed: start = is-1 = (i-1)*12 + 6, span 12 → [start..start+12]
    // Fortran loops i=1..71, picking bitseq(2*i+1) which is Fortran-1-indexed
    // odd bits 3,5,...,143 → Rust bitseq[2,4,...,142]
    for i in 1..=71 {
        let start = (i - 1) * 12 + 6;
        let bit_idx = 2 * i; // Fortran 2*i+1 is 0-indexed Rust 2*i
        for k in 0..12 {
            xq[start + k] = bitseq[bit_idx] * pp[k];
        }
    }
    // Last half-symbol on Q: samples 858..864 use pp[0..6] times bitseq[0]
    for k in 0..6 {
        xq[NSPM - 6 + k] = bitseq[0] * pp[k];
    }

    // I channel: full 12-sample symbols at sample 0, 12, 24, ..., 852
    // Fortran: is = (i-1)*12 + 1, i=1..72, bitseq(2*i) (1-indexed) → Rust bitseq[2*i-1]
    for i in 1..=72 {
        let start = (i - 1) * 12;
        let bit_idx = 2 * i - 1;
        for k in 0..12 {
            xi[start + k] = bitseq[bit_idx] * pp[k];
        }
    }

    // Up-convert: real_audio[n] = xi[n]*cos(omega*n) - xq[n]*sin(omega*n)
    // Equivalent to: Re{(xi + j*xq) * exp(j*omega*n)}
    let mut audio = [0.0f32; NSPM];
    let omega = 2.0 * PI * fc_hz / SAMPLE_RATE as f32;
    for n in 0..NSPM {
        let phase = omega * n as f32;
        audio[n] = xi[n] * phase.cos() - xq[n] * phase.sin();
    }
    audio
}

/// Generate `n_frames` consecutive MSK144 frames with phase-continuous
/// carrier. Used by the engine to render a full slot of repeating message.
pub fn generate_msk144_slot(bits: &[u8; N_CHANSYM], fc_hz: f32, n_frames: usize) -> Vec<f32> {
    let pp = half_sine_pulse();
    let mut bitseq = [0.0f32; N_CHANSYM];
    for i in 0..N_CHANSYM {
        bitseq[i] = 2.0 * bits[i] as f32 - 1.0;
    }
    let mut xq = [0.0f32; NSPM];
    let mut xi = [0.0f32; NSPM];
    for k in 0..6 { xq[k] = bitseq[0] * pp[6 + k]; }
    for i in 1..=71 {
        let start = (i - 1) * 12 + 6;
        let bit_idx = 2 * i;
        for k in 0..12 { xq[start + k] = bitseq[bit_idx] * pp[k]; }
    }
    for k in 0..6 { xq[NSPM - 6 + k] = bitseq[0] * pp[k]; }
    for i in 1..=72 {
        let start = (i - 1) * 12;
        let bit_idx = 2 * i - 1;
        for k in 0..12 { xi[start + k] = bitseq[bit_idx] * pp[k]; }
    }

    // Up-convert with continuous phase across frames.
    let mut out = Vec::with_capacity(n_frames * NSPM);
    let omega = 2.0 * PI * fc_hz / SAMPLE_RATE as f32;
    let mut sample_idx: u64 = 0;
    for _ in 0..n_frames {
        for n in 0..NSPM {
            let phase = omega * sample_idx as f32;
            out.push(xi[n] * phase.cos() - xq[n] * phase.sin());
            sample_idx += 1;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::S8;

    #[test]
    fn channel_bits_have_sync_at_correct_positions() {
        let mut cw = [0u8; 128];
        for i in 0..128 { cw[i] = (i % 2) as u8; }
        let bits = build_channel_bits(&cw);
        // Sync at 0..8
        for i in 0..8 {
            assert_eq!(bits[i], S8[i]);
        }
        // Sync at 56..64
        for i in 0..8 {
            assert_eq!(bits[56 + i], S8[i]);
        }
        // Codeword[0..48] at positions 8..56
        for i in 0..48 {
            assert_eq!(bits[8 + i], cw[i]);
        }
        // Codeword[48..128] at positions 64..144
        for i in 0..80 {
            assert_eq!(bits[64 + i], cw[48 + i]);
        }
    }

    #[test]
    fn frame_has_correct_length() {
        let bits = [0u8; N_CHANSYM];
        let frame = generate_msk144_frame(&bits, 1500.0);
        assert_eq!(frame.len(), NSPM);
    }
}
