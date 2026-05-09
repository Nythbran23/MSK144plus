// crates/msk144plus_engine/tests/tx_rx_round_trip.rs
//
// End-to-end test: encode a message, render to audio, run analytic+demod+BP,
// verify the message comes out the other end. This is the first integration
// milestone for the v2 port — confirms TX path, analytic, demod, LLR mapping,
// and BP all work together correctly.

use msk144plus_dsp::{
    analytic, build_channel_bits, demodulate_frame, generate_msk144_slot,
    softbits_to_ldpc_llr, AnalyticFilter, NSPM,
};
use msk144plus_fec::{decode_128_90_soft, encode_128_90};
use msk144plus_packjt::{pack77_text, unpack77, Message};
use num_complex::Complex32;

fn encode_to_audio(text: &str, fc_hz: f32, n_frames: usize) -> Vec<f32> {
    let payload = pack77_text(text);
    let codeword = encode_128_90(&payload);
    let channel_bits = build_channel_bits(&codeword);
    generate_msk144_slot(&channel_bits, fc_hz, n_frames)
}

fn try_decode_aligned(audio: &[f32], fc_hz: f32) -> Option<String> {
    // Run analytic filter over a NFFT1=8192 chunk of audio
    let nfft = 8192;
    let filter = AnalyticFilter::new(nfft);
    // Use the first 6048 samples (7 frames) + zero pad
    let n_input = audio.len().min(nfft);
    let mut input = vec![0.0f32; nfft];
    input[..n_input].copy_from_slice(&audio[..n_input]);
    let baseband = analytic(&input, &filter);

    // The analytic output is at the original carrier frequency; we need to
    // heterodyne to fc=0. Multiply by exp(-j*2*pi*fc*n/12000).
    let mut bb_at_fc = vec![Complex32::new(0.0, 0.0); n_input];
    let omega = 2.0 * std::f32::consts::PI * fc_hz / 12000.0;
    for n in 0..n_input {
        let phase = omega * n as f32;
        let rot = Complex32::new(phase.cos(), -phase.sin());
        bb_at_fc[n] = baseband[n] * rot;
    }

    // Try demod at frame-aligned offsets: 0, NSPM, 2*NSPM, ...
    // The TX starts at sample 0, so the first frame is at samples 0..NSPM.
    // But the analytic filter introduces some transient ramp; skip the first frame.
    for frame_start in (NSPM..bb_at_fc.len() - NSPM).step_by(NSPM) {
        let mut frame = [Complex32::new(0.0, 0.0); NSPM];
        for i in 0..NSPM {
            frame[i] = bb_at_fc[frame_start + i];
        }
        let demod = demodulate_frame(&frame);
        if demod.n_bad_sync > 4 { continue; }
        let llr = softbits_to_ldpc_llr(&demod.softbits);
        if let Some(r) = decode_128_90_soft(&llr, 10) {
            if r.n_hard_errors < 18 {
                let msg = unpack77(&r.message);
                let text = msg.to_text();
                return Some(text);
            }
        }
    }
    None
}

#[test]
fn cq_k1abc_round_trip() {
    let audio = encode_to_audio("CQ K1ABC FN42", 1500.0, 30);
    let result = try_decode_aligned(&audio, 1500.0);
    assert_eq!(result.as_deref(), Some("CQ K1ABC FN42"));
}

#[test]
fn k1abc_w9xyz_grid_round_trip() {
    let audio = encode_to_audio("K1ABC W9XYZ EN37", 1500.0, 30);
    let result = try_decode_aligned(&audio, 1500.0);
    assert_eq!(result.as_deref(), Some("K1ABC W9XYZ EN37"));
}

#[test]
fn report_round_trip() {
    let audio = encode_to_audio("K1ABC W9XYZ -10", 1500.0, 30);
    let result = try_decode_aligned(&audio, 1500.0);
    assert_eq!(result.as_deref(), Some("K1ABC W9XYZ -10"));
}

#[test]
fn rrr_round_trip() {
    let audio = encode_to_audio("K1ABC W9XYZ RRR", 1500.0, 30);
    let result = try_decode_aligned(&audio, 1500.0);
    assert_eq!(result.as_deref(), Some("K1ABC W9XYZ RRR"));
}

#[test]
fn cq_at_off_center_freq() {
    let audio = encode_to_audio("CQ K1ABC FN42", 1488.0, 30);
    let result = try_decode_aligned(&audio, 1488.0);
    assert_eq!(result.as_deref(), Some("CQ K1ABC FN42"));
}
