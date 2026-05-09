// crates/msk144plus_engine/tests/msk40_round_trip.rs
//
// MSK40 end-to-end test: encode "<K1JT PE1ITR> R+10" to audio, run the
// full RX pipeline including hash check, verify the decode matches.

use msk144plus_dsp::{
    analytic, demodulate_short_frame, generate_msk40_slot, build_channel_bits_msk40,
    short_softbits_to_llr, AnalyticFilter, NSPM_MSK40,
};
use msk144plus_fec::{decode_short, encode_short};
use msk144plus_packjt::jenkins::{format_call_pair, hash12};
use num_complex::Complex32;

#[test]
fn msk40_tx_rx_round_trip() {
    let mycall = "K1JT";
    let hiscall = "PE1ITR";
    // R+10 is rpt index 11 (msk40decodeframe.f90 RPT table).
    let rpt_idx: u16 = 11;

    // Build the 16-bit message: 12-bit hash + 4-bit report
    let pair = format_call_pair(mycall, hiscall);
    let h = hash12(&pair); // 12-bit hash
    let ig: u16 = (h << 4) | rpt_idx;

    // Convert to 16-bit array MSB-first (Fortran genmsk40.f90 line 39-41:
    //   message(i) = iand(1, ishft(ig, 1-i))   for i=1..16
    //   This is bit (i-1) when shifted right by (i-1), but Fortran ishft
    //   with negative: ishft(ig, 1-i) = ishft(ig, -(i-1)) = ig >> (i-1)
    //   So message(i) = bit i-1 of ig (LSB first ordering).
    // Wait - Fortran 1-indexed loop i=1..16: message(1)=bit 0, message(16)=bit 15.
    // That's LSB-first.
    let mut message = [0u8; 16];
    for i in 0..16 {
        message[i] = ((ig >> i) & 1) as u8;
    }
    let codeword = encode_short(&message);
    let bits = build_channel_bits_msk40(&codeword);

    // Generate audio: 50 frames of MSK40 = 50 * 20ms = 1 second of clean signal
    let fc = 1500.0;
    let audio = generate_msk40_slot(&bits, fc, 50);

    // Run analytic
    let nfft = 16384;
    let filter = AnalyticFilter::new(nfft);
    let mut input = vec![0.0f32; nfft];
    input[..audio.len().min(nfft)].copy_from_slice(&audio[..audio.len().min(nfft)]);
    let baseband = analytic(&input, &filter);

    // Heterodyne to fc=0
    let mut bb = vec![Complex32::new(0.0, 0.0); audio.len()];
    let omega = 2.0 * std::f32::consts::PI * fc / 12000.0;
    for n in 0..audio.len() {
        let phase = omega * n as f32;
        let rot = Complex32::new(phase.cos(), -phase.sin());
        bb[n] = baseband[n] * rot;
    }

    // Try to decode at frame-aligned offsets, skipping the analytic transient.
    let mut found = false;
    for frame_start in (3 * NSPM_MSK40..bb.len() - NSPM_MSK40).step_by(NSPM_MSK40) {
        let mut frame = [Complex32::new(0.0, 0.0); NSPM_MSK40];
        for i in 0..NSPM_MSK40 {
            frame[i] = bb[frame_start + i];
        }
        let demod = demodulate_short_frame(&frame);
        if demod.n_bad_sync > 3 {
            continue;
        }
        let llr = short_softbits_to_llr(&demod.softbits, 10.0);
        if let Some(r) = decode_short(&llr, 5) {
            assert_eq!(r.message, message, "BP returned wrong message at frame {}", frame_start);
            // Decode the message bits back to (hash, rpt)
            // Reverse the LSB-first packing from above
            let mut imsg: u16 = 0;
            for i in 0..16 {
                imsg |= (r.message[i] as u16) << i;
            }
            let nrxrpt = imsg & 0xF;
            let nrxhash = (imsg >> 4) & 0xFFF;
            assert_eq!(nrxhash, h, "hash mismatch");
            assert_eq!(nrxrpt, rpt_idx, "rpt mismatch");
            found = true;
            break;
        }
    }
    assert!(found, "should decode the synthetic MSK40 signal");
}
