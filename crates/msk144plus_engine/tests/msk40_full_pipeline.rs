// crates/msk144plus_engine/tests/msk40_full_pipeline.rs
//
// MSK40 end-to-end through decode_slot with ShortMessageConfig.
// Confirms the full mskrtd pipeline (RMS check → analytic → spd MSK144 →
// spd MSK40 with hash check → averaging patterns) handles a synthesised
// MSK40 short-message correctly.
//
// All test cases simulate the PARTNER's transmission: the on-air call
// pair is "<senders_call our_call>" and the receiver (us) is configured
// with mycall = our_call, hiscall = senders_call. This matches the only
// direction the decoder accepts (self-direction is filtered out as
// monitor/echo noise — see encode_msk40_self_direction_is_rejected in
// the engine lib's encode_tests).

use msk144plus_dsp::{build_channel_bits_msk40, generate_msk40_slot};
use msk144plus_engine::{decode_slot, Depth, ShortMessageConfig};
use msk144plus_fec::encode_short;
use msk144plus_packjt::jenkins::{format_call_pair, hash12};

/// Build the 16-bit message that the SENDER transmits.
/// `sender_call` and `receiver_call` together form the "<sender receiver>"
/// hash key — the receiver's run_msk40_decode computes
/// hash("<HISCALL MYCALL>") = hash("<sender_call receiver_call>") and
/// matches against this.
fn build_message(sender_call: &str, receiver_call: &str, rpt: usize) -> [u8; 16] {
    let pair = format_call_pair(sender_call, receiver_call);
    let ihash = hash12(&pair) as u32;
    let ig = 16 * ihash + rpt as u32;
    let mut message = [0u8; 16];
    for i in 0..16 {
        message[i] = ((ig >> i) & 1) as u8;
    }
    message
}

fn make_audio(sender_call: &str, receiver_call: &str, rpt: usize, fc: f32, n_frames: usize) -> Vec<f32> {
    let msg = build_message(sender_call, receiver_call, rpt);
    let codeword = encode_short(&msg);
    let bits = build_channel_bits_msk40(&codeword);
    let raw = generate_msk40_slot(&bits, fc, n_frames);
    // Scale up to int16-like range (decode_slot expects signal RMS >= 1.0 and
    // the audio is normalised internally; but decode_slot needs a strong
    // enough signal to pass the rms gate)
    raw.iter().map(|s| s * 8000.0).collect()
}

#[test]
fn msk40_pipeline_k1jt_pe1itr_r_plus_10() {
    // Scenario: WE are PE1ITR. K1JT (our QSO partner) is transmitting
    // "<K1JT PE1ITR> R+10". So the on-air sender is K1JT, the on-air
    // receiver (us) is PE1ITR.
    let audio = make_audio("K1JT", "PE1ITR", 11, 1500.0, 60); // 60 frames = 1.2s
    // Pad with silence to exceed NZ_MSKRTD = 7168 samples
    let mut slot: Vec<f32> = vec![0.0f32; 12000];
    slot.extend_from_slice(&audio);
    slot.extend(vec![0.0f32; 12000]);

    let cfg = ShortMessageConfig {
        // We are PE1ITR; partner is K1JT.
        mycall: "PE1ITR".to_string(),
        hiscall: "K1JT".to_string(),
        enabled: true,
    };
    let events = decode_slot(&slot, 100.0, 1500.0, Depth::Deep, Some(&cfg));
    eprintln!("events ({}):", events.len());
    for e in &events {
        eprintln!("  '{}' method={} ferr={:.1} xmax={:.2}",
            e.text, e.method, e.freq_offset, e.xmax);
    }
    assert!(
        events.iter().any(|e| e.text.contains("R+10")),
        "expected 'R+10' in events; got: {:?}",
        events.iter().map(|e| e.text.clone()).collect::<Vec<_>>()
    );
}

#[test]
fn msk40_pipeline_w1aw_k1jt_73() {
    // Scenario: WE are K1JT. W1AW (our QSO partner) is transmitting
    // "<W1AW K1JT> 73".
    let audio = make_audio("W1AW", "K1JT", 15, 1500.0, 60);
    let mut slot: Vec<f32> = vec![0.0f32; 12000];
    slot.extend_from_slice(&audio);
    slot.extend(vec![0.0f32; 12000]);

    let cfg = ShortMessageConfig {
        // We are K1JT; partner is W1AW.
        mycall: "K1JT".to_string(),
        hiscall: "W1AW".to_string(),
        enabled: true,
    };
    let events = decode_slot(&slot, 100.0, 1500.0, Depth::Deep, Some(&cfg));
    eprintln!("events ({}):", events.len());
    for e in &events {
        eprintln!("  '{}' method={}", e.text, e.method);
    }
    assert!(
        events.iter().any(|e| e.text.contains("73")),
        "expected '73' in events"
    );
}
