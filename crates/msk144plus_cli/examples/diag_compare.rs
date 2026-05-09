// Synthesize a known MSK40 signal at fc=1500, then run our decode pipeline
// step-by-step alongside what we'd expect at each stage. Then run the same
// pipeline against the unknown test WAV burst and compare characteristics.

use msk144plus_dsp::{
    analytic, build_channel_bits_msk40, generate_msk40_slot,
    AnalyticFilter, NFFT_ANALYTIC, NSPM_MSK40,
    detect_short_candidates, msk40_sync, demodulate_short_frame,
    short_softbits_to_llr,
};
use msk144plus_fec::{decode_short, encode_short};
use msk144plus_packjt::jenkins::{format_call_pair, hash12};
use num_complex::Complex32;

fn build_message(mycall: &str, hiscall: &str, rpt: usize) -> [u8; 16] {
    let pair = format_call_pair(mycall, hiscall);
    let ihash = hash12(&pair) as u32;
    let ig = 16 * ihash + rpt as u32;
    let mut message = [0u8; 16];
    for i in 0..16 {
        message[i] = ((ig >> i) & 1) as u8;
    }
    message
}

fn run_pipeline(label: &str, audio: &[f32], fc: f32, expected_hash: u16) {
    println!("\n=== {} ===", label);
    let nfft = NFFT_ANALYTIC;
    let filter = AnalyticFilter::new(nfft);
    let mut input = vec![0.0f32; nfft];
    let n_copy = nfft.min(audio.len());
    input[..n_copy].copy_from_slice(&audio[..n_copy]);
    let rms = (input.iter().map(|x| x * x).sum::<f32>() / nfft as f32).sqrt();
    if rms > 0.0 {
        let f = 1.0 / rms;
        for x in input.iter_mut() { *x *= f; }
    }
    let cdat = analytic(&input, &filter);

    // Use 8 frames worth starting after the initial transient
    let cbig: Vec<Complex32> = cdat[NSPM_MSK40..NSPM_MSK40 + 8 * NSPM_MSK40].to_vec();

    let cands = detect_short_candidates(&cbig, 100.0, fc);
    println!("MSK40 candidates: {}", cands.len());
    if cands.is_empty() {
        println!("  none found");
        return;
    }
    for (i, c) in cands.iter().take(3).enumerate() {
        println!("  [{}] n_start={} ferr={:.1} detmet={:.2}", i, c.n_start, c.freq_err, c.detmet);
    }

    let cand = &cands[0];
    let n = cbig.len();
    let n_start_0 = cand.n_start.saturating_sub(1);
    let mut ib = n_start_0.saturating_sub(NSPM_MSK40);
    let mut ie = ib + 3 * NSPM_MSK40;
    if ie > n {
        ie = n;
        if ie >= 3 * NSPM_MSK40 { ib = ie - 3 * NSPM_MSK40; } else { return; }
    }
    let window = &cbig[ib..ie];
    let fo = fc + cand.freq_err;

    // Use the all-1s pattern (sum all 3 frames)
    let result = msk40_sync(window, 3, 29.0, 7.2, &[1, 1, 1], 2, fo);
    println!("sync xmax={:.2} ferr={:.1} success={}", result.xmax, result.freq_offset, result.success);

    if !result.success {
        return;
    }

    let peak_loc = result.peak_locations[0];
    let mut ct = [Complex32::new(0.0, 0.0); NSPM_MSK40];
    for k in 0..NSPM_MSK40 { ct[k] = result.averaged_frame[(k + peak_loc) % NSPM_MSK40]; }
    let demod = demodulate_short_frame(&ct);
    println!("demod sync_err={}", demod.n_bad_sync);

    // Print first 40 softbits
    println!("first 8 softbits (sync, expected: S8R = [1,0,1,1,0,0,0,1]):");
    for i in 0..8 {
        let sign = if demod.softbits[i] >= 0.0 { '+' } else { '-' };
        println!("  sb[{}] = {:.3} ({})", i, demod.softbits[i], sign);
    }
    println!("payload softbits (8..40), magnitudes:");
    let avg_mag: f32 = demod.softbits[8..40].iter().map(|x| x.abs()).sum::<f32>() / 32.0;
    let max_mag: f32 = demod.softbits[8..40].iter().map(|x| x.abs()).fold(0.0f32, f32::max);
    let min_mag: f32 = demod.softbits[8..40].iter().map(|x| x.abs()).fold(f32::INFINITY, f32::min);
    println!("  avg|sb|={:.3} max|sb|={:.3} min|sb|={:.3}", avg_mag, max_mag, min_mag);

    let llr = short_softbits_to_llr(&demod.softbits, 0.0);
    let avg_llr: f32 = llr.iter().map(|x| x.abs()).sum::<f32>() / 32.0;
    println!("LLR avg|llr|={:.3}", avg_llr);

    match decode_short(&llr, 50) {  // try 50 iterations instead of 5
        Some(r) => {
            let mut imsg = 0u32;
            for k in 0..16 { imsg |= (r.message[k] as u32) << k; }
            let nrxrpt = (imsg & 0xF) as usize;
            let nrxhash = ((imsg >> 4) & 0xFFF) as u16;
            let m = if nrxhash == expected_hash { " *** MATCH ***" } else { "" };
            println!("BP CONVERGED: hash={} rpt={} (expected hash={}){}", nrxhash, nrxrpt, expected_hash, m);
        }
        None => println!("BP NO CONVERGE"),
    }
}

fn main() {
    // Reference: synthesize K1JT/PE1ITR/R+10 ourselves
    let pair = format_call_pair("K1JT", "PE1ITR");
    let expected_hash = hash12(&pair);
    println!("Expected hash for K1JT/PE1ITR: {} (0x{:x})", expected_hash, expected_hash);

    let msg = build_message("K1JT", "PE1ITR", 11);
    let cw = encode_short(&msg);
    let bits = build_channel_bits_msk40(&cw);
    let synth = generate_msk40_slot(&bits, 1500.0, 30);
    let synth_scaled: Vec<f32> = synth.iter().map(|s| s * 8000.0).collect();
    run_pipeline("SYNTH K1JT/PE1ITR R+10", &synth_scaled, 1500.0, expected_hash);

    // Now the actual WAV file - extract burst region
    let mut reader = hound::WavReader::open(
        "/home/claude/msk144plus_v2/test_samples/MSK144_60ms_pulse.wav"
    ).unwrap();
    let audio: Vec<f32> = reader.samples::<i16>().map(|s| s.unwrap() as f32).collect();
    // Burst is at 2900-2960ms = samples 34800-35520
    let extract_start = 34000;
    let extract_end = 41200.min(audio.len());
    let extract: Vec<f32> = audio[extract_start..extract_end].to_vec();
    run_pipeline("WAV 60ms_pulse extract (60ms burst)", &extract, 1500.0, expected_hash);

    // Also try the longer 140ms burst train
    let mut reader2 = hound::WavReader::open(
        "/home/claude/msk144plus_v2/test_samples/MSK144_20ms30ms_train_s1.wav"
    ).unwrap();
    let audio2: Vec<f32> = reader2.samples::<i16>().map(|s| s.unwrap() as f32).collect();
    // Burst at 1000-1140ms = samples 12000-13680
    // Take 600ms ending after burst
    let extract2_start = 11200;
    let extract2_end = 18400.min(audio2.len());
    let extract2: Vec<f32> = audio2[extract2_start..extract2_end].to_vec();
    run_pipeline("WAV 20ms30ms_train (140ms burst)", &extract2, 1500.0, expected_hash);

    // 40ms_pulse - shorter ping
    let mut reader3 = hound::WavReader::open(
        "/home/claude/msk144plus_v2/test_samples/MSK144_40ms_pulse.wav"
    ).unwrap();
    let audio3: Vec<f32> = reader3.samples::<i16>().map(|s| s.unwrap() as f32).collect();
    // Burst at 3040-3100ms = samples 36480-37200
    let extract3_start = 35600;
    let extract3_end = 42800.min(audio3.len());
    let extract3: Vec<f32> = audio3[extract3_start..extract3_end].to_vec();
    run_pipeline("WAV 40ms_pulse (40ms burst)", &extract3, 1500.0, expected_hash);
}
