use msk144plus_dsp::{
    analytic, AnalyticFilter, NFFT_ANALYTIC, NSPM_MSK40,
    detect_short_candidates, msk40_sync, demodulate_short_frame,
    short_softbits_to_llr,
};
use msk144plus_fec::{decode_short, encode_short};
use msk144plus_packjt::jenkins::{format_call_pair, hash12};
use num_complex::Complex32;

fn main() {
    // Read WAV
    let mut reader = hound::WavReader::open(
        "/home/claude/msk144plus_v2/test_samples/MSK144_60ms_pulse.wav"
    ).unwrap();
    let audio: Vec<f32> = reader.samples::<i16>().map(|s| s.unwrap() as f32).collect();
    println!("Audio: {} samples, {:.2} sec", audio.len(), audio.len() as f32 / 12000.0);

    // Burst is at 2900..2960ms = samples 34800..35520
    let burst_start_sample: usize = 34800;
    let burst_end_sample: usize = 35520;

    // Run analytic filter on a NFFT-sized chunk centered on the burst
    let center = (burst_start_sample + burst_end_sample) / 2;
    let chunk_start: usize = (center as usize).saturating_sub(NFFT_ANALYTIC / 2);
    let mut input = vec![0.0f32; NFFT_ANALYTIC];
    let n_copy = NFFT_ANALYTIC.min(audio.len() - chunk_start);
    input[..n_copy].copy_from_slice(&audio[chunk_start..chunk_start + n_copy]);
    let rms = (input.iter().map(|x| x * x).sum::<f32>() / NFFT_ANALYTIC as f32).sqrt();
    println!("Input RMS = {}", rms);
    if rms > 0.0 {
        let fac = 1.0 / rms;
        for x in input.iter_mut() { *x *= fac; }
    }

    let filter = AnalyticFilter::new(NFFT_ANALYTIC);
    let cdat = analytic(&input, &filter);

    // Use about 4 frames worth of MSK40 (4*240 = 960 samples) starting from
    // where the burst is in cdat.
    let burst_in_cdat_start: usize = (burst_start_sample as usize).saturating_sub(chunk_start);
    let cbig_start = burst_in_cdat_start.saturating_sub(NSPM_MSK40);
    let cbig_end = (cbig_start + 8 * NSPM_MSK40).min(cdat.len());
    let cbig = &cdat[cbig_start..cbig_end];
    println!("cbig: {} samples = {} MSK40 frames", cbig.len(), cbig.len() / NSPM_MSK40);

    println!("\nThe burst is at sample {} within cbig (= burst_start_sample - chunk_start - cbig_start = {} - {} - {} = {})",
        burst_in_cdat_start - cbig_start,
        burst_start_sample, chunk_start, cbig_start, burst_in_cdat_start - cbig_start);

    // Run MSK40 candidate detection
    let cands = detect_short_candidates(cbig, 100.0, 1500.0);
    println!("\nMSK40 candidates: {}", cands.len());
    for (i, c) in cands.iter().enumerate() {
        println!("  [{}] n_start={} ferr={:.1} detmet={:.2} detmet2={:.2} primary={}",
            i, c.n_start, c.freq_err, c.detmet, c.detmet2, c.primary);
    }

    // For each candidate, try to decode
    let pair = format_call_pair("K1JT", "PE1ITR");
    let expected_hash = hash12(&pair);
    println!("\nexpected hash for K1JT/PE1ITR: {} (0x{:x})", expected_hash, expected_hash);

    let navpatterns: [[u8; 3]; 6] = [
        [0,1,0], [1,0,0], [0,0,1], [1,1,0], [0,1,1], [1,1,1],
    ];

    for (ci, cand) in cands.iter().enumerate() {
        println!("\n=== Candidate [{}] ===", ci);
        let n = cbig.len();
        let n_start_0 = cand.n_start.saturating_sub(1);
        let mut ib = n_start_0.saturating_sub(NSPM_MSK40);
        let mut ie = ib + 3 * NSPM_MSK40;
        if ie > n {
            ie = n;
            if ie >= 3 * NSPM_MSK40 { ib = ie - 3 * NSPM_MSK40; } else { continue; }
        }
        let window = &cbig[ib..ie];
        let fo = 1500.0 + cand.freq_err;
        let xsnr = cand.snr_db;
        for (pi, navmask) in navpatterns.iter().enumerate() {
            let result = msk40_sync(window, 3, 29.0, 7.2, navmask, 2, fo);
            if !result.success {
                println!("  pat{} no sync (xmax={:.2})", pi, result.xmax);
                continue;
            }
            println!("  pat{} sync xmax={:.2} ferr={:.1}", pi, result.xmax, result.freq_offset);
            for (pk_idx, &peak_loc) in result.peak_locations.iter().enumerate() {
                for is in 0..3 {
                    let dither = match is { 0 => 0i32, 1 => -1, _ => 1 };
                    let ic0 = ((peak_loc as i32 + dither).max(0).min(NSPM_MSK40 as i32 - 1)) as usize;
                    let mut ct = [Complex32::new(0.0, 0.0); NSPM_MSK40];
                    for k in 0..NSPM_MSK40 { ct[k] = result.averaged_frame[(k + ic0) % NSPM_MSK40]; }
                    let demod = demodulate_short_frame(&ct);
                    if demod.n_bad_sync > 3 {
                        println!("    peak{} dither{} sync_err={} (>3, reject)", pk_idx, dither, demod.n_bad_sync);
                        continue;
                    }
                    let llr = short_softbits_to_llr(&demod.softbits, xsnr);
                    match decode_short(&llr, 5) {
                        Some(r) => {
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
                            // Try LSB-first interpretation (WSJT-X 3.0 standard)
                            let mut imsg_lsb = 0u32;
                            for k in 0..16 { imsg_lsb |= (r.message[k] as u32) << k; }
                            let nrxrpt_lsb = (imsg_lsb & 0xF) as usize;
                            let nrxhash_lsb = ((imsg_lsb >> 4) & 0xFFF) as u16;
                            // Try MSB-first interpretation (older / alternate)
                            let mut imsg_msb = 0u32;
                            for k in 0..16 { imsg_msb = (imsg_msb << 1) | r.message[k] as u32; }
                            let nrxrpt_msb = (imsg_msb & 0xF) as usize;
                            let nrxhash_msb = ((imsg_msb >> 4) & 0xFFF) as u16;
                            let match_lsb = nrxhash_lsb == expected_hash;
                            let match_msb = nrxhash_msb == expected_hash;
                            let m = if match_lsb { " *** LSB MATCH ***" }
                                else if match_msb { " *** MSB MATCH ***" } else { "" };
                            println!("    peak{} dither{} BP-conv: nhammd={} cord={:.2} | LSB(h={},rpt={}) MSB(h={},rpt={}){}",
                                pk_idx, dither, nhammd, cord,
                                nrxhash_lsb, nrxrpt_lsb, nrxhash_msb, nrxrpt_msb, m);
                        }
                        None => {
                            println!("    peak{} dither{} sync_err={} BP NO converge",
                                pk_idx, dither, demod.n_bad_sync);
                        }
                    }
                }
            }
        }
    }
}
