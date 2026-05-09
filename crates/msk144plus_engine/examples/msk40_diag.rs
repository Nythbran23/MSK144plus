// crates/msk144plus_engine/examples/msk40_diag.rs
//
// Diagnostic tool: for each synthetic test file, run the analytic filter
// then dump what MSK40 spd front-end finds. Helps identify whether the
// detector is firing at all, and at what freq_err.

use msk144plus_dsp::{
    analytic, detect_short_candidates, msk40_sync, demodulate_short_frame,
    short_softbits_to_llr, AnalyticFilter, NSPM_MSK40, NFFT_ANALYTIC, NZ_MSKRTD,
};
use msk144plus_fec::{decode_short, encode_short};
use msk144plus_packjt::{
    jenkins::{format_call_pair, hash12},
};
use num_complex::Complex32;
use std::env;

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 4 {
        eprintln!("Usage: msk40_diag <wav> <mycall> <hiscall>");
        std::process::exit(1);
    }
    let path = &args[1];
    let mycall = &args[2];
    let hiscall = &args[3];

    let mut reader = hound::WavReader::open(path).unwrap();
    let audio: Vec<f32> = reader.samples::<i16>().map(|s| s.unwrap() as f32).collect();
    println!("loaded {} samples ({:.2}s)", audio.len(), audio.len() as f32 / 12000.0);

    let pair = format_call_pair(mycall, hiscall);
    let expected_hash = hash12(&pair);
    println!("expected hash for '{}' = {} (0x{:X})", pair, expected_hash, expected_hash);

    let nz = NZ_MSKRTD;
    let filter = AnalyticFilter::new(NFFT_ANALYTIC);

    // Slide window through the slot
    let step = nz / 2;
    let mut k_end = nz;
    let mut window_idx = 0;
    while k_end <= audio.len() {
        let raw = &audio[k_end - nz..k_end];
        let sum_sq: f32 = raw.iter().map(|x| x * x).sum();
        let rms = (sum_sq / nz as f32).sqrt();
        if rms < 1.0 {
            k_end += step;
            window_idx += 1;
            continue;
        }
        let fac = 1.0 / rms;
        let mut d = vec![0.0f32; NFFT_ANALYTIC];
        for i in 0..nz {
            d[i] = fac * raw[i];
        }
        let cdat = analytic(&d, &filter);

        // 8 frames worth of analytic
        let np = 8 * 864;
        if cdat.len() < np {
            k_end += step;
            window_idx += 1;
            continue;
        }
        let cbig = &cdat[..np];

        // Run MSK40 detector
        let cands = detect_short_candidates(cbig, 100.0, 1500.0);
        if !cands.is_empty() {
            println!(
                "  window {} (t={:.2}s, rms={:.0}): {} MSK40 candidates",
                window_idx,
                (k_end - nz) as f32 / 12000.0,
                rms,
                cands.len()
            );
            for (i, c) in cands.iter().enumerate() {
                println!(
                    "    [{}] n_start={} ferr={:+.1} detmet={:.2} detmet2={:.2} primary={}",
                    i, c.n_start, c.freq_err, c.detmet, c.detmet2, c.primary
                );
                // Try a sync on this candidate
                let n_start_0 = c.n_start.saturating_sub(1);
                let mut ib = n_start_0.saturating_sub(NSPM_MSK40);
                let mut ie = ib + 3 * NSPM_MSK40;
                if ie > cbig.len() {
                    ie = cbig.len();
                    if ie >= 3 * NSPM_MSK40 { ib = ie - 3 * NSPM_MSK40; } else { continue; }
                }
                let win = &cbig[ib..ie];
                let fo = 1500.0 + c.freq_err;
                // Try the 7-frame averaging mask (most aggressive: all 3)
                let navmasks: [[u8; 3]; 6] = [
                    [0, 1, 0], [1, 0, 0], [0, 0, 1],
                    [1, 1, 0], [0, 1, 1], [1, 1, 1],
                ];
                for (pi, navmask) in navmasks.iter().enumerate() {
                    let result = msk40_sync(win, 3, 29.0, 7.2, navmask, 2, fo);
                    if !result.success {
                        continue;
                    }
                    println!(
                        "      pat{}: sync xmax={:.2} ferr={:+.1} peaks={:?}",
                        pi, result.xmax, result.freq_offset, result.peak_locations
                    );

                    // Try decode at peak with no dither AND with dither
                    for &peak_loc in &result.peak_locations {
                        for is in 0..3 {
                            let dither = match is { 0 => 0i32, 1 => -1, _ => 1 };
                            let ic0 = ((peak_loc as i32 + dither).max(0).min(NSPM_MSK40 as i32 - 1)) as usize;
                            let mut ct = [Complex32::new(0.0, 0.0); NSPM_MSK40];
                            for k in 0..NSPM_MSK40 {
                                ct[k] = result.averaged_frame[(k + ic0) % NSPM_MSK40];
                            }
                            let demod = demodulate_short_frame(&ct);
                            if demod.n_bad_sync > 3 {
                                continue;
                            }
                            let llr = short_softbits_to_llr(&demod.softbits, c.snr_db);
                            if let Some(r) = decode_short(&llr, 5) {
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
                                let mut imsg = 0u32;
                                for k in 0..16 {
                                    imsg = (imsg << 1) | r.message[k] as u32;
                                }
                                let nrxrpt = (imsg & 0xF) as usize;
                                let nrxhash = ((imsg >> 4) & 0xFFF) as u16;
                                let match_str = if nrxhash == expected_hash { "MATCH" } else { "miss" };
                                println!(
                                    "        peak={} dith={} nbs={} bp_iter={} nhammd={} cord={:.2} nrxhash={} ({}) nrxrpt={}",
                                    peak_loc, dither, demod.n_bad_sync, r.iterations, nhammd, cord, nrxhash, match_str, nrxrpt
                                );
                            } else {
                                println!(
                                    "        peak={} dith={} nbs={} BP did not converge",
                                    peak_loc, dither, demod.n_bad_sync
                                );
                            }
                        }
                    }
                }
            }
        }

        k_end += step;
        window_idx += 1;
    }
}
