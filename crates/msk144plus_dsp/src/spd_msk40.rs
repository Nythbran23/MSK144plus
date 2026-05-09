// crates/msk144plus_dsp/src/spd_msk40.rs
//
// MSK40 sync correlator + ping detector. Faithful port of:
//   - msk40sync.f90       (sync correlation across freq sweep)
//   - msk40_freq_search.f90 (per-freq cyclic-shift correlation)
//   - msk40spd.f90 lines 64-153 (front-end candidate finder)

use crate::decode_frame_msk40::{build_sync_waveform_msk40, NSPM_MSK40};
use crate::spd::raised_cosine_edge_window;
use num_complex::Complex32;
use rustfft::FftPlanner;
use std::f32::consts::PI;

pub const STEP_SAMPLES_MSK40: usize = 60; // 5 ms steps (Fortran: 60)
pub const NFFT_MSK40: usize = NSPM_MSK40; // 240
pub const MAXSTEPS_MSK40: usize = 150;
pub const MAXCAND_MSK40: usize = 5;

pub struct ShortSyncResult {
    pub freq_offset: f32,
    pub averaged_frame: Box<[Complex32; NSPM_MSK40]>,
    pub peak_locations: Vec<usize>,
    pub xmax: f32,
    pub success: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct ShortSpdCandidate {
    pub n_start: usize,
    pub freq_err: f32,
    pub snr_db: f32,
    pub detmet: f32,
    pub detmet2: f32,
    pub primary: bool,
}

fn tweak1(cdat: &[Complex32], f_shift: f32) -> Vec<Complex32> {
    let omega = -2.0 * PI * f_shift / 12000.0;
    let mut out = Vec::with_capacity(cdat.len());
    for (n, &x) in cdat.iter().enumerate() {
        let phase = omega * n as f32;
        let rot = Complex32::new(phase.cos(), phase.sin());
        out.push(x * rot);
    }
    out
}

/// MSK40 freq+shift search. Faithful port of msk40_freq_search.f90.
///
/// Differences from MSK144 freq_search:
///   - NSPM=240 (not 864)
///   - cc(ish) uses ONLY the first sync correlation (ct2[ish..ish+42]),
///     not the second sync at ish+336 (MSK40 has only one sync word per frame)
///   - fac = 1.0 / (24.0 * sqrt(navg))   (vs 48.0 for MSK144)
fn freq_search_msk40(
    cdat: &[Complex32],
    fc: f32,
    if1: i32,
    if2: i32,
    delf: f32,
    nframes: usize,
    navmask: &[u8],
    cb: &[Complex32; 42],
) -> (f32, f32, [Complex32; NSPM_MSK40], [f32; NSPM_MSK40]) {
    let n = nframes * NSPM_MSK40;
    debug_assert_eq!(cdat.len(), n);
    let navg: usize = navmask.iter().map(|&x| x as usize).sum();
    let fac = if navg > 0 {
        1.0 / (24.0 * (navg as f32).sqrt())
    } else {
        0.0
    };

    let mut xmax = 0.0f32;
    let mut bestf = 0.0f32;
    let mut cs_out = [Complex32::new(0.0, 0.0); NSPM_MSK40];
    let mut xccs_out = [0.0f32; NSPM_MSK40];

    for ifr in if1..=if2 {
        let ferr = ifr as f32 * delf;
        let cdat2 = tweak1(cdat, fc + ferr);

        let mut c = [Complex32::new(0.0, 0.0); NSPM_MSK40];
        for i in 0..nframes {
            if navmask[i] == 1 {
                let ib = i * NSPM_MSK40;
                for k in 0..NSPM_MSK40 {
                    c[k] += cdat2[ib + k];
                }
            }
        }

        // cc(ish) = dot_product(ct2[ish..ish+42], cb)
        // Fortran complex dot_product: sum(conjg(a) * b) = sum(conjg(c[k]) * cb[k])
        let mut cc = [Complex32::new(0.0, 0.0); NSPM_MSK40];
        for ish in 0..NSPM_MSK40 {
            let mut acc = Complex32::new(0.0, 0.0);
            for j in 0..42 {
                let idx = (ish + j) % NSPM_MSK40;
                acc += c[idx].conj() * cb[j];
            }
            cc[ish] = acc;
        }

        let mut xcc = [0.0f32; NSPM_MSK40];
        for k in 0..NSPM_MSK40 {
            xcc[k] = cc[k].norm();
        }
        let xb = xcc.iter().fold(0.0f32, |a, &b| a.max(b)) * fac;
        if xb > xmax {
            xmax = xb;
            bestf = ferr;
            cs_out = c;
            xccs_out = xcc;
        }
    }
    (xmax, bestf, cs_out, xccs_out)
}

/// MSK40 sync orchestrator. Faithful port of msk40sync.f90.
pub fn msk40_sync(
    cdat: &[Complex32],
    nframes: usize,
    ntol_hz: f32,
    delf: f32,
    navmask: &[u8],
    npeaks: usize,
    fc: f32,
) -> ShortSyncResult {
    let cb = build_sync_waveform_msk40();
    let if1 = -((ntol_hz / delf).round() as i32);
    let if2 = (ntol_hz / delf).round() as i32;
    let (xmax, bestf, cs, xccs) =
        freq_search_msk40(cdat, fc, if1, if2, delf, nframes, navmask, &cb);

    let mut xcc_work = xccs;
    let mut peak_locations = Vec::with_capacity(npeaks);
    for _ in 0..npeaks {
        let (ic2, _) = xcc_work
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
            .unwrap();
        peak_locations.push(ic2);
        let lo = ic2.saturating_sub(7);
        let hi = (ic2 + 7).min(NSPM_MSK40 - 1);
        for k in lo..=hi {
            xcc_work[k] = 0.0;
        }
    }

    let success = xmax >= 1.3;
    ShortSyncResult {
        freq_offset: bestf,
        averaged_frame: Box::new(cs),
        peak_locations,
        xmax,
        success,
    }
}

/// MSK40 short-ping front-end. Faithful port of msk40spd.f90 lines 64-153.
pub fn detect_short_candidates(
    cbig: &[Complex32],
    ntol_hz: f32,
    fc_hz: f32,
) -> Vec<ShortSpdCandidate> {
    let n = cbig.len();
    if n < NSPM_MSK40 {
        return Vec::new();
    }
    let nstep = ((n - NSPM_MSK40) / STEP_SAMPLES_MSK40).min(MAXSTEPS_MSK40);
    if nstep == 0 {
        return Vec::new();
    }
    let fs = 12000.0_f32;
    let df = fs / NFFT_MSK40 as f32;

    let nfhi = 2.0 * (fc_hz + 500.0);
    let nflo = 2.0 * (fc_hz - 500.0);
    let ihlo = ((nfhi - 2.0 * ntol_hz) / df).round() as i32;
    let ihhi = ((nfhi + 2.0 * ntol_hz) / df).round() as i32;
    let illo = ((nflo - 2.0 * ntol_hz) / df).round() as i32;
    let ilhi = ((nflo + 2.0 * ntol_hz) / df).round() as i32;
    let i2000 = (nflo / df).round() as i32;
    let i4000 = (nfhi / df).round() as i32;

    let rcw = raised_cosine_edge_window();
    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(NFFT_MSK40);

    let mut detmet = vec![0.0f32; nstep];
    let mut detmet2 = vec![0.0f32; nstep];
    let mut detfer = vec![-999.99f32; nstep];

    let mut ctmp = vec![Complex32::new(0.0, 0.0); NFFT_MSK40];

    for istp in 0..nstep {
        let ns = STEP_SAMPLES_MSK40 * istp;
        let ne = ns + NSPM_MSK40;
        if ne > n {
            break;
        }
        for i in 0..NFFT_MSK40 {
            ctmp[i] = Complex32::new(0.0, 0.0);
        }
        for i in 0..NSPM_MSK40 {
            let v = cbig[ns + i];
            ctmp[i] = v * v;
        }
        for i in 0..12 {
            ctmp[i].re *= rcw[i];
            ctmp[i].im *= rcw[i];
        }
        for i in 0..12 {
            ctmp[NSPM_MSK40 - 12 + i].re *= rcw[11 - i];
            ctmp[NSPM_MSK40 - 12 + i].im *= rcw[11 - i];
        }

        fft.process(&mut ctmp);
        let tonespec: Vec<f32> = ctmp.iter().map(|c| c.norm_sqr()).collect();

        let ihlo_c = ihlo.max(1).min(NFFT_MSK40 as i32 - 2) as usize;
        let ihhi_c = ihhi.max(1).min(NFFT_MSK40 as i32 - 2) as usize;
        let illo_c = illo.max(1).min(NFFT_MSK40 as i32 - 2) as usize;
        let ilhi_c = ilhi.max(1).min(NFFT_MSK40 as i32 - 2) as usize;

        let (ihpk, &ah) = (ihlo_c..=ihhi_c)
            .map(|k| (k, &tonespec[k]))
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
            .unwrap_or((ihlo_c, &0.0));
        let count_h = ihhi_c - ihlo_c + 1;
        let sum_h: f32 = tonespec[ihlo_c..=ihhi_c].iter().sum();
        let ahavp = (sum_h - ah) / count_h.max(1) as f32;
        let trath = ah / (ahavp + 0.01);
        let denom_h = ctmp[ihpk] * Complex32::new(2.0, 0.0) - ctmp[ihpk - 1] - ctmp[ihpk + 1];
        let num_h = ctmp[ihpk - 1] - ctmp[ihpk + 1];
        let deltah = if denom_h.norm_sqr() > 1e-30 {
            -(num_h / denom_h).re
        } else {
            0.0
        };

        let (ilpk, &al) = (illo_c..=ilhi_c)
            .map(|k| (k, &tonespec[k]))
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
            .unwrap_or((illo_c, &0.0));
        let count_l = ilhi_c - illo_c + 1;
        let sum_l: f32 = tonespec[illo_c..=ilhi_c].iter().sum();
        let alavp = (sum_l - al) / count_l.max(1) as f32;
        let tratl = al / (alavp + 0.01);
        let denom_l = ctmp[ilpk] * Complex32::new(2.0, 0.0) - ctmp[ilpk - 1] - ctmp[ilpk + 1];
        let num_l = ctmp[ilpk - 1] - ctmp[ilpk + 1];
        let deltal = if denom_l.norm_sqr() > 1e-30 {
            -(num_l / denom_l).re
        } else {
            0.0
        };

        let ferrh = (ihpk as f32 + deltah - i4000 as f32) * df / 2.0;
        let ferrl = (ilpk as f32 + deltal - i2000 as f32) * df / 2.0;
        let ferr = if ah >= al { ferrh } else { ferrl };
        detmet[istp] = ah.max(al);
        detmet2[istp] = trath.max(tratl);
        detfer[istp] = ferr;
    }

    // Median normalisation
    let mut sorted = detmet.clone();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let xmed = sorted[nstep / 4].max(1e-12);
    for v in detmet.iter_mut() {
        *v /= xmed;
    }

    // Primary candidate search: detmet >= 3.5 (note: stricter than MSK144's 3.0)
    let mut detmet_work = detmet.clone();
    let mut candidates = Vec::new();
    for _ in 0..MAXCAND_MSK40 {
        let (il, &dm) = detmet_work
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
            .unwrap();
        if dm < 3.5 {
            break;
        }
        if detfer[il].abs() <= ntol_hz {
            let n_start = il * STEP_SAMPLES_MSK40 + 1;
            let snr = 12.0 * (dm.log10()) / 2.0 - 9.0;
            candidates.push(ShortSpdCandidate {
                n_start,
                freq_err: detfer[il],
                snr_db: snr,
                detmet: dm,
                detmet2: detmet2[il],
                primary: true,
            });
        }
        detmet_work[il] = 0.0;
    }

    // Fallback: detmet2 >= 12.0
    if candidates.len() < 3 {
        let mut detmet2_work = detmet2.clone();
        for cand in &candidates {
            let il = (cand.n_start - 1) / STEP_SAMPLES_MSK40;
            if il < detmet2_work.len() {
                detmet2_work[il] = 0.0;
            }
        }
        let need = MAXCAND_MSK40 - candidates.len();
        for _ in 0..need {
            let (il, &dm2) = detmet2_work
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
                .unwrap();
            if dm2 < 12.0 {
                break;
            }
            if detfer[il].abs() <= ntol_hz {
                let n_start = il * STEP_SAMPLES_MSK40 + 1;
                let snr = 12.0 * (dm2.log10()) / 2.0 - 9.0;
                candidates.push(ShortSpdCandidate {
                    n_start,
                    freq_err: detfer[il],
                    snr_db: snr,
                    detmet: detmet[il],
                    detmet2: dm2,
                    primary: false,
                });
            }
            detmet2_work[il] = 0.0;
        }
    }

    candidates
}
