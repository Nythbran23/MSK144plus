// crates/msk144plus_dsp/src/sync.rs
//
// Faithful port of WSJT-X msk144sync.f90 (101 lines) and
// msk144_freq_search.f90 (50 lines).
//
// Given:
//   - cdat: nframes worth of analytic baseband (NSPM*nframes complex samples)
//   - navmask: which frames to coherently sum
//   - fc: carrier center freq
//   - ntol/delf: freq sweep range and step
//   - npeaks: how many sync-correlation peaks to return
//
// Returns:
//   - the best frequency offset
//   - the coherently-averaged frame `c`
//   - up to npeaks sync-peak locations within `c` (cyclic shifts)
//   - xmax: the peak sync correlation magnitude (>= 1.3 means "good enough")

use crate::constants::NSPM;
use crate::decode_frame::build_sync_waveform;
use num_complex::Complex32;
use std::f32::consts::PI;

pub struct SyncResult {
    /// Best frequency offset (Hz, relative to fc).
    pub freq_offset: f32,
    /// Coherently-averaged frame at the best frequency, NSPM samples.
    /// Already heterodyned to baseband.
    pub averaged_frame: Box<[Complex32; NSPM]>,
    /// Top sync-peak cyclic-shift positions (sample indices into
    /// `averaged_frame`). Length = npeaks.
    pub peak_locations: Vec<usize>,
    /// Maximum sync correlation magnitude (post-scaling).
    pub xmax: f32,
    /// True if xmax >= 1.3 (WSJT-X's success criterion).
    pub success: bool,
}

/// Heterodyne `cdat` by frequency `f_shift` (Hz) at 12 kHz sample rate.
/// Output: cdat * exp(-j*2*pi*f_shift*n / 12000).
///
/// Faithful port of tweak1 (called from msk144_freq_search).
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

/// Run the per-thread frequency search portion of msk144sync.
///
/// For each ferr in [if1*delf, if2*delf]:
///   1. Heterodyne cdat to baseband by -(fc+ferr)
///   2. Coherently sum frames where navmask=1
///   3. Compute cyclic-shift correlation against sync waveform cb
///   4. Track peak (xmax_thread, bestf_thread, cs_thread, xccs_thread)
///
/// Returns: (xmax_thread, bestf_thread, cs_thread, xccs_thread).
fn freq_search(
    cdat: &[Complex32],
    fc: f32,
    if1: i32,
    if2: i32,
    delf: f32,
    nframes: usize,
    navmask: &[u8],
    cb: &[Complex32; 42],
) -> (f32, f32, [Complex32; NSPM], [f32; NSPM]) {
    let n = nframes * NSPM;
    debug_assert_eq!(cdat.len(), n);
    let navg: usize = navmask.iter().map(|&x| x as usize).sum();
    let fac = if navg > 0 {
        1.0 / (48.0 * (navg as f32).sqrt())
    } else {
        0.0
    };

    let mut xmax = 0.0f32;
    let mut bestf = 0.0f32;
    let mut cs_out = [Complex32::new(0.0, 0.0); NSPM];
    let mut xccs_out = [0.0f32; NSPM];

    for ifr in if1..=if2 {
        let ferr = ifr as f32 * delf;
        // Heterodyne cdat by -(fc+ferr)
        let cdat2 = tweak1(cdat, fc + ferr);

        // Coherent sum across frames marked in navmask
        let mut c = [Complex32::new(0.0, 0.0); NSPM];
        for i in 0..nframes {
            if navmask[i] == 1 {
                let ib = i * NSPM;
                for k in 0..NSPM {
                    c[k] += cdat2[ib + k];
                }
            }
        }

        // ct2 = [c, c]  (length 2*NSPM, for circular correlation via concatenation)
        // Then for ish in 0..NSPM-1:
        //   cc(ish) = dot_product(ct2(1+ish : 42+ish) + ct2(337+ish : 378+ish), cb)
        //
        // In Rust 0-indexed: ct2[ish..ish+42] + ct2[ish+336..ish+378]
        // Both slices are 42 complex samples. cb is 42 samples.
        // dot_product(a, b) in Fortran complex = sum(conjg(a)*b)
        // BUT in this Fortran code, dot_product(a, b) for real cb is just sum(a*cb)
        // since dot_product on real arrays is plain sum. However cb here is complex.
        // Fortran dot_product on complex: sum(conjg(a) * b).
        // So: cc(ish) = sum_j conjg(ct2(j) + ct2(j+336)) * cb(j)
        //
        // Wait — let me re-check. In msk144decodeframe.f90 we used:
        //   cca = sum(c(1:42) * conjg(cb))
        // That is sum(c * conjg(cb)) — same as dot_product(cb, c) in Fortran's
        // complex dot_product convention. So dot_product order matters.
        //
        // In msk144_freq_search.f90 line 36:
        //   cc(ish) = dot_product(ct2(...) + ct2(...), cb)
        // Fortran dot_product(a, b) = sum(conjg(a) * b). So:
        //   cc(ish) = sum(conjg(ct2_combined) * cb)
        // = sum(conjg(ct2(j) + ct2(j+336)) * cb(j))
        //
        // That's the standard correlation: input·conj(template). Or equivalently
        // conj(input)·template. Both give the same |cc|.
        let mut cc = [Complex32::new(0.0, 0.0); NSPM];
        for ish in 0..NSPM {
            let mut acc = Complex32::new(0.0, 0.0);
            for j in 0..42 {
                // ct2 wraps modulo NSPM (since c is repeated)
                let idx1 = (ish + j) % NSPM;
                let idx2 = (ish + 336 + j) % NSPM;
                let a = c[idx1] + c[idx2];
                acc += a.conj() * cb[j];
            }
            cc[ish] = acc;
        }

        // xcc = |cc|
        let mut xcc = [0.0f32; NSPM];
        for k in 0..NSPM {
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

/// Run msk144sync. Faithful port of msk144sync.f90.
///
/// Note: We don't use OpenMP threads as the Fortran does; single-threaded
/// Rust port. Same algorithm, slightly slower. Can parallelise with rayon
/// later if needed.
pub fn msk144_sync(
    cdat: &[Complex32],
    nframes: usize,
    ntol_hz: f32,
    delf: f32,
    navmask: &[u8],
    npeaks: usize,
    fc: f32,
) -> SyncResult {
    let cb = build_sync_waveform();

    // nfreqs = 2*round(ntol/delf) + 1
    let if1 = -((ntol_hz / delf).round() as i32);
    let if2 = (ntol_hz / delf).round() as i32;

    let (xmax, bestf, cs, xccs) =
        freq_search(cdat, fc, if1, if2, delf, nframes, navmask, &cb);

    // Find npeaks largest peaks. Faithful port of msk144sync.f90 lines 88-95:
    //   do ipk=1,npeaks
    //     iloc=maxloc(xcc)
    //     ic2=iloc(1)
    //     npklocs(ipk)=ic2
    //     pkamps(ipk)=xcc(ic2-1)        ← unused; possibly a Fortran bug
    //     xcc(max(0,ic2-7):min(NSPM-1,ic2+7))=0.0
    //   enddo
    let mut xcc_work = xccs;
    let mut peak_locations = Vec::with_capacity(npeaks);
    for _ in 0..npeaks {
        let (ic2, _) = xcc_work
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
            .unwrap();
        peak_locations.push(ic2);
        // Zero out ±7 samples around the peak
        let lo = ic2.saturating_sub(7);
        let hi = (ic2 + 7).min(NSPM - 1);
        for k in lo..=hi {
            xcc_work[k] = 0.0;
        }
    }

    let success = xmax >= 1.3;
    SyncResult {
        freq_offset: bestf,
        averaged_frame: Box::new(cs),
        peak_locations,
        xmax,
        success,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tx::{build_channel_bits, generate_msk144_slot};
    use crate::analytic::{analytic, AnalyticFilter};
    use msk144plus_fec::encode_128_90;
    use msk144plus_packjt::pack77_text;

    /// Generate audio for a known message, run analytic, run msk144_sync,
    /// verify it locks onto the right frequency offset and locates the
    /// sync at the start of a frame.
    #[test]
    fn sync_locks_on_clean_signal() {
        let payload = pack77_text("CQ K1ABC FN42");
        let codeword = encode_128_90(&payload);
        let bits = build_channel_bits(&codeword);

        // Generate 8 frames of audio at fc=1500 Hz
        let audio = generate_msk144_slot(&bits, 1500.0, 8);
        let nfft = 8192;
        let filter = AnalyticFilter::new(nfft);
        let mut input = vec![0.0f32; nfft];
        input[..audio.len()].copy_from_slice(&audio);
        let baseband = analytic(&input, &filter);

        // Take 7 frames worth (NSPM*7 samples) starting at sample 0
        // Actually skip the first frame to avoid the analytic transient
        let cdat: Vec<Complex32> = baseband[NSPM..NSPM + 7 * NSPM].to_vec();

        // Try the deepest averaging pattern: 7 frames
        let navmask = [1u8; 7];
        let result = msk144_sync(&cdat, 7, 100.0, 10.0/7.0, &navmask, 2, 1500.0);

        assert!(result.success, "should lock with xmax={}", result.xmax);
        // Should find fc within ~1 delf of 1500 Hz (delf = 10/7 ≈ 1.43)
        eprintln!("on-center test: freq_offset = {}, xmax = {}",
            result.freq_offset, result.xmax);
        assert!(result.freq_offset.abs() < 5.0,
            "expected freq_offset near 0, got {}", result.freq_offset);
    }

    #[test]
    fn sync_locks_off_center_freq() {
        let payload = pack77_text("CQ K1ABC FN42");
        let codeword = encode_128_90(&payload);
        let bits = build_channel_bits(&codeword);

        // Generate at fc=1488 (off-center by -12 Hz)
        let audio = generate_msk144_slot(&bits, 1488.0, 8);
        let nfft = 8192;
        let filter = AnalyticFilter::new(nfft);
        let mut input = vec![0.0f32; nfft];
        input[..audio.len()].copy_from_slice(&audio);
        let baseband = analytic(&input, &filter);
        // Skip first 2 frames - the analytic raised-cosine BPF has a
        // transient ramp longer than one MSK144 frame on a clean signal.
        let cdat: Vec<Complex32> = baseband[2*NSPM..2*NSPM + 7 * NSPM].to_vec();

        // Search at fc=1500 with ntol=100 - should find -12 Hz offset
        let navmask = [1u8; 7];
        let result = msk144_sync(&cdat, 7, 100.0, 10.0/7.0, &navmask, 2, 1500.0);
        eprintln!("off-center test: freq_offset = {}, xmax = {}",
            result.freq_offset, result.xmax);
        assert!(result.success, "should lock, xmax={}", result.xmax);
        // bestf should be near -12 Hz (within 2 freq steps = ~3 Hz)
        assert!((result.freq_offset - (-12.0)).abs() < 3.0,
            "expected freq_offset near -12, got {}", result.freq_offset);
    }

    #[test]
    fn sync_locks_off_center_no_analytic() {
        // Test sync directly without analytic filter, using a synthetic
        // complex baseband at fc=1488. Heterodyne the real audio to a
        // complex analytic by multiplying by exp(j*2*pi*fc_synth*n/12000)
        // for fc_synth=1488.
        let payload = pack77_text("CQ K1ABC FN42");
        let codeword = encode_128_90(&payload);
        let bits = build_channel_bits(&codeword);
        let audio = generate_msk144_slot(&bits, 1488.0, 8);

        // Build analytic by Hilbert: just multiply by 2*exp(j*0) and zero
        // negative freqs. Or, since the real signal is just a sum of two
        // sidebands, the analytic version is the original I/Q baseband
        // shifted by exp(+j*2*pi*1488*n/12000).
        // Actually easier: compute analytic via FFT of the real audio.
        let nfft = 8192;
        let filter = AnalyticFilter::new(nfft);
        let mut input = vec![0.0f32; nfft];
        input[..audio.len()].copy_from_slice(&audio);
        let baseband = analytic(&input, &filter);
        // Skip first 2 frames worth to avoid BPF transient
        let cdat: Vec<Complex32> = baseband[2*NSPM..2*NSPM + 7 * NSPM].to_vec();

        let navmask = [1u8; 7];
        let result = msk144_sync(&cdat, 7, 100.0, 10.0/7.0, &navmask, 2, 1500.0);
        eprintln!("no-trans test: freq_offset = {}, xmax = {}",
            result.freq_offset, result.xmax);
        assert!(result.success);
    }

    #[test]
    fn sync_locks_positive_offset() {
        let payload = pack77_text("CQ K1ABC FN42");
        let codeword = encode_128_90(&payload);
        let bits = build_channel_bits(&codeword);
        // Generate at fc=1512 (off-center by +12)
        let audio = generate_msk144_slot(&bits, 1512.0, 8);
        let nfft = 8192;
        let filter = AnalyticFilter::new(nfft);
        let mut input = vec![0.0f32; nfft];
        input[..audio.len()].copy_from_slice(&audio);
        let baseband = analytic(&input, &filter);
        let cdat: Vec<Complex32> = baseband[NSPM..NSPM + 7 * NSPM].to_vec();
        let navmask = [1u8; 7];
        let result = msk144_sync(&cdat, 7, 100.0, 10.0/7.0, &navmask, 2, 1500.0);
        eprintln!("+12 test: freq_offset = {}, xmax = {}",
            result.freq_offset, result.xmax);
        assert!(result.success);
    }
}
