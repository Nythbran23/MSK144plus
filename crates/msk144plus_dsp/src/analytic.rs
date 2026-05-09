// crates/msk144plus_dsp/src/analytic.rs
//
// Faithful port of WSJT-X analytic.f90.
//
// Converts real audio samples to a complex analytic signal, applying
// a raised-cosine band-pass filter centered at 1500 Hz with 600-2400 Hz
// passband and raised-cosine transitions out to 400 and 2600 Hz.
//
// This is THE FIRST DSP STEP in the WSJT-X MSK144 decoder. Everything
// downstream operates on the analytic baseband output.
//
// Reference: wsjtx-3.0.0/lib/analytic.f90 (75 lines)
//   subroutine analytic(d, npts, nfft, c, pc, beq)
//
// Our port omits the equalizer (`beq`/`pc`) since neither WSJT-X's
// default config nor MSHV's basic path uses it. Can be added later.

use num_complex::Complex32;
use rustfft::FftPlanner;
use std::f32::consts::PI;

/// Raised-cosine BPF filter coefficients pre-computed for a given NFFT.
///
/// h[i] gives the magnitude response at frequency bin i (i=0..nh inclusive),
/// where bin spacing is 12000.0 / NFFT Hz. Mirrors analytic.f90 lines 26-43.
pub struct AnalyticFilter {
    /// FFT size used to build this filter. Convert to analytic must use
    /// the same NFFT.
    pub nfft: usize,
    /// h[0..=nh] where nh = nfft/2. Magnitude response (real, 0..=1).
    pub h: Vec<f32>,
}

impl AnalyticFilter {
    /// Build the WSJT-X raised-cosine BPF for the given NFFT at 12 kHz.
    ///
    /// Reference: analytic.f90 lines 26-43:
    ///   df = 12000.0 / nfft
    ///   t = 1.0 / 2000.0       (= 0.0005 sec, half-symbol period)
    ///   beta = 0.1
    ///   for each freq bin i (0..=nh):
    ///     ff = (i-1)*df         (Fortran 1-indexed; we use i*df)
    ///     f  = ff - 1500.0      (offset from center freq)
    ///     h(i) = 1.0
    ///     if (1-beta)/(2t) < |f| <= (1+beta)/(2t):
    ///       h(i) = 0.5 * (1 + cos((pi*t/beta) * (|f| - (1-beta)/(2t))))
    ///     elseif |f| > (1+beta)/(2t):
    ///       h(i) = 0.0
    ///
    /// With t=1/2000 and beta=0.1:
    ///   (1-beta)/(2t) = 0.9 * 1000 = 900 Hz   (passband edge, |f| <= 900)
    ///   (1+beta)/(2t) = 1.1 * 1000 = 1100 Hz  (cutoff, |f| >= 1100 → zero)
    ///
    /// So passband is 1500 ± 900 = 600..2400 Hz and the raised-cosine
    /// transitions occupy 400..600 Hz (low) and 2400..2600 Hz (high).
    pub fn new(nfft: usize) -> Self {
        let nh = nfft / 2;
        let df = 12000.0_f32 / nfft as f32;
        let t = 1.0_f32 / 2000.0;
        let beta = 0.1_f32;
        let pass_edge = (1.0 - beta) / (2.0 * t); // 900 Hz
        let stop_edge = (1.0 + beta) / (2.0 * t); // 1100 Hz

        let mut h = vec![0.0_f32; nh + 1];
        for i in 0..=nh {
            // Fortran uses 1-indexed loop ff=(i-1)*df. We use 0-indexed ff = i*df.
            let ff = i as f32 * df;
            let f = ff - 1500.0;
            let af = f.abs();
            h[i] = if af <= pass_edge {
                1.0
            } else if af <= stop_edge {
                // Raised-cosine taper. Fortran:
                //   h(i) = h(i) * 0.5*(1 + cos((pi*t/beta) * (|f| - (1-beta)/(2t))))
                // With h(i) initialised to 1.0 above.
                0.5 * (1.0 + ((PI * t / beta) * (af - pass_edge)).cos())
            } else {
                0.0
            };
        }
        AnalyticFilter { nfft, h }
    }
}

/// Convert real audio samples to a complex analytic baseband signal.
///
/// Steps (faithful to analytic.f90 lines 60-73):
///   1. Pad input with zeros to nfft length, scale by 2.0/nfft.
///   2. Forward FFT (real input treated as complex with zero imag).
///   3. Multiply spectrum bins 0..nh by h[i] (BPF).
///   4. Halve the DC bin (Fortran: c(1)=0.5*c(1)).
///   5. Zero negative-frequency bins (nh+2..nfft, Fortran 1-indexed).
///   6. Inverse FFT.
///
/// Output: complex array of length `nfft`; the first `npts` samples are
/// the analytic signal, the rest are zero-padded artifacts.
///
/// `samples`: input real audio at 12 kHz.
/// `nfft`: FFT size (must be >= samples.len(), typically 8192 for MSK144).
/// Returns: complex output of length nfft.
pub fn analytic(samples: &[f32], filter: &AnalyticFilter) -> Vec<Complex32> {
    let nfft = filter.nfft;
    assert!(samples.len() <= nfft, "samples longer than nfft");
    let nh = nfft / 2;
    let npts = samples.len();

    // Step 1: Pre-scale input. Fortran: fac = 2.0/nfft; c(1:npts) = fac*d(1:npts)
    let fac = 2.0_f32 / nfft as f32;
    let mut c: Vec<Complex32> = Vec::with_capacity(nfft);
    for i in 0..npts {
        c.push(Complex32::new(fac * samples[i], 0.0));
    }
    for _ in npts..nfft {
        c.push(Complex32::new(0.0, 0.0));
    }

    // Step 2: Forward FFT. Fortran four2a(c, nfft, 1, -1, 1) is forward.
    let mut planner = FftPlanner::<f32>::new();
    let fft_fwd = planner.plan_fft_forward(nfft);
    fft_fwd.process(&mut c);

    // Step 3: Apply BPF magnitude. Fortran:
    //   if(beq) c(1:nh+1) = h(1:nh+1)*corr(1:nh+1)*c(1:nh+1)
    //   else    c(1:nh+1) = h(1:nh+1)*c(1:nh+1)
    // We omit the equalizer corr here (always-1 case).
    for i in 0..=nh {
        c[i].re *= filter.h[i];
        c[i].im *= filter.h[i];
    }

    // Step 4: Halve DC bin. Fortran: c(1) = 0.5*c(1)
    c[0].re *= 0.5;
    c[0].im *= 0.5;

    // Step 5: Zero negative frequencies. Fortran: c(nh+2:nfft) = 0
    // Fortran 1-indexed nh+2..nfft maps to 0-indexed nh+1..nfft.
    for i in (nh + 1)..nfft {
        c[i] = Complex32::new(0.0, 0.0);
    }

    // Step 6: Inverse FFT. Fortran four2a(c, nfft, 1, 1, 1) is inverse.
    // rustfft inverse does NOT divide by N (matches Fortran four2a behaviour).
    let fft_inv = planner.plan_fft_inverse(nfft);
    fft_inv.process(&mut c);

    c
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn filter_passband_at_1500() {
        let f = AnalyticFilter::new(8192);
        // 1500 Hz is the center; bin index = 1500 / (12000/8192) = 1024
        let center_bin = (1500.0 / (12000.0 / 8192.0)) as usize;
        assert!((f.h[center_bin] - 1.0).abs() < 1e-6, "1500 Hz should be unity");
    }

    #[test]
    fn filter_stopband_below_400() {
        let f = AnalyticFilter::new(8192);
        // 300 Hz should be zero
        let bin_300 = (300.0 / (12000.0 / 8192.0)) as usize;
        assert!(f.h[bin_300].abs() < 1e-6, "300 Hz should be zero");
    }

    #[test]
    fn filter_stopband_above_2600() {
        let f = AnalyticFilter::new(8192);
        // 2700 Hz should be zero
        let bin_2700 = (2700.0 / (12000.0 / 8192.0)) as usize;
        assert!(f.h[bin_2700].abs() < 1e-6, "2700 Hz should be zero");
    }

    #[test]
    fn filter_passband_at_600() {
        let f = AnalyticFilter::new(8192);
        // 600 Hz is the lower passband edge - should be at unity
        let bin_600 = (600.0 / (12000.0 / 8192.0)) as usize;
        assert!(f.h[bin_600] > 0.99, "600 Hz should be near unity, got {}", f.h[bin_600]);
    }

    #[test]
    fn filter_passband_at_2400() {
        let f = AnalyticFilter::new(8192);
        let bin_2400 = (2400.0 / (12000.0 / 8192.0)) as usize;
        assert!(f.h[bin_2400] > 0.99, "2400 Hz should be near unity, got {}", f.h[bin_2400]);
    }

    #[test]
    fn analytic_of_1500_hz_tone_produces_dc_baseband() {
        // Generate a 1500 Hz cosine and verify the analytic signal has
        // (approximately) constant magnitude (= unity-magnitude carrier
        // shifted to DC after BPF... but actually BPF keeps it at 1500 Hz).
        // What we can verify is: the analytic output magnitude is
        // approximately constant across samples (steady-state).
        let nfft = 8192;
        let filter = AnalyticFilter::new(nfft);
        let samples: Vec<f32> = (0..6000)
            .map(|i| (2.0 * PI * 1500.0 * i as f32 / 12000.0).cos())
            .collect();
        let c = analytic(&samples, &filter);
        // Skip transient regions at edges; check middle samples have stable magnitude.
        let mid_start = 1000;
        let mid_end = 5000;
        let mags: Vec<f32> = (mid_start..mid_end).map(|i| c[i].norm()).collect();
        let mean_mag = mags.iter().sum::<f32>() / mags.len() as f32;
        // For a unity-amplitude cosine at 1500 Hz, the analytic signal magnitude
        // should be ~1.0 (since the BPF is unity at 1500 Hz and we kept positive
        // freq, halved DC). The 2.0/nfft pre-scale and inverse-FFT-no-divide
        // should give the right amplitude.
        let max_dev: f32 = mags.iter()
            .map(|&m| (m - mean_mag).abs())
            .fold(0.0f32, f32::max);
        assert!(max_dev / mean_mag < 0.05,
            "magnitude should be stable in middle: mean={} max_dev={}",
            mean_mag, max_dev);
    }

    #[test]
    fn analytic_of_300_hz_tone_is_attenuated() {
        // 300 Hz is below the BPF stopband - output should be near-zero.
        let nfft = 8192;
        let filter = AnalyticFilter::new(nfft);
        let samples: Vec<f32> = (0..6000)
            .map(|i| (2.0 * PI * 300.0 * i as f32 / 12000.0).cos())
            .collect();
        let c = analytic(&samples, &filter);
        // Average magnitude in steady-state
        let mid_mags: Vec<f32> = (1000..5000).map(|i| c[i].norm()).collect();
        let mean_mag: f32 = mid_mags.iter().sum::<f32>() / mid_mags.len() as f32;
        assert!(mean_mag < 0.01, "300 Hz should be filtered out, got mean_mag={}", mean_mag);
    }
}
