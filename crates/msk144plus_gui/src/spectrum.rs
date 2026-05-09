// crates/msk144plus_gui/src/spectrum.rs
//
// Realtime audio spectrum display. Ported from FSK441Plus's spectrum.rs
// with the following adaptations:
//
// * Sample rate is 12000 Hz here (msk144plus_v2's audio pipeline up-
//   resamples to 12 kHz before delivering chunks to the decoder thread)
//   rather than FSK441's 11025 Hz. Bin spacing therefore differs.
//
// * No tone marker lines (per Roger's request — keep the panel as
//   minimal as possible).
//
// * Display range 0..3 kHz. Bin count derived to match.
//
// The spectrum is computed by a dedicated worker thread fed via a
// fan-out clone of the audio capture sender, so it doesn't compete
// with the decoder for time. Each ~1024-sample chunk produces one
// vertical column. A 15-second slot fills ~176 columns at 12 kHz
// (15 * 12000 / 1024 ≈ 175.78); a 30-second slot fills ~352. The UI
// layout self-calibrates from the largest slot it has actually
// observed, so both periods render edge-to-edge.

use rustfft::FftPlanner;
use num_complex::Complex32;

/// FFT size for the per-chunk spectrum. 1024 samples at 12 kHz =
/// 85 ms per column. ~176 columns per 15s slot; ~352 per 30s slot.
pub const FFT_SIZE: usize = 1024;

/// Sample rate of the audio chunks fed in. Matches the decoder
/// pipeline. Bin spacing is sample_rate / FFT_SIZE = 11.7 Hz.
pub const SAMPLE_RATE: f32 = 12000.0;

/// Number of bins to display = bins covering 0..3 kHz.
/// 3000 / (12000/1024) = 256 bins.
pub const DISPLAY_BINS: usize = 256;

/// Compute one column of spectrum magnitudes (normalised 0..1) from an audio chunk.
///
/// Applies a Hann window, FFT, magnitude in dB, then auto-scales by
/// finding the median (= noise floor) and using a 25 dB dynamic range
/// above it. Returns DISPLAY_BINS values from DC up to ~3 kHz.
pub fn compute_column(samples: &[f32], planner: &mut FftPlanner<f32>) -> Vec<f32> {
    let fft = planner.plan_fft_forward(FFT_SIZE);
    let n = samples.len().min(FFT_SIZE);

    let mut buf: Vec<Complex32> = (0..FFT_SIZE)
        .map(|i| {
            if i < n {
                let w = 0.5 * (1.0 - (2.0 * std::f32::consts::PI * i as f32
                    / (FFT_SIZE as f32 - 1.0)).cos());
                Complex32::new(samples[i] * w, 0.0)
            } else {
                Complex32::new(0.0, 0.0)
            }
        })
        .collect();

    fft.process(&mut buf);

    let mags: Vec<f32> = buf[..DISPLAY_BINS]
        .iter()
        .map(|c| {
            let m = c.norm();
            if m > 1e-10 { 20.0 * m.log10() } else { -80.0 }
        })
        .collect();

    // Auto-scale: median as noise floor, 25 dB dynamic range above.
    // Independent per-column normalisation means a quiet slot looks
    // similar in contrast to a loud one — the user sees signal/noise
    // not absolute level.
    let mut sorted = mags.clone();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let floor = sorted[sorted.len() / 2];
    let ceil  = floor + 25.0;

    mags.iter()
        .map(|&v| ((v - floor) / (ceil - floor)).clamp(0.0, 1.0))
        .collect()
}

/// WSJT-style colour ramp: dark blue background, blue→cyan→yellow→white
/// for increasing signal strength. Same curve as FSK441Plus uses, kept
/// pixel-identical so anyone familiar with that visual style sees the
/// same thing here.
pub fn heat_color(v: f32) -> egui::Color32 {
    let v = v.clamp(0.0, 1.0);
    let (r, g, b) = if v < 0.2 {
        let t = v / 0.2;
        (0.0, 0.0, t * 0.5)
    } else if v < 0.5 {
        let t = (v - 0.2) / 0.3;
        (0.0, t * 0.7, 0.5 + t * 0.3)
    } else if v < 0.75 {
        let t = (v - 0.5) / 0.25;
        (t * 0.9, 0.7 + t * 0.3, 0.8 - t * 0.8)
    } else {
        let t = (v - 0.75) / 0.25;
        (0.9 + t * 0.1, 1.0, t * 0.3)
    };
    egui::Color32::from_rgb(
        (r * 255.0) as u8,
        (g * 255.0) as u8,
        (b * 255.0) as u8,
    )
}

/// One column of spectrum data, sent from the worker thread to the UI.
/// Carries both the FFT bins (for the heat-mapped waterfall display)
/// and the time-domain RMS amplitude of the same chunk (for the
/// amplitude trace overlaid along the bottom of the panel — matches
/// MSHV's signal-strength view).
///
/// `bins` is DISPLAY_BINS f32 entries, normalised 0..1 after the
/// median-floor + 25 dB ceiling auto-scale.
/// `rms` is the raw time-domain RMS of the input chunk, before any
/// scaling. UI side estimates a noise floor from the percentile
/// distribution and converts to dB-above-floor for the trace.
#[derive(Debug, Clone)]
pub struct SpectrumColumn {
    pub bins: Vec<f32>,
    pub rms:  f32,
}

/// Spawn the spectrum worker thread.
///
/// Reads audio chunks from `audio_rx`, computes one FFT column per
/// chunk, sends each column to `column_tx`. Exits when `audio_rx`
/// is closed.
///
/// The caller is expected to clone the audio sender at the cpal
/// fan-out site so this worker receives the same chunks as the
/// decoder. The two consumers run independently.
pub fn run_spectrum_thread(
    audio_rx: std::sync::mpsc::Receiver<Vec<f32>>,
    column_tx: std::sync::mpsc::Sender<SpectrumColumn>,
) {
    std::thread::Builder::new()
        .name("msk144-spectrum".into())
        .spawn(move || {
            let mut planner = FftPlanner::<f32>::new();
            while let Ok(chunk) = audio_rx.recv() {
                // For very short chunks (start of stream, audio
                // hiccups) skip the column — windowed FFT of a
                // mostly-zero buffer just produces noise.
                if chunk.len() < FFT_SIZE / 2 {
                    continue;
                }
                // Time-domain RMS for the amplitude trace. Computed
                // from the same chunk that produces the FFT column,
                // so the trace and waterfall are time-aligned. No
                // windowing — we want the raw envelope amplitude.
                let mut sum_sq = 0.0f64;
                for &s in chunk.iter() {
                    sum_sq += (s as f64) * (s as f64);
                }
                let rms = (sum_sq / chunk.len() as f64).sqrt() as f32;

                let bins = compute_column(&chunk, &mut planner);
                let col = SpectrumColumn { bins, rms };
                if column_tx.send(col).is_err() {
                    log::info!("[SPECTRUM] column_tx closed, exiting");
                    return;
                }
            }
            log::info!("[SPECTRUM] audio_rx closed, exiting");
        })
        .expect("spawn spectrum thread");
}
