// crates/dx_runtime/src/audio.rs
//
// Audio input pipeline. Target rate configurable per call (12 kHz
// for MSK144, 11025 Hz for FSK441, etc).
//
// Strategy (matches FSK441+ approach, just with target rate 12 kHz):
//   1. Try to open the device at 12 kHz native (no resampling).
//   2. If that fails, open at the device's default rate and use rubato
//      (polyphase sinc resampler) to convert to 12 kHz.
//   3. f32 first, fall back to i16 for ALSA S16_LE devices.
//
// Each chunk shipped to consumers carries a `capture_unix_ms`
// timestamp derived from cpal's `InputCallbackInfo::timestamp().capture`
// — the host audio system's authoritative claim about when the FIRST
// sample of the chunk was actually recorded by the hardware. Consumers
// (decoder framer in particular) MUST use this timestamp rather than
// `chrono::Utc::now()` at chunk arrival, otherwise OS scheduling
// jitter or audio-pipeline buffering can shift slot boundaries
// relative to wall clock.

use std::sync::mpsc::{Receiver, SyncSender};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use rubato::{Resampler, SincFixedIn, SincInterpolationParameters,
             SincInterpolationType, WindowFunction};

/// Default target rate. Pass a different rate to start_capture() for
/// other modes (FSK441 = 11025, MSK144 = 12000, MSK2K = ?).
pub const DEFAULT_SAMPLE_RATE: u32 = 12000;
#[allow(dead_code)]
pub const AUDIO_BUFFER_SIZE: usize = 1024;

/// One chunk of audio shipped from the cpal callback to the framer /
/// decoder thread, with the wall-clock timestamp of its first sample.
///
/// `capture_unix_ms` is derived from cpal's
/// `InputCallbackInfo::timestamp().capture` via a one-time calibration
/// against `chrono::Utc::now()` taken at the first callback. After
/// calibration, every subsequent timestamp is computed by adding the
/// monotonic cpal-clock delta to the calibration anchor — so all
/// chunks share a coherent timeline regardless of OS scheduling
/// jitter or callback latency. Slot identity (which 30 s wall-clock
/// window each chunk falls in) is computed from this field by the
/// framer; do not derive slot identity from `chrono::Utc::now()`
/// at chunk arrival, which can drift relative to capture time.
#[derive(Debug, Clone)]
pub struct AudioChunk {
    pub samples: Vec<f32>,
    pub capture_unix_ms: i64,
}

pub type AudioChunkTx = std::sync::mpsc::Sender<AudioChunk>;

/// Converts cpal's monotonic `StreamInstant` (mach absolute time on
/// macOS, CLOCK_MONOTONIC on Linux, QPC on Windows) into Unix-epoch
/// milliseconds via a one-time calibration.
///
/// The calibration sets `t0_unix_ms = chrono::Utc::now()` at the
/// moment of the first callback we see. Any error in this anchor
/// (callback dispatch latency between OS audio capture and our
/// closure firing) appears as a constant offset on all subsequent
/// timestamps — typically a few tens of ms on macOS, well below the
/// precision needed for 30 s slot attribution. Crucially, the offset
/// does NOT grow over time: deltas between cpal timestamps are
/// computed against `t0_cpal` (also captured at first callback), so
/// inter-chunk timing is exact.
struct CaptureCalibration {
    t0_cpal: Option<cpal::StreamInstant>,
    t0_unix_ms: i64,
}

impl CaptureCalibration {
    const fn new() -> Self {
        Self { t0_cpal: None, t0_unix_ms: 0 }
    }

    /// Convert a cpal capture timestamp to Unix ms. On the first call,
    /// records the calibration anchor and returns now-ish. On
    /// subsequent calls, returns `t0_unix_ms + (capture - t0_cpal)`.
    fn capture_to_unix_ms(&mut self, capture: cpal::StreamInstant) -> i64 {
        match &self.t0_cpal {
            Some(t0) => {
                let delta = capture.duration_since(t0)
                    .unwrap_or(std::time::Duration::ZERO);
                self.t0_unix_ms + delta.as_millis() as i64
            }
            None => {
                self.t0_cpal = Some(capture);
                self.t0_unix_ms = chrono::Utc::now().timestamp_millis();
                log::info!(
                    "[AUDIO] capture-timestamp calibration anchored: \
                     t0_unix_ms={} (subsequent chunks reported relative \
                     to this anchor via cpal monotonic clock)",
                    self.t0_unix_ms);
                self.t0_unix_ms
            }
        }
    }
}

pub fn list_input_devices() -> Vec<String> {
    crate::audio_devices::list_input_displays()
}

pub fn default_input_device_name() -> Option<String> {
    cpal::default_host()
        .default_input_device()
        .and_then(|d| d.name().ok())
}

pub fn list_all_devices_diagnostic() -> Vec<String> {
    let host = cpal::default_host();
    let mut lines = Vec::new();
    let inputs: Vec<cpal::Device> = host.input_devices().map(|d| d.collect()).unwrap_or_default();
    let outputs: Vec<cpal::Device> = host.output_devices().map(|d| d.collect()).unwrap_or_default();
    let default_in = host.default_input_device().and_then(|d| d.name().ok());
    let default_out = host.default_output_device().and_then(|d| d.name().ok());

    let mut in_seen: std::collections::HashMap<String, usize> = std::collections::HashMap::new();
    for d in &inputs {
        let name = d.name().unwrap_or_else(|_| "<unknown>".into());
        let n = in_seen.entry(name.clone()).or_insert(0);
        let label = if *n == 0 { name.clone() } else { format!("{} #{}", name, *n + 1) };
        let (rate, ch) = match d.default_input_config() {
            Ok(c) => (Some(c.sample_rate().0), Some(c.channels())),
            Err(_) => (None, None),
        };
        let star = if default_in.as_deref() == Some(name.as_str()) && *n == 0 { " *" } else { "" };
        lines.push(format!(
            "IN   {:<28} {} Hz × {}ch{}",
            label,
            rate.map(|r| r.to_string()).unwrap_or_else(|| "?".into()),
            ch.map(|c| c.to_string()).unwrap_or_else(|| "?".into()),
            star,
        ));
        *n += 1;
    }
    let mut out_seen: std::collections::HashMap<String, usize> = std::collections::HashMap::new();
    for d in &outputs {
        let name = d.name().unwrap_or_else(|_| "<unknown>".into());
        let n = out_seen.entry(name.clone()).or_insert(0);
        let label = if *n == 0 { name.clone() } else { format!("{} #{}", name, *n + 1) };
        let (rate, ch) = match d.default_output_config() {
            Ok(c) => (Some(c.sample_rate().0), Some(c.channels())),
            Err(_) => (None, None),
        };
        let star = if default_out.as_deref() == Some(name.as_str()) && *n == 0 { " *" } else { "" };
        lines.push(format!(
            "OUT  {:<28} {} Hz × {}ch{}",
            label,
            rate.map(|r| r.to_string()).unwrap_or_else(|| "?".into()),
            ch.map(|c| c.to_string()).unwrap_or_else(|| "?".into()),
            star,
        ));
        *n += 1;
    }
    lines
}

pub struct CaptureHandle {
    _stop_tx: SyncSender<()>,
    pub audio_info: String,
    /// Per-session liveness flag. The cpal callback closures hold a
    /// clone of this Arc and check it on every chunk; when false,
    /// chunks are dropped at the source rather than forwarded into
    /// `tx`. This is the ONLY reliable way to stop the audio pipeline
    /// on macOS, where dropping the cpal::Stream is asynchronous and
    /// the callback can keep firing for an indeterminate window
    /// after drop. Setting `session_active = false` immediately
    /// halts chunk delivery to all downstream consumers (tee,
    /// framer, decoder, spectrum).
    ///
    /// Owned by App; cloned into every worker that needs to know
    /// whether the current session is still alive. On Stop, App
    /// flips this to false. On the next Listen, App allocates a
    /// fresh Arc<AtomicBool>(true) — old workers see their old
    /// flag stay false and exit; new workers run with the new one.
    pub session_active: Arc<AtomicBool>,
}

pub fn start_capture(
    device_name: Option<String>,
    target_rate: u32,
    tx: AudioChunkTx,
    session_active: Arc<AtomicBool>,
) -> anyhow::Result<CaptureHandle> {
    let host = cpal::default_host();
    let device = if let Some(name) = device_name {
        crate::audio_devices::find_input_device(&name)
            .ok_or_else(|| anyhow::anyhow!("Input device '{}' not found", name))?
    } else {
        host.default_input_device()
            .ok_or_else(|| anyhow::anyhow!("No default input device"))?
    };

    let dev_name = device.name().unwrap_or_default();
    let default_cfg = device.default_input_config()
        .map_err(|e| anyhow::anyhow!("default config: {}", e))?;
    let native_rate = default_cfg.sample_rate().0;
    let n_channels = default_cfg.channels() as usize;
    log::info!("[AUDIO] Device: {}  native_rate={} Hz  channels={}  target_rate={} Hz",
        dev_name, native_rate, n_channels, target_rate);

    let (stop_tx, stop_rx) = std::sync::mpsc::sync_channel::<()>(1);

    // ── Path 1: device supports target_rate natively → no resampling ──
    if native_rate == target_rate {
        return start_native(&device, &dev_name, target_rate, n_channels, tx, stop_tx, stop_rx, session_active);
    }

    // Even if default isn't target_rate, the device might still support it.
    // Probe the supported configs.
    let supports_target = device.supported_input_configs()
        .map(|configs| configs.into_iter().any(|c|
            c.min_sample_rate().0 <= target_rate && c.max_sample_rate().0 >= target_rate))
        .unwrap_or(false);

    if supports_target {
        match start_native(&device, &dev_name, target_rate, n_channels, tx.clone(), stop_tx.clone(), stop_rx, session_active.clone()) {
            Ok(h) => return Ok(h),
            Err(e) => log::warn!("[AUDIO] {} Hz claimed supported but open failed: {} - falling back to {} Hz with resampling",
                target_rate, e, native_rate),
        }
    }

    // ── Path 2: open at native rate, resample with rubato ──
    let (stop_tx2, stop_rx2) = std::sync::mpsc::sync_channel::<()>(1);
    start_with_resample(&device, &dev_name, native_rate, target_rate, n_channels, tx, stop_tx2, stop_rx2, session_active)
}

/// Path 1: open the device at target_rate directly. f32 first, i16 fallback.
fn start_native(
    device: &cpal::Device,
    dev_name: &str,
    target_rate: u32,
    n_channels: usize,
    tx: AudioChunkTx,
    stop_tx: SyncSender<()>,
    stop_rx: Receiver<()>,
    session_active: Arc<AtomicBool>,
) -> anyhow::Result<CaptureHandle> {
    let cfg = cpal::StreamConfig {
        channels:    n_channels as u16,
        sample_rate: cpal::SampleRate(target_rate),
        buffer_size: cpal::BufferSize::Default,
    };
    let ch = n_channels;

    // Each callback closure owns its own calibration. Only one of the
    // two (f32 / i16) ever runs — whichever the device actually
    // accepts — so there's no shared state to coordinate.
    let mut cal_f32 = CaptureCalibration::new();
    let tx1 = tx.clone();
    let active_f32 = session_active.clone();
    let f32_result = device.build_input_stream(
        &cfg,
        move |data: &[f32], info: &cpal::InputCallbackInfo| {
            // Session-stopped check FIRST. On macOS the cpal
            // callback can keep firing for an indeterminate window
            // after the stream is dropped — and on the
            // Stop+Listen flow we want chunks to stop flowing the
            // instant Stop is pressed, even before the stream
            // actually shuts down. Without this check the old
            // session's chunks leak into the new session's
            // pipeline (when start_listening allocates a fresh
            // tee/framer chain), causing duplicate slot drains.
            if !active_f32.load(Ordering::Acquire) { return; }
            let mono: Vec<f32> = if ch == 1 {
                data.iter().map(|&s| s * 32768.0).collect()
            } else {
                data.chunks(ch)
                    .map(|f| (f.iter().sum::<f32>() / ch as f32) * 32768.0)
                    .collect()
            };
            let capture_unix_ms = cal_f32.capture_to_unix_ms(
                info.timestamp().capture);
            let _ = tx1.send(AudioChunk {
                samples: mono,
                capture_unix_ms,
            });
        },
        |e| log::error!("[AUDIO] Stream error: {}", e),
        None,
    );

    let stream = match f32_result {
        Ok(s) => {
            log::info!("[AUDIO] Capturing at {} Hz native (f32, no resampling)", target_rate);
            s
        }
        Err(e_f32) => {
            log::warn!("[AUDIO] f32 native build failed ({}), trying i16", e_f32);
            let mut cal_i16 = CaptureCalibration::new();
            let tx2 = tx.clone();
            let active_i16 = session_active.clone();
            device.build_input_stream(
                &cfg,
                move |data: &[i16], info: &cpal::InputCallbackInfo| {
                    if !active_i16.load(Ordering::Acquire) { return; }
                    let mono: Vec<f32> = if ch == 1 {
                        data.iter().map(|&s| s as f32).collect()
                    } else {
                        data.chunks(ch)
                            .map(|f| f.iter().map(|&s| s as i32).sum::<i32>() as f32 / ch as f32)
                            .collect()
                    };
                    let capture_unix_ms = cal_i16.capture_to_unix_ms(
                        info.timestamp().capture);
                    let _ = tx2.send(AudioChunk {
                        samples: mono,
                        capture_unix_ms,
                    });
                },
                |e| log::error!("[AUDIO] Stream error: {}", e),
                None,
            )
            .map_err(|e_i16| anyhow::anyhow!("f32 native: {}; i16 native: {}", e_f32, e_i16))?
        }
    };

    stream.play().map_err(|e| anyhow::anyhow!("stream play: {}", e))?;
    spawn_holder(stream, stop_rx);
    Ok(CaptureHandle {
        _stop_tx: stop_tx,
        audio_info: format!("Capturing at {} Hz from {}", target_rate, dev_name),
        session_active,
    })
}

/// Path 2: open at native_rate, resample to 12 kHz with rubato (polyphase sinc).
fn start_with_resample(
    device: &cpal::Device,
    dev_name: &str,
    native_rate: u32,
    target_rate: u32,
    n_channels: usize,
    tx: AudioChunkTx,
    stop_tx: SyncSender<()>,
    stop_rx: Receiver<()>,
    session_active: Arc<AtomicBool>,
) -> anyhow::Result<CaptureHandle> {
    let cfg = cpal::StreamConfig {
        channels:    n_channels as u16,
        sample_rate: cpal::SampleRate(native_rate),
        buffer_size: cpal::BufferSize::Default,
    };

    log::info!("[AUDIO] Capturing at {} Hz, resampling to {} Hz with rubato",
        native_rate, target_rate);

    // rubato sinc resampler. Match FSK441+'s parameters.
    // Built fresh at each use site (the type doesn't impl Clone in
    // older rubato versions, and the fields are all small Copy
    // values so re-construction is free).
    let ratio = target_rate as f64 / native_rate as f64;
    let make_params = || SincInterpolationParameters {
        sinc_len: 64,
        f_cutoff: 0.95,
        interpolation: SincInterpolationType::Linear,
        oversampling_factor: 64,
        window: WindowFunction::BlackmanHarris2,
    };
    // Process ~1024 input samples per call (low latency)
    let input_size = 1024;

    // ── Per-callback state: ALL captured by-value into the closure ──
    // The cpal input callback is invoked from CoreAudio's real-time
    // audio thread. Real-time threads must NEVER block on locks —
    // priority inversion or contention with normal-priority threads
    // can cause CoreAudio to silently mark the callback as failed
    // and stop dispatching it altogether (the symptom is the input
    // stream going dead with no log message). An earlier version of
    // this function held all state in `Arc<Mutex<...>>` and locked
    // four times per callback; on macOS with a duplex USB Audio
    // CODEC device that also acts as the TX output, this caused the
    // input stream to die after the first TX completed, leaving the
    // decoder framer chunk-starved for the rest of the QSO.
    //
    // The fix: each callback owns its own state by-value. The
    // closure is `move`-captured, the state lives in the closure's
    // captures, and the only thread that ever touches it is the
    // cpal callback thread. Zero locks, zero priority inversion.
    //
    // The downside: we build TWO `SincFixedIn` resamplers (one for
    // f32, one for i16 fallback) — only one will actually run.
    // That's a one-time cost at startup, irrelevant.
    let ch = n_channels;
    let tx_a = tx.clone();
    let mut cal_f32 = CaptureCalibration::new();
    let mut buf_f32: Vec<f32> = Vec::with_capacity(input_size * 4);
    let mut resampler_f32 = SincFixedIn::<f32>::new(
        ratio, 1.02, make_params(), input_size, 1)
        .map_err(|e| anyhow::anyhow!("rubato init: {}", e))?;
    let mut buf_head_t_ms_f32: f64 = 0.0;
    let active_f32 = session_active.clone();

    // f32 callback: downmix to mono, push to buffer, run resampler when full.
    let f32_result = device.build_input_stream(
        &cfg,
        move |data: &[f32], info: &cpal::InputCallbackInfo| {
            // Session-stopped check FIRST (see start_native for full
            // explanation). Drops chunks at source if Stop was pressed.
            if !active_f32.load(Ordering::Acquire) { return; }
            let cb_capture_ms = cal_f32.capture_to_unix_ms(
                info.timestamp().capture);

            let mono: Vec<f32> = if ch == 1 {
                data.to_vec()
            } else {
                data.chunks(ch)
                    .map(|f| f.iter().sum::<f32>() / ch as f32)
                    .collect()
            };
            // Re-derive buf[0]'s capture time from this callback's
            // authoritative timestamp accounting for any leftover
            // samples already in the buffer.
            let leftover_samples = buf_f32.len();
            let leftover_ms = leftover_samples as f64 * 1000.0
                / native_rate as f64;
            buf_head_t_ms_f32 = cb_capture_ms as f64 - leftover_ms;

            buf_f32.extend_from_slice(&mono);
            while buf_f32.len() >= input_size {
                let drain_t_ms = buf_head_t_ms_f32.round() as i64;
                let chunk: Vec<f32> = buf_f32.drain(..input_size).collect();
                let input = vec![chunk];
                match resampler_f32.process(&input, None) {
                    Ok(out) => {
                        if let Some(out_chunk) = out.into_iter().next() {
                            let scaled: Vec<f32> = out_chunk.iter()
                                .map(|s| s * 32768.0).collect();
                            let _ = tx_a.send(AudioChunk {
                                samples: scaled,
                                capture_unix_ms: drain_t_ms,
                            });
                        }
                    }
                    Err(e) => log::warn!("[AUDIO] resample: {}", e),
                }
                buf_head_t_ms_f32 += input_size as f64 * 1000.0
                    / native_rate as f64;
            }
        },
        |e| log::error!("[AUDIO] Stream error: {}", e),
        None,
    );

    let stream = match f32_result {
        Ok(s) => {
            log::info!("[AUDIO] Resampling stream opened (f32 in)");
            s
        }
        Err(e_f32) => {
            log::warn!("[AUDIO] f32 resample build failed ({}), trying i16", e_f32);
            // Build the i16-fallback's own state, fully independent
            // of the f32 path's. (The f32 closure was consumed by
            // the failed build_input_stream; nothing to clean up.)
            let tx_b = tx.clone();
            let mut cal_i16 = CaptureCalibration::new();
            let mut buf_i16: Vec<f32> = Vec::with_capacity(input_size * 4);
            let mut resampler_i16 = SincFixedIn::<f32>::new(
                ratio, 1.02, make_params(), input_size, 1)
                .map_err(|e| anyhow::anyhow!("rubato init (i16): {}", e))?;
            let mut buf_head_t_ms_i16: f64 = 0.0;
            let active_i16 = session_active.clone();
            device.build_input_stream(
                &cfg,
                move |data: &[i16], info: &cpal::InputCallbackInfo| {
                    if !active_i16.load(Ordering::Acquire) { return; }
                    let cb_capture_ms = cal_i16.capture_to_unix_ms(
                        info.timestamp().capture);
                    let mono: Vec<f32> = if ch == 1 {
                        data.iter().map(|&s| s as f32 / 32768.0).collect()
                    } else {
                        data.chunks(ch).map(|f| {
                            f.iter().map(|&s| s as i32).sum::<i32>() as f32
                                / (32768.0 * ch as f32)
                        }).collect()
                    };
                    let leftover_samples = buf_i16.len();
                    let leftover_ms = leftover_samples as f64 * 1000.0
                        / native_rate as f64;
                    buf_head_t_ms_i16 = cb_capture_ms as f64 - leftover_ms;

                    buf_i16.extend_from_slice(&mono);
                    while buf_i16.len() >= input_size {
                        let drain_t_ms = buf_head_t_ms_i16.round() as i64;
                        let chunk: Vec<f32> = buf_i16.drain(..input_size).collect();
                        let input = vec![chunk];
                        match resampler_i16.process(&input, None) {
                            Ok(out) => {
                                if let Some(out_chunk) = out.into_iter().next() {
                                    let scaled: Vec<f32> = out_chunk.iter()
                                        .map(|s| s * 32768.0).collect();
                                    let _ = tx_b.send(AudioChunk {
                                        samples: scaled,
                                        capture_unix_ms: drain_t_ms,
                                    });
                                }
                            }
                            Err(e) => log::warn!("[AUDIO] resample: {}", e),
                        }
                        buf_head_t_ms_i16 += input_size as f64 * 1000.0
                            / native_rate as f64;
                    }
                },
                |e| log::error!("[AUDIO] Stream error: {}", e),
                None,
            )
            .map_err(|e_i16| anyhow::anyhow!("f32 resample: {}; i16 resample: {}", e_f32, e_i16))?
        }
    };

    stream.play().map_err(|e| anyhow::anyhow!("stream play: {}", e))?;
    spawn_holder(stream, stop_rx);
    Ok(CaptureHandle {
        _stop_tx: stop_tx,
        audio_info: format!("Capturing at {} Hz → {} Hz from {}",
            native_rate, target_rate, dev_name),
        session_active,
    })
}

fn spawn_holder(stream: cpal::Stream, stop_rx: Receiver<()>) {
    struct StreamHolder(cpal::Stream);
    unsafe impl Send for StreamHolder {}
    let holder = StreamHolder(stream);
    std::thread::Builder::new()
        .name("msk144-audio-in".into())
        .spawn(move || {
            let _h = holder; // keeps stream alive until thread exits

            // Platform behaviour matches FSK441+ (which works
            // on Roger's IC-9700 USB CODEC on macOS).
            //
            // Linux: ALSA exposes the IC-9700 USB CODEC as a single
            //   half-duplex device handle. The TX worker can't open
            //   the device while RX holds it. We must drop the cpal
            //   stream before TX opens the output, then re-open it
            //   after TX completes. `stop_rx.recv()` blocks until
            //   `_stop_tx` is dropped (or a stop signal sent), at
            //   which point the thread exits and `_h` drops, which
            //   drops the `cpal::Stream`, which releases the ALSA
            //   handle.
            //
            // macOS / Windows: the IC-9700 exposes
            //   `USB Audio CODEC (RX)` and `USB Audio CODEC (TX)`
            //   as TWO separate CoreAudio / WASAPI devices. RX and
            //   TX never share a handle, so the input stream can
            //   stay alive across TX cycles indefinitely. We park
            //   this thread forever — the stream lives until the
            //   process exits.
            //
            //   Earlier versions of this file also blocked on
            //   `stop_rx.recv()` here on macOS, intended to support
            //   a tear-down/restart pattern at every TX boundary.
            //   That pattern hangs in practice: dropping
            //   `cpal::Stream` on macOS does not synchronously kill
            //   the CoreAudio callback, so any tee thread waiting
            //   for the callback's `tx` clone to drop blocks
            //   indefinitely. The simpler "park forever" pattern
            //   used by FSK441+ avoids the entire problem.
            #[cfg(target_os = "linux")]
            {
                let _ = stop_rx.recv();
                log::info!("[AUDIO] Linux: input stream released");
            }
            #[cfg(not(target_os = "linux"))]
            {
                let _ = stop_rx; // unused on macOS/Windows
                loop {
                    std::thread::sleep(std::time::Duration::from_secs(3600));
                }
            }
        })
        .ok();
}
