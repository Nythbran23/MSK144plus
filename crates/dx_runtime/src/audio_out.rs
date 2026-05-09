// crates/dx_runtime/src/audio_out.rs
//
// MSK144 TX audio output. Direct port of MSK2K's
// crates/msk2k_audio/src/output.rs::AudioOutput::start():
//
//   1. Build StreamConfig with the requested rate. Try a probe build with
//      that config — if cpal accepts it, use it.
//   2. If the probe fails, query default_output_config() to get the
//      device's actual channel count, rebuild around that.
//   3. Try f32 sample format first; fall back to i16.
//   4. Mutex-buffered callback drains samples until the buffer is empty.
//
// Protocol-internal rate is fixed at 12000 Hz. We resample 12000 → device
// rate using rubato (FftFixedIn — same resampler MSK2K uses) so any
// soundcard works regardless of its native rate.

use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use anyhow::{Context, Result};

/// MSK144 protocol-internal audio rate.
pub const PROTOCOL_RATE: u32 = 12_000;

pub fn list_output_devices() -> Vec<String> {
    crate::audio_devices::list_output_displays()
}

pub fn default_output_device_name() -> Option<String> {
    cpal::default_host().default_output_device().and_then(|d| d.name().ok())
}

fn find_device(host: &cpal::Host, label: Option<&str>) -> Option<cpal::Device> {
    let label = match label {
        Some(l) if !l.is_empty() => l,
        _ => return host.default_output_device(),
    };
    crate::audio_devices::find_output_device(label)
}

/// Pick the soundcard rate. Prefer 12000 if the device offers it; fall
/// back to default_output_config()'s rate; final fallback is to ask cpal
/// to figure it out.
fn pick_hardware_rate(device: &cpal::Device) -> u32 {
    let cfgs: Vec<cpal::SupportedStreamConfigRange> = device
        .supported_output_configs()
        .map(|it| it.collect())
        .unwrap_or_default();

    log::info!("[TX] {} supported output configs:", cfgs.len());
    for c in &cfgs {
        log::info!("[TX]   ch={} rate={}-{} fmt={:?}",
            c.channels(), c.min_sample_rate().0,
            c.max_sample_rate().0, c.sample_format());
    }

    // 1. PROTOCOL_RATE if supported
    if cfgs.iter().any(|c|
        c.min_sample_rate().0 <= PROTOCOL_RATE
        && c.max_sample_rate().0 >= PROTOCOL_RATE)
    {
        return PROTOCOL_RATE;
    }
    // 2. Integer multiples of 12000
    for &r in &[24_000, 48_000, 96_000] {
        if cfgs.iter().any(|c|
            c.min_sample_rate().0 <= r && c.max_sample_rate().0 >= r)
        {
            return r;
        }
    }
    // 3. Common rates
    for &r in &[44_100, 22_050, 11_025, 8_000] {
        if cfgs.iter().any(|c|
            c.min_sample_rate().0 <= r && c.max_sample_rate().0 >= r)
        {
            return r;
        }
    }
    // 4. Default config's rate
    if let Ok(cfg) = device.default_output_config() {
        return cfg.sample_rate().0;
    }
    // 5. Last resort
    PROTOCOL_RATE
}

/// Resample using rubato FftFixedIn (matches MSK2K's choice).
fn resample(samples: Vec<f32>, from_rate: u32, to_rate: u32) -> Result<Vec<f32>> {
    if from_rate == to_rate {
        return Ok(samples);
    }
    use rubato::{FftFixedIn, Resampler};
    let chunk_in = 1024;
    let mut resampler = FftFixedIn::<f32>::new(
        from_rate as usize,
        to_rate as usize,
        chunk_in,
        2, // sub_chunks: lower = less latency, higher = more efficient
        1, // mono
    ).map_err(|e| anyhow::anyhow!("rubato FftFixedIn {}->{}: {}", from_rate, to_rate, e))?;

    let ratio = to_rate as f64 / from_rate as f64;
    let mut out: Vec<f32> = Vec::with_capacity(
        ((samples.len() as f64 * ratio) as usize) + 64);

    let mut pos = 0;
    while pos + chunk_in <= samples.len() {
        let chunk: Vec<f32> = samples[pos..pos + chunk_in].to_vec();
        let result = resampler.process(&[chunk], None)
            .map_err(|e| anyhow::anyhow!("rubato process: {}", e))?;
        if let Some(ch0) = result.into_iter().next() {
            out.extend(ch0);
        }
        pos += chunk_in;
    }
    if pos < samples.len() {
        let mut chunk: Vec<f32> = samples[pos..].to_vec();
        chunk.resize(chunk_in, 0.0);
        let result = resampler.process(&[chunk], None)
            .map_err(|e| anyhow::anyhow!("rubato process tail: {}", e))?;
        if let Some(ch0) = result.into_iter().next() {
            let valid = ((samples.len() - pos) as f64 * ratio) as usize;
            out.extend(ch0.into_iter().take(valid));
        }
    }
    log::info!("[TX] resampled {} samples @ {}Hz → {} samples @ {}Hz",
        samples.len(), from_rate, out.len(), to_rate);
    Ok(out)
}

/// Play `samples` (mono, 12 kHz audio). Blocks until playback completes.
///
/// `tx_truncate_ms` shortens the playback by that many milliseconds at
/// the END. This exists to handle half-duplex rigs (notably the IC-9700
/// USB CODEC on macOS) where the PA collapse, T/R relay switching, and
/// monitor-output tail bleed back into the input device for ~300 ms
/// after PTT release. Without truncation, that bleed lands inside the
/// FOLLOWING RX slot (slot start = TX end), producing spurious decodes
/// of our own callsign at `t_in_slot < 0.5s`. With a 300 ms truncation,
/// the bleed stays inside our own TX slot (already filtered) and the
/// next RX slot starts cleanly.
///
/// Pass 0 for no truncation.
pub fn play_buffer(
    device_label: Option<&str>,
    samples: Vec<f32>,
    desired_rate: u32,
    tx_level: f32,
    tx_truncate_ms: u32,
) -> Result<()> {
    if desired_rate != PROTOCOL_RATE {
        anyhow::bail!(
            "MSK144 protocol requires {} Hz internal rate; caller passed {} Hz",
            PROTOCOL_RATE, desired_rate);
    }

    let host = cpal::default_host();
    let device = find_device(&host, device_label)
        .ok_or_else(|| anyhow::anyhow!(
            "no output device found (label={:?})", device_label))?;
    let dev_name = device.name().unwrap_or_default();

    // Apply tx-truncate at the protocol rate (12 kHz) so the trim is
    // exact regardless of the eventual hardware rate. 300 ms = 3600
    // samples at 12 kHz. The trailing 300 ms of an MSK144 30-s slot
    // is silence anyway (the protocol's bursts are short and front-
    // loaded), so trimming has zero impact on what the partner
    // hears — but gives the rig 300 ms of dead air to switch back
    // to RX before our next slot boundary.
    let mut samples = samples;
    if tx_truncate_ms > 0 {
        let trim_samples = (tx_truncate_ms as usize * PROTOCOL_RATE as usize) / 1000;
        if samples.len() > trim_samples {
            let new_len = samples.len() - trim_samples;
            log::info!("[TX] truncating audio: {} → {} samples ({} ms removed from end)",
                samples.len(), new_len, tx_truncate_ms);
            samples.truncate(new_len);
        } else {
            log::warn!("[TX] tx_truncate_ms ({} ms = {} samples) exceeds audio length ({} samples); skipping truncation",
                tx_truncate_ms, trim_samples, samples.len());
        }
    }

    log::info!("[TX] play_buffer: device={:?} input_samples={} (12 kHz) level={:.2}",
        dev_name, samples.len(), tx_level);

    let hw_rate = pick_hardware_rate(&device);
    log::info!("[TX] target hardware rate: {} Hz", hw_rate);

    // ─── MSK2K pattern: probe-build with requested config (1ch @ hw_rate) ───
    // If cpal accepts that, use it. Otherwise fall back to default config.
    let requested_cfg = cpal::StreamConfig {
        channels:    1,
        sample_rate: cpal::SampleRate(hw_rate),
        buffer_size: cpal::BufferSize::Default,
    };

    let (stream_cfg, channels) = {
        let probe = device.build_output_stream(
            &requested_cfg,
            |_: &mut [f32], _: &cpal::OutputCallbackInfo| {},
            |_| {},
            None,
        );
        if probe.is_ok() {
            drop(probe);
            log::info!("[TX] device accepts probe build at 1ch @ {}Hz", hw_rate);
            (requested_cfg, 1usize)
        } else {
            log::info!("[TX] probe build failed; using default_output_config");
            match device.default_output_config() {
                Ok(cfg) => {
                    let ch = cfg.channels();
                    let cfg = cpal::StreamConfig {
                        channels: ch,
                        sample_rate: cpal::SampleRate(hw_rate),
                        buffer_size: cpal::BufferSize::Default,
                    };
                    (cfg, ch as usize)
                }
                Err(e) => {
                    log::warn!("[TX] default_output_config failed: {}; using requested_cfg anyway", e);
                    (requested_cfg, 1usize)
                }
            }
        }
    };

    log::info!("[TX] using {} channel(s) @ {} Hz hardware (protocol = 12000 Hz)",
        channels, hw_rate);

    // Resample protocol → hardware
    let hw_samples = resample(samples, PROTOCOL_RATE, hw_rate)?;
    let scaled: Vec<f32> = hw_samples.iter().map(|s| s * tx_level).collect();

    let buffer: Arc<Mutex<Vec<f32>>> = Arc::new(Mutex::new(scaled));
    let total_input_samples = buffer.lock().unwrap().len();
    let total_output_samples = total_input_samples * channels;
    let done = Arc::new(AtomicBool::new(false));

    let chans_cb = channels;
    let buf_f32  = buffer.clone();
    let done_f32 = done.clone();

    let stream = device.build_output_stream(
        &stream_cfg,
        move |out: &mut [f32], _: &cpal::OutputCallbackInfo| {
            let mut buf = buf_f32.lock().unwrap();
            let frames_needed = out.len() / chans_cb;
            let frames_avail  = buf.len();
            let frames_copy   = frames_needed.min(frames_avail);

            if chans_cb == 1 {
                if frames_copy > 0 {
                    out[..frames_copy].copy_from_slice(&buf[..frames_copy]);
                    buf.drain(..frames_copy);
                }
                if frames_copy < out.len() {
                    out[frames_copy..].fill(0.0);
                }
            } else {
                for i in 0..frames_copy {
                    let s = buf[i];
                    for ch in 0..chans_cb {
                        out[i * chans_cb + ch] = if ch == 0 { s } else { 0.0 };
                    }
                }
                if frames_copy > 0 { buf.drain(..frames_copy); }
                if frames_copy < frames_needed {
                    out[frames_copy * chans_cb..].fill(0.0);
                }
            }
            if buf.is_empty() {
                done_f32.store(true, Ordering::Release);
            }
        },
        |e| log::error!("[TX] f32 stream error: {}", e),
        None,
    )
    .or_else(|primary_err| {
        log::warn!("[TX] f32 build_output_stream failed: {}; trying i16", primary_err);
        let buf_i16  = buffer.clone();
        let done_i16 = done.clone();
        device.build_output_stream(
            &stream_cfg,
            move |out: &mut [i16], _: &cpal::OutputCallbackInfo| {
                let mut buf = buf_i16.lock().unwrap();
                let frames_needed = out.len() / chans_cb;
                let frames_avail  = buf.len();
                let frames_copy   = frames_needed.min(frames_avail);

                if chans_cb == 1 {
                    for i in 0..frames_copy {
                        out[i] = (buf[i] * 32767.0).clamp(-32768.0, 32767.0) as i16;
                    }
                    if frames_copy < out.len() {
                        for s in &mut out[frames_copy..] { *s = 0; }
                    }
                    if frames_copy > 0 { buf.drain(..frames_copy); }
                } else {
                    for i in 0..frames_copy {
                        let s = (buf[i] * 32767.0).clamp(-32768.0, 32767.0) as i16;
                        for ch in 0..chans_cb {
                            out[i * chans_cb + ch] = if ch == 0 { s } else { 0 };
                        }
                    }
                    if frames_copy > 0 { buf.drain(..frames_copy); }
                    if frames_copy < frames_needed {
                        for s in &mut out[frames_copy * chans_cb..] { *s = 0; }
                    }
                }
                if buf.is_empty() {
                    done_i16.store(true, Ordering::Release);
                }
            },
            |e| log::error!("[TX] i16 stream error: {}", e),
            None,
        )
    })
    .with_context(|| format!("build output stream on {} at {} Hz",
        dev_name, hw_rate))?;

    stream.play().context("stream play")?;
    log::info!("[TX] stream playing");

    // Wait for buffer drain. `done` flips true inside the cpal callback
    // when the buffer reaches zero. We poll at 50 ms; once it flips,
    // drop the stream immediately rather than sleeping further. An
    // earlier version sat for 200 ms after drain "to let the last
    // samples play out", but on macOS with a half-duplex USB CODEC
    // that 200 ms IS the bleed-back window we're trying to eliminate.
    // The buffer-empty signal fires AFTER the last sample has been
    // written to the cpal output queue — so there's nothing meaningful
    // for the extra sleep to wait for.
    let total_secs = total_input_samples as f32 / hw_rate as f32;
    let timeout = Duration::from_secs_f32(total_secs + 2.0);
    let start = std::time::Instant::now();
    while !done.load(Ordering::Acquire) {
        if start.elapsed() > timeout {
            log::warn!("[TX] playback timeout after {:.1}s", timeout.as_secs_f32());
            break;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    drop(stream);
    log::info!("[TX] playback complete: {:.2}s elapsed (total={} samples @ {} Hz)",
        start.elapsed().as_secs_f32(), total_output_samples, hw_rate);
    Ok(())
}
