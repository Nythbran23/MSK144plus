// crates/dx_runtime/src/recorder.rs
//
// Auto-WAV-on-event recorder. Continuously buffers the last N seconds of
// audio in memory; when a `trigger_save()` call comes in, writes the
// surrounding window (pre-roll + post-roll) to a WAV file.
//
// Typical use: pre-roll 15 sec, post-roll 15 sec → each saved WAV is a
// 30-sec window centred on the decode event.
//
// Memory cost: 30 sec * 12000 Hz * 4 bytes (f32) = 1.4 MB per buffer. Fine.
//
// Usage from a decoder thread:
//
//   let rec = Recorder::new(SaveConfig {
//       sample_rate: 12000,
//       pre_roll_secs: 15,
//       post_roll_secs: 15,
//       captures_root: paths.captures_dir.clone(),
//   });
//   // Each audio chunk:
//   rec.push_audio(&chunk);
//   // On decode:
//   let path = rec.trigger_save("I3FGX")?;  // returns path WAV will be written to
//   // …continues buffering; once post_roll_secs after trigger has elapsed,
//   // the WAV is finalised automatically on next push_audio() call.

use anyhow::Result;
use std::collections::VecDeque;
use std::path::{Path, PathBuf};
use std::sync::Mutex;

#[derive(Debug, Clone)]
pub struct SaveConfig {
    pub sample_rate: u32,
    pub pre_roll_secs: u32,
    pub post_roll_secs: u32,
    pub captures_root: PathBuf,
}

impl Default for SaveConfig {
    fn default() -> Self {
        Self {
            sample_rate: 12000,
            pre_roll_secs: 15,
            post_roll_secs: 15,
            captures_root: PathBuf::from("captures"),
        }
    }
}

/// Pending save: wait until we have `samples_remaining` more audio samples,
/// then write the buffer to disk.
struct PendingSave {
    target_path: PathBuf,
    samples_remaining: usize,
    /// Samples already in the buffer when the trigger fired (these become
    /// the pre-roll portion of the saved WAV).
    pre_roll_snapshot: Vec<i16>,
    /// Post-trigger samples accumulated since the trigger fired.
    post_roll: Vec<i16>,
}

struct Inner {
    cfg: SaveConfig,
    /// Ring buffer of recent samples (i16 to save memory).
    ring: VecDeque<i16>,
    /// Max ring size = pre_roll_secs * sample_rate
    ring_capacity: usize,
    pending: Vec<PendingSave>,
    /// True while we're transmitting. push_audio() respects this:
    ///   - Does not append samples to ring (so pre-roll for the next decode
    ///     after TX won't include our outgoing audio)
    ///   - Does not extend post_roll for any pending saves (so triggered
    ///     saves don't capture our TX in their post-roll)
    ///   - When transitioning false→true, also forces all pending saves
    ///     to finalise immediately with whatever post-roll they have.
    tx_active: bool,
}

pub struct Recorder {
    inner: Mutex<Inner>,
}

impl Recorder {
    pub fn new(cfg: SaveConfig) -> Self {
        let ring_capacity = cfg.pre_roll_secs as usize * cfg.sample_rate as usize;
        Self {
            inner: Mutex::new(Inner {
                cfg,
                ring: VecDeque::with_capacity(ring_capacity),
                ring_capacity,
                pending: Vec::new(),
                tx_active: false,
            }),
        }
    }

    /// Toggle TX state. Call with `true` when TX starts and `false` when
    /// it ends. While TX is active push_audio() ignores incoming samples
    /// (so they don't pollute the next decode's pre-roll) and pending
    /// saves are not extended (so their post-rolls don't capture our TX).
    /// On transition false→true, all currently-pending saves are flushed
    /// immediately with whatever post-roll they've accumulated so far.
    pub fn set_tx_active(&self, active: bool) {
        let cfg_rate;
        let to_finalise: Vec<PendingSave>;
        {
            let mut inner = self.inner.lock().unwrap();
            let prev = inner.tx_active;
            inner.tx_active = active;
            if active && !prev {
                cfg_rate = inner.cfg.sample_rate;
                to_finalise = std::mem::take(&mut inner.pending);
            } else {
                return;
            }
        }
        for save in to_finalise {
            if let Err(e) = write_wav(&save.target_path, cfg_rate,
                &save.pre_roll_snapshot, &save.post_roll) {
                log::warn!("[REC] pre-TX flush write {} failed: {}",
                    save.target_path.display(), e);
            } else {
                log::info!("[REC] pre-TX flush saved {} ({} pre + {} post samples)",
                    save.target_path.display(),
                    save.pre_roll_snapshot.len(),
                    save.post_roll.len());
            }
        }
    }

    /// Push a chunk of audio samples (already at `cfg.sample_rate`).
    /// Samples are expected as f32 in the int16-equivalent amplitude
    /// range (i.e. after the * 32768 scaling done in the audio module).
    pub fn push_audio(&self, samples: &[f32]) {
        let mut inner = self.inner.lock().unwrap();
        // While TX is active, drop incoming audio entirely. The ring
        // buffer freezes (so the pre-roll for the next decode after TX
        // won't include our own outgoing signal echoed back via the
        // rig's monitor circuit). Pending saves also don't get extended.
        if inner.tx_active {
            return;
        }
        // Convert f32 → i16, push to ring
        for &s in samples {
            let v = s.clamp(-32768.0, 32767.0) as i16;
            if inner.ring.len() == inner.ring_capacity {
                inner.ring.pop_front();
            }
            inner.ring.push_back(v);
            // Append to all pending saves' post_roll
            for p in inner.pending.iter_mut() {
                p.post_roll.push(v);
                if p.samples_remaining > 0 {
                    p.samples_remaining -= 1;
                }
            }
        }

        // Finalise any pending saves whose post_roll is full
        let cfg_rate = inner.cfg.sample_rate;
        let mut to_finalise: Vec<PendingSave> = Vec::new();
        let mut i = 0;
        while i < inner.pending.len() {
            if inner.pending[i].samples_remaining == 0 {
                to_finalise.push(inner.pending.remove(i));
            } else {
                i += 1;
            }
        }
        drop(inner);
        for save in to_finalise {
            if let Err(e) = write_wav(&save.target_path, cfg_rate,
                &save.pre_roll_snapshot, &save.post_roll) {
                log::warn!("[REC] write WAV {} failed: {}", save.target_path.display(), e);
            } else {
                log::info!("[REC] saved {} ({} pre + {} post samples)",
                    save.target_path.display(),
                    save.pre_roll_snapshot.len(),
                    save.post_roll.len());
            }
        }
    }

    /// Trigger a save. Snapshots the current ring buffer as pre-roll,
    /// then captures the next `post_roll_secs` of audio. Returns the
    /// path the WAV will eventually be written to.
    pub fn trigger_save(&self, target_path: PathBuf) -> Result<PathBuf> {
        let mut inner = self.inner.lock().unwrap();
        if let Some(parent) = target_path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let pre_roll_snapshot: Vec<i16> = inner.ring.iter().copied().collect();
        let post_roll_samples = (inner.cfg.post_roll_secs as usize)
            * (inner.cfg.sample_rate as usize);
        inner.pending.push(PendingSave {
            target_path: target_path.clone(),
            samples_remaining: post_roll_samples,
            pre_roll_snapshot,
            post_roll: Vec::with_capacity(post_roll_samples),
        });
        Ok(target_path)
    }

    /// Number of pending saves waiting for their post-roll to complete.
    pub fn pending_count(&self) -> usize {
        self.inner.lock().unwrap().pending.len()
    }

    /// Flush all pending saves immediately (whatever post-roll they have so far).
    /// Useful at app shutdown.
    pub fn flush(&self) -> Result<()> {
        let cfg_rate;
        let mut to_finalise: Vec<PendingSave>;
        {
            let mut inner = self.inner.lock().unwrap();
            cfg_rate = inner.cfg.sample_rate;
            to_finalise = std::mem::take(&mut inner.pending);
        }
        for save in to_finalise.drain(..) {
            if let Err(e) = write_wav(&save.target_path, cfg_rate,
                &save.pre_roll_snapshot, &save.post_roll) {
                log::warn!("[REC] flush write {} failed: {}", save.target_path.display(), e);
            }
        }
        Ok(())
    }
}

fn write_wav(path: &Path, sample_rate: u32, pre: &[i16], post: &[i16]) -> Result<()> {
    let spec = hound::WavSpec {
        channels: 1,
        sample_rate,
        bits_per_sample: 16,
        sample_format: hound::SampleFormat::Int,
    };
    let mut writer = hound::WavWriter::create(path, spec)?;
    for &s in pre { writer.write_sample(s)?; }
    for &s in post { writer.write_sample(s)?; }
    writer.finalize()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_save() {
        let dir = std::env::temp_dir().join(format!(
            "dx_rec_test_{}", std::process::id()
        ));
        let _ = std::fs::create_dir_all(&dir);
        let target = dir.join("test.wav");
        let _ = std::fs::remove_file(&target);

        let rec = Recorder::new(SaveConfig {
            sample_rate: 12000,
            pre_roll_secs: 1,
            post_roll_secs: 1,
            captures_root: dir.clone(),
        });
        // Push 1 sec of samples (12000 samples)
        let pre: Vec<f32> = (0..12000).map(|i| (i as f32) % 100.0).collect();
        rec.push_audio(&pre);
        // Trigger save
        rec.trigger_save(target.clone()).unwrap();
        // Push 1 sec more (this should complete the save)
        let post: Vec<f32> = (0..12000).map(|i| (i as f32 + 5000.0) % 100.0).collect();
        rec.push_audio(&post);

        // File should exist
        assert!(target.exists(), "expected {} to exist", target.display());
        let r = hound::WavReader::open(&target).unwrap();
        let n = r.duration();
        // 12000 pre + 12000 post = 24000 samples = 2 sec
        assert!(n == 24000 || n == 23999, "got {} samples (expected 24000)", n);

        let _ = std::fs::remove_file(&target);
    }
}
