// crates/msk144plus_gui/src/audio.rs
//
// Thin shim re-exporting from dx_runtime::audio so existing imports
// keep working. Future cleanup: import directly from dx_runtime in
// app.rs / decoder.rs and delete this file.

use std::sync::Arc;
use std::sync::atomic::AtomicBool;

pub use dx_runtime::audio::{
    list_input_devices, default_input_device_name, list_all_devices_diagnostic,
    start_capture as start_capture_inner, AudioChunk, AudioChunkTx, CaptureHandle,
    DEFAULT_SAMPLE_RATE,
};

/// MSK144+ target sample rate.
pub const SAMPLE_RATE: u32 = 12000;

/// Compatibility wrapper preserving the (device, tx, session_active)
/// → handle signature. The `session_active` flag is the per-listening-
/// session liveness signal threaded through the entire audio pipeline
/// (cpal callbacks, tee, framer, decoder worker). When App's
/// `tear_down_audio_pipeline` flips it to false, every worker checks
/// the flag on its next tick and exits cleanly. See app.rs for the
/// allocation site and dx_runtime::audio for the cpal-callback hook.
pub fn start_capture(
    device_name: Option<String>,
    tx: AudioChunkTx,
    session_active: Arc<AtomicBool>,
) -> anyhow::Result<CaptureHandle> {
    start_capture_inner(device_name, SAMPLE_RATE, tx, session_active)
}
