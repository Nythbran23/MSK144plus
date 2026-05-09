// crates/msk144plus_gui/src/audio.rs
//
// Thin shim re-exporting from dx_runtime::audio so existing imports
// keep working. Future cleanup: import directly from dx_runtime in
// app.rs / decoder.rs and delete this file.

pub use dx_runtime::audio::{
    list_input_devices, default_input_device_name, list_all_devices_diagnostic,
    start_capture as start_capture_inner, AudioChunk, AudioChunkTx, CaptureHandle,
    DEFAULT_SAMPLE_RATE,
};

/// MSK144+ target sample rate.
pub const SAMPLE_RATE: u32 = 12000;

/// Compatibility wrapper preserving the (device, tx) → handle signature.
pub fn start_capture(
    device_name: Option<String>,
    tx: AudioChunkTx,
) -> anyhow::Result<CaptureHandle> {
    start_capture_inner(device_name, SAMPLE_RATE, tx)
}
