// crates/dx_runtime/src/lib.rs
//
// Shared runtime for amateur radio digital mode applications.
//
// This crate provides the cross-cutting infrastructure that any
// digital-mode receiver app needs: audio capture, persistence (SQLite),
// auto-WAV recording on decode events, file-rotated logging, and
// (eventually) hamlib CAT control.
//
// Each mode app supplies its own decoder; everything else comes from
// here so the same look-and-feel and persistence model works across
// FSK441+, MSK144+, MSK2K, and the future unified app.

pub mod adif;
pub mod audio;
pub mod audio_devices;
pub mod audio_out;
pub mod hamlib;
pub mod logger;
pub mod paths;
pub mod persistence;
pub mod proto;
pub mod pskreporter;
pub mod qso;
pub mod wsjtx_udp;
pub mod recorder;
pub mod rigctld;
pub mod settings;

pub use adif::{AdifLogger, QsoRecord, default_adif_path};
pub use audio::{
    list_input_devices, default_input_device_name, list_all_devices_diagnostic,
    start_capture, AudioChunkTx, CaptureHandle, DEFAULT_SAMPLE_RATE,
};
pub use audio_out::{list_output_devices, default_output_device_name, play_buffer};
pub use hamlib::{HamlibClient, HamlibUpdate, HamlibCmd, band_from_freq_hz};
pub use logger::init as init_logger;
pub use paths::Paths;
pub use persistence::{Database, DecodeRecord, HeardCall};
pub use recorder::{Recorder, SaveConfig};
pub use rigctld::{RigctldLauncher, RigctldOpts, ProcessGuard,
    find_rigctld, list_serial_ports, common_rig_models};
pub use settings::{Settings, AudioConfig, StationConfig, DecoderConfig, UiConfig};
