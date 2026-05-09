// crates/msk144plus_dsp/src/lib.rs
//
// MSK144 DSP primitives, faithfully ported from WSJT-X 3.0.0.

pub mod analytic;
pub mod constants;
pub mod decode_frame;
pub mod decode_frame_msk40;
pub mod spd;
pub mod spd_msk40;
pub mod sync;
pub mod tx;
pub mod tx_msk40;

pub use analytic::{analytic, AnalyticFilter};
pub use constants::*;
pub use decode_frame::{
    build_sync_waveform, demodulate_frame, half_sine_pulse,
    softbits_to_ldpc_llr, DemodulatedFrame,
};
pub use decode_frame_msk40::{
    build_sync_waveform_msk40, demodulate_short_frame, short_softbits_to_llr,
    DemodulatedShortFrame, NSPM_MSK40, N_CHANSYM_MSK40, S8R,
};
pub use spd::{detect_candidates, SpdCandidate};
pub use spd_msk40::{
    detect_short_candidates, msk40_sync, ShortSpdCandidate, ShortSyncResult,
};
pub use sync::{msk144_sync, SyncResult};
pub use tx::{build_channel_bits, generate_msk144_frame, generate_msk144_slot};
pub use tx_msk40::{build_channel_bits_msk40, generate_msk40_frame, generate_msk40_slot};
