// crates/msk144plus_dsp/src/constants.rs
//
// MSK144 protocol constants. Centralised so every module sees the same values.

/// Sample rate in Hz (12 kHz).
pub const SAMPLE_RATE: u32 = 12000;

/// Number of samples per MSK144 message frame (864 = 12 ms × 72 ms).
/// One frame contains 144 channel symbols, each spanning 6 samples.
pub const NSPM: usize = 864;

/// Number of channel symbols per frame.
pub const N_CHANSYM: usize = 144;

/// Samples per channel symbol = NSPM / N_CHANSYM = 6.
pub const SAMPLES_PER_SYM: usize = 6;

/// Sync word S8 (the 8-bit sync sequence that appears at channel-symbol
/// positions 0..7 and 56..63 in each MSK144 frame).
pub const S8: [u8; 8] = [0, 1, 1, 1, 0, 0, 1, 0];

/// FFT size used by the analytic-signal converter (NFFT1 in WSJT-X mskrtd.f90).
pub const NFFT_ANALYTIC: usize = 8192;

/// Block size processed per call to mskrtd (NZ in WSJT-X mskrtd.f90).
/// 7168 samples = ~597 ms. Each mskrtd call analyses the most recent NZ
/// samples ending at the current audio position.
pub const NZ_MSKRTD: usize = 7168;
