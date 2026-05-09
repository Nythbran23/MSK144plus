// crates/msk144plus_fec/src/lib.rs
//
// MSK144 + MSK40 forward error correction.
//
// MSK144: (128,90) LDPC + 13-bit CRC.   Faithful port of WSJT-X bpdecode128_90.f90.
// MSK40:  (32,16)  LDPC.                 Faithful port of WSJT-X bpdecode40.f90.

pub mod parity;
pub mod ldpc;
pub mod encode;
pub mod crc;
pub mod short;

pub use ldpc::{decode_128_90_soft, DecodeResult};
pub use encode::encode_128_90;
pub use crc::{crc13_compute, crc13_check};
pub use short::{decode_short, encode_short, ShortDecodeResult};
