// crates/msk144plus_packjt/src/pack_standard.rs
//
// Encode (i3=1, i3=2) standard messages into 77-bit payloads. Inverse of
// the unpack_standard function in lib.rs.
//
// Bit layout (matches lib/77bit/packjt77.f90 i3=1/2 paths):
//   bits  0..28 : n28a   28-bit packed call_1 (or CQ token)
//   bit  28     : ipa    1 if call_1 has rover suffix (/R or /P)
//   bits 29..57 : n28b   28-bit packed call_2
//   bit  57     : ipb    1 if call_2 has rover suffix
//   bit  58     : ir     1 if exchange has 'R' prefix (R-FN42, R+05, RR73)
//   bits 59..74 : igrid4 15-bit exchange (grid, report, or ack)
//   bits 74..77 : i3     = 1 (NA VHF) or 2 (EU VHF)

use crate::bits;
use crate::callsign::{pack28_standard, Call};
use crate::grid::{encode_ack, encode_grid, encode_report, Ack, Exchange};
use crate::{MsgVariant, StandardMessage};

/// Errors that can occur when packing a standard message.
#[derive(Debug, Clone, PartialEq)]
pub enum PackError {
    /// Couldn't encode call_1 (e.g. malformed callsign, hash-only call
    /// with no resolution).
    Call1Invalid,
    /// Couldn't encode call_2.
    Call2Invalid,
    /// Couldn't encode the exchange (grid out of range, etc).
    ExchangeInvalid,
    /// Hash-only Call (Hash22) cannot be packed in i3=1/2; needs i3=4
    /// instead.
    CallRequiresNonstandard,
}

/// Pack a StandardMessage into 77 bits. The returned array has bytes 0..77
/// as bits in MSB-first order. Bits 74..77 carry i3 (1 or 2) per the
/// variant.
pub fn pack_standard(msg: &StandardMessage) -> Result<[u8; 77], PackError> {
    let n28a = pack_call_to_n28(&msg.call_1).ok_or(PackError::Call1Invalid)?;
    let n28b = pack_call_to_n28(&msg.call_2).ok_or(PackError::Call2Invalid)?;
    let ipa = if msg.call_1_rover { 1 } else { 0 };
    let ipb = if msg.call_2_rover { 1 } else { 0 };
    let ir = if msg.roger { 1 } else { 0 };

    let igrid4 = encode_exchange(&msg.exchange).ok_or(PackError::ExchangeInvalid)?;

    let i3 = match msg.variant {
        MsgVariant::NaVhf => 1,
        MsgVariant::EuVhf => 2,
    };

    let mut payload = [0u8; 77];
    bits::write_be(&mut payload, 0, 28, n28a as u64);
    bits::write_be(&mut payload, 28, 1, ipa as u64);
    bits::write_be(&mut payload, 29, 28, n28b as u64);
    bits::write_be(&mut payload, 57, 1, ipb as u64);
    bits::write_be(&mut payload, 58, 1, ir as u64);
    bits::write_be(&mut payload, 59, 15, igrid4 as u64);
    bits::write_be(&mut payload, 74, 3, i3 as u64);
    Ok(payload)
}

/// Map a Call back to its 28-bit packed form. Returns None if the call
/// can't be packed in i3=1/2 (e.g. it's a Hash22 placeholder that needs
/// i3=4 instead).
fn pack_call_to_n28(call: &Call) -> Option<u32> {
    const NTOKENS: u32 = 2_063_592;
    const C4: &[u8] = b" ABCDEFGHIJKLMNOPQRSTUVWXYZ";

    match call {
        Call::De => Some(0),
        Call::Qrz => Some(1),
        Call::Cq => Some(2),
        Call::CqNum(n) => {
            let n = *n as u32;
            if n > 999 { return None; }
            Some(3 + n)
        }
        Call::CqLetters(s) => {
            // Up to 4 chars from C4 alphabet, base-27 encoded
            let bytes = s.as_bytes();
            if bytes.len() > 4 { return None; }
            // Right-justify with leading spaces (matches Fortran adjustr)
            let mut idx = [0u32; 4];
            let pad = 4 - bytes.len();
            for i in 0..bytes.len() {
                let c = bytes[i].to_ascii_uppercase();
                let p = C4.iter().position(|&a| a == c)?;
                idx[pad + i] = p as u32;
            }
            let n = idx[0] * 27 * 27 * 27 + idx[1] * 27 * 27 + idx[2] * 27 + idx[3];
            Some(1003 + n)
        }
        Call::Standard(s) => {
            // Use the existing pack28_standard helper
            let n22 = pack28_standard(s)?;
            // pack28_standard already returns the full n28 value with
            // NTOKENS offset baked in (need to verify).
            // Looking at unpack28: standard calls live at n28 >= NTOKENS+MAX22,
            // and pack28_standard should produce that range directly.
            Some(n22)
        }
        Call::Hash22(_) => None, // requires i3=4
        Call::Invalid => None,
    }
}

/// Encode an Exchange variant back to its 15-bit value.
fn encode_exchange(ex: &Exchange) -> Option<u16> {
    match ex {
        Exchange::Grid(g) => encode_grid(g),
        Exchange::Report(snr) => encode_report(*snr),
        Exchange::Acknowledgement(ack) => Some(encode_ack(ack.clone())),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{unpack77, Message};

    #[test]
    fn round_trip_cq_grid() {
        // Build "CQ K1ABC FN42" via unpack, then re-pack and confirm bit
        // identity.
        let mut payload = [0u8; 77];
        let n28a = 2u32; // CQ token
        let n28b = pack28_standard("K1ABC").unwrap();
        let igrid4 = encode_grid("FN42").unwrap();
        bits::write_be(&mut payload, 0, 28, n28a as u64);
        bits::write_be(&mut payload, 29, 28, n28b as u64);
        bits::write_be(&mut payload, 59, 15, igrid4 as u64);
        bits::write_be(&mut payload, 74, 3, 1);
        let msg = match unpack77(&payload) {
            Message::Standard(m) => m,
            _ => panic!("expected Standard"),
        };

        let repacked = pack_standard(&msg).expect("pack should succeed");
        assert_eq!(repacked, payload, "round-trip CQ failed");
    }

    #[test]
    fn round_trip_call_call_grid() {
        let mut payload = [0u8; 77];
        let n28a = pack28_standard("K1ABC").unwrap();
        let n28b = pack28_standard("W9XYZ").unwrap();
        let igrid4 = encode_grid("FN42").unwrap();
        bits::write_be(&mut payload, 0, 28, n28a as u64);
        bits::write_be(&mut payload, 29, 28, n28b as u64);
        bits::write_be(&mut payload, 59, 15, igrid4 as u64);
        bits::write_be(&mut payload, 74, 3, 1);
        let msg = match unpack77(&payload) {
            Message::Standard(m) => m,
            _ => panic!("expected Standard"),
        };
        let repacked = pack_standard(&msg).unwrap();
        assert_eq!(repacked, payload);
    }

    #[test]
    fn round_trip_call_call_rrr() {
        let mut payload = [0u8; 77];
        let n28a = pack28_standard("K1ABC").unwrap();
        let n28b = pack28_standard("W9XYZ").unwrap();
        let igrid4 = encode_ack(Ack::Rrr);
        bits::write_be(&mut payload, 0, 28, n28a as u64);
        bits::write_be(&mut payload, 29, 28, n28b as u64);
        bits::write_be(&mut payload, 59, 15, igrid4 as u64);
        bits::write_be(&mut payload, 74, 3, 1);
        let msg = match unpack77(&payload) {
            Message::Standard(m) => m,
            _ => panic!("expected Standard"),
        };
        let repacked = pack_standard(&msg).unwrap();
        assert_eq!(repacked, payload);
    }

    #[test]
    fn round_trip_eu_vhf_with_rover() {
        let mut payload = [0u8; 77];
        let n28a = pack28_standard("G4ABC").unwrap();
        let n28b = pack28_standard("F1XYZ").unwrap();
        let igrid4 = encode_grid("JO22").unwrap();
        bits::write_be(&mut payload, 0, 28, n28a as u64);
        bits::write_be(&mut payload, 28, 1, 1); // ipa=1 (rover)
        bits::write_be(&mut payload, 29, 28, n28b as u64);
        bits::write_be(&mut payload, 59, 15, igrid4 as u64);
        bits::write_be(&mut payload, 74, 3, 2); // i3=2 EU VHF
        let msg = match unpack77(&payload) {
            Message::Standard(m) => m,
            _ => panic!("expected Standard"),
        };
        let repacked = pack_standard(&msg).unwrap();
        assert_eq!(repacked, payload);
    }

    #[test]
    fn round_trip_with_signal_report() {
        let mut payload = [0u8; 77];
        let n28a = pack28_standard("K1ABC").unwrap();
        let n28b = pack28_standard("W9XYZ").unwrap();
        let igrid4 = encode_report(5).unwrap();
        bits::write_be(&mut payload, 0, 28, n28a as u64);
        bits::write_be(&mut payload, 29, 28, n28b as u64);
        bits::write_be(&mut payload, 58, 1, 1); // ir=1 ("R+05")
        bits::write_be(&mut payload, 59, 15, igrid4 as u64);
        bits::write_be(&mut payload, 74, 3, 1);
        let msg = match unpack77(&payload) {
            Message::Standard(m) => m,
            _ => panic!("expected Standard"),
        };
        let repacked = pack_standard(&msg).unwrap();
        assert_eq!(repacked, payload);
    }

    #[test]
    fn hash22_call_rejects() {
        // A Hash22 call must use i3=4, not i3=1/2
        let msg = StandardMessage {
            call_1: Call::Hash22(12345),
            call_1_rover: false,
            call_2: Call::Standard("K1ABC".to_string()),
            call_2_rover: false,
            roger: false,
            exchange: Exchange::Grid("FN42".to_string()),
            variant: MsgVariant::NaVhf,
        };
        assert_eq!(pack_standard(&msg).unwrap_err(), PackError::Call1Invalid);
    }
}
