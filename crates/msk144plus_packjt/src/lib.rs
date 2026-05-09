// crates/msk144plus_packjt/src/lib.rs
//
// MSK144+ packjt77: 77-bit message unpacker.
//
// Scope for v0.1: standard QSO messages only (i3=1 NA VHF, i3=2 EU VHF).
// These cover the vast majority of meteor scatter exchanges. Other message
// types (free text, DXpedition, contests, nonstandard calls) return
// `Message::Unsupported` for now.
//
// The bit layout for i3=1/2 (read big-endian from the 77-bit payload):
//   bits  0..28  : n28a   (28-bit callsign 1)
//   bit  28      : ipa    (rover suffix flag for call 1)
//   bits 29..57  : n28b   (28-bit callsign 2)
//   bit  57      : ipb    (rover suffix flag for call 2)
//   bit  58      : ir     ("Roger" flag)
//   bits 59..74  : igrid4 (15-bit grid or exchange code)
//   bits 74..77  : i3     (message type identifier, 1 or 2)
//
// The rover suffix is "/R" for i3=1 (NA VHF) or "/P" for i3=2 (EU VHF).

pub mod bits;
pub mod callsign;
pub mod free_text;
pub mod grid;
pub mod jenkins;
pub mod nonstandard;
pub mod pack77;
pub mod pack_msk40;
pub mod pack_standard;

pub use callsign::{unpack28, Call};
pub use free_text::{pack_free_text, unpack_free_text};
pub use grid::{decode_exchange, Ack, Exchange};
pub use jenkins::{format_call_pair, hash12, hash22, nhash};
pub use nonstandard::{
    lookup_hash12_in_calls, pack_nonstandard_full, unpack_nonstandard,
    NonstandardAck, NonstandardMessage,
};
pub use pack77::pack77 as pack77_text;
pub use pack_msk40::{pack_msk40, PackMsk40Error, RPT_TABLE as RPT_TABLE_MSK40};
pub use pack_standard::{pack_standard, PackError};

/// Decoded message. Standard covers i3=1/2 (NA/EU VHF), FreeText is
/// i3=0/n3=0 (13-character free text), Nonstandard is i3=4 (compound calls
/// with hashed partner). Unsupported covers other message types we don't
/// yet decode (i3=0/n3=1..6, i3=3, i3=5..7).
#[derive(Debug, Clone, PartialEq)]
pub enum Message {
    Standard(StandardMessage),
    /// i3=0, n3=0: 13-character free text.
    FreeText { text: String },
    /// i3=4: nonstandard compound callsign with hashed partner.
    Nonstandard(NonstandardMessage),
    Unsupported {
        i3: u8,
        n3: u8,
        /// Raw 77-bit payload for diagnostics.
        bits: Box<[u8; 77]>,
    },
    Invalid {
        reason: &'static str,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub enum MsgVariant {
    /// i3=1 - North America VHF, "/R" rover suffix.
    NaVhf,
    /// i3=2 - European VHF, "/P" suffix.
    EuVhf,
}

#[derive(Debug, Clone, PartialEq)]
pub struct StandardMessage {
    /// First (called) callsign or token (CQ, DE, QRZ).
    pub call_1: Call,
    /// Whether call 1 has the rover suffix (/R or /P depending on variant).
    pub call_1_rover: bool,
    /// Second (calling) callsign.
    pub call_2: Call,
    pub call_2_rover: bool,
    /// "R" prefix on the exchange (e.g., "R+05", "R FN42").
    pub roger: bool,
    /// Grid, signal report, or acknowledgement.
    pub exchange: Exchange,
    pub variant: MsgVariant,
}

impl Message {
    /// Render the message in the canonical text form WSJT-X displays. This
    /// is the format we want to reproduce byte-for-byte for side-by-side
    /// testing.
    ///
    /// Examples:
    ///   "K1ABC W9XYZ FN42"
    ///   "CQ K1ABC FN42"
    ///   "K1ABC W9XYZ R FN42"
    ///   "K1ABC W9XYZ +05"
    ///   "K1ABC W9XYZ RR73"
    ///   "K1ABC/R W9XYZ FN42"  (i3=1 with rover)
    ///   "K1ABC/P W9XYZ JO22"  (i3=2)
    pub fn to_text(&self) -> String {
        match self {
            Message::Standard(m) => m.to_text(),
            Message::FreeText { text } => text.trim().to_string(),
            Message::Nonstandard(m) => m.text.clone(),
            Message::Unsupported { i3, n3, .. } => {
                format!("<unsupported i3={} n3={}>", i3, n3)
            }
            Message::Invalid { reason } => format!("<invalid: {}>", reason),
        }
    }
}

impl StandardMessage {
    pub fn to_text(&self) -> String {
        let suffix = match self.variant {
            MsgVariant::NaVhf => "/R",
            MsgVariant::EuVhf => "/P",
        };

        let call_str = |call: &Call, rover: bool| -> String {
            let base = call.to_wire_text();
            // Special-case CQ_NNN / CQ_AAAA: the underscore is wire-only;
            // display uses a space.
            let display_base = if base.starts_with("CQ_") && base.len() > 2 {
                format!("CQ {}", &base[3..])
            } else {
                base
            };
            // Rover suffix only applies to "real" calls, not CQ-style or hash.
            let is_real = matches!(call, Call::Standard(_));
            if rover && is_real {
                format!("{}{}", display_base, suffix)
            } else {
                display_base
            }
        };

        let c1 = call_str(&self.call_1, self.call_1_rover);
        let c2 = call_str(&self.call_2, self.call_2_rover);

        match &self.exchange {
            Exchange::Grid(g) => {
                if self.roger {
                    format!("{} {} R {}", c1, c2, g)
                } else {
                    format!("{} {} {}", c1, c2, g)
                }
            }
            Exchange::Acknowledgement(Ack::None) => format!("{} {}", c1, c2),
            Exchange::Acknowledgement(Ack::Rrr) => format!("{} {} RRR", c1, c2),
            Exchange::Acknowledgement(Ack::Rr73) => format!("{} {} RR73", c1, c2),
            Exchange::Acknowledgement(Ack::Bare73) => format!("{} {} 73", c1, c2),
            Exchange::Report(isnr) => {
                let sign = if *isnr >= 0 { "+" } else { "-" };
                let mag = isnr.unsigned_abs();
                if self.roger {
                    format!("{} {} R{}{:02}", c1, c2, sign, mag)
                } else {
                    format!("{} {} {}{:02}", c1, c2, sign, mag)
                }
            }
        }
    }
}

/// Decode a 77-bit message payload (one byte per bit, 0 or 1). The first
/// 77 bytes of `payload` are read; extra bytes are ignored.
///
/// `recent_calls`: optional ledger of recently-heard callsigns used to
/// resolve i3=4 nonstandard-call messages. Pass None if not maintaining one.
pub fn unpack77_with_calls(payload: &[u8], recent_calls: Option<&[String]>) -> Message {
    if payload.len() < 77 {
        return Message::Invalid { reason: "payload < 77 bits" };
    }
    let i3 = bits::read_be(payload, 74, 3) as u8;
    // n3 is the trailing 3-bit subtype indicator; for i3=0 messages it
    // selects the variant (free text vs DXpedition vs ARRL FD vs telemetry).
    // For i3>=1 the nominal "n3" position overlaps with data bits, so we
    // capture it for diagnostics only.
    let n3 = bits::read_be(payload, 71, 3) as u8;

    match (i3, n3) {
        (1, _) | (2, _) => unpack_standard(payload, i3),
        (0, 0) => {
            // i3=0, n3=0: free text. The 71 message bits sit at positions 0..71
            // (the n3 bits at 71..74 already told us this is free text, and i3
            // at 74..77 is 0).
            let text = unpack_free_text(&payload[..71]);
            Message::FreeText { text }
        }
        (4, _) => {
            // i3=4: nonstandard compound call.
            match unpack_nonstandard(payload, recent_calls) {
                Some(m) => Message::Nonstandard(m),
                None => {
                    let mut bits_copy = [0u8; 77];
                    bits_copy.copy_from_slice(&payload[..77]);
                    Message::Unsupported { i3, n3, bits: Box::new(bits_copy) }
                }
            }
        }
        _ => {
            let mut bits_copy = [0u8; 77];
            bits_copy.copy_from_slice(&payload[..77]);
            Message::Unsupported { i3, n3, bits: Box::new(bits_copy) }
        }
    }
}

/// Decode a 77-bit message payload without a recent_calls table.
/// Equivalent to `unpack77_with_calls(payload, None)`. i3=4 messages
/// will display callsign hashes as `<...>` rather than resolved calls.
pub fn unpack77(payload: &[u8]) -> Message {
    unpack77_with_calls(payload, None)
}

fn unpack_standard(payload: &[u8], i3: u8) -> Message {
    let n28a   = bits::read_be(payload, 0, 28) as u32;
    let ipa    = bits::read_be(payload, 28, 1) as u8;
    let n28b   = bits::read_be(payload, 29, 28) as u32;
    let ipb    = bits::read_be(payload, 57, 1) as u8;
    let ir     = bits::read_be(payload, 58, 1) as u8;
    let igrid4 = bits::read_be(payload, 59, 15) as u16;

    let call_1 = unpack28(n28a);
    let call_2 = unpack28(n28b);

    if matches!(call_1, Call::Invalid) || matches!(call_2, Call::Invalid) {
        return Message::Invalid { reason: "callsign decode failed" };
    }

    let exchange = match decode_exchange(igrid4) {
        Some(e) => e,
        None => return Message::Invalid { reason: "exchange decode failed" },
    };

    let variant = if i3 == 1 { MsgVariant::NaVhf } else { MsgVariant::EuVhf };

    // Sanity: a "CQ ... R ..." or "CQ ... RRR" message is meaningless;
    // WSJT-X rejects these.
    if matches!(call_1, Call::Cq | Call::CqNum(_) | Call::CqLetters(_)) {
        let bad = match &exchange {
            Exchange::Acknowledgement(Ack::Rrr | Ack::Rr73 | Ack::Bare73) => true,
            _ if ir == 1 => true,
            _ => false,
        };
        if bad {
            return Message::Invalid { reason: "CQ + acknowledgement is meaningless" };
        }
    }

    Message::Standard(StandardMessage {
        call_1,
        call_1_rover: ipa == 1,
        call_2,
        call_2_rover: ipb == 1,
        roger: ir == 1,
        exchange,
        variant,
    })
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use callsign::pack28_standard;
    use grid::{encode_ack, encode_grid, encode_report};

    /// Test helper: build a 77-bit i3=1 message from field values.
    fn build_msg(
        n28a: u32, ipa: u8, n28b: u32, ipb: u8, ir: u8, igrid4: u16, i3: u8,
    ) -> [u8; 77] {
        let mut payload = [0u8; 77];
        bits::write_be(&mut payload, 0, 28, n28a as u64);
        bits::write_be(&mut payload, 28, 1, ipa as u64);
        bits::write_be(&mut payload, 29, 28, n28b as u64);
        bits::write_be(&mut payload, 57, 1, ipb as u64);
        bits::write_be(&mut payload, 58, 1, ir as u64);
        bits::write_be(&mut payload, 59, 15, igrid4 as u64);
        bits::write_be(&mut payload, 74, 3, i3 as u64);
        payload
    }

    #[test]
    fn standard_qso_with_grid() {
        let n28a = pack28_standard("K1ABC").unwrap();
        let n28b = pack28_standard("W9XYZ").unwrap();
        let igrid4 = encode_grid("FN42").unwrap();
        let payload = build_msg(n28a, 0, n28b, 0, 0, igrid4, 1);
        let msg = unpack77(&payload);
        assert_eq!(msg.to_text(), "K1ABC W9XYZ FN42");
    }

    #[test]
    fn standard_qso_with_roger_and_grid() {
        let n28a = pack28_standard("K1ABC").unwrap();
        let n28b = pack28_standard("W9XYZ").unwrap();
        let igrid4 = encode_grid("FN42").unwrap();
        let payload = build_msg(n28a, 0, n28b, 0, 1, igrid4, 1);
        assert_eq!(unpack77(&payload).to_text(), "K1ABC W9XYZ R FN42");
    }

    #[test]
    fn standard_qso_with_positive_report() {
        let n28a = pack28_standard("K1ABC").unwrap();
        let n28b = pack28_standard("W9XYZ").unwrap();
        let igrid4 = encode_report(5).unwrap();
        let payload = build_msg(n28a, 0, n28b, 0, 0, igrid4, 1);
        assert_eq!(unpack77(&payload).to_text(), "K1ABC W9XYZ +05");
    }

    #[test]
    fn standard_qso_with_negative_report_and_roger() {
        let n28a = pack28_standard("K1ABC").unwrap();
        let n28b = pack28_standard("W9XYZ").unwrap();
        let igrid4 = encode_report(-15).unwrap();
        let payload = build_msg(n28a, 0, n28b, 0, 1, igrid4, 1);
        assert_eq!(unpack77(&payload).to_text(), "K1ABC W9XYZ R-15");
    }

    #[test]
    fn standard_qso_with_rrr() {
        let n28a = pack28_standard("K1ABC").unwrap();
        let n28b = pack28_standard("W9XYZ").unwrap();
        let igrid4 = encode_ack(Ack::Rrr);
        let payload = build_msg(n28a, 0, n28b, 0, 0, igrid4, 1);
        assert_eq!(unpack77(&payload).to_text(), "K1ABC W9XYZ RRR");
    }

    #[test]
    fn standard_qso_with_rr73() {
        let n28a = pack28_standard("K1ABC").unwrap();
        let n28b = pack28_standard("W9XYZ").unwrap();
        let igrid4 = encode_ack(Ack::Rr73);
        let payload = build_msg(n28a, 0, n28b, 0, 0, igrid4, 1);
        assert_eq!(unpack77(&payload).to_text(), "K1ABC W9XYZ RR73");
    }

    #[test]
    fn cq_with_grid() {
        let n28a = 2; // CQ
        let n28b = pack28_standard("K1ABC").unwrap();
        let igrid4 = encode_grid("FN42").unwrap();
        let payload = build_msg(n28a, 0, n28b, 0, 0, igrid4, 1);
        assert_eq!(unpack77(&payload).to_text(), "CQ K1ABC FN42");
    }

    #[test]
    fn cq_directed_numeric() {
        // "CQ 137 K1ABC FN42"
        let n28a = 3 + 137; // CQ_137
        let n28b = pack28_standard("K1ABC").unwrap();
        let igrid4 = encode_grid("FN42").unwrap();
        let payload = build_msg(n28a, 0, n28b, 0, 0, igrid4, 1);
        assert_eq!(unpack77(&payload).to_text(), "CQ 137 K1ABC FN42");
    }

    #[test]
    fn rover_suffix_na_vhf() {
        let n28a = pack28_standard("K1ABC").unwrap();
        let n28b = pack28_standard("W9XYZ").unwrap();
        let igrid4 = encode_grid("FN42").unwrap();
        // i3=1, both have rover flags
        let payload = build_msg(n28a, 1, n28b, 1, 0, igrid4, 1);
        assert_eq!(unpack77(&payload).to_text(), "K1ABC/R W9XYZ/R FN42");
    }

    #[test]
    fn rover_suffix_eu_vhf() {
        let n28a = pack28_standard("PA3XYZ").unwrap();
        let n28b = pack28_standard("GM4ABC").unwrap();
        let igrid4 = encode_grid("JO22").unwrap();
        // i3=2 -> /P
        let payload = build_msg(n28a, 1, n28b, 1, 0, igrid4, 2);
        assert_eq!(unpack77(&payload).to_text(), "PA3XYZ/P GM4ABC/P JO22");
    }

    #[test]
    fn cq_with_rrr_rejected() {
        let n28a = 2; // CQ
        let n28b = pack28_standard("K1ABC").unwrap();
        let igrid4 = encode_ack(Ack::Rrr);
        let payload = build_msg(n28a, 0, n28b, 0, 0, igrid4, 1);
        match unpack77(&payload) {
            Message::Invalid { .. } => {} // expected
            other => panic!("CQ + RRR should be invalid, got {:?}", other),
        }
    }

    #[test]
    fn unsupported_i3_returns_unsupported() {
        // i3=3 is reserved/undefined in WSJT-X 3.0.0; verify it falls into
        // the Unsupported branch. (i3=4 is now decoded as Nonstandard.)
        let mut payload = [0u8; 77];
        bits::write_be(&mut payload, 74, 3, 3); // i3=3
        match unpack77(&payload) {
            Message::Unsupported { i3, .. } => assert_eq!(i3, 3),
            other => panic!("expected Unsupported, got {:?}", other),
        }
    }

    #[test]
    fn i3_4_decodes_as_nonstandard() {
        // i3=4 should now decode as Nonstandard (previously Unsupported)
        let mut payload = [0u8; 77];
        bits::write_be(&mut payload, 74, 3, 4); // i3=4
        match unpack77(&payload) {
            Message::Nonstandard(_) => {}
            other => panic!("expected Nonstandard, got {:?}", other),
        }
    }

    #[test]
    fn i3_0_n3_0_decodes_as_free_text() {
        // i3=0, n3=0 should decode as free text. With all-zero bits the
        // text is 13 spaces, which to_text() trims to empty.
        let payload = [0u8; 77];
        match unpack77(&payload) {
            Message::FreeText { .. } => {}
            other => panic!("expected FreeText, got {:?}", other),
        }
    }

    /// Real-world callsigns Roger uses.
    #[test]
    fn gw4wnd_qso() {
        let n28a = pack28_standard("GW4WND").unwrap();
        let n28b = pack28_standard("DK5YA").unwrap();
        let igrid4 = encode_grid("IO82").unwrap();
        let payload = build_msg(n28a, 0, n28b, 0, 0, igrid4, 1);
        assert_eq!(unpack77(&payload).to_text(), "GW4WND DK5YA IO82");
    }
}
