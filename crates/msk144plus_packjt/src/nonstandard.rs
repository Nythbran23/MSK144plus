// crates/msk144plus_packjt/src/nonstandard.rs
//
// i3=4 nonstandard-call message unpacker.
//
// Bit layout (77 bits total, with i3=4 in bits 74..77):
//   bits  0..12 : n12   12-bit hash of one of the two callsigns
//   bits 12..70 : n58   58-bit base-38 encoded 11-char compound call
//   bit  70     : iflip 0 = hash is TO-call (left), 1 = hash is FROM-call (right)
//   bits 71..73 : nrpt  ack: 0=none, 1=RRR, 2=RR73, 3=73
//   bit  73     : icq   1 = "CQ <call>" form (single call only)
//   bits 74..77 : i3    = 4
//
// Faithful port of the i3=4 branch in unpack77 from lib/77bit/packjt77.f90
// lines 585-626.
//
// The compound-call alphabet has 38 characters (one fewer than free-text):
//   ' 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ/'
// (space, digits, uppercase letters, slash for portable suffixes)

use crate::bits;

pub const COMPOUND_ALPHABET: &[u8; 38] = b" 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ/";

/// Result of unpacking a nonstandard-call message.
#[derive(Debug, Clone, PartialEq)]
pub struct NonstandardMessage {
    /// Rendered text, e.g. "PJ4/KA1ABC <WA9XYZ> RR73" or "CQ PJ4/KA1ABC".
    pub text: String,
    /// The 12-bit hash that was received.
    pub hash12: u16,
    /// The compound (full 11-char) callsign.
    pub compound_call: String,
    /// Acknowledgement type.
    pub ack: NonstandardAck,
    /// True if this was a "CQ <call>" message.
    pub is_cq: bool,
    /// Whether the hash represents the TO-call (false) or FROM-call (true).
    /// Per WSJT-X iflip convention.
    pub hash_is_from_call: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NonstandardAck {
    None,
    Rrr,
    Rr73,
    Bare73,
}

/// Decode the 11-character compound callsign from 58 bits (base-38).
fn decode_compound_call(n58: u64) -> String {
    let mut chars = [b' '; 11];
    let mut v = n58;
    for i in (0..11).rev() {
        let digit = (v % 38) as usize;
        chars[i] = COMPOUND_ALPHABET[digit];
        v /= 38;
    }
    // Trim trailing spaces. The encoded call is right-justified within
    // 11 chars but Fortran does adjustl() to left-justify, so we trim
    // spaces both sides.
    let s = std::str::from_utf8(&chars).unwrap_or("").trim();
    s.to_string()
}

/// Look up a 12-bit hash in a list of recent callsigns. Returns the
/// callsign whose `hash12` matches, or None.
///
/// `formatter` is the same hash function used for transmission; pass
/// `crate::jenkins::hash12` and a closure that formats single calls
/// with their length-37 buffer (a single call alone, without partner).
pub fn lookup_hash12_in_calls(
    target_hash: u16,
    candidates: &[String],
) -> Option<String> {
    for call in candidates {
        let h = crate::jenkins::hash12(&crate::jenkins::format_call_pair(call, ""));
        if h == target_hash {
            return Some(call.clone());
        }
    }
    None
}

/// Unpack a 77-bit i3=4 payload.
///
/// `payload[0..77]` should hold the 77 message bits. `recent_calls` is an
/// optional ledger used to resolve the 12-bit hash back to a real
/// callsign. If unresolved, the text uses `<...>` as a placeholder
/// (matching WSJT-X's display convention).
pub fn unpack_nonstandard(
    payload: &[u8],
    recent_calls: Option<&[String]>,
) -> Option<NonstandardMessage> {
    if payload.len() < 77 { return None; }

    let n12 = bits::read_be(payload, 0, 12) as u16;
    let n58 = bits::read_be(payload, 12, 58) as u64;
    let iflip = bits::read_be(payload, 70, 1) as u8;
    let nrpt = bits::read_be(payload, 71, 2) as u8;
    let icq = bits::read_be(payload, 73, 1) as u8;

    let compound = decode_compound_call(n58);
    let ack = match nrpt {
        0 => NonstandardAck::None,
        1 => NonstandardAck::Rrr,
        2 => NonstandardAck::Rr73,
        3 => NonstandardAck::Bare73,
        _ => unreachable!(),
    };

    // Try to resolve the hash. WSJT-X displays unresolved hashes as
    // "<...>" - we do the same.
    let resolved = recent_calls
        .and_then(|calls| lookup_hash12_in_calls(n12, calls));
    let hashed_display = match resolved {
        Some(call) => format!("<{}>", call),
        None => "<...>".to_string(),
    };

    let hash_is_from_call = iflip == 1;
    let is_cq = icq == 1;

    let text = if is_cq {
        format!("CQ {}", compound)
    } else {
        let (call_1, call_2) = if hash_is_from_call {
            // hash is FROM, compound is TO
            (compound.clone(), hashed_display.clone())
        } else {
            // hash is TO (left), compound is FROM (right)
            (hashed_display.clone(), compound.clone())
        };
        let suffix = match ack {
            NonstandardAck::None => "",
            NonstandardAck::Rrr => " RRR",
            NonstandardAck::Rr73 => " RR73",
            NonstandardAck::Bare73 => " 73",
        };
        format!("{} {}{}", call_1, call_2, suffix)
    };

    Some(NonstandardMessage {
        text,
        hash12: n12,
        compound_call: compound,
        ack,
        is_cq,
        hash_is_from_call,
    })
}

/// Pack a nonstandard-call message into a 77-bit payload.
///
/// Inputs:
///   `n12`     - 12-bit hash of the partner callsign
///   `compound`- the up-to-11-char compound callsign (e.g. "PJ4/KA1ABC")
///   `iflip`   - 0 if hash is the TO-call (left), 1 if FROM-call (right)
///   `nrpt`    - 0=no ack, 1=RRR, 2=RR73, 3=73
///   `icq`     - 1 for "CQ <call>" form, 0 otherwise
pub fn pack_nonstandard_full(
    n12: u16,
    compound: &str,
    iflip: u8,
    nrpt: u8,
    icq: u8,
) -> [u8; 77] {
    // Encode compound call to 58-bit base-38
    let mut padded = [b' '; 11];
    let upper = compound.to_ascii_uppercase();
    let bytes = upper.as_bytes();
    let n = bytes.len().min(11);
    // Right-pad with spaces (the Fortran loop reads chars from index 11..1
    // and the trim/adjustl removes trailing spaces afterward; so left-justify
    // padded to 11 with trailing spaces).
    for i in 0..n {
        padded[i] = bytes[i];
    }
    let mut n58: u64 = 0;
    for &c in &padded {
        let digit = COMPOUND_ALPHABET.iter().position(|&a| a == c).unwrap_or(0);
        n58 = n58 * 38 + digit as u64;
    }

    let mut out = [0u8; 77];
    // Pack n12 (12 bits MSB-first)
    for i in 0..12 {
        out[i] = ((n12 >> (11 - i)) & 1) as u8;
    }
    // Pack n58 (58 bits MSB-first) starting at bit 12
    for i in 0..58 {
        out[12 + i] = ((n58 >> (57 - i)) & 1) as u8;
    }
    // Single bit fields
    out[70] = iflip & 1;
    out[71] = (nrpt >> 1) & 1;
    out[72] = nrpt & 1;
    out[73] = icq & 1;
    // i3 = 4 = 0b100 in bits 74..77
    out[74] = 1;
    out[75] = 0;
    out[76] = 0;
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn alphabet_size() {
        assert_eq!(COMPOUND_ALPHABET.len(), 38);
    }

    #[test]
    fn decode_compound_round_trip() {
        // Encode "PJ4/KA1ABC", decode it back
        let upper = "PJ4/KA1ABC";
        let mut padded = [b' '; 11];
        let bytes = upper.as_bytes();
        for i in 0..bytes.len() {
            padded[i] = bytes[i];
        }
        let mut n58: u64 = 0;
        for &c in &padded {
            let digit = COMPOUND_ALPHABET.iter().position(|&a| a == c).unwrap_or(0);
            n58 = n58 * 38 + digit as u64;
        }
        let decoded = decode_compound_call(n58);
        assert_eq!(decoded, "PJ4/KA1ABC");
    }

    #[test]
    fn round_trip_call_with_rr73() {
        // Hash = some arbitrary 12-bit value
        let n12 = 1234u16;
        let payload = pack_nonstandard_full(n12, "PJ4/KA1ABC", 1, 2, 0);
        let msg = unpack_nonstandard(&payload, None).expect("should unpack");
        assert_eq!(msg.hash12, n12);
        assert_eq!(msg.compound_call, "PJ4/KA1ABC");
        assert_eq!(msg.ack, NonstandardAck::Rr73);
        assert!(!msg.is_cq);
        assert!(msg.hash_is_from_call);
        // iflip=1 -> hash is FROM call -> compound is TO call
        // Text format: "<TO> <FROM> RR73" with hash on the right
        assert_eq!(msg.text, "PJ4/KA1ABC <...> RR73");
    }

    #[test]
    fn round_trip_cq_call() {
        let payload = pack_nonstandard_full(0, "PJ4/KA1ABC", 0, 0, 1);
        let msg = unpack_nonstandard(&payload, None).unwrap();
        assert!(msg.is_cq);
        assert_eq!(msg.text, "CQ PJ4/KA1ABC");
    }

    #[test]
    fn round_trip_resolved_hash() {
        // We cross-validated that hash12("K1JT", "") = something specific.
        // Let's just test that lookup works.
        let test_call = "K1JT";
        let h = crate::jenkins::hash12(
            &crate::jenkins::format_call_pair(test_call, "")
        );
        let recent = vec![test_call.to_string(), "WA4CQG".to_string()];
        let payload = pack_nonstandard_full(h, "PJ4/KA1ABC", 0, 1, 0);
        let msg = unpack_nonstandard(&payload, Some(&recent)).unwrap();
        // iflip=0 -> hash is TO call, displayed first
        assert_eq!(msg.text, "<K1JT> PJ4/KA1ABC RRR");
    }

    #[test]
    fn unresolved_hash_shows_placeholder() {
        let payload = pack_nonstandard_full(9999, "VK9X/G4ABC", 0, 0, 0);
        let msg = unpack_nonstandard(&payload, None).unwrap();
        assert_eq!(msg.text, "<...> VK9X/G4ABC");
    }
}
