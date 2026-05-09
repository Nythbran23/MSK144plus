// crates/msk144plus_packjt/src/pack_msk40.rs
//
// MSK40 short-message packer. Faithful port of MSHV's `genmsk40` /
// WSJT-X's `genmsk40.f90` packer (just the bit assembly half — the LDPC
// encode and channel-bit synthesis live elsewhere).
//
// MSK40 carries a 12-bit hash of the callsign pair "<MYCALL HISCALL>"
// plus a 4-bit report code. The receiver, knowing both calls, can hash
// them locally and check for a match — that's how a 16-bit message
// reconstructs a full meaningful exchange. Without knowing both calls
// in advance the receiver cannot decode the message at all.
//
// On the air, MSK40 is dramatically shorter than MSK144 (40 bits vs
// 144 bits per frame), giving ~3.6× more transmissions per slot. The
// SNR advantage from the higher repetition rate is what makes Sh msg
// useful in marginal conditions.
//
// Wire layout of the 16-bit message (named `ig` in the Fortran):
//   bits 0..4   : irpt  (4-bit report code, indexes RPT_TABLE)
//   bits 4..16  : ihash (12-bit hash of "<C1 C2>" call pair)
// Equivalent: ig = 16*ihash + irpt.

use crate::jenkins::{format_call_pair, hash12};

/// 16-entry MSK40 report table. Source: msk40decodeframe.f90 lines 27-29
/// and MSHV's `config_rpt_msk40.h`. Index into this table is `irpt`,
/// the 4-bit message-type code transmitted in MSK40.
///
/// Note that these aren't all the SNR-style reports you might expect.
/// The MSK40 message type doubles as the QSO-state indicator: codes 0-6
/// are SNR reports for the initial exchange phase ("+05" etc), 7-13 are
/// R-prefixed SNR reports (R+05, etc) for the RReport phase, 14 is RRR,
/// 15 is 73. So the receiver gets the message type AND the QSO progress
/// stage in one 4-bit field.
pub const RPT_TABLE: [&str; 16] = [
    "-03", "+00", "+03", "+06", "+10", "+13", "+16",
    "R-03", "R+00", "R+03", "R+06", "R+10", "R+13", "R+16",
    "RRR", "73",
];

/// Errors from `pack_msk40`. The packer is strict — bad input is
/// rejected rather than fudged into something plausible — because the
/// receiver also rejects mismatches strictly, and silently producing
/// an invalid packet would just confuse the QSO state machine.
#[derive(Debug, Clone, PartialEq)]
pub enum PackMsk40Error {
    /// Message did not match the expected `"<C1 C2> RPT"` shape. The
    /// angle brackets are required (and are how the encoder routes
    /// MSK40 vs MSK144 in `encode_message_to_audio`).
    BadFormat(String),
    /// The report token wasn't one of the 16 entries in `RPT_TABLE`.
    /// MSK40 cannot carry arbitrary text reports — only the predefined
    /// 4-bit codes.
    UnknownReport(String),
}

impl std::fmt::Display for PackMsk40Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PackMsk40Error::BadFormat(s) =>
                write!(f, "MSK40 bad format (expected '<C1 C2> RPT'): {:?}", s),
            PackMsk40Error::UnknownReport(s) =>
                write!(f, "MSK40 unknown report code (must be one of the 16 \
                          predefined values): {:?}", s),
        }
    }
}

impl std::error::Error for PackMsk40Error {}

/// Pack a bracket-form message text into the 16 message bits used by
/// the MSK40 (32,16) LDPC encoder.
///
/// Input format: `"<MYCALL HISCALL> RPT"` where RPT is one of the 16
/// strings in `RPT_TABLE`. The angle brackets are mandatory; without
/// them, MSK40 isn't applicable (the callsigns must be hashed together,
/// not transmitted explicitly).
///
/// Returns 16 bits in LSB-first order: `out[0]` is bit 0 of `ig`,
/// `out[15]` is bit 15. This matches the layout that `encode_short`
/// expects (the LDPC generator matrix is keyed to LSB-first input).
pub fn pack_msk40(text: &str) -> Result<[u8; 16], PackMsk40Error> {
    let trimmed = text.trim();
    let lt = trimmed.find('<')
        .ok_or_else(|| PackMsk40Error::BadFormat(text.to_string()))?;
    let gt = trimmed.find('>')
        .ok_or_else(|| PackMsk40Error::BadFormat(text.to_string()))?;
    if gt < lt + 2 {
        return Err(PackMsk40Error::BadFormat(text.to_string()));
    }
    // Inside-brackets: "C1 C2"
    let inside = &trimmed[lt + 1..gt];
    let pair_parts: Vec<&str> = inside.split_whitespace().collect();
    if pair_parts.len() != 2 {
        return Err(PackMsk40Error::BadFormat(text.to_string()));
    }
    let c1 = pair_parts[0];
    let c2 = pair_parts[1];

    // After '>' we expect whitespace then the report token.
    let after_close = trimmed[gt + 1..].trim();
    if after_close.is_empty() {
        return Err(PackMsk40Error::BadFormat(text.to_string()));
    }
    // The report is a single whitespace-separated token.
    let rpt_str = after_close.split_whitespace().next().unwrap_or("");

    let irpt = RPT_TABLE.iter().position(|&r| r == rpt_str)
        .ok_or_else(|| PackMsk40Error::UnknownReport(rpt_str.to_string()))?;

    let pair = format_call_pair(c1, c2);
    let ihash = hash12(&pair) as u32;

    // ig = 16 * ihash + irpt (per genmsk40.f90)
    let ig: u32 = (ihash << 4) | (irpt as u32 & 0xF);

    let mut out = [0u8; 16];
    for i in 0..16 {
        out[i] = ((ig >> i) & 1) as u8;
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rpt_table_size() {
        assert_eq!(RPT_TABLE.len(), 16);
    }

    #[test]
    fn pack_basic_73() {
        // Matches the RX side: "<MYCALL HISCALL> 73" packs the hash of
        // the call pair plus irpt=15 (index of "73" in RPT_TABLE).
        let bits = pack_msk40("<GW4WND F1ABC> 73").expect("pack");
        // irpt = 15 = 0b1111 → bits 0..4 should be 1,1,1,1
        assert_eq!(bits[0], 1);
        assert_eq!(bits[1], 1);
        assert_eq!(bits[2], 1);
        assert_eq!(bits[3], 1);
        // Remaining 12 bits = hash; just sanity-check it's not all zero.
        let hash_bits: u32 = (4..16).fold(0u32,
            |acc, i| acc | ((bits[i] as u32) << (i - 4)));
        assert_ne!(hash_bits, 0);
    }

    #[test]
    fn pack_rrr() {
        let bits = pack_msk40("<GW4WND F1ABC> RRR").expect("pack");
        // RRR is index 14 = 0b1110 → bits 0..4 = 0,1,1,1
        assert_eq!(bits[0], 0);
        assert_eq!(bits[1], 1);
        assert_eq!(bits[2], 1);
        assert_eq!(bits[3], 1);
    }

    #[test]
    fn pack_r_plus_06() {
        let bits = pack_msk40("<GW4WND F1ABC> R+06").expect("pack");
        // R+06 is index 10 = 0b1010 → bits 0..4 = 0,1,0,1
        assert_eq!(bits[0], 0);
        assert_eq!(bits[1], 1);
        assert_eq!(bits[2], 0);
        assert_eq!(bits[3], 1);
    }

    #[test]
    fn pack_hash_asymmetric_by_design() {
        // The MSK40 protocol embeds hash of "<C1 C2>" with C1=sender,
        // C2=receiver. Reversing call order is INTENTIONAL — it lets
        // the receiver distinguish "from me to him" vs "from him to me"
        // by computing both orderings locally and matching either. So
        // pack_msk40 should produce DIFFERENT hashes for reversed input.
        let a = pack_msk40("<GW4WND F1ABC> 73").expect("a");
        let b = pack_msk40("<F1ABC GW4WND> 73").expect("b");
        // Report bits (0..4) match (same RPT in both messages).
        for i in 0..4 {
            assert_eq!(a[i], b[i], "report bit {} should match", i);
        }
        // Hash bits (4..16) differ — at least one must differ.
        let any_differ = (4..16).any(|i| a[i] != b[i]);
        assert!(any_differ,
            "hash should differ for reversed call ordering (asymmetric by design)");
    }

    #[test]
    fn pack_rejects_no_brackets() {
        assert!(matches!(
            pack_msk40("GW4WND F1ABC 73"),
            Err(PackMsk40Error::BadFormat(_))));
    }

    #[test]
    fn pack_rejects_empty_pair() {
        assert!(matches!(
            pack_msk40("<> 73"),
            Err(PackMsk40Error::BadFormat(_))));
    }

    #[test]
    fn pack_rejects_unknown_report() {
        assert!(matches!(
            pack_msk40("<GW4WND F1ABC> +07"),  // not in RPT_TABLE
            Err(PackMsk40Error::UnknownReport(_))));
    }

    #[test]
    fn pack_rejects_three_calls() {
        assert!(matches!(
            pack_msk40("<A B C> 73"),
            Err(PackMsk40Error::BadFormat(_))));
    }
}
