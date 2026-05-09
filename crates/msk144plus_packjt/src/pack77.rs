// crates/msk144plus_packjt/src/pack77.rs
//
// Top-level pack77 text dispatcher. Takes a human-readable message
// string and packs it into a 77-bit payload, picking the right
// message type (i3=1/2/4 or i3=0/n3=0).
//
// Mirrors the dispatch order of WSJT-X's pack77 in lib/77bit/packjt77.f90:
//   1. Try i3=4 nonstandard (when message contains <angle brackets>)
//   2. Try i3=1 (NA VHF) standard message
//   3. Try i3=2 (EU VHF) standard message
//   4. Fall back to i3=0/n3=0 free text

use crate::callsign::{pack28_standard, Call};
use crate::free_text::pack_free_text;
use crate::grid::{
    decode_exchange as decode_grid_exchange, encode_ack, encode_grid, encode_report,
    Ack, Exchange,
};
use crate::nonstandard::{COMPOUND_ALPHABET, pack_nonstandard_full};
use crate::pack_standard::pack_standard;
use crate::{MsgVariant, StandardMessage};

/// Pack a message text string into a 77-bit payload.
///
/// Returns the 77 bits with i3 in bits 74..77 and the message in bits 0..74.
pub fn pack77(text: &str) -> [u8; 77] {
    let normalised = normalise_message(text);

    // Try each packer in order
    if let Some(p) = try_pack_nonstandard(&normalised) { return p; }
    if let Some(p) = try_pack_standard(&normalised, MsgVariant::NaVhf) { return p; }
    if let Some(p) = try_pack_standard(&normalised, MsgVariant::EuVhf) { return p; }

    // Fall back to free text. Must produce a payload with i3=0, n3=0.
    pack_free_text_77(&normalised)
}

/// Normalise a message string: trim, uppercase, collapse multiple spaces.
fn normalise_message(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    let mut prev_space = true;
    for c in text.chars() {
        let cu = c.to_ascii_uppercase();
        if cu.is_whitespace() {
            if !prev_space {
                out.push(' ');
                prev_space = true;
            }
        } else {
            out.push(cu);
            prev_space = false;
        }
    }
    // Trim trailing space
    out.trim_end().to_string()
}

/// Attempt to pack as i3=4 nonstandard. Triggers when the message
/// contains a callsign in `<angle brackets>` (the hashed partner).
fn try_pack_nonstandard(msg: &str) -> Option<[u8; 77]> {
    let lt = msg.find('<')?;
    let gt = msg.find('>')?;
    if gt < lt + 2 { return None; }
    let hashed = &msg[lt + 1..gt];

    // Form is one of:
    //   "<HASHED> COMPOUND_CALL [ack]"      iflip=0, hash is left
    //   "COMPOUND_CALL <HASHED> [ack]"      iflip=1, hash is right
    //   "CQ COMPOUND_CALL"                   icq=1, no hash needed
    // (CQ form is handled by the standard packer if compound fits.)

    let words: Vec<&str> = msg.split_whitespace().collect();
    if words.len() < 2 { return None; }

    let (iflip, compound_idx) = if words[0].starts_with('<') && words[0].ends_with('>') {
        (0u8, 1)
    } else if words.len() >= 2 && words[1].starts_with('<') && words[1].ends_with('>') {
        (1u8, 0)
    } else {
        return None;
    };
    if compound_idx >= words.len() { return None; }
    let compound = words[compound_idx];

    // Validate compound chars match alphabet
    if compound.len() > 11 { return None; }
    for c in compound.bytes() {
        if !COMPOUND_ALPHABET.contains(&c) {
            return None;
        }
    }

    // Determine ack
    let nrpt = if words.len() > compound_idx + 1 {
        match words[words.len() - 1] {
            "RRR" => 1u8,
            "RR73" => 2u8,
            "73" => 3u8,
            _ => return None, // unrecognised trailing token - bail
        }
    } else {
        0u8
    };
    let icq = 0u8;

    // Compute the 12-bit hash from the hashed callsign (single call,
    // not a pair; uses the same Jenkins seed=146 with len=37 buffer).
    let n12 = crate::jenkins::hash12(&crate::jenkins::format_call_pair(hashed, ""));

    Some(pack_nonstandard_full(n12, compound, iflip, nrpt, icq))
}

/// Attempt to pack as a standard i3=1 (NA VHF) or i3=2 (EU VHF) message.
fn try_pack_standard(msg: &str, variant: MsgVariant) -> Option<[u8; 77]> {
    let words: Vec<&str> = msg.split_whitespace().collect();
    if words.len() < 2 || words.len() > 4 { return None; }

    let suffix = match variant {
        MsgVariant::NaVhf => "/R",
        MsgVariant::EuVhf => "/P",
    };

    // call_1 might be "CQ", "CQ XXX", "DE", "QRZ", or a callsign possibly
    // with the rover suffix. Detect:
    let (call_1, call_1_rover, call_2_idx) = parse_first_field(&words, suffix)?;

    // call_2 is words[call_2_idx]
    if call_2_idx >= words.len() { return None; }
    let (call_2, call_2_rover) = parse_callsign_with_rover(words[call_2_idx], suffix);
    let call_2 = match call_2 {
        Call::Standard(_) => call_2,
        _ => return None, // call_2 must be a real callsign
    };

    // Exchange may be present at words[call_2_idx+1] possibly preceded
    // by a standalone "R" token (for "R FN42" grid form), OR the token
    // itself may start with 'R' followed by a signal report ("R+05",
    // "R-03", "RR73").
    let mut exch_idx = call_2_idx + 1;
    let mut roger = false;
    if exch_idx < words.len() {
        let tok = words[exch_idx];
        if tok == "R" {
            // Standalone "R" - next token is the exchange (grid usually)
            roger = true;
            exch_idx += 1;
        } else if tok.starts_with('R') && tok.len() >= 3 {
            // "R+05" or "R-03" - report with attached R prefix
            // (but NOT "RRR" or "RR73" which are bare ack tokens - those
            // start with R but don't have the +/- pattern).
            let rest = &tok[1..];
            if rest.starts_with('+') || rest.starts_with('-') {
                if let Ok(_) = rest.parse::<i8>() {
                    roger = true;
                    // Replace the token in our local view with just the
                    // signed report part.
                    return try_pack_standard_with_r_report(
                        words, call_1.clone(), call_1_rover, call_2.clone(),
                        call_2_rover, rest, variant,
                    );
                }
            }
        }
    }

    let exchange = if exch_idx < words.len() {
        parse_exchange(words[exch_idx])?
    } else {
        return None;
    };

    let std_msg = StandardMessage {
        call_1,
        call_1_rover,
        call_2,
        call_2_rover,
        roger,
        exchange,
        variant,
    };
    pack_standard(&std_msg).ok()
}

/// Helper: pack with a pre-parsed R-prefixed signal report
/// (e.g. "+05" already stripped of its leading 'R').
fn try_pack_standard_with_r_report(
    _words: Vec<&str>,
    call_1: Call,
    call_1_rover: bool,
    call_2: Call,
    call_2_rover: bool,
    report_str: &str,
    variant: MsgVariant,
) -> Option<[u8; 77]> {
    let snr = report_str.parse::<i8>().ok()?;
    if snr < -30 || snr > 30 { return None; }
    let std_msg = StandardMessage {
        call_1,
        call_1_rover,
        call_2,
        call_2_rover,
        roger: true,
        exchange: Exchange::Report(snr),
        variant,
    };
    pack_standard(&std_msg).ok()
}

/// Parse the first field, returning (Call, rover_flag, index_of_call_2).
fn parse_first_field(words: &[&str], rover_suffix: &str) -> Option<(Call, bool, usize)> {
    if words[0] == "CQ" {
        // "CQ K1ABC" or "CQ DX K1ABC" or "CQ NNN K1ABC" or "CQ AAAA K1ABC"
        if words.len() < 2 { return None; }
        if words.len() >= 3 && words[1].len() <= 4 {
            // Could be CQ_NNN or CQ_AAAA prefix
            if let Ok(n) = words[1].parse::<u16>() {
                if n <= 999 { return Some((Call::CqNum(n), false, 2)); }
            }
            // CQ_AAAA: 1-4 letters
            if words[1].chars().all(|c| c.is_ascii_alphabetic()) {
                return Some((Call::CqLetters(words[1].to_string()), false, 2));
            }
            // Otherwise "DX" or other 2-3 letter prefix - treat as letter form
            if words[1].chars().all(|c| c.is_ascii_alphabetic()) {
                return Some((Call::CqLetters(words[1].to_string()), false, 2));
            }
        }
        // Plain "CQ K1ABC ..." - words[1] is call_2, but call_1 is bare CQ
        return Some((Call::Cq, false, 1));
    }
    if words[0] == "DE" {
        return Some((Call::De, false, 1));
    }
    if words[0] == "QRZ" {
        return Some((Call::Qrz, false, 1));
    }

    // Plain callsign (possibly with rover)
    let (call, rover) = parse_callsign_with_rover(words[0], rover_suffix);
    match call {
        Call::Standard(_) => Some((call, rover, 1)),
        _ => None,
    }
}

/// Strip rover suffix and return (Call, had_suffix).
fn parse_callsign_with_rover(s: &str, suffix: &str) -> (Call, bool) {
    if let Some(stripped) = s.strip_suffix(suffix) {
        return (Call::Standard(stripped.to_string()), true);
    }
    (Call::Standard(s.to_string()), false)
}

/// Parse an exchange field (grid, signal report, or ack token).
fn parse_exchange(s: &str) -> Option<Exchange> {
    // Ack tokens
    match s {
        "RRR" => return Some(Exchange::Acknowledgement(Ack::Rrr)),
        "RR73" => return Some(Exchange::Acknowledgement(Ack::Rr73)),
        "73" => return Some(Exchange::Acknowledgement(Ack::Bare73)),
        _ => {}
    }
    // Signal report: starts with + or -
    if s.starts_with('+') || s.starts_with('-') {
        if let Ok(snr) = s.parse::<i8>() {
            if snr >= -30 && snr <= 30 {
                return Some(Exchange::Report(snr));
            }
        }
        return None;
    }
    // Otherwise assume grid (4 chars: 2 letters + 2 digits)
    if s.len() == 4 {
        let bs = s.as_bytes();
        if bs[0].is_ascii_alphabetic() && bs[1].is_ascii_alphabetic()
            && bs[2].is_ascii_digit() && bs[3].is_ascii_digit() {
            return Some(Exchange::Grid(s.to_string()));
        }
    }
    None
}

/// Pack as i3=0/n3=0 free text. The 71 message bits hold the text;
/// bits 71..77 hold n3=0 (3 bits) and i3=0 (3 bits) - all zero, so the
/// only thing we set is the 71-bit text payload.
fn pack_free_text_77(text: &str) -> [u8; 77] {
    let bits71 = pack_free_text(text);
    let mut out = [0u8; 77];
    out[..71].copy_from_slice(&bits71);
    // n3=0, i3=0 means bits 71..77 are all zeros - already 0 from init.
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{unpack77, Message};

    fn round_trip(text: &str, expected_text: &str) {
        let bits = pack77(text);
        let msg = unpack77(&bits);
        assert_eq!(msg.to_text(), expected_text,
            "input '{}' packed/unpacked to wrong text", text);
    }

    #[test]
    fn standard_cq_grid() {
        round_trip("CQ K1ABC FN42", "CQ K1ABC FN42");
    }

    #[test]
    fn standard_call_call_grid() {
        round_trip("W9XYZ K1ABC EM72", "W9XYZ K1ABC EM72");
    }

    #[test]
    fn standard_call_call_signal_report() {
        round_trip("W9XYZ K1ABC -03", "W9XYZ K1ABC -03");
    }

    #[test]
    fn standard_call_call_r_report() {
        round_trip("W9XYZ K1ABC R+05", "W9XYZ K1ABC R+05");
    }

    #[test]
    fn standard_call_call_rrr() {
        round_trip("W9XYZ K1ABC RRR", "W9XYZ K1ABC RRR");
    }

    #[test]
    fn standard_call_call_73() {
        round_trip("W9XYZ K1ABC 73", "W9XYZ K1ABC 73");
    }

    #[test]
    fn free_text_fallback() {
        // Random text that doesn't match any structured form should
        // fall through to free text.
        let bits = pack77("HELLO WORLD");
        match unpack77(&bits) {
            Message::FreeText { text } => {
                // text is right-justified to 13 chars (left-padded with
                // spaces); trim() to compare. to_text() does this for us.
                assert_eq!(text.trim(), "HELLO WORLD");
            }
            other => panic!("expected FreeText, got {:?}", other),
        }
    }

    #[test]
    fn nonstandard_with_hashed_partner() {
        // "<K1JT> PJ4/KA1ABC RR73"
        let bits = pack77("<K1JT> PJ4/KA1ABC RR73");
        let msg = unpack77(&bits);
        // Without recent_calls the partner shows as <...>; the compound
        // should be PJ4/KA1ABC.
        match msg {
            Message::Nonstandard(ns) => {
                assert_eq!(ns.compound_call, "PJ4/KA1ABC");
                assert_eq!(ns.ack, crate::nonstandard::NonstandardAck::Rr73);
            }
            other => panic!("expected Nonstandard, got {:?}", other),
        }
    }

    #[test]
    fn case_normalised() {
        // Lowercase input gets uppercased
        let bits = pack77("cq k1abc fn42");
        match unpack77(&bits) {
            Message::Standard(_) => {}
            other => panic!("expected Standard, got {:?}", other),
        }
    }
}
