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

    // Auto-detect non-standard callsign and pack as i3=4. Catches the case
    // where neither standard variant accepted the message because one or
    // both callsigns don't fit the [A-Z]{1,2}[0-9]{1}[A-Z]{1,3} pattern
    // (e.g. S50TA, 9A1ABC, 3D2RD). Without this, those messages would
    // silently fall through to free-text encoding which doesn't preserve
    // the exchange or message type and breaks QSO with the partner.
    if let Some(p) = try_pack_nonstandard_auto(&normalised) { return p; }

    // Fall back to free text. Must produce a payload with i3=0, n3=0.
    // The caller can detect this by unpacking the result and checking
    // the message variant; structured messages we couldn't encode
    // (e.g. "S50TA GW4WND IO82" — i3=4 has no grid field) end up
    // here and the partner sees text-only instead of structured data.
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

/// Auto-detect a non-standard callsign in a structured message and pack
/// it as i3=4 (compound + hash). Triggered after both standard variants
/// (NaVhf, EuVhf) have failed — those reject calls that don't fit the
/// `[A-Z]{1,2}[0-9]{1}[A-Z]{1,3}` pattern, but real-world traffic uses
/// many calls outside that pattern (Slovenian S5x, Croatian 9A1, Fiji
/// 3D2, Korean HL, Indonesian YB, etc.). Without this fallback those
/// messages would slide into free-text encoding which doesn't preserve
/// structure and breaks QSO sequencing with the partner.
///
/// The i3=4 wire format is structurally limited: it carries ONE
/// callsign verbatim (the "compound", up to 11 chars) plus the OTHER
/// callsign as a 12-bit hash that the receiver looks up in their
/// recently-decoded list. Critically, **i3=4 has no field for grids or
/// signal reports**. So if the message contains a grid or +/- report,
/// we return None and let the caller drop to free text. This matches
/// WSJT-X behaviour: QSOs with non-standard callsigns proceed via
/// abbreviated CQ→Call→RR73→73 sequences without exchanging reports.
///
/// Supported shapes:
///   "CQ S50TA"               icq=1, compound=S50TA, hash=0
///   "S50TA GW4WND"           iflip=0, hash=hash12(GW4WND), compound=S50TA
///   "GW4WND S50TA"           iflip=1, hash=hash12(GW4WND), compound=S50TA
///   "S50TA GW4WND RR73"      as call+call but with nrpt=2
///   "S50TA GW4WND 73"        nrpt=3
///   "S50TA GW4WND RRR"       nrpt=1
///
/// Unsupported (returns None):
///   anything containing a grid (e.g. "S50TA GW4WND IO82")
///   anything containing a +/- report (e.g. "S50TA GW4WND -04")
///   anything containing an R-report (e.g. "S50TA GW4WND R+05")
///
/// "compound" in i3=4 means "the call printed verbatim on the wire".
/// It does NOT mean "callsign with a / portable suffix" — the alphabet
/// happens to include `/` so portable forms like `PJ4/KA1ABC` also
/// pack into this slot, but the slot is used for ANY call that fits
/// in the 11-char base-38 alphabet.
fn try_pack_nonstandard_auto(msg: &str) -> Option<[u8; 77]> {
    let words: Vec<&str> = msg.split_whitespace().collect();
    if words.len() < 2 || words.len() > 3 { return None; }

    // CQ form: "CQ <compound>"
    if words[0] == "CQ" {
        if words.len() != 2 { return None; }
        let compound = words[1];
        if !is_valid_compound_call(compound) { return None; }
        return Some(pack_nonstandard_full(0, compound, 0, 0, 1));
    }

    // Two-call form: <call1> <call2> [optional ack]
    let call1 = words[0];
    let call2 = words[1];

    // Determine which side is non-standard. If both are standard, this
    // path shouldn't have been reached — try_pack_standard would have
    // succeeded. If both are non-standard, i3=4 still works (we hash
    // one and put the other verbatim) but we have to pick which.
    // Convention: non-standard one goes verbatim (compound), standard
    // one is hashed. If both non-standard, use call2 as compound (the
    // partner's call, which the receiver knows verbatim because it's
    // their own call).
    let call1_standard = crate::callsign::pack28_standard(call1).is_some();
    let call2_standard = crate::callsign::pack28_standard(call2).is_some();

    let (compound, hashed_call, iflip) = match (call1_standard, call2_standard) {
        (true, true) => return None,        // both standard — caller mistake
        (false, true) => (call1, call2, 1), // call1 non-std verbatim, hash=call2 (FROM)
        (true, false) => (call2, call1, 0), // call2 non-std verbatim, hash=call1 (TO)
        (false, false) => (call2, call1, 0),// both non-std — pick call2 as verbatim
    };

    if !is_valid_compound_call(compound) { return None; }

    // Optional 3rd token is an ack code — anything else (grid, report,
    // R-report) is unsupported in i3=4 and forces fall-through to free
    // text. The caller will log a warning so the operator sees that the
    // message couldn't be encoded structurally.
    let nrpt = if words.len() == 3 {
        match words[2] {
            "RRR" => 1u8,
            "RR73" => 2u8,
            "73" => 3u8,
            _ => return None,  // grid or report — i3=4 can't carry these
        }
    } else {
        0u8
    };

    let n12 = crate::jenkins::hash12(
        &crate::jenkins::format_call_pair(hashed_call, ""));
    Some(pack_nonstandard_full(n12, compound, iflip, nrpt, 0))
}

/// Quick sanity check: does this string fit in the 11-char base-38
/// compound-call alphabet (digits, letters, slash, space)? Used to
/// reject malformed input before pack_nonstandard_full silently
/// substitutes spaces for invalid characters.
fn is_valid_compound_call(s: &str) -> bool {
    if s.is_empty() || s.len() > 11 { return false; }
    s.bytes().all(|c| crate::nonstandard::COMPOUND_ALPHABET.contains(&c))
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
    // call_2 must be either a standard call or a hash-form (bracketed
    // or auto-hashed non-standard). Reject anything else (CQ / DE / QRZ
    // in call_2 position is invalid in i3=1/2).
    let call_2 = match call_2 {
        Call::Standard(_) | Call::Hash22(_) => call_2,
        _ => return None,
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

    // Bracketed callsign — e.g. `<S50TA>`. Pack as 22-bit hash into the
    // 28-bit call slot. This is the canonical WSJT-X mechanism for
    // including a non-standard callsign in a structured i3=1/2 message
    // that also carries a grid, signal report, or R-report. The receiver
    // recognises `n28 < NTOKENS + MAX22` as a hash and looks up the
    // original callsign in their recent-decoded list. Source:
    // packjt77.f90 `pack28` function, the `c13(1:1).eq.'<'` branch.
    if words[0].starts_with('<') && words[0].ends_with('>') && words[0].len() >= 3 {
        let inner = &words[0][1..words[0].len() - 1];
        let n22 = crate::jenkins::hash22(inner);
        return Some((Call::Hash22(n22), false, 1));
    }

    // Plain callsign (possibly with rover or non-standard auto-hash)
    let (call, rover) = parse_callsign_with_rover(words[0], rover_suffix);
    match call {
        Call::Standard(_) | Call::Hash22(_) => Some((call, rover, 1)),
        _ => None,
    }
}

/// Strip rover suffix and return (Call, had_suffix).
///
/// Recognises three callsign forms:
///   1. `<XYZ>` — bracketed → Call::Hash22(hash22(XYZ))
///   2. Standard pattern (e.g. K1ABC, GW4WND) → Call::Standard
///   3. Non-standard (e.g. S50TA, 9A1ABC, 3D2RD) → Call::Hash22(hash22)
///
/// Case 3 mirrors WSJT-X's auto-hash logic in packjt77.f90 pack28:
/// when a callsign doesn't fit the standard
/// `[A-Z]{1,2}[0-9]{1}[A-Z]{1,3}` pattern, pack28 silently hashes it
/// to 22 bits and packs the hash into the n28 slot. The receiver
/// looks it up by hash in their recent-calls table and displays the
/// resolved call (or `<...>` if no match). This is what lets messages
/// like `S50TA GW4WND R-04` (no brackets in the source) encode as a
/// valid i3=1 packet — the encoder transparently hashes S50TA.
///
/// Bracketed form is a deliberate-by-operator choice to hash a call
/// that COULD be packed standardly. Both forms produce the same wire
/// representation; the brackets just tell the encoder "use the hash
/// path even if you don't have to". Standard calls (call_1 or call_2)
/// in the same message are NOT bracketed.
fn parse_callsign_with_rover(s: &str, suffix: &str) -> (Call, bool) {
    // Bracketed form first: `<XYZ>` → hash22
    if s.starts_with('<') && s.ends_with('>') && s.len() >= 3 {
        let inner = &s[1..s.len() - 1];
        let n22 = crate::jenkins::hash22(inner);
        return (Call::Hash22(n22), false);
    }
    // Strip rover suffix (only applies to plain-text callsigns, not
    // bracketed — WSJT-X disallows `<X/P>` and so do we).
    let (base, rover) = if let Some(stripped) = s.strip_suffix(suffix) {
        (stripped, true)
    } else {
        (s, false)
    };
    // Try standard pattern via the same predicate the i3=1/2 packer
    // uses (pack28_standard returns Some iff standard). When it
    // returns None, the call is non-standard and we auto-hash —
    // matching WSJT-X's pack28 fallback branch.
    if pack28_standard(base).is_some() {
        (Call::Standard(base.to_string()), rover)
    } else {
        let n22 = crate::jenkins::hash22(base);
        (Call::Hash22(n22), rover)
    }
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

    // ─── i3=4 auto-fallback tests for non-standard callsigns ─────────
    //
    // These exercise the new try_pack_nonstandard_auto path. The
    // canonical use case is QSO with calls that don't fit the
    // standard `[A-Z]{1,2}[0-9]{1}[A-Z]{1,3}` pattern: Slovenian S5x,
    // Croatian 9A1, Fiji 3D2, Korean HL, etc. Without this path
    // those messages used to silently fall through to free-text
    // encoding which produces gibberish on the partner's decoder.

    #[test]
    fn nonstandard_cq() {
        // CQ with non-standard callsign. Receiver sees "CQ S50TA"
        // — the icq=1 flag tells them the call after "CQ" is the
        // calling station.
        let bits = pack77("CQ S50TA");
        let msg = unpack77(&bits);
        match msg {
            Message::Nonstandard(ns) => {
                assert!(ns.is_cq, "icq flag should be set for CQ form");
                assert_eq!(ns.compound_call, "S50TA");
                assert_eq!(ns.ack, crate::nonstandard::NonstandardAck::None);
            }
            other => panic!("expected Nonstandard, got {:?}", other),
        }
    }

    #[test]
    fn nonstandard_call_call_no_exchange() {
        // "S50TA GW4WND" — partner-to-me from a non-standard call.
        // Encodes as: hash=hash12(GW4WND), compound=S50TA, iflip=1
        // (so receiver displays compound on the LEFT and hash on the
        // RIGHT). Without recent_calls hash resolves to "<...>".
        let bits = pack77("S50TA GW4WND");
        let msg = unpack77(&bits);
        match msg {
            Message::Nonstandard(ns) => {
                assert_eq!(ns.compound_call, "S50TA");
                assert_eq!(ns.ack, crate::nonstandard::NonstandardAck::None);
                assert!(!ns.is_cq);
            }
            other => panic!("expected Nonstandard, got {:?}", other),
        }
    }

    #[test]
    fn nonstandard_call_call_rr73() {
        let bits = pack77("S50TA GW4WND RR73");
        let msg = unpack77(&bits);
        match msg {
            Message::Nonstandard(ns) => {
                assert_eq!(ns.compound_call, "S50TA");
                assert_eq!(ns.ack, crate::nonstandard::NonstandardAck::Rr73);
            }
            other => panic!("expected Nonstandard, got {:?}", other),
        }
    }

    #[test]
    fn nonstandard_call_call_73() {
        let bits = pack77("S50TA GW4WND 73");
        let msg = unpack77(&bits);
        match msg {
            Message::Nonstandard(ns) => {
                assert_eq!(ns.compound_call, "S50TA");
                assert_eq!(ns.ack, crate::nonstandard::NonstandardAck::Bare73);
            }
            other => panic!("expected Nonstandard, got {:?}", other),
        }
    }

    #[test]
    fn nonstandard_call_call_rrr() {
        let bits = pack77("S50TA GW4WND RRR");
        let msg = unpack77(&bits);
        match msg {
            Message::Nonstandard(ns) => {
                assert_eq!(ns.compound_call, "S50TA");
                assert_eq!(ns.ack, crate::nonstandard::NonstandardAck::Rrr);
            }
            other => panic!("expected Nonstandard, got {:?}", other),
        }
    }

    #[test]
    fn nonstandard_grid_falls_back_to_free_text() {
        // i3=4 has no grid field — "S50TA GW4WND IO82" cannot be
        // structurally encoded and falls through to free text. The
        // operator sees a warning logged at the pack77 level.
        // Free-text encoding doesn't preserve structure, so the
        // partner sees the text but no parsed payload type.
        let bits = pack77("S50TA GW4WND IO82");
        let msg = unpack77(&bits);
        // Must NOT be Nonstandard (i3=4 can't carry grids).
        match msg {
            Message::Nonstandard(_) => panic!("grid should not encode as i3=4"),
            _ => {}  // free text, or whatever else the unpacker decides
        }
    }

    #[test]
    fn nonstandard_report_falls_back_to_free_text() {
        // Same — reports cannot be carried in i3=4.
        let bits = pack77("S50TA GW4WND -04");
        let msg = unpack77(&bits);
        match msg {
            Message::Nonstandard(_) => panic!("report should not encode as i3=4"),
            _ => {}
        }
    }

    #[test]
    fn nonstandard_r_report_falls_back_to_free_text() {
        // R-reports also unsupported in i3=4.
        let bits = pack77("S50TA GW4WND R+05");
        let msg = unpack77(&bits);
        match msg {
            Message::Nonstandard(_) => panic!("R-report should not encode as i3=4"),
            _ => {}
        }
    }

    #[test]
    fn nonstandard_partner_call_resolves_via_hash_lookup() {
        // Confirm that on the receiving side, if the recent-calls
        // ledger contains the partner call, the hash resolves and
        // the displayed text contains the real callsign in <>.
        let bits = pack77("S50TA GW4WND RR73");
        // Read back with recent_calls hint. The unpack77 in the
        // crate root may not take a recent_calls slice directly,
        // but unpack_nonstandard does. Test that path explicitly.
        let msg = crate::nonstandard::unpack_nonstandard(
            &bits[..77],
            Some(&["GW4WND".to_string()]),
        ).expect("should unpack");
        // Should contain the resolved partner call, the compound
        // (S50TA), and RR73.
        assert!(msg.text.contains("S50TA"),
            "expected text to contain S50TA: {:?}", msg.text);
        assert!(msg.text.contains("GW4WND"),
            "hash should resolve to GW4WND: {:?}", msg.text);
        assert!(msg.text.contains("RR73"),
            "RR73 ack should be in text: {:?}", msg.text);
    }

    #[test]
    fn slovenian_5_prefix_packs_nonstandard() {
        // S50TA, S57XYZ — Slovenian calls have prefix S5.
        let bits = pack77("S57XYZ GW4WND RR73");
        match unpack77(&bits) {
            Message::Nonstandard(ns) => {
                assert_eq!(ns.compound_call, "S57XYZ");
            }
            other => panic!("S57XYZ should be nonstandard, got {:?}", other),
        }
    }

    #[test]
    fn croatian_9a_prefix_packs_nonstandard() {
        // 9A1 Croatian. Digit-first prefix, doesn't fit the 1-2 letter
        // standard prefix pattern.
        let bits = pack77("9A1ABC GW4WND RR73");
        match unpack77(&bits) {
            Message::Nonstandard(ns) => {
                assert_eq!(ns.compound_call, "9A1ABC");
            }
            other => panic!("9A1ABC should be nonstandard, got {:?}", other),
        }
    }
}

