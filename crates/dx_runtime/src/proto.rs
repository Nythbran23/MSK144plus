// crates/dx_runtime/src/proto.rs
//
// Logical protocol types for the QSO state machine.
//
// VERBATIM PORT from MSK2K src/proto.rs, adapted for MSK144:
//   - Reports are MSK144's signed-dB format (e.g., +05, -02, R+05) instead
//     of MSK2K's OOO/RO shorthand. The state machine logic is identical.
//   - All Payload variants and the render_payload / message_to_payload
//     helpers preserved.

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Format {
    Fmt1,  // Full messages: CQ, calls with reports
    Fmt2,  // Short messages: R-reports, RR, 73
}

#[allow(dead_code)]
#[derive(Debug, Clone, PartialEq)]
pub enum Rendered {
    Text(String),
}

/// Logical payload types for QSO protocol.
#[allow(dead_code)]
#[derive(Debug, Clone, PartialEq)]
pub enum Payload {
    /// CQ call (Format-1, general address)
    Cq {
        from: String,
        grid: Option<String>,
    },

    /// Cold call - calling a station without report (Format-1, private)
    Call {
        from: String,
        to: String,
    },

    /// Call with report (Format-1, private)
    CallWithReport {
        from: String,
        to: String,
        rpt: i16,
    },

    /// Roger + Report (Format-2)
    RReport {
        from: String,
        to: String,
        rpt: i16,
    },

    /// Roger Roger (Format-2)
    Rr {
        from: String,
        to: String,
    },

    /// Combined "rogered + 73" — a single MSK144/FT8/FT4 message that
    /// both acknowledges the partner's R-report AND closes the QSO.
    /// WSJT-X and MSHV both emit this as an alternative to RRR when
    /// the operator wants to compress the final exchange. From the
    /// receiver's perspective it carries the semantics of
    /// (Rr + SeventyThree) — the partner has rogered our R-report
    /// AND said 73 in one transmission. We respond with 73 and close.
    Rr73 {
        from: String,
        to: String,
    },

    /// 73 - end of QSO (Format-2)
    SeventyThree {
        from: String,
        to: String,
    },

    /// Free text (Format-1)
    Text {
        from: String,
        to: Option<String>,
        text: String,
    },
}

#[allow(dead_code)]
impl Payload {
    pub fn from_call(&self) -> &str {
        match self {
            Payload::Cq { from, .. } => from,
            Payload::Call { from, .. } => from,
            Payload::CallWithReport { from, .. } => from,
            Payload::RReport { from, .. } => from,
            Payload::Rr { from, .. } => from,
            Payload::Rr73 { from, .. } => from,
            Payload::SeventyThree { from, .. } => from,
            Payload::Text { from, .. } => from,
        }
    }

    pub fn to_call(&self) -> Option<&str> {
        match self {
            Payload::Cq { .. } => None,
            Payload::Call { to, .. } => Some(to),
            Payload::CallWithReport { to, .. } => Some(to),
            Payload::RReport { to, .. } => Some(to),
            Payload::Rr { to, .. } => Some(to),
            Payload::Rr73 { to, .. } => Some(to),
            Payload::SeventyThree { to, .. } => Some(to),
            Payload::Text { to, .. } => to.as_deref(),
        }
    }

    pub fn format(&self) -> Format {
        match self {
            Payload::Cq { .. } => Format::Fmt1,
            Payload::Call { .. } => Format::Fmt1,
            Payload::CallWithReport { .. } => Format::Fmt1,
            Payload::RReport { .. } => Format::Fmt2,
            Payload::Rr { .. } => Format::Fmt2,
            Payload::Rr73 { .. } => Format::Fmt2,
            Payload::SeventyThree { .. } => Format::Fmt2,
            Payload::Text { .. } => Format::Fmt1,
        }
    }
}

/// Envelope for received messages
#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct RxEnvelope {
    pub payload: Payload,
    pub format: Format,
    pub snr: Option<f32>,
    pub utc_ms: i64,
    pub rx_slot: u8,
}

/// Envelope for messages to transmit
#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct TxEnvelope {
    pub payload: Payload,
    pub format: Format,
    pub raw: String,
}

/// Render a payload to the literal MSK144 text that goes on air.
///
/// Templates match WSJT-X's MSK144 mode:
///   CQ        → "CQ <CALL> <GRID>"  or  "CQ <CALL>"
///   Call      → "<TO> <FROM>"
///   CallReport→ "<TO> <FROM> +NN"
///   RReport   → "<TO> <FROM> R+NN"
///   Rr        → "<TO> <FROM> RRR"
///   73        → "<TO> <FROM> 73"
pub fn render_payload(payload: &Payload) -> Rendered {
    render_payload_with_sh(payload, false)
}

/// Render a payload, with optional MSK40 short-message form for
/// late-stage messages (R-report, RRR, 73).
///
/// When `use_sh = true` AND the payload is one of the late-stage
/// exchange messages (RReport, Rr, SeventyThree), this emits the
/// bracketed-pair form `<FROM TO> RPT` which the encoder routes to
/// the MSK40 modulator. The 12-bit hash of the call pair plus a
/// 4-bit report code fits in 40 channel bits — far shorter than
/// MSK144's 144 bits — giving a substantial SNR advantage in
/// marginal conditions.
///
/// `use_sh` should reflect BOTH the user's "Sh" checkbox AND
/// confirmation that the partner has it enabled too. If only one
/// side has it on, the MSK40 transmissions won't decode at the
/// other end. The QSO engine has no automatic way to detect partner
/// support, so this is purely a user-managed setting.
///
/// CQ / Call / CallWithReport always use the long form: those are
/// the messages that establish the partnership, and the receiver
/// can't compute the hash before knowing both calls.
pub fn render_payload_with_sh(payload: &Payload, use_sh: bool) -> Rendered {
    if use_sh {
        match payload {
            // CallWithReport (Tx2): partner-directed call carrying our
            // report of them. MSK40's RPT_TABLE contains the bare-report
            // half (-03, +00, +03, +06, +10, +13, +16), so this message
            // can be carried as a 16-bit MSK40 short message provided
            // the report value snaps to one of those entries. The
            // operator-side report-snap logic lives in the engine
            // (QsoEngine::set_my_report and the manual TX UI), not here.
            // The renderer just emits the `<from to> +NN` form; if the
            // report doesn't match an RPT_TABLE entry exactly,
            // pack_msk40 will reject the message and the encoder will
            // fall back to long form anyway.
            Payload::CallWithReport { from, to, rpt } => {
                return Rendered::Text(format!(
                    "<{} {}> {}", from, to, format_report(*rpt)));
            }
            Payload::RReport { from, to, rpt } => {
                return Rendered::Text(format!(
                    "<{} {}> R{}", from, to, format_report(*rpt)));
            }
            Payload::Rr { from, to } => {
                return Rendered::Text(format!("<{} {}> RRR", from, to));
            }
            Payload::SeventyThree { from, to } => {
                return Rendered::Text(format!("<{} {}> 73", from, to));
            }
            // RR73 falls through to long form. The MSK40 RPT_TABLE
            // (config_rpt_msk40.h in MSHV) has only 16 entries, none
            // of which is RR73 — only RRR and 73 separately. So an
            // Rr73 payload can't be carried by an MSK40 short
            // message. Use the standard MSK144 long form for this
            // single transmission; the QSO continues normally and
            // subsequent late-stage messages can resume Sh form.
            // The receiver will parse it as a normal "TO FROM RR73"
            // string and dispatch to the Rr73 handler in the QSO
            // state machine.
            Payload::Rr73 { .. } => {}
            // CQ, Call, Text fall through to long form. Note that
            // Call (Tx1) goes long form even with use_sh because
            // we have no partner hash yet at that point in the QSO
            // — the receiver couldn't reverse the hash without
            // already knowing both calls.
            _ => {}
        }
    }
    let s = match payload {
        Payload::Cq { from, grid } => {
            if let Some(g) = grid {
                format!("CQ {} {}", from, g)
            } else {
                format!("CQ {}", from)
            }
        }
        Payload::Call { from, to } => format!("{} {}", to, from),
        Payload::CallWithReport { from, to, rpt } => {
            format!("{} {} {}", to, from, format_report(*rpt))
        }
        Payload::RReport { from, to, rpt } => {
            format!("{} {} R{}", to, from, format_report(*rpt))
        }
        Payload::Rr { from, to } => format!("{} {} RRR", to, from),
        Payload::Rr73 { from, to } => format!("{} {} RR73", to, from),
        Payload::SeventyThree { from, to } => format!("{} {} 73", to, from),
        Payload::Text { from, to, text } => {
            if let Some(to) = to {
                format!("{} {} {}", to, from, text)
            } else {
                format!("CQ {} {}", from, text)
            }
        }
    };
    Rendered::Text(s)
}

/// Format an MSK144 report number with sign: +05, -02, +20, etc.
pub fn format_report(rpt: i16) -> String {
    if rpt >= 0 { format!("+{:02}", rpt) } else { format!("-{:02}", -rpt) }
}

/// Parse an MSK144 decode text line into a logical Payload.
///
/// Examples:
///   "CQ GW4WND IO82"          → Cq { from: GW4WND, grid: Some(IO82) }
///   "CQ GW4WND"               → Cq { from: GW4WND, grid: None }
///   "GW4WND F1ABC"            → Call { from: F1ABC, to: GW4WND }
///   "GW4WND F1ABC +05"        → CallWithReport { from: F1ABC, to: GW4WND, rpt: 5 }
///   "GW4WND F1ABC R+05"       → RReport { from: F1ABC, to: GW4WND, rpt: 5 }
///   "GW4WND F1ABC RRR"        → Rr { from: F1ABC, to: GW4WND }
///   "GW4WND F1ABC 73"         → SeventyThree { from: F1ABC, to: GW4WND }
pub fn parse_decode_text(text: &str) -> Option<Payload> {
    let trimmed = text.trim();
    if trimmed.is_empty() { return None; }

    let toks: Vec<&str> = trimmed.split_whitespace().collect();
    if toks.is_empty() { return None; }

    // "CQ <call>" or "CQ <call> <grid>"
    if toks[0].eq_ignore_ascii_case("CQ") {
        match toks.len() {
            2 => return Some(Payload::Cq { from: toks[1].to_uppercase(), grid: None }),
            3 => {
                if is_callsign(toks[1]) && is_grid(toks[2]) {
                    return Some(Payload::Cq {
                        from: toks[1].to_uppercase(),
                        grid: Some(toks[2].to_uppercase()),
                    });
                } else if is_callsign(toks[1]) {
                    // "CQ <call> <something>" — treat as plain CQ
                    return Some(Payload::Cq { from: toks[1].to_uppercase(), grid: None });
                }
                return None;
            }
            _ => return None,
        }
    }

    // "<to> <from>" / "<to> <from> <thing>"
    // Need at least 2 callsigns
    if toks.len() < 2 || !is_callsign(toks[0]) || !is_callsign(toks[1]) {
        return None;
    }
    let to = toks[0].to_uppercase();
    let from = toks[1].to_uppercase();

    if toks.len() == 2 {
        return Some(Payload::Call { from, to });
    }

    // 3rd token is the message body
    let body = toks[2];

    // 73
    if body == "73" {
        return Some(Payload::SeventyThree { from, to });
    }
    // RR73 — combined "rogered + 73", a single message that both
    // acks our R-report AND closes the QSO. WSJT-X / MSHV emit this
    // as an alternative to RRR for the final exchange (saves one
    // round-trip). Recognised case-insensitively to be lenient with
    // bottom decoders that may present case differently.
    if body.eq_ignore_ascii_case("RR73") {
        return Some(Payload::Rr73 { from, to });
    }
    // RRR
    if body.eq_ignore_ascii_case("RRR") || body.eq_ignore_ascii_case("RR") {
        return Some(Payload::Rr { from, to });
    }
    // R+NN / R-NN  → RReport
    if body.starts_with('R') || body.starts_with('r') {
        if let Some(rpt) = parse_report(&body[1..]) {
            return Some(Payload::RReport { from, to, rpt });
        }
    }
    // +NN / -NN → CallWithReport
    if let Some(rpt) = parse_report(body) {
        return Some(Payload::CallWithReport { from, to, rpt });
    }
    // Could also be a grid as 3rd field — treat as a plain Call
    if is_grid(body) {
        return Some(Payload::Call { from, to });
    }
    None
}

/// Parse a signed report like "+05", "-02", "+20".
fn parse_report(s: &str) -> Option<i16> {
    if s.len() < 2 { return None; }
    let (sign, digits) = if s.starts_with('+') {
        (1i16, &s[1..])
    } else if s.starts_with('-') {
        (-1i16, &s[1..])
    } else {
        return None;
    };
    digits.parse::<i16>().ok().map(|n| sign * n)
}

/// Heuristic callsign detection: at least one letter and one digit, no spaces,
/// up to 7 chars. Allows the slash for portable callsigns (we strip it off).
fn is_callsign(s: &str) -> bool {
    let s = s.trim_matches(|c: char| c == '/' );
    if s.is_empty() || s.len() > 10 { return false; }
    let has_letter = s.chars().any(|c| c.is_ascii_alphabetic());
    let has_digit  = s.chars().any(|c| c.is_ascii_digit());
    let valid = s.chars().all(|c| c.is_ascii_alphanumeric() || c == '/');
    has_letter && has_digit && valid
}

/// Heuristic grid detection: 4 chars, two letters then two digits.
fn is_grid(s: &str) -> bool {
    let s = s.as_bytes();
    if s.len() != 4 { return false; }
    s[0].is_ascii_alphabetic() && s[1].is_ascii_alphabetic()
        && s[2].is_ascii_digit() && s[3].is_ascii_digit()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_cq_with_grid() {
        match parse_decode_text("CQ GW4WND IO82") {
            Some(Payload::Cq { from, grid: Some(g) }) => {
                assert_eq!(from, "GW4WND");
                assert_eq!(g, "IO82");
            }
            other => panic!("expected Cq with grid, got {:?}", other),
        }
    }

    #[test]
    fn parse_call_with_report() {
        match parse_decode_text("GW4WND F1ABC +05") {
            Some(Payload::CallWithReport { from, to, rpt }) => {
                assert_eq!(from, "F1ABC");
                assert_eq!(to, "GW4WND");
                assert_eq!(rpt, 5);
            }
            other => panic!("expected CallWithReport, got {:?}", other),
        }
    }

    #[test]
    fn parse_rreport() {
        match parse_decode_text("GW4WND F1ABC R+05") {
            Some(Payload::RReport { from, to, rpt }) => {
                assert_eq!(from, "F1ABC");
                assert_eq!(to, "GW4WND");
                assert_eq!(rpt, 5);
            }
            other => panic!("expected RReport, got {:?}", other),
        }
    }

    #[test]
    fn parse_rrr() {
        assert!(matches!(parse_decode_text("GW4WND F1ABC RRR"),
            Some(Payload::Rr { .. })));
    }

    #[test]
    fn parse_73() {
        assert!(matches!(parse_decode_text("GW4WND F1ABC 73"),
            Some(Payload::SeventyThree { .. })));
    }

    #[test]
    fn parse_rr73() {
        // RR73 is the WSJT-X / MSHV combined "rogered + 73" message.
        // Must parse as Rr73 (NOT as Rr or SeventyThree separately).
        match parse_decode_text("GW4WND F1ABC RR73") {
            Some(Payload::Rr73 { from, to }) => {
                assert_eq!(from, "F1ABC");
                assert_eq!(to, "GW4WND");
            }
            other => panic!("expected Rr73, got {:?}", other),
        }
    }

    #[test]
    fn parse_rr73_case_insensitive() {
        assert!(matches!(parse_decode_text("GW4WND F1ABC rr73"),
            Some(Payload::Rr73 { .. })));
        assert!(matches!(parse_decode_text("GW4WND F1ABC Rr73"),
            Some(Payload::Rr73 { .. })));
    }

    #[test]
    fn render_rr73() {
        let p = Payload::Rr73 {
            from: "F1ABC".into(),
            to: "GW4WND".into(),
        };
        match render_payload(&p) {
            Rendered::Text(s) => assert_eq!(s, "GW4WND F1ABC RR73"),
        }
    }

    #[test]
    fn render_sh_rr73_falls_back_to_long_form() {
        // Sh msg can't carry RR73 (no slot in the MSK40 RPT_TABLE).
        // When use_sh=true and payload is Rr73, the renderer should
        // emit standard long form so the message goes via MSK144
        // instead. QSO continues; subsequent Sh-eligible messages
        // can resume bracket form.
        let p = Payload::Rr73 {
            from: "F1ABC".into(),
            to: "GW4WND".into(),
        };
        match render_payload_with_sh(&p, true) {
            Rendered::Text(s) => assert_eq!(s, "GW4WND F1ABC RR73"),
        }
    }

    #[test]
    fn parse_cold_call() {
        match parse_decode_text("GW4WND F1ABC") {
            Some(Payload::Call { from, to }) => {
                assert_eq!(from, "F1ABC");
                assert_eq!(to, "GW4WND");
            }
            other => panic!("expected Call, got {:?}", other),
        }
    }

    #[test]
    fn render_call_with_report_sign() {
        let p = Payload::CallWithReport {
            from: "GW4WND".into(),
            to: "F1ABC".into(),
            rpt: 5,
        };
        match render_payload(&p) {
            Rendered::Text(s) => assert_eq!(s, "F1ABC GW4WND +05"),
        }
    }

    #[test]
    fn render_negative_report() {
        let p = Payload::CallWithReport {
            from: "GW4WND".into(),
            to: "F1ABC".into(),
            rpt: -2,
        };
        match render_payload(&p) {
            Rendered::Text(s) => assert_eq!(s, "F1ABC GW4WND -02"),
        }
    }

    #[test]
    fn render_sh_rreport() {
        // Sh form: <FROM TO> R+NN. Sender's call goes first inside
        // the brackets — that's what the receiver hashes when checking
        // the "partner direction" (their MY = our HIS, their HIS = our MY).
        let p = Payload::RReport {
            from: "GW4WND".into(),
            to: "F1ABC".into(),
            rpt: 5,
        };
        match render_payload_with_sh(&p, true) {
            Rendered::Text(s) => assert_eq!(s, "<GW4WND F1ABC> R+05"),
        }
    }

    #[test]
    fn render_sh_rrr() {
        let p = Payload::Rr {
            from: "GW4WND".into(),
            to: "F1ABC".into(),
        };
        match render_payload_with_sh(&p, true) {
            Rendered::Text(s) => assert_eq!(s, "<GW4WND F1ABC> RRR"),
        }
    }

    #[test]
    fn render_sh_seventy_three() {
        let p = Payload::SeventyThree {
            from: "GW4WND".into(),
            to: "F1ABC".into(),
        };
        match render_payload_with_sh(&p, true) {
            Rendered::Text(s) => assert_eq!(s, "<GW4WND F1ABC> 73"),
        }
    }

    #[test]
    fn render_sh_off_uses_long_form() {
        // Even for an Sh-eligible payload, use_sh=false → long form.
        let p = Payload::SeventyThree {
            from: "GW4WND".into(),
            to: "F1ABC".into(),
        };
        match render_payload_with_sh(&p, false) {
            Rendered::Text(s) => assert_eq!(s, "F1ABC GW4WND 73"),
        }
    }

    #[test]
    fn render_sh_does_not_apply_to_cq() {
        // Sh form only applies to RReport/Rr/SeventyThree. CQ always
        // uses long form because the partner doesn't have our call yet
        // and so cannot compute the hash.
        let p = Payload::Cq {
            from: "GW4WND".into(),
            grid: Some("IO82".into()),
        };
        match render_payload_with_sh(&p, true) {
            Rendered::Text(s) => assert_eq!(s, "CQ GW4WND IO82"),
        }
    }

    #[test]
    fn render_sh_does_not_apply_to_call_with_report() {
        // CallWithReport ("F1ABC GW4WND +05") is the FIRST exchange
        // that introduces the partner. It must always be long form
        // because the partner hasn't yet confirmed they've heard us.
        // (WSJT-X behaves the same way — Sh kicks in from message 3
        // onward, not message 2.)
        let p = Payload::CallWithReport {
            from: "GW4WND".into(),
            to: "F1ABC".into(),
            rpt: 5,
        };
        match render_payload_with_sh(&p, true) {
            Rendered::Text(s) => assert_eq!(s, "F1ABC GW4WND +05"),
        }
    }
}
