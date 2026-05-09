// crates/dx_runtime/src/qso.rs
//
// Pure, deterministic QSO/session state machine.
// No egui, no audio, no tokio. Easy to unit test.
//
// VERBATIM PORT from MSK2K src/qso/mod.rs.
// Reports adapted for MSK144's signed-dB convention (+05 instead of OOO).
//
// Region-1 Meteor Scatter Ladder:
// ===============================
//
// SCENARIO ONE - A calls CQ:
//   A: CQ <call> <grid>                ->
//                                      <-  B: <A> <B> +NN
//   A: <B> <A> R+NN                    ->
//                                      <-  B: <A> <B> RRR
//   A: <B> <A> 73 (×repeat)            ->
//                                      <-  B: <A> <B> 73
//
// SCENARIO TWO - A receives cold call:
//                                      <-  B: <A> <B>
//   A: <B> <A> +NN                     ->
//                                      <-  B: <A> <B> R+NN
//   A: <B> <A> RRR                     ->
//                                      <-  B: <A> <B> 73
//
// SCENARIO THREE - A calls specific station:
//   A: <B> <A>                         ->
//                                      <-  B: <A> <B> +NN
//   A: <B> <A> R+NN                    ->
//                                      <-  B: <A> <B> RRR
//   A: <B> <A> 73 (×repeat)            ->

use crate::adif::QsoRecord;
use crate::proto::{render_payload_with_sh, Format, Payload, Rendered, RxEnvelope, TxEnvelope};
use std::time::{SystemTime, UNIX_EPOCH};

fn utc_ms_now() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as i64
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QsoState {
    Idle,
    Listening,
    CallingCq,
    CallingStn,
    SendingReport,
    SendingRReport,
    SendingRr,
    Sending73,
    Done,
}

impl std::fmt::Display for QsoState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            QsoState::Idle => write!(f, "IDLE"),
            QsoState::Listening => write!(f, "LISTEN"),
            QsoState::CallingCq => write!(f, "CALLING_CQ"),
            QsoState::CallingStn => write!(f, "CALLING_STN"),
            QsoState::SendingReport => write!(f, "SEND_RPT"),
            QsoState::SendingRReport => write!(f, "SEND_RRPT"),
            QsoState::SendingRr => write!(f, "SEND_RR"),
            QsoState::Sending73 => write!(f, "SEND_73"),
            QsoState::Done => write!(f, "DONE"),
        }
    }
}

#[allow(dead_code)]
#[derive(Debug, Clone)]
pub enum Intent {
    Listen,
    Cq,
    Call { their: String },
    AnswerCq { their: String, rpt: i16, grid: Option<String> },
    Abort,
}

#[allow(dead_code)]
#[derive(Debug, Clone)]
pub enum EngineEvent {
    StateChanged(QsoState),
    Rx(RxEnvelope),
    Tx(TxEnvelope),
    Info(String),
    QsoComplete { their: String, record: Option<QsoRecord> },
    TheirCallChanged { callsign: String, grid: Option<String> },
}

#[allow(dead_code)]
#[derive(Debug, Clone)]
pub enum Action {
    None,
    Transmit(TxEnvelope),
}

pub struct QsoEngine {
    pub my_call: String,
    pub my_grid: Option<String>,
    pub their_call: Option<String>,
    pub their_grid: Option<String>,
    pub state: QsoState,
    pub my_report: i16,
    pub their_report: Option<i16>,
    pub last_rx: Option<RxEnvelope>,
    pub last_tx: Option<TxEnvelope>,
    pub tx_repeat_count: u8,
    pub max_repeats: u8,
    pub qso_start_utc_ms: Option<i64>,
    pub qso_end_utc_ms: Option<i64>,
    pub band: String,
    pub freq_mhz: Option<f64>,
    /// When true, the late-stage QSO messages (R-report, RRR, 73)
    /// are emitted in MSK40 short-message form (`<FROM TO> RPT`)
    /// instead of standard MSK144. Both stations need it on for
    /// the short messages to decode at the other end. Reflects
    /// the user's "Sh" checkbox in settings.
    pub use_short_msg: bool,
    /// True when the engine entered `Sending73` with the intent to
    /// send RR73 (modern combined-acknowledge convention) rather
    /// than bare 73. Used by next_tx() so that repeat slots in
    /// Sending73 keep emitting whatever the first slot sent — i.e.
    /// if we entered via `(SendingReport, RReport) → Sending73`
    /// we keep sending RR73 across all repeat slots; if we entered
    /// via the legacy `(SendingRReport, Rr) → Sending73` (partner
    /// sent us bare RRR) we keep sending bare 73. Set when we
    /// transition to Sending73; cleared on reset_qso.
    pub final_is_rr73: bool,
}

#[allow(dead_code)]
impl QsoEngine {
    pub fn new(my_call: String) -> Self {
        Self {
            my_call,
            my_grid: None,
            their_call: None,
            their_grid: None,
            state: QsoState::Idle,
            my_report: 5,           // sensible default; overridable from UI
            their_report: None,
            last_rx: None,
            last_tx: None,
            tx_repeat_count: 0,
            max_repeats: 5,
            qso_start_utc_ms: None,
            qso_end_utc_ms: None,
            band: "2M".to_string(),
            freq_mhz: None,
            use_short_msg: false,
            final_is_rr73: false,
        }
    }

    pub fn set_my_call(&mut self, my: String)        { self.my_call = my; }
    pub fn set_my_grid(&mut self, grid: Option<String>) { self.my_grid = grid; }
    pub fn set_their_call(&mut self, their: Option<String>) { self.their_call = their; }
    pub fn set_my_report(&mut self, rpt: i16)         { self.my_report = rpt; }
    pub fn set_use_short_msg(&mut self, on: bool)     { self.use_short_msg = on; }
    pub fn set_band(&mut self, band: String)          { self.band = band; }
    pub fn set_freq_mhz(&mut self, f: Option<f64>)    { self.freq_mhz = f; }

    /// Return our grid in the form the on-air protocol accepts: a
    /// 4-character field-square locator (e.g. "IO82"). The operator's
    /// settings may hold a 6-character extended locator (e.g.
    /// "IO82KM") — that's needed for PSK Reporter spot-position
    /// accuracy and is preserved in ADIF records, but the MSK144
    /// protocol's Cq packing slot is exactly 4 chars and any extra
    /// would either truncate at the codec or produce an invalid
    /// pack. Trim 6→4 here so all on-air Cq paths share the same
    /// derivation.
    fn grid_for_protocol(&self) -> Option<String> {
        self.my_grid.as_ref().map(|g| {
            let trimmed = g.trim();
            if trimmed.len() >= 4 {
                trimmed[..4].to_string()
            } else {
                trimmed.to_string()
            }
        })
    }

    /// Set the slot period (15 or 30 seconds) and adjust `max_repeats`
    /// so QSO timeouts feel similar in wall-clock seconds regardless
    /// of period. With 15s slots, 5 repeats = 75 s of TX before giving
    /// up. Doubling slot length without halving the count would make
    /// QSOs drag for 150 s of repeated 73s on 30s slots, which is
    /// excessive — a typical MS QSO works or doesn't within ~2 min
    /// of a final being heard.
    ///
    /// Mapping:
    ///   15 s slots → max_repeats = 5  (75 s window)
    ///   30 s slots → max_repeats = 3  (90 s window — still generous)
    pub fn set_slot_period(&mut self, period_secs: u32) {
        self.max_repeats = if period_secs >= 30 { 6 } else { 6 };
    }

    fn mark_qso_start(&mut self) {
        if self.qso_start_utc_ms.is_none() {
            self.qso_start_utc_ms = Some(utc_ms_now());
        }
    }

    pub fn make_qso_record(&self) -> Option<QsoRecord> {
        let their = self.their_call.clone()?;
        let start = self.qso_start_utc_ms?;
        let end = self.qso_end_utc_ms.unwrap_or_else(utc_ms_now);
        let rst_sent_str = crate::proto::format_report(self.my_report);
        let rst_rcvd_str = self.their_report.map(crate::proto::format_report);
        Some(QsoRecord::new(
            their,
            self.my_call.clone(),
            start,
            end,
            self.band.clone(),
            self.freq_mhz,
            "MSK144",
            &rst_sent_str,
            rst_rcvd_str.as_deref(),
            self.their_grid.clone(),
        ))
    }

    pub fn on_intent(&mut self, intent: Intent) -> (Action, Vec<EngineEvent>) {
        let mut ev = vec![];
        let mut action = Action::None;

        match intent {
            Intent::Listen => {
                self.reset_qso();
                self.transition(QsoState::Listening, &mut ev);
            }
            Intent::Abort => {
                self.reset_qso();
                self.transition(QsoState::Idle, &mut ev);
            }
            Intent::Cq => {
                self.reset_qso();
                let payload = Payload::Cq {
                    from: self.my_call.clone(),
                    grid: self.grid_for_protocol(),
                };
                action = Action::Transmit(self.make_tx(payload, &mut ev));
                self.transition(QsoState::CallingCq, &mut ev);
            }
            Intent::Call { their } => {
                self.reset_qso();
                self.their_call = Some(their.clone());
                self.mark_qso_start();
                ev.push(EngineEvent::TheirCallChanged { callsign: their.clone(), grid: None });

                let payload = Payload::Call {
                    from: self.my_call.clone(),
                    to: their,
                };
                action = Action::Transmit(self.make_tx(payload, &mut ev));
                self.transition(QsoState::CallingStn, &mut ev);
            }
            Intent::AnswerCq { their, rpt, grid } => {
                self.reset_qso();
                self.their_call = Some(their.clone());
                self.their_grid = grid.clone();
                self.my_report = rpt;
                self.mark_qso_start();
                ev.push(EngineEvent::TheirCallChanged { callsign: their.clone(), grid });

                let payload = Payload::CallWithReport {
                    from: self.my_call.clone(),
                    to: their,
                    rpt,
                };
                action = Action::Transmit(self.make_tx(payload, &mut ev));
                self.transition(QsoState::SendingReport, &mut ev);
            }
        }
        (action, ev)
    }

    /// Manually force the QSO state machine to a specific point in the
    /// exchange and start transmitting the corresponding message. Used
    /// for the operator-driven "Manual TX" override on the top bar —
    /// this is the path for picking up a QSO mid-stream when the
    /// auto-driven state machine doesn't know the partner is still
    /// trying (e.g. you broke off, came back, and now you're hearing
    /// their R-report and want to send RR73 in response).
    ///
    /// Mapping from the dropdown selection to (state, payload):
    ///
    ///   Tx1 (CallingStn)       Call             { from, to: their }
    ///   Tx2 (SendingReport)    CallWithReport   { from, to: their, rpt }
    ///   Tx3 (SendingRReport)   RReport          { from, to: their, rpt }
    ///   Tx4 (SendingRr)        Rr               { from, to: their }
    ///   Tx5 (Sending73)        SeventyThree     { from, to: their }
    ///   Tx6 (CallingCq)        Cq               { from, grid }
    ///
    /// Behaviour:
    ///  - Resets QSO state cleanly (so any half-baked previous QSO
    ///    is dropped).
    ///  - Sets `their_call` and `my_report` from the operator's input.
    ///  - Marks the QSO start time so a final 73 → Done transition
    ///    will produce a logged ADIF record.
    ///  - Returns Action::Transmit + StateChanged + TheirCallChanged
    ///    so the GUI's apply_engine_output path handles it identically
    ///    to a normal intent.
    ///  - After this call, `on_rx` resumes normal state-driven
    ///    behaviour — partner's reply advances the state machine
    ///    from wherever the operator started it.
    ///
    /// `their_call` is required for Tx1..=Tx5 and ignored for Tx6.
    /// `rpt` is used for Tx2 (SendingReport) and Tx3 (SendingRReport)
    /// — we send the operator's typed value as the report we're
    /// claiming to have heard from them. For Tx1/Tx4/Tx5/Tx6 the rpt
    /// argument is stored but doesn't appear on-air.
    ///
    /// Note on Tx4 vs Tx5: both transition to Sending73 (the engine's
    /// "QSO is closing" terminal-tx state), but they emit different
    /// payloads. Tx4 sends RR73 (combined acknowledge + sign-off,
    /// modern convention); Tx5 sends bare 73. The caller picks which
    /// by passing `Sending73` for Tx5 or the dedicated
    /// `force_send_rr73()` method (alias) for Tx4. To keep the API
    /// backwards-compatible we use a sentinel: passing `SendingRr`
    /// is interpreted as "Tx4 = send RR73, transition to Sending73"
    /// — `SendingRr` is no longer entered as an actual state on our
    /// outgoing path; it only exists as a legacy destination if a
    /// partner sends bare RRR (handled in on_rx).
    pub fn force_state(
        &mut self,
        target_state: QsoState,
        their_call: Option<String>,
        rpt: Option<i16>,
    ) -> (Action, Vec<EngineEvent>) {
        let mut ev = vec![];
        self.reset_qso();

        // Only honour the manual rpt override for states that
        // actually emit a report payload (Tx2 = SendingReport,
        // Tx3 = SendingRReport). For Tx1 (cold Call), Tx4 (RR73),
        // Tx5 (73), or Tx6 (CQ), the manual-rpt input from the UI
        // is meaningless — and overwriting `my_report` here would
        // poison the next QSO's exchange because the on_rx auto-
        // update is locked once we're in any Sending* state.
        // (See `report_locked` in on_rx.) Pre-fix behaviour was to
        // unconditionally store `rpt`, which made the manual input
        // sticky across QSOs whenever the operator clicked any
        // manual-TX item, even non-report ones.
        let state_needs_rpt = matches!(
            target_state,
            QsoState::SendingReport | QsoState::SendingRReport,
        );
        if let (Some(r), true) = (rpt, state_needs_rpt) {
            // Clamp to the same range as the auto path (matches
            // protocol-legal report values) and remember it as
            // the report we'll send.
            self.my_report = r.clamp(-4, 24);
        }

        // Build the payload + state transition for the requested
        // override. `their_call` is required for everything except
        // Tx6 (CQ); the GUI side guards against missing target by
        // disabling the dropdown items that need it.
        let payload = match target_state {
            QsoState::CallingCq => {
                Payload::Cq {
                    from: self.my_call.clone(),
                    grid: self.grid_for_protocol(),
                }
            }
            QsoState::CallingStn
            | QsoState::SendingReport
            | QsoState::SendingRReport
            | QsoState::SendingRr
            | QsoState::Sending73 => {
                // Each of these needs a partner call. If the GUI
                // somehow let through a missing target, return
                // a no-op; the operator will see "nothing happened"
                // rather than a panic or a malformed transmission.
                let to = match their_call.clone() {
                    Some(c) if !c.is_empty() => c,
                    _ => return (Action::None, ev),
                };
                self.their_call = Some(to.clone());
                ev.push(EngineEvent::TheirCallChanged {
                    callsign: to.clone(), grid: None });
                match target_state {
                    QsoState::CallingStn =>
                        Payload::Call { from: self.my_call.clone(), to },
                    QsoState::SendingReport =>
                        Payload::CallWithReport {
                            from: self.my_call.clone(),
                            to, rpt: self.my_report,
                        },
                    QsoState::SendingRReport =>
                        Payload::RReport {
                            from: self.my_call.clone(),
                            to, rpt: self.my_report,
                        },
                    // Tx4 (RR73): semantically means "send the
                    // combined acknowledge + sign-off". Engine state
                    // becomes Sending73 (not SendingRr) so the
                    // existing partner-CQ / partner-Call auto-
                    // complete arms in on_rx fire. The payload is
                    // Rr73 which renders as "<them> <me> RR73".
                    QsoState::SendingRr =>
                        Payload::Rr73 { from: self.my_call.clone(), to },
                    QsoState::Sending73 =>
                        Payload::SeventyThree { from: self.my_call.clone(), to },
                    _ => unreachable!(),
                }
            }
            // Other states aren't valid targets for manual override
            // (Idle / Listening / Done are non-transmitting; the
            // dropdown shouldn't offer them).
            _ => return (Action::None, ev),
        };

        // Tx4 (SendingRr in the dropdown) actually transitions us to
        // Sending73 — the engine state for "I sent my final, awaiting
        // partner's confirmation or move-on". This unifies all the
        // auto-complete paths so partner-CQ / partner-Call after our
        // RR73 will still close the QSO and log it. The final_is_rr73
        // flag tells next_tx() to keep emitting RR73 across repeat
        // slots in Sending73 (rather than reverting to bare 73).
        let actual_state = if matches!(target_state, QsoState::SendingRr) {
            self.final_is_rr73 = true;
            QsoState::Sending73
        } else {
            target_state
        };

        self.mark_qso_start();
        let tx = self.make_tx(payload, &mut ev);
        self.transition(actual_state, &mut ev);
        (Action::Transmit(tx), ev)
    }

    pub fn on_rx(&mut self, rx: RxEnvelope) -> (Action, Vec<EngineEvent>) {
        let mut ev = vec![EngineEvent::Rx(rx.clone())];
        self.last_rx = Some(rx.clone());
        let from_call = rx.payload.from_call().to_string();

        let is_from_partner = self.their_call.as_ref()
            .map(|their| normalize_call(&from_call) == normalize_call(their))
            .unwrap_or(false);

        // If the envelope carries an SNR (dB), use it as the report we'll
        // send back to the partner — but ONLY while we haven't yet
        // committed to a report value. The moment we transition into
        // SendingReport / SendingRReport (or anything later in the
        // exchange), the report baked into our outgoing message is
        // locked for the duration of the QSO; subsequent partner
        // decodes are informational, their SNR does NOT change what
        // we transmit. This matches operator-protocol convention: once
        // you've sent "+05" to someone, you keep saying "+05" for the
        // remainder of that QSO even if their next ping is louder.
        let report_locked = matches!(
            self.state,
            QsoState::SendingReport
                | QsoState::SendingRReport
                | QsoState::SendingRr
                | QsoState::Sending73
                | QsoState::Done,
        );
        if !report_locked {
            if let Some(snr) = rx.snr {
                let q = (2.0 * (snr / 2.0).round()).clamp(-4.0, 24.0) as i16;
                self.my_report = q;
            }
        }

        match (&self.state, &rx.payload) {
            // Listening — someone calling me cold
            (QsoState::Listening, Payload::Call { from, to }) if self.is_me(to) => {
                self.their_call = Some(from.clone());
                self.mark_qso_start();
                ev.push(EngineEvent::TheirCallChanged { callsign: from.clone(), grid: None });
                let payload = Payload::CallWithReport {
                    from: self.my_call.clone(),
                    to: from.clone(),
                    rpt: self.my_report,
                };
                let action = Action::Transmit(self.make_tx(payload, &mut ev));
                self.transition(QsoState::SendingReport, &mut ev);
                return (action, ev);
            }
            // Listening — someone calling me with their report
            (QsoState::Listening, Payload::CallWithReport { from, to, rpt }) if self.is_me(to) => {
                self.their_call = Some(from.clone());
                self.their_report = Some(*rpt);
                self.mark_qso_start();
                ev.push(EngineEvent::TheirCallChanged { callsign: from.clone(), grid: None });
                let payload = Payload::RReport {
                    from: self.my_call.clone(),
                    to: from.clone(),
                    rpt: self.my_report,
                };
                let action = Action::Transmit(self.make_tx(payload, &mut ev));
                self.transition(QsoState::SendingRReport, &mut ev);
                return (action, ev);
            }
            // CallingCq — someone replied with their report
            (QsoState::CallingCq, Payload::CallWithReport { from, to, rpt }) if self.is_me(to) => {
                self.their_call = Some(from.clone());
                self.their_report = Some(*rpt);
                self.mark_qso_start();
                ev.push(EngineEvent::TheirCallChanged { callsign: from.clone(), grid: None });
                let payload = Payload::RReport {
                    from: self.my_call.clone(),
                    to: from.clone(),
                    rpt: self.my_report,
                };
                let action = Action::Transmit(self.make_tx(payload, &mut ev));
                self.transition(QsoState::SendingRReport, &mut ev);
                return (action, ev);
            }
            // CallingStn — they replied with report
            (QsoState::CallingStn, Payload::CallWithReport { from, to, rpt })
                if self.is_me(to) && is_from_partner =>
            {
                self.their_report = Some(*rpt);
                let payload = Payload::RReport {
                    from: self.my_call.clone(),
                    to: from.clone(),
                    rpt: self.my_report,
                };
                let action = Action::Transmit(self.make_tx(payload, &mut ev));
                self.transition(QsoState::SendingRReport, &mut ev);
                return (action, ev);
            }
            // SendingReport — they sent us their R-report; we respond
            // with RR73 (combined ack + sign-off). Modern MSK144 / FT8
            // / FT4 convention is to skip the old RRR → 73 two-step
            // and go straight to RR73 as the single final-acknowledge
            // message. Engine state goes directly to Sending73.
            //
            // The legacy SendingRr state is still reachable on the
            // RECEIVING side (next arm below) for partners that send
            // bare RRR — we accept that gracefully, but on our own
            // outgoing path we only ever emit RR73.
            (QsoState::SendingReport, Payload::RReport { from, to, rpt })
                if self.is_me(to) && is_from_partner =>
            {
                self.their_report = Some(*rpt);
                let payload = Payload::Rr73 {
                    from: self.my_call.clone(),
                    to: from.clone(),
                };
                let action = Action::Transmit(self.make_tx(payload, &mut ev));
                self.final_is_rr73 = true;
                self.transition(QsoState::Sending73, &mut ev);
                return (action, ev);
            }
            // SendingRReport — they sent legacy bare RRR. Legacy
            // partners still on the old protocol; respond with 73
            // (not RR73 since RRR doesn't carry the sign-off bit)
            // and go to Sending73 as the QSO closer.
            (QsoState::SendingRReport, Payload::Rr { from, to })
                if self.is_me(to) && is_from_partner =>
            {
                self.tx_repeat_count = 0;
                let payload = Payload::SeventyThree {
                    from: self.my_call.clone(),
                    to: from.clone(),
                };
                let action = Action::Transmit(self.make_tx(payload, &mut ev));
                self.transition(QsoState::Sending73, &mut ev);
                return (action, ev);
            }
            // SendingRReport — they sent RR73 (combined ack + 73); same
            // as above but also semantically closes the QSO from their
            // side. We respond with our own 73 (NOT RRR — they already
            // confirmed) and transition to Sending73. The QSO will
            // complete on our next 73 transmission.
            (QsoState::SendingRReport, Payload::Rr73 { from, to })
                if self.is_me(to) && is_from_partner =>
            {
                self.tx_repeat_count = 0;
                let payload = Payload::SeventyThree {
                    from: self.my_call.clone(),
                    to: from.clone(),
                };
                let action = Action::Transmit(self.make_tx(payload, &mut ev));
                self.transition(QsoState::Sending73, &mut ev);
                return (action, ev);
            }
            // SendingRr — they sent 73; we send 73 back
            (QsoState::SendingRr, Payload::SeventyThree { from, to })
                if self.is_me(to) && is_from_partner =>
            {
                self.tx_repeat_count = 0;
                let payload = Payload::SeventyThree {
                    from: self.my_call.clone(),
                    to: from.clone(),
                };
                let action = Action::Transmit(self.make_tx(payload, &mut ev));
                self.transition(QsoState::Sending73, &mut ev);
                return (action, ev);
            }
            // SendingRr — they sent RR73 (combined ack + 73). Treat
            // as 73; we send 73 back. The "RR" half is redundant
            // here (we'd already moved past RReport when entering
            // SendingRr), but we accept it gracefully.
            (QsoState::SendingRr, Payload::Rr73 { from, to })
                if self.is_me(to) && is_from_partner =>
            {
                self.tx_repeat_count = 0;
                let payload = Payload::SeventyThree {
                    from: self.my_call.clone(),
                    to: from.clone(),
                };
                let action = Action::Transmit(self.make_tx(payload, &mut ev));
                self.transition(QsoState::Sending73, &mut ev);
                return (action, ev);
            }
            // Sending73 — confirmed 73 from them and we've already sent ≥1 73 → QSO complete
            (QsoState::Sending73, Payload::SeventyThree { from, to })
                if self.is_me(to) && is_from_partner && self.tx_repeat_count >= 1 =>
            {
                self.qso_end_utc_ms = Some(utc_ms_now());
                let their = from.clone();
                let record = self.make_qso_record();
                ev.push(EngineEvent::QsoComplete { their, record });
                self.transition(QsoState::Done, &mut ev);
                return (Action::None, ev);
            }
            // Sending73 — partner sent RR73 as their final. Same as 73:
            // close on our next-or-already-sent 73.
            (QsoState::Sending73, Payload::Rr73 { from, to })
                if self.is_me(to) && is_from_partner && self.tx_repeat_count >= 1 =>
            {
                self.qso_end_utc_ms = Some(utc_ms_now());
                let their = from.clone();
                let record = self.make_qso_record();
                ev.push(EngineEvent::QsoComplete { their, record });
                self.transition(QsoState::Done, &mut ev);
                return (Action::None, ev);
            }
            // Sending73 — partner moved on (CQ or calling someone else): early-complete
            (QsoState::Sending73, Payload::Cq { from, .. }) if is_from_partner => {
                ev.push(EngineEvent::Info("Partner calling CQ — terminating QSO early".into()));
                self.qso_end_utc_ms = Some(utc_ms_now());
                let their = from.clone();
                let record = self.make_qso_record();
                ev.push(EngineEvent::QsoComplete { their, record });
                self.transition(QsoState::Done, &mut ev);
                return (Action::None, ev);
            }
            (QsoState::Sending73, Payload::Call { from, .. }) if is_from_partner => {
                ev.push(EngineEvent::Info("Partner calling someone else — terminating QSO early".into()));
                self.qso_end_utc_ms = Some(utc_ms_now());
                let their = from.clone();
                let record = self.make_qso_record();
                ev.push(EngineEvent::QsoComplete { their, record });
                self.transition(QsoState::Done, &mut ev);
                return (Action::None, ev);
            }

            // SendingRReport — we sent R+rpt (full report exchange
            // achieved on our side), but never got their final ack.
            // If they're now CQing or calling someone else, they
            // either heard our R+rpt and consider the QSO complete
            // from their end, OR they gave up and moved on. Either
            // way, both calls + a report were exchanged — that's
            // a valid MSK144 QSO. Log it and close.
            (QsoState::SendingRReport, Payload::Cq { from, .. }) if is_from_partner => {
                ev.push(EngineEvent::Info(
                    "Partner moved to CQ after our R-report — auto-completing QSO".into()));
                self.qso_end_utc_ms = Some(utc_ms_now());
                let their = from.clone();
                let record = self.make_qso_record();
                ev.push(EngineEvent::QsoComplete { their, record });
                self.transition(QsoState::Done, &mut ev);
                return (Action::None, ev);
            }
            (QsoState::SendingRReport, Payload::Call { from, .. }) if is_from_partner => {
                ev.push(EngineEvent::Info(
                    "Partner calling someone else after our R-report — auto-completing".into()));
                self.qso_end_utc_ms = Some(utc_ms_now());
                let their = from.clone();
                let record = self.make_qso_record();
                ev.push(EngineEvent::QsoComplete { their, record });
                self.transition(QsoState::Done, &mut ev);
                return (Action::None, ev);
            }

            // SendingRr — we sent legacy bare RRR (only reachable
            // via legacy receive path from partner sending RRR; not
            // reachable via auto outgoing path anymore). Same logic
            // as SendingRReport / Sending73: if partner moves on,
            // QSO is effectively complete — both reports exchanged
            // plus our acknowledge. Log and close.
            (QsoState::SendingRr, Payload::Cq { from, .. }) if is_from_partner => {
                ev.push(EngineEvent::Info(
                    "Partner moved to CQ after our RRR — auto-completing QSO".into()));
                self.qso_end_utc_ms = Some(utc_ms_now());
                let their = from.clone();
                let record = self.make_qso_record();
                ev.push(EngineEvent::QsoComplete { their, record });
                self.transition(QsoState::Done, &mut ev);
                return (Action::None, ev);
            }
            (QsoState::SendingRr, Payload::Call { from, .. }) if is_from_partner => {
                ev.push(EngineEvent::Info(
                    "Partner calling someone else after our RRR — auto-completing".into()));
                self.qso_end_utc_ms = Some(utc_ms_now());
                let their = from.clone();
                let record = self.make_qso_record();
                ev.push(EngineEvent::QsoComplete { their, record });
                self.transition(QsoState::Done, &mut ev);
                return (Action::None, ev);
            }
            _ => {}
        }
        (Action::None, ev)
    }

    /// Called by the slot scheduler at the start of a TX slot.
    /// Returns the payload to transmit, or None if we shouldn't TX this slot.
    pub fn next_tx(&mut self) -> Option<Payload> {
        match self.state {
            QsoState::CallingCq => Some(Payload::Cq {
                from: self.my_call.clone(),
                grid: self.grid_for_protocol(),
            }),
            QsoState::CallingStn => Some(Payload::Call {
                from: self.my_call.clone(),
                to: self.their_call.clone()?,
            }),
            QsoState::SendingReport => Some(Payload::CallWithReport {
                from: self.my_call.clone(),
                to: self.their_call.clone()?,
                rpt: self.my_report,
            }),
            QsoState::SendingRReport => Some(Payload::RReport {
                from: self.my_call.clone(),
                to: self.their_call.clone()?,
                rpt: self.my_report,
            }),
            QsoState::SendingRr => Some(Payload::Rr {
                from: self.my_call.clone(),
                to: self.their_call.clone()?,
            }),
            QsoState::Sending73 => {
                self.tx_repeat_count += 1;
                if self.tx_repeat_count > self.max_repeats { return None; }
                let to = self.their_call.clone()?;
                if self.final_is_rr73 {
                    // Modern path: we entered Sending73 because partner
                    // sent us R+rpt (or operator picked Tx4=RR73).
                    // Keep emitting RR73 across repeat slots.
                    Some(Payload::Rr73 {
                        from: self.my_call.clone(),
                        to,
                    })
                } else {
                    // Legacy path or operator picked Tx5=73. Plain 73.
                    Some(Payload::SeventyThree {
                        from: self.my_call.clone(),
                        to,
                    })
                }
            }
            _ => None,
        }
    }

    /// Called periodically by the app tick loop.
    /// If we've hit max_repeats on 73 without confirmation, complete the QSO.
    pub fn check_complete(&mut self) -> Option<EngineEvent> {
        if self.state == QsoState::Sending73 && self.tx_repeat_count > self.max_repeats {
            self.qso_end_utc_ms = Some(utc_ms_now());
            let their = self.their_call.clone().unwrap_or_default();
            let record = self.make_qso_record();
            self.state = QsoState::Done;
            return Some(EngineEvent::QsoComplete { their, record });
        }
        None
    }

    fn is_me(&self, call: &str) -> bool {
        normalize_call(call) == normalize_call(&self.my_call)
    }

    fn transition(&mut self, next: QsoState, ev: &mut Vec<EngineEvent>) {
        if self.state != next {
            self.state = next;
            ev.push(EngineEvent::StateChanged(next));
        }
    }

    fn make_tx(&mut self, payload: Payload, ev: &mut Vec<EngineEvent>) -> TxEnvelope {
        let format = payload.format();
        // Sh form is only emitted when both (a) the user has Sh on,
        // and (b) we have a partner call established. The renderer
        // restricts Sh form to RReport / Rr / SeventyThree internally
        // — CQ, Call, CallWithReport always use long form even with
        // Sh enabled (those messages can't be hashed).
        let use_sh = self.use_short_msg && self.their_call.is_some();
        let raw = match render_payload_with_sh(&payload, use_sh) {
            Rendered::Text(s) => s,
        };
        let tx = TxEnvelope { payload, format, raw };
        self.last_tx = Some(tx.clone());
        ev.push(EngineEvent::Tx(tx.clone()));
        tx
    }

    fn reset_qso(&mut self) {
        self.their_call = None;
        self.their_grid = None;
        self.their_report = None;
        self.tx_repeat_count = 0;
        self.qso_start_utc_ms = None;
        self.qso_end_utc_ms = None;
        self.final_is_rr73 = false;
    }
}

fn normalize_call(s: &str) -> String { s.trim().to_uppercase() }

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::{Format, Payload, RxEnvelope};

    fn rx(payload: Payload) -> RxEnvelope {
        let format = payload.format();
        RxEnvelope { payload, format, snr: None, utc_ms: 0, rx_slot: 0 }
    }

    #[test]
    fn scenario_one_cq_caller() {
        let mut a = QsoEngine::new("GW4WND".into());
        a.my_grid = Some("IO82".into());

        let _ = a.on_intent(Intent::Cq);
        assert_eq!(a.state, QsoState::CallingCq);

        let _ = a.on_rx(rx(Payload::CallWithReport {
            from: "F1ABC".into(),
            to: "GW4WND".into(),
            rpt: 5,
        }));
        assert_eq!(a.state, QsoState::SendingRReport);
        assert_eq!(a.their_call, Some("F1ABC".into()));
        assert_eq!(a.their_report, Some(5));

        let _ = a.on_rx(rx(Payload::Rr { from: "F1ABC".into(), to: "GW4WND".into() }));
        assert_eq!(a.state, QsoState::Sending73);

        // First 73 transmitted by next_tx
        let tx = a.next_tx();
        assert!(matches!(tx, Some(Payload::SeventyThree { .. })));
        assert_eq!(a.tx_repeat_count, 1);

        // Their 73 + we've sent ≥1 → complete
        let (_, evs) = a.on_rx(rx(Payload::SeventyThree {
            from: "F1ABC".into(), to: "GW4WND".into()
        }));
        assert_eq!(a.state, QsoState::Done);
        assert!(evs.iter().any(|e| matches!(e, EngineEvent::QsoComplete { .. })));
    }

    #[test]
    fn scenario_two_cold_call_received() {
        let mut a = QsoEngine::new("GW4WND".into());
        let _ = a.on_intent(Intent::Listen);

        let _ = a.on_rx(rx(Payload::Call { from: "F1ABC".into(), to: "GW4WND".into() }));
        assert_eq!(a.state, QsoState::SendingReport);
        assert_eq!(a.their_call, Some("F1ABC".into()));
    }

    #[test]
    fn scenario_three_call_specific_station() {
        let mut a = QsoEngine::new("GW4WND".into());

        let _ = a.on_intent(Intent::Call { their: "F1ABC".into() });
        assert_eq!(a.state, QsoState::CallingStn);

        let _ = a.on_rx(rx(Payload::CallWithReport {
            from: "F1ABC".into(), to: "GW4WND".into(), rpt: 5,
        }));
        assert_eq!(a.state, QsoState::SendingRReport);
    }

    #[test]
    fn rr73_in_sending_rreport_jumps_to_sending73() {
        // We're in SendingRReport (transmitting our R+05). Partner
        // sends RR73 instead of plain RRR — that's a combined
        // ack-and-73 message. We should:
        //  - NOT keep sending R+05 (Roger's reported bug)
        //  - skip the Rr state entirely
        //  - send 73 next and complete the QSO when partner re-confirms
        let mut a = QsoEngine::new("GW4WND".into());
        let _ = a.on_intent(Intent::Call { their: "SM7VUK".into() });
        // Partner replies with their report — we go to SendingRReport
        let _ = a.on_rx(rx(Payload::CallWithReport {
            from: "SM7VUK".into(), to: "GW4WND".into(), rpt: 2,
        }));
        assert_eq!(a.state, QsoState::SendingRReport);

        // Partner sends RR73 (skipping a separate RRR)
        let (action, _) = a.on_rx(rx(Payload::Rr73 {
            from: "SM7VUK".into(), to: "GW4WND".into(),
        }));
        // Engine should respond by sending 73 (NOT RRR, NOT staying
        // in SendingRReport)
        match action {
            Action::Transmit(env) => match env.payload {
                Payload::SeventyThree { ref to, .. } => {
                    assert_eq!(to, "SM7VUK");
                }
                other => panic!("expected SeventyThree, got {:?}", other),
            },
            other => panic!("expected Transmit, got {:?}", other),
        }
        assert_eq!(a.state, QsoState::Sending73);
    }

    #[test]
    fn answer_cq_starts_with_report() {
        let mut a = QsoEngine::new("GW4WND".into());
        let _ = a.on_intent(Intent::AnswerCq {
            their: "F1ABC".into(),
            rpt: -2,
            grid: Some("JN18".into()),
        });
        assert_eq!(a.state, QsoState::SendingReport);
        assert_eq!(a.their_call, Some("F1ABC".into()));
        assert_eq!(a.their_grid, Some("JN18".into()));
        assert_eq!(a.my_report, -2);
    }

    #[test]
    fn report_locked_once_committed() {
        // CallingCq → answer with +05 (snr=5) → SendingRReport, my_report=4 (quantised)
        // Then partner sends another decode at snr=20. The report we're
        // sending should NOT change to +20; it's locked at the value we
        // committed when we transitioned out of CallingCq.
        let mut a = QsoEngine::new("GW4WND".into());
        let _ = a.on_intent(Intent::Cq);

        // Partner answers with their report on us. Their decode arrives
        // with snr=5 → my_report quantises to +6 (next 2-dB step up).
        let mut env = rx(Payload::CallWithReport {
            from: "F1ABC".into(),
            to: "GW4WND".into(),
            rpt: 5,
        });
        env.snr = Some(5.0);
        let _ = a.on_rx(env);
        assert_eq!(a.state, QsoState::SendingRReport);
        let committed = a.my_report;

        // A second decode arrives at much louder SNR. Engine still in
        // SendingRReport. my_report MUST NOT change.
        let mut env2 = rx(Payload::CallWithReport {
            from: "F1ABC".into(),
            to: "GW4WND".into(),
            rpt: 5,
        });
        env2.snr = Some(20.0);
        let _ = a.on_rx(env2);
        assert_eq!(a.my_report, committed,
            "my_report leaked to {} after committing at {}",
            a.my_report, committed);
    }

    #[test]
    fn report_updates_pre_commit() {
        // While in CallingCq with no partner yet, my_report SHOULD update
        // from incoming SNR — so when someone does answer, we send a
        // current report value. The lock only kicks in after we commit.
        let mut a = QsoEngine::new("GW4WND".into());
        let _ = a.on_intent(Intent::Cq);
        // Some random non-partner CQ traffic comes in with high SNR.
        // Even though it's not addressed to us, the SNR field still
        // reflects a strong incoming signal and updates my_report.
        let mut env = rx(Payload::Cq { from: "EA5XYZ".into(), grid: None });
        env.snr = Some(15.0);
        let _ = a.on_rx(env);
        assert!(a.my_report >= 14, "expected my_report ~+15, got {}", a.my_report);
    }

    #[test]
    fn modern_path_sends_rr73_not_rrr() {
        // Modern MSK144 / FT8 / FT4 convention: when partner sends us
        // an R-report, we respond with RR73 (combined ack + 73), not
        // legacy RRR. The engine should transition straight to
        // Sending73 and the next_tx() repeat slots should also emit
        // RR73 (not bare 73).
        let mut a = QsoEngine::new("GW4WND".into());
        let _ = a.on_intent(Intent::Call { their: "F1ABC".into() });
        // Partner replies with their report → we go to SendingRReport
        let _ = a.on_rx(rx(Payload::CallWithReport {
            from: "F1ABC".into(), to: "GW4WND".into(), rpt: 3,
        }));
        assert_eq!(a.state, QsoState::SendingRReport);

        // Partner sends us their R-report
        let (action, _) = a.on_rx(rx(Payload::RReport {
            from: "F1ABC".into(), to: "GW4WND".into(), rpt: 5,
        }));
        // Engine should send RR73 (Rr73 payload), NOT bare RRR (Rr)
        match action {
            Action::Transmit(env) => match env.payload {
                Payload::Rr73 { .. } => {} // ✓
                other => panic!("expected Rr73, got {:?}", other),
            },
            other => panic!("expected Transmit, got {:?}", other),
        }
        assert_eq!(a.state, QsoState::Sending73);
        assert!(a.final_is_rr73, "should have flagged final as RR73");

        // Repeat slot in Sending73 should also emit Rr73, not 73
        let next = a.next_tx();
        match next {
            Some(Payload::Rr73 { .. }) => {} // ✓
            other => panic!("repeat slot expected Rr73, got {:?}", other),
        }
    }

    #[test]
    fn auto_complete_when_partner_cqs_after_our_rr73() {
        // Roger's scenario: we sent RR73, partner moves on to CQ
        // before we can decode a final 73 from them. The engine
        // should treat partner-CQ as confirmation that the QSO is
        // effectively complete and log it.
        let mut a = QsoEngine::new("GW4WND".into());
        let _ = a.on_intent(Intent::Call { their: "I5YDI".into() });
        let _ = a.on_rx(rx(Payload::CallWithReport {
            from: "I5YDI".into(), to: "GW4WND".into(), rpt: -7,
        }));
        let _ = a.on_rx(rx(Payload::RReport {
            from: "I5YDI".into(), to: "GW4WND".into(), rpt: 0,
        }));
        // We're now in Sending73 with final_is_rr73 = true
        assert_eq!(a.state, QsoState::Sending73);

        // First slot: we send RR73
        let _ = a.next_tx();

        // Partner CQs (skipping past us to the next QSO)
        let (_, evs) = a.on_rx(rx(Payload::Cq {
            from: "I5YDI".into(),
            grid: Some("JN54".into()),
        }));
        assert_eq!(a.state, QsoState::Done);
        assert!(evs.iter().any(|e| matches!(e, EngineEvent::QsoComplete { .. })),
            "expected QsoComplete event when partner CQs after our RR73");
    }

    #[test]
    fn auto_complete_when_partner_cqs_after_our_r_report() {
        // We sent R+rpt (full report exchange achieved) but never
        // got a final ack. Partner is now CQing — we should still
        // auto-log the QSO.
        let mut a = QsoEngine::new("GW4WND".into());
        let _ = a.on_intent(Intent::Listen);
        // Partner cold-calls us
        let _ = a.on_rx(rx(Payload::Call {
            from: "DK7RC".into(), to: "GW4WND".into(),
        }));
        assert_eq!(a.state, QsoState::SendingReport);
        // They respond with their R-report — wait, we sent a report
        // and they might send back R+rpt. Let me think about the
        // sequence... Actually the (SendingReport, RReport) arm
        // transitions us to Sending73 directly. So to test
        // auto-complete from SendingRReport we need a different setup.
        //
        // Use the alternate path: we cold-call them, they send
        // CallWithReport, we go to SendingRReport (sending R+rpt).
        // Reset and do that instead.
        let mut a = QsoEngine::new("GW4WND".into());
        let _ = a.on_intent(Intent::Call { their: "DK7RC".into() });
        let _ = a.on_rx(rx(Payload::CallWithReport {
            from: "DK7RC".into(), to: "GW4WND".into(), rpt: 4,
        }));
        assert_eq!(a.state, QsoState::SendingRReport);

        // Partner CQs after our R+rpt — auto-complete
        let (_, evs) = a.on_rx(rx(Payload::Cq {
            from: "DK7RC".into(), grid: None,
        }));
        assert_eq!(a.state, QsoState::Done);
        assert!(evs.iter().any(|e| matches!(e, EngineEvent::QsoComplete { .. })));
    }
}
