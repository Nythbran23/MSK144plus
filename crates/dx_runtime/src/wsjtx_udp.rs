// crates/dx_runtime/src/wsjtx_udp.rs
//
// WSJT-X-format UDP broadcast for QSO logging integration.
//
// Sends two datagrams on QSO completion:
//   * QSOLogged   (msg type 5)  — structured fields, the form
//                                  Logger32/N1MM+/HRD/JTAlert/DX Lab
//                                  Suite primarily consume
//   * LoggedADIF  (msg type 12) — full ADIF record string, used by
//                                  loggers that want the file format
//                                  (e.g. Cloudlog UDP plugin)
//
// Default destination is 127.0.0.1:2237 (WSJT-X / MSHV convention).
// Most loggers either listen on 2237 directly or proxy from there.
//
// Wire format follows the WSJT-X NetworkMessage convention used by
// every JT-mode app: u32-BE magic + schema + type, then a sequence
// of length-prefixed UTF-8 strings interleaved with fixed-width
// numeric fields. QDateTime is encoded as a Julian day number +
// ms-since-midnight + spec byte.
//
// Ported from FSK441Plus's logging.rs with these adaptations:
//  * APP_ID is parameterised so multiple of our apps can use the
//    same module without colliding on identification
//  * No external dependencies beyond std::net::UdpSocket and chrono
//    (already a workspace dep). Synchronous best-effort send — the
//    broadcast is fire-and-forget and shouldn't block QsoComplete.

use chrono::{DateTime, Datelike, Timelike, Utc};
use std::net::UdpSocket;

const MAGIC:            u32 = 0xadbccbda;
const SCHEMA:           u32 = 3;
const TYPE_QSO_LOGGED:  u32 = 5;
const TYPE_LOGGED_ADIF: u32 = 12;

/// One completed-QSO record to broadcast over the WSJT-X UDP protocol.
///
/// All fields are mandatory at the type level but several can be
/// empty strings when the data isn't known (e.g. `dx_grid` for a
/// non-CQ contact where the partner never sent their grid). Loggers
/// tolerate empty strings; missing fields cause parse errors at the
/// listener side.
#[derive(Debug, Clone)]
pub struct LogRecord {
    pub my_call:   String,
    pub my_grid:   String,
    pub dx_call:   String,
    pub dx_grid:   String,
    /// Centre frequency of the contact in Hz. Loggers convert this to
    /// MHz internally and use it for band-deduction.
    pub freq_hz:   u64,
    /// e.g. "MSK144", "FSK441", "FT8". Should match WSJT-X's mode
    /// string convention so the logger's mode dropdown auto-selects.
    pub mode:      String,
    pub rst_sent:  String,
    pub rst_rcvd:  String,
    pub time_on:   DateTime<Utc>,
    pub time_off:  DateTime<Utc>,
    /// Free-text comment. We use it to carry any extra metadata that
    /// the structured fields can't hold.
    pub comment:   String,
    /// "MS" for meteor scatter, "" otherwise. Goes into ADIF PROP_MODE.
    pub prop_mode: String,
}

/// Best-effort UDP broadcast of one QSO. Sends both QSOLogged (struct
/// form) and LoggedADIF (full ADIF) so all common loggers receive
/// what they want. Failures are logged and swallowed — broadcast is
/// fire-and-forget; the QSO has already been written to the local
/// ADIF file by the caller.
///
/// `app_id` should be the program identification string (e.g.
/// "MSK144Plus", "FSK441 Plus"). Goes into the QSOLogged datagram's
/// app_id field and the LoggedADIF's PROGRAMID header.
pub fn broadcast(host: &str, port: u16, app_id: &str, rec: &LogRecord) {
    let addr = format!("{}:{}", host, port);
    let sock = match UdpSocket::bind("0.0.0.0:0") {
        Ok(s) => s,
        Err(e) => {
            log::warn!("[UDP] bind failed: {}", e);
            return;
        }
    };
    if let Err(e) = sock.connect(&addr) {
        log::warn!("[UDP] connect to {} failed: {}", addr, e);
        return;
    }
    let msg1 = build_qso_logged(app_id, rec);
    match sock.send(&msg1) {
        Ok(_)  => log::info!("[UDP] QSOLogged → {} ({} bytes)", addr, msg1.len()),
        Err(e) => log::warn!("[UDP] QSOLogged send failed: {}", e),
    }
    let msg2 = build_logged_adif(app_id, rec);
    match sock.send(&msg2) {
        Ok(_)  => log::info!("[UDP] LoggedADIF → {} ({} bytes)", addr, msg2.len()),
        Err(e) => log::warn!("[UDP] LoggedADIF send failed: {}", e),
    }
}

fn build_qso_logged(app_id: &str, rec: &LogRecord) -> Vec<u8> {
    let mut buf = Vec::with_capacity(256);
    write_header(&mut buf, app_id, TYPE_QSO_LOGGED);
    write_qdatetime(&mut buf, rec.time_off);
    write_utf8(&mut buf, &rec.dx_call);
    write_utf8(&mut buf, &rec.dx_grid);
    write_u64(&mut buf, rec.freq_hz);
    write_utf8(&mut buf, &rec.mode);
    write_utf8(&mut buf, &rec.rst_sent);
    write_utf8(&mut buf, &rec.rst_rcvd);
    write_utf8(&mut buf, "");                      // tx_pwr
    write_utf8(&mut buf, &rec.comment);
    write_utf8(&mut buf, "");                      // name
    write_qdatetime(&mut buf, rec.time_on);
    write_utf8(&mut buf, "");                      // op_call
    write_utf8(&mut buf, &rec.my_call);
    write_utf8(&mut buf, &rec.my_grid);
    write_utf8(&mut buf, "");                      // exch_sent
    write_utf8(&mut buf, "");                      // exch_rcvd
    write_utf8(&mut buf, &rec.prop_mode);
    buf
}

fn build_logged_adif(app_id: &str, rec: &LogRecord) -> Vec<u8> {
    let mut buf = Vec::with_capacity(512);
    write_header(&mut buf, app_id, TYPE_LOGGED_ADIF);
    let adif = format!(
        "\n<ADIF_VER:5>3.1.0\n<PROGRAMID:{}>{}\n<EOH>\n{}<EOR>",
        app_id.len(), app_id, adif_record(rec)
    );
    write_bytes(&mut buf, adif.as_bytes());
    buf
}

fn adif_record(rec: &LogRecord) -> String {
    let freq_mhz = format!("{:.6}", rec.freq_hz as f64 / 1_000_000.0);
    let band = freq_to_band(rec.freq_hz);
    let mut s = String::new();
    s += &af("STATION_CALLSIGN", &rec.my_call);
    s += &af("MY_GRIDSQUARE",    &rec.my_grid);
    s += &af("CALL",             &rec.dx_call);
    s += &af("GRIDSQUARE",       &rec.dx_grid);
    s += &af("MODE",             &rec.mode);
    s += &af("RST_SENT",         &rec.rst_sent);
    s += &af("RST_RCVD",         &rec.rst_rcvd);
    s += &af("QSO_DATE",         &rec.time_on.format("%Y%m%d").to_string());
    s += &af("TIME_ON",          &rec.time_on.format("%H%M%S").to_string());
    s += &af("QSO_DATE_OFF",     &rec.time_off.format("%Y%m%d").to_string());
    s += &af("TIME_OFF",         &rec.time_off.format("%H%M%S").to_string());
    s += &af("FREQ",             &freq_mhz);
    s += &af("BAND",             &band);
    if !rec.prop_mode.is_empty() { s += &af("PROP_MODE", &rec.prop_mode); }
    if !rec.comment.is_empty()   { s += &af("COMMENT",   &rec.comment);   }
    s
}

fn af(tag: &str, val: &str) -> String {
    if val.is_empty() { return String::new(); }
    format!("<{}:{}>{}", tag, val.len(), val)
}

fn freq_to_band(hz: u64) -> String {
    match hz / 1_000_000 {
        1..=2     => "160m", 3..=4   => "80m",   5       => "60m",
        7..=8     => "40m",  10      => "30m",   14..=15 => "20m",
        18..=19   => "17m",  21..=22 => "15m",   24..=25 => "12m",
        28..=30   => "10m",  50..=54 => "6m",    70..=71 => "4m",
        144..=148 => "2m",   430..=440 => "70cm",
        _         => "other",
    }.to_string()
}

fn write_header(buf: &mut Vec<u8>, app_id: &str, msg_type: u32) {
    write_u32(buf, MAGIC);
    write_u32(buf, SCHEMA);
    write_u32(buf, msg_type);
    write_utf8(buf, app_id);
}

fn write_u32(buf: &mut Vec<u8>, v: u32) { buf.extend_from_slice(&v.to_be_bytes()); }
fn write_u64(buf: &mut Vec<u8>, v: u64) { buf.extend_from_slice(&v.to_be_bytes()); }
fn write_utf8(buf: &mut Vec<u8>, s: &str) { write_bytes(buf, s.as_bytes()); }

fn write_bytes(buf: &mut Vec<u8>, bytes: &[u8]) {
    if bytes.is_empty() {
        // Qt's "null QByteArray" sentinel — distinct from empty string
        // (which is encoded as length=0). Most receivers tolerate either
        // for empty fields, but matching WSJT-X exactly avoids edge
        // cases.
        write_u32(buf, 0xffff_ffff);
    } else {
        write_u32(buf, bytes.len() as u32);
        buf.extend_from_slice(bytes);
    }
}

/// Encode a UTC instant as a Qt QDateTime: 8-byte BE Julian day number,
/// 4-byte BE ms-since-midnight, 1-byte spec (1 = UTC). Matches
/// QDateTime::operator<<(QDataStream &) on Qt 5/6.
fn write_qdatetime(buf: &mut Vec<u8>, dt: DateTime<Utc>) {
    let (y, m, d) = (dt.year() as i64, dt.month() as i64, dt.day() as i64);
    // Fliegel & Van Flandern Julian Day Number formula. Branchless
    // for any Gregorian date past 1582; we don't deal with anything
    // earlier so this is sufficient.
    let jdn = (1461 * (y + 4800 + (m - 14) / 12)) / 4
            + (367  * (m - 2 - 12 * ((m - 14) / 12))) / 12
            - (3    * ((y + 4900 + (m - 14) / 12) / 100)) / 4
            + d - 32075;
    let ms = dt.hour()   as u32 * 3_600_000
           + dt.minute() as u32 * 60_000
           + dt.second() as u32 * 1_000
           + dt.nanosecond()    / 1_000_000;
    buf.extend_from_slice(&jdn.to_be_bytes());
    write_u32(buf, ms);
    buf.push(1u8); // Qt::UTC
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_record() -> LogRecord {
        let t_on  = Utc.with_ymd_and_hms(2025, 4, 15, 17, 25, 30).unwrap();
        let t_off = Utc.with_ymd_and_hms(2025, 4, 15, 17, 27, 0).unwrap();
        LogRecord {
            my_call:   "GW4WND".into(),
            my_grid:   "IO82KM".into(),
            dx_call:   "F1ABC".into(),
            dx_grid:   "JN08".into(),
            freq_hz:   144_360_000,
            mode:      "MSK144".into(),
            rst_sent:  "+05".into(),
            rst_rcvd:  "+02".into(),
            time_on:   t_on,
            time_off:  t_off,
            comment:   "".into(),
            prop_mode: "MS".into(),
        }
    }

    use chrono::TimeZone;

    #[test]
    fn qso_logged_starts_with_correct_header() {
        let rec = sample_record();
        let bytes = build_qso_logged("MSK144Plus", &rec);
        assert_eq!(&bytes[0..4], &MAGIC.to_be_bytes());
        assert_eq!(&bytes[4..8], &SCHEMA.to_be_bytes());
        assert_eq!(&bytes[8..12], &TYPE_QSO_LOGGED.to_be_bytes());
        // app_id length-prefix at offset 12
        assert_eq!(
            u32::from_be_bytes([bytes[12], bytes[13], bytes[14], bytes[15]]) as usize,
            "MSK144Plus".len()
        );
    }

    #[test]
    fn logged_adif_contains_call_and_grid() {
        let rec = sample_record();
        let bytes = build_logged_adif("MSK144Plus", &rec);
        let s = String::from_utf8_lossy(&bytes);
        assert!(s.contains("<CALL:5>F1ABC"));
        assert!(s.contains("<GRIDSQUARE:4>JN08"));
        assert!(s.contains("<MODE:6>MSK144"));
        assert!(s.contains("<RST_SENT:3>+05"));
        assert!(s.contains("<BAND:2>2m"));
        assert!(s.contains("<PROP_MODE:2>MS"));
        // Header preamble
        assert!(s.contains("<PROGRAMID:10>MSK144Plus"));
    }

    #[test]
    fn freq_to_band_handles_amateur_bands() {
        assert_eq!(freq_to_band(  3_700_000), "80m");
        assert_eq!(freq_to_band( 14_074_000), "20m");
        assert_eq!(freq_to_band( 50_313_000), "6m");
        assert_eq!(freq_to_band( 70_154_000), "4m");
        assert_eq!(freq_to_band(144_360_000), "2m");
        assert_eq!(freq_to_band(432_500_000), "70cm");
        // 100 MHz — not an amateur band; falls through to "other"
        assert_eq!(freq_to_band(100_000_000), "other");
    }

    #[test]
    fn qdatetime_encodes_julian_day_and_ms() {
        // 2000-01-01 00:00:00 UTC → JDN 2451545
        let dt = Utc.with_ymd_and_hms(2000, 1, 1, 0, 0, 0).unwrap();
        let mut buf = Vec::new();
        write_qdatetime(&mut buf, dt);
        // 8-byte JDN + 4-byte ms + 1 spec
        assert_eq!(buf.len(), 13);
        let jdn = i64::from_be_bytes([
            buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
        ]);
        assert_eq!(jdn, 2_451_545);
        // ms-since-midnight = 0
        let ms = u32::from_be_bytes([buf[8], buf[9], buf[10], buf[11]]);
        assert_eq!(ms, 0);
        assert_eq!(buf[12], 1u8);
    }

    #[test]
    fn empty_string_uses_qt_null_marker() {
        let mut buf = Vec::new();
        write_utf8(&mut buf, "");
        assert_eq!(buf, vec![0xff, 0xff, 0xff, 0xff]);
    }

    #[test]
    fn nonempty_string_uses_length_prefix() {
        let mut buf = Vec::new();
        write_utf8(&mut buf, "abc");
        assert_eq!(buf, vec![0, 0, 0, 3, b'a', b'b', b'c']);
    }
}
