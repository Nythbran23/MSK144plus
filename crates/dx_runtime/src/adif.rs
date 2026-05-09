// crates/dx_runtime/src/adif.rs
//
// ADIF (Amateur Data Interchange Format) logging for QSO records.
//
// Lifted from MSK2K's qso/adif.rs and generalised so PROGRAMID and the
// default file path are derived from the app_name. Same wire format
// (ADIF 3.1.4) so logs from all our apps are interchangeable with
// standard logging tools (e.g. LoTW, eQSL, Cloudlog).

use std::fs::OpenOptions;
use std::io::{BufRead, BufReader, Write};
use std::path::PathBuf;
use chrono::{Utc, TimeZone};
use anyhow::Result;

/// A completed QSO record with all required fields for ADIF logging.
#[derive(Debug, Clone)]
pub struct QsoRecord {
    /// Their callsign
    pub call: String,
    /// Our callsign
    pub operator: String,
    /// QSO date in YYYYMMDD format
    pub qso_date: String,
    /// Start time in HHMMSS format
    pub time_on: String,
    /// End time in HHMMSS format
    pub time_off: String,
    /// Band (e.g., "2M", "70CM")
    pub band: String,
    /// Frequency in MHz (optional)
    pub freq: Option<f64>,
    /// Mode (e.g., "MSK144", "FSK441", "MSK2K")
    pub mode: String,
    /// RST sent (or report e.g. "+10")
    pub rst_sent: String,
    /// RST received
    pub rst_rcvd: String,
    /// Their Maidenhead grid square (e.g., "JO54")
    pub gridsquare: Option<String>,
}

impl QsoRecord {
    /// Build a record from millisecond timestamps + QSO data.
    pub fn new(
        call: String,
        operator: String,
        start_utc_ms: i64,
        end_utc_ms: i64,
        band: String,
        freq: Option<f64>,
        mode: &str,
        rst_sent: &str,
        rst_rcvd: Option<&str>,
        gridsquare: Option<String>,
    ) -> Self {
        let start_dt = Utc.timestamp_millis_opt(start_utc_ms).unwrap();
        let end_dt   = Utc.timestamp_millis_opt(end_utc_ms).unwrap();
        Self {
            call,
            operator,
            qso_date: start_dt.format("%Y%m%d").to_string(),
            time_on:  start_dt.format("%H%M%S").to_string(),
            time_off: end_dt.format("%H%M%S").to_string(),
            band,
            freq,
            mode: mode.to_string(),
            rst_sent: rst_sent.to_string(),
            rst_rcvd: rst_rcvd.unwrap_or("").to_string(),
            gridsquare,
        }
    }

    /// Format as ADIF record line (single line ending in `<EOR>`).
    pub fn to_adif(&self) -> String {
        let mut parts = Vec::new();
        parts.push(adif_field("CALL",     &self.call));
        parts.push(adif_field("OPERATOR", &self.operator));
        parts.push(adif_field("QSO_DATE", &self.qso_date));
        parts.push(adif_field("TIME_ON",  &self.time_on));
        parts.push(adif_field("TIME_OFF", &self.time_off));
        parts.push(adif_field("BAND",     &self.band));
        if let Some(f) = self.freq {
            parts.push(adif_field("FREQ", &format!("{:.6}", f)));
        }
        parts.push(adif_field("MODE",     &self.mode));
        parts.push(adif_field("RST_SENT", &self.rst_sent));
        if !self.rst_rcvd.is_empty() {
            parts.push(adif_field("RST_RCVD", &self.rst_rcvd));
        }
        if let Some(ref grid) = self.gridsquare {
            parts.push(adif_field("GRIDSQUARE", grid));
        }
        parts.push("<EOR>".to_string());
        parts.join(" ")
    }

    /// Format date for display (YYYY-MM-DD)
    pub fn display_date(&self) -> String {
        if self.qso_date.len() == 8 {
            format!("{}-{}-{}",
                &self.qso_date[0..4],
                &self.qso_date[4..6],
                &self.qso_date[6..8])
        } else {
            self.qso_date.clone()
        }
    }

    pub fn display_time_on(&self) -> String  { format_time(&self.time_on) }
    pub fn display_time_off(&self) -> String { format_time(&self.time_off) }
}

fn format_time(t: &str) -> String {
    if t.len() >= 6 {
        format!("{}:{}:{}", &t[0..2], &t[2..4], &t[4..6])
    } else if t.len() >= 4 {
        format!("{}:{}", &t[0..2], &t[2..4])
    } else {
        t.to_string()
    }
}

fn adif_field(name: &str, value: &str) -> String {
    format!("<{}:{}>{}", name, value.len(), value)
}

/// Append-only ADIF file writer.
pub struct AdifLogger {
    path: PathBuf,
    program_id: String,
}

impl AdifLogger {
    /// Create a logger that writes to `path` and uses `program_id` as the
    /// PROGRAMID header (e.g. "MSK144+", "MSK2K", "FSK441+").
    pub fn new(path: impl Into<PathBuf>, program_id: impl Into<String>) -> Self {
        Self {
            path: path.into(),
            program_id: program_id.into(),
        }
    }

    /// Display-friendly path string for log output. Lossy on Windows
    /// non-UTF-8 filenames, but those are rare and the result is only
    /// for human reading.
    pub fn path_display(&self) -> String {
        self.path.display().to_string()
    }

    /// Append a QSO record to the ADIF file. Writes the file header on
    /// the first call (when the file is missing or empty).
    pub fn log_qso(&self, record: &QsoRecord) -> Result<()> {
        let needs_header = !self.path.exists()
            || self.path.metadata().map(|m| m.len() == 0).unwrap_or(true);

        if let Some(parent) = self.path.parent() {
            std::fs::create_dir_all(parent).ok();
        }

        let mut file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(&self.path)?;

        if needs_header {
            writeln!(file, "ADIF Export from {}", self.program_id)?;
            writeln!(file, "<ADIF_VER:5>3.1.4")?;
            let pid = &self.program_id;
            writeln!(file, "<PROGRAMID:{}>{}", pid.len(), pid)?;
            writeln!(file, "<EOH>")?;
            writeln!(file)?;
        }

        writeln!(file, "{}", record.to_adif())?;
        log::info!("📝 Logged QSO to {}: {} at {}",
            self.path.display(), record.call, record.time_on);
        Ok(())
    }

    /// Read all QSO records from the ADIF file. Returns empty vec if file
    /// is missing.
    pub fn read_all(&self) -> Result<Vec<QsoRecord>> {
        if !self.path.exists() {
            return Ok(Vec::new());
        }
        let file = std::fs::File::open(&self.path)?;
        let reader = BufReader::new(file);
        let mut records = Vec::new();
        let mut in_header = true;

        for line_result in reader.lines() {
            let line = line_result?;
            if in_header {
                if line.contains("<EOH>") { in_header = false; }
                continue;
            }
            if line.trim().is_empty() { continue; }
            if let Some(rec) = parse_adif_line(&line) {
                records.push(rec);
            }
        }
        Ok(records)
    }
}

/// Best-effort line parser. Returns None if the line doesn't contain a
/// CALL field; tolerates missing optional fields.
fn parse_adif_line(line: &str) -> Option<QsoRecord> {
    let call       = parse_field(line, "CALL")?;
    let operator   = parse_field(line, "OPERATOR").unwrap_or_default();
    let qso_date   = parse_field(line, "QSO_DATE").unwrap_or_default();
    let time_on    = parse_field(line, "TIME_ON").unwrap_or_default();
    let time_off   = parse_field(line, "TIME_OFF").unwrap_or_default();
    let band       = parse_field(line, "BAND").unwrap_or_default();
    let mode       = parse_field(line, "MODE").unwrap_or_else(|| "MSK144".to_string());
    let rst_sent   = parse_field(line, "RST_SENT").unwrap_or_default();
    let rst_rcvd   = parse_field(line, "RST_RCVD").unwrap_or_default();
    let gridsquare = parse_field(line, "GRIDSQUARE");
    let freq       = parse_field(line, "FREQ").and_then(|s| s.parse::<f64>().ok());
    Some(QsoRecord {
        call, operator, qso_date, time_on, time_off, band, freq,
        mode, rst_sent, rst_rcvd, gridsquare,
    })
}

/// Parse `<NAME:len>value` field. ADIF fields are case-sensitive in the
/// spec but most implementations are tolerant — we match upper-case as
/// canonical here.
fn parse_field(line: &str, name: &str) -> Option<String> {
    let pat = format!("<{}:", name);
    let start = line.find(&pat)?;
    let after_open = start + pat.len();
    let close = after_open + line[after_open..].find('>')?;
    let len: usize = line[after_open..close].parse().ok()?;
    let value_start = close + 1;
    let value_end = (value_start + len).min(line.len());
    Some(line[value_start..value_end].to_string())
}

/// Default ADIF log path for an app: `~/<app>_log.adi`.
/// Matches MSK2K's `~/msk2k_log.adi` convention.
pub fn default_adif_path(app_name: &str) -> PathBuf {
    let home = dirs::home_dir().unwrap_or_else(|| PathBuf::from("."));
    home.join(format!("{}_log.adi", app_name.to_ascii_lowercase()))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp_path(suffix: &str) -> PathBuf {
        let mut p = std::env::temp_dir();
        p.push(format!("dx_adif_test_{}_{}.adi",
            std::process::id() as u64,
            suffix));
        let _ = std::fs::remove_file(&p);
        p
    }

    #[test]
    fn write_and_read_qso() {
        let path = temp_path("write_and_read");
        let logger = AdifLogger::new(&path, "MSK144+");
        let rec = QsoRecord::new(
            "I3FGX".into(), "GW4WND".into(),
            1714200000000, 1714200060000,
            "2M".into(), Some(144.360),
            "MSK144", "+10", Some("+05"),
            Some("JN55".into()),
        );
        logger.log_qso(&rec).unwrap();
        let read = logger.read_all().unwrap();
        assert_eq!(read.len(), 1);
        assert_eq!(read[0].call, "I3FGX");
        assert_eq!(read[0].gridsquare.as_deref(), Some("JN55"));
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn append_multiple() {
        let path = temp_path("append_multiple");
        let logger = AdifLogger::new(&path, "MSK144+");
        for i in 0..3 {
            let rec = QsoRecord::new(
                format!("CALL{}", i), "GW4WND".into(),
                1714200000000, 1714200060000,
                "2M".into(), None,
                "MSK144", "+10", None, None,
            );
            logger.log_qso(&rec).unwrap();
        }
        let read = logger.read_all().unwrap();
        assert_eq!(read.len(), 3);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn adif_field_format() {
        assert_eq!(adif_field("CALL", "GW4WND"), "<CALL:6>GW4WND");
    }
}
