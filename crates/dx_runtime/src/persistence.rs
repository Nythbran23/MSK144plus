// crates/dx_runtime/src/persistence.rs
//
// SQLite persistence for digital-mode receivers.
//
// Schema:
//
//   decodes — every successful decode, ever
//     id INTEGER PRIMARY KEY
//     utc TEXT NOT NULL                ← ISO8601 e.g. "2026-04-27T07:45:52Z"
//     mode TEXT NOT NULL                ← "MSK144" / "MSK40" / "FSK441" / "MSK2K"
//     fc_hz REAL NOT NULL               ← centre frequency the decoder was tuned to
//     freq_offset_hz REAL NOT NULL      ← measured offset from fc
//     xmax REAL NOT NULL                ← sync correlation strength
//     method TEXT NOT NULL              ← "spd-pat0", "avg-4", "msk40-pat3", etc
//     text TEXT NOT NULL                ← decoded message, e.g. "CQ I3FGX JN55"
//     callsign TEXT                     ← extracted call (HISCALL position) when standard format
//     grid TEXT                         ← extracted grid when standard format
//     wav_path TEXT                     ← relative path to saved WAV (NULL if not saved)
//     band_mhz REAL                     ← future: from CAT
//
//   heard_calls — running pool of stations heard, drives MSK40 hiscall rotation
//     callsign TEXT PRIMARY KEY
//     grid TEXT
//     first_heard TEXT NOT NULL
//     last_heard TEXT NOT NULL
//     n_decodes INTEGER NOT NULL DEFAULT 1
//     is_active INTEGER NOT NULL DEFAULT 1   ← 1 = include in MSK40 attempts
//
// schema_version is tracked in `meta` table so future migrations are safe.

use anyhow::{Context, Result};
use rusqlite::{params, Connection, OptionalExtension};
use std::path::{Path, PathBuf};
use std::sync::Mutex;

const CURRENT_SCHEMA_VERSION: i32 = 1;

/// Thread-safe SQLite handle. Wrap the connection in a Mutex so the
/// decoder thread, the UI thread, and any background tasks can all
/// read/write without holding their own connection.
pub struct Database {
    conn: Mutex<Connection>,
    #[allow(dead_code)]
    path: PathBuf,
}

/// One record to insert into the decodes table.
#[derive(Debug, Clone)]
pub struct DecodeRecord {
    pub utc: String,
    pub mode: String,
    pub fc_hz: f32,
    pub freq_offset_hz: f32,
    pub xmax: f32,
    pub method: String,
    pub text: String,
    pub callsign: Option<String>,
    pub grid: Option<String>,
    pub wav_path: Option<String>,
    pub band_mhz: Option<f32>,
}

/// One row from the heard_calls table.
#[derive(Debug, Clone)]
pub struct HeardCall {
    pub callsign: String,
    pub grid: Option<String>,
    pub first_heard: String,
    pub last_heard: String,
    pub n_decodes: u32,
    pub is_active: bool,
}

impl Drop for Database {
    /// Checkpoint the WAL on shutdown so the DB file is complete and
    /// can safely be copied or analysed externally without WAL replay.
    fn drop(&mut self) {
        if let Ok(conn) = self.conn.lock() {
            let _ = conn.execute_batch("PRAGMA wal_checkpoint(TRUNCATE);");
            log::info!("[DB] WAL checkpoint completed on shutdown");
        }
    }
}

impl Database {
    /// Open or create the SQLite database at the given path. Creates parent
    /// directories if needed and runs migrations to the current schema version.
    pub fn open(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref().to_path_buf();
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)
                .with_context(|| format!("create db parent dir {}", parent.display()))?;
        }
        let conn = Connection::open(&path)
            .with_context(|| format!("open sqlite db {}", path.display()))?;
        conn.pragma_update(None, "journal_mode", "WAL")?;
        conn.pragma_update(None, "synchronous", "NORMAL")?;
        let db = Self { conn: Mutex::new(conn), path };
        db.migrate()?;
        Ok(db)
    }

    fn migrate(&self) -> Result<()> {
        let conn = self.conn.lock().unwrap();
        conn.execute_batch(
            "CREATE TABLE IF NOT EXISTS meta (
                key   TEXT PRIMARY KEY,
                value TEXT NOT NULL
             );",
        )?;
        let current_version: Option<i32> = conn
            .query_row(
                "SELECT value FROM meta WHERE key = 'schema_version'",
                [],
                |r| r.get::<_, String>(0).map(|s| s.parse::<i32>().unwrap_or(0)),
            )
            .optional()?;
        let current_version = current_version.unwrap_or(0);

        if current_version < 1 {
            conn.execute_batch(
                "CREATE TABLE IF NOT EXISTS decodes (
                    id              INTEGER PRIMARY KEY AUTOINCREMENT,
                    utc             TEXT NOT NULL,
                    mode            TEXT NOT NULL,
                    fc_hz           REAL NOT NULL,
                    freq_offset_hz  REAL NOT NULL,
                    xmax            REAL NOT NULL,
                    method          TEXT NOT NULL,
                    text            TEXT NOT NULL,
                    callsign        TEXT,
                    grid            TEXT,
                    wav_path        TEXT,
                    band_mhz        REAL
                );
                CREATE INDEX IF NOT EXISTS idx_decodes_utc ON decodes(utc);
                CREATE INDEX IF NOT EXISTS idx_decodes_text ON decodes(text);
                CREATE INDEX IF NOT EXISTS idx_decodes_callsign ON decodes(callsign);

                CREATE TABLE IF NOT EXISTS heard_calls (
                    callsign     TEXT PRIMARY KEY,
                    grid         TEXT,
                    first_heard  TEXT NOT NULL,
                    last_heard   TEXT NOT NULL,
                    n_decodes    INTEGER NOT NULL DEFAULT 1,
                    is_active    INTEGER NOT NULL DEFAULT 1
                );
                CREATE INDEX IF NOT EXISTS idx_heard_last ON heard_calls(last_heard);
                CREATE INDEX IF NOT EXISTS idx_heard_active ON heard_calls(is_active);

                CREATE TABLE IF NOT EXISTS settings (
                    key TEXT PRIMARY KEY,
                    value TEXT NOT NULL
                );",
            )?;
        }

        // Bump schema version
        conn.execute(
            "INSERT OR REPLACE INTO meta (key, value) VALUES ('schema_version', ?1)",
            params![CURRENT_SCHEMA_VERSION.to_string()],
        )?;
        Ok(())
    }

    /// Insert a decode record. Returns the new row id.
    pub fn record_decode(&self, rec: &DecodeRecord) -> Result<i64> {
        let conn = self.conn.lock().unwrap();
        conn.execute(
            "INSERT INTO decodes (
                utc, mode, fc_hz, freq_offset_hz, xmax, method, text,
                callsign, grid, wav_path, band_mhz
             ) VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11)",
            params![
                rec.utc, rec.mode, rec.fc_hz, rec.freq_offset_hz, rec.xmax,
                rec.method, rec.text, rec.callsign, rec.grid,
                rec.wav_path, rec.band_mhz,
            ],
        )?;
        Ok(conn.last_insert_rowid())
    }

    /// Update an existing decode's wav_path field. Used after the WAV
    /// finishes being written.
    pub fn update_wav_path(&self, decode_id: i64, wav_path: &str) -> Result<()> {
        let conn = self.conn.lock().unwrap();
        conn.execute(
            "UPDATE decodes SET wav_path = ?1 WHERE id = ?2",
            params![wav_path, decode_id],
        )?;
        Ok(())
    }

    /// Record having heard a callsign. Inserts new entry or bumps the
    /// last_heard / n_decodes on existing.
    pub fn record_heard_call(
        &self,
        callsign: &str,
        grid: Option<&str>,
        utc: &str,
    ) -> Result<()> {
        let conn = self.conn.lock().unwrap();
        // Upsert: insert if new, else update last_heard + bump count
        conn.execute(
            "INSERT INTO heard_calls (callsign, grid, first_heard, last_heard, n_decodes, is_active)
             VALUES (?1, ?2, ?3, ?3, 1, 1)
             ON CONFLICT(callsign) DO UPDATE SET
                last_heard = excluded.last_heard,
                grid       = COALESCE(excluded.grid, heard_calls.grid),
                n_decodes  = heard_calls.n_decodes + 1",
            params![callsign, grid, utc],
        )?;
        Ok(())
    }

    /// Return the most-recently-heard active callsigns, up to `limit`.
    /// Used to populate the MSK40 hiscall rotation.
    pub fn recent_heard_calls(&self, limit: usize) -> Result<Vec<HeardCall>> {
        let conn = self.conn.lock().unwrap();
        let mut stmt = conn.prepare(
            "SELECT callsign, grid, first_heard, last_heard, n_decodes, is_active
             FROM heard_calls
             WHERE is_active = 1
             ORDER BY last_heard DESC
             LIMIT ?1"
        )?;
        let iter = stmt.query_map(params![limit as i64], |r| {
            Ok(HeardCall {
                callsign: r.get(0)?,
                grid: r.get(1)?,
                first_heard: r.get(2)?,
                last_heard: r.get(3)?,
                n_decodes: r.get::<_, i64>(4)? as u32,
                is_active: r.get::<_, i64>(5)? != 0,
            })
        })?;
        Ok(iter.filter_map(Result::ok).collect())
    }

    /// Set callsign active flag. Used to disable a stale call without deleting it.
    pub fn set_call_active(&self, callsign: &str, is_active: bool) -> Result<()> {
        let conn = self.conn.lock().unwrap();
        conn.execute(
            "UPDATE heard_calls SET is_active = ?1 WHERE callsign = ?2",
            params![if is_active { 1 } else { 0 }, callsign],
        )?;
        Ok(())
    }

    /// Total decodes ever recorded.
    pub fn total_decodes(&self) -> Result<u64> {
        let conn = self.conn.lock().unwrap();
        let n: i64 = conn.query_row("SELECT COUNT(*) FROM decodes", [], |r| r.get(0))?;
        Ok(n as u64)
    }

    /// Stash a free-form key=value setting (for things not in the TOML config —
    /// e.g. last-window-position or transient flags). Use TOML config for
    /// real settings.
    pub fn set_setting(&self, key: &str, value: &str) -> Result<()> {
        let conn = self.conn.lock().unwrap();
        conn.execute(
            "INSERT OR REPLACE INTO settings (key, value) VALUES (?1, ?2)",
            params![key, value],
        )?;
        Ok(())
    }

    pub fn get_setting(&self, key: &str) -> Result<Option<String>> {
        let conn = self.conn.lock().unwrap();
        Ok(conn
            .query_row("SELECT value FROM settings WHERE key = ?1",
                params![key], |r| r.get(0))
            .optional()?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp_db() -> Database {
        let mut p = std::env::temp_dir();
        p.push(format!("dx_runtime_test_{}.sqlite",
            std::process::id() as u64 + std::ptr::addr_of!(p) as u64));
        let _ = std::fs::remove_file(&p);
        Database::open(&p).unwrap()
    }

    #[test]
    fn open_and_record_decode() {
        let db = temp_db();
        let id = db.record_decode(&DecodeRecord {
            utc: "2026-04-27T07:45:52Z".into(),
            mode: "MSK144".into(),
            fc_hz: 1500.0,
            freq_offset_hz: 40.0,
            xmax: 3.5,
            method: "spd-pat0".into(),
            text: "CQ I3FGX JN55".into(),
            callsign: Some("I3FGX".into()),
            grid: Some("JN55".into()),
            wav_path: None,
            band_mhz: Some(144.36),
        }).unwrap();
        assert!(id > 0);
        assert_eq!(db.total_decodes().unwrap(), 1);
    }

    #[test]
    fn heard_calls_upsert() {
        let db = temp_db();
        db.record_heard_call("I3FGX", Some("JN55"), "2026-04-27T07:00:00Z").unwrap();
        db.record_heard_call("I3FGX", Some("JN55"), "2026-04-27T07:30:00Z").unwrap();
        db.record_heard_call("DL7OAP", Some("JO62"), "2026-04-27T07:15:00Z").unwrap();
        let calls = db.recent_heard_calls(10).unwrap();
        assert_eq!(calls.len(), 2);
        // Most recent first
        assert_eq!(calls[0].callsign, "I3FGX");
        assert_eq!(calls[0].n_decodes, 2);
        assert_eq!(calls[1].callsign, "DL7OAP");
        assert_eq!(calls[1].n_decodes, 1);
    }

    #[test]
    fn settings_kv() {
        let db = temp_db();
        db.set_setting("foo", "bar").unwrap();
        assert_eq!(db.get_setting("foo").unwrap().as_deref(), Some("bar"));
        assert_eq!(db.get_setting("missing").unwrap(), None);
        db.set_setting("foo", "baz").unwrap();
        assert_eq!(db.get_setting("foo").unwrap().as_deref(), Some("baz"));
    }
}
