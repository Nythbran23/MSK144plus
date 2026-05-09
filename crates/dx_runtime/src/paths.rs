// crates/dx_runtime/src/paths.rs
//
// Filesystem layout for any app built on dx_runtime.
//
// All app state lives under `~/.<app_name>/` (matches MSK2K's `~/.msk2k`
// convention). Cross-platform via `dirs::home_dir()`:
//   - macOS:   /Users/<name>/.<app_name>/
//   - Linux:   /home/<name>/.<app_name>/
//   - Windows: C:\Users\<name>\.<app_name>\
//
// Layout:
//
//   ~/.<app_name>/
//   ├── config.toml                          ← persistent settings
//   ├── <app_name>.sqlite                    ← decodes + heard_calls + qsos
//   ├── logs/
//   │   └── <app_name>_YYYYMMDD.log          ← daily-rotated log files
//   └── captures/
//       └── YYYYMMDD/
//           └── HHMMSS_<call>.wav            ← auto-saved decode WAVs
//
// Optional, matching MSK2K's `~/msk2k_log.adi`:
//   ~/<app_name>_log.adi                     ← ADIF log
//
// Use `Paths::new(app_name)` to get all paths for an app.

use std::path::PathBuf;
use anyhow::Result;

/// Resolved filesystem paths for an app. All paths are absolute.
#[derive(Debug, Clone)]
pub struct Paths {
    /// Lower-case app identifier, e.g. "msk144plus".
    pub app_name: String,
    /// `~/.<app_name>/`
    pub config_dir: PathBuf,
    /// `~/.<app_name>/config.toml`
    pub config_file: PathBuf,
    /// `~/.<app_name>/<app_name>.sqlite`
    pub db_file: PathBuf,
    /// `~/.<app_name>/logs/`
    pub log_dir: PathBuf,
    /// `~/.<app_name>/captures/`
    pub captures_dir: PathBuf,
    /// `~/<app_name>_log.adi` — ADIF log location matching MSK2K convention.
    pub adif_file: PathBuf,
}

impl Paths {
    /// Build paths for an app using `dirs::home_dir()` as the root.
    /// Does NOT create directories — call `ensure_dirs()` for that.
    ///
    /// `app_name` should be lower-case ASCII without spaces, e.g.
    /// "msk144plus", "fsk441plus", "msk2k", or "dxshop" for the unified app.
    pub fn new(app_name: &str) -> Self {
        let home = dirs::home_dir().unwrap_or_else(|| PathBuf::from("."));
        Self::new_with_root(app_name, &home)
    }

    /// Build paths rooted at an explicit directory (instead of `$HOME`).
    /// Used by the `--config-dir` CLI flag for running multiple instances
    /// side-by-side. The given root is treated as if it were `$HOME` —
    /// e.g. `Paths::new_with_root("msk144plus", "/tmp/instB")` produces
    /// `/tmp/instB/.msk144plus/`, etc.
    pub fn new_with_root(app_name: &str, root: &std::path::Path) -> Self {
        let app = app_name.to_ascii_lowercase();
        let config_dir = root.join(format!(".{}", app));
        let config_file = config_dir.join("config.toml");
        let db_file = config_dir.join(format!("{}.sqlite", app));
        let log_dir = config_dir.join("logs");
        let captures_dir = config_dir.join("captures");
        let adif_file = root.join(format!("{}_log.adi", app));
        Self {
            app_name: app,
            config_dir,
            config_file,
            db_file,
            log_dir,
            captures_dir,
            adif_file,
        }
    }

    /// Create all directories that the app may write to. Idempotent.
    pub fn ensure_dirs(&self) -> Result<()> {
        std::fs::create_dir_all(&self.config_dir)?;
        std::fs::create_dir_all(&self.log_dir)?;
        std::fs::create_dir_all(&self.captures_dir)?;
        Ok(())
    }

    /// Return the path for today's log file (UTC date).
    pub fn log_file_today(&self) -> PathBuf {
        let date = chrono::Utc::now().format("%Y%m%d").to_string();
        self.log_dir.join(format!("{}_{}.log", self.app_name, date))
    }

    /// Return the directory for today's captures (UTC date), creating it
    /// if needed.
    pub fn captures_dir_today(&self) -> Result<PathBuf> {
        let date = chrono::Utc::now().format("%Y%m%d").to_string();
        let dir = self.captures_dir.join(date);
        std::fs::create_dir_all(&dir)?;
        Ok(dir)
    }

    /// Build a capture filename for a decode. UTC HHMMSS prefix + sanitised
    /// callsign suffix.
    ///
    /// All non-alphanumeric characters in the callsign (including `/` from
    /// portable indicators like `DL/G7VZN`) are replaced with `_` to keep
    /// the filename a single path component.
    ///
    /// Example: `141532_DL_G7VZN.wav`
    pub fn capture_path(&self, callsign: &str) -> Result<PathBuf> {
        let dir = self.captures_dir_today()?;
        let time = chrono::Utc::now().format("%H%M%S").to_string();
        let sane: String = callsign
            .chars()
            .map(|c| if c.is_ascii_alphanumeric() { c } else { '_' })
            .collect();
        Ok(dir.join(format!("{}_{}.wav", time, sane)))
    }

    /// Like `capture_path`, but uses the supplied "HHMMSS" timestamp for
    /// the filename rather than `now`. This is the preferred entry point
    /// for decoder-driven saves: the decode happened in a specific 15-s
    /// slot, and the file should be labelled with that slot's end time
    /// — not the wall-clock instant when LDPC happened to finish.
    pub fn capture_path_at(&self, slot_utc_hhmmss: &str, callsign: &str) -> Result<PathBuf> {
        let dir = self.captures_dir_today()?;
        let sane: String = callsign
            .chars()
            .map(|c| if c.is_ascii_alphanumeric() { c } else { '_' })
            .collect();
        Ok(dir.join(format!("{}_{}.wav", slot_utc_hhmmss, sane)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn paths_msk144plus() {
        let p = Paths::new("msk144plus");
        assert_eq!(p.app_name, "msk144plus");
        assert!(p.config_dir.ends_with(".msk144plus"));
        assert!(p.config_file.ends_with("config.toml"));
        assert!(p.db_file.ends_with("msk144plus.sqlite"));
        assert!(p.log_dir.ends_with("logs"));
        assert!(p.captures_dir.ends_with("captures"));
        assert!(p.adif_file.to_string_lossy().ends_with("msk144plus_log.adi"));
    }

    #[test]
    fn capture_filename_sanitises() {
        let p = Paths::new("msk144plus");
        let path = p.capture_path("DL/G7VZN").unwrap();
        let fname = path.file_name().unwrap().to_string_lossy();
        // / replaced with _ so the filename is a single path component
        assert!(fname.contains("DL_G7VZN"), "got fname: {}", fname);
        assert!(fname.ends_with(".wav"));
    }

    #[test]
    fn case_insensitive() {
        let p = Paths::new("MSK144PLUS");
        assert_eq!(p.app_name, "msk144plus");
    }
}
