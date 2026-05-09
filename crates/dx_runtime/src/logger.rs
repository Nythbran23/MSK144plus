// crates/dx_runtime/src/logger.rs
//
// env_logger that ALSO writes to a daily-rotated file in the app's log dir.
//
// Design: a thin wrapper around env_logger. We give env_logger a custom
// `target` writer that fans the message out to:
//   - stderr (so `RUST_LOG=info ./app` still shows live output)
//   - today's log file `~/.<app>/logs/<app>_YYYYMMDD.log`
//
// Daily rotation is implemented by checking the date on every write; if the
// date has changed since the last write, we close the current file and open
// the new one. This is cheap and works without any background tasks.

use std::io::Write;
use std::path::PathBuf;
use std::sync::Mutex;
use anyhow::Result;
use crate::paths::Paths;

struct DailyFile {
    paths: Paths,
    current_date: String,
    file: Option<std::fs::File>,
}

impl DailyFile {
    fn new(paths: Paths) -> Self {
        Self { paths, current_date: String::new(), file: None }
    }

    fn write_line(&mut self, line: &str) {
        let today = chrono::Utc::now().format("%Y%m%d").to_string();
        if today != self.current_date || self.file.is_none() {
            // Open today's file
            let path: PathBuf = self.paths.log_file_today();
            // Make sure dir exists (in case ensure_dirs wasn't called)
            if let Some(parent) = path.parent() {
                let _ = std::fs::create_dir_all(parent);
            }
            self.file = std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(&path)
                .ok();
            self.current_date = today;
        }
        if let Some(f) = self.file.as_mut() {
            let _ = f.write_all(line.as_bytes());
            if !line.ends_with('\n') {
                let _ = f.write_all(b"\n");
            }
            let _ = f.flush();
        }
    }
}

/// Initialise the global logger.
///
/// Reads the standard `RUST_LOG` env var (defaults to "info" if unset).
/// Logs go to stderr AND to `<paths.log_dir>/<app>_YYYYMMDD.log`.
///
/// Should be called exactly once at app startup, before any log calls.
pub fn init(paths: &Paths) -> Result<()> {
    paths.ensure_dirs()?;
    let daily = std::sync::Arc::new(Mutex::new(DailyFile::new(paths.clone())));

    let mut builder = env_logger::Builder::from_env(
        env_logger::Env::default().default_filter_or("info")
    );
    let daily2 = daily.clone();
    builder.format(move |buf, record| {
        let ts = chrono::Utc::now().format("%Y-%m-%dT%H:%M:%SZ");
        let line = format!(
            "[{} {} {}] {}",
            ts,
            record.level(),
            record.target(),
            record.args()
        );
        // Stderr (env_logger's normal destination)
        writeln!(buf, "{}", line)?;
        // Also write to today's log file (best-effort; never fails)
        if let Ok(mut d) = daily2.lock() {
            d.write_line(&line);
        }
        Ok(())
    });
    // Try init; if already initialised (e.g. tests) this is harmless.
    let _ = builder.try_init();
    Ok(())
}
