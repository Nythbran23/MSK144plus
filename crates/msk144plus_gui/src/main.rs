// crates/msk144plus_gui/src/main.rs

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod audio;
mod decoder;
mod app;
mod transmitter;
mod spectrum;
mod geo;

use std::sync::Arc;
use std::path::PathBuf;
use dx_runtime::{Paths, Settings, Database, Recorder, SaveConfig};

const APP_NAME: &str = "msk144plus";

/// Parse CLI flags. Currently supports:
///   --config-dir <path>   override $HOME for paths (lets you run two
///                         instances side-by-side with different state).
///   --my-call <CALL>      override callsign for this run (test / dev).
struct CliArgs {
    config_root: Option<PathBuf>,
    my_call: Option<String>,
}

fn parse_cli() -> CliArgs {
    let mut args = std::env::args().skip(1);
    let mut config_root: Option<PathBuf> = None;
    let mut my_call: Option<String> = None;
    while let Some(a) = args.next() {
        match a.as_str() {
            "--config-dir" => {
                config_root = args.next().map(PathBuf::from);
            }
            "--my-call" => {
                my_call = args.next();
            }
            "-h" | "--help" => {
                eprintln!("MSK144+ usage:");
                eprintln!("  msk144plus_rx [--config-dir <path>] [--my-call <CALL>]");
                eprintln!();
                eprintln!("  --config-dir <path>   Use <path> as the root for state");
                eprintln!("                        (instead of $HOME). State will live");
                eprintln!("                        at <path>/.msk144plus/.");
                eprintln!("  --my-call <CALL>      Override callsign just for this run.");
                std::process::exit(0);
            }
            other => {
                eprintln!("Warning: ignoring unknown argument {:?}", other);
            }
        }
    }
    CliArgs { config_root, my_call }
}

fn main() -> Result<(), eframe::Error> {
    let cli = parse_cli();

    // ── 0. Configure Rayon to prevent Thread Starvation ───────────────────
    // Get total logical cores, leave 1 for IO/UI/Hamlib
    let available_cores = std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(2);
    
    // Ensure we always have at least 1 thread for math
    let rayon_threads = (available_cores - 1).max(1); 

    // Build the global pool. Using unwrap() is safe here because 
    // it's the very start of the app and nothing else has used Rayon yet.
    rayon::ThreadPoolBuilder::new()
        .num_threads(rayon_threads)
        .build_global()
        .unwrap();
    // ──────────────────────────────────────────────────────────────────────

    // ── 1. Filesystem layout ──────────────────────────────────────────────
    let paths = match &cli.config_root {
        Some(root) => Paths::new_with_root(APP_NAME, root),
        None => Paths::new(APP_NAME),
    };
    if let Err(e) = paths.ensure_dirs() {
        eprintln!("Failed to create app directories ({}): {}",
            paths.config_dir.display(), e);
    }

    // ── 2. Logger (file + stderr, daily rotated) ──────────────────────────
    if let Err(e) = dx_runtime::init_logger(&paths) {
        eprintln!("Logger init failed: {}", e);
    }
    log::info!("=== {} v{} starting ===", APP_NAME, env!("CARGO_PKG_VERSION"));
    log::info!("Config dir: {}", paths.config_dir.display());

    // ── 3. Load persistent settings ───────────────────────────────────────
    let mut settings = Settings::load_or_default(APP_NAME, &paths);

    // CLI override for callsign — useful for test loopback runs.
    if let Some(c) = &cli.my_call {
        log::info!("CLI override: callsign = {}", c);
        settings.station.callsign = c.to_uppercase();
    }

    // Make sure callsign matches "looks like a callsign" (3-10 alnum/slash)
    if settings.station.callsign.is_empty() {
        settings.station.callsign = "NOCALL".to_string();
    }
    log::info!("Callsign: {}  Grid: {:?}",
        settings.station.callsign, settings.station.grid);

    // ── 4. Open SQLite DB ─────────────────────────────────────────────────
    let db: Option<Arc<Database>> = match Database::open(&paths.db_file) {
        Ok(db) => {
            let total = db.total_decodes().unwrap_or(0);
            log::info!("DB: {} (total decodes ever: {})", paths.db_file.display(), total);
            Some(Arc::new(db))
        }
        Err(e) => {
            log::error!("Failed to open DB at {}: {}", paths.db_file.display(), e);
            None
        }
    };

    // ── 5. Recorder for auto-WAV-on-decode ────────────────────────────────
    // Pre/post roll = one slot period each, so each captured WAV
    // spans exactly two slots centred on the decode trigger. Picks
    // up the configured slot period (15 / 30) from the loaded
    // settings so the buffer is sized correctly for the operator's
    // preferred slot length.
    let pre_post_secs = settings.station.slot_period_secs;
    let recorder = Arc::new(Recorder::new(SaveConfig {
        sample_rate: 12000,
        pre_roll_secs: pre_post_secs,
        post_roll_secs: pre_post_secs,
        captures_root: paths.captures_dir.clone(),
    }));
    log::info!("Recorder: {}s rolling buffer, captures → {}",
        pre_post_secs * 2, paths.captures_dir.display());

    // ── 6. Launch GUI ─────────────────────────────────────────────────────
    let options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default()
            .with_inner_size([settings.ui.window_width, settings.ui.window_height])
            .with_min_inner_size([750.0, 400.0]),
        ..Default::default()
    };

    let paths_for_app = paths.clone();
    let settings_for_app = settings.clone();
    let db_for_app = db.clone();
    let recorder_for_app = recorder.clone();

    eframe::run_native(
        &format!("MSK144+ v{}", env!("CARGO_PKG_VERSION")),
        options,
        Box::new(move |_cc| Box::new(app::App::with_runtime(
            paths_for_app,
            settings_for_app,
            db_for_app,
            recorder_for_app,
        ))),
    )
}
