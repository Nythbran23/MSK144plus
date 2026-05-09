// crates/dx_runtime/src/settings.rs
//
// Application configuration. Mirrors MSK2K's settings.rs structure so the
// same shape applies across MSK2K, FSK441+, MSK144+, and the future
// unified app.
//
// Layout:
//   ~/.<app_name>/config.toml
//
// Sections:
//   [audio]    input/output device, levels
//   [station]  callsign, grid, band, hamlib config
//   [decoder]  fc, ntol, depth, mode-specific tuning knobs
//   [ui]       window size, max log entries, etc.
//
// Env overrides (non-destructive — only override if env var is set):
//   <APP>_CALLSIGN     — e.g. MSK144PLUS_CALLSIGN=GW4WND
//   <APP>_RX_IN        — audio input device name or substring
//   <APP>_TX_OUT       — audio output device name or substring
//
// Use Settings::load_or_default(app_name, &paths) at startup,
// settings.save(&paths.config_file) on changes.

use serde::{Deserialize, Serialize};
use std::path::Path;
use anyhow::{Context, Result};
use crate::paths::Paths;

/// Top-level configuration. Mirrors MSK2K's `Config` shape.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Settings {
    #[serde(default)]
    pub audio: AudioConfig,
    #[serde(default)]
    pub station: StationConfig,
    #[serde(default)]
    pub decoder: DecoderConfig,
    #[serde(default)]
    pub ui: UiConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct AudioConfig {
    /// Input device name (None = system default)
    pub input_device: Option<String>,
    /// Output device name (None = system default)
    pub output_device: Option<String>,
    /// Target sample rate. MSK144 = 12000, MSK2K = 48000, FSK441 = 11025.
    pub sample_rate: u32,
    /// CPAL buffer size in samples
    pub buffer_size: usize,
    /// Input audio level (0.0 - 1.0). UI slider.
    pub input_level: f32,
    /// Output audio level (0.0 - 1.0). For TX.
    pub output_level: f32,
}

impl Default for AudioConfig {
    fn default() -> Self {
        Self {
            input_device: None,
            output_device: None,
            sample_rate: 12000,
            buffer_size: 1024,
            input_level: 0.8,
            output_level: 0.8,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct StationConfig {
    /// Station callsign (always uppercase)
    pub callsign: String,
    /// Maidenhead locator (e.g. "IO82KM"). 4 or 6 chars.
    pub grid: Option<String>,
    /// Auto-reply to incoming CQ from TARGET
    pub auto_reply_cq: bool,
    /// Auto-send 73 after RR
    pub auto_73: bool,
    /// Band label (e.g. "2M", "70CM", "6M")
    pub band: Option<String>,
    /// Hamlib (rigctld) enabled
    pub hamlib_enabled: bool,
    /// rigctld host (default "localhost")
    pub rigctld_host: String,
    /// rigctld TCP port (default 4532)
    pub rigctld_port: u16,
    /// Rig model id for rigctld (e.g. "3081" for IC-9700, "3073" for IC-7300)
    pub rig_model: String,
    /// Serial port for the rig (e.g. "/dev/cu.usbmodem1421401")
    pub rig_port: String,
    /// Serial baud rate (e.g. "19200")
    pub rig_baud: String,
    /// Auto-launch rigctld as a child process when CAT enabled.
    /// If false, we just connect to an existing rigctld at host:port.
    pub auto_launch_rigctld: bool,
    /// TX audio output level (0.0 - 1.0)
    pub tx_level: f32,
    /// TX output device name (None = system default)
    pub tx_output_device: Option<String>,
    /// Slot period in seconds. IARU Region 1 specifies 30s for 144 MHz
    /// MSK144 operation; the WSJT-X default of 15s is a US/regional
    /// convention. Default here is 30s to match the protocol-correct
    /// R1 specification — operators in R2/R3 can change to 15s via
    /// the top-bar Period selector. Both stations in a QSO must use
    /// the same period to interoperate.
    pub slot_period_secs: u32,
    /// Which slot parity is ours: "Odd" or "Even"
    pub tx_parity: String,
    /// PTT delay in milliseconds before starting audio (gives rig time to key up)
    pub ptt_delay_ms: u32,

    /// Antenna 3-dB horizontal beamwidth in degrees. Used by the
    /// scatter-arc calculator to inset the optimal A/B beam headings
    /// from the arc edges by half-beamwidth, so the antenna's main
    /// lobe stays within the mutually-visible scatter zone.
    /// Typical 2 m yagi: ~50°. Typical 70 cm yagi: ~30°. Default 50°
    /// matches Roger's 4-element 2 m yagi.
    pub ant_bw_horiz: f32,
    /// Antenna 3-dB vertical beamwidth in degrees. Used to flag the
    /// midpoint elevation in red when the scatter point sits below
    /// the upper half of the main lobe. Default 50°.
    pub ant_bw_vert: f32,

    /// Send WSJT-X-format QSOLogged + LoggedADIF UDP datagrams on QSO
    /// completion. Compatible with Logger32, N1MM+, JTAlert, HRD,
    /// DX Lab Suite, Cloudlog UDP plugin, and anything else that
    /// listens for the WSJT-X log-broadcast format. Same trigger as
    /// the local ADIF file write — fire-and-forget, off by default.
    pub udp_logging_enabled: bool,
    /// Host to send the broadcast to. Default 127.0.0.1 (most loggers
    /// run on the same machine). Set to a remote IP if your logger
    /// lives elsewhere on the LAN.
    pub udp_logging_host: String,
    /// Port. WSJT-X / MSHV default is 2237; nearly all logger software
    /// listens here unless explicitly reconfigured.
    pub udp_logging_port: u16,

    /// Submit decode reports to PSK Reporter (the global propagation-
    /// data aggregator). When enabled, every successful decode of a
    /// non-self callsign is queued; batches are flushed every ~5
    /// seconds to report.pskreporter.info:4739 over UDP. Same destination
    /// used by WSJT-X and MSHV — decodes appear on the PSK Reporter
    /// map within ~10 minutes of submission. Off by default; enable
    /// when you want your station visible to the worldwide propagation
    /// database.
    ///
    /// Requires: callsign + 4-or-6-char grid in Station settings, and
    /// a rig CAT connection (so we know the absolute frequency to
    /// report). With no CAT, spots are silently skipped.
    pub psk_reporter_enabled: bool,

    /// Antenna description sent to PSK Reporter as free-text in the
    /// receiver record. Appears in the station info popup on the
    /// pskreporter.info map. Examples: "4-element 2 m yagi at 12 m",
    /// "Dipole at 8 m", "Vertical at ground level". Empty string
    /// means PSK Reporter shows the receiver record without antenna
    /// details — still works, just less informative.
    pub psk_reporter_antenna: String,
}

impl Default for StationConfig {
    fn default() -> Self {
        Self {
            callsign: "NOCALL".to_string(),
            grid: None,
            auto_reply_cq: false,
            auto_73: false,
            band: Some("2M".to_string()),
            hamlib_enabled: false,
            rigctld_host: "localhost".to_string(),
            rigctld_port: 4532,
            rig_model: String::new(),
            rig_port: String::new(),
            rig_baud: "19200".to_string(),
            auto_launch_rigctld: true,
            tx_level: 0.4,
            tx_output_device: None,
            slot_period_secs: 30,
            tx_parity: "Odd".to_string(),
            ptt_delay_ms: 0,
            ant_bw_horiz: 50.0,
            ant_bw_vert:  50.0,
            udp_logging_enabled: false,
            udp_logging_host: "127.0.0.1".to_string(),
            udp_logging_port: 2237,
            psk_reporter_enabled: false,
            psk_reporter_antenna: String::new(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct DecoderConfig {
    /// Centre audio frequency, Hz. MSK144 default 1500.
    pub fc_hz: f32,
    /// ±tolerance around fc, Hz.
    pub ntol_hz: f32,
    /// Depth string: "Fast", "Normal", "Deep". Mirrored from
    /// msk144plus_engine::Depth via simple string mapping; kept stringly so
    /// dx_runtime stays mode-agnostic.
    pub depth: String,
    /// Minimum xmax (sync correlation) to accept a candidate as a real
    /// decode. WSJT-X default ≈ 1.3; raise to 2.0+ to filter noise.
    pub xmin: f32,
    /// Enable MSK40 short-message decoding (single-station)
    pub msk40_enabled: bool,
    /// Enable the soft-bit accumulator decoder path (experimental).
    /// Runs in parallel with the standard SPD/avg-N pipeline. Decodes
    /// from this path appear in the UI with a ` [A]` suffix so they can
    /// be A/B compared against standard decodes for the same slot.
    pub accumulator_enabled: bool,
    /// Tighter ±frequency tolerance (Hz) used by the accumulator when a
    /// QSO is locked to a partner's frequency. Suppresses non-partner
    /// traffic from polluting soft-bit sums. Has no effect outside an
    /// active QSO; in that case the accumulator falls back to `ntol_hz`.
    pub accumulator_ntol_hz: f32,
}

impl Default for DecoderConfig {
    fn default() -> Self {
        Self {
            fc_hz: 1500.0,
            ntol_hz: 200.0,
            depth: "Deep".to_string(),
            xmin: 1.3,
            msk40_enabled: false,
            accumulator_enabled: false,
            accumulator_ntol_hz: 50.0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct UiConfig {
    /// Width × height in logical pixels
    pub window_width:  f32,
    pub window_height: f32,
    /// Logbook footer expanded
    pub logbook_expanded: bool,
    /// Max RX log entries kept in memory
    pub max_rx_entries: usize,
    /// Max SPOTS entries kept in memory
    pub max_spots_entries: usize,
}

impl Default for UiConfig {
    fn default() -> Self {
        Self {
            window_width:  1000.0,
            window_height:  600.0,
            logbook_expanded: false,
            max_rx_entries: 500,
            max_spots_entries: 200,
        }
    }
}

impl Settings {
    /// Load from a TOML file. Returns Err if the file is missing or invalid.
    pub fn load(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref();
        let s = std::fs::read_to_string(path)
            .with_context(|| format!("read settings from {}", path.display()))?;
        let cfg: Settings = toml::from_str(&s)
            .with_context(|| format!("parse settings from {}", path.display()))?;
        Ok(cfg)
    }

    /// Save to a TOML file. Creates parent directories as needed.
    pub fn save(&self, path: impl AsRef<Path>) -> Result<()> {
        let path = path.as_ref();
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let s = toml::to_string_pretty(self)?;
        // Atomic-ish: write temp + rename. Avoids half-written file if app
        // crashes during write.
        let tmp = path.with_extension("toml.tmp");
        std::fs::write(&tmp, s.as_bytes())
            .with_context(|| format!("write {} (tmp)", tmp.display()))?;
        std::fs::rename(&tmp, path)
            .with_context(|| format!("rename {} -> {}", tmp.display(), path.display()))?;
        Ok(())
    }

    /// Load existing config, or build a default one. Always applies env
    /// overrides afterwards. Recommended at startup.
    pub fn load_or_default(app_name: &str, paths: &Paths) -> Self {
        let mut cfg = match Self::load(&paths.config_file) {
            Ok(c) => c,
            Err(e) => {
                log::info!("[CFG] no config at {} ({}), using defaults",
                    paths.config_file.display(), e);
                Self::default()
            }
        };
        cfg.apply_env_overrides(app_name);
        cfg
    }

    /// Apply environment overrides. `app_name` should be lower-case;
    /// env vars are upper-case versions e.g. MSK144PLUS_CALLSIGN.
    pub fn apply_env_overrides(&mut self, app_name: &str) {
        let prefix = app_name.to_uppercase();

        if let Ok(cs) = std::env::var(format!("{}_CALLSIGN", prefix)) {
            let cs = cs.trim();
            if !cs.is_empty() {
                self.station.callsign = cs.to_uppercase();
                log::info!("[CFG] env override: callsign = {}", self.station.callsign);
            }
        }

        if let Ok(rx) = std::env::var(format!("{}_RX_IN", prefix)) {
            let rx = rx.trim();
            if !rx.is_empty() {
                self.audio.input_device = Some(rx.to_string());
                log::info!("[CFG] env override: input_device = {}", rx);
            }
        }

        if let Ok(tx) = std::env::var(format!("{}_TX_OUT", prefix)) {
            let tx = tx.trim();
            if !tx.is_empty() {
                self.audio.output_device = Some(tx.to_string());
                log::info!("[CFG] env override: output_device = {}", tx);
            }
        }

        if let Ok(grid) = std::env::var(format!("{}_GRID", prefix)) {
            let grid = grid.trim();
            if !grid.is_empty() {
                self.station.grid = Some(grid.to_uppercase());
                log::info!("[CFG] env override: grid = {}", grid.to_uppercase());
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_roundtrip() {
        let cfg = Settings::default();
        let s = toml::to_string_pretty(&cfg).unwrap();
        let cfg2: Settings = toml::from_str(&s).unwrap();
        assert_eq!(cfg.station.callsign, cfg2.station.callsign);
        assert_eq!(cfg.decoder.fc_hz, cfg2.decoder.fc_hz);
    }

    #[test]
    fn load_save_roundtrip() {
        let mut p = std::env::temp_dir();
        p.push(format!("dx_runtime_settings_test_{}.toml", std::process::id()));
        let _ = std::fs::remove_file(&p);

        let mut cfg = Settings::default();
        cfg.station.callsign = "GW4WND".to_string();
        cfg.station.grid = Some("IO82KM".to_string());
        cfg.decoder.fc_hz = 1450.0;
        cfg.save(&p).unwrap();

        let loaded = Settings::load(&p).unwrap();
        assert_eq!(loaded.station.callsign, "GW4WND");
        assert_eq!(loaded.station.grid.as_deref(), Some("IO82KM"));
        assert_eq!(loaded.decoder.fc_hz, 1450.0);

        let _ = std::fs::remove_file(&p);
    }

    #[test]
    fn missing_section_uses_default() {
        // Old config files might not have new sections — must still load
        let toml_str = r#"
[station]
callsign = "GW4WND"
"#;
        let cfg: Settings = toml::from_str(toml_str).unwrap();
        assert_eq!(cfg.station.callsign, "GW4WND");
        // decoder section was missing — uses default
        assert_eq!(cfg.decoder.fc_hz, 1500.0);
        assert_eq!(cfg.audio.sample_rate, 12000);
    }

    #[test]
    fn env_override_callsign() {
        std::env::set_var("MSK144PLUS_CALLSIGN", "gw4wnd");
        let mut cfg = Settings::default();
        cfg.apply_env_overrides("msk144plus");
        assert_eq!(cfg.station.callsign, "GW4WND");
        std::env::remove_var("MSK144PLUS_CALLSIGN");
    }
}
