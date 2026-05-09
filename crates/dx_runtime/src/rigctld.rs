// crates/dx_runtime/src/rigctld.rs
//
// rigctld process launcher with RAII cleanup.
//
// Lifted in shape from MSK2K's engine/runtime.rs: locates a bundled
// rigctld next to the running binary first, falls back to system PATH;
// spawns it with the requested model/port/baud; kills cleanly on Drop
// including a PTT-off command via TCP to release the radio's transmit
// state before terminating.
//
// Usage:
//   let _guard = RigctldLauncher::launch(&RigctldOpts {
//       model: "3081", port: "/dev/cu.usbmodem1421401",
//       baud: 19200, listen_port: 4532,
//   })?;
//   // _guard kills rigctld on drop
//
// After launch(), wait ~800ms before connecting HamlibClient so rigctld
// has time to bind its TCP port.

use std::path::PathBuf;
use std::process::{Child, Command};
use std::time::Duration;
use anyhow::{Context, Result};

#[derive(Debug, Clone)]
pub struct RigctldOpts {
    /// Hamlib model number (string, e.g. "3081" for IC-9700, "3073" for IC-7300)
    pub model: String,
    /// Serial device path (e.g. "/dev/cu.usbmodem1421401" on macOS,
    /// "COM4" on Windows, "/dev/ttyUSB0" on Linux)
    pub port: String,
    /// Serial baud rate (e.g. 19200)
    pub baud: u32,
    /// TCP port rigctld will listen on (default 4532)
    pub listen_port: u16,
}

/// Locate the rigctld binary. Search order:
///   1. `<exe_dir>/rigctld` (bundled alongside our binary)
///   2. `<exe_dir>/tools/rigctld`
///   3. `rigctld` from system PATH
pub fn find_rigctld() -> String {
    let binary_name = if cfg!(target_os = "windows") { "rigctld.exe" } else { "rigctld" };

    if let Ok(mut path) = std::env::current_exe() {
        path.pop();
        let local: PathBuf = path.join(binary_name);
        if local.exists() {
            log::info!("[RIGCTLD] Found bundled at {:?}", local);
            return local.to_string_lossy().into_owned();
        }
        let tools: PathBuf = path.join("tools").join(binary_name);
        if tools.exists() {
            log::info!("[RIGCTLD] Found bundled at {:?}", tools);
            return tools.to_string_lossy().into_owned();
        }
    }
    log::info!("[RIGCTLD] Using system PATH lookup for {}", binary_name);
    binary_name.to_string()
}

/// RAII guard around a child rigctld process. Kills the process when dropped,
/// after first sending PTT-off via TCP (best effort) so the rig doesn't get
/// stuck in transmit.
pub struct ProcessGuard {
    child: Child,
    listen_port: u16,
}

impl ProcessGuard {
    /// Process id, useful for diagnostics.
    pub fn pid(&self) -> u32 { self.child.id() }

    /// Has the child exited on its own?
    pub fn has_exited(&mut self) -> Result<bool> {
        match self.child.try_wait()? {
            Some(_) => Ok(true),
            None => Ok(false),
        }
    }
}

impl Drop for ProcessGuard {
    fn drop(&mut self) {
        log::info!("[RIGCTLD] Drop guard: shutting down rigctld (pid={})",
            self.child.id());

        // Best-effort: send T 0 so we don't leave rig in transmit
        let addr = format!("127.0.0.1:{}", self.listen_port);
        if let Ok(parsed) = addr.parse() {
            if let Ok(mut stream) = std::net::TcpStream::connect_timeout(
                &parsed, Duration::from_millis(500))
            {
                use std::io::Write;
                let _ = stream.write_all(b"T 0\n");
                let _ = stream.flush();
                std::thread::sleep(Duration::from_millis(200));
            }
        }

        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

pub struct RigctldLauncher;

impl RigctldLauncher {
    /// Spawn rigctld with the given options. Returns a ProcessGuard that
    /// will kill rigctld when dropped.
    ///
    /// Caller should sleep ~800ms after this returns to give rigctld time
    /// to bind its TCP listen port before the HamlibClient connects.
    pub fn launch(opts: &RigctldOpts) -> Result<ProcessGuard> {
        log::info!("[RIGCTLD] Launching: model={} port={} baud={} listen=4532+",
            opts.model, opts.port, opts.baud);

        // On Linux, suppress DTR/RTS toggle on serial open (some rigs reboot
        // when DTR pulses). MSK2K does this; we mirror.
        #[cfg(target_os = "linux")]
        {
            let _ = Command::new("stty")
                .args(&["-F", &opts.port, "-hupcl", "-crtscts", &opts.baud.to_string()])
                .output();
        }

        let mut cmd = Command::new(find_rigctld());
        cmd.args(&[
            "-m", &opts.model,
            "-r", &opts.port,
            "-s", &opts.baud.to_string(),
            "-t", &opts.listen_port.to_string(),
            "-P", "RIG",  // PTT type: use rig command (T 1/T 0)
        ]);

        // On Windows, hide the console window
        #[cfg(target_os = "windows")]
        {
            use std::os::windows::process::CommandExt;
            cmd.creation_flags(0x08000000); // CREATE_NO_WINDOW
        }

        let child = cmd.spawn()
            .with_context(|| format!(
                "spawn rigctld (model={}, port={}, baud={})",
                opts.model, opts.port, opts.baud))?;

        log::info!("[RIGCTLD] Launched (pid={})", child.id());

        Ok(ProcessGuard { child, listen_port: opts.listen_port })
    }
}

/// List available serial ports on this host. Returns names like
/// "/dev/cu.usbmodem1421401" on macOS or "COM3" on Windows.
///
/// On macOS we look in /dev/cu.* (USB CDC, Bluetooth).
/// On Linux we look in /dev/ttyUSB* and /dev/ttyACM*.
/// On Windows we read the registry's Hardware\\DEVICEMAP\\SERIALCOMM key —
/// or fall back to scanning COM1..COM32 (for now we just list COM1..COM16).
pub fn list_serial_ports() -> Vec<String> {
    let mut ports = Vec::new();

    #[cfg(target_os = "macos")]
    {
        if let Ok(entries) = std::fs::read_dir("/dev") {
            for e in entries.flatten() {
                let name = e.file_name().to_string_lossy().to_string();
                if name.starts_with("cu.") {
                    ports.push(format!("/dev/{}", name));
                }
            }
        }
    }

    #[cfg(target_os = "linux")]
    {
        if let Ok(entries) = std::fs::read_dir("/dev") {
            for e in entries.flatten() {
                let name = e.file_name().to_string_lossy().to_string();
                if name.starts_with("ttyUSB") || name.starts_with("ttyACM") {
                    ports.push(format!("/dev/{}", name));
                }
            }
        }
    }

    #[cfg(target_os = "windows")]
    {
        // Quick scan: try opening COM1..COM16, list any that exist
        for n in 1..=16 {
            let p = format!("COM{}", n);
            if std::path::Path::new(&format!(r"\\.\{}", p)).exists() {
                ports.push(p);
            }
        }
    }

    ports.sort();
    ports
}

/// Common Hamlib rig model presets. Keep this minimal; more can be added
/// as users need them. Returned as (model_id, display_name) tuples.
pub fn common_rig_models() -> Vec<(&'static str, &'static str)> {
    vec![
        ("3073", "Icom IC-7300"),
        ("3081", "Icom IC-9700"),
        ("3079", "Icom IC-705"),
        ("3061", "Icom IC-7100"),
        ("3041", "Icom IC-7610"),
        ("1042", "Yaesu FT-991/A"),
        ("1043", "Yaesu FT-DX10"),
        ("1037", "Yaesu FT-DX101D/MP"),
        ("1035", "Yaesu FT-450"),
        ("2055", "Kenwood TS-590S/SG"),
        ("2057", "Kenwood TS-890S"),
        ("2052", "Kenwood TS-2000"),
        ("3081", "Elecraft K3/K3S"),
        ("1", "Hamlib dummy (testing)"),
        ("2", "rigctld NET (already running)"),
    ]
}
