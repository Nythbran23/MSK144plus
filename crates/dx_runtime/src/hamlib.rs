// crates/dx_runtime/src/hamlib.rs
//
// rigctld (Hamlib daemon) TCP client.
//
// Design choice: blocking TCP in a dedicated thread, std::sync::mpsc for
// command + update channels. Avoids pulling tokio into dx_runtime, since
// a single rigctld connection is trivially served by one thread.
//
// Wire protocol (rigctld port 4532, default):
//   "f\n"   → reads back current frequency in Hz, e.g. "144360000\n"
//   "m\n"   → reads back mode + passband, e.g. "USB\n2400\n"
//   "T 1\n" → set PTT on (returns "RPRT 0\n" on success)
//   "T 0\n" → set PTT off
//
// Lifted in spirit from MSK2K's engine/hamlib.rs (which uses tokio); same
// reconnect-on-disconnect behaviour, same UI signalling.

use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;
use std::sync::mpsc::{channel, Receiver, Sender};
use std::thread;
use std::time::Duration;

#[derive(Debug, Clone)]
pub enum HamlibCmd {
    Ptt(bool),
    GetFreq,
    GetMode,
    /// Set the rig's dial frequency to this value (Hz). Sent as
    /// rigctld `F <hz>\n`. Worker responds by re-querying the rig
    /// (rigctld returns `RPRT n` on success/failure but not the
    /// new freq), so the UI's displayed frequency follows up via
    /// the next poll-tick GetFreq.
    SetFreq(u64),
    Disconnect,
}

#[derive(Debug, Clone)]
pub struct HamlibUpdate {
    /// Last-known dial frequency in Hz, or None if CAT disconnected
    pub freq_hz: Option<u64>,
    /// Last-known mode (e.g. "USB"), or None if not queried
    pub mode: Option<String>,
    /// Whether the connection is currently live
    pub connected: bool,
}

/// Handle for sending commands to the hamlib worker thread. Drop the handle
/// to disconnect cleanly.
pub struct HamlibClient {
    cmd_tx: Sender<HamlibCmd>,
}

impl HamlibClient {
    /// Connect to rigctld at host:port, spawning a worker thread.
    /// `update_tx` receives `HamlibUpdate` events on connect/disconnect/freq
    /// changes.
    ///
    /// The worker auto-reconnects with 5-second backoff on failure.
    /// It also polls `f\n` once per `poll_interval` so the UI stays in sync
    /// without explicit GetFreq commands.
    pub fn spawn(
        host: String,
        port: u16,
        poll_interval: Duration,
        update_tx: Sender<HamlibUpdate>,
    ) -> Self {
        let (cmd_tx, cmd_rx) = channel::<HamlibCmd>();
        let addr = format!("{}:{}", host, port);
        thread::Builder::new()
            .name("hamlib".into())
            .spawn(move || run_worker(addr, cmd_rx, update_tx, poll_interval))
            .expect("spawn hamlib worker");
        Self { cmd_tx }
    }

    pub fn set_ptt(&self, active: bool) {
        let _ = self.cmd_tx.send(HamlibCmd::Ptt(active));
    }

    pub fn refresh(&self) {
        let _ = self.cmd_tx.send(HamlibCmd::GetFreq);
    }

    pub fn refresh_mode(&self) {
        let _ = self.cmd_tx.send(HamlibCmd::GetMode);
    }

    /// Send a frequency change to the rig. Hz, absolute. The worker
    /// just dispatches to rigctld's `F` command and reads the RPRT
    /// response; the UI's displayed frequency catches up on the
    /// next poll-tick GetFreq.
    pub fn set_freq(&self, hz: u64) {
        let _ = self.cmd_tx.send(HamlibCmd::SetFreq(hz));
    }
}

impl Drop for HamlibClient {
    fn drop(&mut self) {
        let _ = self.cmd_tx.send(HamlibCmd::Disconnect);
    }
}

fn run_worker(
    addr: String,
    cmd_rx: Receiver<HamlibCmd>,
    update_tx: Sender<HamlibUpdate>,
    poll_interval: Duration,
) {
    log::info!("[HAMLIB] Worker started, connecting to {}", addr);

    'reconnect: loop {
        // Resolve the address (handles hostnames like "localhost", not just IPs)
        use std::net::ToSocketAddrs;
        let socket_addr = match addr.to_socket_addrs() {
            Ok(mut iter) => match iter.next() {
                Some(sa) => sa,
                None => {
                    log::error!("[HAMLIB] Address {} resolved to no entries", addr);
                    return;
                }
            },
            Err(e) => {
                log::error!("[HAMLIB] Bad address {}: {}", addr, e);
                return;
            }
        };

        // Try to connect (5-sec timeout)
        let stream = match TcpStream::connect_timeout(
            &socket_addr,
            Duration::from_secs(5),
        ) {
            Ok(s) => s,
            Err(e) => {
                log::warn!("[HAMLIB] Connect to {} failed: {}; retrying in 5s", addr, e);
                let _ = update_tx.send(HamlibUpdate {
                    freq_hz: None, mode: None, connected: false,
                });
                if wait_or_disconnect(&cmd_rx, Duration::from_secs(5)) { return; }
                continue 'reconnect;
            }
        };

        log::info!("[HAMLIB] Connected to rigctld at {}", addr);
        let _ = update_tx.send(HamlibUpdate {
            freq_hz: None, mode: None, connected: true,
        });

        // Configure short read/write timeouts so we never block forever on a
        // dead connection.
        let _ = stream.set_read_timeout(Some(Duration::from_secs(3)));
        let _ = stream.set_write_timeout(Some(Duration::from_secs(3)));

        let writer_stream = match stream.try_clone() {
            Ok(s) => s,
            Err(e) => {
                log::error!("[HAMLIB] try_clone failed: {}", e);
                continue 'reconnect;
            }
        };
        let mut writer = writer_stream;
        let mut reader = BufReader::new(stream);

        // Initial freq query
        if !send_cmd(&mut writer, "f\n") {
            continue 'reconnect;
        }
        if let Some(freq) = read_freq_response(&mut reader) {
            let _ = update_tx.send(HamlibUpdate {
                freq_hz: Some(freq), mode: None, connected: true,
            });
        } else {
            log::warn!("[HAMLIB] Initial freq read failed; reconnecting");
            let _ = update_tx.send(HamlibUpdate {
                freq_hz: None, mode: None, connected: false,
            });
            continue 'reconnect;
        }

        // Main loop: handle commands or auto-poll on timeout.
        loop {
            match cmd_rx.recv_timeout(poll_interval) {
                Ok(HamlibCmd::Disconnect) => {
                    log::info!("[HAMLIB] Disconnect requested");
                    return;
                }
                Ok(HamlibCmd::Ptt(active)) => {
                    let cmd = if active { "T 1\n" } else { "T 0\n" };
                    if !send_cmd(&mut writer, cmd) {
                        let _ = update_tx.send(HamlibUpdate {
                            freq_hz: None, mode: None, connected: false,
                        });
                        continue 'reconnect;
                    }
                    let _ = read_rprt_response(&mut reader);
                }
                Ok(HamlibCmd::SetFreq(hz)) => {
                    // rigctld: F <freq_hz>\n  → RPRT n
                    let cmd = format!("F {}\n", hz);
                    if !send_cmd(&mut writer, &cmd) {
                        let _ = update_tx.send(HamlibUpdate {
                            freq_hz: None, mode: None, connected: false,
                        });
                        continue 'reconnect;
                    }
                    let _ = read_rprt_response(&mut reader);
                    // Immediately re-query so the UI reflects the new
                    // value without waiting for the next poll-tick.
                    if !send_cmd(&mut writer, "f\n") {
                        continue 'reconnect;
                    }
                    if let Some(freq) = read_freq_response(&mut reader) {
                        let _ = update_tx.send(HamlibUpdate {
                            freq_hz: Some(freq), mode: None, connected: true,
                        });
                    }
                }
                Ok(HamlibCmd::GetFreq) | Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                    if !send_cmd(&mut writer, "f\n") {
                        let _ = update_tx.send(HamlibUpdate {
                            freq_hz: None, mode: None, connected: false,
                        });
                        continue 'reconnect;
                    }
                    match read_freq_response(&mut reader) {
                        Some(freq) => {
                            let _ = update_tx.send(HamlibUpdate {
                                freq_hz: Some(freq), mode: None, connected: true,
                            });
                        }
                        None => {
                            log::warn!("[HAMLIB] freq read failed; reconnecting");
                            let _ = update_tx.send(HamlibUpdate {
                                freq_hz: None, mode: None, connected: false,
                            });
                            continue 'reconnect;
                        }
                    }
                }
                Ok(HamlibCmd::GetMode) => {
                    if !send_cmd(&mut writer, "m\n") {
                        continue 'reconnect;
                    }
                    if let Some(mode) = read_mode_response(&mut reader) {
                        let _ = update_tx.send(HamlibUpdate {
                            freq_hz: None, mode: Some(mode), connected: true,
                        });
                    }
                }
                Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {
                    log::info!("[HAMLIB] Cmd channel disconnected, shutting down");
                    return;
                }
            }
        }
    }
}

fn send_cmd(writer: &mut TcpStream, cmd: &str) -> bool {
    match writer.write_all(cmd.as_bytes()) {
        Ok(_) => {
            log::debug!("[HAMLIB] TX: {:?}", cmd.trim_end());
            true
        }
        Err(e) => {
            log::warn!("[HAMLIB] write {:?} failed: {}", cmd.trim_end(), e);
            false
        }
    }
}

fn read_freq_response(reader: &mut BufReader<TcpStream>) -> Option<u64> {
    let mut buf = String::new();
    match reader.read_line(&mut buf) {
        Ok(0) => None,
        Ok(_) => buf.trim().parse::<u64>().ok(),
        Err(_) => None,
    }
}

fn read_mode_response(reader: &mut BufReader<TcpStream>) -> Option<String> {
    // "m" returns two lines: mode then passband. We only care about the mode.
    let mut buf = String::new();
    match reader.read_line(&mut buf) {
        Ok(0) | Err(_) => return None,
        Ok(_) => {}
    }
    let mode = buf.trim().to_string();
    // Drain the passband line (best effort)
    let mut bw = String::new();
    let _ = reader.read_line(&mut bw);
    if mode.is_empty() { None } else { Some(mode) }
}

fn read_rprt_response(reader: &mut BufReader<TcpStream>) -> Option<i32> {
    // PTT commands return "RPRT 0\n" on success, "RPRT -<N>\n" on error.
    let mut buf = String::new();
    if reader.read_line(&mut buf).is_err() { return None; }
    let s = buf.trim();
    if let Some(n) = s.strip_prefix("RPRT ") {
        n.parse::<i32>().ok()
    } else {
        None
    }
}

/// Block on the cmd channel for up to `dur`. Returns true if a Disconnect
/// command came in (in which case caller should exit), false otherwise.
fn wait_or_disconnect(cmd_rx: &Receiver<HamlibCmd>, dur: Duration) -> bool {
    match cmd_rx.recv_timeout(dur) {
        Ok(HamlibCmd::Disconnect) => true,
        Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => true,
        _ => false,
    }
}

/// Convenience: convert MHz dial frequency to a band label like "2M", "70CM".
/// Returns None if outside known amateur bands.
pub fn band_from_freq_hz(freq_hz: u64) -> Option<&'static str> {
    let mhz = freq_hz as f64 / 1_000_000.0;
    Some(match mhz {
        f if (1.8..=2.0).contains(&f)     => "160M",
        f if (3.5..=4.0).contains(&f)     => "80M",
        f if (5.250..=5.450).contains(&f) => "60M",
        f if (7.0..=7.3).contains(&f)     => "40M",
        f if (10.1..=10.15).contains(&f)  => "30M",
        f if (14.0..=14.35).contains(&f)  => "20M",
        f if (18.06..=18.17).contains(&f) => "17M",
        f if (21.0..=21.45).contains(&f)  => "15M",
        f if (24.89..=24.99).contains(&f) => "12M",
        f if (28.0..=29.7).contains(&f)   => "10M",
        f if (50.0..=54.0).contains(&f)   => "6M",
        f if (70.0..=70.5).contains(&f)   => "4M",
        f if (144.0..=148.0).contains(&f) => "2M",
        f if (222.0..=225.0).contains(&f) => "1.25M",
        f if (430.0..=440.0).contains(&f) => "70CM",
        f if (902.0..=928.0).contains(&f) => "33CM",
        f if (1240.0..=1300.0).contains(&f) => "23CM",
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn band_lookup_2m() {
        assert_eq!(band_from_freq_hz(144_360_000), Some("2M"));
        assert_eq!(band_from_freq_hz(145_500_000), Some("2M"));
    }

    #[test]
    fn band_lookup_70cm() {
        assert_eq!(band_from_freq_hz(432_200_000), Some("70CM"));
    }

    #[test]
    fn band_lookup_outside() {
        assert_eq!(band_from_freq_hz(100_000_000), None);
        assert_eq!(band_from_freq_hz(2_500_000_000), None);
    }

    #[test]
    fn band_lookup_hf() {
        assert_eq!(band_from_freq_hz(14_074_000),  Some("20M"));
        assert_eq!(band_from_freq_hz(7_074_000),   Some("40M"));
        assert_eq!(band_from_freq_hz(28_500_000),  Some("10M"));
    }
}
