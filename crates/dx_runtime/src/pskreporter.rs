//! PSK Reporter UDP client.
//!
//! PSK Reporter (https://pskreporter.info) is the global propagation-
//! data aggregator. Stations submit "I heard XX1ABC" spot reports via
//! UDP; the server aggregates them and plots them on a real-time
//! propagation map. Same destination is used by WSJT-X and MSHV.
//!
//! ## Wire protocol
//!
//! UDP datagrams to `report.pskreporter.info:4739`. IPFIX format
//! (RFC 5101) with a fixed enterprise OID for amateur radio
//! (Hamradio enterprise number 30351). Each packet:
//!
//! ```text
//! 16-byte header:
//!   u16  version       = 0x000A
//!   u16  total_length  (filled in last)
//!   u32  export_time   (unix seconds at flush time)
//!   u32  sequence_num  (monotonic per session, increments per packet)
//!   u32  observation_id (random u32 fixed for session)
//!
//! Optional template descriptors (first 3 packets, then once/hour):
//!   - Receiver Information template (Set ID 3, Link ID 0x50e2, 5 fields)
//!   - Sender Information template   (Set ID 2, Link ID 0x50e3, 7 fields)
//!
//! Receiver Information data block (one per packet):
//!   u16  template_id  = 0x50e2
//!   u16  block_length
//!   varlen rx_call    (u8 length-prefix + utf8 bytes)
//!   varlen rx_grid
//!   varlen prog_id
//!   varlen rx_antenna
//!   varlen rig_info
//!   [pad to 4-byte boundary]
//!
//! Sender Information data block (one block per packet, multiple records):
//!   u16  template_id  = 0x50e3
//!   u16  block_length
//!   for each spot:
//!     varlen sender_call
//!     5 bytes  freq_hz_BE       (5-byte big-endian uint, supports >4 GHz)
//!     s8       snr_db
//!     varlen sender_mode        ("MSK144")
//!     varlen sender_locator     (their grid, may be empty)
//!     u8       info_source       = 1 (REPORTER_SOURCE_AUTOMATIC)
//!     u32      time_seen         (unix seconds)
//!   [pad to 4-byte boundary]
//! ```
//!
//! ## Flush cadence
//!
//! Spots are queued, then a 5-second timer fires and flushes them in
//! a single UDP packet. Matches MSHV's cadence. The PSK Reporter dev
//! page is explicit: "timed sends must NOT be synchronised to the
//! system clock" — so the 5-second timer starts when the first spot
//! arrives, not at a fixed wall-clock interval.
//!
//! ## Server-side filtering
//!
//! PSK Reporter ignores duplicate spots of the same callsign on the
//! same band within a 20-minute window. We don't dedup client-side;
//! let the server handle it.

use std::net::{SocketAddr, ToSocketAddrs, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{sync_channel, Receiver, SyncSender};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

const PSK_REPORTER_HOST: &str = "report.pskreporter.info";
const PSK_REPORTER_PORT: u16 = 4739;

/// Flush cadence — spots queued during the gap are sent in one packet.
const FLUSH_INTERVAL: Duration = Duration::from_secs(5);

/// Maximum packet payload we'll build. PSK Reporter allows up to ~8 KB;
/// we cap a bit lower to leave room for IP/UDP headers without
/// fragmenting on the typical 1500 B MTU. If a flush would exceed
/// this, we send what we have and queue the rest for the next flush.
const MAX_PACKET_BYTES: usize = 1400;

/// IPFIX wire constants.
const IPFIX_VERSION: u16 = 0x000A;
const TEMPLATE_ID_RX: u16 = 0x50e2;
const TEMPLATE_ID_TX: u16 = 0x50e3;
const HAMRADIO_ENTERPRISE: u32 = 30351;
const REPORTER_SOURCE_AUTOMATIC: u8 = 1;

/// Information element IDs (within the Hamradio enterprise OID).
const IE_SENDER_CALLSIGN: u16  = 1;
const IE_RECEIVER_CALLSIGN: u16 = 2;
const IE_SENDER_LOCATOR: u16    = 3;
const IE_RECEIVER_LOCATOR: u16  = 4;
const IE_FREQUENCY: u16         = 5;
const IE_SNR: u16               = 6;
const IE_DECODING_SOFTWARE: u16 = 8;
const IE_ANTENNA_INFORMATION: u16 = 9;
const IE_MODE: u16              = 10;
const IE_INFORMATION_SOURCE: u16 = 11;
const IE_RIG_INFORMATION: u16   = 13;
/// Standard IPFIX IE (no enterprise bit).
const IE_DATETIME_SECONDS: u16  = 150;

/// One queued spot, sent from the GUI thread to the reporter worker.
#[derive(Debug, Clone)]
struct Spot {
    heard_call: String,
    heard_grid: Option<String>,
    /// Absolute frequency in Hz (rig dial + audio offset).
    freq_hz: u64,
    mode: String,
    /// Signal-to-noise ratio in dB. Clamped to i8 range at queue time.
    snr_db: i8,
    /// Unix seconds at the time the decode was heard. PSK Reporter
    /// uses this for the spot's plot timestamp.
    utc_secs: u32,
}

/// Configuration captured at spawn-time. Receiver info doesn't change
/// during a session; if the operator updates their settings we drop
/// and respawn.
#[derive(Debug, Clone)]
struct ReceiverInfo {
    rx_call: String,
    rx_grid: String,
    program_id: String,
    antenna: String,
    rig_info: String,
}

/// Public handle. Holds the sender side of the spot channel and the
/// stop flag.
pub struct PskReporter {
    sender: SyncSender<Spot>,
    stop_flag: Arc<AtomicBool>,
}

impl PskReporter {
    /// Spawn the worker thread, bind a UDP socket, resolve the
    /// PSK Reporter hostname. Returns immediately — DNS resolution
    /// is done in the worker, so a slow DNS lookup doesn't block
    /// the GUI thread.
    pub fn spawn(
        rx_call: String,
        rx_grid: String,
        antenna: String,
        program_id: String,
        rig_info: String,
    ) -> std::io::Result<Self> {
        let info = ReceiverInfo {
            rx_call,
            rx_grid,
            program_id,
            antenna,
            rig_info,
        };
        // Bounded channel so a runaway producer can't OOM us. If the
        // GUI generates spots faster than the worker can send (unlikely
        // — flush cadence is 5s and a typical decode rate is far below
        // that) the GUI will block briefly on send. 256 spots = many
        // minutes of typical traffic.
        let (tx, rx) = sync_channel::<Spot>(256);
        let stop_flag = Arc::new(AtomicBool::new(false));
        let stop_for_thread = stop_flag.clone();

        thread::Builder::new()
            .name("pskreporter".into())
            .spawn(move || run_worker(info, rx, stop_for_thread))?;

        Ok(Self { sender: tx, stop_flag })
    }

    /// Queue a spot for the next batch flush. Non-blocking unless the
    /// internal channel is full (which would mean the worker isn't
    /// keeping up — should not happen in practice).
    pub fn add_spot(
        &self,
        heard_call: String,
        heard_grid: Option<String>,
        freq_hz: u64,
        mode: &str,
        snr_db: i32,
        utc_secs: u32,
    ) {
        // Info-level log so the operator can see each queued spot at
        // the default log verbosity. Also useful for confirming that
        // spot generation actually fires under various RX conditions
        // — without this it's invisible whether the path is reached.
        log::info!(
            "[PSKR] queue spot: {} grid={:?} freq={} Hz snr={} dB",
            heard_call, heard_grid.as_deref().unwrap_or(""),
            freq_hz, snr_db);

        let spot = Spot {
            heard_call,
            heard_grid,
            freq_hz,
            mode: mode.to_string(),
            snr_db: snr_db.clamp(-127, 127) as i8,
            utc_secs,
        };
        // Use try_send so a full queue (worker stalled) doesn't block
        // the GUI — drop the spot instead. PSK Reporter spots are
        // best-effort; missing one is harmless.
        if self.sender.try_send(spot).is_err() {
            log::warn!("[PSKR] queue full, dropping spot");
        }
    }

    /// Stop the worker; final flush attempt; release resources.
    pub fn stop(self) {
        self.stop_flag.store(true, Ordering::SeqCst);
        // Drop the sender to wake the worker out of its recv timeout.
        drop(self.sender);
        // Worker will exit when it next checks stop_flag or sees
        // channel closure.
    }
}

/// Worker thread. Owns the UDP socket, the spot queue, the flush
/// timer, and the sequence/observation_id state.
fn run_worker(info: ReceiverInfo, rx: Receiver<Spot>, stop: Arc<AtomicBool>) {
    log::info!("[PSKR] worker starting (rx_call={}, grid={})",
        info.rx_call, info.rx_grid);

    // Bind a local UDP socket. Random source port — PSK Reporter's
    // NAT-tracking logic correlates packets by source IP+port, so we
    // KEEP the same port for the session.
    let socket = match UdpSocket::bind("0.0.0.0:0") {
        Ok(s) => s,
        Err(e) => {
            log::error!("[PSKR] failed to bind UDP socket: {}", e);
            return;
        }
    };

    // Resolve the destination once. PSK Reporter's address rarely
    // changes; if we ever fail to resolve at startup, retry on the
    // first packet send.
    let dest_addr = match resolve_dest() {
        Ok(a) => a,
        Err(e) => {
            log::error!("[PSKR] DNS resolve failed: {} — worker exiting", e);
            return;
        }
    };
    log::info!("[PSKR] destination resolved: {}", dest_addr);

    // Random session identifiers.
    let observation_id: u32 = rand_u32();
    let mut sequence_num: u32 = 0;

    // Templates are sent in the first 3 packets (server caches them),
    // then again every 60 minutes to recover from server restarts.
    // Counter increments per flush.
    let mut packets_sent: u32 = 0;
    let mut last_template_send = Instant::now();

    let mut pending: Vec<Spot> = Vec::with_capacity(64);
    let mut flush_due: Option<Instant> = None;

    loop {
        if stop.load(Ordering::SeqCst) { break; }

        // Decide how long to wait for the next spot. If we have
        // a flush deadline, wait at most until then; otherwise wait
        // up to a second so we can re-check the stop flag.
        let wait = match flush_due {
            Some(deadline) => deadline
                .saturating_duration_since(Instant::now())
                .max(Duration::from_millis(50)),
            None => Duration::from_secs(1),
        };

        match rx.recv_timeout(wait) {
            Ok(spot) => {
                pending.push(spot);
                if flush_due.is_none() {
                    // First spot in a fresh batch — schedule a flush.
                    flush_due = Some(Instant::now() + FLUSH_INTERVAL);
                }
            }
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                // Fall through to flush check.
            }
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {
                // Channel closed — stop signalled. Try one final flush.
                break;
            }
        }

        // Flush if it's time, OR if the queue's gotten large enough
        // that adding another spot risks exceeding MAX_PACKET_BYTES.
        let should_flush = flush_due
            .map(|d| Instant::now() >= d)
            .unwrap_or(false)
            || pending.len() >= 32;

        if should_flush && !pending.is_empty() {
            // Decide whether to include template descriptors. Send
            // them in the first 3 packets, then once per hour.
            let include_templates =
                packets_sent < 3 ||
                last_template_send.elapsed() >= Duration::from_secs(3600);

            sequence_num = sequence_num.wrapping_add(1);
            let now_secs = unix_secs_now();
            let packet = build_packet(
                &info,
                &pending,
                sequence_num,
                observation_id,
                now_secs,
                include_templates,
            );

            match socket.send_to(&packet, dest_addr) {
                Ok(n) => {
                    // One line per flush, info-level so it appears at
                    // the default log verbosity. The cadence is bounded
                    // (max one packet per ~5 seconds) so this isn't
                    // noisy. Lets the operator see at a glance whether
                    // submissions are reaching the server.
                    log::info!(
                        "[PSKR] sent {} bytes to {} ({} spots, seq={}, templates={})",
                        n, dest_addr, pending.len(), sequence_num, include_templates);
                    packets_sent = packets_sent.saturating_add(1);
                    if include_templates {
                        last_template_send = Instant::now();
                    }
                }
                Err(e) => {
                    log::warn!("[PSKR] send failed: {} (will retry on next batch)", e);
                }
            }

            pending.clear();
            flush_due = None;
        }
    }

    // Final flush on shutdown — best effort, ignore errors.
    if !pending.is_empty() {
        sequence_num = sequence_num.wrapping_add(1);
        let packet = build_packet(
            &info,
            &pending,
            sequence_num,
            observation_id,
            unix_secs_now(),
            false,
        );
        let _ = socket.send_to(&packet, dest_addr);
        log::info!("[PSKR] final flush: {} spots", pending.len());
    }
    log::info!("[PSKR] worker exiting");
}

fn resolve_dest() -> std::io::Result<SocketAddr> {
    let addrs: Vec<_> = (PSK_REPORTER_HOST, PSK_REPORTER_PORT)
        .to_socket_addrs()?
        .collect();
    addrs.into_iter().next().ok_or_else(|| std::io::Error::new(
        std::io::ErrorKind::AddrNotAvailable,
        format!("no addresses for {}", PSK_REPORTER_HOST),
    ))
}

fn unix_secs_now() -> u32 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs() as u32)
        .unwrap_or(0)
}

fn rand_u32() -> u32 {
    // Cheap session-unique id derived from system time + thread id.
    // Doesn't need cryptographic strength — just needs to be unlikely
    // to collide with another concurrently-running instance on the
    // same network.
    let n = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0);
    let tid = format!("{:?}", thread::current().id());
    let h: u64 = tid.bytes().fold(0u64, |a, b| a.wrapping_mul(31).wrapping_add(b as u64));
    ((n ^ h) as u32) | 0x8000_0000  // ensure non-zero high bit
}

/// Build one IPFIX packet with the given receiver info, spots, and
/// header fields. If `include_templates` is true, the packet also
/// carries the template descriptors PSK Reporter needs to interpret
/// the data records.
fn build_packet(
    info: &ReceiverInfo,
    spots: &[Spot],
    sequence_num: u32,
    observation_id: u32,
    export_time: u32,
    include_templates: bool,
) -> Vec<u8> {
    let mut out = Vec::with_capacity(MAX_PACKET_BYTES);

    // Header (length filled in at end)
    out.extend_from_slice(&IPFIX_VERSION.to_be_bytes());
    out.extend_from_slice(&[0u8; 2]);                       // length placeholder
    out.extend_from_slice(&export_time.to_be_bytes());
    out.extend_from_slice(&sequence_num.to_be_bytes());
    out.extend_from_slice(&observation_id.to_be_bytes());

    if include_templates {
        write_receiver_template(&mut out);
        write_sender_template(&mut out);
    }

    write_receiver_record(&mut out, info);
    write_sender_records(&mut out, spots);

    // Backfill total length
    let total_len = out.len() as u16;
    out[2..4].copy_from_slice(&total_len.to_be_bytes());

    out
}

/// Number of zero-padding bytes needed to round `len` up to a 4-byte
/// boundary.
fn pad_bytes(len: usize) -> usize {
    (4 - (len % 4)) % 4
}

/// Write a varlen UTF-8 string with a 1-byte length prefix.
/// PSK Reporter expects this format for callsigns, grids, and
/// free-text fields. Strings longer than 254 bytes use a 3-byte
/// length-prefix form (0xff + u16 length); we never send that long
/// so we always emit the 1-byte form. Empty strings are valid (just
/// emit `[0x00]`).
fn write_varlen(buf: &mut Vec<u8>, s: &str) {
    let bytes = s.as_bytes();
    // Server interprets >254 as 3-byte form which would confuse
    // parsing if we ever exceeded it. Truncate defensively.
    let n = bytes.len().min(254);
    buf.push(n as u8);
    buf.extend_from_slice(&bytes[..n]);
}

fn write_receiver_template(buf: &mut Vec<u8>) {
    // Set ID 3 = IPFIX *Options Template* (RFC 5101 §3.4.2). This
    // is structurally different from the regular Template (Set ID 2)
    // used for the sender records — Options Templates carry a
    // `scope_field_count` field between `field_count` and the
    // field-IE tuples. MSHV's wire output is the reference here:
    //
    //   u16  set_id           = 3
    //   u16  set_length       (filled in last)
    //   u16  template_id      = 0x50e2
    //   u16  field_count      = 5
    //   u16  scope_field_count = 0   <— THIS WAS MISSING
    //   for each field:
    //     u16  ie_id (high bit set = enterprise)
    //     u16  field_length (0xFFFF = varlen)
    //     u32  enterprise_number
    //
    // Without scope_field_count, PSK Reporter's parser sees a
    // malformed Options Template, rejects the entire packet's
    // receiver registration, and silently drops every sender record
    // that references it. Net effect: spots reach the server but
    // never appear on the map.
    let start = buf.len();
    buf.extend_from_slice(&3u16.to_be_bytes());          // Set ID = 3 (Options Template)
    buf.extend_from_slice(&[0u8; 2]);                     // length placeholder

    buf.extend_from_slice(&TEMPLATE_ID_RX.to_be_bytes());
    buf.extend_from_slice(&5u16.to_be_bytes());           // 5 fields
    buf.extend_from_slice(&0u16.to_be_bytes());           // scope_field_count = 0

    let fields: [(u16, u16); 5] = [
        (IE_RECEIVER_CALLSIGN,    0xFFFF),
        (IE_RECEIVER_LOCATOR,     0xFFFF),
        (IE_DECODING_SOFTWARE,    0xFFFF),
        (IE_ANTENNA_INFORMATION,  0xFFFF),
        (IE_RIG_INFORMATION,      0xFFFF),
    ];
    for (ie, len) in fields {
        // Set the enterprise bit (high bit) on the IE ID.
        let ie_with_bit = ie | 0x8000;
        buf.extend_from_slice(&ie_with_bit.to_be_bytes());
        buf.extend_from_slice(&len.to_be_bytes());
        buf.extend_from_slice(&HAMRADIO_ENTERPRISE.to_be_bytes());
    }

    // Pad and backfill length
    let pad = pad_bytes(buf.len() - start);
    buf.extend(std::iter::repeat(0u8).take(pad));
    let set_len = (buf.len() - start) as u16;
    buf[start + 2 .. start + 4].copy_from_slice(&set_len.to_be_bytes());
}

fn write_sender_template(buf: &mut Vec<u8>) {
    let start = buf.len();
    buf.extend_from_slice(&2u16.to_be_bytes());          // Set ID = 2 (Template)
    buf.extend_from_slice(&[0u8; 2]);                     // length placeholder

    buf.extend_from_slice(&TEMPLATE_ID_TX.to_be_bytes());
    buf.extend_from_slice(&7u16.to_be_bytes());           // 7 fields

    // Sender record fields. freq_hz is fixed-length 5 bytes (NOT
    // varlen) because PSK Reporter wants efficient binary packing
    // and the IE definition uses 5-byte unsigned BE integer.
    // SNR is 1-byte signed.
    let fields_enterprise: &[(u16, u16)] = &[
        (IE_SENDER_CALLSIGN,    0xFFFF),
        (IE_FREQUENCY,          5),
        (IE_SNR,                1),
        (IE_MODE,               0xFFFF),
        (IE_SENDER_LOCATOR,     0xFFFF),
        (IE_INFORMATION_SOURCE, 1),
    ];
    for &(ie, len) in fields_enterprise {
        let ie_with_bit = ie | 0x8000;
        buf.extend_from_slice(&ie_with_bit.to_be_bytes());
        buf.extend_from_slice(&len.to_be_bytes());
        buf.extend_from_slice(&HAMRADIO_ENTERPRISE.to_be_bytes());
    }
    // Standard IE (no enterprise bit) for the timestamp.
    buf.extend_from_slice(&IE_DATETIME_SECONDS.to_be_bytes());
    buf.extend_from_slice(&4u16.to_be_bytes());           // u32 unix seconds

    let pad = pad_bytes(buf.len() - start);
    buf.extend(std::iter::repeat(0u8).take(pad));
    let set_len = (buf.len() - start) as u16;
    buf[start + 2 .. start + 4].copy_from_slice(&set_len.to_be_bytes());
}

fn write_receiver_record(buf: &mut Vec<u8>, info: &ReceiverInfo) {
    let start = buf.len();
    buf.extend_from_slice(&TEMPLATE_ID_RX.to_be_bytes());
    buf.extend_from_slice(&[0u8; 2]);                     // length placeholder

    write_varlen(buf, &info.rx_call);
    write_varlen(buf, &info.rx_grid);
    write_varlen(buf, &info.program_id);
    write_varlen(buf, &info.antenna);
    write_varlen(buf, &info.rig_info);

    let pad = pad_bytes(buf.len() - start);
    buf.extend(std::iter::repeat(0u8).take(pad));
    let set_len = (buf.len() - start) as u16;
    buf[start + 2 .. start + 4].copy_from_slice(&set_len.to_be_bytes());
}

fn write_sender_records(buf: &mut Vec<u8>, spots: &[Spot]) {
    if spots.is_empty() { return; }
    let start = buf.len();
    buf.extend_from_slice(&TEMPLATE_ID_TX.to_be_bytes());
    buf.extend_from_slice(&[0u8; 2]);                     // length placeholder

    for s in spots {
        write_varlen(buf, &s.heard_call);
        // 5-byte big-endian frequency. PSK Reporter expects this exact
        // width — it supports frequencies up to ~1 THz (vastly more
        // than amateur use needs, but lets bands like QO-100 at 2.4
        // GHz fit cleanly).
        let f = s.freq_hz;
        buf.push(((f >> 32) & 0xFF) as u8);
        buf.push(((f >> 24) & 0xFF) as u8);
        buf.push(((f >> 16) & 0xFF) as u8);
        buf.push(((f >>  8) & 0xFF) as u8);
        buf.push((f         & 0xFF) as u8);
        // Signed i8 SNR
        buf.push(s.snr_db as u8);
        write_varlen(buf, &s.mode);
        write_varlen(buf, s.heard_grid.as_deref().unwrap_or(""));
        buf.push(REPORTER_SOURCE_AUTOMATIC);
        // u32 timestamp (NOT varlen — fixed 4 bytes per template)
        buf.extend_from_slice(&s.utc_secs.to_be_bytes());
    }

    let pad = pad_bytes(buf.len() - start);
    buf.extend(std::iter::repeat(0u8).take(pad));
    let set_len = (buf.len() - start) as u16;
    buf[start + 2 .. start + 4].copy_from_slice(&set_len.to_be_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dummy_info() -> ReceiverInfo {
        ReceiverInfo {
            rx_call: "GW4WND".into(),
            rx_grid: "IO82KM".into(),
            program_id: "MSK144Plus 0.2.0".into(),
            antenna: "4-element 2 m yagi".into(),
            rig_info: "Icom IC-9700".into(),
        }
    }

    fn dummy_spot() -> Spot {
        Spot {
            heard_call: "F1ABC".into(),
            heard_grid: Some("IN98".into()),
            freq_hz: 144_360_000,
            mode: "MSK144".into(),
            snr_db: 5,
            utc_secs: 1_700_000_000,
        }
    }

    #[test]
    fn header_basics() {
        // Build a minimal packet and verify the header fields.
        let info = dummy_info();
        let spots = vec![dummy_spot()];
        let pkt = build_packet(&info, &spots, 1, 0xDEADBEEF, 1_700_000_000, false);

        // Must be at least the 16-byte header
        assert!(pkt.len() >= 16);
        // Version
        assert_eq!(u16::from_be_bytes([pkt[0], pkt[1]]), 0x000A);
        // Length matches actual buffer
        assert_eq!(u16::from_be_bytes([pkt[2], pkt[3]]) as usize, pkt.len());
        // Export time
        assert_eq!(u32::from_be_bytes([pkt[4], pkt[5], pkt[6], pkt[7]]), 1_700_000_000);
        // Sequence
        assert_eq!(u32::from_be_bytes([pkt[8], pkt[9], pkt[10], pkt[11]]), 1);
        // Observation ID
        assert_eq!(u32::from_be_bytes([pkt[12], pkt[13], pkt[14], pkt[15]]), 0xDEADBEEF);
    }

    #[test]
    fn receiver_call_appears_verbatim() {
        let info = dummy_info();
        let pkt = build_packet(&info, &[dummy_spot()], 1, 1, 0, true);
        // Find the "GW4WND" bytes anywhere in the packet body.
        // Crude but effective — varlen prefix is 6 (length of GW4WND)
        // immediately followed by the ASCII bytes.
        let needle = b"GW4WND";
        assert!(pkt.windows(needle.len()).any(|w| w == needle),
            "expected GW4WND in packet body");
    }

    #[test]
    fn frequency_encoded_5_byte_be() {
        // 144_360_000 Hz = 0x089A99C0 (32-bit) → 5-byte BE: 00 08 9A 99 C0
        let info = dummy_info();
        let spot = Spot {
            heard_call: "TEST".into(),
            heard_grid: None,
            freq_hz: 144_360_000,
            mode: "MSK144".into(),
            snr_db: 0,
            utc_secs: 0,
        };
        let pkt = build_packet(&info, &[spot], 1, 1, 0, false);

        // Find the byte sequence 00 08 9A 99 C0 anywhere in the packet
        let needle = [0x00, 0x08, 0x9A, 0x99, 0xC0];
        assert!(pkt.windows(needle.len()).any(|w| w == needle),
            "expected 5-byte BE freq encoding in packet");
    }

    #[test]
    fn snr_signed_i8() {
        // -3 dB = 0xFD as u8 (two's complement)
        let info = dummy_info();
        let spot = Spot {
            heard_call: "TEST".into(),
            heard_grid: None,
            freq_hz: 144_360_000,
            mode: "MSK144".into(),
            snr_db: -3,
            utc_secs: 0,
        };
        let pkt = build_packet(&info, &[spot], 1, 1, 0, false);

        // Look for the 5-byte freq followed by the SNR byte 0xFD
        let freq_bytes = [0x00, 0x08, 0x9A, 0x99, 0xC0];
        let pos = pkt.windows(freq_bytes.len())
            .position(|w| w == freq_bytes)
            .expect("freq not found");
        assert_eq!(pkt[pos + freq_bytes.len()], 0xFD,
            "expected SNR -3 = 0xFD after freq");
    }

    #[test]
    fn templates_only_when_requested() {
        let info = dummy_info();
        let spots = vec![dummy_spot()];

        let with_t = build_packet(&info, &spots, 1, 1, 0, true);
        let without_t = build_packet(&info, &spots, 1, 1, 0, false);

        // Template packet must be larger (carries 2 extra Set blocks).
        assert!(with_t.len() > without_t.len(),
            "template packet should be larger ({} vs {})",
            with_t.len(), without_t.len());

        // Template packet contains the receiver-template Set ID 3 + 0x50e2
        // immediately after the 16-byte header.
        let set_id = u16::from_be_bytes([with_t[16], with_t[17]]);
        assert_eq!(set_id, 3, "expected Template set (id=3) first when requested");
    }

    #[test]
    fn pad_bytes_correct() {
        assert_eq!(pad_bytes(0), 0);
        assert_eq!(pad_bytes(1), 3);
        assert_eq!(pad_bytes(2), 2);
        assert_eq!(pad_bytes(3), 1);
        assert_eq!(pad_bytes(4), 0);
        assert_eq!(pad_bytes(5), 3);
        assert_eq!(pad_bytes(7), 1);
        assert_eq!(pad_bytes(8), 0);
    }

    #[test]
    fn varlen_encoding() {
        let mut buf = Vec::new();
        write_varlen(&mut buf, "ABC");
        // [0x03, 'A', 'B', 'C']
        assert_eq!(buf, vec![3, b'A', b'B', b'C']);

        let mut buf2 = Vec::new();
        write_varlen(&mut buf2, "");
        // Empty string = single zero length byte
        assert_eq!(buf2, vec![0]);
    }

    #[test]
    fn empty_grid_renders_as_empty_varlen() {
        let info = dummy_info();
        let spot = Spot {
            heard_call: "TEST5".into(),
            heard_grid: None,
            freq_hz: 144_360_000,
            mode: "MSK144".into(),
            snr_db: 0,
            utc_secs: 0,
        };
        let pkt = build_packet(&info, &[spot], 1, 1, 0, false);
        // Locate "TEST5" in packet
        let needle = b"TEST5";
        let pos = pkt.windows(needle.len()).position(|w| w == needle).unwrap();
        // After the call, we have 5 bytes freq + 1 byte SNR + varlen mode.
        // Mode is "MSK144" (6 bytes) prefixed by length 6.
        // Then varlen grid — should be 0x00 (empty).
        let mode_pos = pos + needle.len() + 5 + 1;
        assert_eq!(pkt[mode_pos], 6, "expected mode length 6");
        // Skip mode body
        let grid_pos = mode_pos + 1 + 6;
        assert_eq!(pkt[grid_pos], 0, "expected empty grid (length 0) when None");
    }

    #[test]
    fn spawn_and_stop_clean() {
        // Smoke test: spawn and stop should not panic or hang.
        // Worker may fail to resolve / send (we're not online), but
        // the lifecycle should complete cleanly.
        let r = PskReporter::spawn(
            "GW4WND".into(),
            "IO82KM".into(),
            "test antenna".into(),
            "msk144plus_test".into(),
            String::new(),
        );
        if let Ok(reporter) = r {
            // Push a couple of spots — they may or may not actually
            // get sent depending on network availability, but the
            // queue interaction shouldn't hang.
            reporter.add_spot("F1TEST".into(), None, 144_360_000, "MSK144", 5, 1_700_000_000);
            reporter.add_spot("DK7RC".into(), Some("JN69".into()),
                144_360_500, "MSK144", -3, 1_700_000_001);
            reporter.stop();
        }
    }

    #[test]
    fn receiver_template_has_scope_field_count() {
        // Set ID 3 = IPFIX Options Template. RFC 5101 requires a
        // scope_field_count field after field_count. Without it,
        // PSK Reporter rejects the entire packet's receiver
        // registration and silently drops the spots — they reach
        // the server but never appear on the map.
        //
        // Wire layout for receiver template (Set ID 3):
        //   offset  size  meaning
        //   0       u16   set_id = 3
        //   2       u16   set_length
        //   4       u16   template_id = 0x50e2
        //   6       u16   field_count = 5
        //   8       u16   scope_field_count = 0   <— THIS
        //   10..    fields
        let info = dummy_info();
        let pkt = build_packet(&info, &[dummy_spot()], 1, 1, 0, true);

        // Header is 16 bytes; receiver template starts immediately
        // after at offset 16.
        let rx_tmpl_offset = 16;
        let set_id = u16::from_be_bytes([
            pkt[rx_tmpl_offset], pkt[rx_tmpl_offset + 1]]);
        assert_eq!(set_id, 3, "expected receiver Options Template at offset 16");

        // Fields start at +4 (template_id), +6 (field_count), +8 (scope_field_count)
        let template_id = u16::from_be_bytes([
            pkt[rx_tmpl_offset + 4], pkt[rx_tmpl_offset + 5]]);
        let field_count = u16::from_be_bytes([
            pkt[rx_tmpl_offset + 6], pkt[rx_tmpl_offset + 7]]);
        let scope_field_count = u16::from_be_bytes([
            pkt[rx_tmpl_offset + 8], pkt[rx_tmpl_offset + 9]]);

        assert_eq!(template_id, 0x50e2, "template id");
        assert_eq!(field_count, 5, "5 receiver fields");
        assert_eq!(scope_field_count, 0,
            "scope_field_count must be 0 for our receiver Options Template");
    }
}
