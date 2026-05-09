// crates/msk144plus_gui/src/transmitter.rs
//
// Slot-aligned MSK144 transmitter.
//
// Mirrors MSK2K's runtime.rs scheduling: at each slot boundary, decides
// whether to TX based on (qso_state ∈ TX-states) AND (current_slot_parity ==
// configured_tx_parity). When TX fires:
//   1. Asserts PTT via Hamlib
//   2. Optional ptt_delay_ms
//   3. Plays the audio buffer (rendered from current message text via
//      msk144plus_engine::encode_message_to_audio)
//   4. Releases PTT
//
// We also need to coordinate with RX: on macOS the IC-9700 USB CODEC has
// separate input + output endpoints that can be open simultaneously, so we
// don't have to stop RX for TX. Linux is harder (single ALSA handle); MSK2K
// flushes the RX task before TX. For now we assume macOS-style concurrent
// I/O and Linux can be addressed when we build for it.

use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use std::thread;
use msk144plus_engine::encode_message_to_audio;

/// What the transmitter should do this slot.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TxMode {
    /// Don't transmit; do nothing each slot.
    Idle,
    /// Transmit `message` every TX slot until mode changes.
    Active,
}

/// Shared state between UI and the transmitter thread. UI updates fields
/// freely; the thread reads them on every slot tick.
#[derive(Clone)]
pub struct TxState {
    pub mode: TxMode,
    /// Full text to TX, e.g. "CQ GW4WND IO82" or "GW4WND I3FGX +10".
    pub message: String,
    /// Audio centre frequency in Hz (matches RX fc usually).
    pub fc_hz: f32,
    /// Slot period in seconds (15 or 30).
    pub slot_period_secs: u32,
    /// Our TX parity: "Odd" (slot index odd) or "Even".
    pub tx_parity: String,
    /// PTT key-up delay in ms.
    pub ptt_delay_ms: u32,
    /// Audio output device label (None = system default).
    pub output_device: Option<String>,
    /// TX audio level 0..1.
    pub tx_level: f32,
}

impl Default for TxState {
    fn default() -> Self {
        Self {
            mode: TxMode::Idle,
            message: String::new(),
            fc_hz: 1500.0,
            slot_period_secs: 15,
            tx_parity: "Odd".to_string(),
            ptt_delay_ms: 0,
            output_device: None,
            tx_level: 0.4,
        }
    }
}

/// Updates emitted by the transmitter thread for the UI.
#[derive(Debug, Clone)]
pub enum TxEvent {
    /// PTT just keyed up; TX is starting.
    Started { slot_index: i64, message: String },
    /// TX completed normally for this slot.
    Finished { slot_index: i64 },
    /// Encoding failed; PTT was never asserted.
    EncodeFailed { reason: String },
}

/// Handle to the transmitter thread.
pub struct Transmitter {
    state: Arc<Mutex<TxState>>,
    stop_flag: Arc<AtomicBool>,
}

impl Transmitter {
    /// Spawn the transmitter worker. Audio is sent to `output_device` (None =
    /// default). PTT is keyed via the supplied `hamlib` (None = no PTT keying,
    /// useful for testing).
    pub fn spawn(
        initial: TxState,
        hamlib: Option<Arc<dx_runtime::HamlibClient>>,
        events: std::sync::mpsc::Sender<TxEvent>,
    ) -> Self {
        let state = Arc::new(Mutex::new(initial));
        let stop_flag = Arc::new(AtomicBool::new(false));

        let state_for_thread = state.clone();
        let stop_for_thread = stop_flag.clone();

        thread::Builder::new()
            .name("tx-scheduler".into())
            .spawn(move || run_scheduler(state_for_thread, stop_for_thread, hamlib, events))
            .expect("spawn tx scheduler");

        Self { state, stop_flag }
    }

    /// Update the transmitter's mode and message.
    pub fn set_mode(&self, mode: TxMode) {
        let mut s = self.state.lock().unwrap();
        s.mode = mode;
    }

    /// Update what the transmitter sends.
    pub fn set_message(&self, text: String) {
        let mut s = self.state.lock().unwrap();
        s.message = text;
    }

    /// Update slot configuration (period + parity).
    pub fn set_slot_config(&self, period_secs: u32, parity: String) {
        let mut s = self.state.lock().unwrap();
        s.slot_period_secs = period_secs;
        s.tx_parity = parity;
    }

    /// Update TX-side output device + level.
    pub fn set_output_config(&self, device: Option<String>, level: f32, ptt_delay_ms: u32) {
        let mut s = self.state.lock().unwrap();
        s.output_device = device;
        s.tx_level = level.clamp(0.0, 1.0);
        s.ptt_delay_ms = ptt_delay_ms;
    }

    /// Update the audio centre frequency.
    pub fn set_fc(&self, fc_hz: f32) {
        let mut s = self.state.lock().unwrap();
        s.fc_hz = fc_hz;
    }
}

impl Drop for Transmitter {
    fn drop(&mut self) {
        self.stop_flag.store(true, Ordering::Release);
    }
}

fn run_scheduler(
    state: Arc<Mutex<TxState>>,
    stop_flag: Arc<AtomicBool>,
    hamlib: Option<Arc<dx_runtime::HamlibClient>>,
    events: std::sync::mpsc::Sender<TxEvent>,
) {
    log::info!("[TX] Scheduler started");
    let mut last_handled_slot: i64 = -1;
    let desired_rate = 12000u32;

    while !stop_flag.load(Ordering::Acquire) {
        // Sleep until next slot boundary
        let snap = state.lock().unwrap().clone();
        let slot_len_ms = snap.slot_period_secs as i64 * 1000;
        let now_ms = utc_now_ms();
        let slot_idx = now_ms / slot_len_ms;
        let next_boundary_ms = (slot_idx + 1) * slot_len_ms;
        let sleep_ms = (next_boundary_ms - now_ms).max(0).min(slot_len_ms);
        // Cap our sleep to 200ms so a settings change is picked up promptly.
        thread::sleep(Duration::from_millis((sleep_ms as u64).min(200)));

        let now_ms = utc_now_ms();
        let snap = state.lock().unwrap().clone();
        let slot_len_ms = snap.slot_period_secs as i64 * 1000;
        let cur_slot_idx = now_ms / slot_len_ms;

        // Are we within ±200ms of a slot boundary, AND haven't yet handled this slot?
        let into_slot = now_ms - cur_slot_idx * slot_len_ms;
        if into_slot > 200 {
            continue;  // missed the boundary; wait for next
        }
        if cur_slot_idx == last_handled_slot {
            continue;
        }
        last_handled_slot = cur_slot_idx;

        // Should we TX this slot?
        let our_parity: i64 = if snap.tx_parity == "Even" { 0 } else { 1 };
        let slot_parity = cur_slot_idx % 2;
        let is_our_slot = slot_parity == our_parity;
        let want_tx = matches!(snap.mode, TxMode::Active);

        if !want_tx || !is_our_slot {
            log::debug!("[TX] slot {} (parity {}) — not TXing (want_tx={}, our={})",
                cur_slot_idx, slot_parity, want_tx, our_parity);
            continue;
        }

        if snap.message.trim().is_empty() {
            log::warn!("[TX] slot {} — TX active but message empty, skipping", cur_slot_idx);
            continue;
        }

        log::info!("[TX] slot {} starting: {:?}", cur_slot_idx, snap.message);

        // 1. Encode message to audio
        let audio = match encode_message_to_audio(
            &snap.message, snap.fc_hz, snap.slot_period_secs)
        {
            Ok(a) => a,
            Err(e) => {
                log::error!("[TX] encode failed: {}", e);
                let _ = events.send(TxEvent::EncodeFailed { reason: e.to_string() });
                continue;
            }
        };

        // 2. Assert PTT
        if let Some(h) = hamlib.as_ref() {
            h.set_ptt(true);
        }

        // 3. PTT delay (let rig key up)
        if snap.ptt_delay_ms > 0 {
            thread::sleep(Duration::from_millis(snap.ptt_delay_ms as u64));
        }

        let _ = events.send(TxEvent::Started {
            slot_index: cur_slot_idx,
            message: snap.message.clone(),
        });

        // 4. Play audio (blocks for the full slot duration)
        if let Err(e) = dx_runtime::play_buffer(
            snap.output_device.as_deref(),
            audio,
            desired_rate,
            snap.tx_level,
            300, // tx_truncate_ms — stop 300 ms before slot end
                 // so the half-duplex CODEC tail/PA collapse doesn't
                 // bleed into the start of the next RX slot
        ) {
            log::error!("[TX] play failed: {}", e);
        }

        // 5. Release PTT
        if let Some(h) = hamlib.as_ref() {
            h.set_ptt(false);
        }

        let _ = events.send(TxEvent::Finished { slot_index: cur_slot_idx });
        log::info!("[TX] slot {} complete", cur_slot_idx);
    }
    log::info!("[TX] Scheduler stopped");
}

fn utc_now_ms() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as i64)
        .unwrap_or(0)
}
