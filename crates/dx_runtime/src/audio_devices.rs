// crates/dx_runtime/src/audio_devices.rs
//
// VERBATIM PORT from FSK441+:
//   src/fsk441rx/app.rs::enumerate_audio_devices  → enumerate_displays
//   src/fsk441rx/tx.rs::parse_device_suffix       → parse_device_suffix
//   src/fsk441rx/tx.rs::find_output_device        → find_output_device
//   src/fsk441rx/tx.rs::find_input_device         → find_input_device
//
// Working code from a shipping app. Don't reinvent. Don't second-guess.

use cpal::traits::{DeviceTrait, HostTrait};
use std::collections::HashMap;

/// Enumerate audio devices and produce display names with disambiguating
/// "(RX)" / "(TX)" / "(RX/TX)" suffixes for duplicates.
///
/// Returns the SAME list for both Input and Output dropdowns — the user
/// picks the (RX) instance for input and the (TX) instance for output.
/// On macOS, IC-9700's USB Audio CODEC appears twice; the suffixes
/// disambiguate which is which.
pub fn enumerate_displays() -> Vec<String> {
    let host = cpal::default_host();

    // host.devices() — sees all devices including duplicate USB CODECs on macOS
    let device_list: Vec<cpal::Device> = {
        let from_all = host.devices().map(|d| d.collect::<Vec<_>>()).unwrap_or_default();
        if !from_all.is_empty() { from_all } else {
            let mut devs: Vec<cpal::Device> = Vec::new();
            if let Ok(d) = host.input_devices()  { devs.extend(d); }
            if let Ok(d) = host.output_devices() { devs.extend(d); }
            devs
        }
    };

    let mut all_devices: Vec<(String, bool, bool)> = Vec::new();
    for d in device_list {
        if let Ok(name) = d.name() {
            let has_in  = d.supported_input_configs().map(|mut c| c.next().is_some()).unwrap_or(false)
                       || d.default_input_config().is_ok();
            let has_out = d.supported_output_configs().map(|mut c| c.next().is_some()).unwrap_or(false)
                       || d.default_output_config().is_ok();
            all_devices.push((name, has_in, has_out));
        }
    }
    all_devices.sort_by(|a, b| a.0.cmp(&b.0));

    // Count duplicates — IC-9700 shows as two "USB Audio CODEC" entries
    let mut name_counts: HashMap<String, usize> = HashMap::new();
    for (name, _, _) in &all_devices {
        *name_counts.entry(name.clone()).or_insert(0) += 1;
    }

    let mut group_caps: HashMap<String, Vec<(bool, bool)>> = HashMap::new();
    for (name, has_in, has_out) in &all_devices {
        if name_counts[name.as_str()] > 1 {
            group_caps.entry(name.clone()).or_default().push((*has_in, *has_out));
        }
    }

    let mut name_indices: HashMap<String, usize> = HashMap::new();
    let display_names: Vec<String> = all_devices.iter().map(|(name, has_in, has_out)| {
        if name_counts[name.as_str()] > 1 {
            let idx = name_indices.entry(name.clone()).or_insert(0);
            *idx += 1;
            let caps = &group_caps[name.as_str()];
            let has_rx_sib = caps.iter().any(|(i, _)| *i);
            let has_tx_sib = caps.iter().any(|(_, o)| *o);
            let label = match (*has_in, *has_out) {
                (true,  false) => "RX".to_string(),
                (false, true)  => "TX".to_string(),
                (true,  true)  => "RX/TX".to_string(),
                (false, false) => {
                    if has_rx_sib && !has_tx_sib { "TX".to_string() }
                    else if has_tx_sib && !has_rx_sib { "RX".to_string() }
                    else { format!("{}", idx) }
                }
            };
            format!("{} ({})", name, label)
        } else {
            name.clone()
        }
    }).collect();

    log::info!("[AUDIO] Devices: {:?}", display_names);
    display_names
}

/// Same list for both Input and Output dropdowns — user picks (RX) / (TX).
pub fn list_input_displays()  -> Vec<String> { enumerate_displays() }
pub fn list_output_displays() -> Vec<String> { enumerate_displays() }

/// Strip " (RX)" / " (TX)" / " (RX/TX)" / " (N)" suffix from a display
/// name to recover the base device name. Returns (base, suffix).
pub fn parse_device_suffix(display_name: &str) -> (String, String) {
    if let Some(pos) = display_name.rfind(" (") {
        if display_name.ends_with(')') {
            let s = display_name[pos+2..display_name.len()-1].trim();
            if s == "RX" || s == "TX" || s == "RX/TX" || s.chars().all(|c| c.is_ascii_digit()) {
                return (display_name[..pos].to_string(), s.to_string());
            }
        }
    }
    (display_name.to_string(), String::new())
}

/// Resolve input device for capture — substring match via cpal.
/// VERBATIM from FSK441+ tx.rs::find_input_device.
pub fn find_input_device(display_name: &str) -> Option<cpal::Device> {
    let (base, _) = parse_device_suffix(display_name);
    let host = cpal::default_host();
    if let Ok(devs) = host.input_devices() {
        for d in devs {
            if let Ok(name) = d.name() {
                if name.contains(&base) || name.contains(display_name) {
                    log::info!("[AUDIO] find_input_device: matched {:?}", name);
                    return Some(d);
                }
            }
        }
    }
    log::warn!("[AUDIO] find_input_device: no match for {:?}, using default", display_name);
    host.default_input_device()
}

/// Find output device by name.
/// VERBATIM from FSK441+ tx.rs::find_output_device.
///
/// On macOS the IC-9700 USB Audio CODEC TX device reports has_out=false
/// via cpal capability checks, but build_output_stream() succeeds. We use
/// host.devices() WITHOUT capability filtering as the fallback so the
/// correct device is found. Then build_output_stream() probes directly.
pub fn find_output_device(display_name: &str) -> Option<cpal::Device> {
    let (base, _) = parse_device_suffix(display_name);
    let host = cpal::default_host();

    // 1. Standard path: output_devices() with substring match (MSK2K approach)
    if let Ok(devs) = host.output_devices() {
        for d in devs {
            if let Ok(n) = d.name() {
                if n.contains(base.as_str()) || base.contains(n.as_str()) {
                    log::info!("[AUDIO] find_output_device via output_devices(): {:?}", n);
                    return Some(d);
                }
            }
        }
    }

    // 2. Fallback: host.devices() WITHOUT has_out check.
    //    IC-9700 TX shows has_out=false from cpal but build_output_stream works.
    if let Ok(devs) = host.devices() {
        for d in devs {
            if let Ok(n) = d.name() {
                if n.contains(base.as_str()) || base.contains(n.as_str()) {
                    log::warn!("[AUDIO] find_output_device via host.devices() (no cap check): {:?}", n);
                    return Some(d);
                }
            }
        }
    }

    log::error!("[AUDIO] Cannot find output device {:?}", display_name);
    None
}
