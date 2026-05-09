// crates/msk144plus_engine/tests/real_wav_regression.rs
//
// Regression tests using real captured WAV files. These signals have
// known decode behaviour in WSJT-X; our v2 port should match.

use msk144plus_engine::{decode_slot, Depth};
use std::path::Path;

const TEST_SAMPLES: &str = "../../test_samples";

fn load_wav(name: &str) -> Option<Vec<f32>> {
    let path = Path::new(TEST_SAMPLES).join(name);
    if !path.exists() {
        eprintln!("Test WAV not found: {}", path.display());
        return None;
    }
    let mut reader = hound::WavReader::open(&path).ok()?;
    Some(
        reader
            .samples::<i16>()
            .map(|s| s.unwrap() as f32)
            .collect(),
    )
}

fn decoded_texts(name: &str, fc: f32) -> Option<Vec<String>> {
    let audio = load_wav(name)?;
    let events = decode_slot(&audio, 100.0, fc, Depth::Deep, None);
    Some(events.into_iter().map(|e| e.text).collect())
}

#[test]
fn k1jt_wa4cqg_em72_at_fc1488() {
    let texts = match decoded_texts("181211_120500.wav", 1488.0) {
        Some(t) => t,
        None => return,
    };
    assert!(
        texts.iter().any(|t| t == "K1JT WA4CQG EM72"),
        "expected 'K1JT WA4CQG EM72' in {:?}", texts
    );
}

#[test]
fn cq_kd9vv_en71_at_fc1500() {
    let texts = match decoded_texts("181211_120800.wav", 1500.0) {
        Some(t) => t,
        None => return,
    };
    assert!(
        texts.iter().any(|t| t == "CQ KD9VV EN71"),
        "expected 'CQ KD9VV EN71' in {:?}", texts
    );
}

#[test]
fn sp9hwy_lz2hv_kn23_at_fc1500() {
    let texts = match decoded_texts("SP9HWY_LZ2HV_MSK144_181002_061345.WAV", 1500.0) {
        Some(t) => t,
        None => return,
    };
    assert!(
        texts.iter().any(|t| t == "SP9HWY LZ2HV KN23"),
        "expected 'SP9HWY LZ2HV KN23' in {:?}", texts
    );
}

#[test]
fn cq_f5ct_jn08() {
    let texts = match decoded_texts("SP6CPH_MSK144_181216_082230_11_3_CQ_F5CT_JN08.WAV", 1500.0) {
        Some(t) => t,
        None => return,
    };
    assert!(
        texts.iter().any(|t| t == "CQ F5CT JN08"),
        "expected 'CQ F5CT JN08' in {:?}", texts
    );
}

#[test]
fn cq_g1ppa_io93() {
    let texts = match decoded_texts("SP6CPH_MSK144_181216_084915_3_2_CQ_G1PPA_IO93.WAV", 1500.0) {
        Some(t) => t,
        None => return,
    };
    assert!(
        texts.iter().any(|t| t == "CQ G1PPA IO93"),
        "expected 'CQ G1PPA IO93' in {:?}", texts
    );
}

#[test]
fn cq_on8dm_jo10() {
    let texts = match decoded_texts("SP6CPH_MSK144_181216_085445_2_3_CQ_ON8DM_JO10.WAV", 1500.0) {
        Some(t) => t,
        None => return,
    };
    assert!(
        texts.iter().any(|t| t == "CQ ON8DM JO10"),
        "expected 'CQ ON8DM JO10' in {:?}", texts
    );
}

#[test]
fn ly3w_pa4vhf_minus05() {
    let texts = match decoded_texts("SP6CPH_MSK144_181216_085545_1_4_LY3W_PA4VHF_-05.WAV", 1500.0) {
        Some(t) => t,
        None => return,
    };
    assert!(
        texts.iter().any(|t| t == "LY3W PA4VHF -05"),
        "expected 'LY3W PA4VHF -05' in {:?}", texts
    );
}

#[test]
fn dl7oap_hb9ruz_jn47() {
    let texts = match decoded_texts("_SP6CPH_MSK144_181216_104030_14_2_DL7OAP_HB9RUZ_JN47.WAV", 1500.0) {
        Some(t) => t,
        None => return,
    };
    assert!(
        texts.iter().any(|t| t == "DL7OAP HB9RUZ JN47"),
        "expected 'DL7OAP HB9RUZ JN47' in {:?}", texts
    );
}
