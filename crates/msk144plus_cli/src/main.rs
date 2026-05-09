// crates/msk144plus_cli/src/main.rs
//
// MSK144+ command-line decoder.

use clap::Parser;
use msk144plus_engine::{decode_slot, Depth, ShortMessageConfig};

#[derive(Parser, Debug)]
#[command(name = "msk144plus_decode")]
#[command(about = "MSK144 decoder (faithful WSJT-X port)")]
struct Args {
    #[arg(value_name = "FILE")]
    wav_path: String,
    #[arg(long, default_value_t = 1500.0)]
    fc: f32,
    #[arg(long, default_value_t = 100.0)]
    ntol: f32,
    /// Decoder depth: fast=1, normal=2, deep=3.
    #[arg(long, default_value_t = 3)]
    depth: u8,
    #[arg(long)]
    quiet: bool,
    /// Operator's own callsign (enables MSK40 short-message decoding when both --mycall and --hiscall are set).
    #[arg(long)]
    mycall: Option<String>,
    /// Other station's callsign for MSK40 short-message decoding.
    #[arg(long)]
    hiscall: Option<String>,
}

fn load_wav(path: &str) -> Vec<f32> {
    let mut reader = hound::WavReader::open(path)
        .unwrap_or_else(|e| panic!("Failed to open {}: {}", path, e));
    let spec = reader.spec();
    if spec.sample_rate != 12000 {
        eprintln!("warning: WAV sample rate is {} Hz, expected 12000", spec.sample_rate);
    }
    if spec.channels != 1 {
        eprintln!("warning: WAV has {} channels, expected mono", spec.channels);
    }
    reader
        .samples::<i16>()
        .map(|s| s.unwrap() as f32)
        .collect()
}

fn main() {
    let args = Args::parse();
    let level = if args.quiet { log::LevelFilter::Warn } else { log::LevelFilter::Info };
    env_logger::Builder::new().filter_level(level).init();

    let depth = match args.depth {
        1 => Depth::Fast,
        2 => Depth::Normal,
        _ => Depth::Deep,
    };
    let short_cfg = match (args.mycall.clone(), args.hiscall.clone()) {
        (Some(mc), Some(hc)) => Some(ShortMessageConfig {
            mycall: mc,
            hiscall: hc,
            enabled: true,
        }),
        _ => None,
    };
    let audio = load_wav(&args.wav_path);
    let events = decode_slot(&audio, args.ntol, args.fc, depth, short_cfg.as_ref());

    for evt in &events {
        println!(
            "000000 {:3.0}  0.0  {:4.0} ?  {} ({})",
            -10.0,
            args.fc + evt.freq_offset,
            evt.text,
            evt.method
        );
    }
}
