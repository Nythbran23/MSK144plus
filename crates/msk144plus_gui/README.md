# MSK144+ RX UI

A minimal egui-based receive-only UI for live MSK144 (and MSK40 short-message)
decoding from the soundcard. Built on the v2 faithful WSJT-X port.

## Build

The whole workspace builds with cargo. The GUI crate pins egui/eframe to
0.24.1 to keep it on stable Rust 1.75 (no `edition2024` requirement).

```
cd msk144plus_v2
cargo build --release -p msk144plus_gui
```

The binary lands at `target/release/msk144plus_rx`.

## macOS run notes (IC-9700)

The IC-9700's USB CODEC (Burr-Brown PCM2902) shows up in the macOS Audio MIDI
Setup as "USB Audio CODEC". It supports 12 kHz mono natively, which is what
the audio module attempts first. If the driver doesn't expose 12 kHz, it
falls back to a multiple of 12 kHz (24/36/48 kHz) and decimates with a
windowed-sinc anti-alias filter.

```
./target/release/msk144plus_rx
```

When the GUI launches:

1. **Device dropdown** — pick "USB Audio CODEC" (or whatever the IC-9700
   shows as). "(default)" uses the macOS default input.
2. **Start** — opens the audio stream. The level meter (top-right of the
   topbar) should show movement.
3. **fc / ntol / depth** — set the centre frequency (default 1500 Hz),
   tolerance (±100 Hz), and decode depth. Deep tries all 4 averaging
   patterns; Normal tries 2; Fast skips averaging entirely.
4. **MSK40 (left sidebar)** — tick to enable short-message decoding with
   your callsign and the other station's callsign. Required for decoding
   `<MYCALL HISCALL> RPT` style messages.
5. **● Record** — appears in the topbar once a stream is running. Toggling
   it on writes every received audio chunk to
   `captures/msk144plus_<unix-time>.wav` at 12 kHz mono 16-bit. Use this
   to gather off-air samples that you can later replay with
   `target/release/msk144plus_decode <wav>` to compare results.

## Decode log

The central panel shows decoded messages, newest at the top. Columns:

| UTC | Freq | xmax | Method | Text |
|---|---|---|---|---|
| Slot timestamp (HHMMSS) | Absolute frequency in Hz (fc + ferr) | Sync correlation strength | spd-pat0..5 / avg-4/5/7 / msk40-pat0..5 | Decoded message |

A single 15-second slot can produce multiple decodes — typically you'll
see the spd front-end catch the first ping, and the averaging-pattern
loop pick up later ones from the same slot.

## Caveats

- **Slot alignment**: The decoder runs on naive 15-second buffers (every
  15s of accumulated audio), not UTC slot boundaries. For typical use
  with the IC-9700, this is fine — the radio is a continuous receiver.
  If you want to compare directly against WSJT-X's output for the same
  signal, time-align by recording the WAV and running the CLI decoder.
- **Linux ALSA**: The audio module uses cpal which works on Linux via
  ALSA. If you want to run while WSJT-X is also using the device,
  you'll need to route audio through pipewire or pulseaudio shared
  loopbacks. The same rule as FSK441+ applies: only one process can
  exclusively claim an ALSA hardware device.
- **No TX**: This is RX-only. Transmit isn't wired up yet.

## Files written

- `captures/msk144plus_<unix-time>.wav` — recorded audio when Record
  is on. Mono 12 kHz 16-bit. Compatible with the CLI decoder.

## What this exists for

The point is to have something running on the IC-9700 to gather real
off-air MSK144 traffic so you can:

1. Confirm the v2 decoder works end-to-end against signals coming over
   the air, not just lab WAVs.
2. Build a corpus of off-air recordings to use as future regression
   tests (the existing 8 real-WAV regression cases are all from 2018
   recordings).
3. Compare decode rates against WSJT-X by pointing both at the same
   recorded WAV.
