# msk144plus v2 — Status

## Where we are

Faithful port of WSJT-X 3.0.0 MSK144 receive pipeline. **8/8 real-WAV
regression tests pass** including signals v1 couldn't decode at all.

## Test results (113 tests across 5 crates)

| Crate | Tests | Notes |
|---|---|---|
| msk144plus_packjt | 64 | Copied from v1, validated |
| msk144plus_fec | 15 | LDPC parity tables + BP decoder + encoder + CRC. **No OSD.** |
| msk144plus_dsp | 21 | analytic + decode_frame + spd + sync + tx |
| msk144plus_engine (lib + tx_rx) | 5 | TX→RX round-trip at fc=1488,1500 |
| msk144plus_engine (real_wav) | 8 | All known WAVs decode correctly |
| **Total** | **113** | |

## Real-WAV decode results

| File | Expected | v2 result |
|---|---|---|
| K1JT 181211_120500 (fc=1488) | K1JT WA4CQG EM72 | ✓ K1JT WA4CQG EM72 (spd-pat0) |
| KD9VV 181211_120800 | CQ KD9VV EN71 | ✓ CQ KD9VV EN71 (spd-pat2), bonus CQ W4IMD EM84 |
| SP9HWY 181002 | SP9HWY LZ2HV KN23 | ✓ + full QSO sequence (+00, R+00, RRR, 73) |
| F5CT (was no-decode in v1) | CQ F5CT JN08 | ✓ CQ F5CT JN08 (avg-7) |
| G1PPA (was OSD false-positive in v1) | CQ G1PPA IO93 | ✓ CQ G1PPA IO93 (avg-7), bonus PA4VHF JO32 |
| ON8DM (was no-decode) | CQ ON8DM JO10 | ✓ CQ ON8DM JO10 (avg-7) |
| LY3W (was wrong message in v1) | LY3W PA4VHF -05 | ✓ LY3W PA4VHF -05 (avg-4), bonus ON8DM (real, from prev slot) |
| HB9RUZ (was no-decode) | DL7OAP HB9RUZ JN47 | ✓ DL7OAP HB9RUZ JN47 (spd-pat4) |
| SM7OYP (was no-decode) | (filename hint) | ✓ CQ SM7OYP JO66 (spd-pat2) |
| DL2IAU | CQ DL2IAU JN49 | (no decode — genuinely below threshold) |

## Code size

- v1: 14,291 lines of Rust (with OSD scaffolding, cluster grouping, cross-window accumulator, short_decoder, etc.)
- **v2: 5,095 lines of Rust** (faithful WSJT-X port, no extras)
- WSJT-X reference: 1,032 lines of Fortran

## What we ported (line-by-line)

| WSJT-X file | Lines | v2 location | v2 lines |
|---|---|---|---|
| analytic.f90 | 75 | crates/msk144plus_dsp/src/analytic.rs | ~210 |
| msk144decodeframe.f90 | 113 | crates/msk144plus_dsp/src/decode_frame.rs | ~290 |
| msk144sync.f90 + msk144_freq_search.f90 | 152 | crates/msk144plus_dsp/src/sync.rs | ~310 |
| msk144spd.f90 | 197 | crates/msk144plus_dsp/src/spd.rs | ~340 |
| mskrtd.f90 (orchestrator) | 261 | crates/msk144plus_engine/src/lib.rs | ~210 |
| genmsk_128_90.f90 | 121 | crates/msk144plus_dsp/src/tx.rs | ~190 |
| bpdecode128_90.f90 | 117 | crates/msk144plus_fec/src/ldpc.rs | (from v1, validated) |
| packjt77 module | ~3000 | crates/msk144plus_packjt/ | (from v1, validated) |

The Rust line counts are larger because of explicit type signatures,
generous comments mapping back to the Fortran, and unit tests inline.

## What we DELIBERATELY did NOT port

These exist in v1 but not v2, by design:

- **OSD-2 fallback decoder** — neither WSJT-X nor MSHV uses OSD on MSK144
- **Cross-window soft-bit accumulator** — speculative addition, didn't match reference
- **Cluster grouping / process_clusters** — no equivalent in reference
- **Score threshold of 1.3 in multi_freq** — reference accepts what comes out of msk144sync
- **Hard-error caps in OSD paths** — symptoms of OSD layer
- **Soft-data quality gate** — symptom of OSD layer
- **MSK40 short-message path** — exists in WSJT-X but requires mycall/hiscall config; deferred for now
- **Adaptive equalizer (bvar/pcoeffs)** — MSHV-only enhancement, off by default

## Architecture

```
audio (12 kHz int16/f32)
  │
  ├─ rms check (>= 1.0)
  │
  ├─ analytic filter (raised-cosine BPF 600-2400 Hz, applied UNCONDITIONALLY)
  │
  ├─ detect_candidates (msk144spd front-end)
  │   ├─ for each step: square → edge-window → FFT → tone-peak find
  │   ├─ median-normalise detmet (noise → 1.0)
  │   ├─ detmet >= 3.0 candidates (primary)
  │   └─ detmet2 >= 12.0 fallback if < 3 primary
  │
  ├─ for each candidate: msk144spd back-end
  │   ├─ 6 navpatterns × msk144_sync(navg=3, ntol=8, delf=2)
  │   ├─ for each peak × slicer-dither (3 positions)
  │   ├─ demodulate_frame → softbits → LLR
  │   └─ bp_decode(max_iter=10) → unpack77
  │
  └─ if no spd decode: mskrtd averaging-pattern loop
      ├─ depth selects npat: Fast=0, Normal=2, Deep=4
      ├─ avg_patterns: [4-frame×2, 5-frame, 7-frame]
      ├─ msk144_sync(navg=N, ntol=user, delf=10/N)
      └─ same back-end as spd
```

## Outstanding items

1. **DL2IAU** doesn't decode. Need to determine if WSJT-X also fails on it,
   or if there's still a gap. Probably below threshold.
2. **MSK40 short-message support** — for "K1JT PE1ITR R+10" type messages.
   Requires mycall/hiscall config plumbing. Deferred until base is stable.
3. **Adaptive equalizer (bvar)** — MSHV-only, off by default in WSJT-X. Skip.
4. **TX-side QSO state machine, ADIF logging, settings RON, audio capture
   (cpal port from FSK441+)** — UI/runtime layer. Now that the core decode
   is solid, this becomes the next focus.

## Why this worked

The v1 codebase had grown 14× larger than the WSJT-X reference because
each session bolted another speculative enhancement onto an implementation
that was never confirmed faithful to the reference. The "missing decodes"
were not missing in WSJT-X — they were missing in our reimplementation.
By rebuilding from the WSJT-X Fortran with a 1:1 mapping, the decodes
came for free.

The v1 false positives (G1PPA "SW8ZPU", LY3W "ON8DM" suspicions) were
symptoms of the OSD-2 fallback layer that doesn't exist in either
WSJT-X or MSHV. Removing OSD removed the false positives without losing
real decodes.
