# MSK144 Decode Pipeline: WSJT-X vs MSHV Side-by-Side Reference

The purpose of this document is to map every step of the MSK144 receive pipeline
across the two reference implementations. We use this as the specification for
the Rust port. Where the two agree, that's the protocol. Where they differ,
those are documented implementation choices we'll make ourselves.

Sources:
- WSJT-X 3.0.0 Fortran in `/home/claude/wsjtx/wsjtx-3.0.0/lib/`
- MSHV C++ in `/tmp/MSHV/src/HvDecoderMs/decodermsk144.cpp`

Conventions: All sample rates are 12 kHz unless stated. NSPM=864 = one MSK144
frame = 72 ms. NPTS=7×NSPM = 6048 samples = 504 ms. A 15 s slot is 180000 samples;
a 30 s slot is 360000. The transmitted message is repeated every NSPM samples,
so a 15 s slot contains 208 frame copies and a 30 s slot contains 416.

═══════════════════════════════════════════════════════════════════════════════
Stage 0 — Audio acquisition and preprocessing
═══════════════════════════════════════════════════════════════════════════════

WSJT-X
------
Top-level entry: `decode_msk144.f90`.
- Reads `id2(0:NMAX-1)` from shared memory: `int*2` samples at 12 kHz, where
  NMAX is enough for a 30-second slot.
- The decoder runs **incrementally**, called from `hspec.f90` line 99 at
  half-block increments (~0.3 s) as audio accumulates. Each call passes a
  **7168-sample window** (`id2(k-7168+1:k)`) ending at the current audio
  position. So a 15 s slot is processed by ~49 overlapping calls to mskrtd,
  each analyzing the most recent ~597 ms of audio.
- `mskrtd(id2, nutc0, tsec, ntol, nrxfreq, ndepth, mycall, hiscall, ...)`
  is the entry-point for one 7168-sample window.

Constants (mskrtd.f90 lines 10-14):
  NZ = 7168          ! 597 ms window
  NSPM = 864         ! 72 ms message frame
  NFFT1 = 8192       ! FFT size for analytic signal
  NPATTERNS = 4      ! averaging patterns

MSHV
----
Top-level entry: `msk_144_40_decode(dat, npts_in, s_istart, rpt_db_msk)`.
- Receives `double *dat`, length `npts_in`, at 12 kHz.
- Calls `analytic_msk144_2` to convert real samples to complex analytic with
  the band-pass filter applied (this is BEFORE detection).
- Calls `detectmsk144(cbig, npts, s_istart, nmessages)` — the ping detector.
- Calls `msk144spd(...)` for each detected ping.
- For each candidate, calls `msk144decodeframe(...)`.
- `msk_144_40_rtd` is the real-time variant called every second; it has the
  same structure but with bookkeeping for incremental display.

Agreement
---------
- 12 kHz int16 input at the I/O boundary
- Analytic-signal conversion is the first DSP step
- Both have a "real-time / per-second" variant and a "full slot" variant

Divergence
----------
- WSJT-X's mskrtd processes 1-second sliding windows through the slot;
  MSHV processes the whole slot at once and uses detectmsk144 as a fast
  ping-locator

═══════════════════════════════════════════════════════════════════════════════
Stage 1 — Analytic signal & band-pass filtering
═══════════════════════════════════════════════════════════════════════════════

WSJT-X: `analytic.f90` (75 lines)
---------------------------------
```
subroutine analytic(d,npts,nfft,c)
  complex c(0:NFFT-1)
  c=cmplx(d(1:nfft),0.0)
  call four2a(c,nfft,1,-1,1)             ! Real-to-complex FFT
  ! Build raised-cosine filter in frequency domain:
  !   passband: 600..2400 Hz (flat)
  !   transition: 600..400 Hz at low edge, 2400..2600 at high edge
  ! Apply to spectrum, zero everything else (also zeroes negative freqs
  ! because we only multiply 0..NFFT/2; this gives the analytic signal)
  call four2a(c,nfft,1,1,1)              ! Inverse FFT
end subroutine
```
Output: complex baseband at 12 kHz, 600-2400 Hz pass, raised-cosine edges.

MSHV: `analytic_msk144` (line 117) and `analytic_msk144_2` (line 217/306)
-------------------------------------------------------------------------
- `analytic_msk144`: Identical structure to WSJT-X analytic.f90 — FFT, apply
  raised-cosine BPF in frequency domain, inverse FFT.
- `analytic_msk144_2`: Adds an "equalizer" feature controlled by `s_msk144_eq`
  (see SetMsk144RxEqual). Multiplies the spectrum by a learned correction
  curve. Off by default.

Agreement
---------
- BPF passband 600-2400 Hz with raised-cosine edges from 400-600 Hz and
  2400-2600 Hz
- Filter applied via FFT → multiply → IFFT
- Negative frequencies zeroed (analytic signal)

Divergence
----------
- MSHV adds optional adaptive equalization
- MSHV has explicit `t=1/2000`, `beta=0.1` parameters in the raised-cosine
  formula; WSJT-X uses the same shape implicitly

Specification: WSJT-X analytic.f90 is the reference. MSHV's basic version
matches it. The equalizer is an MSHV-only enhancement.

═══════════════════════════════════════════════════════════════════════════════
Stage 2 — Ping detection (front-end candidate finder)
═══════════════════════════════════════════════════════════════════════════════

WSJT-X: `msk144spd.f90` (196 lines)
-----------------------------------
Operates on already-analytic baseband.
- Steps in `nstepsize=216` sample chunks (18 ms) across the input buffer.
- At each step `istp`, takes a NSPM=864 sample window of analytic signal.
- Squares the analytic signal: `ctmp = cbig(ns:ne) * cbig(ns:ne)` — this
  collapses the MSK FM modulation into pure tones at 2×f_data.
- Applies raised-cosine edge windowing to the squared signal (12 samples each
  end).
- FFT.
- Looks for peaks in the squared-signal spectrum near f=2×(fc±500) Hz —
  i.e., near the 2×lower-tone (~2000 Hz baseband for fc=1500) and the
  2×upper-tone (~4000 Hz baseband). The window widths are determined by
  `ntol`:
    ihlo = (i4000 - 2*ntol)/2,  ihhi = (i4000 + 2*ntol)/2
    illo = (i2000 - 2*ntol)/2,  ilhi = (i2000 + 2*ntol)/2
- Finds the peak in each window, then computes a "tone ratio":
    `trath = peak_high / (avg_high - peak_high/window_count)`
    `tratl = peak_low  / (avg_low  - peak_low /window_count)`
- `detmet[istp]` = max(peak_high, peak_low) absolute (the tone amplitude)
- `detmet2[istp]` = max(trath, tratl) (peak-to-average ratio)
- `detfer[istp]` = parabolic-interpolated frequency error in Hz from the
  peak
- After the sweep, normalise `detmet` by the 25th-percentile median
  (`xmed = sorted(nstep/4)`) so noise floor → 1.0.
- Pick top candidates: `detmet >= 3.0` AND `|detfer| <= ntol`. Fallback: if
  none found, take the highest detmet.
- For each candidate, take a 3-NSPM-wide window centered on the candidate
  (handles boundary cases).
- Calls `msk144_freq_search` for fine frequency refinement.
- Calls `msk144decodeframe` for the actual decode.

MSHV: `detectmsk144` (line 595, ~536 lines) and `msk144spd` (line 1966, ~261 lines)
-----------------------------------------------------------------------------------
- `detectmsk144` is the equivalent of WSJT-X's spd top half.
- Same structure: step through, square, FFT, peak-find in two tone windows.
- Critical threshold differences:
  - WSJT-X uses `detmet >= 3.0` for the primary candidate gate.
  - **MSHV uses `detmet < 3.5` to break out** (line 764) — slightly stricter.
- MSHV also has a "ping duration" estimator (`MskPingDuration`, line 558)
  that returns how long the high-detmet region lasts.
- MSHV has `opdetmsk144` (line 1217) — the "deep" / operator-clicked variant
  with NAVG=7 frames coherent integration and 1 Hz freq sweep. This is
  separate from detectmsk144.
- MSHV's `msk144spd` (line 1966) is the per-candidate analyser, not the full
  detector — different layering from WSJT-X.

Agreement
---------
- Square the analytic signal to convert MSK FM → tones
- Raised-cosine windowing of the squared signal
- Look for peaks near 2×(fc±500) Hz
- Tone-ratio metric for false-positive rejection
- Median-normalize so noise → 1.0
- detfer/ferr is parabolic-interpolated peak position

Divergence
----------
- Threshold: WSJT-X 3.0, MSHV 3.5
- MSHV has additional ping-duration metric
- MSHV has optdetmsk144 as a separate code path; WSJT-X does deep search via
  averaging patterns inside `mskrtd` instead
- MSHV uses dynamic step size; WSJT-X uses fixed 216 samples

Specification: WSJT-X msk144spd.f90 is the cleanest reference. We port that
first, with detmet >= 3.0 threshold.

═══════════════════════════════════════════════════════════════════════════════
Stage 3 — Sync correlation & multi-frame averaging
═══════════════════════════════════════════════════════════════════════════════

WSJT-X: `msk144sync.f90` (101 lines) + `msk144_freq_search.f90` (50 lines)
--------------------------------------------------------------------------
`msk144sync(cdat, nframes, ntol, delf, navmask, npeaks, fc, fest, npkloc, nsuccess, xmax, c)`:
- Input: `cdat` of length `nframes*NSPM`, mask `navmask` selecting which
  frames to sum, ntol/delf for freq sweep.
- For each freq offset `df = -ntol..ntol step delf`:
  - Heterodyne `cdat` to baseband-relative-to-fc: `cdat * exp(-j2π(fc+df)t)`
  - Coherent average across `navmask`: `c = sum_i cdat(i*NSPM:(i+1)*NSPM) * navmask(i)` / sum(navmask)
  - For each cyclic shift `ish = 0..NSPM-1`:
    - `ct = cshift(c, ish)`
    - `cc(ish) = sum_{j=1..42}(ct(j) + ct(56*6+j-1)) * conj(cb(j))`
    - cb is the precomputed sync waveform (8-symbol S8 = [0,1,1,1,0,0,1,0])
- Find peak: `xmax = max(|cc|)` across all (df, ish) — this is the sync
  correlation strength.
- Returns top `npeaks` peak locations and best frequency.

`msk144_freq_search(cdat, fc, if1, if2, delf, nframes, navmask, fest, xmax, c)`:
- Slimmer freq-only refinement variant, called by msk144spd to lock the
  frequency once a candidate has been found.

The `mskrtd.f90` averaging-pattern loop (lines 146-171):
```
NPATTERNS = 4
iavpatterns:
  [1,1,1,1,0,0,0,0]    ! 4-frame, first half
  [0,0,1,1,1,1,0,0]    ! 4-frame, middle
  [1,1,1,1,1,0,0,0]    ! 5-frame
  [1,1,1,1,1,1,1,0]    ! 7-frame

! ndepth selects how many patterns are tried (operator setting):
!   ndepth=1 (Fast):   npat=0 — short-ping decoder ONLY, skip averaging
!   ndepth=2 (Normal): npat=2 — try only the two 4-frame patterns
!   ndepth=3 (Deep):   npat=4 — try all 4 including 5-frame and 7-frame

do iavg=1, npat
  navg = sum(iavpatterns)
  delf = 10.0 / navg               ! finer freq step for more averaging
  call msk144sync(cdat, 8, ntol, delf, navmask, npeaks, fc, fest, npkloc, nsync, xmax, c)
  do ipk=1, npeaks  ! npeaks=2
    do is=1, 3       ! slicer dither at peak ± 1
      ct = cshift(c, ic0-1)
      call msk144decodeframe(ct, softbits, msg, ndecodesuccess)
      if(ndecodesuccess > 0) goto 900   ! emit decode
    enddo
  enddo
enddo
```

MSHV: `msk144sync` (line 1865) + `msk144_freq_search` (line 1794)
-----------------------------------------------------------------
- Same structure as WSJT-X.
- Same averaging-pattern table.
- Calls `msk144decodeframe_p` (with phase) and `msk144decodeframe` (auto-phase).
- Has `opdetmsk144` as a separate brute-force search at NAVG=7 with 1 Hz
  freq step and full NSPM cyclic shift.

Agreement
---------
- Sync waveform: 8-bit pattern S8=[0,1,1,1,0,0,1,0], first 8 channel symbols
  AND symbols 56..63 (sync repeats at the half-frame boundary)
- Cross-correlation: `cc(ish) = sum_{j=1..42}(ct(j) + ct(56*6+j-1)) * conj(cb(j))`
  where cb is the matched-filter pulse-shaped sync
- Frequency sweep with delf = 10.0 / navg
- 4 averaging patterns: [1,1,1,1,0,0,0,0], [0,0,1,1,1,1,0,0],
  [1,1,1,1,1,0,0,0], [1,1,1,1,1,1,1,0]
- Top 2 peaks per pattern × 3 slicer dithers = 6 decode attempts per pattern

Divergence
----------
- MSHV's deep mode uses fixed NAVG=7 and brute-force; WSJT-X iterates patterns.

Specification: WSJT-X mskrtd's averaging-pattern loop is the spec. Our v1 had
this. The implementation needs to match exactly.

═══════════════════════════════════════════════════════════════════════════════
Stage 4 — Frame demodulation & soft-bit extraction
═══════════════════════════════════════════════════════════════════════════════

WSJT-X: `msk144decodeframe.f90` (112 lines)
-------------------------------------------
Input: 864-sample complex frame `c`, already aligned to sync.
```
1. Estimate carrier phase from SYNC1 and SYNC2 positions:
     cca = sum(c(1:42) * conj(cb))            ! 1st sync correlation
     ccb = sum(c(56*6:56*6+41) * conj(cb))    ! 2nd sync correlation
     phase0a = atan2(imag(cca), real(cca))
     phase0b = atan2(imag(ccb), real(ccb))
     phase0 = (phase0a + phase0b) / 2          ! average

2. Counter-rotate: c = c * exp(-j*phase0)

3. Matched filter with half-sine pulse pp(1:12) = sin((i-1)*pi/12):
     softbits(1)   = sum(imag(c(1:6))*pp(7:12)) + sum(imag(c(859:864))*pp(1:6))
     softbits(2)   = sum(real(c(1:12))*pp)
     do i=2..72:
       softbits(2*i-1) = sum(imag(c(...))*pp)
       softbits(2*i)   = sum(real(c(...))*pp)

4. Hard-decision sync error count:
     nbadsync = number of sync bits where hardbits[i] != S8[i] (in S8-positions)
     if(nbadsync > 4) return                    ! reject

5. Normalize: softbits = softbits / std(softbits)

6. Build 128-bit LLR (skip 8 sync bits at front, 8 at midpoint):
     llr(1:48)  = softbits(9:56)
     llr(49:128) = softbits(65:144)
     llr = 2.0 * llr / (sigma*sigma)            ! sigma=0.60

7. Call bpdecode128_90(llr, apmask, max_iter=10, decoded77, cw, nharderror, niterations)

8. If nharderror >= 0 AND nharderror < 18:
     - Validate i3/n3 type bits (reject reserved combinations)
     - Call unpack77(decoded77, msg)
     - if unpack ok: nsuccess=1
```

MSHV: `msk144decodeframe_p` (line 1554) and `msk144decodeframe` (line 1740)
---------------------------------------------------------------------------
- `_p` variant takes phase0 as argument (caller computed it)
- Auto-phase variant computes phase exactly like WSJT-X.
- Same matched filter, same softbit indexing.
- Same nbadsync > 4 reject, same sigma=0.60, same max_iter=10.
- Same nharderror < 18 success criterion.
- Calls `bpdecode128_90` (their port).
- Calls `unpack77` (their port).

Agreement
---------
EXACT match between WSJT-X and MSHV at this stage. This is the protocol-defined
demodulator and its parameters are fixed:
- Phase estimate from average of 1st-sync and 2nd-sync corrections
- Matched filter with half-sine pp(i)=sin((i-1)π/12)
- nbadsync > 4 → reject
- sigma=0.60 in LLR scaling
- BP max iterations=10
- nharderror < 18 → accept

Specification: This stage is unambiguous. Port WSJT-X msk144decodeframe.f90
verbatim.

═══════════════════════════════════════════════════════════════════════════════
Stage 5 — LDPC belief propagation
═══════════════════════════════════════════════════════════════════════════════

WSJT-X: `bpdecode128_90.f90` (117 lines)
-----------------------------------------
- (128, 90) LDPC code from `ldpc_128_90_generator.f90` parity-check matrix.
- Standard Gallager belief propagation, message-passing on the bipartite
  graph.
- Per-iteration:
  1. Update bit-to-check messages
  2. Apply tanh transform (or its approximation)
  3. Update check-to-bit messages
  4. Sum incoming → bit log-likelihood-ratios (LLRs)
  5. Hard decision and parity check
  6. If all parity equations satisfied → return
- Max iterations: 10
- Returns nharderror = Hamming distance between hard-decision LLR and
  re-encoded codeword. -1 means BP did not converge to a codeword.

MSHV: bpdecode128_90 (in their decoderms.cpp)
---------------------------------------------
Direct port of WSJT-X's Fortran. Same algorithm.

Agreement
---------
Algorithm is fully specified by the LDPC parity matrix + standard BP. Both
implementations are identical.

Specification: Port WSJT-X bpdecode128_90.f90 verbatim. Our v1 has this and
the parity tables are correct.

═══════════════════════════════════════════════════════════════════════════════
Stage 6 — Message unpacking
═══════════════════════════════════════════════════════════════════════════════

WSJT-X: `packjt77.f90` (used both directions)
---------------------------------------------
- Decodes 77-bit message according to i3/n3 type bits:
  - i3=0,n3=0: free text (71 bits, 13×5-bit chars)
  - i3=1,n3=*: standard call+call+grid/report
  - i3=2: EU VHF contest
  - i3=3: ARRL field day
  - i3=4: nonstandard call
  - i3=5: telemetry
- Built on top of pack28/pack15/jenkins-hash-12.

MSHV: their port of packjt77
----------------------------
- Same algorithm, same message types.

Agreement
---------
Fully specified by the FT8/MSK144 message format. Our v1 packjt has 64 tests
passing. Keep it.

═══════════════════════════════════════════════════════════════════════════════
Differences in DECODING STRATEGY (the layer above the protocol)
═══════════════════════════════════════════════════════════════════════════════

Once the per-stage operations are in place, the strategy is how you USE them.
This is where WSJT-X and MSHV diverge most:

| Strategy element        | WSJT-X         | MSHV                      |
|-------------------------|----------------|---------------------------|
| spd threshold           | 3.0            | 3.5                       |
| Per-second sliding      | 1s steps       | Whole-slot detect, then per-ping |
| Averaging patterns      | 4 fixed        | Same 4 + opdet uses fixed 7-frame |
| Deep mode               | iterates patterns | Separate opdetmsk144 with NAVG=7, 1 Hz freq, full cyclic shift |
| OSD on MSK144           | NO             | NO                        |
| MSK40 short mode        | bshmsg flag    | Always tries both 144 and 40 |
| Adaptive equalizer      | none           | optional (off by default) |

Both implementations:
- Use BP with max_iter=10
- Accept nharderror < 18
- Reject nbadsync > 4 in demod
- Use sigma=0.60 in LLR scaling

═══════════════════════════════════════════════════════════════════════════════
What our v1 got wrong
═══════════════════════════════════════════════════════════════════════════════

After this analysis, the deltas of our v1 implementation vs the reference:

1. **Analytic signal not applied at the front**. We made the BPF "opt-in" via
   `use_analytic_filter`. WSJT-X and MSHV both apply it unconditionally as
   the first DSP step. Our v1 default does NOT match the reference.

2. **OSD-2 fallback layer**. Neither WSJT-X nor MSHV uses OSD on MSK144. Our
   v1 added it as a "second-chance" decoder. This is the source of all our
   false-positive cases (ON8DM, SM7OYP, G1PPA, the suspicious LY3W/ON8DM
   decode). Should be removed.

3. **Cross-window soft-bit accumulator**. Not present in either reference for
   the standard MSK144 path. Was added speculatively. Removes.

4. **Cluster grouping**. Our `process_clusters` and `cluster_path` logic has
   no equivalent in either reference. Remove.

5. **Hard-error caps at 18 in OSD paths**. Symptom of (2). Removes with OSD.

6. **Custom soft-data quality gate**. Symptom of (2). Removes with OSD.

7. **Score-threshold of 1.3 in multi_freq.rs**. Not in either reference;
   the reference accepts whatever sync correlation comes out of msk144sync
   (it's already implicitly gated by the bpdecode threshold).

What our v1 got RIGHT (keep, port verbatim):

- packjt77 module: 64 tests, validated
- LDPC parity tables (the constants in fec/parity.rs)
- TX path: encode_128_90, build_channel_bits, generate_msk144_slot
- The BP decoder algorithm (algorithm correct, tests pass)
- The msk144spd port (in `examples/msk144spd_port.rs`) — validate its math
- The general crate structure (dsp/fec/packjt/engine separation)

═══════════════════════════════════════════════════════════════════════════════
Port plan (revised after this analysis)
═══════════════════════════════════════════════════════════════════════════════

Stage 1: Set up `msk144plus_v2/` with empty crates.

Stage 2: Port the unambiguous, isolated stages with reference unit tests:
  a. packjt77 (copy from v1, 64 tests)
  b. bpdecode128_90 (port verbatim, write tests using known LLR vectors)
  c. encode_128_90 + build_channel_bits + generate_msk144_slot (TX path)
  d. analytic.f90 (port verbatim, test against impulse → expected step response)
  e. msk144decodeframe (port verbatim, test against generate→demod → bits roundtrip)

Stage 3: Port the detector and orchestrator:
  f. msk144spd → ping detection candidate list
  g. msk144sync → freq+timing refinement, returns top peaks
  h. mskrtd → top-level: 1s sliding window, 4 averaging patterns, decode loop

Stage 4: Validate against real WAVs:
  - K1JT, KD9VV, SP9HWY must decode (these decode in WSJT-X/MSHV)
  - ON8DM, SM7OYP, G1PPA, F5CT, LY3W must NOT decode (also true in WSJT-X/MSHV)
  - The synthetic test files: should match MSHV behavior (which decodes them
    only via msk40 short-message path with mycall/hiscall set)

Stage 5: ONLY after Stage 4 passes, consider enhancements:
  - MSK40 short-message path (off by default, mycall/hiscall config)
  - Cross-window accumulation (WITH a benchmark proving it improves real decode
    rates without introducing false positives)
  - Anything else
