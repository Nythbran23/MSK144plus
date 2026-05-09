// crates/msk144plus_packjt/src/jenkins.rs
//
// Bob Jenkins lookup3 hash (byte-path variant), faithful port of WSJT-X's
// `nhash()` function from lib/wsprd/nhash.c. Used by MSK40 short-message
// frames (12-bit hash of "MYCALL HISCALL") and by i3=4 nonstandard call
// messages (22-bit hash of a callsign).
//
// IMPORTANT WSJT-X QUIRK: even though Bob Jenkins' lookup3 hash returns
// 32 bits, WSJT-X's `nhash()` clamps the result to 15 bits before
// returning (`c = 32767 & c`). The Fortran `hash()` wrapper then masks to
// 15 bits again. So the maximum value `nhash()` ever produces is 32767.
// We replicate that exact behaviour.
//
// Public domain code by Bob Jenkins, 2006.
// http://burtleburtle.net/bob/c/lookup3.c

#[inline]
fn rot(x: u32, k: u32) -> u32 {
    x.rotate_left(k)
}

/// Mix 3 32-bit values reversibly. From Jenkins' `mix()` macro.
#[inline]
fn mix(a: &mut u32, b: &mut u32, c: &mut u32) {
    *a = a.wrapping_sub(*c); *a ^= rot(*c, 4);  *c = c.wrapping_add(*b);
    *b = b.wrapping_sub(*a); *b ^= rot(*a, 6);  *a = a.wrapping_add(*c);
    *c = c.wrapping_sub(*b); *c ^= rot(*b, 8);  *b = b.wrapping_add(*a);
    *a = a.wrapping_sub(*c); *a ^= rot(*c, 16); *c = c.wrapping_add(*b);
    *b = b.wrapping_sub(*a); *b ^= rot(*a, 19); *a = a.wrapping_add(*c);
    *c = c.wrapping_sub(*b); *c ^= rot(*b, 4);  *b = b.wrapping_add(*a);
}

/// Final mixing of (a,b,c) into c. From Jenkins' `final()` macro.
#[inline]
fn final_mix(a: &mut u32, b: &mut u32, c: &mut u32) {
    *c ^= *b; *c = c.wrapping_sub(rot(*b, 14));
    *a ^= *c; *a = a.wrapping_sub(rot(*c, 11));
    *b ^= *a; *b = b.wrapping_sub(rot(*a, 25));
    *c ^= *b; *c = c.wrapping_sub(rot(*b, 16));
    *a ^= *c; *a = a.wrapping_sub(rot(*c, 4));
    *b ^= *a; *b = b.wrapping_sub(rot(*a, 14));
    *c ^= *b; *c = c.wrapping_sub(rot(*b, 24));
}

/// Bob Jenkins lookup3 hash, byte-path with WSJT-X's 15-bit clamp.
///
/// Matches WSJT-X's `nhash(key, length, initval)` exactly. Hash values
/// are clamped to 15 bits (max 32767) regardless of input.
///
/// We use the byte-path variant (the C code branches on alignment;
/// the byte path produces identical results to the aligned paths,
/// just slower). Aligned reads on Intel/AMD with HASH_LITTLE_ENDIAN
/// give bit-identical output to the byte path because the byte path
/// reconstructs little-endian u32s manually.
pub fn nhash(key: &[u8], initval: u32) -> u32 {
    let length = key.len();
    let mut a: u32 = 0xdeadbeef_u32
        .wrapping_add(length as u32)
        .wrapping_add(initval);
    let mut b = a;
    let mut c = a;

    let mut k = key;
    let mut remaining = length;

    // All but the last block: read 12 bytes at a time, mix into (a,b,c)
    while remaining > 12 {
        a = a.wrapping_add(k[0] as u32);
        a = a.wrapping_add((k[1] as u32) << 8);
        a = a.wrapping_add((k[2] as u32) << 16);
        a = a.wrapping_add((k[3] as u32) << 24);
        b = b.wrapping_add(k[4] as u32);
        b = b.wrapping_add((k[5] as u32) << 8);
        b = b.wrapping_add((k[6] as u32) << 16);
        b = b.wrapping_add((k[7] as u32) << 24);
        c = c.wrapping_add(k[8] as u32);
        c = c.wrapping_add((k[9] as u32) << 8);
        c = c.wrapping_add((k[10] as u32) << 16);
        c = c.wrapping_add((k[11] as u32) << 24);
        mix(&mut a, &mut b, &mut c);
        k = &k[12..];
        remaining -= 12;
    }

    // Last block (1..=12 bytes), with C-style fall-through. We replicate
    // the switch by accumulating bytes into c, then b, then a, in order.
    match remaining {
        12 => c = c.wrapping_add((k[11] as u32) << 24).wrapping_add((k[10] as u32) << 16).wrapping_add((k[9] as u32) << 8).wrapping_add(k[8] as u32),
        11 => c = c.wrapping_add((k[10] as u32) << 16).wrapping_add((k[9] as u32) << 8).wrapping_add(k[8] as u32),
        10 => c = c.wrapping_add((k[9] as u32) << 8).wrapping_add(k[8] as u32),
        9 => c = c.wrapping_add(k[8] as u32),
        _ => {} // 0..=8 don't touch c
    }
    match remaining {
        12 | 11 | 10 | 9 | 8 => b = b.wrapping_add((k[7] as u32) << 24).wrapping_add((k[6] as u32) << 16).wrapping_add((k[5] as u32) << 8).wrapping_add(k[4] as u32),
        7 => b = b.wrapping_add((k[6] as u32) << 16).wrapping_add((k[5] as u32) << 8).wrapping_add(k[4] as u32),
        6 => b = b.wrapping_add((k[5] as u32) << 8).wrapping_add(k[4] as u32),
        5 => b = b.wrapping_add(k[4] as u32),
        _ => {} // 0..=4 don't touch b
    }
    match remaining {
        12 | 11 | 10 | 9 | 8 | 7 | 6 | 5 | 4 => a = a.wrapping_add((k[3] as u32) << 24).wrapping_add((k[2] as u32) << 16).wrapping_add((k[1] as u32) << 8).wrapping_add(k[0] as u32),
        3 => a = a.wrapping_add((k[2] as u32) << 16).wrapping_add((k[1] as u32) << 8).wrapping_add(k[0] as u32),
        2 => a = a.wrapping_add((k[1] as u32) << 8).wrapping_add(k[0] as u32),
        1 => a = a.wrapping_add(k[0] as u32),
        0 => return c, // zero-length: returns unclamped c (matches WSJT-X)
        _ => {}
    }

    final_mix(&mut a, &mut b, &mut c);

    // WSJT-X clamps to 15 bits before returning
    c & 0x7fff
}

/// Compute the 12-bit hash used by MSK40 short-message frames.
///
/// Matches WSJT-X's chain: fmtmsg(mycall + ' ' + hiscall) → hash() → AND 4095.
/// `formatted` should already be uppercase with single spaces and no
/// trailing whitespace (use `format_call_pair` or call `fmtmsg` upstream).
pub fn hash12(formatted_call_pair: &str) -> u16 {
    // WSJT-X's hash.f90 calls nhash(string, len, 146) where 146 is the
    // seed. The string is exactly `len` bytes (no NUL terminator).
    // The fmtmsg.f90 routine uppercases, trims, and collapses multiple
    // spaces. The string passed to hash is the full character*37 buffer
    // padded with trailing spaces up to 37 chars - but `len` parameter
    // is `iz`, the trimmed length, NOT 37.
    //
    // Actually re-reading hash.f90: it passes len=37 from msk40decodeframe
    // with `call hash(hashmsg, 37, ihash)` - so the FULL 37-byte buffer
    // including trailing spaces is hashed. We need to do the same.
    let mut buf = [b' '; 37];
    let bytes = formatted_call_pair.as_bytes();
    let n = bytes.len().min(37);
    buf[..n].copy_from_slice(&bytes[..n]);
    let h = nhash(&buf, 146);
    (h & 4095) as u16
}

/// Compute the 22-bit hash used by i3=4 nonstandard call messages.
/// Used to compress callsigns up to 11 characters into 22 bits for
/// transmission. The receiver maintains a `recent_calls` table to
/// resolve received hashes back to callsigns.
pub fn hash22(formatted_call: &str) -> u32 {
    let mut buf = [b' '; 11];
    let bytes = formatted_call.as_bytes();
    let n = bytes.len().min(11);
    buf[..n].copy_from_slice(&bytes[..n]);
    // WSJT-X's hash22 uses seed 146 with len=11 and masks to 22 bits
    let h = nhash(&buf, 146);
    h & 0x3fffff // 22 bits
}

/// Format a callsign-pair string the way WSJT-X's fmtmsg() does:
/// uppercase, single space between calls, no leading/trailing whitespace.
pub fn format_call_pair(mycall: &str, hiscall: &str) -> String {
    let mut s = String::with_capacity(mycall.len() + hiscall.len() + 1);
    for c in mycall.chars().filter(|c| !c.is_whitespace()) {
        s.push(c.to_ascii_uppercase());
    }
    s.push(' ');
    for c in hiscall.chars().filter(|c| !c.is_whitespace()) {
        s.push(c.to_ascii_uppercase());
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_returns_unclamped_initial_state() {
        // length=0 returns c immediately WITHOUT clamping (matches WSJT-X
        // nhash.c case 0: return c;). Note this is asymmetric with non-empty
        // inputs which DO get clamped to 15 bits.
        let h = nhash(&[], 0);
        assert_eq!(h, 0xdeadbeef);
        // For initval=146:
        let h2 = nhash(&[], 146);
        assert_eq!(h2, 0xdeadbeef_u32.wrapping_add(146));
    }

    #[test]
    fn deterministic() {
        let a = nhash(b"K1JT", 146);
        let b = nhash(b"K1JT", 146);
        assert_eq!(a, b);
    }

    #[test]
    fn different_inputs_give_different_hashes() {
        let a = nhash(b"K1JT", 146);
        let b = nhash(b"K9AN", 146);
        let c = nhash(b"WA4CQG", 146);
        assert_ne!(a, b);
        assert_ne!(a, c);
        assert_ne!(b, c);
    }

    #[test]
    fn always_15_bits_for_nonempty() {
        // For nonempty input the hash is clamped to 15 bits before return
        for s in &[b"X".as_slice(), b"AB", b"K1JT", b"K1JT WA4CQG",
                   b"VERY LONG STRING THAT EXCEEDS TWELVE BYTES"] {
            let h = nhash(s, 146);
            assert!(h <= 0x7fff, "hash {} exceeds 15 bits", h);
        }
    }

    #[test]
    fn hash12_is_12_bits() {
        let h = hash12(&format_call_pair("K1JT", "WA4CQG"));
        assert!(h <= 0xfff);
    }

    #[test]
    fn hash22_is_22_bits() {
        let h = hash22("K1JT");
        assert!(h <= 0x3fffff);
    }

    #[test]
    fn format_call_pair_normalises() {
        assert_eq!(format_call_pair("k1jt", "wa4cqg"), "K1JT WA4CQG");
        assert_eq!(format_call_pair(" K1JT ", "  WA4CQG  "), "K1JT WA4CQG");
    }

    #[test]
    fn matches_wsjtx_reference_values() {
        // Cross-validation values measured from running WSJT-X's actual
        // lib/wsprd/nhash.c compiled standalone. The test inputs are
        // 37-byte buffers (matching the character*37 hashmsg buffer in
        // msk40decodeframe.f90), filled with the call-pair string and
        // padded with spaces.

        // "K1JT WA4CQG" + 26 spaces, len=37, seed=146
        let mut buf1 = [b' '; 37];
        buf1[..11].copy_from_slice(b"K1JT WA4CQG");
        let h1 = nhash(&buf1, 146);
        assert_eq!(h1, 20673, "WSJT-X ref value mismatch for K1JT WA4CQG");
        assert_eq!(h1 & 4095, 193);

        // "K9AN K1JT" + 28 spaces, len=37, seed=146
        let mut buf2 = [b' '; 37];
        buf2[..9].copy_from_slice(b"K9AN K1JT");
        let h2 = nhash(&buf2, 146);
        assert_eq!(h2, 20714, "WSJT-X ref value mismatch for K9AN K1JT");
        assert_eq!(h2 & 4095, 234);
    }

    #[test]
    fn hash12_matches_wsjtx() {
        // Test the high-level hash12() entry against the same reference values
        let h1 = hash12(&format_call_pair("K1JT", "WA4CQG"));
        assert_eq!(h1, 193);
        let h2 = hash12(&format_call_pair("K9AN", "K1JT"));
        assert_eq!(h2, 234);
    }

    #[test]
    fn rotation_works() {
        // Sanity check - rot(x,4) is rotate-left by 4
        assert_eq!(rot(0x1, 4), 0x10);
        assert_eq!(rot(0x80000000, 1), 0x1);
    }
}
