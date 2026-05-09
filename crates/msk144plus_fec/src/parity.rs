// crates/msk144plus_fec/src/parity.rs
//
// LDPC (128,90) parity-check structure tables.
// Transcribed from WSJT-X lib/ldpc_128_90_reordered_parity.f90 and converted
// to 0-indexed Rust constants. The polynomial structure is identical to the
// Fortran original; only the index base changes.
//
// Code parameters:
//   N = 128 (codeword length)
//   K = 90  (information length, of which 77 are message bits and 13 are CRC)
//   M = 38  (parity checks)
//   ncw = 3 (column weight - constant; this is a regular LDPC on the bit side)
//   row weights vary 10-11 (slightly irregular on the check side)

pub const LDPC_N: usize = 128;
pub const LDPC_K: usize = 90;
pub const LDPC_M: usize = 38;
pub const LDPC_NCW: usize = 3;
pub const LDPC_NRW_MAX: usize = 11;

/// Sentinel value for empty slots in [`LDPC_NM`]. 255 was chosen because
/// it's outside the valid index range [0, 127] for bit nodes.
pub const LDPC_NM_EMPTY: u8 = 255;

/// `LDPC_MN[i]` lists the indices of the 3 check nodes connected to bit `i`.
/// Equivalent to Fortran's `Mn(:, i+1)` minus 1.
pub const LDPC_MN: [[u8; LDPC_NCW]; LDPC_N] = [
    [ 20,  33,  35], [  0,   7,  27], [  1,   8,  36], [  2,   6,  18],
    [  3,  15,  31], [  1,   4,  21], [  5,  12,  24], [  9,  30,  32],
    [ 10,  23,  26], [ 11,  14,  22], [ 13,  17,  25], [ 16,  19,  28],
    [ 16,  29,  33], [  5,  33,  34], [  0,   9,  29], [  2,  17,  22],
    [  3,  11,  24], [  4,  27,  35], [  6,  13,  20], [  7,  14,  30],
    [  8,  26,  31], [ 10,  18,  34], [ 12,  15,  36], [ 19,  23,  37],
    [ 20,  21,  25], [ 11,  28,  32], [  0,  16,  34], [  1,  27,  29],
    [  2,   9,  31], [  3,   7,  35], [  4,  18,  28], [  5,  19,  26],
    [  6,  21,  36], [  8,  10,  32], [ 12,  23,  25], [ 13,  30,  33],
    [ 14,  15,  24], [ 12,  17,  37], [  7,  19,  22], [  0,  31,  32],
    [  1,  16,  18], [  2,  23,  33], [  3,   6,  37], [  4,  10,  30],
    [  5,  17,  20], [  8,  14,  35], [  9,  15,  27], [ 11,  25,  29],
    [ 13,  26,  28], [ 21,  24,  34], [ 22,  29,  31], [  3,  10,  36],
    [  0,  13,  22], [  1,   7,  24], [  2,  12,  26], [  4,   9,  36],
    [  5,  15,  30], [  6,  14,  17], [  8,  21,  23], [ 11,  18,  35],
    [ 16,  25,  37], [ 19,  20,  32], [ 19,  27,  34], [  3,  28,  33],
    [  0,  25,  35], [  1,  22,  33], [  2,   8,  37], [  4,   5,  16],
    [  6,  26,  34], [  7,  13,  31], [  9,  14,  21], [ 10,  17,  28],
    [ 11,  12,  27], [ 15,  18,  32], [ 20,  24,  30], [ 23,  29,  36],
    [  0,   2,  20], [  1,  17,  30], [  3,   5,   8], [  4,   7,  32],
    [  6,  28,  31], [  9,  12,  18], [ 10,  21,  22], [ 11,  26,  33],
    [ 13,  14,  29], [ 15,  26,  37], [ 16,  27,  36], [ 19,  24,  25],
    [  4,  23,  34], [  2,   5,  35], [  0,  11,  30], [  1,   3,  32],
    [  2,  15,  29], [  0,   1,  23], [  4,  22,  26], [  5,  27,  31],
    [  6,  16,  35], [  7,  21,  37], [  8,  17,  19], [  9,  20,  28],
    [ 10,  12,  33], [  3,  13,  19], [ 10,  29,  37], [ 13,  34,  36],
    [ 14,  18,  25], [  2,  27,  28], [  6,   7,   8], [  4,  17,  33],
    [ 12,  14,  16], [ 11,  15,  34], [  9,  22,  24], [ 18,  20,  36],
    [ 16,  26,  30], [ 23,  24,  35], [  0,  17,  18], [  5,  25,  32],
    [ 21,  30,  31], [  2,  19,  21], [  3,  20,  26], [  1,  12,  28],
    [  5,   6,  11], [ 14,  23,  31], [  8,  24,  29], [ 22,  36,  37],
    [  4,  15,  25], [ 10,  13,  27], [ 32,  35,  37], [  7,   9,  34],
];

/// `LDPC_NM[j][..LDPC_NRW[j]]` lists the indices of the bit nodes connected
/// to check `j`. Slots beyond `LDPC_NRW[j]` hold [`LDPC_NM_EMPTY`] (255).
/// Equivalent to Fortran's `Nm(1:nrw(j), j+1)` minus 1.
pub const LDPC_NM: [[u8; LDPC_NRW_MAX]; LDPC_M] = [
    [  1,  14,  26,  39,  52,  64,  76,  90,  93, 114, 255],
    [  2,   5,  27,  40,  53,  65,  77,  91,  93, 119, 255],
    [  3,  15,  28,  41,  54,  66,  76,  89,  92, 105, 117],
    [  4,  16,  29,  42,  51,  63,  78,  91, 101, 118, 255],
    [  5,  17,  30,  43,  55,  67,  79,  88,  94, 107, 124],
    [  6,  13,  31,  44,  56,  67,  78,  89,  95, 115, 120],
    [  3,  18,  32,  42,  57,  68,  80,  96, 106, 120, 255],
    [  1,  19,  29,  38,  53,  69,  79,  97, 106, 127, 255],
    [  2,  20,  33,  45,  58,  66,  78,  98, 106, 122, 255],
    [  7,  14,  28,  46,  55,  70,  81,  99, 110, 127, 255],
    [  8,  21,  33,  43,  51,  71,  82, 100, 102, 125, 255],
    [  9,  16,  25,  47,  59,  72,  83,  90, 109, 120, 255],
    [  6,  22,  34,  37,  54,  72,  81, 100, 108, 119, 255],
    [ 10,  18,  35,  48,  52,  69,  84, 101, 103, 125, 255],
    [  9,  19,  36,  45,  57,  70,  84, 104, 108, 121, 255],
    [  4,  22,  36,  46,  56,  73,  85,  92, 109, 124, 255],
    [ 11,  12,  26,  40,  60,  67,  86,  96, 108, 112, 255],
    [ 10,  15,  37,  44,  57,  71,  77,  98, 107, 114, 255],
    [  3,  21,  30,  40,  59,  73,  81, 104, 111, 114, 255],
    [ 11,  23,  31,  38,  61,  62,  87,  98, 101, 117, 255],
    [  0,  18,  24,  44,  61,  74,  76,  99, 111, 118, 255],
    [  5,  24,  32,  49,  58,  70,  82,  97, 116, 117, 255],
    [  9,  15,  38,  50,  52,  65,  82,  94, 110, 123, 255],
    [  8,  23,  34,  41,  58,  75,  88,  93, 113, 121, 255],
    [  6,  16,  36,  49,  53,  74,  87, 110, 113, 122, 255],
    [ 10,  24,  34,  47,  60,  64,  87, 104, 115, 124, 255],
    [  8,  20,  31,  48,  54,  68,  83,  85,  94, 112, 118],
    [  1,  17,  27,  46,  62,  72,  86,  95, 105, 125, 255],
    [ 11,  25,  30,  48,  63,  71,  80,  99, 105, 119, 255],
    [ 12,  14,  27,  47,  50,  75,  84,  92, 102, 122, 255],
    [  7,  19,  35,  43,  56,  74,  77,  90, 112, 116, 255],
    [  4,  20,  28,  39,  50,  69,  80,  95, 116, 121, 255],
    [  7,  25,  33,  39,  61,  73,  79,  91, 115, 126, 255],
    [  0,  12,  13,  35,  41,  63,  65,  83, 100, 107, 255],
    [ 13,  21,  26,  49,  62,  68,  88, 103, 109, 127, 255],
    [  0,  17,  29,  45,  59,  64,  89,  96, 113, 126, 255],
    [  2,  22,  32,  51,  55,  75,  86, 103, 111, 123, 255],
    [ 23,  37,  42,  60,  66,  85,  97, 102, 123, 126, 255],
];

/// Actual row weight for each check node (10 or 11).
pub const LDPC_NRW: [u8; LDPC_M] = [
    10, 10, 11, 10, 11, 11, 10, 10, 10, 10, 10, 10,
    10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
    10, 10, 11, 10, 10, 10, 10, 10, 10, 10, 10, 10,
    10, 10,
];

#[cfg(test)]
mod tests {
    use super::*;

    /// Verify that the MN/NM tables are mutually consistent: if check j is
    /// in MN[i], then i must be in NM[j], and vice versa.
    #[test]
    fn mn_nm_consistency() {
        // Forward direction: for each bit i, each check in MN[i] should
        // contain i in its NM row.
        for i in 0..LDPC_N {
            for &chk in &LDPC_MN[i] {
                let chk = chk as usize;
                let row = &LDPC_NM[chk][..LDPC_NRW[chk] as usize];
                assert!(
                    row.contains(&(i as u8)),
                    "bit {} claims to be in check {}, but check {} doesn't list bit {}",
                    i, chk, chk, i
                );
            }
        }
        // Reverse direction: for each check j, each bit in NM[j][..nrw[j]]
        // should have j in its MN row.
        for j in 0..LDPC_M {
            let nrw = LDPC_NRW[j] as usize;
            for &b in &LDPC_NM[j][..nrw] {
                let b = b as usize;
                assert!(
                    LDPC_MN[b].contains(&(j as u8)),
                    "check {} lists bit {}, but bit {} doesn't list check {}",
                    j, b, b, j
                );
            }
            // Slots beyond nrw should be the empty sentinel.
            for &b in &LDPC_NM[j][nrw..] {
                assert_eq!(b, LDPC_NM_EMPTY, "check {} has non-sentinel beyond nrw", j);
            }
        }
    }

    #[test]
    fn nrw_total_matches_mn_total() {
        // Each MN entry contributes one connection; each NM[..nrw] entry
        // also contributes one. They must be equal.
        let mn_count = LDPC_N * LDPC_NCW;
        let nm_count: usize = LDPC_NRW.iter().map(|&w| w as usize).sum();
        assert_eq!(mn_count, nm_count, "MN and NM disagree on edge count");
    }
}
