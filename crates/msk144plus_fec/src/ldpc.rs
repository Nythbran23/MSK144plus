// crates/msk144plus_fec/src/ldpc.rs
//
// (128,90) LDPC belief-propagation decoder, ported from WSJT-X's
// lib/bpdecode128_90.f90.
//
// Structure:
//   - 128-bit codeword = 77 message bits + 13 CRC bits + 38 parity bits
//   - Bit positions 0..77 = message
//   - Bit positions 77..90 = CRC-13 (validated separately)
//   - Bit positions 90..128 = parity (only used by the BP decoder)
//
// BP message-passing algorithm:
//   tov[bit][k]   message from check_k(=MN[bit][k]) to bit
//   toc[chk][i]   message from bit_i(=NM[chk][i]) to chk
//
//   Iteration:
//     1. zn[bit] = llr[bit] + sum(tov[bit][k] for k in 0..3)
//     2. Hard-slice cw[bit] = sign(zn[bit])
//     3. If all parity checks satisfied AND CRC-13 validates, return success
//     4. Bit -> check: toc[chk][i] = zn[ib] - tov[ib][k_of_chk_in_MN[ib]]
//        where ib = NM[chk][i]
//     5. Check -> bit: tov[bit][k] = 2 * atanh(- product over k' != "this bit"
//        of tanh(-toc[chk][k']/2))
//        where chk = MN[bit][k]
//
// Convergence criteria (matching WSJT-X):
//   - max 10 iterations by default
//   - early stop if unsatisfied-parity count fails to decrease for 3
//     consecutive iterations after iteration 5, with > 10 checks still bad
//   - CRC-13 validated on the 90-bit hard-decoded prefix; failure means we
//     converged to a wrong codeword (or didn't converge)

use crate::crc::crc13_check;
use crate::parity::{
    LDPC_K, LDPC_M, LDPC_MN, LDPC_N, LDPC_NCW, LDPC_NM, LDPC_NRW, LDPC_NRW_MAX,
};

/// Result of a successful BP decode.
#[derive(Debug, Clone, Copy)]
pub struct DecodeResult {
    /// 77-bit message payload (each byte is 0 or 1).
    pub message: [u8; 77],
    /// Hamming distance between the hard-decoded codeword and the soft-input
    /// signs - a "soft confidence" metric. Lower = stronger decode.
    pub n_hard_errors: u32,
    /// BP iterations consumed (0 = decoded on the initial hard-slice without
    /// any iteration).
    pub iterations: u32,
}

/// Decode a (128,90) LDPC soft codeword. Returns `Some(result)` if BP
/// converges to a CRC-13-valid codeword, `None` on failure.
///
/// `llr` convention: positive value = bit 1, negative = bit 0, magnitude
/// proportional to confidence (e.g., 2/sigma^2 scaling - whatever upstream
/// produces). This matches both WSJT-X's `bpdecode128_90` input format and
/// the LLR layout produced by [`crate::dsp::accumulator::AccumulatedFrame`].
///
/// `max_iterations`: BP iteration cap. WSJT-X uses 10. Set higher for slower
/// but slightly more reliable decoding, lower for tighter latency.
pub fn decode_128_90_soft(
    llr: &[f32; LDPC_N],
    max_iterations: u32,
) -> Option<DecodeResult> {
    // tov[bit][k] = message from check MN[bit][k] to bit
    let mut tov = [[0.0f32; LDPC_NCW]; LDPC_N];
    // toc[chk][i] = message from bit NM[chk][i] to chk
    let mut toc = [[0.0f32; LDPC_NRW_MAX]; LDPC_M];
    let mut tanhtoc = [[0.0f32; LDPC_NRW_MAX]; LDPC_M];
    let mut zn = [0.0f32; LDPC_N];
    let mut cw = [0u8; LDPC_N];

    // Initialise bit-to-check messages with the channel LLRs.
    for j in 0..LDPC_M {
        let nrw = LDPC_NRW[j] as usize;
        for i in 0..nrw {
            toc[j][i] = llr[LDPC_NM[j][i] as usize];
        }
    }

    let mut ncnt = 0u32;
    let mut nclast = u32::MAX; // first iteration always counts as "decreased"

    for iter in 0..=max_iterations {
        // 1. Aggregate posterior LLRs at each bit.
        for i in 0..LDPC_N {
            zn[i] = llr[i] + tov[i][0] + tov[i][1] + tov[i][2];
        }

        // 2. Hard-slice.
        for i in 0..LDPC_N {
            cw[i] = (zn[i] > 0.0) as u8;
        }

        // 3. Parity check.
        let mut ncheck = 0u32;
        for j in 0..LDPC_M {
            let nrw = LDPC_NRW[j] as usize;
            let mut s = 0u32;
            for k in 0..nrw {
                s += cw[LDPC_NM[j][k] as usize] as u32;
            }
            if s % 2 != 0 {
                ncheck += 1;
            }
        }

        if ncheck == 0 {
            // All checks satisfied. Validate the embedded CRC on the
            // first 90 bits before declaring success - BP can converge to
            // a non-message codeword whose parity is satisfied but whose
            // CRC isn't.
            if crc13_check(&cw[0..LDPC_K]) {
                let mut message = [0u8; 77];
                message.copy_from_slice(&cw[0..77]);
                let n_hard_errors = (0..LDPC_N)
                    .filter(|&i| {
                        let h = if cw[i] == 1 { 1.0 } else { -1.0 };
                        h * llr[i] < 0.0
                    })
                    .count() as u32;
                return Some(DecodeResult {
                    message,
                    n_hard_errors,
                    iterations: iter,
                });
            }
            // Parity satisfied but CRC failed. Continue iterating - rare
            // but BP can sometimes step out of a non-message codeword into
            // the right one with more iterations.
        }

        // 4. Early-stopping. Match WSJT-X's heuristic: if we've gone 3
        //    iterations without ncheck improving and we're past iteration
        //    5 with still > 10 unsatisfied checks, give up.
        if iter > 0 {
            if ncheck < nclast {
                ncnt = 0;
            } else {
                ncnt += 1;
            }
            if ncnt >= 3 && iter >= 5 && ncheck > 10 {
                return None;
            }
        }
        nclast = ncheck;

        // 5. Bit-to-check messages: subtract from zn[ib] the message from
        //    THIS check, leaving the extrinsic-only contribution.
        for j in 0..LDPC_M {
            let nrw = LDPC_NRW[j] as usize;
            for i in 0..nrw {
                let ib = LDPC_NM[j][i] as usize;
                let mut t = zn[ib];
                // Find which slot in MN[ib] holds check j and subtract that
                // tov contribution. Linear search of 3 entries.
                for kk in 0..LDPC_NCW {
                    if LDPC_MN[ib][kk] as usize == j {
                        t -= tov[ib][kk];
                        break;
                    }
                }
                toc[j][i] = t;
            }
        }

        // 6. Pre-compute tanh(-toc[j][i]/2) for the check-to-bit step.
        for j in 0..LDPC_M {
            let nrw = LDPC_NRW[j] as usize;
            for i in 0..nrw {
                tanhtoc[j][i] = (-toc[j][i] * 0.5).tanh();
            }
        }

        // 7. Check-to-bit messages: 2 * atanh(- product over connected bits
        //    other than this one of tanh(-toc/2)).
        for j in 0..LDPC_N {
            for i in 0..LDPC_NCW {
                let ichk = LDPC_MN[j][i] as usize;
                let nrw = LDPC_NRW[ichk] as usize;
                let mut prod = 1.0f32;
                for k in 0..nrw {
                    if LDPC_NM[ichk][k] as usize != j {
                        prod *= tanhtoc[ichk][k];
                    }
                }
                tov[j][i] = 2.0 * atanh_safe(-prod);
            }
        }
    }

    None
}

/// Numerically safe atanh. The product of tanh values fed in is in (-1, 1)
/// strictly, but rounding can push it to ±1 exactly, where atanh diverges.
/// Clamp before calling.
#[inline]
fn atanh_safe(x: f32) -> f32 {
    // Just inside ±1 to keep the result finite. tanh saturates very fast
    // so values beyond ~0.9999 are effectively ±1 anyway and a hard clamp
    // doesn't change the BP trajectory meaningfully.
    let xc = x.clamp(-0.999_999_9, 0.999_999_9);
    xc.atanh()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: build an LLR vector representing a hard codeword with given
    /// magnitude (positive = bit 1).
    fn hard_to_llr(bits: &[u8; LDPC_N], magnitude: f32) -> [f32; LDPC_N] {
        let mut llr = [0.0f32; LDPC_N];
        for i in 0..LDPC_N {
            llr[i] = if bits[i] == 1 { magnitude } else { -magnitude };
        }
        llr
    }

    #[test]
    fn decodes_all_zero_codeword() {
        // The all-zero message has CRC = 0, parity = 0, codeword = all zeros.
        // BP should converge instantly (iteration 0).
        let cw = [0u8; LDPC_N];
        let llr = hard_to_llr(&cw, 5.0);
        let r = decode_128_90_soft(&llr, 10).expect("should decode");
        assert_eq!(r.message, [0u8; 77]);
        assert_eq!(r.iterations, 0);
        assert_eq!(r.n_hard_errors, 0);
    }

    #[test]
    fn decodes_all_zero_with_one_bit_error() {
        // Flip one bit in the LLR (high confidence in wrong direction).
        // BP should still correct because the (128,90) code can handle
        // small numbers of errors.
        let cw = [0u8; LDPC_N];
        for flip_pos in [0usize, 50, 89, 100, 127] {
            let mut llr = hard_to_llr(&cw, 5.0);
            llr[flip_pos] = 5.0; // wrong sign, full magnitude
            let r = decode_128_90_soft(&llr, 10);
            assert!(
                r.is_some(),
                "single-bit error at position {} should be correctable",
                flip_pos
            );
            let r = r.unwrap();
            assert_eq!(r.message, [0u8; 77]);
            assert_eq!(r.n_hard_errors, 1);
        }
    }

    #[test]
    fn decodes_all_zero_with_two_bit_errors() {
        let cw = [0u8; LDPC_N];
        let mut llr = hard_to_llr(&cw, 5.0);
        llr[10] = 5.0;
        llr[80] = 5.0;
        let r = decode_128_90_soft(&llr, 10).expect("two errors should be correctable");
        assert_eq!(r.message, [0u8; 77]);
    }

    #[test]
    fn rejects_pure_noise() {
        // Random LLRs with small magnitude - no real codeword.
        let mut x: u32 = 0xDEADBEEF;
        let mut llr = [0.0f32; LDPC_N];
        for v in llr.iter_mut() {
            x = x.wrapping_mul(1103515245).wrapping_add(12345);
            let r = ((x >> 16) & 0x7fff) as f32 / 16384.0 - 1.0;
            *v = r * 0.5;
        }
        let r = decode_128_90_soft(&llr, 10);
        // BP should fail to find a CRC-valid codeword in pure noise.
        // Occasionally noise might happen to match, but with magnitude 0.5
        // it's overwhelmingly unlikely.
        assert!(r.is_none(), "decoded a result from pure noise (got {:?})", r);
    }

    #[test]
    fn weak_llr_does_not_panic() {
        // Verify BP doesn't blow up numerically with very small input.
        let llr = [0.001f32; LDPC_N];
        let _ = decode_128_90_soft(&llr, 10);
        // Also try LLRs that would push tanh near ±1
        let llr = [50.0f32; LDPC_N];
        let _ = decode_128_90_soft(&llr, 10);
        let llr = [-50.0f32; LDPC_N];
        let _ = decode_128_90_soft(&llr, 10);
    }

    #[test]
    fn iteration_cap_respected() {
        // With max_iterations=0, BP runs only the iter-0 hard-slice check.
        // For all-zeros input that's enough.
        let cw = [0u8; LDPC_N];
        let llr = hard_to_llr(&cw, 5.0);
        let r = decode_128_90_soft(&llr, 0).expect("iter-0 should suffice for clean input");
        assert_eq!(r.iterations, 0);
    }
}
