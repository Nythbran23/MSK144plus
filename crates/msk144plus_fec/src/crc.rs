// crates/msk144plus_fec/src/crc.rs
//
// CRC-13 used by the MSK144 (128,90) LDPC code.
//
// Polynomial: 0x15D7 (x^13 + x^11 + x^9 + x^8 + x^6 + x^4 + x^2 + x + 1)
// Matches WSJT-X lib/crc13.cpp exactly via boost::augmented_crc<13, 0x15D7>.
//
// CRITICAL: WSJT-X's chkcrc13a doesn't compute the CRC bit-by-bit on the
// 90-bit codeword. Instead it:
//   1. Packs 90 bits into 12 bytes MSB-first (bits 90..96 are zero).
//   2. Masks byte 9 with 0xF8 (zeroing bits 77..79).
//   3. Zeros bytes 10 and 11 (zeroing bits 80..95).
//   4. Computes augmented_crc<13, 0x15D7> over those 12 bytes (96 bits).
//   5. Compares against bits 77..90 read as a 13-bit big-endian number.
//
// Our previous bit-serial 90-bit implementation produced the WRONG residue
// because the boost augmented_crc keeps shifting through all 96 bits,
// including the 6 trailing zeros that put the augmented register into a
// different state than stopping at bit 89.

const POLY: u16 = 0x15D7;

/// Validate a 90-bit codeword (77 message bits followed by 13 CRC bits)
/// using the same byte-padded CRC convention as WSJT-X's `chkcrc13a`.
/// Returns true iff the CRC matches.
pub fn crc13_check(bits: &[u8]) -> bool {
    debug_assert_eq!(bits.len(), 90, "crc13_check expects exactly 90 bits");
    debug_assert!(bits.iter().all(|&b| b <= 1));

    // Pack 90 bits into 12 bytes MSB-first; bits 90..96 stay zero.
    let mut bytes = [0u8; 12];
    for i in 0..90 {
        if bits[i] == 1 {
            bytes[i / 8] |= 1 << (7 - (i % 8));
        }
    }
    // Read received CRC (bits 77..90)
    let mut received_crc: u16 = 0;
    for i in 77..90 {
        received_crc = (received_crc << 1) | (bits[i] as u16);
    }
    // Mask byte 9 with 0xF8 (zero bits 77..79) and zero bytes 10,11.
    bytes[9] &= 0xF8;
    bytes[10] = 0;
    bytes[11] = 0;
    let computed_crc = boost_augmented_crc13(&bytes);
    received_crc == computed_crc
}

/// Compute the 13-bit CRC value to append to a 77-bit message to form a
/// valid 90-bit codeword. Returns the CRC value (low 13 bits).
pub fn crc13_compute(message_bits: &[u8]) -> u16 {
    debug_assert_eq!(message_bits.len(), 77, "CRC-13 message must be 77 bits");
    debug_assert!(message_bits.iter().all(|&b| b <= 1));

    // Pack 77 message bits into the high bits of 12 bytes, leaving 19 bits
    // of trailing zeros (the 13 CRC slot + 6 byte-padding bits).
    let mut bytes = [0u8; 12];
    for i in 0..77 {
        if message_bits[i] == 1 {
            bytes[i / 8] |= 1 << (7 - (i % 8));
        }
    }
    boost_augmented_crc13(&bytes)
}

/// Mirror of `boost::augmented_crc<13, 0x15D7>` from boost/crc.hpp.
///
/// Processes the input byte-by-byte, MSB-first, treating the byte stream
/// as a polynomial in x and dividing by g(x) = x^13 + ... + 1 = 0x15D7
/// (with implicit x^13 leading bit). Returns the 13-bit remainder.
///
/// "Augmented" means the algorithm doesn't pre-shift the input by 13 bits;
/// the caller must ensure the data already has trailing zero bits (or the
/// CRC bits) where the divisor will reach.
fn boost_augmented_crc13(data: &[u8]) -> u16 {
    let mut reg: u16 = 0;
    for &byte in data {
        for bit_idx in (0..8).rev() {
            let bit = (byte >> bit_idx) & 1;
            reg = (reg << 1) | (bit as u16);
            if reg & 0x2000 != 0 {
                reg ^= 0x2000 | POLY;
            }
        }
    }
    reg & 0x1FFF
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rand_bits(seed: u32, n: usize) -> Vec<u8> {
        let mut x = seed;
        (0..n).map(|_| {
            x = x.wrapping_mul(1103515245).wrapping_add(12345);
            ((x >> 16) & 1) as u8
        }).collect()
    }

    #[test]
    fn crc_appended_validates() {
        // Generate random 77-bit messages, append CRC, verify check passes.
        for seed in [0x1234u32, 0xCAFE, 0xDEAD, 0xF00D, 0xBEEF] {
            let msg = rand_bits(seed, 77);
            let crc = crc13_compute(&msg);
            let mut codeword = msg.clone();
            for i in (0..13).rev() {
                codeword.push(((crc >> i) & 1) as u8);
            }
            assert_eq!(codeword.len(), 90);
            assert!(crc13_check(&codeword), "valid codeword failed CRC check (seed {:#x})", seed);
        }
    }

    #[test]
    fn crc_detects_single_bit_errors() {
        // Flipping any one bit in a valid codeword must invalidate the CRC.
        let msg = rand_bits(0xABCD, 77);
        let crc = crc13_compute(&msg);
        let mut codeword = msg.clone();
        for i in (0..13).rev() {
            codeword.push(((crc >> i) & 1) as u8);
        }
        for flip_pos in 0..90 {
            let mut corrupted = codeword.clone();
            corrupted[flip_pos] ^= 1;
            assert!(
                !crc13_check(&corrupted),
                "single-bit flip at position {} not detected",
                flip_pos
            );
        }
    }

    #[test]
    fn crc_zero_message() {
        // CRC of all zeros is all zeros (the remainder of 0 by anything is 0).
        let zeros = vec![0u8; 77];
        let crc = crc13_compute(&zeros);
        assert_eq!(crc, 0);
        let codeword = vec![0u8; 90];
        assert!(crc13_check(&codeword));
    }
}
