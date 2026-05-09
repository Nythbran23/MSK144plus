// crates/msk144plus_packjt/src/bits.rs
//
// Bit-field utilities for the 77-bit packjt77 payload. All multi-bit fields
// are big-endian: bit 0 of a 28-bit field is the most-significant bit.
//
// Input format: the LDPC decoder produces a 77-bit message as a slice of
// u8 where each byte is 0 or 1. This module reads big-endian integers from
// such slices and (for tests) writes them back.

/// Read a big-endian unsigned integer from `bits[start..start+width]`.
/// Each byte in `bits` must be 0 or 1; debug builds panic if not.
pub fn read_be(bits: &[u8], start: usize, width: usize) -> u64 {
    debug_assert!(start + width <= bits.len(), "read_be out of bounds");
    debug_assert!(width <= 64, "read_be width must be <= 64");
    let mut acc: u64 = 0;
    for i in 0..width {
        debug_assert!(bits[start + i] <= 1, "bit must be 0 or 1");
        acc = (acc << 1) | (bits[start + i] as u64 & 1);
    }
    acc
}

/// Write a big-endian unsigned integer into `bits[start..start+width]`.
/// Used by encoders and round-trip tests.
pub fn write_be(bits: &mut [u8], start: usize, width: usize, value: u64) {
    debug_assert!(start + width <= bits.len(), "write_be out of bounds");
    debug_assert!(width <= 64, "write_be width must be <= 64");
    for i in 0..width {
        let shift = width - 1 - i;
        bits[start + i] = ((value >> shift) & 1) as u8;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_small() {
        let mut bits = [0u8; 16];
        write_be(&mut bits, 0, 8, 0xA5);
        assert_eq!(read_be(&bits, 0, 8), 0xA5);
        // Verify big-endian layout: 0xA5 = 1010 0101
        assert_eq!(bits[..8], [1, 0, 1, 0, 0, 1, 0, 1]);
    }

    #[test]
    fn round_trip_28_bit() {
        let mut bits = [0u8; 28];
        write_be(&mut bits, 0, 28, 10_214_965);
        assert_eq!(read_be(&bits, 0, 28), 10_214_965);
    }

    #[test]
    fn round_trip_packed_fields() {
        // Pack three fields into a 77-bit array and read them back
        let mut bits = [0u8; 77];
        write_be(&mut bits, 0, 28, 10_214_965);  // call 1
        write_be(&mut bits, 28, 1, 0);           // ipa
        write_be(&mut bits, 29, 28, 12_751_800); // call 2
        write_be(&mut bits, 57, 1, 0);           // ipb
        write_be(&mut bits, 58, 1, 0);           // r
        write_be(&mut bits, 59, 15, 10_342);     // grid
        write_be(&mut bits, 74, 3, 1);           // i3

        assert_eq!(read_be(&bits, 0, 28),  10_214_965);
        assert_eq!(read_be(&bits, 28, 1),  0);
        assert_eq!(read_be(&bits, 29, 28), 12_751_800);
        assert_eq!(read_be(&bits, 57, 1),  0);
        assert_eq!(read_be(&bits, 58, 1),  0);
        assert_eq!(read_be(&bits, 59, 15), 10_342);
        assert_eq!(read_be(&bits, 74, 3),  1);
    }
}
