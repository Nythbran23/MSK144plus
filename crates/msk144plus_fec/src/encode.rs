// crates/msk144plus_fec/src/encode.rs
//
// (128,90) LDPC encoder for MSK144 TX. Faithful port of WSJT-X's
// encode_128_90.f90, producing bit-exact codewords for the same
// 77-bit message.
//
// Pipeline:
//   1. Append 13-bit CRC to the 77 message bits, producing 90 bits.
//   2. Multiply the 90-bit message vector against the (38×90) generator
//      matrix G to produce 38 parity bits.
//   3. Concatenate [message_90, parity_38] = 128-bit codeword.

use crate::crc::crc13_compute;

/// Hex generator matrix from WSJT-X's lib/ldpc_128_90_generator.f90.
/// Each string is 23 hex characters representing one row of G. The first
/// 22 chars give 4 bits each (88 columns); the last char gives only the
/// upper 2 bits (90 columns total). Columns are indexed MSB-first, so
/// the leftmost bit of the row maps to message[0].
const G_HEX: [&str; 38] = [
    "a08ea80879050a5e94da994",
    "59f3b48040ca089c81ee880",
    "e4070262802e31b7b17d3dc",
    "95cbcbaf032dc3d960bacc8",
    "c4d79b5dcc21161a254ffbc",
    "93fde9cdbf2622a70868424",
    "e73b888bb1b01167379ba28",
    "45a0d0a0f39a7ad2439949c",
    "759acef19444bcad79c4964",
    "71eb4dddf4f5ed9e2ea17e0",
    "80f0ad76fb247d6b4ca8d38",
    "184fff3aa1b82dc66640104",
    "ca4e320bb382ed14cbb1094",
    "52514447b90e25b9e459e28",
    "dd10c1666e071956bd0df38",
    "99c332a0b792a2da8ef1ba8",
    "7bd9f688e7ed402e231aaac",
    "00fcad76eb647d6a0ca8c38",
    "6ac8d0499c43b02eed78d70",
    "2c2c764baf795b4788db010",
    "0e907bf9e280d2624823dd0",
    "b857a6e315afd8c1c925e64",
    "8deb58e22d73a141cae3778",
    "22d3cb80d92d6ac132dfe08",
    "754763877b28c187746855c",
    "1d1bb7cf6953732e04ebca4",
    "2c65e0ea4466ab9f5e1deec",
    "6dc530ca37fc916d1f84870",
    "49bccbbee152355be7ac984",
    "e8387f3f4367cf45a150448",
    "8ce25e03d67d51091c81884",
    "b798012ffa40a93852752c8",
    "2e43307933adfca37adc3c8",
    "ca06e0a42ca1ec782d6c06c",
    "c02b762927556a7039e638c",
    "4a3e9b7d08b6807f8619fac",
    "45e8030f68997bb68544424",
    "7e79362c16773efc6482e30",
];

/// Decoded form of the generator matrix: 38 rows × 90 columns of bits.
/// Built once via OnceLock.
fn generator() -> &'static [[u8; 90]; 38] {
    use std::sync::OnceLock;
    static GEN: OnceLock<[[u8; 90]; 38]> = OnceLock::new();
    GEN.get_or_init(|| {
        let mut g = [[0u8; 90]; 38];
        for (row_idx, hex_row) in G_HEX.iter().enumerate() {
            let bytes = hex_row.as_bytes();
            // First 22 hex chars give 4 bits each (88 cols)
            for j in 0..22 {
                let nibble = hex_to_nibble(bytes[j]);
                for jj in 0..4 {
                    let col = j * 4 + jj;
                    // Match Fortran's btest(istr, 4-jj) with jj in 1..=4
                    // -> jj_zero in 0..4 -> bit position (3-jj_zero)
                    let bit = (nibble >> (3 - jj)) & 1;
                    g[row_idx][col] = bit;
                }
            }
            // Last hex char gives 2 bits (cols 88, 89). Fortran has
            // ibmax=2 and the loop is `do jj=1, 2`, with `btest(istr,
            // 4-jj)` -> bit 3 at jj=1, bit 2 at jj=2.
            let nibble = hex_to_nibble(bytes[22]);
            g[row_idx][88] = (nibble >> 3) & 1;
            g[row_idx][89] = (nibble >> 2) & 1;
        }
        g
    })
}

#[inline]
fn hex_to_nibble(c: u8) -> u8 {
    match c {
        b'0'..=b'9' => c - b'0',
        b'a'..=b'f' => c - b'a' + 10,
        b'A'..=b'F' => c - b'A' + 10,
        _ => 0,
    }
}

/// Encode a 77-bit message into a 128-bit codeword.
///
/// Process:
///   1. Compute 13-bit CRC over message[0..77] padded to 80 bits with
///      three trailing zeros (matches WSJT-X's `tmpchar(78:80)='000'`).
///   2. Append CRC bits to the message, producing a 90-bit message vector.
///   3. For each of the 38 generator-matrix rows, compute the modulo-2
///      sum of (message AND row) - that's a parity bit.
///   4. Codeword = [message_90, parity_38].
pub fn encode_128_90(message_77: &[u8; 77]) -> [u8; 128] {
    debug_assert!(message_77.iter().all(|&b| b <= 1));

    // Build 90-bit message: 77 message bits + 13-bit CRC
    let mut message_90 = [0u8; 90];
    message_90[..77].copy_from_slice(message_77);
    // Note: crc13_compute internally pads to whatever its API expects.
    // We pass the 77 bits and get back the 13-bit CRC.
    let crc = crc13_compute(message_77);
    // CRC bits MSB-first into positions 77..90
    for i in 0..13 {
        message_90[77 + i] = ((crc >> (12 - i)) & 1) as u8;
    }

    // Compute 38 parity bits via G * message (mod 2)
    let g = generator();
    let mut codeword = [0u8; 128];
    codeword[..90].copy_from_slice(&message_90);
    for i in 0..38 {
        let mut sum = 0u32;
        for j in 0..90 {
            sum += (message_90[j] * g[i][j]) as u32;
        }
        codeword[90 + i] = (sum % 2) as u8;
    }

    codeword
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crc::crc13_check;

    #[test]
    fn generator_dimensions() {
        let g = generator();
        assert_eq!(g.len(), 38);
        for row in g.iter() {
            assert_eq!(row.len(), 90);
            // Each entry is 0 or 1
            for &b in row.iter() {
                assert!(b <= 1);
            }
        }
    }

    #[test]
    fn encode_zero_message_is_zero() {
        let msg = [0u8; 77];
        let cw = encode_128_90(&msg);
        // First 77 bits are zero
        for i in 0..77 { assert_eq!(cw[i], 0); }
        // Bits 77..90 are the CRC of all zeros - which may not be zero,
        // but we can compute it independently:
        let crc = crc13_compute(&msg);
        for i in 0..13 {
            assert_eq!(cw[77 + i], ((crc >> (12 - i)) & 1) as u8);
        }
    }

    #[test]
    fn encoded_passes_crc() {
        let msg = [
            // Some bit pattern in 77 bits
            1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1,
            0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1,
            1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1,
            0, 1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1,
            0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1,
        ];
        let cw = encode_128_90(&msg);
        // First 90 bits should pass CRC check
        let mut first_90 = [0u8; 90];
        first_90.copy_from_slice(&cw[..90]);
        assert!(crc13_check(&first_90));
    }

    #[test]
    fn encoded_round_trips_through_bp() {
        // The strongest internal-consistency test: encode a message, feed
        // the codeword as high-confidence LLRs into our BP decoder, and
        // verify it recovers the original 77 message bits.
        //
        // This tests: (a) the generator matrix is correct, (b) the parity
        // matrix is correct, (c) the CRC computation aligns with what the
        // decoder expects, (d) the column ordering of G and H are
        // mutually consistent.
        use crate::ldpc::decode_128_90_soft;
        let msg = [
            1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1,
            0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1,
            1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1,
            0, 1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1,
            0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1,
        ];
        let cw = encode_128_90(&msg);

        // Convert to high-confidence LLRs (sign convention: positive = 1)
        let mut llr = [0.0f32; 128];
        for i in 0..128 {
            llr[i] = if cw[i] == 1 { 5.0 } else { -5.0 };
        }
        let r = decode_128_90_soft(&llr, 30).expect("BP must decode our own codeword");
        // First 77 bits of result should equal the input message
        assert_eq!(&r.message[..], &msg[..],
            "BP recovered different message - encoder/decoder column-order mismatch");
    }
}
