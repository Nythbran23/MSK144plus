// crates/msk144plus_packjt/src/free_text.rs
//
// i3=0, n3=0 free-text message unpacker.
// 71 bits encode a 13-character string drawn from a 42-character alphabet:
//   ' 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ+-./?'
// (space, digits, uppercase letters, plus, minus, dot, slash, question mark)
//
// Encoding: treat the 13-char string as a base-42 number with the leftmost
// (oldest) character as the most-significant digit. The resulting integer
// fits in 71 bits because 42^13 = 4.69e21 < 2^71 = 2.36e21... wait, that's
// 42^13 > 2^71. Looking again: 42^13 ≈ 1.06e21 = ~2^69.9, so it fits with
// margin in 71 bits.
//
// Decoding: load the 71-bit integer, then 13 iterations of divmod-42
// produce the characters from rightmost (least significant) backwards.
//
// Faithful port of unpacktext77 in lib/77bit/packjt77.f90 lines 1511-1532.

/// The 42-character alphabet used by free-text messages.
const FREE_TEXT_ALPHABET: &[u8; 42] = b" 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ+-./?";

/// Unpack 71 bits into a 13-character free-text string.
///
/// `bits[0..71]` should hold the 71 message bits in MSB-first order
/// (matching how packjt77 reads payload bits). Returns the decoded
/// 13-character string with leading spaces preserved.
pub fn unpack_free_text(bits: &[u8]) -> String {
    debug_assert!(bits.len() >= 71);

    // Load 71 bits into a u128 (more than enough headroom).
    // We need MSB-first interpretation: bits[0] is the highest-order bit.
    let mut value: u128 = 0;
    for i in 0..71 {
        value = (value << 1) | (bits[i] as u128);
    }

    // Pull 13 base-42 digits, starting from the least significant
    // (rightmost char) and working backwards.
    let mut chars = [b' '; 13];
    for i in (0..13).rev() {
        let digit = (value % 42) as usize;
        chars[i] = FREE_TEXT_ALPHABET[digit];
        value /= 42;
    }

    // Convert to String. The alphabet is pure ASCII so this is safe.
    String::from_utf8_lossy(&chars).into_owned()
}

/// Encode a 13-character string into 71 bits (used for round-trip testing).
/// Characters not in the alphabet are mapped to space (index 0).
pub fn pack_free_text(text: &str) -> [u8; 71] {
    // Right-justify to 13 chars (left-pad with spaces) like Fortran's adjustr
    let mut padded = [b' '; 13];
    let bytes: Vec<u8> = text.bytes().collect();
    let n = bytes.len().min(13);
    let start = 13 - n;
    for i in 0..n {
        let c = bytes[i].to_ascii_uppercase();
        padded[start + i] = c;
    }

    // Build base-42 integer
    let mut value: u128 = 0;
    for &c in padded.iter() {
        let digit = FREE_TEXT_ALPHABET.iter().position(|&a| a == c).unwrap_or(0);
        value = value * 42 + digit as u128;
    }

    // Serialize as 71 MSB-first bits
    let mut bits = [0u8; 71];
    for i in 0..71 {
        bits[i] = ((value >> (70 - i)) & 1) as u8;
    }
    bits
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_basic() {
        let cases = [
            "TNX BOB 73 GL",  // From WSJT-X free_text.f90 line 7
            "HELLO WORLD",
            "CQ DX 73",
            "12345",
            "",
        ];
        for &text in &cases {
            let bits = pack_free_text(text);
            let decoded = unpack_free_text(&bits);
            // Decoded is right-justified (13 chars, padded with leading spaces)
            let expected: String = format!("{:>13}", text.to_ascii_uppercase());
            assert_eq!(decoded, expected, "round-trip failed for '{}'", text);
        }
    }

    #[test]
    fn unpack_known_pattern() {
        // All-zero bits should decode to 13 spaces
        let bits = [0u8; 71];
        assert_eq!(unpack_free_text(&bits), "             ");
    }

    #[test]
    fn alphabet_is_42_chars() {
        assert_eq!(FREE_TEXT_ALPHABET.len(), 42);
    }

    #[test]
    fn last_char_alone() {
        // Set value = 1 (last bit) -> last char is '0' (alphabet[1])
        let mut bits = [0u8; 71];
        bits[70] = 1;
        let s = unpack_free_text(&bits);
        assert_eq!(&s[12..], "0");
        assert_eq!(&s[..12], "            ");
    }
}
