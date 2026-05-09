// crates/msk144plus_packjt/src/grid.rs
//
// 15-bit grid/exchange field decoder for i3=1/2 (standard) messages.
// Ported from packjt77.f90's to_grid4 and the irpt branches in the i3=1/2
// dispatcher.
//
// Encoding:
//   igrid4 in 0..MAXGRID4 (32400) -> 4-character Maidenhead grid
//                                     j1*1800 + j2*100 + j3*10 + j4
//                                     where j1,j2 in 0..18 (A-R)
//                                     and   j3,j4 in 0..10 (0-9)
//   igrid4 in MAXGRID4.. -> exchange code
//     irpt = igrid4 - MAXGRID4
//       1 -> bare (no extra suffix)
//       2 -> RRR
//       3 -> RR73
//       4 -> 73
//       5..  -> SNR with isnr = irpt - 35, wrapping at isnr > 50

const MAXGRID4: u16 = 18 * 18 * 10 * 10; // 32400

/// Decoded exchange following the two callsigns.
#[derive(Debug, Clone, PartialEq)]
pub enum Exchange {
    /// 4-character Maidenhead grid like "FN42".
    Grid(String),
    /// Roger / 73 acknowledgements with no SNR.
    Acknowledgement(Ack),
    /// Signal report in dB. Range -50..+50 by design; the wire format
    /// is "[+-]NN" or with R prefix if `roger` was set on the message.
    Report(i8),
}

#[derive(Debug, Clone, PartialEq)]
pub enum Ack {
    /// Bare callsigns, no acknowledgement.
    None,
    /// "RRR"
    Rrr,
    /// "RR73"
    Rr73,
    /// "73"
    Bare73,
}

/// Decode the 15-bit `igrid4` field. Returns None if invalid.
pub fn decode_exchange(igrid4: u16) -> Option<Exchange> {
    if igrid4 < MAXGRID4 {
        // 4-character Maidenhead grid
        let mut n = igrid4 as u32;
        let j1 = (n / 1800) as u8; n %= 1800;
        let j2 = (n / 100) as u8;  n %= 100;
        let j3 = (n / 10) as u8;
        let j4 = (n - (j3 as u32) * 10) as u8;
        if j1 >= 18 || j2 >= 18 || j3 >= 10 || j4 >= 10 {
            return None;
        }
        let grid: String = [
            (b'A' + j1) as char,
            (b'A' + j2) as char,
            (b'0' + j3) as char,
            (b'0' + j4) as char,
        ].iter().collect();
        Some(Exchange::Grid(grid))
    } else {
        let irpt = igrid4 - MAXGRID4;
        match irpt {
            0 => None, // unused
            1 => Some(Exchange::Acknowledgement(Ack::None)),
            2 => Some(Exchange::Acknowledgement(Ack::Rrr)),
            3 => Some(Exchange::Acknowledgement(Ack::Rr73)),
            4 => Some(Exchange::Acknowledgement(Ack::Bare73)),
            _ => {
                // SNR: isnr = irpt - 35, wrap if > 50
                let mut isnr = (irpt as i32) - 35;
                if isnr > 50 { isnr -= 101; }
                if !(-50..=50).contains(&isnr) {
                    return None;
                }
                Some(Exchange::Report(isnr as i8))
            }
        }
    }
}

/// Inverse of [`decode_exchange`] for grid strings only. Used by tests and
/// future TX support. Returns `None` for malformed grids.
pub fn encode_grid(grid: &str) -> Option<u16> {
    let bytes = grid.as_bytes();
    if bytes.len() != 4 { return None; }
    let g = grid.to_ascii_uppercase();
    let b = g.as_bytes();
    let j1 = b[0].checked_sub(b'A')? as u32;
    let j2 = b[1].checked_sub(b'A')? as u32;
    let j3 = b[2].checked_sub(b'0')? as u32;
    let j4 = b[3].checked_sub(b'0')? as u32;
    if j1 >= 18 || j2 >= 18 || j3 >= 10 || j4 >= 10 {
        return None;
    }
    Some((j1 * 1800 + j2 * 100 + j3 * 10 + j4) as u16)
}

/// Inverse for SNR reports: encode an SNR in dB into the 15-bit igrid4
/// representation. Inverse of the `irpt - 35 (with wrap)` mapping.
#[allow(dead_code)]
pub fn encode_report(isnr: i8) -> Option<u16> {
    if !(-50..=50).contains(&isnr) { return None; }
    let irpt: i32 = if isnr <= 50 && isnr >= -30 {
        isnr as i32 + 35
    } else {
        // isnr in -50..-30
        isnr as i32 + 35 + 101
    };
    if irpt < 5 { return None; }
    Some(MAXGRID4 + irpt as u16)
}

/// Inverse for acknowledgement codes.
#[allow(dead_code)]
pub fn encode_ack(ack: Ack) -> u16 {
    let irpt = match ack {
        Ack::None => 1,
        Ack::Rrr => 2,
        Ack::Rr73 => 3,
        Ack::Bare73 => 4,
    };
    MAXGRID4 + irpt
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn grid_round_trip() {
        for g in ["AA00", "FN42", "IO82", "RR99", "JO22"] {
            let n = encode_grid(g).unwrap();
            assert_eq!(decode_exchange(n), Some(Exchange::Grid(g.to_string())));
        }
    }

    #[test]
    fn known_grid_value() {
        // FN42 hand-derivation: F=5, N=13, 4=4, 2=2 -> 5*1800+13*100+42 = 10342
        assert_eq!(encode_grid("FN42"), Some(10342));
    }

    #[test]
    fn acks() {
        for (irpt, expected) in [(1, Ack::None), (2, Ack::Rrr), (3, Ack::Rr73), (4, Ack::Bare73)] {
            assert_eq!(
                decode_exchange(MAXGRID4 + irpt),
                Some(Exchange::Acknowledgement(expected))
            );
        }
    }

    #[test]
    fn snr_round_trip() {
        for isnr in [-50i8, -30, -10, 0, 10, 30, 50] {
            let igrid4 = encode_report(isnr).unwrap();
            match decode_exchange(igrid4) {
                Some(Exchange::Report(n)) => assert_eq!(n, isnr, "round-trip for {}", isnr),
                other => panic!("isnr {}: got {:?}", isnr, other),
            }
        }
    }

    #[test]
    fn invalid_grid_returns_none() {
        // igrid4 = 32401 is irpt=1 (Ack::None) actually - past MAXGRID4
        // but irpt=0 = MAXGRID4 itself is unused
        assert_eq!(decode_exchange(MAXGRID4), None);
    }
}
