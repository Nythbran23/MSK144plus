// crates/msk144plus_fec/src/short.rs
//
// MSK40 short-message (32,16) LDPC code.
//
// Faithful port of WSJT-X bpdecode40.f90 + encode_msk40.f90.

pub const COLORDER: [u8; 32] = [
    4, 1, 2, 3, 0, 8, 6, 10,
    13, 28, 20, 23, 17, 15, 27, 25,
    16, 12, 18, 19, 7, 21, 22, 11,
    24, 5, 26, 14, 9, 29, 30, 31,
];

pub const G16: [u16; 16] = [
    0x4428, 0x5a6b, 0x1b04, 0x2c12, 0x60c4, 0x1071, 0xbe6a, 0x36dd,
    0xc580, 0xad9a, 0xeca2, 0x7843, 0x332e, 0xa685, 0x5906, 0x1efe,
];

pub const MN_TABLE: [[usize; 3]; 32] = [
    [0, 5, 12], [1, 2, 13], [3, 7, 14], [4, 10, 11],
    [6, 9, 15], [5, 8, 14], [0, 10, 15], [1, 3, 4],
    [2, 6, 8], [7, 9, 11], [7, 12, 13], [0, 3, 11],
    [1, 5, 9], [2, 10, 14], [4, 8, 13], [6, 12, 14],
    [11, 13, 15], [0, 1, 7], [2, 4, 5], [3, 8, 10],
    [0, 6, 13], [4, 9, 12], [2, 3, 15], [1, 14, 15],
    [5, 6, 11], [6, 7, 10], [0, 8, 9], [1, 10, 12],
    [2, 11, 12], [3, 5, 13], [0, 4, 14], [7, 8, 15],
];

pub const NM_TABLE: [[usize; 7]; 16] = [
    [0, 6, 11, 17, 20, 26, 30],
    [1, 7, 12, 17, 23, 27, 0],
    [1, 8, 13, 18, 22, 28, 0],
    [2, 7, 11, 19, 22, 29, 0],
    [3, 7, 14, 18, 21, 30, 0],
    [0, 5, 12, 18, 24, 29, 0],
    [4, 8, 15, 20, 24, 25, 0],
    [2, 9, 10, 17, 25, 31, 0],
    [5, 8, 14, 19, 26, 31, 0],
    [4, 9, 12, 21, 26, 0, 0],
    [3, 6, 13, 19, 25, 27, 0],
    [3, 9, 11, 16, 24, 28, 0],
    [0, 10, 15, 21, 27, 28, 0],
    [1, 10, 14, 16, 20, 29, 0],
    [2, 5, 13, 15, 23, 30, 0],
    [4, 6, 16, 22, 23, 31, 0],
];

pub const NRW: [usize; 16] = [7, 6, 6, 6, 6, 6, 6, 6, 6, 5, 6, 6, 6, 6, 6, 6];

pub fn encode_short(message_16: &[u8; 16]) -> [u8; 32] {
    let mut gen40 = [[0u8; 16]; 16];
    for i in 0..16 {
        for j in 0..16 {
            let bit_pos = 15 - j;
            if (G16[i] >> bit_pos) & 1 == 1 {
                gen40[i][j] = 1;
            }
        }
    }
    let mut pchecks = [0u8; 16];
    for i in 0..16 {
        let mut nsum = 0u32;
        for j in 0..16 {
            nsum += (message_16[j] as u32) * (gen40[i][j] as u32);
        }
        pchecks[i] = (nsum % 2) as u8;
    }
    let mut itmp = [0u8; 32];
    for i in 0..16 { itmp[i] = pchecks[i]; }
    for i in 0..16 { itmp[16 + i] = message_16[i]; }
    let mut codeword = [0u8; 32];
    for k in 0..32 {
        codeword[COLORDER[k] as usize] = itmp[k];
    }
    codeword
}

fn platanh(x: f32) -> f32 {
    let ax = x.abs();
    let s = if x < 0.0 { -1.0 } else { 1.0 };
    if ax <= 0.664 {
        s * ax * 0.83
    } else if ax <= 0.9217 {
        s * (0.83 * 0.664 + (ax - 0.664) * 2.18)
    } else if ax <= 0.9981 {
        s * (0.83 * 0.664 + 0.2577 * 2.18 + (ax - 0.9217) * 9.06)
    } else if ax <= 0.99999 {
        s * (0.83 * 0.664 + 0.2577 * 2.18 + 0.0764 * 9.06 + (ax - 0.9981) * 71.94)
    } else {
        s * 12.0
    }
}

#[derive(Debug, Clone)]
pub struct ShortDecodeResult {
    pub message: [u8; 16],
    pub codeword: [u8; 32],
    pub iterations: i32,
}

pub fn decode_short(llr: &[f32; 32], max_iterations: usize) -> Option<ShortDecodeResult> {
    const N: usize = 32;
    const M: usize = 16;
    const NCW: usize = 3;

    let mut tov = [[0.0f32; N]; NCW];
    let mut toc = [[0.0f32; M]; 7];
    let mut tanhtoc = [[0.0f32; M]; 7];

    for j in 0..M {
        for i in 0..NRW[j] {
            let bit_idx = NM_TABLE[j][i];
            toc[i][j] = llr[bit_idx];
        }
    }

    for iter in 0..=max_iterations {
        let mut zn = [0.0f32; N];
        for i in 0..N {
            let mut s = llr[i];
            for k in 0..NCW {
                s += tov[k][i];
            }
            zn[i] = s;
        }
        let mut cw = [0u8; N];
        for i in 0..N {
            cw[i] = if zn[i] > 0.0 { 1 } else { 0 };
        }
        let mut ncheck = 0;
        for j in 0..M {
            let mut s = 0u32;
            for i in 0..NRW[j] {
                s += cw[NM_TABLE[j][i]] as u32;
            }
            if s % 2 != 0 { ncheck += 1; }
        }
        if ncheck == 0 {
            let mut codeword = [0u8; N];
            for k in 0..N {
                codeword[k] = cw[COLORDER[k] as usize];
            }
            let mut decoded = [0u8; M];
            for i in 0..M {
                decoded[i] = codeword[M + i];
            }
            return Some(ShortDecodeResult {
                message: decoded, codeword, iterations: iter as i32,
            });
        }
        for j in 0..M {
            for i in 0..NRW[j] {
                let ibj = NM_TABLE[j][i];
                let mut t = zn[ibj];
                for kk in 0..NCW {
                    if MN_TABLE[ibj][kk] == j {
                        t -= tov[kk][ibj];
                    }
                }
                toc[i][j] = t;
            }
        }
        for j in 0..M {
            for i in 0..7 {
                tanhtoc[i][j] = (-toc[i][j] / 2.0).tanh();
            }
        }
        for j in 0..N {
            for i in 0..NCW {
                let ichk = MN_TABLE[j][i];
                let mut tmn = 1.0f32;
                for k in 0..NRW[ichk] {
                    if NM_TABLE[ichk][k] != j {
                        tmn *= tanhtoc[k][ichk];
                    }
                }
                let y = platanh(-tmn);
                tov[i][j] = 2.0 * y;
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_zero_message_yields_zero_codeword() {
        let msg = [0u8; 16];
        let cw = encode_short(&msg);
        for &b in &cw { assert_eq!(b, 0); }
    }

    #[test]
    fn encode_then_decode_recovers_message() {
        let msg: [u8; 16] = [1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1];
        let cw = encode_short(&msg);
        let mut llr = [0.0f32; 32];
        for i in 0..32 {
            llr[i] = if cw[i] == 1 { 5.0 } else { -5.0 };
        }
        let result = decode_short(&llr, 5).expect("BP should converge");
        assert_eq!(result.message, msg);
    }

    #[test]
    fn bp_corrects_single_bit_error() {
        let msg: [u8; 16] = [1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1];
        let cw = encode_short(&msg);
        let mut llr = [0.0f32; 32];
        for i in 0..32 {
            llr[i] = if cw[i] == 1 { 5.0 } else { -5.0 };
        }
        llr[10] = -llr[10];
        let result = decode_short(&llr, 5).expect("BP should fix 1 bit");
        assert_eq!(result.message, msg);
    }
}
