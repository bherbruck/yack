// yack: a 128-bit hashing key (two 64-bit streams), moved out of fingerprint.rs (M4.0a).
// Adapted from cadrs@eb5899ef crates/cadrs_kernel/src/native/fingerprint.rs (MIT OR Apache-2.0, (c) 2026 Rouan van der Ende).
// Changes: the key's hashing core only (its geometry digests stay in fingerprint.rs); `word`
//   made pub(crate) for them.
//! A 128-bit key: everything written goes to both of its streams.

use std::hash::Hasher;

/// A 128-bit key: everything written goes to both streams.
pub struct Key128 {
    a: u64,
    b: u64,
}

/// The first stream's mixer ("moremur", a stronger variant of MurmurHash3's finalizer).
#[inline(always)]
fn mix_a(mut x: u64) -> u64 {
    x ^= x >> 27;
    x = x.wrapping_mul(0x3C79_AC49_2BA7_B653);
    x ^= x >> 33;
    x = x.wrapping_mul(0x1C69_B3F7_4AC4_AE35);
    x ^ (x >> 27)
}

/// The second stream's (SplitMix64's finalizer).
#[inline(always)]
fn mix_b(mut x: u64) -> u64 {
    x ^= x >> 30;
    x = x.wrapping_mul(0xBF58_476D_1CE4_E5B9);
    x ^= x >> 27;
    x = x.wrapping_mul(0x94D0_49BB_1331_11EB);
    x ^ (x >> 31)
}

impl Key128 {
    /// An empty key.
    pub fn new() -> Self {
        Self {
            a: 0x243F_6A88_85A3_08D3,
            b: 0x1319_8A2E_0370_7344,
        }
    }

    /// One word into both streams. (The constants added keep a state off zero, the mixers'
    /// one fixed point, where a run of zero words would leave it.)
    #[inline(always)]
    pub(crate) fn word(&mut self, w: u64) {
        self.a = mix_a(self.a ^ w).wrapping_add(0x9E37_79B9_7F4A_7C15);
        self.b = mix_b(self.b.wrapping_add(w)).wrapping_add(0xD1B5_4A32_D192_ED03);
    }

    /// The key of everything written.
    pub fn key(&self) -> u128 {
        (mix_a(self.a) as u128) << 64 | mix_b(self.b) as u128
    }

    /// Numbers, by their bits.
    pub fn f64s(&mut self, xs: &[f64]) {
        for x in xs {
            self.write_u64(x.to_bits());
        }
    }

}

impl Default for Key128 {
    fn default() -> Self {
        Self::new()
    }
}

impl Hasher for Key128 {
    /// Bytes eight at a time; what is left over as one word more, its length above it (so
    /// "ab" then "c" isn't "a" then "bc").
    fn write(&mut self, bytes: &[u8]) {
        let (chunks, rest) = bytes.as_chunks::<8>();
        for c in chunks {
            self.word(u64::from_le_bytes(*c));
        }
        if !rest.is_empty() {
            let mut w = [0u8; 8];
            w[..rest.len()].copy_from_slice(rest);
            w[7] = rest.len() as u8;
            self.word(u64::from_le_bytes(w));
        }
    }

    #[inline]
    fn write_u64(&mut self, w: u64) {
        self.word(w);
    }

    #[inline]
    fn write_usize(&mut self, w: usize) {
        self.word(w as u64);
    }

    /// The first stream's half ([`Self::key`] is the whole).
    fn finish(&self) -> u64 {
        mix_a(self.a)
    }
}
