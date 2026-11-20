// yack: rustc-hash-shaped deterministic hash maps (FxHasher, 64-bit).
//! The hasher is FxHash as rustc-hash 2.1 defines it on 64-bit targets (multiply-add by
//! `0xf1357aea2e62a9c5`, the result rotated left 26; byte strings through its two-lane
//! multiply-mix), re-implemented here so that the maps cadrs_kernel fills iterate in the same
//! order as with rustc-hash. It is not keyed: the order is a function of the keys alone.

use std::collections::{HashMap, HashSet};
use std::hash::{BuildHasher, Hasher};

/// rustc-hash's `FxHashMap`.
pub type FxHashMap<K, V> = HashMap<K, V, FxBuildHasher>;
/// rustc-hash's `FxHashSet`.
pub type FxHashSet<V> = HashSet<V, FxBuildHasher>;

const K: u64 = 0xf135_7aea_2e62_a9c5;
const SEED1: u64 = 0x243f_6a88_85a3_08d3;
const SEED2: u64 = 0x1319_8a2e_0370_7344;
const PREVENT_TRIVIAL_ZERO_COLLAPSE: u64 = 0xa409_3822_299f_31d0;

/// The Fx hasher.
#[derive(Clone, Default, Debug)]
pub struct FxHasher {
    hash: u64,
}

impl FxHasher {
    /// A hasher starting from `seed`.
    pub const fn with_seed(seed: usize) -> FxHasher {
        FxHasher { hash: seed as u64 }
    }
    #[inline]
    fn add_to_hash(&mut self, i: u64) {
        self.hash = self.hash.wrapping_add(i).wrapping_mul(K);
    }
}

#[inline]
fn multiply_mix(x: u64, y: u64) -> u64 {
    let full = u128::from(x).wrapping_mul(u128::from(y));
    (full as u64) ^ ((full >> 64) as u64)
}

fn le_u64(b: &[u8]) -> u64 {
    let mut a = [0u8; 8];
    a.copy_from_slice(&b[..8]);
    u64::from_le_bytes(a)
}

fn le_u32(b: &[u8]) -> u64 {
    let mut a = [0u8; 4];
    a.copy_from_slice(&b[..4]);
    u64::from(u32::from_le_bytes(a))
}

fn hash_bytes(bytes: &[u8]) -> u64 {
    let len = bytes.len();
    let mut s0 = SEED1;
    let mut s1 = SEED2;
    if len <= 16 {
        if len >= 8 {
            s0 ^= le_u64(&bytes[0..8]);
            s1 ^= le_u64(&bytes[len - 8..]);
        } else if len >= 4 {
            s0 ^= le_u32(&bytes[0..4]);
            s1 ^= le_u32(&bytes[len - 4..]);
        } else if len > 0 {
            let lo = bytes[0];
            let mid = bytes[len / 2];
            let hi = bytes[len - 1];
            s0 ^= u64::from(lo);
            s1 ^= (u64::from(hi) << 8) | u64::from(mid);
        }
    } else {
        let bulk = &bytes[..len - 1];
        for chunk in bulk.chunks_exact(16) {
            let x = le_u64(&chunk[..8]);
            let y = le_u64(&chunk[8..]);
            let t = multiply_mix(s0 ^ x, PREVENT_TRIVIAL_ZERO_COLLAPSE ^ y);
            s0 = s1;
            s1 = t;
        }
        let suffix = &bytes[len - 16..];
        s0 ^= le_u64(&suffix[0..8]);
        s1 ^= le_u64(&suffix[8..16]);
    }
    multiply_mix(s0, s1) ^ (len as u64)
}

impl Hasher for FxHasher {
    #[inline]
    fn write(&mut self, bytes: &[u8]) {
        self.write_u64(hash_bytes(bytes));
    }
    #[inline]
    fn write_u8(&mut self, i: u8) {
        self.add_to_hash(u64::from(i));
    }
    #[inline]
    fn write_u16(&mut self, i: u16) {
        self.add_to_hash(u64::from(i));
    }
    #[inline]
    fn write_u32(&mut self, i: u32) {
        self.add_to_hash(u64::from(i));
    }
    #[inline]
    fn write_u64(&mut self, i: u64) {
        self.add_to_hash(i);
    }
    #[inline]
    fn write_u128(&mut self, i: u128) {
        self.add_to_hash(i as u64);
        self.add_to_hash((i >> 64) as u64);
    }
    #[inline]
    fn write_usize(&mut self, i: usize) {
        self.add_to_hash(i as u64);
    }
    #[inline]
    fn finish(&self) -> u64 {
        self.hash.rotate_left(26)
    }
}

/// rustc-hash's `FxBuildHasher`.
#[derive(Copy, Clone, Default, Debug)]
pub struct FxBuildHasher;

impl BuildHasher for FxBuildHasher {
    type Hasher = FxHasher;
    fn build_hasher(&self) -> FxHasher {
        FxHasher::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::hash::BuildHasher;

    #[test]
    fn matches_rustc_hash_on_integers() {
        // rustc-hash 2.1's own test vectors (64-bit).
        assert_eq!(FxBuildHasher.hash_one(0_u8), 0);
        assert_eq!(FxBuildHasher.hash_one(1_u8), 12157901119326311915);
        assert_eq!(FxBuildHasher.hash_one(u64::MAX), 6288842954450348564);
        assert_eq!(FxBuildHasher.hash_one(1_u128), 13032756267696824044);
        assert_eq!(FxBuildHasher.hash_one(i64::MIN), 33554432);
    }

    struct Bytes(&'static [u8]);
    impl std::hash::Hash for Bytes {
        fn hash<H: Hasher>(&self, state: &mut H) {
            state.write(self.0);
        }
    }

    #[test]
    fn matches_rustc_hash_on_bytes() {
        assert_eq!(FxBuildHasher.hash_one(Bytes(&[])), 17606491139363777937);
        assert_eq!(FxBuildHasher.hash_one(Bytes(&[0])), 5448590020104574886);
        assert_eq!(FxBuildHasher.hash_one(Bytes(&[0, 0, 0, 0, 0, 0])), 16766921560080789783);
        assert_eq!(FxBuildHasher.hash_one(Bytes(b"uwu")), 7168164714682931527);
        assert_eq!(
            FxBuildHasher.hash_one(Bytes(b"These are some bytes for testing rustc_hash.")),
            2349210501944688211
        );
    }
}
