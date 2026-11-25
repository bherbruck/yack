// yack: A compact binary encoding of built geometry, for build states kept on disk. Numbers are written
// Adapted from springrep@a03e8a1 src/codec.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: the Writer and Reader no longer carry face memos (M2.2 T4b); then to yack-codec (M3 W5), paths and visibility only.
// M3 hardening: split into the byte format (`wire`), the geometry's encodings (`geometry`) and the
// arena's (`topo`); the writer's bytes private; the same bytes, written in the same order.
//! A compact binary encoding of built geometry, for build states kept on disk. Numbers are written
//! as their bits, so a decoded solid is the encoded one exactly. A struct is encoded field by field
//! through `fields!`, which names every field without `..`: a field added to the struct and not
//! to its encoding does not compile.

/// [`Codec`] for a struct, field by field: every field named, none skipped with `..`.
macro_rules! fields {
    ($t:ident { $($f:ident),* $(,)? }) => {
        impl $crate::Codec for $t {
            fn put(&self, w: &mut $crate::Writer) {
                let $t { $($f),* } = self;
                $($crate::Codec::put($f, w);)*
            }
            fn get(r: &mut $crate::Reader) -> Option<$t> {
                Some($t { $($f: $crate::Codec::get(r)?),* })
            }
        }
    };
}

mod geometry;
#[cfg(test)]
mod tests;
mod topo;
mod wire;

use yack_topo::Brep;

pub use wire::Reader;
pub(crate) use wire::{Codec, Writer};

/// The solids as bytes that read back as the same solids to the last bit. Faces that share a memo
/// share it again when read. The bytes end in a digest of the rest.
pub fn to_bytes(solids: &[Brep]) -> Vec<u8> {
    let mut w = Writer::default();
    solids.len().put(&mut w);
    for s in solids {
        s.put(&mut w);
    }
    let sum = digest(w.as_bytes());
    sum.put(&mut w);
    w.into_bytes()
}

/// The solids [`to_bytes`] wrote into `bytes`: `None` for any other bytes, a damaged or shortened
/// copy among them. Bytes from another build of the kernel may read as nothing.
pub fn from_bytes(bytes: &[u8]) -> Option<Vec<Brep>> {
    let (body, sum) = bytes.split_at_checked(bytes.len().checked_sub(8)?)?;
    (u64::from_le_bytes(sum.try_into().ok()?) == digest(body)).then_some(())?;
    let mut r = Reader::new(body);
    let solids = Vec::<Brep>::get(&mut r)?;
    (r.remaining() == 0).then_some(solids)
}

/// FNV-1a's offset basis and prime, the digest's start and multiplier.
const DIGEST_BASIS: u64 = 0xcbf2_9ce4_8422_2325;
const DIGEST_PRIME: u64 = 0x0000_0100_0000_01b3;

/// A 64-bit digest of `bytes`, eight at a time: it finds a damaged or truncated file, nothing more.
pub fn digest(bytes: &[u8]) -> u64 {
    let mut h: u64 = DIGEST_BASIS ^ bytes.len() as u64;
    let (words, rest) = bytes.as_chunks::<8>();
    for &c in words {
        h = (h ^ u64::from_le_bytes(c)).wrapping_mul(DIGEST_PRIME).rotate_left(29);
    }
    for &b in rest {
        h = (h ^ u64::from(b)).wrapping_mul(DIGEST_PRIME);
    }
    h
}
