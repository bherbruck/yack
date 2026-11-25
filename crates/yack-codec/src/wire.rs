// yack: the byte format: a writer, a reader, and the encodings of numbers and collections.
// Adapted from springrep@a03e8a1 src/codec.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: split out of codec.rs (M3 hardening): the writer's bytes private, a tagged variant
// written by one function, pairs destructured; the same bytes in the same order.
//! The byte format: numbers little-endian (an `f64` as its bits, a `usize` as a `u64`), a
//! collection as its length and then its items, an `Option` or a variant as a tag byte and then
//! its value.

use std::collections::{BTreeMap, BTreeSet};

/// The bytes written so far.
#[derive(Default)]
pub(crate) struct Writer(Vec<u8>);

impl Writer {
    /// Appends `bytes`.
    fn push(&mut self, bytes: &[u8]) {
        self.0.extend_from_slice(bytes);
    }

    /// The bytes written so far.
    pub(crate) fn as_bytes(&self) -> &[u8] {
        &self.0
    }

    /// The bytes written.
    pub(crate) fn into_bytes(self) -> Vec<u8> {
        self.0
    }
}

/// The bytes not read yet.
pub struct Reader<'a> {
    bytes: &'a [u8],
}

impl<'a> Reader<'a> {
    pub fn new(bytes: &'a [u8]) -> Reader<'a> {
        Reader { bytes }
    }

    /// The next `n` bytes, or `None` where fewer are left.
    pub fn take(&mut self, n: usize) -> Option<&'a [u8]> {
        let (head, rest) = self.bytes.split_at_checked(n)?;
        self.bytes = rest;
        Some(head)
    }

    /// How many bytes are left.
    pub(crate) fn remaining(&self) -> usize {
        self.bytes.len()
    }

    /// The next `N` bytes, as an array.
    fn array<const N: usize>(&mut self) -> Option<[u8; N]> {
        self.take(N)?.try_into().ok()
    }
}

/// A value written to and read back from bytes. `get` is `None` for bytes no `put` wrote.
pub(crate) trait Codec: Sized {
    fn put(&self, w: &mut Writer);
    fn get(r: &mut Reader) -> Option<Self>;
}

/// Writes a variant: its tag, then its value.
pub(crate) fn put_tagged(tag: u8, value: &impl Codec, w: &mut Writer) {
    tag.put(w);
    value.put(w);
}

macro_rules! number {
    ($($t:ty),*) => {$(
        impl Codec for $t {
            fn put(&self, w: &mut Writer) {
                w.push(&self.to_le_bytes());
            }
            fn get(r: &mut Reader) -> Option<$t> {
                Some(<$t>::from_le_bytes(r.array()?))
            }
        }
    )*};
}
number!(u8, u32, u64, u128);

impl Codec for f64 {
    fn put(&self, w: &mut Writer) {
        self.to_bits().put(w);
    }
    fn get(r: &mut Reader) -> Option<f64> {
        Some(f64::from_bits(u64::get(r)?))
    }
}

impl Codec for usize {
    fn put(&self, w: &mut Writer) {
        (*self as u64).put(w);
    }
    fn get(r: &mut Reader) -> Option<usize> {
        usize::try_from(u64::get(r)?).ok()
    }
}

impl Codec for bool {
    fn put(&self, w: &mut Writer) {
        u8::from(*self).put(w);
    }
    fn get(r: &mut Reader) -> Option<bool> {
        match u8::get(r)? {
            0 => Some(false),
            1 => Some(true),
            _ => None,
        }
    }
}

impl Codec for String {
    fn put(&self, w: &mut Writer) {
        self.len().put(w);
        w.push(self.as_bytes());
    }
    fn get(r: &mut Reader) -> Option<String> {
        let n = usize::get(r)?;
        String::from_utf8(r.take(n)?.to_vec()).ok()
    }
}

/// A length read back, refused when the bytes left could not hold that many items: a damaged
/// length must not allocate without bound.
fn length(r: &mut Reader) -> Option<usize> {
    let n = usize::get(r)?;
    (n <= r.remaining()).then_some(n)
}

/// Writes a collection: its length, then each item.
fn put_all<'a, T: Codec + 'a>(len: usize, items: impl IntoIterator<Item = &'a T>, w: &mut Writer) {
    len.put(w);
    for x in items {
        x.put(w);
    }
}

impl<T: Codec> Codec for Vec<T> {
    fn put(&self, w: &mut Writer) {
        put_all(self.len(), self, w);
    }
    fn get(r: &mut Reader) -> Option<Vec<T>> {
        let n = length(r)?;
        (0..n).map(|_| T::get(r)).collect()
    }
}

impl<T: Codec> Codec for Option<T> {
    fn put(&self, w: &mut Writer) {
        match self {
            None => 0u8.put(w),
            Some(x) => put_tagged(1, x, w),
        }
    }
    fn get(r: &mut Reader) -> Option<Option<T>> {
        match u8::get(r)? {
            0 => Some(None),
            1 => Some(Some(T::get(r)?)),
            _ => None,
        }
    }
}

impl<T: Codec> Codec for Box<T> {
    fn put(&self, w: &mut Writer) {
        (**self).put(w);
    }
    fn get(r: &mut Reader) -> Option<Box<T>> {
        T::get(r).map(Box::new)
    }
}

impl<A: Codec, B: Codec> Codec for (A, B) {
    fn put(&self, w: &mut Writer) {
        let (a, b) = self;
        a.put(w);
        b.put(w);
    }
    fn get(r: &mut Reader) -> Option<(A, B)> {
        Some((A::get(r)?, B::get(r)?))
    }
}

impl<T: Codec> Codec for [T; 2] {
    fn put(&self, w: &mut Writer) {
        let [a, b] = self;
        a.put(w);
        b.put(w);
    }
    fn get(r: &mut Reader) -> Option<[T; 2]> {
        Some([T::get(r)?, T::get(r)?])
    }
}

impl<T: Codec + Ord> Codec for BTreeSet<T> {
    fn put(&self, w: &mut Writer) {
        put_all(self.len(), self, w);
    }
    fn get(r: &mut Reader) -> Option<BTreeSet<T>> {
        let n = length(r)?;
        (0..n).map(|_| T::get(r)).collect()
    }
}

impl<K: Codec + Ord, V: Codec> Codec for BTreeMap<K, V> {
    fn put(&self, w: &mut Writer) {
        self.len().put(w);
        for (k, v) in self {
            k.put(w);
            v.put(w);
        }
    }
    fn get(r: &mut Reader) -> Option<BTreeMap<K, V>> {
        let n = length(r)?;
        (0..n).map(|_| Some((K::get(r)?, V::get(r)?))).collect()
    }
}
