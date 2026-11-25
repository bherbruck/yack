// yack: `yack::codec`, yack's own binary format, routed to the engine (M1.5, w-m11-springrep).
// Decoding runs behind the panic firewall: the bytes are untrusted input.
//! yack's own lossless binary format.
use crate::{Error, ErrorKind, Result, Solid, firewall};

/// The solid as bytes that [`decode`] reads back exactly; `Unsupported` for a solid held in
/// cadrs form only (the format carries springrep bodies), never an empty stream in its place.
///
/// ```
/// use yack::prelude::*;
/// let b = block(&Frame::WORLD, [1.0, 2.0, 3.0])?;
/// let bytes = yack::codec::encode(&b)?;
/// assert_eq!(yack::codec::decode(&bytes)?.digest(), b.digest());
/// # Ok::<(), yack::Error>(())
/// ```
pub fn encode(s: &Solid) -> Result<Vec<u8>> {
    Ok(match s.form("codec::encode")? {
        None => yack_codec::to_bytes(&[]),
        Some(b) => yack_codec::to_bytes(core::slice::from_ref(b)),
    })
}
/// The solid encoded in `b`.
///
/// ```
/// use yack::prelude::*;
/// assert_eq!(yack::codec::decode(b"not a stream").unwrap_err().kind, ErrorKind::Invalid);
/// assert!(yack::codec::decode(&yack::codec::encode(&empty())?)?.is_empty());
/// # Ok::<(), yack::Error>(())
/// ```
pub fn decode(b: &[u8]) -> Result<Solid> {
    let op = "codec::decode";
    firewall::read(op, || {
        let parts = yack_codec::from_bytes(b).ok_or_else(|| Error::invalid(op, "b", "not a yack codec stream"))?;
        let mut parts = parts.into_iter();
        match (parts.next(), parts.next()) {
            (None, _) => Ok(Solid::empty()),
            (Some(one), None) => Ok(Solid::from_springrep(one)),
            _ => Err(Error::new(ErrorKind::Unsupported, op, "a stream of several solids")),
        }
    })
}
