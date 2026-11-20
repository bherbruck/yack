// yack: `yack::codec`, yack's own binary format, routed to the engine (M1.5, w-m11-springrep).
// Decoding runs behind the panic firewall: the bytes are untrusted input.
//! yack's own lossless binary format.
use crate::{Error, ErrorKind, Result, Solid, firewall};

/// The solid as bytes that [`decode`] reads back exactly.
pub fn encode(s: &Solid) -> Vec<u8> {
    match s.brep() {
        None => yack_springrep::codec::to_bytes(&[]),
        Some(b) => yack_springrep::codec::to_bytes(core::slice::from_ref(b)),
    }
}
/// The solid encoded in `b`.
pub fn decode(b: &[u8]) -> Result<Solid> {
    let op = "codec::decode";
    firewall::run(op, || {
        let parts = yack_springrep::codec::from_bytes(b).ok_or_else(|| Error::invalid(op, "b", "not a yack codec stream"))?;
        let mut parts = parts.into_iter();
        match (parts.next(), parts.next()) {
            (None, _) => Ok(Solid::empty()),
            (Some(one), None) => Ok(Solid::from_springrep(one)),
            _ => Err(Error::new(ErrorKind::Unsupported, op, "a stream of several solids")),
        }
    })
}
