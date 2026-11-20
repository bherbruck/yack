// yack: conversion between springrep's `Brep` and cadrs's `Body` (PLAN §3.2, M1.9); see MAPPING.md.
//! The bridge between the two temporary engines: [`to_cadrs`] and [`to_springrep`] carry a solid
//! across exactly where both represent it alike, refuse where they cannot, and never hand back a
//! body that fails its target's checker.

use core::fmt;

mod report;
mod to_cadrs;
mod to_springrep;

pub use report::{AGREE, CADRS_AGREE, RoundTrip, round_trip};
pub use to_cadrs::to_cadrs;
pub use to_springrep::{to_springrep, to_springrep_mapped};

/// Why a conversion did not happen.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Error {
    /// The source holds something the target cannot represent (yet), named.
    Unsupported(String),
    /// The converted body failed the target engine's checker; these are its issues.
    Rejected(Vec<String>),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Unsupported(m) => write!(f, "bridge: unsupported: {m}"),
            Error::Rejected(issues) => write!(f, "bridge: the converted body is invalid: {}", issues.join("; ")),
        }
    }
}

impl std::error::Error for Error {}

/// A bridge result.
pub type Result<T> = core::result::Result<T, Error>;
