// yack: springrep's kernel error, kept beside D7's Error until each crate converts (temporary, D26).
// Adapted from springrep@a03e8a1 src/error.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's error.rs to yack_base::kernel (M3 W1', D26); paths and visibility only.
//! springrep's kernel error. Temporary (D26): each crate's M3 hardening moves onto [`crate::Error`]
//! (D7), and this module goes when the last user has.

use std::fmt;

/// Every kernel failure. An operation that cannot produce a valid solid returns one of these rather
/// than a solid that fails springrep's checker.
#[derive(Clone, Debug, PartialEq)]
pub enum Error {
    /// The caller's input is meaningless: a zero-length extrusion, an open profile, an axis that
    /// crosses the profile.
    InvalidInput(String),
    /// The input is meaningful but this kernel does not build it yet.
    Unsupported(String),
    /// A numerical step failed: a projection that did not converge, a singular system.
    Numerical(String),
    /// The result would not be a valid closed manifold solid; the strings are the checker's issues.
    InvalidResult(Vec<String>),
}

pub type Result<T> = std::result::Result<T, Error>;

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::InvalidInput(m) => write!(f, "invalid input: {m}"),
            Error::Unsupported(m) => write!(f, "unsupported: {m}"),
            Error::Numerical(m) => write!(f, "numerical failure: {m}"),
            Error::InvalidResult(issues) => {
                write!(f, "invalid result: {}", issues.join("; "))
            }
        }
    }
}

impl std::error::Error for Error {}

pub fn invalid<T>(msg: impl Into<String>) -> Result<T> {
    Err(Error::InvalidInput(msg.into()))
}
