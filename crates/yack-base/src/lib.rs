// yack: the error type every yack crate returns (PLAN §4.1, DECISIONS D7).
//! yack-base: [`Error`], [`ErrorKind`], [`Stage`] and [`Result`], shared by every yack crate.
//!
//! Internal crate; the public face is the `yack` crate, which re-exports these as they are.
#![deny(missing_docs)]

// springrep's own error, beside D7's until each crate converts (D26).
#[allow(missing_docs)]
pub mod kernel;

use core::fmt;

/// What kind of refusal an [`Error`] is. The adapter matches on this alone.
#[non_exhaustive]
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ErrorKind {
    /// The request is at fault: an argument breaks a stated rule (a negative radius, a skewed frame).
    Invalid,
    /// A selection found nothing (no face within `tol::PICK` of the point).
    NotFound,
    /// A selection matched more than one entity; yack never chooses.
    Ambiguous,
    /// An id from another solid was passed.
    ForeignId,
    /// yack has no way to do this yet: not this function, or not with these arguments.
    Unsupported,
    /// A fair request that yack tried and could not carry out.
    Failed,
    /// The work budget ran out.
    Budget,
    /// A bug in yack: a checker rejection or a caught panic. The adapter reports it as a crash.
    Internal,
}

impl ErrorKind {
    /// The kind as one lower-case word, e.g. `"unsupported"`.
    pub fn word(self) -> &'static str {
        match self {
            ErrorKind::Invalid => "invalid",
            ErrorKind::NotFound => "not found",
            ErrorKind::Ambiguous => "ambiguous",
            ErrorKind::ForeignId => "foreign id",
            ErrorKind::Unsupported => "unsupported",
            ErrorKind::Failed => "failed",
            ErrorKind::Budget => "budget",
            ErrorKind::Internal => "internal",
        }
    }
}

/// Where in an operation a `Failed`, `Budget` or `Internal` error happened.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Stage {
    /// Argument validation, before any work.
    Validate,
    /// Curve and surface intersection.
    Intersect,
    /// Splitting faces and edges along intersections.
    Split,
    /// Classifying pieces in, on or out.
    Classify,
    /// Sewing pieces into a shell.
    Sew,
    /// Fitting approximating geometry.
    Fit,
    /// Healing imported or approximate geometry.
    Heal,
    /// Parsing an exchange file.
    Parse,
    /// The result checker.
    Check,
    /// Meshing.
    Mesh,
}

/// A yack refusal: what kind, which operation, which argument, where, and why.
///
/// ```
/// use yack_base::{Error, ErrorKind};
/// let e = Error::invalid("fillet", "radius", "radius -1 mm is not positive");
/// assert_eq!(e.kind, ErrorKind::Invalid);
/// assert_eq!(e.to_string(), "fillet: radius: radius -1 mm is not positive");
/// ```
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Error {
    /// The kind of refusal.
    pub kind: ErrorKind,
    /// The operation, as named in the API, e.g. `"fillet"`.
    pub op: &'static str,
    /// The parameter at fault, as named in the API, e.g. `"radius"`.
    pub arg: Option<&'static str>,
    /// Where a `Failed`, `Budget` or `Internal` error happened.
    pub stage: Option<Stage>,
    /// What was wrong, with numbers.
    pub detail: String,
}

impl Error {
    /// An error of any kind with no argument or stage.
    pub fn new(kind: ErrorKind, op: &'static str, detail: impl Into<String>) -> Error {
        Error { kind, op, arg: None, stage: None, detail: detail.into() }
    }

    /// `Invalid`: argument `arg` of `op` breaks a rule.
    pub fn invalid(op: &'static str, arg: &'static str, detail: impl Into<String>) -> Error {
        Error { kind: ErrorKind::Invalid, op, arg: Some(arg), stage: Some(Stage::Validate), detail: detail.into() }
    }

    /// `Unsupported`: `op` is not implemented yet (or not for these arguments).
    pub fn unsupported(op: &'static str) -> Error {
        Error::new(ErrorKind::Unsupported, op, "not implemented yet")
    }

    /// `NotFound`: a selection by `arg` found nothing.
    pub fn not_found(op: &'static str, arg: &'static str, detail: impl Into<String>) -> Error {
        Error { kind: ErrorKind::NotFound, op, arg: Some(arg), stage: None, detail: detail.into() }
    }

    /// `Ambiguous`: a selection by `arg` matched more than one entity.
    pub fn ambiguous(op: &'static str, arg: &'static str, detail: impl Into<String>) -> Error {
        Error { kind: ErrorKind::Ambiguous, op, arg: Some(arg), stage: None, detail: detail.into() }
    }

    /// `ForeignId`: argument `arg` holds an id from another solid.
    pub fn foreign_id(op: &'static str, arg: &'static str) -> Error {
        Error {
            kind: ErrorKind::ForeignId,
            op,
            arg: Some(arg),
            stage: None,
            detail: "the id belongs to another solid".to_string(),
        }
    }

    /// `Failed` at `stage`.
    pub fn failed(op: &'static str, stage: Stage, detail: impl Into<String>) -> Error {
        Error { kind: ErrorKind::Failed, op, arg: None, stage: Some(stage), detail: detail.into() }
    }

    /// The same error, reported as coming from `op` (for a value constructor called by an op).
    pub fn in_op(mut self, op: &'static str) -> Error {
        self.op = op;
        self
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: ", self.op)?;
        if let Some(arg) = self.arg {
            write!(f, "{arg}: ")?;
        }
        f.write_str(&self.detail)
    }
}

impl std::error::Error for Error {}

/// `core::result::Result` with yack's [`Error`].
pub type Result<T> = core::result::Result<T, Error>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unsupported_names_the_op() {
        let e = Error::unsupported("fillet");
        assert_eq!(e.kind, ErrorKind::Unsupported);
        assert_eq!(e.to_string(), "fillet: not implemented yet");
    }
}
