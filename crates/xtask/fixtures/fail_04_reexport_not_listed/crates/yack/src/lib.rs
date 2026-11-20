// yack: fixture facade.
//! Facade.
#![deny(missing_docs)]
#![deny(clippy::float_arithmetic)]

/// Version.
pub const VERSION: &str = "0";

pub use yack_geom::{Point3, curve};

/// The facade alone may catch panics.
pub fn guarded() -> bool {
    std::panic::catch_unwind(|| ()).is_ok()
}
