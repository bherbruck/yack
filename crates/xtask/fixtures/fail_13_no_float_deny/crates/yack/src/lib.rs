// yack: fixture facade.
//! Facade.
#![deny(missing_docs)]

/// Version.
pub const VERSION: &str = "0";

pub use yack_geom::Point3;

/// The facade alone may catch panics.
pub fn guarded() -> bool {
    std::panic::catch_unwind(|| ()).is_ok()
}
