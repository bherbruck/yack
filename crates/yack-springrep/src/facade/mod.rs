// yack: thin glue the `yack` facade calls instead of computing itself (gate 13); dies with this crate (M3).
//! Thin functions over springrep's own API, taking plain arrays, so `crates/yack` routes and never
//! computes. Each one does only the arithmetic the protocol's argument shapes need.

pub mod api;
pub use yack_edit::glue as edit;
pub use yack_profile::loops;
pub use yack_step::exchange;
