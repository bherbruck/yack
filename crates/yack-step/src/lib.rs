// yack: STEP B-rep files measured exactly from their own geometry, by ISO 10303-42's rules.
//! An exchange file's solid measured as the standard defines it: its volume and area from its
//! faces' surfaces bounded by their edges' curves.

#[cfg(feature = "wip")]
mod bspline;
#[cfg(feature = "wip")]
mod geom;
#[cfg(feature = "wip")]
mod measure;
#[cfg(feature = "wip")]
mod read;

#[cfg(feature = "wip")]
pub use measure::{FaceMeasure, FileMeasure, measure_file};
