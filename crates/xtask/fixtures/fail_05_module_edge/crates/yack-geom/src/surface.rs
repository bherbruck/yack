// yack: fixture module that reaches a sibling.
use crate::curve::half;

/// Twice half.
pub fn one() -> f64 {
    half() + half()
}
