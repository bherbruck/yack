// yack: fixture module.

/// Half.
pub fn half() -> f64 {
    0.5
}

/// The i-th of two.
#[expect(clippy::panic, reason = "an index contract")]
pub fn of_two(i: usize) -> f64 {
    match i {
        0 | 1 => 0.5,
        _ => panic!("index {i} out of bounds (length 2)"),
    }
}
