// yack: fixture module.

/// Half.
pub fn half() -> f64 {
    0.5
}

/// Swallows a panic.
pub fn quiet() -> bool {
    std::panic::catch_unwind(|| ()).is_ok()
}
