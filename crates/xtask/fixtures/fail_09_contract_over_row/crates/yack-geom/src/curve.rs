// yack: fixture module.

/// Half.
pub fn half() -> f64 {
    0.5
}

/// The first `n` of `s`.
pub fn first(s: &[f64], n: usize) -> Vec<f64> {
    assert!(s.len() >= n, "first: slice too short");
    assert_eq!(n % 2, 0, "first: n must be even");
    s.iter().take(n).copied().collect()
}
