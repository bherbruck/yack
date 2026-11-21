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

/// The first `n` of `s`.
pub fn first(s: &[f64], n: usize) -> Vec<f64> {
    assert!(s.len() >= n, "first: slice too short");
    debug_assert!(n > 0);
    s.iter().take(n).copied().collect()
}

#[cfg(test)]
mod tests {
    #[test]
    fn halves() {
        assert_eq!(super::half(), 0.5);
    }
}
