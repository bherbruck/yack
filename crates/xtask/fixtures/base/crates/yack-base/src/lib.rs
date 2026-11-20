// yack: fixture kernel crate at layer 0.
//! Base.

/// A marked tolerance literal is fine.
pub const TINY: f64 = 1e-9; // tol: fixture

/// One.
pub fn one() -> u32 {
    1
}

#[cfg(test)]
mod tests {
    #[test]
    fn test_code_is_exempt() {
        let _ = 1e-12;
        let _ = std::env::var("X");
        let _ = std::time::Instant::now();
    }
}
