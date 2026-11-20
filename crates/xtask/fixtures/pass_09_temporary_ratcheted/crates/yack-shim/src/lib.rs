// yack: fixture temporary shim.
//! Shim.

/// Parses.
pub fn parse(s: &str) -> u32 {
    s.parse().unwrap()
}
