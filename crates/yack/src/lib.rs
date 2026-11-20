// yack: the public crate; it routes to the kernel crates and never computes (PLAN §4, §6.6).
//! yack, Yet Another CAD Kernel: an exact B-rep geometry kernel in pure, safe Rust.
//!
//! This is the only public crate. The API in `plan/PLAN.md` §4.3 lands here as stubs in M0.3.
#![deny(missing_docs)]
#![deny(clippy::float_arithmetic)]

/// This crate's version.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

#[cfg(test)]
mod tests {
    #[test]
    fn version_is_set() {
        assert!(!super::VERSION.is_empty());
    }
}
