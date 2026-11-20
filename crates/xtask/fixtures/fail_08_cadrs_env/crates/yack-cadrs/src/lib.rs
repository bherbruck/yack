// yack: fixture temporary engine.
//! Cadrs.

/// A switch.
pub fn knob() -> bool {
    std::env::var_os("CADRS_KNOB").is_some()
}
