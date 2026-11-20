// yack: Kernel backends: `native`, the pure-Rust B-rep kernel.
// Adapted from cadrs@eb5899ef crates/cadrs_kernel/src/backend/mod.rs (MIT OR Apache-2.0, (c) 2026 Rouan van der Ende).
// Changes: none (verbatim).
//! Kernel backends: `native`, the pure-Rust B-rep kernel.

/// The native (pure Rust) kernel: [`NativeKernel`](crate::native::NativeKernel).
pub mod native {
    pub use crate::native::NativeKernel;
}
