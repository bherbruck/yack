// yack: tests moved out of the crates M3 extracted, which build their operands with springrep's modules
// above those crates; they move to crates/yack/tests through the public API when springrep goes.
// W11: springrep's own kernel tests, moved here when springrep went (ruling (b)). They build with the
// kernel crates directly and compute their expected values, so the facade's maths ban and the
// workspace lints' test-only exemptions (which reach only #[test] fns) do not fit them.
// TODO(api-owner, hardening, D26): rewrite these tests to the lints, test by test, and drop this allow.
#![allow(clippy::disallowed_methods, clippy::disallowed_types, clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::indexing_slicing)]
mod build;
mod codec;
mod intersect;
mod measure;
mod mesh;
mod rays;
mod sweep;
mod tweak;
