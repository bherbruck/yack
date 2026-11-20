# yack-bench

The brep-bench runner for yack (PLAN §5.2): `yack-bench CASE OUT.stl [OUT.step]`, `--version`,
`--digests`. One program per case in `src/cases/{category}/{name_snake}.rs`; `build.rs` makes the
dispatch table. `harness.py` runs brep-bench's own harness on it without editing the brep-bench
clone (`cargo xtask bench` drives it and ratchets against `scores/bench.json`).

## Line counts

A case's `loc` is its lines that are neither blank nor comments, `use` and `fn` included (PLAN §4.4);
the limit is 10. Documented exceptions:

| Case | Lines | Why |
|---|---|---|
| boolean/golf-ball | 19 | The 62 icosahedral directions (vertices, edge midpoints, face centres) are plain data built in Rust; no kernel call is composed or missing (accepted by the lead) |
| swept/spline-bar-screwed-through-rod | 11 | Motion struct literals need typed `Point3`/`Vec3` fields; open API request: `Motion::screw(axis_point, axis, angle, rise)` taking `impl Into` would make it 10 |
