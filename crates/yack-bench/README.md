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
| cnc/flat-end-mill-profile | 31 | data: the cross's rounded-corner toolpath (tangent points and arc centres) computed inline; one `cut_swept` over `Motion::Then` of slides and turns |
| cnc/bull-nose-profile | 24 | data: the flower's lobe/valley centres and arc sweeps computed inline; one `cut_swept` over `Motion::Then` of turns |
| cnc/pocket-with-an-island | 78 | data: a ~400-move 2.5-axis program (raster intervals, rounded wall path, island circle, bore, pecked holes) for three tools; three `cut_swept` calls |
| cnc/ball-end-surface-raster | 20 | data: 944 raster points sampled from the surface f(x, y); one `cut_swept` over `Motion::Then` of slides |
| cnc/five-axis-dome | 58 | data: the five-axis program (8 roughing levels, 180 meridians, 15 pole circles, 12 seats and holes) as slides and turns for four tools; four `cut_swept` calls |
| swept/tumbling-torus-crossed-union | 12 | two sweeps of one torus (A and B), then the boolean |
| swept/tumbling-torus-crossed-difference | 12 | two sweeps of one torus (A and B), then the boolean |
| swept/tumbling-torus-crossed-intersection | 12 | two sweeps of one torus (A and B), then the boolean |
| swept/straight-bevel-gear-cut-by-its-pinion | 18 | data: the pinion tooth's trapezoid scaled to the inner and outer cone planes |
| swept/hypoid-gear-cut-by-its-pinion | 20 | data: the offset pinion tooth's trapezoid and frame angle |
| swept/worm-wheel-hobbed-by-its-worm | 14 | data: the worm thread's trapezoidal section |
| swept/helicon-face-gear-hobbed-by-its-worm | 15 | data: the worm thread's trapezoidal section |
| swept/spiroid-face-gear-hobbed-by-its-worm | 17 | data: the tapered thread's section on its pitch cone |
| integration/utah-teapot | 71 | data: the teapot's 32 Bézier patch control nets |

## Cases refused on purpose

| Case | Why |
|---|---|
| integration/benchy | A reference shape: it passes only as the original (1385 trimmed faces) within 2e-5. Copying the reference's faces into the program is forbidden by the case text ("the reference's faces are not taken in as its own") and by AGENTS.md rule 9 (no hand-built geometry). A modelled reconstruction would grade wrong, which ranks below a refusal. So it refuses. |
