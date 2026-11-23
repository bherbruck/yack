# yack-bench

The brep-bench runner for yack (PLAN §5.2): `yack-bench CASE OUT.stl [OUT.step]`, `--version`,
`--digests`. One program per case in `src/cases/{category}/{name_snake}.rs`; `build.rs` makes the
dispatch table. `harness.py` runs brep-bench's own harness on it without editing the brep-bench
clone (`cargo xtask bench` drives it and ratchets against `scores/bench.json`).

## Line counts

A case's `loc` is its lines that are neither blank nor comments, `use` and `fn` included (PLAN §4.4);
the limit is 10.

## Data-bound cases

These cases are still over 10 lines. In each, the extra lines are the case's own data (numbers the
case text gives, or a program it spells out), not missing kernel calls; every kernel call is one line.

| Case | Lines | Why |
|---|---|---|
| integration/utah-teapot | 71 | Newell's patch constants: the teapot's 32 Bézier control nets, turned and mirrored round the axis |
| cnc/five-axis-dome | 55 | the five-axis program (8 roughing levels, 180 meridians, 15 pole circles, 12 seats and holes) as slides and turns for four tools |
| cnc/pocket-with-an-island | 39 | the 2.5-axis program for three tools: raster intervals, the rounded wall, the island circle, the bore and the pecked holes |
| cnc/bull-nose-profile | 24 | the flower's lobe and valley centres and each arc's sweep |
| swept/hypoid-gear-cut-by-its-pinion | 18 | pitch-cone data: the blank's section, the tooth's trapezoid and its offset angle |
| boolean/golf-ball | 18 | the 62 icosahedral directions (vertices, edge midpoints, face centres) |
| swept/straight-bevel-gear-cut-by-its-pinion | 17 | pitch-cone data: the blank's section and the tooth's trapezoid at the inner cone plane |
| swept/spiroid-face-gear-hobbed-by-its-worm | 16 | the tapered thread's section on its pitch cone and the ring's section |
| cnc/ball-end-surface-raster | 16 | the surface f(x, y) sampled along 41 zigzag passes (944 points) |
| swept/helicon-face-gear-hobbed-by-its-worm | 14 | the worm thread's trapezoidal section and the ring |
| swept/worm-wheel-hobbed-by-its-worm | 13 | the worm thread's trapezoidal section |
| cnc/flat-end-mill-profile | 12 | the cross's 13 corners, listed from the plunge point |

## Cases refused on purpose

| Case | Why |
|---|---|
| integration/benchy | A reference shape: it passes only as the original (1385 trimmed faces) within 2e-5. Copying the reference's faces into the program is forbidden by the case text ("the reference's faces are not taken in as its own") and by AGENTS.md rule 9 (no hand-built geometry). A modelled reconstruction would grade wrong, which ranks below a refusal. So it refuses. |
