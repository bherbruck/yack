# Donor bake-off ledger (PLAN §3.5)

The order of each op's ladder, and the measured passes it rests on. An op not listed runs springrep
first (`crates/yack/src/ladder.rs`, `CADRS_FIRST`). Each row is one bake-off: the op's family run
with each engine forced (`yack-bench --engine E[@OP]` on a `--features bakeoff` build; `@OP` forces only that op and leaves the operands on the default ladder), case by case.

| Op | Order | Benchmark, family | springrep passes | cadrs passes | Only one engine passes | Time (ms, springrep / cadrs) | Date, binary |
|---|---|---|---|---|---|---|---|
| `chamfer_angle` | native (yack-blend `revolved`), then cadrs | brep-bench `chamfer/*` (18 cases: hemisphere 2, ball-on-plate 3, capped-frustum 3, wide/narrow/spherical sector 7, ball-in-conical-seat 3) | native 18/18 ok (no springrep rung for this op) | 18/18 ok | none: both answer all 18; volume within 6.2e-16 relative, area within 5.4e-16, face counts equal on all 18 | native 452 / cadrs 527 (whole family, CPU, one round) | 2026-11-25, tree fe208d4+75d8024b, `yack-bench --features bakeoff`, `--engine cadrs@chamfer_angle` / `native@chamfer_angle`; res w-m04-conf-67, swarm/res/m48-bake.tsv |
| `ruled` | native (yack-build `ruled`), then cadrs | brep-bench `geometry/hyperboloid-ruled` (the only bench case reaching the op; no conformance family calls it) | native 1/1 ok (no springrep rung for this op) | 1/1 ok | none: both answer; volume and area bit-identical (523.5987755982989 = 500π/3, 426.26652204800996), valid both; native has one ruled side face where cadrs splits it in two (3 faces against 4), every face with a History origin (cadrs 2 of 4) | native ~0 / cadrs 18 (CPU, mean of 5 runs) | 2026-11-25, tree 6a9e679+d52e726a, `yack-bench --features bakeoff` (res w-m11-springrep-239), `--engine cadrs@ruled` / `native@ruled`; full-bench sweep: 506 cases, digests differ only on hyperboloid-ruled, yack-ref/bin/w-m11-bake/sweep-ruled/{default,forced,differ}.txt |
| `project` | springrep only (no ladder) | conformance `projections`, seeds 0-3 (120 cases) | **void**: no valid bake-off | **void** | none measured | none measured | 2026-11-20 row withdrawn; the res w-m11-springrep-86 runs were both springrep |

Notes:
- `project`: the earlier row (117 springrep / 118 cadrs, res w-m11-springrep-86) was invalid. `project`
  was not a ladder op when it was run, so `YACK_ENGINE` did not reach it and both "forced" runs ran
  springrep; the one-case difference on s3 was not an engine difference. The cadrs rung that was then
  added on the strength of that row cost seed-3 `projections` cases at 03fd97d and has been removed.
  `project` runs springrep only until a real bake-off (both rungs reachable, each forced) is measured.
- brep-bench grades from the STEP file and has no `project` case, so only conformance decides this row.
- `offset`, SMLib corpus `offset/offset_04` and `_07` (`off_extrusion` grown 0.3 / 1): springrep declines
  (an extrusion of a cubic directrix has no offset spline within `OFFSET_FIT` = 1e-8). cadrs answers, held in
  cadrs form: carrying its body back is rejected because one edge meets its faces only to 4.0e-4, over the 1e-4
  limit. Its volume is 2.2e-4 below SMLib's. Per-face closed-form check under way (w-m11, 2026-11-25).
- `chamfer_angle`: native declines everything outside a whole circular rim between coaxial plane, cylinder, cone, sphere or torus faces, including any result whose volume is not the body minus or plus the ring; on those it falls through to cadrs. Only meant for the 18 bench cases and their kin.
- `ruled`: the face-count difference (one side face, not two) touches no other case: nothing else calls ruled (grep, and the 506-case sweep).
