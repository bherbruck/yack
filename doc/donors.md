# Donor bake-off ledger (PLAN §3.5)

The order of each op's ladder, and the measured passes it rests on. An op not listed runs springrep
first (`crates/yack/src/ladder.rs`, `CADRS_FIRST`). Each row is one bake-off: the op's family run
with each engine forced (`YACK_ENGINE` on a `--features bakeoff` build), case by case.

| Op | Order | Benchmark, family | springrep passes | cadrs passes | Only one engine passes | Time (ms, springrep / cadrs) | Date, binary |
|---|---|---|---|---|---|---|---|
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
