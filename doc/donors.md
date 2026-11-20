# Donor bake-off ledger (PLAN §3.5)

The order of each op's ladder, and the measured passes it rests on. An op not listed runs springrep
first (`crates/yack/src/ladder.rs`, `CADRS_FIRST`). Each row is one bake-off: the op's family run
with each engine forced (`YACK_ENGINE` on a `--features bakeoff` build), case by case.

| Op | Order | Benchmark, family | springrep passes | cadrs passes | Only one engine passes | Time (ms, springrep / cadrs) | Date, binary |
|---|---|---|---|---|---|---|---|
| `project` | cadrs, springrep | conformance `projections`, seeds 0-3 (120 cases) | 117 (s0 28, s1 30, s2 30, s3 29) | 118 (s0 28, s1 30, s2 30, s3 30) | cadrs only: s3 `projections-efec3338c153cbee` (hidden lines of a cylinder); springrep only: none | 1514 / 851 | 2026-11-20, yack-conformance res w-m11-springrep-86 |

Notes:
- `project`: cadrs-first loses no case on any seed, gains one, and is faster on every seed. The two
  s0 failures (`hidden-lines-of-a-sphere`, `hidden-lines-of-a-box-behind-a-box`) fail on both engines.
- brep-bench grades from the STEP file and has no `project` case, so only conformance decides this row.
