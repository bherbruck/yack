# yack-cadrs import record (M1.8)

Source: cadrs `crates/cadrs_kernel` at `eb5899efab2081c76e82f2401e1462d476189493` (MIT OR Apache-2.0,
(c) 2026 Rouan van der Ende), copied by `import.py` (re-runnable; it reads the pinned commit with
`git show`). Every `.rs` file carries a provenance header naming its own changes.

## Changes to the source

- **External crates** reach `yack-shim`: one package depended on as `nalgebra`; `src/lib.rs` adds
  `extern crate nalgebra as rayon;` (and `rustc_hash`, `uuid`, `web_time`); `tess.rs` adds
  `use nalgebra::spade;` (the shim's CDT). Integration tests that name a facade get the same alias.
- **serde**: `use serde::...` lines, `Serialize`/`Deserialize` derives and `#[serde(..)]` attributes
  stripped (types.rs, naming.rs, projection.rs, construct.rs).
- **Environment**: no environment is read. `env_flag!("X")` is `false`; `std::env::var("X")` and
  `var_os("X")` became `crate::native::switch("X")` / `switch_os("X")`, which always answer
  *unset*. So every switch takes the default it has when unset: the behaviour of cadrs's own bench
  runs, which set none. The table lists all 83.
- **Crate paths** `cadrs_kernel::` → `yack_cadrs::` (doc tests, tests).
- **`pub use types::*;`** in lib.rs spelled out item by item (gate 4: no glob re-exports).
- **Test kit**: the `testkit` feature is on by default (cadrs enabled it for its integration
  tests through a dev-dependency on itself, which gate 1 refuses).
- **`as_slice()` of a fixed-size vector** (27 lines of fingerprint.rs, sweep_cache.rs, tess.rs,
  topo.rs; listed in `import.py`) reads `to_array().as_slice()`: the shim's 2/3/4-vectors keep
  named fields so `v.x` works, and those can't be viewed as a slice without `unsafe`.
- **Wall clock**: the shim's `web_time` clock stands still (elapsed is always 0), so no result
  depends on timing. The only timing that decided anything was sweep_faces.rs's give-up after
  `EFFORT_SECONDS` (4 s) at two sites (the face-splitting loop and the piece-by-piece union); both
  now count rounds against `SWEEP_CHECKS` (4096) instead. Every other `Instant` only feeds debug
  or profile output.
- **Meshing** keeps `tess.rs`; spade's constrained Delaunay triangulation is replaced by the
  shim's own (`yack-shim/src/spade.rs`): same API, different (valid) meshes.
- **Tests needing the SMLib corpus** (`tests/data/smlib`, Apache-2.0, not imported) are
  `#[ignore]`d (hlr, kernel, tess, smb unit tests); hlr's `include_bytes!` of two SMLib files
  became a run-time read so the crate compiles without them.
- Not taken: `tests/data/docs/*.body` (no licence), `blend_tool_tolblend_c9.body` (OCCT-derived),
  the SMLib corpus and the SMLib-generated suites (`smlib_*.rs`, `smlib_common/`), proptest
  (`fuzz_readers.rs`), GPU (`gpu.rs`, the `gpu` feature), `native_debug.rs` (replays on user
  documents), `exchange_corpus.rs` (runs on OCCT's LGPL dataset), and the tests reading
  excluded data. Taken data: `colored_assembly.step`, `occt_box.brep` (cadrs-authored).

## The 83 environment switches (all fixed at their unset defaults)

| Switch | Sites | Unset default kept |
|---|---|---|
| `CADRS_BLEND_DEBUG` | native/blend_corner.rs:375, native/rim_blend.rs:31, native/blend.rs:840, native/blend.rs:1307 (+9) | debug/trace/profile output off |
| `CADRS_BOOL_DEBUG` | native/boolean.rs:826, native/boolean.rs:1650, native/boolean.rs:1674, native/boolean.rs:1791 (+17) | debug/trace/profile output off |
| `CADRS_BOOL_LENIENT` | native/boolean.rs:1780 | a boolean result with crossing faces is refused (strict) |
| `CADRS_BOOL_WORK` | native/boolean.rs:4922 | debug/trace/profile output off |
| `CADRS_BREP_SAMPLES` | native/exchange.rs:674 | test only: no samples written |
| `CADRS_CHECK_COMPARE` | native/topo.rs:2229 | no cross-check of the fast validity check |
| `CADRS_DEBUG_NO_PROPAGATE` | native/boolean.rs:4574 | boolean classification propagates across faces |
| `CADRS_DEBUG_NO_RETRY` | native/boolean.rs:1708, native/boolean.rs:1733, native/boolean.rs:1749 | booleans retry at wider tolerances on open results |
| `CADRS_DEBUG_NO_SNAP` | native/boolean.rs:2845 | boolean vertex snapping on |
| `CADRS_DEFEATURE_DEBUG` | native/defeature.rs:38 | debug/trace/profile output off |
| `CADRS_DRAFT_BY_CUT` | native/local.rs:3179 | drafts by face replacement (not by cutting) |
| `CADRS_DRAFT_DEBUG` | native/local.rs:1213, native/local.rs:1620, native/local.rs:3589 | debug/trace/profile output off |
| `CADRS_DUMP_DIR` | native/kernel.rs:1409, native/kernel.rs:2436, native/kernel.rs:2482, native/offset/round.rs:438 | no failure dumps written |
| `CADRS_EBOP_DEBUG` | native/edge_bop.rs:386 | debug/trace/profile output off |
| `CADRS_EF_DEBUG` | native/boolean.rs:3186, native/boolean.rs:3326 | debug/trace/profile output off |
| `CADRS_EXCHANGE_DEBUG` | native/exchange.rs:56 | debug/trace/profile output off |
| `CADRS_EXCHANGE_FILES` | native/exchange.rs:844 | test only: the corpus test returns early |
| `CADRS_EXPORT_DIR` | native/exchange.rs:975 | test only: no samples written |
| `CADRS_FILLET_DEBUG` | native/fillet.rs:76 | debug/trace/profile output off |
| `CADRS_GEN_DUMP` | native/sweep_generate.rs:6209 | debug/trace/profile output off |
| `CADRS_GEN_EDGES` | native/sweep_generate.rs:489 | debug/trace/profile output off |
| `CADRS_GEN_ENDS` | native/sweep_generate.rs:2995 | debug/trace/profile output off |
| `CADRS_GEN_NODES` | native/sweep_generate.rs:5131 | debug/trace/profile output off |
| `CADRS_GEN_PLUG` | native/sweep_generate.rs:3877, native/sweep_generate.rs:6493 | debug/trace/profile output off |
| `CADRS_GEN_SURVEY` | native/sweep_generate.rs:6231 | debug/trace/profile output off |
| `CADRS_GEN_TRACE` | native/sweep_generate.rs:6142 | debug/trace/profile output off |
| `CADRS_GPU` | native/mass.rs:1337 | the GPU quadrature off (the feature is not built anyway) |
| `CADRS_LOCAL_FILLET_ONLY` | native/blend.rs:1295, native/blend.rs:2781, native/blend.rs:2921, native/blend.rs:2976 (+2) | the boolean blend is the fallback when the local one fails |
| `CADRS_NO_LOCAL_FILLET` | native/blend.rs:2757, native/blend.rs:2904, native/blend.rs:2936, native/blend.rs:2994 (+1) | local (SMLib-style) fillets and chamfers tried first |
| `CADRS_NO_RECOG_C` | native/xbuild.rs:1073 | B-spline curves recognised as analytic on import |
| `CADRS_NO_RECOG_S` | native/xbuild.rs:344 | B-spline surfaces recognised as analytic on import |
| `CADRS_NO_RIM_BLEND` | native/blend.rs:3446, native/fillet.rs:11411 | rim blends / rim chamfers tried |
| `CADRS_NO_SWEEP_CACHE` | native/sweep_cache.rs:79, native/sweep_cache.rs:108, native/sweep_cache.rs:281 | the sweep cache on |
| `CADRS_OCCT_DATA` | native/fillet.rs:14562 | test only: the OCCT dataset tests return early |
| `CADRS_OCCT_REBUILD` | native/brep_occt.rs:2184 | OCCT BRep files read directly when valid |
| `CADRS_OFFSET_DEBUG` | native/offset/wire.rs:1140, native/offset/wire.rs:1159, native/offset/wire.rs:1462, native/offset/round.rs:436 | debug/trace/profile output off |
| `CADRS_ORDER_DEBUG` | native/draw.rs:5108 | debug/trace/profile output off |
| `CADRS_OUTLINE_SLICED` | native/sweep_solid.rs:2657 | exact outlines (not sliced) |
| `CADRS_PAR_DEBUG` | native/par.rs:76 | debug/trace/profile output off |
| `CADRS_PCURVE_DEBUG` | native/pcurve.rs:675 | debug/trace/profile output off |
| `CADRS_PROFILE` | native/boolean.rs:44, native/kernel.rs:2462, native/sweep_solid.rs:2977, native/sweep/frames.rs:1503 (+1) | debug/trace/profile output off |
| `CADRS_SEW_REUSE` | native/surfacing.rs:1160 | sewing reuses matching edges |
| `CADRS_SEW_TRACE` | native/surfacing.rs:1223 | debug/trace/profile output off |
| `CADRS_SHELL_DEBUG` | native/surfacing.rs:658, native/offset/round.rs:876, native/offset/round.rs:942, native/offset/round.rs:1270 (+4) | debug/trace/profile output off |
| `CADRS_SLAB_DEBUG` | native/offset/slab.rs:721, native/offset/slab.rs:1127 | debug/trace/profile output off |
| `CADRS_SNAP_DEBUG` | native/boolean.rs:2889 | debug/trace/profile output off |
| `CADRS_SPLIT_DEBUG` | native/split.rs:316, native/split.rs:449, native/split.rs:476, native/split.rs:489 (+1) | debug/trace/profile output off |
| `CADRS_SSI_DEBUG` | native/ssi.rs:65, native/ssi.rs:441, native/ssi.rs:1747, native/ssi.rs:2217 (+4) | debug/trace/profile output off |
| `CADRS_SSI_DUMP` | native/ssi_numeric.rs:1712 | debug/trace/profile output off |
| `CADRS_SSI_SEEDS` | native/ssi_numeric.rs:1567 | debug/trace/profile output off |
| `CADRS_SWEEP_CLASSES` | native/sweep_moving.rs:3880 | debug/trace/profile output off |
| `CADRS_SWEEP_DEBUG` | native/sweep_pipe.rs:464, native/heal_split.rs:582, native/sweep_faces.rs:99, native/sweep_faces.rs:698 (+56) | debug/trace/profile output off |
| `CADRS_SWEEP_EFFORT` | native/sweep_faces.rs:168 | EFFORT_SECONDS (unused now: SWEEP_CHECKS rounds replace the wall-clock budget) |
| `CADRS_SWEEP_LEADS` | native/sweep_solid.rs:3607 | screw sweeps cut leads in one go |
| `CADRS_SWEEP_SCREW` | native/sweep/frames.rs:514 | screw motions recognised |
| `CADRS_SWEEP_SOLID_DEBUG` | native/sweep_solid.rs:862, native/sweep_solid.rs:984, native/sweep_solid.rs:2087, native/sweep_solid.rs:2736 (+6) | debug/trace/profile output off |
| `CADRS_SWEEP_STRETCHES` | native/sweep_moving.rs:3918 | debug/trace/profile output off |
| `CADRS_TANGENT_DEBUG` | native/boolean.rs:3539 | debug/trace/profile output off |
| `CADRS_TESS_PROFILE` | native/tess.rs:1506 | debug/trace/profile output off |
| `CADRS_TUBE_ACROSS` | native/sweep_tube.rs:2003 | debug/trace/profile output off |
| `CADRS_TUBE_AT` | native/sweep_tube.rs:4457 | debug/trace/profile output off |
| `CADRS_TUBE_CHAIN` | native/sweep_tube.rs:5213 | debug/trace/profile output off |
| `CADRS_TUBE_CORNER` | native/sweep_tube.rs:5427 | debug/trace/profile output off |
| `CADRS_TUBE_CROSS` | native/sweep_tube.rs:3872, native/sweep_tube.rs:4940 | debug/trace/profile output off |
| `CADRS_TUBE_CURVES` | native/sweep_pipe.rs:727, native/sweep_pipe.rs:748, native/sweep_tube.rs:2690, native/sweep_tube.rs:2735 (+2) | debug/trace/profile output off |
| `CADRS_TUBE_ENDS` | native/sweep_tube.rs:2661 | debug/trace/profile output off |
| `CADRS_TUBE_FIRST` | native/sweep_solid.rs:2512 | tube sweeps not forced first |
| `CADRS_TUBE_FIT` | native/sweep_tube.rs:2228 | SHEET_FIT |
| `CADRS_TUBE_FITS` | native/sweep_tube.rs:2764, native/sweep_tube.rs:3377 | debug/trace/profile output off |
| `CADRS_TUBE_MISSES` | native/sweep_tube.rs:4264 | debug/trace/profile output off |
| `CADRS_TUBE_NEAR` | native/sweep_tube.rs:1661, native/sweep_tube.rs:1750 | debug/trace/profile output off |
| `CADRS_TUBE_NODES` | native/sweep_tube.rs:5059, native/sweep_tube.rs:5153, native/sweep_tube.rs:5361, native/sweep_tube.rs:5557 | debug/trace/profile output off |
| `CADRS_TUBE_NO_RESPACE` | native/sweep_tube.rs:4234 | tube sheets respaced |
| `CADRS_TUBE_OUT` | native/sweep_tube.rs:2843 | no tube body written |
| `CADRS_TUBE_PAIR` | native/sweep_tube.rs:3796 | debug/trace/profile output off |
| `CADRS_TUBE_PROBE` | native/sweep_tube.rs:2160 | debug/trace/profile output off |
| `CADRS_TUBE_REACH` | native/sweep_tube.rs:1835 | 0.1 |
| `CADRS_TUBE_SEAM` | native/sweep_tube.rs:2039 | debug/trace/profile output off |
| `CADRS_TUBE_TIMES` | native/sweep_tube.rs:3620, native/sweep_tube.rs:3711, native/sweep_tube.rs:3893, native/sweep_tube.rs:3903 | debug/trace/profile output off |
| `CADRS_TUBE_TWINS` | native/sweep_tube.rs:3950 | debug/trace/profile output off |
| `CADRS_UNION_IN_TURN` | native/boolean.rs:5634 | many-body unions by halves (not one at a time) |
| `CADRS_USD_BREP` | native/smb.rs:1540 | test only: the usd-brep checkout test returns early |
| `CADRS_VERTEX_DEBUG` | native/boolean.rs:341 | debug/trace/profile output off |
