# M2 worklist: yack-springrep module edges vs the target map

Source: `cargo xtask modgraph --crate yack-springrep` (res/w-m02-gate-7.log, tree after M0.2).
Target map: `[modules.yack-springrep]` in layers.toml. Each top-level module maps to the crate it
becomes (PLAN §3.1) and may name only modules bound for that crate's allow-list (or siblings
bound for the same crate). `pick` has no entry (T8 dissolves it). `facade` may name everything.
Gate 5 only notes these until tag `m2` exists, then fails on them.

MEASURED: 29 violating module edges (28 between modules and 1 for the unmapped `pick`). They break
down into 126 sites: 48 live (non-test) and 78 test-only. Only live sites count for the gate. The T-numbers below are my reading of each
`use` against PLAN §3.4 (INFERRED).

## M2.1 (T1, T2, T10), everything under `boolean/`
T1: `boolean::{domain, classify, face_domain}` moves to `src/locate/`
- check.rs:11 (`classify`, `domain::Domain`), check.rs:504 (`domain::Where`)
- distance.rs:19 (`classify`, `domain::{Domain, Where}`, `face_domain`)
- rays.rs:17 (`classify::DIRECTIONS`, `domain::{Domain, Where}`)
- measure.rs:930, :941, :952, :960, :1413 (`face_domain`, `domain::Where`)
- measure/sample.rs:9 (`face_domain`)

T2: `boolean::{csi, ssi, contact, march}` moves to `src/intersect/`, and `same_domain` becomes `intersect::same`
- check.rs:11 (`csi::{curve_surface, signed_distance}`), check.rs:516 (`ssi::same_domain`)
- rays.rs:17 (`csi::curve_surface`)
- sweep/helix.rs:18 (`boolean::csi`)
- mesh/display.rs:20 (`boolean::same_domain`)

T10: `section` and `section_area` move to `boolean`
- measure/section.rs:7-8 (`boolean`, `boolean::half_space`)

## M2.2 (T3, T4, T9): `topo/`, `codec.rs`, `check.rs`
T3: a typed `MemoSlot`; the loose box goes into topo's core caches
- topo/memo.rs:33 (`boolean::FaceImages`; the same struct also references `measure::loose_bbox`)
- mesh/mod.rs:69 (`measure::loose_bbox`, which becomes a topo cache)

T4: all encoding moves to `codec`; memos are no longer encoded
- topo/codec.rs:11 (`codec::{Codec, Reader, Writer, fields}`), topo/codec.rs:119 (`boolean::FaceImages`)
- boolean/mod.rs:2936, :2942 (`codec::fields!` for `FaceImages`/`TrimImage`; memo encoding goes away)

T9: `loop_area` and `fans_at_vertices` move to `topo`
- measure.rs:1028 (`check::fans_at_vertices`)
- measure.rs:24 (`check::coedge_params`; not named in T9, so it should move to topo with the rest)
- Known temporary violation until T9 lands: locate → `check::coedge_params`. When M2.1 moves `face_domain`
  into src/locate/ it still calls `check::coedge_params`; the edge goes away once M2.2 moves that
  function into topo.

## M2.3 (T5): `step::topology::Parts` moves to `topo::build`
- blend/shaped/corner.rs:42 and blend/shaped/uneven.rs:28 (`step::{PartFace, Parts, parts_of, solid_of}`)
- edit/affine.rs:14 (`step`)

## M2.4 (T6, T7, after M2.1)
T6: `tweak` and `through` move to `src/tweak/`
- boolean/snap.rs:17 (`edit::tweak`)

T7: `boolean` → `sweep::extrude` is not a map violation (yack-boolean may use yack-build), but
PLAN still wants the slab built with `topo::build`. Look for it by hand in boolean/.

Also a real back-edge with no T-number: sweep.rs:17/122 calls `boolean::union` (build L7 →
boolean L8). It probably belongs to M2.4 (sweep/); it needs a design call (see below).

## M2.5 (T8, T11, T12): `blend.rs`, `pick.rs`, `mesh/` tests, `error.rs`
T8: `pick` is dissolved; `EdgeName`/`edges_named` move to `topo::names`; `plane_of` moves to `geom`
- blend.rs:47 (`pick::faces_named`), edit.rs:20 (`pick::convex_edges`)
- pick.rs:9-13 (all of pick: `blend::{EdgeName, has_name}`, `boolean`, ...)

T11: tests only; the gate does not count them. Move them to crates/yack/tests/
- mesh/display.rs:315, :316, :357, :393
- mesh/solid.rs:127
- mesh/tests.rs:8, :10, :11, :152, :174, :261, :280, :282, :286, :308, :359, :478
- other modules' tests reach upward the same way: boolean/tests.rs, codec/tests.rs, measure/sample.rs,
  overlap.rs, probe.rs, rays.rs, step/tests.rs, sweep/helix/tests.rs, edit/tweak/tests.rs,
  topo/pcurve.rs:208. These are not gate violations, but they also have to move once the modules
  become crates.

T12: error.rs:7 doc link `[`crate::check`]`. The lexer drops doc comments, so modgraph does not
list it; fix it by hand.

## Not covered by T1-T12: layers.toml gaps or extra moves (lead's decision)
- distance.rs:27, rays.rs:26 → `mesh::bvh`. §3.1 puts the BVH in `math` (from `mesh/bvh.rs`), so move
  mesh/bvh.rs into math.
- distance.rs:266 → `measure::face_box` (locate → measure; both are layer 5)
- mesh/solid.rs:10 → `check` (mesh → check; both are layer 6)
- edit.rs:21 → `profile`: yack-edit's allow-list lacks yack-profile. Widen it?
- overlap.rs:24, probe.rs:205 → `tol`; probe.rs:24 → `numeric`: yack-view lacks tol/numeric. Widen it?
- step/adjust.rs:20, step/heal.rs:25, step/write.rs:16 → `numeric::solve_dense`: yack-step and
  yack-heal lack yack-numeric. Widen them, or move solve_dense callers?
- profile.rs:541 → `topo::FaceName` (profile → topo; both are layer 4). Move FaceName lower, or
  drop the named-circle helper?
- tol.rs:41 → `math::Point3` (`tol::within` takes a point; tol is layer 0). Pass the magnitude
  instead.
- sweep.rs:17 → `boolean` (`boolean::union` at :122; build must never union pieces, per §3.1).

## Rulings (lead, 2026-11-20)
- mesh/bvh.rs moves to math, since a spatial index is generic geometry. This fixes distance.rs:27 and rays.rs:26.
- The allow-lists widen: edit gets profile, view gets tol and numeric, step and heal get numeric. All of these point downward.
- profile → `topo::FaceName`: the naming types (FaceName etc.) move down into yack-base as plain data.
- tol → `math::Point3`: tol must not depend on math. The fns that take a Point3 move from tol into
  math, and tol keeps only constants and scalar compares.
- sweep.rs:122 `boolean::union`: sweep returns its pieces, and the caller sequences the union (the
  yack facade may sequence calls; that is routing, not computing).
- locate → `measure::face_box`: face_box moves down into geom.
- mesh → check: mesh may use check only in `#[cfg(test)]`. Any live use moves to the caller.
