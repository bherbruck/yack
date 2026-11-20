# yack plan

yack (Yet Another CAD Kernel) is an exact B-rep kernel written in pure, safe Rust under the MIT
licence. This file is the single authoritative plan. It merges three architect plans:

- A, "refactor springrep": `yack-ref/plans/A-refactor.md`
- B, "ideal layering": `yack-ref/plans/B-layered.md`
- C, "API and tests first": `yack-ref/plans/C-apifirst.md`

`plan/DECISIONS.md` records each conflict between them and how it was settled. Where this file and
an older plan disagree, this file wins. Changing it needs the lead's approval.

- **Paths.** `yack-ref/` means `/home/brenn/github/yack-ref`. Donor revisions are pinned in §9.
- **Milestones.** M0 to M7 are milestones. Ids like M1.3 are work packages, each sized for one
  coding agent.

---

## 1. Goal and scoreboard

There are two benchmarks, and yack must beat every donor kernel on both.

| Benchmark | Metric | Best donor today | yack target (by M6) |
|---|---|---|---|
| brep-conformance (`yack-ref/brep-conformance`, suite 1.0.0) | pass rate at 1e-7, per seed | springrep 1.0.1: 100% on seed 0, about 97.5% on seeds 1–3 (3,521/3,609 on seed 1). cadrs: 2,930/3,609 on seed 0 | **100% on seeds 0, 1, 2 and 3**, 0 crashes, 0 timeouts |
| | total `time_ms` summed over a seed | springrep seed 1: 1,481 s | **≤ 0.8×** springrep, same machine, same run |
| brep-bench (`yack-ref/brep-bench`, SPEC 1.17, 506 cases, graded from our STEP file) | pass count | cadrs 504; next.BREP 482; OCCT 412; cad-kernel 393; solidcraft 361 (39 wrong); geomsolver 340; springrep 333 (164 refused, 9 wrong locally). Live brepbench.org, 2026-11-20 | **≥ 505/506** |
| | wrong count | cadrs 0 | **0** |
| | mean lines of code per case program (the harness's `highlight.loc`, comments excluded) | about 12–17 across the donors (local count: solidcraft 12, cadrs 13, springrep 14) | **≤ 10 mean, ≤ 25 max**, 0 workarounds |

The results files committed in brep-bench and `analysis/brep-bench.md` are **stale**. Only a live
board read or our own local run counts as a baseline.

Where springrep leaves points on the table locally, out of 173 non-passes: swept 52, edit 33,
offset 30, shell 22, fillet 8, chamfer 6, thicken 4, and a few each in draft, loft, nurbs, sweep,
cnc, fill, geometry and integration. cadrs passes nearly all of them, so it is the primary donor
for swept, edit, offset, shell, chamfer and draft.

---

## 2. Rules (non-negotiable)

**R1. High cohesion, low coupling: a strict crate DAG, enforced mechanically.**

`layers.toml` gives every crate a layer, an owner and an allow-list. A crate depends only on
allow-listed crates in a strictly lower layer. `cargo xtask gate` fails on any violation (§6.6).

**R2. Exactly one public crate, `yack`.** Every other crate has `publish = false`. No `yack`
signature names an internal type.

**R3. The API is extremely obvious.**

The conformance adapter is name for name (one `yack` call plus form translation per protocol
function), and a brep-bench case program is a short, literal reading of the case text.

**R4. Zero external dependencies in shipped crates** (`yack` and everything it depends on, on
every target and feature).

Tool crates are zero-dependency too. The single exception is `yack-conformance`, which uses
`brep-protocol` (MIT) and is never a dependency of anything.

**R5. Layout.** The root `Cargo.toml` is a virtual workspace with `members = ["crates/*"]` and
`resolver = "3"`. Every crate is `crates/<name>`.

**R6. Licences: MIT-compatible code only.**

| Donor | What may be used |
|---|---|
| springrep | Code (MIT) |
| cadrs | Code (MIT OR Apache-2.0, taken as MIT) |
| cad-kernel / geomsolver | Code (MIT) |
| solidcore | Only `crates/ssi` and `crates/nurbs-grid-mesh` (MIT OR Apache-2.0) |
| solidcraft | The user's own code, but **never** `vendor/truck` |
| next.BREP | **Ideas only.** Nobody opens its source (`yack-ref/next.BREP.io_RUST_BREP_KERNEL`). Agents work from `analysis/next-brep.md` |

Every adapted file carries a provenance header (§9).

**R7. One working tree and one target directory (`bin/`).**

`.cargo/config.toml` sets `target-dir = "bin"`. No worktrees, clones, `CARGO_TARGET_DIR` or
per-agent `target/`. Swarms follow one-tree-swarm: one builder, a frozen-base batch branch,
disjoint file claims (`AGENTS.md`).

**R8. Every result is valid or the call errors.** The checker runs on every operation's result.
A refusal is better than a wrong solid. This is springrep's contract, and it is the reason it
scores 100%.

**R9. Determinism.**

No `HashMap`/`HashSet` iteration, env vars, globals, `thread_local!` or wall-clock time in kernel
crates. The ordered `par_map` gives results identical to a serial run.

---

## 3. Crate DAG

### 3.1 Final DAG (after M5)

There are 25 internal crates plus `yack`. All are prefixed `yack-` except `yack` itself.

- **L** is the layer. A crate may use only lower layers.
- **Allowed deps** lists internal crates and is final once M3 ends. Until then the M3 module-graph
  tool can tighten it but never widen it.

| L | Crate | Responsibility (one) | Allowed deps | Donor |
|---|---|---|---|---|
| 0 | `base` | `Error`, `ErrorKind`, `Stage`, `Issue`; `Budget` (deterministic work units); `Limits` (allocation caps); ordered `par_map`; `guard()` panic firewall helper | – | springrep `error.rs`; cadrs `KernelError` stages; cad-kernel `util/{par,meter}.rs`; solidcraft taxonomy |
| 0 | `tol` | Tolerance policy: `LINEAR = 1e-7` mm, `ANGULAR`, `TANGENT`, `RESOLVED`, `PICK = 10·LINEAR`, the named round-off ladder, `between`, `within`, and the ordered "same feature" rulebook | – | springrep `tol.rs`; cadrs `tol.rs` ladder; cad-kernel `tolerant.rs` rulebook |
| 0 | `ssi` | Certified free-form surface/surface intersection on `Patch` (plain data) | – | solidcore `crates/ssi` **as is** (M6) |
| 0 | `gridmesh` | Grid mesher for dense trimmed NURBS | – | solidcore `crates/nurbs-grid-mesh` **as is** (M6) |
| 1 | `numeric` | Quadrature, banded/cyclic solves, QR/SVD/LU, roots, intervals, certified root isolation (Krawczyk), Shewchuk predicates, deterministic `fmath` | base, tol | springrep `numeric.rs`, `mesh/predicates.rs`; cad-kernel `util/{interval,isolate,linalg,expansion,predicates,fmath}` (keep the musl/FreeBSD notice) |
| 1 | `p21` | ISO 10303-21 lexer, parser and writer into a neutral entity graph, with `Limits`. No geometry types | base | springrep `step/part21.rs`; cadrs `step.rs` complex-entity and escape handling |
| 2 | `math` | `Vec2/3`, `Point2/3`, `Dir3`, `Frame`, `Rigid`, `Similarity`, `Affine`, `Aabb`, `Angle`, BVH | numeric, tol, base | springrep `math.rs`, `mesh/bvh.rs`; cadrs `bvh.rs` |
| 3 | `geom` | Analytic-first `Curve`, `Surface`, `Curve2`; one rational B-spline type; eval, derivatives, projection, periods; `Mapped` affine images; analytic recognition; `Enclose` | math, numeric, tol, base | springrep `geom/`; cadrs `nurbs.rs` checked constructors, `recognize.rs`; cad-kernel `affine.rs`, `enclose.rs` design |
| 4 | `intersect` | CCI, CSI and SSI with contact classification, giving a plain `Meeting`: closed forms first, then swept-family, then marching, then certified free-form via `ssi` | geom, ssi, math, numeric, tol, base | springrep `boolean/{csi,ssi,ssi/*,contact,march}`; cadrs `ssi_graph`, `ssi_cci`, `ssi_coincident`, `ssi_numeric`; next.BREP tangent lane (idea) |
| 4 | `profile` | 2-D segments, loops and profiles, with validation, segment labels and exact 2-D region booleans | geom, math, numeric, tol, base | springrep `profile/`; brep-protocol `profile.rs` Gn formulas; solidcraft `Region2` |
| 4 | `topo` | `Brep` arena, typed ids, pcurves, seams, contacts, opaque `Memo` slots, `build::Parts` (the only builder), names/roles, `History` data, similarity maps | geom, math, tol, base | springrep `topo/` + `step/topology.rs::{Parts,solid_of,rebuilt}` + `blend::EdgeName`; solidcraft `history.rs` |
| 5 | `locate` | Point-in-face, point classification (in/on/out), rays, closest point, distance, entity at a point | topo, intersect, geom, math, numeric, tol, base | springrep `boolean/{domain,classify}.rs`, `rays.rs`, `distance.rs`; cadrs `distance.rs` |
| 5 | `measure` | Exact integrals: volume, area, centroid, inertia, tight boxes, Betti numbers, normal and curvature | topo, intersect, geom, math, numeric, tol, base | springrep `measure.rs`; cadrs `mass.rs` (degree-matched Gauss rules); cad-kernel `props.rs` uv gap fix |
| 5 | `codec` | Exact, versioned, capped binary format plus `digest()` | topo, geom, math, base | springrep `codec.rs` (`fields!`); solidcraft `docs/native-brep.md` as the spec |
| 6 | `check` | Validity contract: incidence, closure, geometric agreement, orientation, nesting, face–face crossing (BVH) | topo, locate, measure, intersect, geom, math, tol, base | springrep `check.rs`; cadrs `check_faces.rs` |
| 6 | `mesh` | Watertight tessellation (CDT in metric uv), display mesh with topology ids, tess plan | topo, locate, gridmesh, geom, numeric, math, tol, base | springrep `mesh/`; solidcraft `TessPlan` + mesh topology fields |
| 7 | `tweak` | Moves faces onto offset or moved surfaces and re-intersects their neighbours. No booleans | topo, intersect, locate, check, geom, math, tol, base | springrep `edit/tweak.rs`, `tweak/through.rs` |
| 7 | `heal` | Sewing loose faces, the vertex pool, gap and same-parameter fixes, Gauss-Newton adjust | topo, locate, check, intersect, geom, math, tol, base | springrep `step/{heal,adjust}.rs`, `blend/sew.rs`; cadrs `heal.rs`; cad-kernel `sew.rs` |
| 7 | `build` | Primitives, extrude, revolve, screw, loft, loft_through, sweep, sweep_path, slab, guided loft, NURBS-profile segments. Never unions pieces | profile, topo, check, locate, measure, intersect, geom, numeric, math, tol, base | springrep `primitive*`, `sweep*`, `loft/`; cadrs `loft_build`, `curve_loft`, `sweep_planar`, `sweep_pipe` |
| 8 | `boolean` | Regularised set operations: strategy table (same-solid → prism-2D → analytic → general), imprint pipeline, n-ary, split, section, unify, snap, near-pair culling | build, tweak, heal, check, mesh, measure, locate, intersect, profile, topo, geom, numeric, math, tol, base | springrep `boolean/{mod,arrange,sew,same,snap,near,simplify,unify,hulls,plane}`; cad-kernel `complex.rs` (n-ary design) |
| 9 | `blend` | Fillets and chamfers of every kind, edge chains, corners, setbacks, tool-solid sewing | boolean, build, tweak, heal, check, measure, locate, intersect, profile, topo, geom, numeric, math, tol, base | springrep `blend.rs` + `blend/`; cadrs `fillet*`, `blend*`, `convex_chamfer`, `rim_blend`, `face_blend` |
| 9 | `swept` | Sweep of a **solid** along a motion (slide, turn, screw, along a curve with a frame law), giving the exact envelope | boolean, build, intersect, locate, check, topo, geom, numeric, math, tol, base | cadrs `sweep_moving`, `sweep_solid`, `sweep_tube`, `sweep_ball_pass`, `sweep_mill`; cad-kernel `swept/` closed forms |
| 10 | `edit` | offset, offset_faces, delete_faces, move_faces, thicken, shell, draft, affine | blend, swept, boolean, build, tweak, heal, check, measure, locate, intersect, topo, geom, numeric, math, tol, base | springrep `edit/`; cadrs `offset/`, `local.rs`, `defeature.rs`, `surfacing.rs`; cad-kernel offset-by-definition |
| 10 | `step` | Maps between the `p21` graph and `Brep`: AP203/214/242, units, assemblies, the writer | p21, heal, boolean, check, mesh, topo, geom, math, tol, base | springrep `step/{read,write,geometry}`; cadrs `xbuild.rs`; cad-kernel `step_check.rs` (as a test only) |
| 10 | `view` | Hidden-line projection, section polylines, interference, thin-place probe | boolean, locate, measure, mesh, intersect, topo, geom, math, base | springrep `draw.rs`, `overlap.rs`, `probe.rs`; cadrs `hlr.rs`, `silhouette.rs` |
| 11 | **`yack`** | **The only public crate**: the API in §4, the error mapping, the `catch_unwind` firewall, stamped ids | all of the above | new |

Tool crates have `publish = false`, sit outside the kernel DAG, and nothing in the kernel may
depend on them:

| Crate | Purpose | Deps |
|---|---|---|
| `yack-conformance` | brep-conformance adapter binary | `yack`, `brep-protocol` (git, pinned) |
| `yack-bench` | brep-bench runner plus one program per case | `yack` only |
| `yack-testkit` | Oracles, the `audit()` invariants, metamorphic helpers, seeded generator, Halton membership sampler | `yack` only |
| `xtask` | Gates, conformance and bench drivers, the ratchets | std only (hand-written JSON reader) |

### 3.2 Temporary crates (deleted by plan)

| Crate | Lives | Contents | Deleted when |
|---|---|---|---|
| `yack-kernel` | M1–M4 | springrep `src/` at `a03e8a1` (v1.0.1), copied verbatim, then untangled in place | M4 exit: every module extracted into the §3.1 crates |
| `yack-cadrs` | M2–M5 | cadrs `crates/cadrs_kernel/src` at `eb5899e`. Its external dependencies are replaced by in-crate shims: `na` (the nalgebra subset it uses), `cdt` (springrep's CDT, replacing spade), sequential `par` (replacing rayon), counters (replacing uuid), `BTreeMap` (replacing rustc-hash), and no serde. Every `CADRS_*` env switch is frozen to its default | M5 exit: every op it serves has been re-hosted into a §3.1 crate on `yack-topo` |
| (in `yack`) `bridge.rs`, `route.rs` | M2–M5 | Exact cadrs `Body` ↔ `Brep` conversion, and the per-op fallback table | M5 exit, together with `yack-cadrs` |

`layers.toml` puts both temporary crates in layer 10, with no internal deps. The gate also
enforces these rules on them:

- Only `yack` may depend on them.
- `yack-cadrs` LOC may only fall from M4 onward (a ratchet).
- Inside `yack`, only `route.rs` and `bridge.rs` may name `yack_cadrs`.

### 3.3 Interfaces at crate boundaries

| Boundary | Interface |
|---|---|
| Plain data everywhere | No glob re-exports. No `pub` item that exists only so a sibling crate can reach in. Each crate's surface is snapshotted in `crates/<c>/api.txt` |
| `geom` → `intersect` | Inputs are `&Curve`, `&Surface`, `UvBox` and `TBox`. The output, `Meeting { branches, points, same }`, carries no topology ids |
| `intersect` ↔ `ssi` | Only `intersect::freeform` names `ssi` (gate-checked) |
| `topo` → queries | Read-only `&Brep` accessors plus `Memo<T: MemoSlot>` slots. Each client defines its own slot type, and `topo` never names one. Codecs never encode memos |
| ops → ops | One shape: `fn op(inputs, &Params, &Ctx) -> Result<Made>`, with `Made { brep, history }`. Every op ends in `finish(made, ctx)`, which runs `check` and turns any issue into `Internal { stage: Check }` |
| kernel → `yack` | `yack` wraps `Made` in `Solid`. Only the value types in `layers.toml`'s `reexport` list (`math`: `Point2/3`, `Vec3`, `Dir3`, `Frame`, `Angle`, `Aabb`, `Rigid`, `Similarity`, `Affine`) are re-exported as they are |

### 3.4 springrep's back-edges to cut (M3)

| # | Back-edge | Fix |
|---|---|---|
| T1 | `boolean/{domain,classify}` is used by check, measure, mesh, rays, distance and topo | Move to `src/locate/` |
| T2 | `boolean/{csi,ssi,ssi/*,contact,march}` is used by check, tweak, sweep/helix and rays | Move to `src/intersect/`. `same_domain` becomes `intersect::same` |
| T3 | `topo/memo.rs` names `boolean::FaceImages` and `measure::loose_bbox` | Typed `MemoSlot` extension. The loose box goes into topo's core caches |
| T4 | `topo/codec.rs` ↔ `codec` | All encoding moves into `codec`. Memos are no longer encoded |
| T5 | blend and `edit::affine` use `step::topology::Parts` | Move it to `topo::build` |
| T6 | `boolean/snap.rs` → `edit::tweak` | Move `tweak` and `through` to `src/tweak/` (layer 7) |
| T7 | `boolean` → `sweep::extrude` for the half-space slab | Build it with `topo::build` |
| T8 | `blend` ↔ `pick` ↔ `edit` | `EdgeName` and `edges_named` move to `topo::names`. `pick` is dissolved. `plane_of` moves to `geom` |
| T9 | `check` ↔ `measure` | `loop_area` and `fans_at_vertices` move to `topo` |
| T10 | `measure` → `boolean` (section) | `section` and `section_area` move to `boolean` (to `locate` as plane SSI later) |
| T11 | `mesh` → `boolean`, `blend`, `loft`, `primitive` (tests only) | Move those tests to `crates/yack/tests/` |
| T12 | `error` → `check` (a doc link) | Delete the link |

These god files are split during M4. The file cap is 1,500 lines, with an allowlist that may only
shrink:

| File | Split into |
|---|---|
| `blend.rs` (3,464 lines) | `blend/{api,pick,section,ends,tools,corners}` |
| `boolean/mod.rs` (2,942 lines) | `api`, `nary`, `work/{pierce,cross,graze,faces,place,split,dedup,select,classify}` |
| `csi.rs` (2,537 lines) | `csi/{line,conic,quartic,sampled,graze}` |
| `nurbs.rs` (2,552 lines) | `nurbs/{curve,surface,knots,project}` |

---

## 4. Core types and the public API

### 4.1 Internal core types

```rust
// yack-base
#[non_exhaustive]
pub enum ErrorKind { Invalid, NotFound, Ambiguous, ForeignId, Unsupported, Failed, Budget, Internal }
pub enum Stage { Validate, Intersect, Split, Classify, Sew, Fit, Heal, Parse, Check, Mesh }
pub struct Error {
    pub kind: ErrorKind,
    pub op: &'static str,           // "fillet"
    pub arg: Option<&'static str>,  // "radius": the parameter at fault, as named in the API
    pub stage: Option<Stage>,       // where a Failed/Budget/Internal happened
    pub detail: String,             // what was wrong, with numbers
}
pub type Result<T> = core::result::Result<T, Error>;
pub struct Budget { /* work units, never wall time */ }  impl Budget { pub fn spend(&self, n: u64, s: Stage) -> Result<()>; }
pub struct Ctx<'a> { pub budget: &'a Budget, pub limits: &'a Limits }
pub fn par_map<T: Sync, R: Send>(items: &[T], f: impl Fn(&T) -> R + Sync) -> Vec<R>; // ordered

// yack-tol
pub const LINEAR: f64 = 1e-7;   pub const MODEL_SIZE: f64 = 1000.0;  pub const ANGULAR: f64 = LINEAR / MODEL_SIZE;
pub const RESOLVED: f64 = 0.1;  pub const TANGENT: f64 = LINEAR / RESOLVED;  pub const PICK: f64 = 10.0 * LINEAR;
pub fn between(a: f64, b: f64) -> f64;  pub fn within(d: f64, t: f64, magnitude: f64) -> bool;

// yack-geom: closed, analytic-first enums
pub enum Curve { Line(Line), Circle(Circle), Ellipse(Ellipse), Nurbs(Arc<NurbsCurve>), Intersection(Arc<IntersectionCurve>) }
pub enum Surface { Plane(Plane), Cylinder(Cylinder), Cone(Cone), Sphere(Sphere), Torus(Torus),
                   Extrusion(Arc<Extrusion>), Revolution(Arc<Revolution>), Ruled(Arc<Ruled>),
                   Nurbs(Arc<NurbsSurface>), Mapped(Arc<Mapped>) }

// yack-topo: springrep's arena, append-only Vecs, u32 typed ids, BTreeMap only
pub struct Brep { /* private */ }
pub trait MemoSlot: Any + Send + Sync {}
impl Brep { pub fn memo<T: MemoSlot>(&self, f: FaceId) -> Option<Arc<T>>; pub fn set_memo<T: MemoSlot>(&self, f: FaceId, v: T); }
pub struct Made { pub brep: Brep, pub history: History }
pub enum Change { Kept, Modified, Split, Merged, Generated }
pub enum FaceOrigin { Operand { operand: u8, face: FaceId }, Segment { label: Label, end: Option<CapEnd> },
                      Edge { operand: u8, edge: EdgeId }, Vertex { operand: u8, vertex: VertexId }, Offset { operand: u8, face: FaceId } }
pub struct History { /* faces: (FaceId, Change, Vec<FaceOrigin>) sorted by id; deleted inputs */ }
impl History { pub fn compose(&self, earlier: &History) -> History; }
```

### 4.2 Public API principles

1. **Solids are immutable values.** `Solid(Arc<Inner>)` is `Clone`, `Send` and `Sync`, and it is
   either valid or empty. It is a solid *set*: its pieces may touch.
2. **Where a function lives.**
   - Making a solid from nothing is a free function in `yack::make`.
   - Anything that takes a solid is a method on `Solid`.
   - Exchange is in `yack::step` and `yack::codec`.
   - There are no `_with` variants and no hidden defaults.
3. **Names match the protocol.** Every brep-conformance function has a `yack` item of the same
   name, and adapter code is the mapping in §5.1. One exception is forced by Rust: `box` becomes
   `make::block`. Two are documented spelling choices: `normal_at`/`curvature_at`, and
   `transform_affine` for `affine`.
4. **Units.**
   - Lengths are `f64` in millimetres everywhere. They are documented, and `tol::LINEAR` is in mm.
   - Angles are `Angle`, built only through `deg()` or `rad()`. There is no `From<f64>`.
   - A signed length keeps the protocol's sign convention, with a positive value growing the solid.
     The convention is documented on each method.
5. **Frames are explicit.** Every constructor takes `&Frame`. `Frame::new(origin, x, y, z)` checks
   that the axes are unit, orthogonal and right-handed, and refuses (naming the axis) instead of
   repairing.
6. **Ids are typed and stamped.**
   - `FaceId`, `EdgeId` and `VertexId` are `{stamp, u32}`. An id from another solid gives
     `ForeignId`.
   - Selection is a query, then an action. Ids come from `edge_at`/`face_at` (within `tol::PICK`),
     from walks, or from predicates.
   - A point near two entities gives `Ambiguous`. yack never chooses.
7. **Errors name the operation, the argument, the value and the rule.**
   - For example: `fillet: radius 6 mm does not fit: the ball leaves face 7 at (10, 0, 15) by 1.2 mm`.
   - Input validation runs first in every op (solidcore lost 34 conformance cases by skipping it).
8. **Every result has passed the checker (R8).**
   - A checker rejection or a caught panic is `Internal`. The adapter reports it as a crash, so the
     score is never flattered.
   - `catch_unwind` exists only in `yack`.

### 4.3 Signatures

These are frozen at M0.3. They change only through the API owner (§8). Additions requested by the
brep-bench census (M1.9) land as stubs that return `Unsupported`.

```rust
// ---- yack::prelude::* (what every case program imports) ----
pub use crate::{Solid, Frame, Point2, Point3, Vec3, Dir3, Angle, Plane, Ray, Rigid, Similarity, Affine,
                Profile, Loop, Segment, PathPiece, Spline3, Motion, FrameLaw, Side, Error, ErrorKind, Result};
pub use crate::make::*;
pub fn p2(x: f64, y: f64) -> Point2;  pub fn p3(x: f64, y: f64, z: f64) -> Point3;  pub fn v3(x: f64, y: f64, z: f64) -> Vec3;
pub fn deg(d: f64) -> Angle;  pub fn rad(r: f64) -> Angle;

// ---- values ----
impl Frame { pub const WORLD: Frame; pub fn new(origin: Point3, x: Vec3, y: Vec3, z: Vec3) -> Result<Frame>;
             pub fn at(origin: Point3) -> Frame; pub fn moved(&self, m: &Rigid) -> Frame; }
impl Plane { pub fn new(origin: Point3, normal: Vec3) -> Result<Plane>; }
impl Rigid { pub fn from_rows(m: [[f64; 4]; 3]) -> Result<Rigid>; pub fn translation(v: Vec3) -> Rigid;
             pub fn rotation(axis_point: Point3, axis: Vec3, by: Angle) -> Result<Rigid>; }
impl Similarity { pub fn from_rows(m: [[f64; 4]; 3]) -> Result<Similarity>; }   // orthogonal × c, mirrors allowed
impl Affine     { pub fn from_rows(m: [[f64; 4]; 3]) -> Result<Affine>; }       // det ≠ 0

// ---- 2-D input ----
pub enum Segment {
    Line { a: Point2, b: Point2 },
    Arc { center: Point2, radius: f64, start: Angle, sweep: Angle },        // sweep > 0 is CCW
    Conic { a: Point2, apex: Point2, b: Point2, weight: f64 },
    Blend { a: Point2, apex: Point2, b: Point2, order: u32, weight: f64 },
    Nurbs { degree: u32, knots: Vec<f64>, ctrl: Vec<Point2>, weights: Option<Vec<f64>> },
    PeriodicSpline { degree: u32, knots: Vec<f64>, ctrl: Vec<Point2> },     // the whole loop
}
impl Loop    { pub fn new(segments: Vec<Segment>) -> Result<Loop>; pub fn polygon(pts: &[Point2]) -> Result<Loop>; }
impl Profile { pub fn new(outer: Loop, holes: Vec<Loop>) -> Result<Profile>;  // outer CCW, holes CW; refuses crossings, naming segments
               pub fn rectangle(min: Point2, max: Point2) -> Result<Profile>;
               pub fn circle(center: Point2, radius: f64) -> Result<Profile>;
               pub fn polygon(pts: &[Point2]) -> Result<Profile>; }
pub enum PathPiece { Line { length: f64 }, Arc { radius: f64, angle: Angle } }
impl Spline3 { pub fn new(degree: u32, knots: Vec<f64>, ctrl: Vec<Point3>) -> Result<Spline3>;
               pub fn bezier(ctrl: &[Point3]) -> Result<Spline3>; }
pub enum FrameLaw { RotationMinimising, Fixed, Toward(Vec3) }
pub enum Motion {
    Slide { by: Vec3 },
    Turn { axis_point: Point3, axis: Vec3, angle: Angle },
    Screw { axis_point: Point3, axis: Vec3, angle: Angle, rise_per_turn: f64 },
    Along { path: Spline3, law: FrameLaw },
    Then(Vec<Motion>),
}

// ---- yack::make ----
pub mod make {
    pub fn block(frame: &Frame, size: [f64; 3]) -> Result<Solid>;                  // protocol `box`
    pub fn cylinder(frame: &Frame, radius: f64, height: f64) -> Result<Solid>;
    pub fn sphere(frame: &Frame, radius: f64) -> Result<Solid>;
    pub fn torus(frame: &Frame, major: f64, minor: f64) -> Result<Solid>;
    pub fn cone(frame: &Frame, bottom: f64, top: f64, height: f64) -> Result<Solid>;
    pub fn extrude(frame: &Frame, profile: &Profile, height: f64) -> Result<Solid>;
    pub fn revolve(frame: &Frame, profile: &Profile, angle: Angle) -> Result<Solid>;
    pub fn screw(frame: &Frame, profile: &Profile, rise: f64, angle: Angle) -> Result<Solid>;  // rise per radian
    pub fn loft(frame: &Frame, profile: &Profile, scale: f64, height: f64, top: &Profile) -> Result<Solid>;
    pub fn loft_through(frame: &Frame, sections: &[(f64, Profile)]) -> Result<Solid>;
    pub fn loft_guided(sections: &[(Frame, Profile)], guides: &[Spline3]) -> Result<Solid>;  // brep-bench loft/
    pub fn sweep(frame: &Frame, profile: &Profile, path: &[PathPiece]) -> Result<Solid>;
    pub fn sweep_path(frame: &Frame, profile: &Profile, path: &Spline3, twist: Angle) -> Result<Solid>;
    pub fn slab(frame: &Frame, size: [f64; 2], knots_u: &[f64], knots_v: &[f64], heights: &[Vec<f64>]) -> Result<Solid>;
    pub fn nurbs_solid(frame: &Frame, degree: [u32; 2], knots: [&[f64]; 2], ctrl: &[Vec<Point3>],
                       weights: Option<&[Vec<f64>]>) -> Result<Solid>;      // closed NURBS face(s); brep-bench nurbs/
    pub fn empty() -> Solid;
}

// ---- Solid ----
impl Solid {
    // booleans
    pub fn union(&self, other: &Solid) -> Result<Solid>;
    pub fn intersection(&self, other: &Solid) -> Result<Solid>;
    pub fn difference(&self, tool: &Solid) -> Result<Solid>;
    pub fn union_all(solids: &[Solid]) -> Result<Solid>;
    pub fn difference_all(&self, tools: &[Solid]) -> Result<Solid>;
    pub fn split(&self, plane: &Plane) -> Result<Split>;                      // Split { front, back }; front = normal side
    // blends: one method per protocol function
    pub fn fillet(&self, edges: &[EdgeId], radius: f64) -> Result<Solid>;
    pub fn fillet_radii(&self, edges: &[(EdgeId, f64)]) -> Result<Solid>;
    pub fn fillet_variable(&self, edge: EdgeId, from: Point3, r_from: f64, r_to: f64) -> Result<Solid>;
    pub fn fillet_conic(&self, edges: &[EdgeId], setback: f64, rho: f64) -> Result<Solid>;
    pub fn fillet_g2(&self, edges: &[EdgeId], offset: f64, center: f64) -> Result<Solid>;
    pub fn fillet_setback(&self, edges: &[EdgeId], radius: f64, setback: f64) -> Result<Solid>;
    pub fn fillet_setback_g2(&self, edges: &[EdgeId], radius: f64, setback: f64) -> Result<Solid>;
    pub fn fillet_faces(&self, a: FaceId, b: FaceId, radius: f64) -> Result<Solid>;
    pub fn full_round(&self, side_a: FaceId, middle: FaceId, side_b: FaceId) -> Result<Solid>;
    pub fn chamfer(&self, edges: &[EdgeId], distance: f64) -> Result<Solid>;
    pub fn chamfer_two(&self, edge: EdgeId, face: FaceId, d1: f64, d2: f64) -> Result<Solid>;
    pub fn chamfer_distances(&self, edges: &[(EdgeId, f64)]) -> Result<Solid>;
    // edits (signed distances: + grows)
    pub fn offset(&self, distance: f64) -> Result<Solid>;
    pub fn offset_faces(&self, faces: &[FaceId], distance: f64) -> Result<Solid>;
    pub fn delete_faces(&self, faces: &[FaceId]) -> Result<Solid>;
    pub fn move_faces(&self, faces: &[FaceId], by: &Rigid) -> Result<Solid>;
    pub fn thicken(&self, faces: &[FaceId], thickness: f64) -> Result<Solid>;
    pub fn shell(&self, open: &[FaceId], thickness: f64) -> Result<Solid>;
    pub fn draft(&self, faces: &[FaceId], origin: Point3, pull: Vec3, angle: Angle) -> Result<Solid>;
    pub fn transform(&self, m: &Similarity) -> Result<Solid>;
    pub fn transform_affine(&self, m: &Affine) -> Result<Solid>;             // protocol `affine`
    pub fn swept(&self, motion: &Motion) -> Result<Solid>;                   // every point the solid passes through
    // selection and topology
    pub fn faces(&self) -> Vec<FaceId>;  pub fn edges(&self) -> Vec<EdgeId>;  pub fn vertices(&self) -> Vec<VertexId>;
    pub fn face_at(&self, p: Point3) -> Result<FaceId>;   pub fn edge_at(&self, p: Point3) -> Result<EdgeId>;
    pub fn faces_where(&self, keep: impl Fn(&FaceRef) -> bool) -> Vec<FaceId>;
    pub fn edges_where(&self, keep: impl Fn(&EdgeRef) -> bool) -> Vec<EdgeId>;
    pub fn face(&self, id: FaceId) -> Result<FaceRef<'_>>;  pub fn edge(&self, id: EdgeId) -> Result<EdgeRef<'_>>;
    pub fn edges_of(&self, f: FaceId) -> Result<Vec<EdgeId>>;  pub fn faces_of(&self, e: EdgeId) -> Result<Vec<FaceId>>;
    pub fn edges_between(&self, a: FaceId, b: FaceId) -> Result<Vec<EdgeId>>;
    pub fn history(&self) -> &History;  pub fn digest(&self) -> [u8; 32];  pub fn is_empty(&self) -> bool;
    // queries (cannot fail on a valid solid unless they take a tolerance or build geometry)
    pub fn volume(&self) -> f64;  pub fn area(&self) -> f64;  pub fn centroid(&self) -> Option<Point3>;
    pub fn inertia(&self) -> [[f64; 3]; 3];  pub fn bounding_box(&self) -> Option<Aabb>;  pub fn betti(&self) -> [u64; 3];
    pub fn check(&self) -> Vec<Issue>;                                        // empty for anything yack built
    pub fn classify(&self, pts: &[Point3]) -> Vec<Side>;                      // In | On | Out
    pub fn closest(&self, pts: &[Point3]) -> Vec<Point3>;
    pub fn normal_at(&self, pts: &[Point3]) -> Vec<Option<Vec3>>;             // None at a crease
    pub fn curvature_at(&self, pts: &[Point3]) -> Vec<Option<[f64; 2]>>;
    pub fn distance(&self, other: &Solid) -> f64;
    pub fn interference(&self, other: &Solid) -> Result<Interference>;       // { volume, gap }
    pub fn section_area(&self, plane: &Plane) -> Result<f64>;
    pub fn section(&self, plane: &Plane, chord: f64) -> Result<Vec<Vec<Point3>>>;
    pub fn project(&self, direction: Vec3, chord: f64) -> Result<Drawing>;    // protocol view axes
    pub fn raycast(&self, rays: &[Ray]) -> Result<Vec<Vec<f64>>>;
    pub fn mesh(&self, chord: f64) -> Result<Mesh>;                           // closed, outward, welded
    pub fn display(&self, chord: f64) -> Result<Display>;                     // per-face tris + tri_face + edge polylines
}
impl FaceRef<'_> { pub fn kind(&self) -> SurfaceKind; pub fn normal(&self) -> Option<Vec3>; pub fn area(&self) -> f64;
                   pub fn centroid(&self) -> Point3; pub fn bbox(&self) -> Aabb; }
impl EdgeRef<'_> { pub fn kind(&self) -> CurveKind; pub fn ends(&self) -> [Point3; 2]; pub fn convex(&self) -> Option<bool>;
                   pub fn length(&self) -> f64; pub fn midpoint(&self) -> Point3; }
pub enum SurfaceKind { Plane { normal: Vec3 }, Cylinder { axis: Vec3, radius: f64 }, Cone, Sphere { radius: f64 }, Torus, Swept, Spline }
pub enum CurveKind { Line { direction: Vec3 }, Circle { radius: f64, axis: Vec3 }, Ellipse, Spline }

// ---- exchange ----
pub mod step { pub fn write(s: &Solid) -> Result<String>;                  // AP214, B-rep (never faceted)
               pub fn read(text: &str) -> Result<Solid>;
               pub fn sew(text: &str, tolerance: f64) -> Result<Solid>; }
pub mod codec { pub fn encode(s: &Solid) -> Vec<u8>; pub fn decode(b: &[u8]) -> Result<Solid>; }
pub mod tol { pub const LINEAR: f64; pub const PICK: f64; }
pub const VERSION: &str;  pub const SOURCE: &str;                         // build hash
pub const SUPPORTS: &[&str];                                               // protocol functions that are not stubs
```

### 4.4 What a brep-bench case program looks like

This target is 5 lines, compared with springrep's 22 for the same case:

```rust
//! The box (−6, −6, −8)–(36, 26, 0) less what a ball of radius 3 passes through along a cubic Bézier.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let block = block(&Frame::at(p3(-6.0, -6.0, -8.0)), [42.0, 32.0, 8.0])?;
    let path = Spline3::bezier(&[p3(0.0, 0.0, 0.0), p3(10.0, 0.0, 0.0), p3(20.0, 20.0, 0.0), p3(30.0, 20.0, 0.0)])?;
    let groove = sphere(&Frame::WORLD, 3.0)?.swept(&Motion::Along { path, law: FrameLaw::Fixed })?;
    block.difference(&groove)
}
```

Rules for case programs:

- A case program uses the public API only. `crates/yack-bench/src/shared.rs` may hold shorthand
  only, at most 60 lines in total, and never an operation that composes kernel calls. A
  composition that two cases need is an API gap: it goes to the API owner, not into `shared.rs`.
- `workarounds.toml` stays empty. A workaround is an API gap that gets filed.
- An API gap gets a stub and is filed against the owning crate. A case refuses with yack's own
  error text until its stub is real.

---

## 5. Adapter and bench runner

### 5.1 `crates/yack-conformance` (brep-conformance adapter)

- **Shape.** The adapter implements `brep_protocol::Kernel` with `type Solid = yack::Solid`. It
  has two files: `main.rs` (the impl, one line per method) and `convert.rs` (frames, profiles,
  points to ids, matrices). The target is ≤ 500 LOC, against springrep's 895 and cadrs's ~1k.
- **What it may do.** Only the contract's four things:
  - translate form;
  - address an entity by a point, with `edge_at`/`face_at`;
  - provide the empty solid, with `make::empty()`;
  - keep handles, which immutability already gives.
- **Error mapping**, in a single `fn r(e: yack::Error) -> Refusal`:

  | yack `ErrorKind` | Refusal |
  |---|---|
  | `Invalid`, `NotFound`, `ForeignId` | `invalid` |
  | `Ambiguous`, `Unsupported` | `unsupported` |
  | `Failed`, `Budget` | `failed` |
  | `Internal` | `resume_unwind`, which the suite scores as a crash |

- **`supports()`** returns `yack::SUPPORTS`.
- **Adapter tests.**
  - `supports()` equals the set of overridden methods, so the adapter never claims a function it
    does not answer.
  - Every listed function passes a smoke case.
  - xtask greps the adapter: no `if` or `match` on a returned value except in `r()`.
- **Dependency.** `brep-protocol` is a git dependency pinned to `springcraft-ai/brep-conformance`
  rev `12ffc75`.
- **`cargo xtask conformance [--seed N|--seeds 0-3] [--family F] [--bless] [--record-digests]`.**
  It builds the adapter in release to `bin/`, runs the pinned suite runner from
  `$YACK_REF/brep-conformance` (default `../yack-ref`), and writes `bin/conformance/seed<N>.json`.
  It then compares the run with `scores/conformance/seed<N>.json`, which holds each case id with
  its outcome, tier and `time_ms`, sorted and timestamp-free.

### 5.2 `crates/yack-bench` (brep-bench runner)

- **Layout.** It has the same layout as brep-bench's `kernels/<k>/`:
  - `src/cases/{category}/{name_snake}.rs`, each `pub fn build() -> yack::Result<Solid>`;
  - `build.rs`, which generates the dispatch table;
  - `src/main.rs`.
- **CLI.** `yack-bench CASE OUT.stl [OUT.step]` and `--version` (`yack <VERSION> <SOURCE>`).
  - Given `OUT.step`, it writes STEP with `yack::step::write` and prints `{"status":"ok"}`.
  - Without `OUT.step`, it prints volume, area, centroid, inertia, faces by kind and validity, and
    writes the STL from `mesh()`.
  - A refusal prints `{"status":"refused","reason":"<yack Error Display>"}`.
  - An unknown case id exits non-zero without building.
  - JSON is written by hand, so there are no dependencies.
- **Warm start.** Under `BREP_BENCH_WARM`, the runner stops itself after loading and again after
  replying. It sends `kill -STOP <own pid>` through `std::process::Command`, so it needs no libc.
  This is the only env read, and it is in a tool crate.
- **Release profile.** `lto = "fat"`, `codegen-units = 1`, `panic = "abort"`, `strip = true`.
- **`cargo xtask bench [--cases glob] [--bless]`.**
  - It builds the runner in release to `bin/release/yack-bench`.
  - It keeps a local, never-committed `yack` entry in `$YACK_REF/brep-bench/kernels.json`, then
    runs `harness/run.py` for `yack`.
  - It writes `bin/bench/run.json` and ratchets it against `scores/bench.json` (each case's
    outcome, time and `loc`).
  - It also prints the mean and max `loc`, computed by the harness's `highlight.loc`.
- **Own measures as tests.** `yack-testkit` turns every case's TOML header into a test of yack's
  own measures, so "geometry right, STEP writer wrong" is told apart from "geometry wrong".
- **Upstream.** `crates/yack-bench` is copied as is to `kernels/yack/`, with `yack` pinned to a
  git rev. The orchestrator opens the upstream PR. Agents never post externally.

---

## 6. Test and verification harness

### 6.1 Tiers

| Tier | Question | Where | When |
|---|---|---|---|
| T0 doc tests | Do the documented examples hold? | `crates/yack/src/**` (an example per public fn) | every batch |
| T1 API oracles | Does each public call meet its closed form? | `crates/yack/tests/{make,boolean,blend,edit,query,exchange,select}.rs` (few binaries: link time) | every batch |
| T2 unit | Is each algorithm right on its own? | inside each crate (springrep's 364 unit tests travel with their modules) | every batch |
| T3 property | Do the identities hold on random inputs? | `crates/yack/tests/props.rs` on `yack-testkit::gen` | batch: 200 seeds; nightly: 20k |
| T4 conformance | What does the suite score? | `xtask conformance` | batch: seed 0, families touched + a 200-case smoke sample; nightly: seeds 0–3 full + NIST |
| T5 bench + corpora | Other people's cases | `xtask bench`, `xtask corpus <name>` | batch: categories touched; nightly: full |
| T6 determinism | Bit-identical? | `tests/determinism.rs` + xtask | every batch |
| T7 robustness fuzz | Never panic, hang or return an invalid solid? | `xtask fuzz --seconds N` (seeded random feature chains, one child process per seed) | nightly |
| T8 structure | Is the DAG intact? Is the API surface intended? | `xtask gate`, `xtask api` | every batch |

**No expected value comes from a kernel.**

- Every value is a closed form, a sampled estimate with a stated bound, or an identity.
- `yack-testkit` holds the shared formulas: primitives, Pappus, the ruled loft `hA(1+s+s²)/3`,
  Steiner for convex offsets, the chamfer volume `P·d²/2 − K·d³/3`, the lens.
- `yack-testkit` also has a Halton membership sampler, ported from `brep-conformance/protocol/src/model.rs` (MIT).
- `assert_volume(s, exact, area)` uses the contract's bound `10·ε·A + 1e-9·V`.

### 6.2 `testkit::audit(&Solid)`: run on every result in T1, T3 and T7

1. `check()` is empty.
2. `mesh(0.01)` is closed and outward, and its volume is within `2·chord·area` of `volume()`.
3. Every face has a `History` origin. Fates and origins agree. This is enforced once History
   lands (M5). Before that, it warns.
4. `codec::decode(encode(s)).digest() == s.digest()`.
5. `step::read(step::write(s))` has the same volume within the B17 bound.
6. The operands' digests are unchanged after the operation (immutability).

### 6.3 Property suites (T3)

The suites are zero-dependency and seeded (xorshift), with shrinking. They are ported in idea from
cadrs `metamorphic_booleans`, `stress_booleans`, `determinism` and `chamfer_generated`, and from
solidcraft `fuzz.rs`:

- **Volume identities.** `V(A∪B) + V(A∩B) = V(A) + V(B)`; `V(A∖B) = V(A) − V(A∩B)`; `A∪A = A`;
  `A∖A = ∅`; commutativity; associativity on 3 operands; `union_all` order invariance.
- **Motions.** Invariance under rigid motions and similarities (mirrors included), checked on
  volume, area and Betti numbers. An affine map multiplies volume by |det|.
- **Blends.** A tangent-chain fillet equals filleting each edge of the chain.
- **Drift.** Twelve unions of the same solid, each turned by a tiny angle, stay valid.
- **Near-degenerate placements.** The generator covers boxes, cylinders, spheres, tori, cones and
  prisms on random frames, at offsets of `k·LINEAR`, angles of `k·ANGULAR`, and tangent and
  coincident placements.
- **Failures.** A failure is minimised and written to `crates/yack/tests/regressions/<hash>.rs`.
  Blessing it needs a reason in the commit.

### 6.4 Ratchets (in `scores/`, committed, sorted, timestamp-free)

| File | Fails CI when |
|---|---|
| `scores/conformance/seed{0..3}.json` | Any case gets a worse outcome or a looser tier. A case's median time exceeds 2× baseline (at 1.5× it warns) |
| `scores/bench.json` | Any pass regresses, any new `wrong` appears, or the mean `loc` rises |
| `scores/digests/seed0.txt` | Used in M3 and M4 only: any digest changes. These milestones are pure moves, and the digests prove it |
| `scores/waivers.toml` | Holds the only allowed exceptions. Each names the case, a reason, an owner and an expiry milestone |
| `scores/eps.txt`, `scores/filesize.txt`, `scores/cadrs-loc.txt` | The count goes up |

Improvements update the baseline in the same commit (`--bless` shows the diff). On main, the
nightly job appends a summary row to `scores/history.jsonl`.

### 6.5 Determinism (T6)

- The same op run twice gives the same `digest()`.
- `par_map` with 1 thread and with 8 threads gives equal digests. The thread count is read once in
  `yack`, never inside kernel crates.
- From M6, x86_64 and wasm32-wasip1 give equal digests on 500 generated cases, because `fmath` is
  used everywhere.
- Gates: clippy `disallowed_types` (`HashMap`, `HashSet`) and `disallowed_methods`
  (`f64::sin`/`cos`/… from M6) in kernel crates.

### 6.6 `cargo xtask gate`: mechanical structure gates, all required

`xtask gate` parses `cargo metadata` and the sources with a std-only JSON reader. Each of its
checks has a planted-violation fixture test proving it fails.

1. **Layers.** Every internal edge must be in its crate's `layers.toml` allow-list and must point
   strictly lower. Unlisted crates fail.
2. **One public crate.** Only `yack` lacks `publish = false`. No internal crate depends on `yack`
   or on a tool crate.
3. **Zero dependencies.** No non-workspace package is reachable from `yack` (checked from
   `Cargo.lock`), nor from `xtask`, `yack-bench` or `yack-testkit`.
4. **No leaks.**
   - No glob `pub use` anywhere.
   - No `#[doc(hidden)] pub` in `yack`.
   - No `pub use yack_*` in `yack` except the `reexport` list.
   - The `yack` surface equals `crates/yack/api.txt`. `xtask api` extracts `pub` items from
     source, so it works on stable Rust.
5. **Module graph.** Inside `yack-kernel` (M3) and any crate over 5k LOC, the edges between
   top-level modules must agree with `layers.toml`'s module map.
6. **File size.** No file is over 1,500 lines, except those on a shrinking allowlist. A function
   over 150 lines warns.
7. **Epsilon ratchet.** A float literal below 1e-2 outside `tol` and `numeric` needs a
   `// tol: <reason>` comment. The count in `scores/eps.txt` may only fall. springrep starts it at
   about 500.
8. **Hidden state.** `std::env`, `thread_local!`, `static mut`, `OnceLock`/`Mutex` statics and
   `Instant`/`SystemTime` are banned in kernel crates. Only `yack` calls `catch_unwind`.
9. **Lints.**
   - `unsafe_code = "forbid"`.
   - Denied in kernel non-test code: `unwrap_used`, `expect_used`, `panic`, `todo`,
     `unimplemented` and `dbg_macro`.
   - `indexing_slicing` warns, and is ratcheted to deny by M6.
   - `missing_docs` is denied on `yack`.
10. **Provenance.** Every file under `crates/` starts with either a `yack` header or a provenance
    header (§9) naming an allowlisted donor and path. Any path under `next.BREP`, `vendor/truck`,
    or solidcore outside `crates/{ssi,nurbs-grid-mesh}` fails.
11. **Build directory.** `target-dir = "bin"` is present, `bin/` is ignored, and there is no
    `target/` directory and no `.git/worktrees` entry.

### 6.7 Corpora

| Corpus | Licence | Use | Lives |
|---|---|---|---|
| brep-conformance seeds 0–3, NIST | MIT; US-gov (verify each file) | T4 | `$YACK_REF`, pinned |
| brep-bench, 506 cases + `harness/mass` | MIT OR Apache-2.0 | T5; headers become oracles | `$YACK_REF`, pinned `d522d62` |
| springrep `tests/` (215) + `grazing-tooth-{a,b}.step` + `doc/design.md` anecdotes | MIT | T1 port (M1.8), one regression per anecdote | `crates/yack/tests/` |
| cadrs `metamorphic_*`, `stress_booleans`, `determinism`, `chamfer_generated`, `conformance.rs` (84), `ssi_robust`, `swept_exact`, `mass_exact`, `tess_quality`, `validity`, `fuzz_readers` | MIT | T1/T3 port (M5) | `crates/yack/tests/` |
| cad-kernel `gcs-core/tests/{brep*,arrangement,booleans,mesh*}` (~150), `step_check.rs` | MIT | T1; STEP self-verifier | `crates/yack/tests/`, `yack-testkit` |
| solidcraft `tests/oracle` (83), `examples/regressions` (25); step-conformance (17, CC0) | user's own MIT; CC0 | T5 | `crates/yack/tests/`, `corpus/step-cc0/` |
| ssi-corpus (jyviko `b04c8a6`, 119) | MIT OR Apache-2.0 (25 cases CC-BY: cite) | SSI gate ≥ 118/119 | fetched by `xtask corpus ssi`, SHA-256 manifest |
| solidcore `fixtures/solidcraft` (128) | Apache (facts) | Re-expressed as our own tests. **Never copy the JSON** | `crates/yack/tests/` |
| SMLib/usd-brep (237) | Apache-2.0 data | Optional, M7: needs cadrs `smb.rs` | separate repo `yack-corpus-apache` |
| **Excluded** | – | cadrs `tests/data/docs`, `blend_tool_tolblend_c9.body`, OCCT dataset, STEP Tools files, all of next.BREP, solidcore's SMLib port, `vendor/truck` | never |

---

## 7. Milestones

Each milestone ends with a measured run. "Batch" means one one-tree-swarm batch branch, merged as
one PR. Packages in the same milestone run in parallel unless marked "after".

### M0: Skeleton (half a day, 5 packages)

**Exit:**

- `cargo xtask gate` and `cargo test` are green, and each planted violation fails.
- Conformance seed 0 completes with every case unsupported and 0 crashes; brep-bench completes
  with 506 refused and 0 crashed. `bin/` is untracked.

| Pkg | Owner crate(s) / files | Inputs | Exit test |
|---|---|---|---|
| M0.1 Workspace (lead, first, ~1 h) | root `Cargo.toml`, `rust-toolchain.toml` (1.98.0, the installed toolchain; springrep pins 1.99, so M1.1 verifies it builds on 1.98), `clippy.toml`, `.cargo/config.toml` (`[alias] xtask = "run -p xtask --"`), `layers.toml` (final DAG of §3.1 plus temporary crates; empty crates are not created yet), `.gitignore` (+ `swarm/`), `NOTICE`, `scores/` skeleton | §2, §3, §9 | `cargo build` writes to `bin/`; `cargo xtask` prints help |
| M0.2 Gate (after M0.1) | `crates/xtask/src/{main,meta,json,gate/*}.rs` | §6.6, `layers.toml` | each of the 11 checks has a fixture that fails; the gate passes on the tree |
| M0.3 API stubs (after M0.1) | `crates/yack/src/**`, `crates/yack/api.txt` | §4 | every §4.3 item compiles, is documented and returns `Unsupported` (queries on `empty()` are real); `xtask api` is clean |
| M0.4 Adapter (after M0.3 signatures) | `crates/yack-conformance/**`, `crates/xtask/src/conformance.rs` | §5.1, `yack-ref/brep-conformance-springrep/src/main.rs` (MIT) | full suite runs end to end: all unsupported, 0 crashes; `scores/conformance/seed0.json` written |
| M0.5 Bench runner (after M0.3) | `crates/yack-bench/**` (506 refusing stubs generated from `cases/`), `crates/xtask/src/bench.rs` | §5.2, brep-bench `SPEC.md`, `kernels/cadrs/{build.rs,src/main.rs}` (MIT OR Apache) | `harness/run.py` grades 506 refused, 0 crashed; warm start works; `--version` prints |

### M1: A scored yack backed by springrep (≤ 1 day, 9 packages)

`crates/yack-kernel` holds springrep verbatim. The facade implements §4 by thin calls into it.

**Exit:**

- **Conformance.** Seed 0: 100%. Seeds 1–3: no worse than `runs/springrep-seed{1,2,3}.json`, per case.
- **brep-bench.** ≥ 333 passes, 0 wrong (springrep's 9 wrong fixed or refused), mean `loc` ≤ 12.
- **Other.** 215 ported springrep integration tests pass, digests are recorded, gate green.

| Pkg | Owner | Inputs | Exit test |
|---|---|---|---|
| M1.1 Import springrep | `crates/yack-kernel/**` | springrep `src/` + `build.rs` @ `a03e8a1`, provenance headers, `publish=false` | springrep's own unit tests pass in-crate on 1.98; zero deps |
| M1.2 Facade: make, profile, values | `yack/src/{make,profile,values,prelude}.rs` | §4.3; `yack-kernel` `primitive`, `sweep`, `loft`, `profile` | T1 `make.rs` ports pass |
| M1.3 Facade: booleans and blends | `yack/src/{boolean,blend}.rs` | `yack-kernel` `boolean`, `blend` | T1 `boolean.rs`, `blend.rs` ports pass |
| M1.4 Facade: edits, moves, swept stub | `yack/src/{edit,motion}.rs` | `yack-kernel` `edit` | T1 `edit.rs` ports pass |
| M1.5 Facade: solid, ids, queries, exchange, errors, firewall | `yack/src/{solid,select,query,exchange,error,guard}.rs` | `yack-kernel` `topo`, `measure`, `check`, `mesh`, `step`, `codec` | T1 `query.rs`, `exchange.rs`, `select.rs` pass; stamped-id misuse returns `ForeignId` |
| M1.6 Conformance on seeds 0–3 | `yack-conformance`, `scores/conformance/*`, `scores/digests/seed0.txt` | M1.2–M1.5 | the conformance exit above; time sum recorded |
| M1.7 Bench programs A (boolean, chamfer, cnc, draft, edit, feature, fill, fillet, geometry, integration, loft) | `yack-bench/src/cases/{those}/**` | the case `.md` text; springrep's and cadrs's programs as readings (MIT/Apache, translated, not copied) | each passes or refuses with yack's reason; 0 wrong |
| M1.8 Bench programs B (nurbs, offset, parts, primitives, scale, shell, singular, spline, sweep, swept, tangent, thicken) | `yack-bench/src/cases/{those}/**` | same | same |
| M1.9 Test ports and gap census | `crates/yack/tests/*.rs`, `crates/yack-bench/GAPS.toml` | springrep `tests/` (215); live board + local cadrs run per case | 215 tests pass; `GAPS.toml` lists, for each case yack refuses and cadrs passes, the cadrs ops used and the yack call that should exist |

### M2: Donor union with cadrs (2–3 days, runs alongside M3, 5 packages)

The goal is to get brep-bench from about 333 to at least 500 fast, without touching springrep
code.

**Exit:**

- brep-bench ≥ 500 passes with 0 wrong.
- Conformance seed 0 stays at 100%.
- Seeds 1–3 are ≥ M1 per case, with routing only on refusal (§DECISIONS D9).
- `Cargo.lock` has no external package.
- `yack-cadrs` reproduces cadrs's own seed-0 conformance pass set, ±0 cases, through a test
  adapter.

| Pkg | Owner | Inputs | Exit test |
|---|---|---|---|
| M2.1 Import cadrs with shims | `crates/yack-cadrs/**` | cadrs `crates/cadrs_kernel/src` @ `eb5899e`; in-crate `na`/`cdt`/`par` shims; env switches frozen | builds with zero deps; cadrs's `tests/conformance.rs` (84) passes in-crate |
| M2.2 Bridge | `yack/src/bridge.rs` | cadrs `topo.rs`/`surface.rs` parametrisations; `yack-kernel` `topo::build::Parts` | Body→Brep→Body round trip is exact on 500 generated solids; every converted solid passes `check()`; volumes agree to 1e-12 rel |
| M2.3 Routing and API completion | `yack/src/route.rs`, real bodies for `swept`, `loft_guided`, `nurbs_solid`, `Segment::Nurbs` | `GAPS.toml` | each `GAPS.toml` row is routed or explicitly refused; routed results pass `audit()` |
| M2.4 Bench programs for gap cases | `yack-bench/src/cases/**` for rows in `GAPS.toml` (claims split by category between two agents) | M2.3 | the brep-bench exit above; mean `loc` ≤ 10 |
| M2.5 Conformance routing | `yack-conformance` smoke + `scores/` | M2.3 | seeds 0–3 ratchet green; refusals that turned into passes are blessed |

### M3: Untangle springrep in place (1–2 days, alongside M2, 5 packages)

The work is T1–T12 of §3.4, inside `yack-kernel`, using
`xtask modgraph --crate yack-kernel` (std only: parses `use crate::` paths outside `#[cfg(test)]`
blocks).

**Exit:**

- `xtask modgraph` shows 0 violations.
- `scores/digests/seed0.txt` is **bit-identical**.
- All tests pass.

| Pkg | Tasks | Files |
|---|---|---|
| M3.1 | T1, T2, T10 | everything under `yack-kernel/src/boolean/` (one owner) |
| M3.2 | T3, T4, T9 | `topo/`, `codec.rs`, `check.rs` |
| M3.3 | T5 | `step/topology.rs` → `topo/build.rs` |
| M3.4 (after M3.1) | T6, T7 | `edit/tweak*`, `sweep/` |
| M3.5 | T8, T11, T12 | `blend.rs`, `pick.rs`, `mesh/` tests, `error.rs` |

### M4: Extract crates and harden (2–3 days, ≤ 6 agents per wave)

Each extraction is mechanical, done bottom-up:

1. `git mv src/<m> ../yack-<m>/src`.
2. Make the needed items `pub`.
3. Leave `pub use yack_<m> as <m>;` in `yack-kernel/src/lib.rs`.

Waves: W1 `base`, `tol`, `numeric`, `p21` · W2 `math` · W3 `geom` · W4 `intersect`, `profile`,
`topo` · W5 `locate`, `measure`, `codec` · W6 `check`, `mesh` · W7 `tweak`, `heal`, `build` ·
W8 `boolean` · W9 `blend`, `edit`, `step`, `view`.

After W9, delete `yack-kernel`.

Hardening runs in each crate right after its wave, by its owner: god-file splits, named
epsilons, `Invalid { arg }` on every error, `Budget` meters in intersect, boolean and blend, and
the clippy deny list.

**Exit:**

- `yack-kernel` is deleted, and `layers.toml` is final.
- Digests are bit-identical through W9. After that they change only in commits that say why.
- Epsilon count ≤ 150; no file over 1,500 lines; 0 `unwrap` in kernel crates; scores no worse.

### M5: Re-host cadrs onto the DAG (1–2 weeks, one agent per row)

Each row moves one routed family out of `yack-cadrs` and onto `yack-topo` types, inside its owner
crate, and deletes that route.

The procedure is the same for every row:

1. **Port.** The first commit holds paths, types and the provenance header only. Behaviour
   changes come in later commits.
2. **Swap.** Switch the route.
3. **Verify.** Confirm the per-case bench and conformance outcomes are unchanged or better.

| Pkg | Owner crate | Re-hosted from cadrs | Exit test |
|---|---|---|---|
| M5.1 | `swept` (new) | `sweep_moving`, `sweep_solid`, `sweep_tube`, `sweep_ball_pass`, `sweep_mill` + cad-kernel `swept/` closed forms | brep-bench `swept/` ≥ M2 count; `swept` routes deleted |
| M5.2 | `edit` (delete/move faces) | `defeature.rs`, `local.rs` replace-faces | `edit/` ≥ M2; conformance `delete-faces-*` ≥ 95% on seeds 1–3 |
| M5.3 | `edit` (offset/shell/thicken) | `offset/` (sharp, round, slab, prism), `surfacing.rs` thicken; cad-kernel offset-by-definition | `offset/`, `shell/`, `thicken/` ≥ M2; conformance `offset-*`, `shell-*` ≥ 95% on seeds 1–3 |
| M5.4 | `blend` | `fillet*` variants that springrep refuses, `convex_chamfer`, `rim_blend`, `face_blend` | `fillet/`, `chamfer/` ≥ M2 |
| M5.5 | `build` | `loft_build`, `curve_loft`, NURBS profiles, `sweep_planar`/`sweep_pipe` where faster | `loft/`, `sweep/`, `nurbs/`, `spline/` ≥ M2; `sweep-along-a-spline` median < 2 s (springrep 6 s) |
| M5.6 | `edit::draft` | `local.rs` draft | `draft/` ≥ M2 |
| M5.7 | `topo` + every op | `History` returned by every op; `compose`; cadrs `naming.rs` resolve ladder (as data in `topo`) | audit invariant 3 enforced; `compose` property over random 5-op chains |
| M5.8 | `yack` | delete `route.rs`, `bridge.rs`, `yack-cadrs` | gate rejects any `yack_cadrs` reference; scores no worse |

**Exit:**

`yack-cadrs` is deleted; brep-bench ≥ 503 with 0 wrong; conformance seeds 1–3 ≥ 99.5%.

### M6: Beyond every donor (1–2 weeks)

| Pkg | Owner | Work | Exit test |
|---|---|---|---|
| M6.1 | `ssi` + `intersect::freeform` | solidcore `ssi` as is, used where analytic classification says "free-form/unknown"; A/B on corpus before switching | ssi-corpus ≥ 118/119; scores no worse |
| M6.2 | `numeric` + `intersect` | intervals + `isolate` as certified CSI fallback ("refuse, never miss") | every CSI fixture: all roots or `Failed { stage: Intersect }` |
| M6.3 | `boolean` | strategy table; n-ary as one arrangement; `par_map` face pairs; sector classification before ray parity (next.BREP idea) | `holes-in-one-operation` ≤ one-at-a-time; boolean time −40% at 8 threads; digests equal at 1/8 threads |
| M6.4 | `mesh` + `gridmesh` | apex fan (`display-of-a-cone`), TessPlan split, gridmesh for dense NURBS | display/mesh 100% on seeds 0–3; 318×281 degree-7 surface < 0.5 s |
| M6.5 | `step` | units/SI prefixes, BREP_WITH_VOIDS, FACETED_BREP, `step_check` on every write | NIST 33/33; step-conformance 17/17; brep-bench verdict "B-rep" on every pass |
| M6.6 | `numeric` (`fmath`) | deterministic transcendental functions everywhere | wasm32 = x86_64 digests on 500 cases |
| M6.7 | perf (lead assigns per crate) | profile the top 20 slowest conformance tests | time sum ≤ 0.8× springrep |
| M6.8 | owners of failing cases | last brep-bench and conformance failures, one case at a time | **≥ 505/506, 0 wrong; 100% seeds 0–3** |

### M7: Release

- `cargo xtask bundle` turns the internal crates into private modules of the one published `yack`
  package, rewriting `yack_x::` to `crate::k::x::`.
  - CI builds and tests the bundle.
  - The gate keeps running on the source workspace.
  - Fallback if the bundle fails: publish the internals as `yack-internal-*` (`doc(hidden)`, no
    semver).
- Docs carry an example per function. `examples/{bracket,flanged_tube}.rs` are ported from
  springrep.
- The brep-bench kernel PR goes upstream.
- **Exit:** `yack 0.1.0` is on crates.io, and the README shows both scoreboards.

---

## 8. Parallel work: ownership and flow

- **Ownership is by crate.** `layers.toml` gives every crate one owner. An agent edits only files
  it claimed in crates it owns. A change to another crate's `api.txt` is a SendMessage request to
  its owner; if the owner is idle, the lead assigns it.
- **Standing roles.**

  | Role | Owns | Rules |
  |---|---|---|
  | API owner | `crates/yack`, `api.txt`, `prelude`, the adapter mapping | Reviews every public signature against §4.2. Writes no geometry |
  | Harness owner | `xtask`, `yack-testkit`, `scores/`, `waivers.toml` | Keeps the ratchet honest |
  | Lead | `layers.toml`, `plan/`, merges | Merges batches |

- **One tree, one builder** (`AGENTS.md`, one-tree-swarm): workers edit code only; one builder
  compiles, tests and commits in rounds; frozen-base batch branch; disjoint claims; `bin/` only.
- **Merge rule.** A batch merges when `xtask gate` is green, every ratchet holds, T0–T3 and T6 are
  green, and the API owner has approved any `api.txt` diff. A lower score blocks a merge even with
  green unit tests.
- **Peak agents per milestone.**

  | Milestone | Agents |
  |---|---|
  | M0 | 5 |
  | M1 | 9 |
  | M2 + M3 | 10 together |
  | M4 | 6 per wave |
  | M5 | 8 |
  | M6 | 8 |

  These counts are bounded by disjoint files, not by appetite.

---

## 9. Licence, attribution and provenance

- **yack's licence.** yack is MIT, `Copyright (c) 2026 yack contributors` (`LICENSE`).
- **`NOTICE`** (created at M0.1) names each donor whose code is in the tree, with its copyright
  line and licence, pinned revision and the paths taken:

  | Donor | Copyright | Revision |
  |---|---|---|
  | springrep | `(c) 2026 Springcraft, Inc.`, MIT | `a03e8a1` |
  | cadrs | `(c) 2026 Rouan van der Ende`, MIT OR Apache-2.0, used under MIT | `eb5899e` |
  | geomsolver / cad-kernel | `(c) 2026 Brandon Martin-Anderson`, MIT | `694cb72` |
  | solidcore `crates/ssi` | `(c) 2026 h0witzer`, MIT OR Apache-2.0, used under MIT | (pinned at M6.1) |
  | solidcore `crates/nurbs-grid-mesh` | h0witzer, AlexMitchellMus, used under MIT | (pinned at M6.1) |
  | solidcraft | `(c) 2026 SolidCraft contributors`, MIT | (pinned when used) |
  | brep-conformance protocol (adapter dependency, Halton sampler idea) | `(c) 2026 Springcraft, Inc.`, MIT | `12ffc75` |
  | brep-bench case texts used as oracles | `(c) 2026 the brep-bench contributors`, MIT | `d522d62` |

  `NOTICE` also carries the musl/FreeBSD notice for `fmath`, kept verbatim with the file.
- **Per-file provenance header.** It is the first lines of every adapted file. The gate (§6.6.10)
  checks it.
  ```rust
  // yack: <one-line purpose of this file>
  // Adapted from springrep@a03e8a1 src/boolean/csi.rs (MIT, (c) 2026 Springcraft, Inc.).
  // Changes: moved to yack-intersect; split into csi/{line,conic}.rs.
  ```
  - A file built from several donors lists each donor on its own `Adapted from` line.
  - A file written fresh says `// yack: <purpose>` and nothing else.
  - A file that implements a next.BREP **idea** says `// Idea: next.BREP <topic>, via analysis/next-brep.md; no code seen.`
- **Hard exclusions** (path-checked by the gate): next.BREP source; solidcraft `vendor/truck`;
  solidcore outside `crates/{ssi,nurbs-grid-mesh}` (its `step-import` is ideas only); cadrs
  `tests/data/docs` and OCCT-derived `*.body`; the OCCT dataset; STEP Tools files; SMLib data.
- **Open items** (before M6.1):
  - The origin of solidcore `ssi` ("Form Studio" `intersect.js`) needs confirmation from h0witzer.
  - `ssi` avoids US 12,229,885 by design. Keep that design note intact.

  If the ssi item is not cleared, `yack-ssi` stays out, and cadrs `ssi_numeric` (MIT) is the
  free-form backend.

---

## 10. Risks

| Risk | L/I | Mitigation |
|---|---|---|
| The carve changes behaviour silently (memo eviction, iteration order) | M/H | The digest gate stays bit-identical through M3 and M4 |
| Overfitting seed 0 | H/M | Ratchets on seeds 0–3 from M1, and brep-bench as a second yardstick |
| The two engines (M2–M5) become permanent, giving two topologies and low cohesion | M/H | `yack-cadrs` is quarantined to `route.rs`/`bridge.rs`, with a LOC ratchet that falls from M4. M5.8 deletes it as part of the M5 exit |
| The `Body` ↔ `Brep` bridge loses exactness (parametrisations differ, e.g. cadrs's cone by slant length) | M/H | An exact mapping per surface kind. Round trip tested on 500 generated solids. STEP text as a day-one stopgap only |
| Routing turns a refusal into a wrong result | M/H | Only refusals are routed. Every routed result runs yack's (springrep's, stricter) checker plus `audit()`. On brep-bench, cadrs is 0 wrong live |
| The cadrs nalgebra/spade/rayon shims are big | M/M | Shim only the API surface cadrs uses (an M2.1 census). `cdt` is springrep's. `par` is sequential at first |
| Re-hosting cadrs algorithms onto springrep topology costs more than estimated | H/M | M5 goes row by row, each row independently shippable, and routes stay until a row is green. The M5 exit may slip, but no score is lost |
| The `boolean/` untangle (T1/T2) is large | M/M | One owner for all of `boolean/`. A temporary `FaceImages` slot is allowed. The M1 kernel stays shippable |
| The toolchain is 1.98 while springrep pins 1.99 | L/M | M1.1 verifies. If 1.99 features are used, install 1.99 once, shared, and pin it |
| brep-bench grades our STEP writer | M/H | Own-measure tests per case (§5.2) and `step_check` on every write, so a writer bug shows as "own measures pass, STEP fails" |
| The case LOC target pushes composition into `shared.rs` | M/L | Cap of 60 lines, shorthand only. Composition goes into the API, reviewed by the API owner |
| Licence contamination | L/H | Provenance gate, path exclusions, and next.BREP closed to all porting agents |
| Single-crate publishing (crates.io needs published deps) | C/M | `xtask bundle` at M7; fallback is `yack-internal-*` |
| Absolute mm tolerance at large and small scales | L/M | springrep's absolute policy passes today. Refuse beyond `MODEL_SIZE` bounds with `Invalid` |
| Many crates mean slow links and compiles | M/L | Few test binaries. `cargo check -p`. One warm `bin/`. `lto` only in release runners |
