# yack plan

yack (Yet Another CAD Kernel) is an exact B-rep kernel in pure, safe Rust, licensed MIT OR
Apache-2.0 (§9). This is the single authoritative plan. It merges the final revisions of the three
architect plans in `yack-ref/plans/`: A (refactor the winners), B (layered, strategy ladders) and
C (API and tests first). Conflicts and their resolutions are in `plan/DECISIONS.md`. This file wins
over the older plans, and changes to it need the lead's approval. `yack-ref/` means
`/home/brenn/github/yack-ref`. Milestones run M0–M6; ids like M1.3 are work packages, each one
agent's job.

---

## 1. Goal and scoreboard

yack must beat every donor kernel on both benchmarks. The conformance suite has 3,609 cases per
seed, so scores are given as percentages.

| Benchmark | Metric | Best donor today | yack target (final, M5) |
|---|---|---|---|
| brep-conformance (`yack-ref/brep-conformance`, suite 1.0.0) | pass rate at 1e-7, per seed | springrep 1.0.1: 100% on seed 0 (3,487-case release), about 97.5% on seeds 1–3. cadrs: 81% on seed 0 (local run). cad-kernel: 56%. solidcore: 49.5% | **100% on seeds 0–3**; 0 crashes; 0 timeouts |
| | total `time_ms` over a seed | springrep seed 1: 1,481 s | **≤ 0.7×** springrep, on the same machine |
| brep-bench (`yack-ref/brep-bench`, SPEC 1.17, 506 cases, graded from our STEP file) | pass count | cadrs **504** (0 wrong, 2 refused); next.BREP 482; OCCT 412; cad-kernel 393; solidcraft 361 (39 wrong); geomsolver 340; springrep **333** (164 refused). Live brepbench.org, 2026-11-20 | **≥ 505/506** |
| | wrong count | cadrs 0 | **0, with no waiver ever** |
| | mean case lines (harness `highlight.loc`) | about 12 (springrep, cadrs, solidcraft) | **≤ 8**. Total runner lines ÷ cases ≤ 14. 0 workarounds |
| | mean time per case | cadrs 3.2 ms | **≤ 3.2 ms** |

The brep-bench results committed in the repo, and `analysis/brep-bench.md`, are **stale**. Only a
live board read or our own local run counts as a baseline.

**The two winners are complementary.** springrep passes all of conformance seed 0 and all of
brep-bench boolean, primitives, parts, singular, scale, spline and feature, but refuses bench
swept 52, edit 31, offset 28, shell 20, fillet 7, chamfer 6, thicken 4; its seed 1–3 losses are
almost all conformance `modify/`. cadrs passes 504/506 on brep-bench but only 81% of conformance
seed 0 (fillets 398/627, queries 146/312, constructions 315/448 with 60 crashes, modify 551/612).

So springrep is the **skeleton** (core, booleans, fillets, queries, check), and cadrs is the
**primary donor for** swept, edit, offset, shell, delete/move faces, draft, thicken, chamfer,
heal/sew/fill and measure. A per-op bake-off (§3.5) decides the order wherever both donors
cover an op.

## 2. Rules (non-negotiable)

**R1. High cohesion, low coupling: a strict crate DAG, enforced mechanically.** `layers.toml`
gives every crate a layer, owner and allow-list; a crate depends only on allow-listed crates in
a strictly lower layer; `cargo xtask gate` fails on any violation (§6.6).

**R2. Exactly one public crate, `yack`.** Every other crate has `publish = false`, and no `yack`
signature names an internal type.

**R3. The API is extremely obvious.** The conformance adapter is name for name: one `yack` call,
plus form translation, per protocol function. A brep-bench case program is a short, literal
reading of the case text.

**R4. Zero external dependencies in shipped crates.** This covers `yack` and everything it
depends on, on every target and feature. Tool crates are zero-dependency too. The single
exception is `yack-conformance`, which uses `brep-protocol` (MIT) and is never a dependency of
anything.

**R5. Layout.** The root is a virtual workspace: `members = ["crates/*"]`, `resolver = "3"`.

**R6. Licences.** Code must be MIT OR Apache-2.0, or Apache-2.0-only donor code kept in marked
crates (§9). Allowed: springrep, cadrs, cad-kernel/geomsolver, all of solidcore, solidcraft
(current), truck, the SMLib corpus. Never: next.BREP code (copyright-assignment clause; ideas
only, from `analysis/next-brep.md`; never open `yack-ref/next.BREP.io_RUST_BREP_KERNEL`), files
with no licence, OCCT-derived LGPL data.

**R7. One working tree, one target dir (`bin/`).** No worktrees, clones, `CARGO_TARGET_DIR` or
`target/`. Swarms follow one-tree-swarm: one builder, a frozen-base batch branch, disjoint
claims.

**R8. Valid or error.** The checker runs on every result, and a refusal beats a wrong solid.
This is springrep's contract, and the reason it scores 100%.

**R9. Determinism.** Kernel crates use no `HashMap`/`HashSet` iteration, env vars, globals,
`thread_local!` or wall-clock time. `par_map` gives the same results as a serial run.

## 3. Crate DAG

### 3.1 Final DAG (after M4)

There are 25 internal crates plus `yack`. All are prefixed `yack-` except `yack` itself.

- **L** is the layer. A crate may use only lower layers.
- **Allowed deps** lists internal crates and is final once M2 ends. Until then the M2 module-graph
  tool can tighten it but never widen it.

| L | Crate | Responsibility (one) | Allowed deps | Donor |
|---|---|---|---|---|
| 0 | `base` | `Error`, `ErrorKind`, `Stage`, `Issue`; `Budget` (deterministic work units); `Limits` (allocation caps); ordered `par_map`; `guard()` panic firewall helper | – | springrep `error.rs`; cadrs `KernelError` stages; cad-kernel `util/{par,meter}.rs`; solidcraft taxonomy |
| 0 | `tol` | Tolerance policy: `LINEAR = 1e-7` mm, `ANGULAR`, `TANGENT`, `RESOLVED`, `PICK = 10·LINEAR`, the named round-off ladder, `between`, `within`, and the ordered "same feature" rulebook | – | springrep `tol.rs`; cadrs `tol.rs` ladder; cad-kernel `tolerant.rs` rulebook |
| 0 | `ssi` | Certified free-form surface/surface intersection on `Patch` (plain data) | – | `ssi` **as is**, from the `solidcraft-main/solidcore` snapshot (solidcore plans to delete it) (M5) |
| 0 | `gridmesh` | Grid mesher for dense trimmed NURBS (`license = "Apache-2.0"`: solidcore now marks it Apache) | – | solidcore `crates/nurbs-grid-mesh` **as is** (M5) |
| 1 | `numeric` | Quadrature, banded/cyclic solves, QR/SVD/LU, roots, intervals, certified root isolation (Krawczyk), Shewchuk predicates, deterministic `fmath` | base, tol | springrep `numeric.rs`, `mesh/predicates.rs`; cad-kernel `util/{interval,isolate,linalg,expansion,predicates,fmath}` (keep the musl/FreeBSD notice) |
| 1 | `p21` | ISO 10303-21 lexer, parser and writer into a neutral entity graph, with `Limits`. No geometry types | base | springrep `step/part21.rs`; cadrs `step.rs` complex-entity and escape handling |
| 2 | `math` | `Vec2/3`, `Point2/3`, `Dir3`, `Frame`, `Rigid`, `Similarity`, `Affine`, `Aabb`, `Angle`, BVH | numeric, tol, base | springrep `math.rs`, `mesh/bvh.rs`; cadrs `bvh.rs` |
| 3 | `geom` | Analytic-first `Curve`, `Surface`, `Curve2`; one rational B-spline type; eval, derivatives, projection, periods; `Mapped` affine images; analytic recognition; `Enclose` | math, numeric, tol, base | springrep `geom/`; cadrs `nurbs.rs` checked constructors, `recognize.rs`; cad-kernel `affine.rs`, `enclose.rs` design |
| 4 | `intersect` | CCI, CSI and SSI with contact classification, giving a plain `Meeting`: closed forms first, then swept-family, then marching, then certified free-form via `ssi` | geom, ssi, math, numeric, tol, base | springrep `boolean/{csi,ssi,ssi/*,contact,march}`; cadrs `ssi_graph`, `ssi_cci`, `ssi_coincident`, `ssi_numeric`; next.BREP tangent lane (idea) |
| 4 | `profile` | 2-D segments, loops and profiles, with validation, segment labels and exact 2-D region booleans | geom, math, numeric, tol, base | springrep `profile/`; brep-protocol `profile.rs` Gn formulas; solidcraft `Region2` |
| 4 | `topo` | `Brep` arena, typed ids, pcurves, seams, contacts, opaque `Memo` slots, `build::Parts` (the only builder), names/roles, `History` data, similarity maps | geom, math, tol, base | springrep `topo/` + `step/topology.rs::{Parts,solid_of,rebuilt}` + `blend::EdgeName`; solidcraft `crates/kernel` history + `crates/doc` naming |
| 5 | `locate` | Point-in-face, point classification (in/on/out), rays, closest point, distance, entity at a point | topo, intersect, geom, math, numeric, tol, base | springrep `boolean/{domain,classify}.rs`, `rays.rs`, `distance.rs`; cadrs `distance.rs` |
| 5 | `measure` | Exact integrals: volume, area, centroid, inertia, tight boxes, Betti numbers, normal and curvature | topo, intersect, geom, math, numeric, tol, base | springrep `measure.rs`; cadrs `mass.rs` (degree-matched Gauss rules); cad-kernel `props.rs` uv gap fix |
| 5 | `codec` | Exact, versioned, capped binary format plus `digest()` | topo, geom, math, base | springrep `codec.rs` (`fields!`); `solidcore-record` schema ideas |
| 6 | `check` | Validity contract: incidence, closure, geometric agreement, orientation, nesting, face–face crossing (BVH) | topo, locate, measure, intersect, geom, math, tol, base | springrep `check.rs`; cadrs `check_faces.rs` |
| 6 | `mesh` | Watertight tessellation (CDT in metric uv), display mesh with topology ids, tess plan | topo, locate, gridmesh, geom, numeric, math, tol, base | springrep `mesh/`; `solidcore-tess` (Apache, SolidCraft's mesher) TessPlan + mesh topology fields |
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

### 3.2 Temporary crates (each one deleted by plan, gate-enforced)

`layers.toml` marks these `temporary = true` and gives each a `delete_by` milestone. The gate
fails if any of them exists once the tag for that milestone is set. It also enforces these rules
on them:

Only `yack`, the temporaries themselves and `from_cadrs.rs` port modules may depend on them, and
`yack-cadrs` LOC may only fall from M3 onward.

| Crate | Content | Delete by |
|---|---|---|
| `yack-springrep` | springrep `src/` at `a03e8a1` (v1.0.1), copied verbatim, then untangled in place | M3 |
| `yack-cadrs` | cadrs `crates/cadrs_kernel/src` at the board's pinned `eb5899ef` (the local checkout is already at `eb5899e`), copied verbatim. External deps reach `yack-shim` through Cargo package renames, e.g. `nalgebra = { path = "../yack-shim", package = "yack-shim" }`. serde derives are stripped by script; `uuid` becomes a counter; `web-time` is removed; `tess.rs` (spade) is dropped and meshing goes to springrep's. The 83 `CADRS_*` env switches are fixed at the defaults cadrs's bench run uses. Each alternative branch is deleted, or becomes a named strategy | M4 |
| `yack-shim` | Zero-dep facades shaped like nalgebra, rayon and rustc-hash, backed by `yack-numeric`/`yack-math`. The nalgebra subset: `Point2/3`, `Vector2/3/4/6`, `Unit`, `SMatrix`/`Matrix2..6x4`, `DMatrix`/`DVector`, `Rotation2/3`, `UnitQuaternion`, `Translation3`, `Isometry3`, and the `.lu()`, `.solve()`, `.try_inverse()`, `.svd()` and `.symmetric_eigen()` calls. It mirrors nalgebra's conventions: sorted singular values, partial-pivot LU. Its rayon is an ordered scope map. Its Fx maps are deterministic | M4 |
| `yack-bridge` | Lossless `Brep ↔ cadrs::Body` conversion. It reparametrises the cone (slant ↔ springrep) and the sphere (latitude), carries pcurves as `PCurve::Curve2`, and maps names to history | M4 |

### 3.3 Interfaces at crate boundaries

| Boundary | Interface |
|---|---|
| Plain data everywhere | No glob re-exports. No `pub` item that exists only so a sibling crate can reach in. Each crate's surface is snapshotted in `crates/<c>/api.txt` |
| `geom` → `intersect` | Inputs are `&Curve`, `&Surface`, `UvBox` and `TBox`. The output, `Meeting { branches, points, same }`, carries no topology ids |
| `intersect` ↔ `ssi` | Only `intersect::freeform` names `ssi` (gate-checked) |
| `topo` → queries | Read-only `&Brep` accessors plus `Memo<T: MemoSlot>` slots. Each client defines its own slot type, and `topo` never names one. Codecs never encode memos |
| ops → ops | One shape: `fn op(inputs, &Params, &Ctx) -> Result<Made>`, with `Made { brep, history }`. Every op ends in `finish(made, ctx)`, which runs `check` and turns any issue into `Internal { stage: Check }` |
| kernel → `yack` | `yack` wraps `Made` in `Solid`. Only the value types in `layers.toml`'s `reexport` list (`math`: `Point2/3`, `Vec3`, `Dir3`, `Frame`, `Angle`, `Aabb`, `Rigid`, `Similarity`, `Affine`) are re-exported as they are |

### 3.4 springrep's back-edges to cut (M2)

| # | Back-edge | Fix |
|---|---|---|
| T1 | `boolean/{domain,classify}` is used by check, measure, mesh, rays, distance and topo | Move to `src/locate/` |
| T2 | `boolean/{csi,ssi,ssi/*,contact,march}` is used by check, tweak, sweep/helix and rays | Move to `src/intersect/`. `same_domain` becomes `intersect::same` |
| T3 | `topo/memo.rs` names `boolean::FaceImages` and `measure::loose_bbox` | Typed `MemoSlot` extension. The loose box goes into topo's core caches |
| T4 | `topo/codec.rs` ↔ `codec` | All encoding moves into `codec`. Memos are no longer encoded |
| T5 | blend and `edit::affine` use `step::topology::Parts` | Move it, with the STEP reader's raw assembly, heal and `mesh::solid`, to a new `build` module (yack-build, L7), since it needs check and heal, which topo cannot see |
| T6 | `boolean/snap.rs` → `edit::tweak` | Move `tweak` and `through` to `src/tweak/` (layer 7) |
| T7 | `boolean` → `sweep::extrude` for the half-space slab | Build it with `build` |
| T8 | `blend` ↔ `pick` ↔ `edit` | `EdgeName` and `edges_named` move to `topo::names`. `pick` is dissolved. `plane_of` moves to `geom` |
| T9 | `check` ↔ `measure` | `loop_area` and `fans_at_vertices` move to `topo` |
| T10 | `measure` → `boolean` (section) | `section` and `section_area` move to `boolean` (to `locate` as plane SSI later) |
| T11 | `mesh` → `boolean`, `blend`, `loft`, `primitive` (tests only) | Move those tests to `crates/yack/tests/` |
| T12 | `error` → `check` (a doc link) | Delete the link |

God files split in M3 (cap 1,500 lines, shrinking allowlist): `blend.rs` (3,464) →
`blend/{api,pick,section,ends,tools,corners}`; `boolean/mod.rs` (2,942) → `api`, `nary`,
`work/{pierce,cross,graze,faces,place,split,dedup,select,classify}`; `csi.rs` (2,537) →
`csi/{line,conic,quartic,sampled,graze}`; `nurbs.rs` (2,552) → `nurbs/{curve,surface,knots,project}`.

### 3.5 Two donors in one operation: the strategy ladder and the bake-off

Where two donors cover one operation, each donor's algorithm is a **private strategy module**
inside that op's crate:

```rust
// in yack-base
pub enum Attempt { Built(Made), Declined(String), Invalid(Error) }
pub trait Strategy { const NAME: &'static str; fn try_build(&self, input: &OpInput, ctx: &Ctx) -> Attempt; }
// in each op crate: one entry point, a fixed order, recorded per result
pub fn shell(input: &ShellInput, ctx: &Ctx) -> Result<Made> {
    ladder(&[&cadrs_offset::Shell, &springrep_tweak::Shell], input, ctx)  // order from doc/donors.md
}
```

Rules: (1) fixed, documented order; (2) `Invalid` is **final**; (3) `Declined` (unsupported,
failed, budget, or checker rejection) falls through; (4) the **first `Built` that passes `check`
wins**; (5) all declined → `Failed` with every reason joined; (6) a panic is `Internal` and final;
(7) **strategies never call each other**: each owns `<op-crate>/src/<donor>_<name>.rs`, shared
helpers live in a lower crate; (8) `Made.strategy` records the winner, logged per case by both
benchmarks; (9) this is a decision inside the kernel, which the contract allows; the adapter and
case programs never choose.

**The bake-off decides the order** (M1.10, then refreshed each milestone).

Each op with two or more donors is run donor by donor on both benchmarks. Order = more combined
passes first; ties to the exact-analytic, then the faster. `doc/donors.md` holds op, order,
passes per donor per benchmark, date. A strategy that wins no case on any benchmark or corpus
for two milestones is deleted: the ladder keeps donors where each wins, it is not parking space.

**Expected primaries** (the bake-off confirms or reorders them):

| Primary | Ops |
|---|---|
| springrep | tol, numeric, math, geom, topo, locate, check, booleans, fillets, STEP read and write, mesh |
| cadrs first | swept, delete/move/offset faces, offset, shell, thicken, draft, chamfer, heal/sew/fill, measure (`mass.rs` degree-matched quadrature) |
| per surface pair | intersect: springrep closed forms, cadrs `ssi_graph`/`ssi_cci`/`ssi_coincident`, solidcore `ssi` (free-form) |

Other allowed donors enter only as ladder strategies that win a measured bake-off row. They are
the solidcore SMLib-port kernel (Apache-2.0) and solidcraft's current kernel (solidcore-ops
beneath solidcraft-kernel). Neither is imported wholesale: solidcore scores 49.5% on conformance,
with a 2.4 s median, and solidcraft has 39 wrong on brep-bench.

**Until M3 the op crates do not exist yet.** In M1b the ladder runs inside the facade
(`crates/yack/src/ladder.rs`) over the two temporary engines, through `yack-bridge`. M4 moves
each rung into its op crate and deletes the facade copy.

**Topology both donors can use.** `yack-topo`'s `PCurve` is `{Affine, Projected, Curve2 (by the
edge's t)}`. The last variant is cadrs's OCCT convention, and springrep's STEP writer fits it
anyway. Edges and vertices carry optional per-entity tolerances, as springrep already does for
imports. During M4, each cadrs port goes through one `from_cadrs.rs` adapter in its crate, which
maps `Body`/`usize` to `Brep`/typed ids. Each adapter is deleted once its port is native.

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
pub struct Made { pub brep: Brep, pub history: History, pub strategy: &'static str }
pub enum Change { Kept, Modified, Split, Merged, Generated }
pub enum FaceOrigin { Operand { operand: u8, face: FaceId }, Segment { label: Label, end: Option<CapEnd> },
                      Edge { operand: u8, edge: EdgeId }, Vertex { operand: u8, vertex: VertexId }, Offset { operand: u8, face: FaceId } }
pub struct History { /* faces: (FaceId, Change, Vec<FaceOrigin>) sorted by id; deleted inputs */ }
impl History { pub fn compose(&self, earlier: &History) -> History; }
```

### 4.2 Public API principles

1. **Immutable values.** `Solid(Arc<Inner>)` is `Clone + Send + Sync`, valid or empty, and a
   solid *set* (its pieces may touch).
2. **One place per function.** Making from nothing: `yack::make`. Taking a solid: a method on
   `Solid`. Exchange: `yack::step`, `yack::codec`. No `_with` variants, no hidden defaults.
3. **Protocol names.** Every brep-conformance function has a same-named `yack` item. The
   exceptions are `box` → `make::block` (a Rust keyword), `normal_at`/`curvature_at`, and
   `transform_affine` for `affine`.
4. **Units.** Lengths are `f64` mm (`tol::LINEAR` is mm). Angles are `Angle`, built only by
   `deg()`/`rad()`, with no `From<f64>`. Signed lengths keep the protocol's convention: positive
   grows.
5. **Explicit frames.** Every constructor takes `&Frame`. `Frame::new(origin, x, y, z)` refuses a
   non-unit, skewed or left-handed frame, naming the axis, and never repairs it.
6. **Stamped ids.** `FaceId`, `EdgeId` and `VertexId` are `{stamp, u32}`, and an id from another
   solid gives `ForeignId`. Select first, then act: ids come from `edge_at`/`face_at` (within
   `tol::PICK`), walks or predicates. A point near two entities is `Ambiguous`; yack never
   chooses.
7. **Errors name the op, argument, value and rule.** For example, `fillet: radius 6 mm does not
   fit: the ball leaves face 7 at (10, 0, 15) by 1.2 mm`. Validation runs first in every op.
8. **Checked results (R8).** A checker rejection or caught panic is `Internal`, which the adapter
   reports as a crash. `catch_unwind` exists only in `yack`.

### 4.3 Signatures

These are frozen at M0.3. They change only through the API owner (§8). Additions requested by the
M1c case-writing loop land as stubs that return `Unsupported`.

```rust
// ---- yack::prelude::* (what every case program imports) ----
pub use crate::{Solid, Frame, Point2, Point3, Vec3, Dir3, Angle, Plane, Ray, Rigid, Similarity, Affine,
                Profile, Loop, Segment, PathPiece, Spline3, Motion, FrameLaw, Side, Tag, Error, ErrorKind, Result};
pub use crate::make::*;
pub fn deg(d: f64) -> Angle;  pub fn rad(r: f64) -> Angle;
// Every point/vector parameter is `impl Into<Point3>` / `impl Into<Vec3>` (shown below as Point3/Vec3):
// `[f64; 3]` (and `[f64; 2]` for Point2) converts, so case code writes `[-6.0, -6.0, -8.0]`. One function, no helpers.

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
impl Loop    { pub fn new(segments: Vec<Segment>) -> Result<Loop>; pub fn polygon(corners: &[[f64; 2]]) -> Result<Loop>;
               pub fn starting_at(p: Point2) -> Pen; }   // pen: .line_to(p) .arc_to(end, center) .conic_to(end, apex, w) .label(l) .close()?
impl Profile { pub fn new(outer: Loop, holes: Vec<Loop>) -> Result<Profile>;  // outer CCW, holes CW; refuses crossings, naming segments
               pub fn rectangle(min: Point2, max: Point2) -> Result<Profile>;
               pub fn circle(center: Point2, radius: f64) -> Result<Profile>;
               pub fn polygon(corners: &[[f64; 2]]) -> Result<Profile>; }
pub enum PathPiece { Line { length: f64 }, Arc { radius: f64, angle: Angle } }
impl Spline3 { pub fn new(degree: u32, knots: Vec<f64>, ctrl: Vec<Point3>) -> Result<Spline3>;
               pub fn bezier(ctrl: &[[f64; 3]]) -> Result<Spline3>; }
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
    pub fn union(&self, other: &Solid) -> Result<Solid>;  pub fn intersection(&self, other: &Solid) -> Result<Solid>;
    pub fn difference(&self, tool: &Solid) -> Result<Solid>;
    pub fn union_all(solids: &[Solid]) -> Result<Solid>;  pub fn difference_all(&self, tools: &[Solid]) -> Result<Solid>;
    pub fn split(&self, plane: &Plane) -> Result<Split>;                      // Split { front, back }; front = normal side
    // blends: one method per protocol function
    pub fn fillet(&self, edges: &[EdgeId], radius: f64) -> Result<Solid>;  pub fn fillet_radii(&self, edges: &[(EdgeId, f64)]) -> Result<Solid>;
    pub fn fillet_variable(&self, edge: EdgeId, from: Point3, r_from: f64, r_to: f64) -> Result<Solid>;
    pub fn fillet_conic(&self, edges: &[EdgeId], setback: f64, rho: f64) -> Result<Solid>;
    pub fn fillet_g2(&self, edges: &[EdgeId], offset: f64, center: f64) -> Result<Solid>;
    pub fn fillet_setback(&self, edges: &[EdgeId], radius: f64, setback: f64) -> Result<Solid>;
    pub fn fillet_setback_g2(&self, edges: &[EdgeId], radius: f64, setback: f64) -> Result<Solid>;
    pub fn fillet_faces(&self, a: FaceId, b: FaceId, radius: f64) -> Result<Solid>;
    pub fn full_round(&self, side_a: FaceId, middle: FaceId, side_b: FaceId) -> Result<Solid>;
    pub fn chamfer(&self, edges: &[EdgeId], distance: f64) -> Result<Solid>;  pub fn chamfer_two(&self, edge: EdgeId, face: FaceId, d1: f64, d2: f64) -> Result<Solid>;
    pub fn chamfer_distances(&self, edges: &[(EdgeId, f64)]) -> Result<Solid>;
    // edits (signed distances: + grows)
    pub fn offset(&self, distance: f64) -> Result<Solid>;  pub fn offset_faces(&self, faces: &[FaceId], distance: f64) -> Result<Solid>;
    pub fn delete_faces(&self, faces: &[FaceId]) -> Result<Solid>;
    pub fn move_faces(&self, faces: &[FaceId], by: &Rigid) -> Result<Solid>;
    pub fn thicken(&self, faces: &[FaceId], thickness: f64) -> Result<Solid>;
    pub fn shell(&self, open: &[FaceId], thickness: f64) -> Result<Solid>;
    pub fn draft(&self, faces: &[FaceId], origin: Point3, pull: Vec3, angle: Angle) -> Result<Solid>;
    pub fn transform(&self, m: &Similarity) -> Result<Solid>;  pub fn transform_affine(&self, m: &Affine) -> Result<Solid>;             // protocol `affine`
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
    pub fn named(&self, tag: impl Into<Tag>) -> Solid;   // optional feature name for every face it made (no tag args elsewhere)
    pub fn strategy(&self) -> &'static str;              // which ladder strategy built it (§3.5)
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

This target is 6 counted lines with the `use`; springrep's is 22, cadrs's 9 plus 3 helpers
(rustfmt `max_width = 120` in the runner):

```rust
//! The box (−6, −6, −8)–(36, 26, 0) less what a ball of radius 3 passes through along a cubic Bézier.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let block = block(&Frame::at([-6.0, -6.0, -8.0]), [42.0, 32.0, 8.0])?;
    let path = Spline3::bezier(&[[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [20.0, 20.0, 0.0], [30.0, 20.0, 0.0]])?;
    block.difference(&sphere(&Frame::WORLD, 3.0)?.swept(&Motion::Along { path, law: FrameLaw::Fixed })?)
}
```

Rules for case programs:

- Public API only. **No wrapper of a yack call.** `shared.rs` may only build *case geometry*
  that several cases name in their prose (e.g. "the ring sector between radii 8 and 12"). It
  counts toward the second metric, total runner lines ÷ cases.
- A case over 10 lines, or one needing a helper, is an `api` issue for the API owner (M1c).
  `workarounds.toml` stays empty.
- A missing feature gets a stub. Until the stub is real, the case refuses with yack's own error
  text.

---

## 5. Adapter and bench runner

### 5.1 `crates/yack-conformance` (brep-conformance adapter)

**Shape.** `impl brep_protocol::Kernel` with `type Solid = yack::Solid`, in `main.rs` (one line
per method) and `convert.rs` (frames, profiles, point → id, matrices); ≤ 500 lines (springrep 895,
cadrs ~1k). It does only the contract's four things: translate form, address by point
(`edge_at`/`face_at`), the empty solid (`make::empty()`), and handles (immutability gives them).

**Errors**, in one `fn r(e) -> Refusal`: `Invalid | NotFound | ForeignId` → invalid;
`Ambiguous | Unsupported` → unsupported; `Failed | Budget` → failed; `Internal` → `resume_unwind`
(a crash).

**Tests.** `supports()` returns `yack::SUPPORTS`, equal to the overridden methods; each listed
function passes a smoke case; xtask greps for any `if`/`match` on a returned value outside `r()`.

**Dependency.** `brep-protocol` is a git dependency on `springcraft-ai/brep-conformance` at
`12ffc75`.

**`cargo xtask conformance [--seed N|--seeds 0-3] [--family F] [--bless] [--record-digests]`**
builds the adapter in release into `bin/`, runs the pinned runner from `$YACK_REF/brep-conformance`
(default `../yack-ref`), writes `bin/conformance/seed<N>.json`, and ratchets it against
`scores/conformance/seed<N>.json` (outcome, tier, `time_ms` per case).

### 5.2 `crates/yack-bench` (brep-bench runner)

**Layout** (as brep-bench's `kernels/<k>/`): `src/cases/{category}/{name_snake}.rs`, each
`pub fn build() -> yack::Result<Solid>`; `build.rs` generates the dispatch table; `src/main.rs`.

**CLI.** `yack-bench CASE OUT.stl [OUT.step]`, plus `--version` (prints `yack <VERSION> <SOURCE>`).

| Situation | What it does |
|---|---|
| `OUT.step` given | Writes STEP with `yack::step::write` and prints `{"status":"ok"}` |
| No `OUT.step` | Prints volume, area, centroid, inertia, face kinds and validity, and writes an STL from `mesh()` |
| The case refuses | Prints `{"status":"refused","reason":"<Error Display>"}` |
| Unknown case | Exits non-zero |

Hand-written JSON (no deps). Under `BREP_BENCH_WARM` it stops itself (`kill -STOP` via
`std::process::Command`) after loading and after replying. Release: fat LTO, 1 CGU, abort, strip.

**`cargo xtask bench [--cases glob] [--bless]`**
builds `bin/release/yack-bench`, keeps a local, never-committed `yack` entry in
`$YACK_REF/brep-bench/kernels.json`, runs `harness/run.py`, ratchets outcome, time, `loc` and
strategy per case against `scores/bench.json`, and prints mean/max `loc` and runner lines ÷ cases.

**Own measures.** `yack-testkit` turns each case's TOML header into a test of yack's own measures
("geometry right, writer wrong" vs "geometry wrong"). **Upstream:** the crate is copied as
`kernels/yack/` with `yack` pinned to a git rev, and the orchestrator opens that PR.

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

**No expected value comes from a kernel**: closed forms, bounded sampled estimates or identities
only. `yack-testkit` holds the formulas (primitives, Pappus, ruled loft `hA(1+s+s²)/3`, Steiner,
chamfer `P·d²/2 − K·d³/3`, lens), a Halton membership sampler (from the MIT protocol's `model.rs`)
and `assert_volume` with the contract's bound `10·ε·A + 1e-9·V`.

### 6.2 `testkit::audit(&Solid)`: run on every result in T1, T3 and T7

1. `check()` is empty.
2. `mesh(0.01)` is closed and outward, and its volume is within `2·chord·area` of `volume()`.
3. Every face has a `History` origin. Fates and origins agree. This is enforced once History
   lands (M5). Before that, it warns.
4. `codec::decode(encode(s)).digest() == s.digest()`.
5. `step::read(step::write(s))` has the same volume within the B17 bound.
6. The operands' digests are unchanged after the operation (immutability).

### 6.3 Property suites (T3)

Zero-dependency, seeded (xorshift), with shrinking; ideas from cadrs `metamorphic_booleans`,
`stress_booleans`, `determinism`, `chamfer_generated` and solidcraft `fuzz.rs`. Checks:
`V(A∪B)+V(A∩B) = V(A)+V(B)`, `V(A∖B) = V(A)−V(A∩B)`, `A∪A = A`, `A∖A = ∅`, commutativity,
3-operand associativity, `union_all` order; invariance under rigid motions and similarities
(volume, area, Betti), affine volume × |det|; tangent-chain fillet ≡ per-edge fillet; 12 unions of
a solid turned by tiny angles stay valid. The generator places primitives and prisms on random
frames at `k·LINEAR` offsets, `k·ANGULAR` angles, tangent and coincident. Failures minimise to
`crates/yack/tests/regressions/<hash>.rs`; blessing needs a reason.

### 6.4 Ratchets (in `scores/`, committed, sorted, timestamp-free)

| File | Fails CI when |
|---|---|
| `scores/conformance/seed{0..3}.json` | Any case gets a worse outcome or a looser tier. A case's median time exceeds 2× baseline (at 1.5× it warns) |
| `scores/bench.json` | Any pass regresses, any `wrong` appears (**never waivable**: refused ranks above wrong), the mean `loc` rises, or a case's median time exceeds 2× baseline (at 1.5× it warns). A change in a case's `strategy` (the engine that answers) or `built_by` is reported, not ratcheted |
| `scores/digests/seed0.txt` | Used in M2 and M3 only: any digest changes. These milestones are pure moves, and the digests prove it |
| `scores/waivers.toml` | Holds the only allowed exceptions. Each names the case, a reason, an owner and an expiry milestone |
| `scores/eps.txt`, `scores/filesize.txt`, `scores/cadrs-loc.txt` | The count goes up |

Improvements update the baseline in the same commit (`--bless` shows the diff). On main, the
nightly job appends a summary row to `scores/history.jsonl`.

### 6.5 Determinism (T6)

Same op twice → same `digest()`; `par_map` at 1 and 8 threads → equal digests (thread count read
once, in `yack`); from M5 x86_64 = wasm32-wasip1 digests on 500 cases via `fmath`. Clippy
`disallowed_types` (`HashMap`, `HashSet`) and, from M5, `disallowed_methods` (`f64::sin`, …).

### 6.6 `cargo xtask gate`: mechanical structure gates, all required

`xtask gate` reads `cargo metadata` (with a std-only JSON reader) and the sources. Every check has
a planted-violation fixture proving it fails.

1. **Layers.** Each internal edge is in its crate's allow-list and points strictly lower.
2. **One public crate.** Only `yack` lacks `publish = false`. Nothing depends on `yack` or on a
   tool crate.
3. **Zero dependencies.** No external package is reachable from `yack`, `xtask`, `yack-bench` or
   `yack-testkit` (checked from `Cargo.lock`).
4. **No leaks.** No glob `pub use`. No `#[doc(hidden)] pub` in `yack`. No `pub use yack_*` beyond
   the `reexport` list. The surface equals `crates/yack/api.txt` (`xtask api`, source-level,
   stable Rust).
5. **Module graph.** In `yack-springrep` (M2) and any crate over 5k LOC, top-level module edges
   match `layers.toml`'s module map.
6. **File size.** Files ≤ 1,500 lines, with a shrinking allowlist. A function over 150 lines warns.
7. **Epsilons.** A float literal below 1e-2 outside `tol`/`numeric` needs `// tol: <reason>`. The
   count in `scores/eps.txt` only falls; springrep starts it at about 500.
8. **Hidden state.** No `std::env`, `thread_local!`, `static mut`, `OnceLock`/`Mutex` statics or
   `Instant`/`SystemTime` in kernel crates. Only `yack` calls `catch_unwind`.
9. **Lints.**
   - `unsafe_code = "forbid"`.
   - In kernel non-test code, `unwrap_used`, `expect_used`, `panic`, `todo`, `unimplemented` and
     `dbg_macro` are denied.
   - `indexing_slicing` warns until M5, then is denied.
   - `missing_docs` is denied on `yack`.
10. **Provenance and licence.**
    - Each file has a `yack` or provenance header (§9) naming an allowlisted donor.
    - Any next.BREP path fails.
    - A crate's `license` is `Apache-2.0` iff it holds Apache-only files.
    - The gate prints `yack`'s closure SPDX expression.
11. **Build directory.** `target-dir = "bin"`, `bin/` is ignored, and there is no `target/` and no
    `.git/worktrees`.
12. **Temporaries.** No `temporary = true` crate outlives its `delete_by` tag. Only `yack-cadrs`
    depends on `yack-shim`.
13. **The facade routes and never computes** (the solidcore-ops rule). `crates/yack` has
    `#![deny(clippy::float_arithmetic)]` plus `disallowed-methods` (`sqrt`, trig, `dot`, `cross`,
    …) in its own `clippy.toml`. The gate checks that both are present.

### 6.7 Corpora

| Corpus | Licence | Use | Lives |
|---|---|---|---|
| brep-conformance seeds 0–3, NIST | MIT; US-gov (verify each file) | T4 | `$YACK_REF`, pinned |
| brep-bench, 506 cases + `harness/mass` | MIT OR Apache-2.0 | T5; headers become oracles | `$YACK_REF`, pinned `d522d62` |
| springrep `tests/` (215) + `grazing-tooth-{a,b}.step` + `doc/design.md` anecdotes | MIT | T1 port (M1.8), one regression per anecdote | `crates/yack/tests/` |
| cadrs `metamorphic_*`, `stress_booleans`, `determinism`, `chamfer_generated`, `conformance.rs` (84), `ssi_robust`, `swept_exact`, `mass_exact`, `tess_quality`, `validity`, `fuzz_readers` | MIT | T1/T3 port (M5) | `crates/yack/tests/` |
| cad-kernel `gcs-core/tests/{brep*,arrangement,booleans,mesh*}` (~150), `step_check.rs` | MIT | T1; STEP self-verifier | `crates/yack/tests/`, `yack-testkit` |
| `solidcraft-main/tests/{oracle (92), kernel-cases, step-regressions (11)}`; `step-conformance` (18, CC0); `ssi-corpus` copy | MIT OR Apache; CC0 | T1/T5; ledgers that only shrink | `crates/yack/tests/`, `corpus/` |
| solidcore `kernel_cases/` (2,188), `solidcore-ops/tests` | Apache-2.0 | scenarios re-derived in yack's format | `crates/yack/tests/` (NOTICE) |
| ssi-corpus (jyviko `b04c8a6`, 119) | MIT OR Apache-2.0 (25 cases CC-BY: cite) | SSI gate ≥ 118/119 | fetched by `xtask corpus ssi`, SHA-256 manifest |
| solidcore `fixtures/solidcraft` (128) | Apache (facts) | Re-expressed as our own tests. **Never copy the JSON** | `crates/yack/tests/` |
| SMLib/usd-brep (237) | Apache-2.0 data | M6 nightly, via cadrs `smb.rs` | fetched by `xtask corpus smlib` (no separate repo), NOTICE kept |
| **Excluded** | – | cadrs `tests/data/docs/*.body` (no licence), `blend_tool_tolblend_c9.body` and the OCCT dataset (LGPL), STEP Tools files, all of next.BREP | never |

### 6.8 Contract notes and practices (from solidcraft's 39 wrong; `analysis/solidcraft-current.md`)

- **Chamfer distance is a setback from the edge along each face**, measured in the plane normal to
  the edge. It is not SMLib's face-offset definition, which accounted for 18 of solidcraft's wrong
  results. This is stated on the method doc. Tests run on a sphere rim and a cone, not only on planes.
- **Tolerances scale with the model or the operands.** A fixed absolute 3-D intersection tolerance
  must never snap away a feature larger than the tolerance (14 wrong). Tests: `small-cube-*` at
  s = 0.0015, `thin-lens`, a 1e-6 wedge.
- **Failure replay.** Every `Failed`/`Internal` error can capture its input solids as codec bytes
  (`Error::replay() -> Option<Replay>`). `yack-testkit::replay(path)` re-runs the call alone, and
  `xtask` saves a capture for every failing case.
- **Goldens are not certificates.** "Unchanged" (digests, ratchets) never stands in for
  "correct" (closed forms, certificates).
- **Ledgers only shrink.** Known failures and quarantines live in `scores/*.txt`. A commit that
  adds a line is refused.
- **Reader fuzz.** solidcraft `crates/testkit` (294 lines, MIT OR Apache) is copied as is into
  `yack-testkit`. A mutated STEP or codec file errors within its caps and never panics.

---

## 7. Milestones

Each milestone ends with a measured run of both benchmarks; nothing that passed may regress. A
batch is one one-tree-swarm branch merged as one PR. Packages run in parallel unless marked
"after". B's M6/M6b/M7 targets map to M4/M5 here; C's M5 targets to M5.

### M0: Skeleton (half a day, 5 packages)

**Exit:** gate and tests green, each planted violation fails; conformance completes all
unsupported, 0 crashes; brep-bench completes 506 refused, 0 crashed.

| Pkg | Owner crate(s) / files | Inputs | Exit test |
|---|---|---|---|
| M0.1 Workspace (lead, first, ~1 h) | root `Cargo.toml`, `rust-toolchain.toml` (1.98.0, the installed toolchain; M1.1 checks springrep's 1.99 pin), `clippy.toml`, `.cargo/config.toml` (`[alias] xtask = "run -p xtask --"`), `layers.toml` (§3.1 + §3.2; crates are created when first needed), `NOTICE`, `scores/` skeleton | §2, §3, §9 | `cargo build` writes to `bin/`; `cargo xtask` prints help |
| M0.2 Gate (after M0.1) | `crates/xtask/src/{main,meta,json,gate/*}.rs` | §6.6 | each of the 12 checks has a failing fixture; the gate passes on the tree |
| M0.3 API stubs (after M0.1) | `crates/yack/src/**`, `api.txt` | §4 | every §4.3 item compiles, is documented and returns `Unsupported` (queries on `empty()` are real); `xtask api` is clean |
| M0.4 Adapter (after M0.3) | `crates/yack-conformance/**`, `xtask/src/conformance.rs` | §5.1; `yack-ref/brep-conformance-springrep/src/main.rs` (MIT) | full suite runs with all unsupported and 0 crashes; `scores/conformance/seed0.json` |
| M0.5 Bench runner (after M0.3) | `crates/yack-bench/{build.rs,src/main.rs}`, `xtask/src/bench.rs` | §5.2; brep-bench `SPEC.md`; cadrs runner (MIT OR Apache) | `harness/run.py` grades 506 refused, 0 crashed; warm start; `--version` |

### M1: Scored on both benchmarks (2 days, 3 tracks in parallel, up to 12 agents)

#### M1a: springrep behind the API (≤ 1 day)

**Exit:** conformance seed 0 at 100%, seeds 1–3 no worse per case than
`runs/springrep-seed{1,2,3}.json`; the STEP writer (springrep's) live; digests recorded.

| Pkg | Owner | Inputs | Exit test |
|---|---|---|---|
| M1.1 Import springrep | `crates/yack-springrep/**` | springrep `src/` + `build.rs` @ `a03e8a1`; provenance headers | its unit tests pass in-crate on 1.98; zero deps |
| M1.2 Facade: make, profile, values | `yack/src/{make,profile,values,prelude}.rs` | §4.3 | T1 `make.rs` ports pass |
| M1.3 Facade: booleans, blends | `yack/src/{boolean,blend}.rs` | `yack-springrep` `boolean`, `blend` | T1 ports pass |
| M1.4 Facade: edits, moves, `swept` stub | `yack/src/{edit,motion}.rs` | `yack-springrep` `edit` | T1 ports pass |
| M1.5 Facade: solid, ids, queries, exchange, errors, firewall | `yack/src/{solid,select,query,exchange,error,guard}.rs` | `yack-springrep` `topo`, `measure`, `check`, `mesh`, `step`, `codec` | T1 ports pass; a foreign id gives `ForeignId` |
| M1.6 Conformance baselines | `yack-conformance`, `scores/conformance/*`, `scores/digests/seed0.txt` | M1.2–M1.5 | M1a exit; time sum recorded |

#### M1b: cadrs as a second engine (1–2 days, overlaps M1a)

**Exit:** brep-bench ≥ 500/506, 0 wrong; conformance no worse than M1a on seeds 0–3; no external
package in `Cargo.lock` outside `yack-conformance`; shimmed `yack-cadrs` alone reproduces
upstream's 504.

| Pkg | Owner | Inputs | Exit test |
|---|---|---|---|
| M1.7 Shim | `crates/yack-shim/**` | the census of nalgebra/rayon/rustc-hash calls in cadrs | cadrs's linalg unit tests pass through the shim |
| M1.8 Import cadrs (after M1.7 compiles) | `crates/yack-cadrs/**` | cadrs @ `eb5899ef` | cadrs `tests/conformance.rs` (84) pass in-crate; zero deps |
| M1.9 Bridge | `crates/yack-bridge/**` | both topologies (§3.5) | round trip on all 506 bench results and all conformance results: volume, area and classify samples equal; `check` clean on both sides |
| M1.10 Ladder and bake-off (after M1.9) | `yack/src/ladder.rs`, `doc/donors.md` | §3.5; every op both engines have | each op runs both engines alone on both benchmarks; order committed; `Made.strategy` logged per case |

#### M1c: all 506 case programs, written against the stubs (starts after M0.3)

Agents write from the case prose and rustdoc only, before kernel code exists: an API-obviousness
test. A case over 10 lines, or needing a helper, files an `api` issue; the API owner adds the
obvious form or records why it is inherently long, and never adds a second way.

**Exit:** all 506 compile, mean ≤ 9 lines, and every case passes or refuses with yack's own
error text.

| Pkg | Owner | Categories |
|---|---|---|
| M1.11 | `yack-bench/src/cases/` (claims by category) | swept, edit, offset, shell, thicken, draft, cnc |
| M1.12 | same | fillet, chamfer, boolean, tangent, singular, feature, parts, integration |
| M1.13 | same | primitives, geometry, loft, sweep, nurbs, spline, scale, fill |
| M1.14 Test ports | `crates/yack/tests/*.rs` | springrep `tests/` (215) |

### M2: Untangle springrep in place (1–2 days, 5 packages)

Use `xtask modgraph --crate yack-springrep` to do T1–T12 (§3.4).

**Exit:** 0 violations; digests bit-identical; both benchmarks unchanged.

| Pkg | Tasks | Files |
|---|---|---|
| M2.1 | T1, T2, T10 | everything under `boolean/` (one owner) |
| M2.2 | T3, T4, T9 | `topo/`, `codec.rs`, `check.rs` |
| M2.3 | T5 | `step/topology.rs` → `topo/build.rs` |
| M2.4 (after M2.1) | T6, T7 | `edit/tweak*`, `sweep/` |
| M2.5 | T8, T11, T12 | `blend.rs`, `pick.rs`, `mesh/` tests, `error.rs` |

### M3: Extract springrep into crates and harden (2–3 days, ≤ 6 agents per wave)

Each extraction is `git mv` → make the needed items `pub` → a one-line `pub use` in
`yack-springrep/src/lib.rs`.

Waves: W1 `base`, `tol`, `numeric`, `p21` · W2 `math` · W3 `geom` · W4 `intersect`, `profile`,
`topo` · W5 `locate`, `measure`, `codec` · W6 `check`, `mesh` · W7 `tweak`, `heal`, `build` ·
W8 `boolean` · W9 `blend`, `edit`, `step`, `view`. Then delete `yack-springrep`.

Each owner hardens its crate after its wave: god-file splits, named epsilons, `Invalid { arg }`,
`Budget` meters, clippy deny list.

**Exit:** `yack-springrep` deleted; DAG final apart from the cadrs temporaries; digests
bit-identical through W9; epsilons ≤ 150; no file > 1,500 lines; 0 `unwrap` in kernel crates.

### M4: Re-host cadrs as native ladder strategies (1–2 weeks, one agent per row)

Each row ports cadrs modules into their op crate as a private strategy, through that crate's
`from_cadrs.rs`, with no `yack-shim` imports. Then it moves the rung from `yack/src/ladder.rs`
into the crate and checks both benchmarks per case.

The first commit holds paths, types and the header only. Behaviour changes come in later commits.
If a native port regresses, the bridged rung stays and the gap is filed with the owning crate.

| Pkg | Owner crate | From cadrs | Exit test (B's M6 and M6b targets) |
|---|---|---|---|
| M4.1–3 (3 agents) | `swept` | `sweep/` + `sweep_moving`; `sweep_tube`; `sweep_solid` + `sweep_faces` + `sweep_mill` + `sweep_ball_pass` + `sweep_cache`; cad-kernel `swept/` closed forms | bench swept **91/91**, every pass "B-rep" (none faceted) |
| M4.4 | `edit` (delete/move/replace/split) | `defeature`, `local` replace | bench edit **52/52**; conformance `delete-faces-*` ≥ 99% on seeds 1–3 |
| M4.5–6 (2 agents) | `edit` (offset, shell, thicken) | `offset/{sharp,slab,prism}`; `offset/{round,wire}`, shell, `surfacing` thicken | bench offset **45/45**, shell **48/48**, thicken **23/23**; conformance `modify/*` ≥ 99% on seeds 1–3 |
| M4.7 | `edit::draft` | `local` draft | bench draft **15/15** |
| M4.8 | `blend` (chamfer rung) | `blend::chamfer`, `chamfer_with`, `real_edges`, `convex_chamfer` | bench chamfer 35/35; fillet ≥ cadrs's count |
| M4.9 | `heal` + `build` | `heal`, `heal_split`, `surfacing` fill/sew, `loft_build`, `curve_loft`, sweep laws | bench fill, loft, sweep, nurbs, spline ≥ M1b |
| M4.10 | `measure`, `intersect` | `mass.rs`; `ssi_graph`, `ssi_cci`, `ssi_coincident` | per bake-off; conformance no worse |
| M4.11 | `topo` + every op | `History` on every op; `compose`; cadrs `naming.rs` ladder as data | audit invariant 3 enforced; `compose` property over 5-op chains |
| M4.12 (last) | `yack` | delete `ladder.rs`, `yack-cadrs`, `yack-shim`, `yack-bridge` | gate rejects any reference; scores no worse |

**Exit:** temporaries gone, DAG final; brep-bench ≥ 503, 0 wrong; conformance `modify/*` ≥ 99% on
seeds 1–3.

### M5: Beat everyone (1–2 weeks)

| Pkg | Owner | Work | Exit test |
|---|---|---|---|
| M5.1 | `ssi` + `intersect::freeform` | solidcore `ssi` as is, wherever analytic classification says "free-form"; A/B on the corpus first | ssi-corpus ≥ 118/119; scores no worse |
| M5.2 | `numeric` + `intersect` | intervals + `isolate` as the certified CSI fallback ("refuse, never miss") | every CSI fixture: all roots, or `Failed { stage: Intersect }` |
| M5.3 | `boolean` | strategy tiers (same-solid → prism-2D → analytic → general); n-ary as one arrangement; `par_map` face pairs; sector classification (next.BREP idea) | `holes-in-one-operation` ≤ one-at-a-time; boolean −40% at 8 threads; equal digests at 1 and 8 threads |
| M5.4 | `mesh` + `gridmesh` | apex fan (`display-of-a-cone`), TessPlan, gridmesh for dense NURBS | display/mesh 100% on seeds 0–3; 318×281 deg-7 surface in < 0.5 s |
| M5.5 | `step` | units, BREP_WITH_VOIDS, FACETED_BREP, `step_check` on every write | NIST 33/33; step-conformance 17/17; "B-rep" verdict on 506/506 |
| M5.6 | `numeric` (`fmath`) | deterministic transcendentals | wasm32 = x86_64 digests on 500 cases |
| M5.7 | perf (lead assigns per crate) | top 20 slowest tests per benchmark; spline/helical sweeps | suite ≤ 0.7× springrep; `sweep-along-a-spline` ≤ 1 s; bench mean ≤ 3.2 ms |
| M5.8 | API owner + bench owners | cut case LOC (pen, array points, selection predicates) | mean ≤ 8; total ≤ 14 |
| M5.9 | owners of failing cases | the last failures, one case at a time; donor bake-off rows for solidcore/solidcraft where they win | **conformance 100% on seeds 0–3; bench ≥ 505/506, 0 wrong** |

### M6: Corpora and release

SMLib/usd-brep corpus nightly via a test-only `.smb` reader (cadrs `smb.rs`; fetched, NOTICE
kept). `cargo xtask bundle` folds the internals into one published `yack` (fallback: hidden
`yack-internal-*`). An example per function. Upstream brep-bench kernel PR.
**Exit:** `yack 0.1.0` published; README shows both scoreboards.

---

## 8. Parallel work: ownership and flow

- **Ownership is by crate.** `layers.toml` gives every crate one owner. An agent edits only files
  it claimed in crates it owns. A change to another crate's `api.txt` is a SendMessage request to
  its owner; if the owner is idle, the lead assigns it.
- **Standing roles.** API owner: `crates/yack`, `api.txt`, prelude, adapter mapping; reviews every
  signature against §4.2, writes no geometry. Harness owner: `xtask`, `yack-testkit`, `scores/`,
  `waivers.toml`. Lead: `layers.toml`, `plan/`, `doc/donors.md`, merges.

- **One tree, one builder** (`AGENTS.md`, one-tree-swarm): workers edit code only; one builder
  compiles, tests and commits in rounds; frozen-base batch branch; disjoint claims; `bin/` only.
- **Merge rule.** A batch merges when `xtask gate` is green, every ratchet holds, T0–T3 and T6 are
  green, and the API owner has approved any `api.txt` diff. A lower score blocks a merge even with
  green unit tests.
- **Peak agents per milestone.**

  M0 5 · M1 12 (M1a 6, M1b 4, M1c 3–4, sharing `crates/yack` by file) · M2 5 · M3 6 per wave ·
  M4 12 · M5 9. These counts are bounded by disjoint files, not by appetite.

---

## 9. Licence, attribution and provenance

### Licence policy (user decision, 2026-11-20)

| Code | `license` field | Obligations |
|---|---|---|
| yack's own code (`LICENSE-MIT`, `LICENSE-APACHE`, (c) 2026 yack contributors) | `MIT OR Apache-2.0` | none extra |
| A crate holding **any Apache-2.0-only donor code** | `Apache-2.0` | Every adapted file carries a header. The donor is listed in root `NOTICE`. The gate checks that the field matches the headers |

**`yack`'s effective licence** is the combination over its dependency closure:
- with any Apache-only crate in the closure, it is `(MIT OR Apache-2.0) AND Apache-2.0`, which
  in practice is Apache-2.0 with NOTICE obligations;
- with none, it is `MIT OR Apache-2.0`.

`xtask gate` prints that SPDX expression, and the M6 bundle's `license` field is set to it. When
two strategies tie in the bake-off, the dual-licensed one wins.

### Donors

| Donor | Licence used | Revision |
|---|---|---|
| springrep | MIT, (c) 2026 Springcraft, Inc. | `a03e8a1` |
| cadrs | MIT OR Apache-2.0 (either), (c) 2026 Rouan van der Ende | `eb5899ef` |
| geomsolver / cad-kernel | MIT, (c) 2026 Brandon Martin-Anderson | `694cb72` |
| `ssi`, from `solidcraft-main/solidcore` @ `4b1a170b` | MIT OR Apache-2.0 (either), h0witzer | `4b1a170b` |
| solidcore (in solidcraft-main): SMLib-port kernel, `solidcore-record` (ideas), `-tess`, `-step`, `nurbs-grid-mesh` | **Apache-2.0** (NVIDIA derivative, multi-author) | `4b1a170b` |
| solidcraft `crates/{kernel,doc,testkit}` (history, naming, fuzzer) | MIT OR Apache-2.0 | `4b1a170b` |
| truck | Apache-2.0 (no longer in solidcraft) | pin at first use |
| brep-conformance protocol | MIT, (c) 2026 Springcraft, Inc. | `12ffc75` |
| brep-bench case texts | MIT OR Apache-2.0, (c) the brep-bench contributors | `d522d62` |

`NOTICE` also keeps the musl/FreeBSD notice for `fmath`, word for word.

Current solidcraft (`yack-ref/solidcraft-main`) vendors solidcore and no longer uses truck.
`analysis/solidcraft-current.md` supersedes `analysis/solidcraft.md` and `solidcore.md`.
cadrs and `ssi` may be used under either of their licences.

### Per-file header

Each adapted file starts with this header, which the gate checks:

```rust
// yack: <one-line purpose of this file>
// Adapted from springrep@a03e8a1 src/boolean/csi.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved to yack-intersect; split into csi/{line,conic}.rs.
```

- A file with several donors gives one `Adapted from` line per donor.
- A fresh file has only the `// yack:` line.
- A file built on a next.BREP idea says
  `// Idea: next.BREP <topic>, via analysis/next-brep.md; no code seen.`

### Never allowed (the gate path-checks these)

- next.BREP code.
- Files with no licence, such as cadrs `tests/data/docs/*.body`.
- OCCT-derived LGPL data: `blend_tool_tolblend_c9.body`, the OCCT dataset.
- STEP Tools files.

Large Apache-2.0 data, such as SMLib, is fetched by `xtask corpus` with a SHA-256 manifest and
its NOTICE. It is not committed.

### Open items

- Confirm solidcore `ssi`'s "Form Studio" origin with h0witzer before M5.1. If it cannot be
  confirmed, cadrs `ssi_numeric` is the free-form backend.
- `ssi` was designed to avoid US 12,229,885. Keep that design note.

---

## 10. Risks

| Risk | L/I | Mitigation |
|---|---|---|
| The carve changes behaviour silently (memo eviction, iteration order) | M/H | The digest gate stays bit-identical through M2–M3 |
| Overfitting seed 0 | H/M | Ratchets on seeds 0–3 from M1, and brep-bench as a second yardstick |
| The temporary engines become permanent: two topologies, low cohesion | M/H | `temporary = true` with `delete_by = M4`, enforced by the gate. The `yack-cadrs` LOC ratchet falls from M3. Only the facade ladder touches them |
| The shim differs numerically from nalgebra (SVD order, pivoting) | M/M | Mirror nalgebra's conventions. M1b exit: shimmed cadrs alone reproduces upstream's 504 |
| The bridge loses exactness (cone by slant, sphere by latitude, pcurve conventions) | M/H | An exact mapping per surface kind, plus `PCurve::Curve2`. Round trip on every benchmark result |
| A ladder hides a wrong result, or doubles time when rung 1 declines slowly | M/H | Every rung's result passes the strict checker plus `audit()`. A `wrong` is never waivable. Each rung has a `Budget`. The bake-off order puts the cheaper winner first |
| Porting cadrs onto springrep's boolean and SSI changes behaviour, since it was tuned against cadrs's own | H/H | The bridge comes first, so the score exists before any port. Ports go one module at a time against a per-case ratchet. A regression keeps the bridged rung and files a gap |
| Apache-only code makes the combined `yack` Apache-2.0 | C/L | Allowed by the user. The SPDX closure is printed by the gate and stated in the bundle. MIT-compatible strategies win ties |
| brep-bench grades our STEP writer | M/H | Own-measure tests per case. `step_check` on every write |
| The LOC target pushes composition into `shared.rs` | M/L | No wrappers allowed. A second metric counts the helpers. API issues go to the API owner |
| Single-crate publishing | C/M | `xtask bundle` at M6; fallback is `yack-internal-*` |
