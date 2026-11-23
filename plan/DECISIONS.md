# Decisions

This file records the conflicts between the three architect plans (their final revisions: A rev 2, B
rev 2, C rev 2) and how each was settled:

- A: `yack-ref/plans/A-refactor.md`
- B: `yack-ref/plans/B-layered.md`
- C: `yack-ref/plans/C-apifirst.md`

The resulting plan is `plan/PLAN.md`. To reopen a decision, add a row below it here, dated, and
get the lead's approval.

| # | Question | A | B | C | **Decision** | Why |
|---|---|---|---|---|---|---|
| D1 | Milestone strategy | M1a: springrep wholesale behind the API. M1b: cadrs under a shim, plus a bridge. Then carve with a digest gate | Interfaces first, then donors moved into place | Stubs and harness first; all 506 cases written against the stubs | **C's M0, then A's M1a and M1b in parallel with C's M1c (506 cases against the stubs). Then the carve, with digests bit-identical, then re-hosting cadrs** | Both benchmarks get scored in about 2 days, and the carve is never on the critical path. Writing cases first tests how obvious the API is before any kernel code exists |
| D2 | Crate count and names | 22 | 26 | 19 | **25 internal + `yack`** (PLAN §3.1). Names are single words with the `yack-` prefix | B's `ssi` and `gridmesh` stay as their own crates to keep provenance intact. A's `tweak` sits below `boolean`, which keeps the snap move mechanical (C and B rewrote it). B's `heal` and `swept` are their own crates. A's `locate`/`measure` split is kept. A's `spatial` merges into `math` (as B did). `nurbs` merges into `geom` (as A and C did). C's `naming` is data in `topo`. `inspect`/`draw` become `view` |
| D3 | Where operations live | Free functions in `build`, `combine`, `finish`, `modify` and `measure` | `make::` constructors; everything else a method on `Solid` | Spec structs with `.build(frame, tag)`; methods on `Solid` | **B.** Constructors are free functions in `yack::make`. Every op and query is a method on `Solid` | Method chains give the shortest case programs (the brep-bench `loc` target). There is one obvious place for each call. C's spec structs add a type and a line per constructor |
| D4 | Blend API | One method per protocol function, `fillet_setback(.., g2: bool)` | Same as A | A `Fillet`/`Chamfer` spec builder | **One method per protocol function, names identical, including `fillet_setback_g2`** | Rule 3 asks for a trivial 1:1 adapter. A boolean flag or a builder makes the adapter (and the reader) translate |
| D5 | Lengths | `f64` mm | `Length` newtype | `Length` via `mm()` | **`f64` mm** | The protocol and brep-bench are unitless numbers. A newtype doubles the tokens of every call and buys nothing in a fixed-mm model space. Named parameters plus docs guard against swapping arguments |
| D6 | Angles | `Angle` | `Angle` | `Angle` | **`Angle`, via `deg()`/`rad()`; no `From<f64>`** | Everyone agreed. brep-bench uses degrees and the protocol uses radians, which makes this the real unit trap |
| D7 | Error type | An enum: Invalid, Unsupported, Failed, InvalidResult, Budget, Internal | A struct with `kind`, `op`, `input`, `stage`, `reason` | A rich enum with NotFound, Ambiguous, ForeignId | **A struct `Error { kind, op, arg, stage, detail }`, with C's eight kinds** | One `Display` and one adapter `match` on `kind`. Payloads can grow without breaking callers. C's kinds map 1:1 onto refusal kinds. A checker rejection is `Internal { stage: Check }` |
| D8 | `Internal` in the adapter | Crash | Mapped to `failed` | Crash | **Crash (`resume_unwind`)** | An honest score. A bug never passes as a refusal |
| D9 | Two donors for one op | Bake-off picks exactly one donor per op; no runtime fallback | Strategy ladder: each donor is a private `Strategy` in the op's crate | One primary per op; the other donor's cases gate it | **B's ladder as the long-term mechanism, with A's bake-off (`doc/donors.md`) setting the order.** `Invalid` is final, `Declined` falls through, and the first result that passes `check` wins. Strategies never call each other. Each result records its strategy. A strategy that wins nothing for two milestones is deleted. In M1b–M3 the ladder lives in the facade over the temporary engines. M4 moves each rung into its op crate | Each donor keeps the cases it wins (cadrs: 504 bench; springrep: 100% conformance) without coupling the donors together. The bake-off keeps the order measured, not guessed |
| D9b | Temporary engines | `yack-springrep`, `yack-cadrs`, `yack-shim` (nalgebra-shaped, via Cargo renames), `yack-bridge` | No temporaries; port directly | Port directly | **A's four temporaries, marked `temporary = true` with `delete_by` (springrep: M3; cadrs, shim and bridge: M4), enforced by the gate** | Fastest route to ≥ 500 bench and 100% conformance. The gate makes sure they disappear |
| D9c | Primary donor per layer | springrep skeleton; cadrs for modify and sweeps | springrep core; cadrs for swept, edit, measure, heal, chamfer | the same | **B's table** (PLAN §3.5); the bake-off may reorder it | All three agree |
| D10 | Frame constructor | `Frame::new(origin, x, y)` | Same as A | `Frame::new(origin, x, y, z)`, all checked | **C** | The protocol `Frame` carries x, y and z, so the adapter maps it 1:1. A left-handed or skewed frame is refused, never repaired |
| D11 | Offset sign | Signed | Signed | `Offset::Grow/Shrink` | **Signed `f64`, with positive growing** | The protocol is signed, so the adapter maps 1:1. The convention is documented on each method |
| D12 | Tags and naming in signatures | None (labels on profiles) | `Role` on faces | A `Tag` argument (`impl Into<Tag>`) on every face-creating call | **No `Tag` argument on ops.** An optional `Solid::named(impl Into<Tag>)` takes C's form. Profile segment labels and `History` (M4.11) cover the rest | Every op call is shorter. Naming stays available but is never mandatory |
| D12b | Line-cutting API forms | – | – | `Loop::polygon_mm`, a pen builder, `Spline3::bezier_mm`, `Solid::swept(&Motion)`, `impl Into<Tag>` | **Adopted, minus the `_mm` suffix:** `Loop::polygon(&[[f64;2]])`, `Loop::starting_at(..)` pen, `Spline3::bezier(&[[f64;3]])`, `swept(&Motion)`. Every point parameter is `impl Into<Point3>`, so `[x, y, z]` literals work | Lengths are plain mm (D5), so a suffix adds nothing. Array literals remove the `p3()` helper |
| D12c | Bench line targets | – | < 40 lines per case | mean ≤ 8; total ≤ 14; > 10 lines → API issue; no wrappers | **C** | The lowest mean on the board, earned by the API rather than by helper files |
| D12d | Wrong results | – | – | never waivable | **Never waivable** | A refusal ranks above a wrong result |
| D13 | `split` result | `[Solid; 2]` | `[Solid; 2]` | `Split { front, back }` | **`Split { front, back }`** | Named fields say which side is which. The adapter orders them |
| D14 | Proptest | Zero-dependency testkit | Zero-dependency testkit | proptest as a dev-dependency | **Zero-dependency `yack-testkit::gen`** | Rule 4 in spirit. It also keeps `Cargo.lock` free of external packages, which is simple to gate |
| D15 | brep-protocol in the adapter | Allowed | Allowed (the only exception) | Allowed | **Allowed, `yack-conformance` only, git-pinned** | Reusing the suite's own wire binding removes a class of scoring bugs. It never ships |
| D16 | How to publish one crate | `xtask bundle` | Unstated | `yack-internal-*` crates | **`xtask bundle` first, `yack-internal-*` as the fallback** | "Exactly one public crate" read strictly |
| D16b | Licence (user, 2026-11-20) | MIT | MIT | MIT | **Own code `MIT OR Apache-2.0`. A crate holding Apache-only donor code is `Apache-2.0`, with a header on each file plus `NOTICE`. The effective licence of `yack` is its closure's SPDX expression, printed by the gate** | The user allows Apache donors: solidcore's whole kernel and STEP crates, truck, SMLib data. Still banned: next.BREP, files with no licence, OCCT LGPL data |
| D16c | Extra donors (solidcore SMLib port, current solidcraft, truck) | – | – | – | **Ladder strategies only, and only where one wins a bake-off row. No wholesale import** | solidcore scores 49.5% on conformance with a 2.4 s median; solidcraft has 39 wrong on bench. Neither is a skeleton |
| D16d | Final targets | conformance ≥ 99.8% | bench ≥ 505, conformance ≥ 99.9% (M7) | 100% on 4 seeds, ≥ 505/506, LOC ≤ 8, ≤ 3.2 ms (M5) | **C's M5 targets as the M5 exit; B's per-category M6/M6b targets as the M4 exit** | The most ambitious targets that are still measurable |
| D17 | API surface check | `xtask api` from source | rustdoc JSON | rustdoc JSON | **Source-level extraction to `api.txt`** | rustdoc JSON needs nightly, and the toolchain is stable |
| D18 | Toolchain | 1.99.0 | – | – | **1.98.0 (installed) unless M1.1 finds 1.99 features** | The disk and one-tree rules: no extra toolchains unless they are needed |
| D18b | cadrs revision | `eb5899ef` | `eb5899ef` (board pin) | – | **`eb5899ef`**. The local checkout is already at `eb5899e` | It matches the live 504 |
| D19 | Headline seed | Seeds 0–3 gated | Seeds 1–4 as the headline | Seed 0 as the headline | **Seeds 0–3 all gated, each at 100%** | The goal is 100% on seeds 0–3. Seed 0 alone overfits |
| D20 | Where swarms coordinate | – | – | – | **`swarm/` at the repo root, git-ignored** | One-tree-swarm layout. One file per entry |

| D21 | Facade discipline (from solidcraft-current) | – | – | – | **`crates/yack` routes and never computes:** `deny(clippy::float_arithmetic)` plus a maths `disallowed-methods` list, checked as gate 13 | This is solidcore-ops' rule, and it keeps geometry out of the facade |
| D22 | Failure replay, goldens vs certificates, shrinking ledgers | – | – | – | **Adopted** (PLAN §6.8) | Every failure becomes a standalone repro. "Unchanged" never stands in for "correct" |
| D23 | Chamfer distance definition | – | – | – | **A setback from the edge along each face, in the plane normal to the edge. Not SMLib's face offset** | The offset definition caused 18 of solidcraft's wrong results on brep-bench |
| D24 | Tolerance scale | absolute (springrep) | `Tolerances` in `Ctx` | absolute | **Absolute `LINEAR` stays for agreement checks, but coincidence and snap thresholds scale with the operands and never delete a feature larger than the tolerance** | 14 of solidcraft's wrong results came from an absolute snap |
| D25 | `ssi` source | solidcore | solidcore | solidcore | **The `solidcraft-main/solidcore` snapshot @ `4b1a170b`**, under either licence | solidcore plans to delete `ssi` |
| D26 | springrep's own foundation modules in M3 | – | – | – | **Move first, converge in hardening.** M3's W1'/W2' extract springrep's `numeric` into a new `yack-numeric` (pure move), fold its `tol` into `yack-tol` (the same constants, plus `param` and `between`), and move its `error` and `math` verbatim as modules `yack_base::kernel` and `yack_math::kernel`, re-exported by springrep as `error` and `math`. Each crate's M3 hardening then moves its own code onto `yack_base::Error` (D7) and yack-math's checked types; the `kernel` modules are deleted when the last user converts | The waves must be pure moves with identical digests, and the two designs (springrep's `InvalidInput(String)` errors and pub-field `Vec3`, against D7's struct and yack-math's `Dir3` frames) cannot merge without changing behaviour. Two vocabularies live side by side for one milestone, labelled as such, rather than blocking every wave behind a rewrite |

## Decided by the lead (2026-11-20)

- **D9 / D9b: yes.** The temporary cadrs engine goes in, with the LOC ratchet, and is deleted at
  the end of re-hosting (M4 in PLAN).
- **Toolchain: 1.98.0.** Bump only if M1.1 proves 1.99 is needed.
- **brep-protocol: OK** as a git dependency pinned at `12ffc75`, in the adapter only.
- **Runner crate name:** `yack-bench`.
- **Licence:** MIT OR Apache-2.0. Apache-only donors (the solidcore kernel and its STEP and tess
  crates, truck, the SMLib corpus) are allowed in crates marked `Apache-2.0`, with NOTICE
  entries. No separate corpus repo is needed.

- **Faceted results refuse (2026-11-20).** An exact op never returns a faceted approximation:
  a rung whose result has `is_exact()` false declines, and if no rung is exact the op fails with
  "only a faceted approximation is available". This supersedes the round-4 rule that let faceted
  `together` results through. Reason: ladder9 graded two faceted sweeps as `wrong` (AGENTS rule 8).
  `Solid::is_exact()` stays public for imported solids.

- **Lazy bridge (2026-11-20).** A solid built by cadrs stays in cadrs form; it is bridged to
  springrep only when a springrep rung, guard or query needs it. step::write and volume/area use
  the cadrs form directly. A cadrs-held answer must pass cadrs's Full check (rule 8). Digest: the
  springrep digest when the bridge succeeds, otherwise a domain-tagged digest of cadrs's canonical
  STEP. Reason: springrep can't carry cadrs's uv pcurves (9 tumbling-torus cases, w-m11 -82).
  An explicit 2-D B-spline pcurve kind is the M3 yack-topo answer.

## Still open (not blocking)

- **solidcore `ssi` provenance.** Confirm the "Form Studio" origin with h0witzer before M5.1. If
  it is not confirmed, cadrs `ssi_numeric` is the fallback.
