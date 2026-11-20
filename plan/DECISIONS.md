# Decisions

This file records the conflicts between the three architect plans and how each was settled:

- A: `yack-ref/plans/A-refactor.md`
- B: `yack-ref/plans/B-layered.md`
- C: `yack-ref/plans/C-apifirst.md`

The resulting plan is `plan/PLAN.md`. To reopen a decision, add a row below it here, dated, and
get the lead's approval.

| # | Question | A | B | C | **Decision** | Why |
|---|---|---|---|---|---|---|
| D1 | Milestone strategy | Wrap springrep wholesale behind the final API, score it, then carve with a digest gate | Port module by module into the planned layers | Write API stubs and the harness first, then port | **A's order, with C's M0.** Stubs, adapter and runner come first (half a day). Then springrep goes in wholesale, then the carve, with digests bit-identical | We get a scored kernel in about a day, and the carve never sits on the critical path. The digest gate proves the carve changed no behaviour |
| D2 | Crate count and names | 22 | 26 | 19 | **25 internal + `yack`** (PLAN §3.1). Names are single words with the `yack-` prefix | B's `ssi` and `gridmesh` stay as their own crates to keep provenance intact. A's `tweak` sits below `boolean`, which keeps the snap move mechanical (C and B rewrote it). B's `heal` and `swept` are their own crates. A's `locate`/`measure` split is kept. A's `spatial` merges into `math` (as B did). `nurbs` merges into `geom` (as A and C did). C's `naming` is data in `topo`. `inspect`/`draw` become `view` |
| D3 | Where operations live | Free functions in `build`, `combine`, `finish`, `modify` and `measure` | `make::` constructors; everything else a method on `Solid` | Spec structs with `.build(frame, tag)`; methods on `Solid` | **B.** Constructors are free functions in `yack::make`. Every op and query is a method on `Solid` | Method chains give the shortest case programs (the brep-bench `loc` target). There is one obvious place for each call. C's spec structs add a type and a line per constructor |
| D4 | Blend API | One method per protocol function, `fillet_setback(.., g2: bool)` | Same as A | A `Fillet`/`Chamfer` spec builder | **One method per protocol function, names identical, including `fillet_setback_g2`** | Rule 3 asks for a trivial 1:1 adapter. A boolean flag or a builder makes the adapter (and the reader) translate |
| D5 | Lengths | `f64` mm | `Length` newtype | `Length` via `mm()` | **`f64` mm** | The protocol and brep-bench are unitless numbers. A newtype doubles the tokens of every call and buys nothing in a fixed-mm model space. Named parameters plus docs guard against swapping arguments |
| D6 | Angles | `Angle` | `Angle` | `Angle` | **`Angle`, via `deg()`/`rad()`; no `From<f64>`** | Everyone agreed. brep-bench uses degrees and the protocol uses radians, which makes this the real unit trap |
| D7 | Error type | An enum: Invalid, Unsupported, Failed, InvalidResult, Budget, Internal | A struct with `kind`, `op`, `input`, `stage`, `reason` | A rich enum with NotFound, Ambiguous, ForeignId | **A struct `Error { kind, op, arg, stage, detail }`, with C's eight kinds** | One `Display` and one adapter `match` on `kind`. Payloads can grow without breaking callers. C's kinds map 1:1 onto refusal kinds. A checker rejection is `Internal { stage: Check }` |
| D8 | `Internal` in the adapter | Crash | Mapped to `failed` | Crash | **Crash (`resume_unwind`)** | An honest score. A bug never passes as a refusal |
| D9 | Closing the brep-bench gap (swept, edit, offset, shell) | Grafts in M5, cadrs `defeature` and `offset` only | A new `yack-swept` from geomsolver; cad-kernel offsets | cadrs and cad-kernel ideas in M3 | **A temporary `yack-cadrs` engine (M2) behind `yack` routing, used only when springrep refuses. M5 re-hosts it into DAG crates and deletes it** | cadrs scores 504/506 live with 0 wrong. Routing only refusals cannot lower the pass count. It gets about 500 within days instead of weeks. The quarantine and the LOC ratchet keep it from becoming permanent. **Needs the user's sign-off (see the end of this file)** |
| D10 | Frame constructor | `Frame::new(origin, x, y)` | Same as A | `Frame::new(origin, x, y, z)`, all checked | **C** | The protocol `Frame` carries x, y and z, so the adapter maps it 1:1. A left-handed or skewed frame is refused, never repaired |
| D11 | Offset sign | Signed | Signed | `Offset::Grow/Shrink` | **Signed `f64`, with positive growing** | The protocol is signed, so the adapter maps 1:1. The convention is documented on each method |
| D12 | Tags and naming in signatures | None (labels on profiles) | `Role` on faces | A `Tag` argument on every face-creating call | **No `Tag` argument.** Profile segment labels and `History` (M5.7) cover naming, and name resolution comes later | Fewer arguments, shorter programs. Persistent naming is not scored by either benchmark. History is still required by M5 |
| D13 | `split` result | `[Solid; 2]` | `[Solid; 2]` | `Split { front, back }` | **`Split { front, back }`** | Named fields say which side is which. The adapter orders them |
| D14 | Proptest | Zero-dependency testkit | Zero-dependency testkit | proptest as a dev-dependency | **Zero-dependency `yack-testkit::gen`** | Rule 4 in spirit. It also keeps `Cargo.lock` free of external packages, which is simple to gate |
| D15 | brep-protocol in the adapter | Allowed | Allowed (the only exception) | Allowed | **Allowed, `yack-conformance` only, git-pinned** | Reusing the suite's own wire binding removes a class of scoring bugs. It never ships |
| D16 | How to publish one crate | `xtask bundle` | Unstated | `yack-internal-*` crates | **`xtask bundle` first, `yack-internal-*` as the fallback** | "Exactly one public crate" read strictly |
| D17 | API surface check | `xtask api` from source | rustdoc JSON | rustdoc JSON | **Source-level extraction to `api.txt`** | rustdoc JSON needs nightly, and the toolchain is stable |
| D18 | Toolchain | 1.99.0 | – | – | **1.98.0 (installed) unless M1.1 finds 1.99 features** | The disk and one-tree rules: no extra toolchains unless they are needed |
| D19 | Headline seed | Seeds 0–3 gated | Seeds 1–4 as the headline | Seed 0 as the headline | **Seeds 0–3 all gated, each at 100%** | The goal is 100% on seeds 0–3. Seed 0 alone overfits |
| D20 | Where swarms coordinate | – | – | – | **`swarm/` at the repo root, git-ignored** | One-tree-swarm layout. One file per entry |

## Waiting on the user

- **D9.** Accept a temporary second engine (`yack-cadrs`, code from cadrs under MIT) inside the
  tree from M2 to M5. If you decline, the M2 packages become direct re-hosts, as in M5 of the plan.
  That costs about one to two weeks before brep-bench passes 500.
- **solidcore `ssi` provenance.** Confirm the "Form Studio" origin with h0witzer before M6.1. If
  it is not confirmed, cadrs `ssi_numeric` becomes the free-form backend instead.
