# AGENTS.md: rules for coding agents working on yack

yack is an exact B-rep kernel in pure, safe Rust, licensed MIT OR Apache-2.0.

- The plan is `plan/PLAN.md`, and it is authoritative.
- The settled design questions are in `plan/DECISIONS.md`. Do not reopen them in code. Ask the
  lead instead.
- Reference repos live in `/home/brenn/github/yack-ref`, which this file calls `$YACK_REF`.

## Hard rules

1. **One working tree, one target dir.**
   - Never create a git worktree, a clone, a second checkout, or a `target/` directory.
   - Never set `CARGO_TARGET_DIR`.
   - Everything builds to `bin/`, as set by `.cargo/config.toml`. `bin/` is git-ignored and never
     committed.
   - Cargo locks `bin/`, so concurrent builds wait for each other. That is expected.
2. **Swarm mode.** If `swarm/BRIEF.md` exists, you are in a one-tree-swarm. Follow it.
   - Workers **only edit code**. They never run cargo, tests or formatters, and never touch the
     git index.
   - The builder alone compiles, tests and commits.
   - Claim exact files in `swarm/claims/<task>.txt` before editing.
   - No two agents hold the same file.
   - The tree must compile after every edit you save.
3. **Ownership.** `layers.toml` names the owner of every crate.
   - Edit only files in crates you own, or files you have claimed.
   - Need a change in another crate's public surface (`crates/<c>/api.txt`)? Send its owner a
     request with SendMessage. Do not edit it yourself.
   - `crates/yack` (the public API) belongs to the API owner.
   - `layers.toml` and `plan/` belong to the lead.
4. **Crate DAG.**
   - A crate depends only on the crates listed for it in `layers.toml`, and they must be in a
     strictly lower layer.
   - No glob re-exports.
   - No `pub` item that exists only so that a sibling crate can reach inside.
5. **Zero external dependencies** in every crate except `yack-conformance` (which may use
   `brep-protocol` only). Do not add a dependency. Write the code, or ask the lead.
6. **Kernel crate hygiene.**
   - No `unsafe`.
   - No `unwrap`, `expect`, `panic!`, `todo!` or `dbg!` outside tests.
   - No `HashMap`/`HashSet`; use `BTreeMap`/`BTreeSet`.
   - No `std::env`, `thread_local!`, `static mut`, global `OnceLock`/`Mutex`, `Instant` or
     `SystemTime`.
   - No tolerance literal below 1e-2 outside `yack-tol`/`yack-numeric` unless the line carries
     `// tol: <reason>`.
   - No file over 1,500 lines.
   - Only `yack` calls `catch_unwind`.
   - `crates/yack` **routes and never computes**. It has no float arithmetic and no maths calls
     (`clippy::float_arithmetic` is denied there). Geometry belongs in a kernel crate.
   - Chamfer distances are setbacks along each face, not face offsets.
   - Snap and coincidence thresholds never delete a feature larger than the tolerance.
7. **Strategy ladders** (`plan/PLAN.md` §3.5). When two donors implement one op, each is a
   private `Strategy` module in that op's crate.
   - Strategies never call each other.
   - `Invalid` is final, and `Declined` falls through to the next strategy.
   - The order comes from `doc/donors.md`. Do not reorder it without new bake-off numbers.
8. **Every result is valid or an error.** Operations end in `finish()`, which runs the checker.
   Return a refusal (`Unsupported` or `Failed`) rather than a doubtful solid. On brep-bench, a
   wrong answer is worse than a refusal.
9. **Benchmark honesty.**
   - Case programs and the adapter use the public `yack` API only.
   - Never compute an expected answer.
   - Never hand-build geometry.
   - Never catch a panic to turn it into a refusal.
   - Never special-case a test id.
   - `workarounds.toml` stays empty.
   - A `wrong` result can never be waived.
   - A case program over 10 lines is an API issue for the API owner, not a helper.
   - A missing feature is an API gap: file it with the API owner.
10. **Licences and provenance.**
   - yack's own code is `MIT OR Apache-2.0`.
   - Allowed donors:
     - springrep (MIT)
     - cadrs (MIT OR Apache)
     - cad-kernel / geomsolver (MIT)
     - all of solidcore, including the SMLib-port kernel and STEP crates (Apache-2.0)
     - current solidcraft (`$YACK_REF/solidcraft-main`; it vendors solidcore, no truck; take `ssi` from there)
     - truck (Apache-2.0)
     - the SMLib corpus (Apache-2.0, fetched)
   - A crate that contains **any Apache-2.0-only code** must say `license = "Apache-2.0"` in its
     `Cargo.toml`, and its donor must be in `NOTICE`.
   - **Never use:**
     - next.BREP code (copyright-assignment clause). Never open
       `$YACK_REF/next.BREP.io_RUST_BREP_KERNEL`; use only `$YACK_REF/analysis/next-brep.md`.
     - files without a licence (e.g. cadrs `tests/data/docs/*.body`).
     - OCCT-derived LGPL data.
   - Extra donors (solidcore kernel, solidcraft, truck) enter only as ladder strategies that win a
     measured bake-off row (`doc/donors.md`).
   - Every adapted file starts with a provenance header:
     ```rust
     // yack: <one-line purpose>
     // Adapted from <donor>@<rev> <path> (<licence>, (c) <holder>).
     // Changes: <what changed>.
     ```
     A fresh file has only the `// yack:` line.
   - When porting, the first commit changes only paths, types and the header. Behaviour changes
     go in later commits.
   - Add the donor to `NOTICE` if it is not already listed.
11. **No external posting.** Do not open PRs, issues or comments on GitHub or brepbench.org.
    Draft them in `swarm/done/` and let the orchestrator post.

## Build and run

These commands are for the builder, or for an agent working alone.

| Command | What it does |
|---|---|
| `cargo build` / `cargo build --release` | Build into `bin/` |
| `cargo check -p yack-<crate>` | Fast per-crate check |
| `cargo test -p yack-<crate>` / `cargo test --workspace` | Run tests |
| `cargo xtask gate` | The structure gates: layers, one public crate, zero deps, leaks, file size, epsilons, hidden state, provenance, build dir. **Must be green before every commit** |
| `cargo xtask api` | Diff the `yack` public surface against `crates/yack/api.txt` |
| `cargo xtask conformance --seed 0` | brep-conformance on seed 0, ratcheted against `scores/conformance/` |
| `cargo xtask conformance --seeds 0-3` | Seeds 0–3 |
| `cargo xtask conformance ... --family booleans` | Restrict to one family |
| `cargo xtask conformance ... --bless` | Accept improvements |
| `cargo xtask bench` | brep-bench through `$YACK_REF/brep-bench/harness/run.py`, ratcheted against `scores/bench.json` |
| `cargo xtask bench --cases 'swept/*'` | Restrict to matching cases |
| `cargo xtask bench ... --bless` | Accept improvements |
| `cargo xtask modgraph --crate yack-springrep` | Module-level DAG check (M2) |
| `cargo xtask fuzz --seconds 60` | Seeded robustness fuzz |
| `cargo xtask corpus <name>` | Run a corpus |

Rules for the ratchets:

- A worse outcome on any case fails, including on a seed you did not touch.
- Bless only real improvements, in the same commit as the change that caused them.
- An exception needs a row in `scores/waivers.toml` with a reason, an owner and an expiry
  milestone, and the lead's approval.

## Debugging

Use the mcp-debugger with breakpoints. Do not add logging statements to kernel code.

## Commits

- Small commits, one package or sub-step each, made by the builder (in swarm mode) with explicit
  paths.
- Each message names the package id from the plan (for example `M1.3:`) and ends with the
  co-author line given by the session.
- Never commit `bin/`, `swarm/`, or edits to `$YACK_REF`.
