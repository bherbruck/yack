# CLAUDE.md

@AGENTS.md

These are short reminders. `AGENTS.md` is the full rule set, and `plan/PLAN.md` is the plan.

- **Builds.** Everything builds to `bin/`. Do not set `CARGO_TARGET_DIR`. Do not create
  worktrees, clones or `target/` directories.
- **Swarm mode.** If `swarm/BRIEF.md` exists, follow it. Workers edit code and leave building,
  tests and commits to the builder.
- **The gate.** `cargo xtask gate` must be green before every commit.
- **Ownership.** Edit only the crates you own or the files you have claimed (`layers.toml`).
- **Dependencies.** Shipped crates have zero external dependencies. Provenance headers go on
  every adapted file.
- **next.BREP.** Never open its source. Use ideas from `analysis/next-brep.md` only.
- **Debugging.** Use the mcp-debugger with breakpoints, not logging statements.
