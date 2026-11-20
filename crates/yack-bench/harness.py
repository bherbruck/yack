#!/usr/bin/env python3
# yack: runs brep-bench's own harness (harness/run.py) on yack-bench, touching nothing tracked there.
"""Run brep-bench's harness on the yack-bench runner, without editing the brep-bench clone.

    python3 crates/yack-bench/harness.py --out results.json [--case PREFIX] [--brep-bench DIR]
        [--runner PATH] [--timeout S] [--jobs N] [--scores scores/bench.json [--bless]]

brep-bench lists its kernels in kernels.json; yack is not listed there, and nothing here adds it.
This imports the harness's own modules (harness/run.py, harness/suite.py) and hands them one kernel
entry, `yack`, whose `command` and `source` are absolute paths into this repository (suite.source
joins them onto the clone's root, which leaves an absolute path as it is). Grading, timing and the
STEP measurer are the harness's own, unchanged. The result is written to --out in the harness's
results/<kernel>.json format (also for a part of the suite, which run.py itself does not record).

With --scores, the run is ratcheted against that baseline (PLAN §6.4) and exits 1 on a
regression: any case whose outcome got worse, any `wrong` at all (never waivable), a rise in the
mean `loc` of the cases run, or a case's time over 2x its baseline (1.5x warns; cases under 50 ms
in both are not timed against each other). --bless writes the baseline (merged, for a part of the
suite) instead of failing on what is worse. `loc` is a case program's lines that are neither blank
nor comments (PLAN §4.4); runner lines per case counts every src/**/*.rs file the same way.

The harness still keeps its scratch in the clone, all of it git-ignored there: results/meshes/yack/
and results/mass/yack.json. Nothing else in the clone is written.
"""
import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
DEFAULT_REF = os.path.join(os.path.dirname(REPO), "yack-ref", "brep-bench")


def entry(runner):
    """yack's kernels.json entry, local to this run."""
    return {
        "name": "yack",
        "kernel": "https://github.com/bherbruck/yack",
        "license": "MIT OR Apache-2.0",
        "backend": "Rust",
        "frontend": "Rust",
        "build": ["cargo", "build", "--release", "-p", "yack-bench"],
        "command": [runner],
        "version": [runner, "--version"],
        "source": os.path.join(HERE, "src", "cases", "{category}", "{name_snake}.rs"),
        "step": True,
        "binary": [runner],
    }


# outcomes, best first (SPEC.md "Grading"; refused ranks above wrong, and wrong is the worst)
RANK = {o: i for i, o in enumerate(["pass", "built", "refused", "missing", "timeout", "crashed",
                                    "wrong"])}
FLOOR_MS = 50.0


def loc(path):
    """A Rust file's counted lines: not blank, not a comment."""
    try:
        with open(path, encoding="utf-8") as f:
            return sum(1 for l in f if l.strip() and not l.strip().startswith("//"))
    except OSError:
        return None


def scored(rows, k):
    """Each case's ratcheted measures: outcome, loc and time (the kernel's own, where it paused)."""
    import run  # noqa: E402
    out = {}
    for r in rows:
        secs = r.get("own_seconds") if r.get("own_seconds") is not None else r.get("seconds")
        out[r["case"]] = {"outcome": r["outcome"], "loc": loc(run.source(k, r["case"])),
                          "time_ms": None if secs is None else round(secs * 1000, 1)}
    return out


def mean(xs):
    xs = [x for x in xs if x is not None]
    return round(sum(xs) / len(xs), 3) if xs else None


def ratchet(now, path, bless):
    """Compare `now` with the baseline at `path`; the problems found (empty: no regression)."""
    try:
        with open(path) as f:
            base = json.load(f).get("cases", {})
    except FileNotFoundError:
        base = {}
    problems, warnings = [], []
    for cid, n in sorted(now.items()):
        if n["outcome"] == "wrong":
            problems.append(f"{cid}: wrong (never waivable)")
        b = base.get(cid)
        if b is None:
            continue
        if RANK.get(n["outcome"], 99) > RANK.get(b["outcome"], 99):
            problems.append(f"{cid}: {b['outcome']} -> {n['outcome']}")
        t0, t1 = b.get("time_ms"), n.get("time_ms")
        if t0 is not None and t1 is not None and max(t0, t1) >= FLOOR_MS:
            if t1 > 2 * t0:
                problems.append(f"{cid}: {t0} ms -> {t1} ms (over 2x)")
            elif t1 > 1.5 * t0:
                warnings.append(f"{cid}: {t0} ms -> {t1} ms (over 1.5x)")
    common = [c for c in now if c in base]
    m0, m1 = mean(base[c].get("loc") for c in common), mean(now[c]["loc"] for c in common)
    if m0 is not None and m1 is not None and m1 > m0:
        problems.append(f"mean loc rose: {m0} -> {m1}")
    for w in warnings:
        print(f"warning: {w}")
    if bless:
        merged = dict(base)
        merged.update(now)
        with open(path, "w") as f:
            json.dump({"cases": dict(sorted(merged.items())),
                       "tally": dict(sorted(collections_count(merged).items()))},
                      f, indent=1, sort_keys=True)
            f.write("\n")
        print(f"blessed {len(now)} cases into {path}")
        return []
    if not base:
        print(f"{path}: no baseline yet; run with --bless to record one")
    return problems


def collections_count(cases):
    out = {}
    for c in cases.values():
        out[c["outcome"]] = out.get(c["outcome"], 0) + 1
    return out


def runner_lines():
    """Counted lines of every src/**/*.rs file of the runner."""
    total = 0
    for d, _, fs in os.walk(os.path.join(HERE, "src")):
        total += sum(loc(os.path.join(d, f)) or 0 for f in fs if f.endswith(".rs"))
    return total


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True, help="where to write the results JSON")
    ap.add_argument("--case", default="", help="run the cases whose id starts with this")
    ap.add_argument("--brep-bench", default=DEFAULT_REF, help="the brep-bench clone")
    ap.add_argument("--runner", default=os.path.join(REPO, "bin", "release", "yack-bench"))
    ap.add_argument("--timeout", type=float, default=60)
    ap.add_argument("--jobs", type=int, default=None)
    ap.add_argument("--scores", help="the ratchet baseline, e.g. scores/bench.json")
    ap.add_argument("--bless", action="store_true", help="write the baseline instead of failing")
    args = ap.parse_args()

    runner = os.path.abspath(args.runner)
    if not os.access(runner, os.X_OK):
        sys.exit(f"{runner} is not built: cargo build --release -p yack-bench")
    sys.path.insert(0, os.path.join(os.path.abspath(args.brep_bench), "harness"))
    import run  # noqa: E402  (brep-bench's harness/run.py)

    k = entry(runner)
    chosen = run.cases(args.case)
    if not chosen:
        sys.exit(f"no case id starts with {args.case!r}")
    os.makedirs(os.path.join(run.ROOT, "results"), exist_ok=True)
    whole = not args.case
    start, size = run.overhead(k, args.timeout, None) if whole else (None, run.binary(k))
    rows = run.run_kernel(k, chosen, args.timeout, None, run.MASS_TIMEOUT,
                          args.jobs or run.JOBS)
    result = {"kernel": k["name"], "version": run.version(k), "measured": "exact",
              "run": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "host": run.host(),
              "startup": start, "binary": size, "cases_run": args.case or "all",
              "tally": run.tally(rows), "cases": rows}
    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        json.dump(result, f, indent=1)
        f.write("\n")
    print(f"yack: {run.tally(rows)}; {run.spent(rows)}; written to {out}")
    now = scored(rows, k)
    locs = [n["loc"] for n in now.values() if n["loc"] is not None]
    if locs:
        print(f"loc: mean {mean(locs)}, max {max(locs)}; runner lines per case "
              f"{runner_lines() / len(run.cases()):.2f}")
    if args.scores:
        problems = ratchet(now, os.path.abspath(args.scores), args.bless)
        for p in problems:
            print(f"regression: {p}")
        if problems:
            sys.exit(1)


if __name__ == "__main__":
    main()
