#!/bin/bash
# yack: w-h1's W9 dry run on the real tree: copy springrep, layers.toml and scores to a scratch dir, run
# scripts/w8-apply.py there, check braces and growth, and build the m2-window request. Refuses when a
# --touches or --external file is untracked or holds uncommitted changes (the window would abort).
# usage: bash scripts/w-h1-w9-dryrun.sh SCRATCH_DIR WAVE "CRATE..."   (e.g. w9a yack-blend, or w9cd "yack-step yack-view";
# writes SCRATCH_DIR/WAVE-req.txt)
set -eu
S=$1; W=$2; K=$3; D=$S/dryreal; Y=/home/brenn/github/yack
rm -rf $D; mkdir -p $D/crates
cd $Y
cp -r crates/yack-springrep $D/crates/; for k in $K; do [ -d crates/$k ] && cp -r crates/$k $D/crates/; done; cp layers.toml $D/; cp -r scores $D/; mkdir -p $D/swarm/res; cp swarm/res/w9-sites-*.txt $D/swarm/res/
sleep 1; touch $S/written.marker
(cd $D && python3 $Y/scripts/$W-apply.py $D/crates/yack-springrep)
python3 scripts/brace_check.py $(find $D/crates -name '*.rs') | tail -1
(cd crates/yack-springrep && find . -type f | sort) > $S/sr-before.txt
(cd $D/crates/yack-springrep && find . -type f | sort) > $S/sr-after.txt
T=""
for f in $(comm -23 $S/sr-before.txt $S/sr-after.txt) $(comm -13 $S/sr-before.txt $S/sr-after.txt); do T="$T crates/yack-springrep/${f#./}"; done
for f in $(comm -12 $S/sr-before.txt $S/sr-after.txt); do cmp -s crates/yack-springrep/$f $D/crates/yack-springrep/$f || T="$T crates/yack-springrep/${f#./}"; done
# the window backs up only springrep's src/ and tests/ for --touches: anything else of springrep's
# (its Cargo.toml) goes under --external (w-m04's review)
SRX=""; T2=""; for f in $T; do case $f in crates/yack-springrep/src/*|crates/yack-springrep/tests/*) T2="$T2 $f";; *) SRX="$SRX$f ";; esac; done; T=$T2
# gate 10: every .rs file the wave writes starts with `// yack:`
BAD=$( (cd $D && find $(for k in $K; do echo crates/$k; done) crates/yack-springrep -name '*.rs' -newer $S/written.marker) | while read -r f; do head -1 $D/$f | grep -q '^// yack:' || echo $f; done)
if [ -n "$BAD" ]; then echo "DRY RUN REFUSES: gate 10, no // yack: first line:"; echo "$BAD"; exit 1; fi
# gate 5: the crate's [modules.K] units are exactly its lib.rs `mod` lines (a `mod tests;` counts);
# gate 6: every file over 1,500 lines has a filesize row of its post-apply length
for k1 in $K; do
python3 - "$D" "$k1" <<'EOF' || exit 1
import os, re, sys
d, k = sys.argv[1], sys.argv[2]
lib = open(f"{d}/crates/{k}/src/lib.rs").read()
mods = set(re.findall(r"(?m)^\s*(?:pub(?:\([^)]*\))?\s+)?mod\s+(\w+)\s*;", lib))
lt = open(f"{d}/layers.toml").read()
m = re.search(r"(?ms)^\[modules\." + re.escape(k) + r"\]\n(.*?)(?:^\[|\Z)", lt)
size = sum(len(open(os.path.join(a, f)).read().split("\n")) - 1 for a, _, fs in os.walk(f"{d}/crates/{k}/src") for f in fs if f.endswith(".rs"))
if m:
    units = set(re.findall(r"(?m)^(\w+) = \[", m.group(1)))
    if units != mods:
        sys.exit(f"DRY RUN REFUSES: gate 5, [modules.{k}] units {sorted(units)} != lib.rs mods {sorted(mods)}")
    print(f"gate 5: [modules.{k}] matches lib.rs's mods {sorted(mods)}")
    # the edges too, as modgraph reads them (lead, after W9a's w-h1-45)
    sys.path.insert(0, "/home/brenn/github/yack/scripts")
    from m3_move import live_text, rs_files
    have = {u: set(re.findall(r'"([^"]+)"', v)) for u, v in re.findall(r"(?m)^(\w+) = \[([^\]]*)\]", m.group(1))}
    # m3_move.unit_edges misses grouped `use super::{x::..}` (W9a: saddle -> saddle_patch), so read the
    # heads of every `crate::`/`super::` path and `use` group here
    def heads(text, prefix):
        out = set(re.findall(r"(?<![\w:])" + prefix + r"(\w+)", text))
        for g in re.findall(r"(?<![\w:])use\s+" + prefix + r"\{([^;]*)\};", text):
            depth, cur = 0, ""
            for c in g + ",":
                if c == "{": depth += 1
                if c == "}": depth -= 1
                if c == "," and depth == 0:
                    h = re.match(r"\s*(\w+)", cur); out |= {h.group(1)} if h else set(); cur = ""
                else:
                    cur += c
        return out
    src_ = f"{d}/crates/{k}/src"
    real = {}
    for u in mods:
        refs = set()
        files = [(os.path.join(src_, u + ".rs"), 0)] + [(p, 1) for p in rs_files(os.path.join(src_, u)) if os.path.basename(p) != "tests.rs"]
        for p, depth in files:
            if os.path.exists(p):
                t = live_text(open(p).read())
                refs |= heads(t, "crate::") | heads(t, "super::" * (depth + 1))
        real[u] = set() if u == "tests" else (refs & mods) - {u}  # the module graph reads no test code (W8: tests = [])
    diff = {u: (sorted(real.get(u, set()) - have.get(u, set())), sorted(have.get(u, set()) - real.get(u, set())))
            for u in set(real) | set(have) if real.get(u, set()) != have.get(u, set())}
    if diff:
        sys.exit(f"DRY RUN REFUSES: gate 5, [modules.{k}] edges differ (unit: missing, extra): {diff}")
    print("gate 5: the map's edges are the crate's")
elif size > 5000:
    sys.exit(f"DRY RUN REFUSES: gate 5, {k} is {size} lines and layers.toml has no [modules.{k}]")
rows = dict(r.rsplit(" ", 1) for r in open(f"{d}/scores/filesize.txt").read().split("\n") if r.strip() and not r.startswith("#"))
for a, _, fs in os.walk(f"{d}/crates/{k}"):
    for f in fs:
        if f.endswith(".rs"):
            p = os.path.join(a, f); rel = os.path.relpath(p, d); n = len(open(p).read().split("\n")) - 1
            if n > 1500 and int(rows.get(rel, 0)) < n:
                sys.exit(f"DRY RUN REFUSES: gate 6, {rel} is {n} lines, row {rows.get(rel)}")
print("gate 6: rows cover every long file")
# gate 1: each Cargo dep (and dev-dep) of the crate sits in a lower layer, and springrep above the crate
layer = dict(re.findall(r"(?ms)^\[crate\.(\S+)\]\nlayer = (\d+)", lt))
cargo = open(f"{d}/crates/{k}/Cargo.toml").read()
deps = set(re.findall(r"(?m)^(yack-[\w-]+) = ", cargo))
bad = [x for x in deps if int(layer.get(x, 99)) >= int(layer[k])]
if int(layer["yack-springrep"]) <= int(layer[k]):
    bad.append(f"yack-springrep (layer {layer['yack-springrep']}) not above {k}")
if bad:
    sys.exit(f"DRY RUN REFUSES: gate 1, {k} (layer {layer[k]}) depends on {bad} at or above it")
print(f"gate 1: {k}'s deps all below layer {layer[k]}, springrep above")
EOF
done
# the target crates' new, changed and deleted files (W9c works in an existing crate)
X=""
for k in $K; do
  if [ -d crates/$k ]; then (cd crates/$k && find . -type f | sort) > $S/k-before.txt; else : > $S/k-before.txt; fi
  (cd $D/crates/$k && find . -type f | sort) > $S/k-after.txt
  for f in $(comm -3 $S/k-before.txt $S/k-after.txt | tr -d '\t'); do X="$X crates/$k/${f#./}"; done
  for f in $(comm -12 $S/k-before.txt $S/k-after.txt); do cmp -s crates/$k/$f $D/crates/$k/$f || X="$X crates/$k/${f#./}"; done
done
X="${X# } "
for f in layers.toml scores/eps.txt scores/lints.txt scores/filesize.txt; do cmp -s $f $D/$f || X="$X$f "; done
X="$SRX$X"
# scores another worker holds hunks in go under --dirty-ok (lead's ruling, W7); anything else dirty refuses
DOK=""; for f in scores/eps.txt scores/lints.txt scores/filesize.txt; do case " $X " in *" $f "*) [ -n "$(git status --porcelain -- $f)" ] && DOK="$DOK $f";; esac; done
DIRTY=$(git status --porcelain -- $T $(for f in $X; do [ -e $f ] && case " $DOK " in *" $f "*) ;; *) echo $f;; esac; done))
if [ -n "$DIRTY" ]; then echo "DRY RUN REFUSES: untracked or uncommitted files in the window's lists:"; echo "$DIRTY"; exit 1; fi
# yack-step's measurer is linted and tested with its feature on, and the moved STEP round trips are an
# integration test (`--lib` skips them): m2-window's --extra (w-m04) runs each inside the window
EXTRA=""
case " $K " in *" yack-step "*) EXTRA=' --extra "cargo clippy -p yack-step --features wip --all-targets -- -D warnings" --extra "cargo test --release -p yack-step --features wip" --extra "cargo test --release -p yack-step --test round_trips"';; esac
echo "bash scripts/m2-window.sh $(echo $W | tr a-z A-Z) scripts/$W-apply.py \"-- moved_tests\" --no-pre --touches$T${DOK:+ --dirty-ok$DOK} --external $X$EXTRA" > $S/$W-req.txt
echo "DRY RUN CLEAN: $S/$W-req.txt"
