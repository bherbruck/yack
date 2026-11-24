#!/bin/bash
# yack: lemon A/B for the apex-break attribution: builds yack-step's measure example from the
# current geom.rs without the spindle-torus apex breaks (right-sheet feet kept), measures
# lemon.step with it, and restores geom.rs exactly.
set -u
cd /home/brenn/github/yack
F=crates/yack-step/src/geom.rs; BK=$(mktemp); cp -a $F $BK
trap 'cp -a $BK $F; touch $F' EXIT
python3 - <<'PY'
p='crates/yack-step/src/geom.rs'
s=open(p).read()
a=s.index('            // A spindle torus\'s area integrand has a kink')
b=s.index('            (Surface::Cylinder { .. } | Surface::Cone { .. } | Surface::Sphere { .. } | Surface::Torus { .. }, _) => angular(lo, hi),')
open(p,'w').write(s[:a]+s[b:])
PY
touch $F
O=/home/brenn/github/yack-ref/bin/w-m11-quad
cargo build --release -p yack-step --features wip --example measure && mkdir -p $O && cp bin/release/examples/measure $O/measure-lemon-nobreaks
cp -a $BK $F; touch $F; trap - EXIT
cmp -s $BK $F && echo "GEOM restored identical"
echo "lemon WITHOUT apex breaks (grader: volume 57.29966747737253 area 76.53638161221483):"
$O/measure-lemon-nobreaks crates/yack-step/tests/data/lemon.step
