#!/bin/bash
# yack: M4.8 bake-off: the 18 chamfer_angle bench cases under cadrs and native, one bake-off binary; table in swarm/res/m48-bake.tsv.
# usage (a builder request): bash scripts/m48-bake.sh
set -euo pipefail
cd /home/brenn/github/yack
bash scripts/w-h4-bakebuild.sh m48
python3 scripts/w-h4-bake.py /home/brenn/github/yack-ref/bin/yack-bench-bakeoff-m48 swarm/res/m48-cases.txt 1 cadrs@chamfer_angle native@chamfer_angle > swarm/res/m48-bake.tsv
echo "M48 BAKE OK: swarm/res/m48-bake.tsv"
