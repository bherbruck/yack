#!/bin/bash
# yack: w-h4's W8 dry run on the real tree: copy springrep, layers.toml and scores to a scratch dir, run
# scripts/w8-apply.py there, check braces and growth, and build the m2-window request. Refuses when a
# --touches or --external file is untracked or holds uncommitted changes (the window would abort).
# usage: bash scripts/w-h4-w8-dryrun.sh SCRATCH_DIR   (writes SCRATCH_DIR/w8-req.txt)
set -eu
S=$1; D=$S/dryreal; Y=/home/brenn/github/yack
rm -rf $D; mkdir -p $D/crates
cd $Y
cp -r crates/yack-springrep $D/crates/; cp layers.toml $D/; cp -r scores $D/
(cd $D && python3 $Y/scripts/w8-apply.py $D/crates/yack-springrep)
python3 scripts/brace_check.py $(find $D/crates -name '*.rs') | tail -1
(cd crates/yack-springrep && find . -type f | sort) > $S/sr-before.txt
(cd $D/crates/yack-springrep && find . -type f | sort) > $S/sr-after.txt
T=""
for f in $(comm -23 $S/sr-before.txt $S/sr-after.txt) $(comm -13 $S/sr-before.txt $S/sr-after.txt); do T="$T crates/yack-springrep/${f#./}"; done
for f in $(comm -12 $S/sr-before.txt $S/sr-after.txt); do
  case $f in ./src/*|./tests/*) cmp -s crates/yack-springrep/$f $D/crates/yack-springrep/$f || T="$T crates/yack-springrep/${f#./}";; esac
done
# springrep's Cargo.toml (and the Cargo.lock the build rewrites) lie outside src/ and tests/, which is
# all m2-window backs up for springrep: as externals they are backed up and restored on a failure
X="$(cd $D/crates && find yack-boolean -type f | sort | sed 's|^|crates/|' | tr '\n' ' ')crates/yack-springrep/Cargo.toml Cargo.lock "
for f in layers.toml scores/eps.txt scores/lints.txt scores/filesize.txt; do cmp -s $f $D/$f || X="$X$f "; done
DIRTY=$(git status --porcelain -- $T $(for f in $X; do [ -e $f ] && echo $f; done))
if [ -n "$DIRTY" ]; then echo "DRY RUN REFUSES: untracked or uncommitted files in the window's lists:"; echo "$DIRTY"; exit 1; fi
echo "bash scripts/m2-window.sh W8 scripts/w8-apply.py \"-- moved_tests\" --touches$T --external $X" > $S/w8-req.txt
echo "DRY RUN CLEAN: $S/w8-req.txt"
