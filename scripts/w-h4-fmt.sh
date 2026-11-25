#!/bin/bash
# yack: w-h4's formatter pass over staged files (outside the tree): each .rs under the given
# directories is run through rustfmt from stdin (so out-of-line modules are not followed) with the
# workspace's rustfmt.toml, and rewritten in place; the names of the files it changed are printed.
# usage: bash scripts/w-h4-fmt.sh DIR...
set -u
Y=/home/brenn/github/yack
for d in "$@"; do
  find "$d" -name '*.rs' -type f | sort | while read -r f; do
    if rustfmt --edition 2024 --config-path $Y/rustfmt.toml < "$f" > "$f.fmt" 2> "$f.fmterr"; then
      cmp -s "$f" "$f.fmt" || { echo "FORMATTED $f"; diff "$f" "$f.fmt" | head -40; }
      mv "$f.fmt" "$f"; rm -f "$f.fmterr"
    else
      echo "RUSTFMT FAILED $f"; cat "$f.fmterr"; rm -f "$f.fmt" "$f.fmterr"
    fi
  done
done
echo "FMT DONE"
