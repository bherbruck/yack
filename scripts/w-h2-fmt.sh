#!/bin/bash
# yack: w-h2's rustfmt of its own yack-geom files only (cargo fmt -p would also reformat files
# other workers hold).
# usage: bash scripts/w-h2-fmt.sh FILE...
set -u
cd /home/brenn/github/yack
rustfmt --edition 2024 "$@" && echo "rustfmt ok: $*"
