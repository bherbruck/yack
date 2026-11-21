#!/bin/bash
# The full `xtask modgraph` report for yack-springrep into swarm/res/modgraph-springrep.txt (the builder
# log keeps only the tail), and its summary on stdout.
cd /home/brenn/github/yack
cargo xtask modgraph --crate yack-springrep > swarm/res/modgraph-springrep.txt 2>&1; rc=$?
echo "modgraph rc=$rc"
echo "sites: $(grep -c '^site' swarm/res/modgraph-springrep.txt), live: $(grep '^site' swarm/res/modgraph-springrep.txt | grep -vc '(test)')"
grep -v '^site' swarm/res/modgraph-springrep.txt | tail -15
