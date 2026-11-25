# yack: M3 W9c-0, W9c and W9d in one window (lead's ruling): yack-step's measurer under src/measure/, then
# springrep's step into yack-step, then draw, overlap and probe into yack-view, each by its own apply,
# in order; any refusal stops the chain (the window restores everything).
# Usage: python3 w9cd-apply.py <yack-springrep dir> [--with-external ...]
import os, subprocess, sys

here = os.path.dirname(os.path.abspath(__file__))
root = os.path.abspath(sys.argv[1])
rest = sys.argv[2:]
steps = [("w9c0-apply.py", [os.path.join(os.path.dirname(root), "yack-step")]),
         ("w9c-apply.py", [root] + rest),
         ("w9d-apply.py", [root] + rest)]
for script, args in steps:
    print(f"== {script}", flush=True)
    r = subprocess.run([sys.executable, os.path.join(here, script)] + args)
    if r.returncode != 0:
        raise SystemExit(f"{script} refused (exit {r.returncode}): the chain stops")
print("w9cd: all three applied")
