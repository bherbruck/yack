# yack: M3 W11, one window (lead's ruling): (a) w-m03's rewrite of crates/yack off springrep, (b) yack-bridge's,
# then (c) springrep's deletion with its rows. Each runs with w11-window.sh's arguments; the first to fail
# stops the chain (the window restores everything).
# Usage: python3 w11-apply.py <yack-springrep dir> [--with-external ...]
import os, subprocess, sys

here = os.path.dirname(os.path.abspath(__file__))
for step in ["w-m03-w11-apply.py", "w11b-apply.py", "w11c-apply.py"]:
    print(f"W11: {step}", flush=True)
    rc = subprocess.call([sys.executable, os.path.join(here, step)] + sys.argv[1:])
    if rc:
        sys.exit(f"W11: {step} failed (rc={rc})")
print("W11: all three applied")
