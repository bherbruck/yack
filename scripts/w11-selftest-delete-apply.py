# yack: w11-window.sh self-test (w-m04): deletes yack-springrep whole and leaves a stray file in its place; the window must fail (the workspace names the crate) and bring it back whole from HEAD.
# Usage: python3 w11-selftest-delete-apply.py <yack-springrep dir>
import os, shutil, sys

root = sys.argv[1]
shutil.rmtree(root)
os.makedirs(os.path.join(root, "src"))
open(os.path.join(root, "src", "stray.rs"), "w").write("// w11-window self-test: this file must be removed by the restore\n")
print("deleted", root)
