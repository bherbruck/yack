# yack: w-h1's W10 dry-run check: every name a W10-written module imports is named again in that file (an unused
# import raised springrep's clippy count in w-h1-60). Only the files W10 writes whole, and springrep's facade/api.rs.
import os, re, sys

d = sys.argv[1]
FILES = ["yack-mesh/src/pick.rs", "yack-topo/src/local.rs", "yack-measure/src/guard.rs", "yack-edit/src/glue.rs",
         "yack-edit/src/requirement.rs", "yack-step/src/exchange.rs", "yack-profile/src/loops.rs",
         "yack-bridge/src/builder.rs", "yack-springrep/src/facade/api.rs"]
bad = []
for f in FILES:
    t = open(f"{d}/crates/{f}").read()
    code = re.sub(r'"(?:[^"\\]|\\.)*"', '""', re.sub(r"//[^\n]*", "", t))  # comments and string literals out
    for m in re.finditer(r"(?m)^[ \t]*use ([^;]*);", code):
        rest = code[:m.start()] + code[m.end():]
        for piece in re.split(r"[,{}]", m.group(1)):
            piece = " ".join(piece.split())
            if not piece or piece.endswith("::"):
                continue
            n = piece.split(" as ")[-1].rsplit("::", 1)[-1]
            if n in ("self", "*"):
                continue
            if not re.search(r"(?<![\w:])" + re.escape(n) + r"\b", rest):
                bad.append(f"{f}: `{n}`")
if bad:
    sys.exit("DRY RUN REFUSES: unused imports: " + "; ".join(bad))
print("imports: every name the written modules import is used")
