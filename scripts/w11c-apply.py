# yack: M3 W11 (c), the lead's rulings: crates/yack-springrep goes. Its MIT licence text moves to the repo root
# (its code lives on in the kernel crates), NOTICE's springrep entry names where, and its layers.toml and
# score rows go (removal rows ride here, so HEAD never goes red). Runs after (a) w-m03's and (b) the
# yack-bridge rewiring, which take everything still reaching springrep; refuses if anything live still does.
# Usage: python3 w11c-apply.py <yack-springrep dir> [--with-external ...] [--dry]
import os, re, shutil, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(1, "/home/brenn/github/yack/scripts")
from m3_move import live_text, rd, rs_files, set_row, sub1, wr

root = os.path.abspath(sys.argv[1])
crates = os.path.dirname(root)
ws = os.path.dirname(crates)
DRY = "--dry" in sys.argv

# ---- 0. nothing outside springrep may still reach it ----
left = []
for c in sorted(os.listdir(crates)):
    d = os.path.join(crates, c)
    if d == root or not os.path.isdir(d):
        continue
    ct = os.path.join(d, "Cargo.toml")
    if os.path.exists(ct) and re.search(r"(?m)^yack-springrep\s*=", rd(ct)):
        left.append(f"crates/{c}/Cargo.toml")
    for p in rs_files(d):
        if re.search(r"\byack_springrep\b", live_text(rd(p))):
            left.append(os.path.relpath(p, ws))
if left and not DRY:
    raise SystemExit(f"W11c: still reaching springrep: {left}")
if left:
    print(f"(dry: {len(left)} files still reach springrep; (a) and (b) take them)")
# a doc comment naming a springrep path is not a dependency, but say so
for c in sorted(os.listdir(crates)):
    d = os.path.join(crates, c)
    if d != root and os.path.isdir(d):
        for p in rs_files(d):
            for k, l in enumerate(rd(p).split("\n")):
                if "yack_springrep::" in l and l.lstrip().startswith("//"):
                    print(f"note: a comment names a springrep path: {os.path.relpath(p, ws)}:{k + 1}")

# ---- 1. the licence text and NOTICE ----
lic_src, lic_dst = os.path.join(root, "LICENSE-springrep"), os.path.join(ws, "LICENSE-springrep")
lic = rd(lic_src) if os.path.exists(lic_src) else rd(lic_dst)
np_ = os.path.join(ws, "NOTICE")
OLD = ("  In: crates/yack-springrep (src/, tests/, build.rs, README.md; temporary, deleted by M3), and\n"
       "  crates/yack-p21/src/text.rs (the Part 21 string codec, adapted from src/step/part21.rs).\n"
       "  The full MIT licence text is crates/yack-springrep/LICENSE-springrep; every copied file\n"
       "  carries a provenance header.\n")
NEW = ("  In: the kernel crates its code moved to in M3, crates/yack/tests (its tests), and\n"
       "  crates/yack-p21/src/text.rs (the Part 21 string codec, adapted from src/step/part21.rs).\n"
       "  The full MIT licence text is LICENSE-springrep; every file still adapted from it carries a\n"
       "  provenance header naming its springrep path.\n")
n = rd(np_)
if NEW not in n:
    n = sub1(n, OLD, NEW, np_)

# ---- 2. layers.toml ----
lt = os.path.join(ws, "layers.toml")
s = rd(lt)


def drop_block(s, name):
    m = re.search(r"(?ms)^\[" + re.escape(name) + r"\]\n.*?\n(?=\[|# ----|\Z)", s)
    return s if not m else s[:m.start()] + s[m.end():]


s = drop_block(s, "crate.yack-springrep")
s = drop_block(s, "modules.yack-springrep")
HEAD_OLD = ("# [modules.<crate>]: top-level module = [top-level modules it may name]. yack-springrep's map is\n"
            "# the M2 target (PLAN §3.4, T1-T12): each module maps to the crate it becomes (§3.1) and may name\n"
            "# the modules of that crate's allow-list, plus siblings bound for the same crate. `pick` is\n"
            "# dissolved (T8) and has no entry; `facade` is yack's glue and may name everything.\n")
HEAD_NEW = "# [modules.<crate>]: top-level module = [top-level modules it may name].\n"
if HEAD_NEW not in s:
    s = sub1(s, HEAD_OLD, HEAD_NEW, lt)


def set_deps(s, krate, deps):
    m = re.search(r'(?ms)^\[crate\.' + re.escape(krate) + r'\]\n.*?^deps = \[([^\]]*)\]', s)
    if not m:
        raise SystemExit(f"layers.toml: no [crate.{krate}] deps")
    return s[:m.start(1)] + ", ".join(f'"{d}"' for d in deps) + s[m.end(1):]


s = set_deps(s, "yack", ["*kernel", "yack-cadrs", "yack-bridge"])
BRIDGE = ["yack-cadrs", "yack-base", "yack-check", "yack-geom", "yack-locate", "yack-math", "yack-measure", "yack-mesh",
          "yack-tol", "yack-topo", "yack-blend", "yack-boolean", "yack-build", "yack-edit", "yack-profile", "yack-step",
          "yack-view"]
s = set_deps(s, "yack-bridge", BRIDGE)
if "yack-springrep" in s:
    raise SystemExit("layers.toml still names yack-springrep: " +
                     "; ".join(l for l in s.split("\n") if "yack-springrep" in l))
s = re.sub(r"\n{3,}", "\n\n", s)

# ---- 3. write (or, --dry, show) ----
if DRY:
    import difflib
    sys.stdout.writelines(difflib.unified_diff(rd(np_).splitlines(True), n.splitlines(True), "a/NOTICE", "b/NOTICE"))
    sys.stdout.writelines(difflib.unified_diff(rd(lt).splitlines(True), s.splitlines(True), "a/layers.toml", "b/layers.toml"))
    sys.exit(0)
wr(lic_dst, lic)
wr(np_, n)
wr(lt, s)
sd = os.path.join(ws, "scores")
for f in ["eps.txt", "lints.txt", "filesize.txt", "hidden.txt"]:
    p = os.path.join(sd, f)
    if re.search(r"(?m)^yack-springrep ", rd(p)):
        set_row(p, "yack-springrep", 0)
        print("score row dropped:", f)
    if re.search(r"(?m)^crates/yack-springrep/", rd(p)):
        wr(p, "".join(l for l in rd(p).splitlines(True) if not l.startswith("crates/yack-springrep/")))
        print("file rows dropped:", f)
shutil.rmtree(root)
print("deleted:", os.path.relpath(root, ws))
