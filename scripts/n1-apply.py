# M3 N1 (lead's ruling after W4): W1'-W3 made every `pub(crate)` item of the moved code `pub`; rule 4
# wants `pub` only where code outside the crate names the item. This narrows yack-numeric,
# yack_math::kernel and yack-geom back to `pub(crate)` for the items that were `pub(crate)` in
# springrep and that no live code outside the crate names. Items that only other crates' tests name
# are listed and left alone (their tests would need to move first). Visibility only.
# Usage: python3 n1-apply.py <yack-springrep dir>
import os, re, subprocess, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from m3_move import live_text, rd, rs_files, wr

root = os.path.abspath(sys.argv[1])
crates = os.path.dirname(root)
REPO = "/home/brenn/github/yack"

# (the moved code's files, {file: (rev before the move, springrep origin)})
WAVES = [
    ("yack-numeric/src", {"lib.rs": ("724eebf^", "crates/yack-springrep/src/numeric.rs")}),
    ("yack-math/src", {"kernel.rs": ("4d889ef^", "crates/yack-springrep/src/math.rs"),
                       "kernel/bvh.rs": ("4d889ef^", "crates/yack-springrep/src/math/bvh.rs")}),
    ("yack-geom/src", {f: ("6c0ae6b^", "crates/yack-springrep/src/geom/" + ("mod.rs" if f == "lib.rs" else f))
                       for f in ["lib.rs", "bspline.rs", "chord.rs", "curve.rs", "intersection.rs", "nurbs.rs",
                                 "nurbs/seeds.rs", "surface.rs"]}),
]
ITEM = re.compile(r"pub\(crate\)\s+(?:(?:const|unsafe)\s+)*(fn|struct|enum|const|static|type|trait|mod)\s+(\w+)")
FIELD = re.compile(r"(?m)^\s+pub\(crate\)\s+(\w+)\s*:")
USE = re.compile(r"(?m)^(\s*(?:#\[cfg\(test\)\]\s*)?)pub\(crate\) use ([^;]+);")


def show(rev, path):
    return subprocess.check_output(["git", "-C", REPO, "show", f"{rev}:{path}"]).decode()


def texts(own):
    """(live, test) code of every crate but the moved files `own`."""
    live, test = [], []
    for c in sorted(os.listdir(crates)):
        for sub in ("src", "tests"):
            d = os.path.join(crates, c, sub)
            if not os.path.isdir(d) or c == "xtask":
                continue
            for p in rs_files(d):
                if p in own:
                    continue
                t = rd(p)
                if sub == "tests" or os.path.basename(p) == "tests.rs":
                    test.append(t)
                else:
                    lt = live_text(t)
                    live.append(lt)
                    test.append(t)
    clean = lambda x: re.sub(r"//[^\n]*", "", "\n".join(x))
    return clean(live), clean(test)


def ids(clean):
    path_ids = set(re.findall(r"(?<=::)\s*([A-Za-z_]\w*)", clean))
    for u in re.findall(r"\buse\s+([^;]+);", clean):
        path_ids |= set(re.findall(r"[A-Za-z_]\w*", u))
    member_ids = path_ids | set(re.findall(r"\.\s*([A-Za-z_]\w*)", clean)) | set(
        re.findall(r"(?<![\w.])([a-z_]\w*)\s*:(?!:)", clean))
    return path_ids, member_ids


report = {}
for cdir, files in WAVES:
    own = {os.path.join(crates, cdir, f) for f in files}
    live, test = texts(own)
    lp, lm = ids(live)
    tp, tm = ids(test)
    narrowed, test_only = [], []
    for f, (rev, orig) in files.items():
        p = os.path.join(crates, cdir, f)
        t = rd(p)
        o = show(rev, orig)
        names = [(k, n) for k, n in ITEM.findall(o)] + [("field", n) for n in FIELD.findall(o)]
        for kind, n in names:
            live_set, test_set = (lp, tp) if kind == "mod" else (lm, tm)
            if n in live_set:
                continue
            if n in test_set:
                test_only.append(f"{f}:{kind} {n}")
                continue
            pat = (r"(?m)^(\s+)pub(\s+" + re.escape(n) + r"\s*:)") if kind == "field" else \
                  (r"\bpub(\s+(?:(?:const|unsafe)\s+)*" + kind + r"\s+" + re.escape(n) + r"\b)")
            t2, k = re.subn(pat, (r"\1pub(crate)\2" if kind == "field" else r"pub(crate)\1"), t, count=1)
            if k:
                t = t2
                narrowed.append(f"{f}:{kind} {n}")
        # re-exports that were `pub(crate) use`: names nothing outside reaches go back, split if mixed
        for m in USE.finditer(o):
            pre, spec = m.group(1), m.group(2)
            flat = re.sub(r"\s+", " ", spec).strip()
            cur = re.search(r"(?m)^" + re.escape(pre) + r"pub use " + re.escape(spec) + ";", t)
            if not cur:
                continue
            head, _, inner = flat.partition("::{") if "::{" in flat else (flat.rsplit("::", 1)[0], "", flat.rsplit("::", 1)[1])
            inner = inner.rstrip("}").strip()
            names_ = [x.strip() for x in inner.split(",") if x.strip()]
            keep = [x for x in names_ if x in lp or x in tp]
            drop = [x for x in names_ if x not in keep]
            if not drop:
                continue
            lines = []
            if keep:
                lines.append(f"{pre}pub use {head}::{{{', '.join(keep)}}};" if len(keep) > 1 else f"{pre}pub use {head}::{keep[0]};")
            lines.append(f"{pre}pub(crate) use {head}::{{{', '.join(drop)}}};" if len(drop) > 1 else f"{pre}pub(crate) use {head}::{drop[0]};")
            t = t[:cur.start()] + "\n".join(lines) + t[cur.end():]
            narrowed.append(f"{f}:use {head}::{{{', '.join(drop)}}}")
        if t != rd(p):  # files it does not change stay untouched (others' uncommitted edits)
            wr(p, t)
    report[cdir] = (narrowed, test_only)
for c, (n, to) in report.items():
    print(f"{c}: {len(n)} back to pub(crate): {' '.join(n)}")
    if to:
        print(f"{c}: left pub, named only by other crates' tests: {' '.join(to)}")
