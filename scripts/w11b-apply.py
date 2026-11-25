# yack: M3 W11 (b): yack-bridge off yack_springrep. Every `yack_springrep::<head>` path becomes the crate path
# springrep re-exported under that head (its lib.rs `pub use` lines, read here, are the map; facade::edit and
# facade::exchange are W10's yack_edit::glue and yack_step::exchange), and its Cargo.toml trades yack-springrep
# for the crates the rewritten code names. Paths only; w-m11 reviews the dry-run diff.
# Usage: python3 w11b-apply.py <yack-springrep dir> [--with-external ...] [--dry]
import os, re, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(1, "/home/brenn/github/yack/scripts")
from m3_move import rd, rs_files, wr

root = os.path.abspath(sys.argv[1])
crates = os.path.dirname(root)
B = os.path.join(crates, "yack-bridge")
DRY = "--dry" in sys.argv

# ---- the map, from springrep's own re-exports ----
lib = rd(os.path.join(root, "src", "lib.rs"))
MAP = {}
for path, alias in re.findall(r"(?m)^(?:pub )?use ([\w:]+?)(?: as (\w+))?;", lib):
    MAP[alias or path.rsplit("::", 1)[-1]] = path
err = MAP.pop("error")  # `use yack_base::kernel as error;` then `pub use error::{Error, Result};`
MAP.update({"Error": err + "::Error", "Result": err + "::Result",
            "facade::edit": "yack_edit::glue", "facade::exchange": "yack_step::exchange"})
fm = rd(os.path.join(root, "src", "facade", "mod.rs"))
if "pub use yack_edit::glue as edit;" not in fm or "pub use yack_step::exchange;" not in fm:
    raise SystemExit("springrep's facade/mod.rs is not W10's: run after W10")
HEAD = r"facade::(?:edit|exchange)\b|\w+"


def target(head, f):
    if head not in MAP:
        raise SystemExit(f"{f}: yack_springrep::{head} has no new home")
    return MAP[head]


def entries(g):
    out, depth, cur = [], 0, ""
    for ch in g + ",":
        depth += ch == "{"
        depth -= ch == "}"
        if ch == "," and depth == 0:
            if cur.strip():
                e = " ".join(cur.split())
                out.append(re.sub(r",?\s*\}", "}", re.sub(r"\{\s*", "{", e)))
            cur = ""
        else:
            cur += ch
    return out


def one_use(e, f):
    m = re.match(r"(" + HEAD + r")(?:(::.*)|\s+as\s+(\w+))?$", e, re.S)
    if not m:
        raise SystemExit(f"{f}: cannot read use entry {e!r}")
    head, rest, alias = m.group(1), m.group(2), m.group(3)
    t = target(head, f)
    if rest:
        return f"use {t}{rest};"
    name = alias or head.rsplit("::", 1)[-1]
    return f"use {t};" if t.rsplit("::", 1)[-1] == name else f"use {t} as {name};"


def rewrite(t, f):
    # grouped uses first: `use yack_springrep::{a, b::{..}};` becomes one `use` per entry, same indent
    def group(m):
        return "\n".join(m.group(1) + one_use(e, f) for e in entries(m.group(2)))
    t = re.sub(r"(?m)^([ \t]*)use yack_springrep::\{(.*?)\};", group, t, flags=re.S)
    # everything else: a path's head (in code and comments alike)
    return re.sub(r"\byack_springrep::(" + HEAD + r")", lambda m: target(m.group(1), f), t)


changed = {}
for p in rs_files(os.path.join(B, "src")) + rs_files(os.path.join(B, "tests")):
    t = rd(p)
    if "yack_springrep" in t:
        n = rewrite(t, os.path.relpath(p, B))
        if "yack_springrep" in n:
            raise SystemExit(f"{p}: yack_springrep left after the rewrite")
        changed[p] = n

# ---- Cargo.toml: the crates the code now names ----
CRATES = set(os.listdir(crates)) | set(os.listdir("/home/brenn/github/yack/crates"))  # a dry copy holds only some crates


def named(texts):
    """The workspace crates the code (not its comments) names; `yack_glue` is a yack-cadrs module, not a crate."""
    out = {re.sub("_", "-", c) for t in texts for c in re.findall(r"(?<![\w:])(yack_\w+?)(?:::|\b(?=\s+as\b)|;)", re.sub(r"//[^\n]*", "", t))}
    return out & CRATES


src_t = [changed.get(p, rd(p)) for p in rs_files(os.path.join(B, "src"))]
test_t = [changed.get(p, rd(p)) for p in rs_files(os.path.join(B, "tests"))]
deps = sorted(named(src_t) - {"yack-bridge"})
dev = sorted(named(test_t) - set(deps) - {"yack-bridge"})
cp = os.path.join(B, "Cargo.toml")
c = rd(cp)
m = re.search(r"(?ms)^\[dependencies\]\n(.*?)\n(?=\[|\Z)", c)
have = re.findall(r"(?m)^(yack-[\w-]+) = ", m.group(1))
keep = [l for l in m.group(1).split("\n") if l.strip() and not l.startswith("yack-springrep ")]
for d in deps:
    if d not in have:
        keep.append(f'{d} = {{ path = "../{d}" }}')
c = c[:m.start(1)] + "\n".join(keep) + "\n" + c[m.end(1):]
if dev:
    block = "[dev-dependencies]\n" + "".join(f'{d} = {{ path = "../{d}" }}\n' for d in dev) + "\n"
    if "[dev-dependencies]" in c:
        raise SystemExit("yack-bridge already has [dev-dependencies]: merge by hand")
    c = c.replace("\n[lints]", "\n" + block + "[lints]", 1)
if "fitted-retry = []" not in c:
    raise SystemExit("yack-bridge's fitted-retry feature was lost")
have_all = sorted(set(re.findall(r"(?m)^(yack-[\w-]+) = ", c)))
print("deps:", sorted(set(re.findall(r"(?m)^(yack-[\w-]+) = ", c[:c.find("[dev-dependencies]")] if dev else c))))
print("dev-deps:", dev)
LEAD = {"yack-cadrs", "yack-base", "yack-check", "yack-geom", "yack-locate", "yack-math", "yack-measure", "yack-mesh",
        "yack-tol", "yack-topo", "yack-blend", "yack-boolean", "yack-build", "yack-edit", "yack-profile", "yack-step",
        "yack-view"}
if set(have_all) != LEAD:
    print(f"NOTE: Cargo deps differ from the layers row: extra {sorted(set(have_all) - LEAD)}, missing {sorted(LEAD - set(have_all))}")

if DRY:
    import difflib
    for p, n in sorted(changed.items()):
        sys.stdout.writelines(difflib.unified_diff(rd(p).splitlines(True), n.splitlines(True),
                                                   "a/" + os.path.relpath(p, crates), "b/" + os.path.relpath(p, crates)))
    sys.stdout.writelines(difflib.unified_diff(rd(cp).splitlines(True), c.splitlines(True), "a/yack-bridge/Cargo.toml", "b/yack-bridge/Cargo.toml"))
    sys.exit(0)
for p, n in changed.items():
    wr(p, n)
wr(cp, c)
print("rewrote:", len(changed), "files")
