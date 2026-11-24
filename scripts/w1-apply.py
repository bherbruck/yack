# M3 W1' (D26): springrep's foundation modules into their crates, as pure moves.
#   numeric.rs -> new crate yack-numeric, except the Gauss rules (a static OnceLock cache, fatal in a
#     kernel crate) and what calls them (gauss_fixed, integrate, adapt), which stay in springrep's
#     numeric.rs until yack-numeric's hardening (lead's ruling);
#   tol.rs -> yack-tol gains param() and between() (its constants are springrep's, equal);
#   error.rs -> yack_base::kernel, verbatim, temporary (D26).
# springrep re-exports each under its old name, so no caller changes. Paths and visibility only.
# Usage: python3 w1-apply.py <yack-springrep dir> [--with-external ...]
import os, re, sys

root = os.path.abspath(sys.argv[1])
src = os.path.join(root, "src")
crates = os.path.dirname(root)
ws = os.path.dirname(crates)


def rd(p):
    return open(p).read()


def wr(p, s):
    os.makedirs(os.path.dirname(p), exist_ok=True)
    open(p, "w").write(s)


def sub1(s, a, b, f):
    if s.count(a) != 1:
        raise SystemExit(f"{f}: expected one {a[:80]!r}, found {s.count(a)}")
    return s.replace(a, b)


def edit(p, pairs):
    s = rd(p)
    for a, b in pairs:
        s = sub1(s, a, b, p)
    wr(p, s)


def crate_vis(s):
    return re.sub(r"\bpub\(crate\)", "pub", s)


HEAD = re.compile(r"^(?:#\[|///|//!|pub(?:\([^)]*\))?\s+|fn |const |static |struct |enum |type |impl|mod |use )")


def items(lines, indent=""):
    """Top-level items (at `indent`) as (name, first, end) spans, doc comments and attributes included."""
    n = len(indent)
    starts = [k for k, l in enumerate(lines)
              if l.startswith(indent) and not l[n:n + 1].isspace() and l[n:].strip()
              and re.match(r"(?:pub(?:\([^)]*\))?\s+)?(?:fn|const|static|struct|enum|type|impl|mod|use)\b", l[n:])]
    out = []
    for i, k in enumerate(starts):
        m = re.match(r"(?:pub(?:\([^)]*\))?\s+)?(?:fn|const|static|struct|enum|type|impl|mod|use)\s+([A-Za-z_]\w*)", lines[k][n:])
        name = m.group(1) if m else lines[k][n:].split()[0]
        j = k
        while j > 0 and lines[j - 1].startswith(indent) and re.match(r"(///|#\[)", lines[j - 1][n:]):
            j -= 1
        e = starts[i + 1] if i + 1 < len(starts) else len(lines)
        while e > k and (lines[e - 1].strip() == "" or re.match(r"(///|#\[)", lines[e - 1][n:]) and lines[e - 1].startswith(indent)):
            e -= 1
        out.append((name, j, e))
    return out


# ---------------- numeric ----------------
STAY = {"legendre_rule", "gauss", "gauss_fixed", "integrate", "adapt"}
STAY_TESTS = {"gauss_rules_integrate_polynomials_exactly", "adaptive_quadrature_handles_a_kink"}
num = rd(os.path.join(src, "numeric.rs")).split("\n")
t0 = next(k for k, l in enumerate(num) if l == "#[cfg(test)]")
body, tests = num[:t0], num[t0:]
# the header and module doc: everything before the first item
first = items(body)[0][1]
pre = body[:first]
keep_s, move_s = [], []
for name, j, e in items(body):
    if name in ("OnceLock", "crate") or body[j].startswith("use "):
        continue
    (keep_s if name in STAY else move_s).append("\n".join(body[j:e]))
found = {n for n, _, _ in items(body)}
if not STAY <= found:
    raise SystemExit(f"numeric.rs: missing {STAY - found}")
uses = [l for l in body if l.startswith("use ")]
if uses != ["use std::sync::OnceLock;", "use crate::{Error, Result};"]:
    raise SystemExit(f"numeric.rs: unexpected imports {uses}")
# tests: `mod tests {` ... `}` with `    use super::*;`
ti = tests.index("mod tests {")
inner = tests[ti + 1:len(tests) - 1 - (1 if tests[-1] == "" else 0)]
if tests[-1] == "":
    tests_end = tests[-2]
else:
    tests_end = tests[-1]
if tests_end != "}":
    raise SystemExit("numeric.rs: tests module does not end the file")
tk, tm = [], []
for name, j, e in items(inner, "    "):
    if name == "super":
        continue
    (tk if name in STAY_TESTS else tm).append("\n".join(inner[j:e]))
if len(tk) != len(STAY_TESTS):
    raise SystemExit(f"numeric.rs: found {len(tk)} of the staying tests")
moved = "\n\n".join(move_s)
if re.search(r"(?<![\w.])(gauss|gauss_fixed|integrate|adapt|legendre_rule)\s*\(", moved):
    raise SystemExit("numeric: moved code calls the Gauss rules that stay")
used_err = [n for n in ["Error", "Result"] if re.search(r"\b" + n + r"\b", moved)]
lib = ("// yack: Linear solves, root finding and Kronrod rules shared by the geometry and measurement code.\n"
       "// Adapted from springrep@a03e8a1 src/numeric.rs (MIT, (c) 2026 Springcraft, Inc.).\n"
       "// Changes: moved from springrep's numeric.rs (M3 W1', D26); the Gauss-Legendre rules and their\n"
       "// quadrature stay in springrep until hardening; paths and visibility only.\n"
       "//! Linear solves, root finding and Kronrod rules shared by the geometry and measurement code.\n\n"
       f"use yack_base::kernel::{{{', '.join(used_err)}}};\n\n"
       + crate_vis(moved) + "\n\n#[cfg(test)]\nmod tests {\n    use super::*;\n\n" + "\n\n".join(tm) + "\n}\n")
wr(os.path.join(crates, "yack-numeric", "src", "lib.rs"), lib)
wr(os.path.join(crates, "yack-numeric", "Cargo.toml"),
   '[package]\nname = "yack-numeric"\ndescription = "yack internal crate (not published; use the `yack` crate)"\n'
   'version.workspace = true\nedition.workspace = true\nrust-version.workspace = true\nlicense.workspace = true\n'
   'repository.workspace = true\npublish = false\n\n[dependencies]\nyack-base = { path = "../yack-base" }\n\n'
   '[lints]\nworkspace = true\n')
# what springrep's callers name through crate::numeric
called = set()
for dp, _, fs in os.walk(src):
    for f in fs:
        if f.endswith(".rs") and os.path.join(dp, f) != os.path.join(src, "numeric.rs"):
            t = rd(os.path.join(dp, f))
            for m in re.finditer(r"numeric::(\{[^}]*\}|\w+)", t):
                called |= {x.strip() for x in m.group(1).strip("{}").split(",") if x.strip()}
reexp = sorted((called - STAY), key=lambda x: (not x.isupper(), x))
rest_pre = [l for l in pre]
rest_pre = "\n".join(rest_pre)
rest_pre = re.sub(r"// Changes: none \(verbatim\); this header added\.",
                  "// Changes: all but the Gauss-Legendre rules moved to yack-numeric (M3 W1', D26), re-exported here.",
                  rest_pre, count=1)
kept = ("\n".join(["use std::sync::OnceLock;", "", "pub(crate) use yack_numeric::{",
                   "    " + ", ".join(reexp) + ",", "};"]) + "\n\n" + "\n\n".join(keep_s))
staying = rest_pre.rstrip("\n") + "\n\n" + kept + "\n\n#[cfg(test)]\nmod tests {\n    use super::*;\n\n" + "\n\n".join(tk) + "\n}\n"
wr(os.path.join(src, "numeric.rs"), staying)

# ---------------- tol ----------------
tolf = os.path.join(src, "tol.rs")
t = rd(tolf).split("\n")
fns = [(n, j, e) for n, j, e in items(t) if n in ("param", "between")]
if len(fns) != 2:
    raise SystemExit("tol.rs: param/between not found")
consts = {m.group(1): m.group(2) for m in re.finditer(r"pub const (\w+): f64 = ([^;]+);", rd(tolf))}
yt = os.path.join(crates, "yack-tol", "src", "lib.rs")
ys = rd(yt)
ycon = {m.group(1): m.group(2) for m in re.finditer(r"pub const (\w+): f64 = ([^;]+);", ys)}
for k, v in consts.items():
    if ycon.get(k) != v:
        raise SystemExit(f"tol: {k} is {v} in springrep, {ycon.get(k)} in yack-tol")
for n, _, _ in [(x if x != "between" else "coincidence", j, e) for x, j, e in fns]:
    if re.search(r"(?m)^pub fn " + n + r"\b", ys):
        raise SystemExit(f"tol: yack-tol already has a `{n}`; springrep's must not shadow or replace it")
add = "\n\n".join("\n".join(t[j:e]) for _, j, e in fns)
# yack-tol's own `between` (the larger of two tolerances, M4.1a) keeps its name; springrep's, a
# distance, joins it as `coincidence` (lead's ruling), and its callers are renamed below
add = sub1(add, "pub fn between(a: f64, b: f64) -> f64 {", "pub fn coincidence(a: f64, b: f64) -> f64 {", "tol.rs")
# before yack-tol's tests, if it has any
if "\n#[cfg(test)]" in ys:
    ys = ys.replace("\n#[cfg(test)]", "\n// From springrep's tol.rs (M3 W1', D26).\n" + add + "\n\n#[cfg(test)]", 1)
else:
    ys = ys.rstrip("\n") + "\n\n// From springrep's tol.rs (M3 W1', D26).\n" + add + "\n"
wr(yt, ys)
os.remove(tolf)

renamed = 0
for dp, _, fl in os.walk(src):
    for f in fl:
        p = os.path.join(dp, f)
        if f.endswith(".rs"):
            t2 = rd(p)
            n2 = len(re.findall(r"\btol::between\(", t2))
            if n2:
                wr(p, re.sub(r"\btol::between\(", "tol::coincidence(", t2))
                renamed += n2
if renamed != 8:
    raise SystemExit(f"tol: renamed {renamed} springrep calls of tol::between, expected 8")

# ---------------- error ----------------
er = rd(os.path.join(src, "error.rs"))
er = sub1(er, "// yack: springrep module src/error.rs\n",
          "// yack: springrep's kernel error, kept beside D7's Error until each crate converts (temporary, D26).\n", "error.rs")
er = sub1(er, "// Changes: none (verbatim); this header added.",
          "// Changes: moved from springrep's error.rs to yack_base::kernel (M3 W1', D26); paths and visibility only.", "error.rs")
er = sub1(er, "than a solid that fails [`crate::check`].", "than a solid that fails springrep's checker.", "error.rs")
er = sub1(er, "use std::fmt;\n",
          "//! springrep's kernel error. Temporary (D26): each crate's M3 hardening moves onto [`crate::Error`]\n"
          "//! (D7), and this module goes when the last user has.\n\nuse std::fmt;\n", "error.rs")
wr(os.path.join(crates, "yack-base", "src", "kernel.rs"), crate_vis(er))
os.remove(os.path.join(src, "error.rs"))
yb = os.path.join(crates, "yack-base", "src", "lib.rs")
edit(yb, [("\nuse core::fmt;\n", "\n// springrep's own error, beside D7's until each crate converts (D26).\n#[allow(missing_docs)]\npub mod kernel;\n\nuse core::fmt;\n")])

# ---------------- springrep: re-exports and dependencies ----------------
edit(os.path.join(src, "lib.rs"), [
    ("mod error;\n", "use yack_base::kernel as error;\n"),
    ("pub mod tol;\n", "pub use yack_tol as tol;\n"),
])
edit(os.path.join(root, "Cargo.toml"), [
    ('[dependencies]\n', '[dependencies]\nyack-base = { path = "../yack-base" }\nyack-numeric = { path = "../yack-numeric" }\nyack-tol = { path = "../yack-tol" }\n'),
])
lt = os.path.join(ws, "layers.toml")
s = rd(lt)
m = re.search(r'(?ms)^\[crate\.yack-springrep\]\n.*?^deps = \[([^\]]*)\]', s)
if not m:
    raise SystemExit("layers.toml: no springrep deps")
have = [x.strip().strip('"') for x in m.group(1).split(",") if x.strip()]
want = have + [d for d in ["yack-base", "yack-numeric", "yack-tol"] if d not in have]
s = s[:m.start(1)] + ", ".join(f'"{d}"' for d in want) + s[m.end(1):]
wr(lt, s)
print("ok; numeric re-exports", reexp)
