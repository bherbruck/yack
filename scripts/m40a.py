#!/usr/bin/env python3
# M4.0a "foundations down" (plan/m4-cadrs-map.md): pure moves inside crates/yack-cadrs.
#   B  boolean::curve_bbox (+ its piece-box cache) -> curve;  boolean::VTOL -> tol
#   D  fingerprint::Key128's hashing core -> new module key (its geometry digests stay)
#   -  axes::ax2_x -> math (axes, now empty, is removed)
# Behaviour is unchanged: items move, paths are rewritten, nothing else. Freezes import.py.
import os, re, sys

ROOT = os.environ.get("M40A_ROOT", "/home/brenn/github/yack/crates/yack-cadrs")
N = ROOT + "/src/native"


def rd(p):
    return open(p).read()


def wr(p, s):
    open(p, "w").write(s)


def item_span(text, start_pat):
    """(start, end) of the item whose first line matches start_pat, with its doc comments and
    attributes above it and its braced body (or `;`) below."""
    m = re.search(start_pat, text, re.M)
    if not m:
        sys.exit(f"not found: {start_pat}")
    s = text.rfind("\n", 0, m.start()) + 1
    # Doc comments / attributes above.
    while True:
        prev_end = s - 1
        if prev_end <= 0:
            break
        prev_start = text.rfind("\n", 0, prev_end) + 1
        line = text[prev_start:prev_end].strip()
        if line.startswith("///") or line.startswith("#["):
            s = prev_start
        else:
            break
    # Body.
    i = m.start()
    semi = text.find(";", i)
    brace = text.find("{", i)
    if brace == -1 or (semi != -1 and semi < brace):
        e = semi + 1
    else:
        depth = 0
        j = brace
        while j < len(text):
            c = text[j]
            if c == "{":
                depth += 1
            elif c == "}":
                depth -= 1
                if depth == 0:
                    break
            j += 1
        e = j + 1
    if text[e : e + 1] == "\n":
        e += 1
    return s, e


def cut(text, pat):
    s, e = item_span(text, pat)
    return text[:s] + text[e:].lstrip("\n") if False else text[:s] + text[e:], text[s:e]


def sub_all(old, new, files=None):
    n = 0
    for dp, _, fs in os.walk(ROOT + "/src"):
        for f in fs:
            if not f.endswith(".rs"):
                continue
            p = os.path.join(dp, f)
            s = rd(p)
            t = s.replace(old, new)
            if t != s:
                n += s.count(old)
                wr(p, t)
    return n


def insert_after_uses(text, line):
    """`line` after the last top-level `use` of the file."""
    last = None
    for m in re.finditer(r"^use [^\n]*;\n", text, re.M):
        last = m
    if last is None:
        sys.exit("no use line")
    return text[: last.end()] + line + text[last.end() :]


changes = []


def note(rel, what):
    """Appends `what` to the provenance header's Changes of src/native/<rel>."""
    p = N + "/" + rel
    s = rd(p)
    lines = s.split("\n")
    i = next(k for k, l in enumerate(lines) if l.startswith("// Changes:"))
    j = i
    while j + 1 < len(lines) and lines[j + 1].startswith("//   "):
        j += 1
    last = lines[j].rstrip()
    if last.endswith("none (verbatim)."):
        last = last[: -len("none (verbatim).")] + "M4.0a: " + what + "."
    else:
        last = last.rstrip(".") + "; M4.0a: " + what + "."
    # Wrap at 100 columns.
    out, cur = [], ""
    for w in last.split(" "):
        if cur and len(cur) + 1 + len(w) > 100:
            out.append(cur)
            cur = "//   " + w
        else:
            cur = (cur + " " + w) if cur else w
    out.append(cur)
    lines[j : j + 1] = out
    wr(p, "\n".join(lines))

# ---- B: curve_bbox and its cache -> curve.rs ----
b = rd(N + "/boolean.rs")
moved = []
for pat in [
    r"^const PIECE_BOXES_KEPT: usize",
    r"^type PieceBoxes =",
    r"^thread_local! \{\n    /// The boxes of B-spline curves' pieces",
    r"^pub fn curve_bbox\(",
]:
    if pat.startswith("^thread_local"):
        m = re.search(pat, b, re.M)
        if not m:
            sys.exit("thread_local not found")
        s, e = item_span(b, pat)
        moved.append(b[s:e])
        b = b[:s] + b[e:]
    else:
        b, piece = cut(b, pat)
        moved.append(piece)
b = insert_after_uses(b, "use super::curve::curve_bbox;\n")
wr(N + "/boolean.rs", b)
c = rd(N + "/curve.rs")
block = "\n" + "\n".join(x.rstrip("\n") + "\n" for x in moved)
# The moved code names Bbox (math), tol, nurbs: curve.rs has `use super::math::*`, `use
# super::tol` and `use super::nurbs::NurbsCurve`; `super::nurbs::` paths stay valid.
i = c.find("\n#[cfg(test)]")
c = (c[:i] + block + c[i:]) if i != -1 else (c.rstrip("\n") + "\n" + block)
wr(N + "/curve.rs", c)
changes.append(("curve.rs", "curve_bbox and its piece-box cache moved here from boolean.rs"))
n = sub_all("super::boolean::curve_bbox", "super::curve::curve_bbox")
n += sub_all("crate::native::boolean::curve_bbox", "crate::native::curve::curve_bbox")
changes.append(("*", f"{n} paths boolean::curve_bbox -> curve::curve_bbox"))

# ---- B: VTOL -> tol.rs ----
b = rd(N + "/boolean.rs")
b, vt = cut(b, r"^pub const VTOL: f64 = super::tol::POINT;")
b = insert_after_uses(b, "pub use super::tol::VTOL;\n")
wr(N + "/boolean.rs", b)
t = rd(N + "/tol.rs")
vt = vt.replace("super::tol::POINT", "POINT").replace("[`tol::POINT`]", "[`POINT`]")
m = re.search(r"^pub const POINT: f64[^\n]*\n", t, re.M)
t = t[: m.end()] + "\n" + vt + t[m.end() :]
wr(N + "/tol.rs", t)
n = sub_all("super::boolean::VTOL", "super::tol::VTOL")
changes.append(("tol.rs", f"VTOL moved here from boolean.rs (boolean re-exports it); {n} paths rewritten"))

# ---- D: Key128's core -> key.rs ----
f = rd(N + "/fingerprint.rs")
core = []
for pat in [r"^pub struct Key128 \{", r"^fn mix_a\(", r"^fn mix_b\(", r"^impl Default for Key128 \{", r"^impl Hasher for Key128 \{"]:
    f, piece = cut(f, pat)
    core.append(piece)
# impl Key128 { new, word, key, f64s } -> key.rs; { frame, curve, surface } stay.
s, e = item_span(f, r"^impl Key128 \{")
imp = f[s:e]
body_start = imp.index("{") + 1
body = imp[body_start : imp.rindex("}")]
methods = re.split(r"(?m)^(?=    (?:///|#\[|pub fn |fn ))", body)
keep_core, keep_geom = [], []
pending = ""
for chunk in methods:
    if not chunk.strip():
        pending += chunk
        continue
    if re.match(r"\s*(///|#\[)", chunk) and not re.search(r"^    (pub )?fn ", chunk, re.M):
        pending += chunk
        continue
    name = re.search(r"fn (\w+)", chunk).group(1)
    (keep_core if name in ("new", "word", "key", "f64s") else keep_geom).append(pending + chunk)
    pending = ""
core_impl = "impl Key128 {" + "".join(keep_core).replace("    fn word(", "    pub(crate) fn word(") + "}\n"
geom_impl = "impl Key128 {" + "".join(keep_geom) + "}\n"
f = f[:s] + geom_impl + f[e:]
f = insert_after_uses(f, "use super::key::Key128;\n")
wr(N + "/fingerprint.rs", f)
key = (
    "// yack: a 128-bit hashing key (two 64-bit streams), moved out of fingerprint.rs (M4.0a).\n"
    "// Adapted from cadrs@eb5899ef crates/cadrs_kernel/src/native/fingerprint.rs (MIT OR Apache-2.0, (c) 2026 Rouan van der Ende).\n"
    "// Changes: the key's hashing core only (its geometry digests stay in fingerprint.rs); `word`\n"
    "//   made pub(crate) for them.\n"
    "//! A 128-bit key: everything written goes to both of its streams.\n\n"
    "use std::hash::Hasher;\n\n"
    + "\n".join(x.rstrip("\n") + "\n" for x in core[:3])
    + "\n"
    + core_impl
    + "\n"
    + "\n".join(x.rstrip("\n") + "\n" for x in core[3:])
)
wr(N + "/key.rs", key)
n = sub_all("super::fingerprint::Key128", "super::key::Key128")
n += sub_all("crate::native::fingerprint::Key128", "crate::native::key::Key128")
changes.append(("key.rs", f"new: Key128's core from fingerprint.rs; {n} paths rewritten"))
m = rd(N + "/mod.rs")
decl = (
    "#[cfg(feature = \"testkit\")]\npub mod fingerprint;\n"
)
if decl not in m:
    sys.exit("mod.rs: fingerprint declaration not found")
m = m.replace(
    decl,
    "#[cfg(feature = \"testkit\")]\npub mod key;\n#[cfg(not(feature = \"testkit\"))]\n"
    "#[allow(dead_code, reason = \"items only the test kit uses\")]\npub(crate) mod key;\n" + decl,
)
wr(N + "/mod.rs", m)

# ---- ax2_x -> math.rs; axes removed ----
a = rd(N + "/axes.rs")
a, ax = cut(a, r"^pub fn ax2_x\(")
if re.search(r"^(pub )?(fn|struct|enum|const|type) ", a, re.M):
    sys.exit("axes.rs holds more than ax2_x")
mt = rd(N + "/math.rs")
i = mt.find("\n#[cfg(test)]")
mt = (mt[:i] + "\n" + ax + mt[i:]) if i != -1 else (mt.rstrip("\n") + "\n\n" + ax)
wr(N + "/math.rs", mt)
os.remove(N + "/axes.rs")
m = rd(N + "/mod.rs")
old = (
    "#[cfg(feature = \"testkit\")]\npub mod axes;\n#[cfg(not(feature = \"testkit\"))]\n"
    "#[allow(dead_code, reason = \"items only the test kit uses\")]\npub(crate) mod axes;\n"
)
if old not in m:
    sys.exit("mod.rs: axes declaration not found")
wr(N + "/mod.rs", m.replace(old, ""))
n = sub_all("super::axes::ax2_x", "super::math::ax2_x")
n += sub_all("crate::native::axes::ax2_x", "crate::native::math::ax2_x")
changes.append(("math.rs", f"ax2_x moved here from axes.rs (removed); {n} paths rewritten"))

# ---- freeze import.py ----
p = ROOT + "/import.py"
s = rd(p)
if "FROZEN" not in s:
    s = s.replace(
        "def main():\n",
        "def main():\n    # FROZEN at M4.0a: src/ has been refactored in place since; re-importing would undo it.\n"
        "    sys.exit(\"import.py is frozen since M4.0a (plan/m4-cadrs-map.md): see IMPORT.md\")\n",
        1,
    )
    wr(p, s)
note("curve.rs", "curve_bbox and its piece-box cache moved here from boolean.rs")
note("boolean.rs", "curve_bbox moved to curve.rs, VTOL to tol.rs (re-exported)")
note("tol.rs", "VTOL moved here from boolean.rs")
note("fingerprint.rs", "Key128's hashing core moved to key.rs")
note("math.rs", "ax2_x moved here from axes.rs (removed)")
note("mod.rs", "module key added, axes removed")
for f, what in changes:
    print(f"{f}: {what}")
