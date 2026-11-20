#!/usr/bin/env python3
# M4.0b "drop what yack can't reach" (swarm/done/m4-rehost-plan.md): deletes the cadrs native
# modules at least 90 % unreachable from yack (edges_faces, face_blend, blend_corner, tails,
# split_area, cells; axes goes in M4.0a) and makes each of their few callers, every one an entry
# point yack never calls, refuse instead (Unsupported). Nothing yack reaches changes: digests must
# be identical. Usage: M40B_ROOT=<crate dir> python3 scripts/m40b.py
import os, re, sys

ROOT = os.environ.get("M40B_ROOT", "/home/brenn/github/yack/crates/yack-cadrs")
N = ROOT + "/src/native"
GONE = ["edges_faces", "face_blend", "blend_corner", "tails", "split_area", "cells"]
WHY = '"removed in M4.0b: not reached from yack"'


def rd(p):
    return open(p).read()


def wr(p, s):
    open(p, "w").write(s)


def body_end(t, brace):
    d = 0
    for j in range(brace, len(t)):
        if t[j] == "{":
            d += 1
        elif t[j] == "}":
            d -= 1
            if d == 0:
                return j
    sys.exit("unbalanced braces")


def enclosing_fn(t, at):
    """(start of `fn`, body `{`, body `}`) of the indent-4 fn holding offset `at`."""
    best = None
    for m in re.finditer(r"\n    (?:pub(?:\([^)]*\))? )?fn \w+", t):
        if m.start() > at:
            break
        b = t.find("{", m.end())
        # the body's brace: first `{` after the signature's closing paren at depth 0
        i, d = m.end(), 0
        while i < len(t):
            c = t[i]
            if c == "(":
                d += 1
            elif c == ")":
                d -= 1
            elif c == "{" and d == 0:
                break
            elif c == ";" and d == 0:
                i = -1
                break
            i += 1
        if i < 0:
            continue
        e = body_end(t, i)
        if i < at < e:
            best = (m.start(), i, e)
    return best


# 1. the modules
for m in GONE:
    p = f"{N}/{m}.rs"
    if not os.path.exists(p):
        sys.exit(f"{m}.rs not found")
    os.remove(p)
mod = rd(N + "/mod.rs")
for m in GONE:
    pat = r'#\[cfg\(feature = "testkit"\)\]\npub mod %s;\n#\[cfg\(not\(feature = "testkit"\)\)\]\n(#\[allow[^\n]*\]\n)?pub\(crate\) mod %s;\n' % (m, m)
    mod2 = re.sub(pat, "", mod)
    if mod2 == mod:
        sys.exit(f"mod.rs: {m}'s declaration not found")
    mod = mod2
wr(N + "/mod.rs", mod)

# 2. their callers
stubbed = []
for dp, _, fs in os.walk(ROOT + "/src"):
    for f in fs:
        if not f.endswith(".rs"):
            continue
        p = os.path.join(dp, f)
        t = rd(p)
        changed = False
        while True:
            m = re.search(r"super::(%s)::" % "|".join(GONE), t)
            if not m:
                break
            rel = os.path.relpath(p, ROOT)
            if rel.endswith("native/heal.rs") and m.group(1) == "tails":
                # HealOp::Tails's arm only: the other healing ops stay.
                a = t.rfind("\n", 0, m.start()) + 1
                b = t.find(";", m.end()) + 1
                indent = t[a:m.start()]
                t = t[:a] + f"{indent}return Err(KernelError::Unsupported({WHY}));" + t[b:]
                t = re.sub(r"HealOp::Tails \{\s*max_angle,\s*max_width,?\s*\}", "HealOp::Tails { .. }", t, count=1)
                stubbed.append(f"{rel}: HealOp::Tails")
                changed = True
                continue
            enc = enclosing_fn(t, m.start())
            if enc is None:
                sys.exit(f"{rel}: no fn round {m.group(0)}")
            s, o, e = enc
            name = re.match(r"\n    (?:pub(?:\([^)]*\))? )?fn (\w+)", t[s:]).group(1)
            ret = t[o - 80:o]
            t = t[: o] + "{\n        Err(KernelError::Unsupported(" + WHY + "))\n    }" + t[e + 1 :]
            t = t[: s + 1] + '    #[allow(unused_variables, reason = "M4.0b: refuses, its arguments unread")]\n' + t[s + 1 :]
            stubbed.append(f"{rel}: {name}")
            changed = True
        if changed:
            if "KernelError" not in t.split("\nfn ")[0] and not re.search(r"use [^;]*KernelError", t):
                t = re.sub(r"(\nuse [^\n]*;\n)", r"\1use crate::KernelError;\n", t, count=1)
            wr(p, t)
# 3. what only the removed code used: draw.rs's `sector` (split_by_number's), and cadrs's own
# test of Kernel::fillet_smooth (its smooth corners are blend_corner).
p = N + "/draw.rs"
t = rd(p)
m = re.search(r"\n    /// The solid between the half planes at angles `a0` and `a1`[^\n]*\n(    ///[^\n]*\n)*    fn sector\(", t)
if not m:
    sys.exit("draw.rs: sector not found")
o = t.index("{", t.index("-> Result<Shape>", m.end()))
t = t[: m.start()] + t[body_end(t, o) + 1 :]
wr(p, t)
p = ROOT + "/tests/conformance.rs"
t = rd(p)
m = re.search(r"\n(///[^\n]*\n)*pub fn fillet_smooth_corner\(", t)
if not m:
    sys.exit("conformance.rs: fillet_smooth_corner not found")
o = t.index("{", m.end())
t = t[: m.start()] + t[body_end(t, o) + 1 :]
if t.count("fillet_partial, fillet_smooth_corner, ") != 1:
    sys.exit("conformance.rs: the case list's fillet_smooth_corner not found")
t = t.replace("fillet_partial, fillet_smooth_corner, ", "fillet_partial, ")
wr(p, t)
# Kernel::cells's doc example runs it: the example kept as text, the doc saying it refuses.
p = ROOT + "/src/lib.rs"
t = rd(p)
i = t.index("/// The cells of an arrangement of solids (OCCT's `BopAlgo_CellsBuilder`)".replace("Bop", "BOP"))
j = t.index("    /// ```\n", t.index("    /// # Examples", i))
t = t[:j] + "    /// ```text\n" + t[j + len("    /// ```\n"):]
t = t[:i] + "/// yack (M4.0b): removed, not reached from yack; refuses (Unsupported). Was:\n    " + t[i:]
wr(p, t)
print("deleted:", ", ".join(m + ".rs" for m in GONE))
print("refusing now:", "; ".join(stubbed))
