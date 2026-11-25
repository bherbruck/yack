# yack: M3 W9d (lead's rulings): springrep's draw, overlap and probe become yack-view's three modules,
# under a fresh crate root that holds only their aliases and `pub mod` lines. springrep re-exports each
# by its name. W9a's machinery and guards otherwise.
# Usage: python3 w9d-apply.py <yack-springrep dir> [--with-external ...]
import os, re, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from m3_move import (allow_items, cargo, count, crate_names, deps_guard, live_text, path_guard,
                     precise_pub, rd, rs_files, set_row, springrep_deps, strip_module_paths, sub1,
                     test_reach_guard, unit_edges, wr, allow_panic_at)

root = os.path.abspath(sys.argv[1])
src = os.path.join(root, "src")
crates = os.path.dirname(root)
ws = os.path.dirname(crates)
HERE = os.path.dirname(os.path.abspath(__file__))
KRATE = "yack-view"
N = os.path.join(crates, KRATE, "src")
B = None
PROOF = "w-h1-37"
WAVE = "M3 W9d"

# ---- 1. the crate ----
MODS = ["draw", "overlap", "probe"]
FILES = {m + ".rs": m + ".rs" for m in MODS}

# Every `.expect`/`.unwrap` clippy found in live blend code (swarm/res/w9-sites-blend.txt): all on an
# Option but one, the Bernstein solve's Result. Each panics with the message it had.
SITES = {}
for line in rd(os.path.join(ws, "swarm", "res", "w9-sites-view.txt")).split("\n"):
    m = re.match(r"crates/yack-springrep/src/((?:draw|overlap|probe)\.rs):(\d+):\d+: warning: used `(expect|unwrap)\(\)` on an? `(Option|Result)` value", line)
    if m:
        SITES.setdefault(m.group(1), []).append((int(m.group(2)), m.group(3), m.group(4)))


def convert_panics(t, s_rel):
    # a `tests.rs` is a `#[cfg(test)] mod tests;` file: test code throughout, where expect stays
    if os.path.basename(s_rel) == "tests.rs":
        return t, 0
    sites = SITES.get(s_rel, [])
    lines = t.split("\n")
    live = live_text(t)
    found = len(re.findall(r"\.expect\(|\.unwrap\(\)", live))
    if found != len(sites):
        raise SystemExit(f"{s_rel}: {found} live expect/unwrap, clippy listed {len(sites)}")
    done = 0
    for ln, kind, ty in sorted(sites, reverse=True):
        # the call starts on line ln; its `.expect(`/`.unwrap()` is on that line or the next few
        for k in range(ln - 1, min(ln + 6, len(lines))):
            l = lines[k]
            if kind == "expect" and ".expect(" in l:
                msg = re.search(r'\.expect\(("(?:[^"\\]|\\.)*")\)', l)
                if not msg:
                    raise SystemExit(f"{s_rel}:{k + 1}: an expect whose message spans lines")
                new = (f".unwrap_or_else(|| panic!({msg.group(1)}))" if ty == "Option"
                       else f'.unwrap_or_else(|e| panic!("{{}}: {{e:?}}", {msg.group(1)}))')
                lines[k] = l.replace(msg.group(0), new, 1)
                allow_panic_at(lines, k)
                done += 1
                break
            if kind == "unwrap" and ".unwrap()" in l:
                new = ('.unwrap_or_else(|| panic!("called `Option::unwrap()` on a `None` value"))' if ty == "Option"
                       else '.unwrap_or_else(|e| panic!("called `Result::unwrap()` on an `Err` value: {e:?}"))')
                lines[k] = l.replace(".unwrap()", new, 1)
                allow_panic_at(lines, k)
                done += 1
                break
        else:
            raise SystemExit(f"{s_rel}:{ln}: no {kind} on or after the line clippy gave")
    t = "\n".join(lines)
    if re.search(r"\.expect\(|\.unwrap\(\)", live_text(t)):
        raise SystemExit(f"{s_rel}: live expect/unwrap left after the swaps")
    return t, done


DEPS = None  # printed by the dry run, then pinned
DEV = []
ALIASES = {
    "error": "use yack_base::kernel as error;", "Error": "use yack_base::kernel::Error;",
    "Result": "use yack_base::kernel::Result;", "geom": "use yack_geom as geom;",
    "math": "use yack_math::kernel as math;", "numeric": "use yack_numeric as numeric;",
    "tol": "use yack_tol as tol;", "topo": "use yack_topo as topo;", "intersect": "use yack_intersect as intersect;",
    "names": "use yack_topo::names;", "locate": "use yack_locate as locate;", "rays": "use yack_locate::rays;",
    "distance": "use yack_locate::distance;", "measure": "use yack_measure as measure;",
    "check": "use yack_check as check;", "profile": "use yack_profile as profile;", "mesh": "use yack_mesh as mesh;",
    "tweak": "use yack_tweak as tweak;", "sweep": "use yack_build::sweep;", "loft": "use yack_build::loft;",
    "primitive": "use yack_build::primitive;", "build": "use yack_build::build;",
    "boolean": "use yack_boolean as boolean;", "blend": "use yack_blend as blend;",
}


def header_note(t, s_rel):
    if re.search(r"// Changes: none \(verbatim\); this header added\.", t):
        return re.sub(r"// Changes: none \(verbatim\); this header added\.",
                      f"// Changes: moved from springrep's src/{s_rel} to {KRATE} ({WAVE}); paths and visibility only.", t, count=1)
    if "// Changes:" in t[:800]:
        t = re.sub(r"(// Changes: [^\n]*(?:\n// [^\n]*)*?)\.?\n(?!// )",
                   rf"\1; then to {KRATE} ({WAVE}), paths and visibility only.\n", t, count=1)
        return t.replace(f".; then to {KRATE} ({WAVE})", f"; then to {KRATE} ({WAVE})", 1)
    return t


def drop_root_aliases(text):
    def one(u):
        parts, depth, cur = [], 0, ""
        for c in u.group(1):
            depth += c == "{"
            depth -= c == "}"
            if c == "," and depth == 0:
                parts.append(cur)
                cur = ""
            else:
                cur += c
        parts.append(cur)
        keep = [p for p in parts if p.strip() and p.strip() not in ALIASES]
        return ("use crate::{" + ",".join(keep) + (",\n" if u.group(1).rstrip().endswith(",") else "") + "};") if keep else ""
    return re.sub(r"use crate::\{([^;]*)\};", one, text)


mv, swapped = {}, 0
for s_rel, d_rel in FILES.items():
    t = rd(os.path.join(src, s_rel))
    # the moved files keep their module paths: `crate::overlap::..` names yack-view's own module
    t = header_note(t, s_rel)
    t, n = convert_panics(t, s_rel)
    swapped += n
    mv[d_rel] = t
if swapped != sum(len(v) for v in SITES.values()):
    raise SystemExit(f"{KRATE}: swapped {swapped} of {sum(len(v) for v in SITES.values())} listed sites")
if re.search(r"\bstep::|\bedit::|\bfacade::|\bblend::", re.sub(r"//[^\n]*", "", "\n".join(live_text(v) for v in mv.values()))):
    raise SystemExit(f"{KRATE} still reaches a module above it")
own = set(MODS)
reached_all = crate_names(list(mv.values())) - own
reached = crate_names([live_text(v) for v in mv.values()]) - own
test_only = reached_all - reached
root_items = set()
head = ["// yack: a solid seen from outside: hidden-line drawings, overlaps and sweeps between bodies, and its",
        "// thin places (M3 W9d's crate root over springrep's draw, overlap and probe).",
        "//! A solid seen from outside: [`draw`]'s hidden-line drawings, [`overlap`]'s overlaps, meetings and",
        "//! swept contacts between bodies, and [`probe`]'s thin places.", "",
        "// springrep's view code indexes by position; D26's hardening pass moves it onto checked access,",
        "// as for yack-linalg until M5.", "#![allow(clippy::indexing_slicing)]", "",
        "// springrep's foundation modules (D26), under the names the moved code uses."]
# a name the root's own `use` lines import already (edit's `use crate::{error::{Error, Result}}`, W9b-1)
# gets no alias line: the submodules' `crate::Result` reaches that import
root_uses = ""  # the root is fresh: it imports nothing of its own
# only what a path names below a module (`error::{Error, Result}`, `x::Y`): a bare entry of
# `use crate::{tol, ..}` is an alias the head supplies
imported = set(re.findall(r"(?<!crate)(?<!self)(?<!super)::\s*([A-Za-z_]\w*)", root_uses))
for g in re.findall(r"::\s*\{([^{}]*)\}", root_uses):
    imported |= set(re.findall(r"[A-Za-z_]\w*", g))
head += [ALIASES[n] for n in sorted((reached & set(ALIASES)) - imported)]
head += ["#[cfg(test)]\n" + ALIASES[n] for n in sorted(test_only & set(ALIASES))]
head += [""] + [f"pub mod {m};" for m in MODS]
mv["lib.rs"] = "\n".join(head) + "\n"
unknown = reached_all - set(ALIASES) - own - root_items
if unknown:
    raise SystemExit(f"{KRATE} reaches crate-root names {sorted(unknown)} it does not have")
used_deps = sorted({re.search(r"use (yack_\w+)", ALIASES[n]).group(1).replace("_", "-") for n in reached & set(ALIASES)} | {"yack-base"})
if DEPS is None:
    test_deps = sorted({re.search(r"use (yack_\w+)", ALIASES[n]).group(1).replace("_", "-") for n in test_only & set(ALIASES)} - set(used_deps))
    print("reaches:", used_deps, "test-only:", test_deps)
    DEPS, DEV = used_deps, test_deps
elif sorted(set(used_deps) - set(DEPS)) or sorted(set(DEPS) - set(used_deps)):
    raise SystemExit(f"{KRATE}: code reaches {used_deps}, DEPS lists {DEPS}")
staged = {KRATE: mv}

REAL = "/home/brenn/github/yack/crates"
moving = {os.path.join(src, s) for s in FILES}
outside, outside_tests = [], []
for p in rs_files(src) + rs_files(os.path.join(root, "tests")):
    if p in moving:
        continue
    t = rd(p)
    outside_tests.append(t)
    if os.path.basename(p) != "tests.rs" and "/tests/" not in p and "/moved_tests/" not in p:
        outside.append(live_text(t))
for c in ["yack", "yack-bridge", "yack-step", "yack-cadrs"]:
    d = os.path.join(crates, c, "src") if os.path.isdir(os.path.join(crates, c, "src")) else os.path.join(REAL, c, "src")
    outside += [live_text(rd(p)) for p in rs_files(d) if os.path.basename(p) != "tests.rs"]
# The names made pub are printed by the dry run and pinned here once read (as W8 pinned boolean_with);
# names that match unrelated items outside go in KEEP_PRIVATE.
KEEP_PRIVATE = []
EXPECT_PUB = None  # pinned once the dry run has printed it
masked = "\n".join(outside)
for n in KEEP_PRIVATE:
    masked = re.sub(r"\b" + n + r"\b", n + "_w9_unrelated", masked)
texts, made_pub = precise_pub(staged, masked)
if EXPECT_PUB is not None and made_pub.get(KRATE, []) != EXPECT_PUB:
    raise SystemExit(f"{KRATE}: pub(crate) -> pub for {made_pub.get(KRATE)}, expected {EXPECT_PUB}")
hits = test_reach_guard(texts, made_pub, outside_tests, {KRATE: MODS})
if hits:
    raise SystemExit(f"springrep tests reach crate-private items of {KRATE}: {hits}")
unresolved = path_guard(texts, outside_tests, {KRATE: MODS})
if unresolved:
    raise SystemExit(f"springrep tests name paths {KRATE} does not have: {unresolved}")
lt0 = os.path.join(ws, "layers.toml")
s0 = rd(lt0)
m0 = re.search(r'(?ms)^\[crate\.' + KRATE + r'\]\n.*?^deps = \[([^\]]*)\]', s0)
have0 = [x.strip().strip('"') for x in m0.group(1).split(",") if x.strip()]
add0 = [d for d in DEV if d not in have0]  # dev-only (gate 1 checks them too)
if add0:
    s0 = s0[:m0.start(1)] + ", ".join(f'"{d}"' for d in have0 + add0) + s0[m0.end(1):]
    wr(lt0, s0)
for d_rel, t in texts[KRATE].items():
    if re.search(r"\bHash(Map|Set)\b", live_text(t)):
        t = allow_items(t, r"\bHash(Map|Set)\b", "clippy::disallowed_types",
                        "D26: BTree* in hardening, proven by digests", d_rel)
    wr(os.path.join(N, d_rel), t)
deps_guard(ws, KRATE, DEPS + DEV)
c = cargo(KRATE, DEPS)
c = c.replace("\n[lints]", "\n[dev-dependencies]\n" + "".join(f'{d} = {{ path = "../{d}" }}\n' for d in DEV) + "\n[lints]")
wr(os.path.join(crates, KRATE, "Cargo.toml"), c)
e, pnc = count([N]), count(["--panics", N])
if pnc != swapped:
    raise SystemExit(f"{KRATE}: {pnc} contract panics, expected {swapped}")
size = sum(len(rd(p).split("\n")) - 1 for p in rs_files(N))
for s_rel in FILES:
    os.remove(os.path.join(src, s_rel))

# ---- 2. springrep: re-export and dependency ----
lp = os.path.join(src, "lib.rs")
tl = rd(lp)
for m_ in MODS:
    tl = sub1(tl, f"pub mod {m_};\n", f"pub use yack_view::{m_};\n", lp)
wr(lp, tl)
springrep_deps(ws, root, [KRATE])

# springrep's private crate-root aliases the moved code was the last to use (w-h4's W8 lesson): an alias
# only tests still name becomes #[cfg(test)], and one nothing names goes, so springrep's clippy count
# cannot rise from an unused import
lt_ = rd(lp)
rest_live = "\n".join(live_text(rd(p_)) for p_ in rs_files(src) if p_ != lp and os.path.basename(p_) != "tests.rs"
                      and "/moved_tests/" not in p_)
rest_all = "\n".join(rd(p_) for p_ in rs_files(src) + rs_files(os.path.join(root, "tests")) if p_ != lp)
named = lambda n, t: re.search(r"\bcrate::" + n + r"\b|use crate::\{[^;]*(?<![\w:])" + n + r"\b", t)
for m_ in list(re.finditer(r"(?m)^(?:pub\(crate\) )?use [\w:]+ as (\w+);\n", lt_)):
    n_ = m_.group(1)
    # at the crate root itself a bare `alias::` path names it too (`pub use error::{Error, Result};`)
    root_rest = live_text(lt_.replace(m_.group(0), ""))
    if named(n_, rest_live) or named(n_, root_rest) or re.search(r"(?<![\w:])" + n_ + r"::", root_rest):
        continue
    if named(n_, rest_all):
        if f"#[cfg(test)]\n{m_.group(0)}" not in lt_:
            lt_ = lt_.replace(m_.group(0), "#[cfg(test)]\n" + m_.group(0), 1)
            print("springrep alias test-only:", n_)
    else:
        lt_ = lt_.replace(m_.group(0), "", 1)
        print("springrep alias dropped:", n_)
wr(lp, lt_)

# ---- 3. layers.toml map, scores ----
lt = os.path.join(ws, "layers.toml")
s_ = rd(lt)
if size > 5000 and f"[modules.{KRATE}]" not in s_:
    mp_ = unit_edges(N)
    block = f"[modules.{KRATE}]\n" + "".join(f"{u} = [{', '.join(chr(34) + v + chr(34) for v in vs)}]\n" for u, vs in mp_) + "\n"
    s_ = sub1(s_, "[modules.yack-springrep]\n", block + "[modules.yack-springrep]\n", lt)
wr(lt, s_)
sd = os.path.join(ws, "scores")
set_row(os.path.join(sd, "eps.txt"), KRATE, e)
if pnc:
    set_row(os.path.join(sd, "lints.txt"), KRATE, pnc)
fp = os.path.join(sd, "filesize.txt")
# gate 6: the moved files' rows are their line counts after the apply (its allows add lines), and a file
# under the limit has no row; springrep's rows for the moved files go
rows = [r for r in rd(fp).rstrip("\n").split("\n") if not any(r.startswith(f"crates/yack-springrep/src/{s_rel} ") for s_rel in FILES)]
rows = [r for r in rows if not r.startswith(f"crates/{KRATE}/src/")]
for p_ in rs_files(N):
    n_ = len(rd(p_).split("\n")) - 1
    if n_ > 1500:
        rows.append(f"crates/{KRATE}/src/{os.path.relpath(p_, N)} {n_}")
head_ = [r for r in rows if r.startswith("#")]
wr(fp, "\n".join(head_ + sorted(r for r in rows if r.strip() and not r.startswith("#"))) + "\n")
print("files:", sorted(FILES.values()))
print("made pub:", made_pub.get(KRATE))
print("rows:", (KRATE, e, pnc, size))
