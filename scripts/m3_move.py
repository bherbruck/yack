# M3 extraction waves: the shared steps of a pure move of springrep modules into their crates (D26).
# Each wave's apply script (w4-apply.py, ...) describes its modules and calls these. Every step checks
# what it expects and stops rather than guess.
import os, re, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))


def rd(p):
    return open(p).read()


def wr(p, s):
    os.makedirs(os.path.dirname(p), exist_ok=True)
    open(p, "w").write(s)


def sub1(s, a, b, f):
    if s.count(a) != 1:
        raise SystemExit(f"{f}: expected one {a[:80]!r}, found {s.count(a)}")
    return s.replace(a, b)


def count(args):
    return int(subprocess.check_output([sys.executable, os.path.join(HERE, "eps_count.py")] + args).decode())


def rs_files(d):
    return sorted(os.path.join(a, f) for a, _, fl in os.walk(d) for f in fl if f.endswith(".rs"))


ITEM = re.compile(r"(?:pub(?:\([^)]*\))?\s+)?(?:fn|const|static|struct|enum|type|impl|mod|use|trait)\b")


def items(lines, indent=""):
    """Items at `indent` as (name, first, end) spans, their doc comments and attributes included."""
    n = len(indent)
    starts = [k for k, l in enumerate(lines)
              if l.startswith(indent) and l[n:n + 1] not in (" ", "\t", "") and ITEM.match(l[n:])]
    out = []
    for i, k in enumerate(starts):
        m = re.match(r"(?:pub(?:\([^)]*\))?\s+)?(?:fn|const|static|struct|enum|type|impl|mod|use|trait)\s+([A-Za-z_]\w*)", lines[k][n:])
        name = m.group(1) if m else lines[k][n:].split()[0]
        j = k
        while j > 0 and lines[j - 1].startswith(indent) and re.match(r"(///|#\[)", lines[j - 1][n:]):
            j -= 1
        e = starts[i + 1] if i + 1 < len(starts) else len(lines)
        while e > k and (lines[e - 1].strip() == "" or (lines[e - 1].startswith(indent) and re.match(r"(///|#\[|//)", lines[e - 1][n:]))):
            e -= 1
        out.append((name, j, e))
    return out


def use_names(text, here):
    """name -> full path, for the `use` statements in `text` (a module at `here`, a crate:: path)."""
    out = {}
    for m in re.finditer(r"(?m)^\s*use\s+([^;]+);", text):
        spec = re.sub(r"\s+", "", m.group(1))

        def walk(prefix, s):
            # s: a, a::b, a::{x, y::{z}}, a as b
            parts, depth, cur = [], 0, ""
            for c in s:
                if c == "{":
                    depth += 1
                elif c == "}":
                    depth -= 1
                if c == "," and depth == 0:
                    parts.append(cur)
                    cur = ""
                else:
                    cur += c
            if cur:
                parts.append(cur)
            for p in parts:
                if "::{" in p and p.endswith("}"):
                    head, rest = p.split("::{", 1)
                    walk(prefix + [head], rest[:-1])
                elif p.startswith("{") and p.endswith("}"):
                    walk(prefix, p[1:-1])
                else:
                    segs = p.split("::")
                    name = segs[-1]
                    if name in ("*", "self"):
                        continue
                    full = prefix + segs
                    out[name] = full
        walk([], spec)
    res = {}
    for name, full in out.items():
        if full[0] == "super":
            full = here[:-1] + full[1:]
        elif full[0] == "self":
            full = here + full[1:]
        res[name] = "::".join(full)
    return res


def idents(text):
    text = re.sub(r"//[^\n]*", "", text)
    text = re.sub(r'"(?:\\.|[^"\\])*"', '""', text)
    return set(re.findall(r"(?<![\w:])[A-Za-z_]\w*", text))


def split_tests(path, modpath, forbidden, dest_name, dest_doc, transform=None, preamble=""):
    """Takes the tests in `path`'s `mod tests` that use any of `forbidden` (names its test module
    imports from modules above the destination crate) out of it. Returns the module text to place
    in springrep, with the imports it needs, and rewrites `path` to keep the rest. Moved tests may
    reach only `pub(crate)`/`pub` items of their old module (which become `pub` in the move)."""
    text = rd(path)
    k = text.find("#[cfg(test)]\nmod tests {\n")
    if k < 0:
        raise SystemExit(f"{path}: no `mod tests` block")
    if not text.rstrip().endswith("}"):
        raise SystemExit(f"{path}: the tests module does not end the file")
    head, tmod = text[:k], text[k:]
    lines = tmod.rstrip("\n").split("\n")
    inner = lines[2:-1]
    its = items(inner, "    ")
    uses = [(n, j, e) for n, j, e in its if inner[j].lstrip().startswith(("use ", "#[")) and "use " in "\n".join(inner[j:e])]
    rest = [(n, j, e) for n, j, e in its if (n, j, e) not in uses]
    test_use_text = "\n".join("\n".join(inner[j:e]) for _, j, e in uses)
    tu = use_names(test_use_text, modpath + ["tests"])
    hu = use_names("\n".join(l for l in head.split("\n") if not l.startswith(" ") or l.startswith("    ")), modpath)
    own = {}
    for n, j, e in items(head.split("\n")):
        l = head.split("\n")[j:e]
        sig = next((x for x in l if ITEM.match(x)), "")
        if re.match(r"(fn|const|static|struct|enum|type|trait)\b", sig):
            own[n] = ("private", "::".join(modpath + [n]))
        elif re.match(r"pub(\([^)]*\))?\s+(fn|const|static|struct|enum|type|trait)\b", sig):
            own[n] = ("pub", "::".join(modpath + [n]))
    forbid = {n for n in tu if n in forbidden}
    bodies = {n: "\n".join(inner[j:e]) for n, j, e in rest}
    moving = {n for n, b in bodies.items() if "#[test]" in b and idents(b) & forbid}
    helpers = {n for n, b in bodies.items() if "#[test]" not in b}
    changed = True
    while changed:
        changed = False
        for h in helpers - moving:
            if any(h in idents(bodies[m]) for m in moving):
                moving.add(h)
                changed = True
    stays = [n for n, _, _ in rest if n not in moving or (n in helpers and any(n in idents(bodies[s]) for s in bodies if s not in moving))]
    if not moving:
        raise SystemExit(f"{path}: no test uses {sorted(forbidden)}")
    moved_text = "\n\n".join(bodies[n] for n, _, _ in rest if n in moving)
    if transform:
        moved_text = transform(moved_text)
    stay_text = "\n\n".join(bodies[n] for n in stays)
    if idents(stay_text) & forbid:
        raise SystemExit(f"{path}: a staying test still uses {sorted(idents(stay_text) & forbid)}")
    used = idents(moved_text)
    private = sorted(n for n in used if own.get(n, ("", ""))[0] == "private" and n not in moving)
    if private:
        raise SystemExit(f"{path}: moved tests reach private items {private}")

    paths = {}
    for n in used:
        if n in tu:
            paths[n] = tu[n]
        elif n in own:
            paths[n] = own[n][1]
        elif n in hu:
            paths[n] = hu[n]
    imports = "".join(f"    use {p};\n" for p in sorted(set(paths.values())))
    moved_mod = (f"/// {dest_doc}\n#[cfg(test)]\nmod {dest_name} {{\n{imports}\n" + preamble
                 + moved_text + "\n}\n")
    # the staying test module keeps only the imports its tests still use
    still = idents(stay_text)
    kept_uses = []
    for n, j, e in uses:
        u = "\n".join(inner[j:e])
        if "super::*" in u:
            kept_uses.append(u)
            continue
        names = use_names(u, modpath + ["tests"])
        keep = sorted(x for x in names if x in still)
        if not keep:
            continue
        if len(keep) == len(names):
            kept_uses.append(u)
        else:
            kept_uses.append("\n".join(f"    use {names[x]};" for x in keep))
    new_tmod = "#[cfg(test)]\nmod tests {\n" + "\n".join(kept_uses) + "\n\n" + stay_text + "\n}\n"
    wr(path, head + new_tmod)
    return moved_mod, sorted(moving - helpers), sorted(moving & helpers)


def allow_panic_at(lines, k):
    """`#[allow(clippy::panic)]` on the fn enclosing line k, on the fn's own line (no file grows)."""
    ind = len(lines[k]) - len(lines[k].lstrip())
    for j in range(k, -1, -1):
        l = lines[j]
        li = len(l) - len(l.lstrip())
        if li < ind and re.match(r"\s*(#\[allow\(clippy::panic\)\]\s*)?(pub(\([^)]*\))?\s+)?(const\s+)?fn\s", l):
            if "#[allow(clippy::panic)]" not in l:
                lines[j] = " " * li + "#[allow(clippy::panic)] " + l.lstrip()
            return
    raise SystemExit(f"no enclosing fn for line {k}")


OPTION_UNWRAP = '.unwrap_or_else(|| panic!("called `Option::unwrap()` on a `None` value"))'


def counted_panics(text, sites, f):
    """Each (old, new, n) is an `.expect(msg)`/`.unwrap()` found n times and its
    `.unwrap_or_else(|..| panic!(..))`, which panics with the same message."""
    for a, b, n in sites:
        if text.count(a) != n:
            raise SystemExit(f"{f}: expected {n} of {a!r}, found {text.count(a)}")
        text = text.replace(a, b)
    lines = text.split("\n")
    for k in [k for k, l in enumerate(lines) if re.search(r"unwrap_or_else\(\|\w*\| panic!\(", l)]:
        allow_panic_at(lines, k)
    return "\n".join(lines)


def strip_module_paths(t, mods, f):
    """Inside a moved crate its old module is the crate root: `crate::m::` -> `crate::`, and an
    `m::{..}`/`m::X` entry of a `use crate::{..}` group -> its items."""
    for m in mods:
        t = re.sub(r"\bcrate::" + m + r"::", "crate::", t)
        # inside each `use crate::{..};`, an `m::..` entry names the crate root now
        t = re.sub(r"\buse crate::\{[^;]*\};", lambda u: re.sub(r"(?<![\w:])" + m + r"::\{([^{}]*)\}", r"\1",
                   re.sub(r"(?<![\w:])" + m + r"::(?!\{)", "", u.group(0))) if True else u.group(0), t)
        if re.search(r"(?m)\bcrate::" + m + r"\b|^\s+" + m + r"::", t):
            raise SystemExit(f"{f}: still names {m} as a module")
    return t


def set_row(path, crate, value, lower=None):
    e = rd(path)
    rows = e.rstrip("\n").split("\n")
    head = [r for r in rows if r.startswith("#")]
    body = [r for r in rows if not r.startswith("#") and r.strip() and not r.startswith(crate + " ")]
    if lower:
        name, by = lower
        m = [r for r in body if r.startswith(name + " ")]
        if len(m) != 1:
            raise SystemExit(f"{path}: no {name} row")
        body = [r for r in body if r != m[0]] + [f"{name} {int(m[0].split()[1]) - by}"]
    if value:
        body.append(f"{crate} {value}")
    wr(path, "\n".join(head + sorted(body)) + "\n")


def cargo(name, deps):
    return ('[package]\nname = "' + name + '"\ndescription = "yack internal crate (not published; use the `yack` crate)"\n'
            'version.workspace = true\nedition.workspace = true\nrust-version.workspace = true\nlicense.workspace = true\n'
            'repository.workspace = true\npublish = false\n\n[dependencies]\n'
            + "".join(f'{d} = {{ path = "../{d}" }}\n' for d in deps) + "\n[lints]\nworkspace = true\n")


def springrep_deps(ws, root, add):
    sc = os.path.join(root, "Cargo.toml")
    c = rd(sc)
    for d in add:
        if f"{d} = " not in c:
            c = sub1(c, "[dependencies]\n", f'[dependencies]\n{d} = {{ path = "../{d}" }}\n', sc)
    wr(sc, c)
    lt = os.path.join(ws, "layers.toml")
    s = rd(lt)
    mm = re.search(r'(?ms)^\[crate\.yack-springrep\]\n.*?^deps = \[([^\]]*)\]', s)
    have = [x.strip().strip('"') for x in mm.group(1).split(",") if x.strip()]
    s = s[:mm.start(1)] + ", ".join(f'"{d}"' for d in have + [d for d in add if d not in have]) + s[mm.end(1):]
    wr(lt, s)


def trim_numeric(src):
    """springrep's numeric.rs re-exports only the yack_numeric names springrep still calls."""
    called = set()
    for p in rs_files(src):
        if p != os.path.join(src, "numeric.rs"):
            for m in re.finditer(r"numeric::(\{[^}]*\}|\w+)", rd(p)):
                called |= {x.strip() for x in m.group(1).strip("{}").split(",") if x.strip()}
    np_ = os.path.join(src, "numeric.rs")
    nt = rd(np_)
    m = re.search(r"pub\(crate\) use yack_numeric::\{([^}]*)\};", nt)
    names = [x.strip() for x in m.group(1).split(",") if x.strip()]
    kept = [n for n in names if n in called]
    wr(np_, nt.replace(m.group(0), "pub(crate) use yack_numeric::{" + ", ".join(kept) + "};"))
    return kept


def live_text(t):
    """`t` without comments and #[cfg(test)] items."""
    t = re.sub(r"//[^\n]*", "", t)
    out, i = [], 0
    while True:
        m = re.search(r"#\[cfg\(test\)\]", t[i:])
        if not m:
            out.append(t[i:])
            return "".join(out)
        s = i + m.start()
        out.append(t[i:s])
        j = s + len(m.group(0))
        semi, br = t.find(";", j), t.find("{", j)
        if semi != -1 and (br == -1 or semi < br):
            i = semi + 1
            continue
        depth, k = 0, br
        while True:
            if t[k] == "{":
                depth += 1
            elif t[k] == "}":
                depth -= 1
                if depth == 0:
                    break
            k += 1
        i = k + 1


def unit_edges(src):
    """A crate's top-level modules and their non-test edges, as xtask's modgraph reads them:
    `crate::unit` paths (and `use crate::{unit..}` groups) from anywhere in a unit, and `super::unit`
    from the unit's root file (`super::super::unit` one level down)."""
    lib = rd(os.path.join(src, "lib.rs"))
    units = re.findall(r"(?m)^\s*(?:pub(?:\([^)]*\))?\s+)?mod\s+(\w+)\s*;", live_text(lib))
    edges = {u: set() for u in units}
    for u in units:
        root = os.path.join(src, u + ".rs")
        files = [(root, 0)] + [(p, 1) for p in rs_files(os.path.join(src, u))]
        for p, depth in files:
            if not os.path.exists(p):
                continue
            t = live_text(rd(p))
            for m in re.finditer(r"\b(crate|super(?:::super)*)::(\{[^;]*?\}|\w+)", t):
                head, rest = m.group(1), m.group(2)
                ok = head == "crate" or (head == "super" and depth == 0) or (head == "super::super" and depth == 1)
                if not ok:
                    continue
                names = re.findall(r"(?:^\{|,)\s*(\w+)", rest) if rest.startswith("{") else [rest]
                edges[u] |= {n for n in names if n in units and n != u}
    # transitively closed, lowest first
    closed = {u: set(edges[u]) for u in units}
    changed = True
    while changed:
        changed = False
        for u in units:
            more = set().union(*[closed[v] for v in closed[u]]) - closed[u] if closed[u] else set()
            if more:
                closed[u] |= more
                changed = True
    order = sorted(units, key=lambda u: (len(closed[u]), u))
    return [(u, [v for v in order if v in closed[u] and v != u]) for u in order]


def crate_names(texts):
    """The crate-root names moved code reaches: `crate::x` and the heads of `use crate::{..}` groups."""
    names = set()
    for t in texts:
        t = re.sub(r"//[^\n]*", "", t)
        names |= set(re.findall(r"\bcrate::(\w+)", t))
        for m in re.finditer(r"\bcrate::\{", t):
            depth, k, cur = 1, m.end(), ""
            while depth:
                c = t[k]
                if c == "{":
                    depth += 1
                elif c == "}":
                    depth -= 1
                if depth == 1 and c == ",":
                    names.add(re.split(r"::|\s", cur.strip())[0])
                    cur = ""
                elif depth >= 1 and not (depth == 1 and c == "}"):
                    cur += c if depth == 1 else ""
                    if depth > 1 and c != "{":
                        pass
                k += 1
            if cur.strip():
                names.add(re.split(r"::|\s", cur.strip())[0])
    return names


def allow_items(text, pattern, lint, note, f):
    """`#[allow(lint)]` on each item of live code that names `pattern`, with `note` as a trailing
    comment, on the item's own line so no file grows: a `use`, a struct field, or else the
    enclosing fn. Nothing wider, so a new use elsewhere still trips the lint."""
    cut = text.find("#[cfg(test)]")
    live, tests = (text, "") if cut < 0 else (text[:cut], text[cut:])
    lines = live.split("\n")
    done = set()
    tag = f"#[allow({lint})] "
    for k, l in enumerate(list(lines)):
        if not re.search(pattern, l) or l.lstrip().startswith("//"):
            continue
        ind0 = len(l) - len(l.lstrip())
        owner = next((lines[i] for i in range(k - 1, -1, -1)
                      if lines[i].strip() and len(lines[i]) - len(lines[i].lstrip()) < ind0
                      and not lines[i].lstrip().startswith(("//", "#"))), "")
        in_struct = re.match(r"\s*(pub(\([^)]*\))?\s+)?(struct|enum|union)\s", owner)
        if re.match(r"\s*(pub(\([^)]*\))?\s+)?use\s", l) or (in_struct and re.match(r"\s+(pub(\([^)]*\))?\s+)?\w+:\s", l)):
            j = k
        else:
            ind = len(l) - len(l.lstrip())
            j = next((i for i in range(k, -1, -1) if re.match(r"\s*(#\[[^\]]*\]\s*)*(pub(\([^)]*\))?\s+)?(const\s+)?fn\s", lines[i])
                      and len(lines[i]) - len(lines[i].lstrip()) <= ind), None)
            if j is None:
                raise SystemExit(f"{f}:{k + 1}: no item to hang the allow on")
        if j in done:
            continue
        done.add(j)
        lj = lines[j]
        ind = len(lj) - len(lj.lstrip())
        lines[j] = " " * ind + tag + lj.lstrip() + f" // {note}"
    return "\n".join(lines) + tests


PUB_ITEM = re.compile(r"pub\(crate\)\s+(?:(?:const|unsafe|async)\s+)*(fn|struct|enum|const|static|type|trait|mod|use)\s+([^;{(<:=\s]+)")
PUB_FIELD = re.compile(r"(?m)^(\s+)pub\(crate\)\s+(\w+)\s*:")


def item_text(t, at, field=False):
    """The text of the item whose `pub(crate)` starts at `at`: to the end of its block or `;`."""
    depth, k = 0, at
    while k < len(t):
        c = t[k]
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return t[at:k + 1]
        elif c == ";" and depth == 0:
            return t[at:k + 1]
        elif depth < 0 or (c == "," and depth == 0 and field):
            return t[at:k]
        k += 1
    return t[at:]


def _in_impl(t, at):
    """Whether position `at` lies inside an `impl` or `trait` block."""
    depth, opens = 0, []
    for m in re.finditer(r"\b(impl|trait)\b[^;{]*\{|[{}]", t[:at]):
        g = m.group(0)
        if g == "}":
            if opens:
                opens.pop()
        else:
            opens.append(g.startswith(("impl", "trait")))
    return any(opens)


def precise_pub(crates_texts, outside):
    """`pub(crate)` -> `pub` only where code outside the crate (springrep's live code, the other
    crates, `outside` text) names the item, a module on an outside path, or a type a needed item's
    text names (so a pub item never shows a crate-private type). The rest stays `pub(crate)`.
    `crates_texts`: {crate: {file: text}}. Returns the rewritten texts and the names made pub."""
    made = {}
    for krate, files in crates_texts.items():
        others = outside + "\n".join(t for k2, fs in crates_texts.items() if k2 != krate for t in fs.values())
        clean = re.sub(r"//[^\n]*", "", others)
        # a free item outside reaches only by a path or a `use`; a method or field by `.name`/`::name`
        path_ids = set(re.findall(r"(?<=::)\s*([A-Za-z_]\w*)", clean))
        for u in re.findall(r"\buse\s+([^;]+);", clean):
            path_ids |= set(re.findall(r"[A-Za-z_]\w*", u))
        member_ids = path_ids | set(re.findall(r"\.\s*([A-Za-z_]\w*)", clean)) | set(re.findall(r"(?<![\w.])([a-z_]\w*)\s*:(?!:)", clean))
        items = []  # (file, start, kind, names)
        for f, t in files.items():
            for m in PUB_ITEM.finditer(t):
                kind, name = m.group(1), m.group(2)
                line_start = t.rfind("\n", 0, m.start()) + 1
                member = kind in ("fn", "const", "type") and m.start() - line_start > 0 and _in_impl(t, m.start())
                if kind == "use":
                    body = item_text(t, m.start())
                    names = [x for x in re.findall(r"(?<![\w])[A-Za-z_]\w*", body.split("use", 1)[1])
                             if x not in ("self", "super", "crate", "as")]
                    names = [body.rstrip(";} \n").split("::")[-1].strip("{} ")] if "{" not in body else names
                else:
                    names = [name]
                items.append((f, m.start(), "member" if member else kind, names))
            for m in PUB_FIELD.finditer(t):
                items.append((f, m.start(), "field", [m.group(2)]))
        need = {i for i, it in enumerate(items)
                if set(it[3]) & (member_ids if it[2] in ("member", "field") else path_ids)}
        names_of = {}
        for i, it in enumerate(items):
            for n in it[3]:
                names_of.setdefault(n, set()).add(i)
        sig_ids = {}
        for i, (f, at, kind, _) in enumerate(items):
            if kind in ("fn", "member", "struct", "enum", "type", "const", "static", "trait", "field"):
                txt = item_text(files[f], at, kind == "field")
                if kind in ("fn", "member"):
                    txt = txt.split("{", 1)[0]  # its signature
                sig_ids[i] = set(re.findall(r"(?<![\w])[A-Za-z_]\w*", txt))
        done = set()
        changed = True
        while changed:
            changed = False
            for i in list(need - done):
                done.add(i)
                if i in sig_ids:
                    for n in sig_ids[i]:
                        for j in names_of.get(n, ()):
                            if j not in need and items[j][2] != "field":
                                need.add(j)
                                changed = True
        for f in files:
            t = files[f]
            for i in sorted((i for i in need if items[i][0] == f), key=lambda i: -items[i][1]):
                at = items[i][1]
                at = t.index("pub(crate)", at)
                t = t[:at] + "pub" + t[at + len("pub(crate)"):]
            files[f] = t
        made[krate] = sorted({n for i in need for n in items[i][3]})
    return crates_texts, made
