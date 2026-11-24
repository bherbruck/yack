# Counts gate 7's unmarked small float literals (0 < v < 1e-2, outside #[cfg(test)] items, on a line
# without `// tol:`) in the given .rs files or directories, as xtask's gate/source.rs does, so a move
# window can write the scores/eps.txt rows `gate --bless` would. With --panics, counts gate 9's
# contract panics instead (`panic!`, `assert!`, `assert_eq!`, `assert_ne!` in non-test code).
# Usage: python3 eps_count.py [--panics] PATH...
import os, re, sys


def float_value(t):
    t = t.replace("_", "")
    if t[:2] in ("0x", "0o", "0b"):
        return None
    suffixed = t.endswith("f64") or t.endswith("f32")
    body = t[:-3] if suffixed else t
    if not suffixed and not re.search(r"[.eE]", body):
        return None
    if re.search(r"[A-DF-Za-df-z]", body):
        return None
    try:
        return float(body)
    except ValueError:
        return None


TOK = re.compile(r'//[^\n]*|/\*.*?\*/|r(#*)".*?"\1|b?"(?:\\.|[^"\\])*"|b?\'(?:\\.|[^\'\\])\'|'
                 r"[0-9][0-9_]*(?:\.[0-9][0-9_]*)?(?:[eE][+-]?[0-9_]+)?(?:f64|f32)?(?!\w)(?!\.[0-9])|"
                 r"[0-9][0-9_]*\.(?![\w.])|#\[(?:cfg\((?:all\()?test[^\]]*\)|test)\]|[{}]|[A-Za-z_]\w*|.", re.S)


PANICS = MODE = None


def count(text):
    lines = text.split("\n")
    n = 0
    depth, skip_from, pending = 0, None, False
    for m in TOK.finditer(text):
        tok = m.group(0)
        if tok.startswith("#[") and "test" in tok:
            pending = True
            continue
        if tok == "{":
            depth += 1
            if pending and skip_from is None:
                skip_from = depth
            pending = False
            continue
        if tok == "}":
            if skip_from is not None and depth == skip_from:
                skip_from = None
            depth -= 1
            continue
        if tok == ";" and pending and skip_from is None:
            pending = False  # a cfg(test) `use` or `mod x;`
            continue
        if skip_from is not None:
            continue
        if MODE == "panics":
            if tok in ("panic", "assert", "assert_eq", "assert_ne") and text[m.end():m.end() + 1] == "!":
                n += 1
            continue
        if not tok[:1].isdigit():
            continue
        v = float_value(tok)
        line = lines[text.count("\n", 0, m.start())]
        if v is not None and 0.0 < v < 1e-2 and "// tol:" not in line:
            n += 1
    return n


total = 0
args = sys.argv[1:]
if args and args[0] == "--panics":
    MODE, args = "panics", args[1:]
for p in args:
    fs = [p] if p.endswith(".rs") else [os.path.join(d, f) for d, _, fl in os.walk(p) for f in fl if f.endswith(".rs")]
    # files only a #[cfg(test)] `mod` reaches (springrep names them tests.rs, or puts them in tests/)
    fs = [f for f in fs if os.path.basename(f) != "tests.rs" and "/tests/" not in f]
    for f in fs:
        total += count(open(f).read())
print(total)
