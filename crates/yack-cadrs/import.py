#!/usr/bin/env python3
# yack: re-creates crates/yack-cadrs/{src,tests} from cadrs_kernel at the pinned commit (M1.8).
# Run: python3 crates/yack-cadrs/import.py  (reads $YACK_REF/cadrs through `git show`, so the
# checkout's working tree does not matter). Every transformation is listed in IMPORT.md.
import os, re, subprocess, sys

PIN = "eb5899efab2081c76e82f2401e1462d476189493"
SHORT = PIN[:8]
REF = os.environ.get("YACK_REF", "/home/brenn/github/yack-ref")
REPO = os.path.join(REF, "cadrs")
KROOT = "crates/cadrs_kernel"
DST = os.path.dirname(os.path.abspath(__file__))
LICENCE = "MIT OR Apache-2.0, (c) 2026 Rouan van der Ende"

# Integration tests taken (no tests/data, no SMLib-derived suites, no proptest, no GPU, not
# exchange_corpus.rs: it runs on OCCT's LGPL dataset).
TESTS = [
    "analytic_blocks.rs", "ball_pass.rs", "chamfer_generated.rs", "conformance.rs",
    "constructors.rs", "determinism.rs", "end_mill_program.rs", "evolved.rs",
    "exchange_regressions.rs", "feat_regressions.rs",
    "fillet_full_round.rs", "metamorphic_booleans.rs", "profile_gaps.rs", "small_models.rs",
    "stress_booleans.rs", "surfacing.rs", "surfacing_curves.rs", "swept_exact.rs",
    "thicken_parity.rs", "validity.rs", "meta_common/mod.rs",
]
# Test data taken: cadrs-authored files only (MIT OR Apache-2.0).
DATA = ["tests/data/colored_assembly.step", "tests/data/occt_box.brep"]
# Unit tests in src that need cadrs's tests/data/smlib (the Apache-2.0 SMLib corpus, not taken).
SMLIB_TESTS = {
    "src/native/hlr.rs": ["smlib_sphere_and_torus_outlines"],
    "src/native/kernel.rs": ["shortcuts_match_the_defaults"],
    "src/native/tess.rs": ["cone_apex_rows_dont_refine_for_ever"],
}
# Lines (of the original files) where a fixed-size vector's `as_slice()` is taken: the shim's
# vectors keep named fields (`v.x`), which can't be viewed as a slice without `unsafe`, so these
# read `to_array().as_slice()` (a temporary array) instead.
AS_SLICE = {
    "src/native/fingerprint.rs": [73, 75, 84, 85, 90, 97, 107, 148, 162],
    "src/native/sweep_cache.rs": [63, 184, 185, 214, 215],
    "src/native/tess.rs": [2309],
    "src/native/topo.rs": [1499, 1501, 1502, 1503, 1507, 1508, 1509, 1522, 1523, 1524, 1525, 1540],
}
IGNORE_SMLIB = '#[ignore = "yack: needs cadrs tests/data/smlib (the SMLib corpus, not imported)"]'


def git(*args):
    return subprocess.run(["git", "-C", REPO, *args], check=True, capture_output=True).stdout


def ls_tree(prefix):
    out = git("ls-tree", "-r", "--name-only", PIN, prefix).decode()
    return [p for p in out.splitlines() if p]


def show(path):
    return git("show", f"{PIN}:{path}")


def purpose(text, rel):
    for line in text.splitlines():
        s = line.strip()
        if s.startswith("//!"):
            p = s[3:].strip()
            if p:
                p = p.rstrip(".")
                return p if len(p) <= 88 else p[:85].rstrip() + "..."
        elif s and not s.startswith("//") and not s.startswith("#!"):
            break
    return f"cadrs_kernel's {rel}"


def strip_serde(text, changes):
    n0 = text
    text = re.sub(r"^use serde::[^;]*;\n", "", text, flags=re.M)

    def fix_derive(m):
        items = [i.strip() for i in m.group(1).split(",")]
        keep = [i for i in items if i and i not in ("Serialize", "Deserialize")]
        return "#[derive(" + ", ".join(keep) + ")]"

    text = re.sub(r"#\[derive\(([^)]*)\)\]", fix_derive, text)
    text = re.sub(r"^[ \t]*#\[serde\([^\]]*\)\]\n", "", text, flags=re.M)
    if text != n0:
        changes.append("serde derives and attributes stripped")
    return text


def fix_env(text, rel, changes):
    n0 = text
    text = text.replace("std::env::var_os(", "crate::native::switch_os(")
    text = text.replace("std::env::var(", "crate::native::switch(")
    if rel == "src/native/mod.rs":
        old = (
            "macro_rules! env_flag {\n"
            "    ($name:literal) => {{\n"
            "        static SET: std::sync::OnceLock<bool> = std::sync::OnceLock::new();\n"
            "        *SET.get_or_init(|| crate::native::switch_os($name).is_some())\n"
            "    }};\n"
            "}\n"
        )
        new = (
            "macro_rules! env_flag {\n"
            "    ($name:literal) => {{\n"
            "        // yack: no environment is read; every switch takes its unset default.\n"
            "        let _ = $name;\n"
            "        false\n"
            "    }};\n"
            "}\n"
            "pub(crate) use env_flag;\n"
            "\n"
            "/// yack: the value of environment switch `name`: always unset (yack reads no\n"
            "/// environment, so every `CADRS_*` switch takes its unset default; see\n"
            "/// crates/yack-cadrs/IMPORT.md).\n"
            "#[allow(dead_code, reason = \"some switches are read only by the test kit\")]\n"
            "pub(crate) fn switch(name: &str) -> Result<String, ()> {\n"
            "    let _ = name;\n"
            "    Err(())\n"
            "}\n"
            "\n"
            "/// yack: [`switch`] for the `var_os` readers: always unset.\n"
            "#[allow(dead_code, reason = \"some switches are read only by the test kit\")]\n"
            "pub(crate) fn switch_os(name: &str) -> Option<std::ffi::OsString> {\n"
            "    let _ = name;\n"
            "    None\n"
            "}\n"
        )
        if old not in text:
            sys.exit("native/mod.rs: env_flag macro not found")
        text = text.replace(old + "pub(crate) use env_flag;\n", new)
    if "std::env" in text:
        sys.exit(f"{rel}: an environment read is left")
    if text != n0:
        changes.append("CADRS_* environment switches fixed at their unset defaults")
    return text


def ignore_tests(text, rel, names, attr, changes):
    for name in names:
        pat = re.compile(r"^([ \t]*)#\[test\]\n([ \t]*)fn " + re.escape(name) + r"\b", re.M)
        m = pat.search(text)
        if not m:
            sys.exit(f"{rel}: test {name} not found")
        text = text[: m.start()] + f"{m.group(1)}#[test]\n{m.group(1)}{attr}\n{m.group(2)}fn {name}" + text[m.end():]
    if names:
        changes.append("unit tests needing the SMLib corpus marked #[ignore]: " + ", ".join(names))
    return text


def tests_calling(text, needle):
    """The #[test] functions whose bodies contain `needle`."""
    names = []
    for m in re.finditer(r"#\[test\]\n[ \t]*fn (\w+)", text):
        start = text.index("{", m.end())
        depth, i = 0, start
        while i < len(text):
            c = text[i]
            if c == "{":
                depth += 1
            elif c == "}":
                depth -= 1
                if depth == 0:
                    break
            i += 1
        if needle in text[start:i]:
            names.append(m.group(1))
    return names


def header(text, rel, changes, kind="src"):
    p = purpose(text, rel)
    ch = "; ".join(changes) if changes else "none (verbatim)"
    lines = [
        f"// yack: {p}.",
        f"// Adapted from cadrs@{SHORT} {KROOT}/{rel} ({LICENCE}).",
    ]
    # Wrap the changes line at 100 columns.
    words = ("// Changes: " + ch + ".").split(" ")
    cur = ""
    for w in words:
        if len(cur) + len(w) + 1 > 100:
            lines.append(cur)
            cur = "//   " + w
        else:
            cur = (cur + " " + w) if cur else w
    lines.append(cur)
    return "\n".join(lines) + "\n" + text


def transform(rel, text):
    changes = []
    if rel in AS_SLICE:
        lines = text.split("\n")
        for ln in AS_SLICE[rel]:
            if ".as_slice()" not in lines[ln - 1]:
                sys.exit(f"{rel}:{ln}: no as_slice() there")
            lines[ln - 1] = lines[ln - 1].replace(".as_slice()", ".to_array().as_slice()")
        text = "\n".join(lines)
        changes.append("a fixed-size vector's as_slice() read as to_array().as_slice() (lines "
                       + ", ".join(map(str, AS_SLICE[rel])) + ")")
    if rel.endswith("src/types.rs") or True:
        text = strip_serde(text, changes)
    if rel.startswith("src/"):
        text = fix_env(text, rel, changes)
    n0 = text
    text = text.replace("cadrs_kernel::", "yack_cadrs::")
    if text != n0:
        changes.append("crate paths cadrs_kernel:: -> yack_cadrs::")
    if rel == "src/native/hlr.rs":
        n0 = text
        text = re.sub(
            r'include_bytes!\(\s*"\.\./\.\./tests/data/smlib/([^"]+)"\s*\)',
            r'std::fs::read(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/smlib/\1")).unwrap_or_default().as_slice()',
            text,
        )
        if text != n0:
            changes.append("include_bytes! of SMLib files read at run time instead")
    names = list(SMLIB_TESTS.get(rel, []))
    if rel == "src/native/smb.rs":
        names += tests_calling(text, "fixture(")
    text = ignore_tests(text, rel, names, IGNORE_SMLIB, changes)
    if rel == "src/native/nurbs.rs":
        # A curve whose knots end short of a full clamp (one carried from another kernel) ran
        # its Bézier decomposition past the knots: index out of bounds (conformance seed 1,
        # random-chains, intersection). The segments found so far are returned instead.
        old = "                    alphas[j - mult - 1] = numer / (u[a + j] - u[a]);\n"
        # (On one line: donor files may not grow, gate 6.)
        new = "                    alphas[j - mult - 1] = numer / (match u.get(a + j) { Some(&uj) => uj, None => return out } - u[a]); // yack: short knots\n"
        if text.count(old) != 1:
            sys.exit("nurbs.rs: the decomposition's knot read not found")
        text = text.replace(old, new)
        changes.append("bezier_segments stops at knots that end short of a full clamp (was an index panic)")
    if rel == "src/native/boolean.rs":
        old = "    let segs = n.bezier_segments();\n"
        new = "    let segs = Some(n.bezier_segments()).filter(|s| !s.is_empty())?; // yack: none whole: as before\n"
        if text.count(old) != 1:
            sys.exit("boolean.rs: span_boxes_of's segments not found")
        text = text.replace(old, new)
        changes.append("span_boxes_of declines a curve with no whole Bézier segment (was an index panic)")
    if rel == "src/native/sweep_faces.rs":
        # The wall-clock give-up (EFFORT_SECONDS) cannot fire under the shim's frozen clock: a
        # deterministic budget of checks takes its place at both sites.
        old = "    let over = || clock.elapsed().as_secs_f64() > effort();\n"
        if text.count(old) != 2:
            sys.exit("sweep_faces.rs: the two wall-clock budgets not found")
        new = (
            "    // yack: a deterministic work budget in place of the wall clock (EFFORT_SECONDS):\n"
            "    // at most SWEEP_CHECKS rounds (splits here, unions of a piece below).\n"
            "    let checks = std::cell::Cell::new(0usize);\n"
            "    let over = || {\n"
            "        checks.set(checks.get() + 1);\n"
            "        checks.get() > SWEEP_CHECKS\n"
            "    };\n"
        )
        text = text.replace(old, new)
        old = "const EFFORT_SECONDS: f64 = 4.0;\n"
        if old not in text:
            sys.exit("sweep_faces.rs: EFFORT_SECONDS not found")
        text = text.replace(old, old + (
            "\n/// yack: the rounds a sweep by faces may split, or unite pieces, before it gives up\n"
            "/// (in place of [`EFFORT_SECONDS`] of wall clock: results never depend on timing).\n"
            "const SWEEP_CHECKS: usize = 4096;\n"))
        changes.append("the two wall-clock give-ups (EFFORT_SECONDS) replaced by a deterministic budget of SWEEP_CHECKS rounds")
    if rel == "src/native/tess.rs":
        old = "use spade::handles::FixedVertexHandle;\n"
        if old not in text:
            sys.exit("tess.rs: spade import not found")
        text = text.replace(old, "use nalgebra::spade;\n" + old, 1)
        changes.append("spade reached as yack-shim's spade facade (`use nalgebra::spade;`)")
    if rel == "src/lib.rs":
        # Gate 4 allows no glob re-export: `pub use types::*;` spelled out (every top-level
        # `pub` item of types.rs, as the glob re-exported them).
        types = show(f"{KROOT}/src/types.rs").decode()
        names = sorted(set(re.findall(r"^pub (?:struct|enum|fn|type|const|trait|static) (\w+)", types, re.M)))
        old = "pub use types::*;\n"
        if old not in text:
            sys.exit("lib.rs: `pub use types::*;` not found")
        body, line = [], "pub use types::{"
        for n in names:
            piece = n + ", "
            if len(line) + len(piece) > 99:
                body.append(line.rstrip())
                line = "    "
            line += piece
        body.append(line.rstrip().rstrip(",") + "};")
        text = text.replace(old, "\n".join(body) + "\n")
        changes.append("`pub use types::*` spelled out (no glob re-exports)")
        # yack's own glue module (src/yack_glue.rs, written by w-m03-api, not imported).
        old = "pub mod types;\n"
        if old not in text:
            sys.exit("lib.rs: `pub mod types;` not found")
        text = text.replace(old, old + "// yack: yack's own glue, not cadrs code (yack_glue.rs, src/facade/; not imported).\n"
                            "pub mod yack_glue;\npub mod facade;\n", 1)
        changes.append("yack's glue modules declared")
        text += (
            "\n"
            "// yack: the shim's other facades under their crates' names. yack-shim is one package,\n"
            "// depended on once as `nalgebra` (Cargo takes one name per package), so the others are\n"
            "// these aliases of it.\n"
            "extern crate nalgebra as rayon;\n"
            "extern crate nalgebra as rustc_hash;\n"
            "extern crate nalgebra as uuid;\n"
            "extern crate nalgebra as web_time;\n"
        )
        changes.append("shim aliases rayon, rustc_hash, uuid, web_time")
    if rel.startswith("tests/") and rel.endswith(".rs") and "/" not in rel[len("tests/"):]:
        aliases = [n for n in ("rayon", "rustc_hash", "uuid", "web_time") if re.search(r"\b" + n + r"::", text)]
        if aliases:
            # After the leading comments and inner attributes.
            lines = text.split("\n")
            i = 0
            while i < len(lines) and (lines[i].startswith("//") or lines[i].startswith("#![") or not lines[i].strip()):
                i += 1
            ext = [f"extern crate nalgebra as {n};" for n in aliases]
            lines[i:i] = ext
            text = "\n".join(lines)
            changes.append("extern crate aliases for the shim facades it uses (" + ", ".join(aliases) + ")")
    return header(text, rel, changes)


def write(rel, data):
    out = os.path.join(DST, rel)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "wb") as f:
        f.write(data)


def main():
    srcs = [p for p in ls_tree(f"{KROOT}/src") if p.endswith(".rs")]
    n = 0
    for p in srcs:
        rel = p[len(KROOT) + 1:]
        write(rel, transform(rel, show(p).decode()).encode())
        n += 1
    for t in TESTS:
        rel = "tests/" + t
        write(rel, transform(rel, show(f"{KROOT}/{rel}").decode()).encode())
        n += 1
    for d in DATA:
        write(d, show(f"{KROOT}/{d}"))
    write("LICENSE-cadrs-MIT", show("LICENSE-MIT"))
    print(f"imported {n} files from cadrs@{SHORT}")


if __name__ == "__main__":
    main()
