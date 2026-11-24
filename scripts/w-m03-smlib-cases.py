#!/usr/bin/env python3
# yack: extracts the SMLib corpus's cases and SMLib's recorded results for `xtask corpus smlib`.
# Reads cadrs@eb5899ef crates/cadrs_kernel/tests/smlib_{booleans,fillets,offset}.rs (MIT OR
# Apache-2.0, (c) 2026 Rouan van der Ende), whose expectations are SMLib's own results baked in by
# cadrs's tools/smlib_to_rust.py from the SMLib oracle; the models are NVIDIA usd-brep's test files
# (Apache-2.0). Nothing here computes an expectation: each number is copied as recorded.
#
# usage: scripts/w-m03-smlib-cases.py <cadrs clone> <yack root>
#   writes corpus/smlib-cases.json and corpus/smlib.sha256 (every model file: size and SHA-256)
import hashlib
import json
import os
import re
import sys

CADRS_REV = "eb5899efab2081c76e82f2401e1462d476189493"


def fns(text):
    """(name, body, known_failure) of each `#[test] fn`."""
    out = []
    for m in re.finditer(r"#\[test\]\s*(?:#\[ignore[^\]]*\]\s*)?fn (\w+)\(\) \{", text):
        i, depth = m.end(), 1
        while depth and i < len(text):
            depth += {"{": 1, "}": -1}.get(text[i], 0)
            i += 1
        body = text[m.end():i - 1]
        out.append((m.group(1), body, "known_failure(" in body))
    return out


def expect(body):
    e = {}
    for k in ("volume", "area"):
        m = re.search(rf"{k}: Some\(([-0-9.eE+]+)\)", body)
        e[k] = float(m.group(1)) if m else None
    for k in ("faces", "edges"):
        m = re.search(rf"{k}: Some\((\d+)\)", body)
        e[k] = int(m.group(1)) if m else None
    m = re.search(r"solid: Some\((true|false)\)", body)
    e["solid"] = (m.group(1) == "true") if m else None
    return e


def strings(s):
    # "file.smb", or "part.smp#n": the n-th B-rep of a part
    return re.findall(r'"([^"]+\.(?:sm[bp]|txt)(?:#\d+)?)"', s)


OPS = {"U": "union", "I": "intersection", "D": "difference"}


def booleans(text):
    cases, skipped = [], []
    for name, body, kf in fns(text):
        m = re.search(r"\b(sheet_chain|chain)\(\s*&\[([^\]]*)\],\s*([UID])\b", body)
        if not m:
            skipped.append((name, "not a chain of files (primitives, grids: v1 leaves them out)"))
            continue
        c = {"id": f"booleans/{name}", "op": OPS[m.group(3)], "inputs": [f"booleans/{f}" for f in strings(m.group(2))],
             "expect": expect(body), "cadrs_known_failure": kf}
        if m.group(1) == "sheet_chain":
            c["expect"]["volume"], c["expect"]["solid"] = None, False
        cases.append(c)
    return cases, skipped


def fillets(text):
    cases, skipped = [], []
    for name, body, kf in fns(text):
        m = re.search(r"\bblend\(\s*\"([^\"]+)\",\s*&\[([^\]]*)\],\s*(true|false),\s*Op::(Fillet|Chamfer)\(([-0-9.eE+]+)\)", body)
        if not m:
            skipped.append((name, "not a single-size blend of a file (laws: v1 leaves them out)"))
            continue
        ranges = [[int(a), int(b)] for a, b in re.findall(r"\((\d+),\s*(\d+)\)", m.group(2))]
        cases.append({"id": f"fillets/{name}", "op": m.group(4).lower(), "inputs": [f"fillets/{m.group(1)}"],
                      "size": float(m.group(5)), "edges": {"ranges": ranges, "include": m.group(3) == "true"},
                      "expect": expect(body), "cadrs_known_failure": kf})
    return cases, skipped


def offset(text):
    cases, skipped = [], []
    for name, body, kf in fns(text):
        m = re.search(r"\bshell\(\s*\"([^\"]+)\",\s*([-0-9.eE+]+),\s*&\[([^\]]*)\],\s*(true|false),\s*([-0-9.eE+]+)", body)
        if m:
            open_faces = [int(x) for x in re.findall(r"\d+", m.group(3))]
            cases.append({"id": f"offset/{name}", "op": "shell", "inputs": [f"offset/{m.group(1)}"],
                          "size": float(m.group(2)), "faces": open_faces, "sharp": m.group(4) == "true",
                          "tolerance": float(m.group(5)), "expect": expect(body), "cadrs_known_failure": kf})
            continue
        m = re.search(r"\boffset\(\s*\"([^\"]+)\",\s*([-0-9.eE+]+),\s*None,\s*None", body)
        if m:
            cases.append({"id": f"offset/{name}", "op": "offset", "inputs": [f"offset/{m.group(1)}"],
                          "size": float(m.group(2)), "expect": expect(body), "cadrs_known_failure": kf})
            continue
        skipped.append((name, "an offset with kept faces or a cutter (v1 leaves them out)"))
    return cases, skipped


def main():
    cadrs, root = sys.argv[1], sys.argv[2]
    tests = os.path.join(cadrs, "crates/cadrs_kernel/tests")
    data = os.path.join(tests, "data/smlib")
    cases, skipped = [], []
    for suite, f in (("booleans", booleans), ("fillets", fillets), ("offset", offset)):
        c, s = f(open(os.path.join(tests, f"smlib_{suite}.rs"), encoding="utf-8").read())
        cases += c
        skipped += [(f"{suite}/{n}", why) for n, why in s]
    doc = {
        "header": {
            "what": "SMLib test cases for `cargo xtask corpus smlib`: inputs, operation, and SMLib's own recorded results",
            "models": "NVIDIA usd-brep TestFiles/pt_TestFiles (Apache-2.0, (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES), unmodified; fetched, never committed",
            "cases_from": f"cadrs@{CADRS_REV} crates/cadrs_kernel/tests/smlib_{{booleans,fillets,offset}}.rs (MIT OR Apache-2.0, (c) 2026 Rouan van der Ende), generated by its tools/smlib_to_rust.py with SMLib's results as the oracle",
            "extracted_by": "scripts/w-m03-smlib-cases.py (numbers copied as recorded; none computed)",
            "tolerance": "volume and area relative 1e-4 (cadrs's REL_TOL); face and edge counts only when every input face is planar",
        },
        "cases": sorted(cases, key=lambda c: c["id"]),
        "not_converted": [{"id": i, "why": w} for i, w in sorted(skipped)],
    }
    os.makedirs(os.path.join(root, "corpus"), exist_ok=True)
    with open(os.path.join(root, "corpus/smlib-cases.json"), "w", encoding="utf-8") as fh:
        json.dump(doc, fh, indent=1, sort_keys=False, ensure_ascii=False)
        fh.write("\n")
    lines = []
    for d, _, files in sorted(os.walk(data)):
        for fn in sorted(files):
            p = os.path.join(d, fn)
            rel = os.path.relpath(p, data).replace(os.sep, "/")
            h = hashlib.sha256(open(p, "rb").read()).hexdigest()
            lines.append(f"{h}  {os.path.getsize(p):>9}  {rel}")
    with open(os.path.join(root, "corpus/smlib.sha256"), "w", encoding="utf-8") as fh:
        fh.write(f"# SMLib corpus manifest: SHA-256, size, path under cadrs@{CADRS_REV} tests/data/smlib (NVIDIA usd-brep, Apache-2.0)\n")
        fh.write("\n".join(lines) + "\n")
    per = {}
    for c in cases:
        per[c["id"].split("/")[0]] = per.get(c["id"].split("/")[0], 0) + 1
    print(f"cases {len(cases)} {per}; not converted {len(skipped)}; files {len(lines)}")


main()
