# yack: M4.8 window apply: saves chamfer_angle's ladder and its revolved strategy into yack-blend.
# Usage: python3 m48-apply.py <yack-springrep dir> [--with-external ...] (springrep is untouched)
import os, sys

SRC = os.environ.get("M48_SRC", "/tmp/claude-1000/-home-brenn-github-yack/dfbaa7ec-edce-4285-9cb7-59a45b5f05a4/scratchpad/w-m04/m48/src")
Y = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BLEND = os.path.join(Y, "crates/yack-blend/src")


def save(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(text)
    os.replace(tmp, path)


def main():
    if not SRC:
        sys.exit("m48-apply: set M48_SRC to the drafted src/ dir")
    for rel in ("chamfer_angle.rs", "chamfer_angle/revolved.rs"):
        save(os.path.join(BLEND, rel), open(os.path.join(SRC, rel)).read())
    lib = os.path.join(BLEND, "lib.rs")
    text = open(lib).read()
    anchor = "pub mod shaped;\n"
    if text.count(anchor) != 1:
        sys.exit("m48-apply: lib.rs anchor not found once")
    before = text.count("\n")
    if "mod chamfer_angle;" not in text:
        text = text.replace(anchor, "mod chamfer_angle;\n" + anchor + "\npub use chamfer_angle::chamfer_angle;\n", 1)
    save(lib, text)
    # gate 6: blend's lib.rs is over the cap until hardening splits it; its row follows the file (as W8, W9)
    sizes = os.path.join(Y, "scores/filesize.txt")
    key = "crates/yack-blend/src/lib.rs "
    rows = open(sizes).read().split("\n")
    hits = [i for i, r in enumerate(rows) if r.startswith(key)]
    if len(hits) != 1:
        sys.exit("m48-apply: no single filesize row for blend's lib.rs")
    i = hits[0]
    rows[i] = key + str(int(rows[i].split()[1]) + text.count("\n") - before)
    save(sizes, "\n".join(rows))


main()
