# Brace, bracket and paren balance of Rust sources, comments, strings and chars aside: a dry run's
# cheap stand-in for a compile. Usage: python3 brace_check.py PATH...
import os, re, sys

TOK = re.compile(r'//[^\n]*|/\*.*?\*/|r(#*)".*?"\1|b?"(?:\\.|[^"\\])*"|b?\'(?:\\.|[^\'\\])\'|[{}()\[\]]', re.S)
bad = 0
for p in sys.argv[1:]:
    fs = [p] if p.endswith(".rs") else [os.path.join(d, f) for d, _, fl in os.walk(p) for f in fl if f.endswith(".rs")]
    for f in fs:
        t, stack = open(f).read(), []
        for m in TOK.finditer(t):
            c = m.group(0)
            if c in "{([":
                stack.append((c, t.count("\n", 0, m.start()) + 1))
            elif c in "})]":
                want = {"}": "{", ")": "(", "]": "["}[c]
                if not stack or stack[-1][0] != want:
                    print(f"{f}:{t.count(chr(10), 0, m.start()) + 1}: unmatched {c}")
                    bad += 1
                    break
                stack.pop()
        else:
            if stack:
                print(f"{f}:{stack[-1][1]}: unclosed {stack[-1][0]}")
                bad += 1
print("balanced" if not bad else f"{bad} unbalanced")
sys.exit(1 if bad else 0)
