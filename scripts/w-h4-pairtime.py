# yack: interleaved timing of two yack-bench binaries on the hot cases of yack-intersect's hardening.
# usage: python3 scripts/w-h4-pairtime.py PRE_BIN POST_BIN [ROUNDS] [CASE ...]
# Each round runs every case by PRE then POST (the order alternating by round), so load drift
# cancels; per case it prints both CPU-time totals (user + system of the child) and POST/PRE.
# Outputs go to a private temp dir.
import os, resource, subprocess, sys, tempfile
CASES = ['swept/ring-carried-along-a-spline', 'swept/tumbling-cube', 'fillet/bezier-dome-one-edge']
pre, post = sys.argv[1], sys.argv[2]
rounds = int(sys.argv[3]) if len(sys.argv) > 3 else 5
cases = sys.argv[4:] or CASES
out = tempfile.mkdtemp(prefix='w-h4-pairtime-')
tot = {c: [0.0, 0.0] for c in cases}
for r in range(rounds):
    for c in cases:
        order = ((0, pre), (1, post)) if r % 2 == 0 else ((1, post), (0, pre))
        for side, b in order:
            stl = os.path.join(out, c.replace('/', '_') + f'-{side}.stl')
            r0 = resource.getrusage(resource.RUSAGE_CHILDREN)
            p = subprocess.run([b, c, stl], capture_output=True)
            r1 = resource.getrusage(resource.RUSAGE_CHILDREN)
            tot[c][side] += (r1.ru_utime - r0.ru_utime) + (r1.ru_stime - r0.ru_stime)
            if p.returncode != 0:
                print(f'{c}: {os.path.basename(os.path.dirname(b))} exit {p.returncode}', file=sys.stderr)
worst = 0.0
for c in cases:
    a, b = tot[c]
    worst = max(worst, b / a)
    print(f'{c}: CPU pre {a:.2f} s, post {b:.2f} s over {rounds} rounds, post/pre {b / a:.3f}')
print(f'worst post/pre {worst:.3f}')
