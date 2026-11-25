# yack: interleaved timing of two yack-bench binaries on yack-geom's hot cases (w-h4-pairtime.py's
# pattern), measured in child CPU time as well as wall time, so a loaded machine's waiting time
# drops out. usage: python3 scripts/w-h2-pairtime.py PRE_BIN POST_BIN [ROUNDS] [CASE ...]
import os, resource, subprocess, sys, tempfile, time
CASES = ['swept/ring-carried-along-a-spline', 'swept/tumbling-cube', 'fillet/bezier-dome-one-edge']
pre, post = sys.argv[1], sys.argv[2]
rounds = int(sys.argv[3]) if len(sys.argv) > 3 else 5
cases = sys.argv[4:] or CASES
out = tempfile.mkdtemp(prefix='w-h2-pairtime-')
cpu = {c: [0.0, 0.0] for c in cases}
wall = {c: [0.0, 0.0] for c in cases}
def used():
    r = resource.getrusage(resource.RUSAGE_CHILDREN)
    return r.ru_utime + r.ru_stime
for r in range(rounds):
    for c in cases:
        order = ((0, pre), (1, post)) if r % 2 == 0 else ((1, post), (0, pre))
        for side, b in order:
            stl = os.path.join(out, c.replace('/', '_') + f'-{side}.stl')
            c0, t0 = used(), time.perf_counter()
            p = subprocess.run([b, c, stl], capture_output=True)
            wall[c][side] += time.perf_counter() - t0
            cpu[c][side] += used() - c0
            if p.returncode != 0:
                print(f'{c}: side {side} exit {p.returncode}', file=sys.stderr)
worst = 0.0
for c in cases:
    (a, b), (wa, wb) = cpu[c], wall[c]
    worst = max(worst, b / a)
    print(f'{c}: cpu pre {a:.2f} s post {b:.2f} s post/pre {b / a:.3f} | wall post/pre {wb / wa:.3f} ({rounds} rounds)')
print(f'worst cpu post/pre {worst:.3f}')
