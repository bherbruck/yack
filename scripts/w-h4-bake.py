# yack: one yack-bench bake-off binary on a list of cases, under two engines, interleaved: each case's result and CPU time per engine.
# usage: python3 scripts/w-h4-bake.py BIN CASES_FILE [ROUNDS] [ENGINE_A ENGINE_B [EXTRA ...]]
# EXTRA goes after `--engine E` on every run (e.g. `--op swept`, forcing that op only).
# CASES_FILE holds one case id a line. Prints a tab-separated row per case: case, then for each engine
# status, volume, area, faces, CPU seconds (the mean over ROUNDS, children's user + system).
import json, os, resource, subprocess, sys, tempfile
b, cases_file = sys.argv[1], sys.argv[2]
rounds = int(sys.argv[3]) if len(sys.argv) > 3 else 1
engines = sys.argv[4:6] if len(sys.argv) > 5 else ['cadrs', 'native']
extra = sys.argv[6:]
cases = [l.strip() for l in open(cases_file) if l.strip()]
out = tempfile.mkdtemp(prefix='w-h4-bake-')
res = {}
for r in range(rounds):
    for c in cases:
        for e in (engines if r % 2 == 0 else engines[::-1]):
            stl = os.path.join(out, c.replace('/', '_') + f'-{e}.stl')
            r0 = resource.getrusage(resource.RUSAGE_CHILDREN)
            try:
                p = subprocess.run([b, '--engine', e, *extra, c, stl], capture_output=True, text=True, timeout=600)
                line = p.stdout.strip().splitlines()[-1] if p.stdout.strip() else ''
                try:
                    j = json.loads(line)
                except ValueError:
                    j = {'status': f'exit{p.returncode}', 'reason': p.stderr.strip()[-200:]}
            except subprocess.TimeoutExpired:
                j = {'status': 'timeout'}
            r1 = resource.getrusage(resource.RUSAGE_CHILDREN)
            t = (r1.ru_utime - r0.ru_utime) + (r1.ru_stime - r0.ru_stime)
            k = (c, e)
            prev = res.get(k)
            res[k] = (j, (prev[1] if prev else 0.0) + t)
print('case\t' + '\t'.join(f'{e}:status\t{e}:volume\t{e}:area\t{e}:faces\t{e}:cpu' for e in engines))
for c in cases:
    row = [c]
    for e in engines:
        j, t = res[(c, e)]
        faces = sum(j.get('faces', {}).values()) if isinstance(j.get('faces'), dict) else ''
        st = j.get('status', '?')
        if st != 'ok':
            st += ':' + str(j.get('reason', ''))[:80].replace('\t', ' ')
        row += [st, str(j.get('volume', '')), str(j.get('area', '')), str(faces), f'{t / rounds:.3f}']
    print('\t'.join(row))
