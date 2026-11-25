#!/usr/bin/env python3
# yack: W11 (M3): crates/yack off yack-springrep; paths to the crates the items live in, springrep's tests to crates/yack/tests.
"""W11 (w-m03-api): crates/yack off yack-springrep.

Rewrites every `yack_springrep::` path in crates/yack/src to the crate the item now lives in
(W1-W10's homes; the facade items per w-h1's W10 table), moves springrep's own tests (tests/*.rs,
tests/fixtures, src/moved_tests, facade/api.rs's two remaining tests) to crates/yack/tests with
the same rewrite (ruling (b)), and replaces the yack-springrep dependency in crates/yack/Cargo.toml
by the crates actually named. Prints every file it writes and every path it could not map
(a non-empty list is a failure: exit 1, nothing written).

usage: w11-apply.py [--root DIR] [--dry]   (DIR defaults to the tree; --dry writes nothing)
       w11-apply.py <tree>/crates/yack-springrep [--with-external ...]   (as scripts/m2-window.sh calls it)
"""
import os
import re
import sys

ROOT = '/home/brenn/github/yack'
args = sys.argv[1:]
if '--root' in args:
    ROOT = args[args.index('--root') + 1]
elif args and args[0].rstrip('/').endswith('crates/yack-springrep'):
    # scripts/m2-window.sh's call: APPLY.py <springrep dir> [--with-external FILE...]
    ROOT = os.path.dirname(os.path.dirname(args[0].rstrip('/')))
DRY = '--dry' in args

# springrep's root names (crates/yack-springrep/src/lib.rs, after W10) -> their homes
MODS = {
    'blend': 'yack_blend', 'boolean': 'yack_boolean', 'build': 'yack_build::build', 'check': 'yack_check',
    'codec': 'yack_codec', 'intersect': 'yack_intersect', 'distance': 'yack_locate::distance', 'draw': 'yack_view::draw', 'edit': 'yack_edit',
    'geom': 'yack_geom', 'loft': 'yack_build::loft', 'math': 'yack_math::kernel', 'measure': 'yack_measure',
    'mesh': 'yack_mesh', 'names': 'yack_topo::names', 'overlap': 'yack_view::overlap',
    'primitive': 'yack_build::primitive', 'probe': 'yack_view::probe', 'profile': 'yack_profile',
    'rays': 'yack_locate::rays', 'step': 'yack_step', 'sweep': 'yack_build::sweep', 'tol': 'yack_tol',
    'topo': 'yack_topo', 'Error': 'yack_base::kernel::Error', 'Result': 'yack_base::kernel::Result',
}
# facade/ after W10 (w-h1's w10-plan.txt)
FACADE = {'edit': 'yack_edit::glue', 'exchange': 'yack_step::exchange', 'loops': 'yack_profile::loops'}
API = {}
for item in ['Pick', 'face_within', 'edge_within', 'face_closest', 'nearest_face', 'blend_strays']:
    API[item] = 'yack_mesh::pick::' + item
for item in ['convexity', 'face_normal_at', 'edge_points', 'edge_point', 'radius_collapses', 'rims_square', 'open_at_concave_edge']:
    API[item] = 'yack_topo::local::' + item
for item in ['same_shape', 'walls_cross', 'plane_cuts', 'boxes_overlap', 'face_grid']:
    API[item] = 'yack_measure::guard::' + item
API['periodic_through'] = 'yack_geom::periodic_through'
API['breaks_requirement'] = 'yack_edit::requirement::breaks_requirement'

unmapped = []


def map_path(segs, where):
    """springrep-root path segments -> the new path, or None (recorded)."""
    if not segs:
        return None
    head = segs[0]
    if head == 'facade':
        if len(segs) >= 2 and segs[1] in FACADE:
            return '::'.join([FACADE[segs[1]]] + segs[2:])
        if len(segs) >= 3 and segs[1] == 'api' and segs[2] in API:
            return '::'.join([API[segs[2]]] + segs[3:])
        unmapped.append(f"{where}: facade path {'::'.join(segs)}")
        return None
    if head in MODS:
        return '::'.join([MODS[head]] + segs[1:])
    unmapped.append(f"{where}: {'::'.join(segs)}")
    return None


def split_top(s):
    """Split a use-tree body at its top-level commas."""
    out, depth, cur = [], 0, ''
    for ch in s:
        if ch == '{':
            depth += 1
        elif ch == '}':
            depth -= 1
        if ch == ',' and depth == 0:
            out.append(cur)
            cur = ''
        else:
            cur += ch
    out.append(cur)
    return [x.strip() for x in out if x.strip()]


def leaves(prefix, tree):
    """A use tree flattened to (path segments, alias) leaves."""
    tree = tree.strip()
    if tree.startswith('{') and tree.endswith('}'):
        out = []
        for part in split_top(tree[1:-1]):
            out += leaves(prefix, part)
        return out
    m = re.match(r'^([\w:]*?)::\{(.*)\}$', tree, re.S)
    if m:
        base = prefix + [x for x in m.group(1).split('::') if x]
        out = []
        for part in split_top(m.group(2)):
            out += leaves(base, part)
        return out
    if tree.startswith('{') and tree.endswith('}'):
        out = []
        for part in split_top(tree[1:-1]):
            out += leaves(prefix, part)
        return out
    alias = None
    m = re.match(r'^(.*?)\s+as\s+(\w+)$', tree, re.S)
    if m:
        tree, alias = m.group(1).strip(), m.group(2)
    segs = prefix + [x for x in tree.split('::') if x]
    if segs and segs[-1] == 'self':
        segs = segs[:-1]
    return [(segs, alias)]


def rewrite(text, where, root_names):
    """Rewrite `root::...` paths (root in root_names: `yack_springrep`, and in a moved unit test
    `crate`) to their homes. Returns the new text."""
    roots = '|'.join(root_names)
    api_alias = False
    crate_alias = []

    # 1. use statements whose path starts at a root
    use_re = re.compile(r'^(?P<ind>[ \t]*)(?P<vis>(?:pub(?:\([^)]*\))?\s+)?)use\s+(?P<root>' + roots + r')::(?P<tree>[^;]*);', re.M)

    def use_sub(m):
        nonlocal api_alias
        lines = []
        for segs, alias in leaves([], m.group('tree')):
            if segs == ['facade', 'api'] and (alias in (None, 'api')):
                api_alias = True  # its items are rewritten where they are named
                continue
            new = map_path(segs, where)
            if new is None:
                return m.group(0)
            name = alias if alias else segs[-1]
            tail = '' if new.split('::')[-1] == name else f' as {name}'
            lines.append(f"{m.group('ind')}{m.group('vis')}use {new}{tail};")
        return '\n'.join(lines)

    text = use_re.sub(use_sub, text)

    # 2. `use yack_springrep as sr;`: sr is springrep's root in this file
    for m in re.finditer(r'^[ \t]*use\s+yack_springrep\s+as\s+(\w+);\n', text, re.M):
        crate_alias.append(m.group(1))
    text = re.sub(r'^[ \t]*use\s+yack_springrep\s+as\s+\w+;\n', '', text, flags=re.M)

    # 3. paths written out in full (and through the root alias)
    for root in list(root_names) + crate_alias:
        path_re = re.compile(r'(?<![\w:])' + root + r'::((?:\w+::)*\w+)')

        def path_sub(m):
            segs = m.group(1).split('::')
            # the longest prefix that maps: facade::api::item, facade::loops, or a root module
            new = map_path(segs, where)
            return new if new else m.group(0)

        text = path_re.sub(path_sub, text)

    # 4. `api::item` where facade::api was imported as `api`
    if api_alias:
        def api_sub(m):
            item = m.group(1)
            if item in API:
                return API[item]
            unmapped.append(f'{where}: api::{item}')
            return m.group(0)

        text = re.sub(r'(?<![\w:])api::(\w+)', api_sub, text)
    return text


WORKSPACE = {d.replace('-', '_') for d in os.listdir(os.path.join(ROOT, 'crates'))}


def moved_header(text):
    """A moved test's provenance header records the move (the first `// Changes:` line)."""
    note = ' Moved to crates/yack/tests at W11 (springrep deleted): paths to the crates the items live in.'
    return re.sub(r'^(// Changes: [^\n]*?)\.?$', lambda m: m.group(1) + '.' + note, text, count=1, flags=re.M)


ALLOW = (
    "// W11: springrep's own kernel tests, moved here when springrep went (ruling (b)). They build with the\n"
    "// kernel crates directly and compute their expected values, so the facade's maths ban and the\n"
    "// workspace lints' test-only exemptions (which reach only #[test] fns) do not fit them.\n"
    "#![allow(clippy::disallowed_methods, clippy::disallowed_types, clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::indexing_slicing)]\n"
)


def crate_root(text):
    """A moved test crate's root: the allow block after its leading `//` header lines."""
    lines = text.split('\n')
    i = 0
    while i < len(lines) and (lines[i].startswith('//') and not lines[i].startswith('///')):
        i += 1
    return '\n'.join(lines[:i]) + ('\n' if i else '') + ALLOW + '\n'.join(lines[i:])


def crates_named(text):
    text = '\n'.join(l for l in text.split('\n') if not l.lstrip().startswith('//'))
    named = set(re.findall(r'(?<![\w:])(yack_[a-z_]+)::', text)) | set(re.findall(r'^\s*(?:pub(?:\([^)]*\))?\s+)?use (yack_[a-z_]+)', text, re.M))
    return sorted(named & WORKSPACE)


writes = {}  # path -> new text
removes = []

# crates/yack/src: every file
src = os.path.join(ROOT, 'crates/yack/src')
for name in sorted(os.listdir(src)):
    p = os.path.join(src, name)
    if not name.endswith('.rs'):
        continue
    s = open(p).read()
    if 'yack_springrep' not in s:
        continue
    n = rewrite(s, f'crates/yack/src/{name}', ['yack_springrep'])
    writes[p] = n

# springrep's tests -> crates/yack/tests (ruling (b)): integration tests keep their names under a
# `springrep_` prefix; the fixtures dir goes beside them (include_bytes!("fixtures/..") holds)
SR = os.path.join(ROOT, 'crates/yack-springrep')
T = os.path.join(ROOT, 'crates/yack/tests')
for name in sorted(os.listdir(os.path.join(SR, 'tests'))):
    p = os.path.join(SR, 'tests', name)
    if name == 'fixtures':
        for f in sorted(os.listdir(p)):
            writes[os.path.join(T, 'fixtures', f)] = open(os.path.join(p, f), 'rb').read()
            removes.append(os.path.join(p, f))
        continue
    if not name.endswith('.rs'):
        continue
    s = open(p).read()
    writes[os.path.join(T, 'springrep_' + name)] = crate_root(moved_header(rewrite(s, f'tests/{name}', ['yack_springrep'])))
    removes.append(p)

# src/moved_tests (unit tests of springrep's crate root): one integration test crate,
# crates/yack/tests/springrep_moved/main.rs, its modules beside it; `crate::` was springrep's root
MT = os.path.join(SR, 'src/moved_tests')
for name in sorted(os.listdir(MT)):
    if not name.endswith('.rs'):
        continue
    s = open(os.path.join(MT, name)).read()
    dest = 'main.rs' if name == 'mod.rs' else name
    t = moved_header(rewrite(s, f'moved_tests/{name}', ['crate', 'yack_springrep']))
    writes[os.path.join(T, 'springrep_moved', dest)] = crate_root(t) if dest == 'main.rs' else t
    removes.append(os.path.join(MT, name))

# facade/api.rs's remaining tests (W10 leaves them there, calling the new paths): their `mod tests`
# becomes crates/yack/tests/springrep_facade.rs, `super::item` to the item's home
FA = os.path.join(SR, 'src/facade/api.rs')
if os.path.exists(FA):
    s = open(FA).read()
    m = re.search(r'#\[cfg\(test\)\]\s*mod tests \{\n(.*)\n\}\s*$', s, re.S)
    if m:
        body = '\n'.join(l[4:] if l.startswith('    ') else l for l in m.group(1).split('\n'))
        body = re.sub(r'^use super::\*;\n', '', body, flags=re.M)
        # api.rs's own imports the tests reached through `super` (its `use crate::math::{..}`)
        home = dict(API, Point3='yack_math::kernel::Point3', Vec3='yack_math::kernel::Vec3', Vec2='yack_math::kernel::Vec2')

        def super_use(m):
            out = []
            for segs, alias in leaves([], m.group(2)):
                name = segs[-1] if segs else ''
                if len(segs) != 1 or name not in home:
                    unmapped.append(f'facade/api.rs tests: super::{"::".join(segs)}')
                    return m.group(0)
                out.append(f"{m.group(1)}use {home[name]}{' as ' + alias if alias else ''};")
            return '\n'.join(out)

        body = re.sub(r'^([ \t]*)use super::([^;]*);', super_use, body, flags=re.M)
        body = re.sub(r'(?<![\w:])super::(\w+)', lambda x: API.get(x.group(1), 'super::' + x.group(1)), body)
        if 'super::' in body:
            unmapped.append('facade/api.rs tests: a super:: path with no home')
        head = ('// yack: springrep facade/api.rs\'s last two tests, which need boolean, primitive and edit together.\n'
                '// Adapted from springrep@a03e8a1 src/facade/api.rs (MIT, (c) 2026 Springcraft, Inc.).\n'
                '// Changes: moved to crates/yack/tests at W11 (springrep deleted): paths to the crates the items live in.\n')
        writes[os.path.join(T, 'springrep_facade.rs')] = crate_root(head + rewrite(body, 'facade/api.rs tests', ['crate', 'yack_springrep']))
    else:
        print('  note: facade/api.rs has no `mod tests` (W10 not applied yet, or the tests already moved)')

# Cargo.toml: yack-springrep out, the crates the src names in, the tests' extra crates as dev-deps
cargo = os.path.join(ROOT, 'crates/yack/Cargo.toml')
c = open(cargo).read()
used_src = set()
for p, t in writes.items():
    if p.startswith(src):
        used_src |= set(crates_named(t))
for name in os.listdir(src):
    if name.endswith('.rs') and os.path.join(src, name) not in writes:
        used_src |= set(crates_named(open(os.path.join(src, name)).read()))
used_tests = set()
for p, t in writes.items():
    if p.startswith(T) and isinstance(t, str):
        used_tests |= set(crates_named(t))
dep = lambda k: f'{k.replace("_", "-")} = {{ path = "../{k.replace("_", "-")}" }}'
have = set(re.findall(r'^(yack-[a-z-]+)\s*=', c, re.M))
lines = c.split('\n')
deps_at = lines.index('[dependencies]')
end = next((i for i in range(deps_at + 1, len(lines)) if lines[i].startswith('[')), len(lines))
block = [l for l in lines[deps_at + 1:end] if l.strip() and not l.startswith('yack-springrep')]
for k in sorted(used_src):
    if k.replace('_', '-') not in have and k != 'yack_springrep':
        block.append(dep(k))
block = sorted(set(block), key=lambda l: l.split('=')[0].strip())
dev = sorted(k for k in used_tests if k.replace('_', '-') not in {b.split('=')[0].strip() for b in block} and k != 'yack')
new_lines = lines[:deps_at + 1] + block + [''] + lines[end:]
c2 = '\n'.join(new_lines)
if dev:
    devblock = '[dev-dependencies]\n' + '\n'.join(dep(k) for k in dev) + '\n'
    c2 = c2.replace('\n[features]', '\n' + devblock + '\n[features]', 1) if '[features]' in c2 else c2 + '\n' + devblock
writes[cargo] = c2

# what is left that still names springrep
def code_names_springrep(t):
    return any('yack_springrep' in l for l in t.split('\n') if not l.lstrip().startswith('//'))


left = [f'{p}: still names yack_springrep' for p, t in writes.items() if isinstance(t, str) and code_names_springrep(t)]

print(f'W11 apply ({"dry" if DRY else "write"}) on {ROOT}')
print(f'  rewrites {sum(1 for p in writes if p.startswith(src))} src files; moves {sum(1 for p in writes if p.startswith(T))} test files; removes {len(removes)} from springrep')
print(f'  crates/yack deps: {", ".join(sorted(b.split("=")[0].strip() for b in block))}')
print(f'  dev-deps: {", ".join(k.replace("_", "-") for k in dev) or "none"}')
bad = unmapped + left
for u in bad:
    print('  UNMAPPED', u)
if bad:
    sys.exit(1)
if DRY:
    sys.exit(0)
for p, t in writes.items():
    os.makedirs(os.path.dirname(p), exist_ok=True)
    tmp = p + '.tmp'
    with open(tmp, 'wb' if isinstance(t, bytes) else 'w') as f:
        f.write(t)
    os.replace(tmp, p)
    print('  wrote', os.path.relpath(p, ROOT))
for p in removes:
    os.remove(p)
    print('  removed', os.path.relpath(p, ROOT))
