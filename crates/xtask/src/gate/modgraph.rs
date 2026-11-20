// yack: gate 5 and `cargo xtask modgraph`: top-level module edges inside one crate.
use super::deps::crate_loc;
use super::lex::Src;
use super::{Ctx, Out};
use std::collections::{BTreeMap, BTreeSet};

/// A crate over this many non-blank lines needs a module map.
const MAP_LOC: usize = 5000;

/// One reference from a file of unit module `from` to unit module `to`.
#[derive(Debug, Clone)]
pub struct Site {
    pub from: String,
    pub to: String,
    /// The path segment after `to::`, when there is one (`crate::boolean::csi` → `csi`).
    pub sub: Option<String>,
    pub path: String,
    pub line: usize,
    /// Inside test-only code.
    pub test: bool,
}

/// The unit a module path belongs to: the segment right under `root` (`[]` = the crate root).
fn unit<'a>(path: &'a [String], root: &[String]) -> Option<&'a String> {
    if path.len() > root.len() && path.iter().zip(root).all(|(a, b)| a == b) { path.get(root.len()) } else { None }
}

/// Every path reference (`crate::..`, `super::..`, `self::..`, including `use` groups) from one
/// unit module to another. Units are the modules directly under `root` (default: the crate root).
pub fn sites(srcs: &[Src], root: &[String]) -> Vec<Site> {
    let units: BTreeSet<String> = srcs.iter().filter_map(|s| unit(&s.module_path, root).cloned()).collect();
    let mut out = Vec::new();
    for s in srcs {
        let Some(m) = unit(&s.module_path, root) else { continue };
        let toks = &s.toks;
        let mut k = 0;
        while k < toks.len() {
            let Some(t) = toks.get(k) else { break };
            let starts = t.is_ident("crate") || t.is_ident("super") || t.is_ident("self");
            // only the head of a path (not `x::super`)
            let headed = k > 0 && toks.get(k - 1).is_some_and(|p| p.is_punct("::"));
            if !starts || headed || !toks.get(k + 1).is_some_and(|x| x.is_punct("::")) {
                k += 1;
                continue;
            }
            let test = !s.live(k);
            // resolve the leading crate/super/self chain into an absolute module path
            let mut base: Vec<String> = if t.is_ident("crate") { Vec::new() } else { s.module_path.clone() };
            let mut j = k;
            if t.is_ident("super") {
                base.pop();
            }
            j += 2;
            while toks.get(j).is_some_and(|x| x.is_ident("super")) && toks.get(j + 1).is_some_and(|x| x.is_punct("::")) {
                base.pop();
                j += 2;
            }
            // then plain segments `a::b::` until a group, a glob or the last segment
            let mut heads: Vec<(Vec<String>, usize)> = Vec::new();
            loop {
                match toks.get(j) {
                    Some(x) if x.is_punct("{") => {
                        let close = super::lex::matching(toks, j);
                        let mut depth = 0;
                        let mut expect = true;
                        for g in j + 1..close {
                            let Some(y) = toks.get(g) else { break };
                            if y.is_punct("{") {
                                depth += 1;
                            } else if y.is_punct("}") {
                                depth -= 1;
                            } else if depth == 0 && y.is_punct(",") {
                                expect = true;
                            } else if depth == 0 && expect {
                                if let Some(name) = y.ident() {
                                    let mut p = base.clone();
                                    p.push(name.to_string());
                                    heads.push((p, g));
                                }
                                expect = false;
                            }
                        }
                        break;
                    }
                    Some(x) if x.ident().is_some() => {
                        let name = x.ident().unwrap_or_default().to_string();
                        base.push(name);
                        if base.len() > root.len() || !toks.get(j + 1).is_some_and(|y| y.is_punct("::")) {
                            heads.push((base.clone(), j));
                            break;
                        }
                        j += 2;
                    }
                    _ => break,
                }
            }
            for (p, at) in heads {
                let Some(to) = unit(&p, root) else { continue };
                if to == m || !units.contains(to) {
                    continue;
                }
                let sub = p.get(root.len() + 1).cloned().or_else(|| {
                    toks.get(at + 1)
                        .filter(|x| x.is_punct("::"))
                        .and_then(|_| toks.get(at + 2))
                        .and_then(|x| x.ident())
                        .map(str::to_string)
                });
                let line = toks.get(at).map(|t| t.line).unwrap_or_default();
                out.push(Site { from: m.clone(), to: to.clone(), sub, path: s.path.clone(), line, test });
            }
            k = j + 1;
        }
    }
    out
}

/// Unit module → the other unit modules its non-test code names.
pub fn edges(srcs: &[Src], root: &[String]) -> BTreeMap<String, BTreeSet<String>> {
    let mut out: BTreeMap<String, BTreeSet<String>> =
        srcs.iter().filter_map(|s| unit(&s.module_path, root).cloned()).map(|t| (t, BTreeSet::new())).collect();
    for x in sites(srcs, root).into_iter().filter(|x| !x.test) {
        out.entry(x.from).or_default().insert(x.to);
    }
    out
}

/// Edges that break `[modules.<crate>]`.
fn violations(map: &BTreeMap<String, Vec<String>>, edges: &BTreeMap<String, BTreeSet<String>>) -> Vec<String> {
    let mut out = Vec::new();
    for (m, targets) in edges {
        let Some(allowed) = map.get(m) else {
            out.push(format!("module `{m}` is not in the module map"));
            continue;
        };
        for t in targets.iter().filter(|t| !allowed.contains(t)) {
            out.push(format!("module edge `{m}` → `{t}` is not in the map"));
        }
    }
    out
}

/// Gate 5: in a crate with a `[modules.<crate>]` map (required over 5k LOC), every top-level
/// module edge is in the map. For temporaries the map is the M2 target, enforced from tag `m2`.
pub fn check(ctx: &Ctx, out: &mut Out) {
    for (name, spec, srcs) in ctx.crates() {
        let loc = crate_loc(ctx, name).unwrap_or(0);
        match ctx.layers.modules.get(name) {
            Some(map) => {
                let v = violations(map, &edges(srcs, ctx.layers.module_root(name)));
                if spec.is_temporary() && !ctx.tree.has_tag("m2") {
                    // a temporary's map is its M2 target: reported, enforced from tag `m2`
                    if !v.is_empty() {
                        out.note(format!("`{name}`: {} module-map violations (enforced from tag m2; `cargo xtask modgraph --crate {name}`)", v.len()));
                    }
                } else {
                    for x in v {
                        out.fail(format!("`{name}`: {x}"));
                    }
                }
            }
            None if loc > MAP_LOC && (!spec.is_temporary() || ctx.tree.has_tag("m2")) => {
                out.fail(format!("`{name}` has {loc} lines but no [modules.{name}] map in layers.toml"));
            }
            None => {}
        }
    }
}

/// `cargo xtask modgraph --crate <name>`.
pub fn main(args: &[String]) -> Result<bool, String> {
    let krate = args
        .iter()
        .position(|a| a == "--crate")
        .and_then(|i| args.get(i + 1))
        .ok_or("usage: cargo xtask modgraph --crate <name>")?;
    let ctx = super::Ctx::new(crate::meta::load(&crate::meta::root())?)?;
    let srcs = ctx.srcs.get(krate.as_str()).ok_or(format!("no crate `{krate}`"))?;
    let root = ctx.layers.module_root(krate);
    let e = edges(srcs, root);
    for (m, ts) in &e {
        println!("{m} -> {}", ts.iter().cloned().collect::<Vec<_>>().join(", "));
    }
    let Some(map) = ctx.layers.modules.get(krate.as_str()) else {
        println!("(no [modules.{krate}] map in layers.toml)");
        return Ok(true);
    };
    let v = violations(map, &e);
    for x in &v {
        println!("error: {x}");
    }
    // every site behind a violating edge (test-only sites marked), for the M2 worklists
    println!("== violating sites (from -> to[::sub] at file:line)");
    for x in sites(srcs, root) {
        let allowed = map.get(&x.from).is_some_and(|a| a.contains(&x.to));
        if !allowed {
            let sub = x.sub.as_ref().map(|s| format!("::{s}")).unwrap_or_default();
            let test = if x.test { " (test)" } else { "" };
            println!("site {} -> {}{sub} at {}:{}{test}", x.from, x.to, x.path, x.line);
        }
    }
    Ok(v.is_empty())
}
