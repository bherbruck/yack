// yack: gate 5 and `cargo xtask modgraph`: top-level module edges inside one crate.
use super::deps::crate_loc;
use super::lex::{Src, matching};
use super::{Ctx, Out};
use std::collections::{BTreeMap, BTreeSet};

/// A crate over this many non-blank lines needs a module map.
const MAP_LOC: usize = 5000;

/// One reference from a file of top-level module `from` to top-level module `to`.
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

/// Every `crate::<top>` (and, in a top-level module's own file, `super::<top>`) reference.
pub fn sites(srcs: &[Src]) -> Vec<Site> {
    let tops: BTreeSet<String> = srcs.iter().filter_map(|s| s.top_module.clone()).collect();
    let mut out = Vec::new();
    for s in srcs {
        let Some(m) = &s.top_module else { continue };
        let depth1 = s.path.ends_with(&format!("src/{m}.rs")) || s.path.ends_with(&format!("src/{m}/mod.rs"));
        let toks = &s.toks;
        // the identifier at `k` names a module; `sub` is the segment after it
        let mut push = |k: usize, test: bool| {
            let Some(name) = toks.get(k).and_then(|t| t.ident()) else { return };
            if name == m || !tops.contains(name) {
                return;
            }
            let sub = toks
                .get(k + 1)
                .filter(|x| x.is_punct("::"))
                .and_then(|_| toks.get(k + 2))
                .and_then(|x| x.ident())
                .map(str::to_string);
            let line = toks.get(k).map(|t| t.line).unwrap_or_default();
            out.push(Site { from: m.clone(), to: name.to_string(), sub, path: s.path.clone(), line, test });
        };
        for (k, t) in toks.iter().enumerate() {
            if !(t.is_ident("crate") || (depth1 && t.is_ident("super"))) {
                continue;
            }
            if !toks.get(k + 1).is_some_and(|x| x.is_punct("::")) {
                continue;
            }
            let test = !s.live(k);
            match toks.get(k + 2) {
                Some(x) if x.is_punct("{") => {
                    // use crate::{a, b::c, d::{..}}
                    let close = matching(toks, k + 2);
                    let mut depth = 0;
                    let mut expect_name = true;
                    for j in k + 3..close {
                        let Some(y) = toks.get(j) else { break };
                        if y.is_punct("{") {
                            depth += 1;
                        } else if y.is_punct("}") {
                            depth -= 1;
                        } else if depth == 0 && y.is_punct(",") {
                            expect_name = true;
                        } else if depth == 0 && expect_name && y.ident().is_some() {
                            push(j, test);
                            expect_name = false;
                        }
                    }
                }
                Some(_) => push(k + 2, test),
                None => {}
            }
        }
    }
    out
}

/// Top-level module → the other top-level modules its non-test code names.
pub fn edges(srcs: &[Src]) -> BTreeMap<String, BTreeSet<String>> {
    let mut out: BTreeMap<String, BTreeSet<String>> =
        srcs.iter().filter_map(|s| s.top_module.clone()).map(|t| (t, BTreeSet::new())).collect();
    for x in sites(srcs).into_iter().filter(|x| !x.test) {
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
                let v = violations(map, &edges(srcs));
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
    let e = edges(srcs);
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
    for x in sites(srcs) {
        let allowed = map.get(&x.from).is_some_and(|a| a.contains(&x.to));
        if !allowed {
            let sub = x.sub.as_ref().map(|s| format!("::{s}")).unwrap_or_default();
            let test = if x.test { " (test)" } else { "" };
            println!("site {} -> {}{sub} at {}:{}{test}", x.from, x.to, x.path, x.line);
        }
    }
    Ok(v.is_empty())
}
