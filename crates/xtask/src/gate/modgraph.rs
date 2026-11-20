// yack: gate 5 and `cargo xtask modgraph`: top-level module edges inside one crate.
use super::deps::crate_loc;
use super::lex::{Src, matching};
use super::{Ctx, Out};
use std::collections::{BTreeMap, BTreeSet};

/// A crate over this many non-blank lines needs a module map.
const MAP_LOC: usize = 5000;

/// Top-level module → the other top-level modules its (non-test) code names.
pub fn edges(srcs: &[Src]) -> BTreeMap<String, BTreeSet<String>> {
    let tops: BTreeSet<String> = srcs.iter().filter_map(|s| s.top_module.clone()).collect();
    let mut out: BTreeMap<String, BTreeSet<String>> = tops.iter().map(|t| (t.clone(), BTreeSet::new())).collect();
    for s in srcs {
        let Some(m) = &s.top_module else { continue };
        let depth1 = s.path.ends_with(&format!("src/{m}.rs")) || s.path.ends_with(&format!("src/{m}/mod.rs"));
        let mut found = BTreeSet::new();
        for (k, t) in s.toks.iter().enumerate() {
            if !s.live(k) || !(t.is_ident("crate") || (depth1 && t.is_ident("super"))) {
                continue;
            }
            if !s.toks.get(k + 1).is_some_and(|x| x.is_punct("::")) {
                continue;
            }
            match s.toks.get(k + 2) {
                Some(x) if x.is_punct("{") => {
                    // use crate::{a, b::c, d::{..}}
                    let close = matching(&s.toks, k + 2);
                    let mut depth = 0;
                    let mut expect_name = true;
                    for y in s.toks.get(k + 3..close).unwrap_or_default() {
                        if y.is_punct("{") {
                            depth += 1;
                        } else if y.is_punct("}") {
                            depth -= 1;
                        } else if depth == 0 && y.is_punct(",") {
                            expect_name = true;
                        } else if let Some(name) = y.ident().filter(|_| depth == 0 && expect_name) {
                            found.insert(name.to_string());
                            expect_name = false;
                        }
                    }
                }
                Some(x) => {
                    if let Some(name) = x.ident() {
                        found.insert(name.to_string());
                    }
                }
                None => {}
            }
        }
        let entry = out.entry(m.clone()).or_default();
        entry.extend(found.into_iter().filter(|f| f != m && tops.contains(f)));
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

/// Gate 5: in a crate with a `[modules.<crate>]` map (required over 5k LOC; for temporaries from
/// tag `m2`), every top-level module edge is in the map.
pub fn check(ctx: &Ctx, out: &mut Out) {
    for (name, spec, srcs) in ctx.crates() {
        let loc = crate_loc(ctx, name).unwrap_or(0);
        match ctx.layers.modules.get(name) {
            Some(map) => {
                for v in violations(map, &edges(srcs)) {
                    out.fail(format!("`{name}`: {v}"));
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
    Ok(v.is_empty())
}
