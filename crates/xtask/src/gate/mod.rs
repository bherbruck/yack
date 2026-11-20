// yack: `cargo xtask gate`, the mechanical structure gates of PLAN §6.6.
//
// Every check reads a `Tree` (meta.rs), so each one is proven by a planted-violation fixture in
// crates/xtask/fixtures/ (see the tests at the bottom).
pub mod api;
mod deps;
mod files;
pub mod lex;
mod lints;
pub mod modgraph;
mod source;

use crate::meta::Tree;
use crate::toml::{self, Toml};
use lex::Src;
use std::collections::BTreeMap;

/// One crate's entry in layers.toml.
#[derive(Debug, Clone)]
pub struct Spec {
    pub layer: i64,
    pub kind: String,
    /// The allow-list, with `*kernel` expanded.
    pub deps: Vec<String>,
    pub delete_by: Option<String>,
    pub license: Option<String>,
    pub external: Vec<String>,
}

impl Spec {
    pub fn is_kernel(&self) -> bool {
        self.kind == "kernel"
    }
    pub fn is_tool(&self) -> bool {
        self.kind == "tool"
    }
    pub fn is_temporary(&self) -> bool {
        self.kind == "temporary"
    }
    /// Kernel, temporary engine or the public facade: code that ships.
    pub fn ships(&self) -> bool {
        !self.is_tool()
    }
}

#[derive(Debug, Clone, Default)]
pub struct Layers {
    pub crates: BTreeMap<String, Spec>,
    pub reexport: Vec<String>,
    /// `[modules.<crate>]`: top-level module → allowed top-level modules.
    pub modules: BTreeMap<String, BTreeMap<String, Vec<String>>>,
}

pub fn parse_layers(text: &str) -> Result<Layers, String> {
    let t = toml::parse(text).map_err(|e| format!("layers.toml: {e}"))?;
    let mut crates = BTreeMap::new();
    let table = t.get("crate").and_then(Toml::as_table).cloned().unwrap_or_default();
    for (name, c) in &table {
        let layer = match c.get("layer") {
            Some(Toml::Int(n)) => *n,
            _ => return Err(format!("layers.toml: crate `{name}` has no integer layer")),
        };
        let kind = c.get("kind").and_then(Toml::as_str).unwrap_or_default().to_string();
        if !["kernel", "public", "tool", "temporary"].contains(&kind.as_str()) {
            return Err(format!("layers.toml: crate `{name}` has unknown kind `{kind}`"));
        }
        crates.insert(
            name.clone(),
            Spec {
                layer,
                kind,
                deps: c.get("deps").map(Toml::strings).unwrap_or_default(),
                delete_by: c.get("delete_by").and_then(Toml::as_str).map(str::to_string),
                license: c.get("license").and_then(Toml::as_str).map(str::to_string),
                external: c.get("external").map(Toml::strings).unwrap_or_default(),
            },
        );
    }
    let kernels: Vec<String> = crates.iter().filter(|(_, s)| s.is_kernel()).map(|(n, _)| n.clone()).collect();
    for s in crates.values_mut() {
        if s.deps.iter().any(|d| d == "*kernel") {
            s.deps.retain(|d| d != "*kernel");
            s.deps.extend(kernels.iter().cloned());
        }
    }
    let reexport = t.path(&["reexport", "items"]).map(Toml::strings).unwrap_or_default();
    let mut modules = BTreeMap::new();
    for (krate, m) in t.get("modules").and_then(Toml::as_table).into_iter().flatten() {
        let map = m.as_table().into_iter().flatten().map(|(k, v)| (k.clone(), v.strings())).collect();
        modules.insert(krate.clone(), map);
    }
    Ok(Layers { crates, reexport, modules })
}

/// What the checks read: the tree, layers.toml and every crate's tokenized sources.
pub struct Ctx {
    pub tree: Tree,
    pub layers: Layers,
    /// crate name → its sources.
    pub srcs: BTreeMap<String, Vec<Src>>,
}

impl Ctx {
    pub fn new(tree: Tree) -> Result<Ctx, String> {
        let layers = parse_layers(tree.file("layers.toml").ok_or("layers.toml is missing")?)?;
        let srcs = tree.packages.iter().map(|p| (p.name.clone(), lex::crate_sources(&p.dir, &tree.files))).collect();
        Ok(Ctx { tree, layers, srcs })
    }

    pub fn spec(&self, krate: &str) -> Option<&Spec> {
        self.layers.crates.get(krate)
    }

    /// (crate, spec, sources) for every workspace crate listed in layers.toml.
    pub fn crates(&self) -> impl Iterator<Item = (&str, &Spec, &[Src])> {
        self.tree.packages.iter().filter_map(|p| {
            let spec = self.layers.crates.get(&p.name)?;
            Some((p.name.as_str(), spec, self.srcs.get(&p.name).map(Vec::as_slice).unwrap_or_default()))
        })
    }

    /// A `<key> <count>` ratchet file under scores/ (absent → empty).
    pub fn ratchet(&self, file: &str) -> BTreeMap<String, usize> {
        let text = self.tree.file(&format!("scores/{file}")).unwrap_or_default();
        text.lines()
            .map(str::trim)
            .filter(|l| !l.is_empty() && !l.starts_with('#'))
            .filter_map(|l| {
                let (k, n) = l.rsplit_once(char::is_whitespace)?;
                Some((k.trim().to_string(), n.parse().ok()?))
            })
            .collect()
    }
}

/// A check's findings. Any `fail` fails the gate.
#[derive(Debug, Default)]
pub struct Out {
    pub fails: Vec<String>,
    pub warns: Vec<String>,
    pub notes: Vec<String>,
}

impl Out {
    pub fn fail(&mut self, msg: impl Into<String>) {
        self.fails.push(msg.into());
    }
    pub fn warn(&mut self, msg: impl Into<String>) {
        self.warns.push(msg.into());
    }
    pub fn note(&mut self, msg: impl Into<String>) {
        self.notes.push(msg.into());
    }
}

type Check = fn(&Ctx, &mut Out);

/// The checks of PLAN §6.6, by number.
pub const CHECKS: [(u8, &str, Check); 13] = [
    (1, "layers", deps::layers),
    (2, "one public crate", deps::one_public),
    (3, "zero dependencies", deps::zero_deps),
    (4, "no leaks", api::no_leaks),
    (5, "module graph", modgraph::check),
    (6, "file size", files::file_size),
    (7, "epsilons", source::epsilons),
    (8, "hidden state", source::hidden_state),
    (9, "lints", lints::lints),
    (10, "provenance and licence", files::provenance),
    (11, "build directory", files::build_dir),
    (12, "temporaries", deps::temporaries),
    (13, "facade routes, never computes", lints::facade),
];

/// Runs every check.
pub fn run_all(ctx: &Ctx) -> Vec<(u8, &'static str, Out)> {
    CHECKS
        .iter()
        .map(|(n, name, f)| {
            let mut out = Out::default();
            f(ctx, &mut out);
            (*n, *name, out)
        })
        .collect()
}

/// The current values of the count ratchets: (scores file, contents).
pub fn ratchet_files(ctx: &Ctx) -> Vec<(&'static str, String)> {
    let fmt = |m: BTreeMap<String, usize>| m.into_iter().map(|(k, v)| format!("{k} {v}\n")).collect::<String>();
    vec![
        ("eps.txt", format!("# small float literals without `// tol:` per crate (gate 7; may only fall)\n{}", fmt(source::eps_counts(ctx)))),
        ("filesize.txt", format!("# files over 1500 lines and their allowed length (gate 6; may only fall)\n{}", fmt(files::oversize(ctx)))),
        ("hidden.txt", format!("# hidden-state sites per temporary crate (gate 8; may only fall)\n{}", fmt(source::hidden_counts(ctx)))),
        ("lints.txt", format!("# denied-lint sites per temporary crate (gate 9; may only fall)\n{}", fmt(lints::lint_counts(ctx)))),
        ("cadrs-loc.txt", format!("# yack-cadrs lines of code (gate 12; may only fall)\n{}", fmt(deps::cadrs_loc(ctx)))),
    ]
}

/// `cargo xtask gate [--bless]`.
pub fn main(args: &[String]) -> Result<bool, String> {
    let root = crate::meta::root();
    // `--fixture <name>`: run on crates/xtask/fixtures/base plus that overlay instead of the tree
    if let Some(name) = args.iter().position(|a| a == "--fixture").and_then(|i| args.get(i + 1)) {
        let dir = root.join("crates/xtask/fixtures");
        let overlay = (name != "base").then(|| dir.join(name));
        let ctx = Ctx::new(crate::meta::load_fixture(&dir.join("base"), overlay.as_deref())?)?;
        println!("gate on fixture `{name}`");
        return Ok(report(&run_all(&ctx)));
    }
    let ctx = Ctx::new(crate::meta::load(&root)?)?;
    if args.iter().any(|a| a == "--bless") {
        for (file, text) in ratchet_files(&ctx) {
            let p = root.join("scores").join(file);
            std::fs::write(&p, text).map_err(|e| format!("write {}: {e}", p.display()))?;
            println!("blessed scores/{file}");
        }
    }
    Ok(report(&run_all(&ctx)))
}

/// Prints the results; true when every check passed.
fn report(results: &[(u8, &'static str, Out)]) -> bool {
    let mut ok = true;
    for (n, name, out) in results {
        println!("-- gate {n} {name}");
        for f in out.fails.iter().take(15) {
            println!("error: gate {n}: {f}");
        }
        if out.fails.len() > 15 {
            println!("error: gate {n}: ... and {} more", out.fails.len() - 15);
        }
        for w in out.warns.iter().take(20) {
            println!("  warning: {w}");
        }
        if out.warns.len() > 20 {
            println!("  warning: ... and {} more", out.warns.len() - 20);
        }
        for m in &out.notes {
            println!("  {m}");
        }
        ok &= out.fails.is_empty();
    }
    println!("== gate summary");
    for (n, name, out) in results {
        let status = if out.fails.is_empty() { "OK  " } else { "FAIL" };
        println!("gate {n:>2} {status} ({} findings, {} warnings) {name}", out.fails.len(), out.warns.len());
    }
    let failed: Vec<String> = results.iter().filter(|(_, _, o)| !o.fails.is_empty()).map(|(n, _, _)| n.to_string()).collect();
    if ok {
        println!("gate: all {} checks passed", results.len());
    } else {
        println!("error: gate failed: checks {} (of {})", failed.join(", "), results.len());
    }
    ok
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::meta::load_fixture;
    use std::path::{Path, PathBuf};

    fn fixtures() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("fixtures")
    }

    fn failing(tree: Tree) -> BTreeMap<u8, Vec<String>> {
        let ctx = Ctx::new(tree).unwrap();
        run_all(&ctx).into_iter().filter(|(_, _, o)| !o.fails.is_empty()).map(|(n, _, o)| (n, o.fails)).collect()
    }

    #[test]
    fn base_fixture_passes_every_check() {
        let f = failing(load_fixture(&fixtures().join("base"), None).unwrap());
        assert!(f.is_empty(), "base fixture fails: {f:#?}");
    }

    /// Each `fail_<NN>_<what>` overlay must fail check NN, and only the checks it lists in
    /// `_expect.txt` besides; each `pass_*` overlay must pass every check.
    #[test]
    fn every_planted_violation_fails_its_check() {
        let mut dirs: Vec<_> = std::fs::read_dir(fixtures())
            .unwrap()
            .filter_map(Result::ok)
            .map(|e| e.path())
            .filter(|p| p.file_name().unwrap().to_string_lossy().starts_with("fail_"))
            .collect();
        dirs.sort();
        let mut covered = std::collections::BTreeSet::new();
        let mut problems = Vec::new();
        for d in &dirs {
            let name = d.file_name().unwrap().to_string_lossy().into_owned();
            let n: u8 = name.split('_').nth(1).unwrap().parse().unwrap();
            let mut allowed: Vec<u8> = std::fs::read_to_string(d.join("_expect.txt"))
                .unwrap_or_default()
                .split_whitespace()
                .map(|s| s.parse().unwrap())
                .collect();
            allowed.push(n);
            let f = failing(load_fixture(&fixtures().join("base"), Some(d)).unwrap());
            if !f.contains_key(&n) {
                problems.push(format!("{name}: check {n} did not fail"));
            }
            for (k, msgs) in &f {
                if !allowed.contains(k) {
                    problems.push(format!("{name}: unexpected failure of check {k}: {msgs:?}"));
                }
            }
            covered.insert(n);
        }
        for d in std::fs::read_dir(fixtures()).unwrap().filter_map(Result::ok).map(|e| e.path()) {
            let name = d.file_name().unwrap().to_string_lossy().into_owned();
            if name.starts_with("pass_") {
                let f = failing(load_fixture(&fixtures().join("base"), Some(&d)).unwrap());
                if !f.is_empty() {
                    problems.push(format!("{name}: should pass but fails {f:?}"));
                }
            }
        }
        for (n, name, _) in CHECKS {
            if !covered.contains(&n) {
                problems.push(format!("check {n} ({name}) has no fail_{n:02}_* fixture"));
            }
        }
        assert!(problems.is_empty(), "{problems:#?}");
    }

    #[test]
    fn layers_expand_star_kernel() {
        let l = parse_layers(
            "[crate.a]\nlayer = 0\nkind = \"kernel\"\ndeps = []\n[crate.y]\nlayer = 1\nkind = \"public\"\ndeps = [\"*kernel\"]\n",
        )
        .unwrap();
        assert_eq!(l.crates["y"].deps, ["a"]);
    }
}
