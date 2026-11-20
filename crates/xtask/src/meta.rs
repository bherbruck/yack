// yack: the workspace as the gates see it: files, packages (from `cargo metadata`), tags.
//
// The gates run on a `Tree`, never on the disk directly, so a planted-violation fixture
// (crates/xtask/fixtures/) can stand in for the real workspace.
use crate::json::{self, Json};
use crate::toml::{self, Toml};
use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};
use std::process::Command;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DepKind {
    Normal,
    Dev,
    Build,
}

#[derive(Debug, Clone)]
pub struct Dep {
    /// The package name (not a rename).
    pub name: String,
    pub kind: DepKind,
    /// A path dependency, i.e. on another workspace crate.
    pub internal: bool,
}

#[derive(Debug, Clone)]
pub struct Package {
    pub name: String,
    /// The crate directory relative to the root, e.g. `crates/yack-base`.
    pub dir: String,
    /// `publish` is not `false`.
    pub publishable: bool,
    pub license: Option<String>,
    pub deps: Vec<Dep>,
}

/// The workspace: every file path, the text of the files the gates read, the packages and tags.
#[derive(Debug, Clone, Default)]
pub struct Tree {
    /// Every file, relative to the root, `/`-separated.
    pub paths: BTreeSet<String>,
    /// The text of each readable source/config file (a subset of `paths`).
    pub files: BTreeMap<String, String>,
    /// Directories that must not exist but do: `target`, `crates/<c>/target`, `.git/worktrees`.
    pub forbidden_dirs: BTreeSet<String>,
    pub packages: Vec<Package>,
    /// Git tags (milestones are tagged `m0`, `m1`, ...).
    pub tags: BTreeSet<String>,
}

impl Tree {
    pub fn file(&self, path: &str) -> Option<&str> {
        self.files.get(path).map(String::as_str)
    }

    pub fn package(&self, name: &str) -> Option<&Package> {
        self.packages.iter().find(|p| p.name == name)
    }

    pub fn has_tag(&self, milestone: &str) -> bool {
        self.tags.contains(&milestone.to_ascii_lowercase()) || self.tags.contains(milestone)
    }
}

/// The repository root (xtask lives in `crates/xtask`).
pub fn root() -> PathBuf {
    let here = Path::new(env!("CARGO_MANIFEST_DIR"));
    here.parent().and_then(Path::parent).unwrap_or(here).to_path_buf()
}

const SKIP_TOP: [&str; 6] = ["bin", ".git", "swarm", ".claude", ".playwright-mcp", "node_modules"];
const FIXTURES: &str = "crates/xtask/fixtures";
const MAX_READ: u64 = 8 << 20;

fn is_text(rel: &str) -> bool {
    let name = rel.rsplit('/').next().unwrap_or(rel);
    name == "NOTICE"
        || name == ".gitignore"
        || name == "Cargo.lock"
        || [".rs", ".toml", ".txt"].iter().any(|e| name.ends_with(e))
}

/// Loads the real workspace at `root`.
pub fn load(root: &Path) -> Result<Tree, String> {
    let mut tree = Tree::default();
    walk(root, root, &mut tree)?;
    let wt = root.join(".git").join("worktrees");
    if std::fs::read_dir(&wt).map(|mut d| d.next().is_some()).unwrap_or(false) {
        tree.forbidden_dirs.insert(".git/worktrees".to_string());
    }
    tree.packages = cargo_metadata(root)?;
    if let Ok(out) = Command::new("git").arg("tag").arg("--list").current_dir(root).output() {
        let text = String::from_utf8_lossy(&out.stdout);
        tree.tags = text.lines().map(|l| l.trim().to_string()).filter(|l| !l.is_empty()).collect();
    }
    Ok(tree)
}

fn rel_of(root: &Path, p: &Path) -> String {
    let rel = p.strip_prefix(root).unwrap_or(p);
    rel.components().map(|c| c.as_os_str().to_string_lossy().into_owned()).collect::<Vec<_>>().join("/")
}

fn walk(root: &Path, dir: &Path, tree: &mut Tree) -> Result<(), String> {
    let rd = std::fs::read_dir(dir).map_err(|e| format!("read {}: {e}", dir.display()))?;
    let mut entries: Vec<_> = rd.filter_map(Result::ok).collect();
    entries.sort_by_key(|e| e.file_name());
    for e in entries {
        let p = e.path();
        let rel = rel_of(root, &p);
        let ft = e.file_type().map_err(|e| format!("stat {}: {e}", p.display()))?;
        if ft.is_symlink() {
            tree.paths.insert(rel);
            continue;
        }
        if ft.is_dir() {
            let name = e.file_name().to_string_lossy().into_owned();
            if name == "target" {
                tree.forbidden_dirs.insert(rel);
                continue;
            }
            if (dir == root && SKIP_TOP.contains(&name.as_str())) || rel == FIXTURES {
                continue;
            }
            walk(root, &p, tree)?;
        } else {
            if is_text(&rel)
                && e.metadata().map(|m| m.len() <= MAX_READ).unwrap_or(false)
                && let Ok(text) = std::fs::read_to_string(&p)
            {
                tree.files.insert(rel.clone(), text);
            }
            tree.paths.insert(rel);
        }
    }
    Ok(())
}

fn cargo_metadata(root: &Path) -> Result<Vec<Package>, String> {
    let out = Command::new("cargo")
        .args(["metadata", "--format-version", "1", "--no-deps", "--offline"])
        .current_dir(root)
        .output()
        .map_err(|e| format!("cargo metadata: {e}"))?;
    if !out.status.success() {
        return Err(format!("cargo metadata failed: {}", String::from_utf8_lossy(&out.stderr)));
    }
    let v = json::parse(&String::from_utf8_lossy(&out.stdout))?;
    packages_from_metadata(&v, root)
}

/// The workspace packages of a `cargo metadata` document.
pub fn packages_from_metadata(v: &Json, root: &Path) -> Result<Vec<Package>, String> {
    let members: BTreeSet<&str> =
        v.get("workspace_members").map(Json::as_arr).unwrap_or_default().iter().filter_map(Json::as_str).collect();
    let mut out = Vec::new();
    for p in v.get("packages").map(Json::as_arr).unwrap_or_default() {
        let id = p.get("id").and_then(Json::as_str).unwrap_or_default();
        if !members.contains(id) {
            continue;
        }
        let name = p.get("name").and_then(Json::as_str).ok_or("metadata: package without name")?.to_string();
        let manifest = p.get("manifest_path").and_then(Json::as_str).unwrap_or_default();
        let dir = Path::new(manifest).parent().map(|d| rel_of(root, d)).unwrap_or_default();
        let publishable = match p.get("publish") {
            Some(Json::Arr(regs)) => !regs.is_empty(),
            _ => true,
        };
        let license = p.get("license").and_then(Json::as_str).map(str::to_string);
        let mut deps = Vec::new();
        for d in p.get("dependencies").map(Json::as_arr).unwrap_or_default() {
            let kind = match d.get("kind").and_then(Json::as_str) {
                Some("dev") => DepKind::Dev,
                Some("build") => DepKind::Build,
                _ => DepKind::Normal,
            };
            let internal = d.get("path").and_then(Json::as_str).is_some()
                && matches!(d.get("source"), None | Some(Json::Null));
            let dname = d.get("name").and_then(Json::as_str).unwrap_or_default().to_string();
            deps.push(Dep { name: dname, kind, internal });
        }
        out.push(Package { name, dir, publishable, license, deps });
    }
    out.sort_by(|a, b| a.name.cmp(&b.name));
    Ok(out)
}

// ---- fixtures ----

/// Maps a fixture file name to the tree path it stands for: `dot-x` → `.x`, `name.in` → `name`
/// (so fixture manifests and dotfiles never act on the real repository).
fn fixture_name(rel: &str) -> String {
    rel.split('/')
        .map(|c| {
            let c = c.strip_prefix("dot-").map(|r| format!(".{r}")).unwrap_or_else(|| c.to_string());
            c.strip_suffix(".in").map(str::to_string).unwrap_or(c)
        })
        .collect::<Vec<_>>()
        .join("/")
}

fn read_fixture_dir(base: &Path, dir: &Path, out: &mut BTreeMap<String, String>) -> Result<(), String> {
    let rd = std::fs::read_dir(dir).map_err(|e| format!("read {}: {e}", dir.display()))?;
    for e in rd.filter_map(Result::ok) {
        let p = e.path();
        if p.is_dir() {
            read_fixture_dir(base, &p, out)?;
        } else {
            let text = std::fs::read_to_string(&p).map_err(|e| format!("read {}: {e}", p.display()))?;
            out.insert(rel_of(base, &p), text);
        }
    }
    Ok(())
}

/// A fixture tree: the files of `base`, then those of `overlay` on top.
///
/// Control files at the overlay root (not part of the tree): `_delete.txt` lists paths to remove,
/// `_dirs.txt` forbidden directories that "exist", `_tags.txt` the git tags, `_expect.txt` the
/// extra gate numbers the fixture is allowed to fail.
pub fn load_fixture(base: &Path, overlay: Option<&Path>) -> Result<Tree, String> {
    let mut raw = BTreeMap::new();
    read_fixture_dir(base, base, &mut raw)?;
    let mut control = BTreeMap::new();
    if let Some(o) = overlay {
        let mut over = BTreeMap::new();
        read_fixture_dir(o, o, &mut over)?;
        for (k, v) in over {
            if k.starts_with('_') {
                control.insert(k, v);
            } else {
                raw.insert(k, v);
            }
        }
    }
    let mut files: BTreeMap<String, String> = raw.into_iter().map(|(k, v)| (fixture_name(&k), v)).collect();
    let lines = |k: &str| -> Vec<String> {
        control
            .get(k)
            .map(|t| t.lines().map(str::trim).filter(|l| !l.is_empty() && !l.starts_with('#')).map(str::to_string).collect())
            .unwrap_or_default()
    };
    for d in lines("_delete.txt") {
        files.remove(&d);
    }
    let mut tree = Tree {
        paths: files.keys().cloned().collect(),
        forbidden_dirs: lines("_dirs.txt").into_iter().collect(),
        tags: lines("_tags.txt").into_iter().collect(),
        ..Tree::default()
    };
    tree.packages = packages_from_manifests(&files)?;
    tree.files = files;
    Ok(tree)
}

/// Fixture packages, read from `crates/*/Cargo.toml` (what `cargo metadata` would report).
fn packages_from_manifests(files: &BTreeMap<String, String>) -> Result<Vec<Package>, String> {
    let ws = files.get("Cargo.toml").map(|t| toml::parse(t)).transpose()?;
    let ws_license = ws.as_ref().and_then(|w| w.path(&["workspace", "package", "license"])).and_then(Toml::as_str);
    let mut out = Vec::new();
    for (path, text) in files {
        let Some(dir) = path.strip_suffix("/Cargo.toml") else { continue };
        if !dir.starts_with("crates/") || dir.matches('/').count() != 1 {
            continue;
        }
        let m = toml::parse(text).map_err(|e| format!("{path}: {e}"))?;
        let name = m.path(&["package", "name"]).and_then(Toml::as_str).ok_or(format!("{path}: no name"))?;
        let publishable = m.path(&["package", "publish"]).and_then(Toml::as_bool).unwrap_or(true);
        let license = match m.path(&["package", "license"]) {
            Some(Toml::Str(s)) => Some(s.clone()),
            Some(Toml::Table(_)) => ws_license.map(str::to_string),
            _ => None,
        };
        let mut deps = Vec::new();
        for (table, kind) in [("dependencies", DepKind::Normal), ("dev-dependencies", DepKind::Dev), ("build-dependencies", DepKind::Build)] {
            for (key, spec) in m.get(table).and_then(Toml::as_table).into_iter().flatten() {
                let dname = spec.get("package").and_then(Toml::as_str).unwrap_or(key).to_string();
                deps.push(Dep { name: dname, kind, internal: spec.get("path").is_some() });
            }
        }
        out.push(Package { name: name.to_string(), dir: dir.to_string(), publishable, license, deps });
    }
    Ok(out)
}
