// yack: Reading a STEP file: the product structure gives each solid's placements, and each solid is
// Adapted from springrep@a03e8a1 src/step/read.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: this header added; modified: traversal cap (MAX_VISITS bounds walk()/rep_solids()
//   calls, so a product or mapped-item DAG reused at every level is refused, not expanded); then to yack-step (M3 W9c), paths and visibility only; checked access (hardening S1).
//! Reading a STEP file: the product structure gives each solid's placements, and each solid is
//! built once and copied to every placement.

#[allow(clippy::disallowed_types)] use std::collections::{HashMap, HashSet}; // D26: BTree* in hardening, proven by digests
use std::sync::atomic::{AtomicUsize, Ordering};

use yack_p21::{File, Param};

use super::{
    WRITER,
    geometry::{Geometry, Placing},
    part21,
    topology,
};
use crate::{
    Error, Result,
    build::Healed,
    check,
    math::Point3,
    topo::{Brep, FaceId, FaceName, SolidId},
};

/// What a read produced: every solid that became a valid kernel solid, how far healing moved
/// each one's geometry, and why any other was rejected.
#[derive(Debug)]
pub struct Import {
    pub brep: Brep,
    /// One entry per rejected solid placement: `#id: error`.
    pub rejected: Vec<String>,
    /// One entry per placed solid whose geometry healing moved, in placement order.
    pub healed: Vec<Healing>,
    /// One entry per placed solid kept as the file has it, with local tolerances, in placement
    /// order.
    pub tolerant: Vec<Tolerant>,
    /// Every place a placed solid passes through or touches itself, in placement order. Such a
    /// solid is kept as the file has it: an imported mesh stitched with folds microns deep is still
    /// the part, and other systems keep it too.
    pub crossed: Vec<Crossed>,
    /// One entry per solid of `brep`, in its order: what the file calls it and how it draws it.
    pub bodies: Vec<Body>,
    /// The faces the file gives a colour of their own, apart from their body's.
    pub face_colours: Vec<(FaceId, [f64; 3])>,
}

/// A solid of an import as its file describes it.
#[derive(Clone, Debug, PartialEq)]
pub struct Body {
    pub solid: SolidId,
    /// The file's id of the solid it came from; an assembly places one solid many times.
    pub step_id: u64,
    /// The name of the product whose shape holds the solid, where the file has products.
    pub name: Option<String>,
    /// The uses that place it, from the root assembly down: each use's name, or its id where it
    /// has none. Empty for a solid of a root product.
    pub instance: Vec<String>,
    /// The body's surface colour, red, green and blue in 0 to 1, where the file gives one.
    pub colour: Option<[f64; 3]>,
}

/// Two faces of one placed solid that share no edge but meet.
#[derive(Clone, Debug, PartialEq)]
pub struct Crossed {
    /// The solid's position among `Import::brep`'s solids.
    pub index: usize,
    /// The solid's STEP instance.
    pub step_id: u64,
    /// The two faces, in `Import::brep`.
    pub faces: (FaceId, FaceId),
    /// Where they meet, placed as the solid is.
    pub at: Point3,
    /// How far one passes through the other, in mm: zero where they only touch.
    pub depth: f64,
}

/// A placed solid whose geometry could not be healed to meet within [`crate::tol::LINEAR`]
/// without moving it further than the file says it meets, so its edges and vertices carry the
/// tolerances it does meet within. Those are measured, and may be wider than the file states.
#[derive(Clone, Debug, PartialEq)]
pub struct Tolerant {
    /// The solid's position among `Import::brep`'s solids.
    pub index: usize,
    /// The solid's STEP instance.
    pub step_id: u64,
    /// How many of its vertices and edges carry a tolerance above the kernel's.
    pub vertices: usize,
    pub edges: usize,
    /// The largest tolerance of any of them, in mm.
    pub largest: f64,
    /// The uncertainty the file states for the solid, in mm, where it states one.
    pub stated: Option<f64>,
    /// How many of them carry a tolerance above `stated`: where the file misses its own word.
    pub past_stated: usize,
}

/// How far healing moved one placed solid's geometry to close it within the kernel tolerance.
#[derive(Clone, Debug, PartialEq)]
pub struct Healing {
    /// The solid's position among `Import::brep`'s solids.
    pub index: usize,
    /// The solid's STEP instance.
    pub step_id: u64,
    /// The largest distance any vertex, edge or surface moved, in mm.
    pub moved: f64,
    /// Where that movement was, placed as the solid is.
    pub at: Point3,
    /// What moved most, such as `vertex #12` or `the surface of face #34`.
    pub what: String,
    /// The largest distance any of its surfaces moved, which bounds how far its faces moved.
    pub surfaces: f64,
}

const SOLIDS: [&str; 4] = [
    "MANIFOLD_SOLID_BREP",
    "BREP_WITH_VOIDS",
    "FACETED_BREP",
    "TESSELLATED_SOLID",
];

/// The most solid placements one file may expand to; an assembly that reuses its parts
/// exponentially deep is refused rather than expanded.
const MAX_PLACEMENTS: usize = 1_000_000;

/// The most products and representations one file's assembly walk may visit, counting each
/// reuse again: a graph that reuses a part at every level expands exponentially even where it
/// places no solid, so the walk is bounded, not just its output.
const MAX_VISITS: usize = 1_000_000;

fn cyclic<T>(id: u64, what: &str) -> Result<T> {
    Err(Error::InvalidInput(format!("STEP #{id}: {what}")))
}

/// A solid to build, the representation context its units come from, and where it goes.
struct Placement {
    solid: u64,
    context: Option<u64>,
    at: Placing,
    /// The product definition whose shape holds the solid.
    product: Option<u64>,
    /// The assembly uses that place it, root first.
    uses: Vec<u64>,
}

impl Placement {
    /// What the solid is built from: the same solid in the same units at the same scale is built
    /// once.
    fn key(&self) -> Key {
        (self.solid, self.context, self.at.scale.to_bits())
    }
}

type Key = (u64, Option<u64>, u64);

fn params(f: &File, id: u64, kind: &str) -> Option<Vec<Param>> {
    f.get(id)?.part(kind).map(<[Param]>::to_vec)
}

fn refs(p: Option<&Param>) -> Vec<u64> {
    p.and_then(Param::list)
        .map(|l| l.iter().filter_map(Param::reference).collect())
        .unwrap_or_default()
}

/// The representation an instance is (any entity with a `REPRESENTATION` part or subtype that
/// carries `name, items, context`).
fn representation(f: &File, id: u64) -> Option<(Vec<u64>, Option<u64>)> {
    let e = f.get(id)?;
    let p = &e
        .records()
        .iter()
        .find(|r| {
            r.name.ends_with("REPRESENTATION")
                && !r.name.contains("RELATIONSHIP")
                && r.params.len() >= 3
        })?
        .params;
    Some((refs(p.get(1)), p.get(2).and_then(Param::reference)))
}

#[allow(clippy::disallowed_types)] struct Assembly<'a> { // D26: BTree* in hardening, proven by digests
    file: &'a File,
    /// Representations joined without a transformation (a part's shape and its B-rep).
    links: HashMap<u64, Vec<u64>>,
    /// Each product definition's shape representations.
    reps: HashMap<u64, Vec<u64>>,
    /// Each product definition's children: the child, the relation that places it, and the use.
    children: HashMap<u64, Vec<(u64, u64, u64)>>,
    related: HashSet<u64>,
    /// How many products and representations the walk has visited, against [`MAX_VISITS`].
    visits: std::cell::Cell<usize>,
}

#[allow(clippy::disallowed_types)] impl<'a> Assembly<'a> { // D26: BTree* in hardening, proven by digests
    fn new(file: &'a File) -> Assembly<'a> {
        let mut a = Assembly {
            file,
            links: HashMap::new(),
            reps: HashMap::new(),
            children: HashMap::new(),
            related: HashSet::new(),
            visits: std::cell::Cell::new(0),
        };
        let mut nauo_place: HashMap<u64, u64> = HashMap::new();
        for (_, e) in part21::entities(file) {
            if let Some(p) = e.part("SHAPE_DEFINITION_REPRESENTATION") {
                let (Some(pds), Some(rep)) = (
                    p.first().and_then(Param::reference),
                    p.get(1).and_then(Param::reference),
                ) else {
                    continue;
                };
                if let Some(pd) = params(file, pds, "PRODUCT_DEFINITION_SHAPE")
                    .and_then(|q| q.get(2).and_then(Param::reference))
                {
                    a.reps.entry(pd).or_default().push(rep);
                }
            } else if let Some(p) = e.part("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION") {
                let (Some(rel), Some(pds)) = (
                    p.first().and_then(Param::reference),
                    p.get(1).and_then(Param::reference),
                ) else {
                    continue;
                };
                if let Some(nauo) = params(file, pds, "PRODUCT_DEFINITION_SHAPE")
                    .and_then(|q| q.get(2).and_then(Param::reference))
                {
                    nauo_place.insert(nauo, rel);
                }
            } else if !e.is("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION")
                && let Some(p) = e.part("REPRESENTATION_RELATIONSHIP").or_else(|| {
                    e.part("SHAPE_REPRESENTATION_RELATIONSHIP")
                        .filter(|p| p.len() >= 4)
                })
                && let (Some(r1), Some(r2)) = (
                    p.get(2).and_then(Param::reference),
                    p.get(3).and_then(Param::reference),
                )
            {
                a.links.entry(r1).or_default().push(r2);
                a.links.entry(r2).or_default().push(r1);
            }
        }
        for (id, e) in part21::entities(file) {
            if let Some(p) = e.part("NEXT_ASSEMBLY_USAGE_OCCURRENCE") {
                let (Some(parent), Some(child)) = (
                    p.get(3).and_then(Param::reference),
                    p.get(4).and_then(Param::reference),
                ) else {
                    continue;
                };
                if let Some(&rel) = nauo_place.get(&id) {
                    a.children.entry(parent).or_default().push((child, rel, id));
                    a.related.insert(child);
                }
            }
        }
        a
    }

    /// Counts one more visit of `id`, refusing the file once the walk passes [`MAX_VISITS`].
    fn visit(&self, id: u64) -> Result<()> {
        let n = self.visits.get() + 1;
        self.visits.set(n);
        if n > MAX_VISITS {
            return Err(Error::InvalidInput(format!(
                "STEP #{id}: the assembly expands to more than {MAX_VISITS} product and representation uses"
            )));
        }
        Ok(())
    }

    /// Every representation joined to `rep` without a transformation, `rep` included.
    fn linked(&self, rep: u64) -> Vec<u64> {
        let mut seen = vec![rep];
        let mut i = 0;
        while let Some(&at) = seen.get(i) {
            for &r in self.links.get(&at).into_iter().flatten() {
                if !seen.contains(&r) {
                    seen.push(r);
                }
            }
            i += 1;
        }
        seen
    }

    /// The solids in representation `rep` and those joined to it, at `at`. `path` holds the
    /// representations mapped into this one, so a map that contains itself is an error.
    fn rep_solids(
        &self,
        geo: &mut Geometry,
        rep: u64,
        at: &Placing,
        path: &mut Vec<u64>,
        out: &mut Vec<Placement>,
    ) -> Result<()> {
        if path.contains(&rep) {
            return cyclic(rep, "a mapped representation contains itself");
        }
        self.visit(rep)?;
        path.push(rep);
        let r = self.rep_solids_in(geo, rep, at, path, out);
        path.pop();
        r
    }

    fn rep_solids_in(
        &self,
        geo: &mut Geometry,
        rep: u64,
        at: &Placing,
        path: &mut Vec<u64>,
        out: &mut Vec<Placement>,
    ) -> Result<()> {
        for r in self.linked(rep) {
            let Some((items, context)) = representation(self.file, r) else {
                continue;
            };
            for it in items {
                let Some(e) = self.file.get(it) else { continue };
                if SOLIDS.contains(&part21::kind(e)) {
                    if out.len() >= MAX_PLACEMENTS {
                        return cyclic(it, "the assembly places too many solids");
                    }
                    out.push(Placement {
                        solid: it,
                        context,
                        at: *at,
                        product: None,
                        uses: Vec::new(),
                    });
                } else if let Some(p) = e.part("MAPPED_ITEM") {
                    let _ = (geo.use_context(context), p);
                    if let Some(t) = self.mapped(geo, p) {
                        let (src, place) = t;
                        self.rep_solids(geo, src, &place.then(at), path, out)?;
                    }
                }
            }
        }
        Ok(())
    }

    /// A `MAPPED_ITEM`'s source representation and its placement in the target.
    fn mapped(&self, geo: &mut Geometry, p: &[Param]) -> Option<(u64, Placing)> {
        let map = params(self.file, p.get(1)?.reference()?, "REPRESENTATION_MAP")?;
        let origin = geo.transform_of(map.first()?.reference()?).ok()?;
        let src = map.get(1)?.reference()?;
        let target = geo.transform_of(p.get(2)?.reference()?).ok()?;
        Some((src, origin.inverse().then(&target)))
    }

    /// The motion that places `child`'s shape in its parent's, from the placing relation.
    fn placement(&self, geo: &mut Geometry, child: u64, rel: u64) -> Option<Placing> {
        let e = self.file.get(rel)?;
        let rr = e.part("REPRESENTATION_RELATIONSHIP")?;
        let (r1, r2) = (rr.get(2)?.reference()?, rr.get(3)?.reference()?);
        let op = e
            .part("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION")?
            .first()?
            .reference()?;
        let child_reps: Vec<u64> = self
            .reps
            .get(&child)
            .into_iter()
            .flatten()
            .flat_map(|&r| self.linked(r))
            .collect();
        let child_is_r1 = child_reps.contains(&r1) || !child_reps.contains(&r2);
        let context = |r: u64| representation(self.file, r).and_then(|x| x.1);
        let oe = self.file.get(op)?;
        if let Some(it) = oe.part("ITEM_DEFINED_TRANSFORMATION") {
            geo.use_context(context(r1)).ok()?;
            let a = geo.transform_of(it.get(2)?.reference()?).ok()?;
            geo.use_context(context(r2)).ok()?;
            let b = geo.transform_of(it.get(3)?.reference()?).ok()?;
            let (from, to) = if child_is_r1 { (a, b) } else { (b, a) };
            Some(from.inverse().then(&to))
        } else {
            geo.use_context(context(if child_is_r1 { r2 } else { r1 }))
                .ok()?;
            let t = geo.transform_of(op).ok()?;
            Some(if child_is_r1 { t } else { t.inverse() })
        }
    }

    /// Places `pd` and its children at `at`. `path` holds the products above it, so an assembly
    /// that contains itself is an error.
    fn walk(
        &self,
        geo: &mut Geometry,
        pd: u64,
        at: &Placing,
        path: &mut Vec<u64>,
        seen: &mut HashSet<u64>,
        out: &mut Vec<Placement>,
    ) -> Result<()> {
        if path.contains(&pd) {
            return cyclic(pd, "the assembly contains itself");
        }
        self.visit(pd)?;
        seen.insert(pd);
        path.push(pd);
        let own = out.len();
        for &rep in self.reps.get(&pd).into_iter().flatten() {
            self.rep_solids(geo, rep, at, &mut Vec::new(), out)?;
        }
        for p in out.iter_mut().skip(own) {
            p.product = Some(pd);
        }
        for &(child, rel, used) in self.children.get(&pd).into_iter().flatten() {
            if let Some(t) = self.placement(geo, child, rel) {
                let below = out.len();
                self.walk(geo, child, &t.then(at), path, seen, out)?;
                for p in out.iter_mut().skip(below) {
                    p.uses.insert(0, used);
                }
            }
        }
        path.pop();
        Ok(())
    }

    fn placements(&self, geo: &mut Geometry) -> Result<Vec<Placement>> {
        let mut out = Vec::new();
        let mut roots: Vec<u64> = self
            .reps
            .keys()
            .chain(self.children.keys())
            .copied()
            .filter(|pd| !self.related.contains(pd))
            .collect();
        roots.sort_unstable();
        roots.dedup();
        let mut seen = HashSet::new();
        for pd in roots {
            self.walk(
                geo,
                pd,
                &Placing::IDENTITY,
                &mut Vec::new(),
                &mut seen,
                &mut out,
            )?;
        }
        // A product no root reaches sits on a cycle of assembly uses.
        let mut unreached: Vec<u64> = self
            .children
            .keys()
            .copied()
            .filter(|pd| !seen.contains(pd))
            .collect();
        unreached.sort_unstable();
        match unreached.first() {
            Some(&pd) => cyclic(pd, "the assembly contains itself"),
            None => Ok(out),
        }
    }
}

/// Reads a STEP file as one part: the union of its solids, as the pieces of a solid that touches
/// itself are written one by one and join again. A file with a solid this kernel rejects is an
/// error, since the part would lack it.
pub fn read_part(text: &[u8], feature: &str) -> Result<Brep> {
    let import = read(text, feature)?;
    if !import.rejected.is_empty() {
        return Err(Error::InvalidResult(import.rejected));
    }
    let b = &import.brep;
    let pieces: Vec<Brep> = b
        .solid_ids()
        .map(|s| crate::boolean::solids_of(b, &[s]))
        .collect();
    match &pieces[..] {
        [one] => Ok(one.clone()),
        _ => crate::boolean::union_all(&pieces.iter().collect::<Vec<_>>()),
    }
}

/// Reads a STEP file into kernel solids. Faces carry the names a file this kernel wrote
/// gave them; otherwise each is `<feature>.face_<i>`, numbered in the order the solids are placed.
#[allow(clippy::disallowed_types)] pub fn read(text: &[u8], feature: &str) -> Result<Import> { // D26: BTree* in hardening, proven by digests
    let file = part21::parse(text)?;
    let ours = part21::originating_system(&file) == Some(WRITER);
    let mut geo = Geometry::new(&file);
    let mut placements = Assembly::new(&file).placements(&mut geo)?;
    if placements.is_empty() {
        // Without a product structure, a solid takes its units and uncertainty from a
        // representation that holds it, if one does.
        let mut contexts: HashMap<u64, u64> = HashMap::new();
        for &id in &file.order {
            if let Some((items, Some(ctx))) = representation(&file, id) {
                for it in items {
                    contexts.entry(it).or_insert(ctx);
                }
            }
        }
        for (id, e) in part21::entities(&file) {
            if SOLIDS.contains(&part21::kind(e)) {
                placements.push(Placement {
                    solid: id,
                    context: contexts.get(&id).copied(),
                    at: Placing::IDENTITY,
                    product: None,
                    uses: Vec::new(),
                });
            }
        }
    }
    if placements.is_empty() {
        return Err(Error::InvalidInput("STEP: the file holds no solids".into()));
    }
    let mut keys: Vec<Key> = placements.iter().map(Placement::key).collect();
    keys.sort_unstable();
    keys.dedup();
    let built = build_all(&file, &keys);
    let mut out = Brep::new();
    let mut rejected = Vec::new();
    let mut healed = Vec::new();
    let mut tolerant = Vec::new();
    let mut crossed = Vec::new();
    let mut crossings: HashMap<Key, Vec<check::SelfCrossing>> = HashMap::new();
    let mut counter = 0usize;
    let mut index = 0usize;
    let colours = colours(&file);
    let mut bodies = Vec::new();
    let mut face_colours = Vec::new();
    for pl in placements {
        match built.get(&pl.key()) {
            None => rejected.push(format!("#{}: never built", pl.solid)),
            Some(Err(e)) => rejected.push(format!("#{}: {e}", pl.solid)),
            Some(Ok((b, names, heal, stated))) => {
                if let Some(h) = heal {
                    healed.push(Healing {
                        index,
                        step_id: pl.solid,
                        moved: h.moved,
                        at: pl.at.rigid.point(h.at),
                        what: h.what.clone(),
                        surfaces: h.surfaces,
                    });
                }
                if b.has_tolerances() {
                    let (vs, es) = b.tolerant();
                    let all = vs.iter().map(|x| x.1).chain(es.iter().map(|x| x.1));
                    let past = |t: &f64| stated.is_some_and(|s| *t > s);
                    tolerant.push(Tolerant {
                        index,
                        step_id: pl.solid,
                        vertices: vs.len(),
                        edges: es.len(),
                        largest: vs.iter().map(|x| x.1).fold(0.0, f64::max),
                        stated: *stated,
                        past_stated: all.filter(past).count(),
                    });
                }
                let first = out.face_count();
                let found = crossings
                    .entry(pl.key())
                    .or_insert_with(|| check::self_crossings(b))
                    .clone();
                let solid_index = index;
                index += 1;
                let mut copy = b.clone();
                copy.transform(&pl.at.rigid);
                let mut own_colours = Vec::new();
                let faces = copy.face_ids().collect::<Vec<_>>();
                for (k, (f, (step, name))) in faces.into_iter().zip(names).enumerate() {
                    if let Some(&c) = colours.get(step) {
                        own_colours.push((k, c));
                    }
                    let parsed = if ours { parse_names(name) } else { Vec::new() };
                    copy.face_mut(f).names = if parsed.is_empty() {
                        [FaceName::new(feature, format!("face_{counter}"))].into()
                    } else {
                        parsed.into_iter().collect()
                    };
                    counter += 1;
                }
                let solids = out.append(&copy);
                let ids: Vec<FaceId> = out.face_ids().skip(first).collect();
                face_colours.extend(own_colours.into_iter().filter_map(|(k, c)| Some((*ids.get(k)?, c))));
                bodies.extend(solids.into_iter().map(|solid| Body {
                    solid,
                    step_id: pl.solid,
                    name: pl.product.and_then(|pd| product_name(&file, pd)),
                    instance: pl.uses.iter().map(|&u| use_name(&file, u)).collect(),
                    colour: colours.get(&pl.solid).copied(),
                }));
                for x in found {
                    let (Some(&fa), Some(&fb)) = (ids.get(x.a.index()), ids.get(x.b.index())) else { continue };
                    crossed.push(Crossed {
                        index: solid_index,
                        step_id: pl.solid,
                        faces: (fa, fb),
                        at: pl.at.rigid.point(x.at),
                        depth: x.depth,
                    });
                }
            }
        }
    }
    Ok(Import {
        brep: out,
        rejected,
        healed,
        tolerant,
        crossed,
        bodies,
        face_colours,
    })
}

fn text(p: Option<&Param>) -> Option<String> {
    p.and_then(Param::string)
        .filter(|s| !s.is_empty())
        .map(str::to_string)
}

/// The name of the product a product definition defines, or its id where it has no name.
fn product_name(file: &File, pd: u64) -> Option<String> {
    let formation = params(file, pd, "PRODUCT_DEFINITION")?.get(2)?.reference()?;
    let formed = file.get(formation)?;
    let product = formed
        .records()
        .iter()
        .find(|r| r.name.starts_with("PRODUCT_DEFINITION_FORMATION"))?
        .params
        .get(2)?
        .reference()?;
    let p = params(file, product, "PRODUCT")?;
    text(p.get(1)).or_else(|| text(p.first()))
}

/// What an assembly use is called: its name, else its id, else its number in the file.
fn use_name(file: &File, used: u64) -> String {
    params(file, used, "NEXT_ASSEMBLY_USAGE_OCCURRENCE")
        .and_then(|p| text(p.get(1)).or_else(|| text(p.first())))
        .unwrap_or_else(|| format!("#{used}"))
}

fn all_refs(p: &Param, out: &mut Vec<u64>) {
    match p {
        Param::Ref(r) => out.push(*r),
        Param::List(l) => l.iter().for_each(|x| all_refs(x, out)),
        Param::Typed(_, inner) => all_refs(inner, out),
        _ => {}
    }
}

/// The first colour reached from entity `id` within `depth` references.
fn colour_under(file: &File, id: u64, depth: usize) -> Option<[f64; 3]> {
    let e = file.get(id)?;
    if let Some(p) = e.part("COLOUR_RGB") {
        let at = |i: usize| p.get(i).and_then(Param::real);
        return Some([at(1)?, at(2)?, at(3)?]);
    }
    if let Some(p) = e
        .part("DRAUGHTING_PRE_DEFINED_COLOUR")
        .or_else(|| e.part("PRE_DEFINED_COLOUR"))
    {
        return Some(match p.first().and_then(Param::string)? {
            "red" => [1.0, 0.0, 0.0],
            "green" => [0.0, 1.0, 0.0],
            "blue" => [0.0, 0.0, 1.0],
            "yellow" => [1.0, 1.0, 0.0],
            "magenta" => [1.0, 0.0, 1.0],
            "cyan" => [0.0, 1.0, 1.0],
            "black" => [0.0, 0.0, 0.0],
            "white" => [1.0, 1.0, 1.0],
            _ => return None,
        });
    }
    if depth == 0 {
        return None;
    }
    let mut below = Vec::new();
    for r in e.records() {
        r.params.iter().for_each(|p| all_refs(p, &mut below));
    }
    below
        .into_iter()
        .find_map(|r| colour_under(file, r, depth - 1))
}

/// The surface colour the file's presentation styles give each solid or face they name, by its
/// id. A curve's or a point's colour is not a body's.
#[allow(clippy::disallowed_types)] fn colours(file: &File) -> HashMap<u64, [f64; 3]> { // D26: BTree* in hardening, proven by digests
    // Usage, side style, fill area, fill style, fill colour, colour.
    const DEPTH: usize = 6;
    let mut out = HashMap::new();
    for (_, e) in part21::entities(file) {
        let Some(p) = e
            .part("STYLED_ITEM")
            .or_else(|| e.part("OVER_RIDING_STYLED_ITEM"))
        else {
            continue;
        };
        let Some(item) = p.get(2).and_then(Param::reference) else {
            continue;
        };
        let mut styles = Vec::new();
        for assignment in refs(p.get(1)) {
            if let Some(a) = file.get(assignment) {
                for r in a.records() {
                    r.params.iter().for_each(|x| all_refs(x, &mut styles));
                }
            }
        }
        let colour = styles
            .into_iter()
            .filter(|&s| file.get(s).is_some_and(|x| x.is("SURFACE_STYLE_USAGE")))
            .find_map(|s| colour_under(file, s, DEPTH));
        if let Some(c) = colour {
            out.entry(item).or_insert(c);
        }
    }
    out
}

/// A solid, its faces' ids and names, how far healing moved it, and the uncertainty its file
/// states.
type Built = std::result::Result<(Brep, Vec<(u64, String)>, Option<Healed>, Option<f64>), String>;

/// A rough measure of how long a solid takes to build: its faces, and the control points of its
/// spline surfaces, which healing fits.
fn weight(file: &File, solid: u64) -> usize {
    let refs_of = |id: u64, at: usize| -> Vec<u64> {
        let Some(e) = file.get(id) else {
            return Vec::new();
        };
        let Some(p) = e.records().first().map(|r| &r.params) else {
            return Vec::new();
        };
        match p.get(at) {
            Some(Param::List(l)) => l.iter().filter_map(Param::reference).collect(),
            Some(x) => x.reference().into_iter().collect(),
            None => Vec::new(),
        }
    };
    let mut shells = refs_of(solid, 1);
    shells.extend(refs_of(solid, 2));
    let mut total = 0;
    for sh in shells {
        let sh = match file.get(sh).and_then(|e| e.part("ORIENTED_CLOSED_SHELL")) {
            Some(p) => p.get(2).and_then(Param::reference).unwrap_or(sh),
            None => sh,
        };
        for f in refs_of(sh, 1) {
            total += 1;
            let Some(face) = file.get(f) else { continue };
            let Some(s) = face.records().first().and_then(|r| r.params.get(2)).and_then(Param::reference) else {
                continue;
            };
            let Some(e) = file.get(s) else { continue };
            // A complex instance's B_SPLINE_SURFACE part has no name before its degrees.
            let net = match (
                e.part("B_SPLINE_SURFACE"),
                e.part("B_SPLINE_SURFACE_WITH_KNOTS"),
            ) {
                (Some(p), _) => p.get(2),
                (None, Some(p)) => p.get(3),
                _ => None,
            };
            total += net.and_then(Param::list).map_or(0, |rows| {
                rows.iter().filter_map(Param::list).map(<[_]>::len).sum()
            });
        }
    }
    total
}

/// Builds each distinct solid once. Solids share nothing but the file, so they are built on as
/// many threads as the machine offers, each with its own geometry cache, heaviest first so that
/// no long build starts last. Each result is filed under its own key, whatever finished first.
#[allow(clippy::disallowed_types)] fn build_all(file: &File, keys: &[Key]) -> HashMap<Key, Built> { // D26: BTree* in hardening, proven by digests
    let threads = std::thread::available_parallelism()
        .map_or(1, |n| n.get())
        .min(keys.len().max(1));
    let mut order: Vec<(usize, usize)> = keys
        .iter()
        .enumerate()
        .map(|(k, key)| (weight(file, key.0), k))
        .collect();
    order.sort_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(&b.1)));
    let next = AtomicUsize::new(0);
    let results: Vec<Vec<(usize, Built)>> = std::thread::scope(|scope| {
        let workers: Vec<_> = (0..threads)
            .map(|_| {
                scope.spawn(|| {
                    let mut geo = Geometry::new(file);
                    let mut mine = Vec::new();
                    while let Some(&(_, k)) = order.get(next.fetch_add(1, Ordering::Relaxed)) {
                        let Some(&(solid, context, scale)) = keys.get(k) else { continue };
                        let r = geo
                            .use_scaled(context, f64::from_bits(scale))
                            .and_then(|_| topology::solid(&mut geo, solid))
                            .map(|(b, names, healed)| (b, names, healed, geo.uncertainty()))
                            .map_err(|e| e.to_string());
                        mine.push((k, r));
                    }
                    mine
                })
            })
            .collect();
        workers
            .into_iter()
            .map(|w| w.join().unwrap_or_default())
            .collect()
    });
    let mut out = HashMap::new();
    for (k, r) in results.into_iter().flatten() {
        if let Some(&key) = keys.get(k) {
            out.insert(key, r);
        }
    }
    for &k in keys {
        out.entry(k)
            .or_insert_with(|| Err("the thread building it failed".to_string()));
    }
    out
}

/// The names a face this kernel wrote carries: `feature.face` entries joined by `;`.
fn parse_names(s: &str) -> Vec<FaceName> {
    s.split(';')
        .filter_map(|n| {
            let (f, face) = n.split_once('.')?;
            Some(FaceName::new(f, face))
        })
        .collect()
}
