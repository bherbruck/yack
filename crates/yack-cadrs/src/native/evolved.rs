// yack: Evolved solids (OCCT's `BRepOffsetAPI_MakeEvolved` in volume mode, DRAW's `evolved -v....
// Adapted from cadrs@eb5899ef crates/cadrs_kernel/src/native/evolved.rs (MIT OR Apache-2.0, (c) 2026 Rouan van der Ende).
// Changes: none (verbatim).
//! Evolved solids (OCCT's `BRepOffsetAPI_MakeEvolved` in volume mode, DRAW's `evolved -v`): a
//! profile region swept along a planar spine, held square to it, with its x measured outwards
//! from the spine in the spine's plane and its y up from it.
//!
//! Each spine edge sweeps the profile exactly: a line a prism of it, a circular arc a revolution
//! about the arc's axis. At a corner where the profile's side of the spine turns away (a left
//! turn of a counter-clockwise spine for a profile outside it, x ≥ 0; a right turn for one
//! inside, x ≤ 0) the gap is filled by the profile turned about the corner's vertical axis, so
//! offsets of the spine round off as OCCT's do (arc joins); where it turns in, the neighbouring
//! sweeps overlap. The pieces are united. A closed spine is taken counter-clockwise about its
//! plane's normal, which is "up".
//!
//! [`evolved_wire`] takes the profile as DRAW's `evolved` does: a wire in space (open or
//! closed), measured from the spine point nearest it (`BRepFill_PipeShell`'s placement in the
//! volume mode; the plain mode's global frame is the spine plane's own axes, which agree with
//! it on OCCT's cases), its part along the spine dropped. An open profile is
//! closed by flat lids across the spine's offsets at its ends (`BRepFill_AdvancedEvolved::GetLids`),
//! so the solid holds the points whose signed distance inwards from the spine, at their height,
//! lies in the profile's region. The union of sweeps above is that set only where the region
//! reaches the spine at every height (a sweep's slab may cover a point nearer another edge), so
//! the region is cut into horizontal strips and each strip's spans are built from sweeps of
//! spans that do: a span `[a, b]` away from the spine is the sweep of `[a, 0]` less the sweep
//! of `[b, 0]`, a span reaching past every inward offset is the spine's prism less the sweep of
//! the rest, and sweeps inwards are kept inside the spine's prism (outwards, outside it).

use super::curve::Curve;
use super::features::Tagged;
use super::kernel::NativeKernel;
use super::math::*;
use super::tol;
use super::topo::Body;
use crate::*;

/// A spine edge in the direction of travel.
struct SpineEdge {
    c: Curve,
    t0: f64,
    t1: f64,
}

impl SpineEdge {
    fn point(&self, s: f64) -> P3 {
        self.c.point(self.t0 + (self.t1 - self.t0) * s)
    }
    fn dir(&self, s: f64) -> V3 {
        (self.c.tangent(self.t0 + (self.t1 - self.t0) * s) * (self.t1 - self.t0).signum())
            .normalize()
    }
}

/// A planar spine of lines and circular arcs, ready to sweep along.
struct Spine {
    edges: Vec<SpineEdge>,
    /// The plane's normal: the spine runs counter-clockwise about it when closed.
    up: V3,
    closed: bool,
}

impl Spine {
    fn new(spine: &[(Curve, f64, f64)]) -> Result<Spine> {
        if spine.is_empty() {
            return Err(KernelError::InvalidParameter(
                "an evolved solid needs a spine".into(),
            ));
        }
        let edges: Vec<SpineEdge> = spine
            .iter()
            .map(|(c, t0, t1)| SpineEdge {
                c: c.clone(),
                t0: *t0,
                t1: *t1,
            })
            .collect();
        for e in &edges {
            if !matches!(e.c, Curve::Line { .. } | Curve::Circle { .. }) {
                return Err(KernelError::Unsupported(
                    "evolved solids along spines of curves other than lines and arcs",
                ));
            }
        }
        let n = edges.len();
        let first = edges[0].point(0.0);
        let closed =
            (edges[n - 1].point(1.0) - first).norm() < tol::CONFUSION * (1.0 + first.coords.norm());
        // The spine's plane normal ("up"): counter-clockwise for a closed spine.
        let pts: Vec<P3> = edges
            .iter()
            .flat_map(|e| (0..16).map(move |i| e.point(i as f64 / 16.0)))
            .collect();
        let mut up = V3::zeros();
        for i in 0..pts.len() {
            let (a, b) = (pts[i], pts[(i + 1) % pts.len()]);
            up += (a - pts[0]).cross(&(b - pts[0]));
        }
        let size = pts
            .iter()
            .map(|p| (p - pts[0]).norm())
            .fold(0.0, f64::max)
            .max(tol::ROUNDOFF);
        if up.norm() < tol::REL * size * size {
            // An open straight spine: any normal square to it.
            up = super::math::ax2_x(&edges[0].dir(0.0));
        }
        let up = up.normalize();
        if pts
            .iter()
            .any(|p| (p - pts[0]).dot(&up).abs() > tol::POINT * size)
        {
            return Err(KernelError::InvalidParameter(
                "an evolved solid's spine must be planar".into(),
            ));
        }
        Ok(Spine {
            edges,
            up,
            closed,
        })
    }

    /// The profile placed at a point of the spine with travel direction t: x outwards (t × up).
    fn place(&self, p: P3, t: V3, regions: Vec<Region>) -> Profile {
        let x = t.cross(&self.up).normalize();
        Profile::new(
            Plane {
                origin: p,
                x_dir: nalgebra::Unit::new_normalize(x),
                normal: nalgebra::Unit::new_normalize(x.cross(&self.up)),
            },
            regions,
        )
    }

    /// Where an arc edge's axis lies in the profile's x at the edge's start: `-r` when the arc
    /// bends inwards (its centre on the inner side), `r` when outwards; `None` for a line.
    fn axis_x(&self, e: &SpineEdge) -> Option<f64> {
        let Curve::Circle { c, .. } = &e.c else {
            return None;
        };
        Some((c - e.point(0.0)).dot(&e.dir(0.0).cross(&self.up)))
    }




}

/// The evolved solid of `profile` (closed regions in the profile's (x, y): outwards, up) along
/// `spine` (lines and circular arcs, chained in order, in one plane). The profile must lie on
/// one side of x = 0. Returns the solid's body.
pub fn evolved(
    k: &mut NativeKernel,
    spine: &[(Curve, f64, f64)],
    profile: &[Region],
) -> Result<BodyId> {
    if spine.is_empty() || profile.is_empty() {
        return Err(KernelError::InvalidParameter(
            "an evolved solid needs a spine and a profile".into(),
        ));
    }
    let sp = Spine::new(spine)?;
    // Which side of the spine the profile lies on.
    let mut xs = (f64::INFINITY, f64::NEG_INFINITY);
    for r in profile {
        for c in r
            .outer
            .curves
            .iter()
            .chain(r.holes.iter().flat_map(|h| h.curves.iter()))
        {
            for i in 0..=8 {
                let p = c.point_at(i as f64 / 8.0);
                xs = (xs.0.min(p.x), xs.1.max(p.x));
            }
        }
    }
    let outside = if xs.0 >= -tol::ROUNDOFF {
        true
    } else if xs.1 <= tol::ROUNDOFF {
        false
    } else {
        return Err(KernelError::Unsupported(
            "evolved solids of profiles on both sides of the spine",
        ));
    };
    sweep_union(
        k,
        &sp,
        &|_| Ok(profile.to_vec()),
        &|o| {
            Ok(if o == outside {
                profile.to_vec()
            } else {
                Vec::new()
            })
        },
        false,
    )?
    .ok_or_else(|| {
        KernelError::InvalidParameter("an evolved solid needs a spine and a profile".into())
    })
}

/// The union of the sweeps along each spine edge, and of the knuckles turned about the corners.
/// `regions(axis_x)` gives the profile for an edge (`axis_x` from [`Spine::axis_x`], so an
/// arc's profile can stop at its axis; `None` for lines), `knuckle(outside)` the part of it
/// turned about a corner where the spine turns away from that side. Pieces meeting at a shared
/// cap are glued there when `glue`, else (and where they overlap) united. `None` when every
/// profile is empty.
fn sweep_union(
    k: &mut NativeKernel,
    sp: &Spine,
    regions: &dyn Fn(Option<f64>) -> Result<Vec<Region>>,
    knuckle: &dyn Fn(bool) -> Result<Vec<Region>>,
    glue: bool,
) -> Result<Option<BodyId>> {
    let n = sp.edges.len();
    // The corners: the turn from each edge into the next (none past an open spine's end).
    let turns: Vec<Option<f64>> = (0..n)
        .map(|i| {
            if i + 1 == n && !sp.closed {
                return None;
            }
            let (ta, tb) = (sp.edges[i].dir(1.0), sp.edges[(i + 1) % n].dir(0.0));
            Some(ta.cross(&tb).dot(&sp.up).atan2(ta.dot(&tb)))
        })
        .collect();
    // The knuckle's profile at each corner (the part of the profile on the side the spine
    // turns away from: outside at a left turn).
    let mut knuckles: Vec<Vec<Region>> = Vec::with_capacity(n);
    for t in &turns {
        knuckles.push(match t {
            Some(t) if t.abs() >= tol::ANGULAR => knuckle(*t > 0.0)?,
            _ => Vec::new(),
        });
    }
    // Where part of the profile turns in, the sweeps on either side overlap: starting after
    // such a corner, every other joint is a shared cap.
    let whole = regions(None)?;
    let first = if sp.closed {
        (0..n)
            .position(|i| {
                turns[i].is_some_and(|t| t.abs() >= tol::ANGULAR)
                    && knuckles[i].len() != whole.len()
            })
            .map_or(0, |i| (i + 1) % n)
    } else {
        0
    };
    let mut pieces: Vec<Tagged> = Vec::new();
    for i in (0..n).map(|j| (first + j) % n) {
        let e = &sp.edges[i];
        let p0 = e.point(0.0);
        let rs = regions(sp.axis_x(e))?;
        if !rs.is_empty() {
            let prof = sp.place(p0, e.dir(0.0), rs);
            let r = match &e.c {
                Curve::Line { .. } => super::features::prism_along(k, &prof, e.point(1.0) - p0)?,
                Curve::Circle { c, x, y, .. } => {
                    // About the arc's axis, the way it runs.
                    let ax = x.cross(y) * (e.t1 - e.t0).signum();
                    let sweep = (e.t1 - e.t0).abs();
                    crate::Kernel::revolve(
                        k,
                        &prof,
                        Axis {
                            origin: *c,
                            dir: nalgebra::Unit::new_normalize(ax),
                        },
                        sweep,
                    )?
                }
                _ => {
                    return Err(KernelError::Unsupported(
                        "evolved solids along spines of curves other than lines and arcs",
                    ));
                }
            };
            pieces.push(tagged(k, r)?);
        }
        // The corner at the edge's end: the knuckle turned about it.
        let Some(turn) = turns[i] else { continue };
        let rs = knuckles[i].clone();
        if rs.is_empty() {
            continue;
        }
        let v = e.point(1.0);
        let prof = sp.place(v, e.dir(1.0), rs);
        let axis = Axis {
            origin: v,
            dir: nalgebra::Unit::new_normalize(sp.up * turn.signum()),
        };
        let r = crate::Kernel::revolve(k, &prof, axis, turn.abs())?;
        pieces.push(tagged(k, r)?);
    }
    // Pieces meeting at a shared cap are glued there; overlapping ones united.
    let mut it = pieces.into_iter();
    let Some(mut acc) = it.next() else {
        return Ok(None);
    };
    let mut prev_end = caps(&acc, true);
    for p in it {
        let (start, end) = (caps(&p, false), caps(&p, true));
        // Glued only where the caps meet one to one (else part of the profile turns in there).
        let meet = start.len() == prev_end.len()
            && start
                .iter()
                .all(|a| prev_end.iter().any(|b| same_cap(a, b)));
        acc = match (glue && meet)
            .then(|| super::sweep::path::glue(&acc, &p))
            .flatten()
            .filter(|g| additive(&g.body, &acc.body, &p.body))
        {
            Some(g) => g,
            None => super::features::tagged_union(&acc, &p)?,
        };
        prev_end = end;
    }
    if glue
        && sp.closed
        && let Some(g) = super::sweep::path::glue_self(&acc)
    {
        acc = g;
    }
    Ok(Some(k.insert(acc.body)))
}

/// A piece's start or end caps: their areas and centroids.
fn caps(t: &Tagged, end: bool) -> Vec<(f64, P3)> {
    (0..t.body.faces.len())
        .filter(|&f| {
            t.tags[f].iter().any(|o| {
                if end {
                    matches!(o, Origin::EndCap { .. })
                } else {
                    matches!(o, Origin::StartCap { .. })
                }
            })
        })
        .map(|f| super::mass::face_area(&t.body, f))
        .collect()
}

/// Whether two caps coincide (as the sweep's gluing takes them).
fn same_cap(a: &(f64, P3), b: &(f64, P3)) -> bool {
    (a.1 - b.1).norm() < tol::POINT && (a.0 - b.0).abs() < tol::POINT * (1.0 + a.0)
}

/// A swept piece with its faces' origins (its caps among them), out of the kernel.
fn tagged(k: &mut NativeKernel, r: OpResult) -> Result<Tagged> {
    let id = r.bodies[0];
    let body = Body::clone(k.get(id)?);
    let mut tags = vec![Vec::new(); body.faces.len()];
    for (f, o) in r.history.generated {
        if let Some(t) = tags.get_mut(f.0 as usize) {
            t.push(o);
        }
    }
    crate::Kernel::release(k, id);
    Ok(Tagged { body, tags })
}



/// Whether `g`, `a` and `b` glued, holds their volumes together (they only touched: a glue
/// across an overlap would count it twice).
fn additive(g: &Body, a: &Body, b: &Body) -> bool {
    let (va, vb) = (super::mass::body_volume(a), super::mass::body_volume(b));
    (super::mass::body_volume(g) - va - vb).abs() <= tol::POINT * (va.abs() + vb.abs())
}



// --- Wire profiles (DRAW's `evolved`) ---------------------------------------------------------

















