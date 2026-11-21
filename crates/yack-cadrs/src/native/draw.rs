// yack: A DRAW-style API on the native kernel: OCCT's test commands (`line`, `mkedge`, `wire`,.
// Adapted from cadrs@eb5899ef crates/cadrs_kernel/src/native/draw.rs (MIT OR Apache-2.0, (c) 2026 Rouan van der Ende).
// Changes: none (verbatim).
//! A DRAW-style API on the native kernel: OCCT's test commands (`line`, `mkedge`, `wire`,
//! `box`, `pcylinder`, `bfuse`, `blend`, `fillet2d`, `checkprops`, …) as methods of
//! [`NativeKernel`], so test scripts in DRAW's terms read almost line for line as Rust tests.
//!
//! Arguments follow DRAW's order (points and directions as separate numbers, angles in degrees
//! where DRAW takes degrees). Shapes are [`Shape`] values: vertices, edges, wires and planar
//! faces are held here; solids and sheets are bodies of the kernel.
#![allow(
    clippy::too_many_arguments,
    reason = "the methods take the DRAW commands' arguments as they are"
)]

use std::f64::consts::{PI, TAU};
use crate::KernelError;

use super::curve::Curve;
use super::kernel::NativeKernel;
use super::math::*;
use super::profile::{MergedFace, Piece, Src};
use super::surface::Surface;
use super::sweep::path::Seg;
use crate::*;

/// An edge: a curve between two parameters.
#[derive(Debug, Clone)]
pub struct DEdge {
    pub curve: Curve,
    pub t0: f64,
    pub t1: f64,
}

impl DEdge {
    pub fn start(&self) -> P3 {
        self.curve.point(self.t0)
    }
    pub fn end(&self) -> P3 {
        self.curve.point(self.t1)
    }
    pub fn length(&self) -> f64 {
        self.curve
            .length(self.t0.min(self.t1), self.t0.max(self.t1))
    }
    fn reversed(&self) -> DEdge {
        let (curve, t0, t1) = self.curve.reversed(self.t0, self.t1);
        DEdge { curve, t0, t1 }
    }
}

/// A DRAW shape.
#[derive(Debug, Clone)]
pub enum Shape {
    Vertex(P3),
    Edge(DEdge),
    Wire(Vec<DEdge>),
    /// A planar face: its plane and loops (outer first).
    Face {
        plane: Plane,
        loops: Vec<Vec<DEdge>>,
    },
    /// A solid or sheet of the kernel.
    Body(BodyId),
    /// An edge of a body (from `explode`).
    BodyEdge(BodyId, EdgeId),
    /// A face of a body (from `explode`).
    BodyFace(BodyId, FaceId),
    /// Several shapes (DRAW's `compound`, a result of several wires).
    Compound(Vec<Shape>),
    /// Nothing (an empty boolean result).
    Empty,
}

/// What an operation made of its input (DRAW's `savehistory`): the input and result bodies and
/// the history.
#[derive(Debug, Clone)]
pub struct DrawHistory {
    input: BodyId,
    output: BodyId,
    history: crate::History,
}

/// A placement (DRAW's `plane`): origin, normal and x direction.
#[derive(Debug, Clone, Copy)]
pub struct Ax3 {
    pub origin: P3,
    pub z: V3,
    pub x: V3,
}

impl Ax3 {
    pub fn plane(&self) -> Plane {
        Plane {
            origin: self.origin,
            x_dir: nalgebra::Unit::new_normalize(self.x),
            normal: nalgebra::Unit::new_normalize(self.z),
        }
    }
    pub fn y(&self) -> V3 {
        self.z.cross(&self.x)
    }
}

/// A hyperbola (DRAW's `hyperbola`): C + a·cosh t·X + b·sinh t·Y.
#[derive(Debug, Clone, Copy)]
pub struct Hyperbola {
    pub c: P3,
    pub x: V3,
    pub y: V3,
    pub a: f64,
    pub b: f64,
}



/// OCCT's order of a primitive's sub-shapes (for `explode`): reference points (edge midpoints,
/// face centres) in OCCT's order, carried along through transforms.
#[derive(Debug, Clone, Default)]
struct OcctOrder {
    edges: Vec<P3>,
    faces: Vec<P3>,
    /// The edges (indices into `edges`) OCCT makes degenerate (a vertex on a revolution's
    /// axis): matched where the body has such edges too, else left out.
    degenerate: Vec<usize>,
}

thread_local! {
    static ORDERS: std::cell::RefCell<std::collections::HashMap<BodyId, OcctOrder>> = Default::default();
}

thread_local! {
    /// The cells builder's materials of result cells (`bcadd -m`).
    static MATERIALS: std::cell::RefCell<std::collections::HashMap<BodyId, i64>> = Default::default();
}

fn set_order(b: BodyId, o: OcctOrder) {
    ORDERS.with(|m| m.borrow_mut().insert(b, o));
}

fn order_of(b: BodyId) -> Option<OcctOrder> {
    ORDERS.with(|m| m.borrow().get(&b).cloned())
}

/// OCCT's order for a full cylinder or truncated cone (BRepPrim_OneAxis): faces lateral, top,
/// bottom; edges top circle, seam, bottom circle (the seam at the placement's x).
fn revolved_order(ax: &Ax3, r1: f64, r2: f64, h: f64) -> OcctOrder {
    let w = |x: f64, y: f64, z: f64| ax.origin + ax.x * x + ax.y() * y + ax.z * z;
    let (lo, hi) = (0.0f64.min(h), 0.0f64.max(h));
    let (rlo, rhi) = if h >= 0.0 { (r1, r2) } else { (r2, r1) };
    OcctOrder {
        edges: vec![
            w(-rhi, 0.0, hi),
            w((r1 + r2) / 2.0, 0.0, (lo + hi) / 2.0),
            w(-rlo, 0.0, lo),
        ],
        faces: vec![
            w(0.0, (r1 + r2) / 2.0, (lo + hi) / 2.0),
            w(0.0, 0.0, hi),
            w(0.0, 0.0, lo),
        ],
        degenerate: vec![],
    }
}

/// OCCT's order for any primitive turned about an axis (BRepPrim_OneAxis), from its meridian
/// in (radius, height): the ends `bot` (VMin) and `top` (VMax), a point `mid` half way along it,
/// whether it is closed (a torus's circle), and the angle turned. The shell's faces are the
/// lateral one, the top and bottom (planes square to the axis, where the meridian ends off it)
/// and the start and end (half planes, when the angle is short of a turn); the edges as the
/// faces' wires meet them: the lateral wire top, end, bottom, start; the top wire top, start-top,
/// end-top; the bottom wire bottom, end-bottom, start-bottom; the start wire start-bottom, axis,
/// start-top, start. Degenerate edges (a meridian ending on the axis) count, as in OCCT.
fn one_axis_order(
    ax: &Ax3,
    bot: (f64, f64),
    top: (f64, f64),
    mid: (f64, f64),
    closed: bool,
    angle: f64,
) -> OcctOrder {
    let w =
        |r: f64, th: f64, z: f64| ax.origin + (ax.x * th.cos() + ax.y() * th.sin()) * r + ax.z * z;
    let sides = TAU - angle > 1e-9;
    let on_axis = |r: f64| r.abs() < 1e-9;
    let has_top = !closed && !on_axis(top.0);
    let has_bottom = !closed && !on_axis(bot.0);
    let half = if sides { angle / 2.0 } else { PI };
    let mut edges: Vec<P3> = Vec::new();
    let mut add = |p: P3| {
        if !edges
            .iter()
            .any(|q| (q - p).norm() < 1e-9 * (1.0 + p.coords.norm()))
        {
            edges.push(p);
        }
    };
    let e_top = w(top.0, half, top.1);
    let e_bot = w(bot.0, half, bot.1);
    let e_start = w(mid.0, 0.0, mid.1);
    let e_end = if sides {
        w(mid.0, angle, mid.1)
    } else {
        e_start
    };
    // Lateral.
    add(e_top);
    add(e_end);
    add(if closed { e_top } else { e_bot });
    add(e_start);
    if has_top && sides {
        add(w(top.0 / 2.0, 0.0, top.1));
        add(w(top.0 / 2.0, angle, top.1));
    }
    if has_bottom && sides {
        add(w(bot.0 / 2.0, angle, bot.1));
        add(w(bot.0 / 2.0, 0.0, bot.1));
    }
    if sides && !closed {
        add(w(0.0, 0.0, (bot.1 + top.1) / 2.0));
    }
    let mut faces = vec![w(mid.0, half, mid.1)];
    if has_top {
        faces.push(w(top.0 / 2.0, half, top.1));
    }
    if has_bottom {
        faces.push(w(bot.0 / 2.0, half, bot.1));
    }
    if sides {
        let (r, z) = (
            (mid.0 + if closed { mid.0 } else { 0.0 }) / 2.0,
            (bot.1 + top.1 + 2.0 * mid.1) / 4.0,
        );
        faces.push(w(r, 0.0, z));
        faces.push(w(r, angle, z));
    }
    OcctOrder {
        edges,
        faces,
        degenerate: vec![],
    }
}

/// OCCT's order for a face swept along a path (BRepSweep): the shell holds each wire edge's
/// lateral face in wire order, then the face at the path's start and at its end; a lateral
/// face's wire is the edge swept from the edge's first vertex, from its last (in the edge's
/// own direction), then the edge at the start and at the end. `at(p, s)` moves a point a
/// fraction s along the sweep; `swept(p)` is false where a vertex doesn't move (on a
/// revolution's axis), so it sweeps no edge, and an edge of such points no face. With
/// `closed` (a whole turn) the start and end are the same, with no end faces.
fn sweep_order(
    loops: &[Vec<DEdge>],
    at: &dyn Fn(&P3, f64) -> P3,
    swept: &dyn Fn(&P3) -> bool,
    closed: bool,
) -> OcctOrder {
    let mut edges: Vec<P3> = Vec::new();
    let add = |edges: &mut Vec<P3>, p: P3| {
        if !edges
            .iter()
            .any(|q| (q - p).norm() < 1e-9 * (1.0 + p.coords.norm()))
        {
            edges.push(p);
        }
    };
    let mut faces: Vec<P3> = Vec::new();
    let mut degenerate: Vec<usize> = Vec::new();
    let mid = |e: &DEdge| e.curve.point(0.5 * (e.t0 + e.t1));
    for l in loops {
        for e in l {
            let (a, b) = if e.t0 <= e.t1 {
                (e.start(), e.end())
            } else {
                (e.end(), e.start())
            };
            let m = mid(e);
            if !swept(&a) && !swept(&b) && !swept(&m) {
                continue;
            }
            faces.push(at(&m, 0.5));
            for v in [a, b] {
                if swept(&v) {
                    add(&mut edges, at(&v, 0.5));
                } else if !edges
                    .iter()
                    .any(|q| (q - v).norm() < 1e-9 * (1.0 + v.coords.norm()))
                {
                    // A vertex on the axis: OCCT's degenerate edge there (BRepSweep_Rotation).
                    degenerate.push(edges.len());
                    edges.push(v);
                }
            }
            add(&mut edges, m);
            add(&mut edges, at(&m, 1.0));
        }
    }
    if !closed {
        let pts: Vec<P3> = loops[0].iter().map(mid).collect();
        let c = P3::from(
            pts.iter().fold(V3::zeros(), |acc, p| acc + p.coords) / pts.len().max(1) as f64,
        );
        for s in [0.0, 1.0] {
            for l in loops {
                for e in l {
                    add(&mut edges, at(&mid(e), s));
                }
            }
            faces.push(at(&c, s));
        }
    }
    OcctOrder {
        edges,
        faces,
        degenerate,
    }
}

fn err(m: &str) -> KernelError {
    KernelError::OperationFailed(m.to_string())
}

/// A DRAW placement (`gp_Ax2`): origin, main direction (+z by default) and x direction
/// (projected square to the main one; OCCT's choice when not given). Returns origin, x and y.
fn ax2_of(o: [f64; 3], n: Option<[f64; 3]>, x: Option<[f64; 3]>) -> (P3, V3, V3) {
    let n = V3::from(n.unwrap_or([0.0, 0.0, 1.0])).normalize();
    let xd = match x {
        Some(x) => {
            let x = V3::from(x);
            (x - n * x.dot(&n)).normalize()
        }
        None => ax2_x(&n),
    };
    (P3::from(o), xd, n.cross(&xd))
}

/// OCCT's x direction for a placement with only its main direction (`gp_Ax2(P, V)`).
pub fn ax2_x(v: &V3) -> V3 {
    super::math::ax2_x(v)
}


/// The marker of [`sub`]'s error: the shape exploded has other topology than OCCT's.
pub const TOPOLOGY_MISMATCH: &str = "topology mismatch";

/// Sub-shape `n` (1-based, as DRAW names it: `s_7`) of the `subs` that exploding the shape
/// `of` into `kind` gave. When there are fewer, the shape's topology differs from OCCT's (an
/// edge or face split or merged differently before this command): a failure that says so,
/// not an index out of bounds.
pub fn sub<'a>(subs: &'a [Shape], n: usize, of: &str, kind: &str) -> Result<&'a Shape> {
    n.checked_sub(1).and_then(|i| subs.get(i)).ok_or_else(|| {
        err(&format!("{TOPOLOGY_MISMATCH}: explode {of} gave {} {kind}, the script expects at least {n} ({of}_{n})", subs.len()))
    })
}

/// The marker of [`known_failure`]'s panic for a known failure that panicked.
pub const KNOWN_FAILURE_PANICKED: &str = "a known failure panicked";

/// Ends a generated known failure (`tools/smlib_known_failures/`): `r` is the case's run
/// under `catch_unwind`. A known failure must still fail, and fail by returning its error:
/// this panics (failing the test) when the case passes ("a known failure now passes", the
/// gate's STALE) or when it panicked, unless the panic's message contains `expected_panic`
/// (the list's `[panic: …]`; the gate's PANIC otherwise, a bug the known failure would hide).
pub fn known_failure(case: &str, r: std::thread::Result<Result<()>>, expected_panic: Option<&str>) {
    match r {
        Ok(Ok(())) => {
            panic!("a known failure now passes: remove `{case}` from its known-failure list")
        }
        Ok(Err(e)) => eprintln!("known failure {case}: {e:?}"),
        Err(payload) => {
            let msg = payload
                .downcast_ref::<&str>()
                .map(|s| s.to_string())
                .or_else(|| payload.downcast_ref::<String>().cloned())
                .unwrap_or_default();
            match expected_panic {
                Some(want) if msg.contains(want) => {
                    eprintln!("known failure {case}: panicked as expected: {msg}")
                }
                _ => panic!("{KNOWN_FAILURE_PANICKED}: {case}: {msg}"),
            }
        }
    }
}

impl NativeKernel {
    // --- Geometry ---------------------------------------------------------------------------

    /// `line name x y z dx dy dz`.
    pub fn line(&self, x: f64, y: f64, z: f64, dx: f64, dy: f64, dz: f64) -> Curve {
        Curve::Line {
            o: P3::new(x, y, z),
            d: V3::new(dx, dy, dz).normalize(),
        }
    }

    /// `circle name x y z r` (normal +z).
    pub fn circle(&self, x: f64, y: f64, z: f64, r: f64) -> Curve {
        self.circle_n(x, y, z, 0.0, 0.0, 1.0, r)
    }

    /// `circle name x y z nx ny nz r`.
    pub fn circle_n(&self, x: f64, y: f64, z: f64, nx: f64, ny: f64, nz: f64, r: f64) -> Curve {
        let n = V3::new(nx, ny, nz).normalize();
        let xd = ax2_x(&n);
        Curve::Circle {
            c: P3::new(x, y, z),
            x: xd,
            y: n.cross(&xd),
            r,
        }
    }


    /// `plane name x y z [nx ny nz [xx xy xz]]`.
    pub fn plane(&self, x: f64, y: f64, z: f64, nx: f64, ny: f64, nz: f64) -> Ax3 {
        let n = V3::new(nx, ny, nz).normalize();
        Ax3 {
            origin: P3::new(x, y, z),
            z: n,
            x: ax2_x(&n),
        }
    }


    /// `hyperbola name x y z [nx ny nz [xx xy xz]] major minor` (OCCT's: C + major·cosh t·X +
    /// minor·sinh t·Y; an edge of it needs its parameters, see [`NativeKernel::mkedge_hyperbola`]).
    pub fn hyperbola(
        &self,
        o: [f64; 3],
        n: Option<[f64; 3]>,
        x: Option<[f64; 3]>,
        major: f64,
        minor: f64,
    ) -> Hyperbola {
        let (c, x, y) = ax2_of(o, n, x);
        Hyperbola {
            c,
            x,
            y,
            a: major,
            b: minor,
        }
    }


    /// `orientation name F|R` (and `treverse`): the shape as it is, or turned round (a wire's or
    /// an edge's direction, the sides of a face or of a body's faces).
    pub fn orientation(&mut self, s: &Shape, o: &str) -> Result<Shape> {
        match o.to_ascii_uppercase().as_str() {
            "F" => return Ok(s.clone()),
            "R" => {}
            _ => {
                return Err(KernelError::Unsupported(
                    "internal and external orientations",
                ));
            }
        }
        let rev = |l: &[DEdge]| l.iter().rev().map(DEdge::reversed).collect::<Vec<_>>();
        Ok(match s {
            Shape::Vertex(_) | Shape::Empty => s.clone(),
            Shape::Edge(e) => Shape::Edge(e.reversed()),
            Shape::Wire(w) => Shape::Wire(rev(w)),
            Shape::Face { plane, loops } => Shape::Face {
                plane: Plane {
                    origin: plane.origin,
                    x_dir: plane.x_dir,
                    normal: -plane.normal,
                },
                loops: loops.iter().map(|l| rev(l)).collect(),
            },
            Shape::Body(b) => {
                let mut body = (**self.get(*b)?).clone();
                for f in &mut body.faces {
                    super::topo::Body::reverse_face(f);
                }
                Shape::Body(self.insert(body))
            }
            Shape::Compound(v) => Shape::Compound(
                v.iter()
                    .map(|x| self.orientation(x, o))
                    .collect::<Result<_>>()?,
            ),
            _ => return Err(KernelError::Unsupported("turning a sub-shape round")),
        })
    }

    // --- Topology ---------------------------------------------------------------------------

    /// `vertex name x y z`.
    pub fn vertex(&self, x: f64, y: f64, z: f64) -> Shape {
        Shape::Vertex(P3::new(x, y, z))
    }

    /// `edge name v1 v2`: the segment between two vertices.
    pub fn edge(&self, v1: &Shape, v2: &Shape) -> Result<Shape> {
        let (Shape::Vertex(a), Shape::Vertex(b)) = (v1, v2) else {
            return Err(err("edge takes two vertices"));
        };
        let (curve, l) = Curve::line(*a, *b);
        Ok(Shape::Edge(DEdge {
            curve,
            t0: 0.0,
            t1: l,
        }))
    }

    /// `mkedge name curve t0 t1`.
    pub fn mkedge(&self, c: &Curve, t0: f64, t1: f64) -> Shape {
        Shape::Edge(DEdge {
            curve: c.clone(),
            t0,
            t1,
        })
    }


    /// `ellipse name x y z [nx ny nz [xx xy xz]] major minor` (normal +z, OCCT's x direction
    /// for the normal unless given).
    pub fn ellipse(
        &self,
        o: [f64; 3],
        n: Option<[f64; 3]>,
        x: Option<[f64; 3]>,
        major: f64,
        minor: f64,
    ) -> Curve {
        let (c, xd, yd) = ax2_of(o, n, x);
        Curve::Ellipse {
            c,
            x: xd,
            y: yd,
            a: major,
            b: minor,
        }
    }

    /// `parabola name x y z [nx ny nz [xx xy xz]] focal`: O + t²/(4f)·X + t·Y, as an exact
    /// quadratic on t ∈ [−10⁴, 10⁴] (OCCT's parameters).
    pub fn parabola(
        &self,
        o: [f64; 3],
        n: Option<[f64; 3]>,
        x: Option<[f64; 3]>,
        focal: f64,
    ) -> Result<Curve> {
        if !(focal > 0.0) {
            return Err(err("parabola: the focal length must be positive"));
        }
        let (c, xd, yd) = ax2_of(o, n, x);
        const L: f64 = 1e4;
        // The quadratic's Bézier poles on [−L, L]: its ends and the crossing of their tangents.
        let p = |t: f64| c + xd * (t * t / (4.0 * focal)) + yd * t;
        let mid = c - xd * (L * L / (4.0 * focal));
        let b =
            super::nurbs::NurbsCurve::bezier(&[p(-L), mid, p(L)], &[1.0, 1.0, 1.0]).reparam(-L, L);
        Ok(Curve::Nurbs(std::sync::Arc::new(b)))
    }



    /// `wire name e1 e2 …`: the edges chained end to start (each turned round when needed).
    pub fn wire(&self, edges: &[&Shape]) -> Result<Shape> {
        let mut out: Vec<DEdge> = Vec::new();
        for s in edges {
            let list: Vec<DEdge> = match s {
                Shape::Edge(e) => vec![e.clone()],
                Shape::Wire(w) => w.clone(),
                _ => return Err(err("wire takes edges and wires")),
            };
            for e in list {
                out.push(e);
            }
        }
        orient_chain(&mut out)?;
        Ok(Shape::Wire(out))
    }

    /// `polyline name x1 y1 z1 x2 y2 z2 …`.
    pub fn polyline(&self, pts: &[[f64; 3]]) -> Result<Shape> {
        let mut out = Vec::new();
        for w in pts.windows(2) {
            let (a, b) = (P3::from(w[0]), P3::from(w[1]));
            let (curve, l) = Curve::line(a, b);
            if l > 0.0 {
                out.push(DEdge {
                    curve,
                    t0: 0.0,
                    t1: l,
                });
            }
        }
        if out.is_empty() {
            return Err(err("an empty polyline"));
        }
        Ok(Shape::Wire(out))
    }


    /// `mkplane name wire`: the planar face inside a closed wire.
    pub fn mkplane(&self, w: &Shape) -> Result<Shape> {
        let edges = match w {
            Shape::Wire(w) => w.clone(),
            Shape::Edge(e) => vec![e.clone()],
            _ => return Err(err("mkplane takes a wire")),
        };
        let plane = fit_plane(&edges).ok_or_else(|| err("the wire isn't planar"))?;
        Ok(Shape::Face {
            plane,
            loops: vec![edges],
        })
    }

    /// `add what to`: into a compound, or a wire as a hole in a planar face.
    pub fn add(&self, what: &Shape, to: &Shape) -> Result<Shape> {
        match to {
            Shape::Compound(v) => {
                let mut v = v.clone();
                v.push(what.clone());
                Ok(Shape::Compound(v))
            }
            Shape::Empty => Ok(Shape::Compound(vec![what.clone()])),
            _ => self.add_hole(what, to),
        }
    }

    /// `add wire face`: a hole in a planar face.
    pub fn add_hole(&self, w: &Shape, f: &Shape) -> Result<Shape> {
        let (Shape::Wire(w), Shape::Face { plane, loops }) = (w, f) else {
            return Err(err("add takes a wire and a face"));
        };
        let mut loops = loops.clone();
        loops.push(w.clone());
        Ok(Shape::Face {
            plane: *plane,
            loops,
        })
    }

    /// `profile name [S face] codes…`: DRAW's language for a planar profile, a point moving in
    /// a plane (XY by default) with a direction (1 0 to start). `code` holds the instructions
    /// with numbers already evaluated: `O x y z` and `P nx ny nz xx xy xz` place the plane,
    /// `F x y` sets the start, `X`/`Y`/`L`/`T` move by, `XX`/`YY`/`TT` move to, `IX`/`IY` move
    /// to a vertical/horizontal line, `R`/`RR` turn by/to an angle, `D` sets the direction,
    /// `C radius angle` makes a tangent arc. The profile closes to the start and makes a face,
    /// unless it ends with `W` (a closed wire) or `WW` (an open wire). `face` is the plane of
    /// `S face` (a planar face).
    pub fn profile(&self, face: Option<&Shape>, code: &str) -> Result<Shape> {
        const CONF: f64 = 1e-7;
        let (mut o, mut zn, mut xn) = (P3::origin(), V3::z(), V3::x());
        if let Some(f) = face {
            let plane = match f {
                Shape::Face { plane, .. } => *plane,
                Shape::BodyFace(b, id) => match &self.get(*b)?.faces[id.0 as usize].surf {
                    super::surface::Surface::Plane { f } => Plane {
                        origin: f.o,
                        x_dir: nalgebra::Unit::new_normalize(f.x),
                        normal: nalgebra::Unit::new_normalize(f.x.cross(&f.y)),
                    },
                    _ => {
                        return Err(KernelError::Unsupported(
                            "profile on a face that isn't planar",
                        ));
                    }
                },
                _ => return Err(err("profile S takes a face")),
            };
            (o, zn, xn) = (
                plane.origin,
                plane.normal.into_inner(),
                plane.x_dir.into_inner(),
            );
        }
        let w: Vec<&str> = code.split_whitespace().collect();
        let (mut x0, mut y0, mut x, mut y, mut dx, mut dy) = (0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
        let (mut make_face, mut close, mut first, mut stay_first) = (true, true, true, false);
        // The moves in plane coordinates: a line (start, direction, length) or an arc (centre,
        // start angle, signed sweep, radius).
        enum Move {
            Line([f64; 2], [f64; 2], f64),
            Arc([f64; 2], f64, f64, f64),
        }
        let mut moves: Vec<Move> = Vec::new();
        let mut i = 0;
        let num = |i: usize| -> Result<f64> {
            w.get(i)
                .ok_or_else(|| err("profile: bad number of arguments"))?
                .parse::<f64>()
                .map_err(|_| err("profile: a bad number"))
        };
        while i < w.len() {
            let c = w[i].to_ascii_uppercase();
            let second = c.chars().nth(1);
            let mut line = None;
            let mut arc = None;
            match c.chars().next().unwrap_or(' ') {
                'F' => {
                    if !first {
                        return Err(err("profile: F must precede all moves"));
                    }
                    (x, y) = (num(i + 1)?, num(i + 2)?);
                    (x0, y0) = (x, y);
                    stay_first = true;
                    i += 2;
                }
                'O' => {
                    o = P3::new(num(i + 1)?, num(i + 2)?, num(i + 3)?);
                    stay_first = true;
                    i += 3;
                }
                'P' => {
                    let vn = V3::new(num(i + 1)?, num(i + 2)?, num(i + 3)?);
                    let vx = V3::new(num(i + 4)?, num(i + 5)?, num(i + 6)?);
                    if vn.norm() <= CONF || vx.norm() <= CONF {
                        return Err(err("profile: a null direction"));
                    }
                    zn = vn.normalize();
                    xn = zn
                        .cross(&vx)
                        .cross(&zn)
                        .try_normalize(1e-12)
                        .ok_or_else(|| err("profile: parallel directions"))?;
                    stay_first = true;
                    i += 6;
                }
                'X' => {
                    let mut l = num(i + 1)?;
                    if second == Some('X') {
                        l -= x;
                    }
                    (dx, dy) = (1.0, 0.0);
                    line = Some(l);
                    i += 1;
                }
                'Y' => {
                    let mut l = num(i + 1)?;
                    if second == Some('Y') {
                        l -= y;
                    }
                    (dx, dy) = (0.0, 1.0);
                    line = Some(l);
                    i += 1;
                }
                'L' => {
                    line = Some(num(i + 1)?);
                    i += 1;
                }
                'T' => {
                    let (mut vx, mut vy) = (num(i + 1)?, num(i + 2)?);
                    if second == Some('T') {
                        vx -= x;
                        vy -= y;
                    }
                    let l = vx.hypot(vy);
                    if l > CONF {
                        (dx, dy) = (vx / l, vy / l);
                        line = Some(l);
                    }
                    i += 2;
                }
                'R' => {
                    let a = num(i + 1)?.to_radians();
                    if second == Some('R') {
                        (dx, dy) = (a.cos(), a.sin());
                    } else {
                        (dx, dy) = (a.cos() * dx - a.sin() * dy, a.sin() * dx + a.cos() * dy);
                    }
                    i += 1;
                }
                'D' => {
                    let (vx, vy) = (num(i + 1)?, num(i + 2)?);
                    let l = vx.hypot(vy);
                    if l > CONF {
                        (dx, dy) = (vx / l, vy / l);
                    }
                    i += 2;
                }
                'C' => {
                    let r = num(i + 1)?;
                    if r.abs() > CONF {
                        arc = Some((r, num(i + 2)?.to_radians()));
                    }
                    i += 2;
                }
                'I' => {
                    let v = num(i + 1)?;
                    match second {
                        Some('X') => {
                            if dx.abs() < CONF {
                                return Err(err("profile: can't intersect"));
                            }
                            line = Some((v - x) / dx);
                        }
                        Some('Y') => {
                            if dy.abs() < CONF {
                                return Err(err("profile: can't intersect"));
                            }
                            line = Some((v - y) / dy);
                        }
                        _ => {}
                    }
                    i += 1;
                }
                'W' => {
                    make_face = false;
                    close = second != Some('W');
                    i = w.len() - 1;
                }
                _ => return Err(err(&format!("profile: unknown code {}", w[i]))),
            }
            loop {
                if let Some(mut l) = line.take() {
                    if l < 0.0 {
                        l = -l;
                        (dx, dy) = (-dx, -dy);
                    }
                    moves.push(Move::Line([x, y], [dx, dy], l));
                    x += l * dx;
                    y += l * dy;
                } else if let Some((r, a)) = arc.take() {
                    // The centre is on the left of the direction for a positive radius; the
                    // arc turns left for a positive radius and angle.
                    let (cx, cy) = (x - r * dy, y + r * dx);
                    let ccw = (r > 0.0) == (a > 0.0);
                    let sweep = a.abs() * if ccw { 1.0 } else { -1.0 };
                    let start = (y - cy).atan2(x - cx);
                    let rr = r.abs();
                    moves.push(Move::Arc([cx, cy], start, sweep, rr));
                    let e = start + sweep;
                    (x, y) = (cx + rr * e.cos(), cy + rr * e.sin());
                    (dx, dy) = if ccw {
                        (-e.sin(), e.cos())
                    } else {
                        (e.sin(), -e.cos())
                    };
                }
                first = stay_first;
                stay_first = false;
                i += 1;
                if i == w.len() && close {
                    let (vx, vy) = (x0 - x, y0 - y);
                    let l = vx.hypot(vy);
                    if l > CONF {
                        (dx, dy) = (vx / l, vy / l);
                        line = Some(l);
                        i -= 1;
                        continue;
                    }
                }
                break;
            }
        }
        let yn = zn.cross(&xn);
        let at = |p: [f64; 2]| o + xn * p[0] + yn * p[1];
        let mut edges = Vec::new();
        let mut area = 0.0;
        for m in &moves {
            match *m {
                Move::Line(p, d, l) => {
                    if l <= CONF {
                        continue;
                    }
                    let q = [p[0] + d[0] * l, p[1] + d[1] * l];
                    area += p[0] * q[1] - q[0] * p[1];
                    let (curve, len) = Curve::line(at(p), at(q));
                    edges.push(DEdge {
                        curve,
                        t0: 0.0,
                        t1: len,
                    });
                }
                Move::Arc(c, start, sweep, r) => {
                    // The polygon term of the chord plus the circular segment.
                    let (p, q) = (
                        [c[0] + r * start.cos(), c[1] + r * start.sin()],
                        [
                            c[0] + r * (start + sweep).cos(),
                            c[1] + r * (start + sweep).sin(),
                        ],
                    );
                    area += p[0] * q[1] - q[0] * p[1] + r * r * (sweep - sweep.sin());
                    let xd = xn * start.cos() + yn * start.sin();
                    let yd = zn.cross(&xd) * sweep.signum();
                    edges.push(DEdge {
                        curve: Curve::Circle {
                            c: at(c),
                            x: xd,
                            y: yd,
                            r,
                        },
                        t0: 0.0,
                        t1: sweep.abs(),
                    });
                }
            }
        }
        if edges.is_empty() {
            return Err(err("profile: no edges"));
        }
        if !make_face {
            return Ok(Shape::Wire(edges));
        }
        // OCCT turns the face round when the wire runs clockwise in the plane.
        let normal = if area >= 0.0 { zn } else { -zn };
        Ok(Shape::Face {
            plane: Plane {
                origin: o,
                x_dir: nalgebra::Unit::new_normalize(xn),
                normal: nalgebra::Unit::new_normalize(normal),
            },
            loops: vec![edges],
        })
    }

    // --- Primitives -------------------------------------------------------------------------

    /// `box name dx dy dz`.
    pub fn box_(&mut self, dx: f64, dy: f64, dz: f64) -> Result<Shape> {
        self.box_at(0.0, 0.0, 0.0, dx, dy, dz)
    }

    /// `box name x y z dx dy dz`.
    pub fn box_at(&mut self, x: f64, y: f64, z: f64, dx: f64, dy: f64, dz: f64) -> Result<Shape> {
        let ax = Ax3 {
            origin: P3::new(x, y, z),
            z: V3::z(),
            x: V3::x(),
        };
        self.box_on(&ax, dx, dy, dz)
    }

    /// `box name -pl plane dx dy dz` (a box in a placement).
    pub fn box_on(&mut self, ax: &Ax3, dx: f64, dy: f64, dz: f64) -> Result<Shape> {
        let (x0, x1) = (0.0f64.min(dx), 0.0f64.max(dx));
        let (y0, y1) = (0.0f64.min(dy), 0.0f64.max(dy));
        let mut pl = ax.plane();
        if dz < 0.0 {
            pl.origin += ax.z * dz;
        }
        let rect = |a: (f64, f64), b: (f64, f64), c: (f64, f64), d: (f64, f64)| {
            let p = |q: (f64, f64)| nalgebra::Point2::new(q.0, q.1);
            Loop {
                curves: vec![
                    l2(p(a), p(b)),
                    l2(p(b), p(c)),
                    l2(p(c), p(d)),
                    l2(p(d), p(a)),
                ],
            }
        };
        let region = Region {
            outer: rect((x0, y0), (x1, y0), (x1, y1), (x0, y1)),
            holes: vec![],
            source: None,
        };
        let r = self.extrude(&Profile::new(pl, vec![region]), Extent::Blind(dz.abs()))?;
        // OCCT's order (BRepPrim_GWedge): faces XMin XMax YMin YMax ZMin ZMax; edges as met
        // along the faces' wires.
        let (z0, z1) = (0.0f64.min(dz), 0.0f64.max(dz));
        let (xm, ym, zm) = ((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0);
        let w = |x: f64, y: f64, z: f64| ax.origin + ax.x * x + ax.y() * y + ax.z * z;
        let edges = vec![
            w(x0, y0, zm),
            w(x0, ym, z1),
            w(x0, y1, zm),
            w(x0, ym, z0),
            w(x1, y0, zm),
            w(x1, ym, z1),
            w(x1, y1, zm),
            w(x1, ym, z0),
            w(xm, y0, z0),
            w(xm, y0, z1),
            w(xm, y1, z0),
            w(xm, y1, z1),
        ];
        let faces = vec![
            w(x0, ym, zm),
            w(x1, ym, zm),
            w(xm, y0, zm),
            w(xm, y1, zm),
            w(xm, ym, z0),
            w(xm, ym, z1),
        ];
        set_order(
            r.bodies[0],
            OcctOrder {
                edges,
                faces,
                degenerate: vec![],
            },
        );
        Ok(Shape::Body(r.bodies[0]))
    }

    /// `pcylinder name r h`.
    pub fn pcylinder(&mut self, r: f64, h: f64) -> Result<Shape> {
        self.pcylinder_on(
            &Ax3 {
                origin: P3::origin(),
                z: V3::z(),
                x: V3::x(),
            },
            r,
            h,
        )
    }

    /// `pcylinder name plane r h`.
    pub fn pcylinder_on(&mut self, ax: &Ax3, r: f64, h: f64) -> Result<Shape> {
        let mut pl = ax.plane();
        if h < 0.0 {
            pl.origin += ax.z * h;
        }
        let region = Region {
            outer: Loop {
                curves: vec![Curve2::Circle {
                    center: nalgebra::Point2::origin(),
                    radius: r,
                    source: None,
                }],
            },
            holes: vec![],
            source: None,
        };
        let res = self.extrude(&Profile::new(pl, vec![region]), Extent::Blind(h.abs()))?;
        set_order(res.bodies[0], revolved_order(ax, r, r, h));
        Ok(Shape::Body(res.bodies[0]))
    }

    /// `pcylinder name [plane] r h angle` (degrees): a sector of the cylinder.
    pub fn pcylinder_a(&mut self, ax: Option<&Ax3>, r: f64, h: f64, angle: f64) -> Result<Shape> {
        let ax = ax.copied().unwrap_or(Ax3 {
            origin: P3::origin(),
            z: V3::z(),
            x: V3::x(),
        });
        if angle >= 360.0 - 1e-9 {
            return self.pcylinder_on(&ax, r, h);
        }
        let p = nalgebra::Point2::new;
        let a = angle.to_radians();
        let s = self.revolve_section(&ax, &[p(0.0, 0.0), p(r, 0.0), p(r, h), p(0.0, h)], a)?;
        if let Shape::Body(b) = &s {
            set_order(
                *b,
                one_axis_order(
                    &ax,
                    (r, 0.0f64.min(h)),
                    (r, 0.0f64.max(h)),
                    (r, h / 2.0),
                    false,
                    a,
                ),
            );
        }
        Ok(s)
    }

    /// `pcone name [plane] r1 r2 h angle` (degrees).
    pub fn pcone_a(
        &mut self,
        ax: Option<&Ax3>,
        r1: f64,
        r2: f64,
        h: f64,
        angle: f64,
    ) -> Result<Shape> {
        let ax = ax.copied().unwrap_or(Ax3 {
            origin: P3::origin(),
            z: V3::z(),
            x: V3::x(),
        });
        if angle >= 360.0 - 1e-9 {
            return self.pcone_on(&ax, r1, r2, h);
        }
        let p = nalgebra::Point2::new;
        let mut pts = vec![p(0.0, 0.0)];
        if r1 > 0.0 {
            pts.push(p(r1, 0.0));
        }
        if r2 > 0.0 {
            pts.push(p(r2, h));
        }
        pts.push(p(0.0, h));
        let a = angle.to_radians();
        let s = self.revolve_section(&ax, &pts, a)?;
        if let Shape::Body(b) = &s {
            set_order(
                *b,
                one_axis_order(
                    &ax,
                    (r1, 0.0),
                    (r2, h),
                    ((r1 + r2) / 2.0, h / 2.0),
                    false,
                    a,
                ),
            );
        }
        Ok(s)
    }



    /// `ptorus name [plane] R r a1 a2 [angle]`: a torus zone, the pipe's circle from a1 to a2
    /// (degrees about its centre, from the outside) closed by planes square to the axis down to
    /// it (as OCCT's revolved primitives), turned by `angle`.
    pub fn ptorus_zone(
        &mut self,
        ax: Option<&Ax3>,
        big: f64,
        r: f64,
        a1: f64,
        a2: f64,
        angle: f64,
    ) -> Result<Shape> {
        let ax = ax.copied().unwrap_or(Ax3 {
            origin: P3::origin(),
            z: V3::z(),
            x: V3::x(),
        });
        let (a1, a2) = (a1.to_radians(), a2.to_radians());
        let c = nalgebra::Point2::new(big, 0.0);
        let (p1, p2) = (
            nalgebra::Point2::new(big + r * a1.cos(), r * a1.sin()),
            nalgebra::Point2::new(big + r * a2.cos(), r * a2.sin()),
        );
        if (a2 - a1).abs() >= TAU - 1e-9 {
            return self.ptorus_a(Some(&ax), big, r, angle);
        }
        let mut curves = vec![Curve2::Arc {
            center: c,
            radius: r,
            start_angle: a1,
            sweep: a2 - a1,
            source: None,
        }];
        let (q2, q1) = (
            nalgebra::Point2::new(0.0, p2.y),
            nalgebra::Point2::new(0.0, p1.y),
        );
        if p2.x.abs() > 1e-12 {
            curves.push(l2(p2, q2));
        }
        if (q2 - q1).norm() > 1e-12 {
            curves.push(l2(q2, q1));
        }
        if p1.x.abs() > 1e-12 {
            curves.push(l2(q1, p1));
        }
        let region = Region {
            outer: Loop { curves },
            holes: vec![],
            source: None,
        };
        let res = self.revolve(
            &Profile::new(self.meridian(&ax), vec![region]),
            Axis {
                origin: ax.origin,
                dir: nalgebra::Unit::new_normalize(ax.z),
            },
            angle.min(360.0).to_radians(),
        )?;
        let am = (a1 + a2) / 2.0;
        set_order(
            res.bodies[0],
            one_axis_order(
                &ax,
                (p1.x, p1.y),
                (p2.x, p2.y),
                (big + r * am.cos(), r * am.sin()),
                false,
                angle.min(360.0).to_radians(),
            ),
        );
        Ok(Shape::Body(res.bodies[0]))
    }

    /// `ptorus name [plane] R r angle` (degrees of longitude).
    pub fn ptorus_a(&mut self, ax: Option<&Ax3>, big: f64, r: f64, angle: f64) -> Result<Shape> {
        let ax = ax.copied().unwrap_or(Ax3 {
            origin: P3::origin(),
            z: V3::z(),
            x: V3::x(),
        });
        let region = Region {
            outer: Loop {
                curves: vec![Curve2::Circle {
                    center: nalgebra::Point2::new(big, 0.0),
                    radius: r,
                    source: None,
                }],
            },
            holes: vec![],
            source: None,
        };
        let res = self.revolve(
            &Profile::new(self.meridian(&ax), vec![region]),
            Axis {
                origin: ax.origin,
                dir: nalgebra::Unit::new_normalize(ax.z),
            },
            angle.min(360.0).to_radians(),
        )?;
        set_order(
            res.bodies[0],
            one_axis_order(
                &ax,
                (big + r, 0.0),
                (big + r, 0.0),
                (big - r, 0.0),
                true,
                angle.min(360.0).to_radians(),
            ),
        );
        Ok(Shape::Body(res.bodies[0]))
    }


    /// `pcone name plane r1 r2 h`.
    pub fn pcone_on(&mut self, ax: &Ax3, r1: f64, r2: f64, h: f64) -> Result<Shape> {
        let p = nalgebra::Point2::new;
        let mut pts = vec![p(0.0, 0.0)];
        if r1 > 0.0 {
            pts.push(p(r1, 0.0));
        }
        if r2 > 0.0 {
            pts.push(p(r2, h));
        }
        pts.push(p(0.0, h));
        let s = self.revolve_section(ax, &pts, TAU)?;
        if let Shape::Body(b) = &s {
            set_order(
                *b,
                if r1 > 0.0 && r2 > 0.0 {
                    revolved_order(ax, r1, r2, h)
                } else {
                    one_axis_order(
                        ax,
                        (r1, 0.0),
                        (r2, h),
                        ((r1 + r2) / 2.0, h / 2.0),
                        false,
                        TAU,
                    )
                },
            );
        }
        Ok(s)
    }

    /// `psphere name r`.
    pub fn psphere(&mut self, r: f64) -> Result<Shape> {
        self.psphere_on(
            &Ax3 {
                origin: P3::origin(),
                z: V3::z(),
                x: V3::x(),
            },
            r,
        )
    }

    pub fn psphere_on(&mut self, ax: &Ax3, r: f64) -> Result<Shape> {
        let half = Region {
            outer: Loop {
                curves: vec![
                    Curve2::Arc {
                        center: nalgebra::Point2::origin(),
                        radius: r,
                        start_angle: -PI / 2.0,
                        sweep: PI,
                        source: None,
                    },
                    l2(
                        nalgebra::Point2::new(0.0, r),
                        nalgebra::Point2::new(0.0, -r),
                    ),
                ],
            },
            holes: vec![],
            source: None,
        };
        let res = self.revolve(
            &Profile::new(self.meridian(ax), vec![half]),
            Axis {
                origin: ax.origin,
                dir: nalgebra::Unit::new_normalize(ax.z),
            },
            TAU,
        )?;
        set_order(
            res.bodies[0],
            one_axis_order(ax, (0.0, -r), (0.0, r), (r, 0.0), false, TAU),
        );
        Ok(Shape::Body(res.bodies[0]))
    }



    /// The half plane through a placement's axis, x along its x and y along its axis.
    fn meridian(&self, ax: &Ax3) -> Plane {
        // y_dir = normal × x_dir must be z: normal = x × z.
        Plane {
            origin: ax.origin,
            x_dir: nalgebra::Unit::new_normalize(ax.x),
            normal: nalgebra::Unit::new_normalize(ax.x.cross(&ax.z)),
        }
    }

    fn revolve_section(
        &mut self,
        ax: &Ax3,
        pts: &[nalgebra::Point2<f64>],
        angle: f64,
    ) -> Result<Shape> {
        let n = pts.len();
        let curves = (0..n).map(|i| l2(pts[i], pts[(i + 1) % n])).collect();
        let region = Region {
            outer: Loop { curves },
            holes: vec![],
            source: None,
        };
        let res = self.revolve(
            &Profile::new(self.meridian(ax), vec![region]),
            Axis {
                origin: ax.origin,
                dir: nalgebra::Unit::new_normalize(ax.z),
            },
            angle,
        )?;
        Ok(Shape::Body(res.bodies[0]))
    }

    // --- Sweeps -----------------------------------------------------------------------------

    /// `prism name face dx dy dz` (along the face's normal).
    pub fn prism(&mut self, f: &Shape, dx: f64, dy: f64, dz: f64) -> Result<Shape> {
        if let Some(s) = self.sweep_lower(f, &BodySweep::Translate(V3::new(dx, dy, dz)))? {
            return Ok(s);
        }
        if let Some(sheet) = self.sheet_shape(f)? {
            return Ok(Shape::Body(
                crate::Kernel::sweep_body(self, sheet, &BodySweep::Translate(V3::new(dx, dy, dz)))?
                    .bodies[0],
            ));
        }
        let profile = face_profile(f)?;
        let v = V3::new(dx, dy, dz);
        let n = profile.plane.normal.into_inner();
        let along = v.dot(&n);
        let order = || match f {
            Shape::Face { loops, .. } => {
                Some(sweep_order(loops, &|p, s| p + v * s, &|_| true, false))
            }
            _ => None,
        };
        if (v - n * along).norm() > 1e-9 * v.norm().max(1.0) {
            let r = super::features::prism_along(self, &profile, v)?;
            if let Some(o) = order() {
                set_order(r.bodies[0], o);
            }
            return Ok(Shape::Body(r.bodies[0]));
        }
        let mut p = profile;
        if along < 0.0 {
            p.plane.origin += n * along;
        }
        let r = self.extrude(&p, Extent::Blind(along.abs()))?;
        if let Some(o) = order() {
            set_order(r.bodies[0], o);
        }
        Ok(Shape::Body(r.bodies[0]))
    }

    /// A sheet to sweep into a solid: a sheet body (a face of a surface) or a face of a body as
    /// a sheet of its own; None for DRAW's planar faces (swept as profiles) and other shapes.
    fn sheet_shape(&mut self, s: &Shape) -> Result<Option<BodyId>> {
        Ok(match s {
            Shape::Body(b)
                if self.get(*b)?.solids.is_empty() && !self.get(*b)?.faces.is_empty() =>
            {
                Some(*b)
            }
            Shape::BodyFace(b, f) if (f.0 as usize) < self.get(*b)?.faces.len() => {
                let sheet = super::kernel::subset(self.get(*b)?, &[f.0 as usize], false);
                Some(self.insert(sheet))
            }
            _ => None,
        })
    }

    /// A vertex, edge or wire swept one kind higher (into a wire or a sheet body) by
    /// [`Kernel::sweep_body`]; None for other shapes.
    fn sweep_lower(&mut self, s: &Shape, sweep: &BodySweep) -> Result<Option<Shape>> {
        let body = match s {
            Shape::Vertex(p) => crate::Kernel::create_points(self, &[*p])?.bodies[0],
            Shape::Edge(e) => self.insert(super::kernel::wire_of_curves(&[(
                e.curve.clone(),
                e.t0,
                e.t1,
            )])?),
            Shape::Wire(w) => self.insert(super::kernel::wire_of_curves(
                &w.iter()
                    .map(|e| (e.curve.clone(), e.t0, e.t1))
                    .collect::<Vec<_>>(),
            )?),
            _ => return Ok(None),
        };
        let r = crate::Kernel::sweep_body(self, body, sweep)?;
        self.release(body);
        Ok(Some(Shape::Body(r.bodies[0])))
    }

    /// `revol name face x y z dx dy dz angle` (degrees).
    pub fn revol(
        &mut self,
        f: &Shape,
        x: f64,
        y: f64,
        z: f64,
        dx: f64,
        dy: f64,
        dz: f64,
        angle: f64,
    ) -> Result<Shape> {
        let axis = Axis {
            origin: P3::new(x, y, z),
            dir: nalgebra::Unit::new_normalize(V3::new(dx, dy, dz)),
        };
        if let Some(s) = self.sweep_lower(
            f,
            &BodySweep::Rotate {
                axis,
                angle: angle.to_radians(),
            },
        )? {
            return Ok(s);
        }
        if let Some(sheet) = self.sheet_shape(f)? {
            return Ok(Shape::Body(
                crate::Kernel::sweep_body(
                    self,
                    sheet,
                    &BodySweep::Rotate {
                        axis,
                        angle: angle.to_radians(),
                    },
                )?
                .bodies[0],
            ));
        }
        let profile = face_profile(f)?;
        let (o, d) = (P3::new(x, y, z), V3::new(dx, dy, dz).normalize());
        let a = angle.to_radians();
        let r = self.revolve(
            &profile,
            Axis {
                origin: o,
                dir: nalgebra::Unit::new_normalize(d),
            },
            a,
        )?;
        if let Shape::Face { loops, .. } = f {
            let closed = a.abs() >= TAU - 1e-9;
            let at = |p: &P3, s: f64| {
                let th = a * s;
                let w = p - o;
                let along = d * w.dot(&d);
                let r = w - along;
                o + along + r * th.cos() + d.cross(&r) * th.sin()
            };
            let swept = |p: &P3| {
                let w = p - o;
                (w - d * w.dot(&d)).norm() > 1e-9 * (1.0 + w.norm())
            };
            set_order(r.bodies[0], sweep_order(loops, &at, &swept, closed));
        }
        Ok(Shape::Body(r.bodies[0]))
    }






    /// `generated r h shape`: the faces the offset made from an edge of its input (a step
    /// between neighbours moved apart, a round), as a compound.
    pub fn generated(&self, h: &DrawHistory, s: &Shape) -> Result<Shape> {
        let Shape::BodyEdge(b, e) = s else {
            return Err(err("generated: an edge of the offset's input"));
        };
        if *b != h.input {
            return Err(err("generated: an edge of the offset's input"));
        }
        let faces = h
            .history
            .generated
            .iter()
            .filter(|(_, o)| matches!(o, Origin::FromEdge { body, edge } if body == b && edge == e))
            .map(|(f, _)| Shape::BodyFace(h.output, *f))
            .collect::<Vec<_>>();
        Ok(Shape::Compound(faces))
    }






    /// `restore file name`: a shape from an OCCT BRep file (`explode` follows the file's order);
    /// a file of only wires and edges gives its edges.
    pub fn restore(&mut self, path: &str) -> Result<Shape> {
        let bytes = std::fs::read(path).map_err(|e| err(&format!("{path}: {e}")))?;
        let r = super::brep_occt::read_ordered(&bytes).map_err(|e| err(&format!("{path}: {e}")))?;
        let Some(body) = r.body else {
            if let [p] = r.points[..] {
                return Ok(Shape::Vertex(p));
            }
            let mut es: Vec<DEdge> = r
                .free_edges
                .into_iter()
                .map(|(curve, t0, t1)| DEdge { curve, t0, t1 })
                .collect();
            return Ok(if es.len() == 1 && !r.wires {
                Shape::Edge(es.remove(0))
            } else {
                Shape::Wire(es)
            });
        };
        let (edges, degenerate) = (r.edge_order, r.degenerate_order);
        let nf = body.faces.len();
        let b = self.insert(body);
        let infos = self.faces(b)?;
        let body = self.get(b)?;
        if infos.len() == nf {
            // A point on each face, near its middle.
            let faces = (0..nf)
                .map(|i| {
                    let s = &body.faces[i].surf;
                    let (u, v) = s.project(&infos[i].center, None);
                    s.eval(u, v)
                })
                .collect();
            set_order(
                b,
                OcctOrder {
                    edges,
                    faces,
                    degenerate,
                },
            );
        }
        Ok(Shape::Body(b))
    }


    // --- Evolved solids --------------------------------------------------------------------


    // --- Pipes ------------------------------------------------------------------------------

    /// A spine (a wire or an edge) as a sweep path: its segments chained, and whether it closes.
    fn path_of(&self, s: &Shape) -> Result<(Vec<Seg>, bool)> {
        let seg = |c: &Curve, t0: f64, t1: f64| {
            if t0 <= t1 {
                Seg {
                    c: c.clone(),
                    t0,
                    t1,
                    rev: false,
                }
            } else {
                Seg {
                    c: c.clone(),
                    t0: t1,
                    t1: t0,
                    rev: true,
                }
            }
        };
        let segs: Vec<Seg> = match s {
            Shape::Edge(e) => vec![seg(&e.curve, e.t0, e.t1)],
            Shape::Wire(w) => w.iter().map(|e| seg(&e.curve, e.t0, e.t1)).collect(),
            Shape::BodyEdge(b, id) => {
                let e = self
                    .get(*b)?
                    .edges
                    .get(id.0 as usize)
                    .ok_or_else(|| err("an unknown edge"))?;
                vec![seg(&e.curve, e.t[0], e.t[1])]
            }
            _ => return Err(err("the spine must be a wire")),
        };
        super::sweep::path::chain(segs)
    }

    /// A profile of a pipe: a planar face, or a wire or edge (closed or open). `hint`
    /// is the direction a straight profile's plane is made square to.
    fn section_of(&self, s: &Shape, hint: &V3) -> Result<Section> {
        let wire = |edges: Vec<DEdge>| -> Result<Section> {
            let closed = !edges.is_empty()
                && (edges[0].start() - edges[edges.len() - 1].end()).norm()
                    < 1e-6 * (1.0 + edges[0].length());
            let plane = match fit_plane(&edges) {
                Some(p) => p,
                None => {
                    // A straight profile: the plane through it nearest square to `hint`.
                    let d = (edges[edges.len() - 1].end() - edges[0].start())
                        .try_normalize(1e-12)
                        .ok_or_else(|| err("a profile of no length"))?;
                    let n = (hint - d * hint.dot(&d))
                        .try_normalize(1e-9)
                        .unwrap_or_else(|| ax2_x(&d));
                    Plane {
                        origin: edges[0].start(),
                        x_dir: nalgebra::Unit::new_normalize(d),
                        normal: nalgebra::Unit::new_normalize(n),
                    }
                }
            };
            Ok(Section {
                plane,
                loops: vec![edges],
                closed,
            })
        };
        match s {
            Shape::Face { plane, loops } => Ok(Section {
                plane: *plane,
                loops: loops.clone(),
                closed: true,
            }),
            Shape::BodyFace(b, f) => {
                let body = self.get(*b)?;
                let face = body
                    .faces
                    .get(f.0 as usize)
                    .ok_or_else(|| err("an unknown face"))?;
                let super::surface::Surface::Plane { f: fr } = &face.surf else {
                    return Err(KernelError::Unsupported(
                        "pipes of faces that aren't planar",
                    ));
                };
                let z = fr.x.cross(&fr.y) * if face.sense { 1.0 } else { -1.0 };
                let plane = Plane {
                    origin: fr.o,
                    x_dir: nalgebra::Unit::new_normalize(fr.x),
                    normal: nalgebra::Unit::new_normalize(z),
                };
                let loops = face
                    .loops
                    .iter()
                    .map(|l| {
                        l.iter()
                            .filter(|c| !body.edges[c.edge].degenerate)
                            .map(|c| {
                                let e = &body.edges[c.edge];
                                let d = DEdge {
                                    curve: e.curve.clone(),
                                    t0: e.t[0],
                                    t1: e.t[1],
                                };
                                if c.fwd { d } else { d.reversed() }
                            })
                            .collect()
                    })
                    .collect();
                Ok(Section {
                    plane,
                    loops,
                    closed: true,
                })
            }
            Shape::Wire(w) => wire(w.clone()),
            Shape::Edge(e) => wire(vec![e.clone()]),
            // A face read from a file: a sheet of one face.
            Shape::Body(b) if self.get(*b)?.faces.len() == 1 => {
                self.section_of(&Shape::BodyFace(*b, FaceId(0)), hint)
            }
            _ => Err(KernelError::Unsupported("pipes of this kind of profile")),
        }
    }

    /// Sweeps a section along spines (from [`super::sweep::path::spines`]): a solid, or a sheet sewn
    /// along the joints of its pieces.
    fn sweep_section(
        &mut self,
        sec: &Section,
        spines: Vec<(Vec<Seg>, bool)>,
        control: super::sweep::Frames,
        join: super::sweep::Join,
        solid: bool,
    ) -> Result<Shape> {
        let (faces, chains) = sec.pieces(solid)?;
        let t = super::sweep::path::sweep_spines(
            &faces, &chains, sec.plane, spines, control, join, solid,
        )?;
        if solid {
            return Ok(Shape::Body(self.insert(t.body)));
        }
        // A sheet's pieces sewn along their joints (kept apart where sewing spoils them).
        let size = t.body.verts.iter().fold(Bbox::empty(), |mut b, v| {
            b.add(&v.p);
            b
        });
        let sewn = super::surfacing::sew_faces(t.body.clone(), (size.diagonal() * 1e-9).max(1e-7))
            .ok()
            .and_then(|mut body| {
                body.shells.clear();
                body.solids.clear();
                super::boolean::assemble_shells(&mut body).ok()?;
                // A closed shell stays a shell (OCCT makes a solid only when asked).
                body.solids.clear();
                body.check().is_empty().then_some(body)
            });
        Ok(Shape::Body(self.insert(sewn.unwrap_or(t.body))))
    }

    /// `pipe result spine profile [mode]` (OCCT's `BRepOffsetAPI_MakePipe`): the profile swept
    /// along the whole spine from where it sits on it (as [`NativeKernel::buildsweep`] places
    /// it), carried rigidly with the spine's frame; a solid from a face, a sheet from a wire or
    /// an edge. Where the spine turns a corner the profile keeps its attitude (OCCT's G0 law);
    /// its three trihedron modes are one here (the rotation-minimizing frame).
    pub fn pipe(&mut self, spine: &Shape, profile: &Shape) -> Result<Shape> {
        let solid = match profile {
            Shape::Face { .. } | Shape::BodyFace(..) => true,
            Shape::Body(b) => !self.get(*b)?.faces.is_empty(),
            _ => false,
        };
        let out = self.buildsweep(
            spine,
            &[(profile, false, false)],
            "CF",
            [0.0, 0.0, 1.0],
            'M',
            solid,
        )?;
        if let (Shape::Face { plane, loops }, Shape::Body(b)) = (profile, &out)
            && let [outer] = &loops[..]
        {
            let (path, _) = self.path_of(spine)?;
            let reversed = plane.normal.dot(&path[0].dir(0.0)) < 0.0;
            if let Some(o) = self.pipe_order(*b, outer, reversed) {
                set_order(*b, o);
            }
        }
        Ok(out)
    }

    /// OCCT's order for a face of one loop piped from where it sits (BRepFill_Pipe, through
    /// BRepFill_Sweep): the start face, each lateral face in the order the loop is explored
    /// (backwards from its first edge when the face looks back along the spine: `reversed`),
    /// then the end face; a lateral face's wire is the edge swept from the profile edge's first
    /// vertex, the profile edge at the start, the edge swept from its last vertex, the profile
    /// edge at the end. Found on the body itself: `None` when its start isn't the profile.
    fn pipe_order(&self, b: BodyId, outer: &[DEdge], reversed: bool) -> Option<OcctOrder> {
        let body = self.get(b).ok()?;
        let infos = self.faces(b).ok()?;
        let size = body.verts.iter().fold(Bbox::empty(), |mut bb, v| {
            bb.add(&v.p);
            bb
        });
        let tol = super::tol::CONFUSION * (1.0 + size.diagonal());
        let mid = |i: usize| body.edges[i].point(body.edges[i].mid_t());
        let near_edge = |p: &P3| {
            (0..body.edges.len())
                .filter(|&i| !body.edges[i].degenerate)
                .min_by(|&x, &y| (mid(x) - p).norm().total_cmp(&(mid(y) - p).norm()))
                .filter(|&i| (mid(i) - p).norm() < tol)
        };
        let near_vert =
            |p: &P3| (0..body.verts.len()).find(|&v| (body.verts[v].p - p).norm() < tol);
        let n = outer.len();
        let walk: Vec<usize> = if reversed {
            std::iter::once(0).chain((1..n).rev()).collect()
        } else {
            (0..n).collect()
        };
        let ef = body.edge_faces();
        let starts: Vec<usize> = walk
            .iter()
            .map(|&k| near_edge(&outer[k].curve.point(0.5 * (outer[k].t0 + outer[k].t1))))
            .collect::<Option<_>>()?;
        // The start face holds every start edge.
        let cap = (0..body.faces.len()).find(|&f| starts.iter().all(|&e| ef[e].contains(&f)))?;
        let mut edges: Vec<usize> = starts.clone();
        let mut faces = vec![cap];
        let push = |edges: &mut Vec<usize>, e: usize| {
            if !edges.contains(&e) {
                edges.push(e);
            }
        };
        for (j, &k) in walk.iter().enumerate() {
            let s = starts[j];
            let lat = *ef[s].iter().find(|&&f| f != cap)?;
            faces.push(lat);
            let ring: Vec<usize> = body.faces[lat]
                .loops
                .iter()
                .flatten()
                .map(|c| c.edge)
                .filter(|&e| e != s && !body.edges[e].degenerate)
                .collect();
            let (v0, v1) = (near_vert(&outer[k].start())?, near_vert(&outer[k].end())?);
            let swept_from =
                |v: usize| ring.iter().copied().find(|&e| body.edges[e].v.contains(&v));
            let a = swept_from(v0)?;
            let z = if v1 == v0 { a } else { swept_from(v1)? };
            let end = ring.iter().copied().find(|&e| e != a && e != z)?;
            push(&mut edges, a);
            push(&mut edges, z);
            push(&mut edges, end);
        }
        let end_cap = (0..body.faces.len()).find(|f| !faces.contains(f))?;
        faces.push(end_cap);
        let all = (0..body.edges.len())
            .filter(|&i| !body.edges[i].degenerate)
            .count();
        (edges.len() == all && faces.len() == body.faces.len()).then(|| OcctOrder {
            edges: edges.into_iter().map(mid).collect(),
            faces: faces.into_iter().map(|f| infos[f].center).collect(),
            degenerate: vec![],
        })
    }

    /// `mksweep spine; setsweep mode; addsweep section [-T] [-R]; buildsweep result [-M|-C|-R]
    /// [-S]` (OCCT's `BRepOffsetAPI_MakePipeShell`) with one section (a wire or an edge): placed
    /// on the spine where its plane crosses it nearest its centre (else at the nearer end), turned
    /// square to the spine there with `correct` (-R), moved onto it with `contact` (-T) — its
    /// point nearest the spine onto the spine — and swept both ways from there. `mode` is
    /// setsweep's: `CF`, `FR` and `DT` follow the spine (the rotation-minimizing frame), `FX`
    /// keeps the section's attitude, `CN` keeps its plane through the binormal `dir`.
    /// `transition` is buildsweep's corner treatment (`M`, `C` or `R`); `solid` (-S) closes a
    /// closed section's sheet with its ends (a planar face sweeps a solid).
    pub fn buildsweep(
        &mut self,
        spine: &Shape,
        sections: &[(&Shape, bool, bool)],
        mode: &str,
        dir: [f64; 3],
        transition: char,
        solid: bool,
    ) -> Result<Shape> {
        let (path, closed) = self.path_of(spine)?;
        if sections.len() > 1 {
            return self.sweep_sections(&path, closed, sections, solid);
        }
        let [(section, contact, correct)] = sections else {
            return Err(err("buildsweep needs a section"));
        };
        let mut sec = self.section_of(section, &path[0].dir(0.0))?;
        let control = match mode {
            "CF" | "DT" => SweepControl::None.into(),
            "FR" => super::sweep::Frames::Frenet,
            "FX" => SweepControl::KeepOrientation.into(),
            "CN" => SweepControl::LockDirection(
                nalgebra::Unit::try_new(V3::from(dir), 1e-12)
                    .ok_or_else(|| err("setsweep -CN: a null direction"))?,
            )
            .into(),
            _ => return Err(KernelError::Unsupported("this setsweep mode")),
        };
        // Where the section sits on the spine (GeomFill_SectionPlacement).
        let center = sec.center();
        let n = sec.plane.normal.into_inner();
        let pl = super::surface::Surface::Plane {
            f: super::build::plane_frame(&sec.plane, V3::zeros()),
        };
        let mut best: Option<(f64, usize, f64)> = None;
        for (i, s) in path.iter().enumerate() {
            for t in super::ssi::curve_surface(&s.c, s.t0, s.t1, &pl) {
                let d = (s.c.point(t) - center).norm();
                if best.is_none_or(|(bd, ..)| d < bd) {
                    best = Some((d, i, s.frac_of(&s.c.point(t))));
                }
            }
        }
        let (si, f) = match best {
            Some((_, i, f)) => (i, f),
            None => {
                let last = path.len() - 1;
                let (d0, d1) = (
                    (path[0].start() - center).dot(&n).abs(),
                    (path[last].end() - center).dot(&n).abs(),
                );
                if d0 <= d1 { (0, 0.0) } else { (last, 1.0) }
            }
        };
        let p = path[si].part(f, f).start();
        let tangent = path[si].dir(f);
        if *correct {
            // Turned about its centre so its plane is square to the spine.
            if let Some(axis) = n.cross(&tangent).try_normalize(1e-12) {
                let r = nalgebra::Rotation3::from_axis_angle(
                    &nalgebra::Unit::new_unchecked(axis),
                    n.angle(&tangent),
                )
                .into_inner();
                sec = sec.moved(&Affine {
                    m: r,
                    t: center.coords - r * center.coords,
                });
            }
        }
        if *contact {
            // Moved so its point nearest the spine's point lies on it.
            let q = sec.nearest(&p);
            sec = sec.moved(&Affine::translation(p - q));
        }
        let solid = solid && sec.closed;
        // The section faces the way the spine runs (else the solid would be inside out).
        if sec.plane.normal.dot(&tangent) < 0.0 {
            sec.plane.normal = -sec.plane.normal;
        }
        // Corners treated as OCCT's right or round corners (on spines of straight segments).
        let corners = path
            .windows(2)
            .any(|w| w[0].dir(1.0).dot(&w[1].dir(0.0)) < 1.0 - 1e-9)
            || (closed && path[path.len() - 1].dir(1.0).dot(&path[0].dir(0.0)) < 1.0 - 1e-9);
        if corners && transition != 'M' && path.iter().all(|s| matches!(s.c, Curve::Line { .. })) {
            return self.corner_pipe(&sec, &path, closed, si, f, transition == 'R', solid);
        }
        let spines = split_spines(path, closed, si, f);
        let join = match transition {
            'C' => super::sweep::Join::Mitre,
            'R' => super::sweep::Join::Round,
            _ => super::sweep::Join::None,
        };
        self.sweep_section(&sec, spines, control, join, solid)
    }

    /// The right-corner pipe (see [`NativeKernel::corner_pipe`]) of sections `secs` at the
    /// starts of straight segments: each segment's prism between the planes bisecting its
    /// corners, as a ruled loft between the section carried along the segment onto them, the
    /// pieces glued at those planes (united where the sections, oblique to the segments, don't
    /// meet exactly). A sheet is the solid without its end caps.
    fn miter_pipe(
        &mut self,
        secs: &[Section],
        verts: &[P3],
        dirs: &[V3],
        lens: &[f64],
        closed: bool,
        solid: bool,
    ) -> Result<Shape> {
        use super::features::Tagged;
        use super::loft_build::WireSection;
        let n = secs.len();
        // The plane bisecting the corner at vertex i (none at an open spine's ends).
        let miter = |i: usize| -> Option<(P3, V3)> {
            if !closed && (i == 0 || i == n) {
                return None;
            }
            let (a, b) = (dirs[(i + n - 1) % n], dirs[i % n]);
            Some((verts[i % n], (a + b).normalize()))
        };
        // The section carried along direction d onto a plane: its edges projected along d.
        let onto = |sec: &Section, d: &V3, plane: Option<(P3, V3)>| -> Vec<(Curve, f64, f64)> {
            let m = match plane {
                Some((o, m)) => {
                    let den = d.dot(&m);
                    let mat = nalgebra::Matrix3::identity() - d * m.transpose() / den;
                    Affine {
                        m: mat,
                        t: d * (o.coords.dot(&m) / den),
                    }
                }
                None => Affine::identity(),
            };
            sec.loops[0]
                .iter()
                .map(|e| match &e.curve {
                    Curve::Line { .. } => {
                        let (c, l) = Curve::line(m.point(&e.start()), m.point(&e.end()));
                        (c, 0.0, l)
                    }
                    Curve::Nurbs(_) => (e.curve.transformed(&m), e.t0, e.t1),
                    _ => {
                        let nc = e
                            .curve
                            .to_nurbs(e.t0.min(e.t1), e.t0.max(e.t1))
                            .transformed(&m);
                        let (lo, hi) = nc.range();
                        let c = Curve::Nurbs(std::sync::Arc::new(nc));
                        if e.t0 <= e.t1 {
                            (c, lo, hi)
                        } else {
                            (c, hi, lo)
                        }
                    }
                })
                .collect()
        };
        let mut acc: Option<Tagged> = None;
        let mut fused: Option<Shape> = None;
        for k in 0..n {
            let start = onto(&secs[k], &dirs[k], miter(k));
            let end_sec = secs[k].moved(&Affine::translation(dirs[k] * lens[k]));
            let end = onto(&end_sec, &dirs[k], miter(k + 1));
            let built = super::loft_build::loft_wires(
                self,
                &[
                    WireSection::Wire(start.clone()),
                    WireSection::Wire(end.clone()),
                ],
                true,
                true,
            )?;
            // Its caps: the planar faces through the first points of the two sections.
            let (p0, p1) = (start[0].0.point(start[0].1), end[0].0.point(end[0].1));
            let mut tags = vec![Vec::new(); built.body.faces.len()];
            for (fi, f) in built.body.faces.iter().enumerate() {
                if let super::surface::Surface::Plane { f: fr } = &f.surf {
                    let z = fr.x.cross(&fr.y);
                    let on = |p: &P3| (p - fr.o).dot(&z).abs() < 1e-7 * (1.0 + lens[k]);
                    if on(&p0) && !on(&p1) {
                        tags[fi].push(Origin::StartCap { region: 0 });
                    } else if on(&p1) && !on(&p0) {
                        tags[fi].push(Origin::EndCap { region: 0 });
                    }
                }
            }
            let piece = Tagged {
                body: built.body,
                tags,
            };
            if let Some(f) = fused.take() {
                let b = self.insert(piece.body);
                fused = Some(self.bfuse(&f, &Shape::Body(b))?);
                continue;
            }
            acc = match acc.take() {
                None => Some(piece),
                Some(a) => match super::sweep::path::glue(&a, &piece) {
                    Some(g) => Some(g),
                    None => {
                        let (x, y) = (self.insert(a.body), self.insert(piece.body));
                        fused = Some(self.bfuse(&Shape::Body(x), &Shape::Body(y))?);
                        None
                    }
                },
            };
        }
        let body = match (acc, fused) {
            (_, Some(f)) => {
                let b = self.body_of(&f)?;
                (**self.get(b)?).clone()
            }
            (Some(a), None) => {
                let a = if closed {
                    super::sweep::path::glue_self(&a).unwrap_or(a)
                } else {
                    a
                };
                if solid {
                    return Ok(Shape::Body(self.insert(a.body)));
                }
                // A sheet: without the end caps.
                let keep: Vec<usize> = (0..a.body.faces.len())
                    .filter(|&f| a.tags[f].is_empty())
                    .collect();
                let mut sheet = super::kernel::subset(&a.body, &keep, false);
                sheet.solids.clear();
                return Ok(Shape::Body(self.insert(sheet)));
            }
            (None, None) => return Err(err("no pipe")),
        };
        if solid {
            return Ok(Shape::Body(self.insert(body)));
        }
        let (s0, s1) = (
            secs[0].clone(),
            secs[n - 1].moved(&Affine::translation(dirs[n - 1] * lens[n - 1])),
        );
        Ok(Shape::Body(self.insert(strip_caps(body, &s0, &s1, closed))))
    }

    /// `buildsweep` through several sections (OCCT's `BRepFill_NSections`) along a straight
    /// spine: each section sits at its distance along the spine (a vertex where it projects onto
    /// it, a wire where its plane crosses it), and the surface interpolates them by the
    /// polynomial through them in that distance (linear for two), extrapolated to the spine's
    /// ends. With `contact` (-T) a vertex moves onto the spine and a wire's point nearest it
    /// does; `correct` (-R) turns a wire square to the spine. One wire among vertices (the
    /// sections are then scaled copies of it), or polygons of as many edges.
    fn sweep_sections(
        &mut self,
        path: &[Seg],
        closed: bool,
        sections: &[(&Shape, bool, bool)],
        solid: bool,
    ) -> Result<Shape> {
        use super::loft_build::WireSection;
        let (a, b) = (path[0].start(), path[path.len() - 1].end());
        let len = (b - a).norm();
        let d = (b - a) / len.max(1e-300);
        if closed
            || len < 1e-9
            || path
                .iter()
                .any(|s| !matches!(s.c, Curve::Line { .. }) || s.dir(0.0).dot(&d) < 1.0 - 1e-9)
        {
            return Err(KernelError::Unsupported(
                "sweeps through several sections along a spine that isn't straight",
            ));
        }
        enum Sec {
            Point(P3),
            Wire(Vec<DEdge>),
        }
        let mut secs: Vec<(f64, Sec)> = Vec::new();
        for (shape, contact, correct) in sections {
            if let Shape::Vertex(p) = shape {
                let at = (p - a).dot(&d);
                secs.push((at, Sec::Point(if *contact { a + d * at } else { *p })));
                continue;
            }
            let mut sec = self.section_of(shape, &d)?;
            let center = sec.center();
            let n = sec.plane.normal.into_inner();
            let at = if n.dot(&d).abs() > 1e-9 {
                (sec.plane.origin - a).dot(&n) / n.dot(&d)
            } else {
                (center - a).dot(&d)
            };
            if *correct && let Some(axis) = n.cross(&d).try_normalize(1e-12) {
                let r = nalgebra::Rotation3::from_axis_angle(
                    &nalgebra::Unit::new_unchecked(axis),
                    n.angle(&d),
                )
                .into_inner();
                sec = sec.moved(&Affine {
                    m: r,
                    t: center.coords - r * center.coords,
                });
            }
            if *contact {
                let p = a + d * at;
                let q = sec.nearest(&p);
                sec = sec.moved(&Affine::translation(p - q));
            }
            secs.push((at, Sec::Wire(sec.loops.swap_remove(0))));
        }
        secs.sort_by(|x, y| x.0.total_cmp(&y.0));
        if secs
            .windows(2)
            .any(|w| w[1].0 - w[0].0 < 1e-9 * (1.0 + len))
        {
            return Err(err("buildsweep: two sections at one place on the spine"));
        }
        let wires: Vec<usize> = (0..secs.len())
            .filter(|&i| matches!(secs[i].1, Sec::Wire(_)))
            .collect();
        let polygons = wires.iter().all(|&i| match &secs[i].1 {
            Sec::Wire(w) => w.iter().all(|e| matches!(e.curve, Curve::Line { .. })),
            Sec::Point(_) => true,
        });
        let counts: Vec<usize> = wires
            .iter()
            .map(|&i| {
                if let Sec::Wire(w) = &secs[i].1 {
                    w.len()
                } else {
                    0
                }
            })
            .collect();
        if wires.is_empty()
            || (wires.len() > 1 && (!polygons || counts.iter().any(|c| *c != counts[0])))
        {
            return Err(KernelError::Unsupported(
                "sweeps through sections that aren't one wire among vertices or alike polygons",
            ));
        }
        // The section at distance v: the sections weighted by the Lagrange basis at v.
        let section_at = |v: f64| -> WireSection {
            let w: Vec<f64> = (0..secs.len())
                .map(|i| {
                    (0..secs.len())
                        .filter(|&j| j != i)
                        .map(|j| (v - secs[j].0) / (secs[i].0 - secs[j].0))
                        .product()
                })
                .collect();
            let mut c = V3::zeros();
            for (i, (_, s)) in secs.iter().enumerate() {
                if let Sec::Point(p) = s {
                    c += p.coords * w[i];
                }
            }
            if let [iw] = wires[..] {
                let Sec::Wire(edges) = &secs[iw].1 else {
                    unreachable!()
                };
                if w[iw].abs() < 1e-9 {
                    return WireSection::Point(P3::from(c));
                }
                let m = Affine {
                    m: nalgebra::Matrix3::identity() * w[iw],
                    t: c,
                };
                return WireSection::Wire(
                    edges
                        .iter()
                        .map(|e| (e.curve.transformed(&m), e.t0, e.t1))
                        .collect(),
                );
            }
            let corner = |k: usize, end: bool| -> P3 {
                let mut p = c;
                for &i in &wires {
                    if let Sec::Wire(edges) = &secs[i].1 {
                        p += (if end {
                            edges[k].end()
                        } else {
                            edges[k].start()
                        })
                        .coords
                            * w[i];
                    }
                }
                P3::from(p)
            };
            WireSection::Wire(
                (0..counts[0])
                    .map(|k| {
                        let (cv, l) = Curve::line(corner(k, false), corner(k, true));
                        (cv, 0.0, l)
                    })
                    .collect(),
            )
        };
        let steps = if secs.len() == 2 { 1 } else { 16 };
        let loft: Vec<WireSection> = (0..=steps)
            .map(|j| section_at(len * j as f64 / steps as f64))
            .collect();
        let built = super::loft_build::loft_wires(self, &loft, solid, steps == 1)?;
        Ok(Shape::Body(self.insert(built.body)))
    }

    /// A pipe along a spine of straight segments with OCCT's corner transitions
    /// (`BRepFill_RightCorner`, `BRepFill_RoundCorner`): the section turns with the spine at each
    /// corner (OCCT's "compatible" law: by the least rotation taking one segment's direction to
    /// the next), each segment sweeps a prism of it, and at a corner the prisms are extended and
    /// cut where they meet (`round` false: at the plane bisecting the corner, exact for sections
    /// square to the spine) or meet as they are with the corner's outside filled by the section
    /// turned about the corner (`round`). A sheet is the solid without its end caps.
    fn corner_pipe(
        &mut self,
        sec: &Section,
        path: &[Seg],
        closed: bool,
        si: usize,
        f: f64,
        round: bool,
        solid: bool,
    ) -> Result<Shape> {
        if !sec.closed {
            return Err(KernelError::Unsupported(
                "corner transitions of open profiles",
            ));
        }
        let n = path.len();
        let verts: Vec<P3> = (0..=n)
            .map(|i| {
                if i < n {
                    path[i].start()
                } else {
                    path[n - 1].end()
                }
            })
            .collect();
        let dirs: Vec<V3> = path.iter().map(|s| s.dir(0.0)).collect();
        let lens: Vec<f64> = path.iter().map(Seg::length).collect();
        // The least rotation about `o` taking direction a to b.
        let turn = |a: &V3, b: &V3, o: &P3| -> Affine {
            let r = match a.cross(b).try_normalize(1e-12) {
                Some(axis) => nalgebra::Rotation3::from_axis_angle(
                    &nalgebra::Unit::new_unchecked(axis),
                    a.angle(b),
                )
                .into_inner(),
                None => nalgebra::Matrix3::identity(),
            };
            Affine {
                m: r,
                t: o.coords - r * o.coords,
            }
        };
        // The section at the start of each segment.
        let mut secs: Vec<Section> = vec![sec.clone(); n];
        secs[si] = sec.moved(&Affine::translation(-dirs[si] * (f * lens[si])));
        for k in (0..si).rev() {
            secs[k] = secs[k + 1]
                .moved(&turn(&dirs[k + 1], &dirs[k], &verts[k + 1]))
                .moved(&Affine::translation(-dirs[k] * lens[k]));
        }
        for k in si..n - 1 {
            secs[k + 1] = secs[k]
                .moved(&Affine::translation(dirs[k] * lens[k]))
                .moved(&turn(&dirs[k], &dirs[k + 1], &verts[k + 1]));
        }
        let mut bb = Bbox::empty();
        for p in verts.iter().chain(
            sec.loops
                .iter()
                .flatten()
                .map(|e| e.start())
                .collect::<Vec<_>>()
                .iter(),
        ) {
            bb.add(p);
        }
        let big = bb.diagonal().max(1.0) * 4.0;
        // The half space {(p − o)·m ≤ 0}, as a big box.
        let half = |k: &mut Self, o: &P3, m: &V3| -> Result<Shape> {
            let m = m.normalize();
            let x = ax2_x(&m);
            let ax = Ax3 {
                origin: o - x * big - m.cross(&x) * big - m * (2.0 * big),
                z: m,
                x,
            };
            k.box_on(&ax, 2.0 * big, 2.0 * big, 2.0 * big)
        };
        let corner_at = |i: usize| closed || (i > 0 && i < n);
        if !round {
            return self.miter_pipe(&secs, &verts, &dirs, &lens, closed, solid);
        }
        let mut acc: Option<Shape> = None;
        for k in 0..n {
            let sheet = self.sheet_of_face(&Shape::Face {
                plane: secs[k].plane,
                loops: secs[k].loops.clone(),
            })?;
            let prism =
                crate::Kernel::sweep_body(self, sheet, &BodySweep::Translate(dirs[k] * lens[k]))?
                    .bodies[0];
            self.release(sheet);
            let piece = Shape::Body(prism);
            acc = Some(match acc {
                None => piece,
                Some(a) => self.bfuse(&a, &piece)?,
            });
            if corner_at(k + 1) {
                // The corner's outside: the end section's part away from the turn, turned.
                let next = dirs[(k + 1) % n];
                let v = verts[k + 1];
                let Some(axis) = dirs[k].cross(&next).try_normalize(1e-12) else {
                    continue;
                };
                let inward = next - dirs[k] * next.dot(&dirs[k]);
                let end = secs[k].moved(&Affine::translation(dirs[k] * lens[k]));
                let sheet = self.sheet_of_face(&Shape::Face {
                    plane: end.plane,
                    loops: end.loops.clone(),
                })?;
                let h = half(self, &v, &inward)?;
                let outer = self.bcommon(&Shape::Body(sheet), &h)?;
                let Shape::Body(ob) = outer else { continue };
                let knuckle = crate::Kernel::sweep_body(
                    self,
                    ob,
                    &BodySweep::Rotate {
                        axis: Axis {
                            origin: v,
                            dir: nalgebra::Unit::new_unchecked(axis),
                        },
                        angle: dirs[k].angle(&next),
                    },
                )?
                .bodies[0];
                let a = acc.take().ok_or_else(|| err("no pipe"))?;
                acc = Some(self.bfuse(&a, &Shape::Body(knuckle))?);
            }
        }
        let shape = acc.ok_or_else(|| err("no pipe"))?;
        if solid {
            return Ok(shape);
        }
        // A sheet: the solid's faces but its end caps.
        let b = self.body_of(&shape)?;
        let body = (**self.get(b)?).clone();
        let last = secs[n - 1].moved(&Affine::translation(dirs[n - 1] * lens[n - 1]));
        Ok(Shape::Body(
            self.insert(strip_caps(body, &secs[0], &last, closed)),
        ))
    }

    // --- Surfaces -----------------------------------------------------------------------------











    /// `wedge name dx dy dz ltx` and `wedge name dx dy dz xmin zmin xmax zmax`
    /// (BRepPrim_Wedge): the box [0, dx] × [0, dy] × [0, dz] whose top (y = dy) is narrowed to
    /// [xmin, xmax] × [zmin, zmax] (`ltx`: [0, ltx] × [0, dz]); its faces and edges in OCCT's
    /// order, as a box's.
    pub fn wedge(&mut self, dx: f64, dy: f64, dz: f64, top: [f64; 4]) -> Result<Shape> {
        let [xmin, zmin, xmax, zmax] = top;
        // The corners by (x, y, z) in {0, 1}: the box's, the top ones moved.
        let c = |i: usize, j: usize, k: usize| -> P3 {
            if j == 0 {
                P3::new(dx * i as f64, 0.0, dz * k as f64)
            } else {
                P3::new(
                    if i == 0 { xmin } else { xmax },
                    dy,
                    if k == 0 { zmin } else { zmax },
                )
            }
        };
        // Faces as quads (outward): XMin XMax YMin YMax ZMin ZMax.
        let quads = [
            [c(0, 0, 0), c(0, 0, 1), c(0, 1, 1), c(0, 1, 0)],
            [c(1, 0, 0), c(1, 1, 0), c(1, 1, 1), c(1, 0, 1)],
            [c(0, 0, 0), c(1, 0, 0), c(1, 0, 1), c(0, 0, 1)],
            [c(0, 1, 0), c(0, 1, 1), c(1, 1, 1), c(1, 1, 0)],
            [c(0, 0, 0), c(0, 1, 0), c(1, 1, 0), c(1, 0, 0)],
            [c(0, 0, 1), c(1, 0, 1), c(1, 1, 1), c(0, 1, 1)],
        ];
        let tris: Vec<[P3; 3]> = quads
            .iter()
            .flat_map(|q| [[q[0], q[1], q[2]], [q[0], q[2], q[3]]])
            .collect();
        let size = (dx.abs() + dy.abs() + dz.abs()).max(1e-9);
        let b = crate::Kernel::mesh_solid(self, &tris, 1e-9 * size)?;
        let pairs = [
            (c(0, 0, 0), c(0, 0, 1)),
            (c(0, 0, 1), c(0, 1, 1)),
            (c(0, 1, 0), c(0, 1, 1)),
            (c(0, 0, 0), c(0, 1, 0)),
            (c(1, 0, 0), c(1, 0, 1)),
            (c(1, 0, 1), c(1, 1, 1)),
            (c(1, 1, 0), c(1, 1, 1)),
            (c(1, 0, 0), c(1, 1, 0)),
            (c(0, 0, 0), c(1, 0, 0)),
            (c(0, 0, 1), c(1, 0, 1)),
            (c(0, 1, 0), c(1, 1, 0)),
            (c(0, 1, 1), c(1, 1, 1)),
        ];
        // A top narrowed to a line or a point has no edges of zero length (OCCT's
        // BRepPrim_GWedge::HasEdge), and its sides meet in one edge there, counted once; a top of
        // no area is no face.
        let tol = super::tol::REL * size;
        let mut edges: Vec<P3> = Vec::new();
        for (a, b) in pairs {
            let m = P3::from((a.coords + b.coords) / 2.0);
            if (a - b).norm() > tol && !edges.iter().any(|q| (q - m).norm() <= tol) {
                edges.push(m);
            }
        }
        let faces = quads
            .iter()
            .filter(|q| {
                (q[1] - q[0]).cross(&(q[3] - q[0])).norm()
                    + (q[1] - q[2]).cross(&(q[3] - q[2])).norm()
                    > tol * size
            })
            .map(|q| P3::from(q.iter().fold(V3::zeros(), |a, p| a + p.coords) / 4.0))
            .collect();
        set_order(
            b,
            OcctOrder {
                edges,
                faces,
                degenerate: vec![],
            },
        );
        Ok(Shape::Body(b))
    }

    // --- Form features (BRepFeat) -------------------------------------------------------------

    /// A feature's profile (`featprism`'s Pbase): a planar face, as a kernel profile or a face
    /// of a body.
    fn feat_profile(&self, pbase: &Shape) -> Result<(Profile, Vec<FaceInput>)> {
        match pbase {
            Shape::Face { .. } => Ok((face_profile(pbase)?, vec![])),
            Shape::BodyFace(b, f) => {
                let plane = self.plane_of(*b, *f).ok_or(KernelError::Unsupported(
                    "features of faces that aren't planar",
                ))?;
                Ok((
                    Profile::new(plane, vec![]),
                    vec![FaceInput {
                        body: *b,
                        face: *f,
                        source: 0,
                    }],
                ))
            }
            // A face read from a file: a sheet of one face.
            Shape::Body(b) if self.get(*b)?.faces.len() == 1 => {
                self.feat_profile(&Shape::BodyFace(*b, FaceId(0)))
            }
            _ => Err(err("the feature's profile must be a planar face")),
        }
    }

    /// A face a feature runs up to (`featperform`'s From and Until): a face of a body, a planar
    /// face, or a sheet (its first face).
    fn feat_face(&mut self, s: &Shape) -> Result<(BodyId, FaceId)> {
        match s {
            Shape::BodyFace(b, f) => Ok((*b, *f)),
            Shape::Face { .. } => Ok((self.sheet_of_face(s)?, FaceId(0))),
            Shape::Body(b) if self.get(*b)?.faces.len() == 1 => Ok((*b, FaceId(0))),
            _ => Err(KernelError::Unsupported(
                "features up to shapes other than faces",
            )),
        }
    }

    /// The plane of a planar face of a body.
    fn plane_of(&self, b: BodyId, f: FaceId) -> Option<Plane> {
        self.faces(b)
            .ok()?
            .into_iter()
            .find(|i| i.id == f)
            .and_then(|i| i.plane)
    }

    /// The feature's tool and how it combines with the base: `kind` is `prism` (`params` the
    /// direction), `revol` (`params` the axis's point and direction) or `dprism` (`params[0]`
    /// the draft angle as DRAW takes it: degrees, halved); `fuse` 1 adds it (a boss), 0 removes
    /// it (a pocket), other values give the tool alone.
    fn feat_apply(&mut self, base: &Shape, tool: BodyId, fuse: i64) -> Result<Shape> {
        match fuse {
            1 => self.bfuse(base, &Shape::Body(tool)),
            0 => self.bcut(base, &Shape::Body(tool)),
            _ => Ok(Shape::Body(tool)),
        }
    }

    /// As [`NativeKernel::feat_apply`] for a feature between limit faces: alone (`fuse` 2), it
    /// is the tool's part outside the base (OCCT's local operation keeps that).
    fn feat_apply_limited(&mut self, base: &Shape, tool: BodyId, fuse: i64) -> Result<Shape> {
        match fuse {
            0 | 1 => self.feat_apply(base, tool, fuse),
            _ => self.bcut(&Shape::Body(tool), base),
        }
    }

    /// An extrude of the feature's profile along `dir` with `end` (and `start` along it first).
    fn feat_extrude(
        &mut self,
        pbase: &Shape,
        dir: V3,
        start: f64,
        end: ExtrudeEnd,
        symmetric: bool,
        scene: Vec<BodyId>,
    ) -> Result<BodyId> {
        let (profile, faces) = self.feat_profile(pbase)?;
        let (dir, end, start) = match end {
            ExtrudeEnd::Blind(l) if l < 0.0 => (-dir, ExtrudeEnd::Blind(-l), -start),
            e => (dir, e, start),
        };
        let spec = ExtrudeSpec {
            body: BodyKind::Solid,
            direction: nalgebra::Unit::try_new(dir, 1e-12)
                .ok_or_else(|| err("a null direction"))?,
            start_offset: start,
            end,
            symmetric,
            second: None,
            scene,
            faces,
        };
        Ok(crate::Kernel::extrude_with(self, &profile, &spec)?.bodies[0])
    }

    /// The signed distance along `dir` from the profile's plane to a planar face's plane.
    fn feat_reach(&self, pbase: &Shape, dir: &V3, b: BodyId, f: FaceId) -> Result<f64> {
        let (profile, _) = self.feat_profile(pbase)?;
        let pl = self.plane_of(b, f).ok_or(KernelError::Unsupported(
            "features between faces that aren't planar",
        ))?;
        let n = pl.normal.into_inner();
        let den = dir.dot(&n);
        if den.abs() < 1e-12 {
            return Err(err("the feature runs parallel to its limit face"));
        }
        Ok((pl.origin - profile.plane.origin).dot(&n) / den)
    }

    /// The centre of a feature's profile: the mean of points along its boundary (the line
    /// through it along the feature finds the limit faces, as BRepFeat's test curve).
    fn feat_centre(&self, pbase: &Shape) -> Result<P3> {
        const SAMPLES: usize = 8;
        let sec = self.section_of(pbase, &V3::z())?;
        let pts: Vec<P3> = sec
            .loops
            .first()
            .into_iter()
            .flatten()
            .flat_map(|e| {
                (0..SAMPLES).map(move |i| {
                    e.curve
                        .point(e.t0 + (e.t1 - e.t0) * i as f64 / SAMPLES as f64)
                })
            })
            .collect();
        if pts.is_empty() {
            return Err(err("the feature's profile has no boundary"));
        }
        Ok(P3::from(
            pts.iter().fold(V3::zeros(), |a, p| a + p.coords) / pts.len() as f64,
        ))
    }

    /// The height a feature sweeps to reach everything (BRepFeat's HeightMax): the largest side
    /// of the box round the base, the profile and the bodies of its limit faces.
    fn feat_height(&self, base: BodyId, pbase: &Shape, limits: &[BodyId]) -> Result<f64> {
        let mut bb = crate::Kernel::bounding_box(self, base)?;
        for &l in limits {
            let lb = crate::Kernel::bounding_box(self, l)?;
            bb.min = bb.min.inf(&lb.min);
            bb.max = bb.max.sup(&lb.max);
        }
        for e in self.section_of(pbase, &V3::z())?.loops.iter().flatten() {
            for p in [e.start(), e.end()] {
                bb.min = bb.min.inf(&p);
                bb.max = bb.max.sup(&p);
            }
        }
        Ok((bb.max - bb.min).max().max(super::tol::POINT))
    }

    /// The surface of a feature's limit face.
    fn limit_surface(&self, (b, f): (BodyId, FaceId)) -> Result<Surface> {
        Ok(self
            .get(b)?
            .faces
            .get(f.0 as usize)
            .ok_or_else(|| err("an unknown face"))?
            .surf
            .clone())
    }

    /// Where the line `p + t·d` crosses a feature's limit faces (sorted, with the face crossed
    /// there): on the faces themselves (BRepFeat's intersector), or, when it misses them all,
    /// their whole surfaces (OCCT extends a face made from a surface to the surface's bounds).
    fn limit_crossings(
        &self,
        lims: &[(BodyId, FaceId)],
        p: P3,
        d: V3,
        reach: f64,
    ) -> Result<Vec<(f64, usize)>> {
        let tol = super::tol::POINT * (1.0 + reach);
        let mut on: Vec<(f64, usize)> = Vec::new();
        let mut all: Vec<(f64, usize)> = Vec::new();
        for (i, &(b, f)) in lims.iter().enumerate() {
            let body = self.get(b)?;
            let face = body
                .faces
                .get(f.0 as usize)
                .ok_or_else(|| err("an unknown face"))?;
            let polys = super::classify::face_polygons(body, face);
            for (t, u, v) in super::classify::ray_surface(&face.surf, &p, &d, -reach, reach) {
                all.push((t, i));
                if super::classify::periodic_candidates(&face.surf, u, v, &polys)
                    .into_iter()
                    .any(|uv| {
                        super::classify::in_face(&polys, &face.surf, uv, tol)
                            != super::classify::InFace::Outside
                    })
                {
                    on.push((t, i));
                }
            }
        }
        let mut ts = if on.is_empty() { all } else { on };
        ts.sort_by(|a, b| a.0.total_cmp(&b.0));
        ts.dedup_by(|a, b| (a.0 - b.0).abs() <= tol);
        Ok(ts)
    }

    /// A feature's limit (`featperform`'s From or Until) as faces: a face, a compound of faces,
    /// or every face of a body.
    fn feat_faces(&mut self, s: &Shape) -> Result<Vec<(BodyId, FaceId)>> {
        match s {
            Shape::Compound(v) => {
                let mut out = Vec::new();
                for x in v {
                    out.extend(self.feat_faces(x)?);
                }
                Ok(out)
            }
            Shape::Body(b) if self.get(*b)?.faces.len() > 1 => Ok((0..self.get(*b)?.faces.len())
                .map(|i| (*b, FaceId(i as u64)))
                .collect()),
            _ => Ok(vec![self.feat_face(s)?]),
        }
    }

    /// Where along the line `c + t·travel` a feature stops at its limit face, of its crossings
    /// `ts` (sorted): where it leaves the base's material (a pocket) or enters it (a boss) at
    /// the face; failing that (the line misses the base there), BRepFeat's choice for a face
    /// off the base: the last crossing for a pocket, the first for a boss.
    fn feat_stop(
        &self,
        base: BodyId,
        c: P3,
        travel: V3,
        ts: &[f64],
        fuse: i64,
        h: f64,
    ) -> Result<Option<f64>> {
        Ok(
            match self.feat_transition(base, c, travel, ts, fuse == 1, h)? {
                Some(t) => Some(t),
                None => if fuse == 1 { ts.first() } else { ts.last() }.copied(),
            },
        )
    }

    /// The first of the crossings `ts` along `c + t·travel` where the line enters the base's
    /// material (`entering`) or leaves it.
    fn feat_transition(
        &self,
        base: BodyId,
        c: P3,
        travel: V3,
        ts: &[f64],
        entering: bool,
        h: f64,
    ) -> Result<Option<f64>> {
        let eps = super::tol::REL_POINT * (1.0 + h);
        for &t in ts {
            let before = crate::Kernel::classify(
                self,
                base,
                c + travel * (t - eps),
                eps * super::tol::REL_POINT,
            )? == PointClass::Inside;
            let after = crate::Kernel::classify(
                self,
                base,
                c + travel * (t + eps),
                eps * super::tol::REL_POINT,
            )? == PointClass::Inside;
            if before != after && after == entering {
                return Ok(Some(t));
            }
        }
        Ok(None)
    }

    /// A solid on one side of a feature's limit surface, and whether it is the side holding `q`
    /// (else the other): a half-space box for a plane, the cylinder or sphere itself for those.
    /// `size` is how far it must reach round `q`.
    fn limit_side(&mut self, surf: &Surface, q: P3, size: f64) -> Result<(BodyId, bool)> {
        match surf {
            Surface::Plane { f } => {
                let n = f.x.cross(&f.y).normalize();
                let z = if (q - f.o).dot(&n) >= 0.0 { n } else { -n };
                let x = f.x.normalize();
                let foot = q - n * (q - f.o).dot(&n);
                let ax = Ax3 {
                    origin: foot - x * size - z.cross(&x) * size,
                    z,
                    x,
                };
                let side = self.box_on(&ax, 2.0 * size, 2.0 * size, size)?;
                Ok((self.body_of(&side)?, true))
            }
            Surface::Cylinder { f, r } => {
                let z = f.z.normalize();
                let w = q - f.o;
                let along = w.dot(&z);
                let side = self.pcylinder_on(
                    &Ax3 {
                        origin: f.o + z * (along - size),
                        z,
                        x: f.x,
                    },
                    *r,
                    2.0 * size,
                )?;
                Ok((self.body_of(&side)?, (w - z * along).norm() < *r))
            }
            Surface::Sphere { f, r } => {
                let side = self.psphere_on(
                    &Ax3 {
                        origin: f.o,
                        z: f.z,
                        x: f.x,
                    },
                    *r,
                )?;
                Ok((self.body_of(&side)?, (q - f.o).norm() < *r))
            }
            Surface::Cone { f, r, a } => {
                // The cone's nappe from its apex (or `size` below q) to `size` above q.
                let z = f.z.normalize();
                let ta = a.tan();
                let radius = |h: f64| r + h * ta;
                let zq = (q - f.o).dot(&z);
                let (mut z0, mut z1) = (zq - size, zq + size);
                if ta.abs() > super::tol::ANGULAR {
                    let apex = -r / ta;
                    if ta > 0.0 {
                        z0 = z0.max(apex)
                    } else {
                        z1 = z1.min(apex)
                    }
                }
                if z1 - z0 <= super::tol::POINT {
                    return Err(KernelError::Unsupported(
                        "features up to a cone past its apex",
                    ));
                }
                let side = self.pcone_on(
                    &Ax3 {
                        origin: f.o + z * z0,
                        z,
                        x: f.x,
                    },
                    radius(z0).max(0.0),
                    radius(z1).max(0.0),
                    z1 - z0,
                )?;
                let w = q - f.o;
                let inside = radius(zq) > 0.0 && (w - z * zq).norm() < radius(zq);
                Ok((self.body_of(&side)?, inside))
            }
            _ => Err(KernelError::Unsupported(
                "features up to faces other than planes, cylinders, cones and spheres",
            )),
        }
    }

    /// `tool` less the side of the limit surface holding `q` (BRepFeat::Tool's solid); `None`
    /// when nothing is left.
    fn feat_cut_side(
        &mut self,
        tool: BodyId,
        surf: &Surface,
        q: P3,
        size: f64,
    ) -> Result<Option<BodyId>> {
        let (side, holds_q) = self.limit_side(surf, q, size)?;
        let op = if holds_q {
            BoolOp::Subtract
        } else {
            BoolOp::Intersect
        };
        match crate::Kernel::boolean(self, op, tool, &[side]) {
            Ok(r) => Ok(Some(r.bodies[0])),
            Err(KernelError::EmptyResult(_)) => Ok(None),
            Err(e) => Err(e),
        }
    }

    /// The solid of a body nearest `p` (the piece of a cut-off feature by its profile).
    fn nearest_piece(&mut self, body: BodyId, p: P3) -> Result<BodyId> {
        let pieces = super::exchange::split_into_solids(self.get(body)?);
        if pieces.len() < 2 {
            return Ok(body);
        }
        let mut best: Option<(f64, BodyId)> = None;
        for piece in pieces {
            let id = self.insert(piece);
            let bb = crate::Kernel::bounding_box(self, id)?;
            let d = (p.coords.sup(&bb.min.coords).inf(&bb.max.coords) - p.coords).norm();
            if best.is_none_or(|(bd, _)| d < bd) {
                best = Some((d, id));
            }
        }
        best.map(|(_, id)| id)
            .ok_or_else(|| err("the feature has no solid"))
    }

    /// The feature's solid along `dir` from `s0` to `s1` off the profile's plane: a prism, or
    /// (`dprism`) a drafted one, its walls leaning by the draft angle about the profile's own
    /// plane (so a drafted prism running both ways tapers on through the profile).
    fn feat_tool(
        &mut self,
        pbase: &Shape,
        kind: &str,
        params: [f64; 6],
        dir: V3,
        s0: f64,
        s1: f64,
    ) -> Result<BodyId> {
        let prism = self.feat_extrude(pbase, dir, s0, ExtrudeEnd::Blind(s1 - s0), false, vec![])?;
        let angle = params[0] * PI / 360.0;
        if kind != "dprism" || angle.abs() < super::tol::ANGULAR {
            return Ok(prism);
        }
        let plane = self.feat_profile(pbase)?.0.plane;
        let n = plane.normal.into_inner();
        // The walls: the faces that aren't square to the profile's normal.
        let walls: Vec<FaceId> = self
            .faces(prism)?
            .into_iter()
            .filter(|f| {
                f.plane
                    .is_none_or(|p| p.normal.dot(&n).abs() < 1.0 - super::tol::ANGULAR)
            })
            .map(|f| f.id)
            .collect();
        // The taper is linear along the profile's normal whichever way the prism runs
        // (LocOpe_DPrism's slanted generator through the profile): pulled along the normal by
        // the angle's sign.
        let pull = nalgebra::Unit::new_normalize(n * params[0].signum());
        let spec = DraftSpec {
            faces: walls,
            angle: -angle,
            pull,
            neutral: plane.origin,
            neutral_normal: None,
            tangent_propagation: false,
        };
        Ok(crate::Kernel::draft(self, prism, &spec)?.bodies[0])
    }

    /// A prism or drafted prism up to (`until`) or between (`from`, `until`) faces of any
    /// surface (BRepFeat's tool cut off by its limit faces' solids): the feature long enough to
    /// reach past everything, cut off past the until face (at the first crossing of the line
    /// through the profile's centre for a boss, the last for a pocket) and before the from
    /// face (its first crossing); the piece by the profile, or between the two, is kept.
    fn feat_limited(
        &mut self,
        base: BodyId,
        pbase: &Shape,
        kind: &str,
        params: [f64; 6],
        dir: V3,
        fuse: i64,
        from: Option<&[(BodyId, FaceId)]>,
        until: &[(BodyId, FaceId)],
    ) -> Result<BodyId> {
        let c = self.feat_centre(pbase)?;
        let bodies: Vec<BodyId> = from
            .unwrap_or(&[])
            .iter()
            .chain(until)
            .map(|l| l.0)
            .collect();
        let h = self.feat_height(base, pbase, &bodies)?;
        let reach = FEAT_REACH * h;
        let tu = self.limit_crossings(until, c, dir, reach)?;
        let same = from == Some(until);
        let sens = if !same && !tu.is_empty() && tu.iter().all(|t| t.0 < 0.0) {
            -1.0
        } else {
            1.0
        };
        let travel = dir * sens;
        let tf = match from {
            Some(f) => self.limit_crossings(f, c, dir, reach)?,
            None => Vec::new(),
        };
        // How far the long feature runs: on from the profile, or both ways for a pair of
        // limits on either side of it.
        let both =
            from.is_some() && (same || (!tf.is_empty() && tf.iter().all(|t| t.0 * sens < 0.0)));
        let (s0, s1) = match (kind, from.is_some(), both) {
            ("dprism", true, _) => (-h, h),
            ("dprism", false, _) => (0.0, h),
            (_, true, true) => (-1.5 * h, 1.5 * h),
            _ => (0.0, 2.0 * h),
        };
        let mut tool = self.feat_tool(pbase, kind, params, travel, s0, s1)?;
        let eps = super::tol::POINT * (1.0 + h);
        let mut ts_u: Vec<(f64, usize)> = tu.iter().map(|&(t, i)| (t * sens, i)).collect();
        ts_u.sort_by(|a, b| a.0.total_cmp(&b.0));
        let times: Vec<f64> = ts_u.iter().map(|t| t.0).collect();
        let tu_stop = self
            .feat_stop(base, c, travel, &times, fuse, h)?
            .ok_or_else(|| err("the feature never meets its until face"))?;
        let iu = ts_u.iter().find(|t| t.0 == tu_stop).map_or(0, |t| t.1);
        let su = self.limit_surface(until[iu])?;
        tool = self
            .feat_cut_side(tool, &su, c + travel * (tu_stop + eps), reach)?
            .ok_or_else(|| err("the feature lies past its until face"))?;
        let mut keep = c;
        if let Some(from) = from {
            let mut ts_f: Vec<(f64, usize)> = tf.iter().map(|&(t, i)| (t * sens, i)).collect();
            ts_f.sort_by(|a, b| a.0.total_cmp(&b.0));
            let times: Vec<f64> = ts_f.iter().map(|t| t.0).collect();
            // It starts where it enters the base's material (a pocket) or leaves it (a boss).
            let first = *times
                .first()
                .ok_or_else(|| err("the feature never meets its from face"))?;
            let tf_start = self
                .feat_transition(base, c, travel, &times, fuse != 1, h)?
                .unwrap_or(first);
            let i = ts_f.iter().find(|t| t.0 == tf_start).map_or(0, |t| t.1);
            let sf = self.limit_surface(from[i])?;
            tool = self
                .feat_cut_side(tool, &sf, c + travel * (tf_start - eps), reach)?
                .ok_or_else(|| err("the feature lies before its from face"))?;
            keep = c + travel * (0.5 * (tf_start + tu_stop));
        }
        self.nearest_piece(tool, keep)
    }

    /// `featprism|featrevol|featdprism base pbase skface …; featperform kind result [from] [until]`
    /// (BRepFeat_MakePrism, MakeRevol, MakeDPrism): the profile face `pbase` (on the base's face
    /// `skface`) swept through everything (no limits: a prism both ways, a revolution all
    /// round), up to `until`, or from `from` to `until`; added to the base or cut from it.
    pub fn featperform(
        &mut self,
        base: &Shape,
        pbase: &Shape,
        skface: &Shape,
        kind: &str,
        params: [f64; 6],
        fuse: i64,
        from: Option<&Shape>,
        until: Option<&Shape>,
    ) -> Result<Shape> {
        let b = self.body_of(base)?;
        // From (or Until) the sketch face itself: the feature up to the other (BRepFeat glues
        // it to the sketch face).
        let (from, until) = match (from, until) {
            (Some(f), Some(u)) if same_face(f, skface) => (None, Some(u)),
            (Some(f), Some(u)) if same_face(u, skface) => (None, Some(f)),
            other => other,
        };
        let tool = match kind {
            "prism" | "dprism" => {
                let dir = if kind == "prism" {
                    V3::new(params[0], params[1], params[2]).normalize()
                } else {
                    self.feat_profile(pbase)?.0.plane.normal.into_inner()
                };
                let fl = from.map(|f| self.feat_faces(f)).transpose()?;
                let ul = until.map(|u| self.feat_faces(u)).transpose()?;
                let simple = fl
                    .iter()
                    .chain(ul.iter())
                    .all(|l| l.len() == 1 && self.plane_of(l[0].0, l[0].1).is_some());
                if !simple {
                    // Up to (or between) curved faces or several faces: the long feature cut
                    // off at them.
                    let ul =
                        ul.ok_or_else(|| err("featperform: a From face needs an Until face"))?;
                    let tool =
                        self.feat_limited(b, pbase, kind, params, dir, fuse, fl.as_deref(), &ul)?;
                    return self.feat_apply(base, tool, fuse);
                }
                if kind == "dprism" {
                    let height = match (from, until) {
                        // Through all: both ways, the draft running on through the profile.
                        (None, None) => {
                            let h = self.feat_height(b, pbase, &[])?;
                            let tool = self.feat_tool(pbase, kind, params, dir, -h, h)?;
                            return self.feat_apply(base, tool, fuse);
                        }
                        // Up to a planar face: its height there.
                        (None, Some(u)) => {
                            let (ub, uf) = self.feat_face(u)?;
                            self.feat_reach(pbase, &dir, ub, uf)?
                        }
                        // Between planar faces: the drafted prism between their heights.
                        (Some(f), Some(u)) => {
                            let ((fb, ff), (ub, uf)) = (self.feat_face(f)?, self.feat_face(u)?);
                            let (d0, d1) = (
                                self.feat_reach(pbase, &dir, fb, ff)?,
                                self.feat_reach(pbase, &dir, ub, uf)?,
                            );
                            let tool =
                                self.feat_tool(pbase, kind, params, dir, d0.min(d1), d0.max(d1))?;
                            return self.feat_apply_limited(base, tool, fuse);
                        }
                        (Some(_), None) => {
                            return Err(err("featperform: a From face needs an Until face"));
                        }
                    };
                    return self.featperformval(base, pbase, kind, params, fuse, height);
                }
                match (from, until) {
                    (None, None) => {
                        // Through all: both ways, three times the base's size (as OCCT).
                        let size = crate::Kernel::bounding_box(self, b)
                            .map(|bb| (bb.max - bb.min).norm())
                            .unwrap_or(1.0)
                            .max(1.0);
                        self.feat_extrude(
                            pbase,
                            dir,
                            0.0,
                            ExtrudeEnd::Blind(3.0 * size),
                            true,
                            vec![],
                        )?
                    }
                    (None, Some(u)) => {
                        let (ub, uf) = self.feat_face(u)?;
                        match self.feat_reach(pbase, &dir, ub, uf) {
                            Ok(d) => self.feat_extrude(
                                pbase,
                                dir,
                                0.0,
                                ExtrudeEnd::Blind(d),
                                false,
                                vec![],
                            )?,
                            Err(KernelError::Unsupported(_)) => match self.feat_extrude(
                                pbase,
                                dir,
                                0.0,
                                ExtrudeEnd::UpToFace {
                                    body: ub,
                                    face: uf,
                                    offset: 0.0,
                                },
                                false,
                                vec![],
                            ) {
                                Ok(t) => t,
                                Err(_) => self.feat_extrude(
                                    pbase,
                                    -dir,
                                    0.0,
                                    ExtrudeEnd::UpToFace {
                                        body: ub,
                                        face: uf,
                                        offset: 0.0,
                                    },
                                    false,
                                    vec![],
                                )?,
                            },
                            Err(e) => return Err(e),
                        }
                    }
                    (Some(f), Some(u)) => {
                        let ((fb, ff), (ub, uf)) = (self.feat_face(f)?, self.feat_face(u)?);
                        let (d0, d1) = (
                            self.feat_reach(pbase, &dir, fb, ff)?,
                            self.feat_reach(pbase, &dir, ub, uf)?,
                        );
                        self.feat_extrude(
                            pbase,
                            dir,
                            d0,
                            ExtrudeEnd::Blind(d1 - d0),
                            false,
                            vec![],
                        )?
                    }
                    (Some(_), None) => {
                        return Err(err("featperform: a From face needs an Until face"));
                    }
                }
            }
            "revol" => {
                let axis = Axis {
                    origin: P3::new(params[0], params[1], params[2]),
                    dir: nalgebra::Unit::try_new(V3::new(params[3], params[4], params[5]), 1e-12)
                        .ok_or_else(|| err("a null axis"))?,
                };
                let (profile, faces) = self.feat_profile(pbase)?;
                let mut spec = RevolveSpec {
                    faces,
                    ..RevolveSpec::solid(axis, None)
                };
                // The turn from the profile's plane to a planar face's through the axis (the next
                // time the turning plane meets it).
                let angle_to = |k: &Self, s: &Shape, from_angle: f64| -> Option<f64> {
                    let (fb, ff) = match s {
                        Shape::BodyFace(b, f) => (*b, *f),
                        _ => return None,
                    };
                    let pl = k.plane_of(fb, ff)?;
                    let (z, nu) = (axis.dir.into_inner(), pl.normal.into_inner());
                    if z.dot(&nu).abs() > 1e-9 || (axis.origin - pl.origin).dot(&nu).abs() > 1e-6 {
                        return None;
                    }
                    let np = profile.plane.normal.into_inner();
                    let th = np.cross(&nu).dot(&z).atan2(np.dot(&nu)).rem_euclid(PI);
                    let th = (th - from_angle).rem_euclid(PI);
                    Some(if th < 1e-9 { PI } else { th })
                };
                if let (None, Some(u)) = (from, until)
                    && let Some(th) = angle_to(self, u, 0.0)
                {
                    spec.full = false;
                    spec.end = RevolveEnd::Angle(th);
                    let tool = crate::Kernel::revolve_with(self, &profile, &spec)?.bodies[0];
                    return self.feat_apply_limited(base, tool, fuse);
                }
                if let (Some(f), Some(u)) = (from, until)
                    && let Some(t0) = angle_to(self, f, 0.0)
                {
                    let t0 = if (t0 - PI).abs() < 1e-9 { 0.0 } else { t0 };
                    let t1 = angle_to(self, u, t0).ok_or(KernelError::Unsupported(
                        "revolved features between faces that aren't planes through the axis",
                    ))?;
                    // The profile turned to the From face, then on to the Until face.
                    let r = nalgebra::Rotation3::from_axis_angle(&axis.dir, t0).into_inner();
                    let turned = self.motion(
                        pbase,
                        &Affine {
                            m: r,
                            t: axis.origin.coords - r * axis.origin.coords,
                        },
                    )?;
                    let (profile, faces) = self.feat_profile(&turned)?;
                    let spec = RevolveSpec {
                        faces,
                        ..RevolveSpec::solid(axis, Some(t1))
                    };
                    let tool = crate::Kernel::revolve_with(self, &profile, &spec)?.bodies[0];
                    return self.feat_apply_limited(base, tool, fuse);
                }
                match (from, until) {
                    (None, None) => crate::Kernel::revolve_with(self, &profile, &spec)?.bodies[0],
                    (None, Some(u)) => {
                        let (ub, uf) = self.feat_face(u)?;
                        spec.full = false;
                        spec.end = RevolveEnd::UpToFace {
                            body: ub,
                            face: uf,
                            offset: 0.0,
                        };
                        match crate::Kernel::revolve_with(self, &profile, &spec) {
                            Ok(r) => r.bodies[0],
                            Err(_) => {
                                spec.axis.dir = -spec.axis.dir;
                                crate::Kernel::revolve_with(self, &profile, &spec)?.bodies[0]
                            }
                        }
                    }
                    _ => {
                        return Err(KernelError::Unsupported(
                            "revolved features between two faces",
                        ));
                    }
                }
            }
            _ => return Err(KernelError::Unsupported("this kind of form feature")),
        };
        if from.is_some() || until.is_some() {
            self.feat_apply_limited(base, tool, fuse)
        } else {
            self.feat_apply(base, tool, fuse)
        }
    }




    /// `featperformval kind result value`: the feature with a length (a prism, `value` along
    /// its direction; a drafted prism, `value` high along the profile's normal with its walls
    /// leaning out by the draft angle) or an angle (a revolution, degrees).
    pub fn featperformval(
        &mut self,
        base: &Shape,
        pbase: &Shape,
        kind: &str,
        params: [f64; 6],
        fuse: i64,
        value: f64,
    ) -> Result<Shape> {
        self.featperformval_from(base, pbase, kind, params, fuse, value, None)
    }



    fn featperformval_from(
        &mut self,
        base: &Shape,
        pbase: &Shape,
        kind: &str,
        params: [f64; 6],
        fuse: i64,
        value: f64,
        start: Option<f64>,
    ) -> Result<Shape> {
        let s0 = start.unwrap_or(0.0);
        let tool = match kind {
            "prism" => {
                let dir = V3::new(params[0], params[1], params[2]).normalize();
                self.feat_extrude(pbase, dir, s0, ExtrudeEnd::Blind(value), false, vec![])?
            }
            "dprism" => {
                let n = self.feat_profile(pbase)?.0.plane.normal.into_inner();
                let prism =
                    self.feat_extrude(pbase, n, s0, ExtrudeEnd::Blind(value), false, vec![])?;
                let angle = params[0] * PI / 360.0;
                if angle.abs() < 1e-12 {
                    prism
                } else {
                    // The walls: the faces that aren't square to the profile's normal.
                    let walls: Vec<FaceId> = self
                        .faces(prism)?
                        .into_iter()
                        .filter(|f| f.plane.is_none_or(|p| p.normal.dot(&n).abs() < 1.0 - 1e-9))
                        .map(|f| f.id)
                        .collect();
                    let o = self.feat_profile(pbase)?.0.plane.origin + n * s0;
                    // (As feat_tool: the taper along the normal by the angle's sign.)
                    let pull = nalgebra::Unit::new_normalize(n * params[0].signum());
                    let spec = DraftSpec {
                        faces: walls,
                        angle: -angle,
                        pull,
                        neutral: o,
                        neutral_normal: None,
                        tangent_propagation: false,
                    };
                    crate::Kernel::draft(self, prism, &spec)?.bodies[0]
                }
            }
            "revol" => {
                let axis = Axis {
                    origin: P3::new(params[0], params[1], params[2]),
                    dir: nalgebra::Unit::try_new(V3::new(params[3], params[4], params[5]), 1e-12)
                        .ok_or_else(|| err("a null axis"))?,
                };
                let (profile, faces) = self.feat_profile(pbase)?;
                let a = value.to_radians();
                let axis = if a < 0.0 {
                    Axis {
                        origin: axis.origin,
                        dir: -axis.dir,
                    }
                } else {
                    axis
                };
                let spec = RevolveSpec {
                    faces,
                    ..RevolveSpec::solid(axis, Some(a.abs().min(TAU)))
                };
                crate::Kernel::revolve_with(self, &profile, &spec)?.bodies[0]
            }
            _ => return Err(KernelError::Unsupported("this kind of form feature")),
        };
        self.feat_apply(base, tool, fuse)
    }

    // --- Booleans and transforms ------------------------------------------------------------

    fn body_of(&self, s: &Shape) -> Result<BodyId> {
        match s {
            Shape::Body(b) => Ok(*b),
            _ => Err(KernelError::Unsupported(
                "booleans of shapes other than solids",
            )),
        }
    }

    /// The edges of a shape that isn't a body's (an edge, a wire, a planar face's loops, a
    /// compound of them), each once.
    fn loose_edges_of(s: &Shape) -> Option<Vec<DEdge>> {
        let mut out: Vec<DEdge> = Vec::new();
        let mut add = |e: &DEdge| {
            let m = e.curve.point(0.5 * (e.t0 + e.t1));
            if !out
                .iter()
                .any(|o| (o.curve.point(0.5 * (o.t0 + o.t1)) - m).norm() <= super::tol::CONFUSION)
            {
                out.push(e.clone());
            }
        };
        match s {
            Shape::Edge(e) => add(e),
            Shape::Wire(w) => w.iter().for_each(add),
            Shape::Face { loops, .. } => loops.iter().flatten().for_each(add),
            Shape::Compound(v) => {
                for x in v {
                    Self::loose_edges_of(x)?.iter().for_each(&mut add);
                }
            }
            _ => return None,
        }
        Some(out)
    }

    /// A shape's edges, when it is edges (a wire, an edge).
    fn edges_of(s: &Shape) -> Option<Vec<super::edge_bop::Piece>> {
        let p = |e: &DEdge| super::edge_bop::Piece {
            curve: e.curve.clone(),
            t0: e.t0,
            t1: e.t1,
        };
        match s {
            Shape::Edge(e) => Some(vec![p(e)]),
            Shape::Wire(w) => Some(w.iter().map(p).collect()),
            _ => None,
        }
    }

    /// A boolean with edges: the pieces of the lower-dimensional argument.
    fn edge_bop(&mut self, op: BoolOp, a: &Shape, b: &Shape) -> Result<Shape> {
        use super::edge_bop::{Other, edge_boolean};
        let (ea, eb) = (Self::edges_of(a), Self::edges_of(b));
        let (edges, other_shape, op) = match (&ea, &eb) {
            (Some(e), _) => (e.clone(), b, op),
            // A solid or sheet with edges: common is symmetric; a cut leaves the solid as it was.
            (None, Some(e)) => match op {
                BoolOp::Intersect => (e.clone(), a, op),
                BoolOp::Subtract => return Ok(a.clone()),
                BoolOp::Union => {
                    return Err(KernelError::Unsupported(
                        "a fuse of arguments of different dimensions",
                    ));
                }
            },
            _ => unreachable!(),
        };
        let body;
        let other = match Self::edges_of(other_shape) {
            Some(es) => {
                body = es;
                Other::Edges(&body)
            }
            None => match other_shape {
                Shape::Body(id) => Other::Body(self.get(*id)?),
                Shape::Face { .. } | Shape::BodyFace(..) | Shape::Compound(_) => {
                    let id = self.solid_of(other_shape)?;
                    Other::Body(self.get(id)?)
                }
                Shape::Empty => Other::Edges(&[]),
                _ => {
                    return Err(KernelError::Unsupported(
                        "booleans of edges with this shape",
                    ));
                }
            },
        };
        // A fuse keeps a wire's pieces where the other argument, a wire too, runs along them;
        // else only where it doesn't (OCCT's compound holds each wire as rebuilt, so a stretch
        // two wires share is in both; a loose edge's shared stretch is the wire's).
        let wire_b = matches!(a, Shape::Wire(_)) && matches!(b, Shape::Wire(_));
        let r = edge_boolean(op, &edges, &other, wire_b).ok_or(KernelError::Unsupported(
            "a fuse of arguments of different dimensions",
        ))?;
        if r.is_empty() {
            return Ok(Shape::Empty);
        }
        Ok(Shape::Wire(
            r.into_iter()
                .map(|p| DEdge {
                    curve: p.curve,
                    t0: p.t0,
                    t1: p.t1,
                })
                .collect(),
        ))
    }

    fn bop(&mut self, op: BoolOp, a: &Shape, b: &Shape) -> Result<Shape> {
        // An empty operand (an empty result, an empty compound): A − ∅ = A ∪ ∅ = A, ∅ ∪ B = B,
        // and the rest empty.
        let empty = |s: &Shape| {
            matches!(s, Shape::Empty) || matches!(s, Shape::Compound(v) if v.is_empty())
        };
        match (empty(a), empty(b), op) {
            (false, true, BoolOp::Subtract | BoolOp::Union) => return Ok(a.clone()),
            (true, false, BoolOp::Union) => return Ok(b.clone()),
            (true, _, _) | (_, true, _) => return Ok(Shape::Empty),
            _ => {}
        }
        if Self::edges_of(a).is_some() || Self::edges_of(b).is_some() {
            return self.edge_bop(op, a, b);
        }
        let (a, b) = (self.solid_of(a)?, self.solid_of(b)?);
        // As OCCT's: faces of the two shapes on one surface stay apart.
        let r = match super::boolean::boolean_with(self, op, a, &[b], false) {
            Err(KernelError::EmptyResult(_)) => return Ok(Shape::Empty),
            r => r?,
        };
        match r.bodies.len() {
            1 => Ok(Shape::Body(r.bodies[0])),
            0 => Ok(Shape::Empty),
            _ => {
                // DRAW keeps a compound: fuse the pieces' properties by summing (one body).
                let mut acc = r.bodies[0];
                for &x in &r.bodies[1..] {
                    acc = self.compound(acc, x)?;
                }
                Ok(Shape::Body(acc))
            }
        }
    }

    /// A solid argument: a body, or a compound of them as one body.
    fn solid_of(&mut self, s: &Shape) -> Result<BodyId> {
        match s {
            Shape::Compound(v) if !v.is_empty() => {
                let mut acc = self.solid_of(&v[0])?;
                for x in &v[1..] {
                    let b = self.solid_of(x)?;
                    acc = self.compound(acc, b)?;
                }
                Ok(acc)
            }
            Shape::Face { .. } => self.sheet_of_face(s),
            // A face of a body: that face alone, as a sheet.
            Shape::BodyFace(b, f) => {
                let sheet = super::kernel::subset(self.get(*b)?, &[f.0 as usize], false);
                Ok(self.insert(sheet))
            }
            _ => self.body_of(s),
        }
    }

    /// A planar face as a one-face sheet body facing along its plane's normal (the bottom of a
    /// unit-thick prism of it, turned over).
    fn sheet_of_face(&mut self, f: &Shape) -> Result<BodyId> {
        let Shape::Face { plane, .. } = f else {
            return Err(err("not a planar face"));
        };
        let profile = face_profile(f)?;
        let prism = self.extrude(&profile, Extent::Blind(1.0))?.bodies[0];
        let body = (**self.get(prism)?).clone();
        self.release(prism);
        let n = plane.normal.into_inner();
        let bottom = body
            .faces
            .iter()
            .position(|face| match &face.surf {
                super::surface::Surface::Plane { f: fr } => {
                    let fz = fr.z * if face.sense { 1.0 } else { -1.0 };
                    fz.dot(&n) < -0.5 && (fr.o - plane.origin).dot(&n).abs() < 1e-9
                }
                _ => false,
            })
            .ok_or_else(|| err("the face's prism has no bottom face"))?;
        let mut sheet = super::kernel::subset(&body, &[bottom], false);
        super::topo::Body::reverse_face(&mut sheet.faces[0]);
        Ok(self.insert(sheet))
    }

    /// Two bodies as one (their solids side by side, as DRAW's compounds).
    fn compound(&mut self, a: BodyId, b: BodyId) -> Result<BodyId> {
        let mut body = (**self.get(a)?).clone();
        let other = (**self.get(b)?).clone();
        let (_, _, fo) = body.append(&other);
        let so = body.shells.len();
        for sh in &other.shells {
            body.shells.push(super::topo::Shell {
                faces: sh.faces.iter().map(|f| f + fo).collect(),
                closed: sh.closed,
            });
        }
        for s in &other.solids {
            body.solids.push(super::topo::Solid {
                shells: s.shells.iter().map(|x| x + so).collect(),
            });
        }
        Ok(self.insert(body))
    }

    /// `bfuse result a b`.
    pub fn bfuse(&mut self, a: &Shape, b: &Shape) -> Result<Shape> {
        self.bop(BoolOp::Union, a, b)
    }


    /// `bcut result a b`.
    pub fn bcut(&mut self, a: &Shape, b: &Shape) -> Result<Shape> {
        self.bop(BoolOp::Subtract, a, b)
    }

    /// `bcommon result a b`.
    pub fn bcommon(&mut self, a: &Shape, b: &Shape) -> Result<Shape> {
        self.bop(BoolOp::Intersect, a, b)
    }

    fn motion(&mut self, s: &Shape, m: &Affine) -> Result<Shape> {
        Ok(match s {
            Shape::Vertex(p) => Shape::Vertex(m.point(p)),
            Shape::Edge(e) => Shape::Edge(DEdge {
                curve: e.curve.transformed(m),
                t0: e.t0,
                t1: e.t1,
            }),
            Shape::Wire(w) => Shape::Wire(
                w.iter()
                    .map(|e| DEdge {
                        curve: e.curve.transformed(m),
                        t0: e.t0,
                        t1: e.t1,
                    })
                    .collect(),
            ),
            Shape::Face { plane, loops } => {
                let o = m.point(&plane.origin);
                let x = m.m * plane.x_dir.into_inner();
                let n = (m.m * plane.y_dir().into_inner()).cross(&x);
                Shape::Face {
                    plane: Plane {
                        origin: o,
                        x_dir: nalgebra::Unit::new_normalize(x),
                        normal: nalgebra::Unit::new_normalize(n),
                    },
                    loops: loops
                        .iter()
                        .map(|l| {
                            l.iter()
                                .map(|e| DEdge {
                                    curve: e.curve.transformed(m),
                                    t0: e.t0,
                                    t1: e.t1,
                                })
                                .collect()
                        })
                        .collect(),
                }
            }
            Shape::Body(b) => {
                let body = self.get(*b)?.transformed(m);
                let nb = self.insert(body);
                if let Some(o) = order_of(*b) {
                    set_order(
                        nb,
                        OcctOrder {
                            edges: o.edges.iter().map(|p| m.point(p)).collect(),
                            faces: o.faces.iter().map(|p| m.point(p)).collect(),
                            degenerate: o.degenerate.clone(),
                        },
                    );
                }
                Shape::Body(nb)
            }
            Shape::Compound(v) => {
                Shape::Compound(v.iter().map(|x| self.motion(x, m)).collect::<Result<_>>()?)
            }
            Shape::Empty => Shape::Empty,
            // A face of a body moves as a sheet of its own; an edge as an edge.
            Shape::BodyFace(b, f) => {
                let sheet = super::kernel::subset(self.get(*b)?, &[f.0 as usize], false);
                Shape::Body(self.insert(sheet.transformed(m)))
            }
            Shape::BodyEdge(b, e) => {
                let ed = self
                    .get(*b)?
                    .edges
                    .get(e.0 as usize)
                    .ok_or_else(|| err("an unknown edge"))?;
                Shape::Edge(DEdge {
                    curve: ed.curve.transformed(m),
                    t0: ed.t[0],
                    t1: ed.t[1],
                })
            }
        })
    }

    /// `ttranslate shape dx dy dz`.
    pub fn ttranslate(&mut self, s: &Shape, dx: f64, dy: f64, dz: f64) -> Result<Shape> {
        self.motion(
            s,
            &Affine {
                m: nalgebra::Matrix3::identity(),
                t: V3::new(dx, dy, dz),
            },
        )
    }

    /// `trotate shape x y z dx dy dz angle` (degrees).
    pub fn trotate(
        &mut self,
        s: &Shape,
        x: f64,
        y: f64,
        z: f64,
        dx: f64,
        dy: f64,
        dz: f64,
        angle: f64,
    ) -> Result<Shape> {
        let axis = nalgebra::Unit::new_normalize(V3::new(dx, dy, dz));
        let r = nalgebra::Rotation3::from_axis_angle(&axis, angle.to_radians()).into_inner();
        let o = V3::new(x, y, z);
        self.motion(s, &Affine { m: r, t: o - r * o })
    }

    /// `tscale shape x y z factor`.
    pub fn tscale(&mut self, s: &Shape, x: f64, y: f64, z: f64, f: f64) -> Result<Shape> {
        let o = V3::new(x, y, z);
        self.motion(
            s,
            &Affine {
                m: nalgebra::Matrix3::identity() * f,
                t: o - o * f,
            },
        )
    }


    // --- Sub-shapes, blends -----------------------------------------------------------------

    /// `explode shape e`: the body's edges (in the kernel's order, not OCCT's).
    pub fn explode_edges(&self, s: &Shape) -> Result<Vec<Shape>> {
        // A face's: its loops' edges in their order (OCCT's for a prism's side: the profile
        // edge, the side edge at its end, the far copy, the side edge at its start).
        if let Shape::BodyFace(b, f) = s {
            let (b, body) = (*b, self.get(*b)?);
            let face = body
                .faces
                .get(f.0 as usize)
                .ok_or_else(|| err("explode: no such face"))?;
            let mut out: Vec<Shape> = Vec::new();
            for c in face.loops.iter().flatten() {
                let e = Shape::BodyEdge(b, EdgeId(c.edge as u64));
                if !body.edges[c.edge].degenerate && !out.iter().any(|x| matches!((x, &e), (Shape::BodyEdge(_, a), Shape::BodyEdge(_, z)) if a == z)) {
                    out.push(e);
                }
            }
            return Ok(out);
        }
        if let Some(es) = Self::loose_edges_of(s) {
            return Ok(es.into_iter().map(Shape::Edge).collect());
        }
        let b = self.body_of(s)?;
        let body = self.get(b)?;
        let mut ids: Vec<usize> = (0..body.edges.len())
            .filter(|&i| !body.edges[i].degenerate)
            .collect();
        // A primitive: OCCT's order, each reference point's nearest edge (by midpoint); its
        // degenerate edges count, as OCCT's do.
        if let Some(o) = order_of(b) {
            if o.edges.len() != ids.len() && o.edges.len() == body.edges.len() {
                ids = (0..body.edges.len()).collect();
            }
            if crate::native::env_flag!("CADRS_ORDER_DEBUG") {
                eprintln!(
                    "explode e: OCCT order has {} edges, the body {} ({} not degenerate)",
                    o.edges.len(),
                    body.edges.len(),
                    ids.len()
                );
            }
            let mid = |i: usize| body.edges[i].point(body.edges[i].mid_t());
            if o.edges.len() == ids.len() {
                ids = o
                    .edges
                    .iter()
                    .map(|p| {
                        *ids.iter()
                            .min_by(|&&x, &&y| (mid(x) - p).norm().total_cmp(&(mid(y) - p).norm()))
                            .unwrap()
                    })
                    .collect();
            } else if o.edges.len() > ids.len() && o.edges.len() <= body.edges.len() {
                // Some of OCCT's degenerate edges (a cone's apex) but not all of ours: each
                // reference point takes the nearest edge not yet taken, when that takes every edge
                // that isn't degenerate.
                let mut taken: Vec<usize> = Vec::new();
                for p in &o.edges {
                    if let Some(i) = (0..body.edges.len())
                        .filter(|i| !taken.contains(i))
                        .min_by(|&x, &y| (mid(x) - p).norm().total_cmp(&(mid(y) - p).norm()))
                    {
                        taken.push(i);
                    }
                }
                if ids.iter().all(|i| taken.contains(i)) {
                    ids = taken;
                } else if o.edges.len() - o.degenerate.len() == ids.len() {
                    // The body has none of them: the order without.
                    let kept: Vec<&P3> = o
                        .edges
                        .iter()
                        .enumerate()
                        .filter(|(i, _)| !o.degenerate.contains(i))
                        .map(|(_, p)| p)
                        .collect();
                    ids = kept
                        .iter()
                        .map(|p| {
                            *ids.iter()
                                .min_by(|&&x, &&y| {
                                    (mid(x) - *p).norm().total_cmp(&(mid(y) - *p).norm())
                                })
                                .unwrap()
                        })
                        .collect();
                }
            } else if o.edges.len() - o.degenerate.len() == ids.len() {
                let kept: Vec<&P3> = o
                    .edges
                    .iter()
                    .enumerate()
                    .filter(|(i, _)| !o.degenerate.contains(i))
                    .map(|(_, p)| p)
                    .collect();
                ids = kept
                    .iter()
                    .map(|p| {
                        *ids.iter()
                            .min_by(|&&x, &&y| {
                                (mid(x) - *p).norm().total_cmp(&(mid(y) - *p).norm())
                            })
                            .unwrap()
                    })
                    .collect();
            }
        }
        Ok(ids
            .into_iter()
            .map(|i| Shape::BodyEdge(b, EdgeId(i as u64)))
            .collect())
    }






    /// `blend result shape r e1 e2 …` (one radius).
    pub fn blend(&mut self, s: &Shape, r: f64, edges: &[&Shape]) -> Result<Shape> {
        let b = self.body_of(s)?;
        let ids = edges
            .iter()
            .map(|e| match e {
                Shape::BodyEdge(eb, id) if *eb == b => Ok(*id),
                _ => Err(err("blend takes edges of the shape")),
            })
            .collect::<Result<Vec<_>>>()?;
        let res = self.fillet(b, &ids, r)?;
        Ok(Shape::Body(res.bodies[0]))
    }





    // --- 2D fillets and chamfers ------------------------------------------------------------




    // --- Properties -------------------------------------------------------------------------



    /// `vprops`: the volume of the shape's solids.
    pub fn vprops(&self, s: &Shape) -> Result<f64> {
        Ok(match s {
            Shape::Body(b) => self.mass_properties(*b)?.volume,
            Shape::Compound(v) => v.iter().map(|x| self.vprops(x)).sum::<Result<f64>>()?,
            _ => 0.0,
        })
    }

    /// The shape healing commands (`fixshape r s prec maxtol`, `fixsmall`, `fixwgaps`,
    /// `sameparameter`, `updatetolerance`, `directfaces`, `DT_SplitAngle`, `DT_ClosedSplit`,
    /// `DT_ShapeDivide`, `DT_ToBspl`, `DT_ShapeConvert`): `ops` run by [`crate::Kernel::heal`]
    /// (`native::heal`). A face is healed as a sheet of its own.
    pub fn heal(&mut self, s: &Shape, ops: &[crate::HealOp]) -> Result<Shape> {
        let b = match s {
            Shape::Body(b) => *b,
            Shape::Face { .. } => self.sheet_of_face(s)?,
            Shape::BodyFace(b, f) => {
                let sub = super::kernel::subset(self.get(*b)?, &[f.0 as usize], false);
                self.insert(sub)
            }
            _ => return Err(KernelError::Unsupported("healing of this kind of shape")),
        };
        let r = crate::Kernel::heal(self, b, ops)?;
        Ok(Shape::Body(r.bodies[0]))
    }




















}


/// A pipe's profile: its plane and loops (a face's outer loop first), and whether they close.
#[derive(Debug, Clone)]
struct Section {
    plane: Plane,
    loops: Vec<Vec<DEdge>>,
    closed: bool,
}

impl Section {
    /// Where OCCT places the section's axis (GeomFill_SectionPlacement): a circle's or an
    /// ellipse's centre, else the mean of points spread evenly in each edge's parameter.
    fn center(&self) -> P3 {
        let l = &self.loops[0];
        if let [e] = &l[..]
            && let Curve::Circle { c, .. } | Curve::Ellipse { c, .. } = &e.curve
        {
            return *c;
        }
        let mut acc = V3::zeros();
        let mut n = 0.0;
        for e in l {
            for i in 0..10 {
                acc += e.curve.point(e.t0 + (e.t1 - e.t0) * i as f64 / 10.0).coords;
                n += 1.0;
            }
        }
        if !self.closed {
            acc += l[l.len() - 1].end().coords;
            n += 1.0;
        }
        P3::from(acc / n)
    }

    /// The point of the section nearest `p`.
    fn nearest(&self, p: &P3) -> P3 {
        let mut best = (f64::INFINITY, *p);
        for e in self.loops.iter().flatten() {
            let q = e
                .curve
                .point(e.curve.closest(p, e.t0.min(e.t1), e.t0.max(e.t1)));
            let d = (q - p).norm();
            if d < best.0 {
                best = (d, q);
            }
        }
        best.1
    }

    /// The section moved rigidly.
    fn moved(&self, m: &Affine) -> Section {
        let plane = Plane {
            origin: m.point(&self.plane.origin),
            x_dir: nalgebra::Unit::new_normalize(m.vector(&self.plane.x_dir)),
            normal: nalgebra::Unit::new_normalize(m.vector(&self.plane.normal)),
        };
        let loops = self
            .loops
            .iter()
            .map(|l| {
                l.iter()
                    .map(|e| DEdge {
                        curve: e.curve.transformed(m),
                        t0: e.t0,
                        t1: e.t1,
                    })
                    .collect()
            })
            .collect();
        Section {
            plane,
            loops,
            ..self.clone()
        }
    }

    /// The section in its plane's coordinates: faces (a closed section; capped only when the
    /// sweep is solid) or one open chain.
    fn pieces(&self, solid: bool) -> Result<(Vec<MergedFace>, Vec<Vec<Piece>>)> {
        use super::profile;
        let to_w = super::build::plane_map(&self.plane, &V3::zeros(), 0.0);
        let mt = to_w.m.transpose();
        let to_l = Affine {
            m: mt,
            t: -(mt * to_w.t),
        };
        let pieces = |l: &[DEdge], li: usize| -> Vec<Piece> {
            l.iter()
                .enumerate()
                .map(|(ci, e)| {
                    let (t0, t1, rev) = if e.t0 <= e.t1 {
                        (e.t0, e.t1, false)
                    } else {
                        (e.t1, e.t0, true)
                    };
                    Piece {
                        c: e.curve.transformed(&to_l),
                        t0,
                        t1,
                        rev,
                        src: Src::Curve {
                            region: 0,
                            curve: crate::unsourced_curve(li, ci),
                        },
                        more: vec![],
                    }
                })
                .collect()
        };
        if !self.closed {
            if solid {
                return Err(err("a solid pipe needs a closed profile"));
            }
            return Ok((
                vec![],
                vec![profile::chain_loop_open(pieces(&self.loops[0], 0))?],
            ));
        }
        let mut lps = Vec::new();
        for (li, l) in self.loops.iter().enumerate() {
            let lp = profile::chain_loop(pieces(l, li))?;
            profile::check_closed(&lp)?;
            // Outer loops counter-clockwise, holes clockwise.
            let a = profile::loop_area(&lp);
            lps.push(if (a < 0.0) == (li == 0) {
                profile::reverse_loop(&lp)
            } else {
                lp
            });
        }
        let outer = lps.remove(0);
        let faces = profile::merge_regions(&[profile::Region2 {
            outer,
            holes: lps,
            source: 0,
        }])?;
        Ok((faces, vec![]))
    }
}

/// A pipe's solid as a sheet: without its planar end caps (the faces in the first and the last
/// section's planes; a closed pipe has none).
fn strip_caps(
    body: super::topo::Body,
    first: &Section,
    last: &Section,
    closed: bool,
) -> super::topo::Body {
    let on = |s: &Section, f: &super::topo::Face| match &f.surf {
        super::surface::Surface::Plane { f: fr } => {
            let n = s.plane.normal.into_inner();
            fr.x.cross(&fr.y).cross(&n).norm() < 1e-9
                && (fr.o - s.plane.origin).dot(&n).abs()
                    < 1e-6 * (1.0 + (fr.o - s.plane.origin).norm())
        }
        _ => false,
    };
    let keep: Vec<usize> = (0..body.faces.len())
        .filter(|&i| closed || !(on(first, &body.faces[i]) || on(last, &body.faces[i])))
        .collect();
    let mut sheet = super::kernel::subset(&body, &keep, false);
    sheet.solids.clear();
    sheet
}

/// The spines of a sweep from fraction `f` of segment `si` of a path (as `sweep::path::spines`):
/// one onward from there (a closed path all the way round), and one back to the start.
fn split_spines(path: Vec<Seg>, closed: bool, si: usize, f: f64) -> Vec<(Vec<Seg>, bool)> {
    let tol = 1e-9;
    let last = path.len() - 1;
    if si == 0 && f < tol {
        return vec![(path, false)];
    }
    if !closed && si == last && f > 1.0 - tol {
        return vec![(path.iter().rev().map(Seg::reversed).collect(), false)];
    }
    let mut before: Vec<Seg> = path[..si].to_vec();
    let mut after: Vec<Seg> = Vec::new();
    if f < tol {
        after.extend(path[si..].iter().cloned());
    } else if f > 1.0 - tol {
        before.push(path[si].clone());
        after.extend(path[si + 1..].iter().cloned());
    } else {
        before.push(path[si].part(0.0, f));
        after.push(path[si].part(f, 1.0));
        after.extend(path[si + 1..].iter().cloned());
    }
    if closed {
        after.extend(before);
        return vec![(after, false)];
    }
    let mut out = Vec::new();
    if !after.is_empty() {
        out.push((after, false));
    }
    if !before.is_empty() {
        out.push((before.iter().rev().map(Seg::reversed).collect(), true));
    }
    out
}



fn l2(a: nalgebra::Point2<f64>, b: nalgebra::Point2<f64>) -> Curve2 {
    Curve2::Line { a, b, source: None }
}



/// Turns edges round so each starts where the one before ends.
fn orient_chain(es: &mut [DEdge]) -> Result<()> {
    if es.len() < 2 {
        return Ok(());
    }
    // The first edge's direction: the one whose end meets the second edge.
    let near = |p: P3, e: &DEdge| (e.start() - p).norm().min((e.end() - p).norm());
    if near(es[0].start(), &es[1]) < near(es[0].end(), &es[1]) {
        es[0] = es[0].reversed();
    }
    for i in 1..es.len() {
        let p = es[i - 1].end();
        if (es[i].end() - p).norm() < (es[i].start() - p).norm() {
            es[i] = es[i].reversed();
        }
        if (es[i].start() - p).norm() > 1e-6 {
            return Err(err("the wire's edges don't connect"));
        }
    }
    Ok(())
}

/// The two edges as a corner: `a` running into the shared point, `b` out of it.
fn meet(a: &DEdge, b: &DEdge) -> Result<(DEdge, DEdge)> {
    for ra in [false, true] {
        for rb in [false, true] {
            let a2 = if ra { a.reversed() } else { a.clone() };
            let b2 = if rb { b.reversed() } else { b.clone() };
            if (a2.end() - b2.start()).norm() < 1e-6 {
                return Ok((a2, b2));
            }
        }
    }
    Err(err("the edges don't share a vertex"))
}

/// How far past the feature's height (BRepFeat's HeightMax) its limit faces are searched and
/// its limit solids reach.
const FEAT_REACH: f64 = 4.0;

/// Whether two shapes name the same face.
fn same_face(a: &Shape, b: &Shape) -> bool {
    matches!((a, b), (Shape::BodyFace(ab, af), Shape::BodyFace(bb, bf)) if ab == bb && af == bf)
}



/// A best-fit plane through a wire's sample points (Newell's normal); `None` if not planar.
fn fit_plane(edges: &[DEdge]) -> Option<Plane> {
    let mut pts: Vec<P3> = Vec::new();
    for e in edges {
        for i in 0..16 {
            pts.push(e.curve.point(e.t0 + (e.t1 - e.t0) * i as f64 / 16.0));
        }
    }
    let n = pts.len();
    let mut nrm = V3::zeros();
    for i in 0..n {
        let (a, b) = (pts[i], pts[(i + 1) % n]);
        nrm += V3::new(
            (a.y - b.y) * (a.z + b.z),
            (a.z - b.z) * (a.x + b.x),
            (a.x - b.x) * (a.y + b.y),
        );
    }
    // A single circle edge: its own normal.
    if edges.len() == 1
        && let Curve::Circle { x, y, .. } = &edges[0].curve
    {
        nrm = x.cross(y)
            * if edges[0].t1 >= edges[0].t0 {
                1.0
            } else {
                -1.0
            };
    }
    let nrm = nrm.try_normalize(1e-12)?;
    let o = pts[0];
    let size = pts.iter().map(|p| (p - o).norm()).fold(0.0, f64::max);
    if pts
        .iter()
        .any(|p| (p - o).dot(&nrm).abs() > (size * 1e-6).max(1e-6))
    {
        return None;
    }
    let xd = ax2_x(&nrm);
    Some(Plane {
        origin: o,
        x_dir: nalgebra::Unit::new_normalize(xd),
        normal: nalgebra::Unit::new_normalize(nrm),
    })
}

/// The face as a kernel profile (lines and circular arcs in its plane).
fn face_profile(f: &Shape) -> Result<Profile> {
    let Shape::Face { plane, loops } = f else {
        return Err(err("not a planar face"));
    };
    let to2 = |p: &P3| {
        let d = p - plane.origin;
        nalgebra::Point2::new(d.dot(&plane.x_dir), d.dot(&plane.y_dir()))
    };
    let n = plane.normal.into_inner();
    let conv = |l: &Vec<DEdge>| -> Result<Loop> {
        let mut curves = Vec::new();
        for e in l {
            match &e.curve {
                Curve::Line { .. } => curves.push(l2(to2(&e.start()), to2(&e.end()))),
                Curve::Circle { c, x, y, r } => {
                    let flip = x.cross(y).dot(&n) < 0.0;
                    let cc = to2(c);
                    let full = (e.t1 - e.t0).abs() >= TAU - 1e-9;
                    if full && l.len() == 1 {
                        curves.push(Curve2::Circle {
                            center: cc,
                            radius: *r,
                            source: None,
                        });
                        continue;
                    }
                    let s = to2(&e.start()) - cc;
                    let start = s.y.atan2(s.x);
                    let sweep = (e.t1 - e.t0) * if flip { -1.0 } else { 1.0 };
                    curves.push(Curve2::Arc {
                        center: cc,
                        radius: *r,
                        start_angle: start,
                        sweep,
                        source: None,
                    });
                }
                Curve::Ellipse { c, x, y, a, b } => {
                    let cc = to2(c);
                    let (xd, yd) = (plane.x_dir.into_inner(), plane.y_dir().into_inner());
                    let x2 = nalgebra::Vector2::new(x.dot(&xd), x.dot(&yd));
                    let flip = x.cross(y).dot(&n) < 0.0;
                    curves.push(Curve2::EllipseArc {
                        center: cc,
                        major_radius: *a,
                        minor_radius: if flip { -*b } else { *b },
                        rotation: x2.y.atan2(x2.x),
                        start: e.t0,
                        sweep: e.t1 - e.t0,
                        source: None,
                    });
                }
                Curve::Nurbs(_) => {
                    // Non-rational, up to cubic: exact cubic Bézier spans.
                    let (lo, hi) = (e.t0.min(e.t1), e.t0.max(e.t1));
                    let mut nc = e.curve.to_nurbs(lo, hi);
                    if e.t1 < e.t0 {
                        nc = nc.reversed();
                    }
                    if nc.is_rational() || nc.degree > 3 {
                        return Err(KernelError::Unsupported(
                            "faces bounded by rational or high-degree B-splines",
                        ));
                    }
                    while nc.degree < 3 {
                        nc = nc.elevated();
                    }
                    for span in bezier_spans(&nc) {
                        let q: Vec<_> = span.iter().map(|h| to2(&super::nurbs::dehom(h))).collect();
                        curves.push(Curve2::Bezier {
                            poles: [q[0], q[1], q[2], q[3]],
                            source: None,
                        });
                    }
                }
            }
        }
        Ok(Loop { curves })
    };
    let outer = conv(&loops[0])?;
    let holes = loops[1..].iter().map(conv).collect::<Result<Vec<_>>>()?;
    Ok(Profile::new(
        *plane,
        vec![Region {
            outer,
            holes,
            source: None,
        }],
    ))
}

/// The Bézier spans of a B-spline (its poles per span, interior knots raised to the degree).
fn bezier_spans(c: &super::nurbs::NurbsCurve) -> Vec<Vec<super::nurbs::H>> {
    let p = c.degree;
    let (a, b) = c.range();
    let eps = 1e-12 * (1.0 + a.abs().max(b.abs()));
    let mut inner: Vec<f64> = Vec::new();
    for &k in &c.knots[p + 1..c.knots.len() - p - 1] {
        if k > a + eps && k < b - eps && inner.last().is_none_or(|l| (k - l).abs() > eps) {
            inner.push(k);
        }
    }
    let mut d = c.clone();
    for &u in &inner {
        while d.knots.iter().filter(|k| (**k - u).abs() <= eps).count() < p {
            d = d.insert_knot(u);
        }
    }
    std::iter::once(a)
        .chain(inner.iter().copied())
        .map(|u| {
            let r = d
                .knots
                .iter()
                .rposition(|k| (*k - u).abs() <= eps)
                .unwrap_or(p)
                .min(d.cps.len() - 1)
                .max(p);
            d.cps[r - p..=r].to_vec()
        })
        .collect()
}

/// A planar primitive for the fillet: a line or a circle in the corner's plane.
#[derive(Clone, Copy)]
enum Prim2 {
    Line {
        p: nalgebra::Point2<f64>,
        d: nalgebra::Vector2<f64>,
    },
    Circle {
        c: nalgebra::Point2<f64>,
        r: f64,
    },
}



fn offset(p: &Prim2, s: f64) -> Prim2 {
    match *p {
        Prim2::Line { p, d } => {
            let nrm = nalgebra::Vector2::new(-d.y, d.x).normalize();
            Prim2::Line { p: p + nrm * s, d }
        }
        Prim2::Circle { c, r } => Prim2::Circle { c, r: r + s },
    }
}



#[cfg(test)]
mod tests {
    use super::*;

    /// Each exploded edge's middle point (a degenerate edge's: its vertex), and whether it is
    /// degenerate.
    fn exploded(k: &NativeKernel, s: &Shape) -> Vec<(P3, bool)> {
        let Shape::Body(b) = s else {
            panic!("not a body")
        };
        let body = k.get(*b).unwrap();
        k.explode_edges(s)
            .unwrap()
            .iter()
            .map(|e| {
                let Shape::BodyEdge(_, id) = e else {
                    panic!("not an edge")
                };
                let ed = &body.edges[id.0 as usize];
                (ed.point(ed.mid_t()), ed.degenerate)
            })
            .collect()
    }

    fn near(p: &P3, q: [f64; 3]) -> bool {
        (p - P3::from(q)).norm() < 1e-6
    }

    /// A wedge whose top narrows to a line (`wedge s 40 100 100 0`): OCCT's 9 edges, its sides'
    /// common edge at the top counted once and the top's zero-length edges left out, so s_5 is
    /// the edge along z at (40, 0) (blend simple_a7).
    #[test]
    fn wedge_narrowed_to_a_line_explodes_as_occt() {
        let mut k = NativeKernel::new();
        let s = k.wedge(40.0, 100.0, 100.0, [0.0, 0.0, 0.0, 100.0]).unwrap();
        let es = exploded(&k, &s);
        assert_eq!(es.len(), 9);
        assert!(near(&es[4].0, [40.0, 0.0, 50.0]), "{:?}", es[4]);
        assert!(near(&es[8].0, [20.0, 0.0, 100.0]), "{:?}", es[8]);
    }

    /// A quarter cone up to its apex (`pcone s 50 0 120 90`): OCCT lists the apex's degenerate
    /// edge first, so s_3 is the base's arc (blend simple_c9).
    #[test]
    fn partial_cone_explodes_with_its_apex_edge() {
        let mut k = NativeKernel::new();
        let s = k.pcone_a(None, 50.0, 0.0, 120.0, 90.0).unwrap();
        let es = exploded(&k, &s);
        assert_eq!(es.len(), 7);
        assert!(es[0].1 && near(&es[0].0, [0.0, 0.0, 120.0]), "{:?}", es[0]);
        let r = std::f64::consts::FRAC_1_SQRT_2 * 50.0;
        assert!(near(&es[2].0, [r, r, 0.0]), "{:?}", es[2]);
    }

    /// A face revolved 270° whose wire runs along the axis: a degenerate edge where each lateral
    /// face meets the axis, the axis edge once (11 edges; blend simple_h4's s_10 is the slant's
    /// copy at the end).
    #[test]
    fn revolution_touching_its_axis_explodes_as_occt() {
        let mut k = NativeKernel::new();
        let w1 = k.ellipse([0.0, 0.0, 0.0], None, None, 15.0, 10.0);
        let w1 = k.mkedge(&w1, 0.0, std::f64::consts::FRAC_PI_2);
        let w1 = k.trotate(&w1, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 90.0).unwrap();
        let w2 = k
            .polyline(&[
                [0.0, 0.0, 10.0],
                [0.0, 0.0, -10.0],
                [25.0, 0.0, -10.0],
                [15.0, 0.0, 0.0],
            ])
            .unwrap();
        let w = k.wire(&[&w1, &w2]).unwrap();
        let w = k.mkplane(&w).unwrap();
        let s = k.revol(&w, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 270.0).unwrap();
        let es = exploded(&k, &s);
        assert_eq!(es.len(), 11);
        assert!(es[1].1 && es[4].1 && !es[9].1);
        assert!(near(&es[9].0, [0.0, -20.0, -5.0]), "{:?}", es[9]);
    }

    /// A triangle piped along a quarter circle, its face looking back along the spine: OCCT
    /// explores the loop backwards from its first edge, start face first, so s_4 is the edge
    /// swept from the triangle's first vertex (blend simple_i1).
    #[test]
    fn pipe_explodes_as_occt() {
        let mut k = NativeKernel::new();
        let w = k.circle(-20.0, 0.0, 0.0, 20.0);
        let w = k.mkedge(&w, 0.0, std::f64::consts::FRAC_PI_2);
        let w = k.wire(&[&w]).unwrap();
        let profile = k
            .polyline(&[
                [0.0, 0.0, 0.0],
                [10.0, 0.0, 0.0],
                [5.0, 0.0, 5.0],
                [0.0, 0.0, 0.0],
            ])
            .unwrap();
        let profile = k.mkplane(&profile).unwrap();
        let s = k.pipe(&w, &profile).unwrap();
        let es = exploded(&k, &s);
        assert_eq!(es.len(), 9);
        assert!(
            near(&es[0].0, [5.0, 0.0, 0.0]) && near(&es[1].0, [2.5, 0.0, 2.5]),
            "{:?}",
            &es[..2]
        );
        let h = 20.0 * std::f64::consts::FRAC_1_SQRT_2;
        assert!(near(&es[3].0, [h - 20.0, h, 0.0]), "{:?}", es[3]);
    }

    /// A script's sub-shape past the end of an explode is a "topology mismatch" failure, not an
    /// index out of bounds (review 4: 17 blend known failures panicked in their test code).
    #[test]
    fn sub_past_the_end_is_a_topology_mismatch() {
        let subs = vec![Shape::Vertex(P3::origin()), Shape::Empty];
        assert!(matches!(sub(&subs, 2, "s", "edges"), Ok(Shape::Empty)));
        let e = sub(&subs, 7, "s", "edges").unwrap_err().to_string();
        assert!(
            e.contains(
                "topology mismatch: explode s gave 2 edges, the script expects at least 7 (s_7)"
            ),
            "{e}"
        );
        assert!(sub(&subs, 0, "s", "edges").is_err());
    }

    /// A known failure passes the test by failing with its error, or with the panic it lists.
    #[test]
    fn known_failure_that_fails_or_panics_as_listed() {
        known_failure("g c", Ok(Err(err("the area differs"))), None);
        known_failure(
            "g c",
            Err(Box::new("index out of bounds: the len is 6")),
            Some("index out of bounds"),
        );
    }

    #[test]
    #[should_panic(expected = "a known failure now passes: remove `g c`")]
    fn known_failure_that_passes_is_stale() {
        known_failure("g c", Ok(Ok(())), None);
    }

    #[test]
    #[should_panic(expected = "a known failure panicked: g c: index out of bounds")]
    fn known_failure_that_panics_fails_loudly() {
        known_failure(
            "g c",
            Err(Box::new(String::from("index out of bounds: the len is 6"))),
            None,
        );
    }
}
