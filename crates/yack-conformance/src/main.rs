// yack: the brep-conformance adapter; each protocol function is one yack call (PLAN §5.1).
// Adapted from brep-conformance-springrep@92ee43f src/main.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: every springrep call replaced by its one yack call; the forms moved to convert.rs.
//! Answers the brep-conformance protocol on stdin and stdout with yack. It does only the
//! contract's four things besides calling yack: translate form, address by point, the empty
//! solid (`make::empty`) and handles (yack's solids are immutable values).

mod convert;

use std::process::ExitCode;

use brep_protocol::{Display, DisplayFace, Kernel, Meeting, Mesh, Projection, Refusal, model};
use brep_protocol::{model::Primitive, profile};
use convert::{edges, faces, frame, pairs, path, r, sketch, spline};
use yack::{Affine, Plane, Ray, Rigid, Side, Similarity, Solid, make, rad};

type V = model::Vec3;
type Res<T> = Result<T, Refusal>;

/// Where to log each returned solid's digest (`YACK_DIGESTS`, read once), for the M1.6
/// bit-identical gate (`cargo xtask digest`). Unset: nothing is written.
fn digest_log() -> Option<&'static std::path::PathBuf> {
    static PATH: std::sync::OnceLock<Option<std::path::PathBuf>> = std::sync::OnceLock::new();
    #[allow(clippy::disallowed_methods)] // tool crate: the digest gate's opt-in switch (M1.6)
    PATH.get_or_init(|| std::env::var_os("YACK_DIGESTS").map(std::path::PathBuf::from)).as_ref()
}

/// Appends `digest <hex>` for `s` to the digest log, if one is set, then `strategy <rung>`: the
/// ladder strategy that built it (M1.10). `xtask digest` reads only the `digest` lines.
fn log_digest(s: &Solid) {
    use std::io::Write;
    let Some(path) = digest_log() else { return };
    let hex: String = s.digest().iter().map(|b| format!("{b:02x}")).collect();
    if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
        let _ = writeln!(f, "digest {hex}");
        let _ = writeln!(f, "strategy {}", s.strategy());
        let _ = f.flush();
    }
}

/// A solid returned to the protocol, its digest logged.
fn rec(r: Res<Solid>) -> Res<Solid> {
    if let Ok(s) = &r {
        log_digest(s);
    }
    r
}

/// Two solids returned to the protocol (`split`), front then back, their digests logged.
fn rec2(r: Res<(Solid, Solid)>) -> Res<(Solid, Solid)> {
    if let Ok((a, b)) = &r {
        log_digest(a);
        log_digest(b);
    }
    r
}

struct Yack;

fn arrays(ps: Vec<yack::Point3>) -> Vec<V> {
    ps.into_iter().map(yack::Point3::to_array).collect()
}

fn plane(origin: V, normal: V) -> Res<Plane> {
    Plane::new(origin, normal).map_err(r)
}

#[allow(clippy::redundant_closure_call)] // each Solid-returning method wraps its body for the digest log
impl Kernel for Yack {
    type Solid = Solid;

    fn name(&self) -> String {
        "yack".into()
    }
    fn version(&self) -> String {
        format!("{} {}", yack::VERSION, yack::SOURCE)
    }
    fn supports(&self) -> Vec<String> {
        yack::SUPPORTS.iter().map(|s| (*s).to_string()).collect()
    }
    fn tolerance(&self) -> f64 {
        yack::tol::LINEAR
    }

    fn make(&self, p: &Primitive) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            let f = &frame(&p.frame())?;
            match p {
                Primitive::Box { size, .. } => make::block(f, *size),
                Primitive::Cylinder { radius, height, .. } => make::cylinder(f, *radius, *height),
                Primitive::Sphere { radius, .. } => make::sphere(f, *radius),
                Primitive::Torus { major, minor, .. } => make::torus(f, *major, *minor),
                Primitive::Cone { bottom, top, height, .. } => make::cone(f, *bottom, *top, *height),
                Primitive::Extrude { profile, height, .. } => {
                    make::extrude(f, &sketch(profile)?, *height)
                }
                Primitive::Revolve { profile, angle: a, .. } => {
                    make::revolve(f, &sketch(profile)?, rad(*a))
                }
                Primitive::Screw { profile, rise, angle: a, .. } => {
                    make::screw(f, &sketch(profile)?, *rise, rad(*a))
                }
                Primitive::Sweep { profile, path: p, .. } => {
                    make::sweep(f, &sketch(profile)?, &path(p))
                }
                Primitive::SweepPath { profile, path: p, twist, .. } => {
                    make::sweep_path(f, &sketch(profile)?, &spline(p)?, rad(*twist))
                }
                Primitive::Slab { size, knots_u, knots_v, heights, .. } => {
                    make::slab(f, *size, knots_u, knots_v, heights)
                }
                Primitive::Loft { profile, scale, height, top, .. } => {
                    make::loft(f, &sketch(profile)?, *scale, *height, &sketch(top)?)
                }
            }
            .map_err(r)
        })())
    }
    fn loft_through(&self, f: &model::Frame, ps: &[profile::Profile], hs: &[f64]) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            let sections = ps.iter().map(sketch).collect::<Res<Vec<_>>>()?;
            let sections = pairs(sections, hs, "heights")?.into_iter().map(|(p, h)| (h, p));
            make::loft_through(&frame(f)?, &sections.collect::<Vec<_>>()).map_err(r)
        })())
    }

    // ---- booleans ----
    fn union(&self, a: &Solid, b: &Solid) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            a.union(b).map_err(r)
        })())
    }
    fn intersection(&self, a: &Solid, b: &Solid) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            a.intersection(b).map_err(r)
        })())
    }
    fn difference(&self, a: &Solid, b: &Solid) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            a.difference(b).map_err(r)
        })())
    }
    fn union_all(&self, solids: &[&Solid]) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            Solid::union_all(&solids.iter().map(|&s| s.clone()).collect::<Vec<_>>()).map_err(r)
        })())
    }
    fn difference_all(&self, target: &Solid, tools: &[&Solid]) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            target.difference_all(&tools.iter().map(|&s| s.clone()).collect::<Vec<_>>()).map_err(r)
        })())
    }
    fn split(&self, s: &Solid, origin: V, normal: V) -> Res<(Solid, Solid)> {
        rec2((|| -> Res<(Solid, Solid)> {
            let parts = s.split(&plane(origin, normal)?).map_err(r)?;
            Ok((parts.front, parts.back))
        })())
    }

    // ---- blends ----
    fn fillet(&self, s: &Solid, at: &[V], radius: f64) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            s.fillet(&edges(s, at)?, radius).map_err(r)
        })())
    }
    fn fillet_radii(&self, s: &Solid, at: &[V], radii: &[f64]) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            s.fillet_radii(&pairs(edges(s, at)?, radii, "radii")?).map_err(r)
        })())
    }
    fn fillet_variable(&self, s: &Solid, e: V, from: V, r_from: f64, r_to: f64) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            s.fillet_variable(s.edge_at(e).map_err(r)?, from, r_from, r_to).map_err(r)
        })())
    }
    fn fillet_conic(&self, s: &Solid, at: &[V], setback: f64, rho: f64) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            s.fillet_conic(&edges(s, at)?, setback, rho).map_err(r)
        })())
    }
    fn fillet_g2(&self, s: &Solid, at: &[V], offset: f64, center: f64) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            s.fillet_g2(&edges(s, at)?, offset, center).map_err(r)
        })())
    }
    fn fillet_setback(&self, s: &Solid, at: &[V], radius: f64, setback: f64) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            s.fillet_setback(&edges(s, at)?, radius, setback).map_err(r)
        })())
    }
    fn fillet_setback_g2(&self, s: &Solid, at: &[V], radius: f64, setback: f64) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            s.fillet_setback_g2(&edges(s, at)?, radius, setback).map_err(r)
        })())
    }
    fn fillet_faces(&self, s: &Solid, a: V, b: V, radius: f64) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            s.fillet_faces(s.face_at(a).map_err(r)?, s.face_at(b).map_err(r)?, radius).map_err(r)
        })())
    }
    fn full_round(&self, s: &Solid, a: V, middle: V, b: V) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            let [a, m, b] = [a, middle, b].map(|p| s.face_at(p).map_err(r));
            s.full_round(a?, m?, b?).map_err(r)
        })())
    }
    fn chamfer(&self, s: &Solid, at: &[V], distance: f64) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            s.chamfer(&edges(s, at)?, distance).map_err(r)
        })())
    }
    fn chamfer_two(&self, s: &Solid, e: V, f: V, d1: f64, d2: f64) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            s.chamfer_two(s.edge_at(e).map_err(r)?, s.face_at(f).map_err(r)?, d1, d2).map_err(r)
        })())
    }
    fn chamfer_distances(&self, s: &Solid, at: &[V], distances: &[f64]) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            s.chamfer_distances(&pairs(edges(s, at)?, distances, "distances")?).map_err(r)
        })())
    }

    // ---- edits ----
    fn offset(&self, s: &Solid, distance: f64) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            s.offset(distance).map_err(r)
        })())
    }
    fn offset_faces(&self, s: &Solid, at: &[V], distance: f64) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            s.offset_faces(&faces(s, at)?, distance).map_err(r)
        })())
    }
    fn delete_faces(&self, s: &Solid, at: &[V]) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            s.delete_faces(&faces(s, at)?).map_err(r)
        })())
    }
    fn move_faces(&self, s: &Solid, at: &[V], m: [[f64; 4]; 3]) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            s.move_faces(&faces(s, at)?, &Rigid::from_rows(m).map_err(r)?).map_err(r)
        })())
    }
    fn thicken(&self, s: &Solid, at: &[V], thickness: f64) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            s.thicken(&faces(s, at)?, thickness).map_err(r)
        })())
    }
    fn shell(&self, s: &Solid, open: &[V], thickness: f64) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            s.shell(&faces(s, open)?, thickness).map_err(r)
        })())
    }
    fn draft(&self, s: &Solid, at: &[V], origin: V, pull: V, a: f64) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            s.draft(&faces(s, at)?, origin, pull, rad(a)).map_err(r)
        })())
    }
    fn transform(&self, s: &Solid, m: [[f64; 4]; 3]) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            s.transform(&Similarity::from_rows(m).map_err(r)?).map_err(r)
        })())
    }
    fn affine(&self, s: &Solid, m: [[f64; 4]; 3]) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            s.transform_affine(&Affine::from_rows(m).map_err(r)?).map_err(r)
        })())
    }

    // ---- exchange ----
    fn write_step(&self, s: &Solid) -> Res<String> {
        yack::step::write(s).map_err(r)
    }
    fn read_step(&self, text: &str) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            yack::step::read(text).map_err(r)
        })())
    }
    fn sew(&self, text: &str, tolerance: f64) -> Res<Solid> {
        rec((|| -> Res<Solid> {
            yack::step::sew(text, tolerance).map_err(r)
        })())
    }

    // ---- queries ----
    fn volume(&self, s: &Solid) -> Res<f64> {
        Ok(s.volume())
    }
    fn area(&self, s: &Solid) -> Res<f64> {
        Ok(s.area())
    }
    fn check(&self, s: &Solid) -> Res<Vec<String>> {
        Ok(s.check().into_iter().map(|i| format!("{}: {}", i.code, i.detail)).collect())
    }
    fn classify(&self, s: &Solid, at: &[V]) -> Res<Vec<model::Side>> {
        let side = |x: Side| match x {
            Side::In => model::Side::In,
            Side::On => model::Side::On,
            Side::Out => model::Side::Out,
        };
        Ok(s.classify(at).map_err(r)?.into_iter().map(side).collect())
    }
    fn closest(&self, s: &Solid, at: &[V]) -> Res<Vec<V>> {
        Ok(arrays(s.closest(at).map_err(r)?))
    }
    fn normal(&self, s: &Solid, at: &[V]) -> Res<Vec<Option<V>>> {
        Ok(s.normal_at(at).into_iter().map(|n| n.map(yack::Vec3::to_array)).collect())
    }
    fn curvature(&self, s: &Solid, at: &[V]) -> Res<Vec<Option<[f64; 2]>>> {
        Ok(s.curvature_at(at))
    }
    fn centroid(&self, s: &Solid) -> Res<Option<V>> {
        Ok(s.centroid().map(yack::Point3::to_array))
    }
    fn inertia(&self, s: &Solid) -> Res<[[f64; 3]; 3]> {
        Ok(s.inertia())
    }
    fn bounding_box(&self, s: &Solid) -> Res<[V; 2]> {
        let b = s.bounding_box().ok_or_else(|| Refusal::invalid("the empty solid has no box"))?;
        Ok([b.min.to_array(), b.max.to_array()])
    }
    fn betti(&self, s: &Solid) -> Res<[u64; 3]> {
        Ok(s.betti())
    }
    fn distance(&self, a: &Solid, b: &Solid) -> Res<f64> {
        a.distance(b).map_err(r)
    }
    fn interference(&self, a: &Solid, b: &Solid) -> Res<Meeting> {
        let m = a.interference(b).map_err(r)?;
        Ok(Meeting { volume: m.volume, gap: m.gap })
    }
    fn section_area(&self, s: &Solid, origin: V, normal: V) -> Res<f64> {
        s.section_area(&plane(origin, normal)?).map_err(r)
    }
    fn section(&self, s: &Solid, origin: V, normal: V, chord: f64) -> Res<Vec<Vec<V>>> {
        let pieces = s.section(&plane(origin, normal)?, chord).map_err(r)?;
        Ok(pieces.into_iter().map(arrays).collect())
    }
    fn project(&self, s: &Solid, direction: V, chord: f64) -> Res<Projection> {
        let d = s.project(direction, chord).map_err(r)?;
        let flat = |ls: Vec<Vec<yack::Point2>>| {
            ls.into_iter().map(|l| l.into_iter().map(yack::Point2::to_array).collect()).collect()
        };
        Ok(Projection::new(flat(d.visible), flat(d.hidden)))
    }
    fn raycast(&self, s: &Solid, origin: V, direction: V) -> Res<Vec<f64>> {
        Ok(self.raycasts(s, &[(origin, direction)])?.into_iter().flatten().collect())
    }
    fn raycasts(&self, s: &Solid, rays: &[(V, V)]) -> Res<Vec<Vec<f64>>> {
        let rays = rays.iter().map(|&(o, d)| Ray::new(o, d).map_err(r)).collect::<Res<Vec<_>>>()?;
        s.raycast(&rays).map_err(r)
    }
    fn mesh(&self, s: &Solid, chord: f64) -> Res<Mesh> {
        let m = s.mesh(chord).map_err(r)?;
        Ok(Mesh::new(arrays(m.positions), m.triangles))
    }
    fn display(&self, s: &Solid, chord: f64) -> Res<Display> {
        let d = s.display(chord).map_err(r)?;
        let face = |f: yack::DisplayFace| {
            let normals = f.normals.into_iter().map(yack::Vec3::to_array).collect();
            DisplayFace::new(arrays(f.positions), normals, f.triangles)
        };
        Ok(Display::new(
            d.faces.into_iter().map(face).collect(),
            d.edges.into_iter().map(arrays).collect(),
        ))
    }
}

/// The bake-off build (PLAN §3.5) runs one engine alone when `YACK_ENGINE` names it; unset or
/// anything else keeps the ladder. Read once, before the first request.
#[cfg(feature = "bakeoff")]
fn force_engine() {
    use yack::bakeoff::{Engine, force};
    #[allow(clippy::disallowed_methods)] // tool crate: the bake-off's opt-in switch, read once
    match std::env::var("YACK_ENGINE").as_deref() {
        Ok("springrep") => force(Engine::Springrep),
        Ok("cadrs") => force(Engine::Cadrs),
        _ => {}
    }
}

fn main() -> ExitCode {
    #[cfg(feature = "bakeoff")]
    force_engine();
    match brep_protocol::serve(&Yack) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("yack-conformance: {e}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests;
