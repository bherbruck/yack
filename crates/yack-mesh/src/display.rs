// yack: What a viewer draws: each face meshed as its own triangles, with the exact surface's normal at
// Adapted from springrep@a03e8a1 src/mesh/display.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/mesh/display.rs to yack-mesh (M3 W6); paths and visibility only.
//! What a viewer draws: each face meshed as its own triangles, with the exact surface's normal at
//! every node, and the solid's drawn edges as polylines. Faces share no nodes, so shading is
//! smooth within a face and creased where two faces meet. At a pole or an apex each triangle has
//! its own node, so a cone shades along each ruling up to its point. Faces come in the order of the
//! solid's shells, the order the STEP writer writes them; edges leave out seams, poles and the
//! edges between faces on one surface, which are no creases.
//!
//! [`glb`] packs the faces as one binary glTF mesh in millimetres, z up: `POSITION`, `NORMAL`, a
//! `_FACE` attribute holding each node's face index, the triangles, and `COLOR_0` when each face
//! is given a colour.

#[allow(clippy::disallowed_types)] use std::collections::{BTreeMap, HashMap}; // D26: BTree* in hardening, proven by digests

use super::{MeshOptions, edge_params, tessellate};
use crate::{
    Result,
    intersect::ssi::same_domain,
    math::Point3,
    tol,
    topo::{Brep, EdgeId, FaceId},
};

/// The face mesh's largest distance from the surface, in mm.
pub const DEFLECTION_MM: f64 = 0.03;
/// The face mesh's largest turn of the normal across a triangle, in radians.
pub const ANGLE: f64 = 0.2;
/// The edge polylines' largest distance from their curves, in mm.
pub const EDGE_DEFLECTION_MM: f64 = 0.005;

/// One face as drawn: its nodes, their normals, and its triangles over them.
#[derive(Clone, Debug, Default)]
pub struct Face {
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub triangles: Vec<[u32; 3]>,
}

/// A solid as drawn.
#[derive(Clone, Debug, Default)]
pub struct Display {
    pub faces: Vec<Face>,
    /// The face of the solid each of `faces` draws.
    pub ids: Vec<FaceId>,
    /// Each drawn edge as a polyline.
    pub edges: Vec<Vec<Point3>>,
    /// For each drawn edge, the indices in `faces` of the faces it joins.
    pub edge_faces: Vec<Vec<usize>>,
}

/// The faces in the order of the solid's shells.
fn shell_order(b: &Brep) -> Vec<FaceId> {
    let mut out = Vec::new();
    for s in b.solid_ids() {
        for &sh in &b.solid(s).shells {
            out.extend(b.shell(sh).faces.iter().copied());
        }
    }
    out
}

fn f32s(p: Point3) -> [f32; 3] {
    [p.x as f32, p.y as f32, p.z as f32]
}

/// `b` as a viewer draws it.
pub fn display(b: &Brep) -> Result<Display> {
    display_within(b, DEFLECTION_MM, EDGE_DEFLECTION_MM)
}

/// `b` as a viewer draws it, its faces within `linear` mm of the surface and its edges within
/// `edge_linear` mm of their curves, each turning by at most [`ANGLE`] across a piece.
#[allow(clippy::disallowed_types)] pub fn display_within(b: &Brep, linear: f64, edge_linear: f64) -> Result<Display> { // D26: BTree* in hardening, proven by digests
    let mesh = tessellate(
        b,
        &MeshOptions {
            linear,
            angular: ANGLE,
        },
    )?;
    let ranges: HashMap<FaceId, std::ops::Range<usize>> = mesh.faces.iter().cloned().collect();
    let order = shell_order(b);
    let index: HashMap<FaceId, usize> = order.iter().enumerate().map(|(k, &f)| (f, k)).collect();
    let mut faces = Vec::with_capacity(order.len());
    for &f in &order {
        let face = b.face(f);
        let mut out = Face::default();
        let mut local: BTreeMap<u32, u32> = BTreeMap::new();
        for tri in ranges
            .get(&f)
            .map_or(&[][..], |r| &mesh.triangles[r.clone()])
        {
            let mut t = [0u32; 3];
            for (k, &g) in tri.iter().enumerate() {
                let p = mesh.positions[g as usize];
                let uv = face.surface.project(p);
                let (_, su, sv) = face.surface.d1(uv.x, uv.y);
                let outward = |u: f64, v: f64| {
                    let n = face.surface.normal(u, v);
                    f32s(if face.reversed { -n } else { n })
                };
                // At a pole or an apex one parameter moves the point nowhere, and the normal can
                // turn with the other: there the triangle takes its own node, with the normal
                // along the triangle's middle, at the far side's midpoint's parameter.
                if su.norm() <= tol::LINEAR || sv.norm() <= tol::LINEAR {
                    let far =
                        [tri[(k + 1) % 3], tri[(k + 2) % 3]].map(|j| mesh.positions[j as usize]);
                    let mid = face.surface.project(far[0].lerp(far[1], 0.5));
                    let n = if su.norm() <= sv.norm() {
                        outward(mid.x, uv.y)
                    } else {
                        outward(uv.x, mid.y)
                    };
                    out.positions.push(f32s(p));
                    out.normals.push(n);
                    t[k] = (out.positions.len() - 1) as u32;
                    continue;
                }
                t[k] = *local.entry(g).or_insert_with(|| {
                    out.positions.push(f32s(p));
                    out.normals.push(outward(uv.x, uv.y));
                    (out.positions.len() - 1) as u32
                });
            }
            out.triangles.push(t);
        }
        faces.push(out);
    }
    let mut uses: HashMap<EdgeId, Vec<FaceId>> = HashMap::new();
    for f in b.face_ids() {
        for &lp in &b.face(f).loops {
            for &c in &b.lp(lp).coedges {
                uses.entry(b.coedge(c).edge).or_default().push(f);
            }
        }
    }
    let edge_opt = MeshOptions {
        linear: edge_linear,
        angular: ANGLE,
    };
    let mut edges = Vec::new();
    let mut edge_faces = Vec::new();
    for e in b.edge_ids() {
        let x = b.edge(e);
        let Some(curve) = &x.curve else { continue };
        let two = uses.get(&e).cloned().unwrap_or_default();
        // A seam is one face meeting itself; two faces on one surface make no crease.
        if two.len() == 2
            && (two[0] == two[1] || same_domain(&b.face(two[0]).surface, &b.face(two[1]).surface))
        {
            continue;
        }
        let ts = edge_params(curve, x.range.0, x.range.1, &edge_opt);
        edges.push(ts.iter().map(|&t| curve.eval(t)).collect());
        let mut ks: Vec<usize> = two.iter().filter_map(|f| index.get(f).copied()).collect();
        ks.dedup();
        edge_faces.push(ks);
    }
    Ok(Display {
        faces,
        ids: order,
        edges,
        edge_faces,
    })
}

/// The faces as one binary glTF mesh, `colors` (`#rrggbb`, one per face) giving `COLOR_0`.
pub fn glb(faces: &[Face], colors: Option<&[String]>) -> Vec<u8> {
    let mut pos: Vec<[f32; 3]> = Vec::new();
    let mut nor: Vec<[f32; 3]> = Vec::new();
    let mut ids: Vec<f32> = Vec::new();
    let mut idx: Vec<u32> = Vec::new();
    let mut rgba: Vec<[u8; 4]> = Vec::new();
    for (k, f) in faces.iter().enumerate() {
        let base = pos.len() as u32;
        pos.extend(&f.positions);
        nor.extend(&f.normals);
        ids.extend(std::iter::repeat_n(k as f32, f.positions.len()));
        idx.extend(f.triangles.iter().flat_map(|t| t.map(|i| i + base)));
        if let Some(c) = colors.and_then(|c| c.get(k)) {
            let hex =
                |i: usize| u8::from_str_radix(c.get(i..i + 2).unwrap_or("b4"), 16).unwrap_or(180);
            rgba.extend(std::iter::repeat_n(
                [hex(1), hex(3), hex(5), 255],
                f.positions.len(),
            ));
        }
    }
    let mut bin: Vec<u8> = Vec::new();
    let mut views = Vec::new();
    let mut accessors = Vec::new();
    let mut add = |bytes: Vec<u8>, kind: &str, count: usize, comp: u32, target: u32| {
        let offset = bin.len();
        bin.extend(&bytes);
        while !bin.len().is_multiple_of(4) {
            bin.push(0);
        }
        views.push(format!(
            r#"{{"buffer":0,"byteOffset":{offset},"byteLength":{},"target":{target}}}"#,
            bytes.len()
        ));
        accessors.push((views.len() - 1, comp, count, kind.to_string()));
        accessors.len() - 1
    };
    let le3 = |v: &[[f32; 3]]| {
        v.iter()
            .flat_map(|p| p.iter().flat_map(|c| c.to_le_bytes()))
            .collect()
    };
    let a_pos = add(le3(&pos), "VEC3", pos.len(), 5126, 34962);
    let a_nor = add(le3(&nor), "VEC3", nor.len(), 5126, 34962);
    let a_face = add(
        ids.iter().flat_map(|c| c.to_le_bytes()).collect(),
        "SCALAR",
        ids.len(),
        5126,
        34962,
    );
    let a_idx = add(
        idx.iter().flat_map(|c| c.to_le_bytes()).collect(),
        "SCALAR",
        idx.len(),
        5125,
        34963,
    );
    let a_color = (!rgba.is_empty()).then(|| {
        add(
            rgba.iter().flatten().copied().collect(),
            "VEC4",
            rgba.len(),
            5121,
            34962,
        )
    });
    let (mut lo, mut hi) = ([f32::INFINITY; 3], [f32::NEG_INFINITY; 3]);
    for p in &pos {
        for i in 0..3 {
            lo[i] = lo[i].min(p[i]);
            hi[i] = hi[i].max(p[i]);
        }
    }
    let num = |v: f32| {
        if v.is_finite() {
            format!("{v}")
        } else {
            "0".to_string()
        }
    };
    let acc_json: Vec<String> = accessors
        .iter()
        .enumerate()
        .map(|(i, (view, comp, count, kind))| {
            let mut s = format!(
                r#"{{"bufferView":{view},"componentType":{comp},"count":{count},"type":"{kind}""#
            );
            if i == a_pos {
                s += &format!(
                    r#","min":[{},{},{}],"max":[{},{},{}]"#,
                    num(lo[0]),
                    num(lo[1]),
                    num(lo[2]),
                    num(hi[0]),
                    num(hi[1]),
                    num(hi[2])
                );
            }
            if Some(i) == a_color {
                s += r#","normalized":true"#;
            }
            s + "}"
        })
        .collect();
    let mut attrs = format!(r#""POSITION":{a_pos},"NORMAL":{a_nor},"_FACE":{a_face}"#);
    if let Some(c) = a_color {
        attrs += &format!(r#","COLOR_0":{c}"#);
    }
    let json = format!(
        concat!(
            r#"{{"asset":{{"version":"2.0","generator":"springrep"}},"scene":0,"#,
            r#""scenes":[{{"nodes":[0]}}],"nodes":[{{"mesh":0}}],"#,
            r#""meshes":[{{"primitives":[{{"attributes":{{{attrs}}},"indices":{idx}}}]}}],"#,
            r#""buffers":[{{"byteLength":{len}}}],"bufferViews":[{views}],"accessors":[{accs}]}}"#
        ),
        attrs = attrs,
        idx = a_idx,
        len = bin.len(),
        views = views.join(","),
        accs = acc_json.join(","),
    );
    let mut body = json.into_bytes();
    while !body.len().is_multiple_of(4) {
        body.push(b' ');
    }
    let total = 12 + 8 + body.len() + 8 + bin.len();
    let mut out = Vec::with_capacity(total);
    out.extend(0x4654_6C67u32.to_le_bytes());
    out.extend(2u32.to_le_bytes());
    out.extend((total as u32).to_le_bytes());
    out.extend((body.len() as u32).to_le_bytes());
    out.extend(0x4E4F_534Au32.to_le_bytes());
    out.extend(&body);
    out.extend((bin.len() as u32).to_le_bytes());
    out.extend(0x004E_4942u32.to_le_bytes());
    out.extend(&bin);
    out
}
