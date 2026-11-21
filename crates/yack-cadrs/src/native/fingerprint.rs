// yack: Geometry hashed to the bit, for the caches keyed by what a result depends on (a face'....
// Adapted from cadrs@eb5899ef crates/cadrs_kernel/src/native/fingerprint.rs (MIT OR Apache-2.0, (c) 2026 Rouan van der Ende).
// Changes: a fixed-size vector's as_slice() read as to_array().as_slice() (lines 73, 75, 84, 85,
//   90, 97, 107, 148, 162); M4.0a: Key128's hashing core moved to key.rs.
//! Geometry hashed to the bit, for the caches keyed by what a result depends on (a face's mass
//! integrals, its mesh, its boundary's polygons): two 64-bit streams, each taking every word
//! through a mixer of its own, make a 128-bit key two different inputs don't share in practice.
//!
//! The keys are made for every face of every boolean, of every number its curves and surface
//! hold, so the streams are cheap ones: a word is folded into each state and the state put
//! through a bijective mixer (two multiplications and three shifts), where two SipHash streams
//! fed a word at a time took most of a boolean's bookkeeping. The keys only name entries of
//! caches in memory: nothing outside the process depends on their values.

use std::hash::Hasher;

use super::curve::Curve;
use super::math::Frame;
use super::surface::Surface;
use super::key::Key128;




impl Key128 {    fn frame(&mut self, f: &Frame) {
        self.f64s(f.o.coords.to_array().as_slice());
        for a in [&f.x, &f.y, &f.z] {
            self.f64s(a.to_array().as_slice());
        }
    }

    /// A curve: its kind and every number it is made of.
    pub fn curve(&mut self, c: &Curve) {
        match c {
            Curve::Line { o, d } => {
                self.write_u8(0);
                self.f64s(o.coords.to_array().as_slice());
                self.f64s(d.to_array().as_slice());
            }
            Curve::Circle { c, x, y, r } => {
                self.write_u8(1);
                for v in [&c.coords, x, y] {
                    self.f64s(v.to_array().as_slice());
                }
                self.f64s(&[*r]);
            }
            Curve::Ellipse { c, x, y, a, b } => {
                self.write_u8(2);
                for v in [&c.coords, x, y] {
                    self.f64s(v.to_array().as_slice());
                }
                self.f64s(&[*a, *b]);
            }
            Curve::Nurbs(n) => {
                self.write_u8(3);
                self.write_usize(n.degree);
                self.write_usize(n.knots.len());
                self.f64s(&n.knots);
                for p in &n.cps {
                    self.f64s(p.to_array().as_slice());
                }
            }
        }
    }

    /// A surface: its kind and every number it is made of.
    pub fn surface(&mut self, s: &Surface) {
        match s {
            Surface::Plane { f } => {
                self.write_u8(0);
                self.frame(f);
            }
            Surface::Cylinder { f, r } => {
                self.write_u8(1);
                self.frame(f);
                self.f64s(&[*r]);
            }
            Surface::Cone { f, r, a } => {
                self.write_u8(2);
                self.frame(f);
                self.f64s(&[*r, *a]);
            }
            Surface::Sphere { f, r } => {
                self.write_u8(3);
                self.frame(f);
                self.f64s(&[*r]);
            }
            Surface::Torus { f, rmaj, rmin } => {
                self.write_u8(4);
                self.frame(f);
                self.f64s(&[*rmaj, *rmin]);
            }
            Surface::Revolution { f, m } => {
                self.write_u8(5);
                self.frame(f);
                self.curve(m);
            }
            Surface::Extrusion { c, d } => {
                self.write_u8(6);
                self.curve(c);
                self.f64s(d.to_array().as_slice());
            }
            Surface::Nurbs(n) => {
                self.write_u8(7);
                self.write_usize(n.du);
                self.write_usize(n.dv);
                self.write_usize(n.ku.len());
                self.f64s(&n.ku);
                self.write_usize(n.kv.len());
                self.f64s(&n.kv);
                self.write_usize(n.cps.len());
                for row in &n.cps {
                    self.write_usize(row.len());
                    for p in row {
                        self.f64s(p.to_array().as_slice());
                    }
                }
            }
        }
    }
}


