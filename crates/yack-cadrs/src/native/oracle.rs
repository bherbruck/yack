// yack: A body for the SMLib test oracle (`tools/smlib_oracle`, on NVIDIA's usd-brep SMLib,.
// Adapted from cadrs@eb5899ef crates/cadrs_kernel/src/native/oracle.rs (MIT OR Apache-2.0, (c) 2026 Rouan van der Ende).
// Changes: none (verbatim).
//! A body for the SMLib test oracle (`tools/smlib_oracle`, on NVIDIA's usd-brep SMLib,
//! Apache-2.0), which runs the same operations on it to compare against: as an SMLib `.smb` file
//! ([`smb_text`]).
//!
//! The `.smb` is SMLib's version 42 layout (as `SmApiWriteBrepToFile` writes, its section
//! headers' comment lines included: the reader skips them by count): an infinite void region and
//! one solid region, a shell each holding a use of every face (the outer side facing out), faces
//! on B-spline surfaces (`Object Type 15002`) whose normals point out of the solid, loops of edge
//! uses, edges on B-spline curves (`Object Type 14033`) between vertices. An edge's two uses are
//! linked as SMLib's radial lists are: the first use's next is its own mate and its mate's next
//! the second use's mate; the second's next is the first, its mate's next itself. The UV trim
//! curves are cubics through points along each edge inverted onto the exported surface (a
//! rational arc's parameter isn't our angle; a seam's two uses each on its own side).

use std::fmt::Write;

use super::math::P3;
use super::nurbs::{NurbsCurve, NurbsSurface};
use super::topo::Body;





