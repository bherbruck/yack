# springrep bugs found while it is a temporary engine (for the M3 owners)

Each entry: the repro, what is wrong, what is right, who found it, and where it bites. Nothing
here is fixed in `yack-springrep` while M2 is digest-gated: the fix lands in the M3 crate that
takes over the code, with the repro as its regression test.

## B1. `blend::fillet_edges` rounds an edge it was not given (owner: M3 `yack-blend`)

- **Repro.** `t = primitive::torus(Frame::WORLD, 6, 2)`, `(front, _) = boolean::split(&t, 0, +Z)`,
  `e = measure::edge_at(&front, (0, 8, 0))` (the outer rim of the flat annulus),
  `r = blend::fillet_edges(&front, &[e], 0.5)`.
- **Wrong.** `r` has 4 faces: the torus, the plane, a round on a torus of major radius 7.414 (the
  outer rim, asked for) and a second round on a torus of major radius 4.586 (the inner rim, not
  asked for). The two rim circles seem to be taken as one chain.
- **Right.** 3 faces, only the outer rim rounded. cadrs gives volume 230.29, and 236.871 once
  the round is deleted (the bench's expected value).
- **Found by** w-m17-cadrs (probe `yack-ref/cadrs-on-shim/probe3/src/main.rs`), behind brep-bench
  WRONG `edit/half-torus-outer-rim-round-deleted`.
- **Guarded meanwhile** by the M1.10 ladder: a blend whose rounded edges are not the given edges'
  tangent chains declines (w-m03-api).

## B2. `measure::Outlines::normal_at` calls a mitered chamfer's edge smooth (owner: M3 `yack-locate`)

- **Repro.** brep-bench `chamfer/cube-all-edges-mitered`: at the midpoint (8.5, 1.5, 0.5) of edge 24
  (between two chamfer faces at a corner miter), `normal_at(p, 10·LINEAR)` returns a normal. Same on
  `chamfer/octahedron-all-edges-mitered`, edge 2.
- **Wrong.** The faces there meet at an angle, so the answer is `None` (a crease). Only one of the two
  faces' outlines "holds" the point within the tolerance, so the faces' normals are never compared.
- **Right.** `None`. The same solid carried to cadrs and back (yack-bridge) answers `None`, because
  both faces then hold the point.
- **Found by** w-m11-springrep (the bridge round trip's crease check), 2026-11-20.

## N1 (note for M3 `yack-topo`). springrep cannot carry an explicit 2-D pcurve

- **What.** springrep's `topo::PCurve` is `Affine` or `Projected` (re-project the 3-D edge onto the
  surface). Where a surface takes one 3-D point at two `(u, v)` (a folded or collapsed strip of a
  swept spline), projection may pick the other preimage for each of two coedges meeting there, and
  the checker refuses the loop.
- **Seen in.** The 9 brep-bench `swept/tumbling-torus*` cases: cadrs builds them (its pcurves are
  explicit 2-D B-splines by the edge's t), and they cannot be carried into springrep
  (yack-bridge diagnostics, w-m11-springrep, 2026-11-20). Handled meanwhile by the lazy bridge
  (cadrs-built solids stay in cadrs form until a springrep op needs them).
- **For M3.** Give `yack-topo`'s `PCurve` the PLAN's third kind, `Curve2` (a 2-D B-spline by the
  edge's t), and use it wherever a pcurve is known rather than projected.
