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
