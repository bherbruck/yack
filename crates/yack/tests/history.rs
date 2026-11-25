// yack: M4.11 step 1: every op's `History`, checked against audit invariant 3 (PLAN §6.2), warning only until enforced.
use yack::prelude::*;
use yack::{Change, FaceOrigin};

/// Invariant 3's findings for `result`, made from `inputs`: (a) every face has a row with an
/// origin, (b) each origin names a face of its operand, (c) the change agrees with the origins,
/// (d) every operand face is deleted or an origin, never both.
fn audit(op: &str, inputs: &[&Solid], result: &Solid) -> Vec<String> {
    let h = result.history();
    let mut out = Vec::new();
    if usize::from(h.operands()) != inputs.len() {
        out.push(format!("{op}: {} operands recorded, {} taken", h.operands(), inputs.len()));
    }
    let mut used = Vec::new();
    for f in result.faces() {
        let (Some(change), origins) = (h.change(f), h.origins(f)) else {
            out.push(format!("{op}: {f} has no row"));
            continue;
        };
        if origins.is_empty() {
            out.push(format!("{op}: {f} ({change:?}) has no origin"));
        }
        let mut operand_faces = 0;
        for o in origins {
            if let FaceOrigin::Operand { operand, face } | FaceOrigin::Offset { operand, face } = o {
                match inputs.get(usize::from(*operand)) {
                    Some(s) if s.faces().contains(face) => used.push(*face),
                    _ => out.push(format!("{op}: {f} names {face} of operand {operand}, which has no such face")),
                }
                operand_faces += usize::from(matches!(o, FaceOrigin::Operand { .. }));
            }
        }
        let agrees = match change {
            Change::Generated => operand_faces == 0,
            Change::Kept | Change::Modified | Change::Split => operand_faces == 1,
            Change::Merged => origins.len() >= 2,
        };
        if !agrees {
            out.push(format!("{op}: {f} is {change:?} with {operand_faces} operand faces"));
        }
    }
    for s in inputs {
        for f in s.faces() {
            let (gone, kept) = (h.deleted().contains(&f), used.contains(&f));
            if gone == kept {
                out.push(format!("{op}: operand {f} is {}", if gone { "both deleted and an origin" } else { "neither deleted nor an origin" }));
            }
        }
    }
    out
}

/// Every op of step 1's reach, once: (name, inputs, result).
fn calls() -> Result<Vec<(&'static str, Vec<Solid>, Solid)>> {
    let plate = block(&Frame::WORLD, [10.0, 10.0, 4.0])?;
    let pin = cylinder(&Frame::at([5.0, 5.0, -1.0]), 2.0, 6.0)?;
    let other = block(&Frame::at([5.0, 0.0, 0.0]), [10.0, 10.0, 4.0])?;
    let top = plate.edge_at([5.0, 0.0, 4.0])?;
    let face = plate.face_at([5.0, 5.0, 4.0])?;
    let side = Loop::polygon(&[[0.0, 0.0], [4.0, 0.0], [4.0, 3.0], [0.0, 3.0]])?;
    let profile = Profile::new(side, vec![])?;
    Ok(vec![
        ("block", vec![], plate.clone()),
        ("cylinder", vec![], pin.clone()),
        ("extrude", vec![], extrude(&Frame::WORLD, &profile, 2.0)?),
        ("revolve", vec![], revolve(&Frame::WORLD, &profile, deg(360.0))?),
        ("cutter", vec![], cutter(&Frame::WORLD, Cutter::Ball, 3.0, 20.0)?),
        ("union", vec![plate.clone(), other.clone()], plate.union(&other)?),
        ("intersection", vec![plate.clone(), other.clone()], plate.intersection(&other)?),
        ("difference", vec![plate.clone(), pin.clone()], plate.difference(&pin)?),
        ("difference_all", vec![plate.clone(), pin.clone()], plate.difference_all(core::slice::from_ref(&pin))?),
        ("fillet", vec![plate.clone()], plate.fillet(&[top], 1.0)?),
        ("chamfer", vec![plate.clone()], plate.chamfer(&[top], 1.0)?),
        ("shell", vec![plate.clone()], plate.shell(&[face], 1.0)?),
        ("offset", vec![plate.clone()], plate.offset(0.5)?),
        ("transform", vec![plate.clone()], plate.transform(Rigid::translation([1.0, 2.0, 3.0]))?),
        ("named", vec![plate.clone()], plate.named("plate")),
        ("decode", vec![], yack::codec::decode(&yack::codec::encode(&plate)?)?),
    ])
}

/// Step 1: the findings are printed per op, not yet failed on (PLAN §6.2: invariant 3 warns
/// until History is enforced); the ops the names fully cover are held to it now.
#[test]
fn every_op_has_a_history_and_its_findings_are_reported() -> Result<()> {
    let mut total = 0;
    for (op, inputs, result) in calls()? {
        let refs: Vec<&Solid> = inputs.iter().collect();
        let found = audit(op, &refs, &result);
        for f in &found {
            eprintln!("invariant 3 (warning, {}): {f}", result.strategy());
        }
        {
            assert!(found.is_empty(), "{op}: {found:?}");
        }
        total += found.len();
    }
    eprintln!("invariant 3: {total} findings over every op");
    Ok(())
}

#[test]
fn a_drilled_plate_says_where_each_face_came_from() -> Result<()> {
    let plate = block(&Frame::WORLD, [10.0, 10.0, 4.0])?;
    let pin = cylinder(&Frame::at([5.0, 5.0, -1.0]), 2.0, 6.0)?;
    let drilled = plate.difference(&pin)?;
    let h = drilled.history();
    let wall = drilled.face_at([7.0, 5.0, 2.0])?;
    assert_eq!(h.change(wall), Some(Change::Modified));
    assert!(matches!(h.origins(wall), [FaceOrigin::Operand { operand: 1, .. }]));
    // the pin's two caps are gone, its side became the hole's wall; the plate's four sides are kept
    assert_eq!(h.deleted().len(), 2);
    let kept = h.rows().filter(|(_, c, _)| *c == Change::Kept).count();
    assert_eq!(kept, 4);
    Ok(())
}

#[test]
fn three_hundred_tools_cut_in_one_operation_still_give_a_history() -> Result<()> {
    let plate = block(&Frame::WORLD, [10.0, 10.0, 1.0])?;
    // 300 tools, all clear of the plate: the plate comes back whole, its history says nothing it cannot number
    let tools: Vec<Solid> = (0..300).map(|i| block(&Frame::at([20.0 + 2.0 * f64::from(i), 0.0, 0.0]), [1.0, 1.0, 1.0])).collect::<Result<_>>()?;
    let cut = plate.difference_all(&tools)?;
    assert!((cut.volume() - 100.0).abs() < 1e-9);
    let h = cut.history();
    assert_eq!(h.rows().count(), cut.faces().len());
    Ok(())
}

#[test]
fn a_decoded_solid_takes_part_in_an_operation_with_a_history() -> Result<()> {
    let plate = block(&Frame::WORLD, [10.0, 10.0, 2.0])?;
    let decoded = yack::codec::decode(&yack::codec::encode(&plate)?)?;
    let drilled = decoded.difference(&cylinder(&Frame::at([5.0, 5.0, -1.0]), 2.0, 4.0)?)?;
    assert_eq!(drilled.history().rows().count(), drilled.faces().len());
    Ok(())
}

#[test]
fn a_round_comes_from_the_edge_it_rounds_and_a_corner_from_the_vertex() -> Result<()> {
    let b = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
    let edges = [b.edge_at([5.0, 0.0, 10.0])?, b.edge_at([0.0, 5.0, 10.0])?, b.edge_at([0.0, 0.0, 5.0])?];
    let r = b.fillet(&edges, 1.0)?;
    let h = r.history();
    for (k, e) in edges.iter().enumerate() {
        let from: Vec<FaceId> = h.rows().filter(|(_, _, o)| o.contains(&FaceOrigin::Edge { operand: 0, edge: *e })).map(|(f, _, _)| f).collect();
        assert_eq!(from.len(), 1, "edge {k}");
    }
    let corner = b.vertices().into_iter().find(|&v| b.vertex(v).is_ok_and(|x| x.point().to_array() == [0.0, 0.0, 10.0]));
    let at_corner = h.rows().filter(|(_, _, o)| o.iter().any(|x| matches!(x, FaceOrigin::Vertex { vertex, .. } if Some(*vertex) == corner))).count();
    assert_eq!(at_corner, 1);
    Ok(())
}

#[test]
fn a_step_file_read_back_numbers_its_faces() -> Result<()> {
    let b = block(&Frame::WORLD, [1.0, 2.0, 3.0])?;
    let read = yack::step::read(&yack::step::write(&b)?)?;
    let h = read.history();
    let n: Vec<u32> = h.rows().filter_map(|(_, _, o)| match o { [FaceOrigin::Read { face }] => Some(*face), _ => None }).collect();
    assert_eq!(n, (0..6).collect::<Vec<u32>>());
    Ok(())
}

/// A seeded xorshift: the same chains every run.
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn below(&mut self, n: usize) -> usize {
        usize::try_from(self.next() % u64::try_from(n).unwrap_or(1)).unwrap_or(0)
    }
    fn unit(&mut self) -> f64 {
        f64::from(u32::try_from(self.next() >> 40).unwrap_or(0)) / f64::from(1u32 << 24)
    }
}

/// One step of a chain on `s`: a blend, a boolean with a fresh tool, or a move; `None` when the
/// step does not apply (the chain then tries another).
fn step(r: &mut Rng, s: &Solid) -> Option<(&'static str, Solid, Option<Solid>)> {
    let edges = s.edges();
    match r.below(5) {
        0 => s.fillet(&[*edges.get(r.below(edges.len()))?], 0.3).ok().map(|x| ("fillet", x, None)),
        1 => s.chamfer(&[*edges.get(r.below(edges.len()))?], 0.3).ok().map(|x| ("chamfer", x, None)),
        2 => {
            let tool = block(&Frame::at([2.0 + 6.0 * r.unit(), 2.0 + 6.0 * r.unit(), 2.0 + 4.0 * r.unit()]), [3.0, 3.0, 3.0]).ok()?;
            s.union(&tool).ok().map(|x| ("union", x, Some(tool)))
        }
        3 => {
            let tool = cylinder(&Frame::at([2.0 + 6.0 * r.unit(), 2.0 + 6.0 * r.unit(), -1.0]), 0.5 + r.unit(), 12.0).ok()?;
            s.difference(&tool).ok().map(|x| ("difference", x, Some(tool)))
        }
        _ => s.transform(Rigid::translation([r.unit(), r.unit(), 0.0])).ok().map(|x| ("transform", x, None)),
    }
}

/// M4.11's property: over seeded 5-op chains, every step's history passes invariant 3, the composed history names only faces of the chain's own inputs, and
/// composing is associative.
#[test]
fn composed_histories_over_five_op_chains_reach_back_to_the_inputs() -> Result<()> {
    for seed in 1..=40u64 {
        let mut r = Rng(0x9E37_79B9_7F4A_7C15 ^ seed);
        let start = block(&Frame::WORLD, [10.0, 10.0, 8.0])?;
        let mut inputs: Vec<FaceId> = start.faces();
        let mut s = start;
        let mut hs = Vec::new();
        let mut tries = 0;
        while hs.len() < 5 && tries < 40 {
            tries += 1;
            let Some((op, next, tool)) = step(&mut r, &s) else { continue };
            let mut ops = vec![&s];
            ops.extend(tool.as_ref());
            let found = audit(op, &ops, &next);
            assert!(found.is_empty(), "seed {seed} ({}): {found:?}", next.strategy());
            if let Some(t) = &tool {
                inputs.extend(t.faces());
            }
            hs.push(next.history().clone());
            s = next;
        }
        assert_eq!(hs.len(), 5, "seed {seed}: the chain stalled");
        let whole = hs.iter().skip(1).fold(hs.first().cloned().unwrap_or_default(), |earlier, later| later.compose(0, &earlier));
        for (f, _, origins) in whole.rows() {
            for o in origins {
                if let FaceOrigin::Operand { face, .. } | FaceOrigin::Offset { face, .. } = o {
                    assert!(inputs.contains(face), "seed {seed}: {f} comes from {face}, not a face of the chain's inputs");
                }
            }
        }
        for w in hs.windows(3) {
            let [a, b, c] = w else { continue };
            let (left, right) = (c.compose(0, &b.compose(0, a)), c.compose(0, b).compose(0, a));
            for ((f, x, o), (_, y, p)) in left.rows().zip(right.rows()) {
                assert_eq!((x, o), (y, p), "seed {seed}: {f} composes differently");
            }
            assert_eq!(left.deleted(), right.deleted(), "seed {seed}: deleted faces compose differently");
            assert_eq!(left, right, "seed {seed}: compose is not associative");
        }
    }
    Ok(())
}

#[test]
fn an_extrusion_below_its_plane_is_made_from_nothing_like_one_above() -> Result<()> {
    let rect = Profile::new(Loop::polygon(&[[0.0, 0.0], [3.0, 0.0], [3.0, 2.0], [0.0, 2.0]])?, vec![])?;
    let down = extrude(&Frame::WORLD, &rect, -4.0)?;
    let h = down.history();
    assert_eq!(h.operands(), 0);
    assert!(audit("extrude down", &[], &down).is_empty());
    assert!(h.rows().all(|(_, c, o)| c == Change::Generated && matches!(o, [FaceOrigin::Segment { .. }])));
    Ok(())
}
