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
            eprintln!("invariant 3 (warning): {f}");
        }
        if ["union", "intersection", "difference", "difference_all", "transform", "named"].contains(&op) {
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
