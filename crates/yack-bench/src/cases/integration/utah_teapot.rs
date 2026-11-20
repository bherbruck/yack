// yack: brep-bench case integration/utah-teapot
//! The Utah teapot.
use yack::prelude::*;
// data: Newell's patches as the case gives them: the six rings' profiles (r, z) and the spout's and handle's two
// patches on the side y ≤ 0, row by row
type Net = [[[f64; 3]; 4]; 4];
const RIM: [[f64; 2]; 4] = [[1.4, 2.4], [1.3375, 2.53125], [1.4375, 2.53125], [1.5, 2.4]];
const UPPER: [[f64; 2]; 4] = [[1.5, 2.4], [1.75, 1.875], [2.0, 1.35], [2.0, 0.9]];
const LOWER: [[f64; 2]; 4] = [[2.0, 0.9], [2.0, 0.45], [1.5, 0.225], [1.5, 0.15]];
const BOTTOM: [[f64; 2]; 4] = [[1.5, 0.15], [1.5, 0.075], [1.425, 0.0], [0.0, 0.0]];
const KNOB: [[f64; 2]; 4] = [[0.0, 3.15], [0.8, 3.15], [0.0, 2.85], [0.2, 2.7]];
const LID: [[f64; 2]; 4] = [[0.2, 2.7], [0.4, 2.55], [1.3, 2.55], [1.3, 2.4]];
const SPOUT: [Net; 2] = [
    [
        [[1.7, 0.0, 1.425], [1.7, -0.66, 1.425], [1.7, -0.66, 0.6], [1.7, 0.0, 0.6]],
        [[2.6, 0.0, 1.425], [2.6, -0.66, 1.425], [3.1, -0.66, 0.825], [3.1, 0.0, 0.825]],
        [[2.3, 0.0, 2.1], [2.3, -0.25, 2.1], [2.4, -0.25, 2.025], [2.4, 0.0, 2.025]],
        [[2.7, 0.0, 2.4], [2.7, -0.25, 2.4], [3.3, -0.25, 2.4], [3.3, 0.0, 2.4]],
    ],
    [
        [[2.7, 0.0, 2.4], [2.7, -0.25, 2.4], [3.3, -0.25, 2.4], [3.3, 0.0, 2.4]],
        [[2.8, 0.0, 2.475], [2.8, -0.25, 2.475], [3.525, -0.25, 2.49375], [3.525, 0.0, 2.49375]],
        [[2.9, 0.0, 2.475], [2.9, -0.15, 2.475], [3.45, -0.15, 2.5125], [3.45, 0.0, 2.5125]],
        [[2.8, 0.0, 2.4], [2.8, -0.15, 2.4], [3.2, -0.15, 2.4], [3.2, 0.0, 2.4]],
    ],
];
const HANDLE: [Net; 2] = [
    [
        [[-1.6, 0.0, 2.025], [-1.6, -0.3, 2.025], [-1.5, -0.3, 2.25], [-1.5, 0.0, 2.25]],
        [[-2.3, 0.0, 2.025], [-2.3, -0.3, 2.025], [-2.5, -0.3, 2.25], [-2.5, 0.0, 2.25]],
        [[-2.7, 0.0, 2.025], [-2.7, -0.3, 2.025], [-3.0, -0.3, 2.25], [-3.0, 0.0, 2.25]],
        [[-2.7, 0.0, 1.8], [-2.7, -0.3, 1.8], [-3.0, -0.3, 1.8], [-3.0, 0.0, 1.8]],
    ],
    [
        [[-2.7, 0.0, 1.8], [-2.7, -0.3, 1.8], [-3.0, -0.3, 1.8], [-3.0, 0.0, 1.8]],
        [[-2.7, 0.0, 1.575], [-2.7, -0.3, 1.575], [-3.0, -0.3, 1.35], [-3.0, 0.0, 1.35]],
        [[-2.5, 0.0, 1.125], [-2.5, -0.3, 1.125], [-2.65, -0.3, 0.9375], [-2.65, 0.0, 0.9375]],
        [[-2.0, 0.0, 0.9], [-2.0, -0.3, 0.9], [-1.9, -0.3, 0.6], [-1.9, 0.0, 0.6]],
    ],
];
pub fn build() -> Result<Solid> {
    // a ring's first patch spans the quarter from +x to −y (row i from profile point i; the knob's row 1 is 0.45 where
    // the rule gives 0.448), the others it turned 90°, 180° and 270° about z; a mirror reverses each row to run on
    let ring = |p: [[f64; 2]; 4], knob: bool| -> Vec<Net> {
        let mut first: Net = p.map(|[r, z]| [[r, 0.0, z], [r, -0.56 * r, z], [0.56 * r, -r, z], [0.0, -r, z]]);
        if knob {
            first[1][1][1] = -0.45;
            first[1][2][0] = 0.45;
        }
        let turn = |n: Net| n.map(|row| row.map(|[x, y, z]| [y, -x, z]));
        vec![first, turn(first), turn(turn(first)), turn(turn(turn(first)))]
    };
    let mirror = |n: Net| {
        n.map(|row| {
            let [a, b, c, d] = row.map(|[x, y, z]| [x, -y, z]);
            [d, c, b, a]
        })
    };
    let bezier = |n: &Net| Patch::Bezier { ctrl: n.iter().map(|r| r.iter().map(|&q| q.into()).collect()).collect() };
    let rows = |edge: &[[[f64; 3]; 4]]| edge.iter().map(|r| Spline3::bezier(r)).collect::<Result<Vec<_>>>();
    // the body: the rim, upper and lower body and bottom, closed by the flat disk inside the rim's row 0 at z = 2.4
    let rim = ring(RIM, false);
    let nets = [rim.clone(), ring(UPPER, false), ring(LOWER, false), ring(BOTTOM, false)].concat();
    let mut body: Vec<Patch> = nets.iter().map(bezier).collect();
    body.push(Patch::PlanarFill { boundary: rows(&rim.iter().map(|n| n[0]).collect::<Vec<_>>())? });
    // the lid: the knob and the lid, closed by the flat disk inside the lid's row 3 at z = 2.4
    let lid_ring = ring(LID, false);
    let mut lid: Vec<Patch> = [ring(KNOB, true), lid_ring.clone()].concat().iter().map(bezier).collect();
    lid.push(Patch::PlanarFill { boundary: rows(&lid_ring.iter().map(|n| n[3]).collect::<Vec<_>>())? });
    let mut teapot = sewn(&body)?.union(&sewn(&lid)?)?;
    // the spout and the handle: two patches and their mirrors, closed at each end by the flat face inside the loop
    // of the first patch's row 0 and its mirror's, and of the second's row 3 and its mirror's
    for [a, b] in [SPOUT, HANDLE] {
        let (ma, mb) = (mirror(a), mirror(b));
        let mut part = vec![bezier(&a), bezier(&b), bezier(&ma), bezier(&mb)];
        part.push(Patch::PlanarFill { boundary: rows(&[a[0], ma[0]])? });
        part.push(Patch::PlanarFill { boundary: rows(&[b[3], mb[3]])? });
        teapot = teapot.union(&sewn(&part)?)?;
    }
    Ok(teapot)
}
