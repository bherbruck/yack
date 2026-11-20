// yack: brep-bench case boolean/golf-ball
//! A golf ball.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // the 62 directions: the icosahedron's 12 vertices, its 30 edges' midpoints (vertex pairs 2 apart) and
    // its 20 faces' centres (vertex triples 2 apart pairwise), each as a sum of vertices
    let p = (1.0 + 5f64.sqrt()) / 2.0;
    let signs = [(1.0, p), (1.0, -p), (-1.0, p), (-1.0, -p)];
    let v: Vec<[f64; 3]> = signs.iter().flat_map(|&(a, b)| [[0.0, a, b], [a, b, 0.0], [b, 0.0, a]]).collect();
    let near = |a: &[f64; 3], b: &[f64; 3]| a.iter().zip(b).map(|(x, y)| (x - y).powi(2)).sum::<f64>() < 4.5;
    let add = |a: [f64; 3], b: &[f64; 3]| [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
    let mut dirs = v.clone();
    for (i, a) in v.iter().enumerate() {
        for (j, b) in v.iter().enumerate().skip(i + 1).filter(|(_, b)| near(a, b)) {
            dirs.push(add(*a, b));
            dirs.extend(v.iter().skip(j + 1).filter(|c| near(a, c) && near(b, c)).map(|c| add(add(*a, b), c)));
        }
    }
    dirs.iter().try_fold(sphere(&Frame::WORLD, 10.0)?, |ball, d| {
        let n = d.iter().map(|x| x * x).sum::<f64>().sqrt();
        ball.difference(&sphere(&Frame::at(d.map(|x| 10.6 * x / n)), 1.2)?)
    })
}
