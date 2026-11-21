// yack: brep-bench case boolean/golf-ball
//! A golf ball.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // the 62 directions: the icosahedron's 12 vertices, its 30 edges' midpoints (vertex pairs 2 apart) and
    // its 20 faces' centres (vertex triples 2 apart pairwise), each as a sum of vertices
    let p = (1.0 + 5f64.sqrt()) / 2.0;
    let signs = [(1.0, p), (1.0, -p), (-1.0, p), (-1.0, -p)];
    let v: Vec<_> = signs.iter().flat_map(|&(a, b)| [[0.0, a, b], [a, b, 0.0], [b, 0.0, a]].map(Vec3::from)).collect();
    let near = |a: &Vec3, b: &Vec3| (*a - *b).dot(*a - *b) < 4.5;
    let mut dirs = v.clone();
    for (i, a) in v.iter().enumerate() {
        for (j, b) in v.iter().enumerate().skip(i + 1).filter(|(_, b)| near(a, b)) {
            dirs.push(*a + *b);
            dirs.extend(v.iter().skip(j + 1).filter(|c| near(a, c) && near(b, c)).map(|c| *a + *b + *c));
        }
    }
    dirs.iter().try_fold(sphere(&Frame::WORLD, 10.0)?, |ball, d| {
        let n = d.length();
        ball.difference(&sphere(&Frame::at(d.to_array().map(|x| 10.6 * x / n)), 1.2)?)
    })
}
