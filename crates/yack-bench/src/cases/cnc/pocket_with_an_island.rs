// yack: brep-bench case cnc/pocket-with-an-island
//! A 2.5-axis program of three tools: a pocket round an island cleared in raster passes, a bore and
//! four drilled holes.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // The program as data: each tool's toolpath. The mill rasters the lines y across the region its tip keeps to,
    // then runs round the region's rounded wall and round the island, at each of three depths.
    let intervals = |y: f64| {
        let inset = |d: f64| 32.0 - (49.0 - d * d).sqrt();
        let left = if y < 32.0 { inset(32.0 - y) } else if y > 78.0 { inset(y - 78.0) } else { 25.0 };
        let off = (y - 55.0).abs();
        let half = (400.0 - off * off).max(0.0).sqrt();
        if off < 20.0 { vec![(left, 60.0 - half), (60.0 + half, 160.0 - left)] } else { vec![(left, 160.0 - left)] }
    };
    let wall = Loop::polygon(&[[80.0, 25.0], [135.0, 25.0], [135.0, 85.0], [25.0, 85.0], [25.0, 25.0]])?.rounded(7.0)?;
    let mut pocket = Toolpath::starting_at([0.0, 0.0, 5.0]);
    for z in [-4.0, -8.0, -12.0] {
        for y in (0..15).map(|j| 26.0 + 4.0 * f64::from(j)) {
            for (a, b) in intervals(y) {
                pocket = pocket.line_to([a, y, 5.0]).line_to([a, y, z]).line_to([b, y, z]).line_to([b, y, 5.0]);
            }
        }
        pocket = pocket.line_to([80.0, 25.0, 5.0]).along_loop(&Frame::at([0.0, 0.0, z]), &wall)?;
        pocket = pocket.line_to([80.0, 25.0, 5.0]).line_to([80.0, 55.0, 5.0]).line_to([80.0, 55.0, z]);
        pocket = pocket.turn([60.0, 55.0, z], [0.0, 0.0, -1.0], deg(360.0))?.line_to([80.0, 55.0, 5.0]);
    }
    let mut bore = Toolpath::starting_at([60.0, 55.0, 5.0]);
    for z in [-3.0, -6.0] {
        bore = bore.line_to([60.0, 55.0, z]).line_to([66.0, 55.0, z]);
        bore = bore.turn([60.0, 55.0, z], [0.0, 0.0, 1.0], deg(360.0))?.line_to([60.0, 55.0, z]);
    }
    let mut holes = Toolpath::starting_at([8.0, 8.0, 5.0]);
    for [x, y] in [[8.0, 8.0], [152.0, 8.0], [152.0, 102.0], [8.0, 102.0]] {
        holes = [5.0, -5.0, 2.0, -10.0, 2.0, -14.0, 5.0].iter().fold(holes, |h, &z| h.line_to([x, y, z]));
    }
    let drill = Loop::polygon(&[[0.0, 0.0], [4.0, 4.0], [4.0, 30.0], [0.0, 30.0]])?;
    let plate = block(&Frame::at([0.0, 0.0, -20.0]), [160.0, 110.0, 20.0])?;
    let mill = cylinder(&Frame::at([0.0, 0.0, 5.0]), 5.0, 30.0)?;
    let plate = plate.cut_swept(&mill, &pocket.motion())?;
    let borer = cylinder(&Frame::at([60.0, 55.0, 5.0]), 3.0, 30.0)?;
    let plate = plate.cut_swept(&borer, &bore.line_to([60.0, 55.0, 5.0]).motion())?;
    let drill = revolve(&Frame::at([8.0, 8.0, 5.0]), &Profile::new(drill, vec![])?, deg(360.0))?;
    plate.cut_swept(&drill, &holes.motion())
}
