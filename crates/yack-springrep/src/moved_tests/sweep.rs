// yack: sweep's tests that build their operands with springrep modules above yack-build.
// Adapted from springrep@a03e8a1 src/sweep (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved out of the crate's tests in M3 (W4-W7); paths and imports only.
/// Moved from sweep/helix/tests.rs (M3 W7): they build above yack-build.
#[cfg(test)]
mod sweep_helix_moved {
    use crate::check;
    use crate::math::Frame;
    use crate::math::Vec2;
    use crate::math::Vec3;
    use crate::measure;
    use crate::profile::Profile;
    use crate::sweep::Helix;
    use crate::sweep::helix;
    use crate::topo::Brep;
    use std::f64::consts::TAU;

    fn rel(a: f64, b: f64) -> f64 {
        (a - b).abs() / b.abs()
    }

    fn about_z(pitch: f64, sign: f64) -> Helix {
        Helix {
            origin: Vec3::ZERO,
            axis: Vec3::Z,
            rise: pitch / TAU,
            sign,
        }
    }

    /// A metric thread's ridge on its shank where it runs out, as a screw's: the shank a cylinder at
    /// the major radius, a cone down to the minor over the runout, the minor cylinder; the ridge the
    /// ISO basic profile (crest flat P/8 on the major radius, 60° flanks) reaching 0.1P below the
    /// minor, cut square at the cylinder's edge with the cone and 4 pitches past the cone. So the
    /// crest lies on the plain cylinder and runs across that edge. Each height's slice of the overlap
    /// covers the fraction w(ρ)/P of each circle of the ridge inside the shank, w the profile's width
    /// at ρ, so the overlap is ∫ 2π/P ∫ ρ w(ρ) dρ dz.
    fn runout() -> (Brep, Brep, f64) {
        use crate::profile::{NamedSegment, ProfileLoop, Segment};
        let (p, major) = (1.25, 4.0);
        let depth = 5.0 / 8.0 * 3f64.sqrt() / 2.0 * p;
        let minor = major - depth;
        let base = minor - 0.1 * p;
        let flank = 2.0 / 3f64.sqrt();
        let width = |rho: f64| p / 8.0 + (major - rho) * flank;
        let (top, runout) = (6.0 * p, 2.5 * p);
        let len = top + runout + 6.0 * p;
        let (lo, hi) = (top, top + runout + 4.0 * p);
        let sketch = Frame::new(Vec3::ZERO, -Vec3::Y, Vec3::X).unwrap();
        let poly = |pts: &[[f64; 2]], name: &str| {
            let n = pts.len();
            Profile {
                loops: vec![ProfileLoop {
                    segments: (0..n)
                        .map(|i| {
                            NamedSegment::new(
                                Segment::Line {
                                    a: Vec2::new(pts[i][0], pts[i][1]),
                                    b: Vec2::new(pts[(i + 1) % n][0], pts[(i + 1) % n][1]),
                                },
                                format!("{name}{i}"),
                            )
                        })
                        .collect(),
                }],
            }
        };
        let shank_pts = [
            [0.0, 0.0],
            [major, 0.0],
            [major, top],
            [minor, top + runout],
            [minor, len],
            [0.0, len],
        ];
        let shank = crate::sweep::revolve(
            &poly(&shank_pts, "shank"),
            &sketch,
            Vec3::ZERO,
            Vec3::Z,
            TAU,
            "shank",
        )
        .unwrap();
        let from = lo - p;
        let (wb, wc) = (width(base), width(major));
        let ridge_profile = poly(
            &[
                [base, from - wb / 2.0],
                [major, from - wc / 2.0],
                [major, from + wc / 2.0],
                [base, from + wb / 2.0],
            ],
            "ridge",
        );
        let turns = (hi + p - from) / p;
        let ridge = helix(
            &ridge_profile,
            &sketch,
            &about_z(p, 1.0),
            TAU * turns,
            "ridge",
        )
        .unwrap();
        let band = crate::sweep::extrude(
            &Profile::circle(Vec2::ZERO, 2.0 * (major + 1.0)),
            &Frame::new(Vec3::new(0.0, 0.0, lo), Vec3::Z, Vec3::X).unwrap(),
            hi - lo,
            "band",
        )
        .unwrap();
        let ridge = crate::boolean::intersection(&ridge, &band).unwrap();
        // ∫ 2π/P ρ (α + βρ) dρ from the base to the shank's radius, by height; Gauss–Legendre on each
        // piece is exact for the cubic it is.
        let slice = |r: f64| {
            let (alpha, beta) = (p / 8.0 + major * flank, -flank);
            let f = |x: f64| alpha * x * x / 2.0 + beta * x.powi(3) / 3.0;
            TAU / p * (f(r) - f(base))
        };
        let core = |z: f64| major - (major - minor) * ((z - top) / runout).clamp(0.0, 1.0);
        let gauss = |a: f64, b: f64| {
            let (m, h) = ((a + b) / 2.0, (b - a) / 2.0);
            let (x, w) = ((3.0f64 / 5.0).sqrt(), [5.0 / 9.0, 8.0 / 9.0, 5.0 / 9.0]);
            h * [-x, 0.0, x]
                .iter()
                .zip(w)
                .map(|(t, w)| w * slice(core(m + h * t)))
                .sum::<f64>()
        };
        let overlap = gauss(lo, top + runout) + gauss(top + runout, hi);
        (shank, ridge, overlap)
    }

    #[test]
    fn a_thread_running_out_onto_its_shank_meets_it_in_its_overlap() {
        let (shank, ridge, overlap) = runout();
        let both = crate::boolean::intersection(&shank, &ridge).unwrap();
        assert!(check::check(&both).is_empty());
        assert!(
            rel(measure::volume(&both), overlap) < 1e-7,
            "{}",
            measure::volume(&both)
        );
    }

    #[test]
    fn a_thread_running_out_onto_its_shank_joins_it() {
        let (shank, ridge, overlap) = runout();
        let joined = crate::boolean::union(&shank, &ridge).unwrap();
        assert!(check::check(&joined).is_empty());
        let want = measure::volume(&shank) + measure::volume(&ridge) - overlap;
        assert!(
            rel(measure::volume(&joined), want) < 1e-7,
            "{}",
            measure::volume(&joined)
        );
    }
}
