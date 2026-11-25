// yack: Booleans of many solids at once, against closed forms and against the same booleans one at a
// Adapted from springrep@a03e8a1 tests/many.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: crate paths springrep:: -> yack_springrep::; this header added. Moved to crates/yack/tests at W11 (springrep deleted): paths to the crates the items live in.
//! Booleans of many solids at once, against closed forms and against the same booleans one at a
//! time.
// W11: springrep's own kernel tests, moved here when springrep went (ruling (b)). They build with the
// kernel crates directly and compute their expected values, so the facade's maths ban and the
// workspace lints' test-only exemptions (which reach only #[test] fns) do not fit them.
// TODO(api-owner, hardening, D26): rewrite these tests to the lints, test by test, and drop this allow.
#![allow(clippy::disallowed_methods, clippy::disallowed_types, clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::indexing_slicing)]

use std::f64::consts::PI;

use yack_boolean::difference;
use yack_boolean::difference_all;
use yack_boolean::union;
use yack_boolean::union_all;
use yack_check as check;
use yack_math::kernel::Frame;
use yack_math::kernel::Vec3;
use yack_measure as measure;
use yack_build::primitive::block;
use yack_build::primitive::cylinder;
use yack_build::primitive::sphere;
use yack_topo::Brep;

fn at(x: f64, y: f64, z: f64) -> Frame {
    Frame::new(Vec3::new(x, y, z), Vec3::Z, Vec3::X).unwrap()
}

fn assert_solid(b: &Brep, volume: f64, area: f64) {
    let issues = check::check(b);
    assert!(issues.is_empty(), "{issues:?}");
    let (v, a) = (measure::volume(b), measure::area(b));
    assert!(
        (v - volume).abs() <= 1e-9 * volume,
        "volume {v} vs {volume}"
    );
    assert!((a - area).abs() <= 1e-9 * area, "area {a} vs {area}");
}

fn holes(nx: usize, ny: usize, first: f64, pitch: f64, r: f64) -> Vec<Brep> {
    let mut out = Vec::new();
    for i in 0..nx {
        for j in 0..ny {
            let f = at(first + i as f64 * pitch, first + j as f64 * pitch, -1.0);
            out.push(cylinder(&f, r, 4.0, "hole").unwrap());
        }
    }
    out
}

#[test]
fn a_plate_less_sixteen_holes_apart_is_the_plate_drilled() {
    let plate = block(&Frame::WORLD, Vec3::new(20.0, 20.0, 2.0), "plate").unwrap();
    let tools = holes(4, 4, 2.5, 5.0, 1.0);
    let refs: Vec<&Brep> = tools.iter().collect();
    let got = difference_all(&plate, &refs).unwrap();
    let top = 400.0 - 16.0 * PI;
    assert_solid(&got, top * 2.0, 2.0 * top + 160.0 + 16.0 * 2.0 * PI * 2.0);
    assert_eq!(got.solid_ids().count(), 1);
}

#[test]
fn holes_that_overlap_cut_as_they_do_one_at_a_time() {
    let plate = block(&Frame::WORLD, Vec3::new(12.0, 8.0, 2.0), "plate").unwrap();
    // Each hole shares a lens with the next along its row and its column.
    let tools = holes(4, 3, 2.0, 1.5, 1.0);
    let refs: Vec<&Brep> = tools.iter().collect();
    let got = difference_all(&plate, &refs).unwrap();
    let mut one = plate.clone();
    for t in &tools {
        one = difference(&one, t).unwrap();
    }
    assert_solid(&got, measure::volume(&one), measure::area(&one));
}

#[test]
fn blocks_in_an_overlapping_row_unite_into_one_bar() {
    let parts: Vec<Brep> = (0..5)
        .map(|i| block(&at(0.5 * i as f64, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0), "b").unwrap())
        .collect();
    let refs: Vec<&Brep> = parts.iter().collect();
    let got = union_all(&refs).unwrap();
    assert_solid(&got, 3.0, 2.0 * (3.0 + 3.0 + 1.0));
    assert_eq!(got.solid_ids().count(), 1);
    let mut one = parts[0].clone();
    for p in &parts[1..] {
        one = union(&one, p).unwrap();
    }
    assert_solid(&got, measure::volume(&one), measure::area(&one));
}

#[test]
fn solids_apart_unite_as_themselves() {
    let parts: Vec<Brep> = (0..3)
        .map(|i| sphere(&at(5.0 * i as f64, 0.0, 0.0), 1.0, "s").unwrap())
        .collect();
    let refs: Vec<&Brep> = parts.iter().collect();
    let got = union_all(&refs).unwrap();
    assert_solid(&got, 3.0 * 4.0 / 3.0 * PI, 3.0 * 4.0 * PI);
    assert_eq!(got.solid_ids().count(), 3);
}

#[test]
fn nothing_united_is_nothing_and_nothing_taken_away_leaves_the_solid() {
    assert_eq!(union_all(&[]).unwrap().solid_ids().count(), 0);
    let plate = block(&Frame::WORLD, Vec3::new(2.0, 2.0, 2.0), "plate").unwrap();
    assert_solid(&difference_all(&plate, &[]).unwrap(), 8.0, 24.0);
}

#[test]
fn apart_solids_are_combined_in_one_boolean() {
    // A hundred holes apart are one group, so one boolean cuts them all.
    let tools = holes(10, 10, 2.5, 5.0, 1.0);
    let refs: Vec<&Brep> = tools.iter().collect();
    assert_eq!(yack_boolean::apart_groups(&refs).len(), 1);
    // In a grid where each overlaps its four neighbours, no group holds two neighbours.
    let tools = holes(6, 6, 2.0, 1.5, 1.0);
    let refs: Vec<&Brep> = tools.iter().collect();
    let groups = yack_boolean::apart_groups(&refs);
    assert!(groups.len() <= 4, "{}", groups.len());
    assert_eq!(groups.iter().map(Vec::len).sum::<usize>(), 36);
}
