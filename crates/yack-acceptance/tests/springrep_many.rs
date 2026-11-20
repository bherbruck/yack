// yack: booleans of many solids at once, against closed forms and against the same booleans one at a time.
// Adapted from springrep@a03e8a1 tests/many.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); springrep `Frame::new(o, Z, X)` is
// `Frame::at(o)`; a solid count is `betti()[0]` and an empty result `is_empty()`; the test of springrep's
// internal `apart_groups` is omitted as an API gap.

use std::f64::consts::PI;

use yack::prelude::*;

fn at(x: f64, y: f64, z: f64) -> Frame {
    Frame::at([x, y, z])
}

fn assert_solid(b: &Solid, volume: f64, area: f64) {
    let issues = b.check();
    assert!(issues.is_empty(), "{issues:?}");
    let (v, a) = (b.volume(), b.area());
    assert!((v - volume).abs() <= 1e-9 * volume, "volume {v} vs {volume}");
    assert!((a - area).abs() <= 1e-9 * area, "area {a} vs {area}");
}

fn holes(nx: usize, ny: usize, first: f64, pitch: f64, r: f64) -> Vec<Solid> {
    let mut out = Vec::new();
    for i in 0..nx {
        for j in 0..ny {
            let f = at(first + i as f64 * pitch, first + j as f64 * pitch, -1.0);
            out.push(cylinder(&f, r, 4.0).unwrap());
        }
    }
    out
}

#[test]
fn a_plate_less_sixteen_holes_apart_is_the_plate_drilled() {
    let plate = block(&Frame::WORLD, [20.0, 20.0, 2.0]).unwrap();
    let tools = holes(4, 4, 2.5, 5.0, 1.0);
    let got = plate.difference_all(&tools).unwrap();
    let top = 400.0 - 16.0 * PI;
    assert_solid(&got, top * 2.0, 2.0 * top + 160.0 + 16.0 * 2.0 * PI * 2.0);
    assert_eq!(got.betti()[0], 1);
}

#[test]
fn holes_that_overlap_cut_as_they_do_one_at_a_time() {
    let plate = block(&Frame::WORLD, [12.0, 8.0, 2.0]).unwrap();
    // Each hole shares a lens with the next along its row and its column.
    let tools = holes(4, 3, 2.0, 1.5, 1.0);
    let got = plate.difference_all(&tools).unwrap();
    let mut one = plate.clone();
    for t in &tools {
        one = one.difference(t).unwrap();
    }
    assert_solid(&got, one.volume(), one.area());
}

#[test]
fn blocks_in_an_overlapping_row_unite_into_one_bar() {
    let parts: Vec<Solid> = (0..5).map(|i| block(&at(0.5 * i as f64, 0.0, 0.0), [1.0, 1.0, 1.0]).unwrap()).collect();
    let got = Solid::union_all(&parts).unwrap();
    assert_solid(&got, 3.0, 2.0 * (3.0 + 3.0 + 1.0));
    assert_eq!(got.betti()[0], 1);
    let mut one = parts[0].clone();
    for p in &parts[1..] {
        one = one.union(p).unwrap();
    }
    assert_solid(&got, one.volume(), one.area());
}

#[test]
fn solids_apart_unite_as_themselves() {
    let parts: Vec<Solid> = (0..3).map(|i| sphere(&at(5.0 * i as f64, 0.0, 0.0), 1.0).unwrap()).collect();
    let got = Solid::union_all(&parts).unwrap();
    assert_solid(&got, 3.0 * 4.0 / 3.0 * PI, 3.0 * 4.0 * PI);
    assert_eq!(got.betti()[0], 3);
}

#[test]
fn nothing_united_is_nothing_and_nothing_taken_away_leaves_the_solid() {
    assert!(Solid::union_all(&[]).unwrap().is_empty());
    let plate = block(&Frame::WORLD, [2.0, 2.0, 2.0]).unwrap();
    assert_solid(&plate.difference_all(&[]).unwrap(), 8.0, 24.0);
}

// API GAP (test apart_solids_are_combined_in_one_boolean omitted): it asserts only on springrep's
// internal `boolean::apart_groups` (how many-tool booleans are batched into groups of mutually
// apart tools); the yack public API exposes no grouping, and the batching is an implementation detail.
