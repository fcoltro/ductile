// SPDX-License-Identifier: GPL-3.0-only
//! Imports STEP files written by OpenCascade and checks the volumes it
//! reports for them, so the reader is held to a second kernel's answer.

use ductile_brep::ops::boolean::open_edges;
use ductile_brep::{tessellate, TessOptions};

fn check(name: &str, faces: usize, volume: f64) {
    let path = format!("{}/tests/data/{name}.step", env!("CARGO_MANIFEST_DIR"));
    let imp = ductile_io::step_read::read_step(&std::fs::read_to_string(path).unwrap()).unwrap();
    assert_eq!(imp.bodies.len(), 1, "{name}");
    let b = &imp.bodies[0].body;
    assert_eq!(b.faces.len(), faces, "{name}");
    assert_eq!(open_edges(b), 0, "{name}");
    let v = tessellate(b, &TessOptions { chord_tol: 1e-3, angle_tol: 0.1 }).volume();
    assert!((v - volume).abs() < 1e-3 * volume, "{name}: {v} vs {volume}");
}

#[test]
fn cone() {
    check("occ_cone", 3, 2183.4069);
}

#[test]
fn extruded_ellipse() {
    check("occ_extruded_ellipse", 4, 1884.9556);
}

#[test]
fn filleted_box() {
    check("occ_fillet_box", 27, 21078.9027);
}

#[test]
fn loft() {
    check("occ_loft", 7, 4494.9012);
}

#[test]
fn sphere() {
    check("occ_sphere", 1, 4188.7902);
}

#[test]
fn torus() {
    check("occ_torus", 1, 9869.6044);
}

#[test]
fn vase() {
    check("occ_vase", 1, 1507.9645);
}
