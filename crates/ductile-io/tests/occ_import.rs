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

#[test]
fn assembly_instances_are_placed() {
    let path = format!("{}/tests/data/occ_assembly.step", env!("CARGO_MANIFEST_DIR"));
    let imp = ductile_io::step_read::read_step(&std::fs::read_to_string(path).unwrap()).unwrap();
    assert!(imp.warnings.is_empty(), "{:?}", imp.warnings);
    let names: Vec<&str> = imp.bodies.iter().map(|b| b.name.as_str()).collect();
    assert_eq!(names, ["block", "block", "pin"]);
    let opts = TessOptions { chord_tol: 1e-3, angle_tol: 0.1 };
    let vols: Vec<f64> = imp.bodies.iter().map(|b| tessellate(&b.body, &opts).volume()).collect();
    assert!((vols[0] - 24.0).abs() < 1e-6 && (vols[1] - 24.0).abs() < 1e-6, "{vols:?}");
    assert!((vols[2] - std::f64::consts::PI * 5.0).abs() < 0.05, "{vols:?}");
    // The block (4 x 3 x 2 at the origin) is placed as is, then moved to
    // x = 10 and turned a quarter turn about Z; the pin stands at y = 20.
    let bb: Vec<_> = imp.bodies.iter().map(|b| b.body.bbox()).collect();
    let near = |a: ductile_geom::DVec3, b: [f64; 3]| (a - ductile_geom::DVec3::from(b)).length() < 1e-2;
    assert!(near(bb[0].min, [0.0, 0.0, 0.0]) && near(bb[0].max, [4.0, 3.0, 2.0]), "{:?}", bb[0]);
    assert!(near(bb[1].min, [7.0, 0.0, 0.0]) && near(bb[1].max, [10.0, 4.0, 2.0]), "{:?}", bb[1]);
    assert!((bb[2].min.y - 19.0).abs() < 1e-2 && (bb[2].max.z - 5.0).abs() < 1e-2, "{:?}", bb[2]);
}
