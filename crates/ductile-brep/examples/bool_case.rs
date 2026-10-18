// SPDX-License-Identifier: GPL-3.0-only
//! Scratch reproduction of one boolean case (edit to taste).
use ductile_brep::ops::boolean::{boolean, open_edges, BoolOp};
use ductile_brep::primitives::make_cylinder;
use ductile_brep::tess::{tessellate, TessOptions};
use ductile_geom::{DVec3, Frame};

fn main() {
    let a = make_cylinder(&Frame::new(DVec3::new(-1.0, 0.0, -0.40012521063672857), DVec3::Y, DVec3::Y.any_orthonormal_vector()), 0.75, 1.5);
    let b = make_cylinder(&Frame::new(DVec3::new(-0.5, 0.5, -0.4522784852919963), DVec3::Z, DVec3::Z.any_orthonormal_vector()), 1.25, 2.5);
    if std::env::var_os("DEBUG").is_some() { unsafe { std::env::set_var("DUCTILE_DEBUG_BOOL", "1") } }
    for op in [BoolOp::Union, BoolOp::Intersection, BoolOp::Difference] {
        match boolean(&a, &b, op) {
            Ok(r) => println!("{op:?}: {} faces, {} open, volume {:.5}", r.faces.len(), open_edges(&r), tessellate(&r, &TessOptions { chord_tol: 2e-4, angle_tol: 0.05 }).volume()),
            Err(e) => println!("{op:?}: {e}"),
        }
    }
}
