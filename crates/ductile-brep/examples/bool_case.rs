// SPDX-License-Identifier: GPL-3.0-only
//! Scratch reproduction of one boolean case (edit to taste).
use ductile_brep::ops::boolean::{boolean, open_edges, BoolOp};
use ductile_brep::primitives::{make_box, make_torus};
use ductile_brep::tess::{tessellate, TessOptions};
use ductile_geom::{DQuat, DVec3, Frame};

fn main() {
    let o = DVec3::new(-1.0, -0.1841472796211352, -0.1271061059801859);
    let z = DVec3::Z;
    let x0 = z.any_orthonormal_vector();
    let b = {
        let z = DVec3::Y;
        let f = Frame::new(DVec3::new(0.0, -0.5, 0.0), z, z.any_orthonormal_vector());
        make_box(&f, DVec3::new(1.5, 1.5 * 0.75, 1.5 * 0.5 + 0.5))
    };
    for k in 0..1 {
        let x = DQuat::from_axis_angle(z, 0.7 * k as f64) * x0;
        let a = make_torus(&Frame::new(o, z, x), 1.5, 0.5);
        for op in [BoolOp::Union, BoolOp::Intersection, BoolOp::Difference] {
            match boolean(&a, &b, op) {
                Ok(r) => println!("turn {k} {op:?}: {} faces, {} open, volume {:.5}", r.faces.len(), open_edges(&r), tessellate(&r, &TessOptions { chord_tol: 2e-4, angle_tol: 0.05 }).volume()),
                Err(e) => println!("turn {k} {op:?}: {e}"),
            }
        }
    }
}
