// SPDX-License-Identifier: GPL-3.0-only
//! Scratch reproduction of one modeling case (edit to taste).
use ductile_brep::ops::boolean::{boolean, open_edges, BoolOp};
use ductile_brep::ops::fillet::{blend_edges, Blend};
use ductile_brep::primitives::{make_box, make_cylinder};
use ductile_brep::tess::{tessellate, TessOptions};
use ductile_brep::EdgeId;
use ductile_geom::{DVec3, Frame};

fn main() {
    let bx = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
    let v = make_cylinder(&Frame { origin: DVec3::new(2.0, 1.5, -1.0), ..Frame::WORLD }, 0.5, 4.0);
    let h = make_cylinder(&Frame::new(DVec3::new(-1.0, 1.7, 1.0), DVec3::X, DVec3::Y), 0.35, 6.0);
    let a = boolean(&boolean(&bx, &v, BoolOp::Difference).unwrap(), &h, BoolOp::Difference).unwrap();
    let vol = |b: &ductile_brep::Body| tessellate(b, &TessOptions { chord_tol: 2e-4, angle_tol: 0.05 }).volume();
    let cross: Vec<EdgeId> = a.edge_ids().filter(|e| {
        let ed = a.edge(*e);
        let faces: Vec<bool> = a.edge_faces(*e).iter().map(|(f, _)| matches!(a.face(*f).surface, ductile_geom::Surface::Cylinder { .. })).collect();
        faces.len() == 2 && faces.iter().all(|x| *x) && (ed.mid().x - 2.0).abs() < 0.6
    }).collect();
    println!("{} crossing edges: {:?}", cross.len(), cross.iter().map(|e| e.0).collect::<Vec<_>>());
    for blend in [Blend::Fillet(0.08), Blend::Chamfer(0.08)] {
        for set in [&cross[..1], &cross[..]] {
            match blend_edges(&a, set, blend) {
                Ok(r) => println!("{blend:?} {} edges: {} open, removed {:.5}", set.len(), open_edges(&r), vol(&a) - vol(&r)),
                Err(e) => println!("{blend:?} {} edges: {e}", set.len()),
            }
        }
    }
}
