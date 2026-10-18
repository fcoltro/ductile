// SPDX-License-Identifier: GPL-3.0-only
//! Scratch reproduction of one modeling case (edit to taste).
use ductile_brep::ops::boolean::open_edges;
use ductile_brep::ops::fillet::{blend_edges, Blend};
use ductile_brep::primitives::make_box;
use ductile_brep::tess::{tessellate, TessOptions};
use ductile_brep::{Body, EdgeId};
use ductile_geom::{DVec3, Frame};

fn report(name: &str, r: &Result<Body, impl std::fmt::Display>) {
    match r {
        Ok(b) => println!("{name}: {} faces, {} open, volume {:.5}", b.faces.len(), open_edges(b), tessellate(b, &TessOptions { chord_tol: 2e-4, angle_tol: 0.05 }).volume()),
        Err(e) => println!("{name}: {e}"),
    }
}

fn main() {
    let b = make_box(&Frame::WORLD, DVec3::new(3.0, 2.0, 1.0));
    let top: Vec<EdgeId> = b.edge_ids().filter(|e| b.edge(*e).start().z > 0.9 && b.edge(*e).end().z > 0.9).collect();
    for (first, second) in [(Blend::Chamfer(0.3), Blend::Chamfer(0.2)), (Blend::Fillet(0.3), Blend::Fillet(0.2)), (Blend::Chamfer(0.3), Blend::Fillet(0.2)), (Blend::Fillet(0.3), Blend::Chamfer(0.2)), (Blend::Chamfer(0.2), Blend::Chamfer(0.3)), (Blend::Fillet(0.2), Blend::Fillet(0.3))] {
        for subset in [&top[..1], &top[..2], &top[..]] {
            let a = blend_edges(&b, subset, first).unwrap();
            let vert: Vec<EdgeId> = a.edge_ids().filter(|e| {
                let ed = a.edge(*e);
                (ed.start().x - ed.end().x).abs() < 1e-9 && (ed.start().y - ed.end().y).abs() < 1e-9 && (ed.start().z - ed.end().z).abs() > 0.3
            }).collect();
            for v in &vert {
                let r = blend_edges(&a, &[*v], second);
                if !matches!(&r, Ok(x) if open_edges(x) == 0) {
                    report(&format!("{first:?} on {} top edges, then {second:?} on vertical e{}", subset.len(), v.0), &r);
                }
            }
        }
    }
}
