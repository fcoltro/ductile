// SPDX-License-Identifier: GPL-3.0-only
//! Blends on everyday parts, one case each: reports which ones fail or
//! leave a broken solid.
//!
//! cargo run --release -p ductile-io --example part_probe [OUT_DIR]
//! With OUT_DIR, each result is also written there as STEP to look at.

use ductile_brep::ops::boolean::{boolean, open_edges, BoolOp};
use ductile_brep::ops::fillet::{blend_edges, Blend};
use ductile_brep::primitives::{make_box, make_cylinder, make_sphere};
use ductile_brep::tess::{tessellate, TessOptions};
use ductile_brep::{Body, EdgeId};
use ductile_geom::{DVec3, Frame};

fn at(x: f64, y: f64, z: f64) -> Frame {
    Frame { origin: DVec3::new(x, y, z), ..Frame::WORLD }
}

fn vol(b: &Body) -> f64 {
    tessellate(b, &TessOptions { chord_tol: 1e-3, angle_tol: 0.1 }).volume()
}

fn edges(b: &Body, f: impl Fn(DVec3, DVec3, DVec3) -> bool) -> Vec<EdgeId> {
    b.edge_ids().filter(|e| { let ed = b.edge(*e); f(ed.start(), ed.end(), ed.mid()) }).collect()
}

fn all(b: &Body) -> Vec<EdgeId> {
    b.edge_ids().filter(|e| b.edge_faces(*e).len() == 2).collect()
}

fn report(name: &str, before: &Body, r: Result<Body, impl std::fmt::Display>) -> Option<Body> {
    match r {
        Ok(b) => {
            if let Some(dir) = std::env::args().nth(1) {
                let slug: String = name.trim().chars().map(|c| if c.is_ascii_alphanumeric() { c.to_ascii_lowercase() } else { '_' }).collect();
                let mut f = std::fs::File::create(std::path::Path::new(&dir).join(format!("{slug}.step"))).unwrap();
                ductile_io::step::write_step(&mut f, name, &[(name, &b)]).unwrap();
            }
            let open = open_edges(&b);
            println!("{:<44} {} faces, volume {:.4} -> {:.4}{}", name, b.faces.len(), vol(before), vol(&b), if open > 0 { format!(", {open} OPEN EDGES") } else { String::new() });
            Some(b)
        }
        Err(e) => {
            println!("{name:<44} FAILED: {e}");
            None
        }
    }
}

fn main() {
    let bx = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
    report("box: all edges r0.3", &bx, blend_edges(&bx, &all(&bx), Blend::Fillet(0.3)));
    report("box: all edges smooth r0.3", &bx, blend_edges(&bx, &all(&bx), Blend::Smooth(0.3)));
    report("box: all edges chamfer 0.3", &bx, blend_edges(&bx, &all(&bx), Blend::Chamfer(0.3)));
    let up = edges(&bx, |p, q, _| (p - q).z.abs() > 1.0);
    if let Some(b) = report("box: uprights r0.6", &bx, blend_edges(&bx, &up, Blend::Fillet(0.6))) {
        let top = edges(&b, |p, q, _| p.z > 1.99 && q.z > 1.99);
        report("  then top rim r0.2", &b, blend_edges(&b, &top, Blend::Fillet(0.2)));
        let rest = all(&b);
        report("  then everything r0.2", &b, blend_edges(&b, &rest, Blend::Fillet(0.2)));
        report("  then everything r0.6 (equal)", &b, blend_edges(&b, &rest, Blend::Fillet(0.6)));
    }
    let top = edges(&bx, |p, q, _| p.z > 1.99 && q.z > 1.99);
    if let Some(b) = report("box: top rim r0.3", &bx, blend_edges(&bx, &top, Blend::Fillet(0.3))) {
        let up = edges(&b, |p, q, _| (p - q).z.abs() > 1.0);
        report("  then uprights r0.6 (larger)", &b, blend_edges(&b, &up, Blend::Fillet(0.6)));
    }

    let boss = boolean(&bx, &make_cylinder(&at(2.0, 1.5, 2.0), 0.8, 1.0), BoolOp::Union).unwrap();
    report("boss: foot r0.2", &boss, blend_edges(&boss, &edges(&boss, |_, _, m| (m.z - 2.0).abs() < 1e-6 && (m - DVec3::new(2.0, 1.5, 2.0)).length() < 0.81), Blend::Fillet(0.2)));
    report("boss: foot + top r0.2", &boss, blend_edges(&boss, &edges(&boss, |_, _, m| m.z > 1.99 && (m - DVec3::new(2.0, 1.5, m.z)).length() < 0.81), Blend::Fillet(0.2)));
    report("boss: every edge r0.15", &boss, blend_edges(&boss, &all(&boss), Blend::Fillet(0.15)));

    let blk = boolean(&bx, &make_box(&at(1.0, 1.0, 2.0), DVec3::new(2.0, 1.0, 1.0)), BoolOp::Union).unwrap();
    let on_blk = edges(&blk, |p, q, _| p.z > 1.99 && q.z > 1.99 && [p, q].iter().all(|v| v.x > 0.99 && v.x < 3.01 && v.y > 0.99 && v.y < 2.01));
    report("block on box: its 12 edges r0.2", &blk, blend_edges(&blk, &on_blk, Blend::Fillet(0.2)));
    report("block on box: every edge r0.2", &blk, blend_edges(&blk, &all(&blk), Blend::Fillet(0.2)));

    let holed = boolean(&bx, &make_cylinder(&at(2.0, 1.5, -1.0), 0.5, 4.0), BoolOp::Difference).unwrap();
    report("through hole: both rims r0.2", &holed, blend_edges(&holed, &edges(&holed, |_, _, m| (m - DVec3::new(2.0, 1.5, m.z)).length() < 0.51), Blend::Fillet(0.2)));
    report("through hole: every edge r0.2", &holed, blend_edges(&holed, &all(&holed), Blend::Fillet(0.2)));
    let blind = boolean(&bx, &make_cylinder(&at(2.0, 1.5, 1.0), 0.5, 4.0), BoolOp::Difference).unwrap();
    report("blind hole: rim + floor r0.15", &blind, blend_edges(&blind, &edges(&blind, |_, _, m| (m - DVec3::new(2.0, 1.5, m.z)).length() < 0.51), Blend::Fillet(0.15)));
    let pocket = boolean(&bx, &make_box(&at(1.0, 1.0, 1.0), DVec3::new(2.0, 1.0, 2.0)), BoolOp::Difference).unwrap();
    report("pocket: every edge r0.15", &pocket, blend_edges(&pocket, &all(&pocket), Blend::Fillet(0.15)));

    let tee = boolean(&make_cylinder(&Frame::new(DVec3::new(-2.0, 0.0, 0.0), DVec3::X, DVec3::Y), 0.6, 4.0), &make_cylinder(&at(0.0, 0.0, 0.0), 0.4, 1.5), BoolOp::Union).unwrap();
    report("pipe tee: junction r0.1", &tee, blend_edges(&tee, &edges(&tee, |_, _, m| m.z > 0.0 && m.z < 0.65 && m.x.abs() < 0.41), Blend::Fillet(0.1)));
    let ball = boolean(&bx, &make_sphere(DVec3::new(2.0, 1.5, 2.0), 1.0), BoolOp::Union).unwrap();
    report("dome on box: foot r0.2", &ball, blend_edges(&ball, &edges(&ball, |_, _, m| (m.z - 2.0).abs() < 1e-6 && m.x > 0.9 && m.x < 3.1 && m.y > 0.4 && m.y < 2.6), Blend::Fillet(0.2)));
}
