// SPDX-License-Identifier: GPL-3.0-only
//! Scratch reproduction of one boolean case (edit to taste).
use ductile_brep::ops::boolean::{boolean, open_edges, BoolOp};
use ductile_brep::ops::fillet::{blend_edges, Blend};
use ductile_brep::primitives::{make_box, make_cylinder};
use ductile_brep::tess::{tessellate, TessOptions};
use ductile_brep::EdgeId;
use ductile_geom::{DVec3, Frame};

fn at(x: f64, y: f64, z: f64) -> Frame {
    Frame { origin: DVec3::new(x, y, z), ..Frame::WORLD }
}

fn main() {
    let r: f64 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(0.35);
    let bx = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
    let edges: Vec<EdgeId> = bx.edge_ids().collect();
    let a = blend_edges(&bx, &edges, Blend::Fillet(0.2)).unwrap();
    let b = make_cylinder(&at(0.5, 0.5, -1.0), r, 4.0);
    for e in a.edge_ids() {
        let ed = a.edge(e);
        let (p, q) = (a.vertex(ed.v0), a.vertex(ed.v1));
        if p.z.abs() < 0.01 && q.z.abs() < 0.01 && (p.x < 0.5 || q.x < 0.5) {
            let kind = match &ed.curve { ductile_geom::Curve3::Line { .. } => "line", ductile_geom::Curve3::Circle { .. } => "circle", ductile_geom::Curve3::Nurbs(_) => "nurbs", _ => "other" };
            let ts: Vec<f64> = b.face_ids().flat_map(|f| ductile_brep::ssi::curve_surface_crossings(&ed.curve, ed.t0, ed.t1, &b.face(f).surface, 1e-6)).collect();
            println!("e{} {kind} {p:.3} -> {q:.3} crossings {:?}", e.0, ts.iter().map(|&t| ed.curve.point(t)).collect::<Vec<_>>());
        }
    }
    if std::env::var_os("DEBUG").is_some() { unsafe { std::env::set_var("DUCTILE_DEBUG_BOOL", "1") } }
    for op in [BoolOp::Union, BoolOp::Intersection, BoolOp::Difference] {
        match boolean(&a, &b, op) {
            Ok(r) => println!("{op:?}: {} faces, {} open, volume {:.5}", r.faces.len(), open_edges(&r), tessellate(&r, &TessOptions { chord_tol: 2e-4, angle_tol: 0.05 }).volume()),
            Err(e) => println!("{op:?}: {e}"),
        }
    }
}
