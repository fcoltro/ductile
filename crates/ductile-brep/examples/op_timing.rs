// SPDX-License-Identifier: GPL-3.0-only
//! Times common modeling operations, to keep interactive previews quick.
//!
//! cargo run --release -p ductile-brep --example op_timing
use ductile_brep::ops::boolean::{boolean, BoolOp};
use ductile_brep::ops::fillet::{blend_edges, Blend};
use ductile_brep::primitives::{make_box, make_cylinder};
use ductile_brep::tess::{tessellate, TessOptions};
use ductile_brep::EdgeId;
use ductile_geom::{DVec3, Frame};
use std::time::Instant;

fn at(x: f64, y: f64, z: f64) -> Frame {
    Frame { origin: DVec3::new(x, y, z), ..Frame::WORLD }
}

fn time<T>(name: &str, f: impl FnOnce() -> T) -> T {
    let t = Instant::now();
    let r = f();
    println!("{name:40} {:8.1} ms", t.elapsed().as_secs_f64() * 1e3);
    r
}

fn main() {
    let bx = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
    let cyl = make_cylinder(&at(2.0, 1.5, -1.0), 0.8, 4.0);
    let holed = time("box - cylinder", || boolean(&bx, &cyl, BoolOp::Difference).unwrap());
    time("box + cylinder", || boolean(&bx, &make_cylinder(&at(2.0, 1.5, 1.0), 0.8, 2.0), BoolOp::Union).unwrap());
    let edges: Vec<EdgeId> = bx.edge_ids().collect();
    time("fillet 12 box edges", || blend_edges(&bx, &edges, Blend::Fillet(0.3)).unwrap());
    time("fillet 1 box edge", || blend_edges(&bx, &edges[..1], Blend::Fillet(0.3)).unwrap());
    let he: Vec<EdgeId> = holed.edge_ids().collect();
    let filleted = time("fillet all edges of holed box", || blend_edges(&holed, &he, Blend::Fillet(0.2)).unwrap());
    time("tessellate filleted holed box", || tessellate(&filleted, &TessOptions::default()));
    match time("filleted - cylinder", || boolean(&filleted, &make_cylinder(&at(0.5, 0.5, -1.0), 0.3, 4.0), BoolOp::Difference)) { Ok(_) => {} Err(e) => println!("  -> error {e:?}") }
    for (x, y, r) in [(0.5, 0.5, 0.25), (0.6, 0.5, 0.3), (2.0, 0.3, 0.2), (0.5, 0.5, 0.35)] {
        let c = make_cylinder(&at(x, y, -1.0), r, 4.0);
        match boolean(&filleted, &c, BoolOp::Difference) { Ok(_) => println!("  ok {x} {y} {r}"), Err(e) => println!("  -> {x} {y} {r} error {e:?}") }
    }
}
