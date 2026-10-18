// SPDX-License-Identifier: GPL-3.0-only
//! Fillets and chamfers every edge of a set of test shapes, one at a time
//! and all together, and reports the ones that fail or come out broken
//! (open edges, or a volume change of the wrong sign).
//!
//! cargo run --release -p ductile-brep --example fillet_stress

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

fn shapes() -> Vec<(&'static str, Body)> {
    let bx = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
    let ell = boolean(&bx, &make_box(&at(2.0, 0.0, 1.0), DVec3::new(2.0, 3.0, 1.0)), BoolOp::Difference).unwrap();
    let holed = boolean(&bx, &make_cylinder(&at(2.0, 1.5, -1.0), 0.8, 4.0), BoolOp::Difference).unwrap();
    let boss = boolean(&bx, &make_cylinder(&at(2.0, 1.5, 1.0), 0.8, 2.0), BoolOp::Union).unwrap();
    let pocket = boolean(&bx, &make_box(&at(1.0, 1.0, 1.0), DVec3::new(2.0, 1.0, 2.0)), BoolOp::Difference).unwrap();
    let notch = boolean(&bx, &make_cylinder(&Frame::new(DVec3::new(-1.0, 1.5, 2.0), DVec3::X, DVec3::Y), 0.7, 6.0), BoolOp::Difference).unwrap();
    let dome = boolean(&make_cylinder(&Frame::WORLD, 1.5, 2.0), &make_sphere(DVec3::new(0.0, 0.0, 2.0), 1.5), BoolOp::Union).unwrap();
    let cross = boolean(&make_box(&at(-2.0, -0.5, 0.0), DVec3::new(4.0, 1.0, 1.0)), &make_box(&at(-0.5, -2.0, 0.0), DVec3::new(1.0, 4.0, 1.0)), BoolOp::Union).unwrap();
    let corner_post = boolean(&bx, &make_cylinder(&at(4.0, 3.0, 0.0), 0.8, 3.0), BoolOp::Union).unwrap();
    let corner_bite = boolean(&bx, &make_cylinder(&at(4.0, 0.0, -1.0), 1.2, 4.0), BoolOp::Difference).unwrap();
    vec![("post", corner_post), ("bite", corner_bite), ("box", bx), ("ell", ell), ("holed", holed), ("boss", boss), ("pocket", pocket), ("notch", notch), ("dome", dome), ("cross", cross), ("cylinder", make_cylinder(&Frame::WORLD, 1.0, 2.0))]
}

fn check(name: &str, what: &str, base: f64, r: Result<Body, impl std::fmt::Display>, fails: &mut Vec<String>) {
    match r {
        Err(e) => {
            let e = e.to_string();
            // Seams and smooth edges picked alone rightly have nothing to blend.
            if !(e.contains("seam") || e.contains("smooth")) {
                fails.push(format!("{name} {what}: error {e}"));
            }
        }
        Ok(b) => {
            let open = open_edges(&b);
            let v = vol(&b);
            if open > 0 || v <= 0.0 || (v - base).abs() > 0.5 * base {
                fails.push(format!("{name} {what}: open {open}, volume {v:.4} vs {base:.4}"));
            }
        }
    }
}

fn main() {
    // `fillet_stress SHAPE EDGE` describes one edge and its faces.
    let args: Vec<String> = std::env::args().collect();
    // `fillet_stress SHAPE pairs` blends each pair of edges meeting at a vertex.
    if args.len() == 3 && args[2] == "pairs" {
        let b = shapes().into_iter().find(|s| s.0 == args[1]).expect("shape").1;
        let es: Vec<EdgeId> = b.edge_ids().collect();
        for (i, a) in es.iter().enumerate() {
            for c in &es[i + 1..] {
                let (ea, ec) = (b.edge(*a), b.edge(*c));
                if [ea.v0, ea.v1].iter().any(|v| *v == ec.v0 || *v == ec.v1) {
                    for blend in [Blend::Fillet(0.2), Blend::Chamfer(0.2)] {
                        if let Err(e) = blend_edges(&b, &[*a, *c], blend) {
                            println!("{blend:?} {} + {}: {e}", a.0, c.0);
                        }
                    }
                }
            }
        }
        return;
    }
    if args.len() == 4 {
        // `fillet_stress SHAPE A B`: blend A, then the remains of B.
        let b = shapes().into_iter().find(|s| s.0 == args[1]).expect("shape").1;
        let (ea, eb) = (EdgeId(args[2].parse().unwrap()), EdgeId(args[3].parse().unwrap()));
        let r = blend_edges(&b, &[ea], Blend::Fillet(0.2)).unwrap();
        let orig = b.edge(eb);
        let on = |p: DVec3| orig.curve.point(orig.curve.closest_param_in(p, orig.t0, orig.t1)).distance(p) < 1e-6;
        let now: Vec<EdgeId> = r.edge_ids().filter(|x| on(r.edge(*x).start()) && on(r.edge(*x).end()) && on(r.edge(*x).mid())).collect();
        println!("B is now {:?}", now);
        println!("{:?}", blend_edges(&r, &now, Blend::Fillet(0.2)).map(|x| open_edges(&x)));
        return;
    }
    if args.len() == 3 {
        let b = shapes().into_iter().find(|s| s.0 == args[1]).expect("shape").1;
        let e = EdgeId(args[2].parse().unwrap());
        let ed = b.edge(e);
        println!("edge {:?} {} -> {} curve {:?}", e, ed.start(), ed.end(), ed.curve);
        for (f, c) in b.edge_faces(e) {
            println!("  face {:?} rev {} surface {:?}", f, c.reversed, b.face(f).surface);
        }
        println!("fillet: {:?}", blend_edges(&b, &[e], Blend::Fillet(0.2)).map(|r| (r.faces.len(), open_edges(&r))));
        if let Ok(r) = blend_edges(&b, &[e], Blend::Fillet(0.2)) {
            for x in r.edge_ids() {
                let ed = r.edge(x);
                println!("  after: {:?} {} -> {} faces {}", x, ed.start(), ed.end(), r.edge_faces(x).len());
            }
        }
        return;
    }
    let mut fails = Vec::new();
    let mut runs = 0;
    for (name, b) in shapes() {
        let base = vol(&b);
        let edges: Vec<EdgeId> = b.edge_ids().collect();
        for blend in [Blend::Fillet(0.2), Blend::Chamfer(0.2)] {
            for e in &edges {
                runs += 1;
                check(name, &format!("{blend:?} edge {}", e.0), base, blend_edges(&b, &[*e], blend), &mut fails);
            }
            runs += 1;
            check(name, &format!("{blend:?} all edges"), base, blend_edges(&b, &edges, blend), &mut fails);
        }
    }
    for f in &fails {
        println!("{f}");
    }
    println!("{} of {runs} blends failed", fails.len());
}
