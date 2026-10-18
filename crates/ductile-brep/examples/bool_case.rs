// SPDX-License-Identifier: GPL-3.0-only
//! Scratch reproduction of one modeling case (edit to taste).
use ductile_brep::ops::boolean::{boolean, open_edges, BoolOp};
use ductile_brep::ops::fillet::{blend_edges, Blend};
use ductile_brep::primitives::{make_box, make_cylinder};
use ductile_brep::EdgeId;
use ductile_geom::{DVec3, Frame};

fn main() {
    let bx = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
    let post = boolean(&bx, &make_cylinder(&Frame { origin: DVec3::new(4.0, 3.0, 0.0), ..Frame::WORLD }, 0.8, 3.0), BoolOp::Union).unwrap();
    let all: Vec<EdgeId> = post.edge_ids().collect();
    for e in &all {
        let ed = post.edge(*e);
        let faces: Vec<String> = post.edge_faces(*e).iter().map(|(f, _)| format!("{:?}", post.face(*f).surface).split([' ', '(', '{']).next().unwrap_or("").to_string()).collect();
        println!("e{} {:.2} -> {:.2} {:?}", e.0, ed.start(), ed.end(), faces);
    }
    for skip in &all {
        let set: Vec<EdgeId> = all.iter().copied().filter(|x| x != skip).collect();
        let ok = blend_edges(&post, &set, Blend::Fillet(0.2)).map(|b| open_edges(&b) == 0).unwrap_or(false);
        if ok {
            println!("works without e{}", skip.0);
        }
    }
}
