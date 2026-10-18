// SPDX-License-Identifier: GPL-3.0-only
//! Writes a hexagonal prism with every edge given a smooth (G2) blend to
//! STEP, for checking oblique corner patches in other viewers.
use ductile_brep::ops::fillet::{blend_edges, Blend};
use ductile_brep::primitives::polyhedron;
use ductile_brep::{BodyKind, EdgeId};
use ductile_geom::DVec3;
use std::f64::consts::PI;

fn main() {
    let out = std::env::args().nth(1).expect("usage: hex_step OUT.step");
    let (r0, h, r) = (3.0, 2.0, 0.4);
    let mut pts = Vec::new();
    for z in [0.0, h] {
        for k in 0..6 {
            let a = k as f64 * PI / 3.0;
            pts.push(DVec3::new(r0 * a.cos(), r0 * a.sin(), z));
        }
    }
    let mut faces = vec![(0..6).rev().collect::<Vec<_>>(), (6..12).collect()];
    for k in 0..6 {
        faces.push(vec![k, (k + 1) % 6, 6 + (k + 1) % 6, 6 + k]);
    }
    let b = polyhedron(&pts, &faces, BodyKind::Solid);
    let all: Vec<EdgeId> = b.edge_ids().collect();
    let f = blend_edges(&b, &all, Blend::Smooth(r)).unwrap();
    let mut file = std::fs::File::create(out).unwrap();
    ductile_io::step::write_step(&mut file, "hex", &[("Hex", &f)]).unwrap();
}
