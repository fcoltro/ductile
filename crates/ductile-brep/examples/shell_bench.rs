// SPDX-License-Identifier: GPL-3.0-only
//! Times shelling and meshing a filleted bar.
use ductile_brep::ops::fillet::{blend_edges, Blend};
use ductile_brep::ops::heal::compact;
use ductile_brep::ops::offset::shell;
use ductile_brep::primitives::make_box;
use ductile_brep::{tessellate, TessOptions};
use ductile_geom::{DVec3, Frame};

fn main() {
    let b = make_box(&Frame::WORLD, DVec3::new(6.0, 4.0, 2.0));
    let vertical: Vec<_> = b.edge_ids().filter(|e| (b.edge(*e).start() - b.edge(*e).end()).z.abs() > 1.0).collect();
    let b = compact(&blend_edges(&b, &vertical, Blend::Fillet(1.2)).unwrap());
    let bottom: Vec<_> = b.edge_ids().filter(|e| b.edge(*e).start().z.abs() < 1e-9 && b.edge(*e).end().z.abs() < 1e-9).collect();
    let b = compact(&blend_edges(&b, &bottom, Blend::Fillet(0.5)).unwrap());
    let top = b.face_ids().find(|f| b.face_edges(*f).iter().all(|e| b.edge(*e).start().z > 1.99 && b.edge(*e).end().z > 1.99)).unwrap();
    let s = shell(&b, &[top], 0.15).unwrap();
    for _ in 0..3 {
        let clock = std::time::Instant::now();
        let m = tessellate(&s, &TessOptions::default());
        eprintln!("{:?} {}", clock.elapsed(), m.triangle_count());
    }
}
