// SPDX-License-Identifier: GPL-3.0-only
//! Writes a plain box to STEP, a simple scene for trying the app's
//! interaction by hand or under xdotool.
use ductile_brep::primitives::make_box;
use ductile_geom::{DVec3, Frame};

fn main() {
    let out = std::env::args().nth(1).expect("usage: box_step OUT.step");
    let b = make_box(&Frame { origin: DVec3::new(-2.0, -1.5, 0.0), ..Frame::WORLD }, DVec3::new(4.0, 3.0, 2.0));
    let mut file = std::fs::File::create(out).unwrap();
    ductile_io::step::write_step(&mut file, "box", &[("Box", &b)]).unwrap();
}
