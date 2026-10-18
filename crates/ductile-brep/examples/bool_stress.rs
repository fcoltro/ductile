// SPDX-License-Identifier: GPL-3.0-only
//! Boolean stress test: pairs of primitives at pseudo-random placements
//! (including touching and coincident ones), checked for closed results
//! and for the volume identities
//! V(A u B) + V(A n B) = V(A) + V(B) and V(A - B) = V(A) - V(A n B).
//!
//! cargo run --release -p ductile-brep --example bool_stress [COUNT] [SEED]

use ductile_brep::ops::boolean::{boolean, open_edges, BoolError, BoolOp};
use ductile_brep::primitives::{make_box, make_cylinder, make_frustum, make_sphere, make_torus};
use ductile_brep::tess::{tessellate, TessOptions};
use ductile_brep::Body;
use ductile_geom::{DVec3, Frame};

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    /// Mostly round numbers on a coarse grid, to hit coincidences.
    fn coord(&mut self) -> f64 {
        if self.next() < 0.5 {
            ((self.next() * 4.0).round() - 2.0) * 0.5
        } else {
            self.next() * 2.0 - 1.0
        }
    }
    fn dir(&mut self) -> DVec3 {
        match (self.next() * 4.0) as u32 {
            0 => DVec3::X,
            1 => DVec3::Y,
            2 => DVec3::Z,
            _ => DVec3::new(self.next() - 0.5, self.next() - 0.5, self.next() - 0.5).normalize(),
        }
    }
}

fn shape(r: &mut Rng) -> (String, Body) {
    let o = DVec3::new(r.coord(), r.coord(), r.coord());
    let z = r.dir();
    let x = z.any_orthonormal_vector();
    let f = Frame::new(o, z, x);
    let s = 1.0 + (r.next() * 4.0).round() * 0.5;
    match (r.next() * 5.0) as u32 {
        0 => (format!("box {o} {z} {s}"), make_box(&f, DVec3::new(s, s * 0.75, s * 0.5 + 0.5))),
        1 => (format!("cyl {o} {z} {s}"), make_cylinder(&f, s * 0.5, s)),
        2 => (format!("sphere {o} {s}"), make_sphere(o, s * 0.5)),
        3 => (format!("cone {o} {z} {s}"), make_frustum(&f, s * 0.5, s * 0.2, s)),
        _ => (format!("torus {o} {z} {s}"), make_torus(&f, s * 0.6, s * 0.2)),
    }
}

fn vol(b: &Body) -> f64 {
    tessellate(b, &TessOptions { chord_tol: 2e-4, angle_tol: 0.05 }).volume()
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let count: usize = args.get(1).and_then(|a| a.parse().ok()).unwrap_or(200);
    let mut rng = Rng(args.get(2).and_then(|a| a.parse().ok()).unwrap_or(0x9E3779B97F4A7C15));
    let (mut bad, mut empty) = (0, 0);
    for i in 0..count {
        let (na, a) = shape(&mut rng);
        let (nb, b) = shape(&mut rng);
        let only: Option<usize> = std::env::var("ONLY").ok().and_then(|v| v.parse().ok());
        if only.is_some_and(|o| o != i) {
            continue;
        }
        let (va, vb) = (vol(&a), vol(&b));
        if only.is_some() {
            println!("A {na}: {} faces, volume {va:.5}; B {nb}: {} faces, volume {vb:.5}", a.faces.len(), b.faces.len());
            for op in [BoolOp::Union, BoolOp::Intersection, BoolOp::Difference] {
                match boolean(&a, &b, op) {
                    Ok(r) => println!("{op:?}: {} faces, {} open, volume {:.5}", r.faces.len(), open_edges(&r), vol(&r)),
                    Err(e) => println!("{op:?}: {e}"),
                }
            }
        }
        let run = |op| match boolean(&a, &b, op) {
            Ok(r) if open_edges(&r) > 0 => Err(format!("{op:?}: {} open edges", open_edges(&r))),
            Ok(r) => Ok(vol(&r)),
            Err(BoolError::Empty) => Ok(0.0),
            Err(e) => Err(format!("{op:?}: {e}")),
        };
        let (u, n, d) = (run(BoolOp::Union), run(BoolOp::Intersection), run(BoolOp::Difference));
        let mut problems: Vec<String> = [&u, &n, &d].iter().filter_map(|r| r.as_ref().err().cloned()).collect();
        if let (Ok(u), Ok(n), Ok(d)) = (&u, &n, &d) {
            let tol = 2e-3 * (va + vb);
            if (u + n - va - vb).abs() > tol {
                problems.push(format!("union + intersection = {:.5}, expected {:.5}", u + n, va + vb));
            }
            if (d - (va - n)).abs() > tol {
                problems.push(format!("difference = {d:.5}, expected {:.5}", va - n));
            }
            if *n == 0.0 {
                empty += 1;
            }
        }
        if !problems.is_empty() {
            bad += 1;
            println!("#{i}: A = {na}; B = {nb}");
            for p in problems {
                println!("    {p}");
            }
        }
    }
    println!("{bad} of {count} pairs had problems ({empty} pairs did not overlap)");
}
