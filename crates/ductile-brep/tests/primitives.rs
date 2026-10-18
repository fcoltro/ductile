use ductile_brep::primitives::*;
use ductile_brep::*;
use ductile_geom::{DVec3, Frame};
use std::collections::HashMap;
use std::f64::consts::PI;

fn check(b: &Body, expected_volume: f64, rel: f64) -> BodyMesh {
    let m = tessellate(b, &TessOptions { chord_tol: 1e-3, angle_tol: 0.1 });
    let v = m.volume();
    assert!(((v - expected_volume) / expected_volume).abs() < rel, "volume {v} vs {expected_volume}");
    // Watertight: every undirected mesh edge used exactly twice, in opposite directions.
    let mut pos_id: HashMap<[u64; 3], usize> = HashMap::new();
    let mut directed: HashMap<(usize, usize), i32> = HashMap::new();
    for (t, _) in m.triangles() {
        let ids: Vec<usize> = t
            .iter()
            .map(|p| {
                let k = [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
                let n = pos_id.len();
                *pos_id.entry(k).or_insert(n)
            })
            .collect();
        for i in 0..3 {
            let (a, c) = (ids[i], ids[(i + 1) % 3]);
            if a == c {
                continue;
            }
            *directed.entry((a, c)).or_default() += 1;
        }
    }
    let mut bad = 0;
    for (&(a, c), &n) in &directed {
        let back = directed.get(&(c, a)).copied().unwrap_or(0);
        if n != back {
            bad += 1;
        }
    }
    assert_eq!(bad, 0, "mesh is not watertight ({} bad edges of {})", bad, directed.len());
    m
}

#[test]
fn box_volume() {
    let b = make_box(&Frame::WORLD, DVec3::new(2.0, 3.0, 4.0));
    check(&b, 24.0, 1e-12);
}

#[test]
fn cylinder_volume() {
    let f = Frame::new(DVec3::new(1.0, 2.0, 3.0), DVec3::new(0.0, 1.0, 1.0), DVec3::X);
    let b = make_cylinder(&f, 2.0, 5.0);
    check(&b, PI * 4.0 * 5.0, 2e-3);
}

#[test]
fn cone_volume() {
    let b = make_frustum(&Frame::WORLD, 2.0, 0.0, 3.0);
    check(&b, PI * 4.0 * 3.0 / 3.0, 2e-3);
    let b = make_frustum(&Frame::WORLD, 2.0, 1.0, 3.0);
    check(&b, PI * 3.0 / 3.0 * (4.0 + 2.0 + 1.0), 2e-3);
}

#[test]
fn sphere_volume() {
    let b = make_sphere(DVec3::new(0.5, 0.0, 0.0), 2.0);
    check(&b, 4.0 / 3.0 * PI * 8.0, 3e-3);
}

#[test]
fn torus_volume() {
    let b = make_torus(&Frame::WORLD, 5.0, 1.0);
    check(&b, 2.0 * PI * PI * 5.0 * 1.0, 3e-3);
}

#[test]
fn mirrored_box_volume() {
    let b = make_box(&Frame::WORLD, DVec3::new(2.0, 3.0, 4.0));
    let m = ductile_geom::DMat4::from_scale(DVec3::new(-1.0, 1.0, 1.0));
    check(&b.transformed(&m), 24.0, 1e-12);
    let c = make_cylinder(&Frame::WORLD, 1.0, 2.0).transformed(&m);
    check(&c, PI * 2.0, 3e-3);
}

