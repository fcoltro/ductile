// SPDX-License-Identifier: GPL-3.0-only
//! Unwrap face: a face laid out flat, as a planar sheet.

use crate::profile::{Loop3, Segment};
use crate::tess::{face_coedge_uvs, tessellate_edge, TessOptions};
use crate::topo::{Body, FaceId};
use ductile_geom::{Curve3, DVec2, DVec3, Frame, Surface};

/// Where a surface point lands when the surface is laid flat: exact for
/// planes, cylinders and cones; other surfaces are spread by arc length
/// along their u lines and their middle v line.
fn flatten(s: &Surface, uv: DVec2, lo: DVec2, hi: DVec2) -> DVec2 {
    match s {
        Surface::Plane { frame } => frame.to_local(s.point(uv.x, uv.y)).truncate(),
        Surface::Cylinder { radius, .. } => DVec2::new(radius * uv.x, uv.y),
        Surface::Cone { radius, half_angle, .. } => {
            let (sa, r) = (half_angle.sin(), radius + uv.y * half_angle.tan());
            if sa.abs() < 1e-12 {
                return DVec2::new(radius * uv.x, uv.y);
            }
            let (slant, th) = (r / sa, uv.x * sa);
            DVec2::new(slant * th.cos(), slant * th.sin())
        }
        _ => {
            let speed_u = |u: f64, v: f64| s.derivs(u, v).du.length();
            let speed_v = |u: f64, v: f64| s.derivs(u, v).dv.length();
            let along = |f: &dyn Fn(f64) -> f64, a: f64, b: f64| {
                // Composite Gauss–Legendre, 3 points per piece.
                let n = 24;
                let (g, w) = ([-0.774_596_669_241_483_4, 0.0, 0.774_596_669_241_483_4], [5.0 / 9.0, 8.0 / 9.0, 5.0 / 9.0]);
                let h = (b - a) / n as f64;
                (0..n).map(|i| {
                    let m = a + h * (i as f64 + 0.5);
                    (0..3).map(|k| w[k] * f(m + 0.5 * h * g[k])).sum::<f64>() * 0.5 * h
                }).sum()
            };
            let mid_u = 0.5 * (lo.x + hi.x);
            DVec2::new(along(&|u| speed_u(u, uv.y), mid_u, uv.x), along(&|v| speed_v(mid_u, v), lo.y, uv.y))
        }
    }
}

/// Face `f` of `body` laid flat on `onto`'s xy plane, as a planar sheet
/// with the face's holes.
pub fn unwrap_face(body: &Body, f: FaceId, onto: &Frame) -> Option<Body> {
    let opts = TessOptions::for_size(body.bbox().diagonal().max(1e-3));
    let edges: Vec<Vec<(f64, DVec3)>> = body.edge_ids().map(|e| tessellate_edge(body, e, &opts)).collect();
    let loops = face_coedge_uvs(body, f, &edges);
    let surf = &body.face(f).surface;
    let all: Vec<DVec2> = loops.iter().flatten().flat_map(|(_, s)| s.iter().map(|b| b.uv)).collect();
    let lo = all.iter().fold(DVec2::splat(f64::MAX), |a, b| a.min(*b));
    let hi = all.iter().fold(DVec2::splat(f64::MIN), |a, b| a.max(*b));
    let mut flat: Vec<Loop3> = Vec::new();
    for lp in &loops {
        let mut segs = Vec::new();
        for (_, samples) in lp {
            let pts: Vec<DVec3> = samples.iter().map(|b| {
                let q = flatten(surf, b.uv, lo, hi);
                onto.to_world(DVec3::new(q.x, q.y, 0.0))
            }).collect();
            let mut pts2: Vec<DVec3> = Vec::with_capacity(pts.len());
            for p in pts {
                if pts2.last().is_none_or(|l: &DVec3| l.distance(p) > 1e-12) {
                    pts2.push(p);
                }
            }
            if pts2.len() < 2 {
                continue;
            }
            // Straight runs stay lines; the rest become splines through the samples.
            let (a, b) = (pts2[0], *pts2.last().unwrap());
            let straight = pts2.iter().all(|p| crate::ssi::dist_point_segment3(*p, a, b) < 1e-9 * (1.0 + a.distance(b)));
            if straight {
                segs.push(Segment::line(a, b));
            } else {
                let n = pts2.len().clamp(4, 40);
                let c = if pts2.len() <= 4 { ductile_geom::NurbsCurve::interpolate(&pts2, (pts2.len() - 1).min(3)) } else { crate::curves::fit_curve(&pts2, n, 3)? };
                let (t0, t1) = c.domain();
                segs.push(Segment::new(Curve3::Nurbs(c), t0, t1));
            }
        }
        if !segs.is_empty() {
            flat.push(Loop3 { segments: segs });
        }
    }
    let (outer, holes) = flat.split_first()?;
    super::sheet::patch(outer, holes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::{make_box, make_cylinder};
    use ductile_geom::Surface;

    fn area(b: &Body) -> f64 {
        let m = crate::tess::tessellate(b, &TessOptions { chord_tol: 1e-4, angle_tol: 0.05 });
        m.faces.iter().map(|f| f.triangles.iter().map(|t| {
            let [a, b, c] = t.map(|i| f.positions[i as usize]);
            0.5 * (b - a).cross(c - a).length()
        }).sum::<f64>()).sum()
    }

    #[test]
    fn unwraps_a_cylinder_and_a_box_face() {
        let cyl = make_cylinder(&Frame::WORLD, 1.0, 3.0);
        let side: Vec<FaceId> = cyl.face_ids().filter(|f| matches!(cyl.face(*f).surface, Surface::Cylinder { .. })).collect();
        let total: f64 = side.iter().map(|f| area(&unwrap_face(&cyl, *f, &Frame::WORLD).unwrap())).sum();
        assert!((total - 2.0 * std::f64::consts::PI * 3.0).abs() < 1e-3, "{total}");
        for f in &side {
            let u = unwrap_face(&cyl, *f, &Frame::WORLD).unwrap();
            let m = crate::tess::tessellate(&u, &TessOptions::for_size(10.0));
            assert!(m.faces.iter().flat_map(|f| f.positions.iter()).all(|p| p.z.abs() < 1e-9));
        }
        // A frustum's side unrolls to an annulus sector of the same area.
        let fr = crate::primitives::make_frustum(&Frame::WORLD, 2.0, 1.0, 2.0);
        let side: Vec<FaceId> = fr.face_ids().filter(|f| matches!(fr.face(*f).surface, Surface::Cone { .. })).collect();
        let total: f64 = side.iter().map(|f| area(&unwrap_face(&fr, *f, &Frame::WORLD).unwrap())).sum();
        let want = std::f64::consts::PI * 3.0 * 5f64.sqrt();
        assert!((total - want).abs() < 1e-3, "{total} vs {want}");
        let bx = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
        let u = unwrap_face(&bx, FaceId(0), &Frame::WORLD).unwrap();
        let a = area(&u);
        assert!([12.0, 8.0, 6.0].iter().any(|x| (a - x).abs() < 1e-9), "{a}");
    }
}
