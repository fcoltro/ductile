// SPDX-License-Identifier: GPL-3.0-only
//! Loft: a solid skinned through closed planar profiles.

use crate::profile::{Loop3, Segment};
use crate::topo::{Body, BodyKind, Coedge, EdgeId, VertexId};
use ductile_geom::{Curve3, DVec3, Frame, NurbsCurve, NurbsSurface, Surface};

/// Lofts a solid through two or more closed planar profiles, in order.
///
/// When every profile has the same number of segments, segment `i` of each
/// profile is skinned into its own side face, so corners stay sharp edges
/// (square to square gives four faces). Otherwise each profile is joined into
/// one curve and the side is a single face. The ends are capped by the first
/// and last profiles; sections are placed by chord length between profile
/// centroids, and interpolated with a cubic in the loft direction when there
/// are enough of them.
pub fn loft(profiles: &[Loop3]) -> Option<Body> {
    let m = profiles.len();
    if m < 2 || profiles.iter().any(|p| !p.is_closed() || p.segments.is_empty()) {
        return None;
    }
    let centroids: Vec<DVec3> = profiles
        .iter()
        .map(|p| {
            let pts = p.polygon(1e-3);
            pts.iter().copied().sum::<DVec3>() / pts.len() as f64
        })
        .collect();
    let dir = (centroids[m - 1] - centroids[0]).try_normalize()?;
    // Every profile counter-clockwise seen from the end of the loft.
    let mut loops: Vec<Loop3> = profiles.iter().map(|p| if p.normal().dot(dir) < 0.0 { p.reversed() } else { p.clone() }).collect();
    let n = loops[0].segments.len();
    let matched = n > 1 && loops.iter().all(|l| l.segments.len() == n);
    if matched {
        for k in 1..m {
            loops[k] = align_start(&loops[k - 1], centroids[k - 1], &loops[k], centroids[k]);
        }
    } else {
        // One closed curve per profile, started nearest the first profile's start.
        let mut joined: Vec<Loop3> = Vec::with_capacity(m);
        for k in 0..m {
            let c = NurbsCurve::join(&loops[k].segments.iter().map(directed_nurbs).collect::<Vec<_>>());
            let c = if k == 0 {
                c
            } else {
                let prev = joined[k - 1].segments[0].start() - centroids[k - 1];
                restart_closed(&c, c.closest_param(centroids[k] + prev))
            };
            let (t0, t1) = c.domain();
            joined.push(Loop3 { segments: vec![Segment::new(Curve3::Nurbs(c), t0, t1)] });
        }
        loops = joined;
    }
    // Section parameters by chord length between centroids.
    let mut params = vec![0.0];
    for k in 1..m {
        params.push(params[k - 1] + centroids[k].distance(centroids[k - 1]));
    }
    let total = params[m - 1];
    params.iter_mut().for_each(|p| *p /= total);
    Some(skin(&loops, &params))
}

/// Solid skinned through matching sections: every loop has the same number
/// of segments, segment `i` of each starts at corresponding corners, and the
/// loops run counter-clockwise seen from the last section. Side face `i`
/// interpolates segment `i` of every section at `params`; the first and last
/// sections cap the ends.
pub(crate) fn skin(loops: &[Loop3], params: &[f64]) -> Body {
    let m = loops.len();
    let n = loops[0].segments.len();
    let degree_v = if m >= 4 { 3 } else { m - 1 };
    let centroids: Vec<DVec3> = loops
        .iter()
        .map(|p| {
            let pts = p.polygon(1e-3);
            pts.iter().copied().sum::<DVec3>() / pts.len() as f64
        })
        .collect();
    let mut b = Body::new(BodyKind::Solid);
    // Only the end sections become edges; the inner ones live in the surfaces.
    let ends = [&loops[0], &loops[m - 1]];
    let verts: Vec<Vec<VertexId>> = ends.iter().map(|l| l.segments.iter().map(|s| b.add_vertex(s.start())).collect()).collect();
    // Profile edges, keeping their exact curves; (edge, coedge reversed).
    let prof: Vec<Vec<(EdgeId, bool)>> = ends
        .iter()
        .enumerate()
        .map(|(k, l)| {
            l.segments
                .iter()
                .enumerate()
                .map(|(i, s)| {
                    let (a, z) = (verts[k][i], verts[k][(i + 1) % n]);
                    let (v0, v1) = if s.reversed { (z, a) } else { (a, z) };
                    (b.add_edge(s.curve.clone(), s.t0, s.t1, v0, v1), s.reversed)
                })
                .collect()
        })
        .collect();
    let surfaces: Vec<NurbsSurface> = (0..n)
        .map(|i| {
            let sections: Vec<NurbsCurve> = loops.iter().map(|l| directed_nurbs(&l.segments[i])).collect();
            NurbsSurface::loft_with_params(&sections, degree_v, params)
        })
        .collect();
    // Rail i runs up the start of side face i (and the end of face i - 1).
    let rails: Vec<EdgeId> = (0..n)
        .map(|i| {
            let s = &surfaces[i];
            let ((u0, _), _) = s.domain();
            let c = s.iso_u(u0);
            let (t0, t1) = c.domain();
            b.add_edge(Curve3::Nurbs(c), t0, t1, verts[0][i], verts[1][i])
        })
        .collect();
    for (i, s) in surfaces.into_iter().enumerate() {
        let (e0, r0) = prof[0][i];
        let (e1, r1) = prof[1][i];
        let lp = vec![Coedge::new(e0, r0), Coedge::new(rails[(i + 1) % n], false), Coedge::new(e1, !r1), Coedge::new(rails[i], true)];
        b.add_face(Surface::Nurbs(s), false, vec![lp]);
    }
    let plane = |k: usize| Frame::new(centroids[k], loops[k].normal(), loops[k].segments[0].start() - centroids[k]);
    let cap0: Vec<Coedge> = (0..n).rev().map(|i| Coedge::new(prof[0][i].0, !prof[0][i].1)).collect();
    b.add_face(Surface::Plane { frame: plane(0) }, true, vec![cap0]);
    let cap1: Vec<Coedge> = (0..n).map(|i| Coedge::new(prof[1][i].0, prof[1][i].1)).collect();
    b.add_face(Surface::Plane { frame: plane(m - 1) }, false, vec![cap1]);
    b
}

/// The segment as a NURBS curve running in traversal direction.
pub(crate) fn directed_nurbs(s: &Segment) -> NurbsCurve {
    let c = s.curve.to_nurbs(s.t0, s.t1);
    if s.reversed {
        c.reversed()
    } else {
        c
    }
}

/// `next` with its segments rotated so its corners line up with `prev`'s,
/// comparing positions relative to each profile's centroid.
fn align_start(prev: &Loop3, cp: DVec3, next: &Loop3, cn: DVec3) -> Loop3 {
    let n = next.segments.len();
    let a: Vec<DVec3> = prev.segments.iter().map(|s| s.start() - cp).collect();
    let z: Vec<DVec3> = next.segments.iter().map(|s| s.start() - cn).collect();
    let cost = |r: usize| (0..n).map(|i| a[i].distance(z[(i + r) % n])).sum::<f64>();
    let best = (0..n).min_by(|x, y| cost(*x).total_cmp(&cost(*y))).unwrap_or(0);
    Loop3 { segments: (0..n).map(|i| next.segments[(i + best) % n].clone()).collect() }
}

/// A closed curve restarted at parameter `t`.
fn restart_closed(c: &NurbsCurve, t: f64) -> NurbsCurve {
    let (a, b) = c.domain();
    let span = b - a;
    if t - a < 1e-9 * span || b - t < 1e-9 * span {
        return c.clone();
    }
    let (head, tail) = c.split(t);
    NurbsCurve::join(&[tail, head])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ops::boolean::open_edges;
    use crate::tess::{tessellate, TessOptions};
    use std::f64::consts::PI;

    fn vol(b: &Body) -> f64 {
        tessellate(b, &TessOptions { chord_tol: 1e-4, angle_tol: 0.05 }).volume()
    }

    fn square(c: DVec3, h: f64) -> Loop3 {
        let p = [c + DVec3::new(-h, -h, 0.0), c + DVec3::new(h, -h, 0.0), c + DVec3::new(h, h, 0.0), c + DVec3::new(-h, h, 0.0)];
        Loop3 { segments: (0..4).map(|i| Segment::line(p[i], p[(i + 1) % 4])).collect() }
    }

    fn circle(c: DVec3, r: f64) -> Loop3 {
        Loop3 { segments: vec![Segment::new(Curve3::Circle { frame: Frame { origin: c, ..Frame::WORLD }, radius: r }, 0.0, 2.0 * PI)] }
    }

    #[test]
    fn square_frustum() {
        // Two squares: a frustum of a pyramid, h/3 (A1 + A2 + sqrt(A1 A2)).
        let b = loft(&[square(DVec3::ZERO, 2.0), square(DVec3::new(0.0, 0.0, 3.0), 1.0)]).unwrap();
        assert_eq!(b.faces.len(), 6);
        assert_eq!(open_edges(&b), 0);
        let expect = 3.0 / 3.0 * (16.0 + 4.0 + 8.0);
        assert!((vol(&b) - expect).abs() < 1e-6, "{}", vol(&b));
        // Profile order and direction do not matter.
        let b = loft(&[square(DVec3::new(0.0, 0.0, 3.0), 1.0).reversed(), square(DVec3::ZERO, 2.0)]).unwrap();
        assert!((vol(&b) - expect).abs() < 1e-6, "{}", vol(&b));
    }

    #[test]
    fn circles_through_three_sections() {
        // Equal radii: a cylinder whatever the interpolation.
        let b = loft(&[circle(DVec3::ZERO, 1.0), circle(DVec3::new(0.0, 0.0, 1.0), 1.0), circle(DVec3::new(0.0, 0.0, 2.0), 1.0)]).unwrap();
        assert_eq!(open_edges(&b), 0);
        assert!((vol(&b) - 2.0 * PI).abs() < 2e-3, "{}", vol(&b));
        // A bulge: radius 1, 1.5, 1 at z 0, 1, 2 on a quadratic through them.
        let b = loft(&[circle(DVec3::ZERO, 1.0), circle(DVec3::new(0.0, 0.0, 1.0), 1.5), circle(DVec3::new(0.0, 0.0, 2.0), 1.0)]).unwrap();
        // r(z) = 1 + z - z^2 / 2; volume = pi * integral of r^2 over 0..2.
        let n = 2000;
        let expect: f64 = (0..n)
            .map(|i| {
                let z = 2.0 * (i as f64 + 0.5) / n as f64;
                let r = 1.0 + z - z * z / 2.0;
                PI * r * r * 2.0 / n as f64
            })
            .sum();
        assert!((vol(&b) - expect).abs() < 1e-3 * expect, "{} vs {expect}", vol(&b));
    }

    #[test]
    fn circle_to_square() {
        let b = loft(&[circle(DVec3::ZERO, 10.0), square(DVec3::new(0.0, 0.0, 20.0), 6.0)]).unwrap();
        assert_eq!(b.faces.len(), 3);
        assert_eq!(open_edges(&b), 0);
        // Ruled between them, so between the two prisms' volumes.
        let v = vol(&b);
        assert!(v > 144.0 * 20.0 && v < 100.0 * PI * 20.0, "{v}");
    }
}
