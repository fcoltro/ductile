// SPDX-License-Identifier: GPL-3.0-only
//! Seam edges for faces on periodic surfaces.
//!
//! Some kernels bound a cylindrical band by its two circles alone, or a
//! spherical cap by one circle around the pole. Ductile's tessellator and
//! booleans work in the unwrapped parameter plane and need such faces cut
//! open along an iso-parameter seam, so imported bodies are given one.

use super::boolean::apply_splits;
use crate::topo::{Body, Coedge, EdgeId, FaceId, VertexId};
use ductile_geom::{Curve3, DVec3, Frame, Surface};
use std::collections::HashMap;
use std::f64::consts::{FRAC_PI_2, PI, TAU};

/// Adds seams to every face whose loops wind around a periodic direction
/// without one. Returns the number of seams added.
pub fn add_seams(b: &mut Body) -> usize {
    let tol = ductile_geom::LINEAR_TOL * (b.bbox().diagonal() / 10.0).max(0.01) * 100.0;
    let mut added = 0;
    for fi in 0..b.faces.len() {
        let f = FaceId(fi as u32);
        for along_u in [true, false] {
            if !periodic(&b.faces[fi].surface, along_u) || has_seam(b, f) {
                continue;
            }
            if seam_face(b, f, along_u, tol) {
                added += 1;
            }
        }
    }
    if added > 0 {
        *b = super::heal::compact(b);
    }
    added
}

fn periodic(s: &Surface, along_u: bool) -> bool {
    match s {
        Surface::Cylinder { .. } | Surface::Cone { .. } | Surface::Sphere { .. } => along_u,
        Surface::Torus { .. } => true,
        _ => false,
    }
}

fn has_seam(b: &Body, f: FaceId) -> bool {
    let mut seen: HashMap<EdgeId, usize> = HashMap::new();
    for l in b.face_loops(f) {
        for c in &l.coedges {
            *seen.entry(c.edge).or_default() += 1;
        }
    }
    seen.values().any(|n| *n > 1)
}

/// Parameter along the periodic direction, or `None` at singular points.
fn param(s: &Surface, p: DVec3, along_u: bool) -> Option<f64> {
    let uv = s.project(p, None);
    let singular = match s {
        Surface::Sphere { .. } => uv.y.abs() > FRAC_PI_2 - 1e-6,
        Surface::Cone { radius, half_angle, .. } => (radius + uv.y * half_angle.tan()).abs() < 1e-9,
        _ => false,
    };
    if singular {
        None
    } else {
        Some(if along_u { uv.x } else { uv.y })
    }
}

fn wrap(a: f64) -> f64 {
    (a + PI).rem_euclid(TAU) - PI
}

/// Points of a coedge in traversal order.
fn coedge_points(b: &Body, c: Coedge) -> Vec<DVec3> {
    let e = b.edge(c.edge);
    let n = 48;
    let mut pts: Vec<DVec3> = (0..=n).map(|k| e.curve.point(e.t0 + (e.t1 - e.t0) * k as f64 / n as f64)).collect();
    if c.reversed {
        pts.reverse();
    }
    pts
}

/// Number of turns a loop makes around the periodic direction.
fn winding(b: &Body, s: &Surface, coedges: &[Coedge], along_u: bool) -> i32 {
    let mut total = 0.0;
    let mut last: Option<f64> = None;
    for c in coedges {
        for p in coedge_points(b, *c) {
            if let Some(a) = param(s, p, along_u) {
                if let Some(l) = last {
                    total += wrap(a - l);
                }
                last = Some(a);
            }
        }
    }
    (total / TAU).round() as i32
}

/// Iso-parameter curve through the periodic parameter `a`: the generator,
/// meridian or tube circle for `along_u`, the parallel otherwise.
fn iso_curve(s: &Surface, a: f64, along_u: bool) -> Option<Curve3> {
    let (sa, ca) = a.sin_cos();
    match (s, along_u) {
        (Surface::Cylinder { frame, radius }, true) => Some(Curve3::Line { origin: frame.origin + (frame.x * ca + frame.y * sa) * *radius, dir: frame.z }),
        (Surface::Cone { frame, radius, half_angle }, true) => {
            let d = frame.x * ca + frame.y * sa;
            Some(Curve3::Line { origin: frame.origin + d * *radius, dir: (frame.z + d * half_angle.tan()).normalize() })
        }
        (Surface::Sphere { frame, radius }, true) => {
            let d = frame.x * ca + frame.y * sa;
            Some(Curve3::Circle { frame: Frame { origin: frame.origin, x: d, y: frame.z, z: d.cross(frame.z) }, radius: *radius })
        }
        (Surface::Torus { frame, major, minor }, true) => {
            let d = frame.x * ca + frame.y * sa;
            Some(Curve3::Circle { frame: Frame { origin: frame.origin + d * *major, x: d, y: frame.z, z: d.cross(frame.z) }, radius: *minor })
        }
        (Surface::Torus { frame, major, minor }, false) => {
            Some(Curve3::Circle { frame: Frame { origin: frame.origin + frame.z * (minor * sa), ..*frame }, radius: major + minor * ca })
        }
        _ => None,
    }
}

/// Finds where a loop crosses the periodic parameter `a`: an existing
/// vertex, or a new vertex splitting one of its edges.
fn vertex_at(b: &mut Body, s: &Surface, coedges: &[Coedge], a: f64, along_u: bool, tol: f64) -> Option<VertexId> {
    for c in coedges {
        let (v, _) = b.coedge_vertices(*c);
        if let Some(pa) = param(s, b.vertex(v), along_u) {
            if (wrap(pa - a)).abs() < 1e-7 {
                return Some(v);
            }
        }
    }
    for c in coedges {
        let e = b.edge(c.edge).clone();
        let n = 96;
        let f = |t: f64| param(s, e.curve.point(t), along_u).map(|x| wrap(x - a));
        let ts: Vec<f64> = (0..=n).map(|k| e.t0 + (e.t1 - e.t0) * k as f64 / n as f64).collect();
        for w in ts.windows(2) {
            let (Some(fa), Some(fb)) = (f(w[0]), f(w[1])) else { continue };
            // A sign change that is not the wrap-around jump.
            if fa * fb > 0.0 || (fa - fb).abs() > PI {
                continue;
            }
            let (mut lo, mut hi, mut flo) = (w[0], w[1], fa);
            for _ in 0..60 {
                let mid = 0.5 * (lo + hi);
                let fm = f(mid).unwrap_or(0.0);
                if fm * flo <= 0.0 {
                    hi = mid;
                } else {
                    lo = mid;
                    flo = fm;
                }
            }
            let t = 0.5 * (lo + hi);
            let p = e.curve.point(t);
            for v in [e.v0, e.v1] {
                if b.vertex(v).distance(p) < tol {
                    return Some(v);
                }
            }
            let v = b.add_vertex(p);
            apply_splits(b, HashMap::from([(c.edge, vec![(t, v)])]));
            return Some(v);
        }
    }
    None
}

/// Rotates a loop to start at `v`.
fn start_at(b: &Body, coedges: &[Coedge], v: VertexId) -> Option<Vec<Coedge>> {
    let k = coedges.iter().position(|c| b.coedge_vertices(*c).0 == v)?;
    Some(coedges[k..].iter().chain(&coedges[..k]).copied().collect())
}

/// Seam edge between two vertices along `curve`, with the coedge running
/// from `va` to `vb`.
fn seam_edge(b: &mut Body, curve: Curve3, va: VertexId, vb: VertexId) -> Coedge {
    let period = curve.period();
    let ta = curve.closest_param(b.vertex(va));
    let mut tb = curve.closest_param(b.vertex(vb));
    if let Some(p) = period {
        // Take the shorter way round (seams never exceed half a turn).
        tb = ta + wrap(tb - ta);
        let _ = p;
    }
    if ta < tb {
        Coedge::new(b.add_edge(curve, ta, tb, va, vb), false)
    } else {
        Coedge::new(b.add_edge(curve, tb, ta, vb, va), true)
    }
}

fn reversed(c: Coedge) -> Coedge {
    Coedge::new(c.edge, !c.reversed)
}

fn seam_face(b: &mut Body, f: FaceId, along_u: bool, tol: f64) -> bool {
    let s = b.face(f).surface.clone();
    let loop_ids = b.face(f).loops.clone();
    let winding_loops: Vec<(usize, i32)> = loop_ids
        .iter()
        .enumerate()
        .map(|(k, l)| (k, winding(b, &s, &b.loops[l.idx()].coedges, along_u)))
        .filter(|(_, w)| *w != 0)
        .collect();
    let merged = match winding_loops.as_slice() {
        [(ka, wa), (kb, wb)] if wa * wb < 0 && wa.abs() == 1 => {
            let la = b.loops[loop_ids[*ka].idx()].coedges.clone();
            let Some(&first) = la.iter().find(|c| param(&s, b.vertex(b.coedge_vertices(**c).0), along_u).is_some()) else { return false };
            let va = b.coedge_vertices(first).0;
            let Some(a) = param(&s, b.vertex(va), along_u) else { return false };
            let lb = b.loops[loop_ids[*kb].idx()].coedges.clone();
            let Some(vb) = vertex_at(b, &s, &lb, a, along_u, tol) else { return false };
            let lb = b.loops[loop_ids[*kb].idx()].coedges.clone();
            let (Some(la), Some(lb)) = (start_at(b, &la, va), start_at(b, &lb, vb)) else { return false };
            let Some(curve) = iso_curve(&s, a, along_u) else { return false };
            let seam = seam_edge(b, curve, va, vb);
            let mut l = la;
            l.push(seam);
            l.extend(lb);
            l.push(reversed(seam));
            Some((l, vec![*ka, *kb]))
        }
        [(k, w)] if w.abs() == 1 && along_u && matches!(s, Surface::Sphere { .. } | Surface::Cone { .. }) => {
            // A cap around a pole (or the apex of a cone).
            let la = b.loops[loop_ids[*k].idx()].coedges.clone();
            let Some(&first) = la.iter().find(|c| param(&s, b.vertex(b.coedge_vertices(**c).0), along_u).is_some()) else { return false };
            let va = b.coedge_vertices(first).0;
            let Some(a) = param(&s, b.vertex(va), along_u) else { return false };
            let ccw = (*w > 0) != b.face(f).reversed;
            let pole = match &s {
                Surface::Sphere { frame, radius } => frame.origin + frame.z * if ccw { *radius } else { -*radius },
                Surface::Cone { frame, radius, half_angle } => frame.origin - frame.z * (radius / half_angle.tan()),
                _ => return false,
            };
            let vp = match b.vertices.iter().position(|v| v.point.distance(pole) < tol) {
                Some(i) => VertexId(i as u32),
                None => b.add_vertex(pole),
            };
            let (Some(la), Some(curve)) = (start_at(b, &la, va), iso_curve(&s, a, true)) else { return false };
            let seam = seam_edge(b, curve, va, vp);
            let mut l = la;
            l.push(seam);
            l.push(reversed(seam));
            Some((l, vec![*k]))
        }
        _ => None,
    };
    let Some((l, consumed)) = merged else { return false };
    let new_loop = b.add_loop(l);
    let face = &mut b.faces[f.idx()];
    let rest: Vec<_> = loop_ids.iter().enumerate().filter(|(k, _)| !consumed.contains(k)).map(|(_, l)| *l).collect();
    face.loops = std::iter::once(new_loop).chain(rest).collect();
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ops::boolean::open_edges;
    use crate::tess::{tessellate, TessOptions};
    use crate::topo::BodyKind;

    fn vol(b: &Body) -> f64 {
        tessellate(b, &TessOptions { chord_tol: 1e-4, angle_tol: 0.05 }).volume()
    }

    /// Cylinder whose side is bounded by its two circles only.
    fn seamless_cylinder(r: f64, h: f64) -> Body {
        let mut b = Body::new(BodyKind::Solid);
        let f0 = Frame::WORLD;
        let f1 = Frame { origin: DVec3::new(0.0, 0.0, h), ..f0 };
        // Start the circles at different angles, as other kernels may.
        let v0 = b.add_vertex(DVec3::new(r, 0.0, 0.0));
        let v1 = b.add_vertex(DVec3::new(0.0, r, h));
        let c0 = b.add_edge(Curve3::Circle { frame: f0, radius: r }, 0.0, TAU, v0, v0);
        let c1 = b.add_edge(Curve3::Circle { frame: f1, radius: r }, FRAC_PI_2, FRAC_PI_2 + TAU, v1, v1);
        b.add_face(Surface::Cylinder { frame: f0, radius: r }, false, vec![vec![Coedge::new(c0, false)], vec![Coedge::new(c1, true)]]);
        b.add_face(Surface::Plane { frame: f0 }, true, vec![vec![Coedge::new(c0, true)]]);
        b.add_face(Surface::Plane { frame: f1 }, false, vec![vec![Coedge::new(c1, false)]]);
        b
    }

    #[test]
    fn seams_a_cylinder_band() {
        let mut b = seamless_cylinder(1.0, 2.0);
        assert_eq!(add_seams(&mut b), 1);
        assert_eq!(open_edges(&b), 0);
        assert!((vol(&b) - PI * 2.0).abs() < 2e-3, "{}", vol(&b));
        // Idempotent.
        assert_eq!(add_seams(&mut b), 0);
    }

    #[test]
    fn seams_a_hemisphere_cap() {
        let mut b = Body::new(BodyKind::Solid);
        let v = b.add_vertex(DVec3::new(0.0, 1.0, 0.0));
        let c = b.add_edge(Curve3::Circle { frame: Frame::WORLD, radius: 1.0 }, FRAC_PI_2, FRAC_PI_2 + TAU, v, v);
        b.add_face(Surface::Sphere { frame: Frame::WORLD, radius: 1.0 }, false, vec![vec![Coedge::new(c, false)]]);
        b.add_face(Surface::Plane { frame: Frame::WORLD }, true, vec![vec![Coedge::new(c, true)]]);
        assert_eq!(add_seams(&mut b), 1);
        assert_eq!(open_edges(&b), 0);
        assert!((vol(&b) - 2.0 / 3.0 * PI).abs() < 2e-3, "{}", vol(&b));
    }
}
