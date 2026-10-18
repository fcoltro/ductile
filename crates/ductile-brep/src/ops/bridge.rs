// SPDX-License-Identifier: GPL-3.0-only
//! Bridge surface: a sheet spanning two edges, meeting the faces beyond
//! them with position (G0), tangent (G1) or curvature (G2) continuity.
//!
//! Across the bridge each section is a quintic Bézier: its first three
//! control points are set by the first edge's face (the point, the
//! tangent leaving the face, the curvature the face has in that
//! direction) and its last three by the second's. Sections at matching
//! arc length along the two edges are interpolated along the edges.

use crate::ops::sheet::Sewer;
use crate::topo::{Body, BodyKind, EdgeId};
use ductile_geom::{Curve3, DVec3, DVec4, NurbsCurve, NurbsSurface, Surface};

/// Continuity with the face beyond an edge.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Continuity {
    G0,
    G1,
    G2,
}

/// One end of a bridge: an edge, and the face beyond it if it has one.
#[derive(Clone, Debug)]
pub struct Side {
    pub curve: Curve3,
    pub t0: f64,
    pub t1: f64,
    /// The face's surface, whether its outward normal is the surface's
    /// reversed, and whether the face runs along the edge the other way
    /// (its loop uses the edge reversed).
    pub face: Option<(Surface, bool, bool)>,
    pub continuity: Continuity,
    /// Scales how far the bridge leaves the face before turning.
    pub tension: f64,
}

impl Side {
    /// Edge `e` of `body`, against the face beyond it that leans most
    /// towards `toward` (a boundary edge of a sheet has just one).
    pub fn of_edge(body: &Body, e: EdgeId, toward: DVec3, continuity: Continuity, tension: f64) -> Side {
        let ed = body.edge(e);
        let (t0, t1) = (ed.t0, ed.t1);
        let mid = ed.curve.point(0.5 * (t0 + t1));
        let face = body
            .edge_faces(e)
            .into_iter()
            .map(|(f, c)| {
                let face = body.face(f);
                (face.surface.clone(), face.reversed, c.reversed)
            })
            .max_by(|a, b| {
                let lean = |s: &(Surface, bool, bool)| leave(&ed.curve, 0.5 * (t0 + t1), s).map_or(f64::MIN, |(d, _, _)| d.dot(toward - mid));
                lean(a).total_cmp(&lean(b))
            });
        Side { curve: ed.curve.clone(), t0, t1, face, continuity, tension }
    }
}

/// At parameter `t` of an edge: the unit direction leaving the face
/// across it, the face's outward normal, and the face's normal curvature
/// in that direction (signed along that normal).
fn leave(curve: &Curve3, t: f64, face: &(Surface, bool, bool)) -> Option<(DVec3, DVec3, f64)> {
    let (s, reversed, against) = face;
    let p = curve.point(t);
    let uv = s.project(p, None);
    let d = s.derivs(uv.x, uv.y);
    let mut n = d.du.cross(d.dv).try_normalize()?;
    if *reversed {
        n = -n;
    }
    let along = curve.tangent(t) * if *against { -1.0 } else { 1.0 };
    // The face lies to the left of its loop: leaving it is to the right.
    let out = along.cross(n).try_normalize()?;
    // Normal curvature along `out`: II(w, w) / I(w, w) with w = a du + b dv.
    let (e, f, g) = (d.du.dot(d.du), d.du.dot(d.dv), d.dv.dot(d.dv));
    let det = e * g - f * f;
    if det.abs() < 1e-300 {
        return None;
    }
    let (ou, ov) = (out.dot(d.du), out.dot(d.dv));
    let (a, b) = ((g * ou - f * ov) / det, (e * ov - f * ou) / det);
    let (l, m, nn) = (d.duu.dot(n), d.duv.dot(n), d.dvv.dot(n));
    let k = (l * a * a + 2.0 * m * a * b + nn * b * b) / (e * a * a + 2.0 * f * a * b + g * b * b);
    Some((out, n, k))
}

/// Parameters splitting the edge into `n` pieces of equal length.
fn by_length(c: &Curve3, t0: f64, t1: f64, n: usize) -> Vec<f64> {
    const FINE: usize = 400;
    let ts: Vec<f64> = (0..=FINE).map(|i| t0 + (t1 - t0) * i as f64 / FINE as f64).collect();
    let mut acc = vec![0.0];
    for w in ts.windows(2) {
        acc.push(acc.last().unwrap() + c.point(w[0]).distance(c.point(w[1])));
    }
    let total = *acc.last().unwrap();
    let mut j = 0;
    (0..=n)
        .map(|i| {
            let s = total * i as f64 / n as f64;
            while j + 1 < FINE && acc[j + 1] < s {
                j += 1;
            }
            let span = acc[j + 1] - acc[j];
            let f = if span > 0.0 { ((s - acc[j]) / span).clamp(0.0, 1.0) } else { 0.0 };
            ts[j] + (ts[j + 1] - ts[j]) * f
        })
        .collect()
}

/// The three control points a side sets: on the edge, then leaving the
/// face. `span` is the distance across the bridge there and `other` the
/// matching point on the far edge.
fn rows(side: &Side, t: f64, span: f64, other: DVec3) -> [DVec3; 3] {
    let p = side.curve.point(t);
    let lin = |f: f64| p.lerp(other, f);
    let Some((out, n, k)) = side.face.as_ref().and_then(|f| leave(&side.curve, t, f)).filter(|_| side.continuity != Continuity::G0) else {
        return [p, lin(0.2), lin(0.4)];
    };
    // Quintic: C'(0) = 5 (P1 - P0), C''(0) = 20 (P2 - 2 P1 + P0).
    let d1 = out * span * side.tension;
    let p1 = p + d1 / 5.0;
    let p2 = match side.continuity {
        Continuity::G2 => 2.0 * p1 - p + n * (k * d1.length_squared()) / 20.0,
        _ => p + d1 * 2.0 / 5.0,
    };
    [p, p1, p2]
}

/// A sheet bridging `a` and `b`, or None when they are degenerate.
pub fn bridge_surface(a: &Side, b: &Side) -> Option<Body> {
    const N: usize = 32;
    let ta = by_length(&a.curve, a.t0, a.t1, N);
    let mut tb = by_length(&b.curve, b.t0, b.t1, N);
    // Run the second edge the same way as the first.
    let (a0, a1) = (a.curve.point(ta[0]), a.curve.point(ta[N]));
    let (b0, b1) = (b.curve.point(tb[0]), b.curve.point(tb[N]));
    if a0.distance(b1) + a1.distance(b0) < a0.distance(b0) + a1.distance(b1) {
        tb.reverse();
    }
    // Six rows of control points across, N + 1 points along each.
    let mut grid = vec![Vec::with_capacity(N + 1); 6];
    for i in 0..=N {
        let (pa, pb) = (a.curve.point(ta[i]), b.curve.point(tb[i]));
        let span = pa.distance(pb);
        if span < 1e-9 {
            return None;
        }
        let ra = rows(a, ta[i], span, pb);
        let rb = rows(b, tb[i], span, pa);
        for (j, p) in [ra[0], ra[1], ra[2], rb[2], rb[1], rb[0]].into_iter().enumerate() {
            grid[j].push(p);
        }
    }
    let params: Vec<f64> = (0..=N).map(|i| i as f64 / N as f64).collect();
    let curves: Vec<NurbsCurve> = grid.iter().map(|row| NurbsCurve::interpolate_with_params(row, 3, &params)).collect();
    let nu = curves[0].ctrl.len();
    let mut ctrl: Vec<DVec4> = Vec::with_capacity(nu * 6);
    for i in 0..nu {
        for c in &curves {
            ctrl.push(c.ctrl[i]);
        }
    }
    let knots_v = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
    let s = NurbsSurface::new(3, 5, curves[0].knots.clone(), knots_v, nu, 6, ctrl);
    let size = a0.distance(b0).max(a1.distance(b1)).max(a0.distance(a1));
    let mut sw = Sewer::new(BodyKind::Sheet, 1e-6 * (1.0 + size));
    sw.patch(s);
    Some(sw.body)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::make_box;
    use ductile_geom::Frame;

    /// Two boxes side by side with a gap; bridge their facing top edges.
    fn top_edge(b: &Body, x: f64) -> EdgeId {
        b.edge_ids()
            .find(|e| {
                let ed = b.edge(*e);
                (ed.start().x - x).abs() < 1e-9 && (ed.end().x - x).abs() < 1e-9 && (ed.start().z - 1.0).abs() < 1e-9 && (ed.end().z - 1.0).abs() < 1e-9
            })
            .unwrap()
    }

    #[test]
    fn bridges_two_box_tops_tangentially() {
        let l = make_box(&Frame::WORLD, DVec3::new(1.0, 2.0, 1.0));
        let r = make_box(&Frame { origin: DVec3::new(3.0, 0.0, 0.0), ..Frame::WORLD }, DVec3::new(1.0, 2.0, 1.0));
        let (el, er) = (top_edge(&l, 1.0), top_edge(&r, 3.0));
        for c in [Continuity::G0, Continuity::G1, Continuity::G2] {
            let a = Side::of_edge(&l, el, DVec3::new(3.0, 1.0, 1.0), c, 1.0);
            let b = Side::of_edge(&r, er, DVec3::new(1.0, 1.0, 1.0), c, 1.0);
            let sheet = bridge_surface(&a, &b).unwrap();
            assert_eq!(sheet.faces.len(), 1);
            let Surface::Nurbs(s) = &sheet.face(crate::topo::FaceId(0)).surface else { panic!() };
            // Across the middle: flat between two flat tops.
            for u in [0.1, 0.5, 0.9] {
                let p = s.point(u, 0.5);
                assert!((p.z - 1.0).abs() < 1e-9 && (p.x - 2.0).abs() < 1e-6, "{c:?} {p}");
                // Leaving the left box: along +X, level with its top.
                let d = s.derivs(u, 0.0, 1)[0][1];
                if c != Continuity::G0 {
                    assert!(d.normalize().distance(DVec3::X) < 1e-6, "{c:?} {d}");
                }
            }
        }
    }

    #[test]
    fn meets_a_cylinder_with_its_curvature() {
        // A half cylinder sheet's straight edge bridged to a parallel line
        // of a box: at G2 the bridge's curvature across matches the
        // cylinder's 1 / r where they meet.
        let cyl = crate::primitives::make_cylinder(&Frame::WORLD, 1.0, 2.0);
        let line = cyl.edge_ids().find(|e| cyl.edge(*e).curve.is_linear()).unwrap();
        let ed = cyl.edge(line);
        let p = ed.start();
        let radial = DVec3::new(p.x, p.y, 0.0).normalize();
        let tangent = DVec3::Z.cross(radial);
        let target = p + tangent * 3.0 + radial * 0.5;
        let b_body = make_box(&Frame { origin: target - DVec3::new(0.5, 0.5, 0.0), ..Frame::WORLD }, DVec3::new(1.0, 1.0, 2.0));
        let e2 = b_body.edge_ids().find(|e| b_body.edge(*e).curve.is_linear() && (b_body.edge(*e).end() - b_body.edge(*e).start()).z.abs() > 1.9).unwrap();
        let a = Side::of_edge(&cyl, line, target, Continuity::G2, 1.0);
        let b = Side { face: None, continuity: Continuity::G0, ..Side::of_edge(&b_body, e2, p, Continuity::G0, 1.0) };
        let sheet = bridge_surface(&a, &b).unwrap();
        let Surface::Nurbs(s) = &sheet.face(crate::topo::FaceId(0)).surface else { panic!() };
        let d = s.derivs(0.5, 0.0, 2);
        let (c1, c2) = (d[0][1], d[0][2]);
        let k = c1.cross(c2).length() / c1.length().powi(3);
        assert!((k - 1.0).abs() < 1e-3, "curvature {k}");
    }
}
