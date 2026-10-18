// SPDX-License-Identifier: GPL-3.0-only
//! Extend sheet: grows a surface face past one of its boundary edges,
//! either along its own shape (the last polynomial piece continued, so the
//! extension is curvature continuous and more) or straight along the
//! tangent leaving the edge.

use crate::curves::Extension;
use crate::ops::sheet::Sewer;
use crate::profile::Segment;
use crate::topo::{Body, BodyKind, EdgeId};
use ductile_geom::{DVec3, DVec4, NurbsCurve, NurbsSurface, Surface};

/// A side of a surface's parameter rectangle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Side {
    UStart,
    UEnd,
    VStart,
    VEnd,
}

fn reversed_v(s: &NurbsSurface) -> NurbsSurface {
    s.transposed().reversed_u().transposed()
}

/// `s` grown by about `len` past `side`.
pub fn extend_surface(s: &NurbsSurface, side: Side, len: f64, how: Extension) -> NurbsSurface {
    match side {
        Side::VEnd => extend_v_end(s, len, how),
        Side::VStart => reversed_v(&extend_v_end(&reversed_v(s), len, how)),
        Side::UEnd => extend_v_end(&s.transposed(), len, how).transposed(),
        Side::UStart => reversed_v(&extend_v_end(&reversed_v(&s.transposed()), len, how)).transposed(),
    }
}

/// The last Bézier piece of a curve, in homogeneous control points.
fn last_piece(c: &NurbsCurve) -> Vec<DVec4> {
    c.bezier_segments().pop().unwrap().ctrl
}

/// Control points of the piece continuing `q` from local parameter 1 to
/// `tau`: by blossoming for a natural extension, along the end tangent
/// for a linear one.
fn tail(q: &[DVec4], tau: f64, how: Extension) -> Vec<DVec4> {
    let n = q.len() - 1;
    let linear = how == Extension::Linear && q.iter().all(|h| (h.w - 1.0).abs() < 1e-12);
    if linear {
        let (e, d) = (q[n], q[n] - q[n - 1]);
        return (0..=n).map(|k| e + d * (k as f64 * (tau - 1.0))).collect();
    }
    let blossom = |args: &[f64]| {
        let mut p = q.to_vec();
        for (j, &u) in args.iter().enumerate() {
            for i in 0..n - j {
                p[i] = p[i] * (1.0 - u) + p[i + 1] * u;
            }
        }
        p[0]
    };
    (0..=n)
        .map(|i| {
            let mut args = vec![1.0; n - i];
            args.extend(std::iter::repeat_n(tau, i));
            blossom(&args)
        })
        .collect()
}

fn bezier_length(ctrl: &[DVec4]) -> f64 {
    let n = ctrl.len() - 1;
    let mut knots = vec![0.0; n + 1];
    knots.extend(std::iter::repeat_n(1.0, n + 1));
    NurbsCurve::new(n, knots, ctrl.to_vec()).length()
}

fn extend_v_end(s: &NurbsSurface, len: f64, how: Extension) -> NurbsSurface {
    let n = s.degree_v;
    let column = |i: usize| NurbsCurve::new(n, s.knots_v.clone(), (0..s.nv).map(|j| s.cp(i, j)).collect());
    // The extension is measured along the middle column.
    let mid = last_piece(&column(s.nu / 2));
    let mut tau = 1.0 + len / (bezier_length(&tail(&mid, 1.0 + 1e-6, how)) / 1e-6).max(1e-12);
    for _ in 0..30 {
        let l = bezier_length(&tail(&mid, tau, how));
        let err = len - l;
        if err.abs() < 1e-10 * len.max(1.0) {
            break;
        }
        let speed = l / (tau - 1.0);
        tau = (tau + err / speed.max(1e-12)).max(1.0 + 1e-9);
    }
    let b = *s.knots_v.last().unwrap();
    let k = &s.knots_v;
    let a = k.iter().rev().copied().find(|&x| x < b - 1e-12).unwrap_or(k[0]);
    let b_new = b + (tau - 1.0) * (b - a);
    let mut knots_v: Vec<f64> = k[..k.len() - 1].to_vec();
    knots_v.extend(std::iter::repeat_n(b_new, n + 1));
    let mut ctrl = Vec::with_capacity(s.nu * (s.nv + n));
    for i in 0..s.nu {
        let col = column(i);
        let t = tail(&last_piece(&col), tau, how);
        ctrl.extend((0..s.nv).map(|j| s.cp(i, j)));
        // The tail's first point is the column's last.
        ctrl.extend(t[1..].iter().map(|h| *h * (s.cp(i, s.nv - 1).w / t[0].w)));
    }
    NurbsSurface::new(s.degree_u, n, s.knots_u.clone(), knots_v, s.nu, s.nv + n, ctrl)
}

/// The edges of a face as loop segments.
fn loops(body: &Body, f: crate::topo::FaceId) -> Vec<Vec<Segment>> {
    body.face_loops(f)
        .map(|l| {
            l.coedges
                .iter()
                .map(|c| {
                    let e = body.edge(c.edge);
                    Segment { curve: e.curve.clone(), t0: e.t0, t1: e.t1, reversed: c.reversed }
                })
                .collect()
        })
        .collect()
}

/// Grows the face of sheet `body` along boundary edge `e` by `len`. The
/// face must be untrimmed (bounded by its parameter rectangle).
pub fn extend_sheet(body: &Body, e: EdgeId, len: f64, how: Extension) -> Result<Body, String> {
    let faces = body.edge_faces(e);
    let [(f, _)] = faces[..] else { return Err("pick an open edge of a surface".into()) };
    let face = body.face(f);
    let size = body.bbox().diagonal();
    let tol = 1e-6 * (1.0 + size);
    // The face's surface over its boundary's parameter range.
    let ls = loops(body, f);
    if ls.len() != 1 {
        return Err("the face has holes".into());
    }
    let pts: Vec<DVec3> = ls[0].iter().flat_map(|s| (0..=8).map(move |i| s.curve.point(s.t0 + (s.t1 - s.t0) * i as f64 / 8.0))).collect();
    let ns = match &face.surface {
        Surface::Nurbs(n) => n.clone(),
        other => {
            let uv: Vec<_> = {
                let mut hint = None;
                pts.iter()
                    .map(|p| {
                        let q = other.project(*p, hint);
                        hint = Some(q);
                        q
                    })
                    .collect()
            };
            let (u0, u1) = uv.iter().fold((f64::MAX, f64::MIN), |(a, b), q| (a.min(q.x), b.max(q.x)));
            let (v0, v1) = uv.iter().fold((f64::MAX, f64::MIN), |(a, b), q| (a.min(q.y), b.max(q.y)));
            other.to_nurbs(u0, u1, v0, v1)
        }
    };
    // Untrimmed: every boundary point lies on the rectangle's sides.
    let ((u0, u1), (v0, v1)) = ns.domain();
    let sides = [(Side::UStart, ns.iso_u(u0)), (Side::UEnd, ns.iso_u(u1)), (Side::VStart, ns.iso_v(v0)), (Side::VEnd, ns.iso_v(v1))];
    let near = |c: &NurbsCurve, p: DVec3| {
        let (a, b) = c.domain();
        let t = ductile_geom::Curve3::Nurbs(c.clone()).closest_param_in(p, a, b);
        c.point(t).distance(p)
    };
    if pts.iter().any(|p| sides.iter().all(|(_, c)| near(c, *p) > tol.max(1e-5 * size))) {
        return Err("only untrimmed faces can be extended".into());
    }
    let ed = body.edge(e);
    let probe = [0.25, 0.5, 0.75].map(|f| ed.curve.point(ed.t0 + (ed.t1 - ed.t0) * f));
    let side = sides
        .iter()
        .min_by(|a, b| {
            let d = |c: &NurbsCurve| probe.iter().map(|p| near(c, *p)).sum::<f64>();
            d(&a.1).total_cmp(&d(&b.1))
        })
        .unwrap()
        .0;
    let grown = extend_surface(&ns, side, len, how);
    let mut sw = Sewer::new(body.kind, tol);
    for g in body.face_ids().filter(|g| *g != f) {
        let gl = loops(body, g);
        let other = body.face(g);
        sw.face(other.surface.clone(), other.reversed, &gl[0], &gl[1..]);
    }
    sw.patch_oriented(grown, face.reversed);
    debug_assert!(sw.body.kind == BodyKind::Sheet || body.kind != BodyKind::Sheet);
    Ok(sw.body)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::profile::Loop3;
    use ductile_geom::Curve3;

    fn wave() -> Body {
        let c = NurbsCurve::interpolate(&[DVec3::ZERO, DVec3::new(1.0, 0.0, 0.6), DVec3::new(2.0, 0.0, 0.2), DVec3::new(3.0, 0.0, 0.9)], 3);
        let (t0, t1) = c.domain();
        crate::ops::sheet::extrude_sheet(&[Loop3 { segments: vec![Segment::new(Curve3::Nurbs(c), t0, t1)] }], DVec3::new(0.0, 2.0, 0.0)).unwrap()
    }

    #[test]
    fn natural_extension_continues_the_surface() {
        let b = wave();
        // The free edge at the curve's end (x = 3), running along y.
        let e = b.edge_ids().find(|e| (b.edge(*e).mid().x - 3.0).abs() < 1e-9).unwrap();
        let grown = extend_sheet(&b, e, 1.0, Extension::Natural).unwrap();
        assert_eq!(grown.faces.len(), 1);
        let Surface::Nurbs(s) = &grown.face(crate::topo::FaceId(0)).surface else { panic!() };
        // The edge along the curve, at y = 0, is a unit longer.
        let along = |b: &Body| b.edge_ids().map(|e| b.edge(e)).filter(|e| e.start().y.abs() < 1e-9 && e.end().y.abs() < 1e-9).map(|e| e.length()).sum::<f64>();
        assert!((along(&grown) - along(&b) - 1.0).abs() < 1e-3, "{} vs {}", along(&grown), along(&b));
        // Same curve across the old edge: the section of the grown sheet at
        // y = 1 passes through the old surface's points.
        let Surface::Nurbs(old) = &b.face(crate::topo::FaceId(0)).surface else { panic!() };
        let (_, (v0, v1)) = old.domain();
        for i in 0..=10 {
            let v = v0 + (v1 - v0) * i as f64 / 10.0;
            let p = old.point(old.domain().0 .0, v);
            let q = Surface::Nurbs(s.clone());
            let uv = q.project(p, None);
            assert!(q.point(uv.x, uv.y).distance(p) < 1e-7);
        }
    }

    #[test]
    fn linear_extension_runs_straight_and_planes_grow() {
        let b = wave();
        let e = b.edge_ids().find(|e| (b.edge(*e).mid().x - 3.0).abs() < 1e-9).unwrap();
        let grown = extend_sheet(&b, e, 1.0, Extension::Linear).unwrap();
        let bb = grown.bbox();
        assert!(bb.max.x > 3.2, "{bb:?}");
        // A flat rectangle grows by exactly the length asked.
        let sq = crate::ops::sheet::patch(&Loop3 { segments: (0..4).map(|i| {
            let p = [DVec3::ZERO, DVec3::X, DVec3::new(1.0, 1.0, 0.0), DVec3::Y];
            Segment::line(p[i], p[(i + 1) % 4])
        }).collect() }, &[]).unwrap();
        let e = sq.edge_ids().find(|e| (sq.edge(*e).mid().x - 1.0).abs() < 1e-9).unwrap();
        let g = extend_sheet(&sq, e, 0.5, Extension::Natural).unwrap();
        let bb = g.bbox();
        assert!((bb.max.x - 1.501).abs() < 1e-6 && (bb.max.y - 1.001).abs() < 1e-9, "{bb:?}");
    }
}
