// SPDX-License-Identifier: GPL-3.0-only
//! Isoparam: splits a face in two along one of its surface's
//! iso-parametric curves, keeping the surface (and the faces around it)
//! as they were, with the neighbours' edges split where the new edge
//! meets them.

use crate::ops::sheet::Sewer;
use crate::profile::Segment;
use crate::topo::{Body, BodyKind, FaceId};
use ductile_geom::{Curve3, DVec2, DVec3, Surface};

/// A face's loops as chains of segments.
fn loops_of(b: &Body, f: FaceId) -> Vec<Vec<Segment>> {
    b.face_loops(f)
        .map(|l| {
            l.coedges
                .iter()
                .map(|c| {
                    let e = b.edge(c.edge);
                    Segment { curve: e.curve.clone(), t0: e.t0, t1: e.t1, reversed: c.reversed }
                })
                .collect()
        })
        .collect()
}

/// Parameters along `s` in the direction it runs, from start to end.
fn run(s: &Segment, n: usize) -> Vec<f64> {
    (0..=n)
        .map(|i| {
            let f = i as f64 / n as f64;
            if s.reversed { s.t1 + (s.t0 - s.t1) * f } else { s.t0 + (s.t1 - s.t0) * f }
        })
        .collect()
}

/// `s` split at curve parameter `t`, as the two pieces in running order.
fn split_segment(s: &Segment, t: f64) -> (Segment, Segment) {
    let lo = Segment { t1: t, ..s.clone() };
    let hi = Segment { t0: t, ..s.clone() };
    if s.reversed { (hi, lo) } else { (lo, hi) }
}

/// The surface parameters of the face's outer loop, sampled along each
/// segment and unrolled across periodic seams: per segment, its
/// parameters and their uvs.
fn outer_uvs(s: &Surface, outer: &[Segment]) -> Vec<Vec<(f64, DVec2)>> {
    let mut prev: Option<DVec2> = None;
    outer
        .iter()
        .map(|seg| {
            run(seg, 48)
                .into_iter()
                .map(|t| {
                    let p = seg.curve.point(t);
                    let mut uv = s.project(p, prev);
                    if let Some(q) = prev {
                        uv = s.align_periodic(uv, q);
                    }
                    prev = Some(uv);
                    (t, uv)
                })
                .collect()
        })
        .collect()
}

/// The range a face spans in its surface's parameters, along u (`along_u`)
/// or v: where an isoparam at fraction 0..1 of it lies.
pub fn face_param_range(b: &Body, f: FaceId, along_u: bool) -> Option<(f64, f64)> {
    let face = b.face(f);
    let loops = loops_of(b, f);
    let uvs = outer_uvs(&face.surface, loops.first()?);
    let k = |uv: &DVec2| if along_u { uv.x } else { uv.y };
    let (lo, hi) = uvs.iter().flatten().fold((f64::MAX, f64::MIN), |(lo, hi), (_, uv)| (lo.min(k(uv)), hi.max(k(uv))));
    (hi > lo).then_some((lo, hi))
}

/// Splits face `f` of `b` along the iso-parametric curve at `c` in u
/// (`along_u`: the curve runs along v at u = c) or in v. The curve must
/// cross the face's outer boundary exactly twice.
pub fn split_face_iso(b: &Body, f: FaceId, along_u: bool, c: f64) -> Result<Body, String> {
    split_face_iso_cut(b, f, along_u, c).map(|(body, _)| body)
}

/// As [`split_face_iso`], also returning the new edge's segment.
pub fn split_face_iso_cut(b: &Body, f: FaceId, along_u: bool, c: f64) -> Result<(Body, Segment), String> {
    let face = b.face(f).clone();
    let s = &face.surface;
    let loops = loops_of(b, f);
    let outer = loops.first().ok_or("the face has no boundary")?;
    let uvs = outer_uvs(s, outer);
    let k = |uv: DVec2| if along_u { uv.x } else { uv.y };
    let other = |uv: DVec2| if along_u { uv.y } else { uv.x };
    // Where the boundary crosses the isoparam: segment, curve parameter,
    // point, and the parameter along the isoparam there.
    let mut hits: Vec<(usize, f64, DVec3, f64)> = vec![];
    for (i, samples) in uvs.iter().enumerate() {
        for w in samples.windows(2) {
            let ((ta, ua), (tb, ub)) = (w[0], w[1]);
            let (ga, gb) = (k(ua) - c, k(ub) - c);
            if ga == 0.0 && hits.last().is_some_and(|h| h.0 == i && (h.1 - ta).abs() < 1e-12) {
                continue;
            }
            if !(ga <= 0.0 && gb > 0.0 || ga >= 0.0 && gb < 0.0) {
                continue;
            }
            if ga == 0.0 {
                hits.push((i, ta, outer[i].curve.point(ta), other(ua)));
                continue;
            }
            // Bisect for the crossing, staying in the unrolled chart.
            let (mut lo, mut hi, mut glo, mut near) = (ta, tb, ga, ua);
            for _ in 0..60 {
                let mid = 0.5 * (lo + hi);
                let uv = s.align_periodic(s.project(outer[i].curve.point(mid), Some(near)), near);
                let g = k(uv) - c;
                if (g < 0.0) == (glo < 0.0) && g != 0.0 {
                    (lo, glo, near) = (mid, g, uv);
                } else {
                    hi = mid;
                }
            }
            let t = 0.5 * (lo + hi);
            let uv = s.align_periodic(s.project(outer[i].curve.point(t), Some(near)), near);
            hits.push((i, t, outer[i].curve.point(t), other(uv)));
        }
    }
    let size = b.bbox().diagonal().max(1e-9);
    let tol = 1e-7 * size;
    // A crossing right at a segment's end shows up on both segments.
    let same = |a: &(usize, f64, DVec3, f64), z: &(usize, f64, DVec3, f64)| a.2.distance(z.2) < tol * 10.0 && (a.3 - z.3).abs() < 1e-6 * (1.0 + a.3.abs());
    hits.dedup_by(|a, z| same(a, z));
    if hits.len() > 2 && same(&hits[0], hits.last().unwrap()) {
        hits.pop();
    }
    if hits.len() != 2 {
        return Err(format!("the isoparam crosses the face's edge {} times; it must cross it twice", hits.len()));
    }
    // The isoparam between the two crossings, on the surface.
    let ((u0, u1), (v0, v1)) = {
        let all: Vec<DVec2> = uvs.iter().flatten().map(|x| x.1).collect();
        let lo = all.iter().fold(DVec2::splat(f64::MAX), |a, b| a.min(*b));
        let hi = all.iter().fold(DVec2::splat(f64::MIN), |a, b| a.max(*b));
        ((lo.x, hi.x), (lo.y, hi.y))
    };
    let net = match s {
        Surface::Nurbs(n) => n.clone(),
        _ => s.to_nurbs(u0, u1, v0, v1),
    };
    let iso = if along_u { net.iso_u(c) } else { net.iso_v(c) };
    let (a, z) = (hits[0].3.min(hits[1].3), hits[0].3.max(hits[1].3));
    let iso = Curve3::Nurbs(iso.subcurve(a, z));
    // From crossing `i` to crossing `j` along the isoparam.
    let cut = |i: usize, j: usize| {
        let seg = Segment { curve: iso.clone(), t0: a, t1: z, reversed: false };
        if hits[i].3 <= hits[j].3 { seg } else { seg.flipped() }
    };
    // Each half: along the boundary from one crossing to the other, then
    // back along the isoparam.
    let walk = |from: usize, to: usize| -> Vec<Segment> {
        let (fi, ft) = (hits[from].0, hits[from].1);
        let (ti, tt) = (hits[to].0, hits[to].1);
        let n = outer.len();
        let mut out = vec![];
        let mut i = fi;
        let mut first = true;
        loop {
            let mut seg = outer[i].clone();
            if first {
                seg = split_segment(&seg, ft).1;
            }
            let last = i == ti && !(first && run_before(&outer[i], tt, ft));
            if last {
                // The crossing lies on this segment, after the start.
                out.push(split_segment(&seg, tt).0);
                break;
            }
            out.push(seg);
            first = false;
            i = (i + 1) % n;
        }
        out.retain(|s| s.start().distance(s.end()) > tol || s.curve.point(0.5 * (s.t0 + s.t1)).distance(s.start()) > tol);
        out.push(cut(to, from));
        out
    };
    let halves = [walk(0, 1), walk(1, 0)];
    // Rebuild, splitting the neighbours' edges at the crossings.
    let points = [hits[0].2, hits[1].2];
    let split_at_points = |l: &[Segment]| -> Vec<Segment> {
        let mut out = vec![];
        for seg in l {
            let mut pieces = vec![seg.clone()];
            for p in points {
                let mut next = vec![];
                for piece in pieces {
                    let t = piece.curve.closest_param_in(p, piece.t0, piece.t1);
                    let inner = (t - piece.t0).abs() > 1e-9 * (1.0 + t.abs()) && (piece.t1 - t).abs() > 1e-9 * (1.0 + t.abs());
                    if inner && piece.curve.point(t).distance(p) < tol * 10.0 && piece.start().distance(p) > tol && piece.end().distance(p) > tol {
                        let (x, y) = split_segment(&piece, t);
                        next.push(x);
                        next.push(y);
                    } else {
                        next.push(piece);
                    }
                }
                pieces = next;
            }
            out.extend(pieces);
        }
        out
    };
    let mut sw = Sewer::new(b.kind, tol * 10.0);
    for g in b.face_ids() {
        if g == f {
            for h in &halves {
                // Holes go with the half on their side of the isoparam.
                let side = |p: DVec3| k(s.project(p, None)) > c;
                let mine = side(h[0].curve.point(0.5 * (h[0].t0 + h[0].t1)));
                let holes: Vec<Vec<Segment>> = loops[1..].iter().filter(|l| side(l[0].start()) == mine).cloned().collect();
                sw.face(face.surface.clone(), face.reversed, h, &holes);
            }
        } else {
            let ls = loops_of(b, g);
            let ls: Vec<Vec<Segment>> = ls.iter().map(|l| split_at_points(l)).collect();
            let gf = b.face(g);
            sw.face(gf.surface.clone(), gf.reversed, &ls[0], &ls[1..]);
        }
    }
    let mut out = sw.body;
    if std::env::var("DUCTILE_DEBUG_ISO").is_ok() {
        eprintln!("hits {:?}", hits.iter().map(|h| (h.0, h.1, h.2, h.3)).collect::<Vec<_>>());
        for h in &halves {
            eprintln!("half: {:?}", h.iter().map(|s| (s.start(), s.end())).collect::<Vec<_>>());
        }
        for e in out.edge_ids() {
            if out.edge_faces(e).len() != 2 {
                eprintln!("open edge {:?} {:?} users {}", out.edge(e).start(), out.edge(e).end(), out.edge_faces(e).len());
            }
        }
    }
    if b.kind == BodyKind::Solid && !out.edge_ids().all(|e| out.edge_faces(e).len() == 2) {
        return Err("splitting the face left the solid open".into());
    }
    out.kind = b.kind;
    Ok((out, cut(0, 1)))
}

/// Whether, running along `s`, parameter `a` comes before `b`.
fn run_before(s: &Segment, a: f64, b: f64) -> bool {
    if s.reversed { a > b } else { a < b }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::{make_box, make_cylinder};
    use ductile_geom::Frame;

    #[test]
    fn splits_a_box_side_and_a_cylinder_wall() {
        let b = make_box(&Frame::WORLD, DVec3::new(2.0, 1.0, 1.0));
        let top = b.face_ids().find(|f| b.face_normal(*f, 0.5, 0.5).z > 0.9).unwrap();
        let (lo, hi) = face_param_range(&b, top, true).unwrap();
        let out = split_face_iso(&b, top, true, 0.5 * (lo + hi)).unwrap();
        assert_eq!(out.faces.len(), 7);
        assert_eq!(out.kind, BodyKind::Solid);
        assert!(out.edge_ids().all(|e| out.edge_faces(e).len() == 2));
        let vol = crate::tessellate(&out, &crate::TessOptions::for_size(3.0)).volume();
        assert!((vol - 2.0).abs() < 1e-6, "{vol}");

        let c = make_cylinder(&Frame::WORLD, 1.0, 2.0);
        let wall = c.face_ids().find(|f| matches!(c.face(*f).surface, Surface::Cylinder { .. })).unwrap();
        let (lo, hi) = face_param_range(&c, wall, false).unwrap();
        let out = split_face_iso(&c, wall, false, 0.5 * (lo + hi)).unwrap();
        assert_eq!(out.faces.len(), c.faces.len() + 1);
        assert!(out.edge_ids().all(|e| out.edge_faces(e).len() == 2), "closed");
        let vol = crate::tessellate(&out, &crate::TessOptions::for_size(3.0)).volume();
        assert!((vol - std::f64::consts::PI * 2.0).abs() < 1e-2, "{vol}");
    }

    #[test]
    fn splits_a_nurbs_sheet_on_its_own_isoparam() {
        let grid: Vec<Vec<DVec3>> = (0..4).map(|i| (0..4).map(|j| DVec3::new(i as f64, j as f64, ((i * j) as f64 * 0.7).sin())).collect()).collect();
        let n = ductile_geom::NurbsSurface::interpolate_grid(&grid, 3, 3);
        let mut sw = Sewer::new(BodyKind::Sheet, 1e-7);
        sw.patch(n.clone());
        let sheet = sw.body;
        let f = FaceId(0);
        let (lo, hi) = face_param_range(&sheet, f, false).unwrap();
        let c = lo + 0.3 * (hi - lo);
        let out = split_face_iso(&sheet, f, false, c).unwrap();
        assert_eq!(out.faces.len(), 2);
        // The new edge lies on the isoparam, and is shared by both halves.
        let shared: Vec<_> = out.edge_ids().filter(|e| out.edge_faces(*e).len() == 2).collect();
        assert_eq!(shared.len(), 1);
        let e = out.edge(shared[0]);
        for t in [0.2, 0.5, 0.8] {
            let p = e.curve.point(e.t0 + (e.t1 - e.t0) * t);
            let uv = Surface::Nurbs(n.clone()).project(p, None);
            assert!((uv.y - c).abs() < 1e-6 && n.point(uv.x, uv.y).distance(p) < 1e-7, "{uv}");
        }
    }
}
