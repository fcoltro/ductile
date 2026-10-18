// SPDX-License-Identifier: GPL-3.0-only
//! Offset solids and shells.
//!
//! Every face moves along its outward normal by its own distance while the
//! topology stays as it is: faces get offset surfaces (exact for analytic
//! ones), vertices go to the point at the right distance from all the faces
//! around them, and edges follow the intersection of the two offset faces
//! (or, between tangent faces, the original edge moved along the shared
//! normal). Faces are extended or trimmed as needed, so sharp edges stay
//! sharp. A shell is the solid minus its inward offset; faces to open are
//! pushed outward instead, so the cut runs through them.

use super::boolean::{boolean, BoolError, BoolOp};
use crate::primitives::flip_orientation;
use crate::topo::{Body, EdgeId, FaceId, VertexId};
use ductile_geom::{Curve3, DMat3, DVec2, DVec3, Frame, NurbsCurve, NurbsSurface, Surface};
use std::collections::HashMap;

#[derive(Debug, thiserror::Error, PartialEq)]
pub enum OffsetError {
    #[error("the distance is too large for this shape")]
    TooLarge,
    #[error("this shape cannot be offset yet")]
    Unsupported,
    #[error("faces moving by different amounts meet tangentially")]
    TangentStep,
    #[error("boolean failed: {0}")]
    Boolean(#[from] BoolError),
}

/// Offsets every face of `body` outward by `dist(face)` (negative moves it
/// inward).
pub fn offset_body(body: &Body, dist: &dyn Fn(FaceId) -> f64) -> Result<Body, OffsetError> {
    let size = body.bbox().diagonal().max(1e-9);
    let d: Vec<f64> = body.face_ids().map(dist).collect();
    // Tangent neighbours have to move together, or they would come apart.
    for e in body.edge_ids() {
        let uses = body.edge_faces(e);
        if uses.len() == 2 && (d[uses[0].0.idx()] - d[uses[1].0.idx()]).abs() > 1e-12 {
            let ed = body.edge(e);
            let q = ed.curve.point(0.5 * (ed.t0 + ed.t1));
            let (_, na, _) = normal_at(body, uses[0].0, q, None);
            let (_, nb, _) = normal_at(body, uses[1].0, q, None);
            if na.dot(nb) > 1.0 - 1e-8 {
                return Err(OffsetError::TangentStep);
            }
        }
    }
    let mut out = body.clone();
    // Surfaces.
    for f in body.face_ids() {
        let face = body.face(f);
        // Distance along the surface's own normal.
        let ds = if face.reversed { -d[f.idx()] } else { d[f.idx()] };
        out.faces[f.idx()].surface = offset_surface(&face.surface, ds).ok_or(OffsetError::TooLarge)?;
    }
    // Faces around each vertex.
    let mut vfaces: HashMap<VertexId, Vec<FaceId>> = HashMap::new();
    for e in body.edge_ids() {
        let ed = body.edge(e);
        for (f, _) in body.edge_faces(e) {
            for v in [ed.v0, ed.v1] {
                let list = vfaces.entry(v).or_default();
                if !list.contains(&f) {
                    list.push(f);
                }
            }
        }
    }
    // Vertices.
    for (i, vx) in body.vertices.iter().enumerate() {
        let v = VertexId(i as u32);
        let Some(faces) = vfaces.get(&v) else { continue };
        let p = vertex_point(body, faces, &d, vx.point, size).ok_or(OffsetError::TooLarge)?;
        out.vertices[i].point = p;
    }
    // Edges.
    for e in body.edge_ids() {
        let uses = body.edge_faces(e);
        if uses.is_empty() {
            continue;
        }
        let (fa, fb) = (uses[0].0, uses.get(1).map(|u| u.0).unwrap_or(uses[0].0));
        let ed = body.edge(e).clone();
        let (pa, pb) = (out.vertex(ed.v0), out.vertex(ed.v1));
        let (curve, t0, t1) = offset_edge(body, e, (fa, d[fa.idx()]), (fb, d[fb.idx()]), pa, pb, size).ok_or(OffsetError::TooLarge)?;
        let edge = &mut out.edges[e.idx()];
        edge.curve = curve;
        edge.t0 = t0;
        edge.t1 = t1;
    }
    Ok(out)
}

/// Hollows `body` into walls of `thickness`, opening the faces in `open`.
/// With nothing open the result is a solid with an inner void.
pub fn shell(body: &Body, open: &[FaceId], thickness: f64) -> Result<Body, OffsetError> {
    if thickness <= 0.0 {
        return Err(OffsetError::TooLarge);
    }
    let adjacent = body.edge_ids().any(|e| {
        let fs = body.edge_faces(e);
        fs.len() == 2 && fs[0].0 != fs[1].0 && open.contains(&fs[0].0) && open.contains(&fs[1].0)
    });
    if adjacent {
        // Push the open faces out so the inner solid pokes through them.
        let inner = offset_body(body, &|f| if open.contains(&f) { thickness } else { -thickness })?;
        return Ok(boolean(body, &inner, BoolOp::Difference)?);
    }
    // Open faces stay where they are on the inner solid, so its rim lies in
    // them: each open face becomes a ring around the inner solid's copy.
    let inner = flip_orientation(offset_body(body, &|f| if open.contains(&f) { 0.0 } else { -thickness })?);
    let mut out = body.clone();
    if open.is_empty() {
        out.append(&inner);
        return Ok(out);
    }
    let lo = out.loops.len() as u32;
    let holes: Vec<(FaceId, Vec<crate::topo::LoopId>)> = open.iter().map(|f| (*f, inner.faces[f.idx()].loops.clone())).collect();
    let mut walls = inner;
    walls.faces = walls.faces.iter().enumerate().filter(|(i, _)| !open.iter().any(|f| f.idx() == *i)).map(|(_, f)| f.clone()).collect();
    out.append(&walls);
    for (f, ls) in holes {
        out.faces[f.idx()].loops.extend(ls.iter().map(|l| crate::topo::LoopId(l.0 + lo)));
    }
    Ok(super::heal::compact(&out))
}

/// The surface moved by `d` along its normal; `None` when it collapses.
pub fn offset_surface(s: &Surface, d: f64) -> Option<Surface> {
    Some(match s {
        Surface::Plane { frame } => Surface::Plane { frame: Frame { origin: frame.origin + frame.z * d, ..*frame } },
        Surface::Cylinder { frame, radius } => {
            let r = radius + d;
            (r > 1e-9).then_some(())?;
            Surface::Cylinder { frame: *frame, radius: r }
        }
        Surface::Cone { frame, radius, half_angle } => {
            // Same apex angle; the radius at height 0 grows by d / cos.
            let r = radius + d / half_angle.cos();
            Surface::Cone { frame: *frame, radius: r, half_angle: *half_angle }
        }
        Surface::Sphere { frame, radius } => {
            let r = radius + d;
            (r > 1e-9).then_some(())?;
            Surface::Sphere { frame: *frame, radius: r }
        }
        Surface::Torus { frame, major, minor } => {
            let m = minor + d;
            (m > 1e-9).then_some(())?;
            Surface::Torus { frame: *frame, major: *major, minor: m }
        }
        Surface::Nurbs(n) => {
            // Sampled offset, interpolated over a grid finer than the net.
            let ((u0, u1), (v0, v1)) = n.domain();
            let cu = (3 * n.nu).clamp(12, 48);
            let cv = (3 * n.nv).clamp(12, 48);
            let src = Surface::Nurbs(n.clone());
            let grid: Vec<Vec<DVec3>> = (0..cu)
                .map(|i| {
                    let u = u0 + (u1 - u0) * i as f64 / (cu - 1) as f64;
                    (0..cv)
                        .map(|j| {
                            let v = v0 + (v1 - v0) * j as f64 / (cv - 1) as f64;
                            src.point(u, v) + src.normal(u, v) * d
                        })
                        .collect()
                })
                .collect();
            let us: Vec<f64> = (0..cu).map(|i| i as f64 / (cu - 1) as f64).collect();
            let vs: Vec<f64> = (0..cv).map(|j| j as f64 / (cv - 1) as f64).collect();
            Surface::Nurbs(NurbsSurface::interpolate_grid_with_params(&grid, 3, 3, &us, &vs))
        }
    })
}

/// Outward normal of face `f` near `p`, with the surface parameter.
fn normal_at(body: &Body, f: FaceId, p: DVec3, hint: Option<DVec2>) -> (DVec3, DVec3, DVec2) {
    let s = &body.face(f).surface;
    let uv = s.project(p, hint);
    (s.point(uv.x, uv.y), body.face_normal(f, uv.x, uv.y), uv)
}

/// Point at distance `d_i` outside every face `f_i` (on its extension when
/// needed), optionally kept on the plane through `plane.0` with normal
/// `plane.1`. Starts from `start`.
fn solve_point(body: &Body, cons: &[(FaceId, f64)], plane: Option<(DVec3, DVec3)>, start: DVec3, size: f64) -> Option<DVec3> {
    let mut c = start;
    let mut hints: Vec<Option<DVec2>> = vec![None; cons.len()];
    for _ in 0..60 {
        let mut rows: Vec<(DVec3, f64)> = Vec::with_capacity(4);
        for (i, (f, d)) in cons.iter().enumerate() {
            let (p, n, uv) = normal_at(body, *f, c, hints[i]);
            hints[i] = Some(uv);
            rows.push((n, (c - p).dot(n) - d));
        }
        if let Some((o, n)) = plane {
            rows.push((n, (c - o).dot(n)));
        }
        let err = rows.iter().map(|r| r.1 * r.1).sum::<f64>().sqrt();
        if err < 1e-12 * size {
            return Some(c);
        }
        // Least squares (normal equations) for the step.
        let mut a = DMat3::ZERO;
        let mut b = DVec3::ZERO;
        for (n, r) in &rows {
            a += DMat3::from_cols(*n * n.x, *n * n.y, *n * n.z);
            b += *n * *r;
        }
        if a.determinant().abs() < 1e-18 {
            return None;
        }
        c -= a.inverse() * b;
        if c.distance(start) > size {
            return None;
        }
    }
    None
}

/// Groups faces by normal direction at `p`: faces meeting tangentially
/// there share a group.
fn normal_groups(body: &Body, faces: &[FaceId], p: DVec3) -> Vec<(DVec3, Vec<FaceId>)> {
    let mut groups: Vec<(DVec3, Vec<FaceId>)> = Vec::new();
    for &f in faces {
        let (_, n, _) = normal_at(body, f, p, None);
        match groups.iter_mut().find(|g| g.0.dot(n) > 1.0 - 1e-8) {
            Some(g) => g.1.push(f),
            None => groups.push((n, vec![f])),
        }
    }
    groups
}

fn vertex_point(body: &Body, faces: &[FaceId], d: &[f64], p: DVec3, size: f64) -> Option<DVec3> {
    let groups = normal_groups(body, faces, p);
    // One distance per group; tangent faces must move together.
    let mut cons = Vec::new();
    for (n, fs) in &groups {
        let dist = d[fs[0].idx()];
        if fs.iter().any(|f| (d[f.idx()] - dist).abs() > 1e-12) {
            return None;
        }
        cons.push((fs[0], dist, *n));
    }
    match cons.len() {
        0 => None,
        1 => Some(p + cons[0].2 * cons[0].1),
        2 => {
            // Along the sharp edge's normal plane through the vertex.
            let axis = cons[0].2.cross(cons[1].2).normalize();
            let start = p + (cons[0].2 + cons[1].2).normalize() * cons[0].1;
            solve_point(body, &[(cons[0].0, cons[0].1), (cons[1].0, cons[1].1)], Some((p, axis)), start, size)
        }
        _ => {
            let c: Vec<(FaceId, f64)> = cons.iter().map(|c| (c.0, c.1)).collect();
            let avg = cons.iter().map(|c| c.2).sum::<DVec3>().normalize_or_zero();
            solve_point(body, &c, None, p + avg * cons[0].1, size)
        }
    }
}

/// The offset of edge `e` between faces `a` and `b`, ending at `pa` and
/// `pb`: the curve and its parameter range.
fn offset_edge(body: &Body, e: EdgeId, (fa, da): (FaceId, f64), (fb, db): (FaceId, f64), pa: DVec3, pb: DVec3, size: f64) -> Option<(Curve3, f64, f64)> {
    let ed = body.edge(e);
    let closed = ed.is_closed();
    let span = ed.t1 - ed.t0;
    // Sample past the ends so the moved vertices are covered.
    let speed = ed.length() / span.max(1e-300);
    let ext = if closed || matches!(ed.curve, Curve3::Nurbs(_)) { 0.0 } else { (3.0 * da.abs().max(db.abs()) / speed.max(1e-300)).min(0.45 * span) };
    let n = 48;
    let (s0, s1) = (ed.t0 - ext, ed.t1 + ext);
    let mut pts = Vec::with_capacity(n + 1);
    let mut params = Vec::with_capacity(n + 1);
    let mut hint = None;
    for i in 0..=n {
        let t = s0 + (s1 - s0) * i as f64 / n as f64;
        let q = ed.curve.point(t);
        let (_, na, uva) = normal_at(body, fa, q, hint);
        hint = Some(uva);
        let (_, nb, _) = normal_at(body, fb, q, None);
        let p = if na.dot(nb) > 1.0 - 1e-8 {
            if (da - db).abs() > 1e-12 {
                return None;
            }
            q + na * da
        } else {
            let tan = ed.curve.tangent(t).normalize();
            let start = q + (na + nb).normalize() * da.max(db);
            solve_point(body, &[(fa, da), (fb, db)], Some((q, tan)), start, size)?
        };
        pts.push(p);
        params.push(t);
    }
    let tol = 1e-7 * size;
    // Exact forms where the original is a line or a circle and the offset
    // stays one.
    match &ed.curve {
        Curve3::Line { dir, .. } => {
            let o = pts[0];
            if pts.iter().all(|p| (*p - o).cross(*dir).length() < tol) {
                let c = Curve3::Line { origin: o, dir: *dir };
                let (t0, t1) = ((pa - o).dot(*dir), (pb - o).dot(*dir));
                return (t1 > t0).then_some((c, t0, t1));
            }
        }
        Curve3::Circle { frame, .. } => {
            let h = (pts[0] - frame.origin).dot(frame.z);
            let c0 = frame.origin + frame.z * h;
            let r = (pts[0] - c0).length();
            let fits = pts.iter().zip(&params).all(|(p, t)| {
                let l = frame.to_local(*p);
                (l.z - h).abs() < tol && ((*p - c0).length() - r).abs() < tol && wrap(l.y.atan2(l.x) - t).abs() < 1e-7
            });
            if fits && r > 1e-9 {
                let c = Curve3::Circle { frame: Frame { origin: c0, ..*frame }, radius: r };
                if closed {
                    return Some((c, ed.t0, ed.t1));
                }
                let t0 = c.closest_param_in(pa, ed.t0 - ext, ed.t1 + ext);
                let t1 = c.closest_param_in(pb, ed.t0 - ext, ed.t1 + ext);
                return (t1 > t0).then_some((c, t0, t1));
            }
        }
        _ => {}
    }
    let curve = if closed {
        pts.pop();
        NurbsCurve::interpolate_closed(&pts)
    } else {
        NurbsCurve::interpolate(&pts, 3)
    };
    let (c0, c1) = curve.domain();
    let c = Curve3::Nurbs(curve);
    if closed {
        return Some((c, c0, c1));
    }
    let t0 = c.closest_param_in(pa, c0, c1);
    let t1 = c.closest_param_in(pb, c0, c1);
    if c.point(t0).distance(pa) > 1e-5 * size || c.point(t1).distance(pb) > 1e-5 * size || t1 <= t0 {
        return None;
    }
    Some((c, t0, t1))
}

fn wrap(a: f64) -> f64 {
    ductile_geom::wrap_angle(a, -std::f64::consts::PI)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ops::boolean::open_edges;
    use crate::primitives::*;
    use crate::tess::{tessellate, TessOptions};
    use std::f64::consts::PI;

    fn vol(b: &Body) -> f64 {
        tessellate(b, &TessOptions { chord_tol: 1e-4, angle_tol: 0.05 }).volume()
    }

    #[test]
    fn box_offsets_both_ways() {
        let b = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
        let inner = offset_body(&b, &|_| -0.5).unwrap();
        assert!((vol(&inner) - 3.0 * 2.0 * 1.0).abs() < 1e-9, "{}", vol(&inner));
        // Outward with sharp corners (faces extended, not rounded).
        let outer = offset_body(&b, &|_| 0.5).unwrap();
        assert!((vol(&outer) - 5.0 * 4.0 * 3.0).abs() < 1e-9, "{}", vol(&outer));
    }

    #[test]
    fn shells_an_open_box() {
        let b = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
        let top = b.face_ids().find(|f| b.face_edges(*f).iter().all(|e| b.edge(*e).start().z > 1.9 && b.edge(*e).end().z > 1.9)).unwrap();
        let s = shell(&b, &[top], 0.25).unwrap();
        assert_eq!(open_edges(&s), 0);
        let expect = 24.0 - 3.5 * 2.5 * 1.75;
        assert!((vol(&s) - expect).abs() < 1e-6, "{} vs {expect}", vol(&s));
        // Closed: a void inside.
        let h = shell(&b, &[], 0.25).unwrap();
        assert!((vol(&h) - (24.0 - 3.5 * 2.5 * 1.5)).abs() < 1e-6, "{}", vol(&h));
    }

    #[test]
    fn shells_a_cylinder_cup() {
        let c = make_cylinder(&Frame::WORLD, 2.0, 3.0);
        let top = c.face_ids().find(|f| matches!(&c.face(*f).surface, Surface::Plane { frame } if frame.origin.z > 2.9)).unwrap();
        let s = shell(&c, &[top], 0.2).unwrap();
        assert_eq!(open_edges(&s), 0);
        let expect = vol(&c) - PI * 1.8 * 1.8 * 2.8;
        assert!((vol(&s) - expect).abs() < 2e-3 * expect, "{} vs {expect}", vol(&s));
    }

    #[test]
    fn shells_a_filleted_box() {
        // Tangent faces move together, and the edges between them follow.
        let b = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
        let all: Vec<EdgeId> = b.edge_ids().collect();
        let r = 0.6;
        let f = crate::ops::fillet::blend_edges(&b, &all, crate::ops::fillet::Blend::Fillet(r)).unwrap();
        let t = 0.2;
        let h = shell(&f, &[], t).unwrap();
        assert_eq!(open_edges(&h), 0);
        // The inner void is the rounded box of the inner box with radius r - t.
        let rounded = |a: f64, b: f64, c: f64, r: f64| {
            let (x, y, z) = (a - 2.0 * r, b - 2.0 * r, c - 2.0 * r);
            x * y * z + 2.0 * r * (x * y + y * z + z * x) + PI * r * r * (x + y + z) + 4.0 / 3.0 * PI * r * r * r
        };
        let expect = rounded(4.0, 3.0, 2.0, r) - rounded(4.0 - 2.0 * t, 3.0 - 2.0 * t, 2.0 - 2.0 * t, r - t);
        assert!((vol(&h) - expect).abs() < 3e-3 * expect, "{} vs {expect}", vol(&h));
    }

    #[test]
    fn shells_a_soap_bar_through_its_top() {
        use crate::ops::fillet::{blend_edges, Blend};
        use crate::ops::heal::compact;
        let b = make_box(&Frame::WORLD, DVec3::new(6.0, 4.0, 2.0));
        let vertical: Vec<EdgeId> = b.edge_ids().filter(|e| (b.edge(*e).start() - b.edge(*e).end()).z.abs() > 1.0).collect();
        let b = compact(&blend_edges(&b, &vertical, Blend::Fillet(1.2)).unwrap());
        let bottom: Vec<EdgeId> = b.edge_ids().filter(|e| b.edge(*e).start().z.abs() < 1e-9 && b.edge(*e).end().z.abs() < 1e-9).collect();
        let b = compact(&blend_edges(&b, &bottom, Blend::Fillet(0.5)).unwrap());
        let top = b.face_ids().find(|f| b.face_edges(*f).iter().all(|e| b.edge(*e).start().z > 1.99 && b.edge(*e).end().z > 1.99)).unwrap();
        let t = 0.15;
        let s = shell(&b, &[top], t).unwrap();
        assert_eq!(open_edges(&s), 0);
        let v = vol(&s);
        // Roughly the wall area times the thickness.
        let area = tessellate(&b, &TessOptions { chord_tol: 1e-4, angle_tol: 0.05 }).area();
        let top_area = 6.0 * 4.0 - (4.0 - PI) * 1.2 * 1.2;
        let expect = (area - top_area) * t;
        assert!((v - expect).abs() < 0.1 * expect, "{v} vs {expect}");
    }
}
#[cfg(test)]
mod vase_tests {
    use super::*;
    use crate::ops::boolean::open_edges;
    use crate::ops::fillet::{blend_edges, Blend};
    use crate::profile::{Loop3, Segment};
    use crate::tess::{tessellate, TessOptions};

    #[test]
    fn shells_a_vase_then_rounds_its_rims() {
        let pts: Vec<DVec3> = [(1.6, 0.0), (2.4, 1.5), (1.2, 4.0), (1.0, 5.2), (1.5, 6.5)].iter().map(|(r, z)| DVec3::new(*r, 0.0, *z)).collect();
        let sp = NurbsCurve::interpolate(&pts, 3);
        let (t0, t1) = sp.domain();
        let top = DVec3::Z * 6.5;
        let l = Loop3 { segments: vec![Segment::line(DVec3::ZERO, pts[0]), Segment::new(Curve3::Nurbs(sp), t0, t1), Segment::line(pts[4], top), Segment::line(top, DVec3::ZERO)] };
        let b = crate::ops::revolve::revolve_loop(&l, DVec3::ZERO, DVec3::Z).unwrap();
        let mouth = b.face_ids().find(|f| matches!(b.face(*f).surface, Surface::Plane { .. }) && b.face_edges(*f).iter().all(|e| b.edge(*e).start().z > 6.0)).unwrap();
        let t = 0.2;
        let s = shell(&b, &[mouth], t).unwrap();
        assert_eq!(open_edges(&s), 0);
        let opts = TessOptions { chord_tol: 1e-4, angle_tol: 0.05 };
        let (full, outer) = (tessellate(&b, &opts), tessellate(&s, &opts));
        // Walls: about the area of the wall and base, times the thickness.
        let mouth_area = std::f64::consts::PI * 1.5 * 1.5;
        let expect = (full.area() - mouth_area) * t;
        assert!((outer.volume() - expect).abs() < 0.08 * expect, "{} vs {expect}", outer.volume());
        // The two lip edges round off with rolling-ball fillets.
        let rims: Vec<EdgeId> = s.edge_ids().filter(|e| s.edge(*e).is_closed() && s.edge(*e).start().z > 6.4).collect();
        assert_eq!(rims.len(), 2);
        let f = blend_edges(&s, &rims, Blend::Fillet(0.08)).unwrap();
        assert_eq!(open_edges(&f), 0);
        assert!(tessellate(&f, &opts).volume() < outer.volume());
    }
}
