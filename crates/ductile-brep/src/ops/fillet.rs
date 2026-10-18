// SPDX-License-Identifier: GPL-3.0-only
//! Constant-radius fillets and equal-distance chamfers on edges.
//!
//! Each edge gets a "tool" solid bounded by the faces next to the edge and
//! by the exact blend surface: the material between them is removed from
//! convex edges and added to concave ones by a boolean operation, which
//! takes care of trimming the neighbouring faces. Plane/plane edges are
//! blended by cylinders (or planes for chamfers), circular plane/cylinder
//! edges by tori (or cones). Where three filleted convex edges meet at a
//! vertex, the corner is finished with the rolling ball's sphere.

use super::boolean::{boolean, BoolError, BoolOp};
use super::extrude::extrude_region;
use super::revolve::revolve_loop;
use crate::primitives::{flip_orientation, make_box, make_sphere_in, polyhedron};
use crate::profile::{Loop3, Region, Segment};
use crate::topo::{Body, BodyKind, EdgeId, FaceId, VertexId};
use ductile_geom::{Curve3, DMat3, DVec3, Frame, Surface};
use std::collections::{BTreeSet, HashMap};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Blend {
    /// Rolling-ball fillet of the given radius.
    Fillet(f64),
    /// Chamfer cutting the given distance back along both faces.
    Chamfer(f64),
}

#[derive(Debug, thiserror::Error, PartialEq)]
pub enum FilletError {
    #[error("an edge is not shared by exactly two faces")]
    NotManifold,
    #[error("this kind of edge cannot be blended yet")]
    Unsupported,
    #[error("the edge is smooth (tangent faces)")]
    Smooth,
    #[error("the radius is too large for the geometry")]
    TooLarge,
    #[error("boolean failed: {0}")]
    Boolean(#[from] BoolError),
}

struct EdgeInfo {
    f1: FaceId,
    f2: FaceId,
    /// Edge start and end, oriented along face 1's loop.
    p0: DVec3,
    p1: DVec3,
    t: DVec3,
    n1: DVec3,
    n2: DVec3,
    convex: bool,
}

fn outward(b: &Body, f: FaceId, p: DVec3) -> DVec3 {
    let uv = b.face(f).surface.project(p, None);
    b.face_normal(f, uv.x, uv.y)
}

fn edge_info(b: &Body, e: EdgeId) -> Result<EdgeInfo, FilletError> {
    let uses = b.edge_faces(e);
    if uses.len() != 2 || uses[0].0 == uses[1].0 {
        return Err(FilletError::NotManifold);
    }
    let (f1, c1) = uses[0];
    let f2 = uses[1].0;
    let ed = b.edge(e);
    let (ta, tb, s) = if c1.reversed { (ed.t1, ed.t0, -1.0) } else { (ed.t0, ed.t1, 1.0) };
    let tm = 0.5 * (ed.t0 + ed.t1);
    let pm = ed.curve.point(tm);
    let t = (ed.curve.tangent(tm) * s).normalize();
    let n1 = outward(b, f1, pm);
    let n2 = outward(b, f2, pm);
    let cross = n1.cross(n2);
    if cross.length() < 1e-6 {
        return Err(FilletError::Smooth);
    }
    Ok(EdgeInfo { f1, f2, p0: ed.curve.point(ta), p1: ed.curve.point(tb), t, n1, n2, convex: cross.dot(t) > 0.0 })
}

/// Arc from `a` to `b` around `c` in the plane with normal `n`, taking the
/// minor arc.
fn arc(c: DVec3, a: DVec3, b: DVec3, n: DVec3) -> Segment {
    let r = a.distance(c);
    let x = (a - c) / r;
    let mut z = n.normalize();
    let mut y = z.cross(x);
    let mut phi = (b - c).dot(y).atan2((b - c).dot(x));
    if phi < 0.0 {
        z = -z;
        y = -y;
        phi = (b - c).dot(y).atan2((b - c).dot(x));
    }
    Segment::new(Curve3::Circle { frame: Frame { origin: c, x, y, z }, radius: r }, 0.0, phi)
}

/// Half-space solid on the inner side of the plane through `o` with
/// outward normal `n`.
fn half_space(o: DVec3, n: DVec3, size: f64) -> Body {
    let f = Frame::new(o, -n, ductile_geom::any_perpendicular(n));
    let corner = o - f.x * size - f.y * size;
    make_box(&Frame { origin: corner, ..f }, DVec3::new(2.0 * size, 2.0 * size, size))
}

/// Faces meeting at a vertex.
fn vertex_faces(b: &Body, v: VertexId) -> BTreeSet<FaceId> {
    let mut out = BTreeSet::new();
    for f in b.face_ids() {
        for e in b.face_edges(f) {
            let ed = b.edge(e);
            if ed.v0 == v || ed.v1 == v {
                out.insert(f);
            }
        }
    }
    out
}

/// Blends the given edges. Edges are blended with tools computed on the
/// original body, then applied in turn.
pub fn blend_edges(body: &Body, edges: &[EdgeId], blend: Blend) -> Result<Body, FilletError> {
    let size = body.bbox().diagonal().max(1e-6) * 4.0;
    let set: BTreeSet<EdgeId> = edges.iter().copied().collect();
    let mut infos: HashMap<EdgeId, EdgeInfo> = HashMap::new();
    let mut tools: Vec<(EdgeId, Body)> = Vec::new();
    // Chains of tangent edges, and edges with no exact tool, are blended
    // by rolling the ball along them directly; that keeps every other id
    // valid until the end.
    let mut base = body.clone();
    let mut rolled = false;
    let mut tooled: BTreeSet<EdgeId> = BTreeSet::new();
    for (chain, closed) in super::chain::tangent_chains(body, &set.iter().copied().collect::<Vec<_>>()) {
        if chain.len() > 1 {
            base = super::chain::roll_chain(&base, &chain, closed, blend)?;
            rolled = true;
            continue;
        }
        let e = chain[0].edge;
        let info = edge_info(body, e)?;
        match edge_tool(body, e, &info, blend, size) {
            Ok(t) => {
                tools.push((e, t));
                tooled.insert(e);
                infos.insert(e, info);
            }
            Err(FilletError::Unsupported) => {
                base = if closed {
                    super::roll::roll_closed_edge(&base, e, blend)?
                } else {
                    super::chain::roll_chain(&base, &chain, false, blend)?
                };
                rolled = true;
            }
            Err(err) => return Err(err),
        }
    }
    if rolled {
        base = super::heal::compact(&base);
    }
    let set = tooled;
    let corners = match blend {
        Blend::Fillet(r) => corner_tools(body, &set, &infos, r),
        Blend::Chamfer(_) => Vec::new(),
    };
    // Edge tools stop where the corner patches begin, so that tools of
    // neighbouring edges never overlap.
    for (_, caps) in &corners {
        for (e, o, n) in caps {
            if let Some((_, t)) = tools.iter_mut().find(|(x, _)| x == e) {
                *t = boolean(t, &half_space(*o, *n, size), BoolOp::Intersection)?;
            }
        }
    }
    let mut cut: Vec<Body> = Vec::new();
    let mut add: Vec<Body> = Vec::new();
    for (e, t) in tools {
        if infos[&e].convex {
            cut.push(t);
        } else {
            add.push(t);
        }
    }
    cut.extend(corners.into_iter().map(|(t, _)| t));
    let mut out = base;
    for t in &cut {
        out = boolean(&out, t, BoolOp::Difference)?;
    }
    for t in &add {
        out = boolean(&out, t, BoolOp::Union)?;
    }
    Ok(out)
}

fn edge_tool(b: &Body, e: EdgeId, info: &EdgeInfo, blend: Blend, size: f64) -> Result<Body, FilletError> {
    let ed = b.edge(e);
    let (s1, s2) = (&b.face(info.f1).surface, &b.face(info.f2).surface);
    match (&ed.curve, s1, s2) {
        (Curve3::Line { .. }, Surface::Plane { .. }, Surface::Plane { .. }) => line_tool(b, e, info, blend, size),
        (Curve3::Circle { frame, .. }, _, _) => {
            let (pf, cf) = match (s1, s2) {
                (Surface::Plane { .. }, Surface::Cylinder { .. }) => (info.f1, info.f2),
                (Surface::Cylinder { .. }, Surface::Plane { .. }) => (info.f2, info.f1),
                _ => return Err(FilletError::Unsupported),
            };
            circle_tool(b, frame.origin, pf, cf, info, blend)
        }
        _ => Err(FilletError::Unsupported),
    }
}

fn line_tool(b: &Body, e: EdgeId, info: &EdgeInfo, blend: Blend, size: f64) -> Result<Body, FilletError> {
    let EdgeInfo { p0, p1, t, n1, n2, convex, .. } = *info;
    let len = p0.distance(p1);
    let u1 = n1.cross(t);
    let u2 = t.cross(n2);
    let (profile_pts, radius) = match blend {
        Blend::Fillet(r) => {
            let sgn = if convex { -1.0 } else { 1.0 };
            let m = DMat3::from_cols(n1, n2, t).transpose();
            let o = m.inverse() * DVec3::new(sgn * r, sgn * r, 0.0);
            let q1 = p0 + o - n1 * o.dot(n1);
            let q2 = p0 + o - n2 * o.dot(n2);
            ((q1, q2, Some(p0 + o)), r)
        }
        Blend::Chamfer(d) => ((p0 + u1 * d, p0 + u2 * d, None), d),
    };
    let (q1, q2, center) = profile_pts;
    // Reject blends wider than the faces next to the edge (rough check).
    let reach = (q1 - p0).length().max((q2 - p0).length());
    if reach > size {
        return Err(FilletError::TooLarge);
    }
    let margin = radius * 4.0 + len * 0.05;
    let sh = -t * margin;
    let (pp, a, c) = (p0 + sh, q1 + sh, q2 + sh);
    let mid = match center {
        Some(cc) => arc(cc + sh, a, c, t),
        None => Segment::line(a, c),
    };
    let lp = Loop3 { segments: vec![Segment::line(pp, a), mid, Segment::line(c, pp)] };
    let region = Region { plane: Frame::new(pp, t, a - pp), outer: lp, holes: vec![] };
    let mut tool = extrude_region(&region, t * (len + 2.0 * margin)).ok_or(FilletError::Unsupported)?;
    // Stop the tool at the faces capping the edge's ends.
    let ed = b.edge(e);
    for (v, p) in [(ed.v0, b.vertex(ed.v0)), (ed.v1, b.vertex(ed.v1))] {
        let others: Vec<FaceId> = vertex_faces(b, v).into_iter().filter(|f| *f != info.f1 && *f != info.f2).collect();
        if others.len() == 1 {
            let n = outward(b, others[0], p);
            // Only cap when the end face actually faces away along the edge.
            let along = if p.distance(p1) < p.distance(p0) { t } else { -t };
            if n.dot(along) > 1e-6 {
                tool = boolean(&tool, &half_space(p, n, size), BoolOp::Intersection)?;
            }
        }
    }
    Ok(tool)
}

fn circle_tool(b: &Body, center: DVec3, pf: FaceId, cf: FaceId, info: &EdgeInfo, blend: Blend) -> Result<Body, FilletError> {
    let Surface::Cylinder { frame: cyl, radius: big_r } = &b.face(cf).surface else { return Err(FilletError::Unsupported) };
    let pm = 0.5 * (info.p0 + info.p1);
    let pm = if pm.distance(center) < 1e-9 { info.p0 } else { pm };
    let a = outward(b, pf, pm);
    if cyl.z.cross(a).length() > 1e-9 {
        return Err(FilletError::Unsupported);
    }
    // Radial direction through a point of the edge (any point works).
    let p = info.p0;
    let e = (p - center - a * (p - center).dot(a)).normalize();
    let nc = outward(b, cf, p);
    let sigma = if nc.dot(e) > 0.0 { 1.0 } else { -1.0 };
    let zs = if info.convex { -1.0 } else { 1.0 };
    let big_r = *big_r;
    let at = |rho: f64, z: f64| center + e * rho + a * z;
    let d = match blend {
        Blend::Fillet(r) | Blend::Chamfer(r) => r,
    };
    let rho_p = big_r + zs * sigma * d;
    if rho_p <= 1e-9 {
        return Err(FilletError::TooLarge);
    }
    let corner = at(big_r, 0.0);
    let qp = at(rho_p, 0.0);
    let qc = at(big_r, zs * d);
    let mid = match blend {
        Blend::Fillet(_) => arc(at(rho_p, zs * d), qp, qc, e.cross(a)),
        Blend::Chamfer(_) => Segment::line(qp, qc),
    };
    let lp = Loop3 { segments: vec![Segment::line(corner, qp), mid, Segment::line(qc, corner)] };
    revolve_loop(&lp, center, a).ok_or(FilletError::Unsupported)
}

/// Spherical corner patches where three filleted convex plane/plane edges
/// meet at a vertex of three planar faces.
/// Each comes with the planes (edge, point, outward normal) that cut the
/// incident edge tools back to the patch.
fn corner_tools(b: &Body, set: &BTreeSet<EdgeId>, infos: &HashMap<EdgeId, EdgeInfo>, r: f64) -> Vec<(Body, Vec<(EdgeId, DVec3, DVec3)>)> {
    let mut out = Vec::new();
    for vi in 0..b.vertices.len() {
        let v = VertexId(vi as u32);
        let inc: Vec<EdgeId> = b.edge_ids().filter(|e| (b.edge(*e).v0 == v || b.edge(*e).v1 == v) && b.edge_faces(*e).len() == 2).collect();
        if inc.len() != 3 || !inc.iter().all(|e| set.contains(e) && infos[e].convex && b.edge(*e).curve.is_linear()) {
            continue;
        }
        let faces: Vec<FaceId> = vertex_faces(b, v).into_iter().collect();
        if faces.len() != 3 || !faces.iter().all(|f| b.face(*f).surface.is_planar()) {
            continue;
        }
        let pv = b.vertex(v);
        let ns: Vec<DVec3> = faces.iter().map(|f| outward(b, *f, pv)).collect();
        let m = DMat3::from_cols(ns[0], ns[1], ns[2]).transpose();
        if m.determinant().abs() < 1e-9 {
            continue;
        }
        let c = pv + m.inverse() * DVec3::splat(-r);
        // Pair each face with the edge not lying on it.
        let mut rows: Vec<((DVec3, f64), (DVec3, f64))> = Vec::new();
        let mut caps = Vec::new();
        for (k, f) in faces.iter().enumerate() {
            let fe = b.face_edges(*f);
            let Some(&e) = inc.iter().find(|e| !fe.contains(e)) else { continue };
            let ed = b.edge(e);
            let other = if ed.v0 == v { ed.end() } else { ed.start() };
            let d = (other - pv).normalize();
            rows.push(((ns[k], ns[k].dot(pv)), (d, d.dot(c))));
            caps.push((e, c, -d));
        }
        if rows.len() != 3 {
            continue;
        }
        let mut pts = Vec::new();
        for idx in 0..8 {
            let pick = |i: usize| if idx >> i & 1 == 0 { rows[i].0 } else { rows[i].1 };
            let (a0, a1, a2) = (pick(0), pick(1), pick(2));
            let mm = DMat3::from_cols(a0.0, a1.0, a2.0).transpose();
            if mm.determinant().abs() < 1e-12 {
                break;
            }
            pts.push(mm.inverse() * DVec3::new(a0.1, a1.1, a2.1));
        }
        if pts.len() != 8 {
            continue;
        }
        let quads = vec![vec![0, 4, 6, 2], vec![1, 3, 7, 5], vec![0, 1, 5, 4], vec![2, 6, 7, 3], vec![0, 2, 3, 1], vec![4, 5, 7, 6]];
        let mut cell = polyhedron(&pts, &quads, BodyKind::Solid);
        if crate::tess::tessellate(&cell, &crate::tess::TessOptions::default()).volume() < 0.0 {
            cell = flip_orientation(cell);
        }
        // A skewed sphere frame keeps its seam and poles off the cell's planes.
        let sf = Frame::new(c, DVec3::new(0.21, -0.37, 0.9), DVec3::new(0.83, 0.47, 0.11));
        if let Ok(t) = boolean(&cell, &make_sphere_in(&sf, r), BoolOp::Difference) {
            out.push((t, caps));
        }
    }
    out
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

    fn find_edge(b: &Body, a: DVec3, c: DVec3) -> EdgeId {
        b.edge_ids()
            .find(|e| {
                let ed = b.edge(*e);
                (ed.start().distance(a) < 1e-9 && ed.end().distance(c) < 1e-9) || (ed.start().distance(c) < 1e-9 && ed.end().distance(a) < 1e-9)
            })
            .unwrap()
    }

    #[test]
    fn fillet_one_box_edge() {
        let b = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
        let e = find_edge(&b, DVec3::new(0.0, 0.0, 2.0), DVec3::new(4.0, 0.0, 2.0));
        let r = 0.5;
        let f = blend_edges(&b, &[e], Blend::Fillet(r)).unwrap();
        assert_eq!(open_edges(&f), 0);
        let expect = 24.0 - (r * r - PI * r * r / 4.0) * 4.0;
        assert!((vol(&f) - expect).abs() < 1e-3, "{} vs {expect}", vol(&f));
        let c = blend_edges(&b, &[e], Blend::Chamfer(r)).unwrap();
        assert!((vol(&c) - (24.0 - r * r / 2.0 * 4.0)).abs() < 1e-6, "{}", vol(&c));
    }

    #[test]
    fn fillet_box_corner_three_edges() {
        let b = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
        let v = DVec3::new(4.0, 3.0, 2.0);
        let es = [
            find_edge(&b, v, DVec3::new(0.0, 3.0, 2.0)),
            find_edge(&b, v, DVec3::new(4.0, 0.0, 2.0)),
            find_edge(&b, v, DVec3::new(4.0, 3.0, 0.0)),
        ];
        let r = 0.5;
        let f = blend_edges(&b, &es, Blend::Fillet(r)).unwrap();
        assert_eq!(open_edges(&f), 0);
        // Removed: three edge spandrels (lengths minus r at the corner) and
        // the corner cube minus an eighth ball.
        let spandrel = r * r * (1.0 - PI / 4.0);
        let removed = spandrel * ((4.0 - r) + (3.0 - r) + (2.0 - r)) + (r * r * r - PI * r * r * r / 6.0);
        let expect = 24.0 - removed;
        assert!((vol(&f) - expect).abs() < 2e-3, "{} vs {expect}", vol(&f));
    }

    #[test]
    fn fillet_concave_edge() {
        // L-shaped block: the inner edge gets material added.
        let a = make_box(&Frame::WORLD, DVec3::new(4.0, 2.0, 1.0));
        let c = make_box(&Frame::WORLD, DVec3::new(1.0, 2.0, 3.0));
        let l = boolean(&a, &c, BoolOp::Union).unwrap();
        let e = find_edge(&l, DVec3::new(1.0, 0.0, 1.0), DVec3::new(1.0, 2.0, 1.0));
        let r = 0.4;
        let f = blend_edges(&l, &[e], Blend::Fillet(r)).unwrap();
        assert_eq!(open_edges(&f), 0);
        let expect = vol(&l) + r * r * (1.0 - PI / 4.0) * 2.0;
        assert!((vol(&f) - expect).abs() < 1e-3, "{} vs {expect}", vol(&f));
    }

    #[test]
    fn fillet_cylinder_rims() {
        // Boss top rim (convex) and hole rim on a plate.
        let cyl = make_cylinder(&Frame::WORLD, 2.0, 3.0);
        let top = cyl.edge_ids().find(|e| matches!(cyl.edge(*e).curve, Curve3::Circle { .. }) && cyl.edge(*e).start().z > 2.0).unwrap();
        let r = 0.5;
        let f = blend_edges(&cyl, &[top], Blend::Fillet(r)).unwrap();
        assert_eq!(open_edges(&f), 0);
        // Pappus on the removed spandrel ring.
        let area = r * r * (1.0 - PI / 4.0);
        let rho = 2.0 - r + (r * r * r / 6.0) / area;
        // Measure against the tessellated cylinder to cancel chord error.
        let base = vol(&cyl);
        let expect = base - area * 2.0 * PI * rho;
        assert!((vol(&f) - expect).abs() < 2e-3, "{} vs {expect}", vol(&f));
        let ch = blend_edges(&cyl, &[top], Blend::Chamfer(r)).unwrap();
        assert_eq!(open_edges(&ch), 0);
        let rho_c = 2.0 - r / 3.0;
        let expect_c = base - r * r / 2.0 * 2.0 * PI * rho_c;
        assert!((vol(&ch) - expect_c).abs() < 2e-3, "{} vs {expect_c}", vol(&ch));
    }

    #[test]
    fn fillet_hole_rim() {
        let plate = make_box(&Frame::new(DVec3::new(-3.0, -3.0, 0.0), DVec3::Z, DVec3::X), DVec3::new(6.0, 6.0, 2.0));
        let hole = make_cylinder(&Frame::new(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, DVec3::X), 1.0, 4.0);
        let b = boolean(&plate, &hole, BoolOp::Difference).unwrap();
        let rim = b.edge_ids().find(|e| matches!(b.edge(*e).curve, Curve3::Circle { .. }) && b.edge(*e).start().z > 1.0).unwrap();
        let r = 0.3;
        let f = blend_edges(&b, &[rim], Blend::Fillet(r)).unwrap();
        assert_eq!(open_edges(&f), 0);
        let area = r * r * (1.0 - PI / 4.0);
        let xbar = r * r * r * (5.0 / 6.0 - PI / 4.0) / area;
        let expect = vol(&b) - area * 2.0 * PI * (1.0 + xbar);
        assert!((vol(&f) - expect).abs() < 2e-3, "{} vs {expect}", vol(&f));
    }

    #[test]
    fn rounded_box_all_edges() {
        let b = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
        let all: Vec<EdgeId> = b.edge_ids().collect();
        let r = 0.4;
        let f = blend_edges(&b, &all, Blend::Fillet(r)).unwrap();
        assert_eq!(open_edges(&f), 0);
        // Minkowski sum of the inner box with a ball of radius r.
        let (a, c, d) = (4.0 - 2.0 * r, 3.0 - 2.0 * r, 2.0 - 2.0 * r);
        let expect = a * c * d + 2.0 * r * (a * c + c * d + a * d) + PI * r * r * (a + c + d) + 4.0 / 3.0 * PI * r * r * r;
        assert!((vol(&f) - expect).abs() < 5e-3, "{} vs {expect}", vol(&f));
    }
}

#[cfg(test)]
mod chain_tests {
    use super::*;
    use crate::ops::boolean::open_edges;
    use crate::ops::heal::compact;
    use crate::tess::{tessellate, TessOptions};
    use std::f64::consts::PI;

    fn vol(b: &Body) -> f64 {
        tessellate(b, &TessOptions { chord_tol: 1e-4, angle_tol: 0.05 }).volume()
    }

    #[test]
    fn rounded_corners_then_top_edges() {
        // The classic: round the vertical edges, then fillet the whole top
        // rim in one go as a tangent chain.
        let (w, d, h, c, r) = (4.0, 3.0, 2.0, 0.8, 0.3);
        let b = make_box(&Frame::WORLD, DVec3::new(w, d, h));
        let vertical: Vec<EdgeId> = b.edge_ids().filter(|e| (b.edge(*e).start() - b.edge(*e).end()).z.abs() > 1.0).collect();
        let b = compact(&blend_edges(&b, &vertical, Blend::Fillet(c)).unwrap());
        let top: Vec<EdgeId> = b.edge_ids().filter(|e| {
            let ed = b.edge(*e);
            ed.start().z > h - 1e-6 && ed.end().z > h - 1e-6
        }).collect();
        assert_eq!(top.len(), 8);
        let f = blend_edges(&b, &top, Blend::Fillet(r)).unwrap();
        assert_eq!(open_edges(&f), 0);
        let spandrel = r * r * (1.0 - PI / 4.0);
        let dbar = r * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI);
        let path = 2.0 * (w - 2.0 * c) + 2.0 * (d - 2.0 * c) + 2.0 * PI * (c - dbar);
        let expect = vol(&b) - spandrel * path;
        assert!((vol(&f) - expect).abs() < 2e-3 * spandrel * path, "{} vs {expect}", vol(&f));
    }
}
