// SPDX-License-Identifier: GPL-3.0-only
//! Deleting faces from bodies, healing the gap where the faces around it
//! allow, and pulling faces or edges out into bodies of their own.

use super::heal::{compact, simplify};
use crate::profile::{wire_from_segments, Segment};
use crate::topo::{Body, BodyKind, Coedge, EdgeId, FaceId, VertexId};
use ductile_geom::{Curve3, DVec3, Surface};
use std::collections::{HashMap, HashSet};

/// Removes `faces` from `b`. On a solid, a hole loop of a remaining face
/// whose edges all border deleted faces is dropped as well, so deleting a
/// hole or a boss closes the face it sat in and the solid stays solid.
/// A fillet or chamfer between two planes is removed by growing the
/// planes back to their sharp edge, and a round fillet band (round a boss,
/// say) by growing the faces either side to their circle of contact. When
/// the gap cannot be closed the result is an open sheet.
pub fn delete_faces(b: &Body, faces: &[FaceId]) -> Body {
    let gone: HashSet<FaceId> = faces.iter().copied().collect();
    if gone.is_empty() {
        return b.clone();
    }
    let heal = b.kind == BodyKind::Solid;
    let mut work = b.clone();
    if heal {
        for f in faces {
            let nbrs = neighbours(&work, *f);
            if nbrs.iter().all(|n| !gone.contains(n)) {
                let mut trial = work.clone();
                if remove_blend(&mut trial, *f) {
                    work = trial;
                    continue;
                }
                let mut trial = work.clone();
                if remove_band(&mut trial, *f) {
                    work = trial;
                }
            }
        }
    }
    let b = &work;
    let mut out = b.clone();
    let users = edge_users(b);
    let mut kept = Vec::new();
    for f in b.face_ids() {
        if gone.contains(&f) {
            continue;
        }
        let mut face = b.face(f).clone();
        if heal {
            let borders_gap = |l: &crate::topo::LoopId| {
                b.loops[l.idx()].coedges.iter().all(|c| users[&c.edge].iter().any(|g| *g != f && gone.contains(g)))
            };
            let outer = face.loops.first().copied();
            face.loops.retain(|l| Some(*l) == outer || b.loops[l.idx()].coedges.is_empty() || !borders_gap(l));
        }
        kept.push(face);
    }
    out.faces = kept;
    if out.faces.is_empty() {
        return Body::new(BodyKind::Sheet);
    }
    let closed = edge_users(&out).values().all(|u| u.len() == 2);
    out.kind = if heal && closed { BodyKind::Solid } else { BodyKind::Sheet };
    if out.kind == BodyKind::Solid {
        simplify(&out)
    } else {
        compact(&out)
    }
}

/// The faces `faces` of `b` copied out as an open sheet.
pub fn extract_faces(b: &Body, faces: &[FaceId]) -> Body {
    let mut one = b.clone();
    one.kind = BodyKind::Sheet;
    one.faces = faces.iter().map(|f| b.face(*f).clone()).collect();
    compact(&one)
}

/// The edges `edges` of `b` copied out as a wire body of curves.
pub fn extract_edges(b: &Body, edges: &[EdgeId]) -> Body {
    let segs: Vec<Segment> = edges
        .iter()
        .map(|e| {
            let ed = b.edge(*e);
            Segment { curve: ed.curve.clone(), t0: ed.t0, t1: ed.t1, reversed: false }
        })
        .collect();
    wire_from_segments(&segs)
}

/// The faces across the edges of `f`.
fn neighbours(b: &Body, f: FaceId) -> Vec<FaceId> {
    let users = edge_users(b);
    let mut out: Vec<FaceId> = b.face_loops(f).flat_map(|l| l.coedges.iter()).flat_map(|c| users[&c.edge].iter().copied()).filter(|g| *g != f).collect();
    out.sort();
    out.dedup();
    out
}

/// Grows the two planes either side of the four-sided blend face `f` back
/// to the line where they meet, and closes the end faces over the gap.
/// The ends must meet the planes along straight edges. `f` is left with
/// no loops; returns false (with `b` partly changed) when it does not fit.
fn remove_blend(b: &mut Body, f: FaceId) -> bool {
    let face = b.face(f).clone();
    if face.loops.len() != 1 {
        return false;
    }
    let co = b.loops[face.loops[0].idx()].coedges.clone();
    if co.len() != 4 {
        return false;
    }
    let tol = 1e-6 * b.bbox().diagonal().max(1.0);
    let across = |b: &Body, e: EdgeId| -> Option<FaceId> {
        let fs = b.edge_faces(e);
        if fs.len() != 2 {
            return None;
        }
        fs.iter().map(|x| x.0).find(|x| *x != f)
    };
    let plane = |b: &Body, g: FaceId| match &b.face(g).surface {
        Surface::Plane { frame } => Some((frame.z, frame.z.dot(frame.origin))),
        _ => None,
    };
    for k in 0..2 {
        let (ea, ec, eb, ed) = (co[k].edge, co[k + 1].edge, co[(k + 2) % 4].edge, co[(k + 3) % 4].edge);
        let ring = [ea, ec, eb, ed];
        let (Some(fa), Some(fb), Some(_), Some(_)) = (across(b, ea), across(b, eb), across(b, ec), across(b, ed)) else { continue };
        let (Some((n1, d1)), Some((n2, d2))) = (plane(b, fa), plane(b, fb)) else { continue };
        let dir = n1.cross(n2);
        if dir.length() < 1e-6 {
            continue;
        }
        let dir = dir.normalize();
        let c = n1.dot(n2);
        let p0 = (n1 * (d1 - d2 * c) + n2 * (d2 - d1 * c)) / (1.0 - c * c);
        // Where the sharp edge meets each end: the straight edges leaving
        // the blend's corners run into it.
        let mut merges: Vec<(VertexId, VertexId, DVec3)> = Vec::new();
        for e in [ec, ed] {
            let (v0, v1) = (b.edge(e).v0, b.edge(e).v1);
            let mut pts = Vec::new();
            for v in [v0, v1] {
                let others: Vec<EdgeId> = b.edge_ids().filter(|x| !ring.contains(x) && (b.edge(*x).v0 == v || b.edge(*x).v1 == v)).collect();
                let [o] = others[..] else { break };
                let Curve3::Line { origin, dir: d } = b.edge(o).curve else { break };
                let Some(p) = line_meet(p0, dir, origin, d, tol) else { break };
                pts.push(p);
            }
            if pts.len() != 2 || pts[0].distance(pts[1]) > tol {
                break;
            }
            merges.push((v0, v1, 0.5 * (pts[0] + pts[1])));
        }
        if merges.len() != 2 {
            continue;
        }
        let mut remap: HashMap<VertexId, VertexId> = HashMap::new();
        for (keep, drop, p) in &merges {
            b.vertices[keep.idx()].point = *p;
            remap.insert(*drop, *keep);
        }
        let m = |v: VertexId| *remap.get(&v).unwrap_or(&v);
        for i in 0..b.edges.len() {
            let e = &mut b.edges[i];
            (e.v0, e.v1) = (m(e.v0), m(e.v1));
        }
        // Straight edges into the moved corners stretch to reach them.
        let moved: Vec<VertexId> = merges.iter().map(|x| x.0).collect();
        for i in 0..b.edges.len() {
            let id = EdgeId(i as u32);
            let e = &b.edges[i];
            if ring.contains(&id) || !(moved.contains(&e.v0) || moved.contains(&e.v1)) || !matches!(e.curve, Curve3::Line { .. }) {
                continue;
            }
            let (p, q) = (b.vertex(e.v0), b.vertex(e.v1));
            if p.distance(q) < tol {
                return false;
            }
            let e = &mut b.edges[i];
            e.curve = Curve3::line_through(p, q);
            (e.t0, e.t1) = (0.0, p.distance(q));
        }
        let (a0, a1) = (b.edge(ea).v0, b.edge(ea).v1);
        let sharp = b.add_line_edge(a0, a1);
        for l in 0..b.loops.len() {
            let cs = &mut b.loops[l].coedges;
            for c in cs.iter_mut() {
                if c.edge == ea {
                    c.edge = sharp;
                } else if c.edge == eb {
                    let start = if c.reversed { b.edges[eb.idx()].v1 } else { b.edges[eb.idx()].v0 };
                    *c = Coedge::new(sharp, start != a0);
                }
            }
            cs.retain(|c| c.edge != ec && c.edge != ed);
        }
        let fl = b.faces[f.idx()].loops[0];
        b.loops[fl.idx()].coedges.clear();
        return true;
    }
    false
}

/// Removes a closed blend band — a face bounded by two closed edges and a
/// seam, like a fillet round a circular edge — by growing the faces either
/// side until they meet in the circle where their surfaces cross. `f` is
/// left with no loops; returns false (with `b` partly changed) when the
/// faces do not meet in one circle near the band.
fn remove_band(b: &mut Body, f: FaceId) -> bool {
    let face = b.face(f).clone();
    if face.loops.len() != 1 {
        return false;
    }
    let co = b.loops[face.loops[0].idx()].coedges.clone();
    let mut count: HashMap<EdgeId, usize> = HashMap::new();
    for c in &co {
        *count.entry(c.edge).or_default() += 1;
    }
    let seams: Vec<EdgeId> = count.iter().filter(|(_, n)| **n == 2).map(|(e, _)| *e).collect();
    let rims: Vec<EdgeId> = co.iter().map(|c| c.edge).filter(|e| count[e] == 1).collect();
    let ([seam], [ea, eb]) = (&seams[..], &rims[..]) else { return false };
    let (seam, ea, eb) = (*seam, *ea, *eb);
    if co.len() != 4 || b.edge(ea).v0 != b.edge(ea).v1 || b.edge(eb).v0 != b.edge(eb).v1 {
        return false;
    }
    let across = |b: &Body, e: EdgeId| -> Option<FaceId> {
        let fs = b.edge_faces(e);
        (fs.len() == 2).then(|| fs.iter().map(|x| x.0).find(|x| *x != f)).flatten()
    };
    let (Some(fa), Some(fb)) = (across(b, ea), across(b, eb)) else { return false };
    if fa == fb {
        return false;
    }
    let tol = 1e-6 * b.bbox().diagonal().max(1.0);
    let roi = b.bbox().expanded(b.bbox().diagonal());
    let Some(crate::ssi::Ssi::Curves(curves)) = crate::ssi::intersect_analytic(&b.face(fa).surface, &b.face(fb).surface, &roi, tol) else { return false };
    let (va, vb) = (b.edge(ea).v0, b.edge(eb).v0);
    let mid = 0.5 * (b.vertex(va) + b.vertex(vb));
    // The circle nearest the band.
    let near = |c: &Curve3| c.point(c.closest_param(mid)).distance(mid);
    let Some((Curve3::Circle { frame, radius }, _, _)) = curves.iter().filter(|c| matches!(c.0, Curve3::Circle { .. })).min_by(|x, y| near(&x.0).total_cmp(&near(&y.0))).cloned() else { return false };
    let band = b.vertex(va).distance(b.vertex(vb));
    let start = { let c = Curve3::Circle { frame, radius }; c.point(c.closest_param(mid)) };
    if start.distance(mid) > 2.0 * band + tol {
        return false;
    }
    let ring = Curve3::Circle { frame: ductile_geom::Frame::new(frame.origin, frame.z, start - frame.origin), radius };
    // Which way each old rim ran, at a point of it, decides the new one's
    // orientation in the faces that used it.
    let flipped = |b: &Body, e: EdgeId, rev: bool| {
        let ed = b.edge(e);
        let t = 0.5 * (ed.t0 + ed.t1);
        let along = ed.curve.tangent(t) * if rev { -1.0 } else { 1.0 };
        let q = ed.curve.point(t);
        along.dot(ring.tangent(ring.closest_param(q))) < 0.0
    };
    let mut fixes: Vec<(usize, usize, bool)> = Vec::new();
    for (l, lp) in b.loops.iter().enumerate() {
        for (k, c) in lp.coedges.iter().enumerate() {
            if (c.edge == ea || c.edge == eb) && !face.loops.iter().any(|x| x.idx() == l) {
                fixes.push((l, k, flipped(b, c.edge, c.reversed)));
            }
        }
    }
    b.vertices[va.idx()].point = start;
    for e in b.edges.iter_mut() {
        if e.v0 == vb {
            e.v0 = va;
        }
        if e.v1 == vb {
            e.v1 = va;
        }
    }
    {
        let e = &mut b.edges[ea.idx()];
        (e.curve, e.t0, e.t1) = (ring, 0.0, std::f64::consts::TAU);
    }
    for (l, k, rev) in fixes {
        b.loops[l].coedges[k] = Coedge::new(ea, rev);
    }
    // Straight edges into the moved vertex stretch to reach it.
    for i in 0..b.edges.len() {
        let id = EdgeId(i as u32);
        let e = &b.edges[i];
        if [ea, eb, seam].contains(&id) || (e.v0 != va && e.v1 != va) {
            continue;
        }
        if !matches!(e.curve, Curve3::Line { .. }) {
            return false;
        }
        let (p, q) = (b.vertex(e.v0), b.vertex(e.v1));
        if p.distance(q) < tol {
            return false;
        }
        let e = &mut b.edges[i];
        e.curve = Curve3::line_through(p, q);
        (e.t0, e.t1) = (0.0, p.distance(q));
    }
    let fl = b.faces[f.idx()].loops[0];
    b.loops[fl.idx()].coedges.clear();
    true
}

/// The point on the line (p, d) where the line (q, e) crosses it.
fn line_meet(p: DVec3, d: DVec3, q: DVec3, e: DVec3, tol: f64) -> Option<DVec3> {
    let e = e.normalize();
    let (a, b, c) = (d.dot(d), d.dot(e), e.dot(e));
    let den = a * c - b * b;
    if den.abs() < 1e-12 {
        return None;
    }
    let w = p - q;
    let s = (b * e.dot(w) - c * d.dot(w)) / den;
    let t = (a * e.dot(w) - b * d.dot(w)) / den;
    let (x, y) = (p + d * s, q + e * t);
    (x.distance(y) < tol * 10.0).then_some(x)
}

fn edge_users(b: &Body) -> HashMap<EdgeId, Vec<FaceId>> {
    let mut users: HashMap<EdgeId, Vec<FaceId>> = HashMap::new();
    for f in b.face_ids() {
        for l in b.face_loops(f) {
            for c in &l.coedges {
                users.entry(c.edge).or_default().push(f);
            }
        }
    }
    users
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ops::boolean::{boolean, BoolOp};
    use crate::primitives::{make_box, make_cylinder};
    use crate::tess::{tessellate, TessOptions};
    use ductile_geom::{DVec3, Frame, Surface};

    fn vol(b: &Body) -> f64 {
        tessellate(b, &TessOptions { chord_tol: 1e-4, angle_tol: 0.05 }).volume()
    }

    #[test]
    fn deleting_a_hole_heals_the_solid() {
        let bx = make_box(&Frame::WORLD, DVec3::new(4.0, 4.0, 2.0));
        let cyl = make_cylinder(&Frame::new(DVec3::new(2.0, 2.0, -1.0), DVec3::Z, DVec3::X), 1.0, 4.0);
        let holed = boolean(&bx, &cyl, BoolOp::Difference).unwrap();
        let round: Vec<FaceId> = holed.face_ids().filter(|f| matches!(holed.face(*f).surface, Surface::Cylinder { .. })).collect();
        assert!(!round.is_empty());
        let healed = delete_faces(&holed, &round);
        assert_eq!(healed.kind, BodyKind::Solid);
        assert_eq!(healed.faces.len(), 6);
        assert!((vol(&healed) - 32.0).abs() < 1e-6, "{}", vol(&healed));
    }

    #[test]
    fn deleting_fillets_and_chamfers_grows_the_corner_back() {
        use crate::ops::fillet::{blend_edges, Blend};
        let bx = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
        let e = bx.edge_ids().find(|e| {
            let m = bx.edge(*e).mid();
            m.x > 3.9 && m.z > 1.9
        });
        for blend in [Blend::Fillet(0.5), Blend::Chamfer(0.5)] {
            let rounded = blend_edges(&bx, &[e.unwrap()], blend).unwrap();
            assert_eq!(rounded.faces.len(), 7);
            let f = rounded.face_ids().find(|f| !matches!(rounded.face(*f).surface, Surface::Plane { frame } if frame.z.abs().max_element() > 0.999)).unwrap();
            let sharp = delete_faces(&rounded, &[f]);
            assert_eq!((sharp.kind, sharp.faces.len()), (BodyKind::Solid, 6));
            assert!((vol(&sharp) - 24.0).abs() < 1e-6, "{}", vol(&sharp));
        }
        // Inside corner: a fillet where a step meets the floor.
        let step = boolean(&bx, &make_box(&Frame { origin: DVec3::new(2.0, 0.0, 1.0), ..Frame::WORLD }, DVec3::new(2.0, 3.0, 1.0)), BoolOp::Difference).unwrap();
        let inner = step.edge_ids().find(|e| {
            let m = step.edge(*e).mid();
            (m.x - 2.0).abs() < 1e-9 && (m.z - 1.0).abs() < 1e-9
        });
        let rounded = blend_edges(&step, &[inner.unwrap()], Blend::Fillet(0.4)).unwrap();
        let f = rounded.face_ids().find(|f| matches!(rounded.face(*f).surface, Surface::Cylinder { .. })).unwrap();
        let sharp = delete_faces(&rounded, &[f]);
        assert_eq!(sharp.kind, BodyKind::Solid);
        assert!((vol(&sharp) - 18.0).abs() < 1e-6, "{}", vol(&sharp));
    }

    #[test]
    fn deleting_round_fillets() {
        use crate::ops::fillet::{blend_edges, Blend};
        // A boss on a plate, rounded at its top and filleted at its foot.
        let plate = make_box(&Frame::WORLD, DVec3::new(6.0, 6.0, 1.0));
        let boss = make_cylinder(&Frame::new(DVec3::new(3.0, 3.0, 0.5), DVec3::Z, DVec3::X), 1.5, 2.5);
        let part = boolean(&plate, &boss, BoolOp::Union).unwrap();
        let full = vol(&part);
        let circle_at = |b: &Body, z: f64| b.edge_ids().find(|e| matches!(b.edge(*e).curve, ductile_geom::Curve3::Circle { .. }) && (b.edge(*e).mid().z - z).abs() < 1e-9).unwrap();
        for (z, r) in [(3.0, 0.4), (1.0, 0.3)] {
            let e = circle_at(&part, z);
            let rounded = blend_edges(&part, &[e], Blend::Fillet(r)).unwrap();
            let f = rounded.face_ids().find(|f| matches!(rounded.face(*f).surface, Surface::Torus { .. })).unwrap();
            let sharp = delete_faces(&rounded, &[f]);
            assert_eq!(sharp.kind, BodyKind::Solid, "z {z}");
            assert!((vol(&sharp) - full).abs() < 1e-4, "z {z}: {} vs {full}", vol(&sharp));
        }
    }

    #[test]
    fn deleting_a_box_face_leaves_a_sheet() {
        let bx = make_box(&Frame::WORLD, DVec3::ONE);
        let open = delete_faces(&bx, &[FaceId(0)]);
        assert_eq!(open.kind, BodyKind::Sheet);
        assert_eq!(open.faces.len(), 5);
        let one = extract_faces(&bx, &[FaceId(1)]);
        assert_eq!((one.kind, one.faces.len(), one.edges.len()), (BodyKind::Sheet, 1, 4));
        let wire = extract_edges(&bx, &[EdgeId(0), EdgeId(1)]);
        assert_eq!((wire.kind, wire.edges.len()), (BodyKind::Wire, 2));
    }
}
