// SPDX-License-Identifier: GPL-3.0-only
//! Rolling-ball blends along chains of tangent edges.
//!
//! A chain is a run of edges meeting tangentially (the top edges of an
//! extruded rounded rectangle, a spline profile's edges, the side edge of a
//! prism). The ball is rolled along the whole run and every edge gets its
//! own blend face; neighbouring blend faces share their junction section,
//! an exact arc, so they meet without gaps. The faces along the chain are
//! cut back to the contact curves, the edges that ended at the chain's
//! vertices are shortened to the new contact points, and an open chain is
//! closed off by the face its ends run into (the cap of a prism).

use super::fillet::{Blend, FilletError};
use super::roll::{arc_mid, ball, move_end};
use crate::topo::{Body, Coedge, EdgeId, FaceId, VertexId};
use ductile_geom::{Curve3, DVec2, DVec3, NurbsCurve, NurbsSurface, Surface};

/// One edge of a chain, with `fwd` true when the chain runs along the
/// edge's own direction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Link {
    pub edge: EdgeId,
    pub fwd: bool,
}

/// Groups edges into chains of tangent edges. Each chain is listed in
/// order with its direction, and flagged when it closes on itself.
pub fn tangent_chains(body: &Body, edges: &[EdgeId]) -> Vec<(Vec<Link>, bool)> {
    let tangent_at = |e: EdgeId, v: VertexId, out: bool| {
        let ed = body.edge(e);
        let t = if ed.v0 == v { ed.curve.tangent(ed.t0) } else { -ed.curve.tangent(ed.t1) };
        let t = t.normalize_or_zero();
        if out {
            t
        } else {
            -t
        }
    };
    // The selected edge continuing `e` smoothly through `v`, if exactly one does.
    let next = |e: EdgeId, v: VertexId, used: &[EdgeId]| -> Option<EdgeId> {
        let at: Vec<EdgeId> = edges.iter().copied().filter(|x| *x != e && { let xe = body.edge(*x); xe.v0 == v || xe.v1 == v }).collect();
        if at.len() != 1 || used.contains(&at[0]) || body.edge(at[0]).is_closed() {
            return None;
        }
        let a = tangent_at(e, v, false);
        let b = tangent_at(at[0], v, true);
        (a.dot(b) > (2.0f64).to_radians().cos()).then_some(at[0])
    };
    let mut done: Vec<EdgeId> = Vec::new();
    let mut out = Vec::new();
    for &e in edges {
        if done.contains(&e) {
            continue;
        }
        let ed = body.edge(e);
        if ed.is_closed() {
            done.push(e);
            out.push((vec![Link { edge: e, fwd: true }], true));
            continue;
        }
        let mut chain = vec![Link { edge: e, fwd: true }];
        done.push(e);
        // Forward from the end vertex.
        let closed;
        loop {
            let last = *chain.last().unwrap();
            let le = body.edge(last.edge);
            let v = if last.fwd { le.v1 } else { le.v0 };
            if v == ed.v0 && chain.len() > 1 {
                closed = true;
                break;
            }
            match next(last.edge, v, &done) {
                Some(x) => {
                    done.push(x);
                    chain.push(Link { edge: x, fwd: body.edge(x).v0 == v });
                }
                None => {
                    closed = false;
                    break;
                }
            }
        }
        if !closed {
            // Backward from the start vertex.
            loop {
                let first = chain[0];
                let fe = body.edge(first.edge);
                let v = if first.fwd { fe.v0 } else { fe.v1 };
                match next(first.edge, v, &done) {
                    Some(x) => {
                        done.push(x);
                        chain.insert(0, Link { edge: x, fwd: body.edge(x).v1 == v });
                    }
                    None => break,
                }
            }
        }
        out.push((chain, closed));
    }
    out
}

struct Section {
    p1: DVec3,
    p2: DVec3,
    curve: NurbsCurve,
}

/// Blends a chain of tangent edges (not a single closed edge; see
/// `roll::roll_closed_edge`). Old entities are left unreferenced, so ids
/// stay valid; compact the body afterwards.
pub fn roll_chain(body: &Body, chain: &[Link], closed: bool, blend: Blend) -> Result<Body, FilletError> {
    let k = chain.len();
    let size = body.bbox().diagonal().max(1e-9);
    let reach = match blend {
        Blend::Fillet(r) | Blend::Chamfer(r) => r,
    };
    // Chain vertices: start of each link, plus the end of the last one.
    let start_of = |l: &Link| {
        let e = body.edge(l.edge);
        if l.fwd {
            e.v0
        } else {
            e.v1
        }
    };
    let end_of = |l: &Link| {
        let e = body.edge(l.edge);
        if l.fwd {
            e.v1
        } else {
            e.v0
        }
    };
    let mut verts: Vec<VertexId> = chain.iter().map(start_of).collect();
    verts.push(end_of(&chain[k - 1]));
    for i in 1..k {
        if start_of(&chain[i]) != end_of(&chain[i - 1]) {
            return Err(FilletError::Unsupported);
        }
    }
    if closed != (verts[0] == verts[k]) {
        return Err(FilletError::Unsupported);
    }
    // Side A runs along the chain, side B against it.
    let mut sides: Vec<(FaceId, FaceId)> = Vec::with_capacity(k);
    for l in chain {
        let uses = body.edge_faces(l.edge);
        if uses.len() != 2 || uses[0].0 == uses[1].0 {
            return Err(FilletError::NotManifold);
        }
        let along = |c: &Coedge| c.reversed != l.fwd;
        match (along(&uses[0].1), along(&uses[1].1)) {
            (true, false) => sides.push((uses[0].0, uses[1].0)),
            (false, true) => sides.push((uses[1].0, uses[0].0)),
            _ => return Err(FilletError::NotManifold),
        }
    }
    let others = |v: VertexId| -> Vec<EdgeId> {
        body.edge_ids()
            .filter(|x| !chain.iter().any(|l| l.edge == *x))
            .filter(|x| {
                let xe = body.edge(*x);
                (xe.v0 == v || xe.v1 == v) && !body.edge_faces(*x).is_empty()
            })
            .collect()
    };
    // Open ends run into a cap face: two edges leave the end vertex, one
    // on each side, and the face between them is the cap.
    struct End {
        cap: FaceId,
        ea: EdgeId,
        eb: EdgeId,
    }
    let end_at = |j: usize, (fa, fb): (FaceId, FaceId)| -> Result<End, FilletError> {
        let xs = others(verts[j]);
        if xs.len() != 2 {
            return Err(FilletError::Unsupported);
        }
        let faces = |x: EdgeId| body.edge_faces(x).iter().map(|u| u.0).collect::<Vec<_>>();
        let (f0, f1) = (faces(xs[0]), faces(xs[1]));
        let (ea, eb) = if f0.contains(&fa) && f1.contains(&fb) {
            (xs[0], xs[1])
        } else if f1.contains(&fa) && f0.contains(&fb) {
            (xs[1], xs[0])
        } else {
            return Err(FilletError::Unsupported);
        };
        let cap = faces(ea).into_iter().find(|f| *f != fa).ok_or(FilletError::Unsupported)?;
        if !faces(eb).contains(&cap) || cap == fb {
            return Err(FilletError::Unsupported);
        }
        Ok(End { cap, ea, eb })
    };
    let ends = if closed { None } else { Some((end_at(0, sides[0])?, end_at(k, sides[k - 1])?)) };

    // Sections. The ball's centre lies in the plane normal to the chain,
    // except at an open end, where it lies in the cap's plane so that the
    // end section sits on the cap.
    let mut convex: Option<bool> = None;
    let mut hints: (Option<DVec2>, Option<DVec2>) = (None, None);
    let mut section = |i: usize, t: f64, plane: Option<DVec3>, hints: &mut (Option<DVec2>, Option<DVec2>)| -> Result<Section, FilletError> {
        let l = chain[i];
        let ed = body.edge(l.edge);
        let (fa, fb) = sides[i];
        let (sa, sb) = (&body.face(fa).surface, &body.face(fb).surface);
        let q = ed.curve.point(t);
        let tan = (ed.curve.tangent(t) * if l.fwd { 1.0 } else { -1.0 }).normalize();
        let uva = sa.project(q, hints.0);
        let uvb = sb.project(q, hints.1);
        let na = body.face_normal(fa, uva.x, uva.y);
        let nb = body.face_normal(fb, uvb.x, uvb.y);
        let cross = na.cross(nb);
        if cross.length() < 1e-6 {
            return Err(FilletError::Smooth);
        }
        let cv = cross.dot(tan) > 0.0;
        if *convex.get_or_insert(cv) != cv {
            return Err(FilletError::Unsupported);
        }
        let normal = match plane {
            Some(n) if n.dot(tan) < 0.0 => -n,
            Some(n) => n,
            None => tan,
        };
        let (p1, p2, mid) = match blend {
            Blend::Fillet(r) => {
                let sgn = if cv { -1.0 } else { 1.0 };
                let (c, p1, p2, h1, h2) = ball(body, (fa, sa), (fb, sb), q, normal, na, nb, sgn * r, (uva, uvb)).ok_or(FilletError::TooLarge)?;
                *hints = (Some(h1), Some(h2));
                if p1.distance(q) > size || p2.distance(q) > size {
                    return Err(FilletError::TooLarge);
                }
                let (m, w) = arc_mid(c, p1, p2);
                (p1, p2, (m, w))
            }
            Blend::Chamfer(d) => {
                let (u1, u2) = (na.cross(tan), tan.cross(nb));
                let h1 = sa.project(q + u1 * d, Some(uva));
                let h2 = sb.project(q + u2 * d, Some(uvb));
                *hints = (Some(uva), Some(uvb));
                let (p1, p2) = (sa.point(h1.x, h1.y), sb.point(h2.x, h2.y));
                (p1, p2, ((p1 + p2) * 0.5, 1.0))
            }
        };
        let curve = NurbsCurve::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![p1.extend(1.0), (mid.0 * mid.1).extend(mid.1), p2.extend(1.0)]);
        Ok(Section { p1, p2, curve })
    };
    let cap_normal = |f: FaceId| match &body.face(f).surface {
        Surface::Plane { frame } => Some(frame.z),
        _ => None,
    };
    let mut rows: Vec<Vec<Section>> = Vec::with_capacity(k);
    for (i, l) in chain.iter().enumerate() {
        let ed = body.edge(l.edge);
        let (ta, tb) = if l.fwd { (ed.t0, ed.t1) } else { (ed.t1, ed.t0) };
        let turning: f64 = {
            let m = 32;
            (0..m)
                .map(|s| {
                    let a = ed.curve.tangent(ta + (tb - ta) * s as f64 / m as f64).normalize();
                    let b = ed.curve.tangent(ta + (tb - ta) * (s + 1) as f64 / m as f64).normalize();
                    a.dot(b).clamp(-1.0, 1.0).acos()
                })
                .sum()
        };
        let len = ed.length();
        let m = ((turning / 0.06).max(2.0 * len / reach.max(1e-9)).ceil() as usize).clamp(4, 160);
        let mut row = Vec::with_capacity(m + 1);
        for s in 0..=m {
            if s == 0 && i > 0 {
                // The junction section, shared with the previous edge.
                let prev = rows.last().unwrap();
                let last = prev.last().unwrap();
                row.push(Section { p1: last.p1, p2: last.p2, curve: last.curve.clone() });
                continue;
            }
            let t = ta + (tb - ta) * s as f64 / m as f64;
            let plane = match &ends {
                Some((e0, _)) if i == 0 && s == 0 => cap_normal(e0.cap),
                Some((_, e1)) if i == k - 1 && s == m => cap_normal(e1.cap),
                _ => None,
            };
            row.push(section(i, t, plane, &mut hints)?);
        }
        rows.push(row);
    }
    if closed {
        // The last section is the first one.
        let first = &rows[0][0];
        let (p1, p2, curve) = (first.p1, first.p2, first.curve.clone());
        let last = rows[k - 1].last_mut().unwrap();
        if last.p1.distance(p1) > 1e-6 * size || last.p2.distance(p2) > 1e-6 * size {
            return Err(FilletError::Unsupported);
        }
        *last = Section { p1, p2, curve };
    }
    let convex = convex.unwrap_or(true);
    let surfaces: Vec<NurbsSurface> = rows
        .iter()
        .map(|row| {
            let curves: Vec<NurbsCurve> = row.iter().map(|s| s.curve.clone()).collect();
            let m = row.len() - 1;
            let params: Vec<f64> = (0..=m).map(|s| s as f64 / m as f64).collect();
            NurbsSurface::loft_with_params(&curves, 3, &params)
        })
        .collect();

    let mut out = body.clone();
    // Junction vertices and arcs.
    let nj = if closed { k } else { k + 1 };
    let pv: Vec<(VertexId, VertexId)> = (0..nj)
        .map(|j| {
            let s = if j < k { &rows[j][0] } else { rows[k - 1].last().unwrap() };
            (out.add_vertex(s.p1), out.add_vertex(s.p2))
        })
        .collect();
    let jv = |j: usize| pv[j % nj];
    let junctions: Vec<EdgeId> = (0..nj)
        .map(|j| {
            let s = if j < k { &rows[j][0] } else { rows[k - 1].last().unwrap() };
            let (a, b) = pv[j];
            out.add_edge(Curve3::Nurbs(s.curve.clone()), 0.0, 1.0, a, b)
        })
        .collect();
    let junction = |j: usize| junctions[j % nj];
    // Contact curves, along the chain.
    let mut contacts: Vec<(EdgeId, EdgeId)> = Vec::with_capacity(k);
    for (i, s) in surfaces.iter().enumerate() {
        let ((u0, u1), (v0, v1)) = s.domain();
        let c1 = out.add_edge(Curve3::Nurbs(s.iso_u(u0)), v0, v1, jv(i).0, jv(i + 1).0);
        let c2 = out.add_edge(Curve3::Nurbs(s.iso_u(u1)), v0, v1, jv(i).1, jv(i + 1).1);
        contacts.push((c1, c2));
    }
    // Close the caps: the end arc goes between the two cap edges at the
    // old end vertex.
    if let Some((e0, e1)) = &ends {
        for (end, j) in [(e0, 0), (e1, k)] {
            let v = verts[j];
            let lp = out.faces[end.cap.idx()]
                .loops
                .iter()
                .copied()
                .find(|l| out.loops[l.idx()].coedges.iter().any(|c| c.edge == end.ea))
                .ok_or(FilletError::Unsupported)?;
            let cs = out.loops[lp.idx()].coedges.clone();
            let pos = (0..cs.len()).find(|p| out.coedge_vertices(cs[*p]).1 == v).ok_or(FilletError::Unsupported)?;
            // Coming in along ea means the arc runs from A's side to B's.
            let rev = cs[pos].edge != end.ea;
            out.loops[lp.idx()].coedges.insert(pos + 1, Coedge::new(junction(j), rev));
        }
    }
    // Edges ending at the chain's vertices now end at the contact points.
    for j in 0..nj {
        let v = verts[j];
        let (mut side_a, mut side_b) = (vec![], vec![]);
        for i in [(j + k - 1) % k, j] {
            if (i == j && j < k) || (i != j && (j > 0 || closed)) {
                side_a.push(sides[i].0);
                side_b.push(sides[i].1);
            }
        }
        for x in others(v) {
            let faces: Vec<FaceId> = body.edge_faces(x).iter().map(|u| u.0).collect();
            let to_a = match &ends {
                Some((e0, _)) if j == 0 && (x == e0.ea || x == e0.eb) => x == e0.ea,
                Some((_, e1)) if j == k && (x == e1.ea || x == e1.eb) => x == e1.ea,
                _ => {
                    if faces.iter().all(|f| side_a.contains(f)) {
                        true
                    } else if faces.iter().all(|f| side_b.contains(f)) {
                        false
                    } else {
                        return Err(FilletError::Unsupported);
                    }
                }
            };
            let (pa, pb) = jv(j);
            let (tv, p) = if to_a { (pa, out.vertex(pa)) } else { (pb, out.vertex(pb)) };
            move_end(&mut out, x, v, p, tv, size)?;
        }
    }
    // Faces along the chain now run along the contact curves.
    for (i, l) in chain.iter().enumerate() {
        let (fa, fb) = sides[i];
        set_coedge(&mut out, fa, l.edge, Coedge::new(contacts[i].0, false));
        set_coedge(&mut out, fb, l.edge, Coedge::new(contacts[i].1, true));
    }
    // Blend faces.
    for (i, s) in surfaces.into_iter().enumerate() {
        let (c1, c2) = contacts[i];
        let lp = vec![Coedge::new(c1, true), Coedge::new(junction(i), false), Coedge::new(c2, false), Coedge::new(junction(i + 1), true)];
        let ed = body.edge(chain[i].edge);
        let q = ed.curve.point(0.5 * (ed.t0 + ed.t1));
        let reversed = blend_reversed(&s, blend, convex, q);
        out.add_face(Surface::Nurbs(s), reversed, vec![lp]);
    }
    Ok(out)
}

/// Whether a blend face's surface normal points into the material: the
/// outward side is away from the ball centre on convex edges and towards it
/// on concave ones; for chamfers, away from the edge `q` (or towards it).
pub(crate) fn blend_reversed(surf: &NurbsSurface, blend: Blend, convex: bool, q: DVec3) -> bool {
    let ((u0, u1), (v0, v1)) = surf.domain();
    let (um, vm) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
    let pm = surf.point(um, vm);
    let chord_mid = (surf.point(u0, vm) + surf.point(u1, vm)) * 0.5;
    let outward = match blend {
        Blend::Fillet(_) => {
            let away = (pm - chord_mid).normalize_or_zero();
            if convex {
                away
            } else {
                -away
            }
        }
        Blend::Chamfer(_) => {
            let d = (pm - q).normalize_or_zero();
            if convex {
                -d
            } else {
                d
            }
        }
    };
    Surface::Nurbs(surf.clone()).normal(um, vm).dot(outward) < 0.0
}

fn set_coedge(b: &mut Body, f: FaceId, old: EdgeId, new: Coedge) {
    for l in b.faces[f.idx()].loops.clone() {
        for c in b.loops[l.idx()].coedges.iter_mut() {
            if c.edge == old {
                *c = new;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ops::boolean::open_edges;
    use crate::ops::extrude::extrude_region;
    use crate::ops::heal::compact;
    use crate::profile::{Loop3, Region, Segment};
    use crate::tess::{tessellate, TessOptions};
    use ductile_geom::Frame;
    use std::f64::consts::PI;

    fn vol(b: &Body) -> f64 {
        tessellate(b, &TessOptions { chord_tol: 1e-4, angle_tol: 0.05 }).volume()
    }

    /// Rounded rectangle 2a x 2b with corner radius c, extruded by h.
    fn rounded_slab(a: f64, b: f64, c: f64, h: f64) -> Body {
        let mut segs = Vec::new();
        let corners = [(a - c, b - c, 0.0), (-(a - c), b - c, 0.5 * PI), (-(a - c), -(b - c), PI), (a - c, -(b - c), 1.5 * PI)];
        for (i, (x, y, a0)) in corners.iter().enumerate() {
            let frame = Frame { origin: DVec3::new(*x, *y, 0.0), ..Frame::WORLD };
            segs.push(Segment::new(Curve3::Circle { frame, radius: c }, *a0, a0 + 0.5 * PI));
            let (nx, ny, na) = corners[(i + 1) % 4];
            let p = DVec3::new(*x, *y, 0.0) + c * DVec3::new((a0 + 0.5 * PI).cos(), (a0 + 0.5 * PI).sin(), 0.0);
            let q = DVec3::new(nx, ny, 0.0) + c * DVec3::new(na.cos(), na.sin(), 0.0);
            segs.push(Segment::line(p, q));
        }
        let region = Region { plane: Frame::WORLD, outer: Loop3 { segments: segs }, holes: vec![] }.normalized();
        extrude_region(&region, DVec3::Z * h).unwrap()
    }

    #[test]
    fn chains_the_top_edges_of_a_rounded_slab() {
        let b = rounded_slab(3.0, 2.0, 0.8, 1.5);
        let top: Vec<EdgeId> = b.edge_ids().filter(|e| {
            let ed = b.edge(*e);
            ed.start().z > 1.4 && ed.end().z > 1.4
        }).collect();
        assert_eq!(top.len(), 8);
        let chains = tangent_chains(&b, &top);
        assert_eq!(chains.len(), 1);
        assert!(chains[0].1);
        assert_eq!(chains[0].0.len(), 8);
    }

    #[test]
    fn fillets_the_top_of_a_rounded_slab() {
        let (a, bb, c, h, r) = (3.0, 2.0, 0.8, 1.5, 0.3);
        let b = rounded_slab(a, bb, c, h);
        let top: Vec<EdgeId> = b.edge_ids().filter(|e| {
            let ed = b.edge(*e);
            ed.start().z > 1.4 && ed.end().z > 1.4
        }).collect();
        let chains = tangent_chains(&b, &top);
        let out = compact(&roll_chain(&b, &chains[0].0, true, Blend::Fillet(r)).unwrap());
        assert_eq!(open_edges(&out), 0);
        // Removed: the spandrel (r^2 (1 - pi/4)) swept along the contact
        // path, by Pappus around the corners: straight runs plus corner arcs
        // at radius c - r + centroid offset.
        let spandrel = r * r * (1.0 - PI / 4.0);
        let straight = 2.0 * (2.0 * (a - c)) + 2.0 * (2.0 * (bb - c));
        // Centroid of the spandrel measured from the side face: r(10 - 3pi)/(12 - 3pi).
        let d = r * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI);
        let arcs = 2.0 * PI * (c - d);
        let expect = spandrel * (straight + arcs);
        let removed = vol(&b) - vol(&out);
        assert!((removed - expect).abs() < 2e-3 * expect, "{removed} vs {expect}");
    }

    #[test]
    fn fillets_a_side_edge_of_a_spline_prism() {
        // A D-shaped profile: a spline arch closed by a straight base, so
        // the two side edges run between a plane and a spline face and end
        // on the caps.
        let pts = [DVec3::new(-2.0, 0.0, 0.0), DVec3::new(-1.5, 1.5, 0.0), DVec3::new(0.0, 2.2, 0.0), DVec3::new(1.5, 1.5, 0.0), DVec3::new(2.0, 0.0, 0.0)];
        let arch = NurbsCurve::interpolate(&pts, 3);
        let (t0, t1) = arch.domain();
        let segs = vec![Segment::new(Curve3::Nurbs(arch), t0, t1), Segment::line(pts[4], pts[0])];
        let region = Region { plane: Frame::WORLD, outer: Loop3 { segments: segs }, holes: vec![] }.normalized();
        let b = extrude_region(&region, DVec3::Z * 2.0).unwrap();
        let side = b.edge_ids().find(|e| {
            let ed = b.edge(*e);
            ed.start().distance(DVec3::new(2.0, 0.0, ed.start().z)) < 1e-9 && ed.end().distance(DVec3::new(2.0, 0.0, ed.end().z)) < 1e-9 && (ed.start().z - ed.end().z).abs() > 1.0
        }).unwrap();
        let chains = tangent_chains(&b, &[side]);
        let r = 0.25;
        let out = compact(&roll_chain(&b, &chains[0].0, chains[0].1, Blend::Fillet(r)).unwrap());
        assert_eq!(open_edges(&out), 0);
        let removed = vol(&b) - vol(&out);
        // A corner spandrel for the corner's angle, along the height.
        assert!(removed > 0.0 && removed < 2.0 * r * r, "{removed}");
    }
}
