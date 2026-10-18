// SPDX-License-Identifier: GPL-3.0-only
//! Clean-up of modelling results: faces lying on one plane are merged,
//! edges continuing the same curve through a vertex are joined, and unused
//! entities are dropped.

use crate::topo::{Body, Coedge, EdgeId, FaceId, Loop, LoopId, VertexId};
use ductile_geom::poly2::{point_in_polygon, signed_area};
use ductile_geom::{Curve3, DVec2, DVec3, Surface};
use std::collections::HashMap;
use std::f64::consts::TAU;

/// Merges co-planar faces and continuing edges, then compacts the body.
pub fn simplify(body: &Body) -> Body {
    let tol = ductile_geom::LINEAR_TOL * (body.bbox().diagonal() / 10.0).max(0.01);
    let mut b = merge_planar_faces(body, tol);
    while merge_one_vertex(&mut b, tol) {}
    compact(&b)
}

/// Removes vertices, edges and loops that no face references.
pub fn compact(b: &Body) -> Body {
    let mut out = Body::new(b.kind);
    let mut vmap: HashMap<VertexId, VertexId> = HashMap::new();
    let mut emap: HashMap<EdgeId, EdgeId> = HashMap::new();
    for f in &b.faces {
        let mut loops = Vec::new();
        for l in &f.loops {
            let mut coedges = Vec::new();
            for c in &b.loops[l.idx()].coedges {
                let ne = *emap.entry(c.edge).or_insert_with(|| {
                    let e = b.edge(c.edge);
                    let v0 = *vmap.entry(e.v0).or_insert_with(|| out.add_vertex(b.vertex(e.v0)));
                    let v1 = *vmap.entry(e.v1).or_insert_with(|| out.add_vertex(b.vertex(e.v1)));
                    out.add_edge(e.curve.clone(), e.t0, e.t1, v0, v1)
                });
                coedges.push(Coedge::new(ne, c.reversed));
            }
            loops.push(coedges);
        }
        out.add_face(f.surface.clone(), f.reversed, loops);
    }
    out
}

/// Outward plane (normal, offset) of a planar face.
fn outward_plane(b: &Body, f: FaceId) -> Option<(DVec3, f64, DVec3)> {
    let face = b.face(f);
    let Surface::Plane { frame } = &face.surface else { return None };
    let n = if face.reversed { -frame.z } else { frame.z };
    Some((n, n.dot(frame.origin), frame.x))
}

fn find(parent: &mut [usize], i: usize) -> usize {
    let mut r = i;
    while parent[r] != r {
        r = parent[r];
    }
    let mut j = i;
    while parent[j] != r {
        let n = parent[j];
        parent[j] = r;
        j = n;
    }
    r
}

/// Merges adjacent planar faces that lie on the same oriented plane.
fn merge_planar_faces(b: &Body, tol: f64) -> Body {
    let nf = b.faces.len();
    let planes: Vec<Option<(DVec3, f64, DVec3)>> = b.face_ids().map(|f| outward_plane(b, f)).collect();
    let mut parent: Vec<usize> = (0..nf).collect();
    let mut users: HashMap<EdgeId, Vec<FaceId>> = HashMap::new();
    for f in b.face_ids() {
        for l in b.face_loops(f) {
            for c in &l.coedges {
                users.entry(c.edge).or_default().push(f);
            }
        }
    }
    for fs in users.values() {
        if fs.len() != 2 || fs[0] == fs[1] {
            continue;
        }
        let (Some(p), Some(q)) = (planes[fs[0].idx()], planes[fs[1].idx()]) else { continue };
        if p.0.dot(q.0) > 1.0 - 1e-10 && (p.1 - q.1).abs() < tol * 10.0 {
            let (a, c) = (find(&mut parent, fs[0].idx()), find(&mut parent, fs[1].idx()));
            parent[a] = c;
        }
    }
    let mut groups: HashMap<usize, Vec<FaceId>> = HashMap::new();
    for f in b.face_ids() {
        groups.entry(find(&mut parent, f.idx())).or_default().push(f);
    }
    if groups.values().all(|g| g.len() == 1) {
        return b.clone();
    }
    let mut out = Body { kind: b.kind, vertices: b.vertices.clone(), edges: b.edges.clone(), loops: vec![], faces: vec![] };
    let mut order: Vec<&Vec<FaceId>> = groups.values().collect();
    order.sort_by_key(|g| g[0]);
    for g in order {
        let first = b.face(g[0]);
        let merged = if g.len() > 1 { merge_group(b, g, planes[g[0].idx()].unwrap()) } else { None };
        match merged {
            Some(faces) => {
                for loops in faces {
                    out.add_face(first.surface.clone(), first.reversed, loops);
                }
            }
            None => {
                for &f in g {
                    let face = b.face(f);
                    out.add_face(face.surface.clone(), face.reversed, b.face_loops(f).map(|l| l.coedges.clone()).collect());
                }
            }
        }
    }
    out
}

/// Loops of the union of a group of co-planar faces.
fn merge_group(b: &Body, g: &[FaceId], plane: (DVec3, f64, DVec3)) -> Option<Vec<Vec<Vec<Coedge>>>> {
    let (n, _, x) = plane;
    let x = (x - n * x.dot(n)).normalize();
    let y = n.cross(x);
    let to2 = |p: DVec3| DVec2::new(p.dot(x), p.dot(y));
    let mut count: HashMap<EdgeId, usize> = HashMap::new();
    let mut all: Vec<Coedge> = Vec::new();
    for &f in g {
        for l in b.face_loops(f) {
            for c in &l.coedges {
                *count.entry(c.edge).or_default() += 1;
                all.push(*c);
            }
        }
    }
    let bnd: Vec<Coedge> = all.into_iter().filter(|c| count[&c.edge] == 1).collect();
    if bnd.is_empty() {
        return None;
    }
    // Direction (in the plane) leaving the start and arriving at the end of each coedge.
    let dirs: Vec<(f64, f64)> = bnd
        .iter()
        .map(|c| {
            let e = b.edge(c.edge);
            let (ts, te, s) = if c.reversed { (e.t1, e.t0, -1.0) } else { (e.t0, e.t1, 1.0) };
            let a = to2(e.curve.tangent(ts) * s);
            let r = to2(-e.curve.tangent(te) * s);
            (a.y.atan2(a.x), r.y.atan2(r.x))
        })
        .collect();
    let mut starting: HashMap<VertexId, Vec<usize>> = HashMap::new();
    for (i, c) in bnd.iter().enumerate() {
        starting.entry(b.coedge_vertices(*c).0).or_default().push(i);
    }
    let mut used = vec![false; bnd.len()];
    let mut cycles: Vec<Vec<usize>> = Vec::new();
    for s in 0..bnd.len() {
        if used[s] {
            continue;
        }
        let mut cyc = vec![];
        let mut i = s;
        loop {
            used[i] = true;
            cyc.push(i);
            let end = b.coedge_vertices(bnd[i]).1;
            let r = dirs[i].1;
            let next = starting
                .get(&end)?
                .iter()
                .copied()
                .filter(|&o| !used[o] || o == s)
                .map(|o| {
                    let mut cw = (r - dirs[o].0).rem_euclid(TAU);
                    if cw < 1e-9 {
                        cw += TAU;
                    }
                    (cw, o)
                })
                .min_by(|a, b| a.0.total_cmp(&b.0))?
                .1;
            if next == s {
                break;
            }
            i = next;
        }
        cycles.push(cyc);
    }
    // Outer loops and holes by signed area about the outward normal.
    let poly = |cyc: &[usize]| -> Vec<DVec2> {
        let mut pts = Vec::new();
        for &i in cyc {
            let e = b.edge(bnd[i].edge);
            let mut s: Vec<DVec3> = e.curve.tessellate(e.t0, e.t1, 1e-3 * e.length().max(1e-9)).into_iter().map(|p| p.1).collect();
            if bnd[i].reversed {
                s.reverse();
            }
            s.pop();
            pts.extend(s.into_iter().map(to2));
        }
        pts
    };
    let mut outers: Vec<(f64, Vec<DVec2>, Vec<Coedge>)> = Vec::new();
    let mut holes: Vec<(f64, Vec<DVec2>, Vec<Coedge>)> = Vec::new();
    for cyc in cycles {
        let p = poly(&cyc);
        let a = signed_area(&p);
        let co: Vec<Coedge> = cyc.iter().map(|&i| bnd[i]).collect();
        if a > 0.0 {
            outers.push((a, p, co));
        } else {
            holes.push((a, p, co));
        }
    }
    if outers.is_empty() {
        return None;
    }
    let mut faces: Vec<Vec<Vec<Coedge>>> = outers.iter().map(|o| vec![o.2.clone()]).collect();
    for (a, p, co) in holes {
        let probe = 0.5 * (p[0] + p[1 % p.len()]);
        let owner = outers
            .iter()
            .enumerate()
            .filter(|(_, o)| o.0 > -a && point_in_polygon(probe, &o.1))
            .min_by(|l, r| l.1 .0.total_cmp(&r.1 .0))?
            .0;
        faces[owner].push(co);
    }
    Some(faces)
}

/// True if two edges lie on the same curve.
fn same_curve(a: &Curve3, b: &Curve3, tol: f64) -> bool {
    match (a, b) {
        (Curve3::Line { origin: o1, dir: d1 }, Curve3::Line { origin: o2, dir: d2 }) => {
            d1.cross(*d2).length() < 1e-10 && (*o2 - *o1).cross(*d1).length() < tol
        }
        (Curve3::Circle { frame: f1, radius: r1 }, Curve3::Circle { frame: f2, radius: r2 }) => {
            f1.origin.distance(f2.origin) < tol && f1.z.dot(f2.z) > 1.0 - 1e-10 && f1.x.distance(f2.x) < 1e-9 && (r1 - r2).abs() < tol
        }
        _ => a == b,
    }
}

/// Joins the two edges meeting at one vertex of degree two when they
/// continue the same curve. Returns true if something changed.
fn merge_one_vertex(b: &mut Body, tol: f64) -> bool {
    let mut incident: HashMap<VertexId, Vec<EdgeId>> = HashMap::new();
    let mut used: HashMap<EdgeId, usize> = HashMap::new();
    for l in &b.loops {
        for c in &l.coedges {
            *used.entry(c.edge).or_default() += 1;
        }
    }
    for e in b.edge_ids() {
        if !used.contains_key(&e) {
            continue;
        }
        let ed = b.edge(e);
        incident.entry(ed.v0).or_default().push(e);
        incident.entry(ed.v1).or_default().push(e);
    }
    let live: Vec<LoopId> = b.faces.iter().flat_map(|f| f.loops.iter().copied()).collect();
    let mut keys: Vec<&VertexId> = incident.keys().collect();
    keys.sort();
    for &v in keys {
        let es = &incident[&v];
        if es.len() != 2 || es[0] == es[1] {
            continue;
        }
        let (e1, e2) = (es[0], es[1]);
        let (a, c) = (b.edge(e1).clone(), b.edge(e2).clone());
        if !same_curve(&a.curve, &c.curve, tol) {
            continue;
        }
        // Orient so that e1 ends at v and e2 starts there.
        let (first, second, flip_first) = if a.v1 == v && c.v0 == v {
            (e1, e2, false)
        } else if c.v1 == v && a.v0 == v {
            (e2, e1, false)
        } else {
            (e1, e2, true)
        };
        if flip_first {
            // Opposite parameter directions; skip (rare after booleans).
            continue;
        }
        let (f, s) = (b.edge(first).clone(), b.edge(second).clone());
        // Parameter of the second edge expressed on the first edge's curve.
        let (s0, s1) = match &f.curve {
            Curve3::Line { origin, dir } => ((s.start() - *origin).dot(*dir), (s.end() - *origin).dot(*dir)),
            Curve3::Circle { .. } => {
                let mut s0 = f.curve.closest_param(s.start());
                let p = TAU;
                s0 += ((f.t1 - s0) / p).round() * p;
                (s0, s0 + (s.t1 - s.t0))
            }
            _ => (s.t0, s.t1),
        };
        if (s0 - f.t1).abs() > 1e-7 * (1.0 + f.t1.abs()) || s1 <= f.t1 {
            continue;
        }
        if f.curve.period().map(|p| s1 - f.t0 > p + 1e-9).unwrap_or(false) {
            continue;
        }
        // Every loop must use the pair consecutively and consistently.
        let mut ok = true;
        for &lid in &live {
            let cs = &b.loops[lid.idx()].coedges;
            let n = cs.len();
            for (i, co) in cs.iter().enumerate() {
                if co.edge == first {
                    let nb = if co.reversed { cs[(i + n - 1) % n] } else { cs[(i + 1) % n] };
                    if nb.edge != second || nb.reversed != co.reversed {
                        ok = false;
                    }
                }
            }
        }
        if !ok {
            continue;
        }
        {
            let ed = &mut b.edges[first.idx()];
            ed.t1 = s1;
            ed.v1 = s.v1;
        }
        for &lid in &live {
            b.loops[lid.idx()].coedges.retain(|co| co.edge != second);
        }
        return true;
    }
    false
}

#[allow(dead_code)]
fn loops_of(b: &Body) -> &[Loop] {
    &b.loops
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ops::boolean::{boolean, open_edges, BoolOp};
    use crate::primitives::*;
    use crate::tess::{tessellate, TessOptions};
    use ductile_geom::Frame;

    fn bx(o: DVec3, s: DVec3) -> Body {
        make_box(&Frame { origin: o, ..Frame::WORLD }, s)
    }

    #[test]
    fn union_of_flush_boxes_is_one_box() {
        let a = bx(DVec3::ZERO, DVec3::new(1.0, 1.0, 1.0));
        let b = bx(DVec3::new(1.0, 0.0, 0.0), DVec3::new(1.0, 1.0, 1.0));
        let u = boolean(&a, &b, BoolOp::Union).unwrap();
        let s = simplify(&u);
        assert_eq!(s.faces.len(), 6, "faces");
        assert_eq!(s.edges.len(), 12, "edges");
        assert_eq!(s.vertices.len(), 8, "vertices");
        assert_eq!(open_edges(&s), 0);
        let v = tessellate(&s, &TessOptions::default()).volume();
        assert!((v - 2.0).abs() < 1e-9);
    }

    #[test]
    fn l_shaped_union_merges_caps() {
        let a = bx(DVec3::ZERO, DVec3::new(2.0, 2.0, 1.0));
        let b = bx(DVec3::new(1.0, 1.0, 0.0), DVec3::new(2.0, 2.0, 1.0));
        let s = simplify(&boolean(&a, &b, BoolOp::Union).unwrap());
        assert_eq!(s.faces.len(), 10);
        assert_eq!(open_edges(&s), 0);
        let v = tessellate(&s, &TessOptions::default()).volume();
        assert!((v - 7.0).abs() < 1e-9);
    }
}
