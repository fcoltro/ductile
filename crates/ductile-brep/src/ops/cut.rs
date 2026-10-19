// SPDX-License-Identifier: GPL-3.0-only
//! Cutting solids apart along curves, and splitting bodies into their
//! connected pieces.

use super::boolean::{boolean, BoolError, BoolOp};
use super::sheet::Sewer;
use crate::curves::{extend_segment, Extension};
use crate::ops::extrude::extrude_region;
use crate::profile::{loops_plane, Loop3, Region, Segment};
use crate::tess::{tessellate, TessOptions};
use crate::topo::{Body, BodyKind};
use ductile_geom::{DMat4, DVec3, Frame};

/// The connected pieces of `b`, each its own body. An inner void (a shell
/// facing inward) stays with the piece around it.
pub fn split_shells(b: &Body) -> Vec<Body> {
    let n = b.faces.len();
    if n == 0 {
        return Vec::new();
    }
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(p: &mut [usize], mut i: usize) -> usize {
        while p[i] != i {
            p[i] = p[p[i]];
            i = p[i];
        }
        i
    }
    for e in b.edge_ids() {
        let fs = b.edge_faces(e);
        for w in fs.windows(2) {
            let (x, y) = (find(&mut parent, w[0].0.idx()), find(&mut parent, w[1].0.idx()));
            parent[x] = y;
        }
    }
    let mut groups: Vec<Vec<usize>> = Vec::new();
    let mut root_of: Vec<Option<usize>> = vec![None; n];
    for f in 0..n {
        let r = find(&mut parent, f);
        let g = *root_of[r].get_or_insert_with(|| {
            groups.push(Vec::new());
            groups.len() - 1
        });
        groups[g].push(f);
    }
    if groups.len() == 1 {
        return vec![b.clone()];
    }
    let piece = |faces: &[usize]| {
        let mut one = b.clone();
        one.faces = faces.iter().map(|f| b.faces[*f].clone()).collect();
        super::heal::compact(&one)
    };
    let pieces: Vec<Body> = groups.iter().map(|g| piece(g)).collect();
    if b.kind != BodyKind::Solid {
        return pieces;
    }
    // Voids join the smallest outward piece whose box holds them.
    let opts = TessOptions::for_size(b.bbox().diagonal());
    let vols: Vec<f64> = pieces.iter().map(|p| tessellate(p, &opts).volume()).collect();
    let mut out: Vec<Body> = Vec::new();
    let mut boxes = Vec::new();
    for (p, v) in pieces.iter().zip(&vols) {
        if *v >= 0.0 {
            boxes.push(p.bbox());
            out.push(p.clone());
        }
    }
    for (p, v) in pieces.iter().zip(&vols) {
        if *v < 0.0 {
            let bb = p.bbox();
            let host = (0..out.len()).filter(|i| boxes[*i].contains(bb.min) && boxes[*i].contains(bb.max)).min_by(|x, y| boxes[*x].diagonal().total_cmp(&boxes[*y].diagonal()));
            match host {
                Some(i) => out[i].append(p),
                None => out.push(p.clone()),
            }
        }
    }
    out
}

/// Cuts `solid` along each curve chain, swept straight through it along
/// `dir`: a closed chain cuts out the part inside it, an open chain (lying
/// across the solid in a plane square to `dir`) splits it in two, its ends
/// extended straight past the solid. Returns the pieces.
pub fn cut_with_curves(solid: &Body, chains: &[Loop3], dir: DVec3) -> Result<Vec<Body>, BoolError> {
    let Some(d) = dir.try_normalize() else { return Ok(vec![solid.clone()]) };
    let bb = solid.bbox();
    let mut pieces = vec![solid.clone()];
    for c in chains {
        if c.segments.is_empty() {
            continue;
        }
        let reach = 2.0 * (bb.diagonal() + c.segments[0].start().distance(bb.center()));
        let Some(prism) = cutter(c, d, reach) else { continue };
        let mut next = Vec::new();
        for p in &pieces {
            next.extend(split_by(p, &prism)?);
        }
        pieces = next;
    }
    Ok(pieces)
}

/// Cuts `solid` along `sheet`, which should reach right across it: returns
/// the pieces in front of and behind the sheet.
pub fn cut_with_sheet(solid: &Body, sheet: &Body) -> Result<Vec<Body>, BoolError> {
    match sheet_cutter(solid, sheet) {
        Some(prism) => split_by(solid, &prism),
        None => Ok(vec![solid.clone()]),
    }
}

/// Splits the faces of `solid` where `sheet` crosses them, like
/// [`imprint_curves`], leaving the solid in one piece.
pub fn imprint_sheet(solid: &Body, sheet: &Body) -> Result<Body, BoolError> {
    match sheet_cutter(solid, sheet) {
        Some(prism) => imprint_by(solid, &prism),
        None => Ok(solid.clone()),
    }
}

/// The solid swept from `sheet` far out in front of it, past `solid`.
fn sheet_cutter(solid: &Body, sheet: &Body) -> Option<Body> {
    let mesh = tessellate(sheet, &TessOptions::for_size(sheet.bbox().diagonal()));
    let mut n = DVec3::ZERO;
    for f in &mesh.faces {
        for t in &f.triangles {
            let [a, b, c] = t.map(|i| f.positions[i as usize]);
            n += (b - a).cross(c - a);
        }
    }
    let n = n.try_normalize()?;
    let (sb, cb) = (solid.bbox(), sheet.bbox());
    let reach = 2.0 * (sb.diagonal() + cb.diagonal() + sb.center().distance(cb.center()));
    super::sheet::sweep_sheet(sheet, n * reach)
}

/// The pieces of `p` inside and outside `prism`, each split into its
/// connected parts.
fn split_by(p: &Body, prism: &Body) -> Result<Vec<Body>, BoolError> {
    let mut out = Vec::new();
    for op in [BoolOp::Intersection, BoolOp::Difference] {
        match boolean(p, prism, op) {
            Ok(r) => out.extend(split_shells(&r)),
            // The cutter missed this piece, or took all of it.
            Err(BoolError::Empty) => {}
            Err(e) => return Err(e),
        }
    }
    Ok(out)
}

/// Imprints each curve chain onto the faces of `solid`, swept straight
/// through it along `dir` like [`cut_with_curves`], but leaves the solid
/// in one piece: the faces it crosses are split along the curve.
pub fn imprint_curves(solid: &Body, chains: &[Loop3], dir: DVec3) -> Result<Body, BoolError> {
    let Some(d) = dir.try_normalize() else { return Ok(solid.clone()) };
    let bb = solid.bbox();
    let mut body = solid.clone();
    for c in chains {
        if c.segments.is_empty() {
            continue;
        }
        let reach = 2.0 * (bb.diagonal() + c.segments[0].start().distance(bb.center()));
        let Some(prism) = cutter(c, d, reach) else { continue };
        body = imprint_by(&body, &prism)?;
    }
    Ok(body)
}

/// Knife: splits only the faces the curve chains lie on, along them. The
/// chains are imprinted straight through along `dir`, then the cuts that
/// landed on other faces are merged away again.
pub fn knife(solid: &Body, chains: &[Loop3], dir: DVec3) -> Result<Body, BoolError> {
    let cut = imprint_curves(solid, chains, dir)?;
    let size = solid.bbox().diagonal().max(1e-9);
    let tol = 1e-5 * size;
    let polys: Vec<Vec<DVec3>> = chains
        .iter()
        .map(|c| {
            let mut p = c.polygon(tol * 0.1);
            if let Some(s) = c.segments.last() {
                p.push(s.end());
            }
            p
        })
        .collect();
    let old: Vec<Vec<DVec3>> = solid.edges.iter().map(|e| e.curve.tessellate(e.t0, e.t1, tol * 0.1).into_iter().map(|x| x.1).collect()).collect();
    let near = |p: DVec3, lines: &[Vec<DVec3>]| {
        lines.iter().any(|l| {
            l.windows(2).any(|w| {
                let (a, b) = (w[0], w[1]);
                let t = ((p - a).dot(b - a) / (b - a).length_squared().max(1e-300)).clamp(0.0, 1.0);
                p.distance(a + (b - a) * t) < tol * 10.0
            })
        })
    };
    // Merge back across cuts that are neither the knife's nor old edges.
    let across = |b: &Body, e: crate::topo::EdgeId| {
        let ed = b.edge(e);
        let pts = [0.3, 0.5, 0.7].map(|f| ed.curve.point(ed.t0 + (ed.t1 - ed.t0) * f));
        !pts.iter().all(|p| near(*p, &polys)) && !pts.iter().all(|p| near(*p, &old))
    };
    Ok(super::heal::simplify_across(&cut, &across))
}

/// Offset edge: new edges at distance `d` from each of `edges`, inside
/// the planar faces on either side, imprinted into them. Open offsets run
/// on (along their end tangents) to the edges of the face.
pub fn offset_edges(solid: &Body, edges: &[crate::topo::EdgeId], d: f64) -> Result<Body, BoolError> {
    use crate::curves::{extend_segment, offset_chain, Extension};
    let size = solid.bbox().diagonal().max(1e-9);
    let mut body = solid.clone();
    for &e in edges {
        let ed = solid.edge(e);
        for (f, c) in solid.edge_faces(e) {
            let face = solid.face(f);
            let ductile_geom::Surface::Plane { frame } = &face.surface else { continue };
            let n = if face.reversed { -frame.z } else { frame.z };
            // Run the edge the way the face's loop does: the face is on its left.
            let seg = Segment { curve: ed.curve.clone(), t0: ed.t0, t1: ed.t1, reversed: c.reversed };
            let closed = seg.is_closed();
            let mid = seg.curve.point(0.5 * (ed.t0 + ed.t1));
            let inward = n.cross(seg.tangent_at(0.5 * (ed.t0 + ed.t1))).normalize_or_zero();
            let mut off = Vec::new();
            for s in [d, -d] {
                let o = offset_chain(std::slice::from_ref(&seg), closed, s, n);
                if let Some(first) = o.first() {
                    let q = first.curve.point(0.5 * (first.t0 + first.t1));
                    if (q - mid).dot(inward) > 0.0 {
                        off = o;
                        break;
                    }
                }
            }
            if off.is_empty() {
                continue;
            }
            if !closed {
                let last = off.len() - 1;
                let mut ext = extend_segment(&off[0], false, 2.0 * size, Extension::Linear);
                ext.extend(off[1..last.max(1)].iter().cloned());
                if last > 0 {
                    ext.extend(extend_segment(&off[last], true, 2.0 * size, Extension::Linear));
                } else {
                    let tail = extend_segment(ext.last().unwrap(), true, 2.0 * size, Extension::Linear);
                    ext.pop();
                    ext.extend(tail);
                }
                off = ext;
            }
            body = knife(&body, &[Loop3 { segments: off }], n)?;
        }
    }
    Ok(body)
}

/// Boolean region mode: every closed region the solids enclose together,
/// each its own solid (where two overlap: the overlap, and each one's part
/// outside the other), then split further by any sheets reaching across.
pub fn regions(solids: &[Body], sheets: &[Body]) -> Result<Vec<Body>, BoolError> {
    let solid = |b: Body| if b.faces.is_empty() { Vec::new() } else { split_shells(&b) };
    // An empty overlap or remainder is no region, not a failure.
    let boolean = |a: &Body, b: &Body, op| match boolean(a, b, op) {
        Err(BoolError::Empty) => Ok(Body::new(BodyKind::Solid)),
        r => r,
    };
    let mut it = solids.iter();
    let Some(first) = it.next() else { return Ok(Vec::new()) };
    let mut pieces = solid(first.clone());
    for b in it {
        let mut next = Vec::new();
        let mut outside = b.clone();
        for p in &pieces {
            next.extend(solid(boolean(p, b, BoolOp::Intersection)?));
            next.extend(solid(boolean(p, b, BoolOp::Difference)?));
            if !outside.faces.is_empty() {
                outside = boolean(&outside, p, BoolOp::Difference)?;
            }
        }
        next.extend(solid(outside));
        pieces = next;
    }
    for sh in sheets {
        pieces = pieces.into_iter().flat_map(|p| match cut_with_sheet(&p, sh) {
            Ok(v) if !v.is_empty() => v,
            _ => vec![p],
        }).collect();
    }
    Ok(pieces)
}

/// Complete edge: runs each straight edge on from its ends, across the
/// planar face it points into (a face in the same plane, beyond the edge's
/// end vertex), up to the first edge it meets there, and splits that face
/// along the extension. Finishes partial splits of a face.
pub fn complete_edges(solid: &Body, edges: &[crate::topo::EdgeId]) -> Result<Body, BoolError> {
    let size = solid.bbox().diagonal().max(1e-9);
    let tol = 1e-7 * size;
    // Rays to run: from an end vertex, outward along the edge.
    let mut rays = Vec::new();
    for &e in edges {
        let ed = solid.edge(e);
        let (a, b) = (ed.start(), ed.end());
        let chord = b - a;
        if chord.length() < tol || ed.mid().distance(a + chord * 0.5) > tol * 10.0 {
            continue;
        }
        let d = chord.normalize();
        rays.push((b, d));
        rays.push((a, -d));
    }
    let mut body = solid.clone();
    for (p, d) in rays {
        let mut cut = None;
        for f in body.face_ids() {
            let face = body.face(f);
            let ductile_geom::Surface::Plane { frame } = &face.surface else { continue };
            if frame.z.dot(d).abs() > 1e-6 || frame.z.dot(p - frame.origin).abs() > tol * 10.0 {
                continue;
            }
            // The face's boundary, flattened into its plane.
            let flat = |q: DVec3| {
                let l = frame.to_local(q);
                ductile_geom::DVec2::new(l.x, l.y)
            };
            let mut segs = Vec::new();
            for lp in body.face_loops(f) {
                for c in &lp.coedges {
                    let ed = body.edge(c.edge);
                    let pts = ed.curve.tessellate(ed.t0, ed.t1, tol);
                    segs.extend(pts.windows(2).map(|w| (flat(w[0].1), flat(w[1].1))));
                }
            }
            let (p2, d2) = (flat(p), {
                let l = frame.to_local(p + d) - frame.to_local(p);
                ductile_geom::DVec2::new(l.x, l.y)
            });
            if !segs.iter().any(|(a, b)| point_seg_dist(p2, *a, *b) < tol * 10.0) {
                continue;
            }
            // Ray against the boundary: the nearest crossing ahead.
            let hits: Vec<f64> = segs
                .iter()
                .filter_map(|(a, b)| {
                    let e = *b - *a;
                    let den = d2.perp_dot(e);
                    if den.abs() < 1e-14 {
                        return None;
                    }
                    let w = *a - p2;
                    let t = w.perp_dot(e) / den;
                    let u = w.perp_dot(d2) / den;
                    (t > tol * 100.0 && (-1e-9..=1.0 + 1e-9).contains(&u)).then_some(t)
                })
                .collect();
            let Some(t) = hits.iter().copied().reduce(f64::min) else { continue };
            // The run must go through the face, not along its outside.
            let m = p2 + d2 * (0.5 * t);
            let crossings = segs.iter().filter(|(a, b)| (a.y > m.y) != (b.y > m.y) && m.x < a.x + (m.y - a.y) / (b.y - a.y) * (b.x - a.x)).count();
            if crossings % 2 == 1 {
                cut = Some((p + d * t, if face.reversed { -frame.z } else { frame.z }));
                break;
            }
        }
        if let Some((q, n)) = cut {
            body = knife(&body, &[Loop3 { segments: vec![Segment::line(p, q)] }], n)?;
        }
    }
    Ok(body)
}

fn point_seg_dist(p: ductile_geom::DVec2, a: ductile_geom::DVec2, b: ductile_geom::DVec2) -> f64 {
    let e = b - a;
    let t = ((p - a).dot(e) / e.length_squared().max(1e-300)).clamp(0.0, 1.0);
    p.distance(a + e * t)
}

/// Projects each curve chain onto the faces of `solid` along `dir` (both
/// ways), as a wire body of the curves it traces on them.
pub fn project_curves(solid: &Body, chains: &[Loop3], dir: DVec3) -> Result<Body, BoolError> {
    let Some(d) = dir.try_normalize() else { return Ok(Body::new(BodyKind::Wire)) };
    let bb = solid.bbox();
    let mut segs = Vec::new();
    for c in chains {
        if c.segments.is_empty() {
            continue;
        }
        let reach = 2.0 * (bb.diagonal() + c.segments[0].start().distance(bb.center()));
        let Some(prism) = cutter(c, d, reach) else { continue };
        let w = intersection_curves(solid, &prism)?;
        // An open chain was extended past its ends to build the prism:
        // keep only the curves traced by the chain itself.
        let flat = |p: DVec3| p - d * p.dot(d);
        let tol = 1e-7 * bb.diagonal().max(1.0);
        let mut poly: Vec<DVec3> = c.polygon(tol);
        if let Some(last) = c.segments.last() {
            poly.push(last.end());
        }
        let poly: Vec<DVec3> = poly.into_iter().map(flat).collect();
        let near = |p: DVec3| poly.windows(2).any(|s| {
            let (a, b) = (s[0], s[1]);
            let t = ((p - a).dot(b - a) / (b - a).length_squared().max(1e-30)).clamp(0.0, 1.0);
            p.distance(a + (b - a) * t) < tol * 10.0
        });
        for e in &w.edges {
            if c.is_closed() {
                segs.push(Segment { curve: e.curve.clone(), t0: e.t0, t1: e.t1, reversed: false });
                continue;
            }
            // The stretches of the edge that lie over the chain, their ends
            // found by bisection.
            let inside = |t: f64| near(flat(e.curve.point(t)));
            let n = 64;
            let ts: Vec<f64> = (0..=n).map(|k| e.t0 + (e.t1 - e.t0) * k as f64 / n as f64).collect();
            let edge_of = |mut a: f64, mut b: f64| {
                // inside(a) != inside(b); returns the switch point.
                let ia = inside(a);
                for _ in 0..40 {
                    let m = 0.5 * (a + b);
                    if inside(m) == ia {
                        a = m;
                    } else {
                        b = m;
                    }
                }
                0.5 * (a + b)
            };
            let mut start: Option<f64> = if inside(ts[0]) { Some(ts[0]) } else { None };
            for k in 1..=n {
                let (was, is) = (inside(ts[k - 1]), inside(ts[k]));
                if was != is {
                    let t = edge_of(ts[k - 1], ts[k]);
                    if is {
                        start = Some(t);
                    } else if let Some(s0) = start.take() {
                        segs.push(Segment { curve: e.curve.clone(), t0: s0, t1: t, reversed: false });
                    }
                }
            }
            if let Some(s0) = start {
                if e.t1 - s0 > 1e-12 {
                    segs.push(Segment { curve: e.curve.clone(), t0: s0, t1: e.t1, reversed: false });
                }
            }
        }
    }
    Ok(crate::profile::wire_from_segments(&segs))
}

/// The curves where the surfaces of solids `a` and `b` cross, as a wire
/// body: the edges of their common volume that run between a face of `a`
/// and a face of `b`. Empty when the solids do not overlap.
pub fn intersection_curves(a: &Body, b: &Body) -> Result<Body, BoolError> {
    let common = match boolean(a, b, BoolOp::Intersection) {
        Ok(r) => r,
        Err(BoolError::Empty) => return Ok(Body::new(BodyKind::Wire)),
        Err(e) => return Err(e),
    };
    let on_a: Vec<_> = a.faces.iter().map(|f| &f.surface).collect();
    let on_b: Vec<_> = b.faces.iter().map(|f| &f.surface).collect();
    let mut segs = Vec::new();
    for e in common.edge_ids() {
        let fs = common.edge_faces(e);
        if fs.len() != 2 {
            continue;
        }
        let (s, t) = (&common.face(fs[0].0).surface, &common.face(fs[1].0).surface);
        let (sa, sb, ta, tb) = (on_a.contains(&s), on_b.contains(&s), on_a.contains(&t), on_b.contains(&t));
        if (sa && !sb && tb && !ta) || (sb && !sa && ta && !tb) {
            let ed = common.edge(e);
            segs.push(Segment { curve: ed.curve.clone(), t0: ed.t0, t1: ed.t1, reversed: false });
        }
    }
    Ok(crate::profile::wire_from_segments(&segs))
}

/// `body` with its faces split where they cross the surface of `prism`:
/// the faces of both pieces, cut apart by `prism`, sewn back together
/// without the faces that came from `prism`.
fn imprint_by(body: &Body, prism: &Body) -> Result<Body, BoolError> {
    let tool: Vec<_> = prism.faces.iter().map(|f| f.surface.clone()).collect();
    let tol = 1e-6 * body.bbox().diagonal().max(1.0);
    let mut sewer = Sewer::new(body.kind, tol);
    let mut pieces = 0;
    for op in [BoolOp::Intersection, BoolOp::Difference] {
        let part = match boolean(body, prism, op) {
            Ok(r) => r,
            Err(BoolError::Empty) => continue,
            Err(e) => return Err(e),
        };
        pieces += 1;
        for f in part.face_ids() {
            let face = part.face(f);
            if tool.contains(&face.surface) {
                continue;
            }
            let loops: Vec<Vec<Segment>> = part
                .face_loops(f)
                .map(|l| {
                    l.coedges
                        .iter()
                        .map(|c| {
                            let e = part.edge(c.edge);
                            Segment { curve: e.curve.clone(), t0: e.t0, t1: e.t1, reversed: c.reversed }
                        })
                        .collect()
                })
                .collect();
            if let Some((outer, holes)) = loops.split_first() {
                sewer.face(face.surface.clone(), face.reversed, outer, holes);
            } else {
                sewer.body.add_face(face.surface.clone(), face.reversed, Vec::new());
            }
        }
    }
    // A cutter that missed the body leaves it as it was.
    Ok(if pieces == 2 { sewer.body } else { body.clone() })
}

/// The solid swept by the region a chain bounds (closed), or cuts off on
/// one side (open), from `reach` behind its plane to `reach` in front.
fn cutter(c: &Loop3, d: DVec3, reach: f64) -> Option<Body> {
    let back = DMat4::from_translation(-d * reach);
    let moved = |s: &Segment| Segment { curve: s.curve.transformed(&back), ..s.clone() };
    let lp = if c.is_closed() {
        c.clone()
    } else {
        // Extend both ends straight on, then close round one side.
        let n = c.segments.len();
        let mut segs = extend_segment(&c.segments[0], false, reach, Extension::Linear);
        if n > 1 {
            segs.extend(c.segments[1..n - 1].iter().cloned());
            segs.extend(extend_segment(&c.segments[n - 1], true, reach, Extension::Linear));
        } else {
            let first = segs.pop()?;
            segs.extend(extend_segment(&first, true, reach, Extension::Linear));
        }
        let (s, e) = (segs[0].start(), segs.last()?.end());
        let w = d.cross(e - s).try_normalize()? * (4.0 * reach);
        segs.push(Segment::line(e, e + w));
        segs.push(Segment::line(e + w, s + w));
        segs.push(Segment::line(s + w, s));
        Loop3 { segments: segs }
    };
    let lp = Loop3 { segments: lp.segments.iter().map(moved).collect() };
    let plane = loops_plane(std::slice::from_ref(&lp)).unwrap_or_else(|| Frame::from_normal(lp.segments[0].start(), d));
    let region = Region::from_loops(plane, vec![lp]).into_iter().next()?;
    let solid = extrude_region(&region, d * (2.0 * reach))?;
    Some(solid)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::make_box;
    use ductile_geom::Curve3;
    use std::f64::consts::PI;

    fn vol(b: &Body) -> f64 {
        tessellate(b, &TessOptions { chord_tol: 1e-4, angle_tol: 0.05 }).volume()
    }

    #[test]
    fn cuts_a_box_along_a_line_and_round_a_circle() {
        let b = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
        // A line across the top, cutting along z.
        let line = Loop3 { segments: vec![Segment::line(DVec3::new(1.0, 0.5, 5.0), DVec3::new(1.0, 2.5, 5.0))] };
        let mut parts = cut_with_curves(&b, &[line], DVec3::Z).unwrap();
        parts.sort_by(|x, y| vol(x).total_cmp(&vol(y)));
        assert_eq!(parts.len(), 2);
        assert!((vol(&parts[0]) - 6.0).abs() < 1e-9 && (vol(&parts[1]) - 18.0).abs() < 1e-9, "{} {}", vol(&parts[0]), vol(&parts[1]));
        // A circle: a round plug and the box with a hole.
        let circle = Loop3 { segments: vec![Segment::new(Curve3::Circle { frame: Frame::new(DVec3::new(2.0, 1.5, 0.0), DVec3::Z, DVec3::X), radius: 1.0 }, 0.0, 2.0 * PI)] };
        let mut parts = cut_with_curves(&b, &[circle], DVec3::Z).unwrap();
        parts.sort_by(|x, y| vol(x).total_cmp(&vol(y)));
        assert_eq!(parts.len(), 2);
        assert!((vol(&parts[0]) - 2.0 * PI).abs() < 1e-3 && (vol(&parts[1]) - (24.0 - 2.0 * PI)).abs() < 1e-3);
    }

    #[test]
    fn imprints_a_line_and_a_circle() {
        let b = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
        let line = Loop3 { segments: vec![Segment::line(DVec3::new(1.0, 0.5, 5.0), DVec3::new(1.0, 2.5, 5.0))] };
        let split = imprint_curves(&b, &[line], DVec3::Z).unwrap();
        // Top, bottom and both long sides split in two.
        assert_eq!(split.faces.len(), 10);
        assert_eq!(crate::ops::boolean::open_edges(&split), 0);
        assert!((vol(&split) - 24.0).abs() < 1e-9);
        let circle = Loop3 { segments: vec![Segment::new(Curve3::Circle { frame: Frame::new(DVec3::new(2.0, 1.5, 2.0), DVec3::Z, DVec3::X), radius: 1.0 }, 0.0, 2.0 * PI)] };
        let disc = imprint_curves(&b, &[circle], DVec3::Z).unwrap();
        // Top and bottom each gain a disc inside a ring.
        assert_eq!(disc.faces.len(), 8);
        assert_eq!(crate::ops::boolean::open_edges(&disc), 0);
        assert!((vol(&disc) - 24.0).abs() < 1e-6);
        // A wavy spline across the top.
        let n = ductile_geom::NurbsCurve::interpolate(&[DVec3::new(1.0, -0.5, 2.0), DVec3::new(1.8, 1.0, 2.0), DVec3::new(1.2, 2.0, 2.0), DVec3::new(2.0, 3.5, 2.0)], 3);
        let (t0, t1) = n.domain();
        let wave = Loop3 { segments: vec![Segment::new(Curve3::Nurbs(n), t0, t1)] };
        let waved = imprint_curves(&b, &[wave], DVec3::Z).unwrap();
        assert_eq!(waved.faces.len(), 10);
        assert_eq!(crate::ops::boolean::open_edges(&waved), 0);
        assert!((vol(&waved) - 24.0).abs() < 1e-6);
    }

    #[test]
    fn knife_splits_only_the_face_drawn_on() {
        let b = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
        let check = |k: &Body, faces: usize| {
            assert_eq!(k.faces.len(), faces);
            assert_eq!(crate::ops::boolean::open_edges(k), 0);
            assert!((vol(k) - 24.0).abs() < 1e-6, "{}", vol(k));
        };
        // Right across the top: just the top in two.
        let line = Loop3 { segments: vec![Segment::line(DVec3::new(1.0, 0.0, 2.0), DVec3::new(1.0, 3.0, 2.0))] };
        let k = knife(&b, &[line], DVec3::Z).unwrap();
        check(&k, 7);
        // The two top edges it meets are split there.
        assert_eq!(k.edges.len(), 15);
        // A circle on the top: a disc in a ring, the bottom untouched.
        let circle = Loop3 { segments: vec![Segment::new(Curve3::Circle { frame: Frame::new(DVec3::new(2.0, 1.5, 2.0), DVec3::Z, DVec3::X), radius: 1.0 }, 0.0, 2.0 * PI)] };
        check(&knife(&b, &[circle], DVec3::Z).unwrap(), 7);
        // Down a side face.
        let down = Loop3 { segments: vec![Segment::line(DVec3::new(4.0, 1.0, 0.0), DVec3::new(4.0, 1.0, 2.0))] };
        let k = knife(&b, &[down], DVec3::X).unwrap();
        check(&k, 7);
        // Twice in a row keeps the first cut.
        let again = Loop3 { segments: vec![Segment::line(DVec3::new(3.0, 0.0, 2.0), DVec3::new(3.0, 3.0, 2.0))] };
        let line = Loop3 { segments: vec![Segment::line(DVec3::new(1.0, 0.0, 2.0), DVec3::new(1.0, 3.0, 2.0))] };
        let k = knife(&knife(&b, &[line], DVec3::Z).unwrap(), &[again], DVec3::Z).unwrap();
        check(&k, 8);
    }

    #[test]
    fn regions_of_overlapping_boxes() {
        let a = make_box(&Frame::WORLD, DVec3::new(2.0, 2.0, 2.0));
        let b = make_box(&Frame { origin: DVec3::new(1.0, 1.0, 0.0), ..Frame::WORLD }, DVec3::new(2.0, 2.0, 2.0));
        let far = make_box(&Frame { origin: DVec3::new(10.0, 0.0, 0.0), ..Frame::WORLD }, DVec3::ONE);
        let r = regions(&[a, b, far], &[]).unwrap();
        let mut v: Vec<f64> = r.iter().map(vol).collect();
        v.sort_by(f64::total_cmp);
        assert_eq!(v.len(), 4, "{v:?}");
        for (got, want) in v.iter().zip([1.0, 2.0, 6.0, 6.0]) {
            assert!((got - want).abs() < 1e-6, "{v:?}");
        }
    }

    #[test]
    fn completes_a_partial_split() {
        let b = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
        let full = Loop3 { segments: vec![Segment::line(DVec3::new(1.0, 0.0, 2.0), DVec3::new(1.0, 3.0, 2.0))] };
        let part = Loop3 { segments: vec![Segment::line(DVec3::new(1.0, 1.0, 2.0), DVec3::new(4.0, 1.0, 2.0))] };
        let k = knife(&knife(&b, &[full], DVec3::Z).unwrap(), &[part], DVec3::Z).unwrap();
        assert_eq!(k.faces.len(), 8);
        let e = k.edge_ids().find(|e| { let m = k.edge(*e).mid(); (m.y - 1.0).abs() < 1e-9 && (m.z - 2.0).abs() < 1e-9 }).unwrap();
        let c = complete_edges(&k, &[e]).unwrap();
        assert_eq!(c.faces.len(), 9);
        assert_eq!(crate::ops::boolean::open_edges(&c), 0);
        assert!((vol(&c) - 24.0).abs() < 1e-6);
        assert!(c.edges.iter().any(|e| { let m = e.mid(); (m.y - 1.0).abs() < 1e-9 && (m.x - 0.5).abs() < 1e-9 && (m.z - 2.0).abs() < 1e-9 }));
    }

    #[test]
    fn offsets_edges_into_their_faces() {
        let b = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
        let top_front = b.edge_ids().find(|e| { let m = b.edge(*e).mid(); m.z > 1.9 && m.y < 0.1 }).unwrap();
        let k = offset_edges(&b, &[top_front], 0.5).unwrap();
        // The top and the front each split in two.
        assert_eq!(k.faces.len(), 8);
        assert_eq!(crate::ops::boolean::open_edges(&k), 0);
        assert!((vol(&k) - 24.0).abs() < 1e-6);
        let new: Vec<DVec3> = k.edges.iter().map(|e| e.mid()).filter(|m| (m.y - 0.5).abs() < 1e-9 && (m.z - 2.0).abs() < 1e-9 || (m.z - 1.5).abs() < 1e-9 && m.y.abs() < 1e-9).collect();
        assert_eq!(new.len(), 2, "{new:?}");
    }

    #[test]
    fn intersection_curves_of_a_box_and_a_cylinder() {
        let b = make_box(&Frame::WORLD, DVec3::new(4.0, 4.0, 2.0));
        let cyl = crate::primitives::make_cylinder(&Frame::new(DVec3::new(2.0, 2.0, -1.0), DVec3::Z, DVec3::X), 1.0, 4.0);
        let w = intersection_curves(&b, &cyl).unwrap();
        assert_eq!(w.kind, BodyKind::Wire);
        let len: f64 = w.edges.iter().map(|e| e.length()).sum();
        // Two circles where the cylinder passes through top and bottom.
        assert!((len - 4.0 * PI).abs() < 1e-6, "{len}");
        let far = make_box(&Frame { origin: DVec3::splat(10.0), ..Frame::WORLD }, DVec3::ONE);
        assert!(intersection_curves(&b, &far).unwrap().edges.is_empty());
        // A sphere pierced off-centre by a cylinder: two closed space curves.
        let ball = crate::primitives::make_sphere(DVec3::ZERO, 2.0);
        let rod = crate::primitives::make_cylinder(&Frame::new(DVec3::new(0.7, 0.0, -3.0), DVec3::Z, DVec3::X), 0.8, 6.0);
        let w = intersection_curves(&ball, &rod).unwrap();
        let chains = crate::profile::wire_chains(&w);
        assert_eq!(chains.len(), 2);
        assert!(chains.iter().all(|c| c.is_closed()));
        for e in &w.edges {
            for k in 0..=8 {
                let p = e.curve.point(e.t0 + (e.t1 - e.t0) * k as f64 / 8.0);
                assert!((p.length() - 2.0).abs() < 1e-4 && ((p.x - 0.7).hypot(p.y) - 0.8).abs() < 1e-4, "{p}");
            }
        }
    }

    #[test]
    fn cuts_and_imprints_with_sheets() {
        let b = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
        // A flat sheet through the middle, overhanging the box.
        let edge = Loop3 { segments: vec![Segment::line(DVec3::new(-1.0, -1.0, 1.0), DVec3::new(5.0, -1.0, 1.0))] };
        let flat = crate::ops::sheet::extrude_sheet(&[edge], DVec3::new(0.0, 5.0, 0.0)).unwrap();
        let mut parts = cut_with_sheet(&b, &flat).unwrap();
        parts.sort_by(|x, y| vol(x).total_cmp(&vol(y)));
        assert_eq!(parts.len(), 2);
        assert!((vol(&parts[0]) - 12.0).abs() < 1e-9 && (vol(&parts[1]) - 12.0).abs() < 1e-9);
        // A wavy sheet: the pieces still add up.
        let n = ductile_geom::NurbsCurve::interpolate(&[DVec3::new(-1.0, -1.0, 0.8), DVec3::new(1.0, -1.0, 1.4), DVec3::new(3.0, -1.0, 0.6), DVec3::new(5.0, -1.0, 1.2)], 3);
        let (t0, t1) = n.domain();
        let wave = Loop3 { segments: vec![Segment::new(Curve3::Nurbs(n), t0, t1)] };
        let wavy = crate::ops::sheet::extrude_sheet(&[wave], DVec3::new(0.0, 5.0, 0.0)).unwrap();
        let parts = cut_with_sheet(&b, &wavy).unwrap();
        assert_eq!(parts.len(), 2);
        let total: f64 = parts.iter().map(vol).sum();
        assert!((total - 24.0).abs() < 1e-4, "{total}");
        let split = imprint_sheet(&b, &wavy).unwrap();
        assert_eq!(split.faces.len(), 10);
        assert_eq!(crate::ops::boolean::open_edges(&split), 0);
    }

    #[test]
    fn projects_curves_onto_a_sphere() {
        let ball = crate::primitives::make_sphere(DVec3::ZERO, 2.0);
        let line = Loop3 { segments: vec![Segment::line(DVec3::new(-1.0, 0.5, 5.0), DVec3::new(1.0, 0.5, 5.0))] };
        let w = project_curves(&ball, &[line], DVec3::Z).unwrap();
        // Two arcs, one on top and one underneath, each over x in [-1, 1].
        assert_eq!(w.edges.len(), 2, "{:?}", w.edges.iter().map(|e| (e.start(), e.end())).collect::<Vec<_>>());
        for e in &w.edges {
            for k in 0..=8 {
                let p = e.curve.point(e.t0 + (e.t1 - e.t0) * k as f64 / 8.0);
                assert!((p.length() - 2.0).abs() < 1e-6 && (p.y - 0.5).abs() < 1e-9 && p.x.abs() <= 1.0 + 1e-5, "{p}");
            }
        }
        let circle = Loop3 { segments: vec![Segment::new(Curve3::Circle { frame: Frame::new(DVec3::new(0.0, 0.0, 5.0), DVec3::Z, DVec3::X), radius: 1.0 }, 0.0, 2.0 * PI)] };
        let w = project_curves(&ball, &[circle], DVec3::Z).unwrap();
        let len: f64 = w.edges.iter().map(|e| e.length()).sum();
        assert!((len - 4.0 * PI).abs() < 1e-6, "{len}");
    }

    #[test]
    fn splits_disjoint_shells_but_keeps_voids() {
        let mut two = make_box(&Frame::WORLD, DVec3::ONE);
        two.append(&make_box(&Frame { origin: DVec3::new(3.0, 0.0, 0.0), ..Frame::WORLD }, DVec3::ONE));
        assert_eq!(split_shells(&two).len(), 2);
        let hollow = crate::ops::offset::shell(&make_box(&Frame::WORLD, DVec3::splat(4.0)), &[], 0.5).unwrap();
        let parts = split_shells(&hollow);
        assert_eq!(parts.len(), 1);
        assert!((vol(&parts[0]) - (64.0 - 27.0)).abs() < 1e-9);
    }
}
