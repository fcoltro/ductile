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
            for op in [BoolOp::Intersection, BoolOp::Difference] {
                match boolean(p, &prism, op) {
                    Ok(r) => next.extend(split_shells(&r)),
                    // The curve missed this piece, or took all of it.
                    Err(BoolError::Empty) => {}
                    Err(e) => return Err(e),
                }
            }
        }
        pieces = next;
    }
    Ok(pieces)
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
        let tool: Vec<_> = prism.faces.iter().map(|f| f.surface.clone()).collect();
        let tol = 1e-6 * bb.diagonal().max(1.0);
        let mut sewer = Sewer::new(body.kind, tol);
        let mut pieces = 0;
        for op in [BoolOp::Intersection, BoolOp::Difference] {
            let part = match boolean(&body, &prism, op) {
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
        // A curve that missed the solid leaves it as it was.
        if pieces == 2 {
            body = sewer.body;
        }
    }
    Ok(body)
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
