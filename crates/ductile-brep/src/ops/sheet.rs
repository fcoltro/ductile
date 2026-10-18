// SPDX-License-Identifier: GPL-3.0-only
//! Sheets: open surface bodies made by extruding, revolving and lofting
//! curves, or by filling a closed boundary with a patch.

use crate::ops::{extrude::side_surface, loft::directed_nurbs, revolve::{radial, surface_of}};
use crate::profile::{loops_plane, Loop3, Segment};
use crate::topo::{Body, BodyKind, Coedge, VertexId};
use ductile_geom::{Curve3, DMat4, DQuat, DVec3, Frame, NurbsCurve, NurbsSurface, Surface};

/// Builds a body face by face from boundary curves, sharing vertices and
/// edges that coincide geometrically, so neighbouring faces are sewn.
pub(crate) struct Sewer {
    pub body: Body,
    tol: f64,
}

impl Sewer {
    pub fn new(kind: BodyKind, tol: f64) -> Self {
        Self { body: Body::new(kind), tol }
    }

    fn vertex(&mut self, p: DVec3) -> VertexId {
        match self.body.vertices.iter().position(|v| v.point.distance(p) < self.tol) {
            Some(i) => VertexId(i as u32),
            None => self.body.add_vertex(p),
        }
    }

    /// The coedge running along `seg`, reusing a coincident edge. None for
    /// a segment that has collapsed to a point (a pole).
    fn coedge(&mut self, seg: &Segment) -> Option<Coedge> {
        let (p, q) = (seg.start(), seg.end());
        let mid = seg.curve.point(0.5 * (seg.t0 + seg.t1));
        if p.distance(q) < self.tol && p.distance(mid) < self.tol {
            return None;
        }
        let (a, z) = (self.vertex(p), self.vertex(q));
        // Direction along the segment just after its start, to orient
        // closed edges.
        let probe = seg.curve.point(if seg.reversed { seg.t1 - 0.25 * (seg.t1 - seg.t0) } else { seg.t0 + 0.25 * (seg.t1 - seg.t0) });
        for (i, e) in self.body.edges.iter().enumerate() {
            let same = (e.v0 == a && e.v1 == z) || (e.v0 == z && e.v1 == a);
            if !same || e.mid().distance(mid) > self.tol {
                continue;
            }
            let reversed = if a != z { e.v0 != a } else { e.curve.point(e.t0 + 0.25 * (e.t1 - e.t0)).distance(probe) > self.tol };
            return Some(Coedge::new(crate::topo::EdgeId(i as u32), reversed));
        }
        let (v0, v1) = if seg.reversed { (z, a) } else { (a, z) };
        Some(Coedge::new(self.body.add_edge(seg.curve.clone(), seg.t0, seg.t1, v0, v1), seg.reversed))
    }

    /// Adds a face bounded by `outer` (and `holes`), each loop a chain of
    /// segments end to end, oriented counter-clockwise about the face
    /// normal (holes clockwise).
    pub fn face(&mut self, surface: Surface, reversed: bool, outer: &[Segment], holes: &[Vec<Segment>]) {
        let mut loops = Vec::new();
        for l in std::iter::once(outer).chain(holes.iter().map(|h| h.as_slice())) {
            let lp: Vec<Coedge> = l.iter().filter_map(|s| self.coedge(s)).collect();
            if !lp.is_empty() {
                loops.push(lp);
            }
        }
        self.body.add_face(surface, reversed, loops);
    }

    /// Adds the whole of a NURBS patch as a face bounded by its four
    /// boundary iso-curves (u along the first, v along the second), with
    /// its normal along Su × Sv.
    pub fn patch(&mut self, s: NurbsSurface) {
        let ((u0, u1), (v0, v1)) = s.domain();
        let iso = |c: NurbsCurve, rev: bool| {
            let (a, b) = c.domain();
            Segment { curve: Curve3::Nurbs(c), t0: a, t1: b, reversed: rev }
        };
        let lp = [iso(s.iso_v(v0), false), iso(s.iso_u(u1), false), iso(s.iso_v(v1), true), iso(s.iso_u(u0), true)];
        self.face(Surface::Nurbs(s), false, &lp, &[]);
    }
}

fn tolerance(chains: &[Loop3]) -> f64 {
    let size = chains.iter().flat_map(|c| c.polygon(1e-2)).map(|p| p.abs().max_element()).fold(1.0, f64::max);
    1e-7 * size
}

/// Whether `surface`'s own normal near `p` points against `want`.
fn flipped(surface: &Surface, p: DVec3, want: DVec3) -> bool {
    let uv = surface.project(p, None);
    surface.normal(uv.x, uv.y).dot(want) < 0.0
}

fn moved(s: &Segment, m: &DMat4) -> Segment {
    Segment { curve: s.curve.transformed(m), ..s.clone() }
}

fn mid_point_tangent(s: &Segment) -> (DVec3, DVec3) {
    let t = 0.5 * (s.t0 + s.t1);
    (s.curve.point(t), s.tangent_at(t))
}

/// Extrudes curve chains (open or closed) along `d` into a sheet, one
/// face per segment: planes for lines, cylinders for arcs extruded along
/// their axis, NURBS extrusions otherwise. The sheet's normal is the
/// curve's direction crossed with `d`.
pub fn extrude_sheet(chains: &[Loop3], d: DVec3) -> Option<Body> {
    if d.length() < 1e-12 || chains.iter().all(|c| c.segments.is_empty()) {
        return None;
    }
    let mut sw = Sewer::new(BodyKind::Sheet, tolerance(chains));
    let shift = DMat4::from_translation(d);
    for c in chains {
        for s in &c.segments {
            let (surface, _) = side_surface(s, d);
            let (m, t) = mid_point_tangent(s);
            let rev = flipped(&surface, m, t.cross(d));
            let (p, q) = (s.start(), s.end());
            let lp = [s.clone(), Segment::line(q, q + d), moved(s, &shift).flipped(), Segment::line(p + d, p)];
            sw.face(surface, rev, &lp, &[]);
        }
    }
    Some(sw.body)
}

/// Revolves curve chains (open or closed) by `angle` about the axis
/// through `origin` along `axis` into a sheet. Lines and arcs in a plane
/// through the axis give exact planes, cylinders, cones, spheres and tori.
pub fn revolve_sheet(chains: &[Loop3], origin: DVec3, axis: DVec3, angle: f64) -> Option<Body> {
    let (axis, angle) = if angle < 0.0 { (-axis, -angle) } else { (axis, angle) };
    let a = axis.try_normalize()?;
    if angle < 1e-9 || chains.iter().all(|c| c.segments.is_empty()) {
        return None;
    }
    let angle = angle.min(std::f64::consts::TAU);
    let tol = tolerance(chains);
    let rot = DMat4::from_translation(origin) * DMat4::from_quat(DQuat::from_axis_angle(a, angle)) * DMat4::from_translation(-origin);
    // Arc swept by a point, None on the axis.
    let arc = |p: DVec3| -> Option<Segment> {
        let r = radial(p, origin, a);
        (r.length() > tol).then(|| {
            let c = p - r;
            Segment::new(Curve3::Circle { frame: Frame::new(c, a, r), radius: r.length() }, 0.0, angle)
        })
    };
    let mut sw = Sewer::new(BodyKind::Sheet, tol);
    for c in chains {
        for s in &c.segments {
            let (m, t) = mid_point_tangent(s);
            let rm = radial(m, origin, a);
            let e = rm.try_normalize().or_else(|| radial(s.start(), origin, a).try_normalize()).or_else(|| radial(s.end(), origin, a).try_normalize())?;
            let (surface, _) = surface_of(s, origin, a, e, tol, angle);
            let rev = flipped(&surface, m, t.cross(a.cross(e)));
            let (p, q) = (s.start(), s.end());
            let mut lp = vec![s.clone()];
            lp.extend(arc(q));
            lp.push(moved(s, &rot).flipped());
            lp.extend(arc(p).map(|x| x.flipped()));
            sw.face(surface, rev, &lp, &[]);
        }
    }
    Some(sw.body)
}

/// Lofts a sheet through two or more open (or closed) curve chains, in
/// order. Chains with the same number of segments give one face per
/// segment, so corners stay edges; otherwise each chain is joined into a
/// single curve. Sections are spaced by the distance between chain
/// midpoints.
pub fn loft_sheet(chains: &[Loop3]) -> Option<Body> {
    let m = chains.len();
    if m < 2 || chains.iter().any(|c| c.segments.is_empty()) {
        return None;
    }
    let mut chains: Vec<Loop3> = chains.to_vec();
    // Run every chain the same way as the first.
    for k in 1..m {
        let (a0, a1) = (chains[k - 1].segments[0].start(), chains[k - 1].segments.last().unwrap().end());
        let (b0, b1) = (chains[k].segments[0].start(), chains[k].segments.last().unwrap().end());
        let along = (b0 - a0).length() + (b1 - a1).length();
        let across = (b0 - a1).length() + (b1 - a0).length();
        if !chains[k].is_closed() && across < along {
            chains[k] = chains[k].reversed();
        }
    }
    let n = chains[0].segments.len();
    let matched = n > 1 && chains.iter().all(|c| c.segments.len() == n);
    let sections: Vec<Vec<NurbsCurve>> = if matched {
        chains.iter().map(|c| c.segments.iter().map(directed_nurbs).collect()).collect()
    } else {
        chains.iter().map(|c| vec![NurbsCurve::join(&c.segments.iter().map(directed_nurbs).collect::<Vec<_>>())]).collect()
    };
    let mids: Vec<DVec3> = chains
        .iter()
        .map(|c| {
            let pts = c.polygon(1e-3);
            pts.iter().copied().sum::<DVec3>() / pts.len() as f64
        })
        .collect();
    let mut params = vec![0.0];
    for k in 1..m {
        params.push(params[k - 1] + mids[k].distance(mids[k - 1]).max(1e-9));
    }
    let total = params[m - 1];
    params.iter_mut().for_each(|p| *p /= total);
    let degree_v = if m >= 4 { 3 } else { m - 1 };
    let mut sw = Sewer::new(BodyKind::Sheet, tolerance(&chains));
    for i in 0..sections[0].len() {
        let secs: Vec<NurbsCurve> = sections.iter().map(|s| s[i].clone()).collect();
        sw.patch(NurbsSurface::loft_with_params(&secs, degree_v, &params));
    }
    Some(sw.body)
}

/// Fills a closed boundary with a sheet. A planar boundary gives a trimmed
/// plane (holes allowed). Otherwise the boundary is divided into four
/// sides at its sharpest corners (splitting or joining segments as
/// needed) and filled with a Coons patch.
pub fn patch(boundary: &Loop3, holes: &[Loop3]) -> Option<Body> {
    if !boundary.is_closed() || boundary.segments.is_empty() {
        return None;
    }
    let all: Vec<Loop3> = std::iter::once(boundary.clone()).chain(holes.iter().cloned()).collect();
    let tol = tolerance(&all);
    let mut sw = Sewer::new(BodyKind::Sheet, tol);
    if let Some(plane) = loops_plane(&all) {
        let n = boundary.normal().normalize();
        let frame = Frame::new(plane.origin, n, plane.x);
        let hole_segs: Vec<Vec<Segment>> = holes.iter().map(|h| if h.normal().dot(n) > 0.0 { h.reversed() } else { h.clone() }.segments).collect();
        sw.face(Surface::Plane { frame }, false, &boundary.segments, &hole_segs);
        return Some(sw.body);
    }
    let sides = four_sides(boundary)?;
    let [bottom, right, top, left] = sides;
    // Coons wants the top and left running in +u and +v.
    sw.patch(NurbsSurface::coons(&bottom, &right, &top.reversed(), &left.reversed()));
    Some(sw.body)
}

/// Splits a closed chain into four curves running around it, at its four
/// sharpest corners; smooth or short chains are split by length.
fn four_sides(boundary: &Loop3) -> Option<[NurbsCurve; 4]> {
    let segs: Vec<NurbsCurve> = boundary.segments.iter().map(directed_nurbs).collect();
    let n = segs.len();
    // Corner sharpness at the start of each segment.
    let turn = |i: usize| {
        let a = segs[(i + n - 1) % n].tangent(segs[(i + n - 1) % n].domain().1).normalize_or_zero();
        let b = segs[i].tangent(segs[i].domain().0).normalize_or_zero();
        a.angle_between(b)
    };
    let mut corners: Vec<usize> = (0..n).filter(|i| n > 1 && turn(*i) > 0.1).collect();
    if corners.len() > 4 {
        corners.sort_by(|x, y| turn(*y).total_cmp(&turn(*x)));
        corners.truncate(4);
        corners.sort();
    }
    // Pieces between corners, each one curve.
    let pieces: Vec<NurbsCurve> = if corners.is_empty() {
        vec![NurbsCurve::join(&segs)]
    } else {
        (0..corners.len())
            .map(|k| {
                let (a, b) = (corners[k], corners[(k + 1) % corners.len()]);
                let idx: Vec<usize> = if b > a { (a..b).collect() } else { (a..n).chain(0..b).collect() };
                let cs: Vec<NurbsCurve> = idx.iter().map(|i| segs[*i].clone()).collect();
                if cs.len() == 1 {
                    cs[0].clone()
                } else {
                    NurbsCurve::join(&cs)
                }
            })
            .collect()
    };
    // Split the longest pieces in half until there are four.
    let mut pieces = pieces;
    while pieces.len() < 4 {
        let (i, _) = pieces.iter().enumerate().max_by(|x, y| x.1.length().total_cmp(&y.1.length()))?;
        let c = pieces.remove(i);
        let (a, b) = c.domain();
        let (l, r) = c.split(half_length_param(&c, a, b));
        pieces.insert(i, r);
        pieces.insert(i, l);
    }
    Some([pieces[0].clone(), pieces[1].clone(), pieces[2].clone(), pieces[3].clone()])
}

/// The parameter halving the curve's length.
fn half_length_param(c: &NurbsCurve, a: f64, b: f64) -> f64 {
    let total = c.length();
    let (mut lo, mut hi) = (a, b);
    for _ in 0..50 {
        let mid = 0.5 * (lo + hi);
        if c.subcurve(a, mid).length() < 0.5 * total {
            lo = mid
        } else {
            hi = mid
        }
    }
    0.5 * (lo + hi)
}

/// Thickens a sheet into a solid `t` thick, on the side its normal points
/// to (the other side for negative `t`): the sheet, its offset, and ruled
/// walls joining their boundaries.
pub fn thicken(sheet: &Body, t: f64) -> Result<Body, crate::ops::offset::OffsetError> {
    use crate::ops::offset::{offset_body, OffsetError};
    use crate::primitives::flip_orientation;
    if sheet.kind != BodyKind::Sheet || sheet.faces.is_empty() || t.abs() < 1e-12 {
        return Err(OffsetError::Unsupported);
    }
    if t < 0.0 {
        return thicken(&flip_orientation(sheet.clone()), -t);
    }
    let top = offset_body(sheet, &|_| t)?;
    let mut b = flip_orientation(sheet.clone());
    b.kind = BodyKind::Solid;
    let (nv, ne) = (b.vertices.len() as u32, b.edges.len() as u32);
    b.append(&top);
    // Each edge's uses by the faces of the bottom and the top.
    let mut uses: Vec<Vec<Coedge>> = vec![Vec::new(); b.edges.len()];
    for f in &b.faces {
        for l in &f.loops {
            for c in &b.loops[l.0 as usize].coedges {
                uses[c.edge.idx()].push(*c);
            }
        }
    }
    let mut risers: std::collections::HashMap<u32, crate::topo::EdgeId> = std::collections::HashMap::new();
    let boundary: Vec<u32> = (0..ne).filter(|e| uses[*e as usize].len() == 1).collect();
    for e in boundary {
        let rb = uses[e as usize][0].reversed;
        let rt = uses[(e + ne) as usize][0].reversed;
        let (bot, topc) = (b.edges[e as usize].clone(), b.edges[(e + ne) as usize].clone());
        // The wall runs along the bottom edge from a to z.
        let (a, z) = if rb { (bot.v0, bot.v1) } else { (bot.v1, bot.v0) };
        let mut riser = |b: &mut Body, v: VertexId| *risers.entry(v.0).or_insert_with(|| b.add_line_edge(v, VertexId(v.0 + nv)));
        let (ra, rz) = (riser(&mut b, a), riser(&mut b, z));
        let lp = vec![Coedge::new(crate::topo::EdgeId(e), !rb), Coedge::new(rz, false), Coedge::new(crate::topo::EdgeId(e + ne), !rt), Coedge::new(ra, true)];
        // Wall surface: flat when both edges are straight, else ruled.
        let bn = bot.curve.to_nurbs(bot.t0, bot.t1);
        let tn = topc.curve.to_nurbs(topc.t0, topc.t1);
        let surface = if bot.curve.is_linear() && topc.curve.is_linear() {
            let d = bot.end() - bot.start();
            let up = topc.start() - bot.start();
            Surface::Plane { frame: Frame::new(bot.start(), d.cross(up), d) }
        } else {
            Surface::Nurbs(NurbsSurface::ruled(&bn, &tn))
        };
        // Outward: along the wall's first edge crossed with the way up.
        let tm = 0.5 * (bot.t0 + bot.t1);
        let tan = bot.curve.tangent(tm) * if rb { 1.0 } else { -1.0 };
        let m = bot.curve.point(tm);
        let up = topc.curve.point(topc.curve.closest_param_in(m, topc.t0, topc.t1)) - m;
        let rev = flipped(&surface, m + up * 0.5, tan.cross(up));
        b.add_face(surface, rev, vec![lp]);
    }
    Ok(b)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tess::{tessellate, TessOptions};
    use std::f64::consts::PI;

    fn area(b: &Body) -> f64 {
        let m = tessellate(b, &TessOptions { chord_tol: 1e-4, angle_tol: 0.05 });
        m.area()
    }

    fn open_boundary(b: &Body) -> usize {
        b.edge_ids().filter(|e| b.edge_faces(*e).len() == 1).count()
    }

    fn chain(segs: Vec<Segment>) -> Loop3 {
        Loop3 { segments: segs }
    }

    #[test]
    fn extruded_sheets() {
        // An L of two lines and a quarter arc, extruded 2 up.
        let arc = Segment::new(Curve3::Circle { frame: Frame::new(DVec3::new(3.0, 1.0, 0.0), DVec3::Z, DVec3::NEG_Y), radius: 1.0 }, 0.0, PI / 2.0);
        let c = chain(vec![Segment::line(DVec3::new(0.0, 0.0, 0.0), DVec3::new(3.0, 0.0, 0.0)), arc, Segment::line(DVec3::new(4.0, 1.0, 0.0), DVec3::new(4.0, 3.0, 0.0))]);
        let b = extrude_sheet(&[c], DVec3::new(0.0, 0.0, 2.0)).unwrap();
        assert_eq!(b.kind, BodyKind::Sheet);
        assert_eq!(b.faces.len(), 3);
        // The two inner joins are shared; the rest is boundary.
        assert_eq!(b.edges.len(), 10);
        assert_eq!(open_boundary(&b), 8);
        let expect = 2.0 * (3.0 + PI / 2.0 + 2.0);
        assert!((area(&b) - expect).abs() < 1e-3, "{} vs {expect}", area(&b));
        assert!(matches!(b.faces[1].surface, Surface::Cylinder { .. }));
        // Normals agree across the joins: t x d points to -y on the first face.
        let n0 = b.face_normal(crate::topo::FaceId(0), 1.0, 0.5);
        assert!(n0.y < -0.99, "{n0}");
    }

    #[test]
    fn revolved_sheets() {
        // A quarter circle from the axis: a hemisphere, area 2 pi r^2.
        let arc = Segment::new(Curve3::Circle { frame: Frame::new(DVec3::ZERO, DVec3::NEG_Y, DVec3::Z), radius: 2.0 }, 0.0, PI / 2.0);
        let b = revolve_sheet(&[chain(vec![arc])], DVec3::ZERO, DVec3::Z, 2.0 * PI).unwrap();
        assert_eq!(b.faces.len(), 1);
        assert!((area(&b) - 8.0 * PI).abs() < 1e-2, "{}", area(&b));
        assert_eq!(open_boundary(&b), 1);
        // A slanted line a quarter turn: a quarter of a cone frustum.
        let l = Segment::line(DVec3::new(1.0, 0.0, 0.0), DVec3::new(2.0, 0.0, 1.0));
        let b = revolve_sheet(&[chain(vec![l])], DVec3::ZERO, DVec3::Z, PI / 2.0).unwrap();
        let expect = PI * (1.0 + 2.0) * 2f64.sqrt() / 4.0;
        assert!((area(&b) - expect).abs() < 1e-3, "{} vs {expect}", area(&b));
        assert_eq!(open_boundary(&b), 4);
    }

    #[test]
    fn lofted_sheet() {
        let a = chain(vec![Segment::line(DVec3::ZERO, DVec3::new(2.0, 0.0, 0.0)), Segment::line(DVec3::new(2.0, 0.0, 0.0), DVec3::new(2.0, 2.0, 0.0))]);
        let b = chain(vec![Segment::line(DVec3::new(2.0, 2.0, 3.0), DVec3::new(2.0, 0.0, 3.0)), Segment::line(DVec3::new(2.0, 0.0, 3.0), DVec3::new(0.0, 0.0, 3.0))]);
        // b runs backwards: it is turned around to match a.
        let s = loft_sheet(&[a, b]).unwrap();
        assert_eq!(s.faces.len(), 2);
        assert_eq!(open_boundary(&s), 6);
        assert!((area(&s) - 12.0).abs() < 1e-6, "{}", area(&s));
    }

    #[test]
    fn patches() {
        // Planar: a square with a round hole.
        let p = [DVec3::ZERO, DVec3::new(4.0, 0.0, 0.0), DVec3::new(4.0, 4.0, 0.0), DVec3::new(0.0, 4.0, 0.0)];
        let sq = chain((0..4).map(|i| Segment::line(p[i], p[(i + 1) % 4])).collect());
        let hole = chain(vec![Segment::new(Curve3::Circle { frame: Frame::new(DVec3::new(2.0, 2.0, 0.0), DVec3::Z, DVec3::X), radius: 1.0 }, 0.0, 2.0 * PI)]);
        let b = patch(&sq, &[hole]).unwrap();
        assert!((area(&b) - (16.0 - PI)).abs() < 1e-3, "{}", area(&b));
        // A saddle boundary of four lines: the Coons patch is the
        // hyperbolic paraboloid z = xy / 4 over [0, 2]^2... with corners
        // lifted alternately.
        let q = [DVec3::new(0.0, 0.0, 0.0), DVec3::new(2.0, 0.0, 0.0), DVec3::new(2.0, 2.0, 1.0), DVec3::new(0.0, 2.0, 0.0)];
        let skew = chain((0..4).map(|i| Segment::line(q[i], q[(i + 1) % 4])).collect());
        let b = patch(&skew, &[]).unwrap();
        assert_eq!(b.faces.len(), 1);
        assert_eq!(b.edges.len(), 4);
        let Surface::Nurbs(s) = &b.faces[0].surface else { panic!() };
        let c = s.point(0.5, 0.5);
        assert!((c.z - 0.25).abs() < 1e-9, "{c}");
        // Three sides: one is split so the patch still has four.
        let t = [DVec3::ZERO, DVec3::new(2.0, 0.0, 0.5), DVec3::new(1.0, 2.0, 0.0)];
        let bulge = NurbsCurve::uniform(2, &[t[1], DVec3::new(2.0, 1.5, 1.5), t[2]]);
        let tri = chain(vec![Segment::line(t[0], t[1]), Segment::new(Curve3::Nurbs(bulge), 0.0, 1.0), Segment::line(t[2], t[0])]);
        let b = patch(&tri, &[]).unwrap();
        assert_eq!(b.edges.len(), 4);
        assert!(area(&b) > 0.0);
    }

    fn volume(b: &Body) -> f64 {
        tessellate(b, &TessOptions { chord_tol: 1e-4, angle_tol: 0.05 }).volume()
    }

    #[test]
    fn thickened_sheets() {
        use crate::ops::boolean::open_edges;
        // A flat square, both ways.
        let p = [DVec3::ZERO, DVec3::new(2.0, 0.0, 0.0), DVec3::new(2.0, 2.0, 0.0), DVec3::new(0.0, 2.0, 0.0)];
        let sq = patch(&chain((0..4).map(|i| Segment::line(p[i], p[(i + 1) % 4])).collect()), &[]).unwrap();
        for t in [0.5, -0.5] {
            let b = thicken(&sq, t).unwrap();
            assert_eq!(b.kind, BodyKind::Solid);
            assert_eq!(b.faces.len(), 6);
            assert_eq!(open_edges(&b), 0);
            assert!((volume(&b) - 2.0).abs() < 1e-9, "{t}: {}", volume(&b));
        }
        // A dome: a hemispherical shell, outward.
        let arc = Segment::new(Curve3::Circle { frame: Frame::new(DVec3::ZERO, DVec3::NEG_Y, DVec3::Z), radius: 2.0 }, 0.0, PI / 2.0);
        let dome = revolve_sheet(&[chain(vec![arc])], DVec3::ZERO, DVec3::Z, 2.0 * PI).unwrap();
        let out = dome.face_normal(crate::topo::FaceId(0), 0.3, 0.3).dot(DVec3::Z) > 0.0;
        let t = if out { 0.2 } else { -0.2 };
        let b = thicken(&dome, t).unwrap();
        assert_eq!(open_edges(&b), 0);
        let expect = 2.0 * PI / 3.0 * (2.2f64.powi(3) - 8.0);
        assert!((volume(&b) - expect).abs() < 2e-2, "{} vs {expect}", volume(&b));
        // A Coons patch on a curved boundary.
        let q = [DVec3::new(0.0, 0.0, 0.0), DVec3::new(2.0, 0.0, 0.0), DVec3::new(2.0, 2.0, 1.0), DVec3::new(0.0, 2.0, 0.0)];
        let skew = patch(&chain((0..4).map(|i| Segment::line(q[i], q[(i + 1) % 4])).collect()), &[]).unwrap();
        let b = thicken(&skew, 0.1).unwrap();
        assert_eq!(open_edges(&b), 0);
        let v = volume(&b);
        // Thin enough that the curvature terms of the offset volume stay
        // within a tenth of a percent.
        assert!((v / (0.1 * area(&skew)) - 1.0).abs() < 2e-3, "{v} vs {}", 0.1 * area(&skew));
    }
}
