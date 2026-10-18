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
        self.patch_oriented(s, false);
    }

    /// As [`Sewer::patch`], with the normal flipped when `reversed`.
    pub fn patch_oriented(&mut self, s: NurbsSurface, reversed: bool) {
        let ((u0, u1), (v0, v1)) = s.domain();
        let iso = |c: NurbsCurve, rev: bool| {
            let (a, b) = c.domain();
            Segment { curve: Curve3::Nurbs(c), t0: a, t1: b, reversed: rev }
        };
        let mut lp = vec![iso(s.iso_v(v0), false), iso(s.iso_u(u1), false), iso(s.iso_v(v1), true), iso(s.iso_u(u0), true)];
        if reversed {
            lp = lp.into_iter().rev().map(|x| x.flipped()).collect();
        }
        self.face(Surface::Nurbs(s), reversed, &lp, &[]);
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
    Ok(close_between(sheet, &top))
}

/// The solid swept by moving `sheet` straight along `d`: the sheet, its
/// moved copy and ruled walls between their boundaries. The sheet should
/// not fold back over itself as seen along `d`.
pub fn sweep_sheet(sheet: &Body, d: DVec3) -> Option<Body> {
    use crate::primitives::flip_orientation;
    if sheet.kind != BodyKind::Sheet || sheet.faces.is_empty() || d.length() < 1e-12 {
        return None;
    }
    // Sweep towards the sheet's front, flipping it first if needed.
    let mesh = crate::tess::tessellate(sheet, &crate::tess::TessOptions::for_size(sheet.bbox().diagonal()));
    let mut front = 0.0;
    for f in &mesh.faces {
        for t in &f.triangles {
            let [a, b, c] = [f.positions[t[0] as usize], f.positions[t[1] as usize], f.positions[t[2] as usize]];
            front += (b - a).cross(c - a).dot(d);
        }
    }
    let base = if front < 0.0 { flip_orientation(sheet.clone()) } else { sheet.clone() };
    Some(close_between(&base, &base.transformed(&ductile_geom::DMat4::from_translation(d))))
}

/// The solid enclosed by `sheet` (facing away), `top` (a copy of the sheet
/// with the same topology, lying in front of it) and walls joining their
/// boundary edges.
fn close_between(sheet: &Body, top: &Body) -> Body {
    use crate::primitives::flip_orientation;
    let mut b = flip_orientation(sheet.clone());
    b.kind = BodyKind::Solid;
    let (nv, ne) = (b.vertices.len() as u32, b.edges.len() as u32);
    b.append(top);
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
    b
}

/// The surface beyond one side of a patch: the neighbouring face's surface
/// and a point inside that face (telling which way is away from it).
pub type Neighbour = Option<(Surface, DVec3)>;

/// Fills a closed boundary like [`patch`], but meeting the faces beyond its
/// segments tangentially (G1): `neighbours[i]` is the face beyond
/// `boundary.segments[i]`, or None for a free side. The patch is fitted
/// with its boundary rows fixed, its cross-boundary derivatives in the
/// neighbours' tangent planes, and the rest faired.
pub fn patch_g1(boundary: &Loop3, neighbours: &[Neighbour]) -> Option<Body> {
    if neighbours.iter().all(Option::is_none) || neighbours.len() != boundary.segments.len() {
        return patch(boundary, &[]);
    }
    let [bottom, right, top, left] = four_sides(boundary)?;
    let segs = &boundary.segments;
    // Per segment, the sign that turns n x t away from its neighbour.
    let signs: Vec<f64> = segs
        .iter()
        .zip(neighbours)
        .map(|(s, nb)| {
            let Some((surf, inside)) = nb else { return 1.0 };
            let (m, t) = mid_point_tangent(s);
            let uv = surf.project(m, None);
            let d = surf.normal(uv.x, uv.y).cross(t);
            if d.dot(*inside - m) > 0.0 { -1.0 } else { 1.0 }
        })
        .collect();
    let cross = |p: DVec3| -> Option<(DVec3, DVec3)> {
        // The boundary segment through p.
        let (i, t) = segs
            .iter()
            .enumerate()
            .map(|(i, s)| (i, s.curve.closest_param_in(p, s.t0.min(s.t1), s.t0.max(s.t1))))
            .min_by(|a, b| segs[a.0].curve.point(a.1).distance(p).total_cmp(&segs[b.0].curve.point(b.1).distance(p)))?;
        let (surf, _) = neighbours[i].as_ref()?;
        let uv = surf.project(p, None);
        let n = surf.normal(uv.x, uv.y);
        let d = n.cross(segs[i].tangent_at(t)).try_normalize()?;
        Some((n, d * signs[i]))
    };
    let s = fit_g1([&bottom, &right, &top.reversed(), &left.reversed()], &cross)?;
    let mut sw = Sewer::new(BodyKind::Sheet, tolerance(std::slice::from_ref(boundary)));
    sw.patch(s);
    Some(sw.body)
}

/// Least-squares patch through four boundary curves (bottom and top in
/// +u, left and right in +v, as for Coons) whose derivative across each
/// side lies in the plane with normal `n` and leans along `d`, where
/// `cross(p) = (n, d)` at a boundary point `p` (d pointing into the patch);
/// sides where `cross` is None are free.
fn fit_g1(sides: [&NurbsCurve; 4], cross: &dyn Fn(DVec3) -> Option<(DVec3, DVec3)>) -> Option<NurbsSurface> {
    use crate::ops::corner::Lsq;
    use ductile_geom::nurbs::{basis::{basis_funs, ders_basis_funs, find_span}, make_compatible};
    let prep = |c: &NurbsCurve| {
        let c = if c.is_rational() {
            let (a, b) = c.domain();
            let n = 8 * c.ctrl.len().max(4);
            let pts: Vec<DVec3> = (0..=n).map(|i| c.point(a + (b - a) * i as f64 / n as f64)).collect();
            NurbsCurve::interpolate(&pts, 3)
        } else {
            c.clone()
        };
        if c.degree < 3 { c.elevated_to(3) } else { c }
    };
    let refine = |mut cs: Vec<NurbsCurve>, n: usize| {
        while cs[0].ctrl.len() < n {
            let k = &cs[0].knots;
            let (i, _) = k.windows(2).enumerate().max_by(|a, b| (a.1[1] - a.1[0]).total_cmp(&(b.1[1] - b.1[0])))?;
            let mid = 0.5 * (k[i] + k[i + 1]);
            cs = cs.iter().map(|c| c.insert_knot(mid, 1)).collect();
        }
        Some(cs)
    };
    let uc = refine(make_compatible(&[prep(sides[0]), prep(sides[2])]), 8)?;
    let vc = refine(make_compatible(&[prep(sides[3]), prep(sides[1])]), 8)?;
    let (p, q) = (uc[0].degree, vc[0].degree);
    let (ku, kv) = (uc[0].knots.clone(), vc[0].knots.clone());
    let (nu, nv) = (uc[0].ctrl.len(), vc[0].ctrl.len());
    // Solved relative to the boundary's centre, for conditioning.
    let centre = uc.iter().chain(vc.iter()).flat_map(|c| c.ctrl.iter().map(|h| h.truncate() / h.w)).sum::<DVec3>() / (2 * (uc[0].ctrl.len() + vc[0].ctrl.len())) as f64;
    let cp = |c: &NurbsCurve, i: usize| c.ctrl[i].truncate() / c.ctrl[i].w - centre;
    // Fixed boundary rows, unknown interior.
    #[derive(Clone, Copy)]
    enum S {
        F(DVec3),
        V(usize),
    }
    let mut n_var = 0;
    let mut grid = vec![vec![S::F(DVec3::ZERO); nv]; nu];
    for (i, col) in grid.iter_mut().enumerate() {
        for (j, slot) in col.iter_mut().enumerate() {
            *slot = if j == 0 {
                S::F(cp(&uc[0], i))
            } else if j == nv - 1 {
                S::F(cp(&uc[1], i))
            } else if i == 0 {
                S::F(cp(&vc[0], j))
            } else if i == nu - 1 {
                S::F(cp(&vc[1], j))
            } else {
                n_var += 1;
                S::V(n_var - 1)
            };
        }
    }
    let mut lsq = Lsq::new(3 * n_var);
    // A row sum_k w_k * (component c of grid point k) = rhs, per component.
    let add = |lsq: &mut Lsq, terms: &[((usize, usize), DVec3)], rhs: f64, w: f64| {
        let mut coef = Vec::new();
        let mut r = rhs;
        for &((i, j), a) in terms {
            match grid[i][j] {
                S::F(x) => r -= a.dot(x),
                S::V(k) => {
                    for c in 0..3 {
                        if a[c] != 0.0 {
                            coef.push((3 * k + c, a[c]));
                        }
                    }
                }
            }
        }
        if !coef.is_empty() {
            lsq.row(&coef, r, w);
        }
    };
    // Cross-boundary rows. side: 0 bottom, 1 right, 2 top, 3 left.
    for side in 0..4 {
        let along_u = side % 2 == 0;
        let (curve, knots, deg, n) = if along_u { (&uc[side / 2], &ku, p, nu) } else { (&vc[if side == 3 { 0 } else { 1 }], &kv, q, nv) };
        let (other_k, other_deg, other_n) = if along_u { (&kv, q, nv) } else { (&ku, p, nu) };
        let at_end = side == 1 || side == 2;
        // Derivative across the side, into the patch: f (P_inner - P_edge).
        let f = if at_end { other_deg as f64 / (other_k[other_n - 1 + other_deg] - other_k[other_n - 1]) } else { other_deg as f64 / (other_k[other_deg + 1] - other_k[1]) };
        let (edge, inner) = if at_end { (other_n - 1, other_n - 2) } else { (0, 1) };
        // The side across from this one, sampled at the same parameter.
        let opposite = if along_u { &uc[1 - side / 2] } else { &vc[if side == 3 { 1 } else { 0 }] };
        let samples = 4 * n;
        for s in 0..samples {
            let t = knots[deg] + (knots[n] - knots[deg]) * (s as f64 + 0.5) / samples as f64;
            let at = curve.point(t);
            let Some((nrm, d)) = cross(at) else { continue };
            // How far the patch runs across, as the arc leaving here along
            // d and reaching the opposite side along its own direction (or
            // symmetrically, if that side is free): chord c turning through
            // theta has length c (theta / 2) / sin(theta / 2).
            let o = opposite.point(t);
            let chord = o - at;
            let theta = match cross(o) {
                Some((_, d_o)) => d.angle_between(-d_o),
                None => 2.0 * d.angle_between(chord),
            }
            .min(3.0);
            let reach = if theta > 1e-6 { chord.length() * (theta / 2.0) / (theta / 2.0).sin() } else { chord.length() };
            let span = find_span(n - 1, deg, t, knots);
            let b = basis_funs(span, t, deg, knots);
            let idx = |k: usize, row: usize| if along_u { (k, row) } else { (row, k) };
            // The derivative (into the patch) as terms per component.
            let terms = |dir: DVec3| -> Vec<((usize, usize), DVec3)> {
                let mut v = Vec::new();
                for (m, bm) in b.iter().enumerate() {
                    let k = span - deg + m;
                    // Into the patch: from the edge row to the inner row.
                    v.push((idx(k, inner), dir * (f * bm)));
                    v.push((idx(k, edge), dir * (-f * bm)));
                }
                v
            };
            // In the neighbour's tangent plane (hard), leaning along d (soft).
            add(&mut lsq, &terms(nrm), 0.0, 300.0);
            for c in 0..3 {
                let mut e = DVec3::ZERO;
                e[c] = 1.0;
                add(&mut lsq, &terms(e), reach * d[c], 1.0);
            }
        }
    }
    // Fairness: the surface's energy, integrated over a grid of samples.
    // Third derivatives (change of curvature) by default, so round shapes
    // stay round; a little second derivative energy keeps it regular.
    let m = 3 * nu.max(nv);
    let unit = [DVec3::X, DVec3::Y, DVec3::Z];
    let (ud, vd) = ((ku[p], ku[nu]), (kv[q], kv[nv]));
    for a in 0..m {
        let u = ud.0 + (ud.1 - ud.0) * (a as f64 + 0.5) / m as f64;
        let su = find_span(nu - 1, p, u, &ku);
        let bu = ders_basis_funs(su, u, p, 3.min(p), &ku);
        for c in 0..m {
            let v = vd.0 + (vd.1 - vd.0) * (c as f64 + 0.5) / m as f64;
            let sv = find_span(nv - 1, q, v, &kv);
            let bv = ders_basis_funs(sv, v, q, 3.min(q), &kv);
            // d^(r+s) S / du^r dv^s as terms.
            let term = |r: usize, s: usize, e: DVec3| -> Vec<((usize, usize), DVec3)> {
                let mut t = Vec::new();
                if r > p || s > q {
                    return t;
                }
                for i in 0..=p {
                    for j in 0..=q {
                        let w = bu[r][i] * bv[s][j];
                        if w != 0.0 {
                            t.push(((su - p + i, sv - q + j), e * w));
                        }
                    }
                }
                t
            };
            let w0 = 1.0 / m as f64;
            for e in unit {
                for (r, s, w) in [(3, 0, 1.0), (2, 1, 3.0), (1, 2, 3.0), (0, 3, 1.0)] {
                    add(&mut lsq, &term(r, s, e), 0.0, w0 * f64::sqrt(w));
                }
                for (r, s, w) in [(2, 0, 1.0), (1, 1, 2.0), (0, 2, 1.0)] {
                    add(&mut lsq, &term(r, s, e), 0.0, w0 * f64::sqrt(w) * 0.05);
                }
            }
        }
    }
    let x = lsq.solve();
    let mut ctrl = Vec::with_capacity(nu * nv);
    for col in &grid {
        for slot in col {
            let pt = match *slot {
                S::F(p) => p,
                S::V(k) => DVec3::new(x[3 * k], x[3 * k + 1], x[3 * k + 2]),
            };
            ctrl.push((pt + centre).extend(1.0));
        }
    }
    Some(NurbsSurface::new(p, q, ku, kv, nu, nv, ctrl))
}

/// The NURBS surface of face `f` when the face is the whole surface,
/// bounded by its own boundary iso-curves, so moving its control points
/// keeps it a valid face.
fn whole_patch(b: &Body, f: crate::topo::FaceId) -> Option<&NurbsSurface> {
    let face = b.face(f);
    let Surface::Nurbs(s) = &face.surface else { return None };
    if face.loops.len() != 1 {
        return None;
    }
    let ((u0, u1), (v0, v1)) = s.domain();
    let tol = 1e-6 * (u1 - u0).max(v1 - v0);
    let on_rim = |p: DVec3| {
        let uv = face.surface.project(p, None);
        (uv.x - u0).abs() < tol || (uv.x - u1).abs() < tol || (uv.y - v0).abs() < tol || (uv.y - v1).abs() < tol
    };
    let all = b.face_edges(f).iter().all(|e| {
        let ed = b.edge(*e);
        [0.0, 0.25, 0.5, 0.75, 1.0].iter().all(|k| on_rim(ed.curve.point(ed.t0 + (ed.t1 - ed.t0) * k)))
    });
    all.then_some(s)
}

/// Editable control points of a sheet made of whole NURBS patches: (face,
/// index into the face's net, point). Empty for any other body.
pub fn surface_control_points(b: &Body) -> Vec<(crate::topo::FaceId, u32, DVec3)> {
    if b.kind != BodyKind::Sheet {
        return Vec::new();
    }
    let patches: Option<Vec<_>> = b.face_ids().map(|f| whole_patch(b, f).map(|s| (f, s))).collect();
    let Some(patches) = patches else { return Vec::new() };
    patches.into_iter().flat_map(|(f, s)| s.ctrl.iter().enumerate().map(move |(i, h)| (f, i as u32, h.truncate() / h.w))).collect()
}

/// The sheet with the given control points transformed by `m`. Control
/// points of other faces at the same place (shared boundaries, the seam
/// of a closed surface) move with them, so the faces stay joined.
pub fn move_surface_controls(b: &Body, moves: &[(crate::topo::FaceId, u32)], m: &DMat4) -> Option<Body> {
    if surface_control_points(b).is_empty() {
        return None;
    }
    let size = b.bbox().diagonal().max(1e-9);
    let tol = 1e-9 * size.max(1.0);
    let picked: Vec<DVec3> = moves
        .iter()
        .filter_map(|(f, i)| match &b.face(*f).surface {
            Surface::Nurbs(s) => s.ctrl.get(*i as usize).map(|h| h.truncate() / h.w),
            _ => None,
        })
        .collect();
    let mut sw = Sewer::new(BodyKind::Sheet, 1e-7 * size.max(1.0));
    for f in b.face_ids() {
        let Surface::Nurbs(s) = &b.face(f).surface else { return None };
        let mut s = s.clone();
        for h in s.ctrl.iter_mut() {
            let p = h.truncate() / h.w;
            if picked.iter().any(|q| q.distance(p) < tol) {
                *h = (m.transform_point3(p) * h.w).extend(h.w);
            }
        }
        sw.patch_oriented(s, b.face(f).reversed);
    }
    Some(sw.body)
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

    #[test]
    fn tangent_patch_continues_a_sphere() {
        // The four arcs bounding |x|, |y| <= a on the top of a unit sphere.
        let a: f64 = 0.4;
        let r = (1.0 - a * a).sqrt();
        let z = |x: f64, y: f64| (1.0 - x * x - y * y).sqrt();
        let arc = |fixed_x: bool, c: f64, from: f64, to: f64| {
            // Circle in the plane x = c (or y = c), radius r, angle measured from +z.
            let (origin, normal, xdir) = if fixed_x { (DVec3::new(c, 0.0, 0.0), DVec3::X, DVec3::Z) } else { (DVec3::new(0.0, c, 0.0), DVec3::Y, DVec3::Z) };
            let frame = Frame::new(origin, normal, xdir);
            let ang = |w: f64| {
                let l = frame.to_local(if fixed_x { DVec3::new(c, w, z(c, w)) } else { DVec3::new(w, c, z(w, c)) });
                l.y.atan2(l.x)
            };
            let (t0, t1) = (ang(from), ang(to));
            let s = Segment::new(Curve3::Circle { frame, radius: r }, t0.min(t1), t0.max(t1));
            if t0 > t1 { s.flipped() } else { s }
        };
        let segs = vec![arc(false, -a, -a, a), arc(true, a, -a, a), arc(false, a, a, -a), arc(true, -a, a, -a)];
        let lp = chain(segs);
        for w in 0..4 {
            assert!(lp.segments[w].end().distance(lp.segments[(w + 1) % 4].start()) < 1e-9, "{w}");
        }
        let sphere = Surface::Sphere { frame: Frame::WORLD, radius: 1.0 };
        let nb: Vec<Neighbour> = (0..4)
            .map(|k| {
                // A point of the sphere outside each side.
                let m = lp.segments[k].curve.point(0.5 * (lp.segments[k].t0 + lp.segments[k].t1));
                let out = DVec3::new(m.x, m.y, 0.0).normalize() * 0.8;
                Some((sphere.clone(), DVec3::new(out.x, out.y, z(out.x, out.y))))
            })
            .collect();
        let check = |b: &Body| {
            let Surface::Nurbs(s) = &b.faces[0].surface else { panic!() };
            let mut dev: f64 = 0.0;
            let mut ang: f64 = 0.0;
            for i in 0..=20 {
                for j in 0..=20 {
                    let (u, v) = (i as f64 / 20.0, j as f64 / 20.0);
                    let p = s.point(u, v);
                    dev = dev.max((p.length() - 1.0).abs());
                    if i == 0 || j == 0 || i == 20 || j == 20 {
                        let d = s.derivs(u, v, 1);
                        let n = d[1][0].cross(d[0][1]).normalize();
                        ang = ang.max(n.dot(p.normalize()).abs().min(1.0).acos());
                    }
                }
            }
            (dev, ang.to_degrees())
        };
        let (dev_c, ang_c) = check(&patch(&lp, &[]).unwrap());
        let (dev_g, ang_g) = check(&patch_g1(&lp, &nb).unwrap());
        // Coons meets the sphere at over 2 degrees and sags 0.008; the
        // tangent patch meets it within hundredths of a degree and follows
        // it within a thousandth.
        assert!(ang_c > 1.0, "{ang_c}");
        assert!(ang_g < 0.05, "{ang_g}");
        assert!(dev_g < 1.5e-3 && dev_g < dev_c / 5.0, "{dev_g} vs {dev_c}");
    }

    #[test]
    fn tangent_cap_away_from_the_origin() {
        for (c, r, h) in [(DVec3::ZERO, 1.0, 2.0), (DVec3::new(18.0, -24.0, 0.0), 1.5, 2.5)] {
            let rim = chain(vec![Segment::new(Curve3::Circle { frame: Frame { origin: c, ..Frame::WORLD }, radius: r }, 0.0, 2.0 * PI)]);
            let tube = extrude_sheet(&[rim], DVec3::new(0.0, 0.0, h)).unwrap();
            let e = tube.edge_ids().find(|e| tube.edge(*e).mid().z > h - 0.1 && tube.edge(*e).is_closed()).unwrap();
            let ed = tube.edge(e);
            let lp = chain(vec![Segment::new(ed.curve.clone(), ed.t0, ed.t1)]);
            let inside = c + DVec3::new(0.0, r, 1.0);
            let cap = patch_g1(&lp, &[Some((tube.faces[0].surface.clone(), inside))]).unwrap();
            let Surface::Nurbs(s) = &cap.faces[0].surface else { panic!() };
            let top = s.point(0.5, 0.5);
            assert!(top.z > h + 0.3 * r, "{c}: {top}");
        }
    }

    #[test]
    fn surface_control_points_move() {
        let q = [DVec3::new(0.0, 0.0, 0.0), DVec3::new(2.0, 0.0, 0.0), DVec3::new(2.0, 2.0, 1.0), DVec3::new(0.0, 2.0, 0.0)];
        let skew = patch(&chain((0..4).map(|i| Segment::line(q[i], q[(i + 1) % 4])).collect()), &[]).unwrap();
        let cps = surface_control_points(&skew);
        assert_eq!(cps.len(), 4);
        // Lift one corner: the patch and its edges follow.
        let up = DMat4::from_translation(DVec3::Z);
        let moved = move_surface_controls(&skew, &[(cps[0].0, cps[0].1)], &up).unwrap();
        assert_eq!((moved.faces.len(), moved.edges.len()), (1, 4));
        assert!(moved.vertices.iter().any(|v| v.point.distance(cps[0].2 + DVec3::Z) < 1e-12));
        // A loft of two faces: moving a control point on their shared
        // boundary moves both faces' copies, so they stay joined.
        let a = chain(vec![Segment::line(DVec3::ZERO, DVec3::new(2.0, 0.0, 0.0)), Segment::line(DVec3::new(2.0, 0.0, 0.0), DVec3::new(2.0, 2.0, 0.0))]);
        let b = chain(vec![Segment::line(DVec3::new(0.0, 0.0, 3.0), DVec3::new(2.0, 0.0, 3.0)), Segment::line(DVec3::new(2.0, 0.0, 3.0), DVec3::new(2.0, 2.0, 3.0))]);
        let s = loft_sheet(&[a, b]).unwrap();
        let cps = surface_control_points(&s);
        let shared = cps.iter().find(|c| c.2.distance(DVec3::new(2.0, 0.0, 3.0)) < 1e-12).unwrap();
        let moved = move_surface_controls(&s, &[(shared.0, shared.1)], &DMat4::from_translation(DVec3::new(0.5, -0.5, 0.0))).unwrap();
        assert_eq!(moved.faces.len(), 2);
        assert_eq!(open_boundary(&moved), 6);
        // Trimmed faces have no editable control points.
        let p = [DVec3::ZERO, DVec3::new(4.0, 0.0, 0.0), DVec3::new(4.0, 4.0, 0.0), DVec3::new(0.0, 4.0, 0.0)];
        let flat = patch(&chain((0..4).map(|i| Segment::line(p[i], p[(i + 1) % 4])).collect()), &[]).unwrap();
        assert!(surface_control_points(&flat).is_empty());
    }
}
