// SPDX-License-Identifier: GPL-3.0-only
//! Profiles: closed chains of curve segments used as input to sweeps.

use crate::topo::{Body, BodyKind, FaceId, VertexId};
use ductile_geom::{Curve3, DVec3, Frame};

/// A piece of curve traversed from `t0` to `t1` (or backwards if `reversed`).
#[derive(Clone, Debug, PartialEq)]
pub struct Segment {
    pub curve: Curve3,
    pub t0: f64,
    pub t1: f64,
    pub reversed: bool,
}

impl Segment {
    pub fn new(curve: Curve3, t0: f64, t1: f64) -> Self {
        Self { curve, t0, t1, reversed: false }
    }

    pub fn line(a: DVec3, b: DVec3) -> Self {
        Self::new(Curve3::line_through(a, b), 0.0, a.distance(b))
    }

    pub fn start(&self) -> DVec3 {
        self.curve.point(if self.reversed { self.t1 } else { self.t0 })
    }

    pub fn end(&self) -> DVec3 {
        self.curve.point(if self.reversed { self.t0 } else { self.t1 })
    }

    pub fn flipped(&self) -> Self {
        Self { reversed: !self.reversed, ..self.clone() }
    }

    /// Tangent in traversal direction at the parameter `t`.
    pub fn tangent_at(&self, t: f64) -> DVec3 {
        let d = self.curve.tangent(t);
        if self.reversed {
            -d
        } else {
            d
        }
    }

    pub fn is_closed(&self) -> bool {
        self.start().distance(self.end()) < 1e-9
    }

    /// Sample points in traversal order.
    pub fn sample(&self, tol: f64) -> Vec<DVec3> {
        let mut pts: Vec<DVec3> = self.curve.tessellate(self.t0, self.t1, tol).into_iter().map(|x| x.1).collect();
        if self.reversed {
            pts.reverse();
        }
        pts
    }
}

/// A closed loop of segments, each ending where the next starts.
#[derive(Clone, Debug, PartialEq)]
pub struct Loop3 {
    pub segments: Vec<Segment>,
}

impl Loop3 {
    pub fn reversed(&self) -> Loop3 {
        Loop3 { segments: self.segments.iter().rev().map(|s| s.flipped()).collect() }
    }

    pub fn polygon(&self, tol: f64) -> Vec<DVec3> {
        let mut out = Vec::new();
        for s in &self.segments {
            let mut p = s.sample(tol);
            p.pop();
            out.extend(p);
        }
        out
    }

    /// Newell normal (area-weighted) of the loop; its length is twice the area.
    pub fn normal(&self) -> DVec3 {
        let pts = self.polygon(1e-3);
        let n = pts.len();
        let mut nrm = DVec3::ZERO;
        for i in 0..n {
            let p = pts[i];
            let q = pts[(i + 1) % n];
            nrm += p.cross(q);
        }
        nrm
    }

    pub fn is_closed(&self) -> bool {
        match (self.segments.first(), self.segments.last()) {
            (Some(a), Some(b)) => b.end().distance(a.start()) < 1e-6,
            _ => false,
        }
    }
}

/// A planar region: outer loop plus holes, all lying in `plane`.
#[derive(Clone, Debug, PartialEq)]
pub struct Region {
    pub plane: Frame,
    pub outer: Loop3,
    pub holes: Vec<Loop3>,
}

impl Region {
    /// Orients loops so the outer loop is counter-clockwise about the plane
    /// normal and holes clockwise.
    pub fn normalized(mut self) -> Region {
        let n = self.plane.z;
        if self.outer.normal().dot(n) < 0.0 {
            self.outer = self.outer.reversed();
        }
        for h in self.holes.iter_mut() {
            if h.normal().dot(n) > 0.0 {
                *h = h.reversed();
            }
        }
        self
    }

    /// Region of a planar face of a body.
    pub fn from_face(body: &Body, f: FaceId) -> Option<Region> {
        let face = body.face(f);
        let ductile_geom::Surface::Plane { frame } = &face.surface else { return None };
        let plane = if face.reversed { frame.flipped() } else { *frame };
        let mut loops = Vec::new();
        for l in body.face_loops(f) {
            let segs = l
                .coedges
                .iter()
                .map(|c| {
                    let e = body.edge(c.edge);
                    Segment { curve: e.curve.clone(), t0: e.t0, t1: e.t1, reversed: c.reversed }
                })
                .collect();
            loops.push(Loop3 { segments: segs });
        }
        if loops.is_empty() {
            return None;
        }
        let outer = loops.remove(0);
        Some(Region { plane, outer, holes: loops })
    }

    /// Builds regions from an unordered set of closed loops lying in one
    /// plane: loops nested inside others become holes (even-odd nesting).
    pub fn from_loops(plane: Frame, loops: Vec<Loop3>) -> Vec<Region> {
        use ductile_geom::poly2::{point_in_polygon, signed_area};
        use ductile_geom::DVec2;
        let polys: Vec<Vec<DVec2>> = loops
            .iter()
            .map(|l| l.polygon(1e-3).into_iter().map(|p| plane.to_local(p).truncate()).collect())
            .collect();
        let areas: Vec<f64> = polys.iter().map(|p| signed_area(p).abs()).collect();
        // depth = number of loops containing this loop.
        let depth: Vec<usize> = (0..loops.len())
            .map(|i| {
                (0..loops.len())
                    .filter(|&j| j != i && areas[j] > areas[i] && !polys[i].is_empty() && point_in_polygon(polys[i][0], &polys[j]))
                    .count()
            })
            .collect();
        let mut regions = Vec::new();
        for i in 0..loops.len() {
            if depth[i] % 2 != 0 {
                continue;
            }
            let mut holes = Vec::new();
            for j in 0..loops.len() {
                if depth[j] == depth[i] + 1 && point_in_polygon(polys[j][0], &polys[i]) {
                    holes.push(loops[j].clone());
                }
            }
            regions.push(Region { plane, outer: loops[i].clone(), holes }.normalized());
        }
        regions
    }
}

/// Builds a wire body from segments; segment ends closer than `1e-7`
/// share a vertex.
pub fn wire_from_segments(segments: &[Segment]) -> Body {
    let mut b = Body::new(BodyKind::Wire);
    let mut vert = |b: &mut Body, p: DVec3| -> VertexId {
        match b.vertices.iter().position(|v| v.point.distance(p) < 1e-7) {
            Some(i) => VertexId(i as u32),
            None => b.add_vertex(p),
        }
    };
    for s in segments {
        let (p, q) = (s.curve.point(s.t0), s.curve.point(s.t1));
        let v0 = vert(&mut b, p);
        let v1 = vert(&mut b, q);
        b.add_edge(s.curve.clone(), s.t0, s.t1, v0, v1);
    }
    b
}

/// Chains the edges of a body (normally a wire) into maximal paths through
/// vertices of degree two. Closed chains come back with `is_closed()`.
pub fn wire_chains(b: &Body) -> Vec<Loop3> {
    let ne = b.edges.len();
    let mut incident: Vec<Vec<usize>> = vec![Vec::new(); b.vertices.len()];
    for (i, e) in b.edges.iter().enumerate() {
        incident[e.v0.idx()].push(i);
        if e.v1 != e.v0 {
            incident[e.v1.idx()].push(i);
        }
    }
    let mut used = vec![false; ne];
    let mut out = Vec::new();
    let seg = |i: usize, rev: bool| {
        let e = &b.edges[i];
        Segment { curve: e.curve.clone(), t0: e.t0, t1: e.t1, reversed: rev }
    };
    // Open chains start at vertices not of degree two; then closed loops.
    let starts: Vec<usize> = (0..b.vertices.len()).filter(|v| incident[*v].len() != 2).chain(0..b.vertices.len()).collect();
    for v0 in starts {
        while let Some(&first) = incident[v0].iter().find(|e| !used[**e]) {
            let mut segs = Vec::new();
            let (mut e, mut v) = (first, v0);
            loop {
                used[e] = true;
                let ed = &b.edges[e];
                let rev = ed.v0.idx() != v;
                segs.push(seg(e, rev));
                v = if rev { ed.v0.idx() } else { ed.v1.idx() };
                if incident[v].len() != 2 {
                    break;
                }
                match incident[v].iter().find(|x| !used[**x]) {
                    Some(&n) => e = n,
                    None => break,
                }
            }
            out.push(Loop3 { segments: segs });
        }
    }
    out
}

/// Plane through planar loops, with the normal of the first loop; `None` if
/// the loops are not coplanar.
pub fn loops_plane(loops: &[Loop3]) -> Option<Frame> {
    let first = loops.first()?;
    let pts: Vec<DVec3> = loops.iter().flat_map(|l| l.polygon(1e-3)).collect();
    let mut n = first.normal();
    if n.length() < 1e-12 {
        return None;
    }
    n = n.normalize();
    let c = pts.iter().copied().sum::<DVec3>() / pts.len() as f64;
    let size = pts.iter().map(|p| p.distance(c)).fold(0.0, f64::max).max(1e-9);
    if pts.iter().any(|p| (*p - c).dot(n).abs() > 1e-6 * size.max(1.0)) {
        return None;
    }
    Some(Frame::new(c, n, ductile_geom::any_perpendicular(n)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chains_rectangle_and_open_polyline() {
        let p = [DVec3::ZERO, DVec3::new(2.0, 0.0, 0.0), DVec3::new(2.0, 1.0, 0.0), DVec3::new(0.0, 1.0, 0.0)];
        let mut segs: Vec<Segment> = (0..4).map(|i| Segment::line(p[i], p[(i + 1) % 4])).collect();
        segs.swap(1, 3);
        segs.push(Segment::line(DVec3::new(5.0, 0.0, 0.0), DVec3::new(6.0, 0.0, 0.0)));
        segs.push(Segment::line(DVec3::new(6.0, 1.0, 0.0), DVec3::new(6.0, 0.0, 0.0)));
        let w = wire_from_segments(&segs);
        assert_eq!(w.vertices.len(), 7);
        let chains = wire_chains(&w);
        assert_eq!(chains.len(), 2);
        let closed: Vec<&Loop3> = chains.iter().filter(|c| c.is_closed()).collect();
        assert_eq!(closed.len(), 1);
        assert_eq!(closed[0].segments.len(), 4);
        assert!((closed[0].normal().length() - 4.0).abs() < 1e-9);
        let open = chains.iter().find(|c| !c.is_closed()).unwrap();
        assert_eq!(open.segments.len(), 2);
        for w2 in open.segments.windows(2) {
            assert!(w2[0].end().distance(w2[1].start()) < 1e-9);
        }
        let plane = loops_plane(&[closed[0].clone()]).unwrap();
        assert!(plane.z.z.abs() > 0.999);
    }
}
