// SPDX-License-Identifier: GPL-3.0-only
//! Profiles: closed chains of curve segments used as input to sweeps.

use crate::topo::{Body, FaceId};
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
