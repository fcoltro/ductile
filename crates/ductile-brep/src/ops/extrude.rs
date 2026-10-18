// SPDX-License-Identifier: GPL-3.0-only
//! Linear extrusion of planar regions into solids.

use crate::profile::{Loop3, Region, Segment};
use crate::topo::{Body, BodyKind, Coedge, EdgeId, VertexId};
use ductile_geom::{Curve3, DMat4, DVec3, Frame, NurbsSurface, Surface};

/// Extrudes `region` along `dir` (length and direction). The region's
/// loops are re-oriented as needed, so `dir` may point to either side.
pub fn extrude_region(region: &Region, dir: DVec3) -> Option<Body> {
    if dir.length() < 1e-12 {
        return None;
    }
    let mut region = region.clone();
    // Orient the region so that its normal points along the extrusion.
    if region.plane.z.dot(dir) < 0.0 {
        region.plane = region.plane.flipped();
    }
    let region = region.normalized();
    let d = dir;
    let shift = DMat4::from_translation(d);
    let mut b = Body::new(BodyKind::Solid);
    let mut bottom_loops: Vec<Vec<Coedge>> = Vec::new();
    let mut top_loops: Vec<Vec<Coedge>> = Vec::new();
    for lp in std::iter::once(&region.outer).chain(region.holes.iter()) {
        let (bl, tl) = extrude_loop(&mut b, lp, d, &shift);
        bottom_loops.push(bl);
        top_loops.push(tl);
    }
    let bottom_plane = Frame { ..region.plane.flipped() };
    let top_plane = Frame { origin: region.plane.origin + d, ..region.plane };
    // Bottom: reversed loops (clockwise about +d == counter-clockwise about -d).
    let bottom: Vec<Vec<Coedge>> = bottom_loops
        .into_iter()
        .map(|l| l.into_iter().rev().map(|c| Coedge::new(c.edge, !c.reversed)).collect())
        .collect();
    b.add_face(Surface::Plane { frame: bottom_plane }, false, bottom);
    b.add_face(Surface::Plane { frame: top_plane }, false, top_loops);
    Some(b)
}

/// Creates bottom, top and side topology of one loop. Returns the bottom and
/// top coedge loops oriented counter-clockwise about `d` (for outer loops).
fn extrude_loop(b: &mut Body, lp: &Loop3, d: DVec3, shift: &DMat4) -> (Vec<Coedge>, Vec<Coedge>) {
    let n = lp.segments.len();
    // Vertices at the start of each segment.
    let vb: Vec<VertexId> = lp.segments.iter().map(|s| b.add_vertex(s.start())).collect();
    let vt: Vec<VertexId> = lp.segments.iter().map(|s| b.add_vertex(s.start() + d)).collect();
    // Vertical edges.
    let verticals: Vec<EdgeId> = (0..n).map(|i| b.add_line_edge(vb[i], vt[i])).collect();
    let mut bottom = Vec::new();
    let mut top = Vec::new();
    for (i, s) in lp.segments.iter().enumerate() {
        let j = (i + 1) % n;
        // Edge curves are stored with increasing parameters; orientation is in the coedge.
        let (v0b, v1b) = if s.reversed { (vb[j], vb[i]) } else { (vb[i], vb[j]) };
        let (v0t, v1t) = if s.reversed { (vt[j], vt[i]) } else { (vt[i], vt[j]) };
        let eb = b.add_edge(s.curve.clone(), s.t0, s.t1, v0b, v1b);
        let et = b.add_edge(s.curve.transformed(shift), s.t0, s.t1, v0t, v1t);
        bottom.push(Coedge::new(eb, s.reversed));
        top.push(Coedge::new(et, s.reversed));
        let (surface, reversed) = side_surface(s, d);
        let side_loop = vec![
            Coedge::new(eb, s.reversed),
            Coedge::new(verticals[j], false),
            Coedge::new(et, !s.reversed),
            Coedge::new(verticals[i], true),
        ];
        b.add_face(surface, reversed, vec![side_loop]);
    }
    (bottom, top)
}

/// Surface swept by segment `s` moving along `d`, and whether the face must
/// be reversed so its normal points outward (`tangent × d`).
pub(crate) fn side_surface(s: &Segment, d: DVec3) -> (Surface, bool) {
    match &s.curve {
        Curve3::Line { origin, dir } => {
            let t = if s.reversed { -*dir } else { *dir };
            let n = t.cross(d);
            let frame = Frame::new(*origin, n, t);
            (Surface::Plane { frame }, false)
        }
        Curve3::Circle { frame, radius } if frame.z.cross(d.normalize()).length() < 1e-9 => {
            let ccw_about_d = (frame.z.dot(d) > 0.0) != s.reversed;
            (Surface::Cylinder { frame: *frame, radius: *radius }, !ccw_about_d)
        }
        other => {
            let n = other.to_nurbs(s.t0, s.t1);
            let surf = NurbsSurface::extrusion(&n, d);
            // Su × Sv = C' × d is outward for forward traversal.
            (Surface::Nurbs(surf), s.reversed)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::profile::Loop3;
    use crate::tess::{tessellate, TessOptions};
    use std::f64::consts::{PI, TAU};

    fn rect(w: f64, h: f64) -> Loop3 {
        let p = [DVec3::ZERO, DVec3::new(w, 0.0, 0.0), DVec3::new(w, h, 0.0), DVec3::new(0.0, h, 0.0)];
        Loop3 { segments: (0..4).map(|i| Segment::line(p[i], p[(i + 1) % 4])).collect() }
    }

    #[test]
    fn extrude_rectangle_with_hole() {
        let circle = Loop3 {
            segments: vec![Segment::new(
                Curve3::Circle { frame: Frame { origin: DVec3::new(2.0, 1.5, 0.0), ..Frame::WORLD }, radius: 0.5 },
                0.0,
                TAU,
            )],
        };
        let r = Region { plane: Frame::WORLD, outer: rect(4.0, 3.0), holes: vec![circle] };
        for dir in [DVec3::new(0.0, 0.0, 2.0), DVec3::new(0.0, 0.0, -2.0)] {
            let b = extrude_region(&r, dir).unwrap();
            let m = tessellate(&b, &TessOptions { chord_tol: 1e-4, angle_tol: 0.05 });
            let expect = (12.0 - PI * 0.25) * 2.0;
            assert!((m.volume() - expect).abs() < 1e-3, "volume {} vs {expect}", m.volume());
        }
    }

    #[test]
    fn extrude_slanted_spline() {
        use ductile_geom::NurbsCurve;
        let pts = [DVec3::ZERO, DVec3::new(2.0, -1.0, 0.0), DVec3::new(4.0, 0.0, 0.0)];
        let spline = NurbsCurve::interpolate(&pts, 2);
        let segs = vec![
            Segment::new(Curve3::Nurbs(spline), 0.0, 1.0),
            Segment::line(DVec3::new(4.0, 0.0, 0.0), DVec3::new(4.0, 2.0, 0.0)),
            Segment::line(DVec3::new(4.0, 2.0, 0.0), DVec3::new(0.0, 2.0, 0.0)),
            Segment::line(DVec3::new(0.0, 2.0, 0.0), DVec3::ZERO),
        ];
        let r = Region { plane: Frame::WORLD, outer: Loop3 { segments: segs }, holes: vec![] };
        let b = extrude_region(&r, DVec3::new(0.5, 0.0, 3.0)).unwrap();
        let m = tessellate(&b, &TessOptions { chord_tol: 1e-4, angle_tol: 0.05 });
        // Volume = area * height (oblique prism).
        let area_poly: f64 = {
            let poly = r.outer.polygon(1e-5);
            let mut a = 0.0;
            for i in 0..poly.len() {
                let p = poly[i];
                let q = poly[(i + 1) % poly.len()];
                a += p.x * q.y - q.x * p.y;
            }
            a.abs() * 0.5
        };
        assert!((m.volume() - area_poly * 3.0).abs() < 1e-2, "{} vs {}", m.volume(), area_poly * 3.0);
    }
}
