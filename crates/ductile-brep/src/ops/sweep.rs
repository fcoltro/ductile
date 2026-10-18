// SPDX-License-Identifier: GPL-3.0-only
//! Sweep: a closed profile carried along a path curve.

use super::loft::{directed_nurbs, skin};
use crate::profile::{Loop3, Region, Segment};
use crate::topo::Body;
use ductile_geom::{Curve3, DMat4, DVec3, Frame, NurbsCurve};

/// Sweeps a closed planar profile along an open path (a chain of segments,
/// each starting where the previous ends). The profile keeps its placement
/// relative to the path's start and turns with a rotation-minimizing frame,
/// so it does not twist about the path.
///
/// A straight path is an exact extrusion and a single circular arc an exact
/// revolution; other paths skin the side through closely spaced copies of the
/// profile, with the ends capped by exact copies.
pub fn sweep(profile: &Loop3, path: &[Segment]) -> Option<Body> {
    if !profile.is_closed() || path.is_empty() {
        return None;
    }
    if let [s] = path {
        match &s.curve {
            Curve3::Line { .. } => {
                let plane = Frame::new(profile.segments[0].start(), profile.normal(), profile.segments[0].start() - profile.segments[0].end());
                let region = Region { plane, outer: profile.clone(), holes: vec![] }.normalized();
                return super::extrude::extrude_region(&region, s.end() - s.start());
            }
            Curve3::Circle { frame, .. } => {
                let axis = if s.reversed { -frame.z } else { frame.z };
                return super::revolve::revolve(profile, frame.origin, axis, (s.t1 - s.t0).abs());
            }
            _ => {}
        }
    }
    let rail = NurbsCurve::join(&path.iter().map(directed_nurbs).collect::<Vec<_>>());
    let (a, b) = rail.domain();
    // Section count from how much the path turns and how long it is
    // compared with the profile.
    let size = {
        let pts = profile.polygon(1e-2);
        let c = pts.iter().copied().sum::<DVec3>() / pts.len() as f64;
        pts.iter().map(|p| p.distance(c)).fold(0.0, f64::max).max(1e-9)
    };
    let fine = 512;
    let ts: Vec<f64> = (0..=fine).map(|i| a + (b - a) * i as f64 / fine as f64).collect();
    let xs: Vec<DVec3> = ts.iter().map(|t| rail.point(*t)).collect();
    let tans: Vec<DVec3> = ts.iter().map(|t| rail.tangent(*t).normalize()).collect();
    let turning: f64 = tans.windows(2).map(|w| w[0].dot(w[1]).clamp(-1.0, 1.0).acos()).sum();
    let length: f64 = xs.windows(2).map(|w| w[0].distance(w[1])).sum();
    let sections = ((turning / 0.08).max(length / size).ceil() as usize).clamp(4, 128);
    // Rotation-minimizing frames by double reflection (Wang et al. 2008).
    let up = {
        let n = profile.normal();
        let r = n - tans[0] * n.dot(tans[0]);
        if r.length() > 1e-6 * n.length() {
            r.normalize()
        } else {
            ductile_geom::any_perpendicular(tans[0])
        }
    };
    let mut rs = vec![up];
    for i in 0..fine {
        let v1 = xs[i + 1] - xs[i];
        let c1 = v1.dot(v1);
        if c1 < 1e-24 {
            rs.push(rs[i]);
            continue;
        }
        let rl = rs[i] - v1 * (2.0 / c1 * v1.dot(rs[i]));
        let tl = tans[i] - v1 * (2.0 / c1 * v1.dot(tans[i]));
        let v2 = tans[i + 1] - tl;
        let c2 = v2.dot(v2);
        let r = if c2 < 1e-24 { rl } else { rl - v2 * (2.0 / c2 * v2.dot(rl)) };
        rs.push((r - tans[i + 1] * r.dot(tans[i + 1])).normalize());
    }
    let frame_at = |i: usize| {
        let z = tans[i];
        let x = rs[i];
        DMat4::from_cols(x.extend(0.0), z.cross(x).extend(0.0), z.extend(0.0), xs[i].extend(1.0))
    };
    let to_start = frame_at(0).inverse();
    // Profile counter-clockwise about the path direction.
    let profile = if profile.normal().dot(tans[0]) < 0.0 { profile.reversed() } else { profile.clone() };
    let mut loops = Vec::with_capacity(sections + 1);
    for k in 0..=sections {
        let i = k * fine / sections;
        let m = frame_at(i) * to_start;
        loops.push(Loop3 {
            segments: profile.segments.iter().map(|s| Segment { curve: s.curve.transformed(&m), ..s.clone() }).collect(),
        });
    }
    // Chord-length section parameters along the path.
    let mut acc = vec![0.0];
    for k in 1..=sections {
        let (i0, i1) = (k.saturating_sub(1) * fine / sections, k * fine / sections);
        let d: f64 = xs[i0..=i1].windows(2).map(|w| w[0].distance(w[1])).sum();
        acc.push(acc[k - 1] + d);
    }
    let total = acc[sections].max(1e-12);
    let params: Vec<f64> = acc.iter().map(|x| x / total).collect();
    Some(skin(&loops, &params))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ops::boolean::open_edges;
    use crate::tess::{tessellate, TessOptions};
    use std::f64::consts::PI;

    fn vol(b: &Body) -> f64 {
        tessellate(b, &TessOptions { chord_tol: 1e-4, angle_tol: 0.05 }).volume()
    }

    fn circle(frame: Frame, r: f64) -> Loop3 {
        Loop3 { segments: vec![Segment::new(Curve3::Circle { frame, radius: r }, 0.0, 2.0 * PI)] }
    }

    #[test]
    fn straight_and_arc_paths_are_exact() {
        let prof = circle(Frame::WORLD, 1.0);
        let b = sweep(&prof, &[Segment::line(DVec3::ZERO, DVec3::new(0.0, 0.0, 3.0))]).unwrap();
        assert!((vol(&b) - 3.0 * PI).abs() < 2e-3, "{}", vol(&b));
        // Quarter torus: the profile at (5, 0, 0) in the XZ plane, path around Z.
        let prof = circle(Frame::new(DVec3::new(5.0, 0.0, 0.0), DVec3::Y, DVec3::X), 1.0);
        let path = Segment::new(Curve3::Circle { frame: Frame::WORLD, radius: 5.0 }, 0.0, PI / 2.0);
        let b = sweep(&prof, &[path]).unwrap();
        assert_eq!(open_edges(&b), 0);
        let expect = PI * 1.0 * (PI / 2.0 * 5.0);
        assert!((vol(&b) - expect).abs() < 1e-3 * expect, "{}", vol(&b));
    }

    #[test]
    fn tube_along_an_s_curve() {
        // A tube's volume is its cross-section times the path length
        // (Pappus) while the path's curvature radius exceeds the tube radius.
        let pts = [DVec3::ZERO, DVec3::new(4.0, 2.0, 1.0), DVec3::new(8.0, -2.0, 2.0), DVec3::new(12.0, 0.0, 3.0)];
        let path = NurbsCurve::interpolate(&pts, 3);
        let (t0, t1) = path.domain();
        let tan = path.tangent(t0).normalize();
        let prof = circle(Frame::new(DVec3::ZERO, tan, DVec3::Z), 0.5);
        let len = path.length();
        let b = sweep(&prof, &[Segment::new(Curve3::Nurbs(path), t0, t1)]).unwrap();
        assert_eq!(open_edges(&b), 0);
        // Two end circles and one seam; the inner sections leave no edges.
        assert_eq!(b.edges.len(), 3);
        let expect = PI * 0.25 * len;
        assert!((vol(&b) - expect).abs() < 2e-3 * expect, "{} vs {expect}", vol(&b));
    }
}
