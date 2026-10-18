// SPDX-License-Identifier: GPL-3.0-only
//! Sweep: a closed profile carried along a path curve.

use super::boolean::{boolean, BoolOp};
use super::fillet::half_space;
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
/// profile, with the ends capped by exact copies. Where the path turns
/// sharply, the pieces on either side meet in a mitre: each is extended
/// straight past the corner and cut by the plane bisecting the turn.
pub fn sweep(profile: &Loop3, path: &[Segment]) -> Option<Body> {
    if !profile.is_closed() || path.is_empty() {
        return None;
    }
    // Smooth runs, split where the tangent jumps.
    let mut runs: Vec<&[Segment]> = Vec::new();
    let mut start = 0;
    for i in 1..path.len() {
        if corner_turn(&path[i - 1], &path[i]) > 1f64.to_radians() {
            runs.push(&path[start..i]);
            start = i;
        }
    }
    runs.push(&path[start..]);
    if runs.len() == 1 {
        return sweep_run(profile, path).map(|(b, _)| b);
    }
    // Profile reach about the path, which bounds how far a mitre leans.
    let reach = {
        let c = path[0].start();
        profile.polygon(1e-2).iter().map(|p| p.distance(c)).fold(0.0, f64::max).max(1e-9)
    };
    let mut out: Option<Body> = None;
    let mut prof = profile.clone();
    for (k, run) in runs.iter().enumerate() {
        let (body, end_m) = sweep_run(&prof, run)?;
        out = Some(match out {
            None => body,
            Some(o) => boolean(&o, &body, BoolOp::Union).ok()?,
        });
        let Some(next) = runs.get(k + 1) else { break };
        // The joint: both legs carried straight past the corner and cut by
        // the plane bisecting the turn. Each leg's own overlap past that
        // plane lies inside the other half of the joint.
        let end_prof = transform_loop(&prof, &end_m);
        let c = run.last().unwrap().end();
        let (d0, d1) = (end_dir(run.last().unwrap()), start_dir(&next[0]));
        let turn = d0.dot(d1).clamp(-1.0, 1.0).acos();
        let m = (d0 + d1).normalize_or_zero();
        if turn > 170f64.to_radians() || m == DVec3::ZERO {
            return None;
        }
        let axis = d0.cross(d1);
        let r = if axis.length() < 1e-12 {
            DMat4::IDENTITY
        } else {
            DMat4::from_translation(c) * DMat4::from_axis_angle(axis.normalize(), turn) * DMat4::from_translation(-c)
        };
        let next_prof = transform_loop(&end_prof, &r);
        let len = mitre_length(reach, turn);
        let size = 4.0 * (reach + len);
        let a = boolean(&extrude_loop(&end_prof, d0 * len)?, &half_space(c, m, size), BoolOp::Intersection).ok()?;
        let b = boolean(&extrude_loop(&next_prof, -d1 * len)?, &half_space(c, -m, size), BoolOp::Intersection).ok()?;
        let joint = boolean(&a, &b, BoolOp::Union).ok()?;
        out = Some(boolean(out.as_ref()?, &joint, BoolOp::Union).ok()?);
        prof = next_prof;
    }
    out
}

/// How far past a corner a piece must reach to be cut by the mitre plane.
fn mitre_length(reach: f64, turn: f64) -> f64 {
    2.0 * reach * (turn / 2.0).tan() + reach
}

fn start_dir(s: &Segment) -> DVec3 {
    s.tangent_at(if s.reversed { s.t1 } else { s.t0 }).normalize()
}

fn end_dir(s: &Segment) -> DVec3 {
    s.tangent_at(if s.reversed { s.t0 } else { s.t1 }).normalize()
}

fn corner_turn(a: &Segment, b: &Segment) -> f64 {
    end_dir(a).dot(start_dir(b)).clamp(-1.0, 1.0).acos()
}

fn transform_loop(l: &Loop3, m: &DMat4) -> Loop3 {
    Loop3 { segments: l.segments.iter().map(|s| Segment { curve: s.curve.transformed(m), ..s.clone() }).collect() }
}

/// Straight extrusion of a closed planar loop.
fn extrude_loop(profile: &Loop3, v: DVec3) -> Option<Body> {
    let o = profile.segments[0].start();
    let x = profile.segments[0].end() - o;
    let x = if x.length() > 1e-9 { x } else { ductile_geom::any_perpendicular(profile.normal()) };
    let plane = Frame::new(o, profile.normal(), x);
    let region = Region { plane, outer: profile.clone(), holes: vec![] }.normalized();
    super::extrude::extrude_region(&region, v)
}

/// Sweeps along one smooth run, returning the solid and the transform that
/// carries the start profile onto the end section.
/// A round tube of `radius` along `path`. A closed path makes a ring: it is
/// swept as two halves, cut part-way along segments where it is smooth,
/// whose end caps meet face to face and vanish in their union.
pub fn pipe(path: &Loop3, radius: f64) -> Option<Body> {
    let segs = &path.segments;
    if segs.is_empty() || radius <= 0.0 {
        return None;
    }
    let tube = |segs: &[Segment]| {
        let s0 = &segs[0];
        let t = s0.tangent_at(if s0.reversed { s0.t1 } else { s0.t0 }).normalize_or_zero();
        if t == DVec3::ZERO {
            return None;
        }
        let frame = Frame::new(s0.start(), t, t.any_orthonormal_vector());
        let circle = Segment::new(Curve3::Circle { frame, radius }, 0.0, std::f64::consts::TAU);
        sweep(&Loop3 { segments: vec![circle] }, segs)
    };
    if !path.is_closed() {
        return tube(segs);
    }
    // Cut points (segment, fraction along it in traversal order).
    let n = segs.len();
    let (a, b) = if n == 1 { ((0, 0.25), (0, 0.75)) } else { ((0, 0.5), (n / 2, 0.5)) };
    let mut first = Vec::new();
    let mut second = Vec::new();
    let (a0, a1) = split_segment(&segs[a.0], a.1);
    if a.0 == b.0 {
        let (m, b1) = split_segment(&a1, (b.1 - a.1) / (1.0 - a.1));
        first.push(m);
        second.push(b1);
        second.push(a0);
    } else {
        let (b0, b1) = split_segment(&segs[b.0], b.1);
        first.push(a1);
        first.extend(segs[a.0 + 1..b.0].iter().cloned());
        first.push(b0);
        second.push(b1);
        second.extend(segs[b.0 + 1..].iter().cloned());
        second.push(a0);
    }
    boolean(&tube(&first)?, &tube(&second)?, BoolOp::Union).ok()
}

/// `s` cut at fraction `f` of its parameter range, in traversal order.
fn split_segment(s: &Segment, f: f64) -> (Segment, Segment) {
    let f = if s.reversed { 1.0 - f } else { f };
    let tm = s.t0 + (s.t1 - s.t0) * f;
    let lo = Segment { t1: tm, ..s.clone() };
    let hi = Segment { t0: tm, ..s.clone() };
    if s.reversed {
        (hi, lo)
    } else {
        (lo, hi)
    }
}

fn sweep_run(profile: &Loop3, path: &[Segment]) -> Option<(Body, DMat4)> {
    if let [s] = path {
        match &s.curve {
            Curve3::Line { .. } => {
                let v = s.end() - s.start();
                return Some((extrude_loop(profile, v)?, DMat4::from_translation(v)));
            }
            Curve3::Circle { frame, .. } => {
                let axis = if s.reversed { -frame.z } else { frame.z };
                let angle = s.t1 - s.t0;
                let m = DMat4::from_translation(frame.origin) * DMat4::from_axis_angle(axis, angle) * DMat4::from_translation(-frame.origin);
                return Some((super::revolve::revolve(profile, frame.origin, axis, angle)?, m));
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
        loops.push(transform_loop(&profile, &m));
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
    Some((skin(&loops, &params), frame_at(fine) * to_start))
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

    #[test]
    fn pipes_along_open_and_closed_paths() {
        let r = 0.2;
        let a = PI * r * r;
        let line = Loop3 { segments: vec![Segment::line(DVec3::ZERO, DVec3::new(3.0, 0.0, 0.0))] };
        let p = pipe(&line, r).unwrap();
        assert!((vol(&p) - a * 3.0).abs() < 1e-3, "line {}", vol(&p));
        // A ring: the torus of tube radius r.
        let ring = pipe(&circle(Frame::WORLD, 1.5), r).unwrap();
        assert_eq!(open_edges(&ring), 0);
        assert!((vol(&ring) - a * 2.0 * PI * 1.5).abs() < 2e-3, "ring {}", vol(&ring));
        // A rectangle: mitred corners keep the volume at area x length.
        let c = [DVec3::ZERO, DVec3::new(4.0, 0.0, 0.0), DVec3::new(4.0, 2.0, 0.0), DVec3::new(0.0, 2.0, 0.0)];
        let rect = Loop3 { segments: (0..4).map(|i| Segment::line(c[i], c[(i + 1) % 4])).collect() };
        let frame = pipe(&rect, r).unwrap();
        assert_eq!(open_edges(&frame), 0);
        assert!((vol(&frame) - a * 12.0).abs() < 2e-3, "rectangle {}", vol(&frame));
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
    fn mitred_l_and_u_paths() {
        // A centred profile mitred at corners keeps volume = area x centreline.
        let prof = circle(Frame::new(DVec3::ZERO, DVec3::X, DVec3::Y), 0.5);
        let (a, b, c) = (DVec3::ZERO, DVec3::new(4.0, 0.0, 0.0), DVec3::new(4.0, 3.0, 0.0));
        let body = sweep(&prof, &[Segment::line(a, b), Segment::line(b, c)]).unwrap();
        assert_eq!(open_edges(&body), 0);
        // Two tube faces and two end caps; the mitre leaves no face behind.
        assert_eq!(body.faces.len(), 4, "{}", body.faces.len());
        let expect = PI * 0.25 * 7.0;
        assert!((vol(&body) - expect).abs() < 2e-3 * expect, "{} vs {expect}", vol(&body));
        // A square profile around a U with a 120 degree corner.
        let sq = Loop3 {
            segments: vec![
                Segment::line(DVec3::new(0.0, -0.5, -0.5), DVec3::new(0.0, 0.5, -0.5)),
                Segment::line(DVec3::new(0.0, 0.5, -0.5), DVec3::new(0.0, 0.5, 0.5)),
                Segment::line(DVec3::new(0.0, 0.5, 0.5), DVec3::new(0.0, -0.5, 0.5)),
                Segment::line(DVec3::new(0.0, -0.5, 0.5), DVec3::new(0.0, -0.5, -0.5)),
            ],
        };
        let d = DVec3::new(-2.0, 2.0 * 3f64.sqrt(), 0.0);
        let path = [Segment::line(a, b), Segment::line(b, b + d), Segment::line(b + d, b + d - DVec3::new(4.0, 0.0, 0.0))];
        let body = sweep(&sq, &path).unwrap();
        assert_eq!(open_edges(&body), 0);
        // Top and bottom are one face each; each leg keeps its two walls.
        assert_eq!(body.faces.len(), 10, "{}", body.faces.len());
        let expect = 1.0 * (4.0 + 4.0 + 4.0);
        assert!((vol(&body) - expect).abs() < 1e-3 * expect, "{} vs {expect}", vol(&body));
    }

    #[test]
    fn mitre_between_line_and_arc() {
        // A line into a quarter arc that starts off at right angles.
        let prof = circle(Frame::new(DVec3::ZERO, DVec3::X, DVec3::Y), 0.3);
        let line = Segment::line(DVec3::ZERO, DVec3::new(3.0, 0.0, 0.0));
        // Centred at (5, 0), radius 2, from (3, 0) heading +Y round to (5, 2).
        let arc = Segment::new(Curve3::Circle { frame: Frame::new(DVec3::new(5.0, 0.0, 0.0), DVec3::Z, DVec3::X), radius: 2.0 }, PI / 2.0, PI).flipped();
        let body = sweep(&prof, &[line, arc]).unwrap();
        assert_eq!(open_edges(&body), 0);
        let expect = PI * 0.09 * (3.0 + PI);
        assert!((vol(&body) - expect).abs() < 3e-3 * expect, "{} vs {expect}", vol(&body));
    }

    #[test]
    fn pipe_bent_through_two_planes() {
        // The cut plane at the first corner runs through the profile's seam.
        let bends = [DVec3::new(-16.0, -4.0, 0.6), DVec3::new(-16.0, -10.0, 0.6), DVec3::new(-10.0, -10.0, 0.6), DVec3::new(-10.0, -10.0, 5.0)];
        let pipe = circle(Frame::new(bends[0], bends[1] - bends[0], DVec3::Z), 0.5);
        let legs: Vec<Segment> = bends.windows(2).map(|w| Segment::line(w[0], w[1])).collect();
        let body = sweep(&pipe, &legs).unwrap();
        assert_eq!(open_edges(&body), 0);
        assert_eq!(body.faces.len(), 5);
        let expect = PI * 0.25 * 16.4;
        assert!((vol(&body) - expect).abs() < 2e-3 * expect, "{} vs {expect}", vol(&body));
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
