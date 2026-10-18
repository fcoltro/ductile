// SPDX-License-Identifier: GPL-3.0-only
//! Operations on curve chains: rounding corners and offsetting.

use crate::profile::Segment;
use ductile_geom::{Curve3, DVec3, Frame, NurbsCurve};

/// Point and unit tangent of a segment at `s` in [0, 1] along its
/// traversal direction.
fn point_tangent(seg: &Segment, s: f64) -> (DVec3, DVec3) {
    let t = param(seg, s);
    (seg.curve.point(t), seg.tangent_at(t).normalize())
}

fn param(seg: &Segment, s: f64) -> f64 {
    if seg.reversed {
        seg.t1 + (seg.t0 - seg.t1) * s
    } else {
        seg.t0 + (seg.t1 - seg.t0) * s
    }
}

/// The part of `seg` between fractions `a` and `b` of its traversal.
fn trimmed(seg: &Segment, a: f64, b: f64) -> Segment {
    let (ta, tb) = (param(seg, a), param(seg, b));
    let (t0, t1) = if seg.reversed { (tb, ta) } else { (ta, tb) };
    Segment { t0, t1, ..seg.clone() }
}

/// Rounds the corners of a chain of segments with arcs of radius `r`.
/// `closed` chains also round the corner between the last and first
/// segment. Only corners at points in `at` are rounded when it is given.
/// Corners that are already smooth, or too tight for the radius, stay.
pub fn fillet_corners(segs: &[Segment], closed: bool, r: f64, at: Option<&[DVec3]>) -> Vec<Segment> {
    let n = segs.len();
    if n == 0 || r <= 0.0 {
        return segs.to_vec();
    }
    // Fractions of each segment kept: [start, end].
    let mut keep: Vec<(f64, f64)> = vec![(0.0, 1.0); n];
    let mut arcs: Vec<Option<Segment>> = vec![None; n];
    let corners = if closed { n } else { n - 1 };
    for i in 0..corners {
        let j = (i + 1) % n;
        let (a, b) = (&segs[i], &segs[j]);
        let corner = a.end();
        if let Some(pts) = at {
            if !pts.iter().any(|p| p.distance(corner) < 1e-6) {
                continue;
            }
        }
        let (_, da) = point_tangent(a, 1.0);
        let (_, db) = point_tangent(b, 0.0);
        let axis = da.cross(db);
        if axis.length() < 1e-6 {
            continue;
        }
        let z = axis.normalize();
        // Centre at distance r inside both curves: c = A(sa) + r (z x tA) =
        // B(sb) + r (z x tB), solved by Newton from the straight-line guess.
        let half = 0.5 * (-da).angle_between(db);
        let back = r / half.tan();
        let (la, lb) = (seg_len(a), seg_len(b));
        let mut sa = (1.0 - back / la).clamp(keep[i].0, 1.0);
        let mut sb = (back / lb).clamp(0.0, 1.0);
        let mut ok = false;
        for _ in 0..50 {
            let (pa, ta) = point_tangent(a, sa);
            let (pb, tb) = point_tangent(b, sb);
            let f = (pa + z.cross(ta) * r) - (pb + z.cross(tb) * r);
            if f.length() < 1e-12 * (1.0 + r) {
                ok = true;
                break;
            }
            // Derivatives by finite differences in the corner's plane.
            let h = 1e-7;
            let ca = |s: f64| {
                let (p, t) = point_tangent(a, s);
                p + z.cross(t) * r
            };
            let cb = |s: f64| {
                let (p, t) = point_tangent(b, s);
                p + z.cross(t) * r
            };
            let ja = (ca(sa + h) - ca(sa - h)) / (2.0 * h);
            let jb = -(cb(sb + h) - cb(sb - h)) / (2.0 * h);
            // Least squares on [ja jb] d = -f.
            let (aa, ab, bb) = (ja.dot(ja), ja.dot(jb), jb.dot(jb));
            let det = aa * bb - ab * ab;
            if det.abs() < 1e-300 {
                break;
            }
            let (ra, rb) = (-ja.dot(f), -jb.dot(f));
            sa += (bb * ra - ab * rb) / det;
            sb += (aa * rb - ab * ra) / det;
        }
        if !ok || sa <= keep[i].0 + 1e-9 || sa > 1.0 || sb < 0.0 || sb >= 1.0 - 1e-9 {
            continue;
        }
        let (pa, ta) = point_tangent(a, sa);
        let (pb, _) = point_tangent(b, sb);
        let c = pa + z.cross(ta) * r;
        let x = (pa - c) / r;
        let y = z.cross(x);
        let sweep = (pb - c).dot(y).atan2((pb - c).dot(x));
        if sweep <= 1e-9 {
            continue;
        }
        keep[i].1 = sa;
        keep[j].0 = sb;
        arcs[i] = Some(Segment::new(Curve3::Circle { frame: Frame { origin: c, x, y, z }, radius: r }, 0.0, sweep));
    }
    let mut out = Vec::with_capacity(2 * n);
    for i in 0..n {
        let (s0, s1) = keep[i];
        if s1 - s0 > 1e-9 {
            out.push(if s0 == 0.0 && s1 == 1.0 { segs[i].clone() } else { trimmed(&segs[i], s0, s1) });
        }
        if let Some(arc) = arcs[i].take() {
            out.push(arc);
        }
    }
    out
}

fn seg_len(s: &Segment) -> f64 {
    let pts = s.sample(1e-4);
    pts.windows(2).map(|w| w[0].distance(w[1])).sum::<f64>().max(1e-12)
}

/// Offsets a planar chain by `d` within the plane with normal `normal`
/// (positive to the left of the travel direction). Lines and arcs stay
/// exact, other curves are sampled; neighbouring pieces are extended or
/// trimmed to meet, or joined by an arc around the old corner where they
/// part.
pub fn offset_chain(segs: &[Segment], closed: bool, d: f64, normal: DVec3) -> Vec<Segment> {
    let z = normal.normalize();
    let moved: Vec<Segment> = segs.iter().filter_map(|s| offset_segment(s, d, z)).collect();
    if moved.len() != segs.len() {
        return Vec::new();
    }
    let n = moved.len();
    let mut out: Vec<Segment> = Vec::with_capacity(2 * n);
    let corners = if closed { n } else { n.saturating_sub(1) };
    // Join pieces pairwise: where consecutive offsets cross, trim both to
    // the crossing; where they part, add an arc about the original corner.
    let mut pieces = moved.clone();
    let mut joins: Vec<Option<Segment>> = vec![None; n];
    for i in 0..corners {
        let j = (i + 1) % n;
        let (a, b) = (pieces[i].clone(), pieces[j].clone());
        if a.end().distance(b.start()) < 1e-9 {
            continue;
        }
        let corner = segs[i].end();
        let (_, ta) = point_tangent(&a, 1.0);
        let (_, tb) = point_tangent(&b, 0.0);
        let turn = z.dot(ta.cross(tb));
        // Offsetting to the outside of a turn leaves a gap: bridge it.
        if turn * d < 0.0 {
            let x = (a.end() - corner).normalize();
            let y = z.cross(x);
            let mut sweep = (b.start() - corner).dot(y).atan2((b.start() - corner).dot(x));
            let (fz, fy) = if sweep < 0.0 {
                sweep = -sweep;
                (-z, -y)
            } else {
                (z, y)
            };
            joins[i] = Some(Segment::new(Curve3::Circle { frame: Frame { origin: corner, x, y: fy, z: fz }, radius: d.abs() }, 0.0, sweep));
            continue;
        }
        // Inside of a turn: the offsets overlap; trim them back to where
        // they cross.
        if let Some((sa, sb)) = crossing(&a, &b) {
            pieces[i] = trimmed(&a, 0.0, sa);
            pieces[j] = trimmed(&b, sb, 1.0);
        }
    }
    for i in 0..n {
        out.push(pieces[i].clone());
        if let Some(j) = joins[i].take() {
            out.push(j);
        }
    }
    out
}

/// Fractions where two segments cross (closest approach below 1e-7),
/// searching near the end of `a` and the start of `b`.
fn crossing(a: &Segment, b: &Segment) -> Option<(f64, f64)> {
    let m = 64;
    let mut best = (f64::INFINITY, 1.0, 0.0);
    for i in 0..=m {
        let sa = i as f64 / m as f64;
        let pa = point_tangent(a, sa).0;
        for k in 0..=m {
            let sb = k as f64 / m as f64;
            let dd = pa.distance_squared(point_tangent(b, sb).0);
            if dd < best.0 {
                best = (dd, sa, sb);
            }
        }
    }
    let (_, mut sa, mut sb) = best;
    for _ in 0..50 {
        let (pa, ta) = point_tangent(a, sa);
        let (pb, tb) = point_tangent(b, sb);
        let f = pa - pb;
        if f.length() < 1e-12 {
            break;
        }
        let (la, lb) = (seg_len(a), seg_len(b));
        let (ja, jb) = (ta * la, -tb * lb);
        let (aa, ab, bb) = (ja.dot(ja), ja.dot(jb), jb.dot(jb));
        let det = aa * bb - ab * ab;
        if det.abs() < 1e-300 {
            return None;
        }
        let (ra, rb) = (-ja.dot(f), -jb.dot(f));
        sa = (sa + (bb * ra - ab * rb) / det).clamp(0.0, 1.0);
        sb = (sb + (aa * rb - ab * ra) / det).clamp(0.0, 1.0);
    }
    (point_tangent(a, sa).0.distance(point_tangent(b, sb).0) < 1e-7).then_some((sa, sb))
}

/// One segment offset by `d` to the left of its travel direction about `z`.
fn offset_segment(s: &Segment, d: f64, z: DVec3) -> Option<Segment> {
    match &s.curve {
        Curve3::Line { origin, dir } => {
            let left = z.cross(if s.reversed { -*dir } else { *dir }).normalize();
            Some(Segment { curve: Curve3::Line { origin: *origin + left * d, dir: *dir }, ..s.clone() })
        }
        Curve3::Circle { frame, radius } if frame.z.cross(z).length() < 1e-9 => {
            // Travelling counter-clockwise about z, left is towards the centre.
            let ccw = (frame.z.dot(z) > 0.0) != s.reversed;
            let r = if ccw { radius - d } else { radius + d };
            (r > 1e-9).then(|| Segment { curve: Curve3::Circle { frame: *frame, radius: r }, ..s.clone() })
        }
        _ => {
            let n = 64;
            let pts: Vec<DVec3> = (0..=n)
                .map(|i| {
                    let (p, t) = point_tangent(s, i as f64 / n as f64);
                    p + z.cross(t).normalize() * d
                })
                .collect();
            let c = NurbsCurve::interpolate(&pts, 3);
            let (t0, t1) = c.domain();
            Some(Segment::new(Curve3::Nurbs(c), t0, t1))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::profile::Loop3;
    use std::f64::consts::PI;

    fn rect(w: f64, h: f64) -> Vec<Segment> {
        let p = [DVec3::ZERO, DVec3::new(w, 0.0, 0.0), DVec3::new(w, h, 0.0), DVec3::new(0.0, h, 0.0)];
        (0..4).map(|i| Segment::line(p[i], p[(i + 1) % 4])).collect()
    }

    fn area(segs: &[Segment]) -> f64 {
        let pts = Loop3 { segments: segs.to_vec() }.polygon(1e-5);
        let mut a = 0.0;
        for i in 0..pts.len() {
            let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
            a += p.x * q.y - q.x * p.y;
        }
        a / 2.0
    }

    #[test]
    fn rounds_a_rectangle() {
        let r = 0.5;
        let out = fillet_corners(&rect(4.0, 3.0), true, r, None);
        assert_eq!(out.len(), 8);
        for w in 0..8 {
            assert!(out[w].end().distance(out[(w + 1) % 8].start()) < 1e-9);
        }
        let expect = 12.0 - (4.0 - PI) * r * r;
        assert!((area(&out) - expect).abs() < 1e-4, "{} vs {expect}", area(&out));
        // One corner only.
        let one = fillet_corners(&rect(4.0, 3.0), true, r, Some(&[DVec3::new(4.0, 3.0, 0.0)]));
        assert_eq!(one.len(), 5);
    }

    #[test]
    fn rounds_a_line_into_an_arc() {
        // A line meeting a circle arc at a corner: the fillet touches both.
        let arc = Segment::new(Curve3::Circle { frame: Frame { origin: DVec3::new(2.0, 2.0, 0.0), ..Frame::WORLD }, radius: 2.0 }, -0.5 * PI, 0.0);
        let segs = vec![Segment::line(DVec3::new(-2.0, 0.0, 0.0), DVec3::new(2.0, 0.0, 0.0)).flipped().flipped(), arc];
        // Tangent at the join: no rounding.
        assert_eq!(fillet_corners(&segs, false, 0.3, None).len(), 2);
        let arc = Segment::new(Curve3::Circle { frame: Frame { origin: DVec3::new(2.0, 2.0, 0.0), ..Frame::WORLD }, radius: 2.0 * 2f64.sqrt() }, -0.75 * PI, -0.25 * PI);
        let segs = vec![Segment::line(DVec3::new(-2.0, 0.0, 0.0), DVec3::ZERO), arc];
        let out = fillet_corners(&segs, false, 0.3, None);
        assert_eq!(out.len(), 3);
        assert!(out[0].end().distance(out[1].start()) < 1e-9 && out[1].end().distance(out[2].start()) < 1e-9);
    }

    #[test]
    fn offsets_a_rectangle_both_ways() {
        let out = offset_chain(&rect(4.0, 3.0), true, -0.5, DVec3::Z);
        // Outward (right of a counter-clockwise loop): rounded corners.
        assert_eq!(out.len(), 8);
        let expect = 12.0 + 0.5 * 14.0 + PI * 0.25;
        assert!((area(&out) - expect).abs() < 1e-4, "{} vs {expect}", area(&out));
        let inner = offset_chain(&rect(4.0, 3.0), true, 0.5, DVec3::Z);
        assert_eq!(inner.len(), 4);
        assert!((area(&inner) - 3.0 * 2.0).abs() < 1e-9, "{}", area(&inner));
    }
}
