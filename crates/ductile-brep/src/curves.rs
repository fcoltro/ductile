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

/// A helix about `frame`'s z axis starting on its x axis: `turns` turns
/// (negative for left-handed) of `radius`, rising `height`. Cubic pieces
/// of 15 degrees each, tangent to the helix at every joint and off it by
/// under 1e-7 of the radius.
pub fn helix(frame: &Frame, radius: f64, height: f64, turns: f64) -> NurbsCurve {
    let n = ((turns.abs() * 24.0).ceil() as usize).max(1);
    let w = turns * std::f64::consts::TAU;
    let at = |s: f64| frame.to_world(DVec3::new(radius * (s * w).cos(), radius * (s * w).sin(), height * s));
    let h = 1.0 / n as f64;
    // Handle length of the best cubic arc, 4/3 tan(a/4) of the radius for
    // an arc of angle a, in place of the plain tangent's a/3.
    let da = h * w;
    let fit = if da.abs() > 1e-12 { 4.0 * (da / 4.0).tan() / da } else { 1.0 };
    let tan = |s: f64| frame.dir_to_world(DVec3::new(-radius * w * fit * (s * w).sin(), radius * w * fit * (s * w).cos(), height));
    let mut ctrl = vec![at(0.0)];
    let mut knots = vec![0.0; 4];
    for k in 0..n {
        let (s0, s1) = (k as f64 * h, (k + 1) as f64 * h);
        ctrl.push(at(s0) + tan(s0) * (h / 3.0));
        ctrl.push(at(s1) - tan(s1) * (h / 3.0));
        ctrl.push(at(s1));
        knots.extend(std::iter::repeat(s1).take(if k + 1 == n { 4 } else { 3 }));
    }
    NurbsCurve::new(3, knots, ctrl.into_iter().map(|p| p.extend(1.0)).collect())
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

/// Every segment split where any other crosses or touches it (within
/// 1e-7), for trimming. Returns, per input segment, its pieces in order.
pub fn split_at_crossings(segs: &[Segment]) -> Vec<Vec<Segment>> {
    let tol = 1e-4;
    let polys: Vec<Vec<DVec3>> = segs.iter().map(|s| s.sample(tol)).collect();
    let mut cuts: Vec<Vec<f64>> = vec![Vec::new(); segs.len()];
    for i in 0..segs.len() {
        for j in i + 1..segs.len() {
            for (pa, pb) in polyline_crossings(&polys[i], &polys[j]) {
                // Refine from the polyline crossing to the curves' own.
                if let Some((sa, sb)) = refine_crossing(&segs[i], &segs[j], pa, pb) {
                    cuts[i].push(sa);
                    cuts[j].push(sb);
                }
            }
        }
    }
    segs.iter()
        .zip(cuts)
        .map(|(s, mut c)| {
            c.retain(|x| *x > 1e-7 && *x < 1.0 - 1e-7);
            c.sort_by(f64::total_cmp);
            c.dedup_by(|a, b| (*a - *b).abs() < 1e-7);
            let mut out = Vec::with_capacity(c.len() + 1);
            // A cut circle starts at its first cut rather than at its seam.
            if s.is_closed() && s.curve.period().is_some() && !c.is_empty() {
                let first = c[0];
                for w in c.windows(2) {
                    out.push(trimmed(s, w[0], w[1]));
                }
                out.push(trimmed(s, *c.last().unwrap(), 1.0 + first));
                return out;
            }
            let mut prev = 0.0;
            for x in c.into_iter().chain([1.0]) {
                out.push(trimmed(s, prev, x));
                prev = x;
            }
            out
        })
        .collect()
}

/// Fractions along two sampled curves where their polylines cross.
fn polyline_crossings(a: &[DVec3], b: &[DVec3]) -> Vec<(f64, f64)> {
    let (na, nb) = ((a.len() - 1).max(1) as f64, (b.len() - 1).max(1) as f64);
    let mut out = Vec::new();
    for i in 0..a.len().saturating_sub(1) {
        for k in 0..b.len().saturating_sub(1) {
            let (p, r) = (a[i], a[i + 1] - a[i]);
            let (q, s) = (b[k], b[k + 1] - b[k]);
            let n = r.cross(s);
            let nn = n.length_squared();
            if nn < 1e-24 {
                continue;
            }
            let t = (q - p).cross(s).dot(n) / nn;
            let u = (q - p).cross(r).dot(n) / nn;
            if !(-1e-9..=1.0 + 1e-9).contains(&t) || !(-1e-9..=1.0 + 1e-9).contains(&u) {
                continue;
            }
            // Coplanar segments only.
            let gap = (p + r * t).distance(q + s * u);
            if gap > 1e-6 * (1.0 + r.length() + s.length()) {
                continue;
            }
            out.push(((i as f64 + t) / na, (k as f64 + u) / nb));
        }
    }
    out
}

/// Newton on the fractions of two segments for a common point, from a
/// polyline estimate (polyline fractions approximate curve fractions).
fn refine_crossing(a: &Segment, b: &Segment, mut sa: f64, mut sb: f64) -> Option<(f64, f64)> {
    let (la, lb) = (seg_len(a), seg_len(b));
    // The polyline fraction is by sample index; map through arc length by
    // a few projection steps first.
    for _ in 0..60 {
        let (pa, ta) = point_tangent(a, sa);
        let (pb, tb) = point_tangent(b, sb);
        let f = pa - pb;
        if f.length() < 1e-12 {
            break;
        }
        let (ja, jb) = (ta * dpds(a, sa, la), -tb * dpds(b, sb, lb));
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

/// Speed of a segment with respect to its traversal fraction.
fn dpds(s: &Segment, x: f64, fallback: f64) -> f64 {
    let t = param(s, x);
    let v = s.curve.tangent(t).length() * (s.t1 - s.t0).abs();
    if v.is_finite() && v > 0.0 {
        v
    } else {
        fallback
    }
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

/// Point and first three derivatives of `seg` at one of its ends, with
/// respect to a parameter that grows away from the segment (outwards).
pub fn curve_end(seg: &Segment, at_end: bool) -> [DVec3; 4] {
    // Curve parameter at the end, and the sign that turns it outwards.
    let (t, sign) = match (at_end, seg.reversed) {
        (true, false) => (seg.t1, 1.0),
        (true, true) => (seg.t0, -1.0),
        (false, false) => (seg.t0, -1.0),
        (false, true) => (seg.t1, 1.0),
    };
    let d = seg.curve.derivs(t);
    // Third derivative by a one-sided difference taken inside the segment.
    let h = 1e-3 * (seg.t1 - seg.t0).abs().max(1e-9);
    let dd = |x: f64| seg.curve.derivs(x)[2];
    let d3 = (dd(t) * 3.0 - dd(t - sign * h) * 4.0 + dd(t - 2.0 * sign * h)) * (sign / (2.0 * h));
    [d[0], d[1] * sign, d[2], d3 * sign]
}

/// A curve from the end `a` (point and outward derivatives, see
/// [`curve_end`]) to the end `b`, leaving `a` and entering `b` with
/// continuity G`k` (0 = straight line, 1 tangent, 2 curvature,
/// 3 curvature rate). `tension` scales how far the ends' directions
/// carry into the bridge; 1 is a natural default.
///
/// The bridge is one Bézier of degree 2k + 1 whose first k derivatives at
/// each end equal the end curve's, after the linear reparameterization
/// set by the tension, so the joins are exactly G`k`.
pub fn bridge_curve(a: &[DVec3; 4], b: &[DVec3; 4], k: usize, tension: [f64; 2]) -> NurbsCurve {
    let k = k.min(3);
    let n = 2 * k + 1;
    let gap = a[0].distance(b[0]).max(1e-12);
    let ha = tension[0] * gap / a[1].length().max(1e-12);
    let hb = -tension[1] * gap / b[1].length().max(1e-12);
    let binom = |n: usize, k: usize| (0..k).fold(1.0, |acc, i| acc * (n - i) as f64 / (i + 1) as f64);
    // n! / (n - m)!
    let falling = |m: usize| (0..m).fold(1.0, |acc, i| acc * (n - i) as f64);
    let mut p = vec![DVec3::ZERO; n + 1];
    p[0] = a[0];
    p[n] = b[0];
    for m in 1..=k {
        let mut s = a[m] * ha.powi(m as i32) / falling(m);
        for i in 0..m {
            s -= p[i] * (if (m - i) % 2 == 0 { 1.0 } else { -1.0 } * binom(m, i));
        }
        p[m] = s;
        let mut s = b[m] * hb.powi(m as i32) / falling(m);
        for i in 0..m {
            s -= p[n - i] * (if i % 2 == 0 { 1.0 } else { -1.0 } * binom(m, i));
        }
        p[n - m] = if m % 2 == 0 { s } else { -s };
    }
    let mut knots = vec![0.0; n + 1];
    knots.extend(std::iter::repeat_n(1.0, n + 1));
    NurbsCurve::from_points(n, knots, &p)
}

/// How [`extend_segment`] continues a curve past its end.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Extension {
    /// Along the end tangent.
    Linear,
    /// Along the curve's own shape: lines and arcs stay lines and arcs,
    /// splines continue their last polynomial piece.
    Natural,
}

/// Lengthens `seg` by `len` past its end (`at_end`) or its start. The
/// result replaces `seg`: one segment, or two where a straight piece is
/// added to a curve.
pub fn extend_segment(seg: &Segment, at_end: bool, len: f64, how: Extension) -> Vec<Segment> {
    if len <= 1e-12 {
        return vec![seg.clone()];
    }
    if how == Extension::Linear && !seg.curve.is_linear() {
        let e = curve_end(seg, at_end);
        let tip = e[0] + e[1].normalize() * len;
        return if at_end {
            vec![seg.clone(), Segment::line(e[0], tip)]
        } else {
            vec![Segment::line(tip, e[0]), seg.clone()]
        };
    }
    // Whether the curve parameter grows towards the extended end.
    let up = at_end != seg.reversed;
    let grow = |dt: f64| {
        let (t0, t1) = if up { (seg.t0, seg.t1 + dt) } else { (seg.t0 - dt, seg.t1) };
        Segment { t0, t1, ..seg.clone() }
    };
    match &seg.curve {
        Curve3::Line { .. } => vec![grow(len)],
        Curve3::Circle { radius, .. } => {
            let span = (seg.t1 - seg.t0).abs();
            vec![grow((len / radius).min(std::f64::consts::TAU - span - 1e-6).max(0.0))]
        }
        Curve3::Ellipse { .. } => {
            // Arc length grows monotonically with the angle: bisect.
            let span = (seg.t1 - seg.t0).abs();
            let (mut lo, mut hi) = (0.0, std::f64::consts::TAU - span - 1e-6);
            let extra = |dt: f64| {
                let s = grow(dt);
                s.curve.length(s.t0, s.t1) - seg.curve.length(seg.t0, seg.t1)
            };
            if extra(hi) <= len {
                return vec![grow(hi.max(0.0))];
            }
            for _ in 0..60 {
                let mid = 0.5 * (lo + hi);
                if extra(mid) < len {
                    lo = mid
                } else {
                    hi = mid
                }
            }
            vec![grow(0.5 * (lo + hi))]
        }
        Curve3::Nurbs(_) => {
            // Work on the curve oriented towards the extended end.
            let mut c = seg.curve.to_nurbs(seg.t0, seg.t1);
            if !up {
                c = c.reversed();
            }
            let tail = natural_tail(&c, len);
            let mut joined = NurbsCurve::join(&[c, tail]);
            if !up {
                joined = joined.reversed();
            }
            let (t0, t1) = joined.domain();
            vec![Segment { curve: Curve3::Nurbs(joined), t0, t1, reversed: seg.reversed }]
        }
    }
}

/// The continuation of `c` past its end by arc length `len`: its last
/// Bézier piece, evaluated beyond its parameter range by blossoming.
fn natural_tail(c: &NurbsCurve, len: f64) -> NurbsCurve {
    let last = c.bezier_segments().pop().unwrap();
    let n = last.degree;
    // Blossom of the piece (taken on [0, 1]) at the given arguments.
    let blossom = |args: &[f64]| {
        let mut q = last.ctrl.clone();
        for (j, &u) in args.iter().enumerate() {
            for i in 0..n - j {
                q[i] = q[i] * (1.0 - u) + q[i + 1] * u;
            }
        }
        q[0]
    };
    let piece = |tau: f64| {
        let ctrl = (0..=n)
            .map(|i| {
                let mut args = vec![1.0; n - i];
                args.extend(std::iter::repeat_n(tau, i));
                blossom(&args)
            })
            .collect();
        let mut knots = vec![0.0; n + 1];
        knots.extend(std::iter::repeat_n(1.0, n + 1));
        NurbsCurve::new(n, knots, ctrl)
    };
    // Newton on the piece's length, from the end speed of the curve.
    let speed0 = piece(1.0 + 1e-6).derivs(0.0, 1)[1].length() / 1e-6;
    let mut tau = 1.0 + len / speed0.max(1e-12);
    for _ in 0..20 {
        let p = piece(tau);
        let err = len - p.length();
        if err.abs() < 1e-9 * len.max(1.0) {
            break;
        }
        let speed = p.derivs(1.0, 1)[1].length() / (tau - 1.0);
        tau = (tau + err / speed.max(1e-12)).max(1.0 + 1e-9);
    }
    piece(tau)
}

/// The same curve run the other way: its parameter now goes from the old
/// end to the old start, with lines and arcs kept exact.
pub fn reverse_segment(s: &Segment) -> Segment {
    let (t0, t1) = (s.t0, s.t1);
    let (curve, a, b) = match &s.curve {
        Curve3::Line { origin, dir } => (Curve3::Line { origin: *origin + *dir * (t0 + t1), dir: -*dir }, t0, t1),
        Curve3::Circle { frame, radius } => (Curve3::Circle { frame: Frame::new(frame.origin, -frame.z, frame.x), radius: *radius }, std::f64::consts::TAU - t1, std::f64::consts::TAU - t0),
        Curve3::Ellipse { frame, rx, ry } => (Curve3::Ellipse { frame: Frame::new(frame.origin, -frame.z, frame.x), rx: *rx, ry: *ry }, std::f64::consts::TAU - t1, std::f64::consts::TAU - t0),
        Curve3::Nurbs(n) => {
            let (lo, hi) = n.domain();
            (Curve3::Nurbs(n.reversed()), lo + hi - t1, lo + hi - t0)
        }
    };
    Segment { curve, t0: a, t1: b, reversed: s.reversed }
}

/// A chain of segments run the other way.
pub fn reverse_chain(segs: &[Segment]) -> Vec<Segment> {
    segs.iter().rev().map(reverse_segment).collect()
}

/// The segment's curve as a spline one degree higher (exact).
pub fn raise_degree(s: &Segment) -> Segment {
    let n = s.curve.to_nurbs(s.t0, s.t1);
    let up = n.elevated_to(n.degree + 1);
    let (a, b) = up.domain();
    Segment { curve: Curve3::Nurbs(up), t0: a, t1: b, reversed: s.reversed }
}

/// Least-squares spline of `degree` with `n` control points through the
/// ends of `pts` and close to the rest (The NURBS Book, 9.4.1): chord
/// length parameters, knots spread so every span holds samples.
pub fn fit_curve(pts: &[DVec3], n: usize, degree: usize) -> Option<NurbsCurve> {
    use ductile_geom::nurbs::basis::{basis_funs, find_span};
    let m = pts.len();
    let p = degree.max(1);
    let n = n.max(p + 1);
    if m < n || n < 2 {
        return (m >= 2).then(|| NurbsCurve::interpolate(pts, p.min(m - 1)));
    }
    let mut ub = vec![0.0];
    for w in pts.windows(2) {
        ub.push(ub.last().unwrap() + w[0].distance(w[1]));
    }
    let total = *ub.last().unwrap();
    if total <= 0.0 {
        return None;
    }
    ub.iter_mut().for_each(|u| *u /= total);
    // Knots: averaged over the parameters.
    let mut knots = vec![0.0; p + 1];
    let d = m as f64 / (n - p) as f64;
    for j in 1..n - p {
        let i = (j as f64 * d).floor() as usize;
        let a = j as f64 * d - i as f64;
        knots.push((1.0 - a) * ub[i - 1] + a * ub[i.min(m - 1)]);
    }
    knots.extend(std::iter::repeat_n(1.0, p + 1));
    // Normal equations for the inner control points, ends fixed.
    let basis = |u: f64| {
        let span = find_span(n - 1, p, u, &knots);
        let b = basis_funs(span, u, p, &knots);
        let mut row = vec![0.0; n];
        for (k, v) in b.into_iter().enumerate() {
            row[span - p + k] = v;
        }
        row
    };
    let (q0, qm) = (pts[0], pts[m - 1]);
    let inner = n - 2;
    let mut a = vec![vec![0.0; inner]; inner];
    let mut r = vec![DVec3::ZERO; inner];
    for k in 1..m - 1 {
        let row = basis(ub[k]);
        let rk = pts[k] - q0 * row[0] - qm * row[n - 1];
        for i in 0..inner {
            if row[i + 1] == 0.0 {
                continue;
            }
            r[i] += rk * row[i + 1];
            for j in 0..inner {
                a[i][j] += row[i + 1] * row[j + 1];
            }
        }
    }
    // Gaussian elimination with partial pivoting.
    for c in 0..inner {
        let piv = (c..inner).max_by(|x, y| a[*x][c].abs().total_cmp(&a[*y][c].abs()))?;
        if a[piv][c].abs() < 1e-14 {
            return None;
        }
        a.swap(c, piv);
        r.swap(c, piv);
        for rr in c + 1..inner {
            let f = a[rr][c] / a[c][c];
            if f != 0.0 {
                for k in c..inner {
                    a[rr][k] -= f * a[c][k];
                }
                r[rr] = r[rr] - r[c] * f;
            }
        }
    }
    for c in (0..inner).rev() {
        let mut v = r[c];
        for k in c + 1..inner {
            v -= r[k] * a[c][k];
        }
        r[c] = v / a[c][c];
    }
    let mut ctrl = vec![q0];
    ctrl.extend(r);
    ctrl.push(qm);
    Some(NurbsCurve::from_points(p, knots, &ctrl))
}

/// A chain of curves rebuilt as one spline with `n` control points of
/// `degree`, and the largest distance from the original.
pub fn rebuild_chain(segs: &[Segment], n: usize, degree: usize) -> Option<(NurbsCurve, f64)> {
    const PER: usize = 64;
    let mut pts: Vec<DVec3> = Vec::new();
    for s in segs {
        for i in 0..=PER {
            let f = i as f64 / PER as f64;
            let t = if s.reversed { s.t1 - (s.t1 - s.t0) * f } else { s.t0 + (s.t1 - s.t0) * f };
            let p = s.curve.point(t);
            if pts.last().is_none_or(|q| q.distance(p) > 1e-12) {
                pts.push(p);
            }
        }
    }
    let c = fit_curve(&pts, n, degree)?;
    let cc = Curve3::Nurbs(c.clone());
    let (a, b) = c.domain();
    let dev = pts.iter().map(|p| cc.point(cc.closest_param_in(*p, a, b)).distance(*p)).fold(0.0, f64::max);
    Some((c, dev))
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
    fn splits_a_rectangle_and_a_circle() {
        let mut segs = rect(4.0, 3.0);
        segs.push(Segment::new(Curve3::Circle { frame: Frame { origin: DVec3::new(4.0, 3.0, 0.0), ..Frame::WORLD }, radius: 1.0 }, 0.0, 2.0 * PI));
        let parts = split_at_crossings(&segs);
        // The circle crosses the right and top sides once each.
        assert_eq!(parts[1].len(), 2);
        assert_eq!(parts[2].len(), 2);
        assert_eq!(parts[0].len(), 1);
        assert_eq!(parts[4].len(), 2);
        assert!(parts[1][0].end().distance(DVec3::new(4.0, 2.0, 0.0)) < 1e-9, "{}", parts[1][0].end());
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

    fn curvature(d: &[DVec3]) -> f64 {
        d[1].cross(d[2]).length() / d[1].length().powi(3)
    }

    #[test]
    fn bridges_match_the_ends() {
        let arc = Segment::new(Curve3::Circle { frame: Frame::WORLD, radius: 2.0 }, 0.0, 1.0);
        let line = Segment::line(DVec3::new(5.0, 3.0, 1.0), DVec3::new(8.0, 3.0, 1.0));
        let a = curve_end(&arc, true);
        let b = curve_end(&line, false);
        for k in 0..=3 {
            let c = bridge_curve(&a, &b, k, [1.0, 1.0]);
            assert_eq!(c.degree, 2 * k + 1);
            let s = c.derivs(0.0, 2);
            let e = c.derivs(1.0, 2);
            assert!(s[0].distance(a[0]) < 1e-12 && e[0].distance(b[0]) < 1e-12);
            if k >= 1 {
                assert!(s[1].normalize().distance(a[1].normalize()) < 1e-6, "k={k}");
                // The bridge enters the line along it.
                assert!(e[1].normalize().distance(-b[1].normalize()) < 1e-6, "k={k}");
            }
            if k >= 2 {
                assert!((curvature(&s) - 0.5).abs() < 1e-5, "k={k} {}", curvature(&s));
                assert!(curvature(&e) < 1e-6, "k={k}");
            }
        }
        // G3: the curvature's rate of change at the arc end is zero as on the arc.
        let c = bridge_curve(&a, &b, 3, [1.0, 1.0]);
        let h = 1e-4;
        let dk = (curvature(&c.derivs(h, 2)) - curvature(&c.derivs(0.0, 2))) / h;
        assert!(dk.abs() < 1e-2, "{dk}");
        // Tension pulls the bridge further along the end directions.
        let loose = bridge_curve(&a, &b, 1, [2.0, 2.0]);
        assert!(loose.length() > bridge_curve(&a, &b, 1, [1.0, 1.0]).length());
    }

    #[test]
    fn helix_stays_on_its_cylinder() {
        let f = Frame::new(DVec3::new(1.0, 2.0, 3.0), DVec3::Z, DVec3::X);
        let h = helix(&f, 2.0, 5.0, 3.5);
        let (t0, t1) = h.domain();
        for k in 0..=500 {
            let p = h.point(t0 + (t1 - t0) * k as f64 / 500.0) - f.origin;
            assert!((p.truncate().length() - 2.0).abs() < 2e-7, "{p}");
        }
        assert!((h.point(t1) - f.origin - DVec3::new(-2.0, 0.0, 5.0)).length() < 1e-9);
    }

    #[test]
    fn extends_curves() {
        let line = Segment::line(DVec3::ZERO, DVec3::X);
        let l = extend_segment(&line, true, 0.5, Extension::Natural);
        assert_eq!(l.len(), 1);
        assert!(l[0].end().distance(DVec3::new(1.5, 0.0, 0.0)) < 1e-12);
        let l = extend_segment(&line.flipped(), true, 0.5, Extension::Natural);
        assert!(l[0].end().distance(DVec3::new(-0.5, 0.0, 0.0)) < 1e-12);

        let arc = Segment::new(Curve3::Circle { frame: Frame::WORLD, radius: 2.0 }, 0.0, 1.0);
        let a = extend_segment(&arc, false, 1.0, Extension::Natural);
        assert_eq!(a.len(), 1);
        assert!((a[0].t0 + 0.5).abs() < 1e-12);
        let a = extend_segment(&arc, true, 1.0, Extension::Linear);
        assert_eq!(a.len(), 2);
        assert!((a[1].start().distance(a[1].end()) - 1.0).abs() < 1e-12);

        // A parabola continues as the same parabola.
        let par = NurbsCurve::from_points(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            &[DVec3::ZERO, DVec3::new(0.5, 0.0, 0.0), DVec3::new(1.0, 1.0, 0.0)],
        );
        let seg = Segment::new(Curve3::Nurbs(par.clone()), 0.0, 1.0);
        let before = par.length();
        for (at_end, rev) in [(true, false), (false, false), (true, true)] {
            let s = if rev { seg.flipped() } else { seg.clone() };
            let out = extend_segment(&s, at_end, 0.7, Extension::Natural);
            assert_eq!(out.len(), 1);
            let o = &out[0];
            let Curve3::Nurbs(n) = &o.curve else { panic!() };
            assert!((n.length() - before - 0.7).abs() < 1e-6, "{}", n.length() - before);
            let tip = if at_end { o.end() } else { o.start() };
            assert!((tip.y - tip.x * tip.x).abs() < 1e-9, "{tip}");
            // The untouched end stays put.
            let other = if at_end { o.start() } else { o.end() };
            let orig = if at_end { s.start() } else { s.end() };
            assert!(other.distance(orig) < 1e-12);
        }
    }

    #[test]
    fn rebuilds_a_wiggly_spline_with_fewer_points() {
        let pts: Vec<DVec3> = (0..=40).map(|i| {
            let x = i as f64 / 10.0;
            DVec3::new(x, (x * 1.3).sin(), 0.0)
        }).collect();
        let c = NurbsCurve::interpolate(&pts, 3);
        let (t0, t1) = c.domain();
        let seg = Segment::new(Curve3::Nurbs(c), t0, t1);
        let (r, dev) = rebuild_chain(&[seg.clone()], 8, 3).unwrap();
        assert_eq!(r.ctrl.len(), 8);
        assert!(dev < 5e-2, "{dev}");
        assert!(r.point(r.domain().0).distance(pts[0]) < 1e-12 && r.point(r.domain().1).distance(pts[40]) < 1e-12);
        let (r5, dev5) = rebuild_chain(&[seg], 12, 5).unwrap();
        assert_eq!(r5.degree, 5);
        assert!(dev5 < dev);
    }

    #[test]
    fn reversed_segments_run_backwards() {
        let segs = [
            Segment::line(DVec3::ZERO, DVec3::X),
            Segment::new(Curve3::Circle { frame: Frame::WORLD, radius: 2.0 }, 0.3, 1.2),
            Segment::new(Curve3::Ellipse { frame: Frame::WORLD, rx: 2.0, ry: 1.0 }, 0.5, 2.0),
            Segment::new(Curve3::Nurbs(NurbsCurve::interpolate(&[DVec3::ZERO, DVec3::new(1.0, 1.0, 0.0), DVec3::new(2.0, 0.0, 1.0)], 2)), 0.0, 1.0),
        ];
        for s in &segs {
            let r = reverse_segment(s);
            for f in [0.0, 0.3, 1.0] {
                let p = s.curve.point(s.t0 + (s.t1 - s.t0) * f);
                let q = r.curve.point(r.t1 - (r.t1 - r.t0) * f);
                assert!(p.distance(q) < 1e-9, "{:?}", s.curve);
            }
            let up = raise_degree(s);
            assert!(up.curve.point(up.t0).distance(s.curve.point(s.t0)) < 1e-9 && up.curve.point(0.5 * (up.t0 + up.t1)).distance(s.curve.point(s.curve.closest_param_in(up.curve.point(0.5 * (up.t0 + up.t1)), s.t0, s.t1))) < 1e-9);
        }
    }
}
