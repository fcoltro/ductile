// SPDX-License-Identifier: GPL-3.0-only
use super::basis::{basis_funs, binomial, ders_basis_funs, find_span};
use crate::{BBox3, Frame};
use glam::{DMat4, DVec3, DVec4};
use serde::{Deserialize, Serialize};
use std::f64::consts::{FRAC_PI_2, TAU};

/// A (possibly rational) NURBS curve in 3D. Control points are stored in
/// homogeneous form `(w·x, w·y, w·z, w)`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct NurbsCurve {
    pub degree: usize,
    pub knots: Vec<f64>,
    pub ctrl: Vec<DVec4>,
}

pub(crate) fn hom(p: DVec3, w: f64) -> DVec4 {
    (p * w).extend(w)
}

pub(crate) fn dehom(h: DVec4) -> DVec3 {
    h.truncate() / h.w
}

impl NurbsCurve {
    pub fn new(degree: usize, knots: Vec<f64>, ctrl: Vec<DVec4>) -> Self {
        assert_eq!(knots.len(), ctrl.len() + degree + 1, "knot vector length mismatch");
        Self { degree, knots, ctrl }
    }

    /// Non-rational curve from 3D control points.
    pub fn from_points(degree: usize, knots: Vec<f64>, pts: &[DVec3]) -> Self {
        Self::new(degree, knots, pts.iter().map(|p| p.extend(1.0)).collect())
    }

    /// Clamped uniform B-spline through the given control points.
    pub fn uniform(degree: usize, pts: &[DVec3]) -> Self {
        let p = degree.min(pts.len() - 1);
        Self::from_points(p, super::basis::uniform_clamped_knots(pts.len(), p), pts)
    }

    pub fn line(a: DVec3, b: DVec3) -> Self {
        Self::from_points(1, vec![0.0, 0.0, 1.0, 1.0], &[a, b])
    }

    pub fn n(&self) -> usize {
        self.ctrl.len() - 1
    }

    pub fn domain(&self) -> (f64, f64) {
        (self.knots[self.degree], self.knots[self.knots.len() - 1 - self.degree])
    }

    pub fn control_points(&self) -> Vec<DVec3> {
        self.ctrl.iter().map(|h| dehom(*h)).collect()
    }

    pub fn weights(&self) -> Vec<f64> {
        self.ctrl.iter().map(|h| h.w).collect()
    }

    pub fn is_rational(&self) -> bool {
        let w0 = self.ctrl[0].w;
        self.ctrl.iter().any(|h| (h.w - w0).abs() > 1e-14)
    }

    pub fn point(&self, u: f64) -> DVec3 {
        let p = self.degree;
        let span = find_span(self.n(), p, u, &self.knots);
        let b = basis_funs(span, u, p, &self.knots);
        let mut c = DVec4::ZERO;
        for j in 0..=p {
            c += self.ctrl[span - p + j] * b[j];
        }
        dehom(c)
    }

    /// Point and derivatives up to order `d` (A4.2).
    pub fn derivs(&self, u: f64, d: usize) -> Vec<DVec3> {
        let p = self.degree;
        let du = d.min(p);
        let span = find_span(self.n(), p, u, &self.knots);
        let nders = ders_basis_funs(span, u, p, du, &self.knots);
        let mut cw = vec![DVec4::ZERO; d + 1];
        for k in 0..=du {
            for j in 0..=p {
                cw[k] += self.ctrl[span - p + j] * nders[k][j];
            }
        }
        let mut ck = vec![DVec3::ZERO; d + 1];
        for k in 0..=d {
            let mut v = cw[k].truncate();
            for i in 1..=k {
                v -= ck[k - i] * (binomial(k, i) * cw[i].w);
            }
            ck[k] = v / cw[0].w;
        }
        ck
    }

    pub fn tangent(&self, u: f64) -> DVec3 {
        self.derivs(u, 1)[1]
    }

    pub fn transformed(&self, m: &DMat4) -> Self {
        let ctrl = self
            .ctrl
            .iter()
            .map(|h| hom(m.transform_point3(dehom(*h)), h.w))
            .collect();
        Self { degree: self.degree, knots: self.knots.clone(), ctrl }
    }

    pub fn reversed(&self) -> Self {
        let (a, b) = (self.knots[0], *self.knots.last().unwrap());
        let knots = self.knots.iter().rev().map(|k| a + b - k).collect();
        let ctrl = self.ctrl.iter().rev().copied().collect();
        Self { degree: self.degree, knots, ctrl }
    }

    /// Linearly re-maps the parameter domain to `[a, b]`.
    pub fn reparameterized(&self, a: f64, b: f64) -> Self {
        let (u0, u1) = self.domain();
        let s = (b - a) / (u1 - u0);
        let knots = self.knots.iter().map(|k| a + (k - u0) * s).collect();
        Self { degree: self.degree, knots, ctrl: self.ctrl.clone() }
    }

    pub fn bbox(&self) -> BBox3 {
        // The convex hull property makes the control polygon box conservative.
        BBox3::from_points(self.control_points())
    }

    fn multiplicity(&self, u: f64) -> usize {
        self.knots.iter().filter(|k| (**k - u).abs() < 1e-12).count()
    }

    /// Inserts knot `u` `r` times (A5.1).
    pub fn insert_knot(&self, u: f64, r: usize) -> Self {
        let p = self.degree;
        let s = self.multiplicity(u);
        let r = r.min(p.saturating_sub(s));
        if r == 0 {
            return self.clone();
        }
        let np = self.n();
        let k = find_span(np, p, u, &self.knots);
        let mp = np + p + 1;
        let mut uq = vec![0.0; mp + r + 1];
        for i in 0..=k {
            uq[i] = self.knots[i];
        }
        for i in 1..=r {
            uq[k + i] = u;
        }
        for i in k + 1..=mp {
            uq[i + r] = self.knots[i];
        }
        let mut qw = vec![DVec4::ZERO; np + r + 1];
        for i in 0..=k - p {
            qw[i] = self.ctrl[i];
        }
        for i in k - s..=np {
            qw[i + r] = self.ctrl[i];
        }
        let mut rw: Vec<DVec4> = (0..=p - s).map(|i| self.ctrl[k - p + i]).collect();
        let mut l = 0;
        for j in 1..=r {
            l = k - p + j;
            for i in 0..=p - j - s {
                let alpha = (u - self.knots[l + i]) / (self.knots[i + k + 1] - self.knots[l + i]);
                rw[i] = rw[i + 1] * alpha + rw[i] * (1.0 - alpha);
            }
            qw[l] = rw[0];
            qw[k + r - j - s] = rw[p - j - s];
        }
        for i in l + 1..k - s {
            qw[i] = rw[i - l];
        }
        Self { degree: p, knots: uq, ctrl: qw }
    }

    /// Splits at `u` into two curves sharing the end point.
    pub fn split(&self, u: f64) -> (Self, Self) {
        let (a, b) = self.domain();
        let p = self.degree;
        if u <= a + 1e-12 || u >= b - 1e-12 {
            return (self.clone(), self.clone());
        }
        let c = self.insert_knot(u, p);
        // After insertion u has multiplicity p (or more if it already had).
        let k = c.knots.iter().position(|k| (*k - u).abs() < 1e-12).unwrap();
        let mult = c.multiplicity(u);
        let mut lk: Vec<f64> = c.knots[..k + mult].to_vec();
        lk.push(u);
        let lctrl = c.ctrl[..k].to_vec();
        let mut rk = vec![u];
        rk.extend_from_slice(&c.knots[k..]);
        let rctrl = c.ctrl[k - 1..].to_vec();
        let left = Self::new(p, lk, lctrl);
        let right = Self::new(p, rk, rctrl);
        (left, right)
    }

    /// Returns the sub-curve on `[t0, t1]`.
    pub fn subcurve(&self, t0: f64, t1: f64) -> Self {
        let (a, b) = self.domain();
        let mut c = self.clone();
        if t1 < b - 1e-12 {
            c = c.split(t1).0;
        }
        if t0 > a + 1e-12 {
            c = c.split(t0).1;
        }
        c
    }

    /// Splits into Bézier segments (each with clamped `[0..p+1]` knots).
    pub fn bezier_segments(&self) -> Vec<Self> {
        let mut out = Vec::new();
        let mut rest = self.clone();
        loop {
            let (a, b) = rest.domain();
            let interior = rest.knots.iter().copied().find(|k| *k > a + 1e-12 && *k < b - 1e-12);
            match interior {
                Some(k) => {
                    let (l, r) = rest.split(k);
                    out.push(l);
                    rest = r;
                }
                None => {
                    out.push(rest);
                    break;
                }
            }
        }
        out
    }

    /// Exact degree elevation by one (via Bézier decomposition). The result
    /// has C0 internal knots, which is exact but not minimal.
    pub fn elevated(&self) -> Self {
        let p = self.degree;
        let segs = self.bezier_segments();
        let mut knots = Vec::new();
        let mut ctrl: Vec<DVec4> = Vec::new();
        for (si, s) in segs.iter().enumerate() {
            let (a, b) = s.domain();
            let q = &s.ctrl;
            let mut e = vec![DVec4::ZERO; p + 2];
            e[0] = q[0];
            e[p + 1] = q[p];
            for i in 1..=p {
                let al = i as f64 / (p + 1) as f64;
                e[i] = q[i - 1] * al + q[i] * (1.0 - al);
            }
            if si == 0 {
                knots.extend(std::iter::repeat(a).take(p + 2));
                ctrl.extend_from_slice(&e);
            } else {
                knots.extend(std::iter::repeat(a).take(p + 1));
                ctrl.extend_from_slice(&e[1..]);
            }
            if si == segs.len() - 1 {
                knots.extend(std::iter::repeat(b).take(p + 2));
            }
        }
        Self::new(p + 1, knots, ctrl)
    }

    pub fn elevated_to(&self, degree: usize) -> Self {
        let mut c = self.clone();
        while c.degree < degree {
            c = c.elevated();
        }
        c
    }

    /// Approximate arc length by adaptive Simpson-free Gauss quadrature per span.
    pub fn length(&self) -> f64 {
        // Per knot span: the speed can jump at a C0 knot.
        let (a, b) = self.domain();
        let mut spans: Vec<f64> = self.knots.iter().copied().filter(|k| *k > a && *k < b).collect();
        spans.dedup();
        let bounds: Vec<f64> = std::iter::once(a).chain(spans).chain(std::iter::once(b)).collect();
        let per = (64 / (bounds.len() - 1)).max(8);
        bounds.windows(2).map(|w| gauss_length(|t| self.derivs(t, 1)[1].length(), w[0], w[1], per)).sum()
    }

    /// Parameter of the closest point to `p` (sampling + Newton).
    pub fn closest_param(&self, p: DVec3) -> f64 {
        let (a, b) = self.domain();
        let samples = (self.ctrl.len() * 8).max(32);
        let mut best = (f64::INFINITY, a);
        for i in 0..=samples {
            let t = a + (b - a) * i as f64 / samples as f64;
            let d = self.point(t).distance_squared(p);
            if d < best.0 {
                best = (d, t);
            }
        }
        newton_closest(|t| self.derivs(t, 2), p, best.1, a, b)
    }

    /// Exact circular arc in the frame's XY plane, from angle `a0` sweeping `sweep`
    /// radians (positive = counter-clockwise about frame z). Algorithm A7.1.
    pub fn arc(frame: &Frame, radius: f64, a0: f64, sweep: f64) -> Self {
        let sweep = sweep.clamp(-TAU, TAU);
        let narcs = ((sweep.abs() / FRAC_PI_2) - 1e-9).ceil().max(1.0) as usize;
        let dtheta = sweep / narcs as f64;
        let w1 = (dtheta / 2.0).cos();
        let pt = |ang: f64| frame.to_world(DVec3::new(radius * ang.cos(), radius * ang.sin(), 0.0));
        let tan = |ang: f64| frame.dir_to_world(DVec3::new(-ang.sin(), ang.cos(), 0.0));
        let mut ctrl = vec![hom(pt(a0), 1.0)];
        let mut p0 = pt(a0);
        let mut t0 = tan(a0);
        let mut angle = a0;
        for _ in 0..narcs {
            angle += dtheta;
            let p2 = pt(angle);
            let t2 = tan(angle);
            // Intersect the tangent lines (they're coplanar).
            let p1 = intersect_tangents(p0, t0, p2, t2);
            ctrl.push(hom(p1, w1));
            ctrl.push(hom(p2, 1.0));
            p0 = p2;
            t0 = t2;
        }
        let mut knots = vec![0.0; 3];
        for i in 1..narcs {
            let k = i as f64 / narcs as f64;
            knots.push(k);
            knots.push(k);
        }
        knots.extend([1.0; 3]);
        Self::new(2, knots, ctrl).reparameterized(0.0, sweep.abs())
    }

    pub fn circle(frame: &Frame, radius: f64) -> Self {
        Self::arc(frame, radius, 0.0, TAU)
    }

    /// Global interpolation through `pts` with centripetal parameterization (A9.1).
    pub fn interpolate(pts: &[DVec3], degree: usize) -> Self {
        let n = pts.len();
        assert!(n >= 2);
        let mut d: Vec<f64> = pts.windows(2).map(|w| w[0].distance(w[1]).sqrt()).collect();
        let total: f64 = d.iter().sum();
        if total <= 0.0 {
            d.iter_mut().for_each(|x| *x = 1.0);
        }
        let total: f64 = d.iter().sum();
        let mut params = vec![0.0];
        let mut acc = 0.0;
        for x in &d {
            acc += x / total;
            params.push(acc);
        }
        *params.last_mut().unwrap() = 1.0;
        Self::interpolate_with_params(pts, degree, &params)
    }

    /// Global interpolation through `pts` at the given increasing
    /// parameters from 0 to 1, with averaged knots.
    pub fn interpolate_with_params(pts: &[DVec3], degree: usize, params: &[f64]) -> Self {
        let n = pts.len();
        let p = degree.min(n - 1);
        let mut knots = vec![0.0; p + 1];
        for j in 1..n - p {
            let s: f64 = params[j..j + p].iter().sum();
            knots.push(s / p as f64);
        }
        knots.extend(std::iter::repeat(1.0).take(p + 1));
        let mut a = vec![vec![0.0; n]; n];
        for (i, &u) in params.iter().enumerate() {
            let span = find_span(n - 1, p, u, &knots);
            let b = basis_funs(span, u, p, &knots);
            for j in 0..=p {
                a[i][span - p + j] = b[j];
            }
        }
        let ctrl = solve_dense(a, pts.to_vec());
        Self::from_points(p, knots, &ctrl)
    }

    /// Joins curves that meet end to start into one curve, each piece
    /// taking a parameter span equal to its length. Joints are C0 knots.
    pub fn join(curves: &[NurbsCurve]) -> Self {
        let p = curves.iter().map(|c| c.degree).max().unwrap();
        let mut knots: Vec<f64> = Vec::new();
        let mut ctrl: Vec<DVec4> = Vec::new();
        let mut start = 0.0;
        for (i, c) in curves.iter().enumerate() {
            let len = c.length().max(1e-12);
            let mut c = c.elevated_to(p).reparameterized(start, start + len);
            if i == 0 {
                knots.extend_from_slice(&c.knots[..c.knots.len() - 1]);
                ctrl.extend_from_slice(&c.ctrl);
            } else {
                // Scale the homogeneous points so the shared end weights agree.
                let s = ctrl.last().unwrap().w / c.ctrl[0].w;
                c.ctrl.iter_mut().for_each(|h| *h *= s);
                knots.extend_from_slice(&c.knots[p + 1..c.knots.len() - 1]);
                ctrl.extend_from_slice(&c.ctrl[1..]);
            }
            start += len;
        }
        knots.push(start);
        Self::new(p, knots, ctrl)
    }

    /// Closed C2 cubic through `pts` (the first point is not repeated),
    /// uniformly parameterized. Built as the periodic B-spline whose knots
    /// land on the points, then written out as clamped Bézier pieces so the
    /// rest of the kernel never sees an unclamped knot vector.
    pub fn interpolate_closed(pts: &[DVec3]) -> Self {
        let h: Vec<DVec4> = pts.iter().map(|p| p.extend(1.0)).collect();
        let (knots, ctrl) = interpolate_closed_hom(&h);
        Self::new(3, knots, ctrl)
    }

    /// Polyline approximation with chord deviation below `tol`.
    pub fn tessellate(&self, tol: f64) -> Vec<(f64, DVec3)> {
        let (a, b) = self.domain();
        tessellate_param(|t| self.point(t), a, b, tol, self.ctrl.len().max(2) * 2)
    }
}

fn intersect_tangents(p0: DVec3, t0: DVec3, p2: DVec3, t2: DVec3) -> DVec3 {
    // Solve p0 + a t0 = p2 + b t2 in least-squares sense.
    let w = p0 - p2;
    let a = t0.dot(t0);
    let b = t0.dot(t2);
    let c = t2.dot(t2);
    let d = t0.dot(w);
    let e = t2.dot(w);
    let den = a * c - b * b;
    if den.abs() < 1e-18 {
        return (p0 + p2) * 0.5;
    }
    let s = (b * e - c * d) / den;
    p0 + t0 * s
}

/// Closed C2 cubic interpolation of homogeneous points at uniform
/// parameters on [0, 1] (the first point is not repeated). Returns the
/// clamped knot vector and control points of its Bézier pieces; interpolating
/// homogeneous points lets rows of a rational surface share one structure.
pub fn interpolate_closed_hom(pts: &[DVec4]) -> (Vec<f64>, Vec<DVec4>) {
    let n = pts.len();
    assert!(n >= 3);
    // (P[i-1] + 4 P[i] + P[i+1]) / 6 = Q[i], cyclically.
    let mut a = vec![vec![0.0; n]; n];
    for (i, row) in a.iter_mut().enumerate() {
        row[(i + n - 1) % n] += 1.0 / 6.0;
        row[i] += 4.0 / 6.0;
        row[(i + 1) % n] += 1.0 / 6.0;
    }
    let p = solve_dense(a, pts.to_vec());
    let mut ctrl = vec![pts[0]];
    let mut knots = vec![0.0; 4];
    for i in 0..n {
        let (p0, p1) = (p[i], p[(i + 1) % n]);
        ctrl.extend([(2.0 * p0 + p1) / 3.0, (p0 + 2.0 * p1) / 3.0, pts[(i + 1) % n]]);
        let k = (i + 1) as f64 / n as f64;
        knots.extend(if i + 1 == n { vec![k; 4] } else { vec![k; 3] });
    }
    (knots, ctrl)
}

/// Gauss–Legendre arc length of a parameterized speed function.
pub fn gauss_length(speed: impl Fn(f64) -> f64, a: f64, b: f64, segments: usize) -> f64 {
    const X: [f64; 5] = [0.0, -0.5384693101056831, 0.5384693101056831, -0.9061798459386640, 0.9061798459386640];
    const W: [f64; 5] = [0.5688888888888889, 0.4786286704993665, 0.4786286704993665, 0.2369268850561891, 0.2369268850561891];
    let h = (b - a) / segments as f64;
    let mut total = 0.0;
    for s in 0..segments {
        let m = a + h * (s as f64 + 0.5);
        for k in 0..5 {
            total += W[k] * speed(m + X[k] * h * 0.5) * h * 0.5;
        }
    }
    total
}

/// Newton iteration for the closest point on a parametric curve given
/// `derivs(t) -> [C, C', C'']`.
pub fn newton_closest(derivs: impl Fn(f64) -> Vec<DVec3>, p: DVec3, t0: f64, a: f64, b: f64) -> f64 {
    let mut t = t0;
    for _ in 0..30 {
        let d = derivs(t);
        let diff = d[0] - p;
        let f = d[1].dot(diff);
        let df = d[2].dot(diff) + d[1].length_squared();
        if df.abs() < 1e-300 {
            break;
        }
        let nt = (t - f / df).clamp(a, b);
        if (nt - t).abs() < 1e-14 * (1.0 + t.abs()) {
            t = nt;
            break;
        }
        t = nt;
    }
    t
}

/// Adaptive subdivision of a parametric curve until the midpoint deviation
/// from the chord is below `tol`. Starts from `initial` uniform segments.
pub fn tessellate_param(f: impl Fn(f64) -> DVec3, a: f64, b: f64, tol: f64, initial: usize) -> Vec<(f64, DVec3)> {
    let mut out = vec![(a, f(a))];
    let n = initial.max(1);
    let mut stack = Vec::new();
    for i in (0..n).rev() {
        let t0 = a + (b - a) * i as f64 / n as f64;
        let t1 = a + (b - a) * (i + 1) as f64 / n as f64;
        stack.push((t0, f(t0), t1, f(t1), 0));
    }
    while let Some((t0, p0, t1, p1, depth)) = stack.pop() {
        let tm = 0.5 * (t0 + t1);
        let pm = f(tm);
        let chord = p1 - p0;
        let len2 = chord.length_squared();
        let dev = if len2 < 1e-30 {
            pm.distance(p0)
        } else {
            let s = (pm - p0).dot(chord) / len2;
            pm.distance(p0 + chord * s)
        };
        if dev > tol && depth < 16 {
            stack.push((tm, pm, t1, p1, depth + 1));
            stack.push((t0, p0, tm, pm, depth + 1));
        } else {
            out.push((t1, p1));
        }
    }
    out
}

/// Solves `A x = B` for vector right-hand sides by Gaussian elimination with
/// partial pivoting. Intended for small, well-conditioned systems.
pub fn solve_dense<T>(mut a: Vec<Vec<f64>>, mut b: Vec<T>) -> Vec<T>
where
    T: Copy + std::ops::Sub<Output = T> + std::ops::Mul<f64, Output = T> + std::ops::Div<f64, Output = T>,
{
    let n = a.len();
    for col in 0..n {
        let piv = (col..n).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs())).unwrap();
        a.swap(col, piv);
        b.swap(col, piv);
        let d = a[col][col];
        if d.abs() < 1e-300 {
            continue;
        }
        for row in col + 1..n {
            let f = a[row][col] / d;
            if f == 0.0 {
                continue;
            }
            for k in col..n {
                a[row][k] -= f * a[col][k];
            }
            b[row] = b[row] - b[col] * f;
        }
    }
    for col in (0..n).rev() {
        let mut v = b[col];
        for k in col + 1..n {
            v = v - b[k] * a[col][k];
        }
        b[col] = v / a[col][col];
    }
    b
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn join_keeps_pieces() {
        let f = Frame::WORLD;
        let a = NurbsCurve::line(DVec3::new(1.0, -1.0, 0.0), DVec3::new(1.0, 0.0, 0.0));
        let b = NurbsCurve::arc(&f, 1.0, 0.0, PI);
        let c = NurbsCurve::line(DVec3::new(-1.0, 0.0, 0.0), DVec3::new(1.0, -1.0, 0.0));
        let j = NurbsCurve::join(&[a.clone(), b.clone(), c.clone()]);
        assert!((j.length() - (a.length() + b.length() + c.length())).abs() < 1e-6);
        assert!(j.point(j.domain().0).distance(DVec3::new(1.0, -1.0, 0.0)) < 1e-12);
        assert!(j.point(j.domain().1).distance(DVec3::new(1.0, -1.0, 0.0)) < 1e-12);
        // The arc part stays round.
        for i in 0..=20 {
            let t = 1.0 + PI * i as f64 / 20.0;
            assert!((j.point(t).length() - 1.0).abs() < 1e-9, "{t}");
        }
    }

    #[test]
    fn closed_interpolation_is_c2_through_points() {
        let q = [DVec3::new(0.0, 0.0, 0.0), DVec3::new(4.0, -1.0, 0.0), DVec3::new(5.0, 3.0, 0.0), DVec3::new(1.0, 4.0, 0.0), DVec3::new(-1.0, 2.0, 0.0)];
        let c = NurbsCurve::interpolate_closed(&q);
        for (i, p) in q.iter().enumerate() {
            assert!(c.point(i as f64 / q.len() as f64).distance(*p) < 1e-12);
        }
        // Second derivatives agree across every piece boundary, including the closure.
        for i in 0..q.len() {
            let k = i as f64 / q.len() as f64;
            let before = c.derivs(if i == 0 { 1.0 } else { k - 1e-9 }, 2);
            let after = c.derivs(k + 1e-9, 2);
            assert!(before[1].distance(after[1]) < 1e-6, "C1 at {i}");
            assert!(before[2].distance(after[2]) < 1e-4 * before[2].length().max(1.0), "C2 at {i}");
        }
    }

    #[test]
    fn circle_is_round() {
        let f = Frame::new(DVec3::new(1.0, 2.0, 3.0), DVec3::new(0.3, 0.2, 1.0), DVec3::X);
        let c = NurbsCurve::circle(&f, 2.5);
        let (a, b) = c.domain();
        assert!((b - a - TAU).abs() < 1e-12);
        for i in 0..=100 {
            let t = a + (b - a) * i as f64 / 100.0;
            let p = c.point(t);
            assert!((p.distance(f.origin) - 2.5).abs() < 1e-9, "radius off at {t}");
            assert!(f.to_local(p).z.abs() < 1e-9);
        }
        assert!((c.length() - TAU * 2.5).abs() < 1e-6);
    }

    #[test]
    fn arc_endpoints() {
        let c = NurbsCurve::arc(&Frame::WORLD, 1.0, 0.0, PI * 0.75);
        let (a, b) = c.domain();
        assert!(c.point(a).distance(DVec3::X) < 1e-12);
        let e = DVec3::new((PI * 0.75).cos(), (PI * 0.75).sin(), 0.0);
        assert!(c.point(b).distance(e) < 1e-12);
    }

    #[test]
    fn split_and_insert_preserve_shape() {
        let pts = [DVec3::ZERO, DVec3::new(1.0, 2.0, 0.0), DVec3::new(3.0, -1.0, 1.0), DVec3::new(4.0, 0.0, 0.0), DVec3::new(6.0, 1.0, 2.0)];
        let c = NurbsCurve::uniform(3, &pts);
        let ins = c.insert_knot(0.3, 2);
        let (l, r) = c.split(0.6);
        for i in 0..=20 {
            let t = i as f64 / 20.0;
            assert!(c.point(t).distance(ins.point(t)) < 1e-12);
            let q = if t <= 0.6 { l.point(t) } else { r.point(t) };
            assert!(c.point(t).distance(q) < 1e-12, "split mismatch at {t}");
        }
        let e = c.elevated();
        assert_eq!(e.degree, 4);
        for i in 0..=20 {
            let t = i as f64 / 20.0;
            assert!(c.point(t).distance(e.point(t)) < 1e-10);
        }
    }

    #[test]
    fn derivatives_match_finite_difference() {
        let c = NurbsCurve::circle(&Frame::WORLD, 3.0);
        let t = 1.234;
        let d = c.derivs(t, 2);
        let h = 1e-6;
        let fd = (c.point(t + h) - c.point(t - h)) / (2.0 * h);
        assert!((d[1] - fd).length() < 1e-5);
        let fd2 = (c.point(t + 1e-4) - 2.0 * c.point(t) + c.point(t - 1e-4)) / 1e-8;
        assert!((d[2] - fd2).length() < 1e-3 * d[2].length().max(1.0));
    }

    #[test]
    fn interpolation_hits_points() {
        let pts: Vec<DVec3> = (0..7).map(|i| DVec3::new(i as f64, (i as f64).sin(), 0.0)).collect();
        let c = NurbsCurve::interpolate(&pts, 3);
        for p in &pts {
            let t = c.closest_param(*p);
            assert!(c.point(t).distance(*p) < 1e-8);
        }
    }
}
