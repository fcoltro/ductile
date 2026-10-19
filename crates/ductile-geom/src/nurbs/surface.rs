// SPDX-License-Identifier: GPL-3.0-only
use super::basis::{basis_funs, binomial, ders_basis_funs, find_span};
use super::curve::{dehom, hom, solve_dense, NurbsCurve};
use crate::{BBox3, Frame};
use glam::{DMat4, DVec3, DVec4};
use serde::{Deserialize, Serialize};

/// Tensor-product NURBS surface. Control points are homogeneous and stored
/// row-major: `ctrl[i * nv + j]` where `i` runs along u and `j` along v.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct NurbsSurface {
    pub degree_u: usize,
    pub degree_v: usize,
    pub knots_u: Vec<f64>,
    pub knots_v: Vec<f64>,
    pub nu: usize,
    pub nv: usize,
    pub ctrl: Vec<DVec4>,
}

impl NurbsSurface {
    pub fn new(
        degree_u: usize,
        degree_v: usize,
        knots_u: Vec<f64>,
        knots_v: Vec<f64>,
        nu: usize,
        nv: usize,
        ctrl: Vec<DVec4>,
    ) -> Self {
        assert_eq!(knots_u.len(), nu + degree_u + 1);
        assert_eq!(knots_v.len(), nv + degree_v + 1);
        assert_eq!(ctrl.len(), nu * nv);
        Self { degree_u, degree_v, knots_u, knots_v, nu, nv, ctrl }
    }

    pub fn cp(&self, i: usize, j: usize) -> DVec4 {
        self.ctrl[i * self.nv + j]
    }

    /// True if the surface closes on itself in u: the first and last
    /// control columns coincide on clamped knots.
    pub fn closed_u(&self) -> bool {
        let size = self.ctrl.iter().map(|h| (h.truncate() / h.w).abs().max_element()).fold(1.0, f64::max);
        let clamped = self.knots_u[0] == self.knots_u[self.degree_u] && self.knots_u[self.nu] == self.knots_u[self.nu + self.degree_u];
        clamped && (0..self.nv).all(|j| {
            let (a, b) = (self.cp(0, j), self.cp(self.nu - 1, j));
            (a.truncate() / a.w).distance(b.truncate() / b.w) < 1e-9 * size
        })
    }

    /// True if the surface closes on itself in v.
    pub fn closed_v(&self) -> bool {
        let size = self.ctrl.iter().map(|h| (h.truncate() / h.w).abs().max_element()).fold(1.0, f64::max);
        let clamped = self.knots_v[0] == self.knots_v[self.degree_v] && self.knots_v[self.nv] == self.knots_v[self.nv + self.degree_v];
        clamped && (0..self.nu).all(|i| {
            let (a, b) = (self.cp(i, 0), self.cp(i, self.nv - 1));
            (a.truncate() / a.w).distance(b.truncate() / b.w) < 1e-9 * size
        })
    }

    pub fn domain(&self) -> ((f64, f64), (f64, f64)) {
        (
            (self.knots_u[self.degree_u], self.knots_u[self.nu]),
            (self.knots_v[self.degree_v], self.knots_v[self.nv]),
        )
    }

    pub fn point(&self, u: f64, v: f64) -> DVec3 {
        let (p, q) = (self.degree_u, self.degree_v);
        let su = find_span(self.nu - 1, p, u, &self.knots_u);
        let sv = find_span(self.nv - 1, q, v, &self.knots_v);
        let bu = basis_funs(su, u, p, &self.knots_u);
        let bv = basis_funs(sv, v, q, &self.knots_v);
        let mut s = DVec4::ZERO;
        for k in 0..=p {
            let mut tmp = DVec4::ZERO;
            for l in 0..=q {
                tmp += self.cp(su - p + k, sv - q + l) * bv[l];
            }
            s += tmp * bu[k];
        }
        dehom(s)
    }

    /// Derivatives `skl[k][l]` = ∂^(k+l) S / ∂u^k ∂v^l for k + l <= d (A3.6 + A4.4).
    /// The normal curvature at (u, v) along the tangent direction nearest
    /// `dir`, signed about the normal Su × Sv (positive bending towards it).
    pub fn normal_curvature(&self, u: f64, v: f64, dir: DVec3) -> Option<f64> {
        let d = self.derivs(u, v, 2);
        let n = d[1][0].cross(d[0][1]).try_normalize()?;
        crate::surface::normal_curvature_along([d[1][0], d[0][1], d[2][0], d[1][1], d[0][2]], n, dir)
    }

    pub fn derivs(&self, u: f64, v: f64, d: usize) -> Vec<Vec<DVec3>> {
        let (p, q) = (self.degree_u, self.degree_v);
        let du = d.min(p);
        let dv = d.min(q);
        let su = find_span(self.nu - 1, p, u, &self.knots_u);
        let sv = find_span(self.nv - 1, q, v, &self.knots_v);
        let nu = ders_basis_funs(su, u, p, du, &self.knots_u);
        let nv = ders_basis_funs(sv, v, q, dv, &self.knots_v);
        let mut aw = vec![vec![DVec4::ZERO; d + 1]; d + 1];
        for k in 0..=du {
            let mut temp = vec![DVec4::ZERO; q + 1];
            for s in 0..=q {
                for r in 0..=p {
                    temp[s] += self.cp(su - p + r, sv - q + s) * nu[k][r];
                }
            }
            let dd = (d - k).min(dv);
            for l in 0..=dd {
                for s in 0..=q {
                    aw[k][l] += temp[s] * nv[l][s];
                }
            }
        }
        let mut skl = vec![vec![DVec3::ZERO; d + 1]; d + 1];
        for k in 0..=d {
            for l in 0..=(d - k) {
                let mut v = aw[k][l].truncate();
                for j in 1..=l {
                    v -= skl[k][l - j] * (binomial(l, j) * aw[0][j].w);
                }
                for i in 1..=k {
                    v -= skl[k - i][l] * (binomial(k, i) * aw[i][0].w);
                    let mut v2 = DVec3::ZERO;
                    for j in 1..=l {
                        v2 += skl[k - i][l - j] * (binomial(l, j) * aw[i][j].w);
                    }
                    v -= v2 * binomial(k, i);
                }
                skl[k][l] = v / aw[0][0].w;
            }
        }
        skl
    }

    pub fn transformed(&self, m: &DMat4) -> Self {
        let mut s = self.clone();
        for c in s.ctrl.iter_mut() {
            *c = hom(m.transform_point3(dehom(*c)), c.w);
        }
        s
    }

    pub fn bbox(&self) -> BBox3 {
        BBox3::from_points(self.ctrl.iter().map(|c| dehom(*c)))
    }

    /// Swaps the roles of u and v (flips the normal).
    /// The same surface one degree higher along u (the shape is unchanged).
    pub fn elevated_u(&self) -> Self {
        let cols: Vec<NurbsCurve> = (0..self.nv).map(|j| NurbsCurve::new(self.degree_u, self.knots_u.clone(), (0..self.nu).map(|i| self.ctrl[i * self.nv + j]).collect()).elevated()).collect();
        let nu = cols[0].ctrl.len();
        let mut ctrl = vec![DVec4::ZERO; nu * self.nv];
        for (j, c) in cols.iter().enumerate() {
            for i in 0..nu {
                ctrl[i * self.nv + j] = c.ctrl[i];
            }
        }
        Self::new(self.degree_u + 1, self.degree_v, cols[0].knots.clone(), self.knots_v.clone(), nu, self.nv, ctrl)
    }

    /// The same surface one degree higher along v.
    pub fn elevated_v(&self) -> Self {
        self.transposed().elevated_u().transposed()
    }

    pub fn transposed(&self) -> Self {
        let mut ctrl = Vec::with_capacity(self.ctrl.len());
        for j in 0..self.nv {
            for i in 0..self.nu {
                ctrl.push(self.cp(i, j));
            }
        }
        Self::new(self.degree_v, self.degree_u, self.knots_v.clone(), self.knots_u.clone(), self.nv, self.nu, ctrl)
    }

    /// Reverses the u direction (flips the normal).
    pub fn reversed_u(&self) -> Self {
        let (a, b) = (self.knots_u[0], *self.knots_u.last().unwrap());
        let knots_u = self.knots_u.iter().rev().map(|k| a + b - k).collect();
        let mut ctrl = Vec::with_capacity(self.ctrl.len());
        for i in (0..self.nu).rev() {
            for j in 0..self.nv {
                ctrl.push(self.cp(i, j));
            }
        }
        Self::new(self.degree_u, self.degree_v, knots_u, self.knots_v.clone(), self.nu, self.nv, ctrl)
    }

    /// Iso-parametric curve at constant u.
    pub fn iso_u(&self, u: f64) -> NurbsCurve {
        let p = self.degree_u;
        let span = find_span(self.nu - 1, p, u, &self.knots_u);
        let b = basis_funs(span, u, p, &self.knots_u);
        let ctrl = (0..self.nv)
            .map(|j| (0..=p).map(|k| self.cp(span - p + k, j) * b[k]).sum())
            .collect();
        NurbsCurve::new(self.degree_v, self.knots_v.clone(), ctrl)
    }

    /// Iso-parametric curve at constant v.
    pub fn iso_v(&self, v: f64) -> NurbsCurve {
        let q = self.degree_v;
        let span = find_span(self.nv - 1, q, v, &self.knots_v);
        let b = basis_funs(span, v, q, &self.knots_v);
        let ctrl = (0..self.nu)
            .map(|i| (0..=q).map(|k| self.cp(i, span - q + k) * b[k]).sum())
            .collect();
        NurbsCurve::new(self.degree_u, self.knots_u.clone(), ctrl)
    }

    /// Bilinear patch through four corners `p00, p10, p01, p11` (u, v).
    pub fn bilinear(p00: DVec3, p10: DVec3, p01: DVec3, p11: DVec3) -> Self {
        let k = vec![0.0, 0.0, 1.0, 1.0];
        Self::new(1, 1, k.clone(), k, 2, 2, vec![hom(p00, 1.0), hom(p01, 1.0), hom(p10, 1.0), hom(p11, 1.0)])
    }

    /// Surface of linear extrusion: u follows the curve, v the direction.
    pub fn extrusion(curve: &NurbsCurve, dir: DVec3) -> Self {
        let nu = curve.ctrl.len();
        let mut ctrl = Vec::with_capacity(nu * 2);
        for c in &curve.ctrl {
            ctrl.push(*c);
            ctrl.push(hom(dehom(*c) + dir, c.w));
        }
        Self::new(curve.degree, 1, curve.knots.clone(), vec![0.0, 0.0, 1.0, 1.0], nu, 2, ctrl)
    }

    /// Surface of revolution of `profile` about the axis through `origin` along
    /// `axis`, sweeping `angle` radians. u runs around the axis, v along the profile.
    pub fn revolution(profile: &NurbsCurve, origin: DVec3, axis: DVec3, angle: f64) -> Self {
        let axis = axis.normalize();
        // Use an exact unit-circle arc to obtain knots and weights for u.
        let unit = NurbsCurve::arc(&Frame::WORLD, 1.0, 0.0, angle);
        let nu = unit.ctrl.len();
        let nv = profile.ctrl.len();
        let mut ctrl = vec![DVec4::ZERO; nu * nv];
        for (j, pc) in profile.ctrl.iter().enumerate() {
            let p = dehom(*pc);
            let wp = pc.w;
            // Local frame for this profile point.
            let o = origin + axis * (p - origin).dot(axis);
            let rvec = p - o;
            let r = rvec.length();
            let x = if r > 1e-14 { rvec / r } else { crate::any_perpendicular(axis) };
            let y = axis.cross(x);
            for (i, uc) in unit.ctrl.iter().enumerate() {
                let q = dehom(*uc);
                let world = o + x * (q.x * r) + y * (q.y * r);
                ctrl[i * nv + j] = hom(world, uc.w * wp);
            }
        }
        Self::new(2, profile.degree, unit.knots.clone(), profile.knots.clone(), nu, nv, ctrl)
    }

    /// Ruled surface between two curves (made compatible first). v runs from a to b.
    pub fn ruled(a: &NurbsCurve, b: &NurbsCurve) -> Self {
        let cs = make_compatible(&[a.clone(), b.clone()]);
        let nu = cs[0].ctrl.len();
        let mut ctrl = Vec::with_capacity(nu * 2);
        for i in 0..nu {
            ctrl.push(cs[0].ctrl[i]);
            ctrl.push(cs[1].ctrl[i]);
        }
        Self::new(cs[0].degree, 1, cs[0].knots.clone(), vec![0.0, 0.0, 1.0, 1.0], nu, 2, ctrl)
    }

    /// Bilinearly blended Coons patch filling four boundary curves:
    /// `bottom` (v = 0) and `top` (v = 1) both running in +u, `left`
    /// (u = 0) and `right` (u = 1) both running in +v, meeting at their
    /// ends. Exact for polynomial boundaries; rational ones are first
    /// approximated by cubics, so the patch then meets them within the
    /// approximation's tolerance.
    pub fn coons(bottom: &NurbsCurve, right: &NurbsCurve, top: &NurbsCurve, left: &NurbsCurve) -> Self {
        let poly = |c: &NurbsCurve| {
            if !c.is_rational() {
                return c.clone();
            }
            let (a, b) = c.domain();
            let n = 8 * c.ctrl.len().max(4);
            let pts: Vec<DVec3> = (0..=n).map(|i| c.point(a + (b - a) * i as f64 / n as f64)).collect();
            NurbsCurve::interpolate(&pts, 3)
        };
        let u = make_compatible(&[poly(bottom), poly(top)]);
        let v = make_compatible(&[poly(left), poly(right)]);
        let greville = |c: &NurbsCurve| -> Vec<f64> { (0..c.ctrl.len()).map(|i| c.knots[i + 1..=i + c.degree].iter().sum::<f64>() / c.degree as f64).collect() };
        let (h, g) = (greville(&u[0]), greville(&v[0]));
        let (p00, p10, p01, p11) = (bottom.point(bottom.domain().0), bottom.point(bottom.domain().1), top.point(top.domain().0), top.point(top.domain().1));
        let (nu, nv) = (h.len(), g.len());
        let mut ctrl = Vec::with_capacity(nu * nv);
        for i in 0..nu {
            for j in 0..nv {
                let a = dehom(u[0].ctrl[i]) * (1.0 - g[j]) + dehom(u[1].ctrl[i]) * g[j];
                let b = dehom(v[0].ctrl[j]) * (1.0 - h[i]) + dehom(v[1].ctrl[j]) * h[i];
                let c = (p00 * (1.0 - h[i]) + p10 * h[i]) * (1.0 - g[j]) + (p01 * (1.0 - h[i]) + p11 * h[i]) * g[j];
                ctrl.push(hom(a + b - c, 1.0));
            }
        }
        Self::new(u[0].degree, v[0].degree, u[0].knots.clone(), v[0].knots.clone(), nu, nv, ctrl)
    }

    /// Skinned (lofted) surface interpolating the section curves along v.
    pub fn loft(sections: &[NurbsCurve], degree_v: usize) -> Self {
        assert!(sections.len() >= 2);
        if sections.len() == 2 {
            return Self::ruled(&sections[0], &sections[1]);
        }
        let cs = make_compatible(sections);
        let k = cs.len();
        let nu = cs[0].ctrl.len();
        // Average chord parameterization across control columns.
        let mut params = vec![0.0; k];
        for i in 0..nu {
            let pts: Vec<DVec3> = cs.iter().map(|c| dehom(c.ctrl[i])).collect();
            let d: Vec<f64> = pts.windows(2).map(|w| w[0].distance(w[1])).collect();
            let tot: f64 = d.iter().sum::<f64>().max(1e-12);
            let mut acc = 0.0;
            for (s, dd) in d.iter().enumerate() {
                acc += dd / tot;
                params[s + 1] += acc / nu as f64;
            }
        }
        params[k - 1] = 1.0;
        Self::skin(cs, degree_v, &params)
    }

    /// Loft with the sections placed at the given v parameters (increasing,
    /// 0 to 1). Surfaces lofted from matching sections with the same
    /// parameters share their boundary iso-curves exactly.
    pub fn loft_with_params(sections: &[NurbsCurve], degree_v: usize, params: &[f64]) -> Self {
        assert!(sections.len() >= 2 && params.len() == sections.len());
        if sections.len() == 2 {
            return Self::ruled(&sections[0], &sections[1]);
        }
        Self::skin(make_compatible(sections), degree_v, params)
    }

    fn skin(cs: Vec<NurbsCurve>, degree_v: usize, params: &[f64]) -> Self {
        let k = cs.len();
        let q = degree_v.min(k - 1);
        let nu = cs[0].ctrl.len();
        let mut knots_v = vec![0.0; q + 1];
        for j in 1..k - q {
            knots_v.push(params[j..j + q].iter().sum::<f64>() / q as f64);
        }
        knots_v.extend(std::iter::repeat(1.0).take(q + 1));
        let mut a = vec![vec![0.0; k]; k];
        for (r, &v) in params.iter().enumerate() {
            let span = find_span(k - 1, q, v, &knots_v);
            let b = basis_funs(span, v, q, &knots_v);
            for j in 0..=q {
                a[r][span - q + j] = b[j];
            }
        }
        let mut ctrl = vec![DVec4::ZERO; nu * k];
        for i in 0..nu {
            let rhs: Vec<DVec4> = cs.iter().map(|c| c.ctrl[i]).collect();
            let sol = solve_dense(a.clone(), rhs);
            for (j, s) in sol.into_iter().enumerate() {
                ctrl[i * k + j] = s;
            }
        }
        Self::new(cs[0].degree, q, cs[0].knots.clone(), knots_v, nu, k, ctrl)
    }

    /// Planar rectangle patch spanning the frame's XY plane.
    pub fn plane_patch(frame: &Frame, u0: f64, u1: f64, v0: f64, v1: f64) -> Self {
        let p = |u: f64, v: f64| frame.to_world(DVec3::new(u, v, 0.0));
        Self::bilinear(p(u0, v0), p(u1, v0), p(u0, v1), p(u1, v1))
    }

    /// Interpolating surface through a grid of points `grid[i][j]`, with
    /// centripetal parameters averaged over the rows and columns (A9.4).
    pub fn interpolate_grid(grid: &[Vec<DVec3>], p: usize, q: usize) -> Self {
        let average = |lines: Vec<Vec<DVec3>>| -> Vec<f64> {
            let n = lines[0].len();
            let mut acc = vec![0.0; n];
            let mut count = 0;
            for l in &lines {
                let d: Vec<f64> = l.windows(2).map(|w| w[0].distance(w[1]).sqrt()).collect();
                let total: f64 = d.iter().sum();
                if total <= 1e-12 {
                    continue;
                }
                count += 1;
                let mut s = 0.0;
                for (i, x) in d.iter().enumerate() {
                    s += x / total;
                    acc[i + 1] += s;
                }
            }
            if count == 0 {
                return (0..n).map(|i| i as f64 / (n - 1) as f64).collect();
            }
            let mut out: Vec<f64> = acc.iter().map(|x| x / count as f64).collect();
            out[n - 1] = 1.0;
            out
        };
        let us = average((0..grid[0].len()).map(|j| grid.iter().map(|r| r[j]).collect()).collect());
        let vs = average(grid.to_vec());
        Self::interpolate_grid_with_params(grid, p, q, &us, &vs)
    }

    /// Interpolating surface through `grid[i][j]` at parameters
    /// `(us[i], vs[j])`, each increasing from 0 to 1.
    pub fn interpolate_grid_with_params(grid: &[Vec<DVec3>], p: usize, q: usize, us: &[f64], vs: &[f64]) -> Self {
        // Interpolate along v for every row, then along u for every column;
        // shared parameters give every row the same knots.
        let rows: Vec<NurbsCurve> = grid.iter().map(|r| NurbsCurve::interpolate_with_params(r, q, vs)).collect();
        let q = rows[0].degree;
        let knots_v = rows[0].knots.clone();
        let nvc = rows[0].ctrl.len();
        let cols: Vec<NurbsCurve> = (0..nvc)
            .map(|j| {
                let pts: Vec<DVec3> = rows.iter().map(|r| dehom(r.ctrl[j])).collect();
                NurbsCurve::interpolate_with_params(&pts, p, us)
            })
            .collect();
        let p = cols[0].degree;
        let knots_u = cols[0].knots.clone();
        let nuc = cols[0].ctrl.len();
        let mut ctrl = vec![DVec4::ZERO; nuc * nvc];
        for (j, c) in cols.iter().enumerate() {
            for i in 0..nuc {
                ctrl[i * nvc + j] = c.ctrl[i];
            }
        }
        Self::new(p, q, knots_u, knots_v, nuc, nvc, ctrl)
    }
}

/// Brings curves to a common degree, parameter domain and knot vector.
pub fn make_compatible(curves: &[NurbsCurve]) -> Vec<NurbsCurve> {
    let deg = curves.iter().map(|c| c.degree).max().unwrap();
    let mut cs: Vec<NurbsCurve> = curves.iter().map(|c| c.elevated_to(deg).reparameterized(0.0, 1.0)).collect();
    // Union of interior knots with max multiplicity.
    let mut all: Vec<(f64, usize)> = Vec::new();
    for c in &cs {
        let (a, b) = c.domain();
        let mut i = 0;
        while i < c.knots.len() {
            let k = c.knots[i];
            let mut m = 0;
            while i < c.knots.len() && (c.knots[i] - k).abs() < 1e-12 {
                m += 1;
                i += 1;
            }
            if k > a + 1e-12 && k < b - 1e-12 {
                match all.iter_mut().find(|(x, _)| (*x - k).abs() < 1e-9) {
                    Some(e) => e.1 = e.1.max(m),
                    None => all.push((k, m)),
                }
            }
        }
    }
    for c in cs.iter_mut() {
        for &(k, m) in &all {
            let have = c.knots.iter().filter(|x| (**x - k).abs() < 1e-9).count();
            if have < m {
                *c = c.insert_knot(k, m - have);
            }
        }
        // Snap knots so all curves share bit-identical values.
        for x in c.knots.iter_mut() {
            if let Some((k, _)) = all.iter().find(|(k, _)| (*k - *x).abs() < 1e-9) {
                *x = *k;
            }
        }
    }
    cs
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::TAU;

    #[test]
    fn revolution_makes_cylinder() {
        let prof = NurbsCurve::line(DVec3::new(2.0, 0.0, 0.0), DVec3::new(2.0, 0.0, 5.0));
        let s = NurbsSurface::revolution(&prof, DVec3::ZERO, DVec3::Z, TAU);
        let ((u0, u1), (v0, v1)) = s.domain();
        for i in 0..=12 {
            for j in 0..=4 {
                let u = u0 + (u1 - u0) * i as f64 / 12.0;
                let v = v0 + (v1 - v0) * j as f64 / 4.0;
                let p = s.point(u, v);
                assert!((p.truncate().length() - 2.0).abs() < 1e-9);
            }
        }
    }

    #[test]
    fn surface_derivs_fd() {
        let prof = NurbsCurve::interpolate(&[DVec3::new(1.0, 0.0, 0.0), DVec3::new(2.0, 0.0, 1.0), DVec3::new(1.5, 0.0, 2.0), DVec3::new(1.0, 0.0, 3.0)], 3);
        let s = NurbsSurface::revolution(&prof, DVec3::ZERO, DVec3::Z, 4.0);
        let (u, v) = (1.3, 0.4);
        let d = s.derivs(u, v, 2);
        let h = 1e-6;
        let su = (s.point(u + h, v) - s.point(u - h, v)) / (2.0 * h);
        let sv = (s.point(u, v + h) - s.point(u, v - h)) / (2.0 * h);
        assert!((d[1][0] - su).length() < 1e-5);
        assert!((d[0][1] - sv).length() < 1e-5);
        assert!((d[0][0] - s.point(u, v)).length() < 1e-12);
    }

    #[test]
    fn loft_interpolates_sections() {
        let f = |z: f64, r: f64| NurbsCurve::circle(&Frame::new(DVec3::new(0.0, 0.0, z), DVec3::Z, DVec3::X), r);
        let secs = vec![f(0.0, 1.0), f(1.0, 2.0), f(2.0, 1.5), f(4.0, 1.0)];
        let s = NurbsSurface::loft(&secs, 3);
        let p = s.point(0.25, 0.0);
        assert!((p.truncate().length() - 1.0).abs() < 1e-9);
        let p = s.point(0.25, 1.0);
        assert!((p.z - 4.0).abs() < 1e-9);
    }

    #[test]
    fn coons_patch_meets_its_boundary() {
        let bottom = NurbsCurve::uniform(3, &[DVec3::ZERO, DVec3::new(1.0, 0.0, 0.5), DVec3::new(2.0, 0.0, -0.3), DVec3::new(3.0, 0.0, 0.0)]);
        let top = NurbsCurve::uniform(2, &[DVec3::new(0.0, 2.0, 0.0), DVec3::new(1.5, 2.5, 1.0), DVec3::new(3.0, 2.0, 0.0)]);
        let left = NurbsCurve::line(DVec3::ZERO, DVec3::new(0.0, 2.0, 0.0));
        let right = NurbsCurve::uniform(2, &[DVec3::new(3.0, 0.0, 0.0), DVec3::new(3.5, 1.0, 0.0), DVec3::new(3.0, 2.0, 0.0)]);
        let s = NurbsSurface::coons(&bottom, &right, &top, &left);
        for k in 0..=10 {
            let t = k as f64 / 10.0;
            assert!(s.point(t, 0.0).distance(bottom.point(t)) < 1e-12);
            assert!(s.point(t, 1.0).distance(top.point(t)) < 1e-12);
            assert!(s.point(0.0, t).distance(left.point(t)) < 1e-12);
            assert!(s.point(1.0, t).distance(right.point(t)) < 1e-12);
        }
    }
}
