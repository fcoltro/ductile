// SPDX-License-Identifier: GPL-3.0-only
//! Surfaces that a B-rep face can carry.
//!
//! Analytic surfaces are kept analytic (exact evaluation, closed-form
//! projection and intersection); everything else is a NURBS surface.

use crate::{wrap_angle, BBox3, Frame, NurbsSurface};
use glam::{DMat4, DVec2, DVec3};
use serde::{Deserialize, Serialize};
use std::f64::consts::{FRAC_PI_2, PI, TAU};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Surface {
    /// `origin + u x + v y`; normal `z`.
    Plane { frame: Frame },
    /// u = angle about z, v = height along z. Normal points away from the axis.
    Cylinder { frame: Frame, radius: f64 },
    /// Radius at v is `radius + v·tan(half_angle)`. u = angle, v = height along z.
    Cone { frame: Frame, radius: f64, half_angle: f64 },
    /// u = longitude, v = latitude in `[-π/2, π/2]`. Normal points outward.
    Sphere { frame: Frame, radius: f64 },
    /// u = angle about z, v = angle around the tube. Normal points outward.
    Torus { frame: Frame, major: f64, minor: f64 },
    Nurbs(NurbsSurface),
}

/// Value and first/second partial derivatives of a surface.
#[derive(Clone, Copy, Debug)]
pub struct SurfDerivs {
    pub p: DVec3,
    pub du: DVec3,
    pub dv: DVec3,
    pub duu: DVec3,
    pub duv: DVec3,
    pub dvv: DVec3,
}

impl Surface {
    pub fn plane(origin: DVec3, normal: DVec3) -> Surface {
        Surface::Plane { frame: Frame::from_normal(origin, normal) }
    }

    pub fn frame(&self) -> Option<&Frame> {
        match self {
            Surface::Plane { frame }
            | Surface::Cylinder { frame, .. }
            | Surface::Cone { frame, .. }
            | Surface::Sphere { frame, .. }
            | Surface::Torus { frame, .. } => Some(frame),
            Surface::Nurbs(_) => None,
        }
    }

    /// Period in u, if the surface closes on itself in u.
    pub fn period_u(&self) -> Option<f64> {
        match self {
            Surface::Cylinder { .. } | Surface::Cone { .. } | Surface::Sphere { .. } | Surface::Torus { .. } => Some(TAU),
            Surface::Nurbs(n) if n.closed_u() => Some(n.domain().0 .1 - n.domain().0 .0),
            _ => None,
        }
    }

    pub fn period_v(&self) -> Option<f64> {
        match self {
            Surface::Torus { .. } => Some(TAU),
            Surface::Nurbs(n) if n.closed_v() => Some(n.domain().1 .1 - n.domain().1 .0),
            _ => None,
        }
    }

    /// Start of the base period in each periodic direction.
    pub fn period_start(&self) -> DVec2 {
        match self {
            Surface::Nurbs(n) => DVec2::new(n.domain().0 .0, n.domain().1 .0),
            _ => DVec2::ZERO,
        }
    }

    /// Wraps periodic coordinates into the base period `[start, start + period)`,
    /// with `slack` letting values just below the start stay put.
    pub fn wrap_periodic(&self, mut uv: DVec2, slack: f64) -> DVec2 {
        let st = self.period_start();
        if let Some(p) = self.period_u() {
            uv.x = st.x - slack + (uv.x - st.x + slack).rem_euclid(p);
        }
        if let Some(p) = self.period_v() {
            uv.y = st.y - slack + (uv.y - st.y + slack).rem_euclid(p);
        }
        uv
    }

    /// Parameters of a periodic NURBS surface brought into its domain.
    fn nurbs_param(n: &NurbsSurface, u: f64, v: f64) -> (f64, f64) {
        let ((u0, u1), (v0, v1)) = n.domain();
        let u = if (u < u0 || u > u1) && n.closed_u() { u0 + (u - u0).rem_euclid(u1 - u0) } else { u };
        let v = if (v < v0 || v > v1) && n.closed_v() { v0 + (v - v0).rem_euclid(v1 - v0) } else { v };
        (u, v)
    }

    /// Parameter bounds; `None` means unbounded in that direction.
    pub fn bounds(&self) -> (Option<(f64, f64)>, Option<(f64, f64)>) {
        match self {
            Surface::Plane { .. } => (None, None),
            Surface::Cylinder { .. } | Surface::Cone { .. } => (Some((0.0, TAU)), None),
            Surface::Sphere { .. } => (Some((0.0, TAU)), Some((-FRAC_PI_2, FRAC_PI_2))),
            Surface::Torus { .. } => (Some((0.0, TAU)), Some((0.0, TAU))),
            Surface::Nurbs(n) => {
                let (u, v) = n.domain();
                (Some(u), Some(v))
            }
        }
    }

    pub fn point(&self, u: f64, v: f64) -> DVec3 {
        match self {
            Surface::Plane { frame } => frame.to_world(DVec3::new(u, v, 0.0)),
            Surface::Cylinder { frame, radius } => frame.to_world(DVec3::new(radius * u.cos(), radius * u.sin(), v)),
            Surface::Cone { frame, radius, half_angle } => {
                let r = radius + v * half_angle.tan();
                frame.to_world(DVec3::new(r * u.cos(), r * u.sin(), v))
            }
            Surface::Sphere { frame, radius } => {
                let cv = v.cos();
                frame.to_world(DVec3::new(radius * cv * u.cos(), radius * cv * u.sin(), radius * v.sin()))
            }
            Surface::Torus { frame, major, minor } => {
                let r = major + minor * v.cos();
                frame.to_world(DVec3::new(r * u.cos(), r * u.sin(), minor * v.sin()))
            }
            Surface::Nurbs(n) => {
                let (u, v) = Self::nurbs_param(n, u, v);
                n.point(u, v)
            }
        }
    }

    pub fn derivs(&self, u: f64, v: f64) -> SurfDerivs {
        let (su, cu) = u.sin_cos();
        let (sv, cv) = v.sin_cos();
        let local = |f: &Frame, p: DVec3, du: DVec3, dv: DVec3, duu: DVec3, duv: DVec3, dvv: DVec3| SurfDerivs {
            p: f.to_world(p),
            du: f.dir_to_world(du),
            dv: f.dir_to_world(dv),
            duu: f.dir_to_world(duu),
            duv: f.dir_to_world(duv),
            dvv: f.dir_to_world(dvv),
        };
        match self {
            Surface::Plane { frame } => SurfDerivs {
                p: self.point(u, v),
                du: frame.x,
                dv: frame.y,
                duu: DVec3::ZERO,
                duv: DVec3::ZERO,
                dvv: DVec3::ZERO,
            },
            Surface::Cylinder { frame, radius: r } => local(
                frame,
                DVec3::new(r * cu, r * su, v),
                DVec3::new(-r * su, r * cu, 0.0),
                DVec3::Z,
                DVec3::new(-r * cu, -r * su, 0.0),
                DVec3::ZERO,
                DVec3::ZERO,
            ),
            Surface::Cone { frame, radius, half_angle } => {
                let t = half_angle.tan();
                let r = radius + v * t;
                local(
                    frame,
                    DVec3::new(r * cu, r * su, v),
                    DVec3::new(-r * su, r * cu, 0.0),
                    DVec3::new(t * cu, t * su, 1.0),
                    DVec3::new(-r * cu, -r * su, 0.0),
                    DVec3::new(-t * su, t * cu, 0.0),
                    DVec3::ZERO,
                )
            }
            Surface::Sphere { frame, radius: r } => local(
                frame,
                DVec3::new(r * cv * cu, r * cv * su, r * sv),
                DVec3::new(-r * cv * su, r * cv * cu, 0.0),
                DVec3::new(-r * sv * cu, -r * sv * su, r * cv),
                DVec3::new(-r * cv * cu, -r * cv * su, 0.0),
                DVec3::new(r * sv * su, -r * sv * cu, 0.0),
                DVec3::new(-r * cv * cu, -r * cv * su, -r * sv),
            ),
            Surface::Torus { frame, major: big, minor: m } => {
                let r = big + m * cv;
                local(
                    frame,
                    DVec3::new(r * cu, r * su, m * sv),
                    DVec3::new(-r * su, r * cu, 0.0),
                    DVec3::new(-m * sv * cu, -m * sv * su, m * cv),
                    DVec3::new(-r * cu, -r * su, 0.0),
                    DVec3::new(m * sv * su, -m * sv * cu, 0.0),
                    DVec3::new(-m * cv * cu, -m * cv * su, -m * sv),
                )
            }
            Surface::Nurbs(n) => {
                let (u, v) = Self::nurbs_param(n, u, v);
                let d = n.derivs(u, v, 2);
                SurfDerivs { p: d[0][0], du: d[1][0], dv: d[0][1], duu: d[2][0], duv: d[1][1], dvv: d[0][2] }
            }
        }
    }

    /// Unit normal at (u, v). Analytic surfaces return an exact normal even at
    /// parametric singularities (sphere poles, cone apex).
    pub fn normal(&self, u: f64, v: f64) -> DVec3 {
        match self {
            Surface::Plane { frame } => frame.z,
            Surface::Cylinder { frame, .. } => frame.dir_to_world(DVec3::new(u.cos(), u.sin(), 0.0)),
            Surface::Cone { frame, half_angle, .. } => {
                let (s, c) = half_angle.sin_cos();
                frame.dir_to_world(DVec3::new(c * u.cos(), c * u.sin(), -s))
            }
            Surface::Sphere { frame, .. } => frame.dir_to_world(DVec3::new(v.cos() * u.cos(), v.cos() * u.sin(), v.sin())),
            Surface::Torus { frame, .. } => frame.dir_to_world(DVec3::new(v.cos() * u.cos(), v.cos() * u.sin(), v.sin())),
            Surface::Nurbs(_) => {
                let d = self.derivs(u, v);
                let n = d.du.cross(d.dv);
                if n.length_squared() > 1e-24 {
                    return n.normalize();
                }
                // Degenerate point: nudge towards the domain interior.
                let ((u0, u1), (v0, v1)) = match self {
                    Surface::Nurbs(n) => n.domain(),
                    _ => unreachable!(),
                };
                let uu = u + (0.5 * (u0 + u1) - u) * 1e-4;
                let vv = v + (0.5 * (v0 + v1) - v) * 1e-4;
                let d = self.derivs(uu, vv);
                d.du.cross(d.dv).normalize_or_zero()
            }
        }
    }

    /// Projects `p` onto the surface, returning the closest (u, v).
    /// `hint` seeds the iterative solver for NURBS and resolves periodic
    /// ambiguity: periodic parameters are returned in the period nearest to it.
    pub fn project(&self, p: DVec3, hint: Option<DVec2>) -> DVec2 {
        let uv = match self {
            Surface::Plane { frame } => {
                let l = frame.to_local(p);
                DVec2::new(l.x, l.y)
            }
            Surface::Cylinder { frame, .. } => {
                let l = frame.to_local(p);
                DVec2::new(wrap_angle(l.y.atan2(l.x), 0.0), l.z)
            }
            Surface::Cone { frame, radius, half_angle } => {
                let l = frame.to_local(p);
                let u = wrap_angle(l.y.atan2(l.x), 0.0);
                // Project onto the generator line in the (rho, z) half plane.
                let rho = (l.x * l.x + l.y * l.y).sqrt();
                let (s, c) = half_angle.sin_cos();
                // Generator: (radius + t s, t c) for t along slant, v = t c.
                let t = (rho - radius) * s + l.z * c;
                DVec2::new(u, t * c)
            }
            Surface::Sphere { frame, .. } => {
                let l = frame.to_local(p);
                let rho = (l.x * l.x + l.y * l.y).sqrt();
                let u = if rho < 1e-14 { hint.map(|h| h.x).unwrap_or(0.0) } else { wrap_angle(l.y.atan2(l.x), 0.0) };
                DVec2::new(u, l.z.atan2(rho))
            }
            Surface::Torus { frame, major, .. } => {
                let l = frame.to_local(p);
                let u = wrap_angle(l.y.atan2(l.x), 0.0);
                let rho = (l.x * l.x + l.y * l.y).sqrt();
                DVec2::new(u, wrap_angle(l.z.atan2(rho - major), 0.0))
            }
            Surface::Nurbs(n) => {
                let uv = project_nurbs(n, p, hint.map(|h| self.wrap_periodic(h, 0.0)));
                return match hint {
                    Some(h) => self.align_periodic(uv, h),
                    None => uv,
                };
            }
        };
        match hint {
            Some(h) => self.align_periodic(uv, h),
            None => uv,
        }
    }

    /// Shifts periodic coordinates of `uv` by whole periods to be closest to `near`.
    pub fn align_periodic(&self, mut uv: DVec2, near: DVec2) -> DVec2 {
        if let Some(pu) = self.period_u() {
            uv.x += ((near.x - uv.x) / pu).round() * pu;
        }
        if let Some(pv) = self.period_v() {
            uv.y += ((near.y - uv.y) / pv).round() * pv;
        }
        uv
    }

    pub fn transformed(&self, m: &DMat4) -> Surface {
        let scale = m.transform_vector3(DVec3::X).length();
        match self {
            Surface::Plane { frame } => Surface::Plane { frame: frame.transformed(m) },
            Surface::Cylinder { frame, radius } => Surface::Cylinder { frame: frame.transformed(m), radius: radius * scale },
            Surface::Cone { frame, radius, half_angle } => {
                Surface::Cone { frame: frame.transformed(m), radius: radius * scale, half_angle: *half_angle }
            }
            Surface::Sphere { frame, radius } => Surface::Sphere { frame: frame.transformed(m), radius: radius * scale },
            Surface::Torus { frame, major, minor } => {
                Surface::Torus { frame: frame.transformed(m), major: major * scale, minor: minor * scale }
            }
            Surface::Nurbs(n) => Surface::Nurbs(n.transformed(m)),
        }
    }

    /// Converts the patch `[u0,u1]×[v0,v1]` to an exact NURBS surface.
    pub fn to_nurbs(&self, u0: f64, u1: f64, v0: f64, v1: f64) -> NurbsSurface {
        use crate::NurbsCurve;
        match self {
            Surface::Plane { frame } => {
                let mut s = NurbsSurface::plane_patch(frame, u0, u1, v0, v1);
                reparam(&mut s, u0, u1, v0, v1);
                s
            }
            Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. } => {
                let prof = NurbsCurve::line(self.point(u0, v0), self.point(u0, v1));
                let mut s = NurbsSurface::revolution(&prof, frame.origin, frame.z, u1 - u0);
                reparam(&mut s, u0, u1, v0, v1);
                s
            }
            Surface::Sphere { frame, radius } | Surface::Torus { frame, minor: radius, .. } => {
                let center = match self {
                    Surface::Torus { major, .. } => frame.origin + rot_dir(frame, u0) * *major,
                    _ => frame.origin,
                };
                let x = rot_dir(frame, u0);
                let pf = Frame { origin: center, x, y: frame.z, z: x.cross(frame.z) };
                let prof = NurbsCurve::arc(&pf, *radius, v0, v1 - v0);
                let mut s = NurbsSurface::revolution(&prof, frame.origin, frame.z, u1 - u0);
                reparam(&mut s, u0, u1, v0, v1);
                s
            }
            Surface::Nurbs(n) => n.clone(),
        }
    }

    /// Approximate bounding box of a parameter patch.
    pub fn patch_bbox(&self, u0: f64, u1: f64, v0: f64, v1: f64) -> BBox3 {
        let mut b = BBox3::EMPTY;
        let n = 8;
        for i in 0..=n {
            for j in 0..=n {
                let u = u0 + (u1 - u0) * i as f64 / n as f64;
                let v = v0 + (v1 - v0) * j as f64 / n as f64;
                b.add_point(self.point(u, v));
            }
        }
        b
    }

    pub fn is_planar(&self) -> bool {
        matches!(self, Surface::Plane { .. })
    }

    /// Signed distance-like classification for analytic solids bounded by this
    /// surface: positive outside along the normal.
    pub fn signed_distance(&self, p: DVec3) -> f64 {
        let uv = self.project(p, None);
        let q = self.point(uv.x, uv.y);
        (p - q).dot(self.normal(uv.x, uv.y))
    }
}

fn rot_dir(frame: &Frame, u: f64) -> DVec3 {
    frame.x * u.cos() + frame.y * u.sin()
}

fn reparam(s: &mut NurbsSurface, u0: f64, u1: f64, v0: f64, v1: f64) {
    let ((a, b), (c, d)) = s.domain();
    for k in s.knots_u.iter_mut() {
        *k = u0 + (*k - a) / (b - a) * (u1 - u0);
    }
    for k in s.knots_v.iter_mut() {
        *k = v0 + (*k - c) / (d - c) * (v1 - v0);
    }
}

/// Closest point projection on a NURBS surface by grid search + Newton.
pub fn project_nurbs(n: &NurbsSurface, p: DVec3, hint: Option<DVec2>) -> DVec2 {
    let ((u0, u1), (v0, v1)) = n.domain();
    // From a hint (the previous sample along a curve, usually), Newton
    // alone is enough when it converges without moving away.
    if let Some(h) = hint {
        let h = DVec2::new(h.x.clamp(u0, u1), h.y.clamp(v0, v1));
        let d0 = n.point(h.x, h.y).distance_squared(p);
        if d0 <= 1e-24 {
            return h;
        }
        let (uv, converged) = newton_project(n, p, h);
        let d = n.point(uv.x, uv.y).distance_squared(p);
        // Stopping on a closed surface's seam may just be Newton pushing
        // against it from the wrong side: only trust that when the point
        // is on the surface.
        let border = (n.closed_u() && (uv.x <= u0 || uv.x >= u1)) || (n.closed_v() && (uv.y <= v0 || uv.y >= v1));
        if converged && d <= d0 * (1.0 + 1e-9) && (!border || d < 1e-12) {
            return uv;
        }
    }
    let mut best = match hint {
        Some(h) => {
            let h = DVec2::new(h.x.clamp(u0, u1), h.y.clamp(v0, v1));
            (n.point(h.x, h.y).distance_squared(p), h)
        }
        None => (f64::INFINITY, DVec2::new(u0, v0)),
    };
    // A coarse grid, then Newton from its few nearest points: cheaper than
    // a fine grid and safer than a single start.
    let gu = (n.nu * 2).clamp(6, 32);
    let gv = (n.nv * 2).clamp(6, 32);
    let mut cands: Vec<(f64, DVec2)> = Vec::with_capacity((gu + 1) * (gv + 1));
    for i in 0..=gu {
        for j in 0..=gv {
            let u = u0 + (u1 - u0) * i as f64 / gu as f64;
            let v = v0 + (v1 - v0) * j as f64 / gv as f64;
            cands.push((n.point(u, v).distance_squared(p), DVec2::new(u, v)));
        }
    }
    cands.sort_by(|a, b| a.0.total_cmp(&b.0));
    for &(_, start) in cands.iter().take(3) {
        let uv = newton_project(n, p, start).0;
        let d = n.point(uv.x, uv.y).distance_squared(p);
        if d < best.0 {
            best = (d, uv);
        }
    }
    best.1
}

/// Newton iteration for the closest point from `uv`; also says whether it
/// converged.
fn newton_project(n: &NurbsSurface, p: DVec3, mut uv: DVec2) -> (DVec2, bool) {
    let ((u0, u1), (v0, v1)) = n.domain();
    let scale = (u1 - u0).abs().max((v1 - v0).abs()).max(1e-300);
    for _ in 0..30 {
        let d = n.derivs(uv.x, uv.y, 2);
        let r = d[0][0] - p;
        let (su, sv) = (d[1][0], d[0][1]);
        let (suu, suv, svv) = (d[2][0], d[1][1], d[0][2]);
        let f = su.dot(r);
        let g = sv.dot(r);
        let a = su.dot(su) + r.dot(suu);
        let b = su.dot(sv) + r.dot(suv);
        let c = sv.dot(sv) + r.dot(svv);
        let det = a * c - b * b;
        if det.abs() < 1e-300 {
            return (uv, false);
        }
        let du = (-f * c + g * b) / det;
        let dv = (f * b - g * a) / det;
        let nuv = DVec2::new((uv.x + du).clamp(u0, u1), (uv.y + dv).clamp(v0, v1));
        let step = (nuv - uv).length();
        uv = nuv;
        if step < 1e-14 * scale.max(1.0) {
            return (uv, true);
        }
    }
    (uv, false)
}

#[allow(dead_code)]
const _ASSERT_PI: f64 = PI;

#[cfg(test)]
mod tests {
    use super::*;

    fn check_projection(s: &Surface, u: f64, v: f64) {
        let p = s.point(u, v);
        let uv = s.project(p, Some(DVec2::new(u, v)));
        assert!(s.point(uv.x, uv.y).distance(p) < 1e-8, "{s:?} at {u},{v} -> {uv}");
    }

    #[test]
    fn analytic_projection_roundtrip() {
        let f = Frame::new(DVec3::new(1.0, -2.0, 0.5), DVec3::new(0.2, 1.0, 0.3), DVec3::X);
        let surfs = [
            Surface::Plane { frame: f },
            Surface::Cylinder { frame: f, radius: 2.0 },
            Surface::Cone { frame: f, radius: 2.0, half_angle: 0.3 },
            Surface::Sphere { frame: f, radius: 3.0 },
            Surface::Torus { frame: f, major: 5.0, minor: 1.0 },
        ];
        for s in &surfs {
            for &(u, v) in &[(0.3, 0.2), (2.0, -0.7), (5.0, 1.1)] {
                check_projection(s, u, v);
            }
        }
    }

    #[test]
    fn normals_match_derivs() {
        let f = Frame::WORLD;
        let surfs = [
            Surface::Cylinder { frame: f, radius: 2.0 },
            Surface::Cone { frame: f, radius: 2.0, half_angle: 0.3 },
            Surface::Sphere { frame: f, radius: 3.0 },
            Surface::Torus { frame: f, major: 5.0, minor: 1.0 },
        ];
        for s in &surfs {
            let d = s.derivs(0.7, 0.4);
            let n = d.du.cross(d.dv).normalize();
            assert!(n.dot(s.normal(0.7, 0.4)) > 0.999999, "{s:?}");
            let h = 1e-6;
            let fd = (s.point(0.7 + h, 0.4) - s.point(0.7 - h, 0.4)) / (2.0 * h);
            assert!((fd - d.du).length() < 1e-5);
        }
    }

    #[test]
    fn nurbs_conversion_matches() {
        let f = Frame::new(DVec3::new(0.0, 0.0, 1.0), DVec3::Z, DVec3::X);
        let surfs = [
            Surface::Cylinder { frame: f, radius: 2.0 },
            Surface::Sphere { frame: f, radius: 3.0 },
            Surface::Torus { frame: f, major: 5.0, minor: 1.0 },
            Surface::Cone { frame: f, radius: 2.0, half_angle: 0.3 },
        ];
        for s in &surfs {
            let (u0, u1, v0, v1) = match s {
                Surface::Sphere { .. } => (0.5, 4.0, -1.0, 1.2),
                Surface::Torus { .. } => (0.5, 4.0, 0.3, 5.0),
                _ => (0.5, 4.0, -1.0, 2.0),
            };
            let n = s.to_nurbs(u0, u1, v0, v1);
            for i in 0..=6 {
                for j in 0..=6 {
                    let u = u0 + (u1 - u0) * i as f64 / 6.0;
                    let v = v0 + (v1 - v0) * j as f64 / 6.0;
                    let p = n.point(u, v);
                    // Same surface (not necessarily same parameterization).
                    assert!(s.signed_distance(p).abs() < 1e-8, "{s:?} {u} {v}");
                }
            }
            // Corners coincide exactly.
            assert!(n.point(u0, v0).distance(s.point(u0, v0)) < 1e-9);
            assert!(n.point(u1, v1).distance(s.point(u1, v1)) < 1e-9);
        }
    }
}
