// SPDX-License-Identifier: GPL-3.0-only
//! Curves that a B-rep edge can carry.

use crate::nurbs::curve::{newton_closest, tessellate_param};
use crate::{wrap_angle, BBox3, Frame, NurbsCurve};
use glam::{DMat4, DVec3};
use serde::{Deserialize, Serialize};
use std::f64::consts::TAU;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Curve3 {
    /// `origin + t * dir`, `dir` unit length; t is arc length.
    Line { origin: DVec3, dir: DVec3 },
    /// Circle in the frame's XY plane; t is the angle from frame x.
    Circle { frame: Frame, radius: f64 },
    /// Ellipse in the frame's XY plane; t is the eccentric angle.
    Ellipse { frame: Frame, rx: f64, ry: f64 },
    Nurbs(NurbsCurve),
}

impl Curve3 {
    pub fn line_through(a: DVec3, b: DVec3) -> Curve3 {
        Curve3::Line { origin: a, dir: (b - a).normalize() }
    }

    /// Period of a closed periodic curve (circles and ellipses).
    pub fn period(&self) -> Option<f64> {
        match self {
            Curve3::Circle { .. } | Curve3::Ellipse { .. } => Some(TAU),
            _ => None,
        }
    }

    /// Natural parameter domain. Lines are unbounded, so a nominal huge range is returned.
    pub fn natural_domain(&self) -> (f64, f64) {
        match self {
            Curve3::Line { .. } => (-1e12, 1e12),
            Curve3::Circle { .. } | Curve3::Ellipse { .. } => (0.0, TAU),
            Curve3::Nurbs(n) => n.domain(),
        }
    }

    pub fn point(&self, t: f64) -> DVec3 {
        match self {
            Curve3::Line { origin, dir } => *origin + *dir * t,
            Curve3::Circle { frame, radius } => frame.to_world(DVec3::new(radius * t.cos(), radius * t.sin(), 0.0)),
            Curve3::Ellipse { frame, rx, ry } => frame.to_world(DVec3::new(rx * t.cos(), ry * t.sin(), 0.0)),
            Curve3::Nurbs(n) => n.point(t),
        }
    }

    /// `[C(t), C'(t), C''(t)]`.
    pub fn derivs(&self, t: f64) -> [DVec3; 3] {
        match self {
            Curve3::Line { origin, dir } => [*origin + *dir * t, *dir, DVec3::ZERO],
            Curve3::Circle { frame, radius } => {
                let (s, c) = t.sin_cos();
                [
                    frame.to_world(DVec3::new(radius * c, radius * s, 0.0)),
                    frame.dir_to_world(DVec3::new(-radius * s, radius * c, 0.0)),
                    frame.dir_to_world(DVec3::new(-radius * c, -radius * s, 0.0)),
                ]
            }
            Curve3::Ellipse { frame, rx, ry } => {
                let (s, c) = t.sin_cos();
                [
                    frame.to_world(DVec3::new(rx * c, ry * s, 0.0)),
                    frame.dir_to_world(DVec3::new(-rx * s, ry * c, 0.0)),
                    frame.dir_to_world(DVec3::new(-rx * c, -ry * s, 0.0)),
                ]
            }
            Curve3::Nurbs(n) => {
                let d = n.derivs(t, 2);
                [d[0], d[1], d[2]]
            }
        }
    }

    pub fn tangent(&self, t: f64) -> DVec3 {
        self.derivs(t)[1]
    }

    /// Closest parameter to `p` within `[t0, t1]`.
    pub fn closest_param_in(&self, p: DVec3, t0: f64, t1: f64) -> f64 {
        match self {
            Curve3::Line { origin, dir } => (p - *origin).dot(*dir).clamp(t0, t1),
            Curve3::Circle { frame, .. } => {
                let l = frame.to_local(p);
                let a = l.y.atan2(l.x);
                clamp_periodic(a, t0, t1)
            }
            _ => {
                let n = 64;
                let mut best = (f64::INFINITY, t0);
                for i in 0..=n {
                    let t = t0 + (t1 - t0) * i as f64 / n as f64;
                    let d = self.point(t).distance_squared(p);
                    if d < best.0 {
                        best = (d, t);
                    }
                }
                newton_closest(|t| self.derivs(t).to_vec(), p, best.1, t0, t1)
            }
        }
    }

    /// Closest parameter over the natural domain (periodic curves return a
    /// value in `[0, 2π)`).
    pub fn closest_param(&self, p: DVec3) -> f64 {
        let (a, b) = self.natural_domain();
        self.closest_param_in(p, a, b)
    }

    pub fn transformed(&self, m: &DMat4) -> Curve3 {
        match self {
            Curve3::Line { origin, dir } => {
                let o = m.transform_point3(*origin);
                let d = m.transform_vector3(*dir);
                // Keep t as arc length; non-uniform scale is handled by NURBS conversion.
                Curve3::Line { origin: o, dir: d.normalize() }
            }
            Curve3::Circle { frame, radius } => {
                let s = m.transform_vector3(frame.x).length();
                Curve3::Circle { frame: frame.transformed(m), radius: radius * s }
            }
            Curve3::Ellipse { frame, rx, ry } => {
                let sx = m.transform_vector3(frame.x).length();
                let sy = m.transform_vector3(frame.y).length();
                Curve3::Ellipse { frame: frame.transformed(m), rx: rx * sx, ry: ry * sy }
            }
            Curve3::Nurbs(n) => Curve3::Nurbs(n.transformed(m)),
        }
    }

    /// Exact NURBS representation of the piece on `[t0, t1]`. The resulting
    /// NURBS is parameterized on `[t0, t1]` too, so parameters carry over.
    pub fn to_nurbs(&self, t0: f64, t1: f64) -> NurbsCurve {
        match self {
            Curve3::Line { .. } => NurbsCurve::line(self.point(t0), self.point(t1)).reparameterized(t0, t1),
            Curve3::Circle { frame, radius } => NurbsCurve::arc(frame, *radius, t0, t1 - t0).reparameterized(t0, t1),
            Curve3::Ellipse { frame, rx, ry } => {
                // Scale a unit circle arc anisotropically; exact for ellipses.
                let c = NurbsCurve::arc(&Frame::WORLD, 1.0, t0, t1 - t0);
                let m = frame.matrix() * DMat4::from_scale(DVec3::new(*rx, *ry, 1.0));
                c.transformed(&m).reparameterized(t0, t1)
            }
            Curve3::Nurbs(n) => {
                let (a, b) = n.domain();
                if (t0 - a).abs() < 1e-12 && (t1 - b).abs() < 1e-12 {
                    n.clone()
                } else {
                    n.subcurve(t0, t1)
                }
            }
        }
    }

    /// Parameters and points sampled so that chord deviation is below `tol`.
    pub fn tessellate(&self, t0: f64, t1: f64, tol: f64) -> Vec<(f64, DVec3)> {
        match self {
            Curve3::Line { .. } => vec![(t0, self.point(t0)), (t1, self.point(t1))],
            Curve3::Circle { radius, .. } => {
                let n = arc_segments(*radius, (t1 - t0).abs(), tol);
                (0..=n)
                    .map(|i| {
                        let t = t0 + (t1 - t0) * i as f64 / n as f64;
                        (t, self.point(t))
                    })
                    .collect()
            }
            Curve3::Ellipse { rx, ry, .. } => {
                let n = arc_segments(rx.max(*ry), (t1 - t0).abs(), tol);
                (0..=n)
                    .map(|i| {
                        let t = t0 + (t1 - t0) * i as f64 / n as f64;
                        (t, self.point(t))
                    })
                    .collect()
            }
            Curve3::Nurbs(n) => {
                let init = (n.ctrl.len() * 2).max(4);
                tessellate_param(|t| n.point(t), t0, t1, tol, init)
            }
        }
    }

    pub fn length(&self, t0: f64, t1: f64) -> f64 {
        match self {
            Curve3::Line { .. } => (t1 - t0).abs(),
            Curve3::Circle { radius, .. } => radius * (t1 - t0).abs(),
            _ => crate::nurbs::curve::gauss_length(|t| self.derivs(t)[1].length(), t0, t1, 64),
        }
    }

    pub fn bbox(&self, t0: f64, t1: f64) -> BBox3 {
        // The curve stays within the chord tolerance of its polyline.
        let tol = 1e-3;
        BBox3::from_points(self.tessellate(t0, t1, tol).into_iter().map(|(_, p)| p)).expanded(tol)
    }

    /// True if the curve is a straight line (or a degree-1 two-point NURBS).
    pub fn is_linear(&self) -> bool {
        match self {
            Curve3::Line { .. } => true,
            Curve3::Nurbs(n) => n.degree == 1 && n.ctrl.len() == 2,
            _ => false,
        }
    }
}

/// Number of uniform segments needed so that an arc of `radius` and `sweep`
/// has a sagitta below `tol`.
pub fn arc_segments(radius: f64, sweep: f64, tol: f64) -> usize {
    if radius <= tol {
        return 1;
    }
    let max_step = 2.0 * (1.0 - tol / radius).clamp(-1.0, 1.0).acos();
    let max_step = max_step.clamp(1e-3, std::f64::consts::FRAC_PI_4);
    ((sweep / max_step).ceil() as usize).clamp(1, 4096)
}

/// Clamps a periodic angle to `[t0, t1]`, choosing the representative of
/// `a` (mod 2π) that is inside or closest to the interval.
pub fn clamp_periodic(a: f64, t0: f64, t1: f64) -> f64 {
    let w = wrap_angle(a, t0);
    if w <= t1 {
        return w;
    }
    // Outside the interval: pick the nearer endpoint (accounting for wrap).
    let d_end = w - t1;
    let d_start = t0 + TAU - w;
    if d_end < d_start {
        t1
    } else {
        t0
    }
}
