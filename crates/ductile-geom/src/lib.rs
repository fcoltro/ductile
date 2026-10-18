// SPDX-License-Identifier: GPL-3.0-only
//! Geometry layer of the Ductile modeling kernel.
//!
//! Everything here is pure geometry with no topology: points, frames,
//! bounding boxes, NURBS curves and surfaces, and the analytic curve and
//! surface types (lines, circles, planes, cylinders, ...) that a B-rep face
//! or edge can carry. All computation is in `f64`.

pub mod bbox;
pub mod curve;
pub mod frame;
pub mod nurbs;
pub mod poly2;
pub mod surface;

pub use bbox::BBox3;
pub use curve::Curve3;
pub use frame::Frame;
pub use glam::{DMat3, DMat4, DQuat, DVec2, DVec3, DVec4};
pub use nurbs::{NurbsCurve, NurbsSurface};
pub use surface::Surface;

/// Linear modeling tolerance in model units (millimetres by convention).
/// Two points closer than this are considered coincident.
pub const LINEAR_TOL: f64 = 1e-6;
/// Angular tolerance in radians.
pub const ANGULAR_TOL: f64 = 1e-9;
/// Tolerance for parameter-space comparisons.
pub const PARAM_TOL: f64 = 1e-10;

/// Returns a unit vector perpendicular to `v` (which need not be normalized).
pub fn any_perpendicular(v: DVec3) -> DVec3 {
    let a = v.abs();
    let other = if a.x <= a.y && a.x <= a.z {
        DVec3::X
    } else if a.y <= a.z {
        DVec3::Y
    } else {
        DVec3::Z
    };
    v.cross(other).normalize()
}

/// Wraps an angle into `[lo, lo + 2π)`.
pub fn wrap_angle(a: f64, lo: f64) -> f64 {
    let tau = std::f64::consts::TAU;
    let mut r = (a - lo) % tau;
    if r < 0.0 {
        r += tau;
    }
    lo + r
}
