// SPDX-License-Identifier: GPL-3.0-only
use glam::{DMat3, DMat4, DVec3};
use serde::{Deserialize, Serialize};

/// Right-handed orthonormal coordinate frame. Analytic surfaces and curves
/// are defined in the local coordinates of a frame, with `z` as their axis.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Frame {
    pub origin: DVec3,
    pub x: DVec3,
    pub y: DVec3,
    pub z: DVec3,
}

impl Default for Frame {
    fn default() -> Self {
        Self::WORLD
    }
}

impl Frame {
    pub const WORLD: Frame = Frame { origin: DVec3::ZERO, x: DVec3::X, y: DVec3::Y, z: DVec3::Z };

    /// Builds a frame from an origin, a normal (z) and a hint for x. The hint
    /// is projected to be perpendicular to the normal.
    pub fn new(origin: DVec3, z: DVec3, x_hint: DVec3) -> Frame {
        let z = z.normalize();
        let mut x = x_hint - z * x_hint.dot(z);
        if x.length_squared() < 1e-24 {
            x = crate::any_perpendicular(z);
        }
        let x = x.normalize();
        let y = z.cross(x);
        Frame { origin, x, y, z }
    }

    /// A frame with the given z axis and an arbitrary but deterministic x.
    pub fn from_normal(origin: DVec3, z: DVec3) -> Frame {
        let z = z.normalize();
        // Prefer world X as the x direction so planes facing Z look "natural".
        let hint = if z.cross(DVec3::X).length_squared() > 1e-6 { DVec3::X } else { DVec3::Y };
        Frame::new(origin, z, hint)
    }

    pub fn to_world(&self, local: DVec3) -> DVec3 {
        self.origin + self.x * local.x + self.y * local.y + self.z * local.z
    }

    pub fn dir_to_world(&self, local: DVec3) -> DVec3 {
        self.x * local.x + self.y * local.y + self.z * local.z
    }

    pub fn to_local(&self, world: DVec3) -> DVec3 {
        let d = world - self.origin;
        DVec3::new(d.dot(self.x), d.dot(self.y), d.dot(self.z))
    }

    pub fn dir_to_local(&self, d: DVec3) -> DVec3 {
        DVec3::new(d.dot(self.x), d.dot(self.y), d.dot(self.z))
    }

    pub fn matrix(&self) -> DMat4 {
        DMat4::from_cols(self.x.extend(0.0), self.y.extend(0.0), self.z.extend(0.0), self.origin.extend(1.0))
    }

    /// Applies a rigid (or uniformly scaled) transform. Axes are renormalized.
    /// Under a mirror transform the result is left-handed, so that every
    /// point expressed in the frame maps exactly to the mirrored point.
    pub fn transformed(&self, m: &DMat4) -> Frame {
        let o = m.transform_point3(self.origin);
        let x = m.transform_vector3(self.x).normalize();
        let y = m.transform_vector3(self.y).normalize();
        let z = m.transform_vector3(self.z).normalize();
        Frame { origin: o, x, y, z }
    }

    pub fn is_right_handed(&self) -> bool {
        self.x.cross(self.y).dot(self.z) > 0.0
    }

    pub fn rotation(&self) -> DMat3 {
        DMat3::from_cols(self.x, self.y, self.z)
    }

    pub fn flipped(&self) -> Frame {
        Frame { origin: self.origin, x: self.x, y: -self.y, z: -self.z }
    }
}
