// SPDX-License-Identifier: GPL-3.0-only
//! Orbit camera with perspective and orthographic projections.

use glam::{DMat4, DVec2, DVec3, Mat4};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Projection {
    Perspective,
    Orthographic,
}

#[derive(Clone, Debug)]
pub struct Camera {
    pub target: DVec3,
    pub distance: f64,
    /// Rotation about world Z (radians).
    pub yaw: f64,
    /// Elevation above the XY plane (radians).
    pub pitch: f64,
    pub fov_y: f64,
    pub projection: Projection,
    pub aspect: f64,
    /// Perspective squish: above 1 the camera backs off and narrows its
    /// view by the same factor, flattening perspective while keeping the
    /// framing (a dolly zoom); below 1 it moves in and widens.
    pub squish: f64,
}

impl Default for Camera {
    fn default() -> Self {
        Self {
            target: DVec3::ZERO,
            distance: 30.0,
            yaw: -0.9,
            pitch: 0.55,
            fov_y: 35f64.to_radians(),
            projection: Projection::Perspective,
            aspect: 1.5,
            squish: 1.0,
        }
    }
}

/// A ray in world space.
#[derive(Clone, Copy, Debug)]
pub struct Ray {
    pub origin: DVec3,
    pub dir: DVec3,
}

impl Ray {
    pub fn at(&self, t: f64) -> DVec3 {
        self.origin + self.dir * t
    }

    /// Intersection with a plane, returning the hit point.
    pub fn hit_plane(&self, origin: DVec3, normal: DVec3) -> Option<DVec3> {
        let d = self.dir.dot(normal);
        if d.abs() < 1e-12 {
            return None;
        }
        let t = (origin - self.origin).dot(normal) / d;
        Some(self.at(t))
    }
}

impl Camera {
    /// Sets the squish, moving the camera and changing its field of view so
    /// the target stays framed the same.
    pub fn set_squish(&mut self, s: f64) {
        let s = s.clamp(0.25, 8.0);
        let k = s / self.squish;
        self.fov_y = 2.0 * ((self.fov_y * 0.5).tan() / k).atan();
        self.distance *= k;
        self.squish = s;
    }

    /// Direction from the target towards the eye.
    pub fn back(&self) -> DVec3 {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        DVec3::new(cp * cy, cp * sy, sp)
    }

    pub fn eye(&self) -> DVec3 {
        self.target + self.back() * self.distance
    }

    pub fn up(&self) -> DVec3 {
        let right = self.right();
        self.back().cross(right).normalize()
    }

    pub fn right(&self) -> DVec3 {
        let b = self.back();
        let r = DVec3::Z.cross(b);
        if r.length_squared() < 1e-12 {
            DVec3::new(-self.yaw.sin(), self.yaw.cos(), 0.0)
        } else {
            r.normalize()
        }
    }

    pub fn view(&self) -> DMat4 {
        look_at_rh(self.eye(), self.target, self.up())
    }

    fn near_far(&self) -> (f64, f64) {
        let near = (self.distance * 0.002).max(1e-4);
        let far = self.distance * 200.0 + 1000.0;
        (near, far)
    }

    pub fn proj(&self) -> DMat4 {
        let (near, far) = self.near_far();
        match self.projection {
            Projection::Perspective => perspective_rh(self.fov_y, self.aspect, near, far),
            Projection::Orthographic => {
                let h = self.distance * (self.fov_y * 0.5).tan();
                let w = h * self.aspect;
                // Ortho camera sits far back so geometry behind the target is visible.
                orthographic_rh(-w, w, -h, h, -far, far)
            }
        }
    }

    pub fn view_proj(&self) -> DMat4 {
        self.proj() * self.view()
    }

    pub fn view_proj_f32(&self) -> Mat4 {
        self.view_proj().as_mat4()
    }

    /// World-space ray through a point in normalized device coordinates.
    pub fn ray(&self, ndc: DVec2) -> Ray {
        let inv = self.view_proj().inverse();
        match self.projection {
            Projection::Perspective => {
                let far = inv.project_point3(DVec3::new(ndc.x, ndc.y, 1.0));
                let origin = self.eye();
                Ray { origin, dir: (far - origin).normalize() }
            }
            Projection::Orthographic => {
                let a = inv.project_point3(DVec3::new(ndc.x, ndc.y, 0.0));
                let b = inv.project_point3(DVec3::new(ndc.x, ndc.y, 1.0));
                let dir = (b - a).normalize();
                Ray { origin: a - dir * self.distance * 100.0, dir }
            }
        }
    }

    /// Projects a world point to NDC; returns `None` behind the camera.
    pub fn project(&self, p: DVec3) -> Option<DVec3> {
        let c = self.view_proj() * p.extend(1.0);
        if c.w <= 1e-9 {
            return None;
        }
        Some(c.truncate() / c.w)
    }

    /// World units per screen pixel at the given point for a viewport `height_px` tall.
    pub fn world_per_pixel(&self, at: DVec3, height_px: f64) -> f64 {
        let depth = match self.projection {
            Projection::Perspective => (at - self.eye()).dot(-self.back()).max(1e-6),
            Projection::Orthographic => self.distance,
        };
        2.0 * depth * (self.fov_y * 0.5).tan() / height_px.max(1.0)
    }

    pub fn orbit(&mut self, dx: f64, dy: f64) {
        self.yaw -= dx * 0.008;
        self.pitch = (self.pitch + dy * 0.008).clamp(-1.5607, 1.5607);
    }

    pub fn pan(&mut self, dx: f64, dy: f64, height_px: f64) {
        let s = self.world_per_pixel(self.target, height_px);
        self.target += (-self.right() * dx + self.up() * dy) * s;
    }

    /// Zooms towards a world point under the cursor.
    pub fn zoom(&mut self, factor: f64, toward: Option<DVec3>) {
        let new_d = (self.distance * factor).clamp(1e-3, 1e6);
        if let Some(p) = toward {
            let k = 1.0 - new_d / self.distance;
            self.target += (p - self.target) * k;
        }
        self.distance = new_d;
    }

    /// Frames a bounding sphere.
    pub fn frame(&mut self, center: DVec3, radius: f64) {
        self.target = center;
        self.distance = (radius.max(1e-3) / (self.fov_y * 0.5).sin()) * 1.1;
    }

    /// Snap to a standard view direction (`back` = eye direction from target).
    pub fn set_view(&mut self, back: DVec3) {
        let b = back.normalize();
        self.pitch = b.z.asin();
        if b.x.abs() > 1e-9 || b.y.abs() > 1e-9 {
            self.yaw = b.y.atan2(b.x);
        } else {
            self.yaw = -std::f64::consts::FRAC_PI_2;
        }
    }
}

/// Right-handed view matrix.
pub fn look_at_rh(eye: DVec3, center: DVec3, up: DVec3) -> DMat4 {
    let f = (center - eye).normalize();
    let s = f.cross(up).normalize();
    let u = s.cross(f);
    DMat4::from_cols(
        glam::DVec4::new(s.x, u.x, -f.x, 0.0),
        glam::DVec4::new(s.y, u.y, -f.y, 0.0),
        glam::DVec4::new(s.z, u.z, -f.z, 0.0),
        glam::DVec4::new(-s.dot(eye), -u.dot(eye), f.dot(eye), 1.0),
    )
}

/// Right-handed perspective projection with depth in `[0, 1]`.
pub fn perspective_rh(fov_y: f64, aspect: f64, near: f64, far: f64) -> DMat4 {
    let h = 1.0 / (0.5 * fov_y).tan();
    let w = h / aspect;
    let r = far / (near - far);
    DMat4::from_cols(
        glam::DVec4::new(w, 0.0, 0.0, 0.0),
        glam::DVec4::new(0.0, h, 0.0, 0.0),
        glam::DVec4::new(0.0, 0.0, r, -1.0),
        glam::DVec4::new(0.0, 0.0, r * near, 0.0),
    )
}

/// Right-handed orthographic projection with depth in `[0, 1]`.
pub fn orthographic_rh(left: f64, right: f64, bottom: f64, top: f64, near: f64, far: f64) -> DMat4 {
    let rcp_w = 1.0 / (right - left);
    let rcp_h = 1.0 / (top - bottom);
    let r = 1.0 / (near - far);
    DMat4::from_cols(
        glam::DVec4::new(2.0 * rcp_w, 0.0, 0.0, 0.0),
        glam::DVec4::new(0.0, 2.0 * rcp_h, 0.0, 0.0),
        glam::DVec4::new(0.0, 0.0, r, 0.0),
        glam::DVec4::new(-(left + right) * rcp_w, -(top + bottom) * rcp_h, r * near, 1.0),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn squish_keeps_the_target_framed() {
        let mut c = Camera { aspect: 1.0, ..Default::default() };
        // A point beside the target, square to the view.
        let view = (c.target - c.eye()).normalize();
        let edge = c.target + view.cross(DVec3::Z).cross(view).normalize() * 5.0;
        let before = c.project(edge).unwrap();
        c.set_squish(3.0);
        let after = c.project(edge).unwrap();
        assert!((before.y - after.y).abs() < 1e-9, "{before} {after}");
        assert!(c.distance > 80.0 && c.fov_y < 35f64.to_radians());
        c.set_squish(1.0);
        assert!((c.fov_y - 35f64.to_radians()).abs() < 1e-12 && (c.distance - 30.0).abs() < 1e-9);
    }
}
