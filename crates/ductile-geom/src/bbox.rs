// SPDX-License-Identifier: GPL-3.0-only
use glam::DVec3;
use serde::{Deserialize, Serialize};

/// Axis-aligned bounding box. An empty box has `min > max`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct BBox3 {
    pub min: DVec3,
    pub max: DVec3,
}

impl Default for BBox3 {
    fn default() -> Self {
        Self::EMPTY
    }
}

impl BBox3 {
    pub const EMPTY: BBox3 = BBox3 {
        min: DVec3::splat(f64::INFINITY),
        max: DVec3::splat(f64::NEG_INFINITY),
    };

    pub fn from_points<I: IntoIterator<Item = DVec3>>(pts: I) -> Self {
        let mut b = Self::EMPTY;
        for p in pts {
            b.add_point(p);
        }
        b
    }

    pub fn is_empty(&self) -> bool {
        self.min.x > self.max.x
    }

    pub fn add_point(&mut self, p: DVec3) {
        self.min = self.min.min(p);
        self.max = self.max.max(p);
    }

    pub fn union(&self, o: &BBox3) -> BBox3 {
        BBox3 { min: self.min.min(o.min), max: self.max.max(o.max) }
    }

    pub fn expanded(&self, d: f64) -> BBox3 {
        BBox3 { min: self.min - DVec3::splat(d), max: self.max + DVec3::splat(d) }
    }

    pub fn intersects(&self, o: &BBox3) -> bool {
        !(self.is_empty() || o.is_empty())
            && self.min.x <= o.max.x
            && self.max.x >= o.min.x
            && self.min.y <= o.max.y
            && self.max.y >= o.min.y
            && self.min.z <= o.max.z
            && self.max.z >= o.min.z
    }

    pub fn contains(&self, p: DVec3) -> bool {
        p.cmpge(self.min).all() && p.cmple(self.max).all()
    }

    pub fn center(&self) -> DVec3 {
        (self.min + self.max) * 0.5
    }

    pub fn size(&self) -> DVec3 {
        if self.is_empty() {
            DVec3::ZERO
        } else {
            self.max - self.min
        }
    }

    pub fn diagonal(&self) -> f64 {
        self.size().length()
    }

    /// Slab test. Returns the entry distance along the ray if it hits.
    pub fn ray_hit(&self, origin: DVec3, dir: DVec3) -> Option<f64> {
        let inv = dir.recip();
        let t0 = (self.min - origin) * inv;
        let t1 = (self.max - origin) * inv;
        let tmin = t0.min(t1).max_element();
        let tmax = t0.max(t1).min_element();
        if tmax >= tmin.max(0.0) {
            Some(tmin.max(0.0))
        } else {
            None
        }
    }
}
