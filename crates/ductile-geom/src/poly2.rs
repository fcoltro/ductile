// SPDX-License-Identifier: GPL-3.0-only
//! Small 2D polygon utilities used by trimming and tessellation.

use glam::DVec2;

/// Signed area (positive for counter-clockwise).
pub fn signed_area(poly: &[DVec2]) -> f64 {
    let n = poly.len();
    let mut a = 0.0;
    for i in 0..n {
        let p = poly[i];
        let q = poly[(i + 1) % n];
        a += p.x * q.y - q.x * p.y;
    }
    0.5 * a
}

/// Even-odd point in polygon test.
pub fn point_in_polygon(p: DVec2, poly: &[DVec2]) -> bool {
    let mut inside = false;
    let n = poly.len();
    let mut j = n - 1;
    for i in 0..n {
        let (a, b) = (poly[i], poly[j]);
        if (a.y > p.y) != (b.y > p.y) {
            let x = a.x + (p.y - a.y) / (b.y - a.y) * (b.x - a.x);
            if p.x < x {
                inside = !inside;
            }
        }
        j = i;
    }
    inside
}

/// Even-odd test against a set of loops (outer + holes).
pub fn point_in_loops(p: DVec2, loops: &[Vec<DVec2>]) -> bool {
    loops.iter().filter(|l| l.len() >= 3 && point_in_polygon(p, l)).count() % 2 == 1
}

/// Distance from `p` to segment `ab`.
pub fn dist_to_segment(p: DVec2, a: DVec2, b: DVec2) -> f64 {
    let ab = b - a;
    let l2 = ab.length_squared();
    if l2 < 1e-300 {
        return p.distance(a);
    }
    let t = ((p - a).dot(ab) / l2).clamp(0.0, 1.0);
    p.distance(a + ab * t)
}

/// Proper or touching intersection of segments `ab` and `cd`.
/// Returns parameters `(s, t)` along each segment.
pub fn segment_intersection(a: DVec2, b: DVec2, c: DVec2, d: DVec2) -> Option<(f64, f64)> {
    let r = b - a;
    let s = d - c;
    let den = r.perp_dot(s);
    if den.abs() < 1e-300 {
        return None;
    }
    let ac = c - a;
    let t = ac.perp_dot(s) / den;
    let u = ac.perp_dot(r) / den;
    if (-1e-12..=1.0 + 1e-12).contains(&t) && (-1e-12..=1.0 + 1e-12).contains(&u) {
        Some((t, u))
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn square() {
        let sq = [DVec2::new(0.0, 0.0), DVec2::new(1.0, 0.0), DVec2::new(1.0, 1.0), DVec2::new(0.0, 1.0)];
        assert!((signed_area(&sq) - 1.0).abs() < 1e-12);
        assert!(point_in_polygon(DVec2::new(0.5, 0.5), &sq));
        assert!(!point_in_polygon(DVec2::new(1.5, 0.5), &sq));
    }
}
