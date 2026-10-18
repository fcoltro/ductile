// SPDX-License-Identifier: GPL-3.0-only
//! Object snapping while drawing and placing: the points a pointer locks
//! onto, each with the label shown beside the marker, as in Plasticity
//! (End, Middle, 1/4, 3/4, Center, Intersection, Perpendicular, Tangent,
//! the X / Y / Z lines through the last point, then curves and faces).

use crate::camera::{Camera, Ray};
use crate::document::Document;
use crate::picking::{self, FaceHit};
use ductile_brep::EdgeId;
use ductile_geom::{Curve3, DVec2, DVec3, Frame};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Snap {
    pub point: DVec3,
    pub label: &'static str,
}

/// Screen distance within which a point snap takes over.
const POINT_PX: f64 = 10.0;
/// Screen distance within which an inference line or a curve takes over.
const LINE_PX: f64 = 7.0;

struct Ctx<'a> {
    doc: &'a Document,
    cam: &'a Camera,
    cursor: DVec2,
    size: DVec2,
    hit: Option<FaceHit>,
}

impl Ctx<'_> {
    /// Screen distance from the cursor to a visible point.
    fn px(&self, p: DVec3) -> Option<f64> {
        let (s, _) = picking::to_screen(self.cam, p, self.size)?;
        picking::visible(self.cam, self.doc, p, self.hit.as_ref()).then(|| s.distance(self.cursor))
    }
}

/// The point at fraction `f` of a polyline's length.
fn along(poly: &[DVec3], f: f64) -> DVec3 {
    let total: f64 = poly.windows(2).map(|w| w[0].distance(w[1])).sum();
    let mut left = f * total;
    for w in poly.windows(2) {
        let l = w[0].distance(w[1]);
        if left <= l && l > 0.0 {
            return w[0].lerp(w[1], left / l);
        }
        left -= l;
    }
    *poly.last().unwrap()
}

/// Closest points of two segments.
fn segment_closest(a0: DVec3, a1: DVec3, b0: DVec3, b1: DVec3) -> (DVec3, DVec3) {
    let (d1, d2, r) = (a1 - a0, b1 - b0, a0 - b0);
    let (a, e, f) = (d1.length_squared(), d2.length_squared(), d2.dot(r));
    let (c, b) = (d1.dot(r), d1.dot(d2));
    let denom = a * e - b * b;
    let mut s = if denom > 1e-300 { ((b * f - c * e) / denom).clamp(0.0, 1.0) } else { 0.0 };
    let mut t = if e > 1e-300 { (b * s + f) / e } else { 0.0 };
    if t < 0.0 {
        t = 0.0;
        s = if a > 1e-300 { (-c / a).clamp(0.0, 1.0) } else { 0.0 };
    } else if t > 1.0 {
        t = 1.0;
        s = if a > 1e-300 { ((b - c) / a).clamp(0.0, 1.0) } else { 0.0 };
    }
    (a0 + d1 * s, b0 + d2 * t)
}

/// Closest point of the line through `a` along `u` to the view ray.
fn line_ray_closest(a: DVec3, u: DVec3, ray: &Ray) -> Option<DVec3> {
    let w = a - ray.origin;
    let (b, d, e) = (u.dot(ray.dir), u.dot(w), ray.dir.dot(w));
    let denom = 1.0 - b * b;
    if denom < 1e-9 {
        return None;
    }
    let s = (b * e - d) / denom;
    Some(a + u * s)
}

/// The best snap under the cursor, `anchor` being the last point placed
/// (for inference lines, perpendicular and tangent snaps).
pub fn find(doc: &Document, cam: &Camera, cplane: &Frame, cursor: DVec2, size: DVec2, ray: &Ray, anchor: Option<DVec3>) -> Option<Snap> {
    let hit = picking::pick_face(doc, ray);
    let cx = Ctx { doc, cam, cursor, size, hit };
    let mut best: Option<(f64, Snap)> = None;
    let mut offer = |p: DVec3, label: &'static str, best: &mut Option<(f64, Snap)>| {
        if let Some(d) = cx.px(p) {
            if d < POINT_PX && best.is_none_or(|b| d < b.0) {
                *best = Some((d, Snap { point: p, label }));
            }
        }
    };
    // Edges near the cursor, for the snaps along them.
    // Each with its polyline and the segments of it near the cursor.
    let mut near: Vec<(&ductile_brep::Body, EdgeId, &Vec<DVec3>, Vec<(DVec3, DVec3)>)> = Vec::new();
    for o in doc.objects.iter().filter(|o| o.visible) {
        for v in &o.body.vertices {
            offer(v.point, "End", &mut best);
        }
        for (ei, poly) in o.mesh.edges.iter().enumerate() {
            let e = EdgeId(ei as u32);
            let ed = o.body.edge(e);
            if let Curve3::Circle { frame, .. } | Curve3::Ellipse { frame, .. } = &ed.curve {
                offer(frame.origin, "Center", &mut best);
            }
            if poly.len() < 2 {
                continue;
            }
            let close: Vec<(DVec3, DVec3)> = poly
                .windows(2)
                .filter(|w| {
                    let (Some((a, _)), Some((b, _))) = (picking::to_screen(cam, w[0], size), picking::to_screen(cam, w[1], size)) else { return false };
                    ductile_geom::poly2::dist_to_segment(cursor, a, b) < POINT_PX * 2.0
                })
                .map(|w| (w[0], w[1]))
                .collect();
            if !close.is_empty() {
                near.push((&o.body, e, poly, close));
            }
        }
    }
    for (body, e, poly, _) in &near {
        let (body, e) = (*body, *e);
        let ed = body.edge(e);
        if !ed.is_closed() {
            offer(along(poly, 0.5), "Middle", &mut best);
        }
        offer(along(poly, 0.25), "1/4", &mut best);
        offer(along(poly, 0.75), "3/4", &mut best);
        if let Some(a) = anchor {
            let t = ed.curve.closest_param_in(a, ed.t0, ed.t1);
            let foot = ed.curve.point(t);
            if foot.distance(a) > 1e-9 {
                offer(foot, "Perpendicular", &mut best);
            }
            if let Curve3::Circle { frame, radius } = &ed.curve {
                let l = frame.to_local(a);
                let d = DVec2::new(l.x, l.y).length();
                if d > *radius * (1.0 + 1e-9) {
                    let base = l.y.atan2(l.x);
                    let off = (*radius / d).acos();
                    for ang in [base + off, base - off] {
                        let p = frame.to_world(DVec3::new(radius * ang.cos(), radius * ang.sin(), 0.0));
                        let t = ed.curve.closest_param_in(p, ed.t0, ed.t1);
                        if ed.curve.point(t).distance(p) < 1e-9 * (1.0 + radius) {
                            offer(p, "Tangent", &mut best);
                        }
                    }
                }
            }
        }
    }
    // Crossings of nearby edges.
    let tol = cam.world_per_pixel(hit.map(|h| h.point).unwrap_or(cam.target), size.y) * 2.0;
    for i in 0..near.len() {
        for j in i + 1..near.len() {
            let (ba, ea, _, pa) = &near[i];
            let (bb, eb, _, pb) = &near[j];
            let (ba, ea, bb, eb) = (*ba, *ea, *bb, *eb);
            let (ca, cb) = (ba.edge(ea), bb.edge(eb));
            let shared = [ca.v0, ca.v1].iter().any(|v| [cb.v0, cb.v1].contains(v)) && std::ptr::eq(ba, bb);
            for wa in pa {
                for wb in pb {
                    let (p, q) = segment_closest(wa.0, wa.1, wb.0, wb.1);
                    if p.distance(q) > tol {
                        continue;
                    }
                    // Polish on the curves themselves.
                    let mut x = (p + q) * 0.5;
                    for _ in 0..8 {
                        let pa = ca.curve.point(ca.curve.closest_param_in(x, ca.t0, ca.t1));
                        let pb = cb.curve.point(cb.curve.closest_param_in(pa, cb.t0, cb.t1));
                        x = (pa + pb) * 0.5;
                    }
                    // Edges meeting at a shared vertex already snap there.
                    if shared && [ca.v0, ca.v1].iter().any(|v| ba.vertex(*v).distance(x) < tol) {
                        continue;
                    }
                    offer(x, "Intersection", &mut best);
                }
            }
        }
    }
    if let Some((_, s)) = best {
        return Some(s);
    }
    // Lines through the last point along the construction plane's axes.
    if let Some(a) = anchor {
        let mut line: Option<(f64, Snap)> = None;
        for (u, label) in [(cplane.x, "X"), (cplane.y, "Y"), (cplane.z, "Z")] {
            let Some(p) = line_ray_closest(a, u, ray) else { continue };
            if p.distance(a) < 1e-9 {
                continue;
            }
            if let Some((s, _)) = picking::to_screen(cam, p, size) {
                let d = s.distance(cursor);
                if d < LINE_PX && line.is_none_or(|b| d < b.0) {
                    line = Some((d, Snap { point: p, label }));
                }
            }
        }
        if let Some((_, s)) = line {
            return Some(s);
        }
    }
    // On a curve, then on a face.
    if let Some((o, e)) = picking::pick_edge(doc, cam, cursor, size, LINE_PX, hit.as_ref()) {
        if let Some(ob) = doc.object(o) {
            let ed = ob.body.edge(e);
            let mid = ed.mid();
            let t = ed.curve.closest_param_in(ray.at((mid - ray.origin).dot(ray.dir)), ed.t0, ed.t1);
            // Refine towards the ray: the point of the curve nearest it.
            let mut t = t;
            for _ in 0..4 {
                let p = ed.curve.point(t);
                t = ed.curve.closest_param_in(ray.at((p - ray.origin).dot(ray.dir)), ed.t0, ed.t1);
            }
            let label = if ob.body.kind == ductile_brep::BodyKind::Wire { "Curve" } else { "Edge" };
            return Some(Snap { point: ed.curve.point(t), label });
        }
    }
    hit.map(|h| Snap { point: h.point, label: "Face" })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crossing_segments_meet() {
        let (p, q) = segment_closest(DVec3::new(-1.0, 0.0, 0.0), DVec3::new(1.0, 0.0, 0.0), DVec3::new(0.5, -1.0, 0.0), DVec3::new(0.5, 1.0, 0.0));
        assert!(p.distance(DVec3::new(0.5, 0.0, 0.0)) < 1e-12 && q.distance(p) < 1e-12);
        let mid = along(&[DVec3::ZERO, DVec3::X, DVec3::new(1.0, 3.0, 0.0)], 0.5);
        assert!(mid.distance(DVec3::new(1.0, 1.0, 0.0)) < 1e-12);
    }

    #[test]
    fn inference_line_meets_the_view_ray() {
        let ray = Ray { origin: DVec3::new(2.0, 0.0, 5.0), dir: -DVec3::Z };
        let p = line_ray_closest(DVec3::ZERO, DVec3::X, &ray).unwrap();
        assert!(p.distance(DVec3::new(2.0, 0.0, 0.0)) < 1e-12);
    }
}
