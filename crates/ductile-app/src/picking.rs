// SPDX-License-Identifier: GPL-3.0-only
//! Ray and screen-space picking against tessellated objects.

use crate::camera::{Camera, Ray};
use crate::document::{Document, Pick, SelectMode};
use ductile_brep::{EdgeId, FaceId, VertexId};
use glam::{DVec2, DVec3};

/// Möller–Trumbore ray/triangle intersection, returning the distance.
pub fn ray_triangle(r: &Ray, a: DVec3, b: DVec3, c: DVec3) -> Option<f64> {
    let e1 = b - a;
    let e2 = c - a;
    let p = r.dir.cross(e2);
    let det = e1.dot(p);
    if det.abs() < 1e-14 {
        return None;
    }
    let inv = 1.0 / det;
    let s = r.origin - a;
    let u = s.dot(p) * inv;
    if !(-1e-9..=1.0 + 1e-9).contains(&u) {
        return None;
    }
    let q = s.cross(e1);
    let v = r.dir.dot(q) * inv;
    if v < -1e-9 || u + v > 1.0 + 1e-9 {
        return None;
    }
    let t = e2.dot(q) * inv;
    (t > 0.0).then_some(t)
}

#[derive(Clone, Copy, Debug)]
pub struct FaceHit {
    pub object: u64,
    pub face: FaceId,
    pub t: f64,
    pub point: DVec3,
    pub normal: DVec3,
}

/// Closest face hit along a ray among visible objects.
pub fn pick_face(doc: &Document, ray: &Ray) -> Option<FaceHit> {
    let mut best: Option<FaceHit> = None;
    for o in doc.objects.iter().filter(|o| o.visible) {
        let bb = o.body.bbox().expanded(1e-6);
        if bb.ray_hit(ray.origin, ray.dir).is_none() {
            continue;
        }
        for (fi, f) in o.mesh.faces.iter().enumerate() {
            for t in &f.triangles {
                let (a, b, c) = (f.positions[t[0] as usize], f.positions[t[1] as usize], f.positions[t[2] as usize]);
                if let Some(d) = ray_triangle(ray, a, b, c) {
                    if best.map(|h| d < h.t).unwrap_or(true) {
                        let n = (b - a).cross(c - a).normalize_or_zero();
                        best = Some(FaceHit { object: o.id, face: FaceId(fi as u32), t: d, point: ray.at(d), normal: n });
                    }
                }
            }
        }
    }
    best
}

fn to_screen(cam: &Camera, p: DVec3, size: DVec2) -> Option<(DVec2, f64)> {
    let n = cam.project(p)?;
    if n.z < -1.0 || n.z > 1.0 {
        return None;
    }
    Some((DVec2::new((n.x * 0.5 + 0.5) * size.x, (0.5 - n.y * 0.5) * size.y), n.z))
}

fn seg_dist(p: DVec2, a: DVec2, b: DVec2) -> f64 {
    ductile_geom::poly2::dist_to_segment(p, a, b)
}

/// Whether `p` is visible (not occluded by a face hit nearer than it).
fn visible(cam: &Camera, doc: &Document, p: DVec3, occluder: Option<&FaceHit>) -> bool {
    match occluder {
        None => true,
        Some(h) => {
            let eye_dist = match cam.projection {
                crate::camera::Projection::Perspective => (p - cam.eye()).length(),
                crate::camera::Projection::Orthographic => {
                    let r = cam.ray(DVec2::ZERO);
                    (p - r.origin).dot(r.dir)
                }
            };
            let _ = doc;
            eye_dist <= h.t + cam.world_per_pixel(p, 1000.0) * 4.0 + 1e-6
        }
    }
}

/// Picks the best entity under the cursor for the given selection mode.
/// `cursor` is in viewport pixels (origin top-left).
pub fn pick(doc: &Document, cam: &Camera, cursor: DVec2, size: DVec2, mode: SelectMode) -> Option<Pick> {
    let ndc = DVec2::new(cursor.x / size.x * 2.0 - 1.0, 1.0 - cursor.y / size.y * 2.0);
    let ray = cam.ray(ndc);
    let face_hit = pick_face(doc, &ray);
    let tol_px = 7.0;
    match mode {
        SelectMode::Object => {
            if let Some(h) = face_hit {
                return Some(Pick::Object(h.object));
            }
            // Wires and sheets edges: pick by edge proximity.
            pick_edge(doc, cam, cursor, size, tol_px, None).map(|(o, _)| Pick::Object(o))
        }
        SelectMode::Face => face_hit.map(|h| Pick::Face(h.object, h.face)),
        SelectMode::Edge => pick_edge(doc, cam, cursor, size, tol_px, face_hit.as_ref()).map(|(o, e)| Pick::Edge(o, e)),
        SelectMode::Vertex => pick_vertex(doc, cam, cursor, size, 10.0, face_hit.as_ref()).map(|(o, v)| Pick::Vertex(o, v)),
    }
}

pub fn pick_edge(
    doc: &Document,
    cam: &Camera,
    cursor: DVec2,
    size: DVec2,
    tol_px: f64,
    occluder: Option<&FaceHit>,
) -> Option<(u64, EdgeId)> {
    let mut best: Option<(f64, u64, EdgeId)> = None;
    for o in doc.objects.iter().filter(|o| o.visible) {
        for (ei, poly) in o.mesh.edges.iter().enumerate() {
            for w in poly.windows(2) {
                let (Some((a, _)), Some((b, _))) = (to_screen(cam, w[0], size), to_screen(cam, w[1], size)) else { continue };
                let d = seg_dist(cursor, a, b);
                if d < tol_px && best.map(|x| d < x.0).unwrap_or(true) {
                    let mid = (w[0] + w[1]) * 0.5;
                    if visible(cam, doc, mid, occluder) {
                        best = Some((d, o.id, EdgeId(ei as u32)));
                    }
                }
            }
        }
    }
    best.map(|(_, o, e)| (o, e))
}

pub fn pick_vertex(
    doc: &Document,
    cam: &Camera,
    cursor: DVec2,
    size: DVec2,
    tol_px: f64,
    occluder: Option<&FaceHit>,
) -> Option<(u64, VertexId)> {
    let mut best: Option<(f64, u64, VertexId)> = None;
    for o in doc.objects.iter().filter(|o| o.visible) {
        for (vi, v) in o.body.vertices.iter().enumerate() {
            let Some((s, _)) = to_screen(cam, v.point, size) else { continue };
            let d = s.distance(cursor);
            if d < tol_px && best.map(|x| d < x.0).unwrap_or(true) && visible(cam, doc, v.point, occluder) {
                best = Some((d, o.id, VertexId(vi as u32)));
            }
        }
    }
    best.map(|(_, o, v)| (o, v))
}

/// Picks every entity of the mode whose screen projection lies inside the rectangle.
pub fn pick_box(doc: &Document, cam: &Camera, a: DVec2, b: DVec2, size: DVec2, mode: SelectMode) -> Vec<Pick> {
    let lo = a.min(b);
    let hi = a.max(b);
    let inside = |p: DVec3| to_screen(cam, p, size).map(|(s, _)| s.cmpge(lo).all() && s.cmple(hi).all()).unwrap_or(false);
    let mut out = Vec::new();
    for o in doc.objects.iter().filter(|o| o.visible) {
        match mode {
            SelectMode::Object => {
                let bb = o.body.bbox();
                if !bb.is_empty() && inside(bb.center()) {
                    out.push(Pick::Object(o.id));
                }
            }
            SelectMode::Face => {
                for (fi, f) in o.mesh.faces.iter().enumerate() {
                    if !f.positions.is_empty() && f.positions.iter().all(|p| inside(*p)) {
                        out.push(Pick::Face(o.id, FaceId(fi as u32)));
                    }
                }
            }
            SelectMode::Edge => {
                for (ei, poly) in o.mesh.edges.iter().enumerate() {
                    if !poly.is_empty() && poly.iter().all(|p| inside(*p)) {
                        out.push(Pick::Edge(o.id, EdgeId(ei as u32)));
                    }
                }
            }
            SelectMode::Vertex => {
                for (vi, v) in o.body.vertices.iter().enumerate() {
                    if inside(v.point) {
                        out.push(Pick::Vertex(o.id, VertexId(vi as u32)));
                    }
                }
            }
        }
    }
    out
}
