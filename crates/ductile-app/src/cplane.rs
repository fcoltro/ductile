// SPDX-License-Identifier: GPL-3.0-only
//! Construction planes from the selection, following Plasticity's rules:
//! a face gives its plane, a face and an edge the plane standing on the
//! face along the edge, two opposing parallel faces the plane midway
//! between them, two points the plane through them seen edge-on, three or
//! more points (or a flat curve) the plane through them.

use crate::document::{Document, Pick};
use crate::tools::cplane_on_face;
use ductile_geom::{Curve3, DVec3, Frame};

/// The plane the selection defines, with what it was made from. `view`
/// points from the scene towards the eye.
pub fn from_selection(doc: &Document, view: DVec3) -> Option<(Frame, &'static str)> {
    let faces: Vec<Frame> = doc.selected_faces().into_iter().filter_map(|(o, f)| cplane_on_face(&doc.object(o)?.body, f)).collect();
    let edges = doc.selected_edges();
    if let (Some(face), Some(&(o, e))) = (faces.first(), edges.first()) {
        let ed = doc.object(o)?.body.edge(e);
        let n = face.z;
        let (tm, h) = ((ed.t0 + ed.t1) * 0.5, (ed.t1 - ed.t0) * 1e-3);
        let t = ed.curve.point(tm + h) - ed.curve.point(tm - h);
        let x = (t - n * t.dot(n)).try_normalize()?;
        return Some((Frame::new(ed.curve.point(tm), x.cross(n), x), "standing on the face along the edge"));
    }
    if let [a, b] = faces[..] {
        if a.z.dot(b.z) < -1.0 + 1e-9 {
            let origin = a.origin + a.z * (b.origin - a.origin).dot(a.z) * 0.5;
            return Some((Frame::new(origin, a.z, a.x), "midway between the faces"));
        }
    }
    if let Some(f) = faces.first() {
        return Some((*f, "on the face"));
    }
    let verts: Vec<DVec3> = doc
        .selection
        .iter()
        .filter_map(|p| match p {
            Pick::Vertex(o, v) => Some(doc.object(*o)?.body.vertex(*v)),
            _ => None,
        })
        .collect();
    if let [p, q] = verts[..] {
        let x = (q - p).try_normalize()?;
        let z = x.cross(view).try_normalize()?;
        return Some((Frame::new(p, z, x), "through the two points"));
    }
    if verts.len() >= 3 {
        return through(&verts, view).map(|f| (f, "through the points"));
    }
    // A circle or ellipse gives its own plane; other curves the plane they lie in.
    let mut curves: Vec<&ductile_brep::topo::Edge> = edges.iter().filter_map(|&(o, e)| Some(doc.object(o)?.body.edge(e))).collect();
    for id in doc.selected_objects() {
        if let Some(ob) = doc.object(id).filter(|ob| ob.body.kind == ductile_brep::BodyKind::Wire) {
            curves.extend(ob.body.edges.iter());
        }
    }
    if let [ed] = curves[..] {
        if let Curve3::Circle { frame, .. } | Curve3::Ellipse { frame, .. } = &ed.curve {
            let z = if frame.z.dot(view) < 0.0 { -frame.z } else { frame.z };
            return Some((Frame::new(frame.origin, z, frame.x), "of the curve"));
        }
    }
    let pts: Vec<DVec3> = curves.iter().flat_map(|ed| (0..=16).map(move |i| ed.curve.point(ed.t0 + (ed.t1 - ed.t0) * i as f64 / 16.0))).collect();
    let f = through(&pts, view)?;
    let size = pts.iter().map(|p| p.distance(f.origin)).fold(0.0, f64::max);
    pts.iter().all(|p| (*p - f.origin).dot(f.z).abs() < 1e-6 * (1.0 + size)).then_some((f, "of the curves"))
}

/// The plane best through `pts` (exact for three), facing the viewer.
fn through(pts: &[DVec3], view: DVec3) -> Option<Frame> {
    let c = pts.iter().copied().sum::<DVec3>() / pts.len() as f64;
    // The widest triangle from the centroid sets the orientation...
    let a = *pts.iter().max_by(|p, q| p.distance(c).total_cmp(&q.distance(c)))?;
    let n0 = pts.iter().map(|p| (a - c).cross(*p - c)).max_by(|u, v| u.length().total_cmp(&v.length()))?;
    // ...and every triangle, turned to agree with it, averages it.
    let mut n = DVec3::ZERO;
    for p in pts {
        for q in pts {
            let m = (*p - c).cross(*q - c);
            n += if m.dot(n0) < 0.0 { -m } else { m };
        }
    }
    let n = n.try_normalize()?;
    if n0.length() < 1e-12 * (1.0 + a.distance(c)).powi(2) {
        return None;
    }
    let z = if n.dot(view) < 0.0 { -n } else { n };
    let x = (a - c - z * (a - c).dot(z)).try_normalize()?;
    Some(Frame::new(c, z, x))
}

/// The plane square to the view through `origin`.
pub fn facing(view: DVec3, origin: DVec3) -> Frame {
    let z = view.normalize();
    let up = if z.z.abs() > 0.99 { DVec3::Y } else { DVec3::Z };
    Frame::new(origin, z, up.cross(z).normalize())
}

#[cfg(test)]
mod tests {
    use super::*;
    use ductile_brep::primitives::make_box;

    fn boxed() -> (Document, crate::document::ObjectId) {
        let mut doc = Document::default();
        let id = doc.add_body("Box", make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0)));
        (doc, id)
    }

    fn face_with_normal(doc: &Document, id: crate::document::ObjectId, n: DVec3) -> ductile_brep::FaceId {
        let b = &doc.object(id).unwrap().body;
        (0..b.faces.len() as u32).map(ductile_brep::FaceId).find(|f| cplane_on_face(b, *f).is_some_and(|fr| fr.z.distance(n) < 1e-9)).unwrap()
    }

    #[test]
    fn midplane_and_face_with_edge() {
        let (mut doc, id) = boxed();
        let (top, bottom) = (face_with_normal(&doc, id, DVec3::Z), face_with_normal(&doc, id, -DVec3::Z));
        doc.selection.insert(Pick::Face(id, top));
        doc.selection.insert(Pick::Face(id, bottom));
        let (f, _) = from_selection(&doc, DVec3::Z).unwrap();
        assert!((f.origin.z - 1.0).abs() < 1e-9 && f.z.cross(DVec3::Z).length() < 1e-9);
        // The top face and one of its edges along X: a plane standing up
        // along the edge, its normal along ±Y.
        doc.selection.clear();
        doc.selection.insert(Pick::Face(id, top));
        let b = &doc.object(id).unwrap().body;
        let e = (0..b.edges.len() as u32)
            .map(ductile_brep::EdgeId)
            .find(|e| {
                let ed = b.edge(*e);
                (ed.start().z - 2.0).abs() < 1e-9 && (ed.end().z - 2.0).abs() < 1e-9 && (ed.end() - ed.start()).cross(DVec3::X).length() < 1e-9
            })
            .unwrap();
        doc.selection.insert(Pick::Edge(id, e));
        let (f, _) = from_selection(&doc, DVec3::Z).unwrap();
        assert!(f.z.cross(DVec3::Y).length() < 1e-9 && f.y.distance(DVec3::Z) < 1e-9 && (f.origin.z - 2.0).abs() < 1e-9, "{f:?}");
    }

    #[test]
    fn plane_through_points() {
        let (mut doc, id) = boxed();
        let b = &doc.object(id).unwrap().body;
        let top: Vec<_> = (0..b.vertices.len() as u32).map(ductile_brep::VertexId).filter(|v| (b.vertex(*v).z - 2.0).abs() < 1e-9).collect();
        for v in &top[..3] {
            doc.selection.insert(Pick::Vertex(id, *v));
        }
        let (f, _) = from_selection(&doc, DVec3::new(0.2, 0.1, 1.0)).unwrap();
        assert!(f.z.distance(DVec3::Z) < 1e-9 && (f.origin.z - 2.0).abs() < 1e-9);
        doc.selection.insert(Pick::Vertex(id, top[3]));
        let (f, _) = from_selection(&doc, -DVec3::Z).unwrap();
        assert!(f.z.distance(-DVec3::Z) < 1e-9, "faces the viewer");
    }
}
