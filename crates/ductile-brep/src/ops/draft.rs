// SPDX-License-Identifier: GPL-3.0-only
//! Draft: tilting planar faces about the line where they cross a neutral
//! plane, so a part tapers along its pull direction. The faces around them
//! are re-trimmed to the tilted planes; the topology does not change.

use crate::topo::{Body, EdgeId, FaceId, VertexId};
use ductile_geom::{Curve3, DMat3, DMat4, DVec3, Surface};
use std::collections::HashMap;

/// Tilts each planar face in `faces` by `angle` (radians) about the line
/// where it crosses the neutral plane through `origin` square to `pull`.
/// A positive angle leans the face in towards `pull`, so the part narrows
/// away from the neutral plane. None when a face cannot be drafted (it is
/// not planar or lies square to `pull`) or the faces around it cannot
/// follow (they are curved where they meet it, or would turn inside out).
pub fn draft_faces(b: &Body, faces: &[FaceId], origin: DVec3, pull: DVec3, angle: f64) -> Option<Body> {
    let pull = pull.try_normalize()?;
    let mut planes = HashMap::new();
    for f in faces {
        let face = b.face(*f);
        let Surface::Plane { frame } = &face.surface else { return None };
        let n = if face.reversed { -frame.z } else { frame.z };
        let axis = n.cross(pull).try_normalize()?;
        // A point on both the face's plane and the neutral plane.
        let c = n.dot(pull);
        let (d1, d2) = (n.dot(frame.origin), pull.dot(origin));
        let hinge = (n * (d1 - d2 * c) + pull * (d2 - d1 * c)) / (1.0 - c * c);
        let m = DMat4::from_translation(hinge) * DMat4::from_axis_angle(axis, angle) * DMat4::from_translation(-hinge);
        planes.insert(*f, face.surface.transformed(&m));
    }
    replace_planes(b, &planes)
}

/// `b` with the planar faces in `planes` moved onto new planes. Every
/// vertex touching a moved face goes to where its planes meet, and the
/// straight edges into it follow. None when a moved face meets a curved
/// face, its corners do not meet at a point, or an edge would flip.
pub fn replace_planes(b: &Body, planes: &HashMap<FaceId, Surface>) -> Option<Body> {
    let mut out = b.clone();
    for (f, s) in planes {
        out.faces[f.idx()].surface = s.clone();
    }
    let mut around: Vec<Vec<FaceId>> = vec![Vec::new(); b.vertices.len()];
    for f in b.face_ids() {
        for l in b.face_loops(f) {
            for c in &l.coedges {
                let e = b.edge(c.edge);
                for v in [e.v0, e.v1] {
                    if !around[v.idx()].contains(&f) {
                        around[v.idx()].push(f);
                    }
                }
            }
        }
    }
    let size = b.bbox().diagonal().max(1.0);
    let mut moved: Vec<VertexId> = Vec::new();
    for (v, fs) in around.iter().enumerate() {
        if !fs.iter().any(|f| planes.contains_key(f)) {
            continue;
        }
        let mut m = DMat3::ZERO;
        let mut rhs = DVec3::ZERO;
        let mut rows = Vec::new();
        for f in fs {
            let Surface::Plane { frame } = &out.face(*f).surface else { return None };
            let (n, d) = (frame.z, frame.z.dot(frame.origin));
            m += DMat3::from_cols(n * n.x, n * n.y, n * n.z);
            rhs += n * d;
            rows.push((n, d));
        }
        if m.determinant().abs() < 1e-9 {
            return None;
        }
        let p = m.inverse() * rhs;
        if rows.iter().any(|(n, d)| (n.dot(p) - d).abs() > 1e-7 * size) {
            return None;
        }
        out.vertices[v].point = p;
        moved.push(VertexId(v as u32));
    }
    for i in 0..b.edges.len() {
        let e = b.edge(EdgeId(i as u32));
        if !(moved.contains(&e.v0) || moved.contains(&e.v1)) {
            continue;
        }
        let Curve3::Line { .. } = e.curve else { return None };
        let (p, q) = (out.vertex(e.v0), out.vertex(e.v1));
        if (q - p).dot(e.end() - e.start()) <= 0.0 || p.distance(q) < 1e-9 * size {
            return None;
        }
        let ne = &mut out.edges[i];
        ne.curve = Curve3::line_through(p, q);
        (ne.t0, ne.t1) = (0.0, p.distance(q));
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::make_box;
    use crate::tess::{tessellate, TessOptions};
    use ductile_geom::Frame;

    #[test]
    fn drafts_the_sides_of_a_box() {
        let bx = make_box(&Frame::WORLD, DVec3::new(2.0, 2.0, 1.0));
        let sides: Vec<FaceId> = bx.face_ids().filter(|f| bx.face_normal(*f, 0.5, 0.5).z.abs() < 0.5).collect();
        assert_eq!(sides.len(), 4);
        let a = 10f64.to_radians();
        let d = draft_faces(&bx, &sides, DVec3::ZERO, DVec3::Z, a).unwrap();
        // A frustum of a square pyramid: base 2, top 2 - 2 tan(a).
        let t = 2.0 - 2.0 * a.tan();
        let expect = (4.0 + t * t + 2.0 * t) / 3.0;
        let v = tessellate(&d, &TessOptions::default()).volume();
        assert!((v - expect).abs() < 1e-9, "{v} {expect}");
        let top = d.vertices.iter().map(|v| v.point).filter(|p| p.z > 0.5).map(|p| p.x).fold(f64::MIN, f64::max);
        assert!((top - (2.0 - a.tan())).abs() < 1e-9);
        // Too steep: the top would turn inside out.
        assert!(draft_faces(&bx, &sides, DVec3::ZERO, DVec3::Z, 70f64.to_radians()).is_none());
    }
}
