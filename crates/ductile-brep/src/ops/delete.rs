// SPDX-License-Identifier: GPL-3.0-only
//! Deleting faces from bodies, healing the gap where the faces around it
//! allow, and pulling faces or edges out into bodies of their own.

use super::heal::{compact, simplify};
use crate::profile::{wire_from_segments, Segment};
use crate::topo::{Body, BodyKind, EdgeId, FaceId};
use std::collections::{HashMap, HashSet};

/// Removes `faces` from `b`. On a solid, a hole loop of a remaining face
/// whose edges all border deleted faces is dropped as well, so deleting a
/// hole or a boss closes the face it sat in and the solid stays solid.
/// When the gap cannot be closed that way the result is an open sheet.
pub fn delete_faces(b: &Body, faces: &[FaceId]) -> Body {
    let gone: HashSet<FaceId> = faces.iter().copied().collect();
    if gone.is_empty() {
        return b.clone();
    }
    let mut out = b.clone();
    let users = edge_users(b);
    let heal = b.kind == BodyKind::Solid;
    let mut kept = Vec::new();
    for f in b.face_ids() {
        if gone.contains(&f) {
            continue;
        }
        let mut face = b.face(f).clone();
        if heal {
            let borders_gap = |l: &crate::topo::LoopId| {
                b.loops[l.idx()].coedges.iter().all(|c| users[&c.edge].iter().any(|g| *g != f && gone.contains(g)))
            };
            let outer = face.loops.first().copied();
            face.loops.retain(|l| Some(*l) == outer || !borders_gap(l));
        }
        kept.push(face);
    }
    out.faces = kept;
    if out.faces.is_empty() {
        return Body::new(BodyKind::Sheet);
    }
    let closed = edge_users(&out).values().all(|u| u.len() == 2);
    out.kind = if heal && closed { BodyKind::Solid } else { BodyKind::Sheet };
    if out.kind == BodyKind::Solid {
        simplify(&out)
    } else {
        compact(&out)
    }
}

/// The faces `faces` of `b` copied out as an open sheet.
pub fn extract_faces(b: &Body, faces: &[FaceId]) -> Body {
    let mut one = b.clone();
    one.kind = BodyKind::Sheet;
    one.faces = faces.iter().map(|f| b.face(*f).clone()).collect();
    compact(&one)
}

/// The edges `edges` of `b` copied out as a wire body of curves.
pub fn extract_edges(b: &Body, edges: &[EdgeId]) -> Body {
    let segs: Vec<Segment> = edges
        .iter()
        .map(|e| {
            let ed = b.edge(*e);
            Segment { curve: ed.curve.clone(), t0: ed.t0, t1: ed.t1, reversed: false }
        })
        .collect();
    wire_from_segments(&segs)
}

fn edge_users(b: &Body) -> HashMap<EdgeId, Vec<FaceId>> {
    let mut users: HashMap<EdgeId, Vec<FaceId>> = HashMap::new();
    for f in b.face_ids() {
        for l in b.face_loops(f) {
            for c in &l.coedges {
                users.entry(c.edge).or_default().push(f);
            }
        }
    }
    users
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ops::boolean::{boolean, BoolOp};
    use crate::primitives::{make_box, make_cylinder};
    use crate::tess::{tessellate, TessOptions};
    use ductile_geom::{DVec3, Frame, Surface};

    fn vol(b: &Body) -> f64 {
        tessellate(b, &TessOptions { chord_tol: 1e-4, angle_tol: 0.05 }).volume()
    }

    #[test]
    fn deleting_a_hole_heals_the_solid() {
        let bx = make_box(&Frame::WORLD, DVec3::new(4.0, 4.0, 2.0));
        let cyl = make_cylinder(&Frame::new(DVec3::new(2.0, 2.0, -1.0), DVec3::Z, DVec3::X), 1.0, 4.0);
        let holed = boolean(&bx, &cyl, BoolOp::Difference).unwrap();
        let round: Vec<FaceId> = holed.face_ids().filter(|f| matches!(holed.face(*f).surface, Surface::Cylinder { .. })).collect();
        assert!(!round.is_empty());
        let healed = delete_faces(&holed, &round);
        assert_eq!(healed.kind, BodyKind::Solid);
        assert_eq!(healed.faces.len(), 6);
        assert!((vol(&healed) - 32.0).abs() < 1e-6, "{}", vol(&healed));
    }

    #[test]
    fn deleting_a_box_face_leaves_a_sheet() {
        let bx = make_box(&Frame::WORLD, DVec3::ONE);
        let open = delete_faces(&bx, &[FaceId(0)]);
        assert_eq!(open.kind, BodyKind::Sheet);
        assert_eq!(open.faces.len(), 5);
        let one = extract_faces(&bx, &[FaceId(1)]);
        assert_eq!((one.kind, one.faces.len(), one.edges.len()), (BodyKind::Sheet, 1, 4));
        let wire = extract_edges(&bx, &[EdgeId(0), EdgeId(1)]);
        assert_eq!((wire.kind, wire.edges.len()), (BodyKind::Wire, 2));
    }
}
