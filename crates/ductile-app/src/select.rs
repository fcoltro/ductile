// SPDX-License-Identifier: GPL-3.0-only
//! Selection helpers behind Plasticity's modifier clicks: Alt+click an
//! edge for its loop, Ctrl+Alt+click for its ring, Alt+click a face for
//! the faces tangent to it, Ctrl+Alt+click for the faces parallel to it.

use ductile_brep::{Body, EdgeId, FaceId};
use ductile_geom::{DVec3, Surface};

fn is_seam(b: &Body, e: EdgeId) -> bool {
    let f = b.edge_faces(e);
    f.len() == 2 && f[0].0 == f[1].0
}

/// Outward normal of face `f` near point `p`.
fn normal_at(b: &Body, f: FaceId, p: DVec3) -> DVec3 {
    let uv = b.face(f).surface.project(p, None);
    b.face_normal(f, uv.x, uv.y)
}

/// The edges of the loop through `e`: the boundary loop, of the faces on
/// either side, of the one turned most towards the viewer (`view` points
/// from the scene to the eye). Seams are left out.
pub fn edge_loop(b: &Body, e: EdgeId, view: DVec3) -> Vec<EdgeId> {
    let mid = b.edge(e).mid();
    let Some(f) = b.edge_faces(e).into_iter().map(|(f, _)| f).max_by(|x, y| normal_at(b, *x, mid).dot(view).total_cmp(&normal_at(b, *y, mid).dot(view))) else {
        return vec![e];
    };
    b.face_loops(f)
        .find(|l| l.coedges.iter().any(|c| c.edge == e))
        .map(|l| l.coedges.iter().map(|c| c.edge).filter(|x| !is_seam(b, *x)).collect())
        .unwrap_or_else(|| vec![e])
}

/// The edges across four-sided faces from `e`, and across from those,
/// both ways: a ring around a band of faces.
pub fn edge_ring(b: &Body, e: EdgeId) -> Vec<EdgeId> {
    let mut out = vec![e];
    let mut todo = vec![e];
    while let Some(x) = todo.pop() {
        for (f, _) in b.edge_faces(x) {
            let edges: Vec<EdgeId> = b.face_loops(f).take(1).flat_map(|l| l.coedges.iter().map(|c| c.edge)).filter(|y| !is_seam(b, *y)).collect();
            if edges.len() != 4 {
                continue;
            }
            let Some(i) = edges.iter().position(|y| *y == x) else { continue };
            let opposite = edges[(i + 2) % 4];
            if !out.contains(&opposite) {
                out.push(opposite);
                todo.push(opposite);
            }
        }
    }
    out
}

/// The faces reached from `f` across edges where neighbours meet
/// tangentially (a fillet's run, a smooth shell).
pub fn tangent_faces(b: &Body, f: FaceId) -> Vec<FaceId> {
    let mut out = vec![f];
    let mut todo = vec![f];
    while let Some(g) = todo.pop() {
        for e in b.face_edges(g) {
            let faces = b.edge_faces(e);
            let [(a, _), (c, _)] = faces[..] else { continue };
            let other = if a == g { c } else { a };
            if other == g || out.contains(&other) {
                continue;
            }
            let ed = b.edge(e);
            let smooth = [0.25, 0.5, 0.75].iter().all(|t| {
                let p = ed.curve.point(ed.t0 + (ed.t1 - ed.t0) * t);
                normal_at(b, g, p).dot(normal_at(b, other, p)) > (1f64.to_radians()).cos()
            });
            if smooth {
                out.push(other);
                todo.push(other);
            }
        }
    }
    out
}

/// The faces parallel to `f`: planes with the same or opposite normal,
/// or cylinders on the same axis.
pub fn parallel_faces(b: &Body, f: FaceId) -> Vec<FaceId> {
    let eps = 1e-9;
    match &b.face(f).surface {
        Surface::Plane { frame } => {
            let n = frame.z;
            b.face_ids().filter(|g| matches!(&b.face(*g).surface, Surface::Plane { frame } if frame.z.cross(n).length() < eps)).collect()
        }
        Surface::Cylinder { frame, .. } => {
            let (o, z) = (frame.origin, frame.z);
            b.face_ids()
                .filter(|g| match &b.face(*g).surface {
                    Surface::Cylinder { frame: f2, .. } => f2.z.cross(z).length() < eps && (f2.origin - o).cross(z).length() < 1e-7 * (1.0 + o.length()),
                    _ => false,
                })
                .collect()
        }
        _ => vec![f],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ductile_brep::primitives::make_box;
    use ductile_geom::Frame;

    #[test]
    fn loops_rings_and_parallel_faces_of_a_box() {
        let b = make_box(&Frame::WORLD, DVec3::new(2.0, 1.0, 1.0));
        let top_edge = b.edge_ids().find(|e| {
            let ed = b.edge(*e);
            ed.start().z > 0.5 && ed.end().z > 0.5 && (ed.end() - ed.start()).x.abs() > 1.0
        }).unwrap();
        // From above: the top face's four edges.
        let lp = edge_loop(&b, top_edge, DVec3::Z);
        assert_eq!(lp.len(), 4);
        assert!(lp.iter().all(|e| b.edge(*e).mid().z > 0.5));
        // Around the box through the faces it borders: the four edges along X.
        let ring = edge_ring(&b, top_edge);
        assert_eq!(ring.len(), 4);
        assert!(ring.iter().all(|e| { let ed = b.edge(*e); (ed.end() - ed.start()).x.abs() > 1.0 }));
        let top = b.face_ids().find(|f| b.face_normal(*f, 0.5, 0.5).z > 0.9).unwrap();
        assert_eq!(parallel_faces(&b, top).len(), 2);
        assert_eq!(tangent_faces(&b, top), vec![top]);
    }
}
