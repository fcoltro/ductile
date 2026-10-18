// SPDX-License-Identifier: GPL-3.0-only
//! Join sheets: sews surfaces along the edges they share into one body,
//! turning them to agree on which side is the front, and closing them into
//! a solid when no open edge is left.

use crate::ops::sheet::Sewer;
use crate::profile::Segment;
use crate::topo::{Body, BodyKind, FaceId};
use ductile_geom::Surface;

/// A face to sew: its surface, which way it faces, and its loops.
#[derive(Clone)]
struct Piece {
    surface: Surface,
    reversed: bool,
    loops: Vec<Vec<Segment>>,
}

impl Piece {
    fn flipped(&self) -> Piece {
        let loops = self.loops.iter().map(|l| l.iter().rev().map(Segment::flipped).collect()).collect();
        Piece { surface: self.surface.clone(), reversed: !self.reversed, loops }
    }
}

fn pieces(b: &Body) -> Vec<Piece> {
    b.face_ids()
        .map(|f: FaceId| {
            let face = b.face(f);
            let loops = b
                .face_loops(f)
                .map(|l| {
                    l.coedges
                        .iter()
                        .map(|c| {
                            let e = b.edge(c.edge);
                            Segment { curve: e.curve.clone(), t0: e.t0, t1: e.t1, reversed: c.reversed }
                        })
                        .collect()
                })
                .collect();
            Piece { surface: face.surface.clone(), reversed: face.reversed, loops }
        })
        .collect()
}

/// Whether a boundary segment of one body runs along one of another:
/// Some(true) running the same way, Some(false) the opposite way.
fn shared(a: &[Piece], b: &[Piece], tol: f64) -> Option<bool> {
    let ends = |s: &Segment| (s.start(), s.end(), s.curve.point(0.5 * (s.t0 + s.t1)));
    for pa in a.iter().flat_map(|p| p.loops.iter().flatten()) {
        let (a0, a1, am) = ends(pa);
        for pb in b.iter().flat_map(|p| p.loops.iter().flatten()) {
            let (b0, b1, bm) = ends(pb);
            if am.distance(bm) > tol || a0.distance(a1) < tol {
                continue;
            }
            if a0.distance(b0) < tol && a1.distance(b1) < tol {
                return Some(true);
            }
            if a0.distance(b1) < tol && a1.distance(b0) < tol {
                return Some(false);
            }
        }
    }
    None
}

/// Sews `bodies` (sheets, or solids' faces) into one, within `tol`.
/// Errors when some of them touch none of the others.
pub fn join_sheets(bodies: &[&Body], tol: f64) -> Result<Body, String> {
    let mut groups: Vec<Vec<Piece>> = bodies.iter().map(|b| pieces(b)).collect();
    // Grow from the first body, turning each one joined to agree with it:
    // across a shared edge, two faces facing the same way run it opposite ways.
    let mut placed = vec![false; groups.len()];
    placed[0] = true;
    let mut done = vec![groups[0].clone()];
    loop {
        let mut progress = false;
        for i in 0..groups.len() {
            if placed[i] {
                continue;
            }
            let hit = done.iter().find_map(|d| shared(d, &groups[i], tol));
            if let Some(same) = hit {
                if same {
                    groups[i] = groups[i].iter().map(Piece::flipped).collect();
                }
                placed[i] = true;
                done.push(groups[i].clone());
                progress = true;
            }
        }
        if !progress {
            break;
        }
    }
    if placed.iter().any(|p| !p) {
        return Err("some of the surfaces share no edge with the others".into());
    }
    let sew = |all: &[Vec<Piece>]| {
        let mut sw = Sewer::new(BodyKind::Sheet, tol);
        for p in all.iter().flatten() {
            sw.face(p.surface.clone(), p.reversed, &p.loops[0], &p.loops[1..]);
        }
        sw.body
    };
    let mut body = sew(&done);
    let closed = body.edge_ids().all(|e| body.edge_faces(e).len() == 2);
    if closed {
        body.kind = BodyKind::Solid;
        let size = body.bbox().diagonal();
        let mesh = crate::tessellate(&body, &crate::TessOptions::for_size(size));
        if mesh.volume() < 0.0 {
            let flipped: Vec<Vec<Piece>> = done.iter().map(|g| g.iter().map(Piece::flipped).collect()).collect();
            body = sew(&flipped);
            body.kind = BodyKind::Solid;
        }
    }
    Ok(body)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ops::sheet::extrude_sheet;
    use crate::profile::Loop3;
    use ductile_geom::DVec3;

    fn quad(p: [DVec3; 4]) -> Body {
        crate::ops::sheet::patch(&Loop3 { segments: (0..4).map(|i| Segment::line(p[i], p[(i + 1) % 4])).collect() }, &[]).unwrap()
    }

    #[test]
    fn joins_two_sheets_and_closes_a_box() {
        let v = |x: f64, y: f64, z: f64| DVec3::new(x, y, z);
        // Four walls as one sheet, then floor and roof, the roof facing in.
        let ring = Loop3 { segments: (0..4).map(|i| {
            let p = [v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(1.0, 1.0, 0.0), v(0.0, 1.0, 0.0)];
            Segment::line(p[i], p[(i + 1) % 4])
        }).collect() };
        let walls = extrude_sheet(&[ring], v(0.0, 0.0, 1.0)).unwrap();
        let floor = quad([v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(1.0, 1.0, 0.0), v(0.0, 1.0, 0.0)]);
        let roof = quad([v(0.0, 0.0, 1.0), v(1.0, 0.0, 1.0), v(1.0, 1.0, 1.0), v(0.0, 1.0, 1.0)]);
        let open = join_sheets(&[&walls, &floor], 1e-7).unwrap();
        assert_eq!(open.kind, BodyKind::Sheet);
        assert_eq!(open.faces.len(), 5);
        let solid = join_sheets(&[&walls, &floor, &roof], 1e-7).unwrap();
        assert_eq!(solid.kind, BodyKind::Solid);
        assert!(solid.edge_ids().all(|e| solid.edge_faces(e).len() == 2));
        let vol = crate::tessellate(&solid, &crate::TessOptions::for_size(2.0)).volume();
        assert!((vol - 1.0).abs() < 1e-6, "{vol}");
        let far = quad([v(5.0, 0.0, 0.0), v(6.0, 0.0, 0.0), v(6.0, 1.0, 0.0), v(5.0, 1.0, 0.0)]);
        assert!(join_sheets(&[&walls, &far], 1e-7).is_err());
    }
}
