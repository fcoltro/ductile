// SPDX-License-Identifier: GPL-3.0-only
//! Measure: the distance between two picked things (vertices, edges,
//! faces, objects), or the size of one (an edge's length or radius, a
//! face's area, a solid's volume).

use crate::document::{Document, Pick};
use ductile_brep::{Body, EdgeId, FaceId};
use ductile_geom::{Curve3, DVec3, Surface};

/// A measurement: the text, and the two points a distance runs between.
#[derive(Clone, Debug, PartialEq)]
pub struct Measurement {
    pub text: String,
    pub span: Option<(DVec3, DVec3)>,
}

fn fmt(v: f64) -> String {
    let s = format!("{v:.4}");
    s.trim_end_matches('0').trim_end_matches('.').to_string()
}

/// Geometry of one pick to measure from.
enum Geo<'a> {
    Point(DVec3),
    Edge(&'a Body, EdgeId, Vec<DVec3>),
    Face(&'a Body, FaceId, &'a ductile_brep::tess::FaceMesh),
    /// A whole object: its faces (or, for curves, its edges).
    Many(Vec<Geo<'a>>),
}

fn geo<'a>(doc: &'a Document, p: &Pick) -> Option<Geo<'a>> {
    Some(match *p {
        Pick::Vertex(o, v) => Geo::Point(doc.object(o)?.body.vertex(v)),
        Pick::Edge(o, e) => {
            let ob = doc.object(o)?;
            Geo::Edge(&ob.body, e, ob.mesh.edges.get(e.idx())?.clone())
        }
        Pick::Face(o, f) => {
            let ob = doc.object(o)?;
            Geo::Face(&ob.body, f, ob.mesh.faces.get(f.idx())?)
        }
        Pick::Object(o) => {
            let ob = doc.object(o)?;
            let b: &Body = &ob.body;
            let mut parts: Vec<Geo> = ob.mesh.faces.iter().enumerate().filter(|(_, m)| !m.positions.is_empty()).map(|(i, m)| Geo::Face(b, FaceId(i as u32), m)).collect();
            if parts.is_empty() {
                parts = ob.mesh.edges.iter().enumerate().filter(|(_, p)| !p.is_empty()).map(|(i, p)| Geo::Edge(b, EdgeId(i as u32), p.clone())).collect();
            }
            Geo::Many(parts)
        }
        _ => return None,
    })
}

/// The point of `g` nearest `p`.
fn nearest(g: &Geo, p: DVec3) -> DVec3 {
    match g {
        Geo::Point(q) => *q,
        Geo::Edge(b, e, _) => {
            let ed = b.edge(*e);
            ed.curve.point(ed.curve.closest_param_in(p, ed.t0, ed.t1))
        }
        Geo::Face(b, f, m) => {
            // The nearest point of the face's mesh, then pulled onto the
            // surface from its parameters, which keeps it inside the trim.
            let mut best = (f64::MAX, m.positions.first().copied().unwrap_or(p), ductile_geom::DVec2::ZERO);
            for t in &m.triangles {
                let [i, j, k] = t.map(|x| x as usize);
                let (q, w) = closest_on_triangle(p, m.positions[i], m.positions[j], m.positions[k]);
                let d = q.distance_squared(p);
                if d < best.0 {
                    best = (d, q, m.uvs[i] * w.x + m.uvs[j] * w.y + m.uvs[k] * w.z);
                }
            }
            let s = &b.face(*f).surface;
            let q = s.point(best.2.x, best.2.y);
            let uv = s.project(p, Some(best.2));
            let r = s.point(uv.x, uv.y);
            // Accept the exact foot only while it stays by the mesh.
            let near = best.1.distance(q).max(1e-9) * 4.0;
            if r.distance(best.1) <= near.max(best.1.distance(p) * 0.05) { r } else { best.1 }
        }
        Geo::Many(parts) => parts.iter().map(|g| nearest(g, p)).min_by(|a, c| a.distance(p).total_cmp(&c.distance(p))).unwrap_or(p),
    }
}

fn samples(g: &Geo) -> Vec<DVec3> {
    match g {
        Geo::Point(p) => vec![*p],
        Geo::Edge(_, _, pts) => pts.clone(),
        Geo::Face(_, _, m) => m.positions.clone(),
        Geo::Many(parts) => parts.iter().flat_map(samples).collect(),
    }
}

/// The point of triangle `a b c` nearest `p`, with its barycentric weights.
fn closest_on_triangle(p: DVec3, a: DVec3, b: DVec3, c: DVec3) -> (DVec3, DVec3) {
    let (ab, ac, ap) = (b - a, c - a, p - a);
    let (d1, d2) = (ab.dot(ap), ac.dot(ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return (a, DVec3::X);
    }
    let bp = p - b;
    let (d3, d4) = (ab.dot(bp), ac.dot(bp));
    if d3 >= 0.0 && d4 <= d3 {
        return (b, DVec3::Y);
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return (a + ab * v, DVec3::new(1.0 - v, v, 0.0));
    }
    let cp = p - c;
    let (d5, d6) = (ab.dot(cp), ac.dot(cp));
    if d6 >= 0.0 && d5 <= d6 {
        return (c, DVec3::Z);
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return (a + ac * w, DVec3::new(1.0 - w, 0.0, w));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && d4 - d3 >= 0.0 && d5 - d6 >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (b + (c - b) * w, DVec3::new(0.0, 1.0 - w, w));
    }
    let denom = 1.0 / (va + vb + vc);
    let (v, w) = (vb * denom, vc * denom);
    (a + ab * v + ac * w, DVec3::new(1.0 - v - w, v, w))
}

/// The closest points of two picks: from the closest pair of samples,
/// polished by alternately projecting onto each.
fn closest(a: &Geo, b: &Geo) -> Option<(DVec3, DVec3)> {
    let (sa, sb) = (samples(a), samples(b));
    let mut best: Option<(f64, DVec3, DVec3)> = None;
    for p in &sa {
        for q in &sb {
            let d = p.distance_squared(*q);
            if best.is_none_or(|x| d < x.0) {
                best = Some((d, *p, *q));
            }
        }
    }
    let (_, mut p, mut q) = best?;
    for _ in 0..20 {
        q = nearest(b, p);
        p = nearest(a, q);
    }
    Some((p, q))
}

/// What the selection measures, or None when it holds nothing measurable.
pub fn measure(doc: &Document) -> Option<Measurement> {
    let picks: Vec<&Pick> = doc.selection.iter().collect();
    match picks[..] {
        [a, b] => {
            let (ga, gb) = (geo(doc, a)?, geo(doc, b)?);
            // Parallel planes: the distance between them.
            if let (Geo::Face(ba, fa, _), Geo::Face(bb, fb, _)) = (&ga, &gb) {
                if let (Surface::Plane { frame: p }, Surface::Plane { frame: q }) = (&ba.face(*fa).surface, &bb.face(*fb).surface) {
                    if p.z.cross(q.z).length() < 1e-9 {
                        let (x, y) = closest(&ga, &gb)?;
                        let d = (q.origin - p.origin).dot(p.z).abs();
                        return Some(Measurement { text: format!("Distance {} (parallel faces)", fmt(d)), span: Some((x, x + p.z * (y - x).dot(p.z))) });
                    }
                    let ang = p.z.angle_between(q.z).to_degrees();
                    let (x, y) = closest(&ga, &gb)?;
                    return Some(Measurement { text: format!("Angle {}° · distance {}", fmt(ang.min(180.0 - ang)), fmt(x.distance(y))), span: Some((x, y)) });
                }
            }
            let (x, y) = closest(&ga, &gb)?;
            let d = y - x;
            Some(Measurement { text: format!("Distance {} (dx {} dy {} dz {})", fmt(d.length()), fmt(d.x), fmt(d.y), fmt(d.z)), span: Some((x, y)) })
        }
        [a] => {
            let text = match *a {
                Pick::Edge(o, e) => {
                    let ed = doc.object(o)?.body.edge(e);
                    match &ed.curve {
                        Curve3::Circle { radius, .. } => format!("Radius {} · diameter {} · length {}", fmt(*radius), fmt(2.0 * radius), fmt(ed.length())),
                        _ => format!("Length {}", fmt(ed.length())),
                    }
                }
                Pick::Face(o, f) => {
                    let ob = doc.object(o)?;
                    let area = ob.mesh.faces.get(f.idx()).map(|m| m.triangles.iter().map(|t| {
                        let p = |i: u32| m.positions[i as usize];
                        (p(t[1]) - p(t[0])).cross(p(t[2]) - p(t[0])).length() * 0.5
                    }).sum::<f64>())?;
                    match &ob.body.face(f).surface {
                        Surface::Cylinder { radius, .. } => format!("Area {} · radius {}", fmt(area), fmt(*radius)),
                        Surface::Sphere { radius, .. } => format!("Area {} · radius {}", fmt(area), fmt(*radius)),
                        _ => format!("Area {}", fmt(area)),
                    }
                }
                Pick::Object(o) => {
                    let ob = doc.object(o)?;
                    match ob.body.kind {
                        ductile_brep::BodyKind::Solid => format!("Volume {} · area {}", fmt(ob.mesh.volume()), fmt(ob.mesh.area())),
                        ductile_brep::BodyKind::Sheet => format!("Area {}", fmt(ob.mesh.area())),
                        ductile_brep::BodyKind::Wire => format!("Length {}", fmt(ob.body.edges.iter().map(|e| e.length()).sum())),
                    }
                }
                Pick::Vertex(o, v) => {
                    let p = doc.object(o)?.body.vertex(v);
                    format!("Point {}, {}, {}", fmt(p.x), fmt(p.y), fmt(p.z))
                }
                _ => return None,
            };
            Some(Measurement { text, span: None })
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ductile_brep::primitives::{make_box, make_cylinder};
    use ductile_geom::Frame;

    #[test]
    fn distances_between_picks() {
        let mut doc = Document::default();
        let a = doc.add_body("A", make_box(&Frame::WORLD, DVec3::ONE));
        let c = doc.add_body("C", make_cylinder(&Frame { origin: DVec3::new(4.0, 0.5, 0.0), ..Frame::WORLD }, 1.0, 1.0));
        // The box's side at x = 1 to the cylinder's, centred at x = 4: 2 apart.
        let ba = &doc.object(a).unwrap().body;
        let top = ba.face_ids().find(|f| ba.face_normal(*f, 0.5, 0.5).z > 0.9).unwrap();
        let bottom = ba.face_ids().find(|f| ba.face_normal(*f, 0.5, 0.5).z < -0.9).unwrap();
        doc.selection.insert(Pick::Face(a, top));
        doc.selection.insert(Pick::Face(a, bottom));
        let m = measure(&doc).unwrap();
        assert!(m.text.starts_with("Distance 1 "), "{}", m.text);
        doc.selection.clear();
        doc.selection.insert(Pick::Object(a));
        doc.selection.insert(Pick::Object(c));
        let m = measure(&doc).unwrap();
        let (p, q) = m.span.unwrap();
        assert!((p.distance(q) - 2.0).abs() < 1e-6, "{}", m.text);
        doc.selection.clear();
        let bc = &doc.object(c).unwrap().body;
        let rim = bc.edge_ids().find(|e| matches!(bc.edge(*e).curve, Curve3::Circle { .. })).unwrap();
        doc.selection.insert(Pick::Edge(c, rim));
        assert!(measure(&doc).unwrap().text.starts_with("Radius 1 · diameter 2"));
    }
}
