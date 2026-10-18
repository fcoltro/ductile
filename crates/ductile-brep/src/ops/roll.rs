// SPDX-License-Identifier: GPL-3.0-only
//! Rolling-ball blends of closed edges between arbitrary surfaces.
//!
//! A closed edge (the rim of a vase, a loft's cap edge, a sphere cut flat)
//! has no ends to finish, so it is blended by direct surgery instead of a
//! boolean: the ball is rolled along the edge, the two faces are cut back to
//! its contact curves, and the swept blend surface is inserted between them.
//! The blend surface interpolates exact circular sections (straight ones for
//! chamfers) periodically along the edge.

use super::fillet::{Blend, FilletError};
use crate::topo::{Body, Coedge, EdgeId, FaceId, VertexId};
use ductile_geom::nurbs::curve::interpolate_closed_hom;
use ductile_geom::{Curve3, DMat3, DVec2, DVec3, DVec4, NurbsSurface, Surface};

/// Blends one closed edge shared by two faces. The old edge is left
/// unreferenced in the result (compact the body afterwards), so the ids of
/// all other entities stay valid for further blends.
pub fn roll_closed_edge(body: &Body, e: EdgeId, blend: Blend) -> Result<Body, FilletError> {
    let ed = body.edge(e).clone();
    if !ed.is_closed() {
        return Err(FilletError::Unsupported);
    }
    let uses = body.edge_faces(e);
    if uses.len() != 2 || uses[0].0 == uses[1].0 {
        return Err(FilletError::NotManifold);
    }
    let (f1, c1) = uses[0];
    let (f2, c2) = uses[1];
    let (s1, s2) = (&body.face(f1).surface, &body.face(f2).surface);
    let size = body.bbox().diagonal().max(1e-9);

    // Sections along the edge, uniformly in its parameter.
    let turning = {
        let m = 64;
        (0..m)
            .map(|i| {
                let a = ed.curve.tangent(ed.t0 + (ed.t1 - ed.t0) * i as f64 / m as f64).normalize();
                let b = ed.curve.tangent(ed.t0 + (ed.t1 - ed.t0) * (i + 1) as f64 / m as f64).normalize();
                a.dot(b).clamp(-1.0, 1.0).acos()
            })
            .sum::<f64>()
    };
    let n = ((turning / 0.06).ceil() as usize).clamp(24, 240);
    let period = ed.t1 - ed.t0;
    // Tangent along face 1's traversal of the edge.
    let dir1 = if c1.reversed { -1.0 } else { 1.0 };
    // Sections starting at edge parameter `start`, wrapping around.
    let sample = |start: f64| -> Result<(Vec<Vec<DVec4>>, bool), FilletError> {
        let along = |k: usize| ed.t0 + (start - ed.t0 + period * k as f64 / n as f64).rem_euclid(period);
        let q0 = ed.curve.point(along(0));
        let mut hint1 = Some(s1.project(q0, None));
        let mut hint2 = Some(s2.project(q0, None));
        let mut convex = None;
        let mut rows: Vec<Vec<DVec4>> = vec![vec![DVec4::ZERO; n]; section_len(blend)];
        for k in 0..n {
            let t = along(k);
            let q = ed.curve.point(t);
            let tan = ed.curve.tangent(t).normalize();
            let uv1 = s1.project(q, hint1);
            let uv2 = s2.project(q, hint2);
            let n1 = body.face_normal(f1, uv1.x, uv1.y);
            let n2 = body.face_normal(f2, uv2.x, uv2.y);
            let cross = n1.cross(n2);
            if cross.length() < 1e-6 {
                return Err(FilletError::Smooth);
            }
            let cv = cross.dot(tan * dir1) > 0.0;
            if *convex.get_or_insert(cv) != cv {
                return Err(FilletError::Unsupported);
            }
            let ctrl = match blend {
                Blend::Fillet(r) | Blend::Smooth(r) => {
                    let sgn = if cv { -1.0 } else { 1.0 };
                    let (c, p1, p2, h1, h2) = ball(body, (f1, s1), (f2, s2), q, tan, n1, n2, sgn * r, (uv1, uv2)).ok_or(FilletError::TooLarge)?;
                    hint1 = Some(h1);
                    hint2 = Some(h2);
                    if p1.distance(q) > size || p2.distance(q) > size {
                        return Err(FilletError::TooLarge);
                    }
                    section_ctrl(blend, c, p1, p2)
                }
                Blend::Chamfer(d) => {
                    let t1 = tan * dir1;
                    let (u1, u2) = (n1.cross(t1), t1.cross(n2));
                    let h1 = s1.project(q + u1 * d, Some(uv1));
                    let h2 = s2.project(q + u2 * d, Some(uv2));
                    hint1 = Some(uv1);
                    hint2 = Some(uv2);
                    let (p1, p2) = (s1.point(h1.x, h1.y), s2.point(h2.x, h2.y));
                    vec![p1.extend(1.0), ((p1 + p2) * 0.5).extend(1.0), p2.extend(1.0)]
                }
            };
            for (row, h) in rows.iter_mut().zip(ctrl) {
                row[k] = h;
            }
        }
        Ok((rows, convex.unwrap_or(true)))
    };
    let (mut rows, convex) = sample(ed.t0)?;
    // A periodic face has a seam ending at the edge's vertex. Start the
    // sections where that face's contact curve crosses the seam, so the
    // seam can simply be shortened to meet it.
    let v = ed.v0;
    let seam_of = |f: FaceId| {
        body.edge_ids().filter(|x| *x != e).find(|x| {
            let xe = body.edge(*x);
            (xe.v0 == v || xe.v1 == v) && body.edge_faces(*x).iter().all(|u| u.0 == f)
        })
    };
    let crossing = match (seam_of(f1), seam_of(f2)) {
        (Some(_), Some(_)) => return Err(FilletError::Unsupported),
        (Some(x), None) => Some((x, 0)),
        (None, Some(x)) => Some((x, section_len(blend) - 1)),
        (None, None) => None,
    };
    if let Some((x, row)) = crossing {
        let contact = skin_rows(&rows).iso_u(if row == 0 { 0.0 } else { 1.0 });
        let seam = &body.edge(x);
        let gap = |w: f64| {
            let p = contact.point(w.rem_euclid(1.0));
            let t = seam.curve.closest_param_in(p, seam.t0, seam.t1);
            seam.curve.point(t).distance(p)
        };
        // Coarse scan, then golden-section refinement.
        let m = 4 * n;
        let mut best = (0..m).map(|i| i as f64 / m as f64).min_by(|a, b| gap(*a).total_cmp(&gap(*b))).unwrap_or(0.0);
        let (mut lo, mut hi) = (best - 1.0 / m as f64, best + 1.0 / m as f64);
        for _ in 0..60 {
            let (a, b) = (hi - 0.618 * (hi - lo), lo + 0.618 * (hi - lo));
            if gap(a) < gap(b) {
                hi = b;
            } else {
                lo = a;
            }
            best = 0.5 * (lo + hi);
        }
        // The contact curve's parameter is not the edge's: map the estimate
        // through the section spacing, then polish the start with secant
        // steps on the lead contact point's offset from the seam.
        let point = |r: &Vec<Vec<DVec4>>, j: usize| r[row][j].truncate() / r[row][j].w;
        let k = (best.rem_euclid(1.0) * n as f64).floor() as usize % n;
        let (pk, pk1) = (point(&rows, k), point(&rows, (k + 1) % n));
        let target = contact.point(best.rem_euclid(1.0));
        let frac = ((target - pk).dot(pk1 - pk) / (pk1 - pk).length_squared().max(1e-300)).clamp(-1.0, 2.0);
        let mut start = ed.t0 + period * (k as f64 + frac) / n as f64;
        let offset = |r: &Vec<Vec<DVec4>>| {
            let p = point(r, 0);
            let t = seam.curve.closest_param_in(p, seam.t0, seam.t1);
            let along = point(r, 1) - p;
            ((p - seam.curve.point(t)).dot(along.normalize_or_zero()), along.length())
        };
        rows = sample(start)?.0;
        // A seam that is a crease (a loft through a polygon) makes the
        // contact jump across it, so keep the closest start and snap its
        // lead point onto the seam.
        let mut best_rows = (f64::INFINITY, rows.clone());
        let mut prev: Option<(f64, f64)> = None;
        for _ in 0..12 {
            let (o, spacing) = offset(&rows);
            if o.abs() < best_rows.0 {
                best_rows = (o.abs(), rows.clone());
            }
            if o.abs() < 1e-10 * size {
                break;
            }
            let step = match prev {
                Some((ps, po)) if (o - po).abs() > 1e-300 => -o * (start - ps) / (o - po),
                _ => -o * (period / n as f64) / spacing.max(1e-300),
            };
            prev = Some((start, o));
            start += step;
            // The ball may not settle right on a crease; keep the best so far.
            match sample(start) {
                Ok((r, _)) => rows = r,
                Err(_) => break,
            }
        }
        rows = best_rows.1;
        let p = point(&rows, 0);
        let on_seam = seam.curve.point(seam.curve.closest_param_in(p, seam.t0, seam.t1));
        rows[row][0] = on_seam.extend(1.0);
        // Keep the arc's middle control point consistent with its end.
        let next = if row == 0 { 1 } else { row - 1 };
        let w = rows[next][0].w;
        rows[next][0] += ((on_seam - p) * 0.5 * w).extend(0.0);
    }
    let surf = skin_rows(&rows);

    let mut out = body.clone();
    // Contact curves (u = 0 on face 1, u = 1 on face 2) run along the edge;
    // the seam is the first section.
    let ((u0, u1), (v0, v1)) = surf.domain();
    let contact1 = surf.iso_u(u0);
    let contact2 = surf.iso_u(u1);
    let seam = surf.iso_v(v0);
    let a = out.add_vertex(contact1.point(v0));
    let b = out.add_vertex(contact2.point(v0));
    let e1 = out.add_edge(Curve3::Nurbs(contact1), v0, v1, a, a);
    let e2 = out.add_edge(Curve3::Nurbs(contact2), v0, v1, b, b);
    let es = out.add_edge(Curve3::Nurbs(seam), u0, u1, a, b);

    // Edges ending at the old vertex (the seams of periodic faces) now end
    // at the contact point on their own face.
    for x in body.edge_ids().filter(|x| *x != e) {
        let xe = body.edge(x);
        if xe.v0 != v && xe.v1 != v {
            continue;
        }
        let faces: Vec<FaceId> = body.edge_faces(x).iter().map(|u| u.0).collect();
        let (target, tv) = if faces.iter().all(|f| *f == f1) {
            (out.vertex(a), a)
        } else if faces.iter().all(|f| *f == f2) {
            (out.vertex(b), b)
        } else {
            // A real corner: other faces meet the edge's vertex.
            return Err(FilletError::Unsupported);
        };
        move_end(&mut out, x, v, target, tv, size)?;
    }
    replace_in_face(&mut out, f1, e, e1);
    replace_in_face(&mut out, f2, e, e2);
    let lp = vec![Coedge::new(e1, !c1.reversed), Coedge::new(es, false), Coedge::new(e2, !c2.reversed), Coedge::new(es, true)];
    // Outward normal: away from the ball centre on convex edges, towards it
    // on concave ones.
    let (um, vm) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
    let pm = surf.point(um, vm);
    let ends = (surf.point(u0, vm), surf.point(u1, vm));
    let chord_mid = (ends.0 + ends.1) * 0.5;
    let away = (pm - chord_mid).normalize_or_zero();
    let geo = Surface::Nurbs(surf.clone()).normal(um, vm);
    let outward = match blend {
        Blend::Fillet(_) | Blend::Smooth(_) => {
            if convex {
                away
            } else {
                -away
            }
        }
        Blend::Chamfer(_) => {
            // Flat sections: outward is away from the edge.
            let q = ed.curve.point(ed.t0 + (ed.t1 - ed.t0) * 0.5);
            let d = (pm - q).normalize_or_zero();
            if convex {
                -d
            } else {
                d
            }
        }
    };
    let reversed = geo.dot(outward) < 0.0;
    out.add_face(Surface::Nurbs(surf), reversed, vec![lp]);
    Ok(out)
}

/// Blend surface through the section rows: u across the sections (one row
/// per section control point), v around the edge (closed cubic).
fn skin_rows(rows: &[Vec<DVec4>]) -> NurbsSurface {
    let nu = rows.len();
    let columns: Vec<(Vec<f64>, Vec<DVec4>)> = rows.iter().map(|r| interpolate_closed_hom(r)).collect();
    let knots_v = columns[0].0.clone();
    let nv = columns[0].1.len();
    let mut ctrl = vec![DVec4::ZERO; nu * nv];
    for (i, (_, col)) in columns.iter().enumerate() {
        for (j, h) in col.iter().enumerate() {
            ctrl[i * nv + j] = *h;
        }
    }
    let mut knots_u = vec![0.0; nu];
    knots_u.extend(std::iter::repeat(1.0).take(nu));
    NurbsSurface::new(nu - 1, 3, knots_u, knots_v, nu, nv, ctrl)
}

/// Control points per blend section.
pub(crate) fn section_len(blend: Blend) -> usize {
    match blend {
        Blend::Smooth(_) => 6,
        _ => 3,
    }
}

/// Homogeneous control points of a blend section from `p1` to `p2` for a
/// ball centred at `c`: one rational quadratic arc for a fillet (exact for
/// sweeps under 180°), or for a smooth blend a quintic whose first and last
/// three points lie on the faces' tangents, so it leaves both faces with
/// zero curvature.
pub(crate) fn section_ctrl(blend: Blend, c: DVec3, p1: DVec3, p2: DVec3) -> Vec<DVec4> {
    let (x, w) = arc_mid(c, p1, p2);
    match blend {
        Blend::Smooth(_) => {
            let (a, b) = (0.3, 0.6);
            [p1, p1.lerp(x, a), p1.lerp(x, b), p2.lerp(x, b), p2.lerp(x, a), p2].iter().map(|p| p.extend(1.0)).collect()
        }
        _ => vec![p1.extend(1.0), (x * w).extend(w), p2.extend(1.0)],
    }
}

/// Middle control point and weight of the rational quadratic arc from `p1`
/// to `p2` about `c`.
pub(crate) fn arc_mid(c: DVec3, p1: DVec3, p2: DVec3) -> (DVec3, f64) {
    let (a, b) = (p1 - c, p2 - c);
    let w = (0.5 * a.angle_between(b)).cos();
    (c + (a + b).normalize() * (a.length() / w), w)
}

/// Ball centre for signed distance `sd` from both faces (negative inside
/// the material), in the plane through `q` normal to the edge, with its two
/// contact points and their surface parameters.
#[allow(clippy::too_many_arguments)]
pub(crate) fn ball(
    body: &Body,
    (f1, s1): (FaceId, &Surface),
    (f2, s2): (FaceId, &Surface),
    q: DVec3,
    tan: DVec3,
    n1: DVec3,
    n2: DVec3,
    sd: f64,
    (mut h1, mut h2): (DVec2, DVec2),
) -> Option<(DVec3, DVec3, DVec3, DVec2, DVec2)> {
    let half = 0.5 * n1.angle_between(n2);
    let mut c = q + (n1 + n2).normalize() * (sd / half.cos());
    for _ in 0..50 {
        h1 = s1.project(c, Some(h1));
        h2 = s2.project(c, Some(h2));
        let (p1, p2) = (s1.point(h1.x, h1.y), s2.point(h2.x, h2.y));
        let (m1, m2) = (body.face_normal(f1, h1.x, h1.y), body.face_normal(f2, h2.x, h2.y));
        let f = DVec3::new((c - p1).dot(m1) - sd, (c - p2).dot(m2) - sd, (c - q).dot(tan));
        if f.length() < 1e-11 * (1.0 + sd.abs()) {
            return Some((c, p1, p2, h1, h2));
        }
        let j = DMat3::from_cols(m1, m2, tan).transpose();
        if j.determinant().abs() < 1e-14 {
            return None;
        }
        c -= j.inverse() * f;
    }
    None
}

/// Moves the end of edge `x` at vertex `v` to the point `p` on its curve,
/// re-attaching it to vertex `nv`.
pub(crate) fn move_end(b: &mut Body, x: EdgeId, v: VertexId, p: DVec3, nv: VertexId, size: f64) -> Result<(), FilletError> {
    let xe = b.edges[x.idx()].clone();
    let t = xe.curve.closest_param_in(p, xe.t0 - (xe.t1 - xe.t0), xe.t1 + (xe.t1 - xe.t0));
    if xe.curve.point(t).distance(p) > 1e-5 * size {
        return Err(FilletError::Unsupported);
    }
    let edge = &mut b.edges[x.idx()];
    if xe.v0 == v {
        edge.t0 = t;
        edge.v0 = nv;
    }
    if xe.v1 == v {
        edge.t1 = t;
        edge.v1 = nv;
    }
    if edge.t1 <= edge.t0 {
        return Err(FilletError::TooLarge);
    }
    Ok(())
}

fn replace_in_face(b: &mut Body, f: FaceId, old: EdgeId, new: EdgeId) {
    for l in b.faces[f.idx()].loops.clone() {
        for c in b.loops[l.idx()].coedges.iter_mut() {
            if c.edge == old {
                c.edge = new;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ops::boolean::{boolean, open_edges, BoolOp};
    use crate::ops::heal::compact;
    use crate::primitives::{make_box, make_sphere};
    use crate::profile::{Loop3, Segment};
    use crate::tess::{tessellate, TessOptions};
    use ductile_geom::Frame;
    use std::f64::consts::PI;

    fn vol(b: &Body) -> f64 {
        tessellate(b, &TessOptions { chord_tol: 1e-4, angle_tol: 0.05 }).volume()
    }

    /// Sphere of radius `big` cut flat at height `h`, with its rim filleted
    /// by `r`, built exactly by revolving the meridian profile.
    fn reference(big: f64, h: f64, r: f64) -> f64 {
        let rho = ((big - r).powi(2) - (h - r).powi(2)).sqrt();
        let centre = DVec3::new(rho, 0.0, h - r);
        let contact = centre * (big / (big - r));
        let top = DVec3::new(0.0, 0.0, h);
        let bottom = DVec3::new(0.0, 0.0, -big);
        // Fillet arc from the plane contact (angle 0, straight up) outwards.
        let fillet_frame = Frame::new(centre, DVec3::Y, DVec3::Z);
        let a = (contact - centre).dot(fillet_frame.y).atan2((contact - centre).dot(fillet_frame.x));
        // Sphere meridian: angle from +X towards +Z.
        let sphere_frame = Frame::new(DVec3::ZERO, DVec3::NEG_Y, DVec3::X);
        let s0 = contact.z.atan2(contact.x);
        let l = Loop3 {
            segments: vec![
                Segment::line(top, DVec3::new(rho, 0.0, h)),
                Segment::new(Curve3::Circle { frame: fillet_frame, radius: r }, 0.0, a),
                Segment::new(Curve3::Circle { frame: sphere_frame, radius: big }, -PI / 2.0, s0).flipped(),
                Segment::line(bottom, top),
            ],
        };
        assert!(l.is_closed());
        vol(&crate::ops::revolve::revolve_loop(&l, DVec3::ZERO, DVec3::Z).unwrap())
    }

    #[test]
    fn fillets_the_rim_of_a_cut_sphere() {
        let (big, h, r) = (3.0, 1.0, 0.4);
        let cut = boolean(&make_sphere(DVec3::ZERO, big), &make_box(&Frame { origin: DVec3::new(-5.0, -5.0, h), ..Frame::WORLD }, DVec3::splat(10.0)), BoolOp::Difference).unwrap();
        let rim = cut.edge_ids().find(|e| cut.edge(*e).is_closed() && cut.edge_faces(*e).len() == 2).unwrap();
        let b = compact(&roll_closed_edge(&cut, rim, Blend::Fillet(r)).unwrap());
        assert_eq!(open_edges(&b), 0);
        let expect = reference(big, h, r);
        assert!((vol(&b) - expect).abs() < 2e-4 * expect, "{} vs {expect}", vol(&b));
    }

    #[test]
    fn fillets_a_concave_sphere_boss() {
        // A ball half sunk into a disk; the circle where they meet is concave.
        let (big, r) = (2.0, 0.3);
        let c0 = DVec3::new(0.0, 0.0, -1.0);
        let disk = crate::primitives::make_cylinder(&Frame { origin: DVec3::new(0.0, 0.0, -4.0), ..Frame::WORLD }, 5.0, 4.0);
        let body = boolean(&disk, &make_sphere(c0, big), BoolOp::Union).unwrap();
        let rim = body
            .edge_ids()
            .find(|e| {
                let ed = body.edge(*e);
                ed.is_closed() && body.edge_faces(*e).len() == 2 && (ed.start().truncate().length() - 3f64.sqrt()).abs() < 1e-6
            })
            .unwrap();
        let b = compact(&roll_closed_edge(&body, rim, Blend::Fillet(r)).unwrap());
        assert_eq!(open_edges(&b), 0);
        // Exact reference by revolving the meridian.
        let rho = ((big + r).powi(2) - (r + 1.0).powi(2)).sqrt();
        let centre = DVec3::new(rho, 0.0, r);
        let contact = c0 + (centre - c0) * (big / (big + r));
        let ff = Frame::new(centre, DVec3::Y, DVec3::NEG_Z);
        let a = (contact - centre).dot(ff.y).atan2((contact - centre).dot(ff.x));
        let sf = Frame::new(c0, DVec3::NEG_Y, DVec3::X);
        let sc = (contact - c0).z.atan2((contact - c0).x);
        let pts = [DVec3::new(0.0, 0.0, -4.0), DVec3::new(5.0, 0.0, -4.0), DVec3::new(5.0, 0.0, 0.0), DVec3::new(rho, 0.0, 0.0)];
        let mut segs: Vec<Segment> = pts.windows(2).map(|w| Segment::line(w[0], w[1])).collect();
        segs.push(Segment::new(Curve3::Circle { frame: ff, radius: r }, 0.0, a));
        segs.push(Segment::new(Curve3::Circle { frame: sf, radius: big }, sc, PI / 2.0));
        segs.push(Segment::line(DVec3::new(0.0, 0.0, 1.0), pts[0]));
        let l = Loop3 { segments: segs };
        assert!(l.is_closed());
        let expect = vol(&crate::ops::revolve::revolve_loop(&l, DVec3::ZERO, DVec3::Z).unwrap());
        assert!((vol(&b) - expect).abs() < 2e-4 * expect, "{} vs {expect}", vol(&b));
    }

    #[test]
    fn chamfers_the_rim_of_a_cut_sphere() {
        let cut = boolean(&make_sphere(DVec3::ZERO, 3.0), &make_box(&Frame { origin: DVec3::new(-5.0, -5.0, 1.0), ..Frame::WORLD }, DVec3::splat(10.0)), BoolOp::Difference).unwrap();
        let rim = cut.edge_ids().find(|e| cut.edge(*e).is_closed() && cut.edge_faces(*e).len() == 2).unwrap();
        let b = compact(&roll_closed_edge(&cut, rim, Blend::Chamfer(0.3)).unwrap());
        assert_eq!(open_edges(&b), 0);
        assert!(vol(&b) < vol(&cut) && vol(&b) > 0.98 * vol(&cut), "{} vs {}", vol(&b), vol(&cut));
    }

    #[test]
    fn fillets_the_foot_of_a_twisted_loft() {
        // The loft's seam leaves the circle's section plane, so the first
        // section has to be placed on the seam.
        let circle = Loop3 { segments: vec![Segment::new(Curve3::Circle { frame: Frame::WORLD, radius: 2.5 }, 0.0, 2.0 * PI)] };
        let mid = Loop3 { segments: vec![Segment::new(Curve3::Circle { frame: Frame { origin: DVec3::Z * 2.5, ..Frame::WORLD }, radius: 1.6 }, 0.0, 2.0 * PI)] };
        let q = [DVec3::new(-2.0, -2.0, 5.0), DVec3::new(2.0, -2.0, 5.0), DVec3::new(2.0, 2.0, 5.0), DVec3::new(-2.0, 2.0, 5.0)];
        let square = Loop3 { segments: (0..4).map(|i| Segment::line(q[i], q[(i + 1) % 4])).collect() };
        let loft = crate::ops::loft::loft(&[circle, mid, square]).unwrap();
        let foot = loft.edge_ids().find(|e| loft.edge(*e).is_closed() && loft.edge(*e).start().z < 0.1).unwrap();
        let r = 0.4;
        let b = compact(&roll_closed_edge(&loft, foot, Blend::Fillet(r)).unwrap());
        assert_eq!(open_edges(&b), 0);
        let removed = vol(&loft) - vol(&b);
        assert!(removed > 0.0 && removed < 2.0 * PI * 2.5 * r * r, "{removed}");
    }
}
