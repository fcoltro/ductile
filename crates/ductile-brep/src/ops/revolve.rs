// SPDX-License-Identifier: GPL-3.0-only
//! Revolution of planar profiles about an axis lying in their plane.

use crate::profile::{Loop3, Segment};
use crate::topo::{Body, BodyKind, Coedge, EdgeId, VertexId};
use ductile_geom::{Curve3, DMat4, DQuat, DVec3, Frame, NurbsSurface, Surface};
use std::f64::consts::TAU;

/// Revolves a closed profile a full turn about the axis through `origin`
/// along `axis`. The profile must lie in a plane containing the axis, on
/// one side of it; segments may touch the axis. Lines and circular arcs
/// produce exact planes, cylinders, cones, spheres and tori.
pub fn revolve_loop(profile: &Loop3, origin: DVec3, axis: DVec3) -> Option<Body> {
    revolve(profile, origin, axis, TAU)
}

/// Revolves a closed profile by `angle` radians (right-handed about `axis`;
/// negative angles turn the other way). Partial turns are capped by the
/// profile at both ends.
pub fn revolve(profile: &Loop3, origin: DVec3, axis: DVec3, angle: f64) -> Option<Body> {
    let (axis, angle) = if angle < 0.0 { (-axis, -angle) } else { (axis, angle) };
    if angle < 1e-9 {
        return None;
    }
    let full = angle >= TAU - 1e-9;
    let angle = angle.min(TAU);
    let a = axis.try_normalize()?;
    let tol = 1e-9 * (1.0 + profile.polygon(1e-2).iter().map(|p| p.length()).fold(0.0, f64::max));
    let n = profile.segments.len();
    if n < 2 || !profile.is_closed() {
        return None;
    }
    // Radial direction of the profile half plane.
    let pts = profile.polygon(1e-3);
    let far = pts.iter().copied().max_by(|p, q| radial(*p, origin, a).length().total_cmp(&radial(*q, origin, a).length()))?;
    let e = radial(far, origin, a).try_normalize()?;
    let np = a.cross(e);
    // Make the profile counter-clockwise about np so that s x phi is outward.
    let profile = if profile.normal().dot(np) < 0.0 { profile.reversed() } else { profile.clone() };
    let on_axis = |p: DVec3| radial(p, origin, a).length() < tol.max(1e-9);
    let rot = DMat4::from_translation(origin) * DMat4::from_quat(DQuat::from_axis_angle(a, angle)) * DMat4::from_translation(-origin);

    let mut b = Body::new(BodyKind::Solid);
    let starts: Vec<DVec3> = profile.segments.iter().map(|s| s.start()).collect();
    let verts: Vec<VertexId> = starts.iter().map(|p| b.add_vertex(*p)).collect();
    let verts_end: Vec<VertexId> = starts
        .iter()
        .enumerate()
        .map(|(i, p)| if full || on_axis(*p) { verts[i] } else { b.add_vertex(rot.transform_point3(*p)) })
        .collect();
    // Circle (or arc) edge swept by every profile vertex off the axis.
    let circles: Vec<Option<EdgeId>> = starts
        .iter()
        .enumerate()
        .map(|(i, p)| {
            if on_axis(*p) {
                return None;
            }
            let c = origin + a * (*p - origin).dot(a);
            let frame = Frame::new(c, a, *p - c);
            Some(b.add_edge(Curve3::Circle { frame, radius: p.distance(c) }, 0.0, angle, verts[i], verts_end[i]))
        })
        .collect();
    // Profile edges at the start angle and (for partial turns) the end angle.
    let mut start_edges: Vec<Option<(EdgeId, bool)>> = vec![None; n];
    let mut end_edges: Vec<Option<(EdgeId, bool)>> = vec![None; n];
    for (i, s) in profile.segments.iter().enumerate() {
        let j = (i + 1) % n;
        let axial = on_axis(starts[i]) && on_axis(starts[j]) && on_axis(s.curve.point(0.5 * (s.t0 + s.t1)));
        if full && axial {
            continue;
        }
        let (v0, v1) = if s.reversed { (verts[j], verts[i]) } else { (verts[i], verts[j]) };
        let e0 = b.add_edge(s.curve.clone(), s.t0, s.t1, v0, v1);
        start_edges[i] = Some((e0, s.reversed));
        end_edges[i] = Some(if full || axial {
            (e0, s.reversed)
        } else {
            let (w0, w1) = if s.reversed { (verts_end[j], verts_end[i]) } else { (verts_end[i], verts_end[j]) };
            (b.add_edge(s.curve.transformed(&rot), s.t0, s.t1, w0, w1), s.reversed)
        });
    }
    for (i, s) in profile.segments.iter().enumerate() {
        let j = (i + 1) % n;
        let (Some((e0, r0)), Some((e1, r1))) = (start_edges[i], end_edges[i]) else { continue };
        if e0 == e1 && !full {
            continue; // on the axis: bounded by the caps only
        }
        let (surface, flat) = surface_of(s, origin, a, e, tol, angle);
        // Outward normal at the segment midpoint (profile is CCW about np).
        let tm = 0.5 * (s.t0 + s.t1);
        let mid = s.curve.point(tm);
        let out = s.tangent_at(tm).cross(np);
        let uv = surface.project(mid, None);
        let reversed = surface.normal(uv.x, uv.y).dot(out) < 0.0;
        let loops = if flat && full {
            // Annulus or disk: the circles alone.
            let mut ls = vec![];
            if let Some(c) = circles[j] {
                ls.push(vec![Coedge::new(c, false)]);
            }
            if let Some(c) = circles[i] {
                ls.push(vec![Coedge::new(c, true)]);
            }
            // Outer loop (larger radius) first.
            let rj = radial(starts[j], origin, a).length();
            let ri = radial(starts[i], origin, a).length();
            if ri > rj {
                ls.reverse();
            }
            ls
        } else {
            let mut l = vec![Coedge::new(e0, r0)];
            if let Some(c) = circles[j] {
                l.push(Coedge::new(c, false));
            }
            l.push(Coedge::new(e1, !r1));
            if let Some(c) = circles[i] {
                l.push(Coedge::new(c, true));
            }
            vec![l]
        };
        b.add_face(surface, reversed, loops);
    }
    if !full {
        // Start cap faces -np (reversed profile order), end cap faces the
        // rotated np.
        let pf = Frame::new(starts[0], np, e);
        let cap0: Vec<Coedge> = (0..n).rev().filter_map(|i| start_edges[i].map(|(e, r)| Coedge::new(e, !r))).collect();
        b.add_face(Surface::Plane { frame: pf }, true, vec![cap0]);
        let cap1: Vec<Coedge> = (0..n).filter_map(|i| end_edges[i].map(|(e, r)| Coedge::new(e, r))).collect();
        b.add_face(Surface::Plane { frame: pf.transformed(&rot) }, false, vec![cap1]);
    }
    // Full-turn disks and annuli are bounded by circles alone; drop their
    // unused profile edges.
    Some(if full { crate::ops::heal::compact(&b) } else { b })
}

fn radial(p: DVec3, o: DVec3, a: DVec3) -> DVec3 {
    let d = p - o;
    d - a * d.dot(a)
}

/// Surface swept by a profile segment, and whether it is planar. Lines
/// and arcs in the axis plane give analytic surfaces; anything else is
/// swept as an exact NURBS surface of revolution.
fn surface_of(s: &Segment, o: DVec3, a: DVec3, e: DVec3, tol: f64, angle: f64) -> (Surface, bool) {
    analytic_surface_of(s, o, a, e, tol).unwrap_or_else(|| (Surface::Nurbs(NurbsSurface::revolution(&s.curve.to_nurbs(s.t0, s.t1), o, a, angle)), false))
}

fn analytic_surface_of(s: &Segment, o: DVec3, a: DVec3, e: DVec3, tol: f64) -> Option<(Surface, bool)> {
    let (p, q) = (s.start(), s.end());
    let z = |x: DVec3| (x - o).dot(a);
    let rho = |x: DVec3| radial(x, o, a).length();
    match &s.curve {
        Curve3::Line { .. } => {
            let (zp, zq, rp, rq) = (z(p), z(q), rho(p), rho(q));
            if (zp - zq).abs() < tol {
                let frame = Frame::new(o + a * zp, a, e);
                Some((Surface::Plane { frame }, true))
            } else if (rp - rq).abs() < tol {
                let frame = Frame::new(o + a * zp.min(zq), a, e);
                Some((Surface::Cylinder { frame, radius: rp }, false))
            } else {
                // Cone: radius(z) = r0 + (z - z0) tan(half_angle) along +a.
                let (z0, r0, z1, r1) = if zp < zq { (zp, rp, zq, rq) } else { (zq, rq, zp, rp) };
                let half = ((r1 - r0) / (z1 - z0)).atan();
                let frame = Frame::new(o + a * z0, a, e);
                Some((Surface::Cone { frame, radius: r0, half_angle: half }, false))
            }
        }
        Curve3::Circle { frame: cf, radius } => {
            // The arc's plane must contain the axis.
            if cf.z.dot(a).abs() > 1e-9 {
                return None;
            }
            let c = cf.origin;
            let rc = rho(c);
            let frame = Frame::new(o + a * z(c), a, e);
            if rc < tol {
                Some((Surface::Sphere { frame, radius: *radius }, false))
            } else {
                Some((Surface::Torus { frame, major: rc, minor: *radius }, false))
            }
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tess::{tessellate, TessOptions};
    use std::f64::consts::PI;

    fn poly(pts: &[DVec3]) -> Loop3 {
        Loop3 { segments: (0..pts.len()).map(|i| Segment::line(pts[i], pts[(i + 1) % pts.len()])).collect() }
    }

    fn vol(b: &Body) -> f64 {
        tessellate(b, &TessOptions { chord_tol: 1e-4, angle_tol: 0.05 }).volume()
    }

    #[test]
    fn tube_from_rectangle() {
        let l = poly(&[DVec3::new(1.0, 0.0, 0.0), DVec3::new(2.0, 0.0, 0.0), DVec3::new(2.0, 0.0, 3.0), DVec3::new(1.0, 0.0, 3.0)]);
        let b = revolve_loop(&l, DVec3::ZERO, DVec3::Z).unwrap();
        let expect = PI * (4.0 - 1.0) * 3.0;
        assert!((vol(&b) - expect).abs() < 3e-3, "{} vs {expect}", vol(&b));
        // Orientation-independent.
        let b2 = revolve_loop(&l.reversed(), DVec3::ZERO, DVec3::Z).unwrap();
        assert!((vol(&b2) - expect).abs() < 3e-3);
    }

    #[test]
    fn cone_from_triangle() {
        let l = poly(&[DVec3::ZERO, DVec3::new(0.0, 2.0, 0.0), DVec3::new(0.0, 0.0, 3.0)]);
        let b = revolve_loop(&l, DVec3::ZERO, DVec3::Z).unwrap();
        let expect = PI * 4.0 * 3.0 / 3.0;
        assert!((vol(&b) - expect).abs() < 1e-3, "{} vs {expect}", vol(&b));
    }

    #[test]
    fn sphere_and_torus_from_arcs() {
        // Half disk -> sphere.
        let arc = Segment::new(Curve3::Circle { frame: Frame::new(DVec3::ZERO, DVec3::Y, DVec3::NEG_Z), radius: 1.0 }, 0.0, PI);
        let l = Loop3 { segments: vec![arc.clone(), Segment::line(arc.end(), arc.start())] };
        let b = revolve_loop(&l, DVec3::ZERO, DVec3::Z).unwrap();
        assert!((vol(&b) - 4.0 / 3.0 * PI).abs() < 3e-3, "{}", vol(&b));
        // Quarter-round fillet ring (like a fillet tool).
        let c = DVec3::new(2.0, 0.0, 0.0);
        let q = Segment::new(Curve3::Circle { frame: Frame::new(c, DVec3::NEG_Y, DVec3::X), radius: 0.5 }, 0.0, PI / 2.0);
        let corner = DVec3::new(2.5, 0.0, 0.5);
        let l = Loop3 { segments: vec![q.clone(), Segment::line(q.end(), corner), Segment::line(corner, q.start())] };
        let b = revolve_loop(&l, DVec3::ZERO, DVec3::Z).unwrap();
        // Pappus: spandrel area times the path of its centroid.
        let r: f64 = 0.5;
        let area = r * r * (1.0 - PI / 4.0);
        let rho = 2.0 + (r * r * r / 6.0) / area;
        let expect = area * TAU * rho;
        assert!((vol(&b) - expect).abs() < 1e-3, "{} vs {expect}", vol(&b));
    }

    #[test]
    fn partial_revolves() {
        let l = poly(&[DVec3::new(1.0, 0.0, 0.0), DVec3::new(2.0, 0.0, 0.0), DVec3::new(2.0, 0.0, 3.0), DVec3::new(1.0, 0.0, 3.0)]);
        for (ang, sign) in [(PI / 2.0, 1.0), (PI, -1.0), (1.5 * PI, 1.0)] {
            let b = revolve(&l, DVec3::ZERO, DVec3::Z, ang * sign).unwrap();
            assert_eq!(crate::ops::boolean::open_edges(&b), 0);
            let expect = ang / 2.0 * (4.0 - 1.0) * 3.0;
            assert!((vol(&b) - expect).abs() < 3e-3, "{ang}: {} vs {expect}", vol(&b));
        }
        // Profile touching the axis: a quarter cone.
        let t = poly(&[DVec3::ZERO, DVec3::new(0.0, 2.0, 0.0), DVec3::new(0.0, 0.0, 3.0)]);
        let b = revolve(&t, DVec3::ZERO, DVec3::Z, PI / 2.0).unwrap();
        assert_eq!(crate::ops::boolean::open_edges(&b), 0);
        assert!((vol(&b) - PI * 4.0 * 3.0 / 12.0).abs() < 1e-3, "{}", vol(&b));
    }

    #[test]
    fn spline_profile_vase() {
        let pts = [DVec3::new(1.5, 0.0, 0.0), DVec3::new(2.0, 0.0, 1.0), DVec3::new(1.0, 0.0, 2.5), DVec3::new(1.2, 0.0, 4.0)];
        let sp = ductile_geom::NurbsCurve::interpolate(&pts, 3);
        let (t0, t1) = sp.domain();
        // Volume swept by the spline: integral of pi x^2 dz along it.
        let n = 4000;
        let swept: f64 = (0..n)
            .map(|i| {
                let t = t0 + (t1 - t0) * (i as f64 + 0.5) / n as f64;
                let d = sp.derivs(t, 1);
                PI * d[0].x * d[0].x * d[1].z * (t1 - t0) / n as f64
            })
            .sum();
        let l = Loop3 {
            segments: vec![
                Segment::line(DVec3::ZERO, pts[0]),
                Segment::new(Curve3::Nurbs(sp), t0, t1),
                Segment::line(pts[3], DVec3::new(0.0, 0.0, 4.0)),
                Segment::line(DVec3::new(0.0, 0.0, 4.0), DVec3::ZERO),
            ],
        };
        let b = revolve_loop(&l, DVec3::ZERO, DVec3::Z).unwrap();
        assert_eq!(crate::ops::boolean::open_edges(&b), 0);
        assert!((vol(&b) - swept).abs() < 1e-3 * swept, "{} vs {swept}", vol(&b));
        let half = revolve(&l, DVec3::ZERO, DVec3::Z, PI).unwrap();
        assert_eq!(crate::ops::boolean::open_edges(&half), 0);
        assert!((vol(&half) - swept / 2.0).abs() < 1e-3 * swept, "{} vs {}", vol(&half), swept / 2.0);
    }
}
