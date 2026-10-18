// SPDX-License-Identifier: GPL-3.0-only
//! Primitive solids built directly as B-rep with exact analytic geometry.

use crate::topo::{Body, BodyKind, Coedge, EdgeId, VertexId};
use ductile_geom::{Curve3, DVec3, Frame, Surface};
use std::collections::HashMap;
use std::f64::consts::{FRAC_PI_2, TAU};

/// Builds a polyhedral solid (or sheet) from points and faces given as
/// vertex index lists ordered counter-clockwise when seen from outside.
/// Faces must be planar. Shared edges are created once.
pub fn polyhedron(points: &[DVec3], faces: &[Vec<usize>], kind: BodyKind) -> Body {
    let mut b = Body::new(kind);
    let vs: Vec<VertexId> = points.iter().map(|p| b.add_vertex(*p)).collect();
    let mut edge_map: HashMap<(usize, usize), EdgeId> = HashMap::new();
    for f in faces {
        let n = f.len();
        let mut coedges = Vec::with_capacity(n);
        for i in 0..n {
            let (a, c) = (f[i], f[(i + 1) % n]);
            let key = (a.min(c), a.max(c));
            let e = *edge_map.entry(key).or_insert_with(|| b.add_line_edge(vs[key.0], vs[key.1]));
            coedges.push(Coedge::new(e, a != key.0));
        }
        // Newell normal of the polygon.
        let mut nrm = DVec3::ZERO;
        for i in 0..n {
            let p = points[f[i]];
            let q = points[f[(i + 1) % n]];
            nrm += DVec3::new((p.y - q.y) * (p.z + q.z), (p.z - q.z) * (p.x + q.x), (p.x - q.x) * (p.y + q.y));
        }
        let x_hint = points[f[1]] - points[f[0]];
        let frame = Frame::new(points[f[0]], nrm, x_hint);
        b.add_face(Surface::Plane { frame }, false, vec![coedges]);
    }
    b
}

/// Axis-aligned box spanning the two corners, placed in `frame`.
pub fn make_box(frame: &Frame, size: DVec3) -> Body {
    let (lx, ly, lz) = (size.x.abs(), size.y.abs(), size.z.abs());
    let o = DVec3::new(size.x.min(0.0), size.y.min(0.0), size.z.min(0.0));
    let p = |x: f64, y: f64, z: f64| frame.to_world(o + DVec3::new(x, y, z));
    let pts = [
        p(0.0, 0.0, 0.0),
        p(lx, 0.0, 0.0),
        p(lx, ly, 0.0),
        p(0.0, ly, 0.0),
        p(0.0, 0.0, lz),
        p(lx, 0.0, lz),
        p(lx, ly, lz),
        p(0.0, ly, lz),
    ];
    let faces = vec![
        vec![0, 3, 2, 1], // bottom (-z)
        vec![4, 5, 6, 7], // top (+z)
        vec![0, 1, 5, 4], // front (-y)
        vec![2, 3, 7, 6], // back (+y)
        vec![1, 2, 6, 5], // right (+x)
        vec![3, 0, 4, 7], // left (-x)
    ];
    let mut b = polyhedron(&pts, &faces, BodyKind::Solid);
    if !frame.is_right_handed() {
        b = flip_orientation(b);
    }
    b
}

/// Reverses the orientation of every face (turns a solid inside out).
pub fn flip_orientation(mut b: Body) -> Body {
    for f in b.faces.iter_mut() {
        f.reversed = !f.reversed;
    }
    for l in b.loops.iter_mut() {
        l.coedges.reverse();
        for c in l.coedges.iter_mut() {
            c.reversed = !c.reversed;
        }
    }
    b
}

/// Cylinder with its base circle centred at `frame.origin`, axis along
/// `frame.z`, extending `height` (may be negative).
pub fn make_cylinder(frame: &Frame, radius: f64, height: f64) -> Body {
    make_frustum(frame, radius, radius, height)
}

/// Cone frustum: base radius `r0` at the origin, top radius `r1` at `height`.
/// Either radius may be zero to form a cone with an apex.
pub fn make_frustum(frame: &Frame, r0: f64, r1: f64, height: f64) -> Body {
    let frame = if height < 0.0 {
        Frame { origin: frame.origin + frame.z * height, ..*frame }
    } else {
        *frame
    };
    let (r0, r1, h) = if height < 0.0 { (r1, r0, -height) } else { (r0, r1, height) };
    let mut b = Body::new(BodyKind::Solid);
    let top_frame = Frame { origin: frame.origin + frame.z * h, ..frame };
    let side = if (r0 - r1).abs() < 1e-12 {
        Surface::Cylinder { frame, radius: r0 }
    } else {
        Surface::Cone { frame, radius: r0, half_angle: ((r1 - r0) / h).atan() }
    };
    let vb = b.add_vertex(frame.to_world(DVec3::new(r0, 0.0, 0.0)));
    let vt = b.add_vertex(top_frame.to_world(DVec3::new(r1, 0.0, 0.0)));
    let seam = b.add_line_edge(vb, vt);
    let mut side_loop = Vec::new();
    let mut bottom = None;
    let mut top = None;
    if r0 > 1e-12 {
        let e = b.add_edge(Curve3::Circle { frame, radius: r0 }, 0.0, TAU, vb, vb);
        side_loop.push(Coedge::new(e, false));
        bottom = Some(e);
    }
    side_loop.push(Coedge::new(seam, false));
    if r1 > 1e-12 {
        let e = b.add_edge(Curve3::Circle { frame: top_frame, radius: r1 }, 0.0, TAU, vt, vt);
        side_loop.push(Coedge::new(e, true));
        top = Some(e);
    }
    side_loop.push(Coedge::new(seam, true));
    b.add_face(side, false, vec![side_loop]);
    if let Some(e) = bottom {
        b.add_face(Surface::Plane { frame: frame.flipped() }, false, vec![vec![Coedge::new(e, true)]]);
    }
    if let Some(e) = top {
        b.add_face(Surface::Plane { frame: top_frame }, false, vec![vec![Coedge::new(e, false)]]);
    }
    b
}

pub fn make_sphere(center: DVec3, radius: f64) -> Body {
    make_sphere_in(&Frame { origin: center, ..Frame::WORLD }, radius)
}

pub fn make_sphere_in(frame: &Frame, radius: f64) -> Body {
    let mut b = Body::new(BodyKind::Solid);
    let s = b.add_vertex(frame.to_world(DVec3::new(0.0, 0.0, -radius)));
    let n = b.add_vertex(frame.to_world(DVec3::new(0.0, 0.0, radius)));
    // Meridian in the frame's XZ plane: point(t) = r (cos t x + sin t z).
    let mf = Frame { origin: frame.origin, x: frame.x, y: frame.z, z: frame.x.cross(frame.z) };
    let seam = b.add_edge(Curve3::Circle { frame: mf, radius }, -FRAC_PI_2, FRAC_PI_2, s, n);
    b.add_face(Surface::Sphere { frame: *frame, radius }, false, vec![vec![Coedge::new(seam, false), Coedge::new(seam, true)]]);
    b
}

pub fn make_torus(frame: &Frame, major: f64, minor: f64) -> Body {
    let mut b = Body::new(BodyKind::Solid);
    let v = b.add_vertex(frame.to_world(DVec3::new(major + minor, 0.0, 0.0)));
    // Equator (v = 0) around the axis.
    let eq = b.add_edge(
        Curve3::Circle { frame: *frame, radius: major + minor },
        0.0,
        TAU,
        v,
        v,
    );
    // Tube cross-section (u = 0): point(t) = c + m (cos t x + sin t z).
    let mf = Frame { origin: frame.origin + frame.x * major, x: frame.x, y: frame.z, z: frame.x.cross(frame.z) };
    let mer = b.add_edge(Curve3::Circle { frame: mf, radius: minor }, 0.0, TAU, v, v);
    b.add_face(
        Surface::Torus { frame: *frame, major, minor },
        false,
        vec![vec![Coedge::new(eq, false), Coedge::new(mer, false), Coedge::new(eq, true), Coedge::new(mer, true)]],
    );
    b
}
