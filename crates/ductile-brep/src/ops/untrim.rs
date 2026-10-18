// SPDX-License-Identifier: GPL-3.0-only
//! Untrim: the whole surface under a trimmed face, as a sheet of its own;
//! and reversing a sheet's front and back.

use crate::ops::isoparam::face_param_range;
use crate::ops::sheet::Sewer;
use crate::profile::Segment;
use crate::topo::{Body, BodyKind, FaceId};
use ductile_geom::{Curve3, NurbsCurve, Surface};

/// A sheet of the untrimmed surface under face `f`: a NURBS surface's
/// whole domain; an analytic surface over the face's parameter range,
/// widened to the full turn where the face goes nearly all the way round.
pub fn untrim(b: &Body, f: FaceId) -> Option<Body> {
    let face = b.face(f);
    let s = &face.surface;
    let (mut u, mut v) = (face_param_range(b, f, true)?, face_param_range(b, f, false)?);
    let (bu, bv) = s.bounds();
    let widen = |r: &mut (f64, f64), bound: Option<(f64, f64)>, period: Option<f64>| {
        if let Some((lo, hi)) = bound {
            match period {
                // A periodic direction: the full turn once the face nearly spans it.
                Some(p) if r.1 - r.0 > p * 0.98 => *r = (r.0, r.0 + p),
                Some(_) => {}
                None => *r = (lo, hi),
            }
        }
    };
    match s {
        Surface::Nurbs(n) => {
            let ((u0, u1), (v0, v1)) = n.domain();
            (u, v) = ((u0, u1), (v0, v1));
        }
        _ => {
            widen(&mut u, bu, s.period_u());
            widen(&mut v, bv, s.period_v());
        }
    }
    let net = match s {
        Surface::Nurbs(n) => n.clone(),
        _ => s.to_nurbs(u.0, u.1, v.0, v.1),
    };
    let iso = |c: NurbsCurve, rev: bool| {
        let (a, z) = c.domain();
        Segment { curve: Curve3::Nurbs(c), t0: a, t1: z, reversed: rev }
    };
    let mut lp = vec![iso(net.iso_v(v.0), false), iso(net.iso_u(u.1), false), iso(net.iso_v(v.1), true), iso(net.iso_u(u.0), true)];
    if face.reversed {
        lp = lp.into_iter().rev().map(|x| x.flipped()).collect();
    }
    let size = net.bbox().diagonal().max(1e-9);
    let mut sw = Sewer::new(BodyKind::Sheet, 1e-7 * size);
    sw.face(s.clone(), face.reversed, &lp, &[]);
    Some(sw.body)
}

/// `b` with every face's front and back swapped.
pub fn reverse_sheet(b: &Body) -> Body {
    let mut out = b.clone();
    for face in out.faces.iter_mut() {
        face.reversed = !face.reversed;
    }
    for l in out.loops.iter_mut() {
        l.coedges.reverse();
        for c in l.coedges.iter_mut() {
            c.reversed = !c.reversed;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::make_cylinder;
    use ductile_geom::{DVec3, Frame};

    #[test]
    fn untrims_a_drilled_plate_and_a_cylinder() {
        // A plate with a hole: its top face untrimmed is the plain rectangle.
        let plate = crate::primitives::make_box(&Frame::WORLD, DVec3::new(4.0, 2.0, 1.0));
        let hole = make_cylinder(&Frame { origin: DVec3::new(2.0, 1.0, -1.0), ..Frame::WORLD }, 0.5, 3.0);
        let drilled = crate::ops::boolean::boolean(&plate, &hole, crate::ops::boolean::BoolOp::Difference).unwrap();
        let top = drilled.face_ids().find(|f| matches!(drilled.face(*f).surface, Surface::Plane { frame } if frame.origin.z > 0.99 || frame.z.z.abs() > 0.99 && drilled.face_loops(*f).count() == 2)).unwrap();
        let sheet = untrim(&drilled, top).unwrap();
        assert_eq!(sheet.faces.len(), 1);
        assert_eq!(sheet.face_loops(FaceId(0)).count(), 1);
        let area = crate::tessellate(&sheet, &crate::TessOptions::for_size(5.0)).area();
        assert!((area - 8.0).abs() < 1e-6, "{area}");
        // A quarter of a cylinder wall stays a quarter; a full one is whole.
        let c = make_cylinder(&Frame::WORLD, 1.0, 2.0);
        let wall = c.face_ids().find(|f| matches!(c.face(*f).surface, Surface::Cylinder { .. })).unwrap();
        let sheet = untrim(&c, wall).unwrap();
        let area = crate::tessellate(&sheet, &crate::TessOptions::for_size(3.0)).area();
        assert!((area - std::f64::consts::TAU * 2.0).abs() < 1e-2, "{area}");
        let back = reverse_sheet(&sheet);
        assert_ne!(back.face(FaceId(0)).reversed, sheet.face(FaceId(0)).reversed);
    }
}
