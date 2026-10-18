// SPDX-License-Identifier: GPL-3.0-only
//! Match surface: reshapes a surface along one of its open edges so it
//! lands on another edge and continues the face beyond that edge with
//! position (G0), tangent (G1) or curvature (G2) continuity.
//!
//! The face is turned so the edge is its v = start side; its first one,
//! two or three control rows are then solved so that at the rows'
//! Greville points the surface meets the target edge, leaves it along the
//! target face's tangent plane, and bends with the target face's normal
//! curvature there.

use crate::ops::bridge::{leave, Continuity, Side as Target};
use crate::ops::extend::{edge_side, replace_face, reversed_v, Side};
use crate::topo::{Body, EdgeId};
use ductile_geom::nurbs::basis::{basis_funs, find_span};
use ductile_geom::{Curve3, DVec3, DVec4, NurbsCurve, NurbsSurface};

/// Solves the dense system `a x = b` for several right-hand sides.
fn solve(mut a: Vec<Vec<f64>>, mut b: Vec<DVec3>) -> Option<Vec<DVec3>> {
    let n = b.len();
    for c in 0..n {
        let p = (c..n).max_by(|i, j| a[*i][c].abs().total_cmp(&a[*j][c].abs()))?;
        if a[p][c].abs() < 1e-14 {
            return None;
        }
        a.swap(c, p);
        b.swap(c, p);
        for r in c + 1..n {
            let f = a[r][c] / a[c][c];
            if f != 0.0 {
                for k in c..n {
                    a[r][k] -= f * a[c][k];
                }
                b[r] = b[r] - b[c] * f;
            }
        }
    }
    for c in (0..n).rev() {
        let mut v = b[c];
        for k in c + 1..n {
            v -= b[k] * a[c][k];
        }
        b[c] = v / a[c][c];
    }
    Some(b)
}

/// Arc-length fractions of parameters `us` along the curve `u ↦ f(u)` over [u0, u1].
fn fractions(f: &dyn Fn(f64) -> DVec3, u0: f64, u1: f64, us: &[f64]) -> Vec<f64> {
    const FINE: usize = 400;
    let mut acc = vec![0.0];
    for i in 0..FINE {
        let (a, b) = (u0 + (u1 - u0) * i as f64 / FINE as f64, u0 + (u1 - u0) * (i + 1) as f64 / FINE as f64);
        acc.push(acc[i] + f(a).distance(f(b)));
    }
    let total = acc[FINE].max(1e-300);
    us.iter()
        .map(|u| {
            let x = ((u - u0) / (u1 - u0) * FINE as f64).clamp(0.0, FINE as f64);
            let i = (x.floor() as usize).min(FINE - 1);
            (acc[i] + (acc[i + 1] - acc[i]) * (x - i as f64)) / total
        })
        .collect()
}

/// The parameter at arc-length fraction `f` of an edge.
fn at_fraction(c: &Curve3, t0: f64, t1: f64, f: f64) -> f64 {
    // Bisection on the fraction, which grows with the parameter.
    let (mut lo, mut hi) = (t0, t1);
    let frac = |t: f64| fractions(&|s| c.point(s), t0, t1, &[t])[0];
    for _ in 0..50 {
        let mid = 0.5 * (lo + hi);
        if frac(mid) < f {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

/// Raises the v degree to at least `q` and refines v until there are at
/// least `rows` control rows, so the rows set here leave the far side be.
fn prepare(s: &NurbsSurface, q: usize, rows: usize) -> NurbsSurface {
    let columns = |s: &NurbsSurface| -> Vec<NurbsCurve> { (0..s.nu).map(|i| NurbsCurve::new(s.degree_v, s.knots_v.clone(), (0..s.nv).map(|j| s.cp(i, j)).collect())).collect() };
    let mut cols = columns(s);
    if cols[0].degree < q {
        cols = cols.iter().map(|c| c.elevated_to(q)).collect();
    }
    while cols[0].ctrl.len() < rows {
        let k = &cols[0].knots;
        let (i, _) = k.windows(2).enumerate().max_by(|a, b| (a.1[1] - a.1[0]).total_cmp(&(b.1[1] - b.1[0]))).unwrap();
        let mid = 0.5 * (k[i] + k[i + 1]);
        cols = cols.iter().map(|c| c.insert_knot(mid, 1)).collect();
    }
    let (nv, dv, kv) = (cols[0].ctrl.len(), cols[0].degree, cols[0].knots.clone());
    let ctrl: Vec<DVec4> = cols.iter().flat_map(|c| c.ctrl.iter().copied()).collect();
    NurbsSurface::new(s.degree_u, dv, s.knots_u.clone(), kv, s.nu, nv, ctrl)
}

/// Reshapes `s` along its v = start side to meet `target`.
fn match_v_start(s: &NurbsSurface, target: &Target) -> Option<NurbsSurface> {
    if s.ctrl.iter().any(|h| (h.w - 1.0).abs() > 1e-12) {
        return None;
    }
    let need = match target.continuity {
        Continuity::G0 => 1,
        Continuity::G1 => 2,
        Continuity::G2 => 3,
    };
    let mut s = prepare(s, need.max(2), 2 * need);
    let (p, q) = (s.degree_u, s.degree_v);
    let ((u0, u1), (v0, _)) = s.domain();
    let ku = s.knots_u.clone();
    let greville: Vec<f64> = (0..s.nu).map(|i| ku[i + 1..=i + p].iter().sum::<f64>() / p as f64).collect();
    let fr = fractions(&|u| s.point(u, v0), u0, u1, &greville);
    // Run the target edge the way the side runs.
    let (a0, a1) = (s.point(u0, v0), s.point(u1, v0));
    let (b0, b1) = (target.curve.point(target.t0), target.curve.point(target.t1));
    let flip = a0.distance(b1) + a1.distance(b0) < a0.distance(b0) + a1.distance(b1);
    let ts: Vec<f64> = fr.iter().map(|f| at_fraction(&target.curve, target.t0, target.t1, if flip { 1.0 - f } else { *f })).collect();
    // Collocation matrix of the u basis at the Greville points.
    let n = s.nu;
    let mut m = vec![vec![0.0; n]; n];
    for (i, u) in greville.iter().enumerate() {
        let span = find_span(n - 1, p, *u, &ku);
        for (k, b) in basis_funs(span, *u, p, &ku).into_iter().enumerate() {
            m[i][span - p + k] = b;
        }
    }
    let cp = |s: &NurbsSurface, i: usize, j: usize| s.cp(i, j).truncate();
    let row0 = solve(m.clone(), ts.iter().map(|t| target.curve.point(*t)).collect())?;
    let kv = s.knots_v.clone();
    let (h1, h2) = (kv[q + 1] - v0, kv[q + 2] - v0);
    let mut rows = vec![row0];
    if need >= 2 {
        // Keep how far the surface reaches across: the old first row step.
        let old: Vec<DVec3> = (0..n).map(|i| cp(&s, i, 1) - cp(&s, i, 0)).collect();
        let old_at = solve_free(&m, &old);
        let mut d1 = Vec::with_capacity(n);
        let mut info = Vec::with_capacity(n);
        for (i, t) in ts.iter().enumerate() {
            let face = target.face.as_ref()?;
            let (out, nrm, k) = leave(&target.curve, *t, face)?;
            let len = old_at[i].length() * target.tension;
            d1.push(out * len);
            info.push((nrm, k, len));
        }
        let a = solve(m.clone(), d1.clone())?;
        rows.push((0..n).map(|i| rows[0][i] + a[i]).collect());
        if need >= 3 {
            // Σ N (P2 - P1) / h2 - Σ N (P1 - P0) / h1 = W h1 / (q (q - 1)),
            // with W = κ |S_v|² n and |S_v| = q len / h1.
            let rhs: Vec<DVec3> = info
                .iter()
                .enumerate()
                .map(|(i, (nrm, k, len))| {
                    let sv = q as f64 * len / h1;
                    let w = *nrm * (*k * sv * sv);
                    (w * h1 / (q * (q - 1)) as f64 + d1[i] / h1) * h2
                })
                .collect();
            let b = solve(m.clone(), rhs)?;
            rows.push((0..n).map(|i| rows[1][i] + b[i]).collect());
        }
    }
    for i in 0..n {
        for (j, r) in rows.iter().enumerate() {
            s.ctrl[i * s.nv + j] = r[i].extend(1.0);
        }
    }
    Some(s)
}

/// The values at the Greville points of the spline with control vectors `c`.
fn solve_free(m: &[Vec<f64>], c: &[DVec3]) -> Vec<DVec3> {
    m.iter().map(|row| row.iter().zip(c).map(|(b, v)| *v * *b).sum()).collect()
}

/// Reshapes the face of sheet `body` along its open edge `e` to meet the
/// edge `target` describes with its continuity.
pub fn match_surface(body: &Body, e: EdgeId, target: &Target) -> Result<Body, String> {
    let (f, ns, side, tol) = edge_side(body, e)?;
    let turned = match side {
        Side::VStart => ns,
        Side::VEnd => reversed_v(&ns),
        Side::UStart => ns.transposed(),
        Side::UEnd => reversed_v(&ns.transposed()),
    };
    let fail = || "could not match the surface (rational surfaces, or a target without a face for G1 / G2)".to_string();
    let m = match_v_start(&turned, target).ok_or_else(fail)?;
    let back = match side {
        Side::VStart => m,
        Side::VEnd => reversed_v(&m),
        Side::UStart => m.transposed(),
        Side::UEnd => reversed_v(&m).transposed(),
    };
    Ok(replace_face(body, f, back, tol))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ops::bridge::Side as Target;
    use crate::profile::{Loop3, Segment};
    use ductile_geom::{Frame, Surface};

    /// A wavy sheet lying a little off the end of a cylinder's side: after
    /// matching its near edge to the cylinder's seam line it meets it with
    /// the cylinder's tangent and curvature.
    #[test]
    fn matches_a_sheet_to_a_cylinder() {
        let cyl = crate::primitives::make_cylinder(&Frame::WORLD, 1.0, 2.0);
        let seam = cyl.edge_ids().find(|e| cyl.edge(*e).curve.is_linear()).unwrap();
        let p = cyl.edge(seam).start();
        let radial = DVec3::new(p.x, p.y, 0.0).normalize();
        let tangent = DVec3::Z.cross(radial);
        // The sheet: a curve leaving roughly along the tangent, swept up Z.
        let start = p + tangent * 0.3 + radial * 0.2;
        let c = NurbsCurve::interpolate(&[start, start + tangent * 1.0 + radial * 0.1, start + tangent * 2.0 - radial * 0.3], 3);
        let (t0, t1) = c.domain();
        let sheet = crate::ops::sheet::extrude_sheet(&[Loop3 { segments: vec![Segment::new(Curve3::Nurbs(c), t0, t1)] }], DVec3::Z * 2.0).unwrap();
        let near = sheet.edge_ids().filter(|e| sheet.edge_faces(*e).len() == 1).min_by(|a, b| sheet.edge(*a).mid().distance(p).total_cmp(&sheet.edge(*b).mid().distance(p))).unwrap();
        let target = Target::of_edge(&cyl, seam, start + tangent, Continuity::G2, 1.0);
        let m = match_surface(&sheet, near, &target).unwrap();
        let Surface::Nurbs(s) = &m.face(crate::topo::FaceId(0)).surface else { panic!() };
        // Find the matched side: the iso line lying on the seam.
        let ((u0, u1), (v0, v1)) = s.domain();
        let on_seam = |q: DVec3| DVec3::new(q.x, q.y, 0.0).distance(DVec3::new(p.x, p.y, 0.0)) < 1e-4;
        let (side_u, at) = [(true, u0), (true, u1), (false, v0), (false, v1)]
            .into_iter()
            .find(|(is_u, x)| (0..=4).all(|i| {
                let t = i as f64 / 4.0;
                on_seam(if *is_u { s.point(*x, v0 + (v1 - v0) * t) } else { s.point(u0 + (u1 - u0) * t, *x) })
            }))
            .expect("an edge on the seam");
        // Across that side: tangent along the cylinder's tangent direction,
        // curvature 1 / r towards the axis.
        let w = if side_u { (v0 + v1) * 0.5 } else { (u0 + u1) * 0.5 };
        let d = if side_u { s.derivs(at, w, 2) } else { s.derivs(w, at, 2) };
        let (c1, c2) = if side_u { (d[1][0], d[2][0]) } else { (d[0][1], d[0][2]) };
        let c1n = c1.normalize();
        assert!(c1n.cross(tangent).length() < 1e-9, "tangent {c1n} vs {tangent}");
        let k = c1.cross(c2).length() / c1.length().powi(3);
        assert!((k - 1.0).abs() < 1e-6, "curvature {k}");
    }
}
