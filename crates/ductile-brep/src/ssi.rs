// SPDX-License-Identifier: GPL-3.0-only
//! Intersections between surfaces and between curves and surfaces.
//!
//! Analytic pairs (plane/plane, plane/quadric, coaxial surfaces of
//! revolution, ...) are solved in closed form so the resulting edges carry
//! exact lines, circles and ellipses. Everything else is traced by marching
//! along `n_a × n_b` from seed points with a tangent-plane corrector, and the
//! result is fitted with a cubic NURBS.

use ductile_geom::{BBox3, Curve3, DVec2, DVec3, Frame, NurbsCurve, Surface};

#[derive(Clone, Debug)]
pub enum Ssi {
    /// The surfaces do not meet inside the region of interest.
    None,
    /// The surfaces are the same (coplanar planes, identical quadrics).
    Coincident,
    /// Intersection curves with the parameter range of interest.
    Curves(Vec<(Curve3, f64, f64)>),
}

/// Signed distance from `p` to the surface, positive on the side of the normal.
pub fn signed_distance(s: &Surface, p: DVec3) -> f64 {
    match s {
        Surface::Plane { frame } => (p - frame.origin).dot(frame.z),
        Surface::Cylinder { frame, radius } => {
            let l = frame.to_local(p);
            (l.x * l.x + l.y * l.y).sqrt() - radius
        }
        Surface::Sphere { frame, radius } => (p - frame.origin).length() - radius,
        Surface::Torus { frame, major, minor } => {
            let l = frame.to_local(p);
            let rho = (l.x * l.x + l.y * l.y).sqrt();
            ((rho - major).powi(2) + l.z * l.z).sqrt() - minor
        }
        Surface::Cone { frame, radius, half_angle } => {
            let l = frame.to_local(p);
            let rho = (l.x * l.x + l.y * l.y).sqrt();
            let (sn, cs) = half_angle.sin_cos();
            // Distance to the generator line rho = radius + z tan(a), signed outward.
            (rho - radius) * cs - l.z * sn
        }
        Surface::Nurbs(_) => s.signed_distance(p),
    }
}

/// Unsigned distance to the surface using projection (robust for NURBS too).
pub fn distance(s: &Surface, p: DVec3) -> f64 {
    match s {
        Surface::Nurbs(_) => {
            let uv = s.project(p, None);
            s.point(uv.x, uv.y).distance(p)
        }
        _ => signed_distance(s, p).abs(),
    }
}

fn foot(s: &Surface, p: DVec3, hint: Option<DVec2>) -> (DVec3, DVec3, DVec2) {
    let uv = s.project(p, hint);
    let q = s.point(uv.x, uv.y);
    let n = s.normal(uv.x, uv.y);
    (q, n, uv)
}

/// Moves `p` onto the intersection of two surfaces (tangent plane corrector).
pub fn refine_point(a: &Surface, b: &Surface, p: DVec3, tol: f64) -> Option<DVec3> {
    refine_point_hinted(a, b, p, tol, &mut (None, None))
}

/// `refine_point` starting the projections from (and leaving in) `hints`,
/// the surface parameters of a nearby point.
fn refine_point_hinted(a: &Surface, b: &Surface, p: DVec3, tol: f64, hints: &mut (Option<DVec2>, Option<DVec2>)) -> Option<DVec3> {
    let mut p = p;
    let (mut ha, mut hb) = *hints;
    for _ in 0..40 {
        let (qa, na, uva) = foot(a, p, ha);
        let (qb, nb, uvb) = foot(b, p, hb);
        ha = Some(uva);
        hb = Some(uvb);
        let t = na.cross(nb);
        if t.length_squared() < 1e-16 {
            // Tangential contact: fall back to alternating projection.
            let mid = (qa + qb) * 0.5;
            if qa.distance(qb) < tol {
                return Some(mid);
            }
            p = mid;
            continue;
        }
        // Solve [na; nb; t] x = [na·qa; nb·qb; t·p].
        let m = ductile_geom::DMat3::from_cols(na, nb, t).transpose();
        let rhs = DVec3::new(na.dot(qa), nb.dot(qb), t.dot(p));
        if m.determinant().abs() < 1e-20 {
            return None;
        }
        let x = m.inverse() * rhs;
        let step = x.distance(p);
        p = x;
        if step < tol * 1e-3 {
            break;
        }
    }
    let (qa, _, uva) = foot(a, p, ha);
    let (qb, _, uvb) = foot(b, p, hb);
    let (da, db) = (qa.distance(p), qb.distance(p));
    let ok = da < tol * 10.0 && db < tol * 10.0;
    if ok {
        *hints = (Some(uva), Some(uvb));
    }
    ok.then_some(p)
}

fn tangent_at_hinted(a: &Surface, b: &Surface, p: DVec3, hints: (Option<DVec2>, Option<DVec2>)) -> Option<DVec3> {
    let (_, na, _) = foot(a, p, hints.0);
    let (_, nb, _) = foot(b, p, hints.1);
    let t = na.cross(nb);
    (t.length_squared() > 1e-16).then(|| t.normalize())
}


/// Parameter range of a line clipped to a box.
fn clip_line(origin: DVec3, dir: DVec3, roi: &BBox3) -> Option<(f64, f64)> {
    let mut t0 = f64::NEG_INFINITY;
    let mut t1 = f64::INFINITY;
    for k in 0..3 {
        let (o, d, lo, hi) = (origin[k], dir[k], roi.min[k], roi.max[k]);
        if d.abs() < 1e-15 {
            if o < lo || o > hi {
                return None;
            }
        } else {
            let a = (lo - o) / d;
            let b = (hi - o) / d;
            t0 = t0.max(a.min(b));
            t1 = t1.min(a.max(b));
        }
    }
    (t1 > t0).then_some((t0, t1))
}

fn line_curve(origin: DVec3, dir: DVec3, roi: &BBox3) -> Option<(Curve3, f64, f64)> {
    let dir = dir.normalize();
    clip_line(origin, dir, roi).map(|(a, b)| (Curve3::Line { origin, dir }, a, b))
}

fn circle_curve(center: DVec3, normal: DVec3, x_hint: DVec3, r: f64) -> (Curve3, f64, f64) {
    let frame = Frame::new(center, normal, x_hint);
    (Curve3::Circle { frame, radius: r }, 0.0, std::f64::consts::TAU)
}

/// Profile of a surface of revolution about a common axis, in (rho, z) coordinates.
#[derive(Clone, Copy, Debug)]
enum Profile {
    /// rho = c
    Vertical(f64),
    /// z = c
    Horizontal(f64),
    /// rho = r0 + z * k
    Slant(f64, f64),
    /// circle with centre (rho, z) and radius
    Circle(f64, f64, f64),
}

/// Expresses `s` as a surface of revolution about the axis (origin, dir).
fn revolution_profile(s: &Surface, origin: DVec3, dir: DVec3, tol: f64) -> Option<Profile> {
    let on_axis = |p: DVec3| ((p - origin) - dir * (p - origin).dot(dir)).length() < tol;
    let parallel = |d: DVec3| d.cross(dir).length() < 1e-9;
    let z_of = |p: DVec3| (p - origin).dot(dir);
    match s {
        Surface::Plane { frame } if parallel(frame.z) => Some(Profile::Horizontal(z_of(frame.origin))),
        Surface::Cylinder { frame, radius } if parallel(frame.z) && on_axis(frame.origin) => Some(Profile::Vertical(*radius)),
        Surface::Cone { frame, radius, half_angle } if parallel(frame.z) && on_axis(frame.origin) => {
            let sgn = frame.z.dot(dir).signum();
            let z0 = z_of(frame.origin);
            let k = half_angle.tan() * sgn;
            Some(Profile::Slant(radius - z0 * k, k))
        }
        Surface::Sphere { frame, radius } if on_axis(frame.origin) => Some(Profile::Circle(0.0, z_of(frame.origin), *radius)),
        Surface::Torus { frame, major, minor } if parallel(frame.z) && on_axis(frame.origin) => {
            Some(Profile::Circle(*major, z_of(frame.origin), *minor))
        }
        _ => None,
    }
}

/// Intersection points of two profiles in the (rho, z) half plane.
fn profile_intersections(a: Profile, b: Profile) -> Option<Vec<(f64, f64)>> {
    use Profile::*;
    // Represent lines as (p, d): point + direction in (rho, z).
    let as_line = |p: Profile| -> Option<(DVec2, DVec2)> {
        match p {
            Vertical(c) => Some((DVec2::new(c, 0.0), DVec2::new(0.0, 1.0))),
            Horizontal(c) => Some((DVec2::new(0.0, c), DVec2::new(1.0, 0.0))),
            Slant(r0, k) => Some((DVec2::new(r0, 0.0), DVec2::new(k, 1.0).normalize())),
            Circle(..) => None,
        }
    };
    let line_line = |(p1, d1): (DVec2, DVec2), (p2, d2): (DVec2, DVec2)| -> Option<Vec<(f64, f64)>> {
        let den = d1.perp_dot(d2);
        if den.abs() < 1e-12 {
            // Parallel: coincident if p2 lies on line 1.
            return if (p2 - p1).perp_dot(d1).abs() < 1e-9 { None } else { Some(vec![]) };
        }
        let t = (p2 - p1).perp_dot(d2) / den;
        let q = p1 + d1 * t;
        Some(vec![(q.x, q.y)])
    };
    let line_circle = |(p, d): (DVec2, DVec2), c: DVec2, r: f64| -> Vec<(f64, f64)> {
        let f = p - c;
        let bq = f.dot(d);
        let cq = f.dot(f) - r * r;
        let disc = bq * bq - cq;
        if disc < -1e-12 {
            return vec![];
        }
        let s = disc.max(0.0).sqrt();
        let mut v = vec![p + d * (-bq - s)];
        if s > 1e-12 {
            v.push(p + d * (-bq + s));
        }
        v.into_iter().map(|q| (q.x, q.y)).collect()
    };
    match (as_line(a), as_line(b)) {
        (Some(la), Some(lb)) => line_line(la, lb),
        (Some(l), None) | (None, Some(l)) => {
            let Circle(cr, cz, r) = (if as_line(a).is_none() { a } else { b }) else { unreachable!() };
            Some(line_circle(l, DVec2::new(cr, cz), r))
        }
        (None, None) => {
            let (Circle(r1, z1, a1), Circle(r2, z2, a2)) = (a, b) else { unreachable!() };
            let c1 = DVec2::new(r1, z1);
            let c2 = DVec2::new(r2, z2);
            let d = c1.distance(c2);
            if d < 1e-12 {
                return if (a1 - a2).abs() < 1e-9 { None } else { Some(vec![]) };
            }
            if d > a1 + a2 + 1e-12 || d < (a1 - a2).abs() - 1e-12 {
                return Some(vec![]);
            }
            let x = (d * d + a1 * a1 - a2 * a2) / (2.0 * d);
            let h = (a1 * a1 - x * x).max(0.0).sqrt();
            let u = (c2 - c1) / d;
            let m = c1 + u * x;
            let perp = DVec2::new(-u.y, u.x);
            let mut v = vec![m + perp * h];
            if h > 1e-12 {
                v.push(m - perp * h);
            }
            Some(v.into_iter().map(|q| (q.x, q.y)).collect())
        }
    }
}

/// Intersects two coaxial surfaces of revolution. Returns `None` when the
/// pair is not coaxial; `Some(Ssi)` otherwise.
fn coaxial(a: &Surface, b: &Surface, tol: f64) -> Option<Ssi> {
    let axis_of = |s: &Surface| -> Option<(DVec3, DVec3)> {
        match s {
            Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. } | Surface::Torus { frame, .. } => Some((frame.origin, frame.z)),
            _ => None,
        }
    };
    let (o, d) = axis_of(a).or_else(|| axis_of(b)).or_else(|| match (a, b) {
        // Sphere with a plane or another sphere: axis through the centre(s).
        (Surface::Sphere { frame, .. }, Surface::Plane { frame: pf }) | (Surface::Plane { frame: pf }, Surface::Sphere { frame, .. }) => {
            Some((frame.origin, pf.z))
        }
        _ => None,
    })?;
    let pa = revolution_profile(a, o, d, tol)?;
    let pb = revolution_profile(b, o, d, tol)?;
    let pts = match profile_intersections(pa, pb) {
        None => return Some(Ssi::Coincident),
        Some(p) => p,
    };
    let x_hint = ductile_geom::any_perpendicular(d);
    let mut curves = Vec::new();
    for (rho, z) in pts {
        if rho > tol {
            curves.push(circle_curve(o + d * z, d, x_hint, rho));
        }
    }
    Some(if curves.is_empty() { Ssi::None } else { Ssi::Curves(curves) })
}

/// Intersects two surfaces within the region of interest `roi`. `seeds` are
/// approximate intersection points (from tessellations) used by the
/// marching fallback.
pub fn intersect(a: &Surface, b: &Surface, roi: &BBox3, seeds: &[DVec3], tol: f64) -> Ssi {
    intersect_analytic(a, b, roi, tol).unwrap_or_else(|| march(a, b, roi, seeds, tol))
}

/// Closed-form intersection, or `None` if the pair needs marching.
pub fn intersect_analytic(a: &Surface, b: &Surface, roi: &BBox3, tol: f64) -> Option<Ssi> {
    analytic(a, b, roi, tol).or_else(|| analytic(b, a, roi, tol)).or_else(|| coaxial(a, b, tol))
}

/// Marching intersection from approximate seed points.
pub fn intersect_marching(a: &Surface, b: &Surface, roi: &BBox3, seeds: &[DVec3], tol: f64) -> Ssi {
    // A spline surface whose control points all lie on one side of a plane
    // can only touch it: there is nothing to cross (the touching itself,
    // as along a blend's contact line, is an edge already).
    for (p, q) in [(a, b), (b, a)] {
        if let (Surface::Plane { frame }, Surface::Nurbs(n)) = (p, q) {
            let d: Vec<f64> = n.ctrl.iter().map(|c| (c.truncate() / c.w - frame.origin).dot(frame.z)).collect();
            if d.iter().all(|x| *x > -tol) || d.iter().all(|x| *x < tol) {
                return Ssi::None;
            }
        }
    }
    march(a, b, roi, seeds, tol)
}

fn analytic(a: &Surface, b: &Surface, roi: &BBox3, tol: f64) -> Option<Ssi> {
    match (a, b) {
        (Surface::Plane { frame: fa }, Surface::Plane { frame: fb }) => {
            let d = fa.z.cross(fb.z);
            if d.length() < 1e-10 {
                let dist = (fb.origin - fa.origin).dot(fa.z);
                return Some(if dist.abs() < tol { Ssi::Coincident } else { Ssi::None });
            }
            // Point on both planes closest to fa.origin.
            let n1 = fa.z;
            let n2 = fb.z;
            let h1 = n1.dot(fa.origin);
            let h2 = n2.dot(fb.origin);
            let dd = d.length_squared();
            let p = (n2.cross(d) * h1 + d.cross(n1) * h2) / dd;
            Some(line_curve(p, d, roi).map(|c| Ssi::Curves(vec![c])).unwrap_or(Ssi::None))
        }
        (Surface::Plane { frame: pf }, Surface::Cylinder { frame: cf, radius: r }) => {
            let n = pf.z;
            let ax = cf.z;
            let c = n.dot(ax);
            if c.abs() < 1e-10 {
                let dist = (cf.origin - pf.origin).dot(n);
                if dist.abs() > r + tol {
                    return Some(Ssi::None);
                }
                let base = cf.origin - n * dist;
                let w = ax.cross(n).normalize();
                let h = (r * r - dist * dist).max(0.0).sqrt();
                let mut cs = Vec::new();
                if h < tol {
                    cs.extend(line_curve(base, ax, roi));
                } else {
                    cs.extend(line_curve(base + w * h, ax, roi));
                    cs.extend(line_curve(base - w * h, ax, roi));
                }
                return Some(if cs.is_empty() { Ssi::None } else { Ssi::Curves(cs) });
            }
            let t = (pf.origin - cf.origin).dot(n) / c;
            let center = cf.origin + ax * t;
            if c.abs() > 1.0 - 1e-12 {
                return Some(Ssi::Curves(vec![(Curve3::Circle { frame: Frame { origin: center, ..*cf }, radius: *r }, 0.0, std::f64::consts::TAU)]));
            }
            let minor = ax.cross(n).normalize();
            let major = n.cross(minor).normalize();
            let frame = Frame { origin: center, x: major, y: minor, z: major.cross(minor) };
            Some(Ssi::Curves(vec![(Curve3::Ellipse { frame, rx: r / c.abs(), ry: *r }, 0.0, std::f64::consts::TAU)]))
        }
        (Surface::Plane { frame: pf }, Surface::Sphere { frame: sf, radius: r }) => {
            let d = (sf.origin - pf.origin).dot(pf.z);
            if d.abs() >= r - tol {
                return Some(Ssi::None);
            }
            let rr = (r * r - d * d).sqrt();
            Some(Ssi::Curves(vec![circle_curve(sf.origin - pf.z * d, pf.z, pf.x, rr)]))
        }
        (Surface::Sphere { frame: f1, radius: r1 }, Surface::Sphere { frame: f2, radius: r2 }) => {
            let dv = f2.origin - f1.origin;
            let d = dv.length();
            if d < 1e-12 {
                return Some(if (r1 - r2).abs() < tol { Ssi::Coincident } else { Ssi::None });
            }
            if d >= r1 + r2 - tol || d <= (r1 - r2).abs() + tol {
                return Some(Ssi::None);
            }
            let u = dv / d;
            let x = (d * d + r1 * r1 - r2 * r2) / (2.0 * d);
            let rr = (r1 * r1 - x * x).sqrt();
            Some(Ssi::Curves(vec![circle_curve(f1.origin + u * x, u, ductile_geom::any_perpendicular(u), rr)]))
        }
        (Surface::Cylinder { frame: f1, radius: r1 }, Surface::Cylinder { frame: f2, radius: r2 }) if f1.z.cross(f2.z).length() < 1e-10 => {
            // Parallel axes: intersect the cross-section circles.
            let ax = f1.z;
            let off = f2.origin - f1.origin;
            let off = off - ax * off.dot(ax);
            let d = off.length();
            if d < tol {
                return Some(if (r1 - r2).abs() < tol { Ssi::Coincident } else { Ssi::None });
            }
            if d > r1 + r2 + tol || d < (r1 - r2).abs() - tol {
                return Some(Ssi::None);
            }
            let u = off / d;
            let x = (d * d + r1 * r1 - r2 * r2) / (2.0 * d);
            let h = (r1 * r1 - x * x).max(0.0).sqrt();
            let w = ax.cross(u);
            let mut cs = Vec::new();
            cs.extend(line_curve(f1.origin + u * x + w * h, ax, roi));
            if h > tol {
                cs.extend(line_curve(f1.origin + u * x - w * h, ax, roi));
            }
            Some(if cs.is_empty() { Ssi::None } else { Ssi::Curves(cs) })
        }
        (Surface::Nurbs(x), Surface::Nurbs(y)) if x == y => Some(Ssi::Coincident),
        _ => None,
    }
}

/// Marching intersection for general surface pairs.
fn march(a: &Surface, b: &Surface, roi: &BBox3, seeds: &[DVec3], tol: f64) -> Ssi {
    let diag = roi.diagonal().max(tol * 10.0);
    let roi_x = roi.expanded(diag * 1e-3);
    let mut traces: Vec<Vec<DVec3>> = Vec::new();
    let max_step = diag / 24.0;
    for &seed in seeds {
        // Seeds where the surfaces are (nearly) tangent lie on touching
        // contacts, which cross nothing; marching them is slow and futile.
        let (_, na, ua) = foot(a, seed, None);
        let (_, nb, ub) = foot(b, seed, None);
        if na.cross(nb).length() < 0.02 {
            continue;
        }
        let mut hints = (Some(ua), Some(ub));
        let Some(p0) = refine_point_hinted(a, b, seed, tol, &mut hints) else { continue };
        if !roi_x.contains(p0) {
            continue;
        }
        // Skip seeds already covered by a trace.
        let covered = traces.iter().any(|t| t.windows(2).any(|w| dist_point_segment3(p0, w[0], w[1]) < max_step * 0.25));
        if covered {
            continue;
        }
        let Some(t0) = tangent_at_hinted(a, b, p0, hints) else { continue };
        let (fwd, closed) = trace(a, b, p0, t0, &roi_x, tol, max_step, hints);
        let mut pts = if closed {
            fwd
        } else {
            let (bwd, _) = trace(a, b, p0, -t0, &roi_x, tol, max_step, hints);
            let mut v: Vec<DVec3> = bwd.into_iter().rev().collect();
            v.pop();
            v.extend(fwd);
            v
        };
        if closed {
            pts.push(pts[0]);
        }
        if pts.len() >= 2 {
            traces.push(pts);
        }
    }
    let mut curves = Vec::new();
    for t in traces {
        let t = dedupe(t, tol);
        if t.len() < 2 {
            continue;
        }
        let c = if t.len() == 2 { NurbsCurve::line(t[0], t[1]) } else { NurbsCurve::interpolate(&t, 3) };
        let (d0, d1) = c.domain();
        curves.push((Curve3::Nurbs(c), d0, d1));
    }
    if curves.is_empty() {
        Ssi::None
    } else {
        Ssi::Curves(curves)
    }
}

fn dedupe(pts: Vec<DVec3>, tol: f64) -> Vec<DVec3> {
    let mut out: Vec<DVec3> = Vec::with_capacity(pts.len());
    for p in pts {
        if out.last().map(|q| q.distance(p) > tol * 10.0).unwrap_or(true) {
            out.push(p);
        }
    }
    out
}

/// Traces from `p0` in direction `t0` until leaving the box or closing up.
#[allow(clippy::too_many_arguments)]
fn trace(a: &Surface, b: &Surface, p0: DVec3, t0: DVec3, roi: &BBox3, tol: f64, max_step: f64, mut hints: (Option<DVec2>, Option<DVec2>)) -> (Vec<DVec3>, bool) {
    let mut pts = vec![p0];
    let mut p = p0;
    let mut t = t0;
    let mut h = max_step * 0.25;
    let chord_tol = (tol * 50.0).max(roi.diagonal() * 2e-5);
    let mut travelled = 0.0;
    for _ in 0..20000 {
        let mut hq = hints;
        let Some(q) = refine_point_hinted(a, b, p + t * h, tol, &mut hq) else {
            h *= 0.5;
            if h < tol {
                break;
            }
            continue;
        };
        let Some(tq) = tangent_at_hinted(a, b, q, hq) else { break };
        let tq = if tq.dot(t) < 0.0 { -tq } else { tq };
        let ang = t.dot(tq).clamp(-1.0, 1.0).acos();
        let seg = q.distance(p);
        // Sagitta estimate: seg * angle / 8.
        if (seg * ang / 8.0 > chord_tol || ang > 0.2) && h > tol * 4.0 {
            h *= 0.5;
            continue;
        }
        if seg < tol {
            break;
        }
        travelled += seg;
        // Closing the loop?
        if travelled > h * 3.0 && q.distance(p0) < h * 1.1 && pts.len() > 3 {
            return (pts, true);
        }
        if !roi.contains(q) {
            // Clip the last step to the box boundary (approximately).
            pts.push(q);
            break;
        }
        pts.push(q);
        p = q;
        t = tq;
        hints = hq;
        if seg * ang / 8.0 < chord_tol * 0.25 && ang < 0.05 {
            h = (h * 1.5).min(max_step);
        }
    }
    (pts, false)
}

/// Parameters in `[t0, t1]` where the curve crosses the surface.
pub fn curve_surface_crossings(c: &Curve3, t0: f64, t1: f64, s: &Surface, tol: f64) -> Vec<f64> {
    let samples: Vec<f64> = {
        // Sample spacing follows the curve's shape, not the (much finer)
        // point tolerance; the subdivision below catches close crossings.
        let len = c.point(t0).distance(c.point(0.5 * (t0 + t1))) + c.point(0.5 * (t0 + t1)).distance(c.point(t1));
        let mut v: Vec<f64> = c.tessellate(t0, t1, (tol * 100.0).max(len * 2e-4)).into_iter().map(|x| x.0).collect();
        // Subdivide so sign changes are not missed on curved surfaces.
        let mut w = Vec::with_capacity(v.len() * 4);
        for pair in v.windows(2) {
            for k in 0..4 {
                w.push(pair[0] + (pair[1] - pair[0]) * k as f64 / 4.0);
            }
        }
        w.push(*v.last().unwrap());
        v.clear();
        w
    };
    // Consecutive samples are close, so each projection starts from the
    // last one's parameters.
    let hint = std::cell::Cell::new(None);
    let g = |t: f64| {
        let p = c.point(t);
        match s {
            Surface::Nurbs(_) => {
                let uv = s.project(p, hint.get());
                hint.set(Some(uv));
                (p - s.point(uv.x, uv.y)).dot(s.normal(uv.x, uv.y))
            }
            _ => signed_distance(s, p),
        }
    };
    let mut roots: Vec<f64> = Vec::new();
    let vals: Vec<f64> = samples.iter().map(|t| g(*t)).collect();
    for i in 0..samples.len() {
        if vals[i].abs() < tol {
            // Local minimum of |g| near zero: a touching/crossing sample.
            let left = if i > 0 { vals[i - 1].abs() } else { f64::INFINITY };
            let right = if i + 1 < vals.len() { vals[i + 1].abs() } else { f64::INFINITY };
            if vals[i].abs() <= left && vals[i].abs() <= right {
                roots.push(samples[i]);
            }
        }
        if i + 1 < samples.len() && vals[i] * vals[i + 1] < 0.0 && vals[i].abs() >= tol && vals[i + 1].abs() >= tol {
            // Illinois regula falsi.
            let (mut a, mut b) = (samples[i], samples[i + 1]);
            let (mut fa, mut fb) = (vals[i], vals[i + 1]);
            let mut side = 0;
            let mut r = a;
            for _ in 0..100 {
                r = (a * fb - b * fa) / (fb - fa);
                let fr = g(r);
                if fr.abs() < tol * 1e-4 || (b - a).abs() < 1e-15 {
                    break;
                }
                if fr * fb > 0.0 {
                    b = r;
                    fb = fr;
                    if side == -1 {
                        fa *= 0.5;
                    }
                    side = -1;
                } else {
                    a = r;
                    fa = fr;
                    if side == 1 {
                        fb *= 0.5;
                    }
                    side = 1;
                }
            }
            roots.push(r);
        }
    }
    roots.sort_by(|a, b| a.total_cmp(b));
    let mut out: Vec<f64> = Vec::new();
    for r in roots {
        let p = c.point(r);
        if distance(s, p) > tol * 10.0 {
            continue;
        }
        if out.last().map(|q| c.point(*q).distance(p) > tol * 10.0).unwrap_or(true) {
            out.push(r);
        }
    }
    out
}

/// Distance from `p` to the 3D segment `a`-`b`.
pub fn dist_point_segment3(p: DVec3, a: DVec3, b: DVec3) -> f64 {
    let d = b - a;
    let l2 = d.length_squared();
    let t = if l2 > 0.0 { ((p - a).dot(d) / l2).clamp(0.0, 1.0) } else { 0.0 };
    p.distance(a + d * t)
}
