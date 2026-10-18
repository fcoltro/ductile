// SPDX-License-Identifier: GPL-3.0-only
//! Curvature-continuous corner patches where three smooth (G2) blends of
//! mutually perpendicular plane/plane edges meet.
//!
//! Each edge blend has the same section: in coordinates `(a, b)` running
//! from the rolling-ball centre toward the two faces in units of the radius,
//! the quintic from `(1, 0)` to `(0, 1)` whose end control points lie on the
//! faces' tangents. Read as a "norm" `N2` (the section is `N2(a, b) = 1`,
//! with `N2` homogeneous of degree one), it extends to three directions by
//! composition, `N2(N2(a, b), c)`, averaged over the three ways round. On a
//! side where one coordinate vanishes this is exactly the edge's section,
//! and since the section leaves the faces with zero curvature, `N2(1, c)`
//! differs from one only at third order in `c`: the corner meets each edge
//! blend with matching curvature.
//!
//! The patch is split into three quadrilaterals around its centre, each a
//! quintic B-spline fitted to the corner by least squares with its outer
//! sides fixed to halves of the edge sections and its first rows held on
//! the lines that make the joins to the edge blends tangent and flat.

use crate::topo::{Body, BodyKind, Coedge, EdgeId, VertexId};
use ductile_geom::nurbs::basis::{basis_funs, find_span};
use ductile_geom::nurbs::curve::solve_dense;
use ductile_geom::{Curve3, DVec2, DVec3, DVec4, Frame, NurbsCurve, NurbsSurface, Surface};

/// The normalized smooth section from `(1, 0)` to `(0, 1)`.
const SECTION: [DVec2; 6] = [
    DVec2::new(1.0, 0.0),
    DVec2::new(1.0, 0.3),
    DVec2::new(1.0, 0.6),
    DVec2::new(0.6, 1.0),
    DVec2::new(0.3, 1.0),
    DVec2::new(0.0, 1.0),
];

const DEGREE: usize = 5;
/// Control points per direction of the fitted patches and inner curves.
const NCTRL: usize = 9;

fn knots() -> Vec<f64> {
    let mut k = vec![0.0; DEGREE + 1];
    k.extend([0.25, 0.5, 0.75]);
    k.extend(vec![1.0; DEGREE + 1]);
    k
}

fn section(u: f64) -> DVec2 {
    let mut p = SECTION;
    for k in 1..6 {
        for i in 0..6 - k {
            p[i] = p[i].lerp(p[i + 1], u);
        }
    }
    p[0]
}

/// Section parameter where the ray from the origin through `(a, b)` meets
/// the section.
fn ray_param(a: f64, b: f64) -> f64 {
    let (mut lo, mut hi) = (0.0, 1.0);
    for _ in 0..60 {
        let m = 0.5 * (lo + hi);
        let q = section(m);
        if q.x * b - q.y * a > 0.0 {
            lo = m;
        } else {
            hi = m;
        }
    }
    0.5 * (lo + hi)
}

fn n2(a: f64, b: f64) -> f64 {
    let l = (a * a + b * b).sqrt();
    if l < 1e-300 {
        return 0.0;
    }
    l / section(ray_param(a, b)).length()
}

fn n3(a: f64, b: f64, c: f64) -> f64 {
    (n2(n2(a, b), c) + n2(n2(b, c), a) + n2(n2(c, a), b)) / 3.0
}

/// Parameter of the section half from `(1, 0)` to the middle at which the
/// ray through `(1, s)` meets it, for `s` in `[0, 1]`.
fn half_param(s: f64) -> f64 {
    2.0 * ray_param(1.0, s)
}

/// Quintic Bézier half of the section from `(1, 0)` to the middle, mapped
/// to world points `k + r (a e1 + b e2)`, with the patch knots inserted.
fn half_curve(k: DVec3, r: f64, e1: DVec3, e2: DVec3) -> NurbsCurve {
    let mut p = SECTION;
    let mut left = [DVec2::ZERO; 6];
    for (j, slot) in left.iter_mut().enumerate() {
        *slot = p[0];
        for i in 0..5 - j {
            p[i] = p[i].lerp(p[i + 1], 0.5);
        }
    }
    let ctrl: Vec<DVec4> = left.iter().map(|q| (k + r * (q.x * e1 + q.y * e2)).extend(1.0)).collect();
    let mut c = NurbsCurve::new(DEGREE, [vec![0.0; 6], vec![1.0; 6]].concat(), ctrl);
    for u in [0.25, 0.5, 0.75] {
        c = c.insert_knot(u, 1);
    }
    c
}

/// A control point as an affine function of the fit's unknowns.
#[derive(Clone)]
struct Affine {
    base: DVec3,
    terms: Vec<(usize, DVec3)>,
}

impl Affine {
    fn fixed(p: DVec3) -> Self {
        Self { base: p, terms: vec![] }
    }
}

/// Least-squares fit of affine control points to `(param, point)` samples
/// of a tensor-product (or, with `nv == 1`, curve) B-spline.
fn fit(cps: &[Affine], nu: usize, nv: usize, unknowns: usize, samples: &[(DVec2, DVec3)]) -> Vec<DVec3> {
    let kn = knots();
    let mut ata = vec![vec![0.0; unknowns]; unknowns];
    let mut atb = vec![0.0; unknowns];
    for (uv, p) in samples {
        let su = find_span(nu - 1, DEGREE, uv.x, &kn);
        let bu = basis_funs(su, uv.x, DEGREE, &kn);
        let (sv, bv) = if nv == 1 {
            (0, vec![1.0])
        } else {
            let s = find_span(nv - 1, DEGREE, uv.y, &kn);
            (s, basis_funs(s, uv.y, DEGREE, &kn))
        };
        let mut base = DVec3::ZERO;
        let mut row: Vec<(usize, DVec3)> = Vec::new();
        for (a, wa) in bu.iter().enumerate() {
            for (b, wb) in bv.iter().enumerate() {
                let (i, j) = (su - DEGREE + a, if nv == 1 { 0 } else { sv - DEGREE + b });
                let w = wa * wb;
                let cp = &cps[i * nv + j];
                base += cp.base * w;
                for (u, d) in &cp.terms {
                    row.push((*u, *d * w));
                }
            }
        }
        let res = *p - base;
        for c in 0..3 {
            let coef: Vec<(usize, f64)> = row.iter().map(|(u, d)| (*u, d[c])).filter(|x| x.1 != 0.0).collect();
            for &(u, x) in &coef {
                atb[u] += x * res[c];
                for &(w, y) in &coef {
                    ata[u][w] += x * y;
                }
            }
        }
    }
    for (i, r) in ata.iter_mut().enumerate() {
        r[i] += 1e-12;
    }
    let x = solve_dense(ata, atb);
    cps.iter().map(|cp| cp.base + cp.terms.iter().map(|(u, d)| *d * x[*u]).sum::<DVec3>()).collect()
}

/// Inner curve of the patch from the middle of the section between faces
/// `i` and `j` to the patch centre; it leaves the section along `nk`.
fn inner_curve(point: &dyn Fn(f64, f64, f64) -> DVec3, nk: DVec3) -> NurbsCurve {
    let m = point(1.0, 1.0, 0.0);
    let c = point(1.0, 1.0, 1.0);
    let mut cps = vec![Affine::fixed(m)];
    cps.push(Affine { base: m, terms: vec![(0, nk)] });
    cps.push(Affine { base: m, terms: vec![(1, nk)] });
    let mut n = 2;
    for _ in 3..NCTRL - 1 {
        cps.push(Affine { base: DVec3::ZERO, terms: vec![(n, DVec3::X), (n + 1, DVec3::Y), (n + 2, DVec3::Z)] });
        n += 3;
    }
    cps.push(Affine::fixed(c));
    let samples: Vec<(DVec2, DVec3)> = (0..=60).map(|i| i as f64 / 60.0).map(|t| (DVec2::new(half_param(t), 0.0), point(1.0, 1.0, t))).collect();
    let pts = fit(&cps, NCTRL, 1, n, &samples);
    NurbsCurve::new(DEGREE, knots(), pts.iter().map(|p| p.extend(1.0)).collect())
}

/// Quadrilateral of the patch next to face `i`'s contact point: `u` runs
/// toward face `j`'s side, `v` toward face `k`'s. `side_j`/`side_k` are the
/// section halves from the contact point (with knots matching the patch)
/// and `inner_j`/`inner_k` the inner curves from those halves' ends.
fn quad(point: &dyn Fn(f64, f64) -> DVec3, side_j: &NurbsCurve, side_k: &NurbsCurve, inner_j: &NurbsCurve, inner_k: &NurbsCurve, nj: DVec3, nk: DVec3) -> NurbsSurface {
    let cp = |c: &NurbsCurve, i: usize| c.ctrl[i].truncate();
    let mut cps: Vec<Affine> = Vec::with_capacity(NCTRL * NCTRL);
    let mut n = 0;
    let last = NCTRL - 1;
    for a in 0..NCTRL {
        for b in 0..NCTRL {
            let p = if b == 0 {
                Affine::fixed(cp(side_j, a))
            } else if a == 0 {
                Affine::fixed(cp(side_k, b))
            } else if a == last {
                Affine::fixed(cp(inner_j, b))
            } else if b == last {
                Affine::fixed(cp(inner_k, a))
            } else if a <= 2 && b <= 2 {
                Affine::fixed(cp(side_j, a) + cp(side_k, b) - cp(side_j, 0))
            } else if b <= 2 {
                n += 1;
                Affine { base: cp(side_j, a), terms: vec![(n - 1, nk)] }
            } else if a <= 2 {
                n += 1;
                Affine { base: cp(side_k, b), terms: vec![(n - 1, nj)] }
            } else {
                n += 3;
                Affine { base: DVec3::ZERO, terms: vec![(n - 3, DVec3::X), (n - 2, DVec3::Y), (n - 1, DVec3::Z)] }
            };
            cps.push(p);
        }
    }
    let m = 24;
    let mut samples = Vec::with_capacity((m + 1) * (m + 1));
    for i in 0..=m {
        for j in 0..=m {
            let (s, t) = (i as f64 / m as f64, j as f64 / m as f64);
            samples.push((DVec2::new(half_param(s), half_param(t)), point(s, t)));
        }
    }
    let pts = fit(&cps, NCTRL, NCTRL, n, &samples);
    NurbsSurface::new(DEGREE, DEGREE, knots(), knots(), NCTRL, NCTRL, pts.iter().map(|p| p.extend(1.0)).collect())
}

/// Coedges along a closed chain of vertices joined by the given edges.
pub(crate) fn chain(b: &Body, verts: &[VertexId], edges: &[EdgeId]) -> Vec<Coedge> {
    verts.iter().zip(edges).map(|(v, e)| Coedge::new(*e, b.edge(*e).v0 != *v)).collect()
}

/// Newell normal of a loop through the given points.
fn newell(pts: &[DVec3]) -> DVec3 {
    let mut n = DVec3::ZERO;
    for i in 0..pts.len() {
        let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
        n += DVec3::new((p.y - q.y) * (p.z + q.z), (p.z - q.z) * (p.x + q.x), (p.x - q.x) * (p.y + q.y));
    }
    n
}

/// Adds a face, reversing the loop if it does not run counter-clockwise
/// about the outward normal `out`.
pub(crate) fn add_face(b: &mut Body, surface: Surface, reversed: bool, mut lp: Vec<Coedge>, out: DVec3) {
    let mut pts = Vec::new();
    for c in &lp {
        let e = b.edge(c.edge);
        let mut s: Vec<DVec3> = (0..8).map(|k| e.curve.point(e.t0 + (e.t1 - e.t0) * k as f64 / 8.0)).collect();
        if c.reversed {
            s = (1..=8).map(|k| e.curve.point(e.t1 - (e.t1 - e.t0) * (k - 1) as f64 / 8.0)).collect();
        }
        pts.extend(s);
    }
    if newell(&pts).dot(out) < 0.0 {
        lp.reverse();
        for c in lp.iter_mut() {
            c.reversed = !c.reversed;
        }
    }
    b.add_face(surface, reversed, vec![lp]);
}

/// Exact tool for a smooth blend along a straight edge between two planes:
/// the region between the edge and the blend, given by its section (the
/// corner point `t0`, contacts `a0`, `c0` and the section's homogeneous
/// control points `ctrl`, in a plane normal to the edge direction `dir`),
/// carried along the edge and cut by the end planes `(point, normal)`.
/// Sections on oblique end planes are the normal section projected along
/// the edge, so the blend face is exactly ruled between them.
pub fn smooth_edge_tool(t0: DVec3, a0: DVec3, c0: DVec3, ctrl: &[DVec4], dir: DVec3, ends: [(DVec3, DVec3); 2]) -> Option<Body> {
    let proj = |x: DVec3, (o, n): (DVec3, DVec3)| -> Option<DVec3> {
        let d = dir.dot(n);
        (d.abs() > 1e-9).then(|| x + dir * ((o - x).dot(n) / d))
    };
    let mut b = Body::new(BodyKind::Solid);
    let mut tv = Vec::new();
    let mut av = Vec::new();
    let mut cv = Vec::new();
    let mut curves = Vec::new();
    for end in ends {
        tv.push(b.add_vertex(proj(t0, end)?));
        av.push(b.add_vertex(proj(a0, end)?));
        cv.push(b.add_vertex(proj(c0, end)?));
        let cp: Vec<DVec4> = ctrl.iter().map(|h| {
            let w = h.w;
            proj(h.truncate() / w, end).map(|p| (p * w).extend(w))
        }).collect::<Option<_>>()?;
        curves.push(NurbsCurve::new(ctrl.len() - 1, [vec![0.0; ctrl.len()], vec![1.0; ctrl.len()]].concat(), cp));
    }
    let line = |b: &mut Body, x: VertexId, y: VertexId| {
        let (p, q) = (b.vertex(x), b.vertex(y));
        b.add_edge(Curve3::line_through(p, q), 0.0, p.distance(q), x, y)
    };
    let tt = line(&mut b, tv[0], tv[1]);
    let aa = line(&mut b, av[0], av[1]);
    let cc = line(&mut b, cv[0], cv[1]);
    let ta: Vec<EdgeId> = (0..2).map(|i| line(&mut b, tv[i], av[i])).collect();
    let tc: Vec<EdgeId> = (0..2).map(|i| line(&mut b, tv[i], cv[i])).collect();
    let q: Vec<EdgeId> = (0..2).map(|i| b.add_edge(Curve3::Nurbs(curves[i].clone()), 0.0, 1.0, av[i], cv[i])).collect();
    let centre = [tv[0], av[0], cv[0], tv[1], av[1], cv[1]].iter().map(|v| b.vertex(*v)).sum::<DVec3>() / 6.0;
    let plane = |b: &mut Body, verts: &[VertexId], edges: &[EdgeId]| {
        let pts: Vec<DVec3> = verts.iter().map(|v| b.vertex(*v)).collect();
        let mut n = newell(&pts).normalize();
        if (pts[0] - centre).dot(n) < 0.0 {
            n = -n;
        }
        let lp = chain(b, verts, edges);
        let x = (pts[1] - pts[0]).normalize();
        add_face(b, Surface::Plane { frame: Frame::new(pts[0], n, x) }, false, lp, n);
    };
    plane(&mut b, &[tv[0], av[0], av[1], tv[1]], &[ta[0], aa, ta[1], tt]);
    plane(&mut b, &[tv[0], cv[0], cv[1], tv[1]], &[tc[0], cc, tc[1], tt]);
    for i in 0..2 {
        let (o, n) = ends[i];
        let n = if (o - centre).dot(n) < 0.0 { -n } else { n };
        let lp = chain(&b, &[tv[i], av[i], cv[i]], &[ta[i], q[i], tc[i]]);
        let pts = [b.vertex(tv[i]), b.vertex(av[i])];
        add_face(&mut b, Surface::Plane { frame: Frame::new(pts[0], n, (pts[1] - pts[0]).normalize()) }, false, lp, n);
    }
    // The blend face, ruled along the edge between the two sections.
    let surf = NurbsSurface::ruled(&curves[0], &curves[1]);
    let mid = surf.point(0.5, 0.5);
    let d = surf.derivs(0.5, 0.5, 1);
    let out = (mid - 0.5 * (b.vertex(tv[0]) + b.vertex(tv[1]))).normalize();
    let out = out - dir * out.dot(dir);
    let lp = chain(&b, &[av[0], cv[0], cv[1], av[1]], &[q[0], cc, q[1], aa]);
    add_face(&mut b, Surface::Nurbs(surf), d[1][0].cross(d[0][1]).dot(out) < 0.0, lp, out);
    Some(b)
}

/// The solid to remove at a convex vertex `v` whose three faces have the
/// mutually perpendicular outward normals `ns`, when all three edges get
/// smooth blends of radius `r`: the corner cell between `v` and the plane
/// at distance `r` from each face, less the material the patch keeps.
pub fn smooth_corner_tool(v: DVec3, ns: [DVec3; 3], r: f64) -> Option<Body> {
    for i in 0..3 {
        if ns[i].dot(ns[(i + 1) % 3]).abs() > 1e-9 {
            return None;
        }
    }
    let k = v - r * (ns[0] + ns[1] + ns[2]);
    // Corner point for weights along the three normals.
    let point = |w: [f64; 3]| k + r * (w[0] * ns[0] + w[1] * ns[1] + w[2] * ns[2]) / n3(w[0], w[1], w[2]);
    let mut b = Body::new(BodyKind::Solid);
    let vv = b.add_vertex(v);
    let q: Vec<VertexId> = (0..3).map(|i| b.add_vertex(v - r * ns[i])).collect();
    let a: Vec<VertexId> = (0..3).map(|i| b.add_vertex(k + r * ns[i])).collect();
    // Middles of the sections, indexed by the face pair's third face.
    let m: Vec<VertexId> = (0..3).map(|kk| {
        let (i, j) = ((kk + 1) % 3, (kk + 2) % 3);
        let mut w = [0.0; 3];
        w[i] = 1.0;
        w[j] = 1.0;
        b.add_vertex(point(w))
    }).collect();
    let cc = b.add_vertex(point([1.0; 3]));
    let line = |b: &mut Body, x: VertexId, y: VertexId| {
        let (p, q) = (b.vertex(x), b.vertex(y));
        b.add_edge(Curve3::line_through(p, q), 0.0, p.distance(q), x, y)
    };
    let vq: Vec<EdgeId> = (0..3).map(|i| line(&mut b, vv, q[i])).collect();
    // Line from q[j] to a[i] on face i (i != j).
    let mut qa = [[None; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            if i != j {
                qa[j][i] = Some(line(&mut b, q[j], a[i]));
            }
        }
    }
    let qa = |j: usize, i: usize| qa[j][i].unwrap();
    // Section halves from a[i] toward the middle shared with face j, as
    // curves and edges.
    let mut halves = [[None, None, None], [None, None, None], [None, None, None]];
    let mut half_edges = [[None; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            if i == j {
                continue;
            }
            let c = half_curve(k, r, ns[i], ns[j]);
            let kk = 3 - i - j;
            half_edges[i][j] = Some(b.add_edge(Curve3::Nurbs(c.clone()), 0.0, 1.0, a[i], m[kk]));
            halves[i][j] = Some(c);
        }
    }
    let half = |i: usize, j: usize| halves[i][j].clone().unwrap();
    let half_edge = |i: usize, j: usize| half_edges[i][j].unwrap();
    // Inner curves from each middle to the centre.
    let mut inners: Vec<NurbsCurve> = Vec::new();
    let mut inner_edges: Vec<EdgeId> = Vec::new();
    for kk in 0..3 {
        let (i, j) = ((kk + 1) % 3, (kk + 2) % 3);
        let f = |x: f64, y: f64, z: f64| {
            let mut w = [0.0; 3];
            w[i] = x;
            w[j] = y;
            w[kk] = z;
            point(w)
        };
        let c = inner_curve(&f, ns[kk]);
        inner_edges.push(b.add_edge(Curve3::Nurbs(c.clone()), 0.0, 1.0, m[kk], cc));
        inners.push(c);
    }
    // Faces on the solid's three faces.
    for i in 0..3 {
        let (j, kk) = ((i + 1) % 3, (i + 2) % 3);
        let lp = chain(&b, &[vv, q[j], a[i], q[kk]], &[vq[j], qa(j, i), qa(kk, i), vq[kk]]);
        let x = (b.vertex(q[j]) - v).normalize();
        add_face(&mut b, Surface::Plane { frame: Frame::new(v, ns[i], x) }, false, lp, ns[i]);
    }
    // Cell faces, each holding one section.
    for kk in 0..3 {
        let (i, j) = ((kk + 1) % 3, (kk + 2) % 3);
        let lp = chain(&b, &[q[kk], a[i], m[kk], a[j]], &[qa(kk, i), half_edge(i, j), half_edge(j, i), qa(kk, j)]);
        let o = b.vertex(q[kk]);
        let x = (b.vertex(a[i]) - o).normalize();
        add_face(&mut b, Surface::Plane { frame: Frame::new(o, -ns[kk], x) }, false, lp, -ns[kk]);
    }
    // The patch, in three quadrilaterals.
    for i in 0..3 {
        let (j, kk) = ((i + 1) % 3, (i + 2) % 3);
        let f = |s: f64, t: f64| {
            let mut w = [0.0; 3];
            w[i] = 1.0;
            w[j] = s;
            w[kk] = t;
            point(w)
        };
        let surf = quad(&f, &half(i, j), &half(i, kk), &inners[kk], &inners[j], ns[j], ns[kk]);
        // Outward from the tool is toward the ball centre side.
        let mid = surf.point(0.5, 0.5);
        let d = surf.derivs(0.5, 0.5, 1);
        let sn = d[1][0].cross(d[0][1]);
        let out = (k - mid).normalize();
        let lp = chain(&b, &[a[i], m[kk], cc, m[j]], &[half_edge(i, j), inner_edges[kk], inner_edges[j], half_edge(i, kk)]);
        add_face(&mut b, Surface::Nurbs(surf), sn.dot(out) < 0.0, lp, out);
    }
    Some(b)
}

/// Fraction of the corner cell outside the corner surface, by integrating
/// the analytic corner: for each `(a, b)`, the surface's `c`.
#[cfg(test)]
pub(crate) fn removed_fraction(n: usize) -> f64 {
    let mut inside = 0.0;
    for i in 0..n {
        for j in 0..n {
            let (x, y) = ((i as f64 + 0.5) / n as f64, (j as f64 + 0.5) / n as f64);
            if n3(x, y, 0.0) > 1.0 {
                continue;
            }
            let (mut lo, mut hi) = (0.0, 1.0);
            for _ in 0..40 {
                let mid = 0.5 * (lo + hi);
                if n3(x, y, mid) < 1.0 {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            inside += lo / (n * n) as f64;
        }
    }
    1.0 - inside
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ops::boolean::open_edges;
    use crate::tess::{tessellate, TessOptions};

    #[test]
    fn section_norm_matches_the_section() {
        for u in [0.0, 0.1, 0.37, 0.5, 0.8, 1.0] {
            let p = section(u);
            assert!((n2(p.x, p.y) - 1.0).abs() < 1e-12);
            assert!((n3(p.x, p.y, 0.0) - 1.0).abs() < 1e-12);
            assert!((n3(0.0, p.x, p.y) - 1.0).abs() < 1e-12);
        }
        // Flat to third order off a side.
        let d = n3(0.6, 0.6, 1e-3) - n3(0.6, 0.6, 0.0);
        assert!(d.abs() < 1e-7, "{d}");
    }

    #[test]
    fn corner_tool_is_closed_and_fits() {
        let ns = [DVec3::X, DVec3::Y, DVec3::Z];
        let r = 0.5;
        let t = smooth_corner_tool(DVec3::new(4.0, 3.0, 2.0), ns, r).unwrap();
        assert_eq!(open_edges(&t), 0);
        let vol = tessellate(&t, &TessOptions { chord_tol: 1e-5, angle_tol: 0.05 }).volume();
        let expect = r * r * r * removed_fraction(80);
        assert!(vol > 0.0 && (vol - expect).abs() < 1e-3 * r * r * r, "{vol} vs {expect}");
        // The patch passes through the corner surface.
        for f in t.face_ids() {
            if let Surface::Nurbs(s) = &t.face(f).surface {
                for (u, v) in [(0.3, 0.3), (0.7, 0.2), (0.5, 0.9)] {
                    let p = s.point(u, v);
                    let w = (p - (DVec3::new(4.0, 3.0, 2.0) - DVec3::splat(r))) / r;
                    assert!((n3(w.x, w.y, w.z) - 1.0).abs() < 1e-4, "{}", n3(w.x, w.y, w.z));
                }
            }
        }
    }
}
