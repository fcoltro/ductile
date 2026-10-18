// SPDX-License-Identifier: GPL-3.0-only
//! Curvature-continuous corner patches where three smooth (G2) blends of
//! plane/plane edges meet at a convex vertex.
//!
//! Each edge blend's section is an affine image of one normalized quintic
//! from `(1, 0)` to `(0, 1)` whose end control points lie on the faces'
//! tangents (for perpendicular faces, in coordinates running from the
//! rolling-ball centre toward the two faces in units of the radius). Read as a "norm" `N2` (the section is `N2(a, b) = 1`,
//! with `N2` homogeneous of degree one), it extends to three directions by
//! composition, `N2(N2(a, b), c)`, averaged over the three ways round. On a
//! side where one coordinate vanishes this is exactly the edge's section,
//! and since the section leaves the faces with zero curvature, `N2(1, c)`
//! differs from one only at third order in `c`: the corner meets each edge
//! blend with matching curvature.
//!
//! Between faces at other angles the three sides' affine maps no longer
//! agree, and the corner surface blends them (`CornerMap`).
//!
//! The patch is split into three quadrilaterals around its centre, quintic
//! B-splines fitted together by least squares (`patch3`): outer sides fixed
//! to halves of the edge sections, curvature-continuous joins to the edge
//! blends, shared tangent planes along the inner curves, and thin-plate
//! fairing where the sides' maps disagree.

use crate::topo::{Body, BodyKind, Coedge, EdgeId, VertexId};
use ductile_geom::nurbs::basis::{ders_basis_funs, find_span};
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
/// to world points by the affine `map`, with the patch knots inserted.
fn half_curve(map: impl Fn(DVec2) -> DVec3) -> NurbsCurve {
    let mut p = SECTION;
    let mut left = [DVec2::ZERO; 6];
    for (j, slot) in left.iter_mut().enumerate() {
        *slot = p[0];
        for i in 0..5 - j {
            p[i] = p[i].lerp(p[i + 1], 0.5);
        }
    }
    let ctrl: Vec<DVec4> = left.iter().map(|q| map(*q).extend(1.0)).collect();
    let mut c = NurbsCurve::new(DEGREE, [vec![0.0; 6], vec![1.0; 6]].concat(), ctrl);
    for u in [0.25, 0.5, 0.75] {
        c = c.insert_knot(u, 1);
    }
    c
}

/// Map from corner coordinates (one per face, in units of the section) to
/// world points. Side `k` (where face `k`'s coordinate vanishes) is the
/// section of the edge blend between the other two faces `i`, `j`: the
/// affine image `o + x_i a + x_j b` of the normalized section, whose
/// corner `(1, 1)` is where the faces' tangent lines meet on the edge.
/// Off a side the map moves along that edge by `x_k lam` and slides along
/// the section by `x_k mu`, which keeps it on the edge blend to second
/// order; `lam` and `mu` vary along the section so that at each contact
/// point the move matches the neighbouring side's section. The three
/// sides' maps are blended with weights vanishing to third order off their
/// sides, so the corner meets each edge blend with matching curvature.
struct CornerMap {
    o: [DVec3; 3],
    a: [DVec3; 3],
    b: [DVec3; 3],
    e: [DVec3; 3],
    /// (lam, mu) at the section's start (face `i`) and end (face `j`).
    ends: [[(f64, f64); 2]; 3],
}

impl CornerMap {
    /// `p[i]`: the ball's contact with face `i`; `x[k]`: where side `k`'s
    /// tangent lines meet; `e[k]`: unit direction of the edge between the
    /// faces other than `k`.
    fn new(p: [DVec3; 3], x: [DVec3; 3], e: [DVec3; 3]) -> Self {
        let mut m = CornerMap { o: [DVec3::ZERO; 3], a: [DVec3::ZERO; 3], b: [DVec3::ZERO; 3], e, ends: [[(0.0, 0.0); 2]; 3] };
        for k in 0..3 {
            let (i, j) = ((k + 1) % 3, (k + 2) % 3);
            m.o[k] = p[i] + p[j] - x[k];
            m.a[k] = x[k] - p[j];
            m.b[k] = x[k] - p[i];
        }
        // Solves `want = mu t + lam e` in the face plane they share.
        let split = |want: DVec3, t: DVec3, e: DVec3| {
            let (tt, te, ee) = (t.dot(t), t.dot(e), e.dot(e));
            let det = tt * ee - te * te;
            let (wt, we) = (want.dot(t), want.dot(e));
            ((tt * we - te * wt) / det, (ee * wt - te * we) / det)
        };
        for k in 0..3 {
            let (i, j) = ((k + 1) % 3, (k + 2) % 3);
            // Section derivative at its ends: (0, 1.5) and (-1.5, 0).
            m.ends[k][0] = split(x[j] - p[i], 1.5 * m.b[k], e[k]);
            m.ends[k][1] = split(x[i] - p[j], -1.5 * m.a[k], e[k]);
        }
        m
    }

    fn side(&self, k: usize, x: [f64; 3]) -> DVec3 {
        let (i, j) = ((k + 1) % 3, (k + 2) % 3);
        let (qa, qb) = (x[i], x[j]);
        let scale = (qa * qa + qb * qb).sqrt();
        let [(l0, m0), (l1, m1)] = self.ends[k];
        if scale < 1e-12 {
            return self.o[k] + x[k] * 0.5 * (l0 + l1) * self.e[k];
        }
        let scale = n2(qa, qb);
        let u = ray_param(qa, qb);
        let lam = l0 + (l1 - l0) * u;
        let mu = m0 * (1.0 - u) * (1.0 - 2.0 * u) + m1 * u * (2.0 * u - 1.0);
        let q = section(u + x[k] * mu) * scale;
        self.o[k] + q.x * self.a[k] + q.y * self.b[k] + x[k] * lam * self.e[k]
    }

    fn map(&self, x: [f64; 3]) -> DVec3 {
        let w: Vec<f64> = (0..3).map(|k| (x[(k + 1) % 3] * x[(k + 2) % 3]).powi(3)).collect();
        let total: f64 = w.iter().sum();
        if total < 1e-300 {
            let k = (0..3).min_by(|&a, &b| x[a].total_cmp(&x[b])).unwrap();
            return self.side(k, x);
        }
        (0..3).map(|k| self.side(k, x) * (w[k] / total)).sum()
    }
}

/// Least-squares system of scalar equations over unknown coordinates.
struct Lsq {
    ata: Vec<Vec<f64>>,
    atb: Vec<f64>,
}

impl Lsq {
    fn new(n: usize) -> Self {
        Self { ata: vec![vec![0.0; n]; n], atb: vec![0.0; n] }
    }

    fn row(&mut self, coef: &[(usize, f64)], rhs: f64, w: f64) {
        let w2 = w * w;
        for &(u, x) in coef {
            self.atb[u] += w2 * x * rhs;
            for &(v, y) in coef {
                self.ata[u][v] += w2 * x * y;
            }
        }
    }

    fn solve(mut self) -> Vec<f64> {
        for (i, r) in self.ata.iter_mut().enumerate() {
            r[i] += 1e-12;
        }
        solve_dense(self.ata, self.atb)
    }
}

#[derive(Clone, Copy)]
enum Slot {
    Fixed(DVec3),
    Var(usize),
}

/// The patch's three quadrilaterals, fitted together. Quadrilateral `i`
/// lies next to face `i`'s contact point, with `u` running along the
/// section half toward face `j = i + 1` and `v` along the half toward face
/// `k = i + 2`; `halves[i][j]` is the half from face `i`'s contact toward
/// the middle it shares with face `j` (with the patch's knots), and `e[k]`
/// the direction of the edge blend along side `k`. Inner curve `c` runs
/// from the middle of side `c` to the patch centre, between quadrilaterals
/// `c + 1` (as its `u = 1` side) and `c + 2` (its `v = 1` side).
///
/// The fit holds the joins to the edge blends curvature continuous (the
/// first derivative across each side in the blend's tangent plane, the
/// second giving its normal curvature), makes neighbouring quadrilaterals
/// meet with continuous first and second derivatives along the inner
/// curves (and so at the centre), and otherwise follows the corner surface
/// `point` with thin-plate fairness of weight `fair`, which evens out
/// where that surface's three sides disagree.
fn patch3(point: &dyn Fn([f64; 3]) -> DVec3, halves: &[[Option<NurbsCurve>; 3]; 3], e: [DVec3; 3], fair: f64) -> ([NurbsSurface; 3], [NurbsCurve; 3]) {
    let last = NCTRL - 1;
    let half = |i: usize, j: usize| halves[i][j].as_ref().unwrap();
    let mut n = 0;
    let mut var = || {
        n += 3;
        Slot::Var(n - 3)
    };
    let centre = var();
    let inner: Vec<Vec<Slot>> = (0..3)
        .map(|c| {
            let m = half((c + 1) % 3, (c + 2) % 3).ctrl[last].truncate();
            (0..NCTRL).map(|t| if t == 0 { Slot::Fixed(m) } else if t == last { centre } else { var() }).collect()
        })
        .collect();
    let mut slots = vec![vec![Slot::Fixed(DVec3::ZERO); NCTRL * NCTRL]; 3];
    for (i, qs) in slots.iter_mut().enumerate() {
        let (j, k) = ((i + 1) % 3, (i + 2) % 3);
        for a in 0..NCTRL {
            for b in 0..NCTRL {
                qs[a * NCTRL + b] = if b == 0 {
                    Slot::Fixed(half(i, j).ctrl[a].truncate())
                } else if a == 0 {
                    Slot::Fixed(half(i, k).ctrl[b].truncate())
                } else if a == last {
                    inner[k][b]
                } else if b == last {
                    inner[j][a]
                } else {
                    var()
                };
            }
        }
    }
    let mut sys = Lsq::new(n);
    // Adds `g . sum(coef * point) = rhs`.
    let combo = |sys: &mut Lsq, terms: &[(Slot, f64)], g: DVec3, rhs: f64, w: f64| {
        let mut rhs = rhs;
        let mut coef = Vec::new();
        for &(slot, c) in terms {
            match slot {
                Slot::Fixed(p) => rhs -= c * g.dot(p),
                Slot::Var(x) => {
                    for (axis, gc) in g.to_array().into_iter().enumerate() {
                        if gc != 0.0 {
                            coef.push((x + axis, c * gc));
                        }
                    }
                }
            }
        }
        sys.row(&coef, rhs, w);
    };
    let kn = knots();
    let basis = |t: f64, d: usize| {
        let s = find_span(NCTRL - 1, DEGREE, t, &kn);
        (s - DEGREE, ders_basis_funs(s, t, DEGREE, d, &kn).swap_remove(d))
    };
    // Adds `g . d^(du+dv) P_i / du^du dv^dv (u, v) = rhs`.
    let equation = |sys: &mut Lsq, i: usize, (u, v): (f64, f64), (du, dv): (usize, usize), g: DVec3, rhs: f64, w: f64| {
        let (su, bu) = basis(u, du);
        let (sv, bv) = basis(v, dv);
        let mut terms = Vec::with_capacity(36);
        for (x, wa) in bu.iter().enumerate() {
            for (y, wb) in bv.iter().enumerate() {
                if wa * wb != 0.0 {
                    terms.push((slots[i][(su + x) * NCTRL + sv + y], wa * wb));
                }
            }
        }
        combo(sys, &terms, g, rhs, w);
    };
    let axes = [DVec3::X, DVec3::Y, DVec3::Z];
    let weight = 1e3;
    for i in 0..3 {
        let (j, k) = ((i + 1) % 3, (i + 2) % 3);
        let f = |s: f64, t: f64| {
            let mut w = [0.0; 3];
            w[i] = 1.0;
            w[j] = s;
            w[k] = t;
            point(w)
        };
        let m = 32;
        for a in 0..=m {
            for b in 0..=m {
                let (s, t) = (a as f64 / m as f64, b as f64 / m as f64);
                let p = f(s, t);
                for g in axes {
                    equation(&mut sys, i, (half_param(s), half_param(t)), (0, 0), g, g.dot(p), 1.0);
                }
            }
        }
        let m = 12;
        for a in 0..=m {
            for b in 0..=m {
                let uv = (a as f64 / m as f64, b as f64 / m as f64);
                for g in axes {
                    equation(&mut sys, i, uv, (2, 0), g, 0.0, fair);
                    equation(&mut sys, i, uv, (1, 1), g, 0.0, fair * std::f64::consts::SQRT_2);
                    equation(&mut sys, i, uv, (0, 2), g, 0.0, fair);
                }
            }
        }
        // Joins to the edge blends: `side` runs along the patch parameter
        // `along`, `other` leaves its start across it, `dir` is the
        // extrusion. The derivative's part along the side follows the
        // other side's tangent at the contact point and fades to zero at
        // the middle, where the inner curves leave along the edges.
        for (side, other, dir, along_u) in [(half(i, j), half(i, k), e[k], true), (half(i, k), half(i, j), e[j], false)] {
            let d0 = side.derivs(0.0, 1)[1];
            let mu0 = other.derivs(0.0, 1)[1].dot(d0) / d0.length_squared();
            for a in 0..=40 {
                let t = a as f64 / 40.0;
                let d = side.derivs(t, 2);
                let Some(nrm) = d[1].cross(dir).try_normalize() else { continue };
                let mu = mu0 * (1.0 - t);
                let (at, first, second) = if along_u { ((t, 0.0), (0, 1), (0, 2)) } else { ((0.0, t), (1, 0), (2, 0)) };
                equation(&mut sys, i, at, first, nrm, 0.0, weight);
                equation(&mut sys, i, at, first, d[1].normalize(), mu * d[1].length(), weight);
                equation(&mut sys, i, at, second, nrm, mu * mu * nrm.dot(d[2]), weight);
            }
        }
    }
    // Neighbours share their tangent plane along each inner curve: the
    // corner surface's, turned to hold the curve's direction there, which
    // the curve and both quadrilaterals' derivatives across it keep to.
    let h = 1e-4;
    for c in 0..3 {
        let (qa, qb) = ((c + 1) % 3, (c + 2) % 3);
        let f = |s: f64, t: f64| {
            let mut w = [0.0; 3];
            w[(c + 1) % 3] = 1.0;
            w[(c + 2) % 3] = s;
            w[c] = t;
            point(w)
        };
        for a in 1..=40 {
            let t = a as f64 / 40.0;
            let (t0, t1) = ((t - h).max(0.0), (t + h).min(1.0));
            let across = (f(1.0, t) - f(1.0 - h, t)) / h;
            let along = (f(1.0, t1) - f(1.0, t0)) / (t1 - t0);
            let Some(nrm) = across.cross(along).try_normalize() else { continue };
            let tan = along.normalize();
            let Some(nrm) = (nrm - tan * nrm.dot(tan)).try_normalize() else { continue };
            let tau = half_param(t);
            let (sp, bs) = basis(tau, 1);
            let terms: Vec<(Slot, f64)> = bs.iter().enumerate().map(|(x, w)| (inner[c][sp + x], *w)).collect();
            combo(&mut sys, &terms, nrm, 0.0, weight);
            equation(&mut sys, qa, (1.0, tau), (1, 0), nrm, 0.0, weight);
            equation(&mut sys, qb, (tau, 1.0), (0, 1), nrm, 0.0, weight);
        }
    }
    let x = sys.solve();
    let at = |s: Slot| match s {
        Slot::Fixed(p) => p,
        Slot::Var(i) => DVec3::new(x[i], x[i + 1], x[i + 2]),
    };
    let quads = std::array::from_fn(|i| NurbsSurface::new(DEGREE, DEGREE, knots(), knots(), NCTRL, NCTRL, slots[i].iter().map(|s| at(*s).extend(1.0)).collect()));
    let inners = std::array::from_fn(|c| NurbsCurve::new(DEGREE, knots(), inner[c].iter().map(|s| at(*s).extend(1.0)).collect()));
    (quads, inners)
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

/// The solid to remove at a convex vertex `v` of three planar faces with
/// outward normals `ns`, when all three edges get smooth blends of radius
/// `r`; `ds[k]` is the unit direction from `v` along the edge between the
/// faces other than `k`. The tool is the corner cell between `v`, the
/// faces and the planes through the ball centre normal to the edges (where
/// the edge blends end), less the material the patch keeps.
pub fn smooth_corner_tool(v: DVec3, ns: [DVec3; 3], ds: [DVec3; 3], r: f64) -> Option<Body> {
    let m = ductile_geom::DMat3::from_cols(ns[0], ns[1], ns[2]).transpose();
    if m.determinant().abs() < 1e-6 {
        return None;
    }
    for kk in 0..3 {
        if ds[kk].dot(ns[(kk + 1) % 3]).abs() > 1e-6 || ds[kk].dot(ns[(kk + 2) % 3]).abs() > 1e-6 {
            return None;
        }
    }
    // Ball centre, contacts and where each edge meets its cap plane.
    let k = v + m.inverse() * DVec3::splat(-r);
    let p: [DVec3; 3] = std::array::from_fn(|i| k + r * ns[i]);
    let x: [DVec3; 3] = std::array::from_fn(|kk| v + ds[kk] * (k - v).dot(ds[kk]));
    if (0..3).any(|kk| (x[kk] - v).dot(ds[kk]) <= 1e-9 * r) {
        return None;
    }
    let cm = CornerMap::new(p, x, ds);
    let point = |w: [f64; 3]| {
        let s = n3(w[0], w[1], w[2]);
        cm.map([w[0] / s, w[1] / s, w[2] / s])
    };
    let mut b = Body::new(BodyKind::Solid);
    let vv = b.add_vertex(v);
    let q: Vec<VertexId> = (0..3).map(|i| b.add_vertex(x[i])).collect();
    let a: Vec<VertexId> = (0..3).map(|i| b.add_vertex(p[i])).collect();
    // Middles of the sections, indexed by the face pair's third face.
    let m: Vec<VertexId> = (0..3).map(|kk| {
        let (i, j) = ((kk + 1) % 3, (kk + 2) % 3);
        let mut w = [0.0; 3];
        w[i] = 1.0;
        w[j] = 1.0;
        b.add_vertex(point(w))
    }).collect();
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
            let kk = 3 - i - j;
            let c = half_curve(|q| {
                let mut w = [0.0; 3];
                w[i] = q.x;
                w[j] = q.y;
                cm.side(kk, w)
            });
            half_edges[i][j] = Some(b.add_edge(Curve3::Nurbs(c.clone()), 0.0, 1.0, a[i], m[kk]));
            halves[i][j] = Some(c);
        }
    }
    let half_edge = |i: usize, j: usize| half_edges[i][j].unwrap();
    // Fairness grows with how much the sides' maps disagree inside the
    // corner (not at all where the faces are perpendicular).
    let spread = [[1.0, 1.0, 1.0], [1.0, 0.5, 0.5], [0.5, 1.0, 0.5], [0.5, 0.5, 1.0]]
        .iter()
        .flat_map(|w| {
            let s = n3(w[0], w[1], w[2]);
            let xs = [w[0] / s, w[1] / s, w[2] / s];
            let pts: Vec<DVec3> = (0..3).map(|kk| cm.side(kk, xs)).collect();
            [pts[0].distance(pts[1]), pts[1].distance(pts[2]), pts[2].distance(pts[0])]
        })
        .fold(0.0, f64::max)
        / r;
    let (quads, inners) = patch3(&point, &halves, ds, 0.003 + 0.5 * spread);
    let cc = b.add_vertex(inners[0].point(1.0));
    let inner_edges: Vec<EdgeId> = (0..3).map(|c| b.add_edge(Curve3::Nurbs(inners[c].clone()), 0.0, 1.0, m[c], cc)).collect();
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
        add_face(&mut b, Surface::Plane { frame: Frame::new(o, ds[kk], x) }, false, lp, ds[kk]);
    }
    // The patch, in three quadrilaterals.
    for (i, surf) in quads.into_iter().enumerate() {
        let (j, kk) = ((i + 1) % 3, (i + 2) % 3);
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
        let t = smooth_corner_tool(DVec3::new(4.0, 3.0, 2.0), ns, [-DVec3::X, -DVec3::Y, -DVec3::Z], r).unwrap();
        assert_eq!(open_edges(&t), 0);
        let vol = tessellate(&t, &TessOptions { chord_tol: 1e-5, angle_tol: 0.05 }).volume();
        let expect = r * r * r * removed_fraction(80);
        assert!(vol > 0.0 && (vol - expect).abs() < 1e-3 * r * r * r, "{vol} vs {expect}");
        // The patch follows the corner surface (up to its slight fairing).
        for f in t.face_ids() {
            if let Surface::Nurbs(s) = &t.face(f).surface {
                for (u, v) in [(0.3, 0.3), (0.7, 0.2), (0.5, 0.9)] {
                    let p = s.point(u, v);
                    let w = (p - (DVec3::new(4.0, 3.0, 2.0) - DVec3::splat(r))) / r;
                    assert!((n3(w.x, w.y, w.z) - 1.0).abs() < 3e-4, "{}", n3(w.x, w.y, w.z));
                }
            }
        }
    }

    /// Least Gaussian curvature over a grid on the tool's patch, times r^2.
    fn least_gauss(t: &Body, r: f64) -> f64 {
        let mut least = f64::INFINITY;
        for f in t.face_ids() {
            if let Surface::Nurbs(s) = &t.face(f).surface {
                for i in 0..=20 {
                    for j in 0..=20 {
                        let d = s.derivs(i as f64 / 20.0, j as f64 / 20.0, 2);
                        let (su, sv) = (d[1][0], d[0][1]);
                        let n = su.cross(sv).normalize();
                        let k = (n.dot(d[2][0]) * n.dot(d[0][2]) - n.dot(d[1][1]).powi(2)) / (su.length_squared() * sv.length_squared() - su.dot(sv).powi(2));
                        least = least.min(k * r * r);
                    }
                }
            }
        }
        least
    }

    #[test]
    fn oblique_corner_is_closed_and_nearly_convex() {
        // A hexagonal prism's top corner: two side faces at 120 degrees.
        let v = DVec3::new(3.0, 0.0, 2.0);
        let a = std::f64::consts::PI / 6.0;
        let ns = [DVec3::Z, DVec3::new(a.cos(), -a.sin(), 0.0), DVec3::new(a.cos(), a.sin(), 0.0)];
        let toward_v = |d: DVec3| if d.x > 0.0 { -d } else { d };
        let ds = [-DVec3::Z, toward_v(ns[0].cross(ns[2]).normalize()), toward_v(ns[0].cross(ns[1]).normalize())];
        let r = 0.4;
        let t = smooth_corner_tool(v, ns, ds, r).unwrap();
        assert_eq!(open_edges(&t), 0);
        assert!(tessellate(&t, &TessOptions { chord_tol: 1e-5, angle_tol: 0.05 }).volume() > 0.0);
        // Faired: at most a faint saddle near the centre (a ball would give
        // 1; the unfaired fit gave about -6).
        let k = least_gauss(&t, r);
        assert!(k > -0.3, "{k}");
        let t = smooth_corner_tool(DVec3::new(4.0, 3.0, 2.0), [DVec3::X, DVec3::Y, DVec3::Z], [-DVec3::X, -DVec3::Y, -DVec3::Z], 0.5).unwrap();
        assert!(least_gauss(&t, 0.5) > -1e-3);
    }
}
