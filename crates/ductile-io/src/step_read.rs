// SPDX-License-Identifier: GPL-3.0-only
//! STEP import: B-rep solids and shells with exact geometry.

use crate::step_parse::{parse, Entity, StepFile, Value};
use ductile_brep::{Body, BodyKind, Coedge, EdgeId, VertexId};
use ductile_geom::nurbs::{NurbsCurve, NurbsSurface};
use ductile_geom::{Curve3, DVec3, DVec4, Frame, Surface};
use std::collections::HashMap;
use std::f64::consts::TAU;

/// A body read from a STEP file.
pub struct Imported {
    pub name: String,
    pub body: Body,
}

/// Result of an import: bodies plus warnings about what was skipped.
pub struct StepImport {
    pub bodies: Vec<Imported>,
    pub warnings: Vec<String>,
}

type R<T> = Result<T, String>;

struct Reader<'a> {
    f: &'a StepFile,
    /// Multiplier from file length units to millimetres.
    scale: f64,
    /// Multiplier from file plane-angle units to radians.
    angle: f64,
    /// Half-length used for unbounded curves and sweeps (millimetres).
    reach: f64,
    warnings: Vec<String>,
}

fn real_at(p: &[Value], i: usize) -> R<f64> {
    p.get(i).and_then(|v| v.as_f64()).ok_or_else(|| format!("expected a number at parameter {i}"))
}

fn ref_at(p: &[Value], i: usize) -> R<u64> {
    p.get(i).and_then(|v| v.as_ref()).ok_or_else(|| format!("expected a reference at parameter {i}"))
}

fn refs_at(p: &[Value], i: usize) -> R<Vec<u64>> {
    p.get(i).and_then(|v| v.as_list()).map(|l| l.iter().filter_map(|x| x.as_ref()).collect()).ok_or_else(|| format!("expected a list at parameter {i}"))
}

fn bool_at(p: &[Value], i: usize) -> bool {
    p.get(i).and_then(|v| v.as_bool()).unwrap_or(true)
}

fn expand_knots(mults: &[Value], knots: &[Value]) -> Vec<f64> {
    let mut out = Vec::new();
    for (m, k) in mults.iter().zip(knots) {
        let (Some(m), Some(k)) = (m.as_f64(), k.as_f64()) else { continue };
        for _ in 0..m as usize {
            out.push(k);
        }
    }
    out
}

impl<'a> Reader<'a> {
    fn get(&self, id: u64) -> R<&'a Entity> {
        self.f.entities.get(&id).ok_or_else(|| format!("missing instance #{id}"))
    }

    fn point(&self, id: u64) -> R<DVec3> {
        let e = self.get(id)?;
        match e.name() {
            "CARTESIAN_POINT" => {
                let c = e.params().get(1).and_then(|v| v.as_list()).ok_or("bad point")?;
                let x = |i: usize| c.get(i).and_then(|v| v.as_f64()).unwrap_or(0.0);
                Ok(DVec3::new(x(0), x(1), x(2)) * self.scale)
            }
            "VERTEX_POINT" => self.point(ref_at(e.params(), 1)?),
            n => Err(format!("#{id}: {n} is not a point")),
        }
    }

    fn direction(&self, id: u64) -> R<DVec3> {
        let e = self.get(id)?;
        let c = e.params().get(1).and_then(|v| v.as_list()).ok_or("bad direction")?;
        let x = |i: usize| c.get(i).and_then(|v| v.as_f64()).unwrap_or(0.0);
        DVec3::new(x(0), x(1), x(2)).try_normalize().ok_or_else(|| format!("#{id}: zero direction"))
    }

    fn placement(&self, id: u64) -> R<Frame> {
        let e = self.get(id)?;
        let p = e.params();
        let o = self.point(ref_at(p, 1)?)?;
        let z = match p.get(2).and_then(|v| v.as_ref()) {
            Some(r) => self.direction(r)?,
            None => DVec3::Z,
        };
        let x = match p.get(3).and_then(|v| v.as_ref()) {
            Some(r) => self.direction(r)?,
            None => ductile_geom::any_perpendicular(z),
        };
        // Orthogonalize the reference direction against the axis.
        let x = (x - z * x.dot(z)).try_normalize().unwrap_or_else(|| ductile_geom::any_perpendicular(z));
        Ok(Frame { origin: o, x, y: z.cross(x), z })
    }

    fn curve(&self, id: u64) -> R<Curve3> {
        let e = self.get(id)?;
        if e.is_complex() || e.name().starts_with("B_SPLINE_CURVE") || e.name() == "BEZIER_CURVE" {
            return self.bspline_curve(e).map(Curve3::Nurbs);
        }
        let p = e.params();
        match e.name() {
            "LINE" => {
                let o = self.point(ref_at(p, 1)?)?;
                let v = self.get(ref_at(p, 2)?)?;
                let d = self.direction(ref_at(v.params(), 1)?)?;
                Ok(Curve3::Line { origin: o, dir: d })
            }
            "CIRCLE" => Ok(Curve3::Circle { frame: self.placement(ref_at(p, 1)?)?, radius: real_at(p, 2)? * self.scale }),
            "ELLIPSE" => Ok(Curve3::Ellipse { frame: self.placement(ref_at(p, 1)?)?, rx: real_at(p, 2)? * self.scale, ry: real_at(p, 3)? * self.scale }),
            "SURFACE_CURVE" | "SEAM_CURVE" | "INTERSECTION_CURVE" | "TRIMMED_CURVE" => self.curve(ref_at(p, 1)?),
            "POLYLINE" => {
                let pts: Vec<DVec3> = refs_at(p, 1)?.into_iter().map(|r| self.point(r)).collect::<R<_>>()?;
                if pts.len() < 2 {
                    return Err("polyline with fewer than two points".into());
                }
                let mut knots = vec![0.0];
                let mut s = 0.0;
                for w in pts.windows(2) {
                    s += w[0].distance(w[1]);
                    knots.push(s);
                }
                let mut k = vec![0.0];
                k.extend(knots.iter().copied());
                k.push(s);
                Ok(Curve3::Nurbs(NurbsCurve::from_points(1, k, &pts)))
            }
            n => Err(format!("unsupported curve {n}")),
        }
    }

    fn bspline_curve(&self, e: &Entity) -> R<NurbsCurve> {
        // Simple entities carry the name first; complex parts do not.
        let (degree, pts, mults, knots, weights) = if let Some(b) = e.part("B_SPLINE_CURVE") {
            let k = e.part("B_SPLINE_CURVE_WITH_KNOTS");
            let w = e.part("RATIONAL_B_SPLINE_CURVE").and_then(|r| r.first()).and_then(|v| v.as_list()).map(|l| l.to_vec());
            (real_at(b, 0)? as usize, refs_at(b, 1)?, k.and_then(|k| k.first().cloned()), k.and_then(|k| k.get(1).cloned()), w)
        } else {
            let p = e.params();
            (real_at(p, 1)? as usize, refs_at(p, 2)?, p.get(6).cloned(), p.get(7).cloned(), None)
        };
        let pts: Vec<DVec3> = pts.into_iter().map(|r| self.point(r)).collect::<R<_>>()?;
        let knots = match (mults, knots) {
            (Some(Value::List(m)), Some(Value::List(k))) => expand_knots(&m, &k),
            // Bezier: clamped single span.
            _ => std::iter::repeat_n(0.0, degree + 1).chain(std::iter::repeat_n(1.0, degree + 1)).collect(),
        };
        if knots.len() != pts.len() + degree + 1 {
            return Err(format!("B-spline curve knot count {} does not match {} points of degree {degree}", knots.len(), pts.len()));
        }
        let ctrl: Vec<DVec4> = match weights {
            Some(w) => pts.iter().zip(w.iter()).map(|(p, w)| {
                let w = w.as_f64().unwrap_or(1.0);
                (*p * w).extend(w)
            }).collect(),
            None => pts.iter().map(|p| p.extend(1.0)).collect(),
        };
        Ok(NurbsCurve::new(degree, knots, ctrl))
    }

    fn surface(&self, id: u64) -> R<Surface> {
        let e = self.get(id)?;
        if e.is_complex() || e.name().starts_with("B_SPLINE_SURFACE") {
            return self.bspline_surface(e).map(Surface::Nurbs);
        }
        let p = e.params();
        let s = self.scale;
        match e.name() {
            "PLANE" => Ok(Surface::Plane { frame: self.placement(ref_at(p, 1)?)? }),
            "CYLINDRICAL_SURFACE" => Ok(Surface::Cylinder { frame: self.placement(ref_at(p, 1)?)?, radius: real_at(p, 2)? * s }),
            "CONICAL_SURFACE" => Ok(Surface::Cone { frame: self.placement(ref_at(p, 1)?)?, radius: real_at(p, 2)? * s, half_angle: real_at(p, 3)? * self.angle }),
            "SPHERICAL_SURFACE" => Ok(Surface::Sphere { frame: self.placement(ref_at(p, 1)?)?, radius: real_at(p, 2)? * s }),
            "TOROIDAL_SURFACE" | "DEGENERATE_TOROIDAL_SURFACE" => {
                Ok(Surface::Torus { frame: self.placement(ref_at(p, 1)?)?, major: real_at(p, 2)? * s, minor: real_at(p, 3)? * s })
            }
            "RECTANGULAR_TRIMMED_SURFACE" => self.surface(ref_at(p, 1)?),
            "SURFACE_OF_LINEAR_EXTRUSION" => {
                let c = self.curve_nurbs(ref_at(p, 1)?)?;
                let v = self.get(ref_at(p, 2)?)?;
                let d = self.direction(ref_at(v.params(), 1)?)?;
                Ok(Surface::Nurbs(extrusion_surface(&c, d, self.reach)))
            }
            "SURFACE_OF_REVOLUTION" => {
                let c = self.curve_nurbs(ref_at(p, 1)?)?;
                let ax = self.get(ref_at(p, 2)?)?;
                let o = self.point(ref_at(ax.params(), 1)?)?;
                let a = match ax.params().get(2).and_then(|v| v.as_ref()) {
                    Some(r) => self.direction(r)?,
                    None => DVec3::Z,
                };
                Ok(Surface::Nurbs(revolution_surface(&trim_to_axis(&c, o, a), o, a)))
            }
            n => Err(format!("unsupported surface {n}")),
        }
    }

    /// A curve as NURBS over a finite range (lines are taken as unit long).
    fn curve_nurbs(&self, id: u64) -> R<NurbsCurve> {
        let c = self.curve(id)?;
        Ok(match &c {
            Curve3::Nurbs(n) => n.clone(),
            Curve3::Line { .. } => c.to_nurbs(-self.reach, self.reach),
            Curve3::Circle { .. } | Curve3::Ellipse { .. } => c.to_nurbs(0.0, TAU),
        })
    }

    fn bspline_surface(&self, e: &Entity) -> R<NurbsSurface> {
        let (du, dv, rows, k, w) = if let Some(b) = e.part("B_SPLINE_SURFACE") {
            let k = e.part("B_SPLINE_SURFACE_WITH_KNOTS").map(|k| k.to_vec());
            let w = e.part("RATIONAL_B_SPLINE_SURFACE").and_then(|r| r.first()).cloned();
            (real_at(b, 0)? as usize, real_at(b, 1)? as usize, b.get(2).cloned(), k, w)
        } else {
            let p = e.params();
            (real_at(p, 1)? as usize, real_at(p, 2)? as usize, p.get(3).cloned(), Some(p[8..].to_vec()), None)
        };
        let rows = rows.and_then(|r| r.as_list().map(|l| l.to_vec())).ok_or("bad control net")?;
        let nu = rows.len();
        let mut pts = Vec::new();
        for r in &rows {
            for v in r.as_list().ok_or("bad control row")? {
                pts.push(self.point(v.as_ref().ok_or("bad control point")?)?);
            }
        }
        let nv = pts.len() / nu.max(1);
        let k = k.ok_or("missing knots")?;
        let lists: Vec<&[Value]> = k.iter().filter_map(|v| v.as_list()).collect();
        if lists.len() < 4 {
            return Err("bad surface knots".into());
        }
        let ku = expand_knots(lists[0], lists[2]);
        let kv = expand_knots(lists[1], lists[3]);
        if ku.len() != nu + du + 1 || kv.len() != nv + dv + 1 || nu * nv != pts.len() {
            return Err("B-spline surface knot counts do not match the control net".into());
        }
        let weights: Vec<f64> = match w.as_ref().and_then(|w| w.as_list()) {
            Some(rows) => rows.iter().flat_map(|r| r.as_list().unwrap_or(&[]).iter().map(|x| x.as_f64().unwrap_or(1.0))).collect(),
            None => vec![1.0; pts.len()],
        };
        let ctrl = pts.iter().zip(weights.iter().chain(std::iter::repeat(&1.0))).map(|(p, w)| (*p * *w).extend(*w)).collect();
        Ok(NurbsSurface::new(du, dv, ku, kv, nu, nv, ctrl))
    }
}

/// Seam topology of a whole sphere or torus, as the primitives build it.
fn closed_surface_loops(b: &mut Body, s: &Surface) -> Option<Vec<Vec<Coedge>>> {
    use std::f64::consts::FRAC_PI_2;
    match s {
        Surface::Sphere { frame, radius } => {
            let sp = b.add_vertex(frame.to_world(DVec3::new(0.0, 0.0, -radius)));
            let np = b.add_vertex(frame.to_world(DVec3::new(0.0, 0.0, *radius)));
            let mf = Frame { origin: frame.origin, x: frame.x, y: frame.z, z: frame.x.cross(frame.z) };
            let seam = b.add_edge(Curve3::Circle { frame: mf, radius: *radius }, -FRAC_PI_2, FRAC_PI_2, sp, np);
            Some(vec![vec![Coedge::new(seam, false), Coedge::new(seam, true)]])
        }
        Surface::Torus { frame, major, minor } => {
            let v = b.add_vertex(frame.to_world(DVec3::new(major + minor, 0.0, 0.0)));
            let eq = b.add_edge(Curve3::Circle { frame: *frame, radius: major + minor }, 0.0, TAU, v, v);
            let mf = Frame { origin: frame.origin + frame.x * *major, x: frame.x, y: frame.z, z: frame.x.cross(frame.z) };
            let mer = b.add_edge(Curve3::Circle { frame: mf, radius: *minor }, 0.0, TAU, v, v);
            Some(vec![vec![Coedge::new(eq, false), Coedge::new(mer, false), Coedge::new(eq, true), Coedge::new(mer, true)]])
        }
        Surface::Nurbs(n) => {
            // Closed in one direction with both ends of the other collapsed
            // to poles (a revolved profile meeting the axis twice).
            let pt = |h: DVec4| h.truncate() / h.w;
            let collapsed = |pts: Vec<DVec3>| pts.windows(2).all(|w| w[0].distance(w[1]) < 1e-9 * (1.0 + w[0].length()));
            let (deg, knots, ctrl) = if n.closed_v()
                && collapsed((0..n.nv).map(|j| pt(n.cp(0, j))).collect())
                && collapsed((0..n.nv).map(|j| pt(n.cp(n.nu - 1, j))).collect())
            {
                (n.degree_u, n.knots_u.clone(), (0..n.nu).map(|i| n.cp(i, 0)).collect::<Vec<_>>())
            } else if n.closed_u()
                && collapsed((0..n.nu).map(|i| pt(n.cp(i, 0))).collect())
                && collapsed((0..n.nu).map(|i| pt(n.cp(i, n.nv - 1))).collect())
            {
                (n.degree_v, n.knots_v.clone(), (0..n.nv).map(|j| n.cp(0, j)).collect::<Vec<_>>())
            } else {
                return None;
            };
            let c = NurbsCurve::new(deg, knots, ctrl);
            let (t0, t1) = c.domain();
            let a = b.add_vertex(c.point(t0));
            let z = b.add_vertex(c.point(t1));
            let seam = b.add_edge(Curve3::Nurbs(c), t0, t1, a, z);
            Some(vec![vec![Coedge::new(seam, false), Coedge::new(seam, true)]])
        }
        _ => None,
    }
}

/// The curve swept along unit direction `d` from `-reach` to `reach`.
fn extrusion_surface(c: &NurbsCurve, d: DVec3, reach: f64) -> NurbsSurface {
    let n = c.ctrl.len();
    let mut ctrl = Vec::with_capacity(n * 2);
    for h in &c.ctrl {
        ctrl.push(*h - (d * reach * h.w).extend(0.0));
        ctrl.push(*h + (d * reach * h.w).extend(0.0));
    }
    NurbsSurface::new(c.degree, 1, c.knots.clone(), vec![0.0, 0.0, 1.0, 1.0], n, 2, ctrl)
}

/// A closed profile crossing the axis twice sweeps its surface twice; keep
/// the arc between the crossings.
fn trim_to_axis(c: &NurbsCurve, o: DVec3, a: DVec3) -> NurbsCurve {
    let (t0, t1) = c.domain();
    if c.point(t0).distance(c.point(t1)) > 1e-9 * (1.0 + c.point(t0).length()) {
        return c.clone();
    }
    let rho = |t: f64| {
        let d = c.point(t) - o;
        (d - a * d.dot(a)).length()
    };
    let n = 400;
    let ts: Vec<f64> = (0..=n).map(|k| t0 + (t1 - t0) * k as f64 / n as f64).collect();
    let size = ts.iter().map(|t| rho(*t)).fold(0.0, f64::max).max(1e-12);
    // Local minima of the distance to the axis that touch it.
    let mut hits: Vec<f64> = Vec::new();
    for k in 0..n {
        let (l, m, r) = (rho(ts[(k + n - 1) % n]), rho(ts[k]), rho(ts[k + 1]));
        if m <= l && m <= r {
            // Golden-section refinement around ts[k].
            let (mut lo, mut hi) = (ts[k] - (t1 - t0) / n as f64, ts[k] + (t1 - t0) / n as f64);
            for _ in 0..60 {
                let m1 = hi - (hi - lo) * 0.618;
                let m2 = lo + (hi - lo) * 0.618;
                if rho(m1.clamp(t0, t1)) < rho(m2.clamp(t0, t1)) {
                    hi = m2;
                } else {
                    lo = m1;
                }
            }
            let t = (0.5 * (lo + hi)).clamp(t0, t1);
            if rho(t) < 1e-7 * size && hits.iter().all(|h| (h - t).abs() > (t1 - t0) * 1e-3) {
                hits.push(t);
            }
        }
    }
    if hits.len() == 2 {
        hits.sort_by(f64::total_cmp);
        return c.subcurve(hits[0], hits[1]);
    }
    c.clone()
}

/// Surface of revolution as NURBS with STEP's parametrization: u along
/// the rotation (a rational full circle per profile point), v along the
/// profile, so the normal follows du x dv as STEP defines it.
fn revolution_surface(c: &NurbsCurve, o: DVec3, a: DVec3) -> NurbsSurface {
    let unit = NurbsCurve::circle(&Frame::WORLD, 1.0);
    let n = c.ctrl.len();
    let m = unit.ctrl.len();
    let mut ctrl = vec![DVec4::ZERO; n * m];
    for (j, h) in c.ctrl.iter().enumerate() {
        let p = h.truncate() / h.w;
        let base = o + a * (p - o).dot(a);
        let r = p - base;
        let rad = r.length();
        let x = if rad > 1e-12 { r / rad } else { ductile_geom::any_perpendicular(a) };
        let frame = Frame { origin: base, x, y: a.cross(x), z: a };
        for (i, q) in unit.ctrl.iter().enumerate() {
            let local = q.truncate() / q.w;
            let world = frame.origin + (frame.x * local.x + frame.y * local.y) * rad;
            let w = q.w * h.w;
            ctrl[i * n + j] = (world * w).extend(w);
        }
    }
    NurbsSurface::new(unit.degree, c.degree, unit.knots.clone(), c.knots.clone(), m, n, ctrl)
}

/// Length unit (to millimetres) and angle unit (to radians) of the file.
fn units(f: &StepFile) -> (f64, f64) {
    let mut scale = 1.0;
    let mut angle = 1.0;
    let Some(ctx) = f.order.iter().filter_map(|id| f.entities.get(id)).find_map(|e| e.part("GLOBAL_UNIT_ASSIGNED_CONTEXT")) else {
        return (scale, angle);
    };
    let ids: Vec<u64> = ctx.first().and_then(|v| v.as_list()).map(|l| l.iter().filter_map(|v| v.as_ref()).collect()).unwrap_or_default();
    for id in ids {
        let Some(e) = f.entities.get(&id) else { continue };
        if e.part("LENGTH_UNIT").is_some() {
            scale = unit_factor(f, e, 1000.0);
        } else if e.part("PLANE_ANGLE_UNIT").is_some() {
            angle = unit_factor(f, e, 1.0);
        }
    }
    (scale, angle)
}

/// Factor of a unit relative to its SI base, times `si` (the base in the
/// target unit).
fn unit_factor(f: &StepFile, e: &Entity, si: f64) -> f64 {
    if let Some(p) = e.part("SI_UNIT") {
        let prefix = match p.first().and_then(|v| v.as_enum()) {
            Some("MILLI") => 1e-3,
            Some("CENTI") => 1e-2,
            Some("DECI") => 1e-1,
            Some("KILO") => 1e3,
            Some("MICRO") => 1e-6,
            Some("NANO") => 1e-9,
            _ => 1.0,
        };
        return prefix * si;
    }
    if let Some(p) = e.part("CONVERSION_BASED_UNIT") {
        if let Some(m) = p.get(1).and_then(|v| v.as_ref()).and_then(|r| f.entities.get(&r)) {
            let mp = m.params();
            let value = mp.first().and_then(|v| v.as_f64()).unwrap_or(1.0);
            let base = mp.get(1).and_then(|v| v.as_ref()).and_then(|r| f.entities.get(&r)).map(|b| unit_factor(f, b, si)).unwrap_or(si);
            return value * base;
        }
    }
    si
}

impl Reader<'_> {
    fn shell_body(&mut self, name: String, shells: &[(u64, bool)], kind: BodyKind) -> R<Body> {
        let mut b = Body::new(kind);
        let mut verts: HashMap<u64, VertexId> = HashMap::new();
        // STEP edge -> (our edge, whether our direction is opposite to STEP's).
        let mut edges: HashMap<u64, (EdgeId, bool)> = HashMap::new();
        let mut skipped = 0;
        for &(shell, shell_sense) in shells {
            let se = self.get(shell)?;
            let (faces, sense) = if se.name() == "ORIENTED_CLOSED_SHELL" || se.name() == "ORIENTED_OPEN_SHELL" {
                let inner = self.get(ref_at(se.params(), 2)?)?;
                (refs_at(inner.params(), 1)?, shell_sense == bool_at(se.params(), 3))
            } else {
                (refs_at(se.params(), 1)?, shell_sense)
            };
            for fid in faces {
                if let Err(e) = self.face(&mut b, fid, sense, &mut verts, &mut edges) {
                    skipped += 1;
                    self.warnings.push(format!("{name}: face #{fid} skipped: {e}"));
                }
            }
        }
        if skipped > 0 && kind == BodyKind::Solid {
            b.kind = BodyKind::Sheet;
        }
        Ok(b)
    }

    fn face(&mut self, b: &mut Body, fid: u64, shell_sense: bool, verts: &mut HashMap<u64, VertexId>, edges: &mut HashMap<u64, (EdgeId, bool)>) -> R<()> {
        let fe = self.get(fid)?;
        let p = fe.params();
        let surface = self.surface(ref_at(p, 2)?)?;
        let same = bool_at(p, 3);
        let mut outer: Vec<Vec<Coedge>> = Vec::new();
        let mut holes: Vec<Vec<Coedge>> = Vec::new();
        for bid in refs_at(p, 1)? {
            let be = self.get(bid)?;
            let lp = self.get(ref_at(be.params(), 1)?)?;
            let orient = bool_at(be.params(), 2);
            if lp.name() == "VERTEX_LOOP" {
                continue; // a pole: implied by the surface
            }
            let mut coedges = Vec::new();
            for oid in refs_at(lp.params(), 1)? {
                let oe = self.get(oid)?;
                let ecid = ref_at(oe.params(), 3)?;
                let o = bool_at(oe.params(), 4);
                let (e, flipped) = match edges.get(&ecid) {
                    Some(x) => *x,
                    None => {
                        let x = self.edge(b, ecid, verts)?;
                        edges.insert(ecid, x);
                        x
                    }
                };
                coedges.push(Coedge::new(e, o == flipped));
            }
            if !orient {
                coedges = coedges.iter().rev().map(|c| Coedge::new(c.edge, !c.reversed)).collect();
            }
            if be.name() == "FACE_OUTER_BOUND" {
                outer.push(coedges);
            } else {
                holes.push(coedges);
            }
        }
        let mut loops = outer;
        loops.extend(holes);
        if loops.is_empty() {
            // Closed surfaces bounded only by vertex loops (or nothing).
            loops = closed_surface_loops(b, &surface).ok_or("face without edge loops")?;
        }
        let mut reversed = !same;
        if !shell_sense {
            reversed = !reversed;
            for l in loops.iter_mut() {
                *l = l.iter().rev().map(|c| Coedge::new(c.edge, !c.reversed)).collect();
            }
        }
        b.add_face(surface, reversed, loops);
        Ok(())
    }

    fn edge(&mut self, b: &mut Body, id: u64, verts: &mut HashMap<u64, VertexId>) -> R<(EdgeId, bool)> {
        let e = self.get(id)?;
        let p = e.params();
        let (vs, ve) = (ref_at(p, 1)?, ref_at(p, 2)?);
        let mut vertex = |b: &mut Body, r: u64| -> R<VertexId> {
            if let Some(v) = verts.get(&r) {
                return Ok(*v);
            }
            let v = b.add_vertex(self.point(r)?);
            verts.insert(r, v);
            Ok(v)
        };
        let v_start = vertex(b, vs)?;
        let v_end = vertex(b, ve)?;
        let curve = self.curve(ref_at(p, 3)?)?;
        let same = bool_at(p, 4);
        // Our edges run along increasing curve parameter.
        let (va, vb, flipped) = if same { (v_start, v_end, false) } else { (v_end, v_start, true) };
        let (pa, pb) = (b.vertex(va), b.vertex(vb));
        let ta = curve.closest_param(pa);
        let mut tb = curve.closest_param(pb);
        match curve.period() {
            Some(per) => {
                while tb <= ta + 1e-9 {
                    tb += per;
                }
                if va == vb {
                    tb = ta + per;
                }
            }
            None => {
                if tb <= ta {
                    return Err(format!("edge #{id} runs against its curve"));
                }
            }
        }
        Ok((b.add_edge(curve, ta, tb, va, vb), flipped))
    }
}

/// Reads the B-rep bodies of a STEP file.
pub fn read_step(text: &str) -> crate::Result<StepImport> {
    let f = parse(text).map_err(crate::IoError::Parse)?;
    let (scale, angle) = units(&f);
    // Twice the extent of all points bounds every face in the file.
    let extent = f
        .entities
        .values()
        .filter(|e| e.name() == "CARTESIAN_POINT")
        .filter_map(|e| e.params().get(1).and_then(|v| v.as_list()).map(|c| c.iter().filter_map(|x| x.as_f64()).fold(0.0f64, |m, x| m.max(x.abs()))))
        .fold(1.0f64, f64::max);
    let mut r = Reader { f: &f, scale, angle, reach: 2.0 * extent * scale, warnings: Vec::new() };
    let mut bodies = Vec::new();
    for &id in &f.order {
        let e = &f.entities[&id];
        let name = e.params().first().and_then(|v| v.as_str()).unwrap_or("").to_string();
        let res = match e.name() {
            "MANIFOLD_SOLID_BREP" => ref_at(e.params(), 1).and_then(|s| r.shell_body(name.clone(), &[(s, true)], BodyKind::Solid)),
            "BREP_WITH_VOIDS" => ref_at(e.params(), 1).and_then(|s| {
                let mut shells = vec![(s, true)];
                shells.extend(refs_at(e.params(), 2).unwrap_or_default().into_iter().map(|v| (v, true)));
                r.shell_body(name.clone(), &shells, BodyKind::Solid)
            }),
            "SHELL_BASED_SURFACE_MODEL" => {
                refs_at(e.params(), 1).and_then(|ss| r.shell_body(name.clone(), &ss.into_iter().map(|s| (s, true)).collect::<Vec<_>>(), BodyKind::Sheet))
            }
            _ => continue,
        };
        match res {
            Ok(mut b) => {
                if b.faces.is_empty() {
                    continue;
                }
                ductile_brep::ops::seams::add_seams(&mut b);
                // A closed solid encloses positive volume; some writers leave
                // the sense of whole-surface faces ambiguous.
                if b.kind == BodyKind::Solid && ductile_brep::tessellate(&b, &ductile_brep::TessOptions::for_size(b.bbox().diagonal())).volume() < 0.0 {
                    b = ductile_brep::primitives::flip_orientation(b);
                }
                if b.kind == BodyKind::Solid && ductile_brep::ops::boolean::open_edges(&b) > 0 {
                    r.warnings.push(format!("{}: shell is not closed; imported as a surface", if name.is_empty() { "body" } else { &name }));
                    b.kind = BodyKind::Sheet;
                }
                bodies.push(Imported { name: if name.is_empty() { format!("Body {}", bodies.len() + 1) } else { name }, body: b });
            }
            Err(e) => r.warnings.push(format!("#{id} skipped: {e}")),
        }
    }
    Ok(StepImport { bodies, warnings: r.warnings })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::step::write_step;
    use ductile_brep::ops::boolean::{boolean, open_edges, BoolOp};
    use ductile_brep::primitives::*;
    use ductile_brep::tess::{tessellate, TessOptions};

    fn vol(b: &Body) -> f64 {
        tessellate(b, &TessOptions { chord_tol: 1e-4, angle_tol: 0.05 }).volume()
    }

    #[test]
    fn round_trips_solids() {
        let bx = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
        let cyl = make_cylinder(&Frame { origin: DVec3::new(2.0, 1.5, -1.0), ..Frame::WORLD }, 0.7, 4.0);
        let part = boolean(&bx, &cyl, BoolOp::Difference).unwrap();
        let ball = make_sphere(DVec3::new(10.0, 0.0, 0.0), 1.5);
        let tor = make_torus(&Frame { origin: DVec3::new(0.0, 10.0, 0.0), ..Frame::WORLD }, 2.0, 0.5);
        let cone = make_frustum(&Frame { origin: DVec3::new(10.0, 10.0, 0.0), ..Frame::WORLD }, 2.0, 0.5, 3.0);
        let objs = [("part", &part), ("ball", &ball), ("torus", &tor), ("cone", &cone)];
        let mut out = Vec::new();
        write_step(&mut out, "rt", &objs).unwrap();
        let imp = read_step(&String::from_utf8(out).unwrap()).unwrap();
        assert!(imp.warnings.is_empty(), "{:?}", imp.warnings);
        assert_eq!(imp.bodies.len(), 4);
        for ((name, orig), got) in objs.iter().zip(&imp.bodies) {
            assert_eq!(&got.name, name);
            assert_eq!(got.body.kind, BodyKind::Solid);
            assert_eq!(open_edges(&got.body), 0, "{name}");
            assert!((vol(&got.body) - vol(orig)).abs() < 1e-6 * vol(orig).abs().max(1.0), "{name}: {} vs {}", vol(&got.body), vol(orig));
        }
    }

    #[test]
    fn reads_inches_and_reversed_edges() {
        let bx = make_box(&Frame::WORLD, DVec3::new(1.0, 1.0, 1.0));
        let mut out = Vec::new();
        write_step(&mut out, "inch", &[("cube", &bx)]).unwrap();
        let text = String::from_utf8(out).unwrap();
        // Restate the file in inches.
        let inch = text.replace(
            "(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.))",
            "(CONVERSION_BASED_UNIT('INCH',#9999) LENGTH_UNIT() NAMED_UNIT(#9998))",
        );
        let inch = inch.replace("ENDSEC;\nEND", "#9999=LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#9997);\n#9997=(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.));\n#9998=DIMENSIONAL_EXPONENTS(1.,0.,0.,0.,0.,0.,0.);\nENDSEC;\nEND");
        // Flip every edge's sense: the importer must compensate.
        let flipped: String = inch
            .lines()
            .map(|l| {
                if l.contains("EDGE_CURVE(") {
                    // EDGE_CURVE('',#a,#b,#c,.T.) -> EDGE_CURVE('',#b,#a,#c,.F.)
                    let inner = &l[l.find("EDGE_CURVE(").unwrap() + 11..l.rfind(')').unwrap()];
                    let parts: Vec<&str> = inner.split(',').collect();
                    let head = &l[..l.find("EDGE_CURVE(").unwrap()];
                    format!("{head}EDGE_CURVE({},{},{},{},.F.);", parts[0], parts[2], parts[1], parts[3])
                } else if l.contains("ORIENTED_EDGE(") {
                    l.replace(".T.);", ".X.);").replace(".F.);", ".T.);").replace(".X.);", ".F.);")
                } else {
                    l.to_string()
                }
            })
            .collect::<Vec<_>>()
            .join("\n");
        let imp = read_step(&flipped).unwrap();
        let b = &imp.bodies[0].body;
        assert_eq!(open_edges(b), 0);
        let v = vol(b);
        assert!((v - 25.4f64.powi(3)).abs() < 1e-3 * 25.4f64.powi(3), "{v}");
    }
}
