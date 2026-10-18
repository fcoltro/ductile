// SPDX-License-Identifier: GPL-3.0-only
//! Tessellation of B-rep bodies into triangle meshes for display, picking
//! and mesh export.
//!
//! Every edge is sampled once and the samples are shared by the faces on both
//! sides, so the resulting mesh is watertight. Each face is triangulated in
//! its (metrically scaled) parameter space with a constrained Delaunay
//! triangulation of its trimming loops plus interior points placed according
//! to surface curvature.

use crate::topo::{Body, Coedge, EdgeId, FaceId};
use ductile_geom::poly2::point_in_loops;
use ductile_geom::{DVec2, DVec3, Surface};
use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};
use std::collections::HashMap;

#[derive(Clone, Copy, Debug)]
pub struct TessOptions {
    /// Maximum distance between the mesh and the exact surface.
    pub chord_tol: f64,
    /// Maximum angle (radians) between adjacent normals along curves.
    pub angle_tol: f64,
}

impl Default for TessOptions {
    fn default() -> Self {
        Self { chord_tol: 0.01, angle_tol: 0.2 }
    }
}

impl TessOptions {
    /// Tolerances scaled to the size of a model.
    pub fn for_size(diag: f64) -> Self {
        Self { chord_tol: (diag * 4e-4).max(1e-5), angle_tol: 0.2 }
    }
}

#[derive(Clone, Debug, Default)]
pub struct FaceMesh {
    pub positions: Vec<DVec3>,
    pub normals: Vec<DVec3>,
    pub uvs: Vec<DVec2>,
    pub triangles: Vec<[u32; 3]>,
}

#[derive(Clone, Debug, Default)]
pub struct BodyMesh {
    pub faces: Vec<FaceMesh>,
    /// One polyline per edge, indexed by edge id.
    pub edges: Vec<Vec<DVec3>>,
}

impl BodyMesh {
    pub fn triangle_count(&self) -> usize {
        self.faces.iter().map(|f| f.triangles.len()).sum()
    }

    /// Flattened triangle soup over all faces with the originating face id.
    pub fn triangles(&self) -> impl Iterator<Item = ([DVec3; 3], FaceId)> + '_ {
        self.faces.iter().enumerate().flat_map(|(fi, f)| {
            f.triangles.iter().map(move |t| {
                (
                    [f.positions[t[0] as usize], f.positions[t[1] as usize], f.positions[t[2] as usize]],
                    FaceId(fi as u32),
                )
            })
        })
    }

    /// Signed volume enclosed by the mesh (positive for outward normals).
    pub fn volume(&self) -> f64 {
        self.triangles().map(|(t, _)| t[0].dot(t[1].cross(t[2])) / 6.0).sum()
    }

    pub fn area(&self) -> f64 {
        self.triangles().map(|(t, _)| (t[1] - t[0]).cross(t[2] - t[0]).length() * 0.5).sum()
    }
}

/// Samples an edge, returning (parameter, point) pairs from t0 to t1.
pub fn tessellate_edge(body: &Body, e: EdgeId, opts: &TessOptions) -> Vec<(f64, DVec3)> {
    let edge = body.edge(e);
    let mut pts = edge.curve.tessellate(edge.t0, edge.t1, opts.chord_tol);
    // Enforce an angular limit too so that curved edges on flat faces look smooth.
    if !edge.curve.is_linear() {
        let sweep_steps = estimate_turning(&edge.curve, edge.t0, edge.t1) / opts.angle_tol;
        let min_n = sweep_steps.ceil() as usize;
        if pts.len() - 1 < min_n {
            pts = (0..=min_n)
                .map(|i| {
                    let t = edge.t0 + (edge.t1 - edge.t0) * i as f64 / min_n as f64;
                    (t, edge.curve.point(t))
                })
                .collect();
        }
    }
    // Snap ends to vertices exactly.
    pts.first_mut().unwrap().1 = body.vertex(edge.v0);
    pts.last_mut().unwrap().1 = body.vertex(edge.v1);
    pts
}

fn estimate_turning(curve: &ductile_geom::Curve3, t0: f64, t1: f64) -> f64 {
    let n = 32;
    let mut total = 0.0;
    let mut prev = curve.tangent(t0).normalize_or_zero();
    for i in 1..=n {
        let t = t0 + (t1 - t0) * i as f64 / n as f64;
        let d = curve.tangent(t).normalize_or_zero();
        total += prev.dot(d).clamp(-1.0, 1.0).acos();
        prev = d;
    }
    total
}

pub fn tessellate(body: &Body, opts: &TessOptions) -> BodyMesh {
    let edges: Vec<Vec<(f64, DVec3)>> = body.edge_ids().map(|e| tessellate_edge(body, e, opts)).collect();
    let faces = body.face_ids().map(|f| tessellate_face(body, f, &edges, opts)).collect();
    BodyMesh { faces, edges: edges.into_iter().map(|e| e.into_iter().map(|(_, p)| p).collect()).collect() }
}

/// A boundary sample in UV with its exact 3D position.
#[derive(Clone, Copy, Debug)]
pub struct BSample {
    pub uv: DVec2,
    pub p: DVec3,
}

pub fn is_singular(s: &Surface, uv: DVec2) -> bool {
    match s {
        Surface::Sphere { .. } => (uv.y.abs() - std::f64::consts::FRAC_PI_2).abs() < 1e-9,
        Surface::Cone { radius, half_angle, .. } => (radius + uv.y * half_angle.tan()).abs() < 1e-9,
        Surface::Nurbs(_) => {
            let d = s.derivs(uv.x, uv.y);
            d.du.length() < 1e-9 || d.dv.length() < 1e-9
        }
        _ => false,
    }
}

/// Builds the UV polygon of each loop of face `f`.
/// UV polylines of every coedge of face `f`, grouped by loop, with seams
/// placed on the correct side of the period, periodic coordinates unwrapped
/// continuously and hole loops shifted into the outer loop's period.
pub fn face_coedge_uvs(body: &Body, f: FaceId, edges: &[Vec<(f64, DVec3)>]) -> Vec<Vec<(Coedge, Vec<BSample>)>> {
    let face = body.face(f);
    let surf = &face.surface;
    let pu = surf.period_u();
    let pv = surf.period_v();
    // Edges used twice by this face are seams.
    let mut use_count: HashMap<EdgeId, usize> = HashMap::new();
    for l in body.face_loops(f) {
        for c in &l.coedges {
            *use_count.entry(c.edge).or_default() += 1;
        }
    }
    // Orientation of the parameter plane relative to the outward normal.
    let flip = face_uv_flipped(body, f);
    let mut out = Vec::new();
    for l in body.face_loops(f) {
        let mut coedge_uvs: Vec<Vec<BSample>> = Vec::new();
        for c in &l.coedges {
            coedge_uvs.push(project_coedge(surf, *c, &edges[c.edge.idx()]));
        }
        // Fix seam coedges to the correct side of the period.
        let mut fixed = vec![false; l.coedges.len()];
        for (i, c) in l.coedges.iter().enumerate() {
            if use_count[&c.edge] < 2 {
                continue;
            }
            let s = &mut coedge_uvs[i];
            let a = s.first().unwrap().uv;
            let b = s.last().unwrap().uv;
            let du = s.iter().map(|x| x.uv.x).fold(f64::NEG_INFINITY, f64::max)
                - s.iter().map(|x| x.uv.x).fold(f64::INFINITY, f64::min);
            let dv = s.iter().map(|x| x.uv.y).fold(f64::NEG_INFINITY, f64::max)
                - s.iter().map(|x| x.uv.y).fold(f64::INFINITY, f64::min);
            if let (Some(p), true) = (pu, du < 1e-6) {
                // Constant-u seam. Moving +v with interior on the left => high u.
                let going_up = (b.y > a.y) != flip;
                let base = ductile_geom::wrap_angle(a.x, -1e-7);
                let target = if going_up { base + p } else { base };
                for x in s.iter_mut() {
                    x.uv.x = target;
                }
                fixed[i] = true;
            } else if let (Some(p), true) = (pv, dv < 1e-6) {
                // Constant-v seam. Moving +u with interior on the left => low v.
                let going_right = (b.x > a.x) != flip;
                let base = ductile_geom::wrap_angle(a.y, -1e-7);
                let target = if going_right { base } else { base + p };
                for x in s.iter_mut() {
                    x.uv.y = target;
                }
                fixed[i] = true;
            }
        }
        // Unwrap the remaining coedges by continuity, starting after a fixed one.
        let n = l.coedges.len();
        // Start after a fixed seam, else right after a singular point (where
        // the periodic coordinate is undefined), else anywhere.
        let start = fixed.iter().position(|x| *x).unwrap_or_else(|| {
            (0..n).find(|&i| is_singular(surf, coedge_uvs[(i + n - 1) % n].last().unwrap().uv)).unwrap_or(0)
        });
        let mut prev_end: Option<DVec2> = None;
        for k in 0..n {
            let i = (start + k) % n;
            if fixed[i] {
                // Seam coordinate is fixed; shift the free coordinate by whole
                // periods to continue from the previous coedge.
                let s = &mut coedge_uvs[i];
                {
                    let first = s[0].uv;
                    // The first fixed coedge anchors the loop in the base period.
                    let pe = prev_end.unwrap_or_else(|| {
                        DVec2::new(
                            pu.map(|_| ductile_geom::wrap_angle(first.x, -1e-7)).unwrap_or(first.x),
                            pv.map(|_| ductile_geom::wrap_angle(first.y, -1e-7)).unwrap_or(first.y),
                        )
                    });
                    let aligned = surf.align_periodic(first, pe);
                    let mut shift = aligned - first;
                    // Never move the fixed seam coordinate.
                    if pu.is_some() && s.iter().all(|x| (x.uv.x - first.x).abs() < 1e-9) {
                        shift.x = 0.0;
                    } else {
                        shift.y = 0.0;
                    }
                    for x in s.iter_mut() {
                        x.uv += shift;
                    }
                }
                prev_end = Some(s.last().unwrap().uv);
                continue;
            }
            let s = &mut coedge_uvs[i];
            let own = s.iter().map(|x| x.uv).find(|uv| !is_singular(surf, *uv)).unwrap_or(s[0].uv);
            let mut hint = match prev_end {
                Some(p) if !is_singular(surf, p) => p,
                _ => own,
            };
            for x in s.iter_mut() {
                x.uv = surf.align_periodic(x.uv, hint);
                hint = x.uv;
            }
            prev_end = Some(hint);
        }
        // Resolve singular endpoints: take u (or v) from the neighbour sample.
        for s in coedge_uvs.iter_mut() {
            let m = s.len();
            if m >= 2 {
                if is_singular(surf, s[0].uv) {
                    s[0].uv.x = s[1].uv.x;
                }
                if is_singular(surf, s[m - 1].uv) {
                    s[m - 1].uv.x = s[m - 2].uv.x;
                }
            }
        }
        if let Some(p) = pu {
            resolve_singular_chains(surf, &mut coedge_uvs, &fixed, flip, out.is_empty(), p);
        }
        out.push(l.coedges.iter().copied().zip(coedge_uvs).collect::<Vec<_>>());
    }
    // Shift hole loops into the outer loop's period.
    if out.len() > 1 {
        let all0: Vec<BSample> = out[0].iter().flat_map(|(_, s)| s.iter().copied()).collect();
        let (umin, umax) = range(&all0, |s| s.uv.x);
        let (vmin, vmax) = range(&all0, |s| s.uv.y);
        let center = DVec2::new(0.5 * (umin + umax), 0.5 * (vmin + vmax));
        for h in out.iter_mut().skip(1) {
            let allh: Vec<BSample> = h.iter().flat_map(|(_, s)| s.iter().copied()).collect();
            let (a, b) = range(&allh, |s| s.uv.x);
            let (c, d) = range(&allh, |s| s.uv.y);
            let hc = DVec2::new(0.5 * (a + b), 0.5 * (c + d));
            let shift = surf.align_periodic(hc, center) - hc;
            for (_, s) in h.iter_mut() {
                for x in s.iter_mut() {
                    x.uv += shift;
                }
            }
        }
    }
    out
}

/// A loop passing through a singular point (a pole or apex) has no
/// continuous u there, so each chain of coedges between singular points is
/// shifted by whole periods to give the loop its required orientation
/// (counter-clockwise outer loops, clockwise holes) with the smallest area.
fn resolve_singular_chains(surf: &Surface, uvs: &mut [Vec<BSample>], fixed: &[bool], flip: bool, outer: bool, period: f64) {
    let n = uvs.len();
    let starts: Vec<usize> = (0..n).filter(|&i| is_singular(surf, uvs[(i + n - 1) % n].last().unwrap().uv)).collect();
    if starts.len() < 2 {
        return;
    }
    // Chains as index lists; a chain holding a fixed seam never moves.
    let mut chains: Vec<Vec<usize>> = Vec::new();
    for (k, &s0) in starts.iter().enumerate() {
        let s1 = starts[(k + 1) % starts.len()];
        let mut c = vec![];
        let mut i = s0;
        loop {
            c.push(i);
            i = (i + 1) % n;
            if i == s1 {
                break;
            }
        }
        chains.push(c);
    }
    let movable: Vec<usize> = (0..chains.len()).filter(|&c| !chains[c].iter().any(|&i| fixed[i])).collect();
    let anchor_fixed = movable.len() < chains.len();
    // Keep one chain still when nothing is fixed.
    let movable: Vec<usize> = if anchor_fixed { movable } else { movable.into_iter().skip(1).collect() };
    if movable.is_empty() || movable.len() > 6 {
        return;
    }
    let sign = if flip { -1.0 } else { 1.0 };
    let area_with = |uvs: &[Vec<BSample>], shifts: &[f64]| -> f64 {
        let mut poly: Vec<DVec2> = Vec::new();
        for (ci, c) in chains.iter().enumerate() {
            let du = movable.iter().position(|&m| m == ci).map(|k| shifts[k]).unwrap_or(0.0);
            for &i in c {
                poly.extend(uvs[i].iter().map(|x| x.uv + DVec2::new(du, 0.0)));
            }
        }
        ductile_geom::poly2::signed_area(&poly) * sign
    };
    let mut best: Option<(f64, Vec<f64>)> = None;
    let combos = 3usize.pow(movable.len() as u32);
    for code in 0..combos {
        let mut c = code;
        let shifts: Vec<f64> = (0..movable.len())
            .map(|_| {
                let k = (c % 3) as f64 - 1.0;
                c /= 3;
                k * period
            })
            .collect();
        let a = area_with(uvs, &shifts);
        if (a > 0.0) == outer && best.as_ref().map(|b| a.abs() < b.0).unwrap_or(true) {
            best = Some((a.abs(), shifts));
        }
    }
    if let Some((_, shifts)) = best {
        for (k, &ci) in movable.iter().enumerate() {
            for &i in &chains[ci] {
                for x in uvs[i].iter_mut() {
                    x.uv.x += shifts[k];
                }
            }
        }
    }
}

/// Flattened UV polygon of each loop of face `f`.
pub fn face_uv_loops(body: &Body, f: FaceId, edges: &[Vec<(f64, DVec3)>]) -> Vec<Vec<BSample>> {
    face_coedge_uvs(body, f, edges)
        .into_iter()
        .map(|l| {
            let mut poly: Vec<BSample> = Vec::new();
            for (_, s) in l {
                for x in s {
                    if let Some(last) = poly.last() {
                        if last.uv.distance(x.uv) < 1e-12 {
                            continue;
                        }
                    }
                    poly.push(x);
                }
            }
            if poly.len() > 1 && poly[0].uv.distance(poly.last().unwrap().uv) < 1e-9 {
                poly.pop();
            }
            poly
        })
        .collect()
}

fn range(s: &[BSample], f: impl Fn(&BSample) -> f64) -> (f64, f64) {
    s.iter().map(&f).fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), x| (a.min(x), b.max(x)))
}

/// True if counter-clockwise loops in UV correspond to clockwise loops
/// about the face's outward normal.
pub fn face_uv_flipped(body: &Body, f: FaceId) -> bool {
    let face = body.face(f);
    let s = &face.surface;
    // Pick a regular parameter point.
    let uv = match s {
        Surface::Nurbs(n) => {
            let ((u0, u1), (v0, v1)) = n.domain();
            DVec2::new(0.37 * u0 + 0.63 * u1, 0.41 * v0 + 0.59 * v1)
        }
        _ => DVec2::new(0.3, 0.2),
    };
    let d = s.derivs(uv.x, uv.y);
    let geo = d.du.cross(d.dv);
    let outward = body.face_normal(f, uv.x, uv.y);
    geo.dot(outward) < 0.0
}

fn project_coedge(surf: &Surface, c: Coedge, samples: &[(f64, DVec3)]) -> Vec<BSample> {
    let mut pts: Vec<DVec3> = samples.iter().map(|s| s.1).collect();
    if c.reversed {
        pts.reverse();
    }
    let mut out = Vec::with_capacity(pts.len());
    let mut hint: Option<DVec2> = None;
    for p in pts {
        let uv = surf.project(p, hint);
        hint = Some(uv);
        out.push(BSample { uv, p });
    }
    out
}

/// Tessellates one face.
pub fn tessellate_face(body: &Body, f: FaceId, edges: &[Vec<(f64, DVec3)>], opts: &TessOptions) -> FaceMesh {
    let face = body.face(f);
    let surf = &face.surface;
    let loops = if face.loops.is_empty() { natural_loops(surf) } else { face_uv_loops(body, f, edges) };
    if loops.is_empty() || loops[0].len() < 3 {
        return FaceMesh::default();
    }
    let uv_loops: Vec<Vec<DVec2>> = loops.iter().map(|l| l.iter().map(|s| s.uv).collect()).collect();

    // Parameter-space bounding box.
    let mut lo = DVec2::splat(f64::INFINITY);
    let mut hi = DVec2::splat(f64::NEG_INFINITY);
    for l in &uv_loops {
        for p in l {
            lo = lo.min(*p);
            hi = hi.max(*p);
        }
    }
    // Metric scale so the triangulation is roughly isotropic in 3D.
    let (su, sv, mut step_u, mut step_v) = surface_metrics(surf, lo, hi, opts);
    // Straight directions (cylinder rulings) need no subdivision on their
    // own, but trimmed faces still need interior points there, or long
    // triangles would join distant boundary samples across the curvature.
    if step_u.is_finite() && !step_v.is_finite() && sv > 0.0 {
        step_v = step_u * su / sv;
    } else if step_v.is_finite() && !step_u.is_finite() && su > 0.0 {
        step_u = step_v * sv / su;
    }
    let scale = DVec2::new(su, sv);

    let mut cdt: ConstrainedDelaunayTriangulation<Point2<f64>> = ConstrainedDelaunayTriangulation::new();
    // Remember exact 3D boundary points keyed by scaled UV.
    let mut exact: HashMap<(u64, u64), DVec3> = HashMap::new();
    let key = |p: DVec2| (p.x.to_bits(), p.y.to_bits());
    for l in &loops {
        let mut handles = Vec::with_capacity(l.len());
        for s in l {
            let q = s.uv * scale;
            match cdt.insert(Point2::new(q.x, q.y)) {
                Ok(h) => {
                    exact.insert(key(q), s.p);
                    handles.push(h);
                }
                Err(_) => continue,
            }
        }
        for i in 0..handles.len() {
            let a = handles[i];
            let b = handles[(i + 1) % handles.len()];
            if a != b {
                cdt.try_add_constraint(a, b);
            }
        }
    }

    // Interior points on a grid, skipping those too close to the boundary.
    if step_u.is_finite() || step_v.is_finite() {
        let nu = if step_u.is_finite() { ((hi.x - lo.x) / step_u).ceil().clamp(1.0, 400.0) as usize } else { 1 };
        let nv = if step_v.is_finite() { ((hi.y - lo.y) / step_v).ceil().clamp(1.0, 400.0) as usize } else { 1 };
        if nu > 1 || nv > 1 {
            let du = (hi.x - lo.x) / nu as f64;
            let dv = (hi.y - lo.y) / nv as f64;
            let min_clear = 0.35 * (du * su).min(dv * sv).max(1e-12);
            let segs: Vec<(DVec2, DVec2)> = uv_loops
                .iter()
                .flat_map(|l| (0..l.len()).map(move |i| (l[i] * scale, l[(i + 1) % l.len()] * scale)))
                .collect();
            let grid = SegmentGrid::new(&segs, min_clear.max(1e-9));
            for i in 1..nu {
                for j in 1..nv {
                    let uv = DVec2::new(lo.x + du * i as f64, lo.y + dv * j as f64);
                    if !point_in_loops(uv, &uv_loops) {
                        continue;
                    }
                    let q = uv * scale;
                    if grid.min_dist(q) < min_clear {
                        continue;
                    }
                    let _ = cdt.insert(Point2::new(q.x, q.y));
                }
            }
        }
    }

    // Collect vertices.
    let mut mesh = FaceMesh::default();
    let mut index_of: HashMap<usize, u32> = HashMap::new();
    for v in cdt.vertices() {
        let q = DVec2::new(v.position().x, v.position().y);
        let uv = q / scale;
        let p = exact.get(&key(q)).copied().unwrap_or_else(|| surf.point(uv.x, uv.y));
        index_of.insert(v.fix().index(), mesh.positions.len() as u32);
        mesh.positions.push(p);
        mesh.uvs.push(uv);
        mesh.normals.push(body.face_normal(f, uv.x, uv.y));
    }
    // Inside/outside by flood fill from the hull, toggling at constraint
    // edges; this rejects the near-degenerate hull slivers that a centroid
    // test lets through when a boundary is numerically almost straight.
    let mut inside: Vec<Option<bool>> = vec![None; cdt.num_all_faces()];
    let mut queue = std::collections::VecDeque::new();
    for t in cdt.inner_faces() {
        for e in t.adjacent_edges() {
            if e.rev().face().is_outer() && inside[t.fix().index()].is_none() {
                inside[t.fix().index()] = Some(e.is_constraint_edge());
                queue.push_back(t.fix());
            }
        }
    }
    while let Some(ff) = queue.pop_front() {
        let s = inside[ff.index()].unwrap();
        for e in cdt.face(ff).adjacent_edges() {
            if let Some(nb) = e.rev().face().as_inner() {
                if inside[nb.fix().index()].is_none() {
                    inside[nb.fix().index()] = Some(s ^ e.is_constraint_edge());
                    queue.push_back(nb.fix());
                }
            }
        }
    }
    for t in cdt.inner_faces() {
        let vs = t.vertices();
        let q: Vec<DVec2> = vs.iter().map(|v| DVec2::new(v.position().x, v.position().y)).collect();
        let c = (q[0] + q[1] + q[2]) / 3.0 / scale;
        let is_in = inside[t.fix().index()].unwrap_or_else(|| point_in_loops(c, &uv_loops));
        if !is_in {
            continue;
        }
        let mut idx = [0u32; 3];
        for k in 0..3 {
            idx[k] = index_of[&vs[k].fix().index()];
        }
        // Orient by the outward normal.
        let (a, b, cc) = (mesh.positions[idx[0] as usize], mesh.positions[idx[1] as usize], mesh.positions[idx[2] as usize]);
        let geo = (b - a).cross(cc - a);
        let n = body.face_normal(f, c.x, c.y);
        if geo.dot(n) < 0.0 {
            idx.swap(1, 2);
        }
        mesh.triangles.push(idx);
    }
    // Fix up normals at singular points (use the triangle normals around them).
    for (i, uv) in mesh.uvs.clone().iter().enumerate() {
        if is_singular(surf, *uv) {
            let mut acc = DVec3::ZERO;
            for t in &mesh.triangles {
                if t.contains(&(i as u32)) {
                    let (a, b, c) = (mesh.positions[t[0] as usize], mesh.positions[t[1] as usize], mesh.positions[t[2] as usize]);
                    acc += (b - a).cross(c - a);
                }
            }
            if acc.length_squared() > 0.0 && !matches!(surf, Surface::Sphere { .. }) {
                mesh.normals[i] = acc.normalize();
            }
        }
    }
    mesh
}

/// Loops for a closed surface with no trimming (whole torus / sphere in one face).
fn natural_loops(surf: &Surface) -> Vec<Vec<BSample>> {
    let (bu, bv) = surf.bounds();
    let (Some((u0, u1)), Some((v0, v1))) = (bu, bv) else { return vec![] };
    let n = 64;
    let mut l = Vec::new();
    let mut push = |u: f64, v: f64| l.push(BSample { uv: DVec2::new(u, v), p: surf.point(u, v) });
    for i in 0..n {
        push(u0 + (u1 - u0) * i as f64 / n as f64, v0);
    }
    for i in 0..n {
        push(u1, v0 + (v1 - v0) * i as f64 / n as f64);
    }
    for i in 0..n {
        push(u1 - (u1 - u0) * i as f64 / n as f64, v1);
    }
    for i in 0..n {
        push(u0, v1 - (v1 - v0) * i as f64 / n as f64);
    }
    vec![l]
}

/// Returns (scale_u, scale_v, step_u, step_v): metric scale factors and the
/// parameter step needed to meet the chord tolerance.
fn surface_metrics(surf: &Surface, lo: DVec2, hi: DVec2, opts: &TessOptions) -> (f64, f64, f64, f64) {
    let tol = opts.chord_tol;
    match surf {
        Surface::Plane { .. } => (1.0, 1.0, f64::INFINITY, f64::INFINITY),
        Surface::Cylinder { radius, .. } => {
            let a = step_for_radius(*radius, tol, opts.angle_tol);
            (*radius, 1.0, a, f64::INFINITY)
        }
        Surface::Cone { radius, half_angle, .. } => {
            let rmax = (radius + lo.y * half_angle.tan()).abs().max((radius + hi.y * half_angle.tan()).abs());
            let a = step_for_radius(rmax, tol, opts.angle_tol);
            (rmax.max(1e-6), 1.0 / half_angle.cos(), a, (hi.y - lo.y) / 2.0)
        }
        Surface::Sphere { radius, .. } => {
            let a = step_for_radius(*radius, tol, opts.angle_tol);
            (*radius, *radius, a, a)
        }
        Surface::Torus { major, minor, .. } => {
            let a = step_for_radius(major + minor, tol, opts.angle_tol);
            let b = step_for_radius(*minor, tol, opts.angle_tol);
            (major + minor, *minor, a, b)
        }
        Surface::Nurbs(_) => {
            let n = 6;
            let (mut su, mut sv, mut cu, mut cv) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
            for i in 0..=n {
                for j in 0..=n {
                    let u = lo.x + (hi.x - lo.x) * i as f64 / n as f64;
                    let v = lo.y + (hi.y - lo.y) * j as f64 / n as f64;
                    let d = surf.derivs(u, v);
                    su = su.max(d.du.length());
                    sv = sv.max(d.dv.length());
                    cu = cu.max(d.duu.length());
                    cv = cv.max(d.dvv.length());
                }
            }
            let step = |c: f64, s: f64| {
                let h_chord = if c > 1e-12 { (8.0 * tol / c).sqrt() } else { f64::INFINITY };
                // Angular limit: |S'| h / R  ~ angle, with R ~ |S'|^2 / |S''|.
                let h_ang = if c > 1e-12 && s > 1e-12 { opts.angle_tol * s / c } else { f64::INFINITY };
                h_chord.min(h_ang)
            };
            let span_u = hi.x - lo.x;
            let span_v = hi.y - lo.y;
            (su.max(1e-9), sv.max(1e-9), step(cu, su).min(span_u / 2.0), step(cv, sv).min(span_v / 2.0))
        }
    }
}

fn step_for_radius(r: f64, tol: f64, ang: f64) -> f64 {
    if r <= tol {
        return std::f64::consts::FRAC_PI_2;
    }
    let chord = 2.0 * (1.0 - tol / r).clamp(-1.0, 1.0).acos();
    chord.min(ang).max(1e-3)
}

/// Uniform grid over segments for fast nearest-segment distance queries.
struct SegmentGrid {
    origin: DVec2,
    cell: f64,
    nx: usize,
    ny: usize,
    cells: Vec<Vec<usize>>,
    segs: Vec<(DVec2, DVec2)>,
}

impl SegmentGrid {
    fn new(segs: &[(DVec2, DVec2)], cell: f64) -> Self {
        let mut lo = DVec2::splat(f64::INFINITY);
        let mut hi = DVec2::splat(f64::NEG_INFINITY);
        for (a, b) in segs {
            lo = lo.min(a.min(*b));
            hi = hi.max(a.max(*b));
        }
        let mut cell = cell;
        let extent = (hi - lo).max(DVec2::splat(1e-12));
        while (extent.x / cell) * (extent.y / cell) > 4.0e6 {
            cell *= 2.0;
        }
        let nx = (extent.x / cell).ceil() as usize + 1;
        let ny = (extent.y / cell).ceil() as usize + 1;
        let mut cells = vec![Vec::new(); nx * ny];
        for (i, (a, b)) in segs.iter().enumerate() {
            let mn = a.min(*b);
            let mx = a.max(*b);
            let x0 = ((mn.x - lo.x) / cell).floor() as usize;
            let y0 = ((mn.y - lo.y) / cell).floor() as usize;
            let x1 = (((mx.x - lo.x) / cell).floor() as usize).min(nx - 1);
            let y1 = (((mx.y - lo.y) / cell).floor() as usize).min(ny - 1);
            for x in x0..=x1 {
                for y in y0..=y1 {
                    cells[y * nx + x].push(i);
                }
            }
        }
        Self { origin: lo, cell, nx, ny, cells, segs: segs.to_vec() }
    }

    /// Distance to the nearest segment if it is within one cell, else infinity.
    fn min_dist(&self, p: DVec2) -> f64 {
        let cx = ((p.x - self.origin.x) / self.cell).floor() as isize;
        let cy = ((p.y - self.origin.y) / self.cell).floor() as isize;
        let mut best = f64::INFINITY;
        for x in cx - 1..=cx + 1 {
            for y in cy - 1..=cy + 1 {
                if x < 0 || y < 0 || x as usize >= self.nx || y as usize >= self.ny {
                    continue;
                }
                for &i in &self.cells[y as usize * self.nx + x as usize] {
                    let (a, b) = self.segs[i];
                    best = best.min(ductile_geom::poly2::dist_to_segment(p, a, b));
                }
            }
        }
        best
    }
}
