// SPDX-License-Identifier: GPL-3.0-only
//! PolySplines: a polygon mesh (a subdivision cage) turned into smooth,
//! untrimmed spline surfaces.
//!
//! Where four quadrilaterals meet at every corner of a quadrilateral, its
//! Catmull-Clark limit surface is exactly the bicubic B-spline over the
//! 4×4 grid of points around it: one Bézier patch, curvature continuous
//! (C2) with its regular neighbours. Each extraordinary vertex (three,
//! five or more quadrilaterals meeting) is cut out with the ring of
//! quadrilaterals around it, and the hole filled tangentially (G1) by an
//! N-sided fill whose edges are exactly the surrounding patches' edges.
//! Meshes with triangles or other polygons, or with extraordinary vertices
//! too close together, are subdivided first (Catmull-Clark) until each
//! one stands apart.

use crate::topo::{Body, BodyKind};
use ductile_geom::{DVec3, NurbsCurve, NurbsSurface};
use std::collections::{BTreeSet, HashMap};

#[derive(Clone, Debug)]
pub struct PolyMesh {
    pub points: Vec<DVec3>,
    /// Polygons as indices into `points`, counter-clockwise seen from
    /// outside.
    pub faces: Vec<Vec<usize>>,
}

#[derive(Debug, PartialEq, Eq)]
pub enum PolySplineError {
    /// An edge with more than two faces, or faces facing opposite ways.
    NotManifold,
    /// Mesh boundaries are supported only where their vertices are
    /// regular (two faces along the boundary, one at a corner) and away
    /// from extraordinary vertices.
    Boundary,
    /// Extraordinary vertices still too close after subdividing.
    Crowded,
    /// Fitting a fill failed.
    Fit,
}

impl std::fmt::Display for PolySplineError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            PolySplineError::NotManifold => "the mesh is not manifold (an edge has more than two faces, or faces disagree on which way is out)",
            PolySplineError::Boundary => "an open edge of the mesh meets an extraordinary vertex; close the mesh or add a supporting loop there",
            PolySplineError::Crowded => "extraordinary vertices sit too close together, even after subdividing",
            PolySplineError::Fit => "a fill round an extraordinary vertex could not be fitted",
        })
    }
}

/// Adjacency of a polygon mesh.
struct Topo {
    /// Directed edge (a, b) to the face that runs a then b.
    directed: HashMap<(usize, usize), usize>,
    /// Faces at each vertex.
    vertex_faces: Vec<Vec<usize>>,
}

impl Topo {
    fn new(m: &PolyMesh) -> Result<Topo, PolySplineError> {
        let mut directed = HashMap::new();
        let mut vertex_faces = vec![Vec::new(); m.points.len()];
        for (f, poly) in m.faces.iter().enumerate() {
            for i in 0..poly.len() {
                let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
                if directed.insert((a, b), f).is_some() {
                    return Err(PolySplineError::NotManifold);
                }
                vertex_faces[a].push(f);
            }
        }
        Ok(Topo { directed, vertex_faces })
    }

    fn is_boundary_edge(&self, a: usize, b: usize) -> bool {
        !self.directed.contains_key(&(b, a))
    }

    fn interior(&self, m: &PolyMesh, v: usize) -> bool {
        self.vertex_faces[v].iter().all(|f| {
            let p = &m.faces[*f];
            let i = p.iter().position(|x| *x == v).unwrap();
            !self.is_boundary_edge(v, p[(i + 1) % p.len()]) && !self.is_boundary_edge(p[(i + p.len() - 1) % p.len()], v)
        })
    }

    /// An interior vertex where other than four faces meet, or a boundary
    /// vertex where other than two (one at a corner) do.
    fn extraordinary(&self, m: &PolyMesh, v: usize) -> bool {
        let n = self.vertex_faces[v].len();
        if self.interior(m, v) { n != 4 } else { n > 2 }
    }
}

/// One Catmull-Clark step: every face split into quadrilaterals, one per
/// corner; boundaries follow the cubic B-spline curve rules.
pub fn catmull_clark(m: &PolyMesh) -> Result<PolyMesh, PolySplineError> {
    let topo = Topo::new(m)?;
    let nv = m.points.len();
    let face_pt: Vec<DVec3> = m.faces.iter().map(|f| f.iter().map(|i| m.points[*i]).sum::<DVec3>() / f.len() as f64).collect();
    // Undirected edges, numbered, with their faces.
    let mut edge_id: HashMap<(usize, usize), usize> = HashMap::new();
    let mut edge_faces: Vec<Vec<usize>> = Vec::new();
    let mut edge_ends: Vec<(usize, usize)> = Vec::new();
    for (f, poly) in m.faces.iter().enumerate() {
        for i in 0..poly.len() {
            let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
            let key = (a.min(b), a.max(b));
            let id = *edge_id.entry(key).or_insert_with(|| {
                edge_faces.push(Vec::new());
                edge_ends.push(key);
                edge_faces.len() - 1
            });
            edge_faces[id].push(f);
        }
    }
    if edge_faces.iter().any(|f| f.len() > 2) {
        return Err(PolySplineError::NotManifold);
    }
    let edge_pt: Vec<DVec3> = edge_ends
        .iter()
        .zip(&edge_faces)
        .map(|(&(a, b), fs)| {
            let mid = m.points[a] + m.points[b];
            if fs.len() == 2 { (mid + face_pt[fs[0]] + face_pt[fs[1]]) / 4.0 } else { mid / 2.0 }
        })
        .collect();
    let mut points = Vec::with_capacity(nv + edge_pt.len() + face_pt.len());
    for v in 0..nv {
        let p = m.points[v];
        let fs = &topo.vertex_faces[v];
        if fs.is_empty() {
            points.push(p);
            continue;
        }
        let edges: BTreeSet<usize> = fs
            .iter()
            .flat_map(|f| {
                let poly = &m.faces[*f];
                let i = poly.iter().position(|x| *x == v).unwrap();
                let (a, b) = (poly[(i + poly.len() - 1) % poly.len()], poly[(i + 1) % poly.len()]);
                [edge_id[&(a.min(v), a.max(v))], edge_id[&(b.min(v), b.max(v))]]
            })
            .collect();
        let boundary: Vec<usize> = edges.iter().copied().filter(|e| edge_faces[*e].len() == 1).collect();
        let other = |e: usize| {
            let (a, b) = edge_ends[e];
            m.points[if a == v { b } else { a }]
        };
        points.push(if boundary.is_empty() {
            let n = edges.len() as f64;
            let f = fs.iter().map(|f| face_pt[*f]).sum::<DVec3>() / fs.len() as f64;
            let r = edges.iter().map(|e| (p + other(*e)) / 2.0).sum::<DVec3>() / n;
            (f + 2.0 * r + (n - 3.0) * p) / n
        } else if boundary.len() == 2 {
            (6.0 * p + other(boundary[0]) + other(boundary[1])) / 8.0
        } else {
            p
        });
    }
    points.extend(edge_pt.iter().copied());
    points.extend(face_pt.iter().copied());
    let e_index = |a: usize, b: usize| nv + edge_id[&(a.min(b), a.max(b))];
    let mut faces = Vec::new();
    for (f, poly) in m.faces.iter().enumerate() {
        let k = poly.len();
        for i in 0..k {
            let (prev, v, next) = (poly[(i + k - 1) % k], poly[i], poly[(i + 1) % k]);
            faces.push(vec![v, e_index(v, next), nv + edge_pt.len() + f, e_index(prev, v)]);
        }
    }
    Ok(PolyMesh { points, faces })
}

/// Whether the mesh can be splined as it is: all quadrilaterals, each with
/// at most one extraordinary corner, and the faces just outside each
/// extraordinary vertex's ring all regular.
fn ready(m: &PolyMesh, topo: &Topo) -> Result<bool, PolySplineError> {
    if m.faces.iter().any(|f| f.len() != 4) {
        return Ok(false);
    }
    let ev: Vec<bool> = (0..m.points.len()).map(|v| topo.extraordinary(m, v)).collect();
    for f in &m.faces {
        if f.iter().filter(|v| ev[**v]).count() > 1 {
            return Ok(false);
        }
    }
    for v in (0..m.points.len()).filter(|v| ev[*v]) {
        if !topo.interior(m, v) {
            return Err(PolySplineError::Boundary);
        }
        for &f in &topo.vertex_faces[v] {
            let p = &m.faces[f];
            let i = p.iter().position(|x| *x == v).unwrap();
            // The two edges of the ring face away from v.
            for (a, b) in [(p[(i + 1) % 4], p[(i + 2) % 4]), (p[(i + 2) % 4], p[(i + 3) % 4])] {
                let Some(&g) = topo.directed.get(&(b, a)) else { return Err(PolySplineError::Boundary) };
                if m.faces[g].iter().any(|x| ev[*x]) {
                    return Ok(false);
                }
            }
        }
    }
    Ok(true)
}

/// `f`'s corners starting at `v`.
fn from(f: &[usize], v: usize) -> [usize; 4] {
    let i = f.iter().position(|x| *x == v).unwrap();
    [f[i], f[(i + 1) % 4], f[(i + 2) % 4], f[(i + 3) % 4]]
}

/// The bicubic Bézier patch of a regular quadrilateral: u from its first
/// corner to its second, v from its first to its fourth. Points missing
/// past a boundary are reflected across it.
fn regular_patch(m: &PolyMesh, topo: &Topo, q: &[usize]) -> NurbsSurface {
    let p = |i: usize| m.points[i];
    let mut g = [[None::<usize>; 4]; 4];
    let (v0, v1, v2, v3) = (q[0], q[1], q[2], q[3]);
    g[1][1] = Some(v0);
    g[2][1] = Some(v1);
    g[2][2] = Some(v2);
    g[1][2] = Some(v3);
    // Across each edge: the neighbour's two far corners.
    let across = |a: usize, b: usize| topo.directed.get(&(b, a)).map(|f| from(&m.faces[*f], b));
    if let Some(f) = across(v0, v1) {
        (g[1][0], g[2][0]) = (Some(f[2]), Some(f[3]));
    }
    if let Some(f) = across(v1, v2) {
        (g[3][1], g[3][2]) = (Some(f[2]), Some(f[3]));
    }
    if let Some(f) = across(v2, v3) {
        (g[2][3], g[1][3]) = (Some(f[2]), Some(f[3]));
    }
    if let Some(f) = across(v3, v0) {
        (g[0][2], g[0][1]) = (Some(f[2]), Some(f[3]));
    }
    // Corners: the face at the corner vertex holding both neighbours.
    for (c, a, b, at) in [(v0, g[1][0], g[0][1], (0, 0)), (v1, g[3][1], g[2][0], (3, 0)), (v2, g[2][3], g[3][2], (3, 3)), (v3, g[0][2], g[1][3], (0, 3))] {
        let (Some(a), Some(b)) = (a, b) else { continue };
        if let Some(f) = topo.vertex_faces[c].iter().find(|f| m.faces[**f].contains(&a) && m.faces[**f].contains(&b)) {
            g[at.0][at.1] = m.faces[*f].iter().copied().find(|x| *x != c && *x != a && *x != b);
        }
    }
    let mut pos = [[DVec3::ZERO; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            if let Some(k) = g[i][j] {
                pos[i][j] = p(k);
            }
        }
    }
    // Reflect missing edge rows, then missing corners.
    for k in 1..3 {
        if g[k][0].is_none() {
            pos[k][0] = 2.0 * pos[k][1] - pos[k][2];
        }
        if g[k][3].is_none() {
            pos[k][3] = 2.0 * pos[k][2] - pos[k][1];
        }
        if g[0][k].is_none() {
            pos[0][k] = 2.0 * pos[1][k] - pos[2][k];
        }
        if g[3][k].is_none() {
            pos[3][k] = 2.0 * pos[2][k] - pos[1][k];
        }
    }
    for (i, j, di) in [(0, 0, 1i32), (3, 0, -1), (3, 3, -1), (0, 3, 1)] {
        if g[i][j].is_none() {
            let i1 = (i as i32 + di) as usize;
            let i2 = (i as i32 + 2 * di) as usize;
            pos[i][j] = 2.0 * pos[i1][j] - pos[i2][j];
        }
    }
    // Uniform cubic B-spline to Bézier, in each direction.
    const M: [[f64; 4]; 4] = [[1.0, 4.0, 1.0, 0.0], [0.0, 4.0, 2.0, 0.0], [0.0, 2.0, 4.0, 0.0], [0.0, 1.0, 4.0, 1.0]];
    let mut ctrl = Vec::with_capacity(16);
    for a in 0..4 {
        for b in 0..4 {
            let mut s = DVec3::ZERO;
            for i in 0..4 {
                for j in 0..4 {
                    s += pos[i][j] * (M[a][i] * M[b][j] / 36.0);
                }
            }
            ctrl.push(s.extend(1.0));
        }
    }
    let k = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    NurbsSurface::new(3, 3, k.clone(), k, 4, 4, ctrl)
}

/// Edge `i` of a patch built for corners `q` (from `q[i]` to `q[i + 1]`)
/// as a curve running that way.
fn patch_edge(s: &NurbsSurface, i: usize) -> NurbsCurve {
    match i {
        0 => s.iso_v(0.0),
        1 => s.iso_u(1.0),
        2 => s.iso_v(1.0).reversed(),
        _ => s.iso_u(0.0).reversed(),
    }
}

/// Splines a polygon mesh: a solid when the mesh is closed, else a sheet.
pub fn polysplines(mesh: &PolyMesh) -> Result<Body, PolySplineError> {
    let mut m = mesh.clone();
    let mut topo = Topo::new(&m)?;
    let mut steps = 0;
    while !ready(&m, &topo)? {
        if steps == 3 {
            return Err(PolySplineError::Crowded);
        }
        m = catmull_clark(&m)?;
        topo = Topo::new(&m)?;
        steps += 1;
    }
    let ev: Vec<bool> = (0..m.points.len()).map(|v| topo.extraordinary(&m, v)).collect();
    // Regular patches, by face.
    let mut patches: HashMap<usize, NurbsSurface> = HashMap::new();
    for (f, q) in m.faces.iter().enumerate() {
        if !q.iter().any(|v| ev[*v]) {
            patches.insert(f, regular_patch(&m, &topo, q));
        }
    }
    let size = m.points.iter().fold(1.0f64, |a, p| a.max(p.abs().max_element()));
    let closed = topo.directed.keys().all(|(a, b)| topo.directed.contains_key(&(*b, *a)));
    let mut sw = crate::ops::sheet::Sewer::new(if closed { BodyKind::Solid } else { BodyKind::Sheet }, 1e-7 * size);
    let mut faces: Vec<usize> = patches.keys().copied().collect();
    faces.sort();
    for f in faces {
        sw.patch(patches[&f].clone());
    }
    // Fills round extraordinary vertices.
    for v in (0..m.points.len()).filter(|v| ev[*v]) {
        // The ring in order: face k runs v, e_k, d_k, e_(k+1).
        let mut ring = vec![from(&m.faces[topo.vertex_faces[v][0]], v)];
        loop {
            let last = ring.last().unwrap();
            let next = from(&m.faces[topo.directed[&(v, last[3])]], v);
            if next == ring[0] {
                break;
            }
            if ring.len() > topo.vertex_faces[v].len() {
                return Err(PolySplineError::NotManifold);
            }
            ring.push(next);
        }
        let ns = ring.len();
        // The hole's edges: each the edge of the regular patch beyond it,
        // that patch, which of its edges it is, and whether the curve runs
        // against that edge.
        let outer = |a: usize, b: usize, flip: bool| -> Option<(NurbsCurve, &NurbsSurface, usize, bool)> {
            let g = *topo.directed.get(&(b, a))?;
            let s = patches.get(&g)?;
            let i = m.faces[g].iter().position(|x| *x == b)?;
            let c = uniform(&patch_edge(s, i));
            Some(if flip { (c.reversed(), s, i, true) } else { (c, s, i, false) })
        };
        let mut sides = Vec::with_capacity(2 * ns);
        for here in &ring {
            // Corner k is d_k: on along to e_(k+1), back to e_k (both
            // against the patches' own edges, which run the other way).
            sides.push(outer(here[2], here[3], true).ok_or(PolySplineError::Fit)?);
            sides.push(outer(here[1], here[2], false).ok_or(PolySplineError::Fit)?);
        }
        let cs: Vec<NurbsCurve> = sides.iter().map(|s| s.0.clone()).collect();
        let cross = |e: usize, t: f64| -> Option<(DVec3, DVec3)> {
            let (c, s, i, flip) = &sides[e];
            let w = if *flip { 1.0 - t } else { t };
            let (u, v) = match i {
                0 => (w, 0.0),
                1 => (1.0, w),
                2 => (1.0 - w, 1.0),
                _ => (0.0, 1.0 - w),
            };
            let d = s.derivs(u, v, 1);
            let n = d[1][0].cross(d[0][1]).try_normalize()?;
            let mut into = n.cross(c.tangent(t)).try_normalize()?;
            if into.dot(s.point(0.5, 0.5) - c.point(t)) > 0.0 {
                into = -into;
            }
            Some((n, into))
        };
        for s in crate::ops::sheet::fill_quads(&cs, &cross).ok_or(PolySplineError::Fit)? {
            sw.patch(s);
        }
    }
    Ok(sw.body)
}

/// A Bézier cubic on [0, 1] with knots inserted at quarters (seven
/// control points, enough for a fill this small), the same curve exactly.
fn uniform(c: &NurbsCurve) -> NurbsCurve {
    let mut c = c.reparameterized(0.0, 1.0);
    for k in 1..4 {
        c = c.insert_knot(k as f64 / 4.0, 1);
    }
    c
}

/// A mesh body's triangles as a polygon mesh.
pub fn from_triangles(points: &[DVec3], tris: &[[u32; 3]]) -> PolyMesh {
    let faces = tris.iter().map(|t| t.iter().map(|i| *i as usize).collect()).collect();
    PolyMesh { points: points.to_vec(), faces }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ops::boolean::open_edges;

    fn cube() -> PolyMesh {
        let points = (0..8).map(|i| DVec3::new((i & 1) as f64, ((i >> 1) & 1) as f64, ((i >> 2) & 1) as f64)).collect();
        let faces = vec![vec![0, 2, 3, 1], vec![4, 5, 7, 6], vec![0, 1, 5, 4], vec![2, 6, 7, 3], vec![0, 4, 6, 2], vec![1, 3, 7, 5]];
        PolyMesh { points, faces }
    }

    #[test]
    fn subdivides_a_cube() {
        let c = catmull_clark(&cube()).unwrap();
        assert_eq!((c.points.len(), c.faces.len()), (26, 24));
        // Corners move in to (F + 2R) / 3: 2/9 of the way along each axis.
        for p in &c.points[..8] {
            assert!((p - DVec3::splat(0.5)).abs().distance(DVec3::splat(0.5 - 2.0 / 9.0)) < 1e-12, "{p}");
        }
    }

    #[test]
    fn splines_a_cube_into_a_closed_smooth_solid() {
        let b = polysplines(&cube()).unwrap();
        assert_eq!(b.kind, BodyKind::Solid);
        assert_eq!(open_edges(&b), 0);
        // Every corner is a three-valent extraordinary vertex; after one
        // step each sub-face still touches one, so it takes two: 96
        // quadrilaterals, of which the 8 rings of 3 are filled.
        assert_eq!(b.faces.len(), 96);
        let kink = worst_kink(&b);
        assert!(kink < 0.2, "seams kink by {kink} degrees");
    }

    /// The largest angle (degrees) between the normals of the two faces
    /// at any edge, sampled along each.
    fn worst_kink(b: &Body) -> f64 {
        let mut worst: f64 = 0.0;
        for e in b.edge_ids() {
            let fs = b.edge_faces(e);
            if fs.len() != 2 {
                continue;
            }
            let ed = b.edge(e);
            for k in 1..8 {
                let p = ed.curve.point(ed.t0 + (ed.t1 - ed.t0) * k as f64 / 8.0);
                let n: Vec<DVec3> = fs
                    .iter()
                    .map(|(f, _)| {
                        let uv = b.face(*f).surface.project(p, None);
                        b.face_normal(*f, uv.x, uv.y)
                    })
                    .collect();
                worst = worst.max(n[0].dot(n[1]).clamp(-1.0, 1.0).acos().to_degrees());
            }
        }
        worst
    }

    #[test]
    fn splines_a_triangle_mesh() {
        // An octahedron: eight triangles, four meeting at each corner.
        let points = vec![DVec3::X, -DVec3::X, DVec3::Y, -DVec3::Y, DVec3::Z, -DVec3::Z];
        let faces = vec![vec![0, 2, 4], vec![2, 1, 4], vec![1, 3, 4], vec![3, 0, 4], vec![2, 0, 5], vec![1, 2, 5], vec![3, 1, 5], vec![0, 3, 5]];
        let b = polysplines(&PolyMesh { points, faces }).unwrap();
        assert_eq!(b.kind, BodyKind::Solid);
        assert_eq!(open_edges(&b), 0);
        assert!(worst_kink(&b) < 0.2, "{}", worst_kink(&b));
    }

    #[test]
    fn splines_a_torus_cage_into_curvature_continuous_patches() {
        let (nu, nv) = (8, 4);
        let mut points = Vec::new();
        for i in 0..nu {
            for j in 0..nv {
                let (a, c) = (i as f64 * std::f64::consts::TAU / nu as f64, j as f64 * std::f64::consts::TAU / nv as f64);
                let r = 3.0 + 1.2 * c.cos();
                points.push(DVec3::new(r * a.cos(), r * a.sin(), 1.2 * c.sin()));
            }
        }
        let id = |i: usize, j: usize| (i % nu) * nv + (j % nv);
        let faces = (0..nu).flat_map(|i| (0..nv).map(move |j| vec![id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1)])).collect();
        let b = polysplines(&PolyMesh { points, faces }).unwrap();
        // All regular: one patch per quadrilateral, nothing subdivided.
        assert_eq!(b.faces.len(), 32);
        assert_eq!(open_edges(&b), 0);
        assert!(worst_kink(&b) < 1e-4, "{}", worst_kink(&b));
    }
}
