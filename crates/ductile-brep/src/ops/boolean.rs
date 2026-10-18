// SPDX-License-Identifier: GPL-3.0-only
//! Exact boolean operations between solids.
//!
//! The algorithm works on one merged working body:
//! 1. edges of each operand are cut where they cross faces of the other;
//! 2. every pair of overlapping faces is intersected and the parts of the
//!    intersection curves lying inside both faces become shared edges
//!    (coincident faces imprint each other's boundaries instead);
//! 3. each face is split along its new edges by a planar walk in UV space;
//! 4. fragments are classified against the other operand and kept or
//!    dropped according to the operation;
//! 5. the kept fragments are stitched into a closed body.

use crate::ssi::{self, Ssi};
use crate::tess::{face_coedge_uvs, face_uv_flipped, face_uv_loops, is_singular, tessellate, tessellate_edge, BodyMesh, FaceMesh, TessOptions};
use crate::topo::{Body, BodyKind, Coedge, EdgeId, FaceId, Loop, VertexId};
use ductile_geom::poly2::{point_in_loops, point_in_polygon, signed_area};
use ductile_geom::{BBox3, Curve3, DVec2, DVec3, Surface};
use std::collections::{BTreeSet, HashMap};
use std::f64::consts::TAU;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoolOp {
    Union,
    Intersection,
    /// First operand minus the second.
    Difference,
}

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum BoolError {
    #[error("boolean operands must be solids")]
    NotSolid,
    #[error("the result is empty")]
    Empty,
    #[error("the result is not closed ({0} open edges)")]
    Open(usize),
}

/// Position of a point relative to a trimmed face.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Pos {
    In,
    On,
    Out,
}

/// Point-in-face classifier that does not depend on topology ids.
struct Zone {
    surf: Surface,
    loops: Vec<Vec<DVec2>>,
    center: DVec2,
    bbox: BBox3,
    boundary: Vec<(Curve3, f64, f64, BBox3)>,
}

impl Zone {
    fn new(w: &Body, f: FaceId, samples: &[Vec<(f64, DVec3)>], bbox: BBox3, tol: f64) -> Zone {
        let face = w.face(f);
        let loops: Vec<Vec<DVec2>> = face_uv_loops(w, f, samples).into_iter().map(|l| l.into_iter().map(|s| s.uv).collect()).collect();
        let (lo, hi) = uv_range(loops.iter().flatten());
        let boundary = w
            .face_edges(f)
            .into_iter()
            .map(|e| {
                let ed = w.edge(e);
                (ed.curve.clone(), ed.t0, ed.t1, ed.curve.bbox(ed.t0, ed.t1).expanded(tol * 20.0))
            })
            .collect();
        Zone { surf: face.surface.clone(), loops, center: (lo + hi) * 0.5, bbox, boundary }
    }

    fn classify(&self, p: DVec3, tol: f64) -> Pos {
        if !self.bbox.contains(p) || ssi::distance(&self.surf, p) > tol * 10.0 {
            return Pos::Out;
        }
        for (c, t0, t1, bb) in &self.boundary {
            if bb.contains(p) {
                let t = c.closest_param_in(p, *t0, *t1);
                if c.point(t).distance(p) < tol * 10.0 {
                    return Pos::On;
                }
            }
        }
        if self.loops.is_empty() {
            return Pos::In;
        }
        let uv = self.surf.align_periodic(self.surf.project(p, None), self.center);
        if point_in_loops(uv, &self.loops) {
            Pos::In
        } else {
            Pos::Out
        }
    }
}

/// `p` moved onto the intersection of two surfaces it lies close to, by
/// projecting onto each in turn (which converges where they cross).
fn onto_both(a: &Surface, b: &Surface, mut p: DVec3) -> DVec3 {
    let (mut ha, mut hb) = (None, None);
    for _ in 0..12 {
        let ua = a.project(p, ha);
        ha = Some(ua);
        p = a.point(ua.x, ua.y);
        let ub = b.project(p, hb);
        hb = Some(ub);
        let q = b.point(ub.x, ub.y);
        let step = q.distance(p);
        p = q;
        if step < 1e-13 {
            break;
        }
    }
    p
}

fn uv_range<'a>(pts: impl Iterator<Item = &'a DVec2>) -> (DVec2, DVec2) {
    pts.fold((DVec2::splat(f64::INFINITY), DVec2::splat(f64::NEG_INFINITY)), |(a, b), p| (a.min(*p), b.max(*p)))
}

fn mesh_bbox(m: &FaceMesh, pad: f64) -> BBox3 {
    BBox3::from_points(m.positions.iter().copied()).expanded(pad)
}

/// Outward normal of face `f` at a point on it.
fn outward_normal(w: &Body, f: FaceId, p: DVec3) -> DVec3 {
    let face = w.face(f);
    let uv = face.surface.project(p, None);
    w.face_normal(f, uv.x, uv.y)
}

/// Computes `a op b`.
pub fn boolean(a: &Body, b: &Body, op: BoolOp) -> Result<Body, BoolError> {
    if a.kind != BodyKind::Solid || b.kind != BodyKind::Solid {
        return Err(BoolError::NotSolid);
    }
    // A whole sphere whose poles or seam lie on the other body's surfaces
    // is turned so they keep clear: the same solid, without a needlessly
    // degenerate intersection.
    let (ra, rb) = (reseamed(a, b), reseamed(b, a));
    if ra.is_some() || rb.is_some() {
        return boolean(ra.as_ref().unwrap_or(a), rb.as_ref().unwrap_or(b), op);
    }
    let diag = a.bbox().union(&b.bbox()).diagonal().max(1e-9);
    let tol = ductile_geom::LINEAR_TOL * (diag / 10.0).max(0.01);
    let mesh_opts = TessOptions::for_size(diag);
    let fine_opts = TessOptions { chord_tol: (diag * 2e-5).max(tol * 10.0), angle_tol: 0.1 };
    let timing = std::env::var_os("DUCTILE_TIME").is_some();
    let mut clock = std::time::Instant::now();
    let mut mark = |what: &str| {
        if timing {
            eprintln!("  boolean {what}: {:?}", clock.elapsed());
            clock = std::time::Instant::now();
        }
    };
    let mesh_a = tessellate(a, &mesh_opts);
    let mesh_b = tessellate(b, &mesh_opts);
    mark("mesh");
    let na_faces = a.faces.len();
    let na_edges = a.edges.len();
    let mut w = a.clone();
    w.append(b);
    // B's vertices coinciding with A's vertices become A's.
    let na_verts = a.vertices.len();
    let remap: Vec<Option<VertexId>> = (na_verts..w.vertices.len())
        .map(|i| {
            let p = w.vertices[i].point;
            (0..na_verts).map(|j| (w.vertices[j].point.distance(p), j)).filter(|x| x.0 < tol * 10.0).min_by(|x, y| x.0.total_cmp(&y.0)).map(|x| VertexId(x.1 as u32))
        })
        .collect();
    for ed in w.edges.iter_mut().skip(na_edges) {
        for v in [&mut ed.v0, &mut ed.v1] {
            if v.idx() >= na_verts {
                if let Some(t) = remap[v.idx() - na_verts] {
                    *v = t;
                }
            }
        }
    }
    let from_a = |f: FaceId| f.idx() < na_faces;
    let face_mesh = |f: FaceId| if f.idx() < na_faces { &mesh_a.faces[f.idx()] } else { &mesh_b.faces[f.idx() - na_faces] };
    let pad = mesh_opts.chord_tol * 4.0 + tol * 20.0;
    let fbox: Vec<BBox3> = w.face_ids().map(|f| mesh_bbox(face_mesh(f), pad)).collect();

    // Point classifiers of the original faces.
    let samples: Vec<Vec<(f64, DVec3)>> = w.edge_ids().map(|e| tessellate_edge(&w, e, &fine_opts)).collect();
    let zones: Vec<Zone> = w.face_ids().map(|f| Zone::new(&w, f, &samples, fbox[f.idx()], tol)).collect();

    mark("zones");
    // 1. Edge / face crossings.
    let mut crossings: Vec<(EdgeId, DVec3)> = Vec::new();
    for e in w.edge_ids() {
        let ea = e.idx() < na_edges;
        let edge = w.edge(e).clone();
        let ebox = edge.curve.bbox(edge.t0, edge.t1).expanded(tol * 20.0);
        for f in w.face_ids() {
            if from_a(f) == ea || !ebox.intersects(&fbox[f.idx()]) {
                continue;
            }
            let s = &w.face(f).surface;
            // Uneven fractions, so a closed curve crossing a surface at its
            // start and half-way round is not mistaken for lying in it.
            let on = [0.382, 0.618, 0.173, 0.851, 0.0, 1.0]
                .iter()
                .all(|&k| ssi::distance(s, edge.curve.point(edge.t0 + (edge.t1 - edge.t0) * k)) < tol * 10.0);
            if on {
                continue;
            }
            for t in ssi::curve_surface_crossings(&edge.curve, edge.t0, edge.t1, s, tol) {
                let mut p = edge.curve.point(t);
                // Where the edge only touches the surface (as along a
                // blend's flat contact) the crossing is ill-conditioned;
                // a vertex of the face lying on the edge nearby is the
                // intended point.
                // The edge's own ends count when they lie on the surface.
                let close = |q: &DVec3| q.distance(p) < diag * 1e-3;
                let ends = [edge.v0, edge.v1].into_iter().map(|v| w.vertex(v)).filter(|q| close(q) && ssi::distance(s, *q) < tol * 10.0);
                let near = w
                    .face_edges(f)
                    .into_iter()
                    .flat_map(|fe| [w.edge(fe).v0, w.edge(fe).v1])
                    .map(|v| w.vertex(v))
                    .filter(|q| close(q) && edge.curve.point(edge.curve.closest_param_in(*q, edge.t0, edge.t1)).distance(*q) < tol * 10.0)
                    .chain(ends)
                    .min_by(|a, b| a.distance(p).total_cmp(&b.distance(p)));
                if let Some(q) = near {
                    p = q;
                }
                if zones[f.idx()].classify(p, tol) != Pos::Out {
                    crossings.push((e, p));
                }
            }
        }
    }
    // Edges of the two operands crossing each other (as where boundaries
    // imprint on a shared plane): computed on the curves themselves, since
    // the same point found as an edge touching a tangent face is not.
    for ea in (0..na_edges).map(|i| EdgeId(i as u32)) {
        for eb in (na_edges..w.edges.len()).map(|i| EdgeId(i as u32)) {
            if let Some(p) = edge_edge_crossing(&w, ea, eb, &samples, tol) {
                crossings.push((ea, p));
                crossings.push((eb, p));
            }
        }
    }
    let mut splits: HashMap<EdgeId, Vec<(f64, VertexId)>> = HashMap::new();
    let mut crossing_points: Vec<DVec3> = Vec::new();
    for (e, p) in crossings {
        let v = find_vertex(&w, p, tol * 10.0).unwrap_or_else(|| w.add_vertex(p));
        crossing_points.push(w.vertex(v));
        let edge = w.edge(e);
        if v == edge.v0 || v == edge.v1 {
            continue;
        }
        let t = edge.curve.closest_param_in(w.vertex(v), edge.t0, edge.t1);
        splits.entry(e).or_default().push((t, v));
    }
    // Crossing vertices lying inside other edges (tangent touches, edges
    // meeting inside coplanar faces) split those edges too.
    // Vertices of either operand lying inside the other's edges count too.
    let mut new_verts: Vec<VertexId> = crossing_points.iter().filter_map(|p| find_vertex(&w, *p, tol * 10.0)).collect();
    new_verts.extend(w.edges.iter().flat_map(|e| [e.v0, e.v1]));
    new_verts.sort();
    new_verts.dedup();
    for e in w.edge_ids() {
        let edge = w.edge(e).clone();
        let bb = edge.curve.bbox(edge.t0, edge.t1).expanded(tol * 20.0);
        for &v in &new_verts {
            let p = w.vertex(v);
            if v == edge.v0 || v == edge.v1 || !bb.contains(p) {
                continue;
            }
            let t = edge.curve.closest_param_in(p, edge.t0, edge.t1);
            if edge.curve.point(t).distance(p) < tol * 10.0 {
                splits.entry(e).or_default().push((t, v));
            }
        }
    }
    for list in splits.values_mut() {
        list.sort_by(|a, b| a.0.total_cmp(&b.0));
        list.dedup_by_key(|x| x.1);
    }
    apply_splits(&mut w, splits);

    mark("crossings");
    // 2. Face / face intersections.
    let mut edge_index: HashMap<(u32, u32), Vec<EdgeId>> = HashMap::new();
    for e in w.edge_ids() {
        edge_index.entry(edge_key(&w, e)).or_default().push(e);
    }
    let nfaces = w.faces.len();
    let mut extras: Vec<BTreeSet<EdgeId>> = vec![BTreeSet::new(); nfaces];
    let mut coincident: Vec<(FaceId, FaceId)> = Vec::new();
    let face_ids: Vec<FaceId> = w.face_ids().collect();
    for &fa in face_ids.iter().filter(|f| from_a(**f)) {
        for &fb in face_ids.iter().filter(|f| !from_a(**f)) {
            if !fbox[fa.idx()].intersects(&fbox[fb.idx()]) {
                continue;
            }
            let sa = w.face(fa).surface.clone();
            let sb = w.face(fb).surface.clone();
            let roi = intersect_boxes(&fbox[fa.idx()], &fbox[fb.idx()]);
            let res = match ssi::intersect_analytic(&sa, &sb, &roi, tol) {
                Some(r) => r,
                None => {
                    let mut seeds: Vec<DVec3> = crossing_points.iter().copied().filter(|p| roi.contains(*p)).collect();
                    seeds.extend(mesh_seeds(face_mesh(fa), &sb, &roi, 400));
                    seeds.extend(mesh_seeds(face_mesh(fb), &sa, &roi, 400));
                    let mut ends: Vec<DVec3> = Vec::new();
                    for e in w.face_edges(fa).into_iter().chain(w.face_edges(fb)) {
                        ends.push(w.vertex(w.edge(e).v0));
                        ends.push(w.vertex(w.edge(e).v1));
                    }
                    ends.extend(crossing_points.iter().copied().filter(|p| roi.contains(*p)));
                    let r = ssi::intersect_marching_to(&sa, &sb, &roi, &seeds, &ends, tol);
                    if debug() {
                        let n = if let Ssi::Curves(c) = &r { c.len() } else { 0 };
                        eprintln!("march faces {} x {}: {} seeds -> {} curves", fa.idx(), fb.idx(), seeds.len(), n);
                        if let Ssi::Curves(cs) = &r {
                            for (c, c0, c1) in cs {
                                eprintln!("   curve {:.4} -> {:.4}", c.point(*c0), c.point(*c1));
                            }
                        }
                    }
                    r
                }
            };
            match res {
                Ssi::None => {}
                Ssi::Coincident => coincident.push((fa, fb)),
                Ssi::Curves(curves) => {
                    let mut verts: BTreeSet<VertexId> = BTreeSet::new();
                    for e in w.face_edges(fa).into_iter().chain(w.face_edges(fb)) {
                        verts.insert(w.edge(e).v0);
                        verts.insert(w.edge(e).v1);
                    }
                    let verts: Vec<VertexId> = verts.into_iter().collect();
                    for (c, c0, c1) in curves {
                        for (ta, tb, va, vb) in curve_pieces(&mut w, &c, c0, c1, &verts, tol) {
                            // The traced curve only approximates the section;
                            // classify a point brought onto both surfaces.
                            let mid = onto_both(&sa, &sb, c.point(0.5 * (ta + tb)));
                            let pa = zones[fa.idx()].classify(mid, tol);
                            let pb = zones[fb.idx()].classify(mid, tol);
                            if debug() {
                                eprintln!("   piece {}x{} v{}->v{} mid {mid:.4}: {pa:?} {pb:?}", fa.idx(), fb.idx(), va.0, vb.0);
                            }
                            if pa == Pos::Out || pb == Pos::Out || (pa == Pos::On && pb == Pos::On) {
                                continue;
                            }
                            let e = find_edge(&w, &edge_index, va, vb, mid, tol).unwrap_or_else(|| {
                                let e = w.add_edge(c.clone(), ta, tb, va, vb);
                                edge_index.entry(edge_key(&w, e)).or_default().push(e);
                                e
                            });
                            for (f, pos) in [(fa, pa), (fb, pb)] {
                                if pos == Pos::In && !w.face_edges(f).contains(&e) {
                                    extras[f.idx()].insert(e);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    mark("ssi");
    // Coincident faces imprint each other's boundary edges.
    for &(fa, fb) in &coincident {
        for (src, dst) in [(fb, fa), (fa, fb)] {
            for e in w.face_edges(src) {
                if zones[dst.idx()].classify(w.edge(e).mid(), tol) == Pos::In {
                    extras[dst.idx()].insert(e);
                }
            }
        }
    }

    // Drop imprints duplicating an edge already imprinted on the same face.
    for ex in extras.iter_mut() {
        let mut kept: Vec<EdgeId> = Vec::new();
        for &e in ex.iter() {
            let mid = w.edge(e).mid();
            let dup = kept.iter().any(|&k| {
                let ed = w.edge(k);
                edge_key(&w, k) == edge_key(&w, e) && ed.curve.point(ed.curve.closest_param_in(mid, ed.t0, ed.t1)).distance(mid) < tol * 100.0
            });
            if !dup {
                kept.push(e);
            }
        }
        *ex = kept.into_iter().collect();
    }
    // Intersection curves crossing each other inside a face (where the
    // faces touch at a point and the section crosses itself, as a plane
    // tangent to a torus's inner rim) meet at a shared vertex there.
    {
        let samples: Vec<Vec<(f64, DVec3)>> = w.edge_ids().map(|e| tessellate_edge(&w, e, &fine_opts)).collect();
        let mut splits: HashMap<EdgeId, Vec<(f64, VertexId)>> = HashMap::new();
        for ex in &extras {
            let list: Vec<EdgeId> = ex.iter().copied().collect();
            for i in 0..list.len() {
                for j in i + 1..list.len() {
                    let Some(p) = edge_edge_crossing(&w, list[i], list[j], &samples, tol) else { continue };
                    let v = find_vertex(&w, p, tol * 10.0).unwrap_or_else(|| w.add_vertex(p));
                    for e in [list[i], list[j]] {
                        let ed = w.edge(e);
                        if v != ed.v0 && v != ed.v1 {
                            let t = ed.curve.closest_param_in(w.vertex(v), ed.t0, ed.t1);
                            splits.entry(e).or_default().push((t, v));
                        }
                    }
                }
            }
        }
        for list in splits.values_mut() {
            list.sort_by(|a, b| a.0.total_cmp(&b.0));
            list.dedup_by_key(|x| x.1);
        }
        if !splits.is_empty() {
            let pieces = apply_splits(&mut w, splits);
            for ex in extras.iter_mut() {
                let grown: Vec<EdgeId> = ex.iter().filter_map(|e| pieces.get(e)).flatten().copied().collect();
                ex.extend(grown);
            }
        }
    }
    if debug() {
        eprintln!("crossings {} coincident {:?}", crossing_points.len(), coincident);
        for p in &crossing_points {
            eprintln!("   crossing at {p:.5}");
        }
        for (f, ex) in extras.iter().enumerate() {
            if !ex.is_empty() {
                let d: Vec<String> = ex.iter().map(|e| { let ed = w.edge(*e); format!("e{}[v{}{:.4}->v{}{:.4}]", e.0, ed.v0.0, w.vertex(ed.v0), ed.v1.0, w.vertex(ed.v1)) }).collect();
                eprintln!("face {f} ({:?}) extras {}", std::mem::discriminant(&w.faces[f].surface), d.join(" "));
            }
        }
    }
    // 3. Split faces.
    let samples: Vec<Vec<(f64, DVec3)>> = w.edge_ids().map(|e| tessellate_edge(&w, e, &fine_opts)).collect();
    struct Frag {
        orig: FaceId,
        loops: Vec<Vec<Coedge>>,
        /// Index into the mesh of split fragments, if this is one.
        mesh: Option<usize>,
    }
    let mut frags: Vec<Frag> = Vec::new();
    for f in w.face_ids() {
        let ex: Vec<EdgeId> = extras[f.idx()].iter().copied().collect();
        let split = if ex.is_empty() { None } else { split_face(&w, f, &ex, &samples) };
        match split {
            Some(faces) => {
                if debug() {
                    eprintln!("face {} split into {} fragments", f.idx(), faces.len());
                }
                frags.extend(faces.into_iter().map(|loops| Frag { orig: f, loops, mesh: Some(0) }))
            }
            None => {
                if !ex.is_empty() {
                    log::warn!("boolean: could not split face {}", f.idx());
                }
                let loops = w.face_loops(f).map(|l| l.coedges.clone()).collect();
                frags.push(Frag { orig: f, loops, mesh: None });
            }
        }
    }

    mark("split");
    // 4. Classify fragments.
    // Only split fragments need meshing; whole faces keep their meshes.
    let mut fb_body = Body { kind: BodyKind::Sheet, vertices: w.vertices.clone(), edges: w.edges.clone(), loops: vec![], faces: vec![] };
    for fr in frags.iter_mut() {
        if fr.mesh.is_some() {
            let face = w.face(fr.orig);
            fr.mesh = Some(fb_body.add_face(face.surface.clone(), face.reversed, fr.loops.clone()).idx());
        }
    }
    let frag_mesh = tessellate(&fb_body, &mesh_opts);
    mark("fragment mesh");
    let mut keep: Vec<(usize, bool)> = Vec::new();
    for (i, fr) in frags.iter().enumerate() {
        let m = match fr.mesh {
            Some(k) => &frag_mesh.faces[k],
            None => face_mesh(fr.orig),
        };
        let Some(p) = interior_point(m) else { continue };
        // Triangle centroids sag below curved surfaces: snap onto the face.
        let surf = &w.face(fr.orig).surface;
        let uv = surf.project(p, None);
        let p = surf.point(uv.x, uv.y);
        let fa = from_a(fr.orig);
        let mut cls = None;
        for &(x, y) in &coincident {
            let partner = if x == fr.orig {
                y
            } else if y == fr.orig {
                x
            } else {
                continue;
            };
            if zones[partner.idx()].classify(p, tol) != Pos::Out {
                let same = outward_normal(&w, fr.orig, p).dot(outward_normal(&w, partner, p)) > 0.0;
                cls = Some(if same { Class::OnSame } else { Class::OnOpp });
                break;
            }
        }
        let cls = cls.unwrap_or_else(|| {
            // Close to the other body (a sliver beside a tangent contact)
            // its mesh is too coarse to tell; its exact faces are not.
            let near = face_ids.iter().filter(|f| from_a(**f) != fa).filter_map(|&f| {
                let s = &w.face(f).surface;
                let uv = s.project(p, None);
                let q = s.point(uv.x, uv.y);
                (zones[f.idx()].classify(q, tol) == Pos::In).then(|| (q.distance(p), f, q))
            });
            if let Some((_, f, q)) = near.min_by(|a, b| a.0.total_cmp(&b.0)).filter(|n| n.0 < diag * 1e-2 && n.0 > tol * 10.0) {
                return if (p - q).dot(outward_normal(&w, f, q)) < 0.0 { Class::In } else { Class::Out };
            }
            let other = if fa { &mesh_b } else { &mesh_a };
            if point_in_mesh(p, other) {
                Class::In
            } else {
                Class::Out
            }
        });
        let decision = match (op, fa, cls) {
            (BoolOp::Union, true, Class::Out | Class::OnSame) => Some(false),
            (BoolOp::Union, false, Class::Out) => Some(false),
            (BoolOp::Intersection, true, Class::In | Class::OnSame) => Some(false),
            (BoolOp::Intersection, false, Class::In) => Some(false),
            (BoolOp::Difference, true, Class::Out | Class::OnOpp) => Some(false),
            (BoolOp::Difference, false, Class::In) => Some(true),
            _ => None,
        };
        if debug() {
            eprintln!("frag {i} of face {} {:?} at {p:.3} -> {:?}", fr.orig.idx(), cls, decision);
        }
        if let Some(flip) = decision {
            keep.push((i, flip));
        }
    }
    if keep.is_empty() {
        return Err(BoolError::Empty);
    }

    mark("classify");
    // 5. Assemble.
    let faces: Vec<(Surface, bool, Vec<Vec<Coedge>>)> = keep
        .into_iter()
        .map(|(i, flip)| {
            let fr = &frags[i];
            let face = w.face(fr.orig);
            if flip {
                let loops = fr.loops.iter().map(|l| l.iter().rev().map(|c| Coedge::new(c.edge, !c.reversed)).collect()).collect();
                (face.surface.clone(), !face.reversed, loops)
            } else {
                (face.surface.clone(), face.reversed, fr.loops.clone())
            }
        })
        .collect();
    let body = super::heal::simplify(&assemble(&w, faces, tol));
    mark("assemble");
    let open = open_edges(&body);
    if debug() && open > 0 {
        let mut uses: HashMap<EdgeId, Vec<(usize, bool)>> = HashMap::new();
        for (fi, f) in body.faces.iter().enumerate() {
            for l in &f.loops {
                for c in &body.loops[l.idx()].coedges {
                    uses.entry(c.edge).or_default().push((fi, c.reversed));
                }
            }
        }
        for (e, u) in uses {
            if u.len() != 2 || u[0].1 == u[1].1 {
                let ed = body.edge(e);
                eprintln!("open edge e{} {:.3}->{:.3} uses {:?}", e.0, ed.start(), ed.end(), u);
            }
        }
    }
    if open > 0 {
        return Err(BoolError::Open(open));
    }
    Ok(body)
}

/// `a` rebuilt with its poles and seam turned away from the surfaces of
/// `other`, when `a` is a whole sphere and they lie on or near them.
fn reseamed(a: &Body, other: &Body) -> Option<Body> {
    use ductile_geom::{DQuat, Frame};
    if a.faces.len() != 1 {
        return None;
    }
    let Surface::Sphere { frame, radius } = &a.faces[0].surface else { return None };
    let r = *radius;
    // The poles and points down the seam, and how far the nearest is from
    // any surface of `other`.
    let clearance = |f: &Frame| {
        let pts: Vec<DVec3> = (0..=8).map(|k| {
            let v = -std::f64::consts::FRAC_PI_2 + std::f64::consts::PI * k as f64 / 8.0;
            f.to_world(DVec3::new(r * v.cos(), 0.0, r * v.sin()))
        }).collect();
        other.faces.iter().flat_map(|face| pts.iter().map(move |p| {
            let uv = face.surface.project(*p, None);
            face.surface.point(uv.x, uv.y).distance(*p)
        })).fold(f64::MAX, f64::min)
    };
    let margin = 1e-3 * r;
    if clearance(frame) > margin {
        return None;
    }
    let turned = (1..24).map(|k| {
        let q = DQuat::from_axis_angle(DVec3::new(0.3, 0.5, 0.81).normalize(), 0.37 * k as f64);
        Frame { origin: frame.origin, x: q * frame.x, y: q * frame.y, z: q * frame.z }
    });
    let best = turned.map(|f| (clearance(&f), f)).max_by(|x, y| x.0.total_cmp(&y.0))?;
    (best.0 > margin).then(|| {
        let mut s = crate::primitives::make_sphere_in(&best.1, r);
        if a.faces[0].reversed != s.faces[0].reversed {
            s = crate::primitives::flip_orientation(s);
        }
        s
    })
}

fn debug() -> bool {
    std::env::var_os("DUCTILE_DEBUG_BOOL").is_some()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Class {
    In,
    Out,
    OnSame,
    OnOpp,
}

/// Point where two edges cross transversally away from their ends, if
/// they do: the nearest pair of their sample polylines, refined by Newton
/// on the curves.
fn edge_edge_crossing(w: &Body, ea: EdgeId, eb: EdgeId, samples: &[Vec<(f64, DVec3)>], tol: f64) -> Option<DVec3> {
    let (sa, sb) = (&samples[ea.idx()], &samples[eb.idx()]);
    let bb = |s: &[(f64, DVec3)]| BBox3::from_points(s.iter().map(|x| x.1)).expanded(tol * 20.0);
    if !bb(sa).intersects(&bb(sb)) {
        return None;
    }
    let (a, b) = (w.edge(ea), w.edge(eb));
    // Shared or touching ends are handled as vertices already.
    for v in [a.v0, a.v1] {
        if v == b.v0 || v == b.v1 {
            return None;
        }
    }
    let mut best: Option<(f64, f64, f64)> = None;
    for i in 0..sa.len().saturating_sub(1) {
        for j in 0..sb.len().saturating_sub(1) {
            let (p0, p1, q0, q1) = (sa[i].1, sa[i + 1].1, sb[j].1, sb[j + 1].1);
            let (s, t) = segment_closest(p0, p1, q0, q1);
            let d = p0.lerp(p1, s).distance(q0.lerp(q1, t));
            if best.map_or(true, |x| d < x.0) {
                best = Some((d, sa[i].0 + (sa[i + 1].0 - sa[i].0) * s, sb[j].0 + (sb[j + 1].0 - sb[j].0) * t));
            }
        }
    }
    let (d, mut s, mut t) = best?;
    let span = |e: &crate::topo::Edge| (e.t1 - e.t0).abs();
    if d > (span(a).max(span(b)) * 1e-2).max(tol * 1e3) {
        return None;
    }
    for _ in 0..30 {
        let (da, db) = (a.curve.derivs(s), b.curve.derivs(t));
        let r = da[0] - db[0];
        // Gauss-Newton on |a(s) - b(t)|².
        let (aa, ab, bb2) = (da[1].dot(da[1]), -da[1].dot(db[1]), db[1].dot(db[1]));
        let (ga, gb) = (da[1].dot(r), -db[1].dot(r));
        let det = aa * bb2 - ab * ab;
        if det.abs() < 1e-14 * (aa * bb2).max(1e-300) {
            return None;
        }
        let ds = -(ga * bb2 - gb * ab) / det;
        let dt = -(aa * gb - ab * ga) / det;
        s = (s + ds).clamp(a.t0.min(a.t1), a.t0.max(a.t1));
        t = (t + dt).clamp(b.t0.min(b.t1), b.t0.max(b.t1));
        if ds.abs() + dt.abs() < 1e-14 * (1.0 + s.abs() + t.abs()) {
            break;
        }
    }
    let (p, q) = (a.curve.point(s), b.curve.point(t));
    if p.distance(q) > tol {
        return None;
    }
    // Transversal, and inside both edges.
    let (ta, tb) = (a.curve.tangent(s).normalize(), b.curve.tangent(t).normalize());
    if ta.cross(tb).length() < 1e-3 {
        return None;
    }
    let p = 0.5 * (p + q);
    let ends = [a.v0, a.v1, b.v0, b.v1];
    if ends.iter().any(|v| w.vertex(*v).distance(p) < tol * 10.0) {
        return None;
    }
    Some(p)
}

/// Parameters of the closest points of segments `p0p1` and `q0q1`.
fn segment_closest(p0: DVec3, p1: DVec3, q0: DVec3, q1: DVec3) -> (f64, f64) {
    let (d1, d2, r) = (p1 - p0, q1 - q0, p0 - q0);
    let (a, e, f) = (d1.dot(d1), d2.dot(d2), d2.dot(r));
    if a < 1e-300 && e < 1e-300 {
        return (0.0, 0.0);
    }
    if a < 1e-300 {
        return (0.0, (f / e).clamp(0.0, 1.0));
    }
    let c = d1.dot(r);
    if e < 1e-300 {
        return ((-c / a).clamp(0.0, 1.0), 0.0);
    }
    let b = d1.dot(d2);
    let den = a * e - b * b;
    let mut s = if den > 1e-300 { ((b * f - c * e) / den).clamp(0.0, 1.0) } else { 0.0 };
    let mut t = (b * s + f) / e;
    if t < 0.0 {
        t = 0.0;
        s = (-c / a).clamp(0.0, 1.0);
    } else if t > 1.0 {
        t = 1.0;
        s = ((b - c) / a).clamp(0.0, 1.0);
    }
    (s, t)
}

fn intersect_boxes(a: &BBox3, b: &BBox3) -> BBox3 {
    BBox3 { min: a.min.max(b.min), max: a.max.min(b.max) }
}

fn find_vertex(w: &Body, p: DVec3, eps: f64) -> Option<VertexId> {
    let mut best: Option<(f64, VertexId)> = None;
    for (i, v) in w.vertices.iter().enumerate() {
        let d = v.point.distance(p);
        if d < eps && best.map(|b| d < b.0).unwrap_or(true) {
            best = Some((d, VertexId(i as u32)));
        }
    }
    best.map(|b| b.1)
}

fn edge_key(w: &Body, e: EdgeId) -> (u32, u32) {
    let ed = w.edge(e);
    (ed.v0.0.min(ed.v1.0), ed.v0.0.max(ed.v1.0))
}

fn find_edge(w: &Body, index: &HashMap<(u32, u32), Vec<EdgeId>>, va: VertexId, vb: VertexId, mid: DVec3, tol: f64) -> Option<EdgeId> {
    let key = (va.0.min(vb.0), va.0.max(vb.0));
    index.get(&key)?.iter().copied().find(|&e| {
        let ed = w.edge(e);
        let t = ed.curve.closest_param_in(mid, ed.t0, ed.t1);
        ed.curve.point(t).distance(mid) < tol * 100.0
    })
}

/// Cuts edges at the given (parameter, vertex) positions, rewriting loops.
pub(crate) fn apply_splits(w: &mut Body, splits: HashMap<EdgeId, Vec<(f64, VertexId)>>) -> HashMap<EdgeId, Vec<EdgeId>> {
    let mut pieces: HashMap<EdgeId, Vec<EdgeId>> = HashMap::new();
    for (e, mut list) in splits {
        list.sort_by(|a, b| a.0.total_cmp(&b.0));
        let edge = w.edge(e).clone();
        let eps = (edge.t1 - edge.t0) * 1e-9;
        let mut ts = vec![(edge.t0, edge.v0)];
        for (t, v) in list {
            let last = *ts.last().unwrap();
            if t > last.0 + eps && t < edge.t1 - eps && v != last.1 && v != edge.v1 {
                ts.push((t, v));
            }
        }
        ts.push((edge.t1, edge.v1));
        if ts.len() == 2 {
            continue;
        }
        let mut ids = Vec::new();
        for k in 0..ts.len() - 1 {
            let ((t0, v0), (t1, v1)) = (ts[k], ts[k + 1]);
            if k == 0 {
                let ed = &mut w.edges[e.idx()];
                ed.t1 = t1;
                ed.v1 = v1;
                ids.push(e);
            } else {
                ids.push(w.add_edge(edge.curve.clone(), t0, t1, v0, v1));
            }
        }
        pieces.insert(e, ids);
    }
    for l in w.loops.iter_mut() {
        let mut out = Vec::with_capacity(l.coedges.len());
        for c in &l.coedges {
            match pieces.get(&c.edge) {
                None => out.push(*c),
                Some(ids) if c.reversed => out.extend(ids.iter().rev().map(|&i| Coedge::new(i, true))),
                Some(ids) => out.extend(ids.iter().map(|&i| Coedge::new(i, false))),
            }
        }
        l.coedges = out;
    }
    pieces
}

/// Splits an intersection curve at the vertices lying on it. Returns
/// increasing parameter intervals with their end vertices; closed curves
/// without vertices get a new vertex at their start.
fn curve_pieces(w: &mut Body, c: &Curve3, c0: f64, c1: f64, verts: &[VertexId], tol: f64) -> Vec<(f64, f64, VertexId, VertexId)> {
    let periodic = c.period().filter(|p| (c1 - c0 - p).abs() < 1e-9);
    let closed = periodic.is_some() || c.point(c0).distance(c.point(c1)) < tol * 10.0;
    let mut cuts: Vec<(f64, VertexId)> = verts
        .iter()
        .filter_map(|&v| {
            let p = w.vertex(v);
            let t = c.closest_param_in(p, c0, c1);
            (c.point(t).distance(p) < tol * 10.0).then_some((t, v))
        })
        .collect();
    if let Some(p) = periodic {
        for x in cuts.iter_mut() {
            x.0 = c0 + (x.0 - c0).rem_euclid(p);
        }
    }
    cuts.sort_by(|a, b| a.0.total_cmp(&b.0));
    cuts.dedup_by(|a, b| a.1 == b.1 || (a.0 - b.0).abs() < 1e-12);
    let mut out = Vec::new();
    if cuts.is_empty() {
        if closed {
            let v = find_vertex(w, c.point(c0), tol * 10.0).unwrap_or_else(|| w.add_vertex(c.point(c0)));
            out.push((c0, c1, v, v));
        }
        return out;
    }
    if !closed {
        for k in 0..cuts.len().saturating_sub(1) {
            out.push((cuts[k].0, cuts[k + 1].0, cuts[k].1, cuts[k + 1].1));
        }
        return out;
    }
    if let Some(p) = periodic {
        let n = cuts.len();
        for k in 0..n {
            let (ta, va) = cuts[k];
            let (tb, vb) = if k + 1 < n { cuts[k + 1] } else { (cuts[0].0 + p, cuts[0].1) };
            out.push((ta, tb, va, vb));
        }
        return out;
    }
    // Closed but not periodic (a traced loop): cut at the curve's start too.
    let p0 = c.point(c0);
    let vs = find_vertex(w, p0, tol * 10.0).unwrap_or_else(|| w.add_vertex(p0));
    let mut all = vec![(c0, vs)];
    all.extend(cuts.into_iter().filter(|x| x.1 != vs));
    all.push((c1, vs));
    for k in 0..all.len() - 1 {
        if all[k + 1].0 - all[k].0 > 1e-12 {
            out.push((all[k].0, all[k + 1].0, all[k].1, all[k + 1].1));
        }
    }
    out
}

/// Approximate intersection points of a face mesh with a surface.
fn mesh_seeds(m: &FaceMesh, s: &Surface, roi: &BBox3, max: usize) -> Vec<DVec3> {
    // Only points near the region of interest matter; each projection
    // starts from the previous point's parameters (mesh points come in
    // runs of neighbours).
    let near = roi.expanded(roi.diagonal() * 0.05);
    let mut hint = None;
    let d: Vec<f64> = m
        .positions
        .iter()
        .map(|p| {
            if !near.contains(*p) {
                return f64::NAN;
            }
            match s {
                Surface::Nurbs(_) => {
                    let uv = s.project(*p, hint);
                    hint = Some(uv);
                    (*p - s.point(uv.x, uv.y)).dot(s.normal(uv.x, uv.y))
                }
                _ => ssi::signed_distance(s, *p),
            }
        })
        .collect();
    let mut out = Vec::new();
    for t in &m.triangles {
        for k in 0..3 {
            let (i, j) = (t[k] as usize, t[(k + 1) % 3] as usize);
            if i < j && d[i] * d[j] < 0.0 {
                let a = m.positions[i];
                let b = m.positions[j];
                out.push(a + (b - a) * (d[i] / (d[i] - d[j])));
            }
        }
    }
    if out.len() > max {
        let step = out.len() / max + 1;
        out = out.into_iter().step_by(step).collect();
    }
    out
}

/// Centroid of the largest triangle of a face mesh.
fn interior_point(m: &FaceMesh) -> Option<DVec3> {
    // The centroid farthest from the fragment's boundary: points near the
    // boundary may sit on (or tangent to) the other operand's surface.
    let mut count: HashMap<(u32, u32), u32> = HashMap::new();
    for t in &m.triangles {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            *count.entry((a.min(b), a.max(b))).or_default() += 1;
        }
    }
    let border: Vec<(DVec3, DVec3)> =
        count.iter().filter(|(_, n)| **n == 1).map(|((a, b), _)| (m.positions[*a as usize], m.positions[*b as usize])).collect();
    let mut best: Option<(f64, DVec3)> = None;
    // A spread of a few hundred triangles finds a point well inside.
    let stride = (m.triangles.len() / 256).max(1);
    for t in m.triangles.iter().step_by(stride) {
        let [a, b, c] = t.map(|i| m.positions[i as usize]);
        if (b - a).cross(c - a).length() <= 1e-14 {
            continue;
        }
        let g = (a + b + c) / 3.0;
        let d = border.iter().map(|(p, q)| ssi::dist_point_segment3(g, *p, *q)).fold(f64::INFINITY, f64::min);
        if best.map(|x| d > x.0).unwrap_or(true) {
            best = Some((d, g));
        }
    }
    best.map(|b| b.1)
}

/// Ray-parity point-in-solid test against a closed mesh, by majority vote
/// over directions whose rays do not graze edges.
fn point_in_mesh(p: DVec3, mesh: &BodyMesh) -> bool {
    let dirs = [
        DVec3::new(0.5711, 0.6234, 0.5340),
        DVec3::new(-0.3127, 0.8123, -0.4921),
        DVec3::new(0.7073, -0.4472, 0.5477),
        DVec3::new(-0.6011, -0.5523, 0.5775),
        DVec3::new(0.1937, 0.2711, -0.9428),
    ];
    let tris: Vec<[DVec3; 3]> = mesh.triangles().map(|t| t.0).collect();
    let bbox = BBox3::from_points(tris.iter().flatten().copied());
    if !bbox.contains(p) {
        return false;
    }
    let (mut ins, mut outs) = (0, 0);
    for d in dirs {
        let d = d.normalize();
        let mut count = 0;
        let mut ambiguous = false;
        for t in &tris {
            match ray_tri(p, d, t) {
                RayHit::Miss => {}
                RayHit::Hit => count += 1,
                RayHit::Graze => {
                    ambiguous = true;
                    break;
                }
            }
        }
        if ambiguous {
            continue;
        }
        if count % 2 == 1 {
            ins += 1;
        } else {
            outs += 1;
        }
        if ins >= 2 || outs >= 2 {
            break;
        }
    }
    ins > outs
}

enum RayHit {
    Miss,
    Hit,
    Graze,
}

fn ray_tri(o: DVec3, d: DVec3, t: &[DVec3; 3]) -> RayHit {
    let e1 = t[1] - t[0];
    let e2 = t[2] - t[0];
    let pv = d.cross(e2);
    let det = e1.dot(pv);
    let scale = e1.length() * e2.length();
    if scale < 1e-300 || e1.cross(e2).length() < 1e-12 * scale || det.abs() < 1e-12 * scale {
        return RayHit::Miss;
    }
    let inv = 1.0 / det;
    let s = o - t[0];
    let u = s.dot(pv) * inv;
    let q = s.cross(e1);
    let v = d.dot(q) * inv;
    let dist = e2.dot(q) * inv;
    let eps = 1e-9;
    if u < -eps || v < -eps || u + v > 1.0 + eps || dist < -1e-12 {
        return RayHit::Miss;
    }
    if u < eps || v < eps || u + v > 1.0 - eps || dist.abs() < 1e-12 {
        return RayHit::Graze;
    }
    RayHit::Hit
}

// ------------------------------------------------------------ face split --

struct HalfEdge {
    from: usize,
    to: usize,
    uv: Vec<DVec2>,
    coedge: Option<Coedge>,
    /// Extra (interior) edge this half-edge belongs to.
    extra: Option<EdgeId>,
    twin: Option<usize>,
    /// Signed curvature (left turn positive) at the start, to order
    /// half-edges leaving a node with the same tangent.
    curv_out: f64,
    /// Same at the end, for the half-edge traversed backwards.
    curv_in_rev: f64,
    dir_out: f64,
    dir_in_rev: f64,
}

/// Splits face `f` along the interior edges `extras`. Returns the loops of
/// each resulting face (outer loop first), or `None` if the walk failed.
fn split_face(w: &Body, f: FaceId, extras: &[EdgeId], samples: &[Vec<(f64, DVec3)>]) -> Option<Vec<Vec<Vec<Coedge>>>> {
    let face = w.face(f);
    let surf = &face.surface;
    if face.loops.is_empty() {
        return None;
    }
    let flip = face_uv_flipped(w, f);
    // Mirror u so that the face interior is on the left of every loop.
    let fix = |uv: DVec2| if flip { DVec2::new(-uv.x, uv.y) } else { uv };
    let boundary = face_coedge_uvs(w, f, samples);
    let (lo, hi) = uv_range(boundary.iter().flatten().flat_map(|(_, s)| s.iter().map(|x| &x.uv)));
    let center = (lo + hi) * 0.5;
    let span = (hi - lo).length().max(1e-12);
    let eps_node = span * 1e-6;

    let mut nodes: Vec<(VertexId, DVec2)> = Vec::new();
    let loose = span * 1e-3;
    let node_of = |nodes: &mut Vec<(VertexId, DVec2)>, v: VertexId, uv: DVec2, limit: f64| -> usize {
        let best = nodes
            .iter()
            .enumerate()
            .filter(|(_, n)| n.0 == v)
            .map(|(i, n)| (n.1.distance(uv), i))
            .min_by(|a, b| a.0.total_cmp(&b.0));
        match best {
            Some((d, i)) if d < limit => i,
            _ => {
                nodes.push((v, uv));
                nodes.len() - 1
            }
        }
    };
    let mut hes: Vec<HalfEdge> = Vec::new();
    let mk = |from: usize, to: usize, uv: Vec<DVec2>, coedge: Option<Coedge>, extra: Option<EdgeId>| -> HalfEdge {
        let (dir_out, dir_in_rev) = polyline_dirs(&uv);
        HalfEdge {
            from,
            to,
            curv_out: start_curvature(&uv),
            curv_in_rev: start_curvature(&uv.iter().rev().copied().collect::<Vec<_>>()),
            uv,
            coedge,
            extra,
            twin: None,
            dir_out,
            dir_in_rev,
        }
    };
    for lp in &boundary {
        let n = lp.len();
        for i in 0..n {
            let (c, s) = &lp[i];
            let (va, vb) = w.coedge_vertices(*c);
            let uv: Vec<DVec2> = s.iter().map(|x| fix(x.uv)).collect();
            let a = node_of(&mut nodes, va, uv[0], eps_node);
            let end = *uv.last().unwrap();
            let b = node_of(&mut nodes, vb, end, eps_node);
            let mut he = mk(a, b, uv, Some(*c), None);
            refine_dirs(&mut he, w, surf, *c, flip);
            hes.push(he);
            let next_start = fix(lp[(i + 1) % n].1[0].uv);
            if next_start.distance(end) > eps_node {
                // Degenerate boundary piece (e.g. along a pole).
                let b2 = node_of(&mut nodes, vb, next_start, eps_node);
                hes.push(mk(b, b2, vec![end, next_start], None, None));
            }
        }
    }
    for &e in extras {
        let pts: Vec<DVec3> = samples[e.idx()].iter().map(|s| s.1).collect();
        let m = pts.len() / 2;
        let mut uv = vec![DVec2::ZERO; pts.len()];
        uv[m] = surf.align_periodic(surf.project(pts[m], None), center);
        for k in m + 1..pts.len() {
            uv[k] = surf.project(pts[k], Some(uv[k - 1]));
        }
        for k in (0..m).rev() {
            uv[k] = surf.project(pts[k], Some(uv[k + 1]));
        }
        let last = uv.len() - 1;
        if last >= 1 {
            if is_singular(surf, uv[0]) {
                uv[0].x = uv[1].x;
            }
            if is_singular(surf, uv[last]) {
                uv[last].x = uv[last - 1].x;
            }
        }
        let uv: Vec<DVec2> = uv.into_iter().map(fix).collect();
        let ed = w.edge(e);
        let a = extra_node(&mut nodes, &mut hes, ed.v0, uv[0], loose);
        let b = extra_node(&mut nodes, &mut hes, ed.v1, *uv.last().unwrap(), loose);
        let mut fwd = mk(a, b, uv.clone(), Some(Coedge::new(e, false)), Some(e));
        refine_dirs(&mut fwd, w, surf, Coedge::new(e, false), flip);
        let rev_uv: Vec<DVec2> = uv.into_iter().rev().collect();
        let mut bwd = mk(b, a, rev_uv, Some(Coedge::new(e, true)), Some(e));
        refine_dirs(&mut bwd, w, surf, Coedge::new(e, true), flip);
        fwd.twin = Some(hes.len() + 1);
        bwd.twin = Some(hes.len());
        hes.push(fwd);
        hes.push(bwd);
    }

    // Prune dangling interior edges.
    let mut alive = vec![true; hes.len()];
    loop {
        let mut changed = false;
        for i in 0..hes.len() {
            let Some(e) = hes[i].extra else { continue };
            if !alive[i] || hes[i].from == hes[i].to {
                continue;
            }
            let dangling = [hes[i].from, hes[i].to].iter().any(|&n| {
                hes.iter().enumerate().all(|(j, h)| !alive[j] || (h.from != n && h.to != n) || h.extra == Some(e))
            });
            if dangling {
                for j in 0..hes.len() {
                    if hes[j].extra == Some(e) {
                        alive[j] = false;
                    }
                }
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }

    let mut out_at: Vec<Vec<usize>> = vec![Vec::new(); nodes.len()];
    for (i, h) in hes.iter().enumerate() {
        if alive[i] {
            out_at[h.from].push(i);
        }
    }
    let next = |h: usize| -> Option<usize> {
        // Effective angles: an edge bending left (positive curvature) lies
        // slightly counter-clockwise of a straight one with the same tangent.
        let eff = |a: f64, k: f64| a + (k * span).atan() * 1e-6;
        let r = eff(hes[h].dir_in_rev, hes[h].curv_in_rev);
        out_at[hes[h].to]
            .iter()
            .copied()
            .map(|o| {
                let mut cw = (r - eff(hes[o].dir_out, hes[o].curv_out)).rem_euclid(TAU);
                if cw < 1e-9 || hes[h].twin == Some(o) {
                    cw += TAU;
                }
                (cw, o)
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|x| x.1)
    };
    let mut used = vec![false; hes.len()];
    let mut cycles: Vec<Vec<usize>> = Vec::new();
    for start in 0..hes.len() {
        if !alive[start] || used[start] {
            continue;
        }
        let mut cyc = vec![];
        let mut h = start;
        loop {
            used[h] = true;
            cyc.push(h);
            let Some(nh) = next(h) else {
                if debug() {
                    eprintln!("  face {} walk: no way out of node {} after half-edge {h}", f.idx(), hes[h].to);
                }
                return None;
            };
            h = nh;
            if h == start {
                break;
            }
            if used[h] || cyc.len() > hes.len() {
                if debug() {
                    let d: Vec<String> = cyc.iter().map(|&k| format!("{}:{}->{}", k, hes[k].from, hes[k].to)).collect();
                    eprintln!("  face {} walk: revisits half-edge {h} after {}", f.idx(), d.join(" "));
                    for (k, he) in hes.iter().enumerate() {
                        eprintln!("    he {k} {}->{} out {:.3} inrev {:.3} extra {:?} uv0 {:.4} uv1 {:.4}", he.from, he.to, he.dir_out, he.dir_in_rev, he.extra.map(|e| e.0), he.uv[0], he.uv.last().unwrap());
                    }
                }
                return None;
            }
        }
        cycles.push(cyc);
    }

    // Outer loops (positive area) and holes (negative).
    let mut outers: Vec<(f64, Vec<DVec2>, Vec<usize>)> = Vec::new();
    let mut holes: Vec<(f64, Vec<DVec2>, Vec<usize>)> = Vec::new();
    for cyc in cycles {
        let mut poly: Vec<DVec2> = Vec::new();
        for &h in &cyc {
            let uv = &hes[h].uv;
            poly.extend_from_slice(&uv[..uv.len() - 1]);
        }
        let area = signed_area(&poly);
        if debug() {
            let desc: Vec<String> = cyc
                .iter()
                .map(|&h| {
                    let he = &hes[h];
                    format!("{}({:.2},{:.2})->{}({:.2},{:.2}){}", he.from, he.uv[0].x, he.uv[0].y, he.to, he.uv.last().unwrap().x, he.uv.last().unwrap().y, if he.coedge.is_none() { "*" } else if he.extra.is_some() { "x" } else { "" })
                })
                .collect();
            eprintln!("  face {} cycle area {area:.4}: {}", f.idx(), desc.join(" "));
        }
        if area.abs() < span * span * 1e-12 {
            continue;
        }
        if area > 0.0 {
            outers.push((area, poly, cyc));
        } else {
            holes.push((area, poly, cyc));
        }
    }
    if outers.is_empty() {
        return None;
    }
    let mut loops: Vec<Vec<Vec<usize>>> = outers.iter().map(|o| vec![o.2.clone()]).collect();
    for (area, poly, cyc) in holes {
        let probe = 0.5 * (poly[0] + poly[1]);
        let owner = outers
            .iter()
            .enumerate()
            .filter(|(_, o)| o.0 > -area * (1.0 + 1e-9) && point_in_polygon(probe, &o.1))
            .min_by(|a, b| a.1 .0.total_cmp(&b.1 .0))
            .map(|x| x.0);
        match owner {
            Some(i) => loops[i].push(cyc),
            None => log::warn!("boolean: hole loop without an outer loop on face {}", f.idx()),
        }
    }
    Some(
        loops
            .into_iter()
            .map(|ls| ls.into_iter().map(|cyc| cyc.into_iter().filter_map(|h| hes[h].coedge).collect::<Vec<_>>()).filter(|l| !l.is_empty()).collect())
            .collect(),
    )
}

/// Node for the end of an interior edge: an existing node of the same
/// vertex, or a new node splitting a degenerate boundary piece (a pole)
/// that the end lands on.
fn extra_node(nodes: &mut Vec<(VertexId, DVec2)>, hes: &mut Vec<HalfEdge>, v: VertexId, uv: DVec2, loose: f64) -> usize {
    let best = nodes.iter().enumerate().filter(|(_, n)| n.0 == v).map(|(i, n)| (n.1.distance(uv), i)).min_by(|a, b| a.0.total_cmp(&b.0));
    if let Some((d, i)) = best {
        if d < loose {
            return i;
        }
    }
    let n = nodes.len();
    nodes.push((v, uv));
    let hit = hes.iter().position(|h| {
        h.coedge.is_none() && nodes[h.from].0 == v && nodes[h.to].0 == v && ductile_geom::poly2::dist_to_segment(uv, h.uv[0], h.uv[1]) < loose
    });
    if let Some(i) = hit {
        let (a, b) = (hes[i].uv[0], hes[i].uv[1]);
        let to = hes[i].to;
        let (o1, r1) = polyline_dirs(&[a, uv]);
        let (o2, r2) = polyline_dirs(&[uv, b]);
        hes[i].to = n;
        hes[i].uv = vec![a, uv];
        hes[i].dir_out = o1;
        hes[i].dir_in_rev = r1;
        hes.push(HalfEdge { from: n, to, uv: vec![uv, b], coedge: None, extra: None, twin: None, curv_out: 0.0, curv_in_rev: 0.0, dir_out: o2, dir_in_rev: r2 });
    }
    n
}

/// Signed curvature of a polyline at its start from its first three
/// distinct points.
fn start_curvature(uv: &[DVec2]) -> f64 {
    let mut pts = vec![uv[0]];
    for p in &uv[1..] {
        if p.distance(*pts.last().unwrap()) > 1e-12 {
            pts.push(*p);
            if pts.len() == 3 {
                break;
            }
        }
    }
    if pts.len() < 3 {
        return 0.0;
    }
    let (a, b, c) = (pts[0], pts[1], pts[2]);
    2.0 * (b - a).perp_dot(c - b) / ((b - a).length() * (c - b).length() * (c - a).length())
}

/// Directions (angles) leaving the start and arriving back from the end of a polyline.
fn polyline_dirs(uv: &[DVec2]) -> (f64, f64) {
    let n = uv.len();
    let first = uv.iter().skip(1).find(|p| p.distance(uv[0]) > 1e-14).copied().unwrap_or(uv[n - 1]);
    let last = uv.iter().rev().skip(1).find(|p| p.distance(uv[n - 1]) > 1e-14).copied().unwrap_or(uv[0]);
    let a = first - uv[0];
    let b = last - uv[n - 1];
    (a.y.atan2(a.x), b.y.atan2(b.x))
}

/// Replaces polyline directions by exact curve tangents mapped to UV.
fn refine_dirs(he: &mut HalfEdge, w: &Body, surf: &Surface, c: Coedge, flip: bool) {
    let ed = w.edge(c.edge);
    let (ts, te) = if c.reversed { (ed.t1, ed.t0) } else { (ed.t0, ed.t1) };
    let sign = if c.reversed { -1.0 } else { 1.0 };
    let unfix = |uv: DVec2| if flip { DVec2::new(-uv.x, uv.y) } else { uv };
    let map = |uv_fixed: DVec2, t3: DVec3| -> Option<f64> {
        let uv = unfix(uv_fixed);
        let d = surf.derivs(uv.x, uv.y);
        let (a, b, c2) = (d.du.dot(d.du), d.du.dot(d.dv), d.dv.dot(d.dv));
        let det = a * c2 - b * b;
        if det.abs() < 1e-14 * (a * c2).max(1e-300) {
            return None;
        }
        let (r1, r2) = (d.du.dot(t3), d.dv.dot(t3));
        let x = (r1 * c2 - r2 * b) / det;
        let y = (a * r2 - b * r1) / det;
        let v = if flip { DVec2::new(-x, y) } else { DVec2::new(x, y) };
        (v.length() > 1e-300).then(|| v.y.atan2(v.x))
    };
    let start = he.uv[0];
    let end = *he.uv.last().unwrap();
    if let Some(a) = map(start, ed.curve.tangent(ts) * sign) {
        he.dir_out = a;
    }
    if let Some(a) = map(end, -ed.curve.tangent(te) * sign) {
        he.dir_in_rev = a;
    }
}

// -------------------------------------------------------------- assembly --

/// Builds a body from face definitions referencing `w`'s edges, merging
/// coincident vertices and duplicate edges.
fn assemble(w: &Body, faces: Vec<(Surface, bool, Vec<Vec<Coedge>>)>, tol: f64) -> Body {
    // Vertices used, merged by proximity.
    let mut vmap: HashMap<VertexId, VertexId> = HashMap::new();
    let mut out = Body::new(BodyKind::Solid);
    let mut vertex_of = |out: &mut Body, v: VertexId| -> VertexId {
        if let Some(&n) = vmap.get(&v) {
            return n;
        }
        let p = w.vertex(v);
        let n = find_vertex(out, p, tol * 10.0).unwrap_or_else(|| out.add_vertex(p));
        vmap.insert(v, n);
        n
    };
    // Edges, merged when endpoints match and geometry overlaps.
    let mut emap: HashMap<EdgeId, (EdgeId, bool)> = HashMap::new();
    let mut by_key: HashMap<(u32, u32), Vec<EdgeId>> = HashMap::new();
    let mut new_faces = Vec::new();
    for (surface, reversed, loops) in faces {
        let mut new_loops = Vec::new();
        for l in loops {
            let mut nl = Vec::new();
            for c in l {
                let (ne, flipped) = if let Some(&m) = emap.get(&c.edge) {
                    m
                } else {
                    let ed = w.edge(c.edge).clone();
                    let v0 = vertex_of(&mut out, ed.v0);
                    let v1 = vertex_of(&mut out, ed.v1);
                    let key = (v0.0.min(v1.0), v0.0.max(v1.0));
                    let mid = ed.mid();
                    let found = by_key.get(&key).and_then(|list| {
                        list.iter().copied().find_map(|k| {
                            let ke = out.edge(k);
                            let t = ke.curve.closest_param_in(mid, ke.t0, ke.t1);
                            if ke.curve.point(t).distance(mid) > tol * 100.0 {
                                return None;
                            }
                            let same = ke.curve.tangent(t).dot(ed.curve.tangent(0.5 * (ed.t0 + ed.t1))) > 0.0;
                            Some((k, !same))
                        })
                    });
                    let m = found.unwrap_or_else(|| {
                        let k = out.add_edge(ed.curve.clone(), ed.t0, ed.t1, v0, v1);
                        by_key.entry(key).or_default().push(k);
                        (k, false)
                    });
                    emap.insert(c.edge, m);
                    m
                };
                // Drop edges that collapsed to a point.
                let e = out.edge(ne);
                if e.v0 == e.v1 && e.length() < tol * 10.0 {
                    continue;
                }
                nl.push(Coedge::new(ne, c.reversed != flipped));
            }
            if !nl.is_empty() {
                new_loops.push(nl);
            }
        }
        if !new_loops.is_empty() {
            new_faces.push((surface, reversed, new_loops));
        }
    }
    for (s, r, l) in new_faces {
        out.add_face(s, r, l);
    }
    out
}

/// Number of edges not used exactly twice in opposite directions.
pub fn open_edges(b: &Body) -> usize {
    let mut uses: HashMap<EdgeId, (u32, u32)> = HashMap::new();
    for l in &b.loops {
        for c in &l.coedges {
            let u = uses.entry(c.edge).or_default();
            if c.reversed {
                u.1 += 1;
            } else {
                u.0 += 1;
            }
        }
    }
    uses.values().filter(|u| **u != (1, 1)).count()
}

#[allow(dead_code)]
fn debug_loops(b: &Body) -> Vec<Loop> {
    b.loops.clone()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::*;
    use ductile_geom::Frame;
    use std::f64::consts::PI;

    fn vol(b: &Body) -> f64 {
        tessellate(b, &TessOptions { chord_tol: 1e-4, angle_tol: 0.05 }).volume()
    }

    fn bx(o: DVec3, s: DVec3) -> Body {
        make_box(&Frame { origin: o, ..Frame::WORLD }, s)
    }

    /// Checks V(A u B) + V(A n B) = V(A) + V(B) and V(A - B) = V(A) - V(A n B),
    /// with every result closed.
    fn identities(a: &Body, b: &Body) {
        let (va, vb) = (vol(a), vol(b));
        let r: Vec<Body> = [BoolOp::Union, BoolOp::Intersection, BoolOp::Difference].iter().map(|op| boolean(a, b, *op).unwrap()).collect();
        assert!(r.iter().all(|x| open_edges(x) == 0));
        let (u, n, d) = (vol(&r[0]), vol(&r[1]), vol(&r[2]));
        assert!((u + n - va - vb).abs() < 1e-4 * (va + vb), "{u} + {n} vs {va} + {vb}");
        assert!((d - (va - n)).abs() < 1e-4 * (va + vb), "{d} vs {va} - {n}");
    }

    #[test]
    fn plane_through_a_sphere_seam_and_poles() {
        // The box's y = 0 face holds the sphere's seam and both poles.
        let s = make_sphere(DVec3::new(-0.9, 0.0, -0.5), 0.75);
        let b = make_box(&Frame::new(DVec3::new(0.5, 0.0, -0.8), DVec3::Y, DVec3::NEG_Z), DVec3::new(2.5, 1.9, 1.75));
        identities(&s, &b);
    }

    #[test]
    fn plane_tangent_to_a_torus_inner_rim() {
        // The box's x = 0 face touches the torus's inner equator, so the
        // section through it crosses itself there.
        let t = make_torus(&Frame::new(DVec3::new(-1.0, -0.18, -0.13), DVec3::Z, DVec3::X), 1.5, 0.5);
        let b = make_box(&Frame::new(DVec3::new(0.0, -0.5, 0.0), DVec3::Y, DVec3::NEG_Z), DVec3::new(1.5, 1.125, 1.25));
        identities(&t, &b);
    }

    #[test]
    fn cylinder_cutting_a_torus_tube_across_both_seams() {
        // What is left of the torus wraps round its tube, so merging its
        // pieces must keep one seam to cut it open by.
        let t = make_torus(&Frame::new(DVec3::new(-0.5, 0.5, 0.4), DVec3::Y, DVec3::Y.any_orthonormal_vector()), 1.2, 0.4);
        let c = make_cylinder(&Frame::new(DVec3::new(-0.5, 0.2028, -0.3708), DVec3::X, DVec3::X.any_orthonormal_vector()), 0.75, 1.5);
        identities(&t, &c);
    }

    #[test]
    fn cone_grazing_a_box_corner() {
        // The cone just covers one box corner; its section with the box's
        // side plane is traced loosely enough to stray off the cone.
        let z = DVec3::X;
        let b = make_box(&Frame::new(DVec3::new(-0.42591171230699776, 0.6304555355089796, 0.613134042526325), z, z.any_orthonormal_vector()), DVec3::new(1.5, 1.125, 1.25));
        let z = DVec3::new(0.5219992156033791, -0.5721212937582825, 0.632608918794073);
        let c = make_frustum(&Frame::new(DVec3::new(-0.07450286010188134, 0.5060630142844724, 0.0), z, z.any_orthonormal_vector()), 0.75, 0.3, 1.5);
        identities(&b, &c);
    }

    #[test]
    fn cylinders_touching_tangentially_at_a_point() {
        // Their outermost lines cross at one point where both surfaces
        // face -x, leaving slivers too thin for the mesh to classify.
        let a = make_cylinder(&Frame::new(DVec3::new(-1.0, 0.0, -0.40012521063672857), DVec3::Y, DVec3::Y.any_orthonormal_vector()), 0.75, 1.5);
        let b = make_cylinder(&Frame::new(DVec3::new(-0.5, 0.5, -0.4522784852919963), DVec3::Z, DVec3::Z.any_orthonormal_vector()), 1.25, 2.5);
        identities(&a, &b);
    }

    #[test]
    fn overlapping_boxes() {
        let a = bx(DVec3::ZERO, DVec3::new(2.0, 2.0, 2.0));
        let b = bx(DVec3::new(1.0, 1.0, 1.0), DVec3::new(2.0, 2.0, 2.0));
        let u = boolean(&a, &b, BoolOp::Union).unwrap();
        assert!((vol(&u) - 15.0).abs() < 1e-6, "union {}", vol(&u));
        let i = boolean(&a, &b, BoolOp::Intersection).unwrap();
        assert!((vol(&i) - 1.0).abs() < 1e-6, "intersection {}", vol(&i));
        let d = boolean(&a, &b, BoolOp::Difference).unwrap();
        assert!((vol(&d) - 7.0).abs() < 1e-6, "difference {}", vol(&d));
    }

    #[test]
    fn coplanar_boxes() {
        let a = bx(DVec3::ZERO, DVec3::new(2.0, 2.0, 1.0));
        let b = bx(DVec3::new(1.0, 1.0, 0.0), DVec3::new(2.0, 2.0, 1.0));
        let u = boolean(&a, &b, BoolOp::Union).unwrap();
        assert!((vol(&u) - 7.0).abs() < 1e-6, "union {}", vol(&u));
        let d = boolean(&a, &b, BoolOp::Difference).unwrap();
        assert!((vol(&d) - 3.0).abs() < 1e-6, "difference {}", vol(&d));
        // Pocket: tool flush with the top face.
        let c = bx(DVec3::new(0.5, 0.5, 0.5), DVec3::new(1.0, 1.0, 0.5));
        let p = boolean(&a, &c, BoolOp::Difference).unwrap();
        assert!((vol(&p) - 3.5).abs() < 1e-6, "pocket {}", vol(&p));
    }

    #[test]
    fn box_minus_cylinder() {
        let a = bx(DVec3::ZERO, DVec3::new(4.0, 4.0, 1.0));
        let c = make_cylinder(&Frame { origin: DVec3::new(2.0, 2.0, -1.0), ..Frame::WORLD }, 1.0, 3.0);
        let d = boolean(&a, &c, BoolOp::Difference).unwrap();
        let expect = 16.0 - PI;
        assert!((vol(&d) - expect).abs() < 1e-3, "through hole {} vs {expect}", vol(&d));
        // Blind hole.
        let c2 = make_cylinder(&Frame { origin: DVec3::new(2.0, 2.0, 0.5), ..Frame::WORLD }, 1.0, 1.0);
        let d2 = boolean(&a, &c2, BoolOp::Difference).unwrap();
        let expect2 = 16.0 - PI * 0.5;
        assert!((vol(&d2) - expect2).abs() < 1e-3, "blind hole {} vs {expect2}", vol(&d2));
    }

    #[test]
    fn sphere_and_box() {
        let s = make_sphere(DVec3::ZERO, 1.0);
        let b = bx(DVec3::new(0.0, -2.0, -2.0), DVec3::new(4.0, 4.0, 4.0));
        let i = boolean(&s, &b, BoolOp::Intersection).unwrap();
        let expect = 2.0 / 3.0 * PI;
        assert!((vol(&i) - expect).abs() < 2e-3, "half sphere {} vs {expect}", vol(&i));
        let u = boolean(&s, &b, BoolOp::Union).unwrap();
        let expect_u = 64.0 + 2.0 / 3.0 * PI;
        assert!((vol(&u) - expect_u).abs() < 2e-3, "union {} vs {expect_u}", vol(&u));
    }

    #[test]
    fn crossing_cylinders() {
        // Perpendicular cylinders of different radii (marched intersection).
        let a = make_cylinder(&Frame { origin: DVec3::new(0.0, 0.0, -2.0), ..Frame::WORLD }, 1.0, 4.0);
        let b = make_cylinder(&Frame::new(DVec3::new(-2.0, 0.0, 0.0), DVec3::X, DVec3::Y), 0.6, 4.0);
        let i = vol(&boolean(&a, &b, BoolOp::Intersection).unwrap());
        let u = vol(&boolean(&a, &b, BoolOp::Union).unwrap());
        let d = vol(&boolean(&a, &b, BoolOp::Difference).unwrap());
        let (va, vb) = (vol(&a), vol(&b));
        assert!(i > 0.5 && i < vb, "intersection {i}");
        assert!((u - (va + vb - i)).abs() < 1e-2, "union {u} vs {}", va + vb - i);
        assert!((d - (va - i)).abs() < 1e-2, "difference {d} vs {}", va - i);
    }

    #[test]
    fn torus_and_box() {
        // Half of a torus (plane through the axis cuts both seams' circles).
        let t = make_torus(&Frame::WORLD, 2.0, 0.5);
        let b = bx(DVec3::new(0.0, -4.0, -2.0), DVec3::new(4.0, 8.0, 4.0));
        let i = boolean(&t, &b, BoolOp::Intersection).unwrap();
        let expect = PI * 0.25 * 2.0 * PI * 2.0 * 0.5;
        assert!((vol(&i) - expect).abs() < 1e-2, "half torus {} vs {expect}", vol(&i));
        // Slice off the top: plane cuts the torus in two circles.
        let top = bx(DVec3::new(-4.0, -4.0, 0.25), DVec3::new(8.0, 8.0, 2.0));
        let d = boolean(&t, &top, BoolOp::Difference).unwrap();
        let v_full = vol(&t);
        let v_cap = vol(&boolean(&t, &top, BoolOp::Intersection).unwrap());
        assert!((vol(&d) + v_cap - v_full).abs() < 1e-2, "{} + {} vs {}", vol(&d), v_cap, v_full);
    }

    #[test]
    fn cone_and_sphere() {
        let c = make_frustum(&Frame { origin: DVec3::new(0.0, 0.0, -1.5), ..Frame::WORLD }, 1.2, 0.2, 3.0);
        let s = make_sphere(DVec3::new(0.0, 0.3, 0.0), 1.0);
        let (vc, vs) = (vol(&c), vol(&s));
        let i = vol(&boolean(&c, &s, BoolOp::Intersection).unwrap());
        let u = vol(&boolean(&c, &s, BoolOp::Union).unwrap());
        let d = vol(&boolean(&s, &c, BoolOp::Difference).unwrap());
        assert!((u - (vc + vs - i)).abs() < 2e-2, "union {u} vs {}", vc + vs - i);
        assert!((d - (vs - i)).abs() < 2e-2, "difference {d} vs {}", vs - i);
    }

    #[test]
    fn sphere_notch_at_corner() {
        // Degenerate on purpose: the sphere's pole lies on a box edge.
        let a = bx(DVec3::ZERO, DVec3::new(8.0, 6.0, 3.0));
        let s = make_sphere(DVec3::new(8.0, 0.0, 3.0), 1.4);
        let d = boolean(&a, &s, BoolOp::Difference).unwrap();
        let expect = 144.0 - 4.0 / 3.0 * PI * 1.4f64.powi(3) / 8.0;
        assert!((vol(&d) - expect).abs() < 2e-3, "{} vs {expect}", vol(&d));
    }
}
