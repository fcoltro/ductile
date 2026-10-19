// SPDX-License-Identifier: GPL-3.0-only
//! Constant-radius fillets and equal-distance chamfers on edges.
//!
//! Each edge gets a "tool" solid bounded by the faces next to the edge and
//! by the exact blend surface: the material between them is removed from
//! convex edges and added to concave ones by a boolean operation, which
//! takes care of trimming the neighbouring faces. Plane/plane edges are
//! blended by cylinders (or planes for chamfers), circular plane/cylinder
//! edges by tori (or cones). Where three filleted convex edges meet at a
//! vertex, the corner is finished with the rolling ball's sphere.

use super::boolean::{boolean, BoolError, BoolOp};
use super::extrude::extrude_region;
use super::revolve::revolve_loop;
use crate::primitives::{flip_orientation, make_box, make_sphere_in, polyhedron};
use crate::profile::{Loop3, Region, Segment};
use crate::topo::{Body, BodyKind, EdgeId, FaceId, VertexId};
use ductile_geom::{Curve3, DMat3, DVec2, DVec3, Frame, Surface};
use std::collections::{BTreeSet, HashMap};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Blend {
    /// Rolling-ball fillet of the given radius.
    Fillet(f64),
    /// Curvature-continuous (G2) blend with the same contact lines as a
    /// fillet of the given radius: its sections leave the faces with zero
    /// curvature, so the reflections flow across the joins.
    Smooth(f64),
    /// Chamfer cutting the given distance back along both faces.
    Chamfer(f64),
}

#[derive(Debug, thiserror::Error, PartialEq)]
pub enum FilletError {
    #[error("the edge is a seam or a free edge, with nothing to blend")]
    NotManifold,
    #[error("this kind of edge cannot be blended yet")]
    Unsupported,
    #[error("the edge is smooth (tangent faces)")]
    Smooth,
    #[error("the radius is too large for the geometry")]
    TooLarge,
    #[error("boolean failed: {0}")]
    Boolean(#[from] BoolError),
}

thread_local! {
    /// Set while retrying a blend with edges between planes and cylinders
    /// rolled rather than given exact tools.
    static ROLL_EXTRUSIONS: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
    static CURVED_CAPS: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
    /// Set while retrying with tangent chains rolled, after their exact
    /// tools (`chain_tools`) were too much for the boolean; `CHAIN_TOOLS`
    /// notes that a try used them.
    static ROLL_CHAINS: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
    static CHAIN_TOOLS: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// `FilletError::Unsupported`, noting where with `DUCTILE_DEBUG_FILLET`.
pub(crate) fn unsupported(line: u32, file: &str) -> FilletError {
    if std::env::var_os("DUCTILE_DEBUG_FILLET").is_some() {
        eprintln!("unsupported at {file}:{line}");
    }
    FilletError::Unsupported
}

struct EdgeInfo {
    f1: FaceId,
    f2: FaceId,
    /// Edge start and end, oriented along face 1's loop.
    p0: DVec3,
    p1: DVec3,
    t: DVec3,
    n1: DVec3,
    n2: DVec3,
    convex: bool,
}

fn outward(b: &Body, f: FaceId, p: DVec3) -> DVec3 {
    let uv = b.face(f).surface.project(p, None);
    b.face_normal(f, uv.x, uv.y)
}

fn edge_info(b: &Body, e: EdgeId) -> Result<EdgeInfo, FilletError> {
    let uses = b.edge_faces(e);
    if uses.len() != 2 || uses[0].0 == uses[1].0 {
        return Err(FilletError::NotManifold);
    }
    let (f1, c1) = uses[0];
    let f2 = uses[1].0;
    let ed = b.edge(e);
    let (ta, tb, s) = if c1.reversed { (ed.t1, ed.t0, -1.0) } else { (ed.t0, ed.t1, 1.0) };
    let tm = 0.5 * (ed.t0 + ed.t1);
    let pm = ed.curve.point(tm);
    let t = (ed.curve.tangent(tm) * s).normalize();
    let n1 = outward(b, f1, pm);
    let n2 = outward(b, f2, pm);
    let cross = n1.cross(n2);
    if cross.length() < 1e-6 {
        return Err(FilletError::Smooth);
    }
    Ok(EdgeInfo { f1, f2, p0: ed.curve.point(ta), p1: ed.curve.point(tb), t, n1, n2, convex: cross.dot(t) > 0.0 })
}

/// Arc from `a` to `b` around `c` in the plane with normal `n`, taking the
/// minor arc.
fn arc(c: DVec3, a: DVec3, b: DVec3, n: DVec3) -> Segment {
    let r = a.distance(c);
    let x = (a - c) / r;
    let mut z = n.normalize();
    let mut y = z.cross(x);
    let mut phi = (b - c).dot(y).atan2((b - c).dot(x));
    if phi < 0.0 {
        z = -z;
        y = -y;
        phi = (b - c).dot(y).atan2((b - c).dot(x));
    }
    Segment::new(Curve3::Circle { frame: Frame { origin: c, x, y, z }, radius: r }, 0.0, phi)
}

/// Half-space solid on the inner side of the plane through `o` with
/// outward normal `n`.
pub(crate) fn half_space(o: DVec3, n: DVec3, size: f64) -> Body {
    let f = Frame::new(o, -n, ductile_geom::any_perpendicular(n));
    let corner = o - f.x * size - f.y * size;
    make_box(&Frame { origin: corner, ..f }, DVec3::new(2.0 * size, 2.0 * size, size))
}

/// Faces meeting at a vertex.
fn vertex_faces(b: &Body, v: VertexId) -> BTreeSet<FaceId> {
    let mut out = BTreeSet::new();
    for f in b.face_ids() {
        for e in b.face_edges(f) {
            let ed = b.edge(e);
            if ed.v0 == v || ed.v1 == v {
                out.insert(f);
            }
        }
    }
    out
}

/// Blends the given edges. Edges are blended with tools computed on the
/// original body, then applied in turn. Where those tools do not fit
/// together, the edges are blended one after another instead, each found
/// again on the body the earlier ones left.
pub fn blend_edges(body: &Body, edges: &[EdgeId], blend: Blend) -> Result<Body, FilletError> {
    // Seams and edges between tangent faces have nothing to blend; picked
    // along with others (a whole face's outline, say) they are left out.
    let blendable: Vec<EdgeId> = edges.iter().copied().filter(|e| !matches!(edge_info(body, *e), Err(FilletError::NotManifold | FilletError::Smooth))).collect();
    if !blendable.is_empty() && blendable.len() < edges.len() {
        return blend_edges(body, &blendable, blend);
    }
    if let Some(Ok(b)) = mixed_corners_first(body, edges, blend) {
        return Ok(b);
    }
    let first = blend_selected(body, edges, blend);
    if first.is_ok() || edges.is_empty() {
        return first;
    }
    // The exact tools along plane/cylinder edges can meet other tools
    // badly at mixed (convex and concave) corners; rolling the ball along
    // those edges instead may do.
    ROLL_EXTRUSIONS.set(true);
    let rolled = blend_selected(body, edges, blend);
    ROLL_EXTRUSIONS.set(false);
    if rolled.is_ok() {
        return rolled;
    }
    // Straight tools stop at the tangent plane of a curved end face, which
    // pokes into a hole the edge runs into; stop at the face itself
    // instead (second, as meeting the face exactly is harder on booleans).
    CURVED_CAPS.set(true);
    let capped = blend_selected(body, edges, blend);
    CURVED_CAPS.set(false);
    if capped.is_ok() {
        return capped;
    }
    // An edge running on tangentially into others (into an earlier
    // fillet's arc, say) may have no sensible end on its own: blend the
    // whole run instead. Only through clearly sharp edges: the boundaries
    // of earlier blends are tangent, or nearly so where a corner patch is
    // fitted.
    let sharp = |x: EdgeId| edge_info(body, x).map(|i| i.n1.cross(i.n2).length() > 0.09).unwrap_or(false);
    let mut grown: Vec<EdgeId> = Vec::new();
    for &e in edges {
        for x in super::chain::grow_tangent_chain(body, e, &sharp) {
            if !grown.contains(&x) {
                grown.push(x);
            }
        }
    }
    if std::env::var_os("DUCTILE_DEBUG_FILLET").is_some() {
        eprintln!("blend failed ({:?}); tangent chains {:?}", first.as_ref().err(), grown.iter().map(|e| e.0).collect::<Vec<_>>());
    }
    if grown.len() > edges.len() && grown.contains(&edges[0]) {
        match blend_selected(body, &grown, blend) {
            Ok(b) => return Ok(b),
            // The run the edge belongs to cannot take the radius, so
            // neither can the edge.
            Err(FilletError::TooLarge) => return Err(FilletError::TooLarge),
            Err(e) if std::env::var_os("DUCTILE_DEBUG_FILLET").is_some() => eprintln!("tangent chain: {e:?}"),
            Err(_) => {}
        }
    }
    first
}

/// Mixed corners, where a convex edge meets two concave ones (a boss's
/// upright edge and its base) or the other way round: the odd edge out is
/// blended first, so that the other two then run on through its round as
/// one tangent chain and the ball rolls a smooth toroidal corner, as one
/// would fillet them by hand. None when the selection has no such corner.
fn mixed_corners_first(body: &Body, edges: &[EdgeId], blend: Blend) -> Option<Result<Body, FilletError>> {
    let set: BTreeSet<EdgeId> = edges.iter().copied().collect();
    let mut odd: BTreeSet<EdgeId> = BTreeSet::new();
    for vi in 0..body.vertices.len() {
        let v = VertexId(vi as u32);
        let inc: Vec<EdgeId> = body.edge_ids().filter(|e| (body.edge(*e).v0 == v || body.edge(*e).v1 == v) && body.edge_faces(*e).len() == 2).collect();
        if inc.len() != 3 || !inc.iter().all(|e| set.contains(e)) {
            continue;
        }
        let Ok(conv) = inc.iter().map(|e| edge_info(body, *e).map(|i| i.convex)).collect::<Result<Vec<bool>, _>>() else { continue };
        let n = conv.iter().filter(|c| **c).count();
        if n == 1 || n == 2 {
            let lone = n == 1;
            odd.extend(inc.iter().zip(&conv).filter(|(_, c)| **c == lone).map(|(e, _)| *e));
        }
    }
    if odd.is_empty() || odd.len() == set.len() {
        return None;
    }
    let first: Vec<EdgeId> = odd.iter().copied().collect();
    let b = match blend_selected(body, &first, blend) {
        Ok(b) => b,
        Err(e) => return Some(Err(e)),
    };
    // The other edges, found again on the new body along their old curves,
    // with the rounds the first pass left between them.
    let tol = 1e-6 * body.bbox().diagonal().max(1.0);
    let on_edge = |orig: &crate::topo::Edge, p: DVec3| orig.curve.point(orig.curve.closest_param_in(p, orig.t0, orig.t1)).distance(p) < tol;
    let on = |x: EdgeId, of: &[EdgeId]| {
        let ed = b.edge(x);
        of.iter().any(|e| {
            let o = body.edge(*e);
            on_edge(o, ed.start()) && on_edge(o, ed.end()) && on_edge(o, ed.mid())
        })
    };
    let rest: Vec<EdgeId> = set.iter().copied().filter(|e| !odd.contains(e)).collect();
    let now: Vec<EdgeId> = b.edge_ids().filter(|x| b.edge_faces(*x).len() == 2 && on(*x, &rest)).collect();
    let all: Vec<EdgeId> = body.edge_ids().collect();
    let new_edge = |x: EdgeId| !on(x, &all);
    let mut grown: Vec<EdgeId> = Vec::new();
    for &e in &now {
        for x in super::chain::grow_tangent_chain(&b, e, &|x| now.contains(&x) || new_edge(x)) {
            if !grown.contains(&x) {
                grown.push(x);
            }
        }
    }
    if grown.is_empty() {
        return Some(Ok(b));
    }
    Some(blend_selected(&b, &grown, blend))
}

/// Exact tools for a tangent chain of lines and arcs between planes and
/// cylinders (the rim of an extruded profile): each edge's own tool, the
/// straight ones cut square where they run on into the next edge, so the
/// tools meet face to face. Rounds become exact cylinders and tori (or a
/// sphere, where the radius equals an arc's) instead of rolled surfaces.
/// None when an edge has no exact tool.
fn chain_tools(body: &Body, chain: &[super::chain::Link], closed: bool, blend: Blend, size: f64) -> Option<Vec<(EdgeId, EdgeInfo, Body)>> {
    if matches!(blend, Blend::Smooth(_)) || ROLL_EXTRUSIONS.get() || ROLL_CHAINS.get() {
        return None;
    }
    let mut out = Vec::new();
    for (k, l) in chain.iter().enumerate() {
        let ed = body.edge(l.edge);
        let info = edge_info(body, l.edge).ok()?;
        let fits = match (&ed.curve, &body.face(info.f1).surface, &body.face(info.f2).surface) {
            (Curve3::Line { .. }, Surface::Plane { .. }, Surface::Plane { .. }) => true,
            (Curve3::Circle { .. }, Surface::Plane { .. }, Surface::Cylinder { .. }) | (Curve3::Circle { .. }, Surface::Cylinder { .. }, Surface::Plane { .. }) => true,
            _ => false,
        };
        if !fits || out.first().is_some_and(|(_, i, _): &(EdgeId, EdgeInfo, Body)| i.convex != info.convex) {
            return None;
        }
        let mut tool = edge_tool(body, l.edge, &info, blend, size).ok()?;
        if ed.curve.is_linear() {
            // Ends shared with the chain's neighbours, in chain order.
            let (first, last) = (k == 0 && !closed, k + 1 == chain.len() && !closed);
            let (start, end) = if l.fwd { (ed.start(), ed.end()) } else { (ed.end(), ed.start()) };
            let along = (end - start).normalize();
            if !first {
                tool = boolean(&tool, &half_space(start, -along, size), BoolOp::Intersection).ok()?;
            }
            if !last {
                tool = boolean(&tool, &half_space(end, along, size), BoolOp::Intersection).ok()?;
            }
        }
        out.push((l.edge, info, tool));
    }
    CHAIN_TOOLS.set(true);
    Some(out)
}

fn blend_selected(body: &Body, edges: &[EdgeId], blend: Blend) -> Result<Body, FilletError> {
    match blend_together(body, edges, blend) {
        Err(err @ (FilletError::Boolean(_) | FilletError::Unsupported)) if edges.len() > 1 => {
            let back: Vec<EdgeId> = edges.iter().rev().copied().collect();
            blend_in_turn(body, edges, blend).or_else(|_| blend_in_turn(body, &back, blend)).map_err(|_| err)
        }
        r => r,
    }
}

/// Blends `edges` of `body` one at a time. Each original edge is found on
/// the current body as the edges lying along it, which are blended
/// together; edges an earlier blend swallowed are passed over. The order
/// matters (a rolled blend copes with a tool-cut face at its end better
/// than the other way round), so each round blends the first remaining
/// edge that works.
fn blend_in_turn(body: &Body, edges: &[EdgeId], blend: Blend) -> Result<Body, FilletError> {
    let tol = 1e-6 * body.bbox().diagonal().max(1.0);
    let mut b = body.clone();
    let mut left: Vec<EdgeId> = edges.iter().copied().filter(|e| !matches!(edge_info(body, *e), Err(FilletError::NotManifold | FilletError::Smooth))).collect();
    while !left.is_empty() {
        let mut done = None;
        let mut last_err = FilletError::Unsupported;
        for (k, e) in left.iter().enumerate() {
            let orig = body.edge(*e);
            let on = |p: DVec3| orig.curve.point(orig.curve.closest_param_in(p, orig.t0, orig.t1)).distance(p) < tol;
            let now: Vec<EdgeId> = b
                .edge_ids()
                .filter(|x| {
                    let ed = b.edge(*x);
                    on(ed.start()) && on(ed.end()) && on(ed.mid()) && b.edge_faces(*x).len() == 2
                })
                .collect();
            if now.is_empty() {
                done = Some((k, None));
                break;
            }
            match blend_together(&b, &now, blend) {
                Ok(r) => {
                    done = Some((k, Some(r)));
                    break;
                }
                Err(err) => last_err = err,
            }
        }
        let Some((k, r)) = done else { return Err(last_err) };
        left.remove(k);
        if let Some(r) = r {
            b = r;
        }
    }
    Ok(b)
}

fn blend_together(body: &Body, edges: &[EdgeId], blend: Blend) -> Result<Body, FilletError> {
    CHAIN_TOOLS.set(false);
    let first = blend_tooled(body, edges, blend);
    let first = match first {
        Err(FilletError::Boolean(_)) if CHAIN_TOOLS.get() && !ROLL_CHAINS.get() => {
            ROLL_CHAINS.set(true);
            let r = blend_tooled(body, edges, blend);
            ROLL_CHAINS.set(false);
            r
        }
        r => r,
    };
    match first {
        // A lone edge whose tool the boolean cannot take (its blend runs
        // tangentially into an earlier one) is rolled instead, ending in
        // the face that caps it.
        Err(err @ FilletError::Boolean(_)) if edges.len() == 1 => {
            let link = super::chain::Link { edge: edges[0], fwd: true };
            let closed = body.edge(edges[0]).is_closed();
            let rolled = if closed { super::roll::roll_closed_edge(body, edges[0], blend) } else { super::chain::roll_chain(body, &[link], false, blend) };
            match rolled {
                Ok(b) => Ok(super::heal::compact(&b)),
                Err(e) => {
                    if std::env::var_os("DUCTILE_DEBUG_FILLET").is_some() {
                        eprintln!("rolled fallback: {e:?}");
                    }
                    Err(err)
                }
            }
        }
        r => r,
    }
}

fn blend_tooled(body: &Body, edges: &[EdgeId], blend: Blend) -> Result<Body, FilletError> {
    let size = body.bbox().diagonal().max(1e-6) * 4.0;
    let mut set: BTreeSet<EdgeId> = edges.iter().copied().collect();
    // In a selection of several edges, seams and smooth edges have nothing
    // to blend and are passed over; alone, they report why.
    if set.len() > 1 {
        set.retain(|e| !matches!(edge_info(body, *e), Err(FilletError::NotManifold | FilletError::Smooth)));
        if set.is_empty() {
            return Err(FilletError::Smooth);
        }
    }
    let mut infos: HashMap<EdgeId, EdgeInfo> = HashMap::new();
    let mut tools: Vec<(EdgeId, Body)> = Vec::new();
    // Chains of tangent edges, and edges with no exact tool, are blended
    // by rolling the ball along them directly; that keeps every other id
    // valid until the end.
    let mut base = body.clone();
    let mut rolled = false;
    let mut tooled: BTreeSet<EdgeId> = BTreeSet::new();
    let mut smooth_lines: Vec<EdgeId> = Vec::new();
    for (chain, closed) in super::chain::tangent_chains(body, &set.iter().copied().collect::<Vec<_>>()) {
        if chain.len() > 1 {
            if let Some(ts) = chain_tools(body, &chain, closed, blend, size) {
                if std::env::var_os("DUCTILE_DEBUG_FILLET").is_some() {
                    eprintln!("exact tools along chain {:?}", chain.iter().map(|l| l.edge.0).collect::<Vec<_>>());
                }
                for (e, info, t) in ts {
                    tools.push((e, t));
                    infos.insert(e, info);
                }
                continue;
            }
            base = super::chain::roll_chain(&base, &chain, closed, blend)?;
            rolled = true;
            continue;
        }
        let e = chain[0].edge;
        let info = edge_info(body, e)?;
        if matches!(blend, Blend::Smooth(_)) && is_line_between_planes(body, e, &info) {
            // Built below, once the corners they run into are known.
            smooth_lines.push(e);
            tooled.insert(e);
            infos.insert(e, info);
            continue;
        }
        match edge_tool(body, e, &info, blend, size) {
            Ok(t) => {
                tools.push((e, t));
                tooled.insert(e);
                infos.insert(e, info);
            }
            Err(FilletError::Unsupported) => {
                base = if closed {
                    super::roll::roll_closed_edge(&base, e, blend)?
                } else {
                    super::chain::roll_chain(&base, &chain, false, blend)?
                };
                rolled = true;
            }
            Err(err) => return Err(err),
        }
    }
    if rolled {
        base = super::heal::compact(&base);
    }
    let set = tooled;
    let corners = match blend {
        Blend::Fillet(r) | Blend::Smooth(r) => corner_tools(body, &set, &infos, blend, r),
        Blend::Chamfer(_) => Vec::new(),
    };
    // Edge tools stop where the corner patches begin, so that tools of
    // neighbouring edges never overlap.
    for (_, caps) in &corners {
        for (e, o, n) in caps {
            if let Some((_, t)) = tools.iter_mut().find(|(x, _)| x == e) {
                *t = boolean(t, &half_space(*o, *n, size), BoolOp::Intersection)?;
            }
        }
    }
    if let Blend::Smooth(r) = blend {
        for &e in &smooth_lines {
            let caps: Vec<(DVec3, DVec3)> = corners.iter().flat_map(|(_, c)| c.iter()).filter(|c| c.0 == e).map(|c| (c.1, c.2)).collect();
            tools.push((e, smooth_line_tool(body, e, &infos[&e], blend, r, &caps, size)?));
        }
    }
    let mut cut: Vec<Body> = Vec::new();
    let mut add: Vec<Body> = Vec::new();
    for (e, t) in tools {
        if infos[&e].convex {
            cut.push(t);
        } else {
            add.push(t);
        }
    }
    cut.extend(corners.into_iter().map(|(t, _)| t));
    let mut out = base;
    for t in &cut {
        let clock = std::time::Instant::now();
        out = boolean(&out, t, BoolOp::Difference)?;
        if std::env::var_os("DUCTILE_TIME").is_some() {
            eprintln!("cut {} faces -> {:?}", t.faces.len(), clock.elapsed());
        }
    }
    for t in &add {
        out = boolean(&out, t, BoolOp::Union)?;
    }
    Ok(out)
}

fn is_line_between_planes(b: &Body, e: EdgeId, info: &EdgeInfo) -> bool {
    matches!((&b.edge(e).curve, &b.face(info.f1).surface, &b.face(info.f2).surface), (Curve3::Line { .. }, Surface::Plane { .. }, Surface::Plane { .. }))
}

/// Exact tool for a smooth blend along a straight edge between planes,
/// ending at the corner planes `caps` (point, normal) where it meets a
/// corner patch, else at the face capping each end, else past the end.
fn smooth_line_tool(b: &Body, e: EdgeId, info: &EdgeInfo, blend: Blend, r: f64, caps: &[(DVec3, DVec3)], size: f64) -> Result<Body, FilletError> {
    let EdgeInfo { p0, t, n1, n2, convex, .. } = *info;
    let sgn = if convex { -1.0 } else { 1.0 };
    let m = DMat3::from_cols(n1, n2, t).transpose();
    let o = m.inverse() * DVec3::new(sgn * r, sgn * r, 0.0);
    let (q1, q2) = (p0 + o - n1 * o.dot(n1), p0 + o - n2 * o.dot(n2));
    if (q1 - p0).length().max((q2 - p0).length()) > size {
        return Err(FilletError::TooLarge);
    }
    let ctrl = super::roll::section_ctrl(blend, p0 + o, q1, q2);
    let ends = line_ends(b, e, info, caps, r);
    super::corner::smooth_edge_tool(p0, q1, q2, &ctrl, t, ends).ok_or_else(|| unsupported(line!(), file!()))
}

/// End planes `(point, normal)` for a tool along the straight edge `e`,
/// in order along the edge: a corner plane from `caps` near the end, else
/// the planar face capping it, else a plane past the end.
fn line_ends(b: &Body, e: EdgeId, info: &EdgeInfo, caps: &[(DVec3, DVec3)], r: f64) -> [(DVec3, DVec3); 2] {
    let EdgeInfo { p0, p1, t, .. } = *info;
    let margin = r * 4.0 + p0.distance(p1) * 0.05;
    let ed = b.edge(e);
    let mut ends = Vec::new();
    for (v, p) in [(ed.v0, b.vertex(ed.v0)), (ed.v1, b.vertex(ed.v1))] {
        let along = if p.distance(p1) < p.distance(p0) { t } else { -t };
        if let Some(c) = caps.iter().filter(|c| (c.0 - p).dot(along) < 0.0 && c.0.distance(p) < 4.0 * r).min_by(|x, y| x.0.distance(p).total_cmp(&y.0.distance(p))) {
            ends.push((c.0, c.1));
            continue;
        }
        let others: Vec<FaceId> = vertex_faces(b, v).into_iter().filter(|f| *f != info.f1 && *f != info.f2).collect();
        if let [f] = others[..] {
            let n = outward(b, f, p);
            if b.face(f).surface.is_planar() && n.dot(along) > 1e-6 {
                ends.push((p, n));
                continue;
            }
            // A convex edge rising from a floor (a boss's upright edge)
            // stops at the floor, so its cut does not groove it.
            if info.convex && b.face(f).surface.is_planar() && n.dot(along) < -1e-6 {
                ends.push((p, -n));
                continue;
            }
        }
        ends.push((p + along * margin, along));
    }
    // Order the ends as start then end of the section's sweep.
    if (ends[0].0 - p0).dot(t) > (ends[1].0 - p0).dot(t) {
        ends.swap(0, 1);
    }
    [ends[0], ends[1]]
}

/// Blends straight edges between planes with a radius varying linearly
/// along each edge, from `r0` at its start vertex to `r1` at its end. The
/// round blend between two planes is then a cone tangent to both; the
/// smooth (G2) one is its curvature-continuous counterpart. Each blend runs
/// out into the faces capping the edge's ends; edges must not share a
/// vertex.
pub fn blend_edges_variable(body: &Body, edges: &[EdgeId], r0: f64, r1: f64, smooth: bool) -> Result<Body, FilletError> {
    if r0 <= 0.0 || r1 <= 0.0 {
        return Err(FilletError::TooLarge);
    }
    let size = body.bbox().diagonal().max(1e-6) * 4.0;
    let mut seen = BTreeSet::new();
    let (mut cut, mut add) = (Vec::new(), Vec::new());
    for &e in edges {
        let ed = body.edge(e);
        if !seen.insert(ed.v0) || !seen.insert(ed.v1) {
            return Err(unsupported(line!(), file!()));
        }
        let info = edge_info(body, e)?;
        if !is_line_between_planes(body, e, &info) {
            return Err(unsupported(line!(), file!()));
        }
        let EdgeInfo { p0, t, n1, n2, convex, .. } = info;
        let (start, len) = (body.vertex(ed.v0), ed.start().distance(ed.end()));
        // Radius at a point on the edge's line.
        let radius = |p: DVec3| {
            let f = (p - start).dot((ed.end() - start) / len) / len;
            r0 + (r1 - r0) * f
        };
        let blend = if smooth { Blend::Smooth(1.0) } else { Blend::Fillet(1.0) };
        let sgn = if convex { -1.0 } else { 1.0 };
        let m = DMat3::from_cols(n1, n2, t).transpose().inverse();
        let station = |p: DVec3| -> Result<super::corner::Station, FilletError> {
            let r = radius(p);
            if r <= 0.0 {
                return Err(FilletError::TooLarge);
            }
            let o = m * DVec3::new(sgn * r, sgn * r, 0.0);
            let (q1, q2) = (p + o - n1 * o.dot(n1), p + o - n2 * o.dot(n2));
            if (q1 - p).length().max((q2 - p).length()) > size {
                return Err(FilletError::TooLarge);
            }
            Ok(super::corner::Station { t: p, a: q1, c: q2, ctrl: super::roll::section_ctrl(blend, p + o, q1, q2) })
        };
        let ends = line_ends(body, e, &info, &[], r0.max(r1));
        let tool = super::corner::ruled_edge_tool([station(p0)?, station(info.p1)?], ends).ok_or_else(|| unsupported(line!(), file!()))?;
        if convex {
            cut.push(tool);
        } else {
            add.push(tool);
        }
    }
    let mut out = body.clone();
    for t in &cut {
        out = boolean(&out, t, BoolOp::Difference)?;
    }
    for t in &add {
        out = boolean(&out, t, BoolOp::Union)?;
    }
    Ok(out)
}

fn edge_tool(b: &Body, e: EdgeId, info: &EdgeInfo, blend: Blend, size: f64) -> Result<Body, FilletError> {
    let ed = b.edge(e);
    // Smooth blends have tools along straight edges between planes only
    // (built by `smooth_line_tool`); elsewhere the ball is rolled for them.
    if matches!(blend, Blend::Smooth(_)) {
        return Err(unsupported(line!(), file!()));
    }
    let (s1, s2) = (&b.face(info.f1).surface, &b.face(info.f2).surface);
    match (&ed.curve, s1, s2) {
        (Curve3::Line { .. }, Surface::Plane { .. }, Surface::Plane { .. }) => line_tool(b, e, info, blend, size),
        (Curve3::Line { .. }, Surface::Plane { .. } | Surface::Cylinder { .. }, Surface::Plane { .. } | Surface::Cylinder { .. }) if !ROLL_EXTRUSIONS.get() => extrusion_tool(b, e, info, blend, size),
        (Curve3::Circle { frame, .. }, _, _) => {
            let arc = (!ed.is_closed()).then(|| ed.mid());
            match (s1, s2) {
                (Surface::Plane { .. }, Surface::Cylinder { .. }) => circle_tool(b, frame.origin, info.f1, info.f2, info, blend, arc),
                (Surface::Cylinder { .. }, Surface::Plane { .. }) => circle_tool(b, frame.origin, info.f2, info.f1, info, blend, arc),
                _ => coaxial_tool(b, frame.origin, frame.z, info, blend, arc),
            }
        }
        _ => Err(FilletError::Unsupported),
    }
}

fn line_tool(b: &Body, e: EdgeId, info: &EdgeInfo, blend: Blend, size: f64) -> Result<Body, FilletError> {
    let EdgeInfo { p0, p1, t, n1, n2, convex, .. } = *info;
    let len = p0.distance(p1);
    let u1 = n1.cross(t);
    let u2 = t.cross(n2);
    let (profile_pts, radius) = match blend {
        Blend::Fillet(r) | Blend::Smooth(r) => {
            let sgn = if convex { -1.0 } else { 1.0 };
            let m = DMat3::from_cols(n1, n2, t).transpose();
            let o = m.inverse() * DVec3::new(sgn * r, sgn * r, 0.0);
            let q1 = p0 + o - n1 * o.dot(n1);
            let q2 = p0 + o - n2 * o.dot(n2);
            ((q1, q2, Some(p0 + o)), r)
        }
        Blend::Chamfer(d) => ((p0 + u1 * d, p0 + u2 * d, None), d),
    };
    let (q1, q2, center) = profile_pts;
    // Reject blends wider than the faces next to the edge (rough check).
    let reach = (q1 - p0).length().max((q2 - p0).length());
    if reach > size {
        return Err(FilletError::TooLarge);
    }
    let margin = radius * 4.0 + len * 0.05;
    let sh = -t * margin;
    let (pp, a, c) = (p0 + sh, q1 + sh, q2 + sh);
    let mid = match (center, blend) {
        (Some(cc), Blend::Smooth(_)) => {
            let ctrl = super::roll::section_ctrl(blend, cc + sh, a, c);
            Segment::new(Curve3::Nurbs(ductile_geom::NurbsCurve::new(5, [vec![0.0; 6], vec![1.0; 6]].concat(), ctrl)), 0.0, 1.0)
        }
        (Some(cc), _) => arc(cc + sh, a, c, t),
        (None, _) => Segment::line(a, c),
    };
    let lp = Loop3 { segments: vec![Segment::line(pp, a), mid, Segment::line(c, pp)] };
    let region = Region { plane: Frame::new(pp, t, a - pp), outer: lp, holes: vec![] };
    let tool = extrude_region(&region, t * (len + 2.0 * margin)).ok_or_else(|| unsupported(line!(), file!()))?;
    cap_line_tool(b, e, info, tool, size)
}

/// Fillet (or chamfer) along part of a straight edge between two planes:
/// from `s0` to `s1`, fractions of the way from the edge's start to its
/// end (limit points). The blend ends square across the edge, in a flat
/// face where the sharp edge carries on.
pub fn blend_edge_between(body: &Body, e: EdgeId, blend: Blend, s0: f64, s1: f64) -> Result<Body, FilletError> {
    let info = edge_info(body, e)?;
    if !is_line_between_planes(body, e, &info) {
        return Err(unsupported(line!(), file!()));
    }
    let (a, b) = (s0.min(s1).clamp(0.0, 1.0), s0.max(s1).clamp(0.0, 1.0));
    if b - a < 1e-6 {
        return Err(unsupported(line!(), file!()));
    }
    if (a, b) == (0.0, 1.0) {
        return blend_edges(body, &[e], blend);
    }
    // Fractions run along the edge's own direction; `info` runs along
    // face 1's loop.
    let along = body.edge(e).start().distance(info.p0) < body.edge(e).start().distance(info.p1);
    let (a, b) = if along { (a, b) } else { (1.0 - b, 1.0 - a) };
    let EdgeInfo { p0, p1, t, n1, n2, convex, .. } = info;
    let len = p0.distance(p1);
    let size = body.bbox().diagonal().max(1e-9);
    let start = p0 + t * (len * a);
    let (q1, q2, mid) = match blend {
        Blend::Fillet(r) | Blend::Smooth(r) => {
            let sgn = if convex { -1.0 } else { 1.0 };
            let m = DMat3::from_cols(n1, n2, t).transpose();
            let o = m.inverse() * DVec3::new(sgn * r, sgn * r, 0.0);
            let (q1, q2) = (start + o - n1 * o.dot(n1), start + o - n2 * o.dot(n2));
            let mid = if matches!(blend, Blend::Smooth(_)) {
                let ctrl = super::roll::section_ctrl(blend, start + o, q1, q2);
                Segment::new(Curve3::Nurbs(ductile_geom::NurbsCurve::new(5, [vec![0.0; 6], vec![1.0; 6]].concat(), ctrl)), 0.0, 1.0)
            } else {
                arc(start + o, q1, q2, t)
            };
            (q1, q2, mid)
        }
        Blend::Chamfer(d) => {
            let (q1, q2) = (start + n1.cross(t) * d, start + t.cross(n2) * d);
            (q1, q2, Segment::line(q1, q2))
        }
    };
    if (q1 - start).length().max((q2 - start).length()) > size {
        return Err(FilletError::TooLarge);
    }
    let lp = Loop3 { segments: vec![Segment::line(start, q1), mid, Segment::line(q2, start)] };
    let region = Region { plane: Frame::new(start, t, q1 - start), outer: lp, holes: vec![] };
    let tool = extrude_region(&region, t * (len * (b - a))).ok_or_else(|| unsupported(line!(), file!()))?;
    let op = if convex { BoolOp::Difference } else { BoolOp::Union };
    Ok(boolean(body, &tool, op)?)
}

/// Stops a tool along the straight edge `e` at the planar faces capping
/// the edge's ends (where they face away along it).
fn cap_line_tool(b: &Body, e: EdgeId, info: &EdgeInfo, mut tool: Body, size: f64) -> Result<Body, FilletError> {
    let EdgeInfo { p0, p1, t, .. } = *info;
    let ed = b.edge(e);
    for (v, p) in [(ed.v0, b.vertex(ed.v0)), (ed.v1, b.vertex(ed.v1))] {
        let others: Vec<FaceId> = vertex_faces(b, v).into_iter().filter(|f| *f != info.f1 && *f != info.f2).collect();
        if others.len() == 1 {
            let n = outward(b, others[0], p);
            // Only cap when the end face actually faces away along the edge.
            let along = if p.distance(p1) < p.distance(p0) { t } else { -t };
            if info.convex && b.face(others[0]).surface.is_planar() && n.dot(along) < -1e-6 {
                // Rising from a floor: keep the cut above it.
                tool = boolean(&tool, &half_space(p, -n, size), BoolOp::Intersection)?;
            } else if n.dot(along) > 1e-6 {
                tool = match &b.face(others[0]).surface {
                    // A curved end face (a hole the edge runs into) bends
                    // away from its tangent plane: stop at the face itself.
                    Surface::Cylinder { frame, radius } if CURVED_CAPS.get() => {
                        let axis = frame.z;
                        let radial = p - frame.origin - axis * (p - frame.origin).dot(axis);
                        let solid = crate::primitives::make_cylinder(&Frame::new(frame.origin - axis * size, axis, frame.x), *radius, 2.0 * size);
                        let inside = n.dot(radial) > 0.0;
                        boolean(&tool, &solid, if inside { BoolOp::Intersection } else { BoolOp::Difference })?
                    }
                    _ => boolean(&tool, &half_space(p, n, size), BoolOp::Intersection)?,
                };
            }
        }
    }
    Ok(tool)
}

/// A face's cross-section square to a straight edge, in the section's
/// coordinates with the edge at the origin.
#[derive(Clone, Copy)]
enum Profile {
    /// A plane: the line through the origin with outward normal `n`.
    Line { n: DVec2 },
    /// A cylinder along the edge: the circle about `c` of radius `rad`;
    /// `convex` when the material is inside it.
    Circle { c: DVec2, rad: f64, convex: bool },
}

impl Profile {
    /// Radius of the circle of centres at signed offset `s` (outward
    /// positive) from a circular profile.
    fn offset_radius(rad: f64, convex: bool, s: f64) -> f64 {
        if convex { rad + s } else { rad - s }
    }

    /// Foot of `x` on the profile.
    fn foot(&self, x: DVec2) -> DVec2 {
        match *self {
            Profile::Line { n } => x - n * n.dot(x),
            Profile::Circle { c, rad, .. } => c + (x - c).normalize() * rad,
        }
    }

    /// The point `d` along the profile from the origin, setting off in
    /// direction `dir`.
    fn walk(&self, dir: DVec2, d: f64) -> DVec2 {
        match *self {
            Profile::Line { .. } => dir * d,
            Profile::Circle { c, rad, .. } => {
                let r0 = -c;
                // Turning the radius by +angle moves along perp(r0).
                let sgn = if DVec2::new(-r0.y, r0.x).dot(dir) > 0.0 { 1.0 } else { -1.0 };
                let a = sgn * d / rad;
                c + DVec2::new(r0.x * a.cos() - r0.y * a.sin(), r0.x * a.sin() + r0.y * a.cos())
            }
        }
    }
}

/// Exact tool for a blend along a straight edge between planes and
/// cylinders running along it: the cross-section is the same all along,
/// lines and circles, so the round blend is the circle tangent to both
/// (a cylinder once extruded) and the chamfer a chord.
fn extrusion_tool(b: &Body, e: EdgeId, info: &EdgeInfo, blend: Blend, size: f64) -> Result<Body, FilletError> {
    let EdgeInfo { p0, p1, t, convex, .. } = *info;
    let len = p0.distance(p1);
    let x = info.n1.reject_from(t).normalize();
    let y = t.cross(x);
    let to2 = |v: DVec3| DVec2::new(v.dot(x), v.dot(y));
    let profile = |f: FaceId| -> Option<Profile> {
        let n = to2(outward(b, f, p0)).normalize();
        match &b.face(f).surface {
            Surface::Plane { .. } => Some(Profile::Line { n }),
            Surface::Cylinder { frame, radius } if frame.z.cross(t).length() < 1e-9 => {
                let c = to2(frame.origin - p0);
                Some(Profile::Circle { c, rad: *radius, convex: n.dot(-c) > 0.0 })
            }
            _ => None,
        }
    };
    let (Some(pr1), Some(pr2)) = (profile(info.f1), profile(info.f2)) else { return Err(unsupported(line!(), file!())) };
    let (n1, n2) = (to2(info.n1).normalize(), to2(info.n2).normalize());
    let det = n1.x * n2.y - n1.y * n2.x;
    if det.abs() < 1e-9 {
        return Err(FilletError::Smooth);
    }
    let into = |n: DVec2, other: DVec2| {
        let d = DVec2::new(-n.y, n.x);
        if (d.dot(other) < 0.0) == convex { d } else { -d }
    };
    let (d1, d2) = (into(n1, n2), into(n2, n1));
    let (q1, q2, centre, reach) = match blend {
        Blend::Fillet(r) | Blend::Smooth(r) => {
            let s = if convex { -r } else { r };
            // Near the centre found from the tangent lines at the edge.
            let guess = DVec2::new(s * n2.y - n1.y * s, n1.x * s - s * n2.x) / det;
            let mut cands: Vec<DVec2> = Vec::new();
            match (pr1, pr2) {
                (Profile::Line { n }, Profile::Circle { c, rad, convex: cv }) | (Profile::Circle { c, rad, convex: cv }, Profile::Line { n }) => {
                    let rho = Profile::offset_radius(rad, cv, s);
                    let d = DVec2::new(-n.y, n.x);
                    let o = n * s - c;
                    let (bb, cc) = (d.dot(o), o.length_squared() - rho * rho);
                    let disc = bb * bb - cc;
                    if rho > 0.0 && disc >= 0.0 {
                        for k in [-1.0, 1.0] {
                            cands.push(n * s + d * (-bb + k * disc.sqrt()));
                        }
                    }
                }
                (Profile::Circle { c: ca, rad: ra, convex: cva }, Profile::Circle { c: cb, rad: rb, convex: cvb }) => {
                    let (r0, r1) = (Profile::offset_radius(ra, cva, s), Profile::offset_radius(rb, cvb, s));
                    let dd = cb - ca;
                    let l = dd.length();
                    if r0 > 0.0 && r1 > 0.0 && l > 1e-12 && l <= r0 + r1 && l >= (r0 - r1).abs() {
                        let a = (r0 * r0 - r1 * r1 + l * l) / (2.0 * l);
                        let h = (r0 * r0 - a * a).max(0.0).sqrt();
                        let m = ca + dd * (a / l);
                        let pdir = DVec2::new(-dd.y, dd.x) / l;
                        cands.push(m + pdir * h);
                        cands.push(m - pdir * h);
                    }
                }
                _ => return Err(unsupported(line!(), file!())),
            }
            let c = cands.into_iter().min_by(|a, b| a.distance(guess).total_cmp(&b.distance(guess))).ok_or(FilletError::TooLarge)?;
            if c.distance(guess) > r {
                return Err(FilletError::TooLarge);
            }
            let (q1, q2) = (pr1.foot(c), pr2.foot(c));
            (q1, q2, Some(c), r)
        }
        Blend::Chamfer(d) => (pr1.walk(d1, d), pr2.walk(d2, d), None, d),
    };
    if q1.dot(d1) <= 0.0 || q2.dot(d2) <= 0.0 {
        return Err(FilletError::TooLarge);
    }
    let margin = reach * 4.0 + len * 0.05;
    let base = p0 - t * margin;
    let at = |v: DVec2| base + x * v.x + y * v.y;
    let side = |pr: Profile, from: DVec2, to: DVec2| match pr {
        Profile::Line { .. } => Segment::line(at(from), at(to)),
        Profile::Circle { c, .. } => arc(at(c), at(from), at(to), t),
    };
    let mid = match centre {
        Some(c) => arc(at(c), at(q1), at(q2), t),
        None => Segment::line(at(q1), at(q2)),
    };
    let lp = Loop3 { segments: vec![side(pr1, DVec2::ZERO, q1), mid, side(pr2, q2, DVec2::ZERO)] };
    let region = Region { plane: Frame::new(base, t, at(q1) - base), outer: lp, holes: vec![] };
    let tool = extrude_region(&region, t * (len + 2.0 * margin)).ok_or_else(|| unsupported(line!(), file!()))?;
    cap_line_tool(b, e, info, tool, size)
}

/// The tool along a circular edge between a plane and a cylinder square to
/// it: the fillet section swept round the circle, or, for an arc through
/// `arc` (a point midway along it), round just that arc.
/// Whether `s` is a surface of revolution about the axis through `c` along
/// `a` whose meridian is a straight line (a plane square to the axis, or a
/// coaxial cylinder or cone).
fn straight_meridian_about(s: &Surface, c: DVec3, a: DVec3) -> bool {
    let on_axis = |o: DVec3| (o - c).cross(a).length() < 1e-9 * (1.0 + (o - c).length());
    match s {
        Surface::Plane { frame } => frame.z.cross(a).length() < 1e-9,
        Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. } => frame.z.cross(a).length() < 1e-9 && on_axis(frame.origin),
        _ => false,
    }
}

/// Exact tool for a blend along a circular edge between two coaxial faces
/// whose meridians are straight (planes, cylinders, cones): in the plane
/// through the axis the two faces are lines, the round blend is the circle
/// tangent to both (revolved: a torus) and the chamfer a chord (a cone).
fn coaxial_tool(b: &Body, center: DVec3, a: DVec3, info: &EdgeInfo, blend: Blend, arc_mid: Option<DVec3>) -> Result<Body, FilletError> {
    let (s1, s2) = (&b.face(info.f1).surface, &b.face(info.f2).surface);
    if !straight_meridian_about(s1, center, a) || !straight_meridian_about(s2, center, a) {
        return Err(unsupported(line!(), file!()));
    }
    let p = info.p0;
    let e = (p - center - a * (p - center).dot(a)).normalize();
    let big_r = (p - center).dot(e);
    // Meridian coordinates (along e, along a) relative to the edge point.
    let to2 = |v: DVec3| DVec2::new(v.dot(e), v.dot(a));
    let at = |q: DVec2| p + e * q.x + a * q.y;
    let (n1, n2) = (to2(outward(b, info.f1, p)).normalize(), to2(outward(b, info.f2, p)).normalize());
    let det = n1.x * n2.y - n1.y * n2.x;
    if det.abs() < 1e-9 {
        return Err(FilletError::Smooth);
    }
    let sgn = if info.convex { -1.0 } else { 1.0 };
    // Along each face, away from the edge.
    let into = |n: DVec2, other: DVec2| {
        let d = DVec2::new(-n.y, n.x);
        if (d.dot(other) < 0.0) == info.convex { d } else { -d }
    };
    let (d1, d2) = (into(n1, n2), into(n2, n1));
    let (q1, q2, mid) = match blend {
        Blend::Fillet(r) | Blend::Smooth(r) => {
            // Centre at signed distance sgn r from both face lines.
            let rhs = DVec2::new(sgn * r, sgn * r);
            let c = DVec2::new(rhs.x * n2.y - n1.y * rhs.y, n1.x * rhs.y - rhs.x * n2.x) / det;
            let (q1, q2) = (c - n1 * sgn * r, c - n2 * sgn * r);
            if q1.dot(d1) <= 0.0 || q2.dot(d2) <= 0.0 {
                return Err(FilletError::TooLarge);
            }
            (q1, q2, Some(c))
        }
        Blend::Chamfer(d) => (d1 * d, d2 * d, None),
    };
    if [q1, q2].iter().chain(mid.iter()).any(|q| big_r + q.x <= 1e-9) {
        return Err(FilletError::TooLarge);
    }
    let seg = match mid {
        Some(c) => arc(at(c), at(q1), at(q2), e.cross(a)),
        None => Segment::line(at(q1), at(q2)),
    };
    let lp = Loop3 { segments: vec![Segment::line(p, at(q1)), seg, Segment::line(at(q2), p)] };
    let Some(m) = arc_mid else { return revolve_loop(&lp, center, a).ok_or_else(|| unsupported(line!(), file!())) };
    let f = a.cross(e);
    let turn = |q: DVec3| {
        let v = q - center;
        v.dot(f).atan2(v.dot(e)).rem_euclid(std::f64::consts::TAU)
    };
    let (end, through) = (turn(info.p1), turn(m));
    let (axis, angle) = if through < end { (a, end) } else { (-a, std::f64::consts::TAU - end) };
    super::revolve::revolve(&lp, center, axis, angle).ok_or_else(|| unsupported(line!(), file!()))
}

fn circle_tool(b: &Body, center: DVec3, pf: FaceId, cf: FaceId, info: &EdgeInfo, blend: Blend, arc_mid: Option<DVec3>) -> Result<Body, FilletError> {
    let Surface::Cylinder { frame: cyl, radius: big_r } = &b.face(cf).surface else { return Err(unsupported(line!(), file!())) };
    let pm = 0.5 * (info.p0 + info.p1);
    let pm = if pm.distance(center) < 1e-9 { info.p0 } else { pm };
    let a = outward(b, pf, pm);
    if cyl.z.cross(a).length() > 1e-9 {
        return Err(unsupported(line!(), file!()));
    }
    // Radial direction through a point of the edge (any point works).
    let p = info.p0;
    let e = (p - center - a * (p - center).dot(a)).normalize();
    let nc = outward(b, cf, p);
    let sigma = if nc.dot(e) > 0.0 { 1.0 } else { -1.0 };
    let zs = if info.convex { -1.0 } else { 1.0 };
    let big_r = *big_r;
    let at = |rho: f64, z: f64| center + e * rho + a * z;
    let d = match blend {
        Blend::Fillet(r) | Blend::Smooth(r) | Blend::Chamfer(r) => r,
    };
    let mut rho_p = big_r + zs * sigma * d;
    // A radius equal to the arc's closes the torus into a sphere about
    // the axis (a rim filleted at the corner radius).
    if rho_p.abs() <= 1e-9 * big_r.max(1.0) {
        rho_p = 0.0;
    } else if rho_p < 0.0 {
        return Err(FilletError::TooLarge);
    }
    let corner = at(big_r, 0.0);
    let qp = at(rho_p, 0.0);
    let qc = at(big_r, zs * d);
    let mid = match blend {
        Blend::Fillet(_) | Blend::Smooth(_) => arc(at(rho_p, zs * d), qp, qc, e.cross(a)),
        Blend::Chamfer(_) => Segment::line(qp, qc),
    };
    let lp = Loop3 { segments: vec![Segment::line(corner, qp), mid, Segment::line(qc, corner)] };
    let Some(m) = arc_mid else { return revolve_loop(&lp, center, a).ok_or_else(|| unsupported(line!(), file!())) };
    // Sweep from p0 to p1 the way round that passes the arc's middle.
    // Angles round `a`, counter-clockwise from `e`.
    let f = a.cross(e);
    let turn = |q: DVec3| {
        let v = q - center;
        v.dot(f).atan2(v.dot(e)).rem_euclid(std::f64::consts::TAU)
    };
    let (end, through) = (turn(info.p1), turn(m));
    let (axis, angle) = if through < end { (a, end) } else { (-a, std::f64::consts::TAU - end) };
    super::revolve::revolve(&lp, center, axis, angle).ok_or_else(|| unsupported(line!(), file!()))
}

/// Spherical corner patches where three filleted convex plane/plane edges
/// meet at a vertex of three planar faces.
/// Each comes with the planes (edge, point, outward normal) that cut the
/// incident edge tools back to the patch.
fn corner_tools(b: &Body, set: &BTreeSet<EdgeId>, infos: &HashMap<EdgeId, EdgeInfo>, blend: Blend, r: f64) -> Vec<(Body, Vec<(EdgeId, DVec3, DVec3)>)> {
    let mut out = Vec::new();
    for vi in 0..b.vertices.len() {
        let v = VertexId(vi as u32);
        let inc: Vec<EdgeId> = b.edge_ids().filter(|e| (b.edge(*e).v0 == v || b.edge(*e).v1 == v) && b.edge_faces(*e).len() == 2).collect();
        if inc.len() != 3 || !inc.iter().all(|e| set.contains(e) && infos[e].convex && b.edge(*e).curve.is_linear()) {
            continue;
        }
        let faces: Vec<FaceId> = vertex_faces(b, v).into_iter().collect();
        if faces.len() != 3 || !faces.iter().all(|f| b.face(*f).surface.is_planar()) {
            continue;
        }
        let pv = b.vertex(v);
        let ns: Vec<DVec3> = faces.iter().map(|f| outward(b, *f, pv)).collect();
        let m = DMat3::from_cols(ns[0], ns[1], ns[2]).transpose();
        if m.determinant().abs() < 1e-9 {
            continue;
        }
        let c = pv + m.inverse() * DVec3::splat(-r);
        // Pair each face with the edge not lying on it.
        let mut rows: Vec<((DVec3, f64), (DVec3, f64))> = Vec::new();
        let mut caps = Vec::new();
        for (k, f) in faces.iter().enumerate() {
            let fe = b.face_edges(*f);
            let Some(&e) = inc.iter().find(|e| !fe.contains(e)) else { continue };
            let ed = b.edge(e);
            let other = if ed.v0 == v { ed.end() } else { ed.start() };
            let d = (other - pv).normalize();
            rows.push(((ns[k], ns[k].dot(pv)), (d, d.dot(c))));
            caps.push((e, c, -d));
        }
        if rows.len() != 3 {
            continue;
        }
        if let Blend::Smooth(_) = blend {
            let ds = [rows[0].1 .0, rows[1].1 .0, rows[2].1 .0];
            if let Some(t) = super::corner::smooth_corner_tool(pv, [ns[0], ns[1], ns[2]], ds, r) {
                out.push((t, caps));
            }
            continue;
        }
        let mut pts = Vec::new();
        for idx in 0..8 {
            let pick = |i: usize| if idx >> i & 1 == 0 { rows[i].0 } else { rows[i].1 };
            let (a0, a1, a2) = (pick(0), pick(1), pick(2));
            let mm = DMat3::from_cols(a0.0, a1.0, a2.0).transpose();
            if mm.determinant().abs() < 1e-12 {
                break;
            }
            pts.push(mm.inverse() * DVec3::new(a0.1, a1.1, a2.1));
        }
        if pts.len() != 8 {
            continue;
        }
        let quads = vec![vec![0, 4, 6, 2], vec![1, 3, 7, 5], vec![0, 1, 5, 4], vec![2, 6, 7, 3], vec![0, 2, 3, 1], vec![4, 5, 7, 6]];
        let mut cell = polyhedron(&pts, &quads, BodyKind::Solid);
        if crate::tess::tessellate(&cell, &crate::tess::TessOptions::default()).volume() < 0.0 {
            cell = flip_orientation(cell);
        }
        // A skewed sphere frame keeps its seam and poles off the cell's planes.
        let sf = Frame::new(c, DVec3::new(0.21, -0.37, 0.9), DVec3::new(0.83, 0.47, 0.11));
        if let Ok(t) = boolean(&cell, &make_sphere_in(&sf, r), BoolOp::Difference) {
            out.push((t, caps));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ops::boolean::open_edges;
    use crate::primitives::*;
    use crate::tess::{tessellate, TessOptions};
    use std::f64::consts::PI;

    fn vol(b: &Body) -> f64 {
        tessellate(b, &TessOptions { chord_tol: 1e-4, angle_tol: 0.05 }).volume()
    }

    #[test]
    fn fillet_running_tangentially_into_an_earlier_one() {
        // From the workflow stress test: the bottom edge filleted, the
        // upright corner chamfered, then the chamfer's front edge filleted;
        // its blend meets the first fillet tangentially at the bottom.
        let (x, y, z) = (3.290107170143669, 2.7499716551606714, 1.328096334801292);
        let b = make_box(&Frame::WORLD, DVec3::new(x, y, z));
        let pick = |b: &Body, f: &dyn Fn(DVec3, DVec3) -> bool| b.edge_ids().find(|e| f(b.edge(*e).start(), b.edge(*e).end())).unwrap();
        let e = pick(&b, &|p, q| (p.x - x).abs() < 1e-9 && (q.x - x).abs() < 1e-9 && p.z.abs() < 1e-9 && q.z.abs() < 1e-9);
        let b = blend_edges(&b, &[e], Blend::Fillet(0.12947949379266827)).unwrap();
        let e = pick(&b, &|p, q| (p.x - x).abs() < 1e-9 && (q.x - x).abs() < 1e-9 && p.y.abs() < 1e-9 && q.y.abs() < 1e-9);
        let b = blend_edges(&b, &[e], Blend::Chamfer(0.0627334840228452)).unwrap();
        let e = pick(&b, &|p, q| p.y.abs() < 1e-9 && q.y.abs() < 1e-9 && (p.x - q.x).abs() < 1e-9 && (p.x - x + 0.0627).abs() < 1e-3);
        let f = blend_edges(&b, &[e], Blend::Fillet(0.14645104439179424)).unwrap();
        assert_eq!(open_edges(&f), 0);
        assert!(vol(&f) < vol(&b) && vol(&f) > vol(&b) - 0.01, "{} vs {}", vol(&f), vol(&b));
    }

    #[test]
    fn fillets_a_boss_corner_of_mixed_edges() {
        // A boss on a plate: at its foot the upright edge is convex and the
        // two foot edges concave. The upright fillet alone stops at the
        // plate instead of grooving it; all three together meet in a corner.
        let plate = make_box(&Frame::WORLD, DVec3::new(4.0, 4.0, 1.0));
        let boss = make_box(&Frame { origin: DVec3::new(1.0, 1.0, 1.0), ..Frame::WORLD }, DVec3::new(2.0, 2.0, 1.0));
        let b = crate::ops::boolean::boolean(&plate, &boss, crate::ops::boolean::BoolOp::Union).unwrap();
        let at = DVec3::new(1.0, 1.0, 1.0);
        let edges: Vec<EdgeId> = b.edge_ids().filter(|e| b.edge(*e).start().distance(at) < 1e-9 || b.edge(*e).end().distance(at) < 1e-9).collect();
        assert_eq!(edges.len(), 3);
        let r = 0.3;
        let upright = *edges.iter().find(|e| (b.edge(**e).start() - b.edge(**e).end()).z.abs() > 0.5).unwrap();
        let f = blend_edges(&b, &[upright], Blend::Fillet(r)).unwrap();
        assert_eq!(open_edges(&f), 0);
        let cut = r * r * (1.0 - PI / 4.0);
        assert!((vol(&f) - (20.0 - cut)).abs() < 1e-3, "{}", vol(&f));
        let f = blend_edges(&b, &edges, Blend::Fillet(r)).unwrap();
        assert_eq!(open_edges(&f), 0);
        // Two foot fillets add material, the upright one takes a little.
        assert!(vol(&f) > 20.0 + cut && vol(&f) < 20.0 + 4.0 * cut, "{}", vol(&f));
    }

    #[test]
    fn fillets_a_rim_at_its_corner_radius_into_spheres() {
        // Uprights rounded first, then the top rim at the same radius: each
        // corner closes into an exact sphere.
        let b = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
        let up: Vec<EdgeId> = b.edge_ids().filter(|e| (b.edge(*e).start() - b.edge(*e).end()).z.abs() > 1.0).collect();
        let r = 0.6;
        let b = blend_edges(&b, &up, Blend::Fillet(r)).unwrap();
        let rim: Vec<EdgeId> = b.edge_ids().filter(|e| b.edge(*e).start().z > 1.99 && b.edge(*e).end().z > 1.99).collect();
        let f = blend_edges(&b, &rim, Blend::Fillet(r)).unwrap();
        assert_eq!(open_edges(&f), 0);
        assert_eq!(f.face_ids().filter(|x| matches!(f.face(*x).surface, Surface::Sphere { .. })).count(), 4);
        // The straight runs lose r²(1 - π/4) each; every corner loses the
        // quarter cylinder minus the sphere's octant.
        let straight = 2.0 * (4.0 - 2.0 * r) + 2.0 * (3.0 - 2.0 * r);
        let corner = PI * r * r * r / 4.0 - PI * r * r * r / 6.0;
        let want = vol(&b) - straight * r * r * (1.0 - PI / 4.0) - 4.0 * corner;
        assert!((vol(&f) - want).abs() < 2e-3, "{} vs {want}", vol(&f));
    }

    #[test]
    fn fillets_between_limit_points() {
        let b = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
        let e = b.edge_ids().find(|e| { let m = b.edge(*e).mid(); m.z > 1.9 && m.y < 0.1 }).unwrap();
        for blend in [Blend::Fillet(0.5), Blend::Chamfer(0.5)] {
            let f = blend_edge_between(&b, e, blend, 0.25, 0.75).unwrap();
            assert_eq!(open_edges(&f), 0);
            let cut = if matches!(blend, Blend::Fillet(_)) { 0.25 * (1.0 - PI / 4.0) } else { 0.125 } * 2.0;
            assert!((vol(&f) - (24.0 - cut)).abs() < 1e-3, "{blend:?} {}", vol(&f));
            // Two square ends where the sharp edge carries on.
            assert_eq!(f.faces.len(), 6 + 3, "{blend:?}");
        }
    }

    #[test]
    fn blends_the_edges_where_a_side_hole_breaks_the_top() {
        // A hole along x centred in the top face leaves straight edges
        // between the top plane and the hole's cylinder.
        let b = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
        let hole = make_cylinder(&Frame::new(DVec3::new(-1.0, 1.5, 1.8), DVec3::X, DVec3::Y), 0.5, 6.0);
        let h = crate::ops::boolean::boolean(&b, &hole, crate::ops::boolean::BoolOp::Difference).unwrap();
        let rims: Vec<EdgeId> = h
            .edge_ids()
            .filter(|e| {
                let ed = h.edge(*e);
                matches!(ed.curve, Curve3::Line { .. }) && ed.start().z > 1.99 && ed.end().z > 1.99 && (ed.start().y - 1.5).abs() < 0.6 && (ed.start().y - 1.5).abs() > 0.1
            })
            .collect();
        assert!(rims.len() >= 2);
        for blend in [Blend::Fillet(0.1), Blend::Chamfer(0.1)] {
            let r = blend_edges(&h, &rims, blend).unwrap();
            assert_eq!(open_edges(&r), 0, "{blend:?}");
            let removed = vol(&h) - vol(&r);
            assert!(removed > 0.0 && removed < 2.0 * 0.01 * 4.0, "{blend:?} removed {removed}");
        }
    }

    #[test]
    fn stops_a_pocket_edge_blend_at_the_hole_it_runs_into() {
        // A side hole breaks through the pocket's floor; the floor's inner
        // edges end on the hole's curved wall.
        use crate::ops::boolean::{boolean, BoolOp};
        let b = make_box(&Frame::WORLD, DVec3::new(3.0, 2.0, 2.0));
        let pocket = make_box(&Frame { origin: DVec3::new(1.0, 0.3, 1.0), ..Frame::WORLD }, DVec3::new(1.0, 1.4, 2.0));
        let hole = make_cylinder(&Frame::new(DVec3::new(-1.0, 1.2, 0.8), DVec3::X, DVec3::Y), 0.3, 5.0);
        let h = boolean(&boolean(&b, &pocket, BoolOp::Difference).unwrap(), &hole, BoolOp::Difference).unwrap();
        let inner: Vec<EdgeId> = h
            .edge_ids()
            .filter(|e| {
                let ed = h.edge(*e);
                let (p, q) = (ed.start(), ed.end());
                (p.z - 1.0).abs() < 1e-9 && (q.z - 1.0).abs() < 1e-9 && (p.x - q.x).abs() < 1e-9 && p.y.min(q.y) < 0.31
            })
            .collect();
        assert_eq!(inner.len(), 2);
        for blend in [Blend::Fillet(0.1), Blend::Chamfer(0.1)] {
            let r = blend_edges(&h, &inner, blend).unwrap();
            assert_eq!(open_edges(&r), 0, "{blend:?}");
            let added = vol(&r) - vol(&h);
            assert!(added > 0.0 && added < 2.0 * 0.01 * 0.7, "{blend:?} added {added}");
        }
    }

    #[test]
    fn blends_vertical_edges_after_the_top_ones() {
        // The vertical edges then run into the earlier blends' faces, and
        // straight on into their end arcs.
        let b = make_box(&Frame::WORLD, DVec3::new(3.0, 2.0, 1.0));
        let top: Vec<EdgeId> = b.edge_ids().filter(|e| b.edge(*e).start().z > 0.9 && b.edge(*e).end().z > 0.9).collect();
        for (first, second) in [(Blend::Chamfer(0.3), Blend::Chamfer(0.2)), (Blend::Fillet(0.3), Blend::Fillet(0.2)), (Blend::Fillet(0.2), Blend::Fillet(0.3)), (Blend::Fillet(0.3), Blend::Chamfer(0.2))] {
            let a = blend_edges(&b, &top, first).unwrap();
            let v = a
                .edge_ids()
                .find(|e| {
                    let ed = a.edge(*e);
                    (ed.start().x - ed.end().x).abs() < 1e-9 && (ed.start().y - ed.end().y).abs() < 1e-9
                })
                .unwrap();
            let r = blend_edges(&a, &[v], second).unwrap();
            assert_eq!(open_edges(&r), 0, "{first:?} then {second:?}");
            assert!(vol(&r) < vol(&a));
        }
    }

    #[test]
    fn fillets_the_edge_between_a_chamfer_and_the_cylinder() {
        // Chamfer a cylinder's top rim, then round the chamfer's lower
        // edge: a circle between a cone and a cylinder, blended by a torus.
        let c = make_cylinder(&Frame::WORLD, 1.0, 2.0);
        let rim = c.edge_ids().find(|e| c.edge(*e).start().z > 1.9 && matches!(c.edge(*e).curve, Curve3::Circle { .. })).unwrap();
        let ch = blend_edges(&c, &[rim], Blend::Chamfer(0.2)).unwrap();
        let low = ch
            .edge_ids()
            .find(|e| {
                let ed = ch.edge(*e);
                (ed.start().z - 1.8).abs() < 1e-6 && matches!(ed.curve, Curve3::Circle { .. })
            })
            .unwrap();
        let f = blend_edges(&ch, &[low], Blend::Fillet(0.1)).unwrap();
        assert_eq!(open_edges(&f), 0);
        // Removed: r^2 (cot(t/2) - (pi - t)/2) about the rim, t = 135 degrees.
        let t = 0.75 * PI;
        let expect = 2.0 * PI * 0.01 * (1.0 / (t / 2.0).tan() - (PI - t) / 2.0);
        let removed = vol(&ch) - vol(&f);
        assert!((removed - expect).abs() < 0.2 * expect, "{removed} vs {expect}");
    }

    fn find_edge(b: &Body, a: DVec3, c: DVec3) -> EdgeId {
        b.edge_ids()
            .find(|e| {
                let ed = b.edge(*e);
                (ed.start().distance(a) < 1e-9 && ed.end().distance(c) < 1e-9) || (ed.start().distance(c) < 1e-9 && ed.end().distance(a) < 1e-9)
            })
            .unwrap()
    }

    #[test]
    fn fillet_one_box_edge() {
        let b = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
        let e = find_edge(&b, DVec3::new(0.0, 0.0, 2.0), DVec3::new(4.0, 0.0, 2.0));
        let r = 0.5;
        let f = blend_edges(&b, &[e], Blend::Fillet(r)).unwrap();
        assert_eq!(open_edges(&f), 0);
        let expect = 24.0 - (r * r - PI * r * r / 4.0) * 4.0;
        assert!((vol(&f) - expect).abs() < 1e-3, "{} vs {expect}", vol(&f));
        let c = blend_edges(&b, &[e], Blend::Chamfer(r)).unwrap();
        assert!((vol(&c) - (24.0 - r * r / 2.0 * 4.0)).abs() < 1e-6, "{}", vol(&c));
    }

    #[test]
    fn fillet_box_corner_three_edges() {
        let b = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
        let v = DVec3::new(4.0, 3.0, 2.0);
        let es = [
            find_edge(&b, v, DVec3::new(0.0, 3.0, 2.0)),
            find_edge(&b, v, DVec3::new(4.0, 0.0, 2.0)),
            find_edge(&b, v, DVec3::new(4.0, 3.0, 0.0)),
        ];
        let r = 0.5;
        let f = blend_edges(&b, &es, Blend::Fillet(r)).unwrap();
        assert_eq!(open_edges(&f), 0);
        // Removed: three edge spandrels (lengths minus r at the corner) and
        // the corner cube minus an eighth ball.
        let spandrel = r * r * (1.0 - PI / 4.0);
        let removed = spandrel * ((4.0 - r) + (3.0 - r) + (2.0 - r)) + (r * r * r - PI * r * r * r / 6.0);
        let expect = 24.0 - removed;
        assert!((vol(&f) - expect).abs() < 2e-3, "{} vs {expect}", vol(&f));
    }

    #[test]
    fn fillet_concave_edge() {
        // L-shaped block: the inner edge gets material added.
        let a = make_box(&Frame::WORLD, DVec3::new(4.0, 2.0, 1.0));
        let c = make_box(&Frame::WORLD, DVec3::new(1.0, 2.0, 3.0));
        let l = boolean(&a, &c, BoolOp::Union).unwrap();
        let e = find_edge(&l, DVec3::new(1.0, 0.0, 1.0), DVec3::new(1.0, 2.0, 1.0));
        let r = 0.4;
        let f = blend_edges(&l, &[e], Blend::Fillet(r)).unwrap();
        assert_eq!(open_edges(&f), 0);
        let expect = vol(&l) + r * r * (1.0 - PI / 4.0) * 2.0;
        assert!((vol(&f) - expect).abs() < 1e-3, "{} vs {expect}", vol(&f));
    }

    #[test]
    fn fillet_cylinder_rims() {
        // Boss top rim (convex) and hole rim on a plate.
        let cyl = make_cylinder(&Frame::WORLD, 2.0, 3.0);
        let top = cyl.edge_ids().find(|e| matches!(cyl.edge(*e).curve, Curve3::Circle { .. }) && cyl.edge(*e).start().z > 2.0).unwrap();
        let r = 0.5;
        let f = blend_edges(&cyl, &[top], Blend::Fillet(r)).unwrap();
        assert_eq!(open_edges(&f), 0);
        // Pappus on the removed spandrel ring.
        let area = r * r * (1.0 - PI / 4.0);
        let rho = 2.0 - r + (r * r * r / 6.0) / area;
        // Measure against the tessellated cylinder to cancel chord error.
        let base = vol(&cyl);
        let expect = base - area * 2.0 * PI * rho;
        assert!((vol(&f) - expect).abs() < 2e-3, "{} vs {expect}", vol(&f));
        let ch = blend_edges(&cyl, &[top], Blend::Chamfer(r)).unwrap();
        assert_eq!(open_edges(&ch), 0);
        let rho_c = 2.0 - r / 3.0;
        let expect_c = base - r * r / 2.0 * 2.0 * PI * rho_c;
        assert!((vol(&ch) - expect_c).abs() < 2e-3, "{} vs {expect_c}", vol(&ch));
    }

    #[test]
    fn fillet_hole_rim() {
        let plate = make_box(&Frame::new(DVec3::new(-3.0, -3.0, 0.0), DVec3::Z, DVec3::X), DVec3::new(6.0, 6.0, 2.0));
        let hole = make_cylinder(&Frame::new(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, DVec3::X), 1.0, 4.0);
        let b = boolean(&plate, &hole, BoolOp::Difference).unwrap();
        let rim = b.edge_ids().find(|e| matches!(b.edge(*e).curve, Curve3::Circle { .. }) && b.edge(*e).start().z > 1.0).unwrap();
        let r = 0.3;
        let f = blend_edges(&b, &[rim], Blend::Fillet(r)).unwrap();
        assert_eq!(open_edges(&f), 0);
        let area = r * r * (1.0 - PI / 4.0);
        let xbar = r * r * r * (5.0 / 6.0 - PI / 4.0) / area;
        let expect = vol(&b) - area * 2.0 * PI * (1.0 + xbar);
        assert!((vol(&f) - expect).abs() < 2e-3, "{} vs {expect}", vol(&f));
    }

    #[test]
    fn rounded_box_all_edges() {
        let b = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
        let all: Vec<EdgeId> = b.edge_ids().collect();
        let r = 0.4;
        let f = blend_edges(&b, &all, Blend::Fillet(r)).unwrap();
        assert_eq!(open_edges(&f), 0);
        // Minkowski sum of the inner box with a ball of radius r.
        let (a, c, d) = (4.0 - 2.0 * r, 3.0 - 2.0 * r, 2.0 - 2.0 * r);
        let expect = a * c * d + 2.0 * r * (a * c + c * d + a * d) + PI * r * r * (a + c + d) + 4.0 / 3.0 * PI * r * r * r;
        assert!((vol(&f) - expect).abs() < 5e-3, "{} vs {expect}", vol(&f));
    }
}

#[cfg(test)]
mod chain_tests {
    use super::*;
    use crate::ops::boolean::open_edges;
    use crate::ops::heal::compact;
    use crate::tess::{tessellate, TessOptions};
    use std::f64::consts::PI;

    fn vol(b: &Body) -> f64 {
        tessellate(b, &TessOptions { chord_tol: 1e-4, angle_tol: 0.05 }).volume()
    }

    #[test]
    fn rounded_corners_then_top_edges() {
        // The classic: round the vertical edges, then fillet the whole top
        // rim in one go as a tangent chain.
        let (w, d, h, c, r) = (4.0, 3.0, 2.0, 0.8, 0.3);
        let b = make_box(&Frame::WORLD, DVec3::new(w, d, h));
        let vertical: Vec<EdgeId> = b.edge_ids().filter(|e| (b.edge(*e).start() - b.edge(*e).end()).z.abs() > 1.0).collect();
        let b = compact(&blend_edges(&b, &vertical, Blend::Fillet(c)).unwrap());
        let top: Vec<EdgeId> = b.edge_ids().filter(|e| {
            let ed = b.edge(*e);
            ed.start().z > h - 1e-6 && ed.end().z > h - 1e-6
        }).collect();
        assert_eq!(top.len(), 8);
        let f = blend_edges(&b, &top, Blend::Fillet(r)).unwrap();
        assert_eq!(open_edges(&f), 0);
        let spandrel = r * r * (1.0 - PI / 4.0);
        let dbar = r * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI);
        let path = 2.0 * (w - 2.0 * c) + 2.0 * (d - 2.0 * c) + 2.0 * PI * (c - dbar);
        let expect = vol(&b) - spandrel * path;
        assert!((vol(&f) - expect).abs() < 2e-3 * spandrel * path, "{} vs {expect}", vol(&f));
    }
}

#[cfg(test)]
mod smooth_tests {
    use super::*;
    use crate::ops::boolean::open_edges;
    use crate::ops::heal::compact;
    use crate::tess::{tessellate, TessOptions};
    use std::f64::consts::PI;

    fn vol(b: &Body) -> f64 {
        tessellate(b, &TessOptions { chord_tol: 1e-4, angle_tol: 0.05 }).volume()
    }

    /// Area cut off a right-angled corner by the smooth section with
    /// contact points `r` from the corner (shoelace over the quintic).
    fn smooth_corner_area(r: f64) -> f64 {
        let ctrl = crate::ops::roll::section_ctrl(Blend::Smooth(r), DVec3::new(r, r, 0.0), DVec3::new(r, 0.0, 0.0), DVec3::new(0.0, r, 0.0));
        let c = ductile_geom::NurbsCurve::new(5, vec![0.0; 6].into_iter().chain(vec![1.0; 6]).collect(), ctrl);
        let n = 4000;
        let pts: Vec<DVec3> = (0..=n).map(|i| c.point(i as f64 / n as f64)).collect();
        // Polygon corner -> (r,0) -> curve -> (0,r) -> corner.
        let mut poly = vec![DVec3::ZERO];
        poly.extend(pts);
        let mut a = 0.0;
        for i in 0..poly.len() {
            let (p, q) = (poly[i], poly[(i + 1) % poly.len()]);
            a += p.x * q.y - q.x * p.y;
        }
        a.abs() / 2.0
    }

    #[test]
    fn smooth_box_all_edges() {
        let (w, d, h, r) = (4.0, 3.0, 2.0, 0.5);
        let b = make_box(&Frame::WORLD, DVec3::new(w, d, h));
        let all: Vec<EdgeId> = b.edge_ids().collect();
        let t = std::time::Instant::now();
        let f = blend_edges(&b, &all, Blend::Smooth(r)).unwrap();
        eprintln!("smooth box: {:?}", t.elapsed());
        assert_eq!(open_edges(&f), 0);
        // Six faces, twelve edge blends, eight corners of three patches.
        assert_eq!(f.faces.len(), 6 + 12 + 24);
        assert!(worst_crease(&f) < 1e-3);
        let edges = 4.0 * ((w - 2.0 * r) + (d - 2.0 * r) + (h - 2.0 * r));
        let removed = smooth_corner_area(r) * edges + 8.0 * r * r * r * crate::ops::corner::removed_fraction(80);
        let expect = w * d * h - removed;
        assert!((vol(&f) - expect).abs() < 2e-3 * removed, "{} vs {expect}", vol(&f));
    }

    #[test]
    fn smooth_top_rim_with_sharp_corners() {
        // The four top edges: neighbouring blends run into each other in
        // mitres over the sharp vertical edges.
        let (w, d, h, r) = (4.0, 3.0, 2.0, 0.5);
        let b = make_box(&Frame::WORLD, DVec3::new(w, d, h));
        let top: Vec<EdgeId> = b.edge_ids().filter(|e| { let ed = b.edge(*e); ed.start().z > 1.9 && ed.end().z > 1.9 }).collect();
        let f = blend_edges(&b, &top, Blend::Smooth(r)).unwrap();
        assert_eq!(open_edges(&f), 0);
        // Over each corner square the two blends overlap in a mitre.
        let a = smooth_corner_area(r);
        let removed = a * 2.0 * ((w - 2.0 * r) + (d - 2.0 * r)) + 4.0 * mitre_corner(r);
        let expect = w * d * h - removed;
        assert!((vol(&f) - expect).abs() < 2e-3 * removed, "{} vs {expect}", vol(&f));
    }

    /// Material removed over an r-by-r corner square where two smooth blends
    /// meet in a mitre: there the deeper of the two sections governs.
    fn mitre_corner(r: f64) -> f64 {
        let ctrl = crate::ops::roll::section_ctrl(Blend::Smooth(r), DVec3::new(r, r, 0.0), DVec3::new(r, 0.0, 0.0), DVec3::new(0.0, r, 0.0));
        let c = ductile_geom::NurbsCurve::new(5, vec![0.0; 6].into_iter().chain(vec![1.0; 6]).collect(), ctrl);
        // depth(x): how deep the section cuts at distance x from the side
        // face (the section runs from (r, 0) on the top to (0, r) on the
        // side, about the corner at the origin).
        let n = 4000;
        let pts: Vec<DVec3> = (0..=n).map(|i| c.point(i as f64 / n as f64)).collect();
        let depth = |x: f64| -> f64 {
            let mut best = (f64::INFINITY, 0.0);
            for p in &pts {
                let e = (p.x - x).abs();
                if e < best.0 {
                    best = (e, p.y);
                }
            }
            best.1
        };
        // At (x, y) over the square the deeper cut is depth(min(x, y)).
        let m = 400;
        let mut v = 0.0;
        for i in 0..m {
            for j in 0..m {
                let (x, y) = ((i as f64 + 0.5) * r / m as f64, (j as f64 + 0.5) * r / m as f64);
                v += depth(x.min(y)) * (r / m as f64).powi(2);
            }
        }
        v
    }

    #[test]
    fn smooth_blend_on_a_box_edge() {
        let b = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
        let e = b.edge_ids().find(|e| {
            let ed = b.edge(*e);
            ed.start().z > 1.9 && ed.end().z > 1.9 && ed.start().y.abs() < 1e-9 && ed.end().y.abs() < 1e-9
        }).unwrap();
        let r = 0.5;
        let f = blend_edges(&b, &[e], Blend::Smooth(r)).unwrap();
        assert_eq!(open_edges(&f), 0);
        let removed = 24.0 - vol(&f);
        let expect = smooth_corner_area(r) * 4.0;
        // Shallower than the round fillet in the middle, flatter at the ends.
        assert!(expect < 4.0 * r * r * (1.0 - PI / 4.0));
        assert!((removed - expect).abs() < 1e-3 * expect, "{removed} vs {expect}");
    }

    #[test]
    fn smooth_rims_on_a_soap_bar() {
        let b = make_box(&Frame::WORLD, DVec3::new(6.0, 4.0, 2.0));
        let vertical: Vec<EdgeId> = b.edge_ids().filter(|e| (b.edge(*e).start() - b.edge(*e).end()).z.abs() > 1.0).collect();
        let b = compact(&blend_edges(&b, &vertical, Blend::Smooth(1.2)).unwrap());
        assert_eq!(open_edges(&b), 0);
        let rims: Vec<EdgeId> = b.edge_ids().filter(|e| (b.edge(*e).start().z - b.edge(*e).end().z).abs() < 1e-9 && b.edge(*e).start().z > 1.99).collect();
        let f = blend_edges(&b, &rims, Blend::Smooth(0.4)).unwrap();
        assert_eq!(open_edges(&f), 0);
        assert!(vol(&f) < vol(&b));
    }

    /// Largest angle between the two faces' normals across any edge.
    fn worst_crease(b: &Body) -> f64 {
        let mut worst: f64 = 0.0;
        for e in b.edge_ids() {
            let uses = b.edge_faces(e);
            if uses.len() != 2 {
                continue;
            }
            let ed = b.edge(e);
            for k in 1..4 {
                let p = ed.curve.point(ed.t0 + (ed.t1 - ed.t0) * k as f64 / 4.0);
                let (n1, n2) = (outward(b, uses[0].0, p), outward(b, uses[1].0, p));
                worst = worst.max(n1.angle_between(n2));
            }
        }
        worst
    }

    #[test]
    fn smooth_hexagonal_prism() {
        // Faces meeting at 120 and 90 degrees: oblique G2 corners.
        let (r0, h, r) = (3.0, 2.0, 0.4);
        let mut pts = Vec::new();
        for z in [0.0, h] {
            for k in 0..6 {
                let a = k as f64 * PI / 3.0;
                pts.push(DVec3::new(r0 * a.cos(), r0 * a.sin(), z));
            }
        }
        let mut faces = vec![(0..6).rev().collect::<Vec<_>>(), (6..12).collect()];
        for k in 0..6 {
            faces.push(vec![k, (k + 1) % 6, 6 + (k + 1) % 6, 6 + k]);
        }
        let b = polyhedron(&pts, &faces, BodyKind::Solid);
        assert!(vol(&b) > 0.0);
        let all: Vec<EdgeId> = b.edge_ids().collect();
        let f = blend_edges(&b, &all, Blend::Smooth(r)).unwrap();
        assert_eq!(open_edges(&f), 0);
        assert_eq!(f.faces.len(), 8 + 18 + 12 * 3);
        let crease = worst_crease(&f);
        assert!(crease < 2e-3, "crease {crease}");
        let (vf, vb) = (vol(&f), vol(&b));
        assert!(vf < vb && vf > vb - 1.5, "{vf} vs {vb}");
    }

    #[test]
    fn variable_round_and_smooth_blends() {
        // A box edge whose radius grows from 0.3 to 0.8 along it: the
        // round blend is a cone, removing (1 - pi/4) r^2 per unit length.
        let (w, d, h) = (4.0, 3.0, 2.0);
        let b = make_box(&Frame::WORLD, DVec3::new(w, d, h));
        let e = b.edge_ids().find(|e| {
            let ed = b.edge(*e);
            ed.start().z > 1.9 && ed.end().z > 1.9 && ed.start().y.abs() < 1e-9 && ed.end().y.abs() < 1e-9
        }).unwrap();
        let (r0, r1) = (0.3, 0.8);
        let mean_sq = (r0 * r0 + r0 * r1 + r1 * r1) / 3.0;
        let f = blend_edges_variable(&b, &[e], r0, r1, false).unwrap();
        assert_eq!(open_edges(&f), 0);
        assert_eq!(f.faces.len(), 7);
        let removed = w * d * h - vol(&f);
        let expect = (1.0 - PI / 4.0) * mean_sq * w;
        assert!((removed - expect).abs() < 1e-3 * expect, "{removed} vs {expect}");
        // The radius really varies: the blend is narrow at one end, wide
        // at the other.
        let ed = b.edge(e);
        let (start, end) = (ed.start(), ed.end());
        let width = |p: DVec3| f.vertices.iter().filter(|v| (v.point - p).dot(end - start).abs() < 1e-6 && v.point.z > h - 1e-6).map(|v| v.point.y).filter(|y| *y > 1e-9).fold(f64::INFINITY, f64::min);
        assert!((width(start) - r0).abs() < 1e-9 && (width(end) - r1).abs() < 1e-9, "{} {}", width(start), width(end));
        let f = blend_edges_variable(&b, &[e], r0, r1, true).unwrap();
        assert_eq!(open_edges(&f), 0);
        let removed = w * d * h - vol(&f);
        let expect = smooth_corner_area(1.0) * mean_sq * w;
        assert!((removed - expect).abs() < 1e-3 * expect, "{removed} vs {expect}");
    }

    #[test]
    fn variable_blend_between_slanted_ends() {
        // A sheared box: the top front edge ends in slanted faces, so the
        // cone is cut by oblique planes (exact rational quartics).
        let pts = [(0.0, 0.0, 0.0), (4.0, 0.0, 0.0), (4.0, 3.0, 0.0), (0.0, 3.0, 0.0), (1.0, 0.0, 2.0), (5.0, 0.0, 2.0), (5.0, 3.0, 2.0), (1.0, 3.0, 2.0)].map(|(x, y, z)| DVec3::new(x, y, z));
        let faces = vec![vec![0, 3, 2, 1], vec![4, 5, 6, 7], vec![0, 1, 5, 4], vec![1, 2, 6, 5], vec![2, 3, 7, 6], vec![3, 0, 4, 7]];
        let b = polyhedron(&pts, &faces, BodyKind::Solid);
        let e = b.edge_ids().find(|e| {
            let ed = b.edge(*e);
            ed.start().z > 1.9 && ed.end().z > 1.9 && ed.start().y.abs() < 1e-9 && ed.end().y.abs() < 1e-9
        }).unwrap();
        for smooth in [false, true] {
            let f = blend_edges_variable(&b, &[e], 0.3, 0.8, smooth).unwrap();
            assert_eq!(open_edges(&f), 0);
            assert!(vol(&f) < vol(&b));
            // Nearly constant radius matches the constant blend.
            let near = blend_edges_variable(&b, &[e], 0.5, 0.5 + 1e-7, smooth).unwrap();
            let constant = blend_edges(&b, &[e], if smooth { Blend::Smooth(0.5) } else { Blend::Fillet(0.5) }).unwrap();
            assert!((vol(&near) - vol(&constant)).abs() < 1e-4, "{} vs {}", vol(&near), vol(&constant));
        }
    }
}

#[cfg(test)]
mod arc_tests {
    use super::*;
    use crate::ops::boolean::open_edges;
    use crate::primitives::{make_box, make_cylinder};
    use crate::tess::{tessellate, TessOptions};
    use ductile_geom::Frame;
    use std::f64::consts::PI;

    fn vol(b: &Body) -> f64 {
        tessellate(b, &TessOptions { chord_tol: 1e-4, angle_tol: 0.05 }).volume()
    }

    #[test]
    fn fillets_the_arc_ends_of_a_groove() {
        // A half-round groove along the top of a box: its ends are
        // half-circle edges whose tools must stop at the top face.
        let bx = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
        let groove = boolean(&bx, &make_cylinder(&Frame::new(DVec3::new(-1.0, 1.5, 2.0), DVec3::X, DVec3::Y), 0.7, 6.0), BoolOp::Difference).unwrap();
        let arcs: Vec<EdgeId> = groove.edge_ids().filter(|e| matches!(groove.edge(*e).curve, Curve3::Circle { .. }) && !groove.edge(*e).is_closed()).collect();
        assert_eq!(arcs.len(), 2);
        let before = vol(&groove);
        for e in &arcs {
            let r = blend_edges(&groove, &[*e], Blend::Fillet(0.2)).unwrap();
            assert_eq!(open_edges(&r), 0);
            let cut = before - vol(&r);
            // Pappus: half a turn of the corner section, area (1 - pi/4) r^2,
            // whose centroid sits (10 - 3 pi)/(12 - 3 pi) r out from the
            // groove's radius.
            let r: f64 = 0.2;
            let rho = 0.7 + (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI) * r;
            let expect = PI * rho * (1.0 - PI / 4.0) * r * r;
            assert!((cut - expect).abs() < 1e-3 * expect.max(0.1), "{cut} {expect}");
        }
        // A seam picked with real edges is passed over.
        let seam = groove.edge_ids().find(|e| groove.edge_faces(*e).len() == 2 && groove.edge_faces(*e)[0].0 == groove.edge_faces(*e)[1].0);
        if let Some(s) = seam {
            assert!(blend_edges(&groove, &[s, arcs[0]], Blend::Fillet(0.2)).is_ok());
        }
    }
}

/// Full round: replaces `face` (the top of a rib, between two parallel
/// planar side faces) with a half cylinder tangent to both sides, its
/// radius half the rib's width.
pub fn full_round(body: &Body, face: FaceId) -> Result<Body, FilletError> {
    if !body.face(face).surface.is_planar() {
        return Err(unsupported(line!(), file!()));
    }
    let pts: Vec<DVec3> = body.face_edges(face).iter().flat_map(|e| {
        let ed = body.edge(*e);
        (0..=8).map(move |i| ed.curve.point(ed.t0 + (ed.t1 - ed.t0) * i as f64 / 8.0))
    }).collect();
    if pts.is_empty() {
        return Err(unsupported(line!(), file!()));
    }
    let c = pts.iter().copied().sum::<DVec3>() / pts.len() as f64;
    let n = outward(body, face, c);
    // The two sides: planar neighbours facing apart, square to the face.
    let mut sides: Vec<(FaceId, DVec3, DVec3)> = Vec::new();
    for e in body.face_edges(face) {
        for (g, _) in body.edge_faces(e) {
            if g == face || sides.iter().any(|s| s.0 == g) || !body.face(g).surface.is_planar() {
                continue;
            }
            let ed = body.edge(e);
            let p = ed.curve.point(0.5 * (ed.t0 + ed.t1));
            let m = outward(body, g, p);
            if m.dot(n).abs() < 0.3 {
                sides.push((g, m, p));
            }
        }
    }
    let pair = sides.iter().enumerate().flat_map(|(i, a)| sides[i + 1..].iter().map(move |b| (*a, *b))).find(|(a, b)| a.1.dot(b.1) < -0.999);
    let Some(((_, ma, pa), (_, _, pb))) = pair else { return Err(unsupported(line!(), file!())) };
    let s = ma.normalize();
    let w = (pa - pb).dot(s).abs();
    if w < 1e-9 {
        return Err(FilletError::TooLarge);
    }
    let t = n.cross(s).normalize();
    let up = s.cross(t).normalize() * n.dot(s.cross(t)).signum();
    let r = 0.5 * w;
    // The section in the plane square to the rib, at the face's start.
    let t0 = pts.iter().map(|p| (*p - c).dot(t)).fold(f64::INFINITY, f64::min);
    let t1 = pts.iter().map(|p| (*p - c).dot(t)).fold(f64::NEG_INFINITY, f64::max);
    let top = pts.iter().map(|p| (*p - c).dot(up)).fold(f64::NEG_INFINITY, f64::max);
    let low = pts.iter().map(|p| (*p - c).dot(up)).fold(f64::INFINITY, f64::min);
    let mid_s = (pa - c).dot(s) - r;
    let o = c + t * t0 + s * mid_s;
    let centre = o + up * (low - r);
    let (l, rgt) = (centre - s * r, centre + s * r);
    let crown = centre + up * r;
    let lift = up * (top - (low - r) + r);
    let outer = Loop3 {
        segments: vec![
            Segment::line(l, l + lift),
            Segment::line(l + lift, rgt + lift),
            Segment::line(rgt + lift, rgt),
            arc(centre, rgt, crown, t),
            arc(centre, crown, l, t),
        ],
    };
    let region = Region { plane: Frame::new(o, t, s), outer, holes: vec![] };
    let tool = extrude_region(&region, t * (t1 - t0)).ok_or_else(|| unsupported(line!(), file!()))?;
    Ok(boolean(body, &tool, BoolOp::Difference)?)
}

#[cfg(test)]
mod full_round_tests {
    use super::*;

    #[test]
    fn rounds_the_top_of_a_rib() {
        // A rib 2 wide, 3 tall, 6 long standing on a base plate.
        let rib = make_box(&Frame { origin: DVec3::new(0.0, 0.0, 0.0), ..Frame::WORLD }, DVec3::new(6.0, 2.0, 3.0));
        let top = rib.face_ids().find(|f| rib.face_edges(*f).iter().all(|e| rib.edge(*e).start().z > 2.99 && rib.edge(*e).end().z > 2.99)).unwrap();
        let out = full_round(&rib, top).unwrap();
        let v = crate::tessellate(&out, &crate::TessOptions { chord_tol: 1e-4, angle_tol: 0.05 }).volume();
        let want = 6.0 * 2.0 * 3.0 - 6.0 * (2.0 * 1.0 - std::f64::consts::PI * 0.5);
        assert!((v - want).abs() < 4e-3, "{v} {want}");
        assert!(out.faces.iter().any(|f| matches!(f.surface, Surface::Cylinder { .. })));
    }
}
