// SPDX-License-Identifier: GPL-3.0-only
//! Rolling-ball blends along chains of tangent edges.
//!
//! A chain is a run of edges meeting tangentially (the top edges of an
//! extruded rounded rectangle, a spline profile's edges, the side edge of a
//! prism). The ball is rolled along the whole run and every edge gets its
//! own blend face; neighbouring blend faces share their junction section,
//! an exact arc, so they meet without gaps. The faces along the chain are
//! cut back to the contact curves, the edges that ended at the chain's
//! vertices are shortened to the new contact points, and an open chain is
//! closed off by the face its ends run into (the cap of a prism).

use super::fillet::{Blend, FilletError};
use super::roll::{ball, move_end, section_ctrl};
use crate::topo::{Body, Coedge, EdgeId, FaceId, VertexId};
use ductile_geom::{Curve3, DVec2, DVec3, NurbsCurve, NurbsSurface, Surface};

/// One edge of a chain, with `fwd` true when the chain runs along the
/// edge's own direction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Link {
    pub edge: EdgeId,
    pub fwd: bool,
}

/// The edges running on smoothly from `e` through each of its vertices,
/// for as long as exactly one edge continues tangentially and `accept`
/// takes it (as picking one edge of a filleted outline picks the whole
/// outline), `e` included.
pub fn grow_tangent_chain(body: &Body, e: EdgeId, accept: &dyn Fn(EdgeId) -> bool) -> Vec<EdgeId> {
    let dir_at = |x: EdgeId, v: VertexId| {
        let ed = body.edge(x);
        // Pointing away from `v` along the edge.
        if ed.v0 == v { ed.curve.tangent(ed.t0) } else { -ed.curve.tangent(ed.t1) }.normalize_or_zero()
    };
    let mut out = vec![e];
    if body.edge(e).is_closed() {
        return out;
    }
    let cos = (2.0f64).to_radians().cos();
    for start_end in [true, false] {
        let (mut cur, mut v) = {
            let ed = body.edge(e);
            (e, if start_end { ed.v1 } else { ed.v0 })
        };
        loop {
            let into = -dir_at(cur, v);
            let next: Vec<EdgeId> = body
                .edge_ids()
                .filter(|x| *x != cur && !body.edge(*x).is_closed() && (body.edge(*x).v0 == v || body.edge(*x).v1 == v))
                .filter(|x| dir_at(*x, v).dot(into) > cos)
                .collect();
            let [x] = next[..] else { break };
            if out.contains(&x) || !accept(x) {
                break;
            }
            out.push(x);
            let xe = body.edge(x);
            v = if xe.v0 == v { xe.v1 } else { xe.v0 };
            cur = x;
        }
    }
    out
}

/// Groups edges into chains of tangent edges. Each chain is listed in
/// order with its direction, and flagged when it closes on itself.
pub fn tangent_chains(body: &Body, edges: &[EdgeId]) -> Vec<(Vec<Link>, bool)> {
    let tangent_at = |e: EdgeId, v: VertexId, out: bool| {
        let ed = body.edge(e);
        let t = if ed.v0 == v { ed.curve.tangent(ed.t0) } else { -ed.curve.tangent(ed.t1) };
        let t = t.normalize_or_zero();
        if out {
            t
        } else {
            -t
        }
    };
    // The selected edge continuing `e` smoothly through `v`, if exactly one does.
    let next = |e: EdgeId, v: VertexId, used: &[EdgeId]| -> Option<EdgeId> {
        let at: Vec<EdgeId> = edges.iter().copied().filter(|x| *x != e && { let xe = body.edge(*x); xe.v0 == v || xe.v1 == v }).collect();
        if at.len() != 1 || used.contains(&at[0]) || body.edge(at[0]).is_closed() {
            return None;
        }
        let a = tangent_at(e, v, false);
        let b = tangent_at(at[0], v, true);
        (a.dot(b) > (2.0f64).to_radians().cos()).then_some(at[0])
    };
    let mut done: Vec<EdgeId> = Vec::new();
    let mut out = Vec::new();
    for &e in edges {
        if done.contains(&e) {
            continue;
        }
        let ed = body.edge(e);
        if ed.is_closed() {
            done.push(e);
            out.push((vec![Link { edge: e, fwd: true }], true));
            continue;
        }
        let mut chain = vec![Link { edge: e, fwd: true }];
        done.push(e);
        // Forward from the end vertex.
        let closed;
        loop {
            let last = *chain.last().unwrap();
            let le = body.edge(last.edge);
            let v = if last.fwd { le.v1 } else { le.v0 };
            if v == ed.v0 && chain.len() > 1 {
                closed = true;
                break;
            }
            match next(last.edge, v, &done) {
                Some(x) => {
                    done.push(x);
                    chain.push(Link { edge: x, fwd: body.edge(x).v0 == v });
                }
                None => {
                    closed = false;
                    break;
                }
            }
        }
        if !closed {
            // Backward from the start vertex.
            loop {
                let first = chain[0];
                let fe = body.edge(first.edge);
                let v = if first.fwd { fe.v0 } else { fe.v1 };
                match next(first.edge, v, &done) {
                    Some(x) => {
                        done.push(x);
                        chain.insert(0, Link { edge: x, fwd: body.edge(x).v1 == v });
                    }
                    None => break,
                }
            }
        }
        out.push((chain, closed));
    }
    out
}

#[derive(Clone)]
struct Section {
    p1: DVec3,
    p2: DVec3,
    curve: NurbsCurve,
}

/// Blends a chain of tangent edges (not a single closed edge; see
/// `roll::roll_closed_edge`). Old entities are left unreferenced, so ids
/// stay valid; compact the body afterwards.
pub fn roll_chain(body: &Body, chain: &[Link], closed: bool, blend: Blend) -> Result<Body, FilletError> {
    let k = chain.len();
    let size = body.bbox().diagonal().max(1e-9);
    let reach = match blend {
        Blend::Fillet(r) | Blend::Smooth(r) | Blend::Chamfer(r) => r,
    };
    // Chain vertices: start of each link, plus the end of the last one.
    let start_of = |l: &Link| {
        let e = body.edge(l.edge);
        if l.fwd {
            e.v0
        } else {
            e.v1
        }
    };
    let end_of = |l: &Link| {
        let e = body.edge(l.edge);
        if l.fwd {
            e.v1
        } else {
            e.v0
        }
    };
    let mut verts: Vec<VertexId> = chain.iter().map(start_of).collect();
    verts.push(end_of(&chain[k - 1]));
    for i in 1..k {
        if start_of(&chain[i]) != end_of(&chain[i - 1]) {
            return Err(super::fillet::unsupported(line!(), file!()));
        }
    }
    if closed != (verts[0] == verts[k]) {
        return Err(super::fillet::unsupported(line!(), file!()));
    }
    // Side A runs along the chain, side B against it.
    let mut sides: Vec<(FaceId, FaceId)> = Vec::with_capacity(k);
    for l in chain {
        let uses = body.edge_faces(l.edge);
        if uses.len() != 2 || uses[0].0 == uses[1].0 {
            return Err(FilletError::NotManifold);
        }
        let along = |c: &Coedge| c.reversed != l.fwd;
        match (along(&uses[0].1), along(&uses[1].1)) {
            (true, false) => sides.push((uses[0].0, uses[1].0)),
            (false, true) => sides.push((uses[1].0, uses[0].0)),
            _ => return Err(FilletError::NotManifold),
        }
    }
    let others = |v: VertexId| -> Vec<EdgeId> {
        body.edge_ids()
            .filter(|x| !chain.iter().any(|l| l.edge == *x))
            .filter(|x| {
                let xe = body.edge(*x);
                (xe.v0 == v || xe.v1 == v) && !body.edge_faces(*x).is_empty()
            })
            .collect()
    };
    // Open ends run into a cap face: two edges leave the end vertex, one
    // on each side, and the face between them is the cap.
    struct End {
        cap: FaceId,
        ea: EdgeId,
        eb: EdgeId,
    }
    let end_at = |j: usize, (fa, fb): (FaceId, FaceId)| -> Result<End, FilletError> {
        // A seam leaving the end runs inside one side's face; it follows
        // that face's contact curve and plays no part in the cap.
        let xs: Vec<EdgeId> = others(verts[j]).into_iter().filter(|x| {
            let fs = body.edge_faces(*x);
            !(fs.len() == 2 && fs[0].0 == fs[1].0)
        }).collect();
        if xs.len() != 2 {
            return Err(super::fillet::unsupported(line!(), file!()));
        }
        let faces = |x: EdgeId| body.edge_faces(x).iter().map(|u| u.0).collect::<Vec<_>>();
        let (f0, f1) = (faces(xs[0]), faces(xs[1]));
        let (ea, eb) = if f0.contains(&fa) && f1.contains(&fb) {
            (xs[0], xs[1])
        } else if f1.contains(&fa) && f0.contains(&fb) {
            (xs[1], xs[0])
        } else {
            return Err(super::fillet::unsupported(line!(), file!()));
        };
        let cap = faces(ea).into_iter().find(|f| *f != fa).ok_or_else(|| super::fillet::unsupported(line!(), file!()))?;
        if !faces(eb).contains(&cap) || cap == fb {
            return Err(super::fillet::unsupported(line!(), file!()));
        }
        Ok(End { cap, ea, eb })
    };
    let ends = if closed { None } else { Some((end_at(0, sides[0])?, end_at(k, sides[k - 1])?)) };

    // Sections. The ball's centre lies in the plane normal to the chain,
    // except at an open end, where it lies in the cap's plane so that the
    // end section sits on the cap.
    let mut convex: Option<bool> = None;
    let mut hints: (Option<DVec2>, Option<DVec2>) = (None, None);
    let mut section = |i: usize, t: f64, plane: Option<DVec3>, hints: &mut (Option<DVec2>, Option<DVec2>)| -> Result<Section, FilletError> {
        let l = chain[i];
        let ed = body.edge(l.edge);
        let (fa, fb) = sides[i];
        let (sa, sb) = (&body.face(fa).surface, &body.face(fb).surface);
        let q = ed.curve.point(t);
        let tan = (ed.curve.tangent(t) * if l.fwd { 1.0 } else { -1.0 }).normalize();
        // A fitted edge can end in a tiny hook whose tangent points back;
        // take the direction from a little way in instead.
        let tan = {
            let inner = t + (0.5 * (ed.t0 + ed.t1) - t) * 0.05;
            let ti = (ed.curve.tangent(inner) * if l.fwd { 1.0 } else { -1.0 }).normalize();
            if tan.dot(ti) < 0.0 {
                // Along the chord from there to here, which keeps to the
                // edge's run even through the hook.
                let fwd = if (t - inner) * (ed.t1 - ed.t0) * if l.fwd { 1.0 } else { -1.0 } > 0.0 { 1.0 } else { -1.0 };
                ((q - ed.curve.point(inner)) * fwd).try_normalize().unwrap_or(ti)
            } else {
                tan
            }
        };
        let uva = sa.project(q, hints.0);
        let uvb = sb.project(q, hints.1);
        let na = body.face_normal(fa, uva.x, uva.y);
        let nb = body.face_normal(fb, uvb.x, uvb.y);
        let cross = na.cross(nb);
        if cross.length() < 1e-6 {
            return Err(FilletError::Smooth);
        }
        let cv = cross.dot(tan) > 0.0;
        if *convex.get_or_insert(cv) != cv {
            return Err(super::fillet::unsupported(line!(), file!()));
        }
        let normal = match plane {
            Some(n) if n.dot(tan) < 0.0 => -n,
            Some(n) => n,
            None => tan,
        };
        let (p1, p2, ctrl) = match blend {
            Blend::Fillet(r) | Blend::Smooth(r) => {
                let sgn = if cv { -1.0 } else { 1.0 };
                let Some((c, p1, p2, h1, h2)) = ball(body, (fa, sa), (fb, sb), q, normal, na, nb, sgn * r, (uva, uvb)) else {
                    if std::env::var_os("DUCTILE_DEBUG_FILLET").is_some() {
                        eprintln!("no ball at {q:.4} (edge {}, t {t:.4})", l.edge.0);
                    }
                    return Err(FilletError::TooLarge);
                };
                *hints = (Some(h1), Some(h2));
                if p1.distance(q) > size || p2.distance(q) > size {
                    return Err(FilletError::TooLarge);
                }
                (p1, p2, section_ctrl(blend, c, p1, p2))
            }
            Blend::Chamfer(d) => {
                let (u1, u2) = (na.cross(tan), tan.cross(nb));
                let h1 = sa.project(q + u1 * d, Some(uva));
                let h2 = sb.project(q + u2 * d, Some(uvb));
                *hints = (Some(uva), Some(uvb));
                let (p1, p2) = (sa.point(h1.x, h1.y), sb.point(h2.x, h2.y));
                (p1, p2, vec![p1.extend(1.0), ((p1 + p2) * 0.5).extend(1.0), p2.extend(1.0)])
            }
        };
        let deg = ctrl.len() - 1;
        let mut knots = vec![0.0; deg + 1];
        knots.extend(std::iter::repeat(1.0).take(deg + 1));
        let curve = NurbsCurve::new(deg, knots, ctrl);
        Ok(Section { p1, p2, curve })
    };
    let cap_normal = |f: FaceId| match &body.face(f).surface {
        Surface::Plane { frame } => Some(frame.z),
        _ => None,
    };
    let mut rows: Vec<Vec<Section>> = Vec::with_capacity(k);
    for (i, l) in chain.iter().enumerate() {
        let ed = body.edge(l.edge);
        let (ta, tb) = if l.fwd { (ed.t0, ed.t1) } else { (ed.t1, ed.t0) };
        let turning: f64 = {
            let m = 32;
            (0..m)
                .map(|s| {
                    let a = ed.curve.tangent(ta + (tb - ta) * s as f64 / m as f64).normalize();
                    let b = ed.curve.tangent(ta + (tb - ta) * (s + 1) as f64 / m as f64).normalize();
                    a.dot(b).clamp(-1.0, 1.0).acos()
                })
                .sum()
        };
        let len = ed.length();
        let m = ((turning / 0.06).max(2.0 * len / reach.max(1e-9)).ceil() as usize).clamp(4, 160);
        let mut row = Vec::with_capacity(m + 1);
        for s in 0..=m {
            if s == 0 && i > 0 {
                // The junction section, shared with the previous edge.
                let prev = rows.last().unwrap();
                let last = prev.last().unwrap();
                row.push(Section { p1: last.p1, p2: last.p2, curve: last.curve.clone() });
                continue;
            }
            let t = ta + (tb - ta) * s as f64 / m as f64;
            let plane = match &ends {
                Some((e0, _)) if i == 0 && s == 0 => cap_normal(e0.cap),
                Some((_, e1)) if i == k - 1 && s == m => cap_normal(e1.cap),
                _ => None,
            };
            row.push(section(i, t, plane, &mut hints)?);
        }
        rows.push(row);
    }
    // An open end whose cap is not square to the blend: the sections
    // there cross the cap, so the blend ends in the curve where its surface
    // meets the cap, led into by a few curves on that surface from the
    // last section clear of the cap.
    if let Some((e0, e1)) = &ends {
        const W: usize = 8;
        let tol = 1e-7 * size;
        // Per end: (row, at the row's start, deepest t kept, lead-in curves
        // from the section clear of the cap to the cap curve).
        let mut trims: Vec<(usize, bool, f64, Vec<Section>)> = Vec::new();
        for (end, first) in [(e0, true), (e1, false)] {
            let cap = &body.face(end.cap).surface;
            let i = if first { 0 } else { k - 1 };
            let sec = if first { &rows[i][0] } else { rows[i].last().unwrap() };
            if crate::ssi::signed_distance(cap, sec.p1).abs() < tol && crate::ssi::signed_distance(cap, sec.p2).abs() < tol {
                continue;
            }
            let l = chain[i];
            let ed = body.edge(l.edge);
            let (ta, tb) = if l.fwd { (ed.t0, ed.t1) } else { (ed.t1, ed.t0) };
            let m = rows[i].len() - 1;
            let step = (tb - ta) / m as f64;
            let (t_end, inward) = if first { (ta, step) } else { (tb, -step) };
            let mut at = |t: f64| -> Result<Section, FilletError> { section(i, t, None, &mut (None, None)) };
            let off = |sec: &Section, w: f64| crate::ssi::signed_distance(cap, sec.curve.point(w));
            // Where each section point (by its parameter w) meets the cap.
            let mut tc = Vec::with_capacity(W + 1);
            for n in 0..=W {
                let w = n as f64 / W as f64;
                let (mut t0, mut t1) = (t_end, t_end + 0.5 * inward);
                let (mut f0, mut f1) = (off(&at(t0)?, w), off(&at(t1)?, w));
                for _ in 0..40 {
                    if f1.abs() < 1e-10 * size || (f1 - f0).abs() < 1e-300 {
                        break;
                    }
                    let tn = t1 - f1 * (t1 - t0) / (f1 - f0);
                    (t0, f0) = (t1, f1);
                    t1 = tn;
                    if ((t1 - t_end) / inward).abs() > 0.5 * m as f64 {
                        if std::env::var_os("DUCTILE_DEBUG_FILLET").is_some() {
                            eprintln!("cap crossing at w {w:.3} not found: t {t1:.4} from end {t_end:.4}, step {inward:.4} of {m}, offsets {f0:.3e} {f1:.3e}; cap f{} {:?}, end section {:.4} {:.4}", end.cap.0, std::mem::discriminant(cap), at(t_end)?.p1, at(t_end)?.p2);
                        }
                        return Err(super::fillet::unsupported(line!(), file!()));
                    }
                    f1 = off(&at(t1)?, w);
                }
                if f1.abs() > 1e-8 * size {
                    return Err(super::fillet::unsupported(line!(), file!()));
                }
                tc.push(t1);
            }
            let deepest = tc.iter().map(|t| (t - t_end) / inward).fold(f64::MIN, f64::max);
            let t_lim = t_end + inward * (deepest + 0.3);
            let mut lead = vec![at(t_lim)?];
            for lambda in [1.0 / 3.0, 2.0 / 3.0, 1.0] {
                let pts: Vec<DVec3> = (0..=W).map(|n| {
                    let w = n as f64 / W as f64;
                    at(t_lim + lambda * (tc[n] - t_lim)).map(|s| s.curve.point(w))
                }).collect::<Result<_, _>>()?;
                let params: Vec<f64> = (0..=W).map(|n| n as f64 / W as f64).collect();
                let curve = NurbsCurve::interpolate_with_params(&pts, 3, &params);
                lead.push(Section { p1: pts[0], p2: pts[W], curve });
            }
            if first {
                lead.reverse();
            }
            trims.push((i, first, t_end + inward * (deepest + 0.8), lead));
        }
        for i in 0..k {
            let mine: Vec<&(usize, bool, f64, Vec<Section>)> = trims.iter().filter(|t| t.0 == i).collect();
            if mine.is_empty() {
                continue;
            }
            let l = chain[i];
            let ed = body.edge(l.edge);
            let (ta, tb) = if l.fwd { (ed.t0, ed.t1) } else { (ed.t1, ed.t0) };
            let m = rows[i].len() - 1;
            let row = std::mem::take(&mut rows[i]);
            // Stations are spaced evenly from ta to tb.
            let mut out: Vec<Section> = Vec::new();
            for t in mine.iter().filter(|t| t.1) {
                out.extend(t.3.iter().cloned());
            }
            for (s, sec) in row.into_iter().enumerate() {
                let t = ta + (tb - ta) * s as f64 / m as f64;
                let clear = mine.iter().all(|&&(_, first, keep, _)| if first { (t - keep) * (tb - ta) >= 0.0 } else { (keep - t) * (tb - ta) >= 0.0 });
                if clear {
                    out.push(sec);
                }
            }
            for t in mine.iter().filter(|t| !t.1) {
                out.extend(t.3.iter().cloned());
            }
            if out.len() < 2 {
                return Err(super::fillet::unsupported(line!(), file!()));
            }
            rows[i] = out;
        }
    }
    // Where another edge leaves a junction along one side's face (a
    // cylinder's seam, the edge between two faces of a split surface), the
    // junction section is the one whose contact point lies on that edge,
    // so the edge can be cut back to it.
    {
        let sgn = if convex.unwrap_or(true) { -1.0 } else { 1.0 };
        let junctions: Vec<usize> = if closed { (0..k).collect() } else { (1..k).collect() };
        for j in junctions {
            let v = verts[j];
            let (prev, next) = ((j + k - 1) % k, j);
            for x in others(v) {
                let faces: Vec<FaceId> = body.edge_faces(x).iter().map(|u| u.0).collect();
                let on_a = faces.iter().all(|f| *f == sides[prev].0 || *f == sides[next].0);
                let on_b = faces.iter().all(|f| *f == sides[prev].1 || *f == sides[next].1);
                if on_a == on_b {
                    continue;
                }
                // The face the edge runs on, and the other side's face.
                let (fa, fb) = if on_a { (sides[next].0, sides[next].1) } else { (sides[next].1, sides[next].0) };
                let ctrl;
                let (p1, p2);
                if let Blend::Chamfer(d) = blend {
                    // The chamfer's line starts `d` along the edge; the
                    // other side keeps its point.
                    let Some(pa) = point_along(body, x, v, d) else { continue };
                    let old = &rows[next][0];
                    (p1, p2) = if on_a { (pa, old.p2) } else { (old.p1, pa) };
                    ctrl = vec![p1.extend(1.0), ((p1 + p2) * 0.5).extend(1.0), p2.extend(1.0)];
                } else {
                    let Some((c, pa, pb)) = ball_on_edge(body, x, v, fa, fb, sgn * reach) else { continue };
                    (p1, p2) = if on_a { (pa, pb) } else { (pb, pa) };
                    ctrl = section_ctrl(blend, c, p1, p2);
                }
                let deg = ctrl.len() - 1;
                let mut knots = vec![0.0; deg + 1];
                knots.extend(std::iter::repeat(1.0).take(deg + 1));
                let sec = Section { p1, p2, curve: NurbsCurve::new(deg, knots, ctrl) };
                *rows[prev].last_mut().unwrap() = Section { p1, p2, curve: sec.curve.clone() };
                rows[next][0] = sec;
                break;
            }
        }
    }
    if closed {
        // The last section is the first one.
        let first = &rows[0][0];
        let (p1, p2, curve) = (first.p1, first.p2, first.curve.clone());
        let last = rows[k - 1].last_mut().unwrap();
        // Fitted intersection curves meet with slightly different
        // tangents, which tilts the section planes a little.
        let tol = (1e-6 * size).max(0.02 * reach);
        if last.p1.distance(p1) > tol || last.p2.distance(p2) > tol {
            if std::env::var_os("DUCTILE_DEBUG_FILLET").is_some() {
                eprintln!("closed chain sections differ: {:.3e} {:.3e} (p1 {p1:.4} vs {:.4})", last.p1.distance(p1), last.p2.distance(p2), last.p1);
            }
            return Err(super::fillet::unsupported(line!(), file!()));
        }
        *last = Section { p1, p2, curve };
    }
    // Lead-in sections can bunch up on one rail where the cap is met at a
    // shallow angle; lofting through near-repeated contact points puts a
    // hook in that rail. Drop interior sections that barely move a rail.
    for row in rows.iter_mut() {
        if row.len() < 5 {
            continue;
        }
        let mut steps: Vec<f64> = row.windows(2).map(|w| w[0].p1.distance(w[1].p1).max(w[0].p2.distance(w[1].p2))).collect();
        steps.sort_by(|a, b| a.total_cmp(b));
        let typical = steps[steps.len() / 2];
        let mut kept: Vec<Section> = vec![row[0].clone()];
        for (n, s) in row.iter().enumerate().skip(1) {
            let last = kept.last().unwrap();
            let moved = s.p1.distance(last.p1).min(s.p2.distance(last.p2));
            if n == row.len() - 1 || moved > 0.15 * typical {
                kept.push(s.clone());
            }
        }
        // The section before the last may now sit right against it.
        if kept.len() >= 4 {
            let n = kept.len();
            let moved = kept[n - 1].p1.distance(kept[n - 2].p1).min(kept[n - 1].p2.distance(kept[n - 2].p2));
            if moved < 0.15 * typical {
                kept.remove(n - 2);
            }
        }
        if kept.len() >= 2 {
            *row = kept;
        }
    }
    let convex = convex.unwrap_or(true);
    let surfaces: Vec<NurbsSurface> = rows
        .iter()
        .map(|row| {
            let curves: Vec<NurbsCurve> = row.iter().map(|s| s.curve.clone()).collect();
            // Sections by how far the contact points move between them, so
            // a closer last section (a lead-in to a cap) does not make the
            // surface overshoot and fold back at its end.
            let gaps: Vec<f64> = row.windows(2).map(|w| w[0].p1.distance(w[1].p1).max(w[0].p2.distance(w[1].p2)).max(1e-12)).collect();
            let total: f64 = gaps.iter().sum();
            let mut acc = 0.0;
            let mut params = vec![0.0];
            for g in &gaps {
                acc += g / total;
                params.push(acc);
            }
            *params.last_mut().unwrap() = 1.0;
            NurbsSurface::loft_with_params(&curves, 3, &params)
        })
        .collect();

    let mut out = body.clone();
    // Junction vertices and arcs.
    let nj = if closed { k } else { k + 1 };
    let pv: Vec<(VertexId, VertexId)> = (0..nj)
        .map(|j| {
            let s = if j < k { &rows[j][0] } else { rows[k - 1].last().unwrap() };
            (out.add_vertex(s.p1), out.add_vertex(s.p2))
        })
        .collect();
    let jv = |j: usize| pv[j % nj];
    let junctions: Vec<EdgeId> = (0..nj)
        .map(|j| {
            let s = if j < k { &rows[j][0] } else { rows[k - 1].last().unwrap() };
            let (a, b) = pv[j];
            out.add_edge(Curve3::Nurbs(s.curve.clone()), 0.0, 1.0, a, b)
        })
        .collect();
    let junction = |j: usize| junctions[j % nj];
    // Contact curves, along the chain.
    let mut contacts: Vec<(EdgeId, EdgeId)> = Vec::with_capacity(k);
    for (i, s) in surfaces.iter().enumerate() {
        let ((u0, u1), (v0, v1)) = s.domain();
        let c1 = out.add_edge(Curve3::Nurbs(s.iso_u(u0)), v0, v1, jv(i).0, jv(i + 1).0);
        let c2 = out.add_edge(Curve3::Nurbs(s.iso_u(u1)), v0, v1, jv(i).1, jv(i + 1).1);
        contacts.push((c1, c2));
    }
    // Close the caps: the end arc goes between the two cap edges at the
    // old end vertex.
    if let Some((e0, e1)) = &ends {
        for (end, j) in [(e0, 0), (e1, k)] {
            let v = verts[j];
            let lp = out.faces[end.cap.idx()]
                .loops
                .iter()
                .copied()
                .find(|l| out.loops[l.idx()].coedges.iter().any(|c| c.edge == end.ea))
                .ok_or_else(|| super::fillet::unsupported(line!(), file!()))?;
            let cs = out.loops[lp.idx()].coedges.clone();
            let pos = (0..cs.len()).find(|p| out.coedge_vertices(cs[*p]).1 == v).ok_or_else(|| super::fillet::unsupported(line!(), file!()))?;
            // Coming in along ea means the arc runs from A's side to B's.
            let rev = cs[pos].edge != end.ea;
            out.loops[lp.idx()].coedges.insert(pos + 1, Coedge::new(junction(j), rev));
        }
    }
    // Edges ending at the chain's vertices now end at the contact points.
    for j in 0..nj {
        let v = verts[j];
        let (mut side_a, mut side_b) = (vec![], vec![]);
        for i in [(j + k - 1) % k, j] {
            if (i == j && j < k) || (i != j && (j > 0 || closed)) {
                side_a.push(sides[i].0);
                side_b.push(sides[i].1);
            }
        }
        for x in others(v) {
            let faces: Vec<FaceId> = body.edge_faces(x).iter().map(|u| u.0).collect();
            let to_a = match &ends {
                Some((e0, _)) if j == 0 && (x == e0.ea || x == e0.eb) => x == e0.ea,
                Some((_, e1)) if j == k && (x == e1.ea || x == e1.eb) => x == e1.ea,
                _ => {
                    if faces.iter().all(|f| side_a.contains(f)) {
                        true
                    } else if faces.iter().all(|f| side_b.contains(f)) {
                        false
                    } else {
                        return Err(super::fillet::unsupported(line!(), file!()));
                    }
                }
            };
            let (pa, pb) = jv(j);
            let (tv, p) = if to_a { (pa, out.vertex(pa)) } else { (pb, out.vertex(pb)) };
            move_end(&mut out, x, v, p, tv, size)?;
        }
    }
    // Faces along the chain now run along the contact curves.
    for (i, l) in chain.iter().enumerate() {
        let (fa, fb) = sides[i];
        set_coedge(&mut out, fa, l.edge, Coedge::new(contacts[i].0, false));
        set_coedge(&mut out, fb, l.edge, Coedge::new(contacts[i].1, true));
    }
    // Blend faces.
    for (i, s) in surfaces.into_iter().enumerate() {
        let (c1, c2) = contacts[i];
        let lp = vec![Coedge::new(c1, true), Coedge::new(junction(i), false), Coedge::new(c2, false), Coedge::new(junction(i + 1), true)];
        let ed = body.edge(chain[i].edge);
        let q = ed.curve.point(0.5 * (ed.t0 + ed.t1));
        let reversed = blend_reversed(&s, blend, convex, q);
        out.add_face(Surface::Nurbs(s), reversed, vec![lp]);
    }
    Ok(out)
}

/// The point of edge `x` at distance `d` from its vertex `v`.
fn point_along(body: &Body, x: EdgeId, v: VertexId, d: f64) -> Option<DVec3> {
    let xe = body.edge(x);
    let (s0, dir) = if xe.v0 == v { (xe.t0, 1.0) } else { (xe.t1, -1.0) };
    let p0 = xe.curve.point(s0);
    let mut s = s0 + dir * d / xe.curve.tangent(s0).length().max(1e-12);
    for _ in 0..20 {
        let p = xe.curve.point(s);
        let err = p.distance(p0) - d;
        if err.abs() < 1e-10 * (1.0 + d) {
            break;
        }
        s -= dir * err / xe.curve.tangent(s).length().max(1e-12);
    }
    let in_range = s >= xe.t0.min(xe.t1) - 1e-9 && s <= xe.t0.max(xe.t1) + 1e-9;
    in_range.then(|| xe.curve.point(s))
}

/// The ball of signed radius `sd` (negative inside the material) touching
/// face `fa` on edge `x` near its vertex `v`, and touching face `fb`:
/// centre and the two contact points.
fn ball_on_edge(body: &Body, x: EdgeId, v: VertexId, fa: FaceId, fb: FaceId, sd: f64) -> Option<(DVec3, DVec3, DVec3)> {
    let xe = body.edge(x);
    let (sa, sb) = (&body.face(fa).surface, &body.face(fb).surface);
    let (s0, dir) = if xe.v0 == v { (xe.t0, 1.0) } else { (xe.t1, -1.0) };
    let speed = xe.curve.tangent(s0).length().max(1e-12);
    let hint = std::cell::Cell::new((None, None));
    // Signed distance of the ball centre over `fb`, less the radius.
    let g = |s: f64| -> Option<(f64, DVec3, DVec3, DVec3)> {
        let pa = xe.curve.point(s);
        let (ha, hb) = hint.get();
        let uva = sa.project(pa, ha);
        let c = pa + body.face_normal(fa, uva.x, uva.y) * sd;
        let uvb = sb.project(c, hb);
        hint.set((Some(uva), Some(uvb)));
        let pb = sb.point(uvb.x, uvb.y);
        Some(((c - pb).dot(body.face_normal(fb, uvb.x, uvb.y)) - sd, c, pa, pb))
    };
    // Secant from the vertex into the edge.
    let (mut a, mut b) = (s0, s0 + dir * sd.abs() / speed);
    let (mut fa_, mut fb_) = (g(a)?.0, g(b)?.0);
    for _ in 0..40 {
        if (fb_ - fa_).abs() < 1e-300 {
            break;
        }
        let c = b - fb_ * (b - a) / (fb_ - fa_);
        (a, fa_) = (b, fb_);
        b = c;
        fb_ = g(b)?.0;
        if fb_.abs() < 1e-10 * sd.abs().max(1e-9) {
            break;
        }
    }
    let (res, c, pa, pb) = g(b)?;
    // Only a nearby solution, on the edge's own run.
    let reach = 4.0 * sd.abs() / speed;
    (res.abs() < 1e-7 * (1.0 + sd.abs()) && (b - s0) * dir > -1e-9 && (b - s0).abs() < reach).then_some((c, pa, pb))
}

/// Whether a blend face's surface normal points into the material: the
/// outward side is away from the ball centre on convex edges and towards it
/// on concave ones; for chamfers, away from the edge `q` (or towards it).
pub(crate) fn blend_reversed(surf: &NurbsSurface, blend: Blend, convex: bool, q: DVec3) -> bool {
    let ((u0, u1), (v0, v1)) = surf.domain();
    let (um, vm) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
    let pm = surf.point(um, vm);
    let chord_mid = (surf.point(u0, vm) + surf.point(u1, vm)) * 0.5;
    let outward = match blend {
        Blend::Fillet(_) | Blend::Smooth(_) => {
            let away = (pm - chord_mid).normalize_or_zero();
            if convex {
                away
            } else {
                -away
            }
        }
        Blend::Chamfer(_) => {
            let d = (pm - q).normalize_or_zero();
            if convex {
                -d
            } else {
                d
            }
        }
    };
    Surface::Nurbs(surf.clone()).normal(um, vm).dot(outward) < 0.0
}

fn set_coedge(b: &mut Body, f: FaceId, old: EdgeId, new: Coedge) {
    for l in b.faces[f.idx()].loops.clone() {
        for c in b.loops[l.idx()].coedges.iter_mut() {
            if c.edge == old {
                *c = new;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ops::boolean::open_edges;
    use crate::ops::extrude::extrude_region;
    use crate::ops::heal::compact;
    use crate::profile::{Loop3, Region, Segment};
    use crate::tess::{tessellate, TessOptions};
    use ductile_geom::Frame;
    use std::f64::consts::PI;

    fn vol(b: &Body) -> f64 {
        tessellate(b, &TessOptions { chord_tol: 1e-4, angle_tol: 0.05 }).volume()
    }

    /// Rounded rectangle 2a x 2b with corner radius c, extruded by h.
    fn rounded_slab(a: f64, b: f64, c: f64, h: f64) -> Body {
        let mut segs = Vec::new();
        let corners = [(a - c, b - c, 0.0), (-(a - c), b - c, 0.5 * PI), (-(a - c), -(b - c), PI), (a - c, -(b - c), 1.5 * PI)];
        for (i, (x, y, a0)) in corners.iter().enumerate() {
            let frame = Frame { origin: DVec3::new(*x, *y, 0.0), ..Frame::WORLD };
            segs.push(Segment::new(Curve3::Circle { frame, radius: c }, *a0, a0 + 0.5 * PI));
            let (nx, ny, na) = corners[(i + 1) % 4];
            let p = DVec3::new(*x, *y, 0.0) + c * DVec3::new((a0 + 0.5 * PI).cos(), (a0 + 0.5 * PI).sin(), 0.0);
            let q = DVec3::new(nx, ny, 0.0) + c * DVec3::new(na.cos(), na.sin(), 0.0);
            segs.push(Segment::line(p, q));
        }
        let region = Region { plane: Frame::WORLD, outer: Loop3 { segments: segs }, holes: vec![] }.normalized();
        extrude_region(&region, DVec3::Z * h).unwrap()
    }

    #[test]
    fn chains_the_top_edges_of_a_rounded_slab() {
        let b = rounded_slab(3.0, 2.0, 0.8, 1.5);
        let top: Vec<EdgeId> = b.edge_ids().filter(|e| {
            let ed = b.edge(*e);
            ed.start().z > 1.4 && ed.end().z > 1.4
        }).collect();
        assert_eq!(top.len(), 8);
        let chains = tangent_chains(&b, &top);
        assert_eq!(chains.len(), 1);
        assert!(chains[0].1);
        assert_eq!(chains[0].0.len(), 8);
    }

    #[test]
    fn blends_where_two_drilled_holes_cross() {
        // The crossing curves run over both holes' seams and split faces,
        // so other edges leave the chain's junctions at an angle.
        use crate::ops::boolean::{boolean, BoolOp};
        use crate::ops::fillet::blend_edges;
        use crate::primitives::{make_box, make_cylinder};
        let bx = make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0));
        let v = make_cylinder(&Frame { origin: DVec3::new(2.0, 1.5, -1.0), ..Frame::WORLD }, 0.5, 4.0);
        let h = make_cylinder(&Frame::new(DVec3::new(-1.0, 1.7, 1.0), DVec3::X, DVec3::Y), 0.35, 6.0);
        let a = boolean(&boolean(&bx, &v, BoolOp::Difference).unwrap(), &h, BoolOp::Difference).unwrap();
        let cross: Vec<EdgeId> = a
            .edge_ids()
            .filter(|e| {
                let cyl = a.edge_faces(*e).iter().filter(|(f, _)| matches!(a.face(*f).surface, Surface::Cylinder { .. })).count();
                cyl == 2 && (a.edge(*e).mid().x - 2.0).abs() < 0.6
            })
            .collect();
        for blend in [Blend::Fillet(0.08), Blend::Chamfer(0.08)] {
            let r = blend_edges(&a, &cross, blend).unwrap();
            assert_eq!(open_edges(&r), 0, "{blend:?}");
            assert!(vol(&r) < vol(&a), "{blend:?}");
        }
    }

    #[test]
    fn ends_on_a_cap_met_at_an_angle() {
        // A boss standing in a side hole: the curve where the boss meets
        // the hole ends on the boss's bottom, which the hole's wall meets
        // slantwise, so the blend ends in the curve where it crosses the
        // bottom rather than in a section.
        use crate::ops::boolean::{boolean, BoolOp};
        use crate::ops::fillet::blend_edges;
        use crate::primitives::{make_box, make_cylinder};
        let bx = make_box(&Frame::WORLD, DVec3::new(4.0, 4.0, 2.0));
        let h = make_cylinder(&Frame::new(DVec3::new(-1.0, 2.0, 1.0), DVec3::X, DVec3::Y), 0.6, 6.0);
        let boss = make_cylinder(&Frame { origin: DVec3::new(2.0, 1.7, 1.2), ..Frame::WORLD }, 0.5, 1.5);
        let a = boolean(&boolean(&bx, &h, BoolOp::Difference).unwrap(), &boss, BoolOp::Union).unwrap();
        let e = a
            .edge_ids()
            .find(|e| {
                let fs = a.edge_faces(*e);
                fs.len() == 2 && fs[0].0 != fs[1].0 && fs.iter().all(|(f, _)| matches!(a.face(*f).surface, Surface::Cylinder { .. }))
            })
            .unwrap();
        for blend in [Blend::Fillet(0.06), Blend::Chamfer(0.06)] {
            let r = blend_edges(&a, &[e], blend).unwrap();
            assert_eq!(open_edges(&r), 0, "{blend:?}");
            assert!((vol(&r) - vol(&a)).abs() < 0.01 * vol(&a), "{blend:?}");
        }
    }

    #[test]
    fn grows_one_top_edge_into_the_whole_outline() {
        let b = rounded_slab(3.0, 2.0, 0.8, 1.5);
        let e = b.edge_ids().find(|e| {
            let ed = b.edge(*e);
            ed.start().z > 1.4 && ed.end().z > 1.4
        }).unwrap();
        let mut chain = grow_tangent_chain(&b, e, &|_| true);
        assert_eq!(chain.len(), 8);
        chain.sort();
        chain.dedup();
        assert_eq!(chain.len(), 8);
        // A vertical edge meets the outlines at right angles: alone.
        let v = b.edge_ids().find(|e| {
            let ed = b.edge(*e);
            (ed.start().z - ed.end().z).abs() > 1.0
        });
        if let Some(v) = v {
            assert_eq!(grow_tangent_chain(&b, v, &|_| true), vec![v]);
        }
    }

    #[test]
    fn fillets_the_top_of_a_rounded_slab() {
        let (a, bb, c, h, r) = (3.0, 2.0, 0.8, 1.5, 0.3);
        let b = rounded_slab(a, bb, c, h);
        let top: Vec<EdgeId> = b.edge_ids().filter(|e| {
            let ed = b.edge(*e);
            ed.start().z > 1.4 && ed.end().z > 1.4
        }).collect();
        let chains = tangent_chains(&b, &top);
        let out = compact(&roll_chain(&b, &chains[0].0, true, Blend::Fillet(r)).unwrap());
        assert_eq!(open_edges(&out), 0);
        // Removed: the spandrel (r^2 (1 - pi/4)) swept along the contact
        // path, by Pappus around the corners: straight runs plus corner arcs
        // at radius c - r + centroid offset.
        let spandrel = r * r * (1.0 - PI / 4.0);
        let straight = 2.0 * (2.0 * (a - c)) + 2.0 * (2.0 * (bb - c));
        // Centroid of the spandrel measured from the side face: r(10 - 3pi)/(12 - 3pi).
        let d = r * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI);
        let arcs = 2.0 * PI * (c - d);
        let expect = spandrel * (straight + arcs);
        let removed = vol(&b) - vol(&out);
        assert!((removed - expect).abs() < 2e-3 * expect, "{removed} vs {expect}");
    }

    #[test]
    fn fillets_a_side_edge_of_a_spline_prism() {
        // A D-shaped profile: a spline arch closed by a straight base, so
        // the two side edges run between a plane and a spline face and end
        // on the caps.
        let pts = [DVec3::new(-2.0, 0.0, 0.0), DVec3::new(-1.5, 1.5, 0.0), DVec3::new(0.0, 2.2, 0.0), DVec3::new(1.5, 1.5, 0.0), DVec3::new(2.0, 0.0, 0.0)];
        let arch = NurbsCurve::interpolate(&pts, 3);
        let (t0, t1) = arch.domain();
        let segs = vec![Segment::new(Curve3::Nurbs(arch), t0, t1), Segment::line(pts[4], pts[0])];
        let region = Region { plane: Frame::WORLD, outer: Loop3 { segments: segs }, holes: vec![] }.normalized();
        let b = extrude_region(&region, DVec3::Z * 2.0).unwrap();
        let side = b.edge_ids().find(|e| {
            let ed = b.edge(*e);
            ed.start().distance(DVec3::new(2.0, 0.0, ed.start().z)) < 1e-9 && ed.end().distance(DVec3::new(2.0, 0.0, ed.end().z)) < 1e-9 && (ed.start().z - ed.end().z).abs() > 1.0
        }).unwrap();
        let chains = tangent_chains(&b, &[side]);
        let r = 0.25;
        let out = compact(&roll_chain(&b, &chains[0].0, chains[0].1, Blend::Fillet(r)).unwrap());
        assert_eq!(open_edges(&out), 0);
        let removed = vol(&b) - vol(&out);
        // A corner spandrel for the corner's angle, along the height.
        assert!(removed > 0.0 && removed < 2.0 * r * r, "{removed}");
    }
}
