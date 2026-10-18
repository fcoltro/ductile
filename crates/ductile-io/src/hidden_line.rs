// SPDX-License-Identifier: GPL-3.0-only
//! Hidden-line drawings: the edges and silhouettes of tessellated bodies as
//! seen from a camera, split into visible and hidden runs, written as SVG.

use ductile_brep::BodyMesh;
use glam::{DMat4, DVec2, DVec3};

/// Where the drawing is seen from.
#[derive(Clone, Copy, Debug)]
pub struct View {
    pub view_proj: DMat4,
    /// The eye for a perspective view; None for an orthographic one.
    pub eye: Option<DVec3>,
    /// The viewing direction (into the screen).
    pub forward: DVec3,
    /// Drawing size in pixels.
    pub size: DVec2,
}

impl View {
    fn screen(&self, p: DVec3) -> Option<DVec2> {
        let c = self.view_proj * p.extend(1.0);
        if c.w <= 1e-12 {
            return None;
        }
        let n = c.truncate() / c.w;
        Some(DVec2::new((n.x + 1.0) * 0.5 * self.size.x, (1.0 - n.y) * 0.5 * self.size.y))
    }

    /// Ray origin and direction toward `p`, and the distance to it.
    fn ray_to(&self, p: DVec3, far: f64) -> (DVec3, DVec3, f64) {
        match self.eye {
            Some(e) => ((e), (p - e).normalize_or_zero(), p.distance(e)),
            None => (p - self.forward * far, self.forward, far),
        }
    }

    fn facing(&self, n: DVec3, at: DVec3) -> bool {
        let d = match self.eye {
            Some(e) => at - e,
            None => self.forward,
        };
        n.dot(d) < 0.0
    }
}

/// Polylines of a drawing, in pixels with y down.
#[derive(Clone, Debug, Default)]
pub struct Drawing {
    pub visible: Vec<Vec<DVec2>>,
    pub hidden: Vec<Vec<DVec2>>,
}

struct Grid {
    min: DVec2,
    cell: f64,
    n: usize,
    cells: Vec<Vec<u32>>,
}

impl Grid {
    fn idx(&self, p: DVec2) -> (usize, usize) {
        let q = ((p - self.min) / self.cell).floor();
        (q.x.clamp(0.0, (self.n - 1) as f64) as usize, q.y.clamp(0.0, (self.n - 1) as f64) as usize)
    }
}

fn ray_triangle(o: DVec3, d: DVec3, t: &[DVec3; 3]) -> Option<f64> {
    let (e1, e2) = (t[1] - t[0], t[2] - t[0]);
    let p = d.cross(e2);
    let det = e1.dot(p);
    if det.abs() < 1e-18 {
        return None;
    }
    let s = o - t[0];
    let u = s.dot(p) / det;
    if !(-1e-9..=1.0 + 1e-9).contains(&u) {
        return None;
    }
    let q = s.cross(e1);
    let v = d.dot(q) / det;
    if v < -1e-9 || u + v > 1.0 + 1e-9 {
        return None;
    }
    Some(e2.dot(q) / det)
}

/// The edges and silhouettes of `meshes` seen from `view`, split where they
/// pass behind a surface. `tol` is how close to a surface a line may run
/// and still count as on it (about the tessellation's chord tolerance).
pub fn hidden_lines(meshes: &[&BodyMesh], view: &View, tol: f64) -> Drawing {
    let tris: Vec<[DVec3; 3]> = meshes.iter().flat_map(|m| m.triangles().map(|(t, _)| t)).collect();
    let (mut lo, mut hi) = (DVec3::splat(f64::MAX), DVec3::splat(f64::MIN));
    for t in &tris {
        for p in t {
            lo = lo.min(*p);
            hi = hi.max(*p);
        }
    }
    let far = if tris.is_empty() { 1.0 } else { 4.0 * (hi - lo).length() + view.eye.map_or(0.0, |e| e.distance(lo)) + 1.0 };
    // Bin the triangles' screen boxes so each sample tests only its own.
    let n = 96;
    let cell = view.size.max_element() / n as f64;
    let mut grid = Grid { min: DVec2::ZERO, cell, n, cells: vec![vec![]; n * n] };
    for (i, t) in tris.iter().enumerate() {
        let s: Option<Vec<DVec2>> = t.iter().map(|p| view.screen(*p)).collect();
        let (a, b) = match s {
            Some(s) => (s.iter().fold(DVec2::MAX, |m, p| m.min(*p)), s.iter().fold(DVec2::MIN, |m, p| m.max(*p))),
            None => (DVec2::ZERO, view.size),
        };
        if b.x < 0.0 || b.y < 0.0 || a.x > view.size.x || a.y > view.size.y {
            continue;
        }
        let ((x0, y0), (x1, y1)) = (grid.idx(a), grid.idx(b));
        for y in y0..=y1 {
            for x in x0..=x1 {
                grid.cells[y * n + x].push(i as u32);
            }
        }
    }
    let visible = |p: DVec3, own: [u32; 2]| -> bool {
        let Some(s) = view.screen(p) else { return false };
        if s.x < 0.0 || s.y < 0.0 || s.x > view.size.x || s.y > view.size.y {
            return true;
        }
        let (x, y) = grid.idx(s);
        let (o, d, dist) = view.ray_to(p, far);
        !grid.cells[y * n + x].iter().filter(|i| !own.contains(i)).any(|i| {
            let t = &tris[*i as usize];
            ray_triangle(o, d, t).is_some_and(|h| h > 0.0 && h < dist - tol && (o + d * h).distance(p) > 2.0 * tol)
        })
    };
    // The lines: every edge, and the mesh edges where a surface turns away.
    // Silhouettes skip their own two triangles, which a ray grazes.
    let mut lines: Vec<(Vec<DVec3>, [u32; 2])> = meshes.iter().flat_map(|m| m.edges.iter().filter(|e| e.len() > 1).map(|e| (e.clone(), [u32::MAX; 2]))).collect();
    let mut base = 0u32;
    for m in meshes {
        for f in &m.faces {
            let mut by_edge: std::collections::HashMap<(u32, u32), (usize, u32, u32)> = Default::default();
            let tri_n: Vec<(DVec3, DVec3)> = f
                .triangles
                .iter()
                .map(|t| {
                    let p = [f.positions[t[0] as usize], f.positions[t[1] as usize], f.positions[t[2] as usize]];
                    ((p[1] - p[0]).cross(p[2] - p[0]), (p[0] + p[1] + p[2]) / 3.0)
                })
                .collect();
            for (ti, t) in f.triangles.iter().enumerate() {
                for k in 0..3 {
                    let (a, b) = (t[k], t[(k + 1) % 3]);
                    let key = (a.min(b), a.max(b));
                    if let Some((other, ..)) = by_edge.remove(&key) {
                        let (na, ca) = tri_n[ti];
                        let (nb, cb) = tri_n[other];
                        if na.length_squared() > 0.0 && nb.length_squared() > 0.0 && view.facing(na, ca) != view.facing(nb, cb) {
                            lines.push((vec![f.positions[a as usize], f.positions[b as usize]], [base + ti as u32, base + other as u32]));
                        }
                    } else {
                        by_edge.insert(key, (ti, a, b));
                    }
                }
            }
            base += f.triangles.len() as u32;
        }
    }
    let mut out = Drawing::default();
    for (l, own) in lines {
        // Samples about two pixels apart, each visible or not.
        let mut pts: Vec<(DVec3, bool)> = vec![];
        for w in l.windows(2) {
            let len = match (view.screen(w[0]), view.screen(w[1])) {
                (Some(a), Some(b)) => a.distance(b),
                _ => 0.0,
            };
            let k = ((len / 2.0).ceil() as usize).clamp(1, 2000);
            for i in 0..k {
                let p = w[0].lerp(w[1], i as f64 / k as f64);
                pts.push((p, visible(p, own)));
            }
        }
        let last = *l.last().unwrap();
        pts.push((last, visible(last, own)));
        let mut run: Vec<DVec2> = vec![];
        let mut state = pts[0].1;
        for (i, (p, v)) in pts.iter().enumerate() {
            let Some(s) = view.screen(*p) else { continue };
            if *v != state {
                // Split halfway between the samples.
                let prev = pts[i - 1].0;
                if let Some(m) = view.screen(prev.lerp(*p, 0.5)) {
                    run.push(m);
                    let done = std::mem::replace(&mut run, vec![m]);
                    if done.len() > 1 {
                        if state { out.visible.push(done) } else { out.hidden.push(done) }
                    }
                }
                state = *v;
            }
            run.push(s);
        }
        if run.len() > 1 {
            if state { out.visible.push(run) } else { out.hidden.push(run) }
        }
    }
    out
}

/// The drawing as an SVG document: visible lines solid, hidden lines (when
/// `hidden`) dashed and grey, in their own groups.
pub fn to_svg(d: &Drawing, size: DVec2, hidden: bool) -> String {
    let path = |l: &Vec<DVec2>| {
        let mut s = String::from("<path d=\"");
        for (i, p) in l.iter().enumerate() {
            s += &format!("{}{:.2} {:.2}", if i == 0 { "M" } else { " L" }, p.x, p.y);
        }
        s + "\"/>\n"
    };
    let mut s = format!(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{w:.0}\" height=\"{h:.0}\" viewBox=\"0 0 {w:.2} {h:.2}\">\n",
        w = size.x,
        h = size.y
    );
    if hidden && !d.hidden.is_empty() {
        s += "<g id=\"hidden\" fill=\"none\" stroke=\"#9a9a9a\" stroke-width=\"0.75\" stroke-dasharray=\"4 3\" stroke-linecap=\"round\">\n";
        d.hidden.iter().for_each(|l| s += &path(l));
        s += "</g>\n";
    }
    s += "<g id=\"visible\" fill=\"none\" stroke=\"#000000\" stroke-width=\"1.25\" stroke-linecap=\"round\" stroke-linejoin=\"round\">\n";
    d.visible.iter().for_each(|l| s += &path(l));
    s + "</g>\n</svg>\n"
}

#[cfg(test)]
mod tests {
    use super::*;
    use ductile_brep::{primitives::make_box, tessellate, TessOptions};
    use ductile_geom::Frame;

    fn ortho(forward: DVec3, up: DVec3) -> View {
        // Six units across the 600 pixels.
        let right = forward.cross(up).normalize();
        let up = right.cross(forward);
        let rows = [(right / 3.0).extend(0.0), (up / 3.0).extend(0.0), (forward / 100.0).extend(0.5), glam::DVec4::W];
        View { view_proj: DMat4::from_cols(rows[0], rows[1], rows[2], rows[3]).transpose(), eye: None, forward, size: DVec2::new(600.0, 600.0) }
    }

    fn length(ls: &[Vec<DVec2>]) -> f64 {
        ls.iter().flat_map(|l| l.windows(2).map(|w| w[0].distance(w[1]))).sum()
    }

    #[test]
    fn a_box_from_a_corner_hides_three_edges() {
        let b = make_box(&Frame { origin: DVec3::splat(-1.0), ..Frame::WORLD }, DVec3::splat(2.0));
        let m = tessellate(&b, &TessOptions::default());
        let f = DVec3::new(-1.0, -1.3, -0.8).normalize();
        let d = hidden_lines(&[&m], &ortho(f, DVec3::Z), 1e-3);
        // Nine of the twelve edges show; the three meeting at the far corner hide.
        let (v, h) = (length(&d.visible), length(&d.hidden));
        assert!((v / h - 3.0).abs() < 0.1, "{v} {h}");
        let svg = to_svg(&d, DVec2::new(600.0, 600.0), true);
        assert!(svg.contains("id=\"hidden\"") && svg.contains("id=\"visible\""));
    }

    #[test]
    fn a_cylinder_seen_from_the_side_has_silhouettes() {
        let b = ductile_brep::primitives::make_cylinder(&Frame::WORLD, 1.0, 2.0);
        let m = tessellate(&b, &TessOptions::default());
        let d = hidden_lines(&[&m], &ortho(DVec3::Y, DVec3::Z), 1e-3);
        // Side on, the outline is a rectangle 2 wide and 2 tall: two silhouette
        // lines and the two rims seen edge on.
        let xs: Vec<f64> = d.visible.iter().flatten().map(|p| p.x).collect();
        let (lo, hi) = (xs.iter().cloned().fold(f64::MAX, f64::min), xs.iter().cloned().fold(f64::MIN, f64::max));
        assert!(((hi - lo) - 200.0).abs() < 2.0, "{lo} {hi}");
        let vertical: f64 = d.visible.iter().flat_map(|l| l.windows(2)).filter(|w| (w[0].x - w[1].x).abs() < 1e-3).map(|w| (w[0].y - w[1].y).abs()).sum();
        assert!(vertical > 390.0, "{vertical}");
    }
}
