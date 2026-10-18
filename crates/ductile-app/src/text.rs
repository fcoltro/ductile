// SPDX-License-Identifier: GPL-3.0-only
//! Text: letters as curves, from a TrueType or OpenType font's outlines,
//! laid out on a plane so they can be extruded like any closed sketch.

use ductile_brep::profile::Segment;
use ductile_geom::{Curve3, DVec3, DVec4, Frame, NurbsCurve};
use ttf_parser::OutlineBuilder;

/// The font that ships with Ductile (Ubuntu Light, Ubuntu Font Licence).
pub fn default_font() -> &'static [u8] {
    epaint_default_fonts::UBUNTU_LIGHT
}

/// Font files found in the usual Linux font folders, sorted by name.
pub fn system_fonts() -> Vec<std::path::PathBuf> {
    let mut dirs = vec![std::path::PathBuf::from("/usr/share/fonts"), std::path::PathBuf::from("/usr/local/share/fonts")];
    if let Some(home) = std::env::var_os("HOME") {
        dirs.push(std::path::Path::new(&home).join(".local/share/fonts"));
        dirs.push(std::path::Path::new(&home).join(".fonts"));
    }
    let mut out = vec![];
    let mut todo = dirs;
    while let Some(d) = todo.pop() {
        let Ok(rd) = std::fs::read_dir(&d) else { continue };
        for e in rd.flatten() {
            let p = e.path();
            if p.is_dir() {
                todo.push(p);
            } else if p.extension().and_then(|x| x.to_str()).is_some_and(|x| x.eq_ignore_ascii_case("ttf") || x.eq_ignore_ascii_case("otf")) {
                out.push(p);
            }
        }
    }
    out.sort_by_key(|p| p.file_name().map(|n| n.to_ascii_lowercase()));
    out
}

/// Collects a glyph's contours as segments in font units, offset by `x`.
struct Outline {
    x: f64,
    start: DVec3,
    at: DVec3,
    contours: Vec<Vec<Segment>>,
    current: Vec<Segment>,
}

impl Outline {
    fn p(&self, x: f32, y: f32) -> DVec3 {
        DVec3::new(self.x + x as f64, y as f64, 0.0)
    }

    fn bezier(&mut self, pts: Vec<DVec3>) {
        let d = pts.len() - 1;
        let knots: Vec<f64> = std::iter::repeat_n(0.0, d + 1).chain(std::iter::repeat_n(1.0, d + 1)).collect();
        let ctrl: Vec<DVec4> = pts.iter().map(|p| p.extend(1.0)).collect();
        self.current.push(Segment::new(Curve3::Nurbs(NurbsCurve::new(d, knots, ctrl)), 0.0, 1.0));
    }
}

impl ttf_parser::OutlineBuilder for Outline {
    fn move_to(&mut self, x: f32, y: f32) {
        self.close();
        self.start = self.p(x, y);
        self.at = self.start;
    }
    fn line_to(&mut self, x: f32, y: f32) {
        let q = self.p(x, y);
        if q.distance(self.at) > 1e-9 {
            self.current.push(Segment::line(self.at, q));
        }
        self.at = q;
    }
    fn quad_to(&mut self, x1: f32, y1: f32, x: f32, y: f32) {
        let (c, q) = (self.p(x1, y1), self.p(x, y));
        self.bezier(vec![self.at, c, q]);
        self.at = q;
    }
    fn curve_to(&mut self, x1: f32, y1: f32, x2: f32, y2: f32, x: f32, y: f32) {
        let (c1, c2, q) = (self.p(x1, y1), self.p(x2, y2), self.p(x, y));
        self.bezier(vec![self.at, c1, c2, q]);
        self.at = q;
    }
    fn close(&mut self) {
        if self.at.distance(self.start) > 1e-9 {
            self.current.push(Segment::line(self.at, self.start));
        }
        self.at = self.start;
        if !self.current.is_empty() {
            self.contours.push(std::mem::take(&mut self.current));
        }
    }
}

/// The outlines of `text` in `font`, on `frame`'s XY plane from its origin
/// (the baseline's start), scaled so capital letters stand `height` tall.
/// Lines break at newlines. Each closed contour is one loop.
pub fn text_loops(font: &[u8], text: &str, frame: &Frame, height: f64) -> Result<Vec<Vec<Segment>>, String> {
    let face = ttf_parser::Face::parse(font, 0).map_err(|e| format!("can't read the font: {e}"))?;
    let upem = face.units_per_em() as f64;
    let cap = face.capital_height().map(|h| h as f64).filter(|h| *h > 0.0).unwrap_or(0.7 * upem);
    let scale = height / cap;
    let line = (face.ascender() as f64 - face.descender() as f64 + face.line_gap() as f64).max(upem);
    let mut ob = Outline { x: 0.0, start: DVec3::ZERO, at: DVec3::ZERO, contours: vec![], current: vec![] };
    let mut placed: Vec<(f64, Vec<Vec<Segment>>)> = vec![];
    for (row, l) in text.lines().enumerate() {
        ob.x = 0.0;
        let mut prev: Option<ttf_parser::GlyphId> = None;
        for ch in l.chars() {
            let Some(g) = face.glyph_index(ch) else { continue };
            if let (Some(p), Some(kern)) = (prev, face.tables().kern) {
                if let Some(k) = kern.subtables.into_iter().filter(|s| s.horizontal && !s.variable).find_map(|s| s.glyphs_kerning(p, g)) {
                    ob.x += k as f64;
                }
            }
            face.outline_glyph(g, &mut ob);
            ob.close();
            ob.x += face.glyph_hor_advance(g).unwrap_or(0) as f64;
            prev = Some(g);
        }
        placed.push((row as f64 * line, std::mem::take(&mut ob.contours)));
    }
    let to_world = |p: DVec3, dy: f64| frame.origin + frame.x * (p.x * scale) + frame.y * ((p.y - dy) * scale);
    let m = |s: &Segment, dy: f64| -> Segment {
        match &s.curve {
            Curve3::Nurbs(n) => {
                let ctrl = n.ctrl.iter().map(|h| to_world(h.truncate() / h.w, dy).extend(1.0)).collect();
                Segment::new(Curve3::Nurbs(NurbsCurve::new(n.degree, n.knots.clone(), ctrl)), s.t0, s.t1)
            }
            _ => Segment::line(to_world(s.start(), dy), to_world(s.end(), dy)),
        }
    };
    Ok(placed.into_iter().flat_map(|(dy, cs)| cs.into_iter().map(move |c| c.iter().map(|s| m(s, dy)).collect::<Vec<_>>())).collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn letters_become_closed_loops_of_the_right_height() {
        let loops = text_loops(default_font(), "Ob", &Frame::WORLD, 10.0).unwrap();
        // O has two contours, b has two.
        assert_eq!(loops.len(), 4);
        for l in &loops {
            assert!(l.first().unwrap().start().distance(l.last().unwrap().end()) < 1e-9);
            for w in l.windows(2) {
                assert!(w[0].end().distance(w[1].start()) < 1e-9);
            }
        }
        let top = loops.iter().flatten().flat_map(|s| s.sample(1e-3)).map(|p| p.y).fold(f64::MIN, f64::max);
        // The O overshoots its cap height a little; the b's ascender rises above it.
        assert!(top > 10.0 && top < 12.5, "{top}");
        // A wire body of the loops makes extrudable regions.
        let segs: Vec<Segment> = loops.into_iter().flatten().collect();
        let wire = ductile_brep::profile::wire_from_segments(&segs);
        assert_eq!(ductile_brep::profile::wire_chains(&wire).len(), 4);
    }
}
