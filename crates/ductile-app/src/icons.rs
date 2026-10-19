// SPDX-License-Identifier: GPL-3.0-only
//! Toolbar icons, drawn as line art so they stay crisp at any scale and
//! follow the theme colours. Each icon is laid out on a unit square.

use crate::commands::Command;
use egui::{pos2, Color32, Painter, Pos2, Rect, Shape, Stroke};
use std::f32::consts::{PI, TAU};

struct Pen<'a> {
    painter: &'a Painter,
    rect: Rect,
    stroke: Stroke,
}

impl Pen<'_> {
    fn at(&self, x: f32, y: f32) -> Pos2 {
        pos2(self.rect.min.x + x * self.rect.width(), self.rect.min.y + y * self.rect.height())
    }
    fn line(&self, pts: &[(f32, f32)]) {
        self.painter.add(Shape::line(pts.iter().map(|p| self.at(p.0, p.1)).collect(), self.stroke));
    }
    fn closed(&self, pts: &[(f32, f32)]) {
        self.painter.add(Shape::closed_line(pts.iter().map(|p| self.at(p.0, p.1)).collect(), self.stroke));
    }
    fn fill(&self, pts: &[(f32, f32)], c: Color32) {
        self.painter.add(Shape::convex_polygon(pts.iter().map(|p| self.at(p.0, p.1)).collect(), c, Stroke::NONE));
    }
    /// An elliptical arc from angle `a0` to `a1` (radians, y down).
    fn arc(&self, c: (f32, f32), rx: f32, ry: f32, a0: f32, a1: f32) {
        let n = 24;
        let pts: Vec<(f32, f32)> = (0..=n)
            .map(|i| {
                let a = a0 + (a1 - a0) * i as f32 / n as f32;
                (c.0 + rx * a.cos(), c.1 + ry * a.sin())
            })
            .collect();
        self.line(&pts);
    }
    fn ellipse(&self, c: (f32, f32), rx: f32, ry: f32) {
        self.arc(c, rx, ry, 0.0, TAU);
    }
    fn arrow(&self, from: (f32, f32), to: (f32, f32)) {
        self.line(&[from, to]);
        let (dx, dy) = (to.0 - from.0, to.1 - from.1);
        let l = (dx * dx + dy * dy).sqrt().max(1e-6);
        let (ux, uy) = (dx / l * 0.14, dy / l * 0.14);
        self.line(&[(to.0 - ux - uy * 0.6, to.1 - uy + ux * 0.6), to, (to.0 - ux + uy * 0.6, to.1 - uy - ux * 0.6)]);
    }
}

/// Paints the icon for `cmd` into `rect`. False when there is none, so
/// the caller can fall back to a text label.
pub fn paint(cmd: Command, painter: &Painter, rect: Rect, color: Color32) -> bool {
    let p = Pen { painter, rect, stroke: Stroke::new(1.4, color) };
    let soft = color.gamma_multiply(0.35);
    match cmd {
        Command::Box => {
            p.closed(&[(0.15, 0.32), (0.5, 0.15), (0.85, 0.32), (0.85, 0.7), (0.5, 0.87), (0.15, 0.7)]);
            p.line(&[(0.15, 0.32), (0.5, 0.49), (0.85, 0.32)]);
            p.line(&[(0.5, 0.49), (0.5, 0.87)]);
        }
        Command::Cylinder => {
            p.ellipse((0.5, 0.25), 0.32, 0.12);
            p.line(&[(0.18, 0.25), (0.18, 0.75)]);
            p.line(&[(0.82, 0.25), (0.82, 0.75)]);
            p.arc((0.5, 0.75), 0.32, 0.12, 0.0, PI);
        }
        Command::Cone => {
            p.line(&[(0.18, 0.75), (0.5, 0.12), (0.82, 0.75)]);
            p.arc((0.5, 0.75), 0.32, 0.12, 0.0, PI);
            p.arc((0.5, 0.75), 0.32, 0.12, PI, TAU);
        }
        Command::Sphere => {
            p.ellipse((0.5, 0.5), 0.36, 0.36);
            p.arc((0.5, 0.5), 0.36, 0.12, 0.0, PI);
            p.arc((0.5, 0.5), 0.12, 0.36, -PI / 2.0, PI / 2.0);
        }
        Command::Torus => {
            p.ellipse((0.5, 0.5), 0.4, 0.24);
            p.arc((0.5, 0.47), 0.17, 0.07, 0.0, PI);
            p.arc((0.5, 0.53), 0.14, 0.06, PI, TAU);
        }
        Command::Curve => {
            p.line(&[(0.15, 0.8), (0.85, 0.2)]);
            p.fill(&[(0.1, 0.75), (0.2, 0.75), (0.2, 0.85), (0.1, 0.85)], color);
            p.fill(&[(0.8, 0.15), (0.9, 0.15), (0.9, 0.25), (0.8, 0.25)], color);
        }
        Command::Spline => {
            let pts: Vec<(f32, f32)> = (0..=20)
                .map(|i| {
                    let t = i as f32 / 20.0;
                    (0.12 + 0.76 * t, 0.5 - 0.28 * (t * TAU).sin())
                })
                .collect();
            p.line(&pts);
            Pen { stroke: Stroke::new(1.0, soft), ..p }.line(&[(0.12, 0.5), (0.31, 0.15), (0.69, 0.85), (0.88, 0.5)]);
        }
        Command::Arc => {
            p.arc((0.5, 0.7), 0.36, 0.42, PI, TAU);
            p.line(&[(0.14, 0.7), (0.86, 0.7)]);
        }
        Command::Rectangle => p.closed(&[(0.15, 0.25), (0.85, 0.25), (0.85, 0.75), (0.15, 0.75)]),
        Command::Circle => {
            p.ellipse((0.5, 0.5), 0.35, 0.35);
            p.fill(&[(0.46, 0.46), (0.54, 0.46), (0.54, 0.54), (0.46, 0.54)], color);
        }
        Command::Ellipse => p.ellipse((0.5, 0.5), 0.4, 0.24),
        Command::Polygon => {
            let pts: Vec<(f32, f32)> = (0..6).map(|i| i as f32 / 6.0 * TAU - PI / 2.0).map(|a| (0.5 + 0.36 * a.cos(), 0.5 + 0.36 * a.sin())).collect();
            p.closed(&pts);
        }
        Command::Extrude => {
            p.closed(&[(0.15, 0.65), (0.55, 0.65), (0.85, 0.85), (0.45, 0.85)]);
            p.arrow((0.5, 0.75), (0.5, 0.12));
        }
        Command::Revolve => {
            Pen { stroke: Stroke::new(1.0, soft), ..p }.line(&[(0.5, 0.08), (0.5, 0.92)]);
            p.arc((0.5, 0.55), 0.34, 0.14, 0.2, PI + 1.2);
            p.line(&[(0.62, 0.2), (0.78, 0.2), (0.78, 0.45)]);
        }
        Command::Loft => {
            p.ellipse((0.5, 0.2), 0.18, 0.07);
            p.closed(&[(0.18, 0.72), (0.82, 0.72), (0.82, 0.88), (0.18, 0.88)]);
            p.line(&[(0.32, 0.2), (0.18, 0.72)]);
            p.line(&[(0.68, 0.2), (0.82, 0.72)]);
        }
        Command::Sweep => {
            Pen { stroke: Stroke::new(1.0, soft), ..p }.arc((0.85, 0.85), 0.6, 0.6, PI, 1.5 * PI);
            p.ellipse((0.25, 0.8), 0.1, 0.1);
            p.ellipse((0.8, 0.25), 0.1, 0.1);
            p.arc((0.85, 0.85), 0.7, 0.7, PI, 1.5 * PI);
            p.arc((0.85, 0.85), 0.5, 0.5, PI, 1.5 * PI);
        }
        Command::Pipe => {
            p.arc((0.85, 0.85), 0.72, 0.72, PI, 1.5 * PI);
            p.arc((0.85, 0.85), 0.42, 0.42, PI, 1.5 * PI);
            p.ellipse((0.28, 0.85), 0.15, 0.06);
            p.ellipse((0.85, 0.28), 0.06, 0.15);
        }
        Command::Fillet => {
            p.line(&[(0.15, 0.85), (0.15, 0.55)]);
            p.arc((0.55, 0.55), 0.4, 0.4, PI, 1.5 * PI);
            p.line(&[(0.55, 0.15), (0.85, 0.15)]);
            Pen { stroke: Stroke::new(1.0, soft), ..p }.line(&[(0.15, 0.55), (0.15, 0.15), (0.55, 0.15)]);
        }
        Command::Chamfer => {
            p.line(&[(0.15, 0.85), (0.15, 0.5), (0.5, 0.15), (0.85, 0.15)]);
            Pen { stroke: Stroke::new(1.0, soft), ..p }.line(&[(0.15, 0.5), (0.15, 0.15), (0.5, 0.15)]);
        }
        Command::Shell => {
            p.closed(&[(0.12, 0.2), (0.88, 0.2), (0.88, 0.85), (0.12, 0.85)]);
            p.line(&[(0.25, 0.2), (0.25, 0.72), (0.75, 0.72), (0.75, 0.2)]);
        }
        Command::Union | Command::Subtract | Command::Intersect => {
            let (a, b) = ((0.38, 0.5), (0.62, 0.5));
            let r = 0.26;
            // The lens where the two circles overlap spans ±acos(0.12/r) on each.
            let h = (0.12f32 / r).acos();
            let lens = |pen: &Pen| {
                pen.arc(a, r, r, -h, h);
                pen.arc(b, r, r, PI - h, PI + h);
            };
            match cmd {
                Command::Union => {
                    p.arc(a, r, r, h, TAU - h);
                    p.arc(b, r, r, -(PI - h), PI - h);
                }
                Command::Subtract => {
                    p.arc(a, r, r, h, TAU - h);
                    p.arc(b, r, r, PI - h, PI + h);
                    Pen { stroke: Stroke::new(1.0, soft), ..p }.arc(b, r, r, -(PI - h), PI - h);
                }
                _ => {
                    lens(&p);
                    let s = Pen { stroke: Stroke::new(1.0, soft), ..p };
                    s.arc(a, r, r, h, TAU - h);
                    s.arc(b, r, r, -(PI - h), PI - h);
                }
            }
        }
        Command::Move => {
            p.arrow((0.5, 0.5), (0.5, 0.1));
            p.arrow((0.5, 0.5), (0.5, 0.9));
            p.arrow((0.5, 0.5), (0.1, 0.5));
            p.arrow((0.5, 0.5), (0.9, 0.5));
        }
        Command::Rotate => {
            p.arc((0.5, 0.5), 0.34, 0.34, -0.3, 1.5 * PI);
            let e = (0.5 + 0.34 * (-0.3f32).cos(), 0.5 + 0.34 * (-0.3f32).sin());
            p.line(&[(e.0 - 0.14, e.1 - 0.02), e, (e.0 + 0.02, e.1 + 0.14)]);
        }
        Command::Scale => {
            p.closed(&[(0.12, 0.55), (0.45, 0.55), (0.45, 0.88), (0.12, 0.88)]);
            Pen { stroke: Stroke::new(1.0, soft), ..p }.closed(&[(0.12, 0.12), (0.88, 0.12), (0.88, 0.88), (0.12, 0.88)]);
            p.arrow((0.5, 0.5), (0.82, 0.18));
        }
        Command::MirrorX => {
            Pen { stroke: Stroke::new(1.0, soft), ..p }.line(&[(0.5, 0.08), (0.5, 0.92)]);
            p.closed(&[(0.4, 0.2), (0.4, 0.8), (0.12, 0.8)]);
            p.closed(&[(0.6, 0.2), (0.6, 0.8), (0.88, 0.8)]);
        }
        Command::RadialArray => {
            for i in 0..6 {
                let a = i as f32 / 6.0 * TAU;
                p.ellipse((0.5 + 0.32 * a.cos(), 0.5 + 0.32 * a.sin()), 0.08, 0.08);
            }
        }
        Command::Duplicate => {
            p.closed(&[(0.32, 0.12), (0.88, 0.12), (0.88, 0.68), (0.32, 0.68)]);
            p.line(&[(0.32, 0.32), (0.12, 0.32), (0.12, 0.88), (0.68, 0.88), (0.68, 0.68)]);
        }
        Command::Delete => {
            p.closed(&[(0.25, 0.3), (0.75, 0.3), (0.7, 0.88), (0.3, 0.88)]);
            p.line(&[(0.15, 0.3), (0.85, 0.3)]);
            p.line(&[(0.4, 0.3), (0.42, 0.15), (0.58, 0.15), (0.6, 0.3)]);
            p.line(&[(0.43, 0.45), (0.45, 0.75)]);
            p.line(&[(0.57, 0.45), (0.55, 0.75)]);
        }
        Command::Palette => {
            p.ellipse((0.43, 0.43), 0.26, 0.26);
            p.line(&[(0.62, 0.62), (0.86, 0.86)]);
        }
        Command::Boolean => {
            p.ellipse((0.38, 0.5), 0.26, 0.26);
            p.ellipse((0.62, 0.5), 0.26, 0.26);
        }
        Command::LinearArray => {
            for (x, y) in [(0.14, 0.14), (0.56, 0.14), (0.14, 0.56), (0.56, 0.56)] {
                p.closed(&[(x, y), (x + 0.3, y), (x + 0.3, y + 0.3), (x, y + 0.3)]);
            }
        }
        Command::Trim => {
            p.ellipse((0.28, 0.75), 0.13, 0.13);
            p.ellipse((0.72, 0.75), 0.13, 0.13);
            p.line(&[(0.36, 0.64), (0.78, 0.12)]);
            p.line(&[(0.64, 0.64), (0.22, 0.12)]);
        }
        Command::SplitCurve => {
            p.line(&[(0.1, 0.7), (0.42, 0.45)]);
            p.line(&[(0.58, 0.55), (0.9, 0.3)]);
            p.fill(&[(0.44, 0.38), (0.56, 0.5), (0.44, 0.62), (0.32, 0.5)], color);
        }
        Command::BridgeCurve => {
            p.line(&[(0.08, 0.8), (0.3, 0.8)]);
            p.line(&[(0.7, 0.2), (0.92, 0.2)]);
            Pen { stroke: Stroke::new(1.4, soft), ..p }.line(&[(0.3, 0.8), (0.5, 0.8), (0.5, 0.2), (0.7, 0.2)]);
        }
        Command::Measure => {
            p.closed(&[(0.08, 0.62), (0.62, 0.08), (0.92, 0.38), (0.38, 0.92)]);
            for k in 1..5 {
                let t = k as f32 * 0.2;
                let (x, y) = (0.08 + 0.54 * t, 0.62 - 0.54 * t);
                p.line(&[(x, y), (x + 0.1, y + 0.1)]);
            }
        }
        Command::Helix => {
            // A coil seen from the side.
            let pts: Vec<(f32, f32)> = (0..=96).map(|i| {
                let t = i as f32 / 96.0;
                let a = t * 3.5 * TAU;
                (0.5 + 0.32 * a.cos(), 0.85 - 0.7 * t + 0.08 * a.sin())
            }).collect();
            p.line(&pts);
        }
        Command::Spiral => {
            let pts: Vec<(f32, f32)> = (0..=64).map(|i| {
                let a = i as f32 / 64.0 * 3.0 * TAU;
                let r = 0.06 + 0.36 * i as f32 / 64.0;
                (0.5 + r * a.cos(), 0.5 + r * a.sin())
            }).collect();
            p.line(&pts);
        }
        Command::ModeVertex => {
            Pen { stroke: Stroke::new(1.0, soft), ..p }.closed(&[(0.2, 0.2), (0.8, 0.2), (0.8, 0.8), (0.2, 0.8)]);
            for (x, y) in [(0.2, 0.2), (0.8, 0.2), (0.8, 0.8), (0.2, 0.8)] {
                p.fill(&[(x - 0.09, y - 0.09), (x + 0.09, y - 0.09), (x + 0.09, y + 0.09), (x - 0.09, y + 0.09)], color);
            }
        }
        Command::ModeEdge | Command::ModeFace | Command::ModeObject => {
            // A cube in line art; the mode's element is drawn solid.
            let top = [(0.5, 0.12), (0.88, 0.3), (0.5, 0.48), (0.12, 0.3)];
            let left = [(0.12, 0.3), (0.5, 0.48), (0.5, 0.9), (0.12, 0.72)];
            let right = [(0.5, 0.48), (0.88, 0.3), (0.88, 0.72), (0.5, 0.9)];
            match cmd {
                Command::ModeFace => p.fill(&top, color),
                Command::ModeObject => {
                    p.fill(&top, color);
                    p.fill(&left, color.gamma_multiply(0.75));
                    p.fill(&right, color.gamma_multiply(0.55));
                }
                _ => {}
            }
            let thin = Pen { stroke: Stroke::new(1.1, if cmd == Command::ModeEdge { soft } else { color }), ..p };
            thin.closed(&top);
            thin.closed(&left);
            thin.closed(&right);
            if cmd == Command::ModeEdge {
                Pen { stroke: Stroke::new(2.2, color), ..p }.line(&[(0.5, 0.48), (0.5, 0.9)]);
            }
        }
        Command::ToggleSection => {
            p.ellipse((0.5, 0.5), 0.36, 0.36);
            p.fill(&[(0.5, 0.14), (0.5, 0.86), (0.36, 0.83), (0.24, 0.75), (0.16, 0.6), (0.14, 0.5), (0.16, 0.4), (0.24, 0.25), (0.36, 0.17)], color);
        }
        Command::ToggleOrtho => {
            p.closed(&[(0.15, 0.3), (0.55, 0.3), (0.55, 0.85), (0.15, 0.85)]);
            Pen { stroke: Stroke::new(1.0, soft), ..p }.closed(&[(0.45, 0.15), (0.85, 0.15), (0.85, 0.7), (0.45, 0.7)]);
        }
        Command::ToggleXray => {
            p.closed(&[(0.18, 0.18), (0.82, 0.18), (0.82, 0.82), (0.18, 0.82)]);
            Pen { stroke: Stroke::new(1.0, soft), ..p }.line(&[(0.18, 0.82), (0.82, 0.18)]);
            Pen { stroke: Stroke::new(1.0, soft), ..p }.line(&[(0.18, 0.5), (0.5, 0.18)]);
            Pen { stroke: Stroke::new(1.0, soft), ..p }.line(&[(0.5, 0.82), (0.82, 0.5)]);
        }
        Command::ToggleEdges => {
            p.closed(&[(0.15, 0.15), (0.85, 0.15), (0.85, 0.85), (0.15, 0.85)]);
            p.line(&[(0.15, 0.5), (0.85, 0.5)]);
            p.line(&[(0.5, 0.15), (0.5, 0.85)]);
        }
        _ => return false,
    }
    true
}

/// Three dots: a "more" menu.
pub fn dots(painter: &Painter, rect: Rect, color: Color32) {
    for k in [-1.0, 0.0, 1.0] {
        painter.circle_filled(rect.center() + egui::vec2(k * rect.width() * 0.3, 0.0), 1.6, color);
    }
}

/// A window with one side bar marked: the left or right panel toggle.
pub fn sidebar(painter: &Painter, rect: Rect, color: Color32, left: bool) {
    let r = rect.shrink(2.0);
    painter.rect_stroke(r, 2.0, Stroke::new(1.3, color), egui::StrokeKind::Inside);
    let w = r.width() * 0.32;
    let bar = if left { Rect::from_min_size(r.min, egui::vec2(w, r.height())) } else { Rect::from_min_size(pos2(r.max.x - w, r.min.y), egui::vec2(w, r.height())) };
    painter.rect_filled(bar.shrink(2.0), 1.0, color);
}

/// A chevron pointing down (open) or right (closed).
pub fn chevron(painter: &Painter, rect: Rect, color: Color32, open: bool) {
    let c = rect.center();
    let s = rect.width().min(rect.height()) * 0.22;
    let pts = if open { vec![c + egui::vec2(-s, -s * 0.5), c + egui::vec2(0.0, s * 0.5), c + egui::vec2(s, -s * 0.5)] } else { vec![c + egui::vec2(-s * 0.5, -s), c + egui::vec2(s * 0.5, 0.0), c + egui::vec2(-s * 0.5, s)] };
    painter.add(Shape::line(pts, Stroke::new(1.4, color)));
}

/// A cross: close or leave.
pub fn cross(painter: &Painter, rect: Rect, color: Color32) {
    let r = rect.shrink(rect.width() * 0.3);
    painter.line_segment([r.left_top(), r.right_bottom()], Stroke::new(1.4, color));
    painter.line_segment([r.right_top(), r.left_bottom()], Stroke::new(1.4, color));
}

/// An eye, open or struck through, for visibility toggles.
pub fn eye(painter: &Painter, rect: Rect, color: Color32, open: bool) {
    let p = Pen { painter, rect, stroke: Stroke::new(1.3, color) };
    // An almond: the lids meet in points at either corner.
    let lid = |sign: f32| -> Vec<(f32, f32)> {
        (0..=16)
            .map(|i| {
                let t = i as f32 / 16.0;
                (0.06 + 0.88 * t, 0.5 + sign * 0.62 * t * (1.0 - t) * 2.0 * 0.62)
            })
            .collect()
    };
    p.line(&lid(-1.0));
    p.line(&lid(1.0));
    if open {
        p.painter.circle_filled(p.at(0.5, 0.5), rect.height() * 0.17, color);
    } else {
        p.line(&[(0.18, 0.88), (0.82, 0.12)]);
    }
}

/// A padlock, shut or open.
pub fn lock(painter: &Painter, rect: Rect, color: Color32, shut: bool) {
    let p = Pen { painter, rect, stroke: Stroke::new(1.3, color) };
    p.closed(&[(0.22, 0.48), (0.78, 0.48), (0.78, 0.92), (0.22, 0.92)]);
    if shut {
        p.arc((0.5, 0.48), 0.2, 0.32, std::f32::consts::PI, std::f32::consts::TAU);
    } else {
        p.arc((0.72, 0.4), 0.2, 0.28, std::f32::consts::PI, std::f32::consts::TAU);
    }
}

/// A folder, for Outliner groups.
pub fn folder(painter: &Painter, rect: Rect, color: Color32) {
    let p = Pen { painter, rect, stroke: Stroke::new(1.2, color) };
    p.closed(&[(0.1, 0.25), (0.4, 0.25), (0.5, 0.36), (0.9, 0.36), (0.9, 0.82), (0.1, 0.82)]);
}

/// A shaded ball previewing a shader mode, drawn from scratch: a lit
/// sphere for plain shading, warm metal for rendered, stripes for zebra,
/// rings for isophotes, green to red for draft and blue/red for face
/// direction.
pub fn swatch(p: &Painter, rect: Rect, a: crate::render::Analysis) {
    use crate::render::Analysis;
    let c = rect.center();
    let r = rect.width().min(rect.height()) * 0.5;
    let light = c + egui::vec2(-0.35, -0.4) * r;
    // Rows of the ball, each a horizontal chord coloured by its height.
    let rows = (r * 2.0) as i32;
    let row_colour = |t: f32| -> Option<Color32> {
        match a {
            Analysis::Zebra => Some(if ((t + 1.0) * 4.0) as i32 % 2 == 0 { Color32::from_gray(225) } else { Color32::from_gray(25) }),
            Analysis::Draft => Some(if t < -0.15 {
                Color32::from_rgb(70, 170, 80)
            } else if t < 0.15 {
                Color32::from_rgb(220, 190, 60)
            } else {
                Color32::from_rgb(200, 60, 55)
            }),
            _ => None,
        }
    };
    if row_colour(0.0).is_some() {
        for i in 0..rows {
            let y = -r + (i as f32 + 0.5);
            let half = (r * r - y * y).max(0.0).sqrt();
            if let Some(col) = row_colour(y / r) {
                p.line_segment([egui::pos2(c.x - half, c.y + y), egui::pos2(c.x + half, c.y + y)], Stroke::new(1.2, col));
            }
        }
    } else {
        let (dark, bright) = match a {
            Analysis::Render => (Color32::from_rgb(70, 32, 24), Color32::from_rgb(255, 214, 190)),
            Analysis::Direction => (Color32::from_rgb(20, 40, 110), Color32::from_rgb(140, 175, 255)),
            _ => (Color32::from_gray(52), Color32::from_gray(225)),
        };
        if a == Analysis::Direction {
            p.circle_filled(c, r, Color32::from_rgb(190, 50, 45));
        }
        // Concentric discs shrinking toward the highlight.
        let steps = 22;
        for i in 0..steps {
            let t = i as f32 / (steps - 1) as f32;
            let rad = r * (1.0 - t * 0.92) * if a == Analysis::Direction { 0.86 } else { 1.0 };
            let centre = c.lerp(light, t * 0.9) + if a == Analysis::Direction { egui::vec2(-0.1, -0.1) * r } else { egui::Vec2::ZERO };
            p.circle_filled(centre, rad, lerp_colour(dark, bright, t.powf(1.6)));
        }
    }
    if a == Analysis::Isophote {
        for k in 1..5 {
            p.circle_stroke(c.lerp(light, 0.5 - k as f32 * 0.08), r * 0.2 * k as f32, Stroke::new(1.0, Color32::from_rgb(40, 60, 160)));
        }
    }
}

/// A small ball shaded in one of the looks of plain shaded mode.
pub fn look_swatch(p: &Painter, rect: Rect, l: crate::render::Look) {
    use crate::render::Look;
    let c = rect.center();
    let r = rect.width().min(rect.height()) * 0.5;
    let light = c + egui::vec2(-0.35, -0.4) * r;
    if l == Look::Chrome {
        // Sky over a dark horizon over a warm floor.
        p.circle_filled(c, r, Color32::from_rgb(70, 60, 52));
        let rows = (r * 2.0) as i32;
        for i in 0..rows {
            let y = -r + (i as f32 + 0.5);
            let half = (r * r - y * y).max(0.0).sqrt();
            let t = y / r;
            let col = if t < 0.05 { lerp_colour(Color32::from_rgb(245, 248, 255), Color32::from_rgb(130, 145, 165), (t + 1.0).clamp(0.0, 1.0)) } else { lerp_colour(Color32::from_rgb(25, 22, 20), Color32::from_rgb(85, 70, 58), t) };
            p.line_segment([egui::pos2(c.x - half, c.y + y), egui::pos2(c.x + half, c.y + y)], Stroke::new(1.2, col));
        }
        return;
    }
    let (dark, bright) = match l {
        Look::Clay => (Color32::from_gray(52), Color32::from_gray(225)),
        Look::Ceramic => (Color32::from_gray(120), Color32::from_gray(250)),
        Look::Graphite => (Color32::from_rgb(70, 80, 95), Color32::from_gray(120)),
        Look::Wax => (Color32::from_rgb(90, 35, 25), Color32::from_rgb(240, 150, 120)),
        Look::Chrome => unreachable!(),
    };
    let steps = 22;
    for i in 0..steps {
        let t = i as f32 / (steps - 1) as f32;
        p.circle_filled(c.lerp(light, t * 0.9), r * (1.0 - t * 0.92), lerp_colour(dark, bright, t.powf(1.6)));
    }
    if l == Look::Ceramic {
        p.circle_filled(light, r * 0.12, Color32::WHITE);
    }
}

fn lerp_colour(a: Color32, b: Color32, t: f32) -> Color32 {
    let l = |x: u8, y: u8| (x as f32 + (y as f32 - x as f32) * t).round() as u8;
    Color32::from_rgb(l(a.r(), b.r()), l(a.g(), b.g()), l(a.b(), b.b()))
}
