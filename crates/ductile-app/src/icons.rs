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
        _ => return false,
    }
    true
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
