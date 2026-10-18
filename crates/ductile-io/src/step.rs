// SPDX-License-Identifier: GPL-3.0-only
//! STEP (ISO 10303-21) exchange of exact B-rep solids, AP214
//! "automotive_design" schema, which every mainstream CAD system reads.
//!
//! Solids are written as `MANIFOLD_SOLID_BREP`s of `ADVANCED_FACE`s with
//! their exact analytic or NURBS geometry; lengths are millimetres.

use ductile_brep::{Body, BodyKind, EdgeId, FaceId, VertexId};
use ductile_geom::nurbs::{NurbsCurve, NurbsSurface};
use ductile_geom::{Curve3, DVec3, Frame, Surface};
use std::collections::HashMap;
use std::fmt::Write as _;

/// STEP REAL literal: always carries a decimal point.
fn real(x: f64) -> String {
    let x = if x.abs() < 1e-300 { 0.0 } else { x };
    let s = format!("{x}");
    if s.contains('.') || s.contains('e') || s.contains("inf") || s.contains("NaN") {
        s
    } else {
        s + "."
    }
}

/// STEP string literal (ASCII only; quotes doubled, other characters
/// replaced).
fn string(s: &str) -> String {
    let mut out = String::from("'");
    for c in s.chars() {
        match c {
            '\'' => out.push_str("''"),
            '\\' => out.push_str("\\\\"),
            ' '..='~' => out.push(c),
            _ => out.push('_'),
        }
    }
    out.push('\'');
    out
}

fn list(items: impl IntoIterator<Item = String>) -> String {
    format!("({})", items.into_iter().collect::<Vec<_>>().join(","))
}

fn refs(ids: &[usize]) -> String {
    list(ids.iter().map(|i| format!("#{i}")))
}

/// Accumulates numbered entity instances.
struct Writer {
    lines: Vec<String>,
}

impl Writer {
    fn add(&mut self, body: impl AsRef<str>) -> usize {
        self.lines.push(body.as_ref().to_string());
        self.lines.len()
    }

    fn point(&mut self, p: DVec3) -> usize {
        self.add(format!("CARTESIAN_POINT('',({},{},{}))", real(p.x), real(p.y), real(p.z)))
    }

    fn direction(&mut self, d: DVec3) -> usize {
        let d = d.normalize();
        self.add(format!("DIRECTION('',({},{},{}))", real(d.x), real(d.y), real(d.z)))
    }

    fn placement(&mut self, f: &Frame) -> usize {
        let o = self.point(f.origin);
        let z = self.direction(f.z);
        let x = self.direction(f.x);
        self.add(format!("AXIS2_PLACEMENT_3D('',#{o},#{z},#{x})"))
    }

    fn curve(&mut self, c: &Curve3) -> usize {
        match c {
            Curve3::Line { origin, dir } => {
                let p = self.point(*origin);
                let d = self.direction(*dir);
                let v = self.add(format!("VECTOR('',#{d},1.)"));
                self.add(format!("LINE('',#{p},#{v})"))
            }
            Curve3::Circle { frame, radius } => {
                let a = self.placement(frame);
                self.add(format!("CIRCLE('',#{a},{})", real(*radius)))
            }
            Curve3::Ellipse { frame, rx, ry } => {
                // STEP wants semi_axis_1 >= semi_axis_2 along the reference direction.
                let (f, a, b) = if rx >= ry {
                    (*frame, *rx, *ry)
                } else {
                    (Frame { origin: frame.origin, x: frame.y, y: -frame.x, z: frame.z }, *ry, *rx)
                };
                let p = self.placement(&f);
                self.add(format!("ELLIPSE('',#{p},{},{})", real(a), real(b)))
            }
            Curve3::Nurbs(n) => self.bspline_curve(n),
        }
    }

    fn bspline_curve(&mut self, n: &NurbsCurve) -> usize {
        let pts: Vec<usize> = n.ctrl.iter().map(|h| self.point(h.truncate() / h.w)).collect();
        let (mults, knots) = knot_runs(&n.knots);
        let rational = n.ctrl.iter().any(|h| (h.w - 1.0).abs() > 1e-12);
        let pts = refs(&pts);
        let mults = list(mults.iter().map(|m| m.to_string()));
        let knots = list(knots.iter().map(|k| real(*k)));
        if rational {
            let w = list(n.ctrl.iter().map(|h| real(h.w)));
            self.add(format!(
                "(BOUNDED_CURVE() B_SPLINE_CURVE({},{pts},.UNSPECIFIED.,.F.,.F.) B_SPLINE_CURVE_WITH_KNOTS({mults},{knots},.UNSPECIFIED.) CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE({w}) REPRESENTATION_ITEM(''))",
                n.degree
            ))
        } else {
            self.add(format!("B_SPLINE_CURVE_WITH_KNOTS('',{},{pts},.UNSPECIFIED.,.F.,.F.,{mults},{knots},.UNSPECIFIED.)", n.degree))
        }
    }

    fn surface(&mut self, s: &Surface) -> usize {
        match s {
            Surface::Plane { frame } => {
                let a = self.placement(frame);
                self.add(format!("PLANE('',#{a})"))
            }
            Surface::Cylinder { frame, radius } => {
                let a = self.placement(frame);
                self.add(format!("CYLINDRICAL_SURFACE('',#{a},{})", real(*radius)))
            }
            Surface::Cone { frame, radius, half_angle } => {
                // STEP's semi-angle is positive: point the axis toward growth.
                let f = if *half_angle < 0.0 { Frame { origin: frame.origin, x: frame.x, y: -frame.y, z: -frame.z } } else { *frame };
                let a = self.placement(&f);
                self.add(format!("CONICAL_SURFACE('',#{a},{},{})", real(*radius), real(half_angle.abs())))
            }
            Surface::Sphere { frame, radius } => {
                let a = self.placement(frame);
                self.add(format!("SPHERICAL_SURFACE('',#{a},{})", real(*radius)))
            }
            Surface::Torus { frame, major, minor } => {
                let a = self.placement(frame);
                self.add(format!("TOROIDAL_SURFACE('',#{a},{},{})", real(*major), real(*minor)))
            }
            Surface::Nurbs(n) => self.bspline_surface(n),
        }
    }

    fn bspline_surface(&mut self, n: &NurbsSurface) -> usize {
        let mut rows = Vec::new();
        for i in 0..n.nu {
            let row: Vec<usize> = (0..n.nv).map(|j| n.cp(i, j)).map(|h| self.point(h.truncate() / h.w)).collect();
            rows.push(refs(&row));
        }
        let rows = list(rows);
        let (mu, ku) = knot_runs(&n.knots_u);
        let (mv, kv) = knot_runs(&n.knots_v);
        let mu = list(mu.iter().map(|m| m.to_string()));
        let mv = list(mv.iter().map(|m| m.to_string()));
        let ku = list(ku.iter().map(|k| real(*k)));
        let kv = list(kv.iter().map(|k| real(*k)));
        let rational = n.ctrl.iter().any(|h| (h.w - 1.0).abs() > 1e-12);
        if rational {
            let w = list((0..n.nu).map(|i| list((0..n.nv).map(|j| real(n.cp(i, j).w)))));
            self.add(format!(
                "(BOUNDED_SURFACE() B_SPLINE_SURFACE({},{},{rows},.UNSPECIFIED.,.F.,.F.,.F.) B_SPLINE_SURFACE_WITH_KNOTS({mu},{mv},{ku},{kv},.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE({w}) REPRESENTATION_ITEM('') SURFACE())",
                n.degree_u, n.degree_v
            ))
        } else {
            self.add(format!(
                "B_SPLINE_SURFACE_WITH_KNOTS('',{},{},{rows},.UNSPECIFIED.,.F.,.F.,.F.,{mu},{mv},{ku},{kv},.UNSPECIFIED.)",
                n.degree_u, n.degree_v
            ))
        }
    }

    fn solid(&mut self, name: &str, b: &Body) -> usize {
        let mut verts: HashMap<VertexId, usize> = HashMap::new();
        let mut edges: HashMap<EdgeId, usize> = HashMap::new();
        let mut faces = Vec::new();
        for f in b.face_ids() {
            faces.push(self.face(b, f, &mut verts, &mut edges));
        }
        let shell = self.add(format!("CLOSED_SHELL('',{})", refs(&faces)));
        self.add(format!("MANIFOLD_SOLID_BREP({},#{shell})", string(name)))
    }

    fn face(&mut self, b: &Body, f: FaceId, verts: &mut HashMap<VertexId, usize>, edges: &mut HashMap<EdgeId, usize>) -> usize {
        let face = b.face(f);
        let mut bounds = Vec::new();
        for (k, l) in b.face_loops(f).enumerate() {
            let mut oes = Vec::new();
            for c in &l.coedges {
                let ec = match edges.get(&c.edge) {
                    Some(&e) => e,
                    None => {
                        let ed = b.edge(c.edge);
                        let mut vert = |w: &mut Writer, v: VertexId| *verts.entry(v).or_insert_with(|| {
                            let p = w.point(b.vertex(v));
                            w.add(format!("VERTEX_POINT('',#{p})"))
                        });
                        let v0 = vert(self, ed.v0);
                        let v1 = vert(self, ed.v1);
                        let curve = self.curve(&ed.curve);
                        let sense = if ed.t1 >= ed.t0 { ".T." } else { ".F." };
                        let e = self.add(format!("EDGE_CURVE('',#{v0},#{v1},#{curve},{sense})"));
                        edges.insert(c.edge, e);
                        e
                    }
                };
                let orient = if c.reversed { ".F." } else { ".T." };
                oes.push(self.add(format!("ORIENTED_EDGE('',*,*,#{ec},{orient})")));
            }
            let lp = self.add(format!("EDGE_LOOP('',{})", refs(&oes)));
            let kind = if k == 0 { "FACE_OUTER_BOUND" } else { "FACE_BOUND" };
            bounds.push(self.add(format!("{kind}('',#{lp},.T.)")));
        }
        let s = self.surface(&face.surface);
        let same = if face.reversed { ".F." } else { ".T." };
        self.add(format!("ADVANCED_FACE('',{},#{s},{same})", refs(&bounds)))
    }
}

/// Distinct knot values and their multiplicities.
fn knot_runs(knots: &[f64]) -> (Vec<usize>, Vec<f64>) {
    let mut mults = Vec::new();
    let mut vals: Vec<f64> = Vec::new();
    for &k in knots {
        match vals.last() {
            Some(&v) if (k - v).abs() <= 1e-12 * (1.0 + v.abs()) => *mults.last_mut().unwrap() += 1,
            _ => {
                vals.push(k);
                mults.push(1);
            }
        }
    }
    (mults, vals)
}

/// Writes the solids among `objects` (name, body) as one STEP part.
/// Returns the number of solids written.
pub fn write_step(out: &mut impl std::io::Write, part_name: &str, objects: &[(&str, &Body)]) -> crate::Result<usize> {
    let mut w = Writer { lines: Vec::new() };
    let ctx_app = w.add("APPLICATION_CONTEXT('core data for automotive mechanical design processes')");
    w.add(format!("APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{ctx_app})"));
    let pctx = w.add(format!("PRODUCT_CONTEXT('',#{ctx_app},'mechanical')"));
    let name = string(part_name);
    let product = w.add(format!("PRODUCT({name},{name},'',(#{pctx}))"));
    w.add(format!("PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(#{product}))"));
    let pdf = w.add(format!("PRODUCT_DEFINITION_FORMATION('','',#{product})"));
    let pdctx = w.add(format!("PRODUCT_DEFINITION_CONTEXT('part definition',#{ctx_app},'design')"));
    let pd = w.add(format!("PRODUCT_DEFINITION('design','',#{pdf},#{pdctx})"));
    let pds = w.add(format!("PRODUCT_DEFINITION_SHAPE('','',#{pd})"));
    let mm = w.add("(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.))");
    let rad = w.add("(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.))");
    let sr = w.add("(NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT())");
    let unc = w.add(format!("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#{mm},'distance_accuracy_value','confusion accuracy')"));
    let ctx = w.add(format!(
        "(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{unc})) GLOBAL_UNIT_ASSIGNED_CONTEXT((#{mm},#{rad},#{sr})) REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY'))"
    ));
    let origin = w.placement(&Frame::WORLD);
    let mut items = vec![origin];
    for (n, b) in objects {
        if b.kind == BodyKind::Solid && !b.faces.is_empty() {
            items.push(w.solid(n, b));
        }
    }
    let rep = w.add(format!("ADVANCED_BREP_SHAPE_REPRESENTATION({name},{},#{ctx})", refs(&items)));
    w.add(format!("SHAPE_DEFINITION_REPRESENTATION(#{pds},#{rep})"));

    let mut s = String::new();
    s.push_str("ISO-10303-21;\nHEADER;\n");
    let _ = writeln!(s, "FILE_DESCRIPTION(('Ductile model'),'2;1');");
    let _ = writeln!(s, "FILE_NAME({name},'',(''),(''),'Ductile','Ductile {}','');", env!("CARGO_PKG_VERSION"));
    s.push_str("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\nENDSEC;\nDATA;\n");
    for (i, l) in w.lines.iter().enumerate() {
        let _ = writeln!(s, "#{}={};", i + 1, l);
    }
    s.push_str("ENDSEC;\nEND-ISO-10303-21;\n");
    out.write_all(s.as_bytes())?;
    Ok(items.len() - 1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use ductile_brep::primitives::*;

    #[test]
    fn writes_well_formed_file() {
        let b = make_box(&Frame::WORLD, DVec3::new(1.0, 2.0, 3.0));
        let s = make_sphere_in(&Frame::WORLD, 1.0);
        let mut out = Vec::new();
        let n = write_step(&mut out, "Test 'part'", &[("box", &b), ("ball", &s)]).unwrap();
        assert_eq!(n, 2);
        let text = String::from_utf8(out).unwrap();
        assert!(text.contains("MANIFOLD_SOLID_BREP('box'"));
        assert!(text.contains("SPHERICAL_SURFACE"));
        assert!(text.contains("'Test ''part'''"));
        assert_eq!(text.matches("ADVANCED_FACE").count(), 7);
        // Every reference points at a defined instance.
        let count = text.lines().filter(|l| l.starts_with('#')).count();
        for cap in text.split('#').skip(1) {
            let num: String = cap.chars().take_while(|c| c.is_ascii_digit()).collect();
            if let Ok(n) = num.parse::<usize>() {
                assert!(n >= 1 && n <= count, "dangling #{n}");
            }
        }
        assert_eq!(real(1.0), "1.");
        assert_eq!(real(-0.25), "-0.25");
    }
}
