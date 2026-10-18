// SPDX-License-Identifier: GPL-3.0-only
//! SVG import: the outlines of an SVG drawing's shapes as planar curves,
//! one chain per subpath, on the XY plane with y pointing up and one SVG
//! user unit to one model unit.

use ductile_brep::profile::Segment;
use ductile_geom::{Curve3, DVec3, DVec4, NurbsCurve};

/// The subpaths of every shape in `svg` (paths, rectangles, circles,
/// ellipses, lines, polygons; not text), each a chain of segments end to
/// end; closed subpaths end where they start.
pub fn read_svg(svg: &str) -> crate::Result<Vec<Vec<Segment>>> {
    let tree = usvg::Tree::from_str(svg, &usvg::Options::default()).map_err(|e| crate::IoError::Parse(format!("SVG: {e}")))?;
    let mut out = vec![];
    collect(tree.root(), &mut out);
    Ok(out)
}

fn collect(g: &usvg::Group, out: &mut Vec<Vec<Segment>>) {
    for n in g.children() {
        match n {
            usvg::Node::Group(c) => collect(c, out),
            usvg::Node::Path(p) => path_chains(p.data(), p.abs_transform(), out),
            _ => {}
        }
    }
}

fn bezier(pts: Vec<DVec3>) -> Segment {
    let d = pts.len() - 1;
    let knots: Vec<f64> = std::iter::repeat_n(0.0, d + 1).chain(std::iter::repeat_n(1.0, d + 1)).collect();
    let ctrl: Vec<DVec4> = pts.iter().map(|p| p.extend(1.0)).collect();
    Segment::new(Curve3::Nurbs(NurbsCurve::new(d, knots, ctrl)), 0.0, 1.0)
}

fn path_chains(data: &usvg::tiny_skia_path::Path, t: usvg::Transform, out: &mut Vec<Vec<Segment>>) {
    use usvg::tiny_skia_path::PathSegment as S;
    let map = |p: usvg::tiny_skia_path::Point| {
        let (x, y) = (p.x as f64, p.y as f64);
        DVec3::new(t.sx as f64 * x + t.kx as f64 * y + t.tx as f64, -(t.ky as f64 * x + t.sy as f64 * y + t.ty as f64), 0.0)
    };
    let (mut start, mut at) = (DVec3::ZERO, DVec3::ZERO);
    let mut chain: Vec<Segment> = vec![];
    let mut flush = |chain: &mut Vec<Segment>| {
        if !chain.is_empty() {
            out.push(std::mem::take(chain));
        }
    };
    for s in data.segments() {
        match s {
            S::MoveTo(p) => {
                flush(&mut chain);
                start = map(p);
                at = start;
            }
            S::LineTo(p) => {
                let q = map(p);
                if q.distance(at) > 1e-9 {
                    chain.push(Segment::line(at, q));
                }
                at = q;
            }
            S::QuadTo(c, p) => {
                let q = map(p);
                chain.push(bezier(vec![at, map(c), q]));
                at = q;
            }
            S::CubicTo(c1, c2, p) => {
                let q = map(p);
                chain.push(bezier(vec![at, map(c1), map(c2), q]));
                at = q;
            }
            S::Close => {
                if at.distance(start) > 1e-9 {
                    chain.push(Segment::line(at, start));
                }
                at = start;
                flush(&mut chain);
            }
        }
    }
    flush(&mut chain);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_shapes_as_closed_chains() {
        let svg = r#"<svg xmlns="http://www.w3.org/2000/svg" width="100" height="100">
            <rect x="10" y="10" width="30" height="20" fill="black"/>
            <g transform="translate(50 0)"><circle cx="20" cy="20" r="10" fill="red"/></g>
            <path d="M 0 60 Q 20 40 40 60 L 40 80 Z" stroke="black" fill="none"/>
        </svg>"#;
        let chains = read_svg(svg).unwrap();
        assert_eq!(chains.len(), 3);
        for c in &chains {
            assert!(c.first().unwrap().start().distance(c.last().unwrap().end()) < 1e-6);
        }
        // The rectangle: 30 wide, y flipped below the axis.
        let pts: Vec<DVec3> = chains[0].iter().map(|s| s.start()).collect();
        let (minx, maxx) = pts.iter().fold((f64::MAX, f64::MIN), |a, p| (a.0.min(p.x), a.1.max(p.x)));
        assert!((maxx - minx - 30.0).abs() < 1e-4 && pts.iter().all(|p| p.y <= -10.0 + 1e-6));
        // The circle, moved by its group, around (70, -20) with radius 10.
        for s in &chains[1] {
            for p in s.sample(1e-3) {
                assert!((p.distance(DVec3::new(70.0, -20.0, 0.0)) - 10.0).abs() < 0.05, "{p}");
            }
        }
    }
}
