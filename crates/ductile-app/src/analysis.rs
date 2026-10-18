// SPDX-License-Identifier: GPL-3.0-only
//! Shape analysis drawn over the model: curvature combs on curves and
//! edges, and the continuity (G0 / G1 / G2) where two of them meet, the
//! everyday tools for judging class-A curves.

use ductile_geom::{Curve3, DVec3};

/// The curvature vector at `t`: towards the centre of curvature, of
/// length 1 / radius.
pub fn curvature(c: &Curve3, t: f64) -> DVec3 {
    let [_, d1, d2] = c.derivs(t);
    let s2 = d1.length_squared();
    if s2 < 1e-300 {
        return DVec3::ZERO;
    }
    (d2 - d1 * (d2.dot(d1) / s2)) / s2
}

/// Parameters to sample a comb at: evenly within each knot span of a
/// spline (curvature jumps at knots, so spans are never straddled), evenly
/// over the range otherwise.
fn samples(c: &Curve3, t0: f64, t1: f64) -> Vec<f64> {
    let mut breaks = vec![t0];
    if let Curve3::Nurbs(n) = c {
        breaks.extend(n.knots.iter().copied().filter(|k| *k > t0 + 1e-12 && *k < t1 - 1e-12));
        breaks.dedup_by(|a, b| (*a - *b).abs() < 1e-12);
    }
    breaks.push(t1);
    let spans = breaks.len() - 1;
    let per = (240 / spans).clamp(4, 48);
    let mut out = Vec::with_capacity(spans * per + 1);
    for w in breaks.windows(2) {
        for i in 0..per {
            out.push(w[0] + (w[1] - w[0]) * i as f64 / per as f64);
        }
    }
    out.push(t1);
    out
}

/// A curvature comb: teeth from the curve outwards (away from the centre
/// of curvature, as Plasticity draws them) of length `scale`·curvature,
/// and the envelope through their tips.
pub fn comb(c: &Curve3, t0: f64, t1: f64, scale: f64) -> (Vec<(DVec3, DVec3)>, Vec<DVec3>) {
    let mut teeth = Vec::new();
    let mut tips = Vec::new();
    for t in samples(c, t0, t1) {
        let p = c.point(t);
        let tip = p - curvature(c, t) * scale;
        teeth.push((p, tip));
        tips.push(tip);
    }
    (teeth, tips)
}

/// How two curves meet at a point, `a` and `b` being their parameters
/// there: "G0" (touching), "G1" (tangent), "G2" (same curvature) or None
/// if they don't touch.
pub fn continuity(ca: &Curve3, ta: f64, cb: &Curve3, tb: f64, tol: f64) -> Option<&'static str> {
    if ca.point(ta).distance(cb.point(tb)) > tol {
        return None;
    }
    let (Some(u), Some(v)) = (ca.tangent(ta).try_normalize(), cb.tangent(tb).try_normalize()) else { return Some("G0") };
    // Either may run either way through the point.
    if u.cross(v).length() > 1e-3 {
        return Some("G0");
    }
    let (ka, kb) = (curvature(ca, ta), curvature(cb, tb));
    let scale = ka.length().max(kb.length());
    if scale < 1e-12 || (ka - kb).length() < 0.02 * scale {
        Some("G2")
    } else {
        Some("G1")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ductile_geom::{Frame, NurbsCurve};

    #[test]
    fn circle_comb_is_even_and_points_out() {
        let c = Curve3::Circle { frame: Frame::WORLD, radius: 2.0 };
        let (teeth, _) = comb(&c, 0.0, 1.0, 1.0);
        for (p, tip) in teeth {
            assert!((tip.length() - 2.5).abs() < 1e-9 && p.distance(tip) - 0.5 < 1e-9);
        }
        assert!(curvature(&Curve3::line_through(DVec3::ZERO, DVec3::X), 0.3).length() < 1e-15);
    }

    #[test]
    fn tells_tangent_from_curvature_continuous() {
        let line = Curve3::line_through(DVec3::new(-1.0, 0.0, 0.0), DVec3::ZERO);
        let circle = Curve3::Circle { frame: Frame::new(DVec3::Y, DVec3::Z, -DVec3::Y), radius: 1.0 };
        // The circle starts at the origin heading along +X.
        assert!(circle.point(0.0).length() < 1e-12);
        assert_eq!(continuity(&line, 1.0, &circle, 0.0, 1e-9), Some("G1"));
        let kink = Curve3::line_through(DVec3::ZERO, DVec3::new(0.0, 1.0, 0.0));
        assert_eq!(continuity(&line, 1.0, &kink, 0.0, 1e-9), Some("G0"));
        assert_eq!(continuity(&line, 0.0, &kink, 0.0, 1e-9), None);
        // A straight cubic spline continues a line with zero curvature.
        let pts = [DVec3::ZERO, DVec3::new(1.0, 0.0, 0.0), DVec3::new(2.0, 0.0, 0.0), DVec3::new(3.0, 1.0, 0.0)];
        let s = Curve3::Nurbs(NurbsCurve::new(3, vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0], pts.iter().map(|p| p.extend(1.0)).collect()));
        assert_eq!(continuity(&line, 1.0, &s, 0.0, 1e-9), Some("G2"));
    }
}
