// SPDX-License-Identifier: GPL-3.0-only
//! Random modeling sessions: a block gets holes, bosses, pockets, fillets
//! and chamfers in random order and places, each step building on the
//! last, as a person would model a part. Reports steps that fail or leave
//! a broken solid (open edges, or a volume change of the wrong sign).
//!
//! cargo run --release -p ductile-brep --example workflow_stress [SESSIONS] [SEED]
//! `ONLY=n` runs one session and prints each step.

use ductile_brep::ops::boolean::{boolean, open_edges, BoolOp};
use ductile_brep::ops::fillet::{blend_edges, Blend};
use ductile_brep::primitives::{make_box, make_cylinder};
use ductile_brep::tess::{tessellate, TessOptions};
use ductile_brep::{Body, EdgeId};
use ductile_geom::{DVec3, Frame};

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.next()
    }
    fn pick(&mut self, n: usize) -> usize {
        ((self.next() * n as f64) as usize).min(n - 1)
    }
}

fn at(x: f64, y: f64, z: f64) -> Frame {
    Frame { origin: DVec3::new(x, y, z), ..Frame::WORLD }
}

fn vol(b: &Body) -> f64 {
    tessellate(b, &TessOptions { chord_tol: 1e-3, angle_tol: 0.1 }).volume()
}

/// Sign the volume change should have: -1 less, +1 more.
enum Step {
    Cut(String, Body),
    Add(String, Body),
    Blend(String, Vec<EdgeId>, Blend),
}

fn random_step(rng: &mut Rng, b: &Body, size: DVec3) -> Step {
    let (lo, hi) = {
        let bb = b.bbox();
        (bb.min, bb.max)
    };
    let x = rng.range(lo.x + 0.3, hi.x - 0.3);
    let y = rng.range(lo.y + 0.3, hi.y - 0.3);
    match rng.pick(6) {
        0 => {
            let r = rng.range(0.15, 0.6);
            Step::Cut(format!("hole r{r:.3} at ({x:.3}, {y:.3})"), make_cylinder(&at(x, y, lo.z - 1.0), r, hi.z - lo.z + 2.0))
        }
        1 => {
            let r = rng.range(0.2, 0.6);
            let z = rng.range(lo.z + 0.3 * size.z, hi.z - 0.2);
            Step::Add(format!("boss r{r:.3} at ({x:.3}, {y:.3}, {z:.3})"), make_cylinder(&at(x, y, z), r, hi.z - z + rng.range(0.2, 0.8)))
        }
        2 => {
            let (w, d) = (rng.range(0.3, 1.2), rng.range(0.3, 1.2));
            let depth = rng.range(0.2, 0.7) * size.z;
            Step::Cut(format!("pocket {w:.3}x{d:.3} deep {depth:.3} at ({x:.3}, {y:.3})"), make_box(&at(x - w / 2.0, y - d / 2.0, hi.z - depth), DVec3::new(w, d, depth + 1.0)))
        }
        3 => {
            let r = rng.range(0.1, 0.5);
            Step::Cut(format!("side hole r{r:.3} at y {y:.3}"), make_cylinder(&Frame::new(DVec3::new(lo.x - 1.0, y, rng.range(lo.z + 0.6, hi.z - 0.6)), DVec3::X, DVec3::Y), r, hi.x - lo.x + 2.0))
        }
        k => {
            let edges: Vec<EdgeId> = b.edge_ids().filter(|e| b.edge(*e).length() > 0.4 && b.edge_faces(*e).len() == 2).collect();
            let n = 1 + rng.pick(3);
            let mut pick: Vec<EdgeId> = (0..n).map(|_| edges[rng.pick(edges.len())]).collect();
            pick.sort();
            pick.dedup();
            let r = rng.range(0.05, 0.15);
            let blend = if k == 4 { Blend::Fillet(r) } else { Blend::Chamfer(r) };
            Step::Blend(format!("{blend:?} edges {:?}", pick.iter().map(|e| e.0).collect::<Vec<_>>()), pick, blend)
        }
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let sessions: usize = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(40);
    let seed: u64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(1);
    let only: Option<usize> = std::env::var("ONLY").ok().and_then(|s| s.parse().ok());
    let mut fails = Vec::new();
    let mut steps_run = 0;
    for s in 0..sessions {
        if only.is_some_and(|o| o != s) {
            continue;
        }
        let mut rng = Rng(seed.wrapping_mul(1_000_003).wrapping_add(s as u64 * 7919));
        let size = DVec3::new(rng.range(3.0, 5.0), rng.range(2.0, 4.0), rng.range(1.0, 2.0));
        let mut b = make_box(&Frame::WORLD, size);
        let mut v = vol(&b);
        for k in 0..6 {
            let mut step = random_step(&mut rng, &b, size);
            // `DEBUG_STEP=k [EDGE=n]`: debug output for step k (blending
            // only edge n, if given).
            let debug_step = std::env::var("DEBUG_STEP").ok().and_then(|s| s.parse::<usize>().ok()) == Some(k);
            if debug_step {
                if let (Step::Blend(_, edges, _), Some(n)) = (&mut step, std::env::var("EDGE").ok().and_then(|s| s.parse::<u32>().ok())) {
                    *edges = vec![EdgeId(n)];
                }
                unsafe {
                    std::env::set_var("DUCTILE_DEBUG_BOOL", "1");
                    std::env::set_var("DUCTILE_DEBUG_FILLET", "1");
                }
            }
            if only.is_some() {
                if k == 0 {
                    println!("  block {size:.3}");
                }
                if let Step::Blend(_, edges, _) = &step {
                    for e in edges {
                        let ed = b.edge(*e);
                        let kind = |c: &ductile_geom::Curve3| format!("{c:?}").split([' ', '(', '{']).next().unwrap_or("").to_string();
                        let faces: Vec<String> = b.edge_faces(*e).iter().map(|(f, _)| format!("f{} {}", f.0, format!("{:?}", b.face(*f).surface).split([' ', '(', '{']).next().unwrap_or(""))).collect();
                        println!("    e{} {} {:.3} -> {:.3} faces {:?}", e.0, kind(&ed.curve), ed.start(), ed.end(), faces);
                    }
                }
            }
            let (desc, r, sign) = match step {
                Step::Cut(d, t) => (d, boolean(&b, &t, BoolOp::Difference).map_err(|e| e.to_string()), -1.0),
                Step::Add(d, t) => (d, boolean(&b, &t, BoolOp::Union).map_err(|e| e.to_string()), 1.0),
                Step::Blend(d, e, bl) => {
                    let r = blend_edges(&b, &e, bl).map_err(|e| e.to_string());
                    if only.is_some() && r.is_err() {
                        for x in &e {
                            println!("    alone e{}: {:?}", x.0, blend_edges(&b, &[*x], bl).map(|r| r.faces.len()).map_err(|e| e.to_string()));
                        }
                    }
                    (d, r, 0.0)
                }
            };
            steps_run += 1;
            if only.is_some() {
                println!("  step {k}: {desc}: {}", match &r { Ok(x) => format!("ok, {} faces", x.faces.len()), Err(e) => e.clone() });
            }
            match r {
                // A blend too big for its edges is the user's call, not a bug.
                Err(e) if sign == 0.0 && (e.contains("too large") || e.contains("smooth") || e.contains("seam")) => continue,
                Err(e) => {
                    fails.push(format!("session {s} step {k} ({desc}): {e}"));
                    break;
                }
                Ok(nb) => {
                    let open = open_edges(&nb);
                    let nv = vol(&nb);
                    let wrong = if sign < 0.0 { nv > v + 1e-6 } else if sign > 0.0 { nv < v - 1e-6 } else { (nv - v).abs() > 0.2 * v };
                    if open > 0 || nv <= 0.0 || wrong {
                        fails.push(format!("session {s} step {k} ({desc}): open {open}, volume {v:.4} -> {nv:.4}"));
                        break;
                    }
                    b = nb;
                    v = nv;
                }
            }
        }
    }
    for f in &fails {
        println!("{f}");
    }
    println!("{} of {sessions} sessions failed ({steps_run} steps)", fails.len());
}
