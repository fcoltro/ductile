// SPDX-License-Identifier: GPL-3.0-only
//! Prints what Ductile reads from STEP files: `step_info FILE...`

fn main() {
    for path in std::env::args().skip(1) {
        let text = match std::fs::read_to_string(&path) {
            Ok(t) => t,
            Err(e) => {
                println!("{path}: {e}");
                continue;
            }
        };
        match ductile_io::step_read::read_step(&text) {
            Ok(imp) => {
                println!("{path}: {} bodies", imp.bodies.len());
                for b in &imp.bodies {
                    let mesh = ductile_brep::tessellate(&b.body, &ductile_brep::TessOptions { chord_tol: 1e-3, angle_tol: 0.1 });
                    println!(
                        "  {:20} {:?} faces={} open={} volume={:.4}",
                        b.name,
                        b.body.kind,
                        b.body.faces.len(),
                        ductile_brep::ops::boolean::open_edges(&b.body),
                        mesh.volume()
                    );
                }
                for w in &imp.warnings {
                    println!("  warning: {w}");
                }
            }
            Err(e) => println!("{path}: {e}"),
        }
    }
}
