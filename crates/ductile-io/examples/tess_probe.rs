// Prints per-face mesh areas of a STEP file's bodies at a coarse and a fine
// tolerance, to find faces that tessellate badly.
use ductile_brep::{tessellate, TessOptions};
fn main() {
    let path = std::env::args().nth(1).unwrap();
    let imp = ductile_io::step_read::read_step(&std::fs::read_to_string(path).unwrap()).unwrap();
    for b in &imp.bodies {
        let b = &b.body;
        let coarse = TessOptions::for_size(b.bbox().diagonal());
        let fine = TessOptions { chord_tol: 1e-4, angle_tol: 0.05 };
        let flipped = ductile_brep::primitives::flip_orientation(b.clone());
        println!("whole coarse {:.3} flipped coarse {:.3} faces {} reversed {} tris {} {} loops {:?}", tessellate(b, &coarse).volume(), tessellate(&flipped, &coarse).volume(), b.faces.len(), b.faces[0].reversed, tessellate(&flipped, &coarse).triangles().count(), tessellate(&flipped, &TessOptions { chord_tol: 1e-4, angle_tol: 0.05 }).triangles().count(), b.face_loops(ductile_brep::FaceId(0)).map(|l| l.coedges.len()).collect::<Vec<_>>());
        for f in b.face_ids() {
            let mut one = b.clone();
            one.faces = vec![b.face(f).clone()];
            let (mc, mf) = (tessellate(&one, &coarse), tessellate(&one, &fine));
            println!("face {} {:?}: coarse area {:.3} vol {:.3} tris {} | fine area {:.3} vol {:.3}", f.idx(), std::mem::discriminant(&b.face(f).surface), mc.area(), mc.volume(), mc.triangles().count(), mf.area(), mf.volume());
        }
    }
}
