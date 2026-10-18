// SPDX-License-Identifier: GPL-3.0-only
//! Mesh export (STL, OBJ) from tessellated bodies.

use ductile_brep::BodyMesh;
use std::io::Write;

/// Writes a binary STL containing all given meshes.
pub fn write_stl<W: Write>(w: &mut W, meshes: &[&BodyMesh]) -> crate::Result<()> {
    let mut header = [0u8; 80];
    let tag = b"Ductile binary STL";
    header[..tag.len()].copy_from_slice(tag);
    w.write_all(&header)?;
    let count: usize = meshes.iter().map(|m| m.triangle_count()).sum();
    w.write_all(&(count as u32).to_le_bytes())?;
    for m in meshes {
        for (t, _) in m.triangles() {
            let n = (t[1] - t[0]).cross(t[2] - t[0]).normalize_or_zero();
            for v in [n, t[0], t[1], t[2]] {
                for c in [v.x, v.y, v.z] {
                    w.write_all(&(c as f32).to_le_bytes())?;
                }
            }
            w.write_all(&[0, 0])?;
        }
    }
    Ok(())
}

/// Writes a Wavefront OBJ with one object per mesh, sharing vertices within faces.
pub fn write_obj<W: Write>(w: &mut W, meshes: &[(&str, &BodyMesh)]) -> crate::Result<()> {
    writeln!(w, "# Exported by Ductile")?;
    let mut base = 1usize;
    for (name, m) in meshes {
        writeln!(w, "o {}", name.replace(char::is_whitespace, "_"))?;
        for f in &m.faces {
            for p in &f.positions {
                writeln!(w, "v {} {} {}", p.x, p.y, p.z)?;
            }
            for n in &f.normals {
                writeln!(w, "vn {} {} {}", n.x, n.y, n.z)?;
            }
            for t in &f.triangles {
                let (a, b, c) = (t[0] as usize + base, t[1] as usize + base, t[2] as usize + base);
                writeln!(w, "f {a}//{a} {b}//{b} {c}//{c}")?;
            }
            base += f.positions.len();
        }
    }
    Ok(())
}
