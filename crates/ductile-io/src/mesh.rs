// SPDX-License-Identifier: GPL-3.0-only
//! Mesh export (STL, OBJ) from tessellated bodies, and import of meshes
//! as reference bodies.

use ductile_brep::{BodyMesh, TriMesh};
use glam::DVec3;
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

/// Merges corners closer than a millionth of the mesh's size, so triangles
/// share them (STL repeats every corner).
pub fn weld(soup: &[[DVec3; 3]]) -> TriMesh {
    let (mut lo, mut hi) = (DVec3::splat(f64::MAX), DVec3::splat(f64::MIN));
    for p in soup.iter().flatten() {
        lo = lo.min(*p);
        hi = hi.max(*p);
    }
    let q = ((hi - lo).max_element() * 1e-6).max(1e-12);
    let mut at: std::collections::HashMap<[i64; 3], u32> = Default::default();
    let mut m = TriMesh::default();
    for t in soup {
        let idx = t.map(|p| {
            let k = ((p - lo) / q).round();
            *at.entry([k.x as i64, k.y as i64, k.z as i64]).or_insert_with(|| {
                m.positions.push(p);
                m.positions.len() as u32 - 1
            })
        });
        if idx[0] != idx[1] && idx[1] != idx[2] && idx[0] != idx[2] {
            m.triangles.push(idx);
        }
    }
    m
}

/// Reads a binary or ASCII STL.
pub fn read_stl(data: &[u8]) -> crate::Result<TriMesh> {
    let binary_len = (data.len() >= 84).then(|| 84 + 50 * u32::from_le_bytes([data[80], data[81], data[82], data[83]]) as usize);
    let mut soup = vec![];
    if binary_len == Some(data.len()) || !data.trim_ascii_start().starts_with(b"solid") {
        let n = binary_len.ok_or_else(|| crate::IoError::Parse("STL file too short".into()))?;
        if data.len() < n {
            return Err(crate::IoError::Parse("STL file is cut short".into()));
        }
        let f = |o: usize| f32::from_le_bytes([data[o], data[o + 1], data[o + 2], data[o + 3]]) as f64;
        for i in 0..(n - 84) / 50 {
            let o = 84 + 50 * i + 12;
            soup.push([0, 1, 2].map(|k| DVec3::new(f(o + 12 * k), f(o + 12 * k + 4), f(o + 12 * k + 8))));
        }
    } else {
        let text = String::from_utf8_lossy(data);
        let mut corners = vec![];
        for line in text.lines() {
            let mut w = line.split_whitespace();
            if w.next() == Some("vertex") {
                let v: Vec<f64> = w.filter_map(|x| x.parse().ok()).collect();
                if v.len() < 3 {
                    return Err(crate::IoError::Parse(format!("bad STL vertex: {line}")));
                }
                corners.push(DVec3::new(v[0], v[1], v[2]));
            } else if line.trim() == "endfacet" && corners.len() >= 3 {
                let c = std::mem::take(&mut corners);
                for k in 1..c.len() - 1 {
                    soup.push([c[0], c[k], c[k + 1]]);
                }
            }
        }
    }
    if soup.is_empty() {
        return Err(crate::IoError::Parse("no triangles in the STL file".into()));
    }
    Ok(weld(&soup))
}

/// Reads a Wavefront OBJ: one mesh per object (`o`) or group (`g`), with
/// polygons split into triangles.
pub fn read_obj(text: &str) -> crate::Result<Vec<(String, TriMesh)>> {
    let mut verts: Vec<DVec3> = vec![];
    let mut out: Vec<(String, Vec<[u32; 3]>)> = vec![(String::new(), vec![])];
    for line in text.lines() {
        let mut w = line.split_whitespace();
        match w.next() {
            Some("v") => {
                let v: Vec<f64> = w.take(3).filter_map(|x| x.parse().ok()).collect();
                if v.len() < 3 {
                    return Err(crate::IoError::Parse(format!("bad OBJ vertex: {line}")));
                }
                verts.push(DVec3::new(v[0], v[1], v[2]));
            }
            Some("f") => {
                let idx: Vec<u32> = w
                    .map(|c| {
                        let i: i64 = c.split('/').next().and_then(|x| x.parse().ok()).unwrap_or(0);
                        if i < 0 { verts.len() as i64 + i } else { i - 1 }
                    })
                    .filter(|i| *i >= 0 && (*i as usize) < verts.len())
                    .map(|i| i as u32)
                    .collect();
                let tris = &mut out.last_mut().unwrap().1;
                for k in 1..idx.len().saturating_sub(1) {
                    tris.push([idx[0], idx[k], idx[k + 1]]);
                }
            }
            Some("o") | Some("g") => {
                let name = w.collect::<Vec<_>>().join(" ");
                if out.last().is_some_and(|o| o.1.is_empty()) {
                    out.last_mut().unwrap().0 = name;
                } else {
                    out.push((name, vec![]));
                }
            }
            _ => {}
        }
    }
    let meshes: Vec<(String, TriMesh)> = out
        .into_iter()
        .filter(|o| !o.1.is_empty())
        .map(|(name, tris)| {
            // Each object keeps only the vertices it uses.
            let mut map: std::collections::HashMap<u32, u32> = Default::default();
            let mut m = TriMesh::default();
            for t in tris {
                m.triangles.push(t.map(|i| {
                    *map.entry(i).or_insert_with(|| {
                        m.positions.push(verts[i as usize]);
                        m.positions.len() as u32 - 1
                    })
                }));
            }
            (name, m)
        })
        .collect();
    if meshes.is_empty() {
        return Err(crate::IoError::Parse("no faces in the OBJ file".into()));
    }
    Ok(meshes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use ductile_brep::{primitives::make_box, tessellate, TessOptions};
    use ductile_geom::Frame;

    #[test]
    fn stl_and_obj_read_back_what_was_written() {
        let b = make_box(&Frame::WORLD, DVec3::new(2.0, 3.0, 4.0));
        let m = tessellate(&b, &TessOptions::default());
        let mut stl = vec![];
        write_stl(&mut stl, &[&m]).unwrap();
        let t = read_stl(&stl).unwrap();
        assert_eq!(t.triangles.len(), m.triangle_count());
        assert_eq!(t.positions.len(), 8, "corners are welded");
        let body = ductile_brep::Body::from_mesh(t);
        let back = tessellate(&body, &TessOptions::default());
        assert!((back.volume() - 24.0).abs() < 1e-4, "{}", back.volume());
        // A flat box keeps hard edges: every corner normal is a face normal.
        assert!(back.faces[0].normals.iter().all(|n| n.abs().max_element() > 0.999));
        let mut obj = vec![];
        write_obj(&mut obj, &[("Box", &m)]).unwrap();
        let objs = read_obj(std::str::from_utf8(&obj).unwrap()).unwrap();
        assert_eq!(objs.len(), 1);
        assert_eq!(objs[0].0, "Box");
        assert_eq!(objs[0].1.triangles.len(), m.triangle_count());
        let ascii = "solid t\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid t\n";
        assert_eq!(read_stl(ascii.as_bytes()).unwrap().triangles.len(), 1);
        let moved = ductile_brep::Body::from_mesh(read_stl(ascii.as_bytes()).unwrap()).transformed(&glam::DMat4::from_translation(DVec3::Z));
        assert_eq!(moved.bbox().min.z, 1.0);
    }
}
