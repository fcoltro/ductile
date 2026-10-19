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

/// Reads a 3MF package: one mesh per build item (components flattened,
/// transforms applied), named after its object.
pub fn read_3mf(data: &[u8]) -> crate::Result<Vec<(String, TriMesh)>> {
    use std::io::Read;
    let bad = |e: String| crate::IoError::Parse(e);
    let mut zip = zip::ZipArchive::new(std::io::Cursor::new(data)).map_err(|e| bad(format!("not a 3MF package: {e}")))?;
    let mut read = |name: &str| -> Option<String> {
        let mut f = zip.by_name(name.trim_start_matches('/')).ok()?;
        let mut s = String::new();
        f.read_to_string(&mut s).ok()?;
        Some(s)
    };
    // The model part the package's relationships point at.
    let model_path = read("_rels/.rels")
        .and_then(|rels| {
            let doc = roxmltree::Document::parse(&rels).ok()?;
            doc.descendants().find(|n| n.attribute("Type").is_some_and(|t| t.ends_with("/3dmodel"))).and_then(|n| n.attribute("Target")).map(str::to_string)
        })
        .unwrap_or_else(|| "3D/3dmodel.model".into());
    let xml = read(&model_path).ok_or_else(|| bad(format!("the package has no model at {model_path}")))?;
    let doc = roxmltree::Document::parse(&xml).map_err(|e| bad(e.to_string()))?;
    // Row-major 3x4: p' = [x y z 1] * M.
    let transform = |s: Option<&str>| -> glam::DMat4 {
        let v: Vec<f64> = s.unwrap_or("").split_whitespace().filter_map(|x| x.parse().ok()).collect();
        if v.len() != 12 {
            return glam::DMat4::IDENTITY;
        }
        glam::DMat4::from_cols(
            glam::DVec4::new(v[0], v[1], v[2], 0.0),
            glam::DVec4::new(v[3], v[4], v[5], 0.0),
            glam::DVec4::new(v[6], v[7], v[8], 0.0),
            glam::DVec4::new(v[9], v[10], v[11], 1.0),
        )
    };
    struct Obj<'a> {
        name: String,
        mesh: Option<TriMesh>,
        components: Vec<(&'a str, glam::DMat4)>,
    }
    let mut objects: std::collections::HashMap<&str, Obj> = Default::default();
    for o in doc.descendants().filter(|n| n.has_tag_name("object")) {
        let Some(id) = o.attribute("id") else { continue };
        let mesh = o.children().find(|n| n.has_tag_name("mesh")).map(|m| {
            let mut t = TriMesh::default();
            for v in m.descendants().filter(|n| n.has_tag_name("vertex")) {
                let c = |k: &str| v.attribute(k).and_then(|x| x.parse::<f64>().ok()).unwrap_or(0.0);
                t.positions.push(DVec3::new(c("x"), c("y"), c("z")));
            }
            let n = t.positions.len() as u32;
            for tri in m.descendants().filter(|n| n.has_tag_name("triangle")) {
                let i = |k: &str| tri.attribute(k).and_then(|x| x.parse::<u32>().ok()).unwrap_or(u32::MAX);
                let idx = [i("v1"), i("v2"), i("v3")];
                if idx.iter().all(|&k| k < n) {
                    t.triangles.push(idx);
                }
            }
            t
        });
        let components = o.descendants().filter(|n| n.has_tag_name("component")).filter_map(|c| Some((c.attribute("objectid")?, transform(c.attribute("transform"))))).collect();
        objects.insert(id, Obj { name: o.attribute("name").unwrap_or("").to_string(), mesh, components });
    }
    fn flatten(objects: &std::collections::HashMap<&str, Obj>, id: &str, m: glam::DMat4, depth: usize, out: &mut TriMesh) {
        let Some(o) = objects.get(id) else { return };
        if depth > 16 {
            return;
        }
        if let Some(t) = &o.mesh {
            let base = out.positions.len() as u32;
            out.positions.extend(t.positions.iter().map(|p| m.transform_point3(*p)));
            let flip = m.determinant() < 0.0;
            out.triangles.extend(t.triangles.iter().map(|tri| if flip { [base + tri[0], base + tri[2], base + tri[1]] } else { tri.map(|k| base + k) }));
        }
        for (c, cm) in &o.components {
            flatten(objects, c, m * *cm, depth + 1, out);
        }
    }
    let mut out = vec![];
    for item in doc.descendants().filter(|n| n.has_tag_name("item")) {
        let Some(id) = item.attribute("objectid") else { continue };
        let mut t = TriMesh::default();
        flatten(&objects, id, transform(item.attribute("transform")), 0, &mut t);
        if !t.triangles.is_empty() {
            out.push((objects.get(id).map(|o| o.name.clone()).unwrap_or_default(), t));
        }
    }
    if out.is_empty() {
        return Err(bad("no meshes in the 3MF package".into()));
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use ductile_brep::{primitives::make_box, tessellate, TessOptions};
    use ductile_geom::Frame;

    #[test]
    fn reads_a_3mf_with_a_moved_component() {
        use std::io::Write;
        let model = r#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
 <resources>
  <object id="1" type="model" name="Tri"><mesh>
   <vertices><vertex x="0" y="0" z="0"/><vertex x="1" y="0" z="0"/><vertex x="0" y="1" z="0"/></vertices>
   <triangles><triangle v1="0" v2="1" v3="2"/></triangles>
  </mesh></object>
  <object id="2" type="model" name="Pair"><components>
   <component objectid="1"/>
   <component objectid="1" transform="1 0 0 0 1 0 0 0 1 10 0 5"/>
  </components></object>
 </resources>
 <build><item objectid="2" transform="1 0 0 0 1 0 0 0 1 0 0 1"/></build>
</model>"#;
        let mut buf = std::io::Cursor::new(Vec::new());
        {
            let mut z = zip::ZipWriter::new(&mut buf);
            let o = zip::write::SimpleFileOptions::default();
            z.start_file("_rels/.rels", o).unwrap();
            z.write_all(br#"<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>"#).unwrap();
            z.start_file("3D/3dmodel.model", o).unwrap();
            z.write_all(model.as_bytes()).unwrap();
            z.finish().unwrap();
        }
        let m = read_3mf(buf.get_ref()).unwrap();
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].0, "Pair");
        assert_eq!(m[0].1.triangles.len(), 2);
        assert!(m[0].1.positions.contains(&DVec3::new(11.0, 0.0, 6.0)));
        assert!(m[0].1.positions.contains(&DVec3::new(0.0, 1.0, 1.0)));
    }

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
