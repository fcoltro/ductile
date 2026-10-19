// SPDX-License-Identifier: GPL-3.0-only
//! Boundary representation topology.
//!
//! A [`Body`] owns flat arenas of vertices, edges, loops and faces. Entities
//! refer to each other by index. Bodies are treated as immutable values by
//! the modeling operations: every operation builds a new body, which makes
//! undo/redo and history trivial.
//!
//! Orientation conventions:
//! * An edge runs from `v0` at parameter `t0` to `v1` at `t1` (`t0 < t1`).
//! * A face's outward normal is the surface normal, negated if `reversed`.
//! * Loops are ordered so that, looking against the outward normal, the face
//!   interior lies to the left of every coedge (outer loops run
//!   counter-clockwise, holes clockwise).

use ductile_geom::{BBox3, Curve3, DMat4, DVec3, Surface};
use serde::{Deserialize, Serialize};

macro_rules! id_type {
    ($name:ident) => {
        #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
        pub struct $name(pub u32);
        impl $name {
            pub fn idx(self) -> usize {
                self.0 as usize
            }
        }
    };
}

id_type!(VertexId);
id_type!(EdgeId);
id_type!(FaceId);
id_type!(LoopId);

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Vertex {
    pub point: DVec3,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Edge {
    pub curve: Curve3,
    pub t0: f64,
    pub t1: f64,
    pub v0: VertexId,
    pub v1: VertexId,
}

impl Edge {
    pub fn start(&self) -> DVec3 {
        self.curve.point(self.t0)
    }
    pub fn end(&self) -> DVec3 {
        self.curve.point(self.t1)
    }
    pub fn mid(&self) -> DVec3 {
        self.curve.point(0.5 * (self.t0 + self.t1))
    }
    pub fn is_closed(&self) -> bool {
        self.v0 == self.v1
    }
    pub fn length(&self) -> f64 {
        self.curve.length(self.t0, self.t1)
    }
}

/// A use of an edge inside a loop.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Coedge {
    pub edge: EdgeId,
    /// True if the loop traverses the edge from `v1` to `v0`.
    pub reversed: bool,
}

impl Coedge {
    pub fn new(edge: EdgeId, reversed: bool) -> Self {
        Self { edge, reversed }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Loop {
    pub coedges: Vec<Coedge>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Face {
    pub surface: Surface,
    pub reversed: bool,
    /// First loop is the outer boundary, the rest are holes. A face with no
    /// loops is a closed surface (e.g. an untrimmed torus).
    pub loops: Vec<LoopId>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum BodyKind {
    /// Closed, oriented shell(s) enclosing volume.
    Solid,
    /// Open surface(s).
    Sheet,
    /// Edges only (curves).
    Wire,
    /// A triangle mesh carried as it is (imported OBJ or STL), for reference.
    Mesh,
}

/// Triangles with shared corners, for a [`BodyKind::Mesh`] body.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct TriMesh {
    pub positions: Vec<DVec3>,
    pub triangles: Vec<[u32; 3]>,
    /// The polygons the triangles were cut from, when the file had them
    /// (OBJ quads): a subdivision cage for PolySplines.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub polygons: Vec<Vec<u32>>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Body {
    pub kind: BodyKind,
    pub vertices: Vec<Vertex>,
    pub edges: Vec<Edge>,
    pub loops: Vec<Loop>,
    pub faces: Vec<Face>,
    /// The triangles of a mesh body.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mesh: Option<TriMesh>,
}

impl Body {
    pub fn new(kind: BodyKind) -> Self {
        Self { kind, vertices: vec![], edges: vec![], loops: vec![], faces: vec![], mesh: None }
    }

    /// A mesh body holding `mesh`.
    pub fn from_mesh(mesh: TriMesh) -> Self {
        Self { mesh: Some(mesh), ..Self::new(BodyKind::Mesh) }
    }

    pub fn add_vertex(&mut self, p: DVec3) -> VertexId {
        self.vertices.push(Vertex { point: p });
        VertexId(self.vertices.len() as u32 - 1)
    }

    /// Adds an edge along `curve` on `[t0, t1]` between existing vertices.
    pub fn add_edge(&mut self, curve: Curve3, t0: f64, t1: f64, v0: VertexId, v1: VertexId) -> EdgeId {
        debug_assert!(t1 > t0, "edge parameter range must increase");
        self.edges.push(Edge { curve, t0, t1, v0, v1 });
        EdgeId(self.edges.len() as u32 - 1)
    }

    /// Straight edge between two vertices.
    pub fn add_line_edge(&mut self, v0: VertexId, v1: VertexId) -> EdgeId {
        let a = self.vertex(v0);
        let b = self.vertex(v1);
        let len = a.distance(b);
        self.add_edge(Curve3::line_through(a, b), 0.0, len, v0, v1)
    }

    pub fn add_loop(&mut self, coedges: Vec<Coedge>) -> LoopId {
        self.loops.push(Loop { coedges });
        LoopId(self.loops.len() as u32 - 1)
    }

    pub fn add_face(&mut self, surface: Surface, reversed: bool, loops: Vec<Vec<Coedge>>) -> FaceId {
        let loops = loops.into_iter().map(|l| self.add_loop(l)).collect();
        self.faces.push(Face { surface, reversed, loops });
        FaceId(self.faces.len() as u32 - 1)
    }

    pub fn vertex(&self, v: VertexId) -> DVec3 {
        self.vertices[v.idx()].point
    }

    pub fn edge(&self, e: EdgeId) -> &Edge {
        &self.edges[e.idx()]
    }

    pub fn face(&self, f: FaceId) -> &Face {
        &self.faces[f.idx()]
    }

    pub fn face_loops(&self, f: FaceId) -> impl Iterator<Item = &Loop> {
        self.faces[f.idx()].loops.iter().map(move |l| &self.loops[l.idx()])
    }

    pub fn face_ids(&self) -> impl Iterator<Item = FaceId> {
        (0..self.faces.len() as u32).map(FaceId)
    }

    pub fn edge_ids(&self) -> impl Iterator<Item = EdgeId> {
        (0..self.edges.len() as u32).map(EdgeId)
    }

    /// Start and end vertex of a coedge in loop direction.
    pub fn coedge_vertices(&self, c: Coedge) -> (VertexId, VertexId) {
        let e = self.edge(c.edge);
        if c.reversed {
            (e.v1, e.v0)
        } else {
            (e.v0, e.v1)
        }
    }

    /// Outward normal of face `f` at surface parameter (u, v).
    pub fn face_normal(&self, f: FaceId, u: f64, v: f64) -> DVec3 {
        let face = self.face(f);
        let n = face.surface.normal(u, v);
        if face.reversed {
            -n
        } else {
            n
        }
    }

    /// Faces using edge `e` (one for boundary edges of sheets, two for manifold edges).
    pub fn edge_faces(&self, e: EdgeId) -> Vec<(FaceId, Coedge)> {
        let mut out = Vec::new();
        for (fi, f) in self.faces.iter().enumerate() {
            for l in &f.loops {
                for c in &self.loops[l.idx()].coedges {
                    if c.edge == e {
                        out.push((FaceId(fi as u32), *c));
                    }
                }
            }
        }
        out
    }

    /// Edges bounding face `f`.
    pub fn face_edges(&self, f: FaceId) -> Vec<EdgeId> {
        let mut out: Vec<EdgeId> =
            self.face_loops(f).flat_map(|l| l.coedges.iter().map(|c| c.edge)).collect();
        out.sort();
        out.dedup();
        out
    }

    pub fn bbox(&self) -> BBox3 {
        let mut b = BBox3::from_points(self.vertices.iter().map(|v| v.point).chain(self.mesh.iter().flat_map(|m| m.positions.iter().copied())));
        for e in &self.edges {
            b = b.union(&e.curve.bbox(e.t0, e.t1));
        }
        b
    }

    /// Applies an affine transform (rigid or uniform scale) to the body.
    pub fn transformed(&self, m: &DMat4) -> Body {
        let mut out = self.clone();
        for v in out.vertices.iter_mut() {
            v.point = m.transform_point3(v.point);
        }
        let mirror = m.determinant() < 0.0;
        for e in out.edges.iter_mut() {
            let scale = m.transform_vector3(DVec3::X).length();
            let new_curve = e.curve.transformed(m);
            // Lines are arc-length parameterized: rescale parameters.
            if let Curve3::Line { .. } = e.curve {
                e.t0 *= scale;
                e.t1 *= scale;
            }
            e.curve = new_curve;
        }
        for f in out.faces.iter_mut() {
            f.surface = f.surface.transformed(m);
        }
        if let Some(t) = out.mesh.as_mut() {
            for p in t.positions.iter_mut() {
                *p = m.transform_point3(*p);
            }
            if mirror {
                for tri in t.triangles.iter_mut() {
                    tri.swap(1, 2);
                }
            }
        }
        if mirror {
            // Mirroring flips handedness. Analytic surfaces compute their
            // normal from the (re-orthonormalized) frame, so it stays
            // outward; NURBS normals come from Su × Sv and flip. Loops
            // always need reversing to stay counter-clockwise.
            for f in out.faces.iter_mut() {
                if matches!(f.surface, Surface::Nurbs(_)) {
                    f.reversed = !f.reversed;
                }
            }
            for l in out.loops.iter_mut() {
                l.coedges.reverse();
                for c in l.coedges.iter_mut() {
                    c.reversed = !c.reversed;
                }
            }
        }
        out
    }

    /// Appends all entities of `other` into `self` (no merging).
    pub fn append(&mut self, other: &Body) {
        let vo = self.vertices.len() as u32;
        let eo = self.edges.len() as u32;
        let lo = self.loops.len() as u32;
        self.vertices.extend(other.vertices.iter().cloned());
        for e in &other.edges {
            let mut e = e.clone();
            e.v0 = VertexId(e.v0.0 + vo);
            e.v1 = VertexId(e.v1.0 + vo);
            self.edges.push(e);
        }
        for l in &other.loops {
            let coedges = l.coedges.iter().map(|c| Coedge { edge: EdgeId(c.edge.0 + eo), reversed: c.reversed }).collect();
            self.loops.push(Loop { coedges });
        }
        for f in &other.faces {
            let mut f = f.clone();
            for l in f.loops.iter_mut() {
                *l = LoopId(l.0 + lo);
            }
            self.faces.push(f);
        }
    }
}
