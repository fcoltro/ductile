// SPDX-License-Identifier: GPL-3.0-only
//! The modeling document: objects, selection and undo history.

use ductile_brep::{tessellate, Body, BodyKind, BodyMesh, EdgeId, FaceId, TessOptions, VertexId};
use std::collections::BTreeSet;
use std::sync::Arc;

pub type ObjectId = u64;

#[derive(Clone)]
pub struct Object {
    pub id: ObjectId,
    pub name: String,
    pub body: Arc<Body>,
    pub mesh: Arc<BodyMesh>,
    pub color: [f32; 3],
    pub visible: bool,
    /// Bumped whenever geometry changes so the renderer re-uploads.
    pub revision: u64,
}

/// Something the user can select or hover.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Pick {
    Object(ObjectId),
    Face(ObjectId, FaceId),
    Edge(ObjectId, EdgeId),
    Vertex(ObjectId, VertexId),
    /// Control point `index` of a curve edge.
    Control(ObjectId, EdgeId, u32),
    /// Control point `index` of a surface face's net.
    SurfaceControl(ObjectId, FaceId, u32),
}

impl Pick {
    pub fn object(&self) -> ObjectId {
        match *self {
            Pick::Object(o) | Pick::Face(o, _) | Pick::Edge(o, _) | Pick::Vertex(o, _) | Pick::Control(o, _, _) | Pick::SurfaceControl(o, _, _) => o,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SelectMode {
    Vertex,
    Edge,
    Face,
    Object,
}

#[derive(Clone)]
struct Snapshot {
    objects: Vec<Object>,
    label: String,
}

pub struct Document {
    pub objects: Vec<Object>,
    pub selection: BTreeSet<Pick>,
    pub hover: Option<Pick>,
    pub mode: SelectMode,
    next_id: ObjectId,
    revision: u64,
    undo: Vec<Snapshot>,
    redo: Vec<Snapshot>,
    pub dirty: bool,
    pub path: Option<std::path::PathBuf>,
}

pub const DEFAULT_COLOR: [f32; 3] = [0.62, 0.63, 0.65];

impl Default for Document {
    fn default() -> Self {
        Self {
            objects: vec![],
            selection: BTreeSet::new(),
            hover: None,
            mode: SelectMode::Object,
            next_id: 1,
            revision: 1,
            undo: vec![],
            redo: vec![],
            dirty: false,
            path: None,
        }
    }
}

pub fn mesh_body(body: &Body) -> BodyMesh {
    let diag = body.bbox().diagonal().max(1e-3);
    tessellate(body, &TessOptions::for_size(diag.min(1e4)))
}

impl Document {
    pub fn object(&self, id: ObjectId) -> Option<&Object> {
        self.objects.iter().find(|o| o.id == id)
    }

    fn bump(&mut self) -> u64 {
        self.revision += 1;
        self.revision
    }

    /// Records the current state for undo. Call before mutating.
    pub fn checkpoint(&mut self, label: &str) {
        self.undo.push(Snapshot { objects: self.objects.clone(), label: label.to_string() });
        if self.undo.len() > 200 {
            self.undo.remove(0);
        }
        self.redo.clear();
        self.dirty = true;
    }

    pub fn undo(&mut self) -> Option<String> {
        let s = self.undo.pop()?;
        self.redo.push(Snapshot { objects: std::mem::replace(&mut self.objects, s.objects), label: s.label.clone() });
        self.validate_selection();
        Some(s.label)
    }

    pub fn redo(&mut self) -> Option<String> {
        let s = self.redo.pop()?;
        self.undo.push(Snapshot { objects: std::mem::replace(&mut self.objects, s.objects), label: s.label.clone() });
        self.validate_selection();
        Some(s.label)
    }

    pub fn can_undo(&self) -> bool {
        !self.undo.is_empty()
    }
    pub fn can_redo(&self) -> bool {
        !self.redo.is_empty()
    }

    fn validate_selection(&mut self) {
        let ids: BTreeSet<ObjectId> = self.objects.iter().map(|o| o.id).collect();
        self.selection.retain(|p| ids.contains(&p.object()));
        self.hover = None;
    }

    /// Adds a body as a new object (without a checkpoint).
    pub fn add_body(&mut self, name: &str, body: Body) -> ObjectId {
        let id = self.next_id;
        self.next_id += 1;
        let mesh = Arc::new(mesh_body(&body));
        let rev = self.bump();
        let n = self.objects.iter().filter(|o| o.name.starts_with(name)).count();
        let name = if n == 0 { name.to_string() } else { format!("{name}.{n:03}") };
        self.objects.push(Object { id, name, body: Arc::new(body), mesh, color: DEFAULT_COLOR, visible: true, revision: rev });
        self.dirty = true;
        id
    }

    pub fn replace_body(&mut self, id: ObjectId, body: Body) {
        let rev = self.bump();
        if let Some(o) = self.objects.iter_mut().find(|o| o.id == id) {
            o.mesh = Arc::new(mesh_body(&body));
            o.body = Arc::new(body);
            o.revision = rev;
        }
        // Sub-object picks may no longer be valid.
        self.selection.retain(|p| !matches!(p, Pick::Face(o, _) | Pick::Edge(o, _) | Pick::Vertex(o, _) | Pick::Control(o, _, _) | Pick::SurfaceControl(o, _, _) if *o == id));
        self.dirty = true;
    }

    pub fn remove(&mut self, id: ObjectId) {
        self.objects.retain(|o| o.id != id);
        self.selection.retain(|p| p.object() != id);
        if self.hover.map(|h| h.object()) == Some(id) {
            self.hover = None;
        }
        self.dirty = true;
    }

    pub fn selected_objects(&self) -> Vec<ObjectId> {
        let mut v: Vec<ObjectId> = self.selection.iter().map(|p| p.object()).collect();
        v.dedup();
        v.sort();
        v.dedup();
        v
    }

    pub fn selected_solids(&self) -> Vec<ObjectId> {
        self.selection
            .iter()
            .filter_map(|p| match p {
                Pick::Object(o) => Some(*o),
                _ => None,
            })
            .filter(|o| self.object(*o).map(|x| x.body.kind != BodyKind::Wire).unwrap_or(false))
            .collect()
    }

    pub fn selected_faces(&self) -> Vec<(ObjectId, FaceId)> {
        self.selection.iter().filter_map(|p| if let Pick::Face(o, f) = p { Some((*o, *f)) } else { None }).collect()
    }

    pub fn selected_edges(&self) -> Vec<(ObjectId, EdgeId)> {
        self.selection.iter().filter_map(|p| if let Pick::Edge(o, e) = p { Some((*o, *e)) } else { None }).collect()
    }

    pub fn clear(&mut self) {
        *self = Document::default();
    }

    pub fn bbox(&self) -> ductile_geom::BBox3 {
        let mut b = ductile_geom::BBox3::EMPTY;
        for o in self.objects.iter().filter(|o| o.visible) {
            b = b.union(&o.body.bbox());
        }
        b
    }
}
