// SPDX-License-Identifier: GPL-3.0-only
//! The modeling document: objects, selection and undo history.

use ductile_brep::{tessellate, Body, BodyKind, BodyMesh, EdgeId, FaceId, TessOptions, VertexId};
use std::collections::BTreeSet;
use std::sync::Arc;

pub type ObjectId = u64;
pub type GroupId = u64;

/// An Outliner group: objects and subgroups shown, hidden or locked together.
#[derive(Clone, Debug, PartialEq)]
pub struct Group {
    pub id: GroupId,
    pub name: String,
    pub parent: Option<GroupId>,
    pub visible: bool,
    pub locked: bool,
    /// Expanded in the Outliner.
    pub open: bool,
}

#[derive(Clone)]
pub struct Object {
    pub id: ObjectId,
    pub name: String,
    pub body: Arc<Body>,
    pub mesh: Arc<BodyMesh>,
    pub color: [f32; 3],
    pub visible: bool,
    /// Locked objects show but can't be selected.
    pub locked: bool,
    /// The Outliner group holding it.
    pub group: Option<GroupId>,
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
    groups: Vec<Group>,
    label: String,
}

pub struct Document {
    pub objects: Vec<Object>,
    pub groups: Vec<Group>,
    /// The active collection: new objects go into it.
    pub active_group: Option<GroupId>,
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
            groups: vec![],
            active_group: None,
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
        self.undo.push(Snapshot { objects: self.objects.clone(), groups: self.groups.clone(), label: label.to_string() });
        if self.undo.len() > 200 {
            self.undo.remove(0);
        }
        self.redo.clear();
        self.dirty = true;
    }

    pub fn undo(&mut self) -> Option<String> {
        let s = self.undo.pop()?;
        let groups = std::mem::replace(&mut self.groups, s.groups);
        self.redo.push(Snapshot { objects: std::mem::replace(&mut self.objects, s.objects), groups, label: s.label.clone() });
        self.validate_selection();
        Some(s.label)
    }

    pub fn redo(&mut self) -> Option<String> {
        let s = self.redo.pop()?;
        let groups = std::mem::replace(&mut self.groups, s.groups);
        self.undo.push(Snapshot { objects: std::mem::replace(&mut self.objects, s.objects), groups, label: s.label.clone() });
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
        if self.active_group.is_some_and(|g| self.group(g).is_none()) {
            self.active_group = None;
        }
    }

    /// Adds a body as a new object (without a checkpoint).
    pub fn add_body(&mut self, name: &str, body: Body) -> ObjectId {
        let id = self.next_id;
        self.next_id += 1;
        let mesh = Arc::new(mesh_body(&body));
        let rev = self.bump();
        let n = self.objects.iter().filter(|o| o.name.starts_with(name)).count();
        let name = if n == 0 { name.to_string() } else { format!("{name}.{n:03}") };
        self.objects.push(Object { id, name, body: Arc::new(body), mesh, color: DEFAULT_COLOR, visible: true, locked: false, group: self.active_group, revision: rev });
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
        v.sort();
        v.dedup();
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

    /// A fresh id for a group (shared with objects', so never clashing).
    pub fn new_group_id(&mut self) -> GroupId {
        self.next_id += 1;
        self.next_id - 1
    }

    pub fn group(&self, id: GroupId) -> Option<&Group> {
        self.groups.iter().find(|g| g.id == id)
    }

    /// Group `g` and the groups it sits in, innermost first.
    pub fn ancestry(&self, g: Option<GroupId>) -> Vec<&Group> {
        let mut out: Vec<&Group> = vec![];
        let mut cur = g.and_then(|g| self.group(g));
        while let Some(x) = cur {
            if out.iter().any(|y| y.id == x.id) {
                break;
            }
            out.push(x);
            cur = x.parent.and_then(|p| self.group(p));
        }
        out
    }

    /// Whether object `id` can't be selected: locked itself or by a group.
    pub fn is_locked(&self, id: ObjectId) -> bool {
        self.object(id).is_none_or(|o| o.locked || self.ancestry(o.group).iter().any(|g| g.locked))
    }

    /// Whether group `g` sits inside group `outer` (or is it).
    pub fn within(&self, g: Option<GroupId>, outer: GroupId) -> bool {
        self.ancestry(g).iter().any(|x| x.id == outer)
    }

    /// Objects in group `g`, at any depth.
    pub fn group_members(&self, g: GroupId) -> Vec<ObjectId> {
        self.objects.iter().filter(|o| self.within(o.group, g)).map(|o| o.id).collect()
    }

    /// A new group holding the selected objects, inside the group they all
    /// share (or the active one). Returns None when nothing is selected.
    pub fn group_selected(&mut self) -> Option<GroupId> {
        let ids = self.selected_objects();
        if ids.is_empty() {
            return None;
        }
        let first = self.object(ids[0])?.group;
        let parent = if ids.iter().all(|i| self.object(*i).map(|o| o.group) == Some(first)) { first } else { self.active_group };
        let id = self.new_group_id();
        let n = self.groups.len() + 1;
        self.groups.push(Group { id, name: format!("Group {n}"), parent, visible: true, locked: false, open: true });
        for o in self.objects.iter_mut().filter(|o| ids.contains(&o.id)) {
            o.group = Some(id);
        }
        self.dirty = true;
        Some(id)
    }

    /// Dissolves group `g`, handing its objects and subgroups to its parent.
    pub fn ungroup(&mut self, g: GroupId) {
        let Some(parent) = self.group(g).map(|x| x.parent) else { return };
        let hidden = self.group(g).is_some_and(|x| !x.visible);
        for o in self.objects.iter_mut().filter(|o| o.group == Some(g)) {
            o.group = parent;
            o.visible &= !hidden;
        }
        for x in self.groups.iter_mut().filter(|x| x.parent == Some(g)) {
            x.parent = parent;
            x.visible &= !hidden;
        }
        self.groups.retain(|x| x.id != g);
        if self.active_group == Some(g) {
            self.active_group = parent;
        }
        self.dirty = true;
    }

    /// Shows or hides group `g` with everything in it.
    pub fn set_group_visible(&mut self, g: GroupId, visible: bool) {
        let inner: Vec<GroupId> = self.groups.iter().filter(|x| self.within(Some(x.id), g)).map(|x| x.id).collect();
        for x in self.groups.iter_mut().filter(|x| inner.contains(&x.id)) {
            x.visible = visible;
        }
        for o in self.objects.iter_mut().filter(|o| o.group.is_some_and(|og| inner.contains(&og))) {
            o.visible = visible;
        }
        if !visible {
            let locked: BTreeSet<ObjectId> = self.group_members(g).into_iter().collect();
            self.selection.retain(|p| !locked.contains(&p.object()));
        }
    }

    pub fn bbox(&self) -> ductile_geom::BBox3 {
        let mut b = ductile_geom::BBox3::EMPTY;
        for o in self.objects.iter().filter(|o| o.visible) {
            b = b.union(&o.body.bbox());
        }
        b
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ductile_brep::primitives::make_box;
    use ductile_geom::{DVec3, Frame};

    #[test]
    fn groups_nest_hide_lock_and_dissolve() {
        let mut doc = Document::default();
        let ids: Vec<ObjectId> = (0..3).map(|i| doc.add_body("Box", make_box(&Frame { origin: DVec3::X * i as f64 * 2.0, ..Frame::WORLD }, DVec3::ONE))).collect();
        doc.selection.extend([Pick::Object(ids[0]), Pick::Object(ids[1])]);
        let outer = doc.group_selected().unwrap();
        doc.selection.clear();
        doc.selection.insert(Pick::Object(ids[0]));
        let inner = doc.group_selected().unwrap();
        assert_eq!(doc.group(inner).unwrap().parent, Some(outer));
        assert_eq!(doc.group_members(outer), vec![ids[0], ids[1]]);
        // Locking the outer group locks what is inside the inner one.
        doc.groups.iter_mut().find(|g| g.id == outer).unwrap().locked = true;
        assert!(doc.is_locked(ids[0]) && !doc.is_locked(ids[2]));
        doc.groups.iter_mut().find(|g| g.id == outer).unwrap().locked = false;
        doc.set_group_visible(outer, false);
        assert!(!doc.object(ids[0]).unwrap().visible && doc.object(ids[2]).unwrap().visible);
        // New objects go into the active group.
        doc.active_group = Some(inner);
        let made = doc.add_body("Box", make_box(&Frame::WORLD, DVec3::ONE));
        assert_eq!(doc.object(made).unwrap().group, Some(inner));
        // Dissolving the outer group hands its children to the root, shown.
        doc.ungroup(outer);
        assert_eq!(doc.group(inner).unwrap().parent, None);
        assert_eq!(doc.object(ids[1]).unwrap().group, None);
        assert!(!doc.object(ids[1]).unwrap().visible, "hidden by its group, it stays hidden once ungrouped");
    }
}
