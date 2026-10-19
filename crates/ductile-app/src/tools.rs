// SPDX-License-Identifier: GPL-3.0-only
//! Interactive modeling commands.
//!
//! Each command is a small state machine fed with viewport events. Commands
//! show a live preview, accept typed values in the command panel, and commit
//! a single undoable change to the document when finished.

use crate::background::Background;
use crate::camera::Ray;
use crate::document::{Document, ObjectId, Pick, SelectMode};
use crate::render::LineInstance;
use ductile_brep::primitives::{make_box, make_frustum, make_sphere_in, make_torus};
use ductile_brep::{Body, EdgeId, FaceId};
use ductile_geom::{DMat4, DQuat, DVec3, Frame};

/// Construction plane on which new geometry is placed.
#[derive(Clone, Copy, Debug)]
pub struct CPlane {
    pub frame: Frame,
}

impl Default for CPlane {
    fn default() -> Self {
        Self { frame: Frame::WORLD }
    }
}

/// What the viewport tells a tool each frame.
#[derive(Clone, Copy, Debug)]
pub struct PointerInfo {
    pub ray: Ray,
    /// Snapped point on the construction plane or on geometry.
    pub point: Option<DVec3>,
    /// Point on the construction plane under the cursor (unsnapped).
    pub plane_point: Option<DVec3>,
    pub clicked: bool,
    pub world_per_px: f64,
}

pub enum ToolOutcome {
    Continue,
    Finished,
    Cancelled,
}

pub trait Tool {
    fn name(&self) -> &'static str;
    /// One-line instruction for the status bar.
    fn hint(&self) -> String;
    fn pointer(&mut self, doc: &Document, cplane: &CPlane, p: &PointerInfo) -> ToolOutcome;
    /// Body to show as a live preview.
    fn preview(&self, doc: &Document) -> Option<Body>;
    /// Extra guide lines.
    fn guides(&self, _out: &mut Vec<LineInstance>) {}
    /// Typed parameters panel; return true if values changed.
    fn panel(&mut self, ui: &mut egui::Ui) -> bool;
    /// Apply the result to the document (already checkpointed by the caller).
    fn commit(&mut self, doc: &mut Document) -> bool;
    /// Called when Enter is pressed.
    fn accept(&mut self) -> ToolOutcome {
        ToolOutcome::Finished
    }
    fn wants_selection(&self) -> Option<SelectMode> {
        None
    }
    /// Objects hidden while the preview stands in for them.
    fn hidden_objects(&self) -> Vec<ObjectId> {
        vec![]
    }
    /// Typed digits from the keyboard (modal value entry).
    fn typed_value(&mut self, _v: Option<f64>) {}
    /// Axis constraint keys.
    fn constrain_axis(&mut self, _axis: usize) {}
    /// The command's own keys (Plasticity's in-command keys); true when
    /// the key was used.
    fn command_key(&mut self, _key: egui::Key, _m: egui::Modifiers) -> bool {
        false
    }
    /// Whether a preview is still computing in the background.
    fn busy(&self) -> bool {
        false
    }
    /// The last point placed, which snapping infers lines, perpendiculars
    /// and tangents from.
    fn anchor(&self) -> Option<DVec3> {
        None
    }
}

fn drag_value(ui: &mut egui::Ui, label: &str, v: &mut f64, speed: f64) -> bool {
    ui.horizontal(|ui| {
        ui.label(label);
        ui.add(egui::DragValue::new(v).speed(speed).max_decimals(4)).changed()
    })
    .inner
}

// ------------------------------------------------------------------ Box ----

pub struct BoxTool {
    corner: Option<DVec3>,
    opposite: Option<DVec3>,
    height: f64,
    stage: u8,
    /// Typed sizes override the cursor when set.
    size: DVec3,
}

impl BoxTool {
    pub fn new() -> Self {
        Self { corner: None, opposite: None, height: 0.0, stage: 0, size: DVec3::ZERO }
    }

    fn frame_and_size(&self, cplane: &CPlane) -> Option<(Frame, DVec3)> {
        let c = self.corner?;
        let o = self.opposite.unwrap_or(c);
        let f = Frame { origin: c, ..cplane.frame };
        let d = f.dir_to_local(o - c);
        Some((f, DVec3::new(d.x, d.y, self.height)))
    }
}

thread_local! {
    static CPLANE: std::cell::Cell<CPlane> = std::cell::Cell::new(CPlane::default());
}

/// The construction plane in effect for the current frame (set by the app).
pub fn set_current_cplane(c: CPlane) {
    CPLANE.with(|x| x.set(c));
}
pub fn current_cplane() -> CPlane {
    CPLANE.with(|x| x.get())
}

fn height_from_ray(base: DVec3, axis: DVec3, ray: &Ray) -> f64 {
    // Closest point between the axis line and the view ray.
    let w = base - ray.origin;
    let a = axis.dot(axis);
    let b = axis.dot(ray.dir);
    let c = ray.dir.dot(ray.dir);
    let d = axis.dot(w);
    let e = ray.dir.dot(w);
    let den = a * c - b * b;
    if den.abs() < 1e-12 {
        return 0.0;
    }
    (b * e - c * d) / den
}

impl Tool for BoxTool {
    fn anchor(&self) -> Option<DVec3> {
        self.corner
    }
    fn name(&self) -> &'static str {
        "Box"
    }
    fn hint(&self) -> String {
        match self.stage {
            0 => "Box: click the first corner".into(),
            1 => "Box: click the opposite corner".into(),
            _ => "Box: move to set the height, click to finish".into(),
        }
    }
    fn pointer(&mut self, _doc: &Document, cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        match self.stage {
            0 => {
                if p.clicked {
                    if let Some(pt) = p.point {
                        self.corner = Some(pt);
                        self.stage = 1;
                    }
                }
            }
            1 => {
                if let Some(c) = self.corner {
                    if let Some(pt) = p.ray.hit_plane(c, cplane.frame.z) {
                        let pt = p.point.filter(|s| cplane.frame.dir_to_local(*s - c).z.abs() < 1e-9).unwrap_or(pt);
                        self.opposite = Some(pt);
                        let d = cplane.frame.dir_to_local(pt - c);
                        self.size.x = d.x;
                        self.size.y = d.y;
                    }
                }
                if p.clicked {
                    self.stage = 2;
                }
            }
            _ => {
                if let (Some(c), Some(o)) = (self.corner, self.opposite) {
                    let base = (c + o) * 0.5;
                    let h = height_from_ray(base, cplane.frame.z, &p.ray);
                    self.height = snap_len(h, p.world_per_px);
                    self.size.z = self.height;
                }
                if p.clicked {
                    return ToolOutcome::Finished;
                }
            }
        }
        ToolOutcome::Continue
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        let (f, s) = self.frame_and_size(&current_cplane())?;
        if s.x.abs() < 1e-9 || s.y.abs() < 1e-9 {
            return None;
        }
        let s = if s.z.abs() < 1e-9 { DVec3::new(s.x, s.y, 1e-3 * s.x.abs().max(s.y.abs())) } else { s };
        Some(make_box(&f, s))
    }
    fn guides(&self, out: &mut Vec<LineInstance>) {
        if let Some(c) = self.corner {
            out.push(LineInstance::point(c, [1.0, 0.85, 0.3, 1.0], 7.0));
        }
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut ch = false;
        ch |= drag_value(ui, "Width", &mut self.size.x, 0.1);
        ch |= drag_value(ui, "Length", &mut self.size.y, 0.1);
        ch |= drag_value(ui, "Height", &mut self.size.z, 0.1);
        if ch {
            if let Some(c) = self.corner {
                let f = current_cplane().frame;
                self.opposite = Some(c + f.x * self.size.x + f.y * self.size.y);
                self.height = self.size.z;
                if self.stage < 2 {
                    self.stage = 2;
                }
            }
        }
        ch
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let Some(b) = self.preview(doc) else { return false };
        if self.height.abs() < 1e-9 {
            return false;
        }
        let id = doc.add_body("Box", b);
        doc.selection.clear();
        doc.selection.insert(Pick::Object(id));
        true
    }
}

fn snap_len(v: f64, wpp: f64) -> f64 {
    // Round to a "nice" increment relative to the pixel size so values read well.
    let step = nice_step(wpp * 4.0);
    (v / step).round() * step
}

pub fn nice_step(x: f64) -> f64 {
    let e = 10f64.powf(x.max(1e-9).log10().floor());
    let m = x / e;
    let n = if m <= 1.0 {
        1.0
    } else if m <= 2.0 {
        2.0
    } else if m <= 5.0 {
        5.0
    } else {
        10.0
    };
    n * e
}

// ------------------------------------------------------ Cylinder / Cone ----

#[derive(Clone, Copy, PartialEq)]
pub enum RoundKind {
    Cylinder,
    Cone,
    Sphere,
    Torus,
    /// A helix curve round the axis, `turns` turns up the height.
    Helix,
}

pub struct RoundTool {
    kind: RoundKind,
    center: Option<DVec3>,
    radius: f64,
    radius2: f64,
    height: f64,
    turns: f64,
    stage: u8,
}

impl RoundTool {
    pub fn new(kind: RoundKind) -> Self {
        Self { kind, center: None, radius: 0.0, radius2: 0.0, height: 0.0, turns: 5.0, stage: 0 }
    }
    fn stages(&self) -> u8 {
        match self.kind {
            RoundKind::Sphere => 2,
            _ => 3,
        }
    }
}

impl Tool for RoundTool {
    fn name(&self) -> &'static str {
        match self.kind {
            RoundKind::Cylinder => "Cylinder",
            RoundKind::Cone => "Cone",
            RoundKind::Sphere => "Sphere",
            RoundKind::Torus => "Torus",
            RoundKind::Helix => "Helix",
        }
    }
    fn hint(&self) -> String {
        let n = self.name();
        match (self.stage, self.kind) {
            (0, _) => format!("{n}: click the center"),
            (1, _) => format!("{n}: click to set the radius"),
            (_, RoundKind::Torus) => format!("{n}: move to set the tube radius, click to finish"),
            _ => format!("{n}: move to set the height, click to finish"),
        }
    }
    fn pointer(&mut self, _doc: &Document, cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        match self.stage {
            0 => {
                if p.clicked {
                    if let Some(pt) = p.point {
                        self.center = Some(pt);
                        self.stage = 1;
                    }
                }
            }
            1 => {
                if let Some(c) = self.center {
                    if let Some(pt) = p.ray.hit_plane(c, cplane.frame.z) {
                        let pt = p.point.filter(|s| cplane.frame.dir_to_local(*s - c).z.abs() < 1e-9).unwrap_or(pt);
                        self.radius = snap_len(pt.distance(c), p.world_per_px);
                    }
                }
                if p.clicked && self.radius > 1e-9 {
                    self.stage += 1;
                    if self.stage >= self.stages() {
                        return ToolOutcome::Finished;
                    }
                }
            }
            _ => {
                if let Some(c) = self.center {
                    match self.kind {
                        RoundKind::Torus => {
                            if let Some(pt) = p.ray.hit_plane(c, cplane.frame.z) {
                                let d = pt.distance(c);
                                self.radius2 = snap_len((d - self.radius).abs(), p.world_per_px).min(self.radius * 0.99);
                            }
                        }
                        _ => {
                            let h = height_from_ray(c, cplane.frame.z, &p.ray);
                            self.height = snap_len(h, p.world_per_px);
                        }
                    }
                }
                if p.clicked {
                    return ToolOutcome::Finished;
                }
            }
        }
        ToolOutcome::Continue
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        let c = self.center?;
        if self.radius <= 1e-9 {
            return None;
        }
        let f = Frame { origin: c, ..current_cplane().frame };
        let h = if self.height.abs() < 1e-9 { self.radius * 1e-3 } else { self.height };
        Some(match self.kind {
            RoundKind::Cylinder => make_frustum(&f, self.radius, self.radius, h),
            RoundKind::Cone => make_frustum(&f, self.radius, self.radius2.max(0.0), h),
            RoundKind::Sphere => make_sphere_in(&f, self.radius),
            RoundKind::Torus => make_torus(&f, self.radius, self.radius2.max(self.radius * 0.05)),
            RoundKind::Helix => {
                let c = ductile_brep::curves::helix(&f, self.radius, h, self.turns);
                let (t0, t1) = c.domain();
                ductile_brep::profile::wire_from_segments(&[ductile_brep::profile::Segment::new(ductile_geom::Curve3::Nurbs(c), t0, t1)])
            }
        })
    }
    fn guides(&self, out: &mut Vec<LineInstance>) {
        if let Some(c) = self.center {
            out.push(LineInstance::point(c, [1.0, 0.85, 0.3, 1.0], 7.0));
        }
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut ch = drag_value(ui, "Radius", &mut self.radius, 0.05);
        match self.kind {
            RoundKind::Cylinder => ch |= drag_value(ui, "Height", &mut self.height, 0.1),
            RoundKind::Cone => {
                ch |= drag_value(ui, "Top radius", &mut self.radius2, 0.05);
                ch |= drag_value(ui, "Height", &mut self.height, 0.1);
            }
            RoundKind::Torus => ch |= drag_value(ui, "Tube radius", &mut self.radius2, 0.05),
            RoundKind::Helix => {
                ch |= drag_value(ui, "Height", &mut self.height, 0.1);
                ch |= drag_value(ui, "Turns", &mut self.turns, 0.1);
            }
            RoundKind::Sphere => {}
        }
        if ch && self.center.is_some() && self.stage < self.stages() - 1 {
            self.stage = self.stages() - 1;
        }
        ch
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        if self.kind != RoundKind::Sphere && self.kind != RoundKind::Torus && self.height.abs() < 1e-9 {
            return false;
        }
        let Some(b) = self.preview(doc) else { return false };
        let id = doc.add_body(self.name(), b);
        doc.selection.clear();
        doc.selection.insert(Pick::Object(id));
        true
    }
}

// ------------------------------------------------------------ Transform ----

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum TransformKind {
    Move,
    Rotate,
    Scale,
}

/// Blender/Plasticity-style modal transform: press G/R/S, optionally X/Y/Z to
/// constrain, move the mouse, click (or type a value) to confirm.
pub struct TransformTool {
    pub kind: TransformKind,
    targets: Vec<ObjectId>,
    /// Selected curve control points; when any, only these move.
    controls: Vec<(ObjectId, ductile_brep::EdgeId, u32)>,
    surface_controls: Vec<(ObjectId, ductile_brep::FaceId, u32)>,
    /// Selected faces of solids; when any, these move and the faces
    /// around them are re-trimmed to follow.
    faces: Vec<(ObjectId, FaceId)>,
    pivot: DVec3,
    start: Option<DVec3>,
    current: Option<DVec3>,
    pub axis: Option<usize>,
    /// Typed value: distance, degrees or factor.
    pub typed: Option<f64>,
    value: f64,
    /// Surface control points: how many grid steps around the selection
    /// follow with a smooth falloff, and whether the mirror (in u) moves too.
    pub falloff: usize,
    pub mirror: bool,
}

impl TransformTool {
    pub fn new(kind: TransformKind, doc: &Document) -> Self {
        let controls: Vec<_> = doc.selection.iter().filter_map(|p| if let Pick::Control(o, e, i) = p { Some((*o, *e, *i)) } else { None }).collect();
        let surface_controls: Vec<_> = doc.selection.iter().filter_map(|p| if let Pick::SurfaceControl(o, f, i) = p { Some((*o, *f, *i)) } else { None }).collect();
        let targets = doc.selected_objects();
        let faces: Vec<(ObjectId, FaceId)> = if controls.is_empty() && surface_controls.is_empty() { doc.selected_faces() } else { Vec::new() };
        let mut bb = ductile_geom::BBox3::EMPTY;
        for (o, f) in &faces {
            if let Some(ob) = doc.object(*o) {
                let b = &ob.body;
                bb = bb.union(&ductile_geom::BBox3::from_points(b.face_loops(*f).flat_map(|l| l.coedges.iter()).map(|c| b.vertex(b.coedge_vertices(*c).0))));
            }
        }
        for (o, f, i) in &surface_controls {
            if let Some(ductile_geom::Surface::Nurbs(s)) = doc.object(*o).map(|ob| ob.body.face(*f).surface.clone()) {
                if let Some(h) = s.ctrl.get(*i as usize) {
                    bb = bb.union(&ductile_geom::BBox3::from_points([h.truncate() / h.w]));
                }
            }
        }
        if controls.is_empty() && surface_controls.is_empty() && faces.is_empty() {
            for id in &targets {
                if let Some(o) = doc.object(*id) {
                    bb = bb.union(&o.body.bbox());
                }
            }
        } else if faces.is_empty() {
            for (o, e, i) in &controls {
                if let Some(ductile_geom::Curve3::Nurbs(n)) = doc.object(*o).map(|ob| ob.body.edge(*e).curve.clone()) {
                    if let Some(p) = n.control_points().get(*i as usize) {
                        bb = bb.union(&ductile_geom::BBox3::from_points([*p]));
                    }
                }
            }
        }
        let pivot = if bb.is_empty() { DVec3::ZERO } else { bb.center() };
        Self { kind, targets, controls, surface_controls, faces, pivot, start: None, current: None, axis: None, typed: None, value: 0.0, falloff: 0, mirror: false }
    }

    /// The object's body after applying `m` to it, or to its selected
    /// control points.
    fn moved(&self, id: ObjectId, body: &Body, m: &DMat4) -> Body {
        if !self.faces.is_empty() {
            // Moving faces: the planes go where `m` takes them, the faces
            // around follow; the body stays put when they cannot.
            let mine: Vec<FaceId> = self.faces.iter().filter(|f| f.0 == id).map(|f| f.1).collect();
            let planes: std::collections::HashMap<FaceId, ductile_geom::Surface> = mine.iter().map(|f| (*f, body.face(*f).surface.transformed(m))).collect();
            if let Some(b) = ductile_brep::ops::draft::replace_planes(body, &planes) {
                return b;
            }
            // A push straight out along the faces' normals: offset them, which
            // also re-trims curved faces around them.
            let shift = m.transform_point3(DVec3::ZERO);
            let along: Option<Vec<f64>> = mine
                .iter()
                .map(|f| {
                    let ductile_geom::Surface::Plane { frame } = &body.face(*f).surface else { return None };
                    let n = if body.face(*f).reversed { -frame.z } else { frame.z };
                    let moved_n = m.transform_vector3(n);
                    ((moved_n - n).length() < 1e-9 && (shift - n * shift.dot(n)).length() < 1e-9 * (1.0 + shift.length())).then_some(shift.dot(n))
                })
                .collect();
            if let Some(d) = along {
                let dist: std::collections::HashMap<FaceId, f64> = mine.iter().copied().zip(d).collect();
                if let Ok(b) = ductile_brep::ops::offset::offset_body(body, &|f| dist.get(&f).copied().unwrap_or(0.0)) {
                    return b;
                }
            }
            return body.clone();
        }
        if self.controls.is_empty() && self.surface_controls.is_empty() {
            return body.transformed(m);
        }
        let on_surface: Vec<_> = self.surface_controls.iter().filter(|c| c.0 == id).map(|c| (c.1, c.2)).collect();
        if !on_surface.is_empty() {
            return ductile_brep::ops::sheet::move_surface_controls_with(body, &on_surface, m, self.falloff, self.mirror).unwrap_or_else(|| body.clone());
        }
        let mine: Vec<_> = self.controls.iter().filter(|c| c.0 == id).map(|c| (c.1, c.2)).collect();
        ductile_brep::profile::move_controls(body, &mine, m)
    }

    pub fn has_targets(&self) -> bool {
        !self.targets.is_empty()
    }

    /// The point the transform turns and scales about.
    pub fn pivot(&self) -> DVec3 {
        self.pivot
    }

    fn axis_dir(&self) -> Option<DVec3> {
        self.axis.map(|a| [DVec3::X, DVec3::Y, DVec3::Z][a])
    }

    pub fn matrix(&self) -> DMat4 {
        let (Some(s), Some(c)) = (self.start, self.current) else {
            return match (self.kind, self.typed) {
                (_, None) => DMat4::IDENTITY,
                _ => self.typed_matrix(),
            };
        };
        if self.typed.is_some() {
            return self.typed_matrix();
        }
        match self.kind {
            TransformKind::Move => {
                let mut d = c - s;
                if let Some(a) = self.axis_dir() {
                    d = a * d.dot(a);
                }
                DMat4::from_translation(d)
            }
            TransformKind::Rotate => {
                let axis = self.axis_dir().unwrap_or(current_cplane().frame.z);
                let a = (s - self.pivot) - axis * (s - self.pivot).dot(axis);
                let b = (c - self.pivot) - axis * (c - self.pivot).dot(axis);
                if a.length() < 1e-9 || b.length() < 1e-9 {
                    return DMat4::IDENTITY;
                }
                let ang = a.cross(b).dot(axis).atan2(a.dot(b));
                let step = 5f64.to_radians();
                let ang = (ang / step).round() * step;
                rotate_about(self.pivot, axis, ang)
            }
            TransformKind::Scale => {
                let d0 = (s - self.pivot).length();
                let d1 = (c - self.pivot).length();
                if d0 < 1e-9 {
                    return DMat4::IDENTITY;
                }
                let f = ((d1 / d0) * 100.0).round() / 100.0;
                scale_about(self.pivot, f.max(1e-3))
            }
        }
    }

    fn typed_matrix(&self) -> DMat4 {
        let v = self.typed.unwrap_or(0.0);
        match self.kind {
            TransformKind::Move => {
                let dir = self.axis_dir().unwrap_or_else(|| match (self.start, self.current) {
                    (Some(s), Some(c)) if (c - s).length() > 1e-9 => (c - s).normalize(),
                    _ => DVec3::X,
                });
                DMat4::from_translation(dir * v)
            }
            TransformKind::Rotate => {
                rotate_about(self.pivot, self.axis_dir().unwrap_or(current_cplane().frame.z), v.to_radians())
            }
            TransformKind::Scale => scale_about(self.pivot, if v.abs() < 1e-9 { 1.0 } else { v }),
        }
    }
}

pub fn rotate_about(p: DVec3, axis: DVec3, ang: f64) -> DMat4 {
    DMat4::from_translation(p) * DMat4::from_quat(DQuat::from_axis_angle(axis.normalize(), ang)) * DMat4::from_translation(-p)
}

pub fn scale_about(p: DVec3, f: f64) -> DMat4 {
    DMat4::from_translation(p) * DMat4::from_scale(DVec3::splat(f)) * DMat4::from_translation(-p)
}

impl Tool for TransformTool {
    fn name(&self) -> &'static str {
        match self.kind {
            TransformKind::Move => "Move",
            TransformKind::Rotate => "Rotate",
            TransformKind::Scale => "Scale",
        }
    }
    fn hint(&self) -> String {
        let ax = match self.axis {
            Some(0) => " along X",
            Some(1) => " along Y",
            Some(2) => " along Z",
            _ => "",
        };
        format!("{}{ax}: move the mouse, X/Y/Z to constrain, type a value, click or Enter to confirm, Esc to cancel", self.name())
    }
    fn pointer(&mut self, _doc: &Document, cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        // Work on a plane through the pivot facing the camera (or the axis plane).
        let normal = match (self.kind, self.axis_dir()) {
            (TransformKind::Rotate, Some(a)) => a,
            (TransformKind::Rotate, None) => cplane.frame.z,
            (_, Some(a)) => {
                let side = a.cross(p.ray.dir);
                let n = side.cross(a);
                if n.length_squared() < 1e-12 {
                    -p.ray.dir
                } else {
                    n.normalize()
                }
            }
            _ => -p.ray.dir,
        };
        let hit = p.ray.hit_plane(self.pivot, normal);
        if self.start.is_none() {
            self.start = hit;
        }
        if let Some(h) = hit {
            // Snapping to geometry for moves.
            self.current = Some(if self.kind == TransformKind::Move { p.point.filter(|_| self.axis.is_none()).unwrap_or(h) } else { h });
        }
        self.value = match self.kind {
            TransformKind::Move => self.matrix().w_axis.truncate().length(),
            _ => 0.0,
        };
        if p.clicked {
            return ToolOutcome::Finished;
        }
        ToolOutcome::Continue
    }
    fn preview(&self, doc: &Document) -> Option<Body> {
        let m = self.matrix();
        let mut out: Option<Body> = None;
        for id in &self.targets {
            if let Some(o) = doc.object(*id) {
                let b = self.moved(*id, &o.body, &m);
                match &mut out {
                    None => out = Some(b),
                    Some(acc) => acc.append(&b),
                }
            }
        }
        out
    }
    fn guides(&self, out: &mut Vec<LineInstance>) {
        out.push(LineInstance::point(self.pivot, [1.0, 1.0, 1.0, 1.0], 6.0));
        if let Some(a) = self.axis_dir() {
            let col = [[0.9, 0.3, 0.3, 1.0], [0.4, 0.85, 0.3, 1.0], [0.3, 0.5, 0.95, 1.0]][self.axis.unwrap()];
            out.push(LineInstance::new(self.pivot - a * 1e4, self.pivot + a * 1e4, col, 1.5));
        }
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let label = match self.kind {
            TransformKind::Move => "Distance",
            TransformKind::Rotate => "Angle (°)",
            TransformKind::Scale => "Factor",
        };
        let mut v = self.typed.unwrap_or(match self.kind {
            TransformKind::Scale => 1.0,
            TransformKind::Move => self.value,
            _ => 0.0,
        });
        let ch = drag_value(ui, label, &mut v, 0.05);
        if ch {
            self.typed = Some(v);
        }
        ui.horizontal(|ui| {
            ui.label("Axis");
            for (i, n) in ["X", "Y", "Z"].iter().enumerate() {
                if ui.selectable_label(self.axis == Some(i), *n).clicked() {
                    self.axis = if self.axis == Some(i) { None } else { Some(i) };
                }
            }
        });
        let mut ch = ch;
        if !self.surface_controls.is_empty() {
            ui.horizontal(|ui| {
                ui.label("Falloff");
                ch |= ui.add(egui::DragValue::new(&mut self.falloff).range(0..=20)).changed();
                ch |= ui.checkbox(&mut self.mirror, "Mirror").changed();
            });
        }
        ch
    }
    fn hidden_objects(&self) -> Vec<ObjectId> {
        self.targets.clone()
    }
    fn typed_value(&mut self, v: Option<f64>) {
        self.typed = v;
    }
    fn constrain_axis(&mut self, axis: usize) {
        self.axis = if self.axis == Some(axis) { None } else { Some(axis) };
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let m = self.matrix();
        if m.abs_diff_eq(DMat4::IDENTITY, 1e-12) {
            return false;
        }
        for id in self.targets.clone() {
            if let Some(o) = doc.object(id) {
                let b = self.moved(id, &o.body, &m);
                if self.faces.is_empty() && self.controls.is_empty() && self.surface_controls.is_empty() {
                    doc.move_body(id, b, &m);
                } else {
                    doc.replace_body(id, b);
                }
            }
        }
        true
    }
}


// -------------------------------------------------------------- Extrude ----

/// Push/pull of a planar face (pulling out adds material, pushing in cuts),
/// or extrusion of the regions bounded by selected closed planar curves
/// into a new solid.
pub struct ExtrudeTool {
    /// Solid whose face is pushed or pulled; `None` when extruding curves.
    target: Option<ObjectId>,
    body: Option<std::sync::Arc<Body>>,
    regions: Vec<ductile_brep::profile::Region>,
    /// Open curves, extruded into a sheet when there are no regions.
    open: Vec<ductile_brep::profile::Loop3>,
    normal: DVec3,
    base: DVec3,
    start_t: Option<f64>,
    pub distance: f64,
    typed: Option<f64>,
    /// For curves drawn on a solid's planar face: that solid, and the
    /// face's outward normal.
    onto: Option<(ObjectId, std::sync::Arc<Body>, DVec3)>,
    pub op: ExtrudeOp,
    bg: Background<(f64, ExtrudeOp), (Option<Body>, Option<String>)>,
}

/// What an extrusion of curves drawn on a solid does to that solid.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum ExtrudeOp {
    /// Join when pulled out of the face, cut when pushed into it.
    Auto,
    NewBody,
    Join,
    Cut,
    Intersect,
}

/// The visible solid one of whose planar faces holds region `r`, with that
/// face's outward normal.
fn solid_under_region(doc: &Document, r: &ductile_brep::profile::Region) -> Option<(ObjectId, std::sync::Arc<Body>, DVec3)> {
    use ductile_geom::poly2::point_in_polygon;
    let pts = r.outer.polygon(1e-3);
    let centre = pts.iter().copied().sum::<DVec3>() / pts.len().max(1) as f64;
    for o in doc.objects.iter().filter(|o| o.visible && o.body.kind == ductile_brep::BodyKind::Solid) {
        let tol = 1e-7 * o.body.bbox().diagonal().max(1.0);
        for f in o.body.face_ids() {
            let face = o.body.face(f);
            let ductile_geom::Surface::Plane { frame } = &face.surface else { continue };
            if frame.z.dot(r.plane.z).abs() < 1.0 - 1e-9 || pts.iter().any(|p| (*p - frame.origin).dot(frame.z).abs() > tol) {
                continue;
            }
            let Some(fr) = ductile_brep::profile::Region::from_face(&o.body, f) else { continue };
            let flat = |l: &ductile_brep::profile::Loop3| -> Vec<ductile_geom::DVec2> { l.polygon(1e-3).into_iter().map(|p| fr.plane.to_local(p).truncate()).collect() };
            let c = fr.plane.to_local(centre).truncate();
            if point_in_polygon(c, &flat(&fr.outer)) && !fr.holes.iter().any(|h| point_in_polygon(c, &flat(h))) {
                let n = if face.reversed { -frame.z } else { frame.z };
                return Some((o.id, o.body.clone(), n));
            }
        }
    }
    None
}

/// Regions bounded by the closed curves of the selected wire objects.
pub fn selected_curve_regions(doc: &Document) -> Vec<ductile_brep::profile::Region> {
    use ductile_brep::profile::{loops_plane, wire_chains, wire_from_segments, Region, Segment};
    let mut ids: Vec<ObjectId> = doc.selection.iter().map(|p| p.object()).collect();
    ids.dedup();
    // Curves drawn separately but meeting end to end (an arc and the line
    // closing it) chain into one profile.
    let segs: Vec<Segment> = ids
        .iter()
        .filter_map(|id| doc.object(*id))
        .filter(|o| o.body.kind == ductile_brep::BodyKind::Wire)
        .flat_map(|o| o.body.edges.iter().map(|e| Segment::new(e.curve.clone(), e.t0, e.t1)).collect::<Vec<_>>())
        .collect();
    let loops: Vec<_> = wire_chains(&wire_from_segments(&segs)).into_iter().filter(|l| l.is_closed()).collect();
    match loops_plane(&loops) {
        Some(plane) => Region::from_loops(plane, loops),
        None => Vec::new(),
    }
}

/// Chains of the selected wire objects; separately drawn curves that meet
/// end to end join up.
fn selected_chains(doc: &Document) -> Vec<ductile_brep::profile::Loop3> {
    use ductile_brep::profile::{wire_chains, wire_from_segments, Segment};
    let segs: Vec<Segment> = doc
        .selected_objects()
        .iter()
        .filter_map(|id| doc.object(*id))
        .filter(|o| o.body.kind == ductile_brep::BodyKind::Wire)
        .flat_map(|o| o.body.edges.iter().map(|e| Segment::new(e.curve.clone(), e.t0, e.t1)).collect::<Vec<_>>())
        .collect();
    wire_chains(&wire_from_segments(&segs))
}

/// Lofts a solid through the selected closed curves, ordered along the
/// line through the two profiles farthest apart.
pub fn loft_selected(doc: &Document) -> Result<Body, String> {
    let chains = selected_chains(doc);
    let closed = chains.iter().filter(|l| l.is_closed()).count();
    // Closed profiles make a solid; open ones a sheet.
    let sheet = closed < 2;
    let mut loops: Vec<_> = chains.into_iter().filter(|l| l.is_closed() != sheet).collect();
    if loops.len() < 2 {
        return Err("Loft: select two or more closed curves (a solid) or open curves (a surface)".into());
    }
    let centroid = |l: &ductile_brep::profile::Loop3| {
        let p = l.polygon(1e-2);
        p.iter().copied().sum::<DVec3>() / p.len() as f64
    };
    let cs: Vec<DVec3> = loops.iter().map(centroid).collect();
    let (mut a, mut b) = (0, 1);
    for i in 0..cs.len() {
        for j in i + 1..cs.len() {
            if cs[i].distance(cs[j]) > cs[a].distance(cs[b]) {
                (a, b) = (i, j);
            }
        }
    }
    let axis = cs[b] - cs[a];
    loops.sort_by(|x, y| centroid(x).dot(axis).total_cmp(&centroid(y).dot(axis)));
    if sheet {
        return ductile_brep::ops::sheet::loft_sheet(&loops).ok_or_else(|| "Loft failed".into());
    }
    ductile_brep::ops::loft::loft(&loops).ok_or_else(|| "Loft failed: the profiles must be planar and apart".into())
}

/// Fills each selected closed curve with a surface: a trimmed plane when
/// it is planar (closed curves inside it become holes), else a Coons patch.
pub fn patch_selected(doc: &Document) -> Result<Body, String> {
    let closed: Vec<_> = selected_chains(doc).into_iter().filter(|l| l.is_closed()).collect();
    if closed.is_empty() {
        return patch_edges(doc);
    }
    let mut out: Option<Body> = None;
    let mut add = |b: Body| match &mut out {
        None => out = Some(b),
        Some(o) => o.append(&b),
    };
    let regions = ductile_brep::profile::loops_plane(&closed).map(|plane| ductile_brep::profile::Region::from_loops(plane, closed.clone())).unwrap_or_default();
    if !regions.is_empty() {
        for r in &regions {
            add(ductile_brep::ops::sheet::patch(&r.outer, &r.holes).ok_or("Patch failed")?);
        }
    } else {
        for c in &closed {
            add(ductile_brep::ops::sheet::patch(c, &[]).ok_or("Patch failed")?);
        }
    }
    out.ok_or_else(|| "Patch failed".into())
}

/// Fills the loop of selected edges with a surface that meets the faces
/// beyond them tangentially.
fn patch_edges(doc: &Document) -> Result<Body, String> {
    use ductile_brep::profile::{wire_chains, wire_from_segments, Segment};
    let edges = doc.selected_edges();
    if edges.is_empty() {
        return Err("Patch: select closed curves, or edges around a hole (edge mode: 2)".into());
    }
    // Each edge with the face beyond it and a point inside that face.
    let mut info = Vec::new();
    for (o, e) in &edges {
        let body = &doc.object(*o).ok_or("Patch: missing object")?.body;
        let ed = body.edge(*e);
        let seg = Segment::new(ed.curve.clone(), ed.t0, ed.t1);
        let nb = body.edge_faces(*e).first().map(|(f, _)| {
            let mut one = (**body).clone();
            one.faces = vec![body.face(*f).clone()];
            let m = ductile_brep::tessellate(&one, &ductile_brep::TessOptions::for_size(body.bbox().diagonal()));
            let pts: Vec<DVec3> = m.faces.iter().flat_map(|f| f.positions.iter().copied()).collect();
            let inside = pts.iter().copied().sum::<DVec3>() / pts.len().max(1) as f64;
            (body.face(*f).surface.clone(), inside)
        });
        info.push((seg, nb));
    }
    let segs: Vec<Segment> = info.iter().map(|i| i.0.clone()).collect();
    let chain = wire_chains(&wire_from_segments(&segs)).into_iter().find(|c| c.is_closed()).ok_or("Patch: the selected edges must form a closed loop")?;
    // The neighbour of each chained segment, matched by midpoint.
    let mid = |s: &Segment| s.curve.point(0.5 * (s.t0 + s.t1));
    let neighbours: Vec<ductile_brep::ops::sheet::Neighbour> = chain
        .segments
        .iter()
        .map(|s| info.iter().min_by(|a, b| mid(&a.0).distance(mid(s)).total_cmp(&mid(&b.0).distance(mid(s)))).and_then(|i| i.1.clone()))
        .collect();
    ductile_brep::ops::sheet::patch_g1(&chain, &neighbours).ok_or_else(|| "Patch failed".into())
}

/// Cuts the selected solids along the selected curves, swept through them
/// square to the curves' plane (the construction plane for a straight
/// line). Returns each cut solid with its pieces.
pub fn cut_selected(doc: &Document) -> Result<(Vec<ObjectId>, Vec<Vec<Body>>), String> {
    let (solids, cutter) = cut_inputs(doc, "Cut")?;
    let mut pieces = Vec::new();
    for id in &solids {
        let b = &doc.object(*id).ok_or("Cut: missing object")?.body;
        let parts = match &cutter {
            Cutter::Curves(chains, dir) => ductile_brep::ops::cut::cut_with_curves(b, chains, *dir),
            Cutter::Sheets(sheets) => {
                let mut parts = vec![(**b).clone()];
                for s in sheets {
                    let mut next = Vec::new();
                    for p in &parts {
                        next.extend(ductile_brep::ops::cut::cut_with_sheet(p, s).map_err(|e| format!("Cut failed: {e}"))?);
                    }
                    parts = next;
                }
                Ok(parts)
            }
        }
        .map_err(|e| format!("Cut failed: {e}"))?;
        pieces.push(parts);
    }
    Ok((solids, pieces))
}

/// Imprints the selected curves or sheets onto the selected solids,
/// splitting the faces they cross; returns the solids and their imprinted
/// bodies.
pub fn imprint_selected(doc: &Document) -> Result<(Vec<ObjectId>, Vec<Body>), String> {
    let (solids, cutter) = cut_inputs(doc, "Imprint")?;
    let mut out = Vec::new();
    for id in &solids {
        let b = &doc.object(*id).ok_or("Imprint: missing object")?.body;
        let r = match &cutter {
            Cutter::Curves(chains, dir) => ductile_brep::ops::cut::imprint_curves(b, chains, *dir),
            Cutter::Sheets(sheets) => sheets.iter().try_fold((**b).clone(), |acc, s| ductile_brep::ops::cut::imprint_sheet(&acc, s)),
        };
        out.push(r.map_err(|e| format!("Imprint failed: {e}"))?);
    }
    Ok((solids, out))
}

/// Projects the selected curves onto the selected solids, square to the
/// curves' plane (or along the construction plane's normal); one wire body
/// per solid the curves land on.
pub fn project_selected(doc: &Document) -> Result<Vec<Body>, String> {
    let (solids, cutter) = cut_inputs(doc, "Project")?;
    let Cutter::Curves(chains, dir) = cutter else { return Err("Project: select curves to project, not surfaces".into()) };
    let mut out = Vec::new();
    for id in &solids {
        let b = &doc.object(*id).ok_or("Project: missing object")?.body;
        let w = ductile_brep::ops::cut::project_curves(b, &chains, dir).map_err(|e| format!("Project failed: {e}"))?;
        if !w.edges.is_empty() {
            out.push(w);
        }
    }
    if out.is_empty() {
        return Err("Project: the curves miss the selected solids".into());
    }
    Ok(out)
}

/// What the selected solids are cut or imprinted with.
enum Cutter {
    /// Curve chains swept through the solids along a direction.
    Curves(Vec<ductile_brep::profile::Loop3>, DVec3),
    /// Sheets reaching right across the solids.
    Sheets(Vec<std::sync::Arc<Body>>),
}

/// The selected solids and what to cut them with: the selected sheets, or
/// else the selected curve chains, swept square to their plane (or along
/// the construction plane's normal for a straight line).
fn cut_inputs(doc: &Document, what: &str) -> Result<(Vec<ObjectId>, Cutter), String> {
    let of_kind = |k: ductile_brep::BodyKind| -> Vec<ObjectId> { doc.selected_objects().into_iter().filter(|id| doc.object(*id).is_some_and(|o| o.body.kind == k)).collect() };
    let solids = of_kind(ductile_brep::BodyKind::Solid);
    let sheets: Vec<std::sync::Arc<Body>> = of_kind(ductile_brep::BodyKind::Sheet).into_iter().filter_map(|id| doc.object(id).map(|o| o.body.clone())).collect();
    let chains = selected_chains(doc);
    if solids.is_empty() || (chains.is_empty() && sheets.is_empty()) {
        return Err(format!("{what}: select solids and the curves or surfaces to {} them with", what.to_lowercase()));
    }
    if !sheets.is_empty() {
        return Ok((solids, Cutter::Sheets(sheets)));
    }
    let dir = ductile_brep::profile::loops_plane(&chains).filter(|_| chains.iter().any(|c| c.segments.len() > 1 || !c.segments[0].curve.is_linear())).map(|f| f.z).unwrap_or(current_cplane().frame.z);
    Ok((solids, Cutter::Curves(chains, dir)))
}

/// A construction plane lying on planar face `f`: its normal facing out of
/// the solid, its x axis along the face's longest straight edge, and its
/// origin at the middle of the face's corners.
pub fn cplane_on_face(b: &Body, f: FaceId) -> Option<Frame> {
    let face = b.face(f);
    let ductile_geom::Surface::Plane { frame } = &face.surface else { return None };
    let z = if face.reversed { -frame.z } else { frame.z };
    let edges: Vec<&ductile_brep::topo::Edge> = b.face_loops(f).take(1).flat_map(|l| l.coedges.iter()).map(|c| b.edge(c.edge)).collect();
    let longest = edges.iter().filter(|e| e.curve.is_linear()).max_by(|a, c| a.length().total_cmp(&c.length()));
    let x = longest.map(|e| e.end() - e.start()).unwrap_or(frame.x);
    let x = (x - z * x.dot(z)).try_normalize().unwrap_or(frame.x);
    let pts: Vec<DVec3> = b.face_loops(f).take(1).flat_map(|l| l.coedges.iter()).map(|c| b.vertex(b.coedge_vertices(*c).0)).collect();
    let mid = if pts.is_empty() { frame.origin } else { pts.iter().copied().sum::<DVec3>() / pts.len() as f64 };
    let origin = mid - z * (mid - frame.origin).dot(z);
    Some(Frame::new(origin, z, x))
}

/// Mirrors the selected objects. With a planar face selected the mirror
/// plane is that face, and a solid mirrored across its own face is joined
/// to its reflection (model one half, mirror the other); otherwise the
/// plane is square to the construction plane's `axis` through its origin.
pub fn mirror_selected(doc: &mut Document, cplane: &Frame, axis: usize) -> Result<String, String> {
    use ductile_brep::ops::boolean::{boolean, BoolOp};
    let face = doc.selected_faces().into_iter().find_map(|(o, f)| {
        let b = &doc.object(o)?.body;
        match &b.face(f).surface {
            ductile_geom::Surface::Plane { frame } => Some((o, frame.origin, frame.z)),
            _ => None,
        }
    });
    let (origin, n) = match face {
        Some((_, p, n)) => (p, n),
        None => (cplane.origin, [cplane.x, cplane.y, cplane.z][axis]),
    };
    let n = n.normalize();
    // Reflection x -> x - 2 n (n . (x - origin)).
    let m = DMat4::from_translation(origin) * DMat4::from_mat3(ductile_geom::DMat3::IDENTITY - 2.0 * ductile_geom::DMat3::from_cols(n * n.x, n * n.y, n * n.z)) * DMat4::from_translation(-origin);
    let ids = doc.selected_objects();
    doc.selection.clear();
    let mut joined = 0;
    for id in ids {
        let Some(ob) = doc.object(id).cloned() else { continue };
        let mirrored = ob.body.transformed(&m);
        if face.map(|f| f.0) == Some(id) && ob.body.kind == ductile_brep::BodyKind::Solid {
            let whole = boolean(&ob.body, &mirrored, BoolOp::Union).map_err(|e| format!("Mirror: could not join the halves: {e}"))?;
            doc.replace_body(id, whole);
            doc.selection.insert(Pick::Object(id));
            joined += 1;
        } else {
            let nid = doc.add_body(&ob.name, mirrored);
            doc.selection.insert(Pick::Object(nid));
        }
    }
    Ok(if joined > 0 { "Mirrored across the face and joined into one solid".into() } else { "Mirrored".into() })
}

/// The curves where each pair of selected solids cross, one wire body per
/// pair that meets.
pub fn intersection_curves_selected(doc: &Document) -> Result<Vec<Body>, String> {
    let solids: Vec<std::sync::Arc<Body>> = doc.selected_objects().into_iter().filter_map(|id| doc.object(id).filter(|o| o.body.kind == ductile_brep::BodyKind::Solid).map(|o| o.body.clone())).collect();
    if solids.len() < 2 {
        return Err("Intersection curves: select two or more solids".into());
    }
    let mut out = Vec::new();
    for i in 0..solids.len() {
        for j in i + 1..solids.len() {
            let w = ductile_brep::ops::cut::intersection_curves(&solids[i], &solids[j]).map_err(|e| format!("Intersection curves failed: {e}"))?;
            if !w.edges.is_empty() {
                out.push(w);
            }
        }
    }
    if out.is_empty() {
        return Err("Intersection curves: the selected solids do not meet".into());
    }
    Ok(out)
}

/// Deletes the selection: whole objects in object mode, faces (healing
/// holes where it can) in face mode, and curve edges in edge mode. Returns
/// a status line, or `None` when nothing was deleted.
pub fn delete_selected(doc: &mut Document) -> Option<String> {
    use ductile_brep::ops::delete::{delete_faces, extract_edges};
    use std::collections::BTreeMap;
    match doc.mode {
        SelectMode::Object => {
            let ids = doc.selected_objects();
            if ids.is_empty() {
                return None;
            }
            doc.checkpoint("Delete");
            for id in &ids {
                doc.remove(*id);
            }
            Some(format!("Deleted {} object(s)", ids.len()))
        }
        SelectMode::Face => {
            let mut by: BTreeMap<ObjectId, Vec<FaceId>> = BTreeMap::new();
            for (o, f) in doc.selected_faces() {
                by.entry(o).or_default().push(f);
            }
            if by.is_empty() {
                return None;
            }
            doc.checkpoint("Delete faces");
            let mut opened = 0;
            for (id, faces) in by {
                let Some(o) = doc.object(id) else { continue };
                let was = o.body.kind;
                let b = delete_faces(&o.body, &faces);
                if b.faces.is_empty() {
                    doc.remove(id);
                } else {
                    opened += usize::from(was == ductile_brep::BodyKind::Solid && b.kind != was);
                    doc.replace_body(id, b);
                }
            }
            doc.selection.clear();
            Some(if opened > 0 { "Deleted faces; the gap could not be healed, so the solid is now an open sheet".into() } else { "Deleted faces".into() })
        }
        SelectMode::Edge => {
            let mut by: BTreeMap<ObjectId, Vec<EdgeId>> = BTreeMap::new();
            for (o, e) in doc.selected_edges() {
                if doc.object(o).is_some_and(|ob| ob.body.kind == ductile_brep::BodyKind::Wire) {
                    by.entry(o).or_default().push(e);
                }
            }
            if by.is_empty() {
                return Some("Delete: only curve edges can be deleted; delete faces in face mode".into());
            }
            doc.checkpoint("Delete edges");
            for (id, gone) in by {
                let Some(o) = doc.object(id) else { continue };
                let keep: Vec<EdgeId> = o.body.edge_ids().filter(|e| !gone.contains(e)).collect();
                if keep.is_empty() {
                    doc.remove(id);
                } else {
                    let b = extract_edges(&o.body, &keep);
                    doc.replace_body(id, b);
                }
            }
            doc.selection.clear();
            Some("Deleted curves".into())
        }
        SelectMode::Vertex => None,
    }
}

/// Copies the selection into new objects: whole objects, selected faces as
/// a sheet, or selected edges as curves. Returns the new objects.
pub fn duplicate_selected(doc: &mut Document) -> Vec<ObjectId> {
    use ductile_brep::ops::delete::{extract_edges, extract_faces};
    let mut made = Vec::new();
    let faces = doc.selected_faces();
    let edges = doc.selected_edges();
    let objects: Vec<ObjectId> = doc.selection.iter().filter_map(|p| if let Pick::Object(o) = p { Some(*o) } else { None }).collect();
    if faces.is_empty() && edges.is_empty() && objects.is_empty() {
        return made;
    }
    doc.checkpoint("Duplicate");
    for id in doc.selected_objects() {
        let Some(o) = doc.object(id).cloned() else { continue };
        let fs: Vec<FaceId> = faces.iter().filter(|(x, _)| *x == id).map(|(_, f)| *f).collect();
        let es: Vec<EdgeId> = edges.iter().filter(|(x, _)| *x == id).map(|(_, e)| *e).collect();
        if objects.contains(&id) {
            let copy = doc.add_body(&o.name, (*o.body).clone());
            if let Some(c) = doc.objects.iter_mut().find(|x| x.id == copy) {
                (c.group, c.color) = (o.group, o.color);
            }
            made.push(copy);
        }
        if !fs.is_empty() {
            made.push(doc.add_body("Surface", extract_faces(&o.body, &fs)));
        }
        if !es.is_empty() {
            made.push(doc.add_body("Curve", extract_edges(&o.body, &es)));
        }
    }
    made
}

/// Joins the selected curves into one curve object, replacing them; ends
/// that meet share a vertex, so the result chains through them.
pub fn join_selected(doc: &mut Document) -> Result<ObjectId, String> {
    let ids: Vec<ObjectId> = doc.selected_objects().into_iter().filter(|id| doc.object(*id).is_some_and(|o| o.body.kind == ductile_brep::BodyKind::Wire)).collect();
    if ids.len() < 2 {
        return Err("Join curves: select two or more curves".into());
    }
    let segs: Vec<ductile_brep::profile::Segment> = selected_chains(doc).into_iter().flat_map(|c| c.segments).collect();
    doc.checkpoint("Join curves");
    for id in &ids {
        doc.remove(*id);
    }
    Ok(doc.add_body("Curve", ductile_brep::profile::wire_from_segments(&segs)))
}

/// Sews the selected surfaces into one along the edges they share; true
/// when they closed up into a solid.
pub fn join_sheets_selected(doc: &mut Document) -> Result<(ObjectId, bool), String> {
    let ids: Vec<ObjectId> = doc.selected_objects().into_iter().filter(|id| doc.object(*id).is_some_and(|o| o.body.kind == ductile_brep::BodyKind::Sheet)).collect();
    if ids.len() < 2 {
        return Err("Join: select two or more surfaces".into());
    }
    let bodies: Vec<std::sync::Arc<Body>> = ids.iter().filter_map(|id| doc.object(*id).map(|o| o.body.clone())).collect();
    let size = bodies.iter().map(|b| b.bbox().diagonal()).fold(0.0, f64::max);
    let refs: Vec<&Body> = bodies.iter().map(|b| b.as_ref()).collect();
    let joined = ductile_brep::ops::join::join_sheets(&refs, 1e-5 * (1.0 + size)).map_err(|e| format!("Join: {e}"))?;
    let solid = joined.kind == ductile_brep::BodyKind::Solid;
    doc.checkpoint("Join surfaces");
    for id in &ids {
        doc.remove(*id);
    }
    Ok((doc.add_body(if solid { "Solid" } else { "Surface" }, joined), solid))
}

/// The selected curve objects with their chains.
fn selected_wires(doc: &Document) -> Vec<(ObjectId, Vec<ductile_brep::profile::Loop3>)> {
    doc.selected_objects()
        .into_iter()
        .filter_map(|id| doc.object(id).filter(|o| o.body.kind == ductile_brep::BodyKind::Wire).map(|o| (id, ductile_brep::profile::wire_chains(&o.body))))
        .collect()
}

/// Applies `f` to every segment chain of the selected curves.
pub fn edit_selected_curves(doc: &mut Document, name: &str, f: &dyn Fn(&[ductile_brep::profile::Segment]) -> Vec<ductile_brep::profile::Segment>) -> Result<usize, String> {
    let wires = selected_wires(doc);
    if wires.is_empty() {
        return Err(format!("{name}: select curves"));
    }
    doc.checkpoint(name);
    for (id, chains) in &wires {
        let segs: Vec<_> = chains.iter().flat_map(|c| f(&c.segments)).collect();
        doc.replace_body(*id, ductile_brep::profile::wire_from_segments(&segs));
    }
    Ok(wires.len())
}

/// Rebuild curve: refits the selected curves as single splines with a
/// chosen number of control points and degree.
pub struct RebuildCurveTool {
    targets: Vec<(ObjectId, Vec<ductile_brep::profile::Loop3>)>,
    pub points: usize,
    pub degree: usize,
}

impl RebuildCurveTool {
    pub fn new(doc: &Document) -> Option<Self> {
        let targets = selected_wires(doc);
        if targets.is_empty() {
            return None;
        }
        Some(Self { targets, points: 8, degree: 3 })
    }

    fn rebuilt(&self) -> Vec<(ObjectId, Body, f64)> {
        use ductile_brep::profile::Segment;
        self.targets
            .iter()
            .filter_map(|(id, chains)| {
                let mut segs = Vec::new();
                let mut dev: f64 = 0.0;
                for c in chains {
                    let (n, d) = ductile_brep::curves::rebuild_chain(&c.segments, self.points, self.degree)?;
                    let (a, b) = n.domain();
                    segs.push(Segment::new(ductile_geom::Curve3::Nurbs(n), a, b));
                    dev = dev.max(d);
                }
                Some((*id, ductile_brep::profile::wire_from_segments(&segs), dev))
            })
            .collect()
    }
}

impl Tool for RebuildCurveTool {
    fn name(&self) -> &'static str {
        "Rebuild curve"
    }
    fn hint(&self) -> String {
        let dev = self.rebuilt().iter().map(|r| r.2).fold(0.0, f64::max);
        format!("Rebuild curve: {} control points, degree {}; deviation {:.5}; Enter to finish", self.points, self.degree, dev)
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        if p.clicked { ToolOutcome::Finished } else { ToolOutcome::Continue }
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        let mut out: Option<Body> = None;
        for (_, b, _) in self.rebuilt() {
            match &mut out {
                None => out = Some(b),
                Some(o) => o.append(&b),
            }
        }
        out
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let (mut n, mut d) = (self.points as f64, self.degree as f64);
        let mut ch = drag_value(ui, "Control points", &mut n, 0.1);
        ch |= drag_value(ui, "Degree", &mut d, 0.05);
        self.degree = (d.round() as usize).clamp(1, 7);
        self.points = (n.round() as usize).clamp(self.degree + 1, 200);
        ch
    }
    fn typed_value(&mut self, v: Option<f64>) {
        if let Some(v) = v {
            self.points = (v.round().max(0.0) as usize).clamp(self.degree + 1, 200);
        }
    }
    fn hidden_objects(&self) -> Vec<ObjectId> {
        self.targets.iter().map(|t| t.0).collect()
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let r = self.rebuilt();
        for (id, b, _) in &r {
            doc.replace_body(*id, b.clone());
        }
        !r.is_empty()
    }
}

/// Sweeps the selected closed curve along the selected open one.
pub fn sweep_selected(doc: &Document) -> Result<Body, String> {
    let chains = selected_chains(doc);
    let (closed, open): (Vec<_>, Vec<_>) = chains.into_iter().partition(|l| l.is_closed());
    let (Some(profile), Some(path)) = (closed.first(), open.first()) else {
        return Err("Sweep: select a closed profile curve and an open path curve".into());
    };
    if closed.len() > 1 || open.len() > 1 {
        return Err("Sweep: select one profile and one path".into());
    }
    // Run the path from the end nearer the profile.
    let pc = {
        let p = profile.polygon(1e-2);
        p.iter().copied().sum::<DVec3>() / p.len() as f64
    };
    let path = if path.segments.last().unwrap().end().distance(pc) < path.segments[0].start().distance(pc) { path.reversed() } else { path.clone() };
    ductile_brep::ops::sweep::sweep(profile, &path.segments).ok_or_else(|| "Sweep failed".into())
}

impl ExtrudeTool {
    /// Starts on the first selected planar face, or else on the regions of
    /// the selected closed curves.
    pub fn new(doc: &Document) -> Option<Self> {
        let face = doc.selection.iter().find_map(|p| match p {
            Pick::Face(o, f) => Some((*o, *f)),
            _ => None,
        });
        let (target, body, regions) = match face {
            Some((id, f)) => {
                let o = doc.object(id)?;
                let region = ductile_brep::profile::Region::from_face(&o.body, f)?;
                (Some(id), Some(o.body.clone()), vec![region])
            }
            None => {
                let regions = selected_curve_regions(doc);
                if regions.is_empty() {
                    let open: Vec<_> = selected_chains(doc).into_iter().filter(|c| !c.is_closed()).collect();
                    if open.is_empty() {
                        return None;
                    }
                    // Out of the curves' plane when they span one, else
                    // along the construction plane normal.
                    let normal = ductile_brep::profile::loops_plane(&open).filter(|_| open.iter().any(|c| c.segments.iter().any(|s| !s.curve.is_linear()) || c.segments.len() > 1)).map(|f| f.z).unwrap_or(current_cplane().frame.z);
                    let poly: Vec<DVec3> = open.iter().flat_map(|c| c.polygon(1e-3)).collect();
                    let base = poly.iter().copied().sum::<DVec3>() / poly.len().max(1) as f64;
                    return Some(Self { target: None, body: None, regions, open, normal, base, start_t: None, distance: 0.0, typed: None, onto: None, op: ExtrudeOp::Auto, bg: Background::default() });
                }
                (None, None, regions)
            }
        };
        let poly = regions[0].outer.polygon(1e-3);
        let base = poly.iter().copied().sum::<DVec3>() / poly.len().max(1) as f64;
        let onto = if target.is_none() { solid_under_region(doc, &regions[0]) } else { None };
        Some(Self {
            target,
            body,
            open: Vec::new(),
            normal: regions[0].plane.z,
            regions,
            base,
            start_t: None,
            distance: 0.0,
            typed: None,
            onto,
            op: ExtrudeOp::Auto,
            bg: Background::default(),
        })
    }

    fn value(&self) -> f64 {
        self.typed.unwrap_or(self.distance)
    }

    fn key(&self) -> (f64, ExtrudeOp) {
        (self.value(), self.op)
    }

    fn job(&self) -> Box<dyn FnOnce() -> (Option<Body>, Option<String>) + Send> {
        use ductile_brep::ops::boolean::{boolean, BoolOp};
        let d = self.value();
        let (regions, open, normal, body) = (self.regions.clone(), self.open.clone(), self.normal, self.body.clone());
        let onto = self.on_solid_op(d).zip(self.onto.as_ref().map(|o| o.1.clone()));
        let face_op = self.bool_op(d > 0.0);
        Box::new(move || {
            if d.abs() < 1e-9 {
                return (None, None);
            }
            if regions.is_empty() {
                return match ductile_brep::ops::sheet::extrude_sheet(&open, normal * d) {
                    Some(b) => (Some(b), None),
                    None => (None, Some("could not extrude the curves".into())),
                };
            }
            let mut acc: Result<Option<Body>, String> = Ok(None);
            for r in &regions {
                let Ok(prev) = acc else { break };
                acc = match ductile_brep::ops::extrude::extrude_region(r, normal * d) {
                    None => Err("could not build the extrusion".into()),
                    Some(prism) => match prev {
                        None => Ok(Some(prism)),
                        Some(p) => boolean(&p, &prism, BoolOp::Union).map(Some).map_err(|e| e.to_string()),
                    },
                };
            }
            match (acc, &body) {
                (Err(e), _) => (None, Some(e)),
                (Ok(None), _) => (None, None),
                (Ok(Some(prism)), None) => match &onto {
                    Some((op, solid)) => match boolean(solid, &prism, *op) {
                        Ok(b) => (Some(b), None),
                        Err(e) => (None, Some(e.to_string())),
                    },
                    None => (Some(prism), None),
                },
                (Ok(Some(prism)), Some(body)) => match face_op {
                    None => (Some(prism), None),
                    Some(op) => match boolean(body, &prism, op) {
                        Ok(b) => (Some(b), None),
                        Err(e) => (None, Some(e.to_string())),
                    },
                },
            }
        })
    }

    /// The boolean an extrusion of curves applies to the solid they were
    /// drawn on, if any.
    fn on_solid_op(&self, d: f64) -> Option<ductile_brep::ops::boolean::BoolOp> {
        let (_, _, n) = self.onto.as_ref()?;
        self.bool_op(self.normal.dot(*n) * d > 0.0)
    }

    /// The boolean chosen, `auto_union` saying what Auto does.
    fn bool_op(&self, auto_union: bool) -> Option<ductile_brep::ops::boolean::BoolOp> {
        use ductile_brep::ops::boolean::BoolOp;
        match self.op {
            ExtrudeOp::NewBody => None,
            ExtrudeOp::Join => Some(BoolOp::Union),
            ExtrudeOp::Cut => Some(BoolOp::Difference),
            ExtrudeOp::Intersect => Some(BoolOp::Intersection),
            ExtrudeOp::Auto => Some(if auto_union { BoolOp::Union } else { BoolOp::Difference }),
        }
    }

    /// The object the result replaces, if any.
    fn replaced(&self) -> Option<ObjectId> {
        let on = self.onto.as_ref().filter(|_| self.on_solid_op(self.value()).is_some()).map(|o| o.0);
        self.target.filter(|_| self.op != ExtrudeOp::NewBody).or(on)
    }
}

impl Tool for ExtrudeTool {
    fn name(&self) -> &'static str {
        "Extrude"
    }
    fn hint(&self) -> String {
        match self.bg.latest().and_then(|c| c.1) {
            Some(e) => format!("Extrude failed: {e}"),
            None if self.onto.is_some() || self.body.is_some() => format!(
                "Extrude ({}): move along the normal or Tab to type a distance ({:.4}); Q join, W cut, Shift+E intersect, B new body",
                match self.op {
                    ExtrudeOp::Auto => "auto",
                    ExtrudeOp::NewBody => "new body",
                    ExtrudeOp::Join => "join",
                    ExtrudeOp::Cut => "cut",
                    ExtrudeOp::Intersect => "intersect",
                },
                self.value()
            ),
            None => format!("Extrude: move along the normal or Tab to type a distance ({:.4}), click or Enter to finish", self.value()),
        }
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        let t = height_from_ray(self.base, self.normal, &p.ray);
        let t0 = *self.start_t.get_or_insert(t);
        if self.typed.is_none() {
            self.distance = snap_len(t - t0, p.world_per_px);
        }
        if p.clicked {
            return ToolOutcome::Finished;
        }
        ToolOutcome::Continue
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        self.bg.get(&self.key(), || self.job()).and_then(|r| r.0)
    }
    fn busy(&self) -> bool {
        self.bg.busy()
    }
    fn guides(&self, out: &mut Vec<LineInstance>) {
        let tip = self.base + self.normal * self.value();
        out.push(LineInstance::new(self.base, tip, [1.0, 0.85, 0.3, 1.0], 2.0));
        out.push(LineInstance::point(tip, [1.0, 0.85, 0.3, 1.0], 7.0));
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut d = self.value();
        let mut ch = drag_value(ui, "Distance", &mut d, 0.05);
        if ch {
            self.typed = Some(d);
        }
        if self.onto.is_some() || self.body.is_some() {
            ui.horizontal(|ui| {
                ui.label("Solid");
                for (op, name) in [(ExtrudeOp::Auto, "Auto"), (ExtrudeOp::NewBody, "New"), (ExtrudeOp::Join, "Join"), (ExtrudeOp::Cut, "Cut"), (ExtrudeOp::Intersect, "Intersect")] {
                    ch |= ui.selectable_value(&mut self.op, op, name).changed();
                }
            });
        }
        ch
    }
    fn hidden_objects(&self) -> Vec<ObjectId> {
        match self.replaced() {
            Some(t) if self.bg.latest().and_then(|r| r.0).is_some() => vec![t],
            _ => vec![],
        }
    }
    fn typed_value(&mut self, v: Option<f64>) {
        self.typed = v;
    }
    fn command_key(&mut self, key: egui::Key, m: egui::Modifiers) -> bool {
        if self.onto.is_none() && self.body.is_none() || m.ctrl || m.alt {
            return false;
        }
        self.op = match (key, m.shift) {
            (egui::Key::Q, false) => ExtrudeOp::Join,
            (egui::Key::W, false) => ExtrudeOp::Cut,
            (egui::Key::E, true) => ExtrudeOp::Intersect,
            (egui::Key::B, false) => ExtrudeOp::NewBody,
            _ => return false,
        };
        true
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let Some(b) = self.bg.wait(&self.key(), || self.job()).0 else { return false };
        doc.selection.clear();
        match self.replaced() {
            Some(t) => doc.replace_body(t, b),
            None => {
                let id = doc.add_body("Extrusion", b);
                doc.selection.insert(Pick::Object(id));
            }
        }
        true
    }
}

// --------------------------------------------------------------- Curves ----

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum CurveKind {
    Polyline,
    /// Smooth curve through the clicked points.
    Spline,
    /// Arc through start, end and a third point.
    Arc,
    Rectangle,
    Circle,
    Ellipse,
    /// Regular polygon from its center and a corner.
    Polygon,
    /// Rectangle from its center and a corner.
    CenterRectangle,
    /// Rectangle from one side's two ends and the height.
    ThreePointRectangle,
    /// Circle across a diameter.
    TwoPointCircle,
    /// Circle through three points.
    ThreePointCircle,
    /// Arc from its center, start and end.
    CenterArc,
    /// Slot: two centers and a radius.
    Slot,
    /// Arc leaving the end of a curve tangentially (or a direction clicked
    /// after the start), to an end point.
    TangentArc,
}

impl CurveKind {
    /// The kind `C` switches to inside the command, as Plasticity's
    /// center / corner / three-point toggles.
    fn next_variant(self) -> Option<CurveKind> {
        use CurveKind::*;
        Some(match self {
            Rectangle => CenterRectangle,
            CenterRectangle => ThreePointRectangle,
            ThreePointRectangle => Rectangle,
            Circle => TwoPointCircle,
            TwoPointCircle => ThreePointCircle,
            ThreePointCircle => Circle,
            Arc => CenterArc,
            CenterArc => Arc,
            _ => return None,
        })
    }

    /// Clicks that place the whole curve, for kinds placed point by point.
    fn clicks(self) -> Option<usize> {
        use CurveKind::*;
        match self {
            Arc | ThreePointCircle | CenterArc | ThreePointRectangle | Slot => Some(3),
            TwoPointCircle => Some(2),
            _ => None,
        }
    }
}

/// Draws planar curves on the construction plane as wire objects.
pub struct CurveTool {
    kind: CurveKind,
    points: Vec<DVec3>,
    cursor: Option<DVec3>,
    /// Typed size: radius for circles and polygons, width/height for
    /// rectangles, semi-axes for ellipses.
    size: DVec3,
    /// In-plane angle of a polygon's first corner.
    angle: f64,
    sides: usize,
    closed: bool,
    /// Typed value of the point-by-point kinds: a three-point rectangle's
    /// height, a slot's radius, a center arc's sweep in degrees, a
    /// two-point circle's diameter.
    typed: Option<f64>,
    /// A tangent arc's start direction, from the curve it leaves.
    tangent: Option<DVec3>,
    /// Knife (K): the finished curve splits the solid faces it lies on
    /// instead of becoming a curve object.
    pub knife: bool,
}

/// The end of a curve or edge nearest `q` within `tol`, with the
/// direction leaving the curve there.
fn curve_end_near(doc: &Document, q: DVec3, tol: f64) -> Option<(DVec3, DVec3)> {
    let mut best: Option<(f64, DVec3, DVec3)> = None;
    for o in doc.objects.iter().filter(|o| o.visible) {
        for e in &o.body.edges {
            for (t, sign) in [(e.t0, -1.0), (e.t1, 1.0)] {
                let p = e.curve.point(t);
                let d = p.distance(q);
                if d < tol && best.is_none_or(|b| d < b.0) {
                    best = Some((d, p, e.curve.tangent(t) * sign));
                }
            }
        }
    }
    best.map(|b| (b.1, b.2))
}

/// The arc from `p` leaving along `t` to `q` in the plane with normal `z`,
/// or a line when `q` lies straight ahead.
fn tangent_arc(p: DVec3, t: DVec3, q: DVec3, z: DVec3) -> Option<ductile_brep::profile::Segment> {
    use ductile_brep::profile::Segment;
    let t = (t - z * t.dot(z)).try_normalize()?;
    let d = q - p;
    let n = z.cross(t);
    let h = d.dot(n);
    if d.length() < 1e-9 {
        return None;
    }
    if h.abs() < 1e-9 * d.length() {
        return Some(Segment::line(p, q));
    }
    let r = d.length_squared() / (2.0 * h);
    let c = p + n * r;
    let axis = if h > 0.0 { z } else { -z };
    let x = (p - c).normalize();
    let y = axis.cross(x);
    let w = q - c;
    let mut sweep = w.dot(y).atan2(w.dot(x));
    if sweep <= 0.0 {
        sweep += std::f64::consts::TAU;
    }
    Some(Segment::new(ductile_geom::Curve3::Circle { frame: Frame::new(c, axis, x), radius: r.abs() }, 0.0, sweep))
}

impl CurveTool {
    pub fn new(kind: CurveKind) -> Self {
        Self { kind, points: vec![], cursor: None, size: DVec3::ZERO, angle: 0.0, sides: 6, closed: false, typed: None, tangent: None, knife: false }
    }

    /// Points the multi-click kinds pass through, with the cursor as the
    /// next one while the curve is open.
    fn path(&self) -> Vec<DVec3> {
        let mut pts = self.points.clone();
        if !self.closed {
            pts.extend(self.cursor.filter(|c| pts.last().is_none_or(|l| l.distance(*c) > 1e-9)));
        }
        pts
    }

    fn segments(&self) -> Vec<ductile_brep::profile::Segment> {
        use ductile_brep::profile::Segment;
        let f = current_cplane().frame;
        let mut pts = self.points.clone();
        match self.kind {
            CurveKind::Spline => {
                let pts = self.path();
                if self.closed && pts.len() >= 3 {
                    let c = ductile_geom::NurbsCurve::interpolate_closed(&pts);
                    let (t0, t1) = c.domain();
                    return vec![Segment::new(ductile_geom::Curve3::Nurbs(c), t0, t1)];
                }
                if pts.len() < 2 {
                    return vec![];
                }
                let c = ductile_geom::NurbsCurve::interpolate(&pts, 3);
                let (t0, t1) = c.domain();
                vec![Segment::new(ductile_geom::Curve3::Nurbs(c), t0, t1)]
            }
            CurveKind::Arc => {
                let pts = self.path();
                match pts.len() {
                    2 => vec![Segment::line(pts[0], pts[1])],
                    3.. => arc_through(pts[0], pts[1], pts[2]).map(|s| vec![s]).unwrap_or_else(|| vec![Segment::line(pts[0], pts[1])]),
                    _ => vec![],
                }
            }
            CurveKind::Ellipse => {
                let Some(&c) = pts.first() else { return vec![] };
                let (rx, ry) = (self.size.x.abs(), self.size.y.abs());
                if rx < 1e-9 || ry < 1e-9 {
                    return vec![];
                }
                let frame = Frame { origin: c, ..f };
                vec![Segment::new(ductile_geom::Curve3::Ellipse { frame, rx, ry }, 0.0, std::f64::consts::TAU)]
            }
            CurveKind::Polygon => {
                let Some(&c) = pts.first() else { return vec![] };
                if self.size.x < 1e-9 {
                    return vec![];
                }
                let n = self.sides.max(3);
                let q: Vec<DVec3> = (0..n)
                    .map(|i| {
                        let a = self.angle + std::f64::consts::TAU * i as f64 / n as f64;
                        c + (f.x * a.cos() + f.y * a.sin()) * self.size.x
                    })
                    .collect();
                (0..n).map(|i| Segment::line(q[i], q[(i + 1) % n])).collect()
            }
            CurveKind::Polyline => {
                if !self.closed {
                    pts.extend(self.cursor);
                }
                let mut segs: Vec<Segment> = pts.windows(2).filter(|w| w[0].distance(w[1]) > 1e-9).map(|w| Segment::line(w[0], w[1])).collect();
                if self.closed && pts.len() > 2 {
                    segs.push(Segment::line(pts[pts.len() - 1], pts[0]));
                }
                segs
            }
            CurveKind::Rectangle => {
                let Some(&c) = pts.first() else { return vec![] };
                let (w, h) = (self.size.x, self.size.y);
                if w.abs() < 1e-9 || h.abs() < 1e-9 {
                    return vec![];
                }
                let q = [c, c + f.x * w, c + f.x * w + f.y * h, c + f.y * h];
                (0..4).map(|i| Segment::line(q[i], q[(i + 1) % 4])).collect()
            }
            CurveKind::Circle => {
                let Some(&c) = pts.first() else { return vec![] };
                if self.size.x < 1e-9 {
                    return vec![];
                }
                let frame = Frame { origin: c, ..f };
                vec![Segment::new(ductile_geom::Curve3::Circle { frame, radius: self.size.x }, 0.0, std::f64::consts::TAU)]
            }
            CurveKind::CenterRectangle => {
                let Some(&c) = pts.first() else { return vec![] };
                let (w, h) = (self.size.x.abs(), self.size.y.abs());
                if w < 1e-9 || h < 1e-9 {
                    return vec![];
                }
                let q = [c - f.x * w - f.y * h, c + f.x * w - f.y * h, c + f.x * w + f.y * h, c - f.x * w + f.y * h];
                (0..4).map(|i| Segment::line(q[i], q[(i + 1) % 4])).collect()
            }
            _ => self.point_segments(&self.path(), &f),
        }
    }

    /// The curves of the kinds placed point by point, from the points so
    /// far (the cursor last).
    fn point_segments(&self, pts: &[DVec3], f: &Frame) -> Vec<ductile_brep::profile::Segment> {
        use ductile_brep::profile::Segment;
        use ductile_geom::Curve3;
        use std::f64::consts::{PI, TAU};
        let circle = |c: DVec3, x: DVec3, r: f64, sweep: f64| Segment::new(Curve3::Circle { frame: Frame::new(c, f.z, x), radius: r }, 0.0, sweep);
        match (self.kind, pts) {
            (_, [] | [_]) => vec![],
            (CurveKind::TangentArc, [a, rest @ ..]) => {
                let (t, end) = match (self.tangent, rest) {
                    (Some(t), [.., e]) => (t, *e),
                    (None, [dir, e, ..]) => (*dir - *a, *e),
                    (None, [e]) => return vec![Segment::line(*a, *e)],
                    _ => return vec![],
                };
                tangent_arc(*a, t, end, f.z).map(|s| vec![s]).unwrap_or_default()
            }
            (CurveKind::TwoPointCircle, [a, b, ..]) => {
                let r = self.typed.map(|d| d.abs() * 0.5).unwrap_or(a.distance(*b) * 0.5);
                let x = (*b - *a).try_normalize().unwrap_or(f.x);
                if r < 1e-9 {
                    return vec![];
                }
                vec![circle(*a + x * r, -x, r, TAU)]
            }
            (_, [a, b]) => vec![Segment::line(*a, *b)],
            (CurveKind::ThreePointCircle, [a, b, m, ..]) => match arc_through(*a, *b, *m) {
                Some(s) => {
                    let Curve3::Circle { frame, radius } = s.curve else { unreachable!() };
                    vec![Segment::new(Curve3::Circle { frame, radius }, 0.0, TAU)]
                }
                None => vec![Segment::line(*a, *b)],
            },
            (CurveKind::CenterArc, [c, s, e, ..]) => {
                let r = c.distance(*s);
                let x = (*s - *c).try_normalize().unwrap_or(f.x);
                let l = f.dir_to_local(*e - *c);
                let (u, v) = (f.dir_to_local(x), f.dir_to_local(f.z.cross(x)));
                let mut sweep = (l.x * v.x + l.y * v.y).atan2(l.x * u.x + l.y * u.y);
                if sweep <= 0.0 {
                    sweep += TAU;
                }
                let sweep = self.typed.map(|d| d.to_radians().clamp(-TAU, TAU)).unwrap_or(sweep);
                if r < 1e-9 || sweep.abs() < 1e-9 {
                    return vec![];
                }
                if sweep < 0.0 {
                    return vec![Segment::new(Curve3::Circle { frame: Frame::new(*c, -f.z, x), radius: r }, 0.0, -sweep)];
                }
                vec![circle(*c, x, r, sweep)]
            }
            (CurveKind::ThreePointRectangle, [a, b, q, ..]) => {
                let Some(d) = (*b - *a).try_normalize() else { return vec![] };
                let n = f.z.cross(d);
                let h = self.typed.unwrap_or((*q - *a).dot(n));
                if h.abs() < 1e-9 {
                    return vec![Segment::line(*a, *b)];
                }
                let c = [*a, *b, *b + n * h, *a + n * h];
                (0..4).map(|i| Segment::line(c[i], c[(i + 1) % 4])).collect()
            }
            (CurveKind::Slot, [p0, p1, q, ..]) => {
                let Some(d) = (*p1 - *p0).try_normalize() else { return vec![] };
                let n = d.cross(f.z);
                let off = *q - *p0;
                let r = self.typed.map(f64::abs).unwrap_or((off - d * off.dot(d).clamp(0.0, p0.distance(*p1))).length());
                if r < 1e-9 {
                    return vec![Segment::line(*p0, *p1)];
                }
                vec![
                    Segment::line(*p0 + n * r, *p1 + n * r),
                    circle(*p1, n, r, PI),
                    Segment::line(*p1 - n * r, *p0 - n * r),
                    circle(*p0, -n, r, PI),
                ]
            }
            _ => vec![],
        }
    }
}

impl CurveTool {
    fn kind_hint(&self) -> String {
        match (self.kind, self.points.len()) {
            (CurveKind::Polyline, 0) => "Curve: click the first point".into(),
            (CurveKind::Polyline, _) => "Curve: click points; click the first point to close, Enter to finish open".into(),
            (CurveKind::Rectangle, 0) => "Rectangle: click the first corner (C: from the center)".into(),
            (CurveKind::Rectangle, _) => "Rectangle: click the opposite corner or type the size".into(),
            (CurveKind::Circle, 0) => "Circle: click the center (C: two points)".into(),
            (CurveKind::Circle, _) => "Circle: click to set the radius or type it".into(),
            (CurveKind::Spline, 0) => "Spline: click the first point".into(),
            (CurveKind::Spline, _) => "Spline: click points it passes through; click the first point to close, Enter to finish open".into(),
            (CurveKind::Arc, 0) => "Arc: click the start (C: from the center)".into(),
            (CurveKind::Arc, 1) => "Arc: click the end".into(),
            (CurveKind::Arc, _) => "Arc: click a point the arc passes through".into(),
            (CurveKind::Ellipse, 0) => "Ellipse: click the center".into(),
            (CurveKind::Ellipse, _) => "Ellipse: click a corner of its bounding box or type the radii".into(),
            (CurveKind::Polygon, 0) => "Polygon: click the center".into(),
            (CurveKind::Polygon, _) => "Polygon: click a corner or type the radius; set the sides in the panel".into(),
            (CurveKind::CenterRectangle, 0) => "Rectangle from the center: click the center (C: three points)".into(),
            (CurveKind::CenterRectangle, _) => "Rectangle from the center: click a corner or type the half size".into(),
            (CurveKind::ThreePointRectangle, 0) => "Rectangle by three points: click one end of a side (C: from a corner)".into(),
            (CurveKind::ThreePointRectangle, 1) => "Rectangle by three points: click the other end of the side".into(),
            (CurveKind::ThreePointRectangle, _) => "Rectangle by three points: click or type the height".into(),
            (CurveKind::TwoPointCircle, 0) => "Circle by two points: click one end of a diameter (C: three points)".into(),
            (CurveKind::TwoPointCircle, _) => "Circle by two points: click the other end or type the diameter".into(),
            (CurveKind::ThreePointCircle, 0) => "Circle by three points: click the first point (C: from the center)".into(),
            (CurveKind::ThreePointCircle, _) => "Circle by three points: click the next point".into(),
            (CurveKind::CenterArc, 0) => "Arc from the center: click the center (C: three points)".into(),
            (CurveKind::CenterArc, 1) => "Arc from the center: click the start".into(),
            (CurveKind::CenterArc, _) => "Arc from the center: click the end or type the angle".into(),
            (CurveKind::Slot, 0) => "Slot: click the first center".into(),
            (CurveKind::Slot, 1) => "Slot: click the second center".into(),
            (CurveKind::Slot, _) => "Slot: click or type the radius".into(),
            (CurveKind::TangentArc, 0) => "Tangent arc: click the end of a curve to leave it tangentially (or any point, then a direction)".into(),
            (CurveKind::TangentArc, 1) if self.tangent.is_none() => "Tangent arc: click the direction to leave in".into(),
            (CurveKind::TangentArc, _) => "Tangent arc: click the end; D flips the direction".into(),
        }
    }
}

impl Tool for CurveTool {
    fn anchor(&self) -> Option<DVec3> {
        self.points.last().copied()
    }
    fn name(&self) -> &'static str {
        match self.kind {
            CurveKind::Polyline => "Curve",
            CurveKind::Spline => "Spline",
            CurveKind::Arc => "Arc",
            CurveKind::Rectangle => "Rectangle",
            CurveKind::Circle => "Circle",
            CurveKind::Ellipse => "Ellipse",
            CurveKind::Polygon => "Polygon",
            CurveKind::CenterRectangle | CurveKind::ThreePointRectangle => "Rectangle",
            CurveKind::TwoPointCircle | CurveKind::ThreePointCircle => "Circle",
            CurveKind::CenterArc => "Arc",
            CurveKind::Slot => "Slot",
            CurveKind::TangentArc => "Tangent arc",
        }
    }
    fn hint(&self) -> String {
        let h = self.kind_hint();
        if self.knife {
            format!("Knife (K to stop): {h}")
        } else {
            format!("{h} (K: knife)")
        }
    }
    fn pointer(&mut self, _doc: &Document, cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        let pt = match self.points.first() {
            Some(c) => p.point.filter(|s| cplane.frame.dir_to_local(*s - *c).z.abs() < 1e-9).or_else(|| p.ray.hit_plane(*c, cplane.frame.z)),
            None => p.point,
        };
        self.cursor = pt;
        if let (Some(c), Some(q)) = (self.points.first().copied(), pt) {
            match self.kind {
                CurveKind::Rectangle => {
                    let d = cplane.frame.dir_to_local(q - c);
                    self.size = DVec3::new(d.x, d.y, 0.0);
                }
                CurveKind::Circle => self.size.x = snap_len(c.distance(q), p.world_per_px),
                CurveKind::Ellipse => {
                    let d = cplane.frame.dir_to_local(q - c);
                    self.size = DVec3::new(snap_len(d.x.abs(), p.world_per_px), snap_len(d.y.abs(), p.world_per_px), 0.0);
                }
                CurveKind::Polygon => {
                    let d = cplane.frame.dir_to_local(q - c);
                    self.size.x = snap_len(d.truncate().length(), p.world_per_px);
                    self.angle = d.y.atan2(d.x);
                }
                CurveKind::CenterRectangle => {
                    let d = cplane.frame.dir_to_local(q - c);
                    self.size = DVec3::new(d.x, d.y, 0.0);
                }
                _ => {}
            }
        }
        if !p.clicked {
            return ToolOutcome::Continue;
        }
        let Some(mut q) = pt else { return ToolOutcome::Continue };
        if self.kind == CurveKind::TangentArc {
            if self.points.is_empty() {
                if let Some((e, t)) = curve_end_near(_doc, q, p.world_per_px * 10.0) {
                    q = e;
                    self.tangent = Some(t);
                }
            }
            if self.points.last().is_some_and(|l| l.distance(q) < 1e-9) {
                return ToolOutcome::Continue;
            }
            self.points.push(q);
            let n = if self.tangent.is_some() { 2 } else { 3 };
            return if self.points.len() >= n { ToolOutcome::Finished } else { ToolOutcome::Continue };
        }
        if let Some(n) = self.kind.clicks() {
            if self.points.last().is_some_and(|l| l.distance(q) < 1e-9) {
                return ToolOutcome::Continue;
            }
            self.points.push(q);
            // A typed value stands in for the last click.
            let done = self.points.len() >= n || (self.typed.is_some() && self.points.len() + 1 >= n);
            return if done { ToolOutcome::Finished } else { ToolOutcome::Continue };
        }
        match self.kind {
            CurveKind::Polyline | CurveKind::Spline => {
                if self.points.len() > 2 && q.distance(self.points[0]) < p.world_per_px * 8.0 {
                    self.closed = true;
                    return ToolOutcome::Finished;
                }
                if self.points.last().map(|l| l.distance(q) > 1e-9).unwrap_or(true) {
                    self.points.push(q);
                }
                ToolOutcome::Continue
            }
            _ => {
                if self.points.is_empty() {
                    self.points.push(q);
                    ToolOutcome::Continue
                } else {
                    ToolOutcome::Finished
                }
            }
        }
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        let segs = self.segments();
        if segs.is_empty() {
            return None;
        }
        Some(ductile_brep::profile::wire_from_segments(&segs))
    }
    fn guides(&self, out: &mut Vec<LineInstance>) {
        for p in &self.points {
            out.push(LineInstance::point(*p, [1.0, 0.85, 0.3, 1.0], 6.0));
        }
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut ch = false;
        match self.kind {
            CurveKind::Rectangle => {
                ch |= drag_value(ui, "Width", &mut self.size.x, 0.1);
                ch |= drag_value(ui, "Height", &mut self.size.y, 0.1);
            }
            CurveKind::Circle => ch |= drag_value(ui, "Radius", &mut self.size.x, 0.05),
            CurveKind::Ellipse => {
                ch |= drag_value(ui, "Radius X", &mut self.size.x, 0.05);
                ch |= drag_value(ui, "Radius Y", &mut self.size.y, 0.05);
            }
            CurveKind::Polygon => {
                ch |= drag_value(ui, "Radius", &mut self.size.x, 0.05);
                let mut n = self.sides as f64;
                if drag_value(ui, "Sides", &mut n, 0.1) {
                    self.sides = (n.round() as usize).clamp(3, 64);
                    ch = true;
                }
            }
            CurveKind::CenterRectangle => {
                ch |= drag_value(ui, "Half width", &mut self.size.x, 0.1);
                ch |= drag_value(ui, "Half height", &mut self.size.y, 0.1);
            }
            CurveKind::Polyline | CurveKind::Spline | CurveKind::Arc | CurveKind::ThreePointCircle | CurveKind::TangentArc => {
                ui.label(format!("{} points", self.points.len()));
            }
            CurveKind::ThreePointRectangle | CurveKind::Slot | CurveKind::CenterArc | CurveKind::TwoPointCircle => {
                let label = match self.kind {
                    CurveKind::ThreePointRectangle => "Height",
                    CurveKind::Slot => "Radius",
                    CurveKind::CenterArc => "Angle",
                    _ => "Diameter",
                };
                let mut v = self.typed.unwrap_or(0.0);
                if drag_value(ui, label, &mut v, 0.05) {
                    self.typed = Some(v);
                    ch = true;
                }
            }
        }
        ch
    }
    fn command_key(&mut self, key: egui::Key, m: egui::Modifiers) -> bool {
        if key == egui::Key::K && !m.any() {
            self.knife = !self.knife;
            return true;
        }
        if key == egui::Key::D && !m.any() && self.kind == CurveKind::TangentArc {
            if let Some(t) = &mut self.tangent {
                *t = -*t;
                return true;
            }
        }
        if key != egui::Key::C || m.any() {
            return false;
        }
        let Some(next) = self.kind.next_variant() else { return false };
        // Keep the first point; the variants all start with one.
        self.kind = next;
        self.points.truncate(1);
        self.typed = None;
        self.size = DVec3::ZERO;
        true
    }
    fn typed_value(&mut self, v: Option<f64>) {
        if let Some(v) = v {
            match self.kind {
                CurveKind::Circle | CurveKind::Polygon => self.size.x = v.abs(),
                CurveKind::Ellipse => {
                    self.size.x = v.abs();
                    self.size.y = v.abs();
                }
                CurveKind::Rectangle => {
                    self.size.x = v;
                    self.size.y = v;
                }
                CurveKind::CenterRectangle => {
                    self.size.x = v;
                    self.size.y = v;
                }
                CurveKind::ThreePointRectangle | CurveKind::Slot | CurveKind::CenterArc | CurveKind::TwoPointCircle => self.typed = Some(v),
                CurveKind::Polyline | CurveKind::Spline | CurveKind::Arc | CurveKind::ThreePointCircle | CurveKind::TangentArc => {}
            }
        } else {
            self.typed = None;
        }
    }
    fn accept(&mut self) -> ToolOutcome {
        if matches!(self.kind, CurveKind::Polyline | CurveKind::Spline) {
            // Enter ends the curve at the last clicked point.
            self.cursor = None;
        }
        ToolOutcome::Finished
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        if matches!(self.kind, CurveKind::Polyline | CurveKind::Spline) && !self.closed {
            self.cursor = None;
        }
        let segs = self.segments();
        if segs.is_empty() {
            return false;
        }
        if self.knife && knife_objects(doc, &segs) > 0 {
            return true;
        }
        let id = doc.add_body(self.name(), ductile_brep::profile::wire_from_segments(&segs));
        doc.selection.clear();
        doc.selection.insert(Pick::Object(id));
        true
    }
}

/// Knife: splits the faces of the visible solids that the curve lies on.
/// How many solids it cut.
fn knife_objects(doc: &mut Document, segs: &[ductile_brep::profile::Segment]) -> usize {
    let chain = ductile_brep::profile::Loop3 { segments: segs.to_vec() };
    let n = chain.normal();
    let dir = if n.length() > 1e-9 && ductile_brep::profile::loops_plane(std::slice::from_ref(&chain)).is_some() { n.normalize() } else { current_cplane().frame.z };
    let pts: Vec<DVec3> = segs.iter().flat_map(|s| s.sample(1e-3)).collect();
    let ids: Vec<ObjectId> = doc
        .objects
        .iter()
        .filter(|o| o.visible && !doc.is_locked(o.id) && o.body.kind == ductile_brep::BodyKind::Solid && pts.iter().any(|p| o.body.bbox().expanded(1e-6).contains(*p)))
        .map(|o| o.id)
        .collect();
    let mut cut = 0;
    for id in ids {
        let b = doc.object(id).unwrap().body.clone();
        if let Ok(k) = ductile_brep::ops::cut::knife(&b, std::slice::from_ref(&chain), dir) {
            if k.faces.len() > b.faces.len() {
                doc.replace_body(id, k);
                cut += 1;
            }
        }
    }
    cut
}

/// Circular arc from `a` to `b` through `m`, or None when they are collinear.
fn arc_through(a: DVec3, b: DVec3, m: DVec3) -> Option<ductile_brep::profile::Segment> {
    // Circumcenter of the triangle a, m, b.
    let (u, v) = (m - a, b - a);
    let n = u.cross(v);
    if n.length_squared() < 1e-18 * u.length_squared().max(1e-30) * v.length_squared().max(1e-30) {
        return None;
    }
    let c = a + (n.cross(u) * v.length_squared() + v.cross(n) * u.length_squared()) / (2.0 * n.length_squared());
    // Counter-clockwise about n goes a -> m -> b.
    let frame = Frame::new(c, n, a - c);
    let l = frame.to_local(b);
    let mut sweep = l.y.atan2(l.x);
    if sweep <= 0.0 {
        sweep += std::f64::consts::TAU;
    }
    Some(ductile_brep::profile::Segment::new(ductile_geom::Curve3::Circle { frame, radius: c.distance(a) }, 0.0, sweep))
}

/// Fillet or chamfer of the selected edges of one solid, with the radius
/// dragged along the edge's bisector.
/// Cross-section of an edge blend.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BlendShape {
    /// Circular, from a rolling ball.
    Round,
    /// Curvature continuous with the faces.
    Smooth,
    Chamfer,
}

pub struct FilletTool {
    target: ObjectId,
    body: std::sync::Arc<Body>,
    edges: Vec<ductile_brep::EdgeId>,
    base: DVec3,
    dir: DVec3,
    start_t: Option<f64>,
    pub radius: f64,
    pub shape: BlendShape,
    /// Radius varying linearly along each edge, to `end_radius` at its end.
    pub variable: bool,
    pub end_radius: f64,
    /// Limit points: blend only between these fractions along the edge
    /// (a single straight edge between flat faces).
    pub limited: bool,
    pub limits: (f64, f64),
    typed: Option<f64>,
    bg: Background<FilletKey, (Option<Body>, Option<String>)>,
}

type FilletKey = (f64, BlendShape, bool, f64, Option<(f64, f64)>);

impl FilletTool {
    /// Starts on the selected edges of the first object that has any.
    pub fn new(doc: &Document, chamfer: bool) -> Option<Self> {
        let sel = doc.selected_edges();
        let target = sel.first()?.0;
        let edges: Vec<_> = sel.iter().filter(|(o, _)| *o == target).map(|(_, e)| *e).collect();
        let o = doc.object(target)?;
        let b = &o.body;
        let ed = b.edge(edges[0]);
        let base = ed.mid();
        // Drag handle: into the material, between the two faces.
        let n: DVec3 = b
            .edge_faces(edges[0])
            .iter()
            .map(|(f, _)| {
                let uv = b.face(*f).surface.project(base, None);
                b.face_normal(*f, uv.x, uv.y)
            })
            .sum();
        let dir = (-n).try_normalize().unwrap_or(DVec3::Z);
        let shape = if chamfer { BlendShape::Chamfer } else { BlendShape::Round };
        Some(Self { target, body: o.body.clone(), edges, base, dir, start_t: None, radius: 0.0, shape, variable: false, end_radius: 1.0, limited: false, limits: (0.25, 0.75), typed: None, bg: Background::default() })
    }

    fn value(&self) -> f64 {
        self.typed.unwrap_or(self.radius)
    }

    fn key(&self) -> FilletKey {
        (self.value(), self.shape, self.variable, self.end_radius, (self.limited && self.edges.len() == 1).then_some(self.limits))
    }

    /// The blend computation for the current values, owning its inputs.
    fn job(&self) -> Box<dyn FnOnce() -> (Option<Body>, Option<String>) + Send> {
        use ductile_brep::ops::fillet::{blend_edge_between, blend_edges, blend_edges_variable, Blend};
        let (body, edges, (r, shape, variable, end_radius, limits)) = (self.body.clone(), self.edges.clone(), self.key());
        Box::new(move || {
            if r <= 1e-9 {
                (None, None)
            } else if let Some((s0, s1)) = limits {
                let blend = match shape {
                    BlendShape::Round => Blend::Fillet(r),
                    BlendShape::Smooth => Blend::Smooth(r),
                    BlendShape::Chamfer => Blend::Chamfer(r),
                };
                match blend_edge_between(&body, edges[0], blend, s0, s1) {
                    Ok(b) => (Some(b), None),
                    Err(e) => (None, Some(e.to_string())),
                }
            } else if variable && shape != BlendShape::Chamfer {
                match blend_edges_variable(&body, &edges, r, end_radius.max(1e-6), shape == BlendShape::Smooth) {
                    Ok(b) => (Some(b), None),
                    Err(e) => (None, Some(e.to_string())),
                }
            } else {
                let blend = match shape {
                    BlendShape::Round => Blend::Fillet(r),
                    BlendShape::Smooth => Blend::Smooth(r),
                    BlendShape::Chamfer => Blend::Chamfer(r),
                };
                match blend_edges(&body, &edges, blend) {
                    Ok(b) => (Some(b), None),
                    Err(e) => (None, Some(e.to_string())),
                }
            }
        })
    }
}

impl Tool for FilletTool {
    fn name(&self) -> &'static str {
        if self.shape == BlendShape::Chamfer {
            "Chamfer"
        } else {
            "Fillet"
        }
    }
    fn hint(&self) -> String {
        match self.bg.latest().and_then(|c| c.1) {
            Some(e) => format!("{} failed: {e}", self.name()),
            None => format!("{}: drag or type a {} ({:.4}), click or Enter to finish", self.name(), if self.shape == BlendShape::Chamfer { "distance" } else { "radius" }, self.value()),
        }
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        let t = height_from_ray(self.base, self.dir, &p.ray);
        let t0 = *self.start_t.get_or_insert(t);
        if self.typed.is_none() {
            self.radius = snap_len((t - t0).max(0.0), p.world_per_px);
        }
        if p.clicked {
            return ToolOutcome::Finished;
        }
        ToolOutcome::Continue
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        self.bg.get(&self.key(), || self.job()).and_then(|r| r.0)
    }
    fn busy(&self) -> bool {
        self.bg.busy()
    }
    fn guides(&self, out: &mut Vec<LineInstance>) {
        let tip = self.base + self.dir * self.value();
        out.push(LineInstance::new(self.base, tip, [1.0, 0.85, 0.3, 1.0], 2.0));
        out.push(LineInstance::point(tip, [1.0, 0.85, 0.3, 1.0], 7.0));
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut r = self.value();
        let mut ch = drag_value(ui, if self.shape == BlendShape::Chamfer { "Distance" } else { "Radius" }, &mut r, 0.01);
        if ch {
            self.typed = Some(r.max(0.0));
        }
        ui.horizontal(|ui| {
            for (shape, label) in [(BlendShape::Round, "Round"), (BlendShape::Smooth, "Smooth (G2)"), (BlendShape::Chamfer, "Chamfer")] {
                ch |= ui.radio_value(&mut self.shape, shape, label).changed();
            }
        });
        if self.shape != BlendShape::Chamfer {
            ch |= ui.checkbox(&mut self.variable, "Variable radius (straight edges)").changed();
            if self.variable {
                let mut e = self.end_radius;
                if drag_value(ui, "End radius", &mut e, 0.01) {
                    self.end_radius = e.max(1e-4);
                    ch = true;
                }
            }
        }
        if self.edges.len() == 1 {
            ch |= ui.checkbox(&mut self.limited, "Limit points (L)").changed();
            if self.limited {
                ui.horizontal(|ui| {
                    ch |= ui.add(egui::DragValue::new(&mut self.limits.0).speed(0.005).range(0.0..=1.0).prefix("From ")).changed();
                    ch |= ui.add(egui::DragValue::new(&mut self.limits.1).speed(0.005).range(0.0..=1.0).prefix("To ")).changed();
                });
            }
        }
        ch
    }
    fn command_key(&mut self, key: egui::Key, m: egui::Modifiers) -> bool {
        if key == egui::Key::L && !m.ctrl && !m.alt && self.edges.len() == 1 {
            self.limited = !self.limited;
            return true;
        }
        false
    }
    fn hidden_objects(&self) -> Vec<ObjectId> {
        if self.bg.latest().and_then(|r| r.0).is_some() {
            vec![self.target]
        } else {
            vec![]
        }
    }
    fn typed_value(&mut self, v: Option<f64>) {
        self.typed = v.map(|x| x.abs());
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let Some(b) = self.bg.wait(&self.key(), || self.job()).0 else { return false };
        doc.replace_body(self.target, b);
        doc.selection.clear();
        true
    }
}

// ----------------------------------------------------------------- Pipe ----

/// A round tube along each selected curve chain, its radius dragged out
/// from the start of the first.
pub struct PipeTool {
    paths: Vec<ductile_brep::profile::Loop3>,
    base: DVec3,
    dir: DVec3,
    pub radius: f64,
    typed: Option<f64>,
    bg: Background<f64, (Option<Body>, Option<String>)>,
}

impl PipeTool {
    pub fn new(doc: &Document) -> Option<Self> {
        let paths = selected_chains(doc);
        let s0 = paths.first()?.segments.first()?;
        let t = s0.tangent_at(if s0.reversed { s0.t1 } else { s0.t0 }).normalize_or_zero();
        // Handle across the path, in the construction plane where it can be.
        let z = current_cplane().frame.z;
        let dir = t.cross(z).try_normalize().unwrap_or_else(|| t.any_orthonormal_vector());
        let base = s0.start();
        Some(Self { paths, base, dir, radius: 0.0, typed: None, bg: Background::default() })
    }

    fn value(&self) -> f64 {
        self.typed.unwrap_or(self.radius)
    }

    fn job(&self) -> Box<dyn FnOnce() -> (Option<Body>, Option<String>) + Send> {
        let (paths, r) = (self.paths.clone(), self.value());
        Box::new(move || {
            if r <= 1e-9 {
                return (None, None);
            }
            let mut out: Option<Body> = None;
            for p in &paths {
                let Some(b) = ductile_brep::ops::sweep::pipe(p, r) else { return (None, Some("the tube intersects itself or the curve turns too sharply".into())) };
                match &mut out {
                    None => out = Some(b),
                    Some(o) => o.append(&b),
                }
            }
            (out, None)
        })
    }
}

impl Tool for PipeTool {
    fn name(&self) -> &'static str {
        "Pipe"
    }
    fn hint(&self) -> String {
        match self.bg.latest().and_then(|c| c.1) {
            Some(e) => format!("Pipe failed: {e}"),
            None => format!("Pipe: drag or type a radius ({:.4}), click or Enter to finish", self.value()),
        }
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        if self.typed.is_none() {
            self.radius = snap_len(height_from_ray(self.base, self.dir, &p.ray).abs(), p.world_per_px);
        }
        if p.clicked {
            return ToolOutcome::Finished;
        }
        ToolOutcome::Continue
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        self.bg.get(&self.value(), || self.job()).and_then(|r| r.0)
    }
    fn busy(&self) -> bool {
        self.bg.busy()
    }
    fn guides(&self, out: &mut Vec<LineInstance>) {
        let tip = self.base + self.dir * self.value();
        out.push(LineInstance::new(self.base, tip, [1.0, 0.85, 0.3, 1.0], 2.0));
        out.push(LineInstance::point(tip, [1.0, 0.85, 0.3, 1.0], 7.0));
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut v = self.value();
        let ch = drag_value(ui, "Radius", &mut v, 0.01);
        if ch {
            self.typed = Some(v.max(0.0));
        }
        ch
    }
    fn typed_value(&mut self, v: Option<f64>) {
        self.typed = v.map(|x| x.abs());
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let r = self.value();
        let mut any = false;
        for p in self.paths.clone() {
            if let Some(b) = ductile_brep::ops::sweep::pipe(&p, r) {
                doc.add_body("Pipe", b);
                any = true;
            }
        }
        any
    }
}

// ---------------------------------------------------------------- Shell ----

/// What the shell tool does with its distance.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OffsetMode {
    /// Hollow the solid, opening the selected faces.
    Shell,
    /// Move every face.
    Solid,
    /// Move the selected faces; their neighbours stretch to follow.
    Faces,
}

/// Shell (hollow) a solid with walls of a given thickness, opening the
/// selected faces; or offset the whole solid, or just the selected faces,
/// by a signed distance.
pub struct ShellTool {
    target: ObjectId,
    body: std::sync::Arc<Body>,
    open: Vec<ductile_brep::FaceId>,
    base: DVec3,
    dir: DVec3,
    start_t: Option<f64>,
    pub value: f64,
    pub mode: OffsetMode,
    typed: Option<f64>,
    bg: Background<(f64, OffsetMode), (Option<Body>, Option<String>)>,
}

impl ShellTool {
    /// Starts on the selected faces' solid, or on the first selected solid.
    pub fn new(doc: &Document, mode: OffsetMode) -> Option<Self> {
        let offset = mode != OffsetMode::Shell;
        let faces = doc.selected_faces();
        let (target, open) = match faces.first() {
            Some((o, _)) => (*o, faces.iter().filter(|(x, _)| x == o).map(|(_, f)| *f).collect::<Vec<_>>()),
            None => (*doc.selected_solids().first()?, vec![]),
        };
        if mode == OffsetMode::Faces && open.is_empty() {
            return None;
        }
        let o = doc.object(target)?;
        let b = &o.body;
        if b.kind == ductile_brep::BodyKind::Wire {
            return None;
        }
        // Drag handle on a face, along its normal: into the material for a
        // shell, outward for an offset.
        let f = open.first().copied().unwrap_or(ductile_brep::FaceId(0));
        let base = b.face_edges(f).first().map(|e| b.edge(*e).mid()).unwrap_or(DVec3::ZERO);
        let uv = b.face(f).surface.project(base, None);
        let n = b.face_normal(f, uv.x, uv.y);
        let dir = if offset { n } else { -n };
        Some(Self { target, body: o.body.clone(), open: if mode == OffsetMode::Solid { vec![] } else { open }, base, dir, start_t: None, value: 0.0, mode, typed: None, bg: Background::default() })
    }

    fn amount(&self) -> f64 {
        self.typed.unwrap_or(self.value)
    }

    fn key(&self) -> (f64, OffsetMode) {
        (self.amount(), self.mode)
    }

    fn job(&self) -> Box<dyn FnOnce() -> (Option<Body>, Option<String>) + Send> {
        use ductile_brep::ops::offset::{offset_body, shell};
        let (body, open, (t, mode)) = (self.body.clone(), self.open.clone(), self.key());
        Box::new(move || {
            if t.abs() <= 1e-9 || (mode == OffsetMode::Shell && t < 0.0) {
                return (None, None);
            }
            let r = match mode {
                OffsetMode::Shell => shell(&body, &open, t),
                OffsetMode::Solid => offset_body(&body, &|_| t),
                OffsetMode::Faces => offset_body(&body, &|f| if open.contains(&f) { t } else { 0.0 }),
            };
            match r {
                Ok(b) => (Some(b), None),
                Err(e) => (None, Some(e.to_string())),
            }
        })
    }
}

impl Tool for ShellTool {
    fn name(&self) -> &'static str {
        match self.mode {
            OffsetMode::Shell => "Shell",
            OffsetMode::Solid => "Offset solid",
            OffsetMode::Faces => "Offset faces",
        }
    }
    fn hint(&self) -> String {
        match self.bg.latest().and_then(|c| c.1) {
            Some(e) => format!("{} failed: {e}", self.name()),
            None => format!("{}: drag or type a {} ({:.4}), click or Enter to finish", self.name(), if self.mode != OffsetMode::Shell { "distance" } else { "thickness" }, self.amount()),
        }
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        let t = height_from_ray(self.base, self.dir, &p.ray);
        let t0 = *self.start_t.get_or_insert(t);
        if self.typed.is_none() {
            let d = t - t0;
            self.value = if self.mode != OffsetMode::Shell { snap_len(d.abs(), p.world_per_px).copysign(d) } else { snap_len(d.max(0.0), p.world_per_px) };
        }
        if p.clicked {
            return ToolOutcome::Finished;
        }
        ToolOutcome::Continue
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        self.bg.get(&self.key(), || self.job()).and_then(|r| r.0)
    }
    fn busy(&self) -> bool {
        self.bg.busy()
    }
    fn guides(&self, out: &mut Vec<LineInstance>) {
        let tip = self.base + self.dir * self.amount();
        out.push(LineInstance::new(self.base, tip, [1.0, 0.85, 0.3, 1.0], 2.0));
        out.push(LineInstance::point(tip, [1.0, 0.85, 0.3, 1.0], 7.0));
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut t = self.amount();
        let ch = drag_value(ui, if self.mode != OffsetMode::Shell { "Distance" } else { "Thickness" }, &mut t, 0.01);
        if ch {
            self.typed = Some(if self.mode != OffsetMode::Shell { t } else { t.max(0.0) });
        }
        ch
    }
    fn hidden_objects(&self) -> Vec<ObjectId> {
        if self.bg.latest().and_then(|r| r.0).is_some() {
            vec![self.target]
        } else {
            vec![]
        }
    }
    fn typed_value(&mut self, v: Option<f64>) {
        self.typed = v.map(|x| if self.mode != OffsetMode::Shell { x } else { x.abs() });
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let Some(b) = self.bg.wait(&self.key(), || self.job()).0 else { return false };
        doc.replace_body(self.target, b);
        doc.selection.clear();
        true
    }
}

// ---------------------------------------------------------- Curve edits ----

/// Rounds the corners of the selected curves (only the selected vertices'
/// corners when any are picked), or offsets the curves in their plane.
pub struct CurveEditTool {
    targets: Vec<(ObjectId, std::sync::Arc<Body>)>,
    corners: Vec<DVec3>,
    pub offset: bool,
    pub value: f64,
    base: DVec3,
    dir: DVec3,
    start_t: Option<f64>,
    typed: Option<f64>,
}

impl CurveEditTool {
    pub fn new(doc: &Document, offset: bool) -> Option<Self> {
        let targets: Vec<_> = doc
            .selected_objects()
            .into_iter()
            .filter_map(|id| doc.object(id).filter(|o| o.body.kind == ductile_brep::BodyKind::Wire).map(|o| (id, o.body.clone())))
            .collect();
        if targets.is_empty() {
            return None;
        }
        let corners: Vec<DVec3> = doc.selection.iter().filter_map(|p| if let Pick::Vertex(o, v) = p { doc.object(*o).map(|ob| ob.body.vertex(*v)) } else { None }).collect();
        let b = &targets[0].1;
        let base = b.vertices.first().map(|v| v.point).unwrap_or(DVec3::ZERO);
        let plane = current_cplane().frame;
        // Drag across the curve, in its plane.
        let e0 = &b.edges[0];
        let t = e0.curve.tangent(e0.t0).normalize_or_zero();
        let dir = plane.z.cross(t).try_normalize().unwrap_or(plane.x);
        Some(Self { targets, corners, offset, value: 0.0, base, dir, start_t: None, typed: None })
    }

    fn amount(&self) -> f64 {
        self.typed.unwrap_or(self.value)
    }

    fn edited(&self, b: &Body) -> Option<Body> {
        use ductile_brep::profile::{loops_plane, wire_chains, wire_from_segments};
        let v = self.amount();
        if v.abs() < 1e-9 {
            return None;
        }
        let chains = wire_chains(b);
        let normal = loops_plane(&chains).map(|f| f.z).unwrap_or(current_cplane().frame.z);
        let mut segs = Vec::new();
        for c in &chains {
            let closed = c.is_closed();
            let out = if self.offset {
                // Positive grows closed curves: offset to the right of a
                // counter-clockwise loop.
                let ccw = closed && c.normal().dot(normal) > 0.0;
                let d = if ccw { -v } else { v };
                ductile_brep::curves::offset_chain(&c.segments, closed, d, normal)
            } else {
                let at = (!self.corners.is_empty()).then_some(self.corners.as_slice());
                ductile_brep::curves::fillet_corners(&c.segments, closed, v.abs(), at)
            };
            if out.is_empty() {
                return None;
            }
            segs.extend(out);
        }
        Some(wire_from_segments(&segs))
    }
}

impl Tool for CurveEditTool {
    fn name(&self) -> &'static str {
        if self.offset {
            "Offset curve"
        } else {
            "Fillet curve"
        }
    }
    fn hint(&self) -> String {
        format!("{}: drag or type a {} ({:.4}), click or Enter to finish", self.name(), if self.offset { "distance" } else { "radius" }, self.amount())
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        if self.typed.is_none() {
            let t = height_from_ray(self.base, self.dir, &p.ray);
            let t0 = *self.start_t.get_or_insert(t);
            let d = t - t0;
            self.value = if self.offset { snap_len(d.abs(), p.world_per_px).copysign(d) } else { snap_len(d.abs(), p.world_per_px) };
        }
        if p.clicked {
            return ToolOutcome::Finished;
        }
        ToolOutcome::Continue
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        let mut out: Option<Body> = None;
        for (_, b) in &self.targets {
            if let Some(e) = self.edited(b) {
                match &mut out {
                    None => out = Some(e),
                    Some(o) => o.append(&e),
                }
            }
        }
        out
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut v = self.amount();
        let ch = drag_value(ui, if self.offset { "Distance" } else { "Radius" }, &mut v, 0.01);
        if ch {
            self.typed = Some(v);
        }
        ch
    }
    fn hidden_objects(&self) -> Vec<ObjectId> {
        // An offset adds a curve; a fillet replaces it.
        if self.offset {
            vec![]
        } else {
            self.targets.iter().map(|t| t.0).collect()
        }
    }
    fn typed_value(&mut self, v: Option<f64>) {
        self.typed = v;
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let mut any = false;
        for (id, b) in self.targets.clone() {
            let Some(e) = self.edited(&b) else { continue };
            any = true;
            if self.offset {
                let name = doc.object(id).map(|o| o.name.clone()).unwrap_or_else(|| "Curve".into());
                doc.add_body(&name, e);
            } else {
                doc.replace_body(id, e);
            }
        }
        any
    }
}

// --------------------------------------------------------- Bridge/Extend ----

/// An open end of a curve chain in a wire object.
#[derive(Clone)]
struct ChainEnd {
    object: ObjectId,
    chain: usize,
    at_end: bool,
    /// Point and outward derivatives.
    d: [DVec3; 4],
}

/// Open chain ends of the selected curve objects; only those at selected
/// vertices when any are selected.
fn selected_chain_ends(doc: &Document) -> (Vec<(ObjectId, std::sync::Arc<Body>)>, Vec<ChainEnd>) {
    use ductile_brep::profile::wire_chains;
    let targets: Vec<_> = doc
        .selected_objects()
        .into_iter()
        .filter_map(|id| doc.object(id).filter(|o| o.body.kind == ductile_brep::BodyKind::Wire).map(|o| (id, o.body.clone())))
        .collect();
    let picked: Vec<DVec3> = doc.selection.iter().filter_map(|p| if let Pick::Vertex(o, v) = p { doc.object(*o).map(|ob| ob.body.vertex(*v)) } else { None }).collect();
    let mut ends = Vec::new();
    for (id, b) in &targets {
        for (ci, c) in wire_chains(b).iter().enumerate() {
            if c.is_closed() || c.segments.is_empty() {
                continue;
            }
            for at_end in [false, true] {
                let seg = if at_end { c.segments.last().unwrap() } else { &c.segments[0] };
                let d = ductile_brep::curves::curve_end(seg, at_end);
                if picked.is_empty() || picked.iter().any(|p| p.distance(d[0]) < 1e-6) {
                    ends.push(ChainEnd { object: *id, chain: ci, at_end, d });
                }
            }
        }
    }
    (targets, ends)
}

/// Bridge curves: a new curve joining two open curve ends with G0 to G3
/// continuity.
pub struct BridgeTool {
    ends: [ChainEnd; 2],
    pub continuity: usize,
    pub tension: [f64; 2],
}

impl BridgeTool {
    pub fn new(doc: &Document) -> Option<Self> {
        let (_, ends) = selected_chain_ends(doc);
        // The closest pair of ends on different chains.
        let mut best: Option<(f64, usize, usize)> = None;
        for i in 0..ends.len() {
            for j in i + 1..ends.len() {
                if ends[i].object == ends[j].object && ends[i].chain == ends[j].chain && ends.len() > 2 {
                    continue;
                }
                let d = ends[i].d[0].distance(ends[j].d[0]);
                if d > 1e-9 && best.is_none_or(|b| d < b.0) {
                    best = Some((d, i, j));
                }
            }
        }
        let (_, i, j) = best?;
        Some(Self { ends: [ends[i].clone(), ends[j].clone()], continuity: 2, tension: [1.0, 1.0] })
    }

    fn curve(&self) -> Body {
        let c = ductile_brep::curves::bridge_curve(&self.ends[0].d, &self.ends[1].d, self.continuity, self.tension);
        let (t0, t1) = c.domain();
        ductile_brep::profile::wire_from_segments(&[ductile_brep::profile::Segment::new(ductile_geom::Curve3::Nurbs(c), t0, t1)])
    }
}

impl Tool for BridgeTool {
    fn name(&self) -> &'static str {
        "Bridge curves"
    }
    fn hint(&self) -> String {
        format!("Bridge curves: G{} continuity, tension {:.2}; set them in the panel, click or Enter to finish", self.continuity, self.tension[0])
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        if p.clicked {
            return ToolOutcome::Finished;
        }
        ToolOutcome::Continue
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        Some(self.curve())
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut ch = ui
            .horizontal(|ui| {
                let mut ch = false;
                for (k, label) in ["G0", "G1", "G2", "G3"].into_iter().enumerate() {
                    ch |= ui.radio_value(&mut self.continuity, k, label).changed();
                }
                ch
            })
            .inner;
        if self.continuity > 0 {
            ch |= drag_value(ui, "Tension start", &mut self.tension[0], 0.01);
            ch |= drag_value(ui, "Tension end", &mut self.tension[1], 0.01);
            self.tension = self.tension.map(|t| t.clamp(0.05, 10.0));
        }
        ch
    }
    fn typed_value(&mut self, v: Option<f64>) {
        if let Some(v) = v {
            self.tension = [v.clamp(0.05, 10.0); 2];
        }
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        doc.add_body("Bridge", self.curve());
        true
    }
}

/// Extend curve: lengthen open curves past their ends, along their own
/// shape or straight along the end tangent.
pub struct ExtendTool {
    targets: Vec<(ObjectId, std::sync::Arc<Body>)>,
    ends: Vec<ChainEnd>,
    pub natural: bool,
    pub value: f64,
    start_t: Option<f64>,
    typed: Option<f64>,
}

impl ExtendTool {
    pub fn new(doc: &Document) -> Option<Self> {
        let (targets, ends) = selected_chain_ends(doc);
        if ends.is_empty() {
            return None;
        }
        Some(Self { targets, ends, natural: true, value: 0.0, start_t: None, typed: None })
    }

    fn amount(&self) -> f64 {
        self.typed.unwrap_or(self.value).max(0.0)
    }

    fn extended(&self, id: ObjectId, b: &Body) -> Option<Body> {
        use ductile_brep::curves::{extend_segment, Extension};
        use ductile_brep::profile::{wire_chains, wire_from_segments};
        let len = self.amount();
        let mine: Vec<&ChainEnd> = self.ends.iter().filter(|e| e.object == id).collect();
        if len < 1e-9 || mine.is_empty() {
            return None;
        }
        let how = if self.natural { Extension::Natural } else { Extension::Linear };
        let mut segs = Vec::new();
        for (ci, c) in wire_chains(b).into_iter().enumerate() {
            let mut s = c.segments;
            for e in mine.iter().filter(|e| e.chain == ci) {
                if e.at_end {
                    let last = s.pop()?;
                    s.extend(extend_segment(&last, true, len, how));
                } else {
                    let first = s.remove(0);
                    let mut out = extend_segment(&first, false, len, how);
                    out.extend(s);
                    s = out;
                }
            }
            segs.extend(s);
        }
        Some(wire_from_segments(&segs))
    }
}

impl Tool for ExtendTool {
    fn name(&self) -> &'static str {
        "Extend curve"
    }
    fn hint(&self) -> String {
        format!("Extend curve: drag or type a length ({:.4}), click or Enter to finish", self.amount())
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        if self.typed.is_none() {
            let e = &self.ends[0];
            let t = height_from_ray(e.d[0], e.d[1].normalize_or_zero(), &p.ray);
            let t0 = *self.start_t.get_or_insert(t);
            self.value = snap_len((t - t0).max(0.0), p.world_per_px);
        }
        if p.clicked {
            return ToolOutcome::Finished;
        }
        ToolOutcome::Continue
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        let mut out: Option<Body> = None;
        for (id, b) in &self.targets {
            if let Some(e) = self.extended(*id, b) {
                match &mut out {
                    None => out = Some(e),
                    Some(o) => o.append(&e),
                }
            }
        }
        out
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut v = self.amount();
        let mut ch = drag_value(ui, "Length", &mut v, 0.01);
        if ch {
            self.typed = Some(v);
        }
        ch |= ui.horizontal(|ui| ui.radio_value(&mut self.natural, true, "Natural").changed() | ui.radio_value(&mut self.natural, false, "Linear").changed()).inner;
        ch
    }
    fn hidden_objects(&self) -> Vec<ObjectId> {
        self.targets.iter().map(|t| t.0).filter(|id| self.ends.iter().any(|e| e.object == *id)).collect()
    }
    fn typed_value(&mut self, v: Option<f64>) {
        self.typed = v;
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let mut any = false;
        for (id, b) in self.targets.clone() {
            if let Some(e) = self.extended(id, &b) {
                doc.replace_body(id, e);
                any = true;
            }
        }
        any
    }
}

// ------------------------------------------------------- Sheet edges ----

/// Extend sheet: grows surfaces past their selected open edges.
pub struct ExtendSheetTool {
    targets: Vec<(ObjectId, std::sync::Arc<Body>, ductile_brep::EdgeId)>,
    base: DVec3,
    dir: DVec3,
    pub natural: bool,
    pub value: f64,
    start_t: Option<f64>,
    typed: Option<f64>,
    cache: std::cell::RefCell<Option<(f64, bool, Vec<Option<Body>>, Option<String>)>>,
}

impl ExtendSheetTool {
    pub fn new(doc: &Document) -> Option<Self> {
        let mut targets: Vec<(ObjectId, std::sync::Arc<Body>, ductile_brep::EdgeId)> = Vec::new();
        for (o, e) in doc.selected_edges() {
            let ob = doc.object(o)?;
            if ob.body.kind != ductile_brep::BodyKind::Sheet || ob.body.edge_faces(e).len() != 1 || targets.iter().any(|t| t.0 == o) {
                continue;
            }
            targets.push((o, ob.body.clone(), e));
        }
        let (_, b, e) = targets.first()?;
        // Drag across the first edge, away from its face.
        let ed = b.edge(*e);
        let (f, c) = b.edge_faces(*e)[0];
        let tm = 0.5 * (ed.t0 + ed.t1);
        let base = ed.curve.point(tm);
        let uv = b.face(f).surface.project(base, None);
        let n = b.face_normal(f, uv.x, uv.y);
        let along = ed.curve.tangent(tm) * if c.reversed { -1.0 } else { 1.0 };
        let dir = along.cross(n).try_normalize()?;
        Some(Self { targets, base, dir, natural: true, value: 0.0, start_t: None, typed: None, cache: Default::default() })
    }

    fn amount(&self) -> f64 {
        self.typed.unwrap_or(self.value).max(0.0)
    }

    fn results(&self) -> (Vec<Option<Body>>, Option<String>) {
        use ductile_brep::curves::Extension;
        let (len, nat) = (self.amount(), self.natural);
        if let Some((cl, cn, r, e)) = &*self.cache.borrow() {
            if *cl == len && *cn == nat {
                return (r.clone(), e.clone());
            }
        }
        let how = if nat { Extension::Natural } else { Extension::Linear };
        let mut err = None;
        let res = self
            .targets
            .iter()
            .map(|(_, b, e)| {
                if len < 1e-9 {
                    return None;
                }
                ductile_brep::ops::extend::extend_sheet(b, *e, len, how).map_err(|x| err = Some(x)).ok()
            })
            .collect::<Vec<_>>();
        *self.cache.borrow_mut() = Some((len, nat, res.clone(), err.clone()));
        (res, err)
    }
}

impl Tool for ExtendSheetTool {
    fn name(&self) -> &'static str {
        "Extend surface"
    }
    fn hint(&self) -> String {
        match self.cache.borrow().as_ref().and_then(|c| c.3.clone()) {
            Some(e) => format!("Extend surface: {e}"),
            None => format!("Extend surface: drag or type a length ({:.4}), click or Enter to finish", self.amount()),
        }
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        if self.typed.is_none() {
            let t = height_from_ray(self.base, self.dir, &p.ray);
            let t0 = *self.start_t.get_or_insert(t);
            self.value = snap_len((t - t0).max(0.0), p.world_per_px);
        }
        if p.clicked {
            return ToolOutcome::Finished;
        }
        ToolOutcome::Continue
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        let mut out: Option<Body> = None;
        for b in self.results().0.into_iter().flatten() {
            match &mut out {
                None => out = Some(b),
                Some(o) => o.append(&b),
            }
        }
        out
    }
    fn guides(&self, out: &mut Vec<LineInstance>) {
        let tip = self.base + self.dir * self.amount();
        out.push(LineInstance::new(self.base, tip, [1.0, 0.85, 0.3, 1.0], 2.0));
        out.push(LineInstance::point(tip, [1.0, 0.85, 0.3, 1.0], 7.0));
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut v = self.amount();
        let mut ch = drag_value(ui, "Length", &mut v, 0.01);
        if ch {
            self.typed = Some(v);
        }
        ch |= ui.horizontal(|ui| ui.radio_value(&mut self.natural, true, "Natural").changed() | ui.radio_value(&mut self.natural, false, "Linear").changed()).inner;
        ch
    }
    fn hidden_objects(&self) -> Vec<ObjectId> {
        if self.amount() < 1e-9 {
            return vec![];
        }
        self.targets.iter().zip(self.results().0).filter(|(_, r)| r.is_some()).map(|(t, _)| t.0).collect()
    }
    fn typed_value(&mut self, v: Option<f64>) {
        self.typed = v;
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let mut any = false;
        let res = self.results().0;
        for ((id, _, _), r) in self.targets.iter().zip(res) {
            if let Some(b) = r {
                doc.replace_body(*id, b);
                any = true;
            }
        }
        any
    }
}

/// The boolean the Boolean command applies.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BooleanKind {
    Union,
    Difference,
    Intersection,
    /// Both the difference and the intersection, as separate solids.
    Slice,
    /// Every closed region the solids (and selected sheets) enclose, each
    /// its own solid.
    Region,
}

/// Boolean (Q): combines the first selected solid with the others, with
/// Plasticity's keys to pick the operation while the result previews.
pub struct BooleanTool {
    target: (ObjectId, std::sync::Arc<Body>),
    tools: Vec<(ObjectId, std::sync::Arc<Body>)>,
    /// Selected sheets, splitting the regions in Region mode.
    sheets: Vec<std::sync::Arc<Body>>,
    pub kind: BooleanKind,
    pub keep_tools: bool,
    cache: std::cell::RefCell<Option<(BooleanKind, Result<Vec<Body>, String>)>>,
}

impl BooleanTool {
    pub fn new(doc: &Document) -> Option<Self> {
        let ids = doc.selected_solids();
        let mut it = ids.iter().filter_map(|id| doc.object(*id).map(|o| (*id, o.body.clone())));
        let target = it.next()?;
        let tools: Vec<_> = it.collect();
        if tools.is_empty() {
            return None;
        }
        let sheets = doc.selected_objects().into_iter().filter_map(|id| doc.object(id)).filter(|o| o.body.kind == ductile_brep::BodyKind::Sheet).map(|o| o.body.clone()).collect();
        Some(Self { target, tools, sheets, kind: BooleanKind::Difference, keep_tools: false, cache: Default::default() })
    }

    fn result(&self) -> Result<Vec<Body>, String> {
        use ductile_brep::ops::boolean::{boolean, BoolOp};
        if let Some((k, r)) = &*self.cache.borrow() {
            if *k == self.kind {
                return r.clone();
            }
        }
        let run = |op: BoolOp| -> Result<Body, String> {
            let mut body = (*self.target.1).clone();
            for (_, t) in &self.tools {
                body = boolean(&body, t, op).map_err(|e| e.to_string())?;
            }
            Ok(body)
        };
        let r = match self.kind {
            BooleanKind::Union => run(BoolOp::Union).map(|b| vec![b]),
            BooleanKind::Difference => run(BoolOp::Difference).map(|b| vec![b]),
            BooleanKind::Intersection => run(BoolOp::Intersection).map(|b| vec![b]),
            BooleanKind::Slice => run(BoolOp::Difference).and_then(|a| Ok(vec![a, run(BoolOp::Intersection)?])),
            BooleanKind::Region => {
                let solids: Vec<Body> = std::iter::once(&self.target).chain(&self.tools).map(|t| (*t.1).clone()).collect();
                let sheets: Vec<Body> = self.sheets.iter().map(|s| (**s).clone()).collect();
                match ductile_brep::ops::cut::regions(&solids, &sheets) {
                    Ok(v) if v.is_empty() => Err("no closed regions".into()),
                    r => r.map_err(|e| e.to_string()),
                }
            }
        };
        *self.cache.borrow_mut() = Some((self.kind, r.clone()));
        r
    }
}

impl Tool for BooleanTool {
    fn name(&self) -> &'static str {
        match self.kind {
            BooleanKind::Union => "Union",
            BooleanKind::Difference => "Subtract",
            BooleanKind::Intersection => "Intersect",
            BooleanKind::Slice => "Slice",
            BooleanKind::Region => "Region",
        }
    }
    fn hint(&self) -> String {
        match self.result() {
            Err(e) => format!("{} failed: {e}", self.name()),
            Ok(_) => format!(
                "Boolean ({}): Q union, W subtract, Shift+E intersect, Shift+Q slice, Shift+R region, T keep tools ({}); Enter to finish",
                self.name().to_lowercase(),
                if self.keep_tools { "on" } else { "off" }
            ),
        }
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        if p.clicked { ToolOutcome::Finished } else { ToolOutcome::Continue }
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        let mut r = self.result().ok()?.into_iter();
        let mut out = r.next()?;
        for b in r {
            out.append(&b);
        }
        Some(out)
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut ch = false;
        ui.horizontal(|ui| {
            for (k, name) in [(BooleanKind::Union, "Union"), (BooleanKind::Difference, "Subtract"), (BooleanKind::Intersection, "Intersect"), (BooleanKind::Slice, "Slice"), (BooleanKind::Region, "Region")] {
                ch |= ui.selectable_value(&mut self.kind, k, name).changed();
            }
        });
        ch |= ui.checkbox(&mut self.keep_tools, "Keep tools").changed();
        ch
    }
    fn command_key(&mut self, key: egui::Key, m: egui::Modifiers) -> bool {
        if m.ctrl || m.alt {
            return false;
        }
        match (key, m.shift) {
            (egui::Key::Q, false) => self.kind = BooleanKind::Union,
            (egui::Key::W, false) => self.kind = BooleanKind::Difference,
            (egui::Key::E, true) => self.kind = BooleanKind::Intersection,
            (egui::Key::Q, true) => self.kind = BooleanKind::Slice,
            (egui::Key::R, true) => self.kind = BooleanKind::Region,
            (egui::Key::T, false) => self.keep_tools = !self.keep_tools,
            _ => return false,
        }
        true
    }
    fn hidden_objects(&self) -> Vec<ObjectId> {
        if self.result().is_err() {
            return vec![];
        }
        let mut v = vec![self.target.0];
        if !self.keep_tools {
            v.extend(self.tools.iter().map(|t| t.0));
        }
        v
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let Ok(mut r) = self.result() else { return false };
        if r.is_empty() {
            return false;
        }
        let first = r.remove(0);
        doc.replace_body(self.target.0, first);
        doc.selection.clear();
        doc.selection.insert(Pick::Object(self.target.0));
        for b in r {
            let id = doc.add_body("Slice", b);
            doc.selection.insert(Pick::Object(id));
        }
        if !self.keep_tools {
            for (id, _) in &self.tools {
                doc.remove(*id);
            }
        }
        true
    }
}

/// Match surface: reshapes a surface along its open edge to land on
/// another edge and continue the face beyond it.
pub struct MatchSurfaceTool {
    /// (object, its body, edge) for the surface to change, then the target.
    picks: [(ObjectId, std::sync::Arc<Body>, ductile_brep::EdgeId); 2],
    pub continuity: ductile_brep::ops::bridge::Continuity,
    pub tension: f64,
    cache: std::cell::RefCell<Option<(String, Result<Body, String>)>>,
}

impl MatchSurfaceTool {
    pub fn new(doc: &Document) -> Option<Self> {
        let edges = doc.selected_edges();
        let [(oa, ea), (ob, eb)] = edges[..] else { return None };
        let pick = |o: ObjectId, e| Some((o, doc.object(o)?.body.clone(), e));
        let (a, b) = (pick(oa, ea)?, pick(ob, eb)?);
        let open = |p: &(ObjectId, std::sync::Arc<Body>, ductile_brep::EdgeId)| p.1.kind == ductile_brep::BodyKind::Sheet && p.1.edge_faces(p.2).len() == 1;
        // Change the newer surface to meet the older one's edge.
        let picks = match (open(&a), open(&b)) {
            (true, true) if oa > ob => [a, b],
            (true, true) => [b, a],
            (true, false) => [a, b],
            (false, true) => [b, a],
            (false, false) => return None,
        };
        Some(Self { picks, continuity: ductile_brep::ops::bridge::Continuity::G2, tension: 1.0, cache: Default::default() })
    }

    fn result(&self) -> Result<Body, String> {
        use ductile_brep::ops::bridge::Side;
        let key = format!("{:?} {} {:?}", self.continuity, self.tension, self.picks[0].0);
        if let Some((k, r)) = &*self.cache.borrow() {
            if *k == key {
                return r.clone();
            }
        }
        let [(_, body, e), (_, tb, te)] = &self.picks;
        let target = Side::of_edge(tb, *te, body.edge(*e).mid(), self.continuity, self.tension);
        // The face beyond the target edge that leans towards the surface.
        let r = ductile_brep::ops::match_face::match_surface(body, *e, &target);
        *self.cache.borrow_mut() = Some((key, r.clone()));
        r
    }
}

impl Tool for MatchSurfaceTool {
    fn name(&self) -> &'static str {
        "Match surface"
    }
    fn hint(&self) -> String {
        match self.result() {
            Err(e) => format!("Match surface: {e}"),
            Ok(_) => "Match surface: pick G0 / G1 / G2 and the tension, Swap to change the other surface; Enter to finish".into(),
        }
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        if p.clicked { ToolOutcome::Finished } else { ToolOutcome::Continue }
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        self.result().ok()
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        use ductile_brep::ops::bridge::Continuity;
        let mut ch = false;
        ui.horizontal(|ui| {
            for (c, name) in [(Continuity::G0, "G0"), (Continuity::G1, "G1"), (Continuity::G2, "G2")] {
                ch |= ui.selectable_value(&mut self.continuity, c, name).changed();
            }
        });
        ch |= drag_value(ui, "Tension", &mut self.tension, 0.01);
        self.tension = self.tension.clamp(0.05, 5.0);
        let other_open = self.picks[1].1.kind == ductile_brep::BodyKind::Sheet && self.picks[1].1.edge_faces(self.picks[1].2).len() == 1;
        if other_open && ui.button("Swap").clicked() {
            self.picks.swap(0, 1);
            ch = true;
        }
        ch
    }
    fn typed_value(&mut self, v: Option<f64>) {
        use ductile_brep::ops::bridge::Continuity;
        self.continuity = match v.map(|v| v.round() as i64) {
            Some(0) => Continuity::G0,
            Some(1) => Continuity::G1,
            Some(2) => Continuity::G2,
            _ => return,
        };
    }
    fn hidden_objects(&self) -> Vec<ObjectId> {
        if self.result().is_ok() { vec![self.picks[0].0] } else { vec![] }
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let Ok(b) = self.result() else { return false };
        doc.replace_body(self.picks[0].0, b);
        true
    }
}

/// A sheet spanning two selected edges, meeting the faces beyond them
/// with the continuity chosen per side.
pub struct BridgeSurfaceTool {
    sides: [ductile_brep::ops::bridge::Side; 2],
    cache: std::cell::RefCell<Option<(Vec<u8>, Option<Body>)>>,
}

impl BridgeSurfaceTool {
    pub fn new(doc: &Document) -> Option<Self> {
        use ductile_brep::ops::bridge::{Continuity, Side};
        let edges = doc.selected_edges();
        let [(oa, ea), (ob, eb)] = edges[..] else { return None };
        let (ba, bb) = (&doc.object(oa)?.body, &doc.object(ob)?.body);
        let (ma, mb) = (ba.edge(ea).mid(), bb.edge(eb).mid());
        Some(Self { sides: [Side::of_edge(ba, ea, mb, Continuity::G2, 1.0), Side::of_edge(bb, eb, ma, Continuity::G2, 1.0)], cache: Default::default() })
    }

    fn key(&self) -> Vec<u8> {
        format!("{:?}", self.sides.iter().map(|s| (s.continuity, s.tension)).collect::<Vec<_>>()).into_bytes()
    }

    fn result(&self) -> Option<Body> {
        let k = self.key();
        if let Some((ck, b)) = &*self.cache.borrow() {
            if *ck == k {
                return b.clone();
            }
        }
        let b = ductile_brep::ops::bridge::bridge_surface(&self.sides[0], &self.sides[1]);
        *self.cache.borrow_mut() = Some((k, b.clone()));
        b
    }
}

impl Tool for BridgeSurfaceTool {
    fn name(&self) -> &'static str {
        "Bridge surface"
    }
    fn hint(&self) -> String {
        "Bridge surface: pick G0 / G1 / G2 and the tension per side; 0 1 2 set both sides; Enter to finish".into()
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        if p.clicked { ToolOutcome::Finished } else { ToolOutcome::Continue }
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        self.result()
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        use ductile_brep::ops::bridge::Continuity;
        let mut ch = false;
        for (i, s) in self.sides.iter_mut().enumerate() {
            ui.horizontal(|ui| {
                ui.label(if i == 0 { "First" } else { "Second" });
                for (c, name) in [(Continuity::G0, "G0"), (Continuity::G1, "G1"), (Continuity::G2, "G2")] {
                    ch |= ui.selectable_value(&mut s.continuity, c, name).changed();
                }
            });
            ch |= drag_value(ui, "Tension", &mut s.tension, 0.01);
            s.tension = s.tension.clamp(0.05, 5.0);
        }
        ch
    }
    fn typed_value(&mut self, v: Option<f64>) {
        use ductile_brep::ops::bridge::Continuity;
        let c = match v.map(|v| v.round() as i64) {
            Some(0) => Continuity::G0,
            Some(1) => Continuity::G1,
            Some(2) => Continuity::G2,
            _ => return,
        };
        for s in &mut self.sides {
            s.continuity = c;
        }
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let Some(b) = self.result() else { return false };
        let id = doc.add_body("Bridge surface", b);
        doc.selection.clear();
        doc.selection.insert(Pick::Object(id));
        true
    }
}

// -------------------------------------------------------------- Thicken ----

/// Thicken: turn the selected surfaces into solids by giving them a
/// thickness, along their normals or (negative) against them.
pub struct ThickenTool {
    targets: Vec<(ObjectId, std::sync::Arc<Body>)>,
    base: DVec3,
    normal: DVec3,
    pub value: f64,
    start_t: Option<f64>,
    typed: Option<f64>,
    cache: std::cell::RefCell<Option<(f64, Vec<Option<Body>>, Option<String>)>>,
}

impl ThickenTool {
    pub fn new(doc: &Document) -> Option<Self> {
        let targets: Vec<_> = doc
            .selected_objects()
            .into_iter()
            .filter_map(|id| doc.object(id).filter(|o| o.body.kind == ductile_brep::BodyKind::Sheet).map(|o| (id, o.body.clone())))
            .collect();
        let b = &targets.first()?.1;
        // Drag along the normal of the first face, from its middle.
        let f = b.face_ids().next()?;
        let mesh = ductile_brep::tessellate(b, &ductile_brep::TessOptions::for_size(b.bbox().diagonal()));
        let fm = mesh.faces.first()?;
        let k = fm.positions.len() / 2;
        let (base, normal) = (*fm.positions.get(k)?, fm.normals.get(k).copied().unwrap_or_else(|| b.face_normal(f, 0.5, 0.5)));
        Some(Self { targets, base, normal, value: 0.0, start_t: None, typed: None, cache: std::cell::RefCell::new(None) })
    }

    fn amount(&self) -> f64 {
        self.typed.unwrap_or(self.value)
    }

    fn results(&self) -> (Vec<Option<Body>>, Option<String>) {
        let t = self.amount();
        if let Some((ct, r, e)) = &*self.cache.borrow() {
            if *ct == t {
                return (r.clone(), e.clone());
            }
        }
        let mut err = None;
        let res: Vec<Option<Body>> = self
            .targets
            .iter()
            .map(|(_, b)| {
                if t.abs() < 1e-9 {
                    return None;
                }
                match ductile_brep::ops::sheet::thicken(b, t) {
                    Ok(s) => Some(s),
                    Err(e) => {
                        err = Some(e.to_string());
                        None
                    }
                }
            })
            .collect();
        *self.cache.borrow_mut() = Some((t, res.clone(), err.clone()));
        (res, err)
    }
}

impl Tool for ThickenTool {
    fn name(&self) -> &'static str {
        "Thicken"
    }
    fn hint(&self) -> String {
        match self.cache.borrow().as_ref().and_then(|c| c.2.clone()) {
            Some(e) => format!("Thicken failed: {e}"),
            None => format!("Thicken: drag or type a thickness ({:.4}), negative for the back side; click or Enter to finish", self.amount()),
        }
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        if self.typed.is_none() {
            let t = height_from_ray(self.base, self.normal, &p.ray);
            let t0 = *self.start_t.get_or_insert(t);
            let d = t - t0;
            self.value = snap_len(d.abs(), p.world_per_px).copysign(d);
        }
        if p.clicked {
            return ToolOutcome::Finished;
        }
        ToolOutcome::Continue
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        let mut out: Option<Body> = None;
        for b in self.results().0.into_iter().flatten() {
            match &mut out {
                None => out = Some(b),
                Some(o) => o.append(&b),
            }
        }
        out
    }
    fn guides(&self, out: &mut Vec<LineInstance>) {
        let tip = self.base + self.normal * self.amount();
        out.push(LineInstance::new(self.base, tip, [1.0, 0.85, 0.3, 1.0], 2.0));
        out.push(LineInstance::point(tip, [1.0, 0.85, 0.3, 1.0], 7.0));
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut v = self.amount();
        let ch = drag_value(ui, "Thickness", &mut v, 0.01);
        if ch {
            self.typed = Some(v);
        }
        ch
    }
    fn hidden_objects(&self) -> Vec<ObjectId> {
        if self.amount().abs() < 1e-9 {
            vec![]
        } else {
            self.targets.iter().map(|t| t.0).collect()
        }
    }
    fn typed_value(&mut self, v: Option<f64>) {
        self.typed = v;
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let (res, _) = self.results();
        let mut any = false;
        for ((id, _), r) in self.targets.clone().into_iter().zip(res) {
            if let Some(b) = r {
                doc.replace_body(id, b);
                any = true;
            }
        }
        any
    }
}

// ---------------------------------------------------------------- Draft ----

/// Tilts the selected planar faces of a solid about the line where they
/// cross a neutral plane square to the construction plane's normal, laid
/// through the lowest point of the faces so their base stays put. Dragging
/// moves the faces' top edge in or out; the angle can be typed in degrees.
pub struct DraftTool {
    id: ObjectId,
    body: std::sync::Arc<Body>,
    faces: Vec<FaceId>,
    pull: DVec3,
    /// Middle of the faces' top, where the drag handle sits.
    top: DVec3,
    normal: DVec3,
    height: f64,
    /// Degrees; positive leans the faces in towards the pull direction.
    pub value: f64,
    start_t: Option<f64>,
    typed: Option<f64>,
    cache: std::cell::RefCell<Option<(f64, Option<Body>)>>,
}

impl DraftTool {
    pub fn new(doc: &Document, pull: DVec3) -> Option<Self> {
        let picked = doc.selected_faces();
        let id = picked.first()?.0;
        let o = doc.object(id)?;
        if o.body.kind != ductile_brep::BodyKind::Solid {
            return None;
        }
        let faces: Vec<FaceId> = picked.iter().filter(|(x, _)| *x == id).map(|(_, f)| *f).collect();
        let b = &o.body;
        let pts: Vec<DVec3> = faces.iter().flat_map(|f| b.face_loops(*f).flat_map(|l| l.coedges.iter()).map(|c| b.vertex(b.coedge_vertices(*c).0)).collect::<Vec<_>>()).collect();
        let lo = pts.iter().map(|p| p.dot(pull)).fold(f64::MAX, f64::min);
        let hi = pts.iter().map(|p| p.dot(pull)).fold(f64::MIN, f64::max);
        let tops: Vec<DVec3> = pts.iter().copied().filter(|p| p.dot(pull) > hi - 1e-9 * (1.0 + hi.abs())).collect();
        let top = tops.iter().copied().sum::<DVec3>() / tops.len().max(1) as f64;
        let normal = b.face_normal(faces[0], 0.5, 0.5);
        let normal = (normal - pull * normal.dot(pull)).try_normalize().unwrap_or(normal);
        Some(Self { id, body: o.body.clone(), faces, pull, top, normal, height: (hi - lo).max(1e-6), value: 0.0, start_t: None, typed: None, cache: std::cell::RefCell::new(None) })
    }

    fn angle(&self) -> f64 {
        self.typed.unwrap_or(self.value)
    }

    fn bottom(&self) -> DVec3 {
        self.top - self.pull * self.height
    }

    fn result(&self) -> Option<Body> {
        let a = self.angle();
        if let Some((ca, r)) = &*self.cache.borrow() {
            if *ca == a {
                return r.clone();
            }
        }
        let r = if a.abs() < 1e-9 { None } else { ductile_brep::ops::draft::draft_faces(&self.body, &self.faces, self.bottom(), self.pull, a.to_radians()) };
        *self.cache.borrow_mut() = Some((a, r.clone()));
        r
    }
}

impl Tool for DraftTool {
    fn name(&self) -> &'static str {
        "Draft"
    }
    fn hint(&self) -> String {
        if self.angle().abs() > 1e-9 && self.result().is_none() {
            return format!("Draft: {:.1}° does not fit; the faces around these must be flat and meet them along straight edges", self.angle());
        }
        format!("Draft: drag the top edge in or out, or type an angle ({:.1}°); click or Enter to finish", self.angle())
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        if self.typed.is_none() {
            let t = height_from_ray(self.top, self.normal, &p.ray);
            let t0 = *self.start_t.get_or_insert(t);
            let deg = (-(t - t0) / self.height).atan().to_degrees();
            self.value = (deg * 2.0).round() / 2.0;
        }
        if p.clicked {
            return ToolOutcome::Finished;
        }
        ToolOutcome::Continue
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        self.result()
    }
    fn guides(&self, out: &mut Vec<LineInstance>) {
        let lo = self.bottom();
        let tip = self.top - self.normal * self.height * self.angle().to_radians().tan();
        out.push(LineInstance::new(lo, self.top, [0.6, 0.6, 0.6, 1.0], 1.0));
        out.push(LineInstance::new(lo, tip, [1.0, 0.85, 0.3, 1.0], 2.0));
        out.push(LineInstance::point(tip, [1.0, 0.85, 0.3, 1.0], 7.0));
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut v = self.angle();
        let ch = drag_value(ui, "Angle (°)", &mut v, 0.1);
        if ch {
            self.typed = Some(v);
        }
        ch
    }
    fn hidden_objects(&self) -> Vec<ObjectId> {
        if self.result().is_some() {
            vec![self.id]
        } else {
            vec![]
        }
    }
    fn typed_value(&mut self, v: Option<f64>) {
        self.typed = v;
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        match self.result() {
            Some(b) => {
                doc.replace_body(self.id, b);
                true
            }
            None => false,
        }
    }
}

// ----------------------------------------------------------------- Trim ----

/// Click pieces of curves, between their crossings with other curves, to
/// delete them. Works on all visible curves.
pub struct TrimTool {
    /// Per curve object, its pieces (None once trimmed away).
    pieces: Vec<(ObjectId, Vec<Option<ductile_brep::profile::Segment>>)>,
    hover: Option<(usize, usize)>,
    trimmed: usize,
}

impl TrimTool {
    pub fn new(doc: &Document) -> Option<Self> {
        let ids: Vec<ObjectId> = doc.objects.iter().filter(|o| o.visible && o.body.kind == ductile_brep::BodyKind::Wire).map(|o| o.id).collect();
        if ids.is_empty() {
            return None;
        }
        let mut segs = Vec::new();
        let mut owner = Vec::new();
        for id in &ids {
            for e in &doc.object(*id)?.body.edges {
                segs.push(ductile_brep::profile::Segment::new(e.curve.clone(), e.t0, e.t1));
                owner.push(*id);
            }
        }
        let parts = ductile_brep::curves::split_at_crossings(&segs);
        let mut pieces: Vec<(ObjectId, Vec<Option<ductile_brep::profile::Segment>>)> = ids.iter().map(|id| (*id, Vec::new())).collect();
        for (o, ps) in owner.into_iter().zip(parts) {
            let slot = pieces.iter_mut().find(|p| p.0 == o).unwrap();
            slot.1.extend(ps.into_iter().map(Some));
        }
        Some(Self { pieces, hover: None, trimmed: 0 })
    }

    fn pick(&self, p: &PointerInfo) -> Option<(usize, usize)> {
        let mut best = (f64::INFINITY, None);
        for (i, (_, ps)) in self.pieces.iter().enumerate() {
            for (k, s) in ps.iter().enumerate() {
                let Some(s) = s else { continue };
                for w in s.sample(1e-3).windows(2) {
                    let d = ray_segment_distance(&p.ray, w[0], w[1]);
                    if d < best.0 {
                        best = (d, Some((i, k)));
                    }
                }
            }
        }
        (best.0 < 8.0 * p.world_per_px).then_some(best.1).flatten()
    }
}

/// Distance from a view ray to a line segment.
fn ray_segment_distance(ray: &Ray, a: DVec3, b: DVec3) -> f64 {
    let mut best = f64::INFINITY;
    for i in 0..=8 {
        let q = a.lerp(b, i as f64 / 8.0);
        let t = (q - ray.origin).dot(ray.dir).max(0.0);
        best = best.min((ray.origin + ray.dir * t).distance(q));
    }
    best
}

impl Tool for TrimTool {
    fn name(&self) -> &'static str {
        "Trim"
    }
    fn hint(&self) -> String {
        format!("Trim: click curve pieces to delete them ({} removed), Enter to finish", self.trimmed)
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        self.hover = self.pick(p);
        if p.clicked {
            if let Some((i, k)) = self.hover.take() {
                self.pieces[i].1[k] = None;
                self.trimmed += 1;
            }
        }
        ToolOutcome::Continue
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        let segs: Vec<_> = self.pieces.iter().flat_map(|(_, ps)| ps.iter().flatten().cloned()).collect();
        (!segs.is_empty()).then(|| ductile_brep::profile::wire_from_segments(&segs))
    }
    fn guides(&self, out: &mut Vec<LineInstance>) {
        if let Some((i, k)) = self.hover {
            if let Some(s) = &self.pieces[i].1[k] {
                for w in s.sample(1e-3).windows(2) {
                    out.push(LineInstance::new(w[0], w[1], [1.0, 0.3, 0.25, 1.0], 4.0));
                }
            }
        }
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        ui.label(format!("{} pieces removed", self.trimmed));
        false
    }
    fn hidden_objects(&self) -> Vec<ObjectId> {
        self.pieces.iter().map(|p| p.0).collect()
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        if self.trimmed == 0 {
            return false;
        }
        for (id, ps) in &self.pieces {
            if ps.iter().all(|s| s.is_some()) {
                continue;
            }
            let segs: Vec<_> = ps.iter().flatten().cloned().collect();
            if segs.is_empty() {
                doc.remove(*id);
            } else {
                doc.replace_body(*id, ductile_brep::profile::wire_from_segments(&segs));
            }
        }
        true
    }
}

// --------------------------------------------------------- Split curve ----

/// Click points on curves to split them there (open curves fall in two,
/// closed ones open up), or with `knot` to add a control point there
/// without changing the shape.
pub struct SplitCurveTool {
    pub knot: bool,
    /// Per curve object, its chains: (segments, closed).
    curves: Vec<(ObjectId, Vec<(Vec<ductile_brep::profile::Segment>, bool)>)>,
    hover: Option<(usize, usize, usize, f64, DVec3)>,
    marks: Vec<DVec3>,
    changed: Vec<bool>,
}

impl SplitCurveTool {
    pub fn new(doc: &Document, knot: bool) -> Option<Self> {
        let curves: Vec<_> = doc
            .objects
            .iter()
            .filter(|o| o.visible && !doc.is_locked(o.id) && o.body.kind == ductile_brep::BodyKind::Wire)
            .map(|o| {
                let chains = ductile_brep::profile::wire_chains(&o.body).into_iter().map(|l| {
                    let closed = l.is_closed();
                    (l.segments, closed)
                });
                (o.id, chains.collect())
            })
            .collect::<Vec<_>>();
        let n = curves.len();
        (n > 0).then(|| Self { knot, curves, hover: None, marks: Vec::new(), changed: vec![false; n] })
    }

    /// The nearest curve point under the pointer: (object, chain, segment, parameter, point).
    fn pick(&self, ray: &Ray, reach: f64) -> Option<(usize, usize, usize, f64, DVec3)> {
        let mut best = (f64::INFINITY, None);
        for (i, (_, chains)) in self.curves.iter().enumerate() {
            for (c, (segs, _)) in chains.iter().enumerate() {
                for (k, s) in segs.iter().enumerate() {
                    let pts = s.curve.tessellate(s.t0, s.t1, 1e-3);
                    for w in pts.windows(2) {
                        let d = ray_segment_distance(ray, w[0].1, w[1].1);
                        if d < best.0 {
                            // Refine: the curve point nearest the ray near this piece.
                            let q = w[0].1.lerp(w[1].1, 0.5);
                            let on_ray = ray.origin + ray.dir * (q - ray.origin).dot(ray.dir).max(0.0);
                            let (lo, hi) = (w[0].0.min(w[1].0), w[0].0.max(w[1].0));
                            let t = s.curve.closest_param_in(on_ray, lo, hi);
                            best = (d, Some((i, c, k, t, s.curve.point(t))));
                        }
                    }
                }
            }
        }
        (best.0 < reach).then_some(best.1).flatten()
    }

    fn apply(&mut self, (i, c, k, t, p): (usize, usize, usize, f64, DVec3)) {
        let chains = &mut self.curves[i].1;
        let (segs, closed) = chains[c].clone();
        if self.knot {
            chains[c].0[k] = ductile_brep::curves::insert_knot_segment(&segs[k], t);
        } else {
            let parts = ductile_brep::curves::split_chain(&segs, closed, k, t);
            chains.remove(c);
            for (j, part) in parts.into_iter().enumerate() {
                chains.insert(c + j, (part, false));
            }
        }
        self.changed[i] = true;
        self.marks.push(p);
    }
}

impl Tool for SplitCurveTool {
    fn name(&self) -> &'static str {
        if self.knot {
            "Insert knot"
        } else {
            "Split curve"
        }
    }
    fn hint(&self) -> String {
        let what = if self.knot { "add a control point (shape kept)" } else { "split it" };
        format!("{}: click a curve where to {what} ({} so far), K to switch, Enter to finish", self.name(), self.marks.len())
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        self.hover = self.pick(&p.ray, 8.0 * p.world_per_px);
        if p.clicked {
            if let Some(h) = self.hover.take() {
                self.apply(h);
            }
        }
        ToolOutcome::Continue
    }
    fn command_key(&mut self, key: egui::Key, m: egui::Modifiers) -> bool {
        if key == egui::Key::K && m.is_none() {
            self.knot = !self.knot;
            return true;
        }
        false
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        let segs: Vec<_> = self.curves.iter().flat_map(|(_, cs)| cs.iter().flat_map(|c| c.0.iter().cloned())).collect();
        (!segs.is_empty()).then(|| ductile_brep::profile::wire_from_segments(&segs))
    }
    fn guides(&self, out: &mut Vec<LineInstance>) {
        let cross = |out: &mut Vec<LineInstance>, p: DVec3, col: [f32; 4]| {
            let r = 0.02 * (1.0 + p.length() * 0.01);
            for d in [DVec3::X, DVec3::Y, DVec3::Z] {
                out.push(LineInstance::new(p - d * r, p + d * r, col, 3.0));
            }
        };
        for p in &self.marks {
            cross(out, *p, [1.0, 0.85, 0.2, 1.0]);
        }
        if let Some(h) = self.hover {
            cross(out, h.4, [1.0, 0.3, 0.25, 1.0]);
        }
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let before = self.knot;
        ui.horizontal(|ui| {
            ui.radio_value(&mut self.knot, false, "Split");
            ui.radio_value(&mut self.knot, true, "Insert knot");
        });
        ui.label(format!("{} done", self.marks.len()));
        before != self.knot
    }
    fn hidden_objects(&self) -> Vec<ObjectId> {
        self.curves.iter().map(|c| c.0).collect()
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        if self.marks.is_empty() {
            return false;
        }
        for (i, (id, chains)) in self.curves.iter().enumerate() {
            if !self.changed[i] {
                continue;
            }
            if self.knot {
                let segs: Vec<_> = chains.iter().flat_map(|c| c.0.iter().cloned()).collect();
                doc.replace_body(*id, ductile_brep::profile::wire_from_segments(&segs));
                continue;
            }
            // The first piece stays in the object; the rest become new curves.
            let name = doc.object(*id).map(|o| o.name.clone()).unwrap_or_else(|| "Curve".into());
            let mut it = chains.iter();
            if let Some(first) = it.next() {
                doc.replace_body(*id, ductile_brep::profile::wire_from_segments(&first.0));
            }
            for c in it {
                doc.add_body(&name, ductile_brep::profile::wire_from_segments(&c.0));
            }
        }
        true
    }
}

// --------------------------------------------------------- Deform curve ----

/// Lays the selected curves, drawn over a flat reference face, onto a
/// target face: click the reference, then the target.
pub struct DeformCurveTool {
    curves: Vec<(ObjectId, Vec<ductile_brep::profile::Segment>)>,
    /// Reference plane frame and its face's extent in that frame.
    from: Option<(Frame, ductile_geom::DVec2, ductile_geom::DVec2)>,
    /// Target surface and its face's parameter range.
    onto: Option<(ductile_geom::Surface, ductile_geom::DVec2, ductile_geom::DVec2)>,
    pub how: ductile_brep::curves::Deform,
    pub keep: bool,
    result: Option<Vec<Vec<ductile_brep::profile::Segment>>>,
}

impl DeformCurveTool {
    pub fn new(doc: &Document) -> Option<Self> {
        let curves: Vec<_> = doc
            .selected_objects()
            .into_iter()
            .filter_map(|id| doc.object(id).filter(|o| o.body.kind == ductile_brep::BodyKind::Wire).map(|o| (id, o.body.edges.iter().map(|e| ductile_brep::profile::Segment::new(e.curve.clone(), e.t0, e.t1)).collect())))
            .collect();
        (!curves.is_empty()).then(|| Self { curves, from: None, onto: None, how: Default::default(), keep: false, result: None })
    }

    fn update(&mut self) {
        self.result = match (&self.from, &self.onto) {
            (Some((f, a, b)), Some((s, c, d))) => self.curves.iter().map(|(_, segs)| ductile_brep::curves::deform_chain(segs, f, *a, *b, s, *c, *d, &self.how)).collect(),
            _ => None,
        };
    }
}

impl Tool for DeformCurveTool {
    fn name(&self) -> &'static str {
        "Deform curve"
    }
    fn hint(&self) -> String {
        match (&self.from, &self.onto) {
            (None, _) => "Deform curve: click the flat reference face the curves lie on".into(),
            (Some(_), None) => "Deform curve: click the target face".into(),
            _ => "Deform curve: adjust scale, offset and flips; Enter to finish".into(),
        }
    }
    fn pointer(&mut self, doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        if !p.clicked || self.onto.is_some() {
            return ToolOutcome::Continue;
        }
        let Some(hit) = crate::picking::pick_face(doc, &p.ray) else { return ToolOutcome::Continue };
        let Some(o) = doc.object(hit.object) else { return ToolOutcome::Continue };
        let fm = &o.mesh.faces[hit.face.idx()];
        let face = o.body.face(hit.face);
        if self.from.is_none() {
            let ductile_geom::Surface::Plane { frame } = &face.surface else { return ToolOutcome::Continue };
            let n = if face.reversed { -frame.z } else { frame.z };
            let f = Frame::new(frame.origin, n, frame.x);
            let (lo, hi) = fm.positions.iter().fold((ductile_geom::DVec2::splat(f64::MAX), ductile_geom::DVec2::splat(f64::MIN)), |(a, b), q| {
                let l = f.to_local(*q).truncate();
                (a.min(l), b.max(l))
            });
            self.from = Some((f, lo, hi));
        } else {
            let (lo, hi) = fm.uvs.iter().fold((ductile_geom::DVec2::splat(f64::MAX), ductile_geom::DVec2::splat(f64::MIN)), |(a, b), q| (a.min(*q), b.max(*q)));
            // Heights go out of the face, whichever way its surface runs.
            if face.reversed {
                self.how.flip = !self.how.flip;
            }
            self.onto = Some((face.surface.clone(), lo, hi));
            self.update();
        }
        ToolOutcome::Continue
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        let segs: Vec<_> = self.result.as_ref()?.iter().flatten().cloned().collect();
        Some(ductile_brep::profile::wire_from_segments(&segs))
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let before = (self.how, self.keep);
        egui::Grid::new("deform").num_columns(4).show(ui, |ui| {
            for (label, v) in [("Scale", 0), ("Offset", 1)] {
                ui.label(label);
                for i in 0..3 {
                    let x = if v == 0 { &mut self.how.scale[i] } else { &mut self.how.offset[i] };
                    ui.add(egui::DragValue::new(x).speed(0.01).prefix(["U ", "V ", "N "][i]));
                }
                ui.end_row();
            }
        });
        ui.horizontal(|ui| {
            ui.checkbox(&mut self.how.mirror, "Mirror");
            ui.checkbox(&mut self.how.swap, "Swap UV");
            ui.checkbox(&mut self.how.flip, "Flip normal");
        });
        ui.checkbox(&mut self.keep, "Keep the original curves");
        let changed = before != (self.how, self.keep);
        if changed {
            self.update();
        }
        changed
    }
    fn hidden_objects(&self) -> Vec<ObjectId> {
        if self.result.is_some() && !self.keep {
            self.curves.iter().map(|c| c.0).collect()
        } else {
            vec![]
        }
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let Some(res) = self.result.take() else { return false };
        for ((id, _), segs) in self.curves.iter().zip(res) {
            let b = ductile_brep::profile::wire_from_segments(&segs);
            if self.keep {
                let name = doc.object(*id).map(|o| o.name.clone()).unwrap_or_else(|| "Curve".into());
                doc.add_body(&name, b);
            } else {
                doc.replace_body(*id, b);
            }
        }
        true
    }
}

// ------------------------------------------------------------ Slide CV ----

/// Slides the selected curve control points along their control polygon
/// (towards the next point for a positive distance): drag, or type it.
pub struct SlideCvTool {
    /// Per control: object, edge, index, start position and slide direction.
    cvs: Vec<(ObjectId, EdgeId, u32, DVec3, DVec3)>,
    start: Option<DVec3>,
    pub distance: f64,
    typed: Option<f64>,
}

impl SlideCvTool {
    pub fn new(doc: &Document) -> Option<Self> {
        let mut cvs = Vec::new();
        for p in &doc.selection {
            let Pick::Control(o, e, i) = *p else { continue };
            let Some(ob) = doc.object(o) else { continue };
            let ductile_geom::Curve3::Nurbs(n) = &ob.body.edge(e).curve else { continue };
            let pts = n.control_points();
            let i = i as usize;
            if pts.len() < 2 || i >= pts.len() {
                continue;
            }
            let (a, b) = (pts[i.saturating_sub(1)], pts[(i + 1).min(pts.len() - 1)]);
            let Some(d) = (b - a).try_normalize() else { continue };
            cvs.push((o, e, i as u32, pts[i], d));
        }
        (!cvs.is_empty()).then(|| Self { cvs, start: None, distance: 0.0, typed: None })
    }

    fn slid(&self, body: &Body, id: ObjectId) -> Body {
        let d = self.typed.unwrap_or(self.distance);
        let mut b = body.clone();
        for (_, e, i, _, dir) in self.cvs.iter().filter(|c| c.0 == id) {
            b = ductile_brep::profile::move_controls(&b, &[(*e, *i)], &DMat4::from_translation(*dir * d));
        }
        b
    }

    fn objects(&self) -> Vec<ObjectId> {
        let mut v: Vec<ObjectId> = self.cvs.iter().map(|c| c.0).collect();
        v.dedup();
        v
    }
}

impl Tool for SlideCvTool {
    fn name(&self) -> &'static str {
        "Slide CV"
    }
    fn hint(&self) -> String {
        format!("Slide CV: drag along the control polygon or type a distance ({:.3}), Enter to finish", self.typed.unwrap_or(self.distance))
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        let (_, _, _, at, dir) = self.cvs[0];
        // The point on the slide line nearest the view ray.
        let w = p.ray.dir.cross(dir);
        let n = dir.cross(w);
        let q = if n.length_squared() > 1e-18 { p.ray.hit_plane(at, n.normalize()) } else { None };
        let Some(q) = q else { return ToolOutcome::Continue };
        let t = (q - at).dot(dir);
        match self.start {
            None if p.clicked => self.start = Some(at + dir * t),
            Some(s) => {
                self.distance = snap_len((at + dir * t - s).dot(dir).abs(), p.world_per_px) * (at + dir * t - s).dot(dir).signum();
                if p.clicked {
                    return ToolOutcome::Finished;
                }
            }
            None => {}
        }
        ToolOutcome::Continue
    }
    fn preview(&self, doc: &Document) -> Option<Body> {
        let segs: Vec<_> = self
            .objects()
            .into_iter()
            .filter_map(|id| doc.object(id).map(|o| self.slid(&o.body, id)))
            .flat_map(|b| b.edges.into_iter().map(|e| ductile_brep::profile::Segment::new(e.curve, e.t0, e.t1)))
            .collect();
        Some(ductile_brep::profile::wire_from_segments(&segs))
    }
    fn guides(&self, out: &mut Vec<LineInstance>) {
        for (_, _, _, at, dir) in &self.cvs {
            out.push(LineInstance::new(*at - *dir * 0.5, *at + *dir * 0.5, [0.35, 0.75, 1.0, 1.0], 2.0));
        }
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut d = self.typed.unwrap_or(self.distance);
        let changed = ui.add(egui::DragValue::new(&mut d).speed(0.01).prefix("Distance ")).changed();
        if changed {
            self.typed = Some(d);
        }
        changed
    }
    fn typed_value(&mut self, v: Option<f64>) {
        self.typed = v;
    }
    fn hidden_objects(&self) -> Vec<ObjectId> {
        self.objects()
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        if self.typed.unwrap_or(self.distance).abs() < 1e-12 {
            return false;
        }
        for id in self.objects() {
            if let Some(o) = doc.object(id) {
                let b = self.slid(&o.body, id);
                doc.replace_body(id, b);
            }
        }
        true
    }
}

// ------------------------------------------------------- Remove fillets ----

/// Finds the fillets of the selected solids up to a radius and grows the
/// faces around them back to sharp edges.
pub struct RemoveFilletsTool {
    targets: Vec<(ObjectId, std::sync::Arc<Body>)>,
    pub max_radius: f64,
    /// None: any; Some(true): rounds only; Some(false): inside fillets only.
    pub convex: Option<bool>,
    result: Vec<(ObjectId, Body, usize)>,
}

impl RemoveFilletsTool {
    pub fn new(doc: &Document) -> Option<Self> {
        let targets: Vec<_> = doc.selected_solids().into_iter().filter_map(|id| doc.object(id).map(|o| (id, o.body.clone()))).collect();
        if targets.is_empty() {
            return None;
        }
        let size = targets.iter().map(|t| t.1.bbox().diagonal()).fold(0.0, f64::max);
        let mut t = Self { targets, max_radius: (0.05 * size).max(1e-3), convex: None, result: vec![] };
        t.update();
        Some(t)
    }

    fn update(&mut self) {
        self.result = self
            .targets
            .iter()
            .map(|(id, b)| {
                let (r, n) = ductile_brep::ops::delete::remove_fillets(b, self.max_radius, self.convex);
                (*id, r, n)
            })
            .collect();
    }

    fn count(&self) -> usize {
        self.result.iter().map(|r| r.2).sum()
    }
}

impl Tool for RemoveFilletsTool {
    fn name(&self) -> &'static str {
        "Remove fillets"
    }
    fn hint(&self) -> String {
        format!("Remove fillets: {} found up to radius {:.3}; Enter to remove them", self.count(), self.max_radius)
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, _p: &PointerInfo) -> ToolOutcome {
        ToolOutcome::Continue
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        let ok: Vec<&Body> = self.result.iter().filter(|r| r.2 > 0).map(|r| &r.1).collect();
        match ok[..] {
            [] => None,
            [b] => Some(b.clone()),
            _ => {
                // Several solids: their edges as one wire, enough to show the change.
                let segs: Vec<_> = ok.iter().flat_map(|b| b.edges.iter().map(|e| ductile_brep::profile::Segment::new(e.curve.clone(), e.t0, e.t1))).collect();
                Some(ductile_brep::profile::wire_from_segments(&segs))
            }
        }
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let before = (self.max_radius, self.convex);
        ui.horizontal(|ui| {
            ui.label("Max radius");
            ui.add(egui::DragValue::new(&mut self.max_radius).speed(0.005).range(1e-6..=1e6));
        });
        ui.horizontal(|ui| {
            ui.radio_value(&mut self.convex, None, "Any");
            ui.radio_value(&mut self.convex, Some(true), "Convex");
            ui.radio_value(&mut self.convex, Some(false), "Concave");
        });
        let changed = before != (self.max_radius, self.convex);
        if changed {
            self.update();
        }
        ui.label(format!("{} fillet(s) to remove", self.count()));
        changed
    }
    fn typed_value(&mut self, v: Option<f64>) {
        if let Some(v) = v.filter(|v| *v > 0.0) {
            self.max_radius = v;
            self.update();
        }
    }
    fn hidden_objects(&self) -> Vec<ObjectId> {
        self.result.iter().filter(|r| r.2 > 0).map(|r| r.0).collect()
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let mut any = false;
        for (id, b, n) in std::mem::take(&mut self.result) {
            if n > 0 {
                doc.replace_body(id, b);
                any = true;
            }
        }
        any
    }
}

// ---------------------------------------------------------- Offset edge ----

/// New edges offset from the selected edges into the planar faces beside
/// them: drag or type the distance.
pub struct OffsetEdgeTool {
    target: (ObjectId, std::sync::Arc<Body>),
    edges: Vec<EdgeId>,
    pub distance: f64,
    typed: Option<f64>,
    result: Option<Body>,
    shown: f64,
}

impl OffsetEdgeTool {
    pub fn new(doc: &Document) -> Option<Self> {
        let picks = doc.selected_edges();
        let (o, _) = *picks.first()?;
        let body = doc.object(o)?.body.clone();
        if body.kind != ductile_brep::BodyKind::Solid {
            return None;
        }
        let edges: Vec<EdgeId> = picks.iter().filter(|(x, _)| *x == o).map(|(_, e)| *e).collect();
        let d = 0.05 * body.bbox().diagonal();
        let mut t = Self { target: (o, body), edges, distance: d, typed: None, result: None, shown: f64::NAN };
        t.update();
        Some(t)
    }

    fn value(&self) -> f64 {
        self.typed.unwrap_or(self.distance)
    }

    fn update(&mut self) {
        if self.shown == self.value() {
            return;
        }
        self.shown = self.value();
        self.result = ductile_brep::ops::cut::offset_edges(&self.target.1, &self.edges, self.value()).ok();
    }
}

impl Tool for OffsetEdgeTool {
    fn name(&self) -> &'static str {
        "Offset edge"
    }
    fn hint(&self) -> String {
        format!("Offset edge: drag or type the distance ({:.3}), Enter to imprint", self.value())
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        if let Some(q) = p.point.or(p.plane_point) {
            let b = &self.target.1;
            let e = b.edge(self.edges[0]);
            let t = e.curve.closest_param_in(q, e.t0.min(e.t1), e.t0.max(e.t1));
            if p.clicked {
                return ToolOutcome::Finished;
            }
            if self.typed.is_none() && p.point.is_some() {
                self.distance = snap_len(e.curve.point(t).distance(q), p.world_per_px).max(1e-6);
                self.update();
            }
        }
        ToolOutcome::Continue
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        self.result.clone()
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut d = self.value();
        let changed = ui.add(egui::DragValue::new(&mut d).speed(0.005).range(1e-6..=1e6).prefix("Distance ")).changed();
        if changed {
            self.typed = Some(d);
            self.update();
        }
        if self.result.is_none() {
            ui.label("The offset does not fit inside the faces");
        }
        changed
    }
    fn typed_value(&mut self, v: Option<f64>) {
        self.typed = v.filter(|v| *v > 0.0);
        self.update();
    }
    fn hidden_objects(&self) -> Vec<ObjectId> {
        vec![self.target.0]
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        match self.result.take() {
            Some(b) if b.faces.len() > self.target.1.faces.len() => {
                doc.replace_body(self.target.0, b);
                true
            }
            _ => false,
        }
    }
}

// ------------------------------------------------- Dependent offset face ----

/// Dependent offset face: the selected planar faces of a solid move up to
/// a face clicked next (of the same or another object): straight out along
/// their normals when it is parallel, onto its plane otherwise.
pub struct OffsetFaceToTool {
    target: (ObjectId, std::sync::Arc<Body>),
    faces: Vec<FaceId>,
    hover: Option<(ObjectId, FaceId)>,
    result: Option<Body>,
}

impl OffsetFaceToTool {
    pub fn new(doc: &Document) -> Option<Self> {
        let picks = doc.selected_faces();
        let (o, _) = *picks.first()?;
        let body = doc.object(o)?.body.clone();
        if body.kind != ductile_brep::BodyKind::Solid {
            return None;
        }
        let faces: Vec<FaceId> = picks.iter().filter(|(x, f)| *x == o && matches!(body.face(*f).surface, ductile_geom::Surface::Plane { .. })).map(|(_, f)| *f).collect();
        if faces.is_empty() {
            return None;
        }
        Some(Self { target: (o, body), faces, hover: None, result: None })
    }

    /// The body with the faces moved up to `onto`'s plane.
    fn up_to(&self, doc: &Document, onto: (ObjectId, FaceId)) -> Option<Body> {
        use ductile_geom::Surface;
        if onto.0 == self.target.0 && self.faces.contains(&onto.1) {
            return None;
        }
        let Surface::Plane { frame: to } = &doc.object(onto.0)?.body.face(onto.1).surface else { return None };
        let b = &*self.target.1;
        let mut dist = std::collections::HashMap::new();
        let mut planes = std::collections::HashMap::new();
        for &f in &self.faces {
            let face = b.face(f);
            let Surface::Plane { frame } = &face.surface else { continue };
            let n = if face.reversed { -frame.z } else { frame.z };
            dist.insert(f, (to.origin - frame.origin).dot(n));
            let same = to.z.dot(frame.z) >= 0.0;
            let fr = if same { *to } else { ductile_geom::Frame { y: -to.y, z: -to.z, ..*to } };
            planes.insert(f, Surface::Plane { frame: fr });
            if to.z.cross(frame.z).length() > 1e-9 {
                dist.clear();
            }
        }
        if dist.len() == self.faces.len() {
            if let Ok(r) = ductile_brep::ops::offset::offset_body(b, &|f| dist.get(&f).copied().unwrap_or(0.0)) {
                return Some(r);
            }
        }
        ductile_brep::ops::draft::replace_planes(b, &planes)
    }
}

impl Tool for OffsetFaceToTool {
    fn name(&self) -> &'static str {
        "Offset face to"
    }
    fn hint(&self) -> String {
        match (self.hover, &self.result) {
            (Some(_), None) => "Offset face to: the faces cannot reach that face (pick a flat one)".into(),
            _ => "Offset face to: click the face to offset up to".into(),
        }
    }
    fn pointer(&mut self, doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        let hit = crate::picking::pick_face(doc, &p.ray).map(|h| (h.object, h.face));
        if hit != self.hover {
            self.hover = hit;
            self.result = hit.and_then(|h| self.up_to(doc, h));
        }
        if p.clicked && self.result.is_some() {
            return ToolOutcome::Finished;
        }
        ToolOutcome::Continue
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        ui.label(format!("{} face(s) to move", self.faces.len()));
        false
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        self.result.clone()
    }
    fn hidden_objects(&self) -> Vec<ObjectId> {
        if self.result.is_some() { vec![self.target.0] } else { vec![] }
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        match self.result.take() {
            Some(b) => {
                doc.replace_body(self.target.0, b);
                true
            }
            None => false,
        }
    }
}

// --------------------------------------------------- Constrained surface ----

/// A smooth sheet through clicked points: click three or more, Backspace
/// drops the last, Enter makes the sheet.
pub struct ConstrainedSurfaceTool {
    points: Vec<DVec3>,
    cursor: Option<DVec3>,
    /// Bending weight: higher is smoother and looser.
    pub smooth: f64,
}

impl ConstrainedSurfaceTool {
    pub fn new() -> Self {
        Self { points: Vec::new(), cursor: None, smooth: 1e-5 }
    }

    fn surface(&self) -> Option<Body> {
        ductile_brep::ops::sheet::constrained_surface(&self.points, self.smooth)
    }
}

impl Tool for ConstrainedSurfaceTool {
    fn name(&self) -> &'static str {
        "Constrained surface"
    }
    fn hint(&self) -> String {
        format!("Constrained surface: click points it passes through ({} so far), Backspace drops the last, Enter to finish", self.points.len())
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        self.cursor = p.point.or(p.plane_point);
        if p.clicked {
            if let Some(q) = self.cursor {
                self.points.push(q);
            }
        }
        ToolOutcome::Continue
    }
    fn command_key(&mut self, key: egui::Key, m: egui::Modifiers) -> bool {
        if key == egui::Key::Backspace && !m.any() {
            return self.points.pop().is_some();
        }
        false
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        self.surface()
    }
    fn guides(&self, out: &mut Vec<LineInstance>) {
        for p in &self.points {
            out.push(LineInstance::point(*p, [1.0, 0.85, 0.2, 1.0], 8.0));
        }
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut k = -self.smooth.log10();
        let changed = ui.add(egui::Slider::new(&mut k, 2.0..=8.0).text("Tightness")).changed();
        if changed {
            self.smooth = 10f64.powf(-k);
        }
        ui.label(format!("{} points", self.points.len()));
        changed
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let Some(b) = self.surface() else { return false };
        let id = doc.add_body("Surface", b);
        doc.selection.clear();
        doc.selection.insert(Pick::Object(id));
        true
    }
}

// ---------------------------------------------------------------- Array ----

/// Copies of the selected objects in a row (along a construction plane axis,
/// X/Y/Z to switch) or around the construction plane's normal.
pub struct ArrayTool {
    targets: Vec<(ObjectId, std::sync::Arc<Body>)>,
    pub radial: bool,
    pub count: usize,
    /// Linear: spacing between copies. Radial: total angle in degrees.
    pub amount: f64,
    pub axis: usize,
    pub center: DVec3,
    base: DVec3,
    start_t: Option<f64>,
    typed: Option<f64>,
    frame: Frame,
}

impl ArrayTool {
    pub fn new(doc: &Document, radial: bool) -> Option<Self> {
        let targets: Vec<_> = doc.selected_objects().into_iter().filter_map(|id| doc.object(id).map(|o| (id, o.body.clone()))).collect();
        if targets.is_empty() {
            return None;
        }
        let mut bb = ductile_geom::BBox3::EMPTY;
        for (_, b) in &targets {
            bb = bb.union(&b.bbox());
        }
        let frame = current_cplane().frame;
        let size = bb.diagonal().max(1e-3);
        Some(Self {
            targets,
            radial,
            count: if radial { 6 } else { 4 },
            amount: if radial { 360.0 } else { (size * 1.2 * 100.0).round() / 100.0 },
            axis: 0,
            center: frame.origin,
            base: bb.center(),
            start_t: None,
            typed: None,
            frame,
        })
    }

    fn axis_dir(&self) -> DVec3 {
        [self.frame.x, self.frame.y, self.frame.z][self.axis]
    }

    /// Transform of copy `k` (k = 1 .. count - 1).
    fn transform(&self, k: usize) -> ductile_geom::DMat4 {
        use ductile_geom::DMat4;
        let a = self.typed.unwrap_or(self.amount);
        if self.radial {
            // A full turn spreads the copies evenly; otherwise the angle is
            // the span from the first copy to the last.
            let full = (a.abs() - 360.0).abs() < 1e-9;
            let step = if full { a / self.count as f64 } else { a / (self.count.max(2) - 1) as f64 };
            let rot = DMat4::from_axis_angle(self.frame.z, (step * k as f64).to_radians());
            DMat4::from_translation(self.center) * rot * DMat4::from_translation(-self.center)
        } else {
            DMat4::from_translation(self.axis_dir() * a * k as f64)
        }
    }

    fn copies(&self) -> Vec<(ObjectId, Body)> {
        (1..self.count).flat_map(|k| {
            let m = self.transform(k);
            self.targets.iter().map(move |(id, b)| (*id, b.transformed(&m)))
        }).collect()
    }
}

impl Tool for ArrayTool {
    fn name(&self) -> &'static str {
        if self.radial {
            "Radial array"
        } else {
            "Linear array"
        }
    }
    fn hint(&self) -> String {
        if self.radial {
            format!("Radial array: {} copies over {:.1}°, set them in the panel, Enter to finish", self.count, self.typed.unwrap_or(self.amount))
        } else {
            format!("Linear array: drag or type the spacing ({:.4}), X/Y/Z for the direction, Enter to finish", self.typed.unwrap_or(self.amount))
        }
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        if !self.radial && self.typed.is_none() {
            let t = height_from_ray(self.base, self.axis_dir(), &p.ray);
            let t0 = *self.start_t.get_or_insert(t - self.amount);
            let d = t - t0;
            self.amount = snap_len(d.abs(), p.world_per_px).copysign(d);
        }
        if p.clicked {
            return ToolOutcome::Finished;
        }
        ToolOutcome::Continue
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        let mut out: Option<Body> = None;
        for (_, b) in self.copies() {
            match &mut out {
                None => out = Some(b),
                Some(o) => o.append(&b),
            }
        }
        out
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut ch = ui.horizontal(|ui| ui.radio_value(&mut self.radial, false, "Linear").changed() | ui.radio_value(&mut self.radial, true, "Radial").changed()).inner;
        let mut n = self.count as f64;
        if drag_value(ui, "Count", &mut n, 0.1) {
            self.count = (n.round() as usize).clamp(2, 1000);
            ch = true;
        }
        let mut a = self.typed.unwrap_or(self.amount);
        if drag_value(ui, if self.radial { "Angle" } else { "Spacing" }, &mut a, if self.radial { 1.0 } else { 0.01 }) {
            self.typed = Some(a);
            ch = true;
        }
        if self.radial {
            let mut c = [self.center.x, self.center.y, self.center.z];
            ui.horizontal(|ui| {
                ui.label("Center");
                for v in c.iter_mut() {
                    ch |= ui.add(egui::DragValue::new(v).speed(0.05).max_decimals(4)).changed();
                }
            });
            self.center = DVec3::from_array(c);
        }
        ch
    }
    fn typed_value(&mut self, v: Option<f64>) {
        self.typed = v;
    }
    fn constrain_axis(&mut self, axis: usize) {
        self.axis = axis;
        self.start_t = None;
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let copies = self.copies();
        if copies.is_empty() {
            return false;
        }
        doc.selection.clear();
        for (id, b) in copies {
            let name = doc.object(id).map(|o| o.name.clone()).unwrap_or_else(|| "Copy".into());
            let new = doc.add_body(&name, b);
            doc.selection.insert(Pick::Object(new));
        }
        true
    }
}

// -------------------------------------------------------------- Revolve ----

/// Revolves the regions of the selected closed curves about an axis in
/// their plane: a selected straight edge, or the construction plane's Y
/// (or X) axis.
pub struct RevolveTool {
    regions: Vec<ductile_brep::profile::Region>,
    /// Open curves, revolved into a sheet when there are no regions.
    open: Vec<ductile_brep::profile::Loop3>,
    edge_axis: Option<(DVec3, DVec3)>,
    /// 0: selected edge (if any), 1: construction plane X, 2: Y.
    pub axis_choice: usize,
    pub degrees: f64,
    cache: std::cell::RefCell<Option<(usize, f64, Option<Body>, Option<String>)>>,
}

impl RevolveTool {
    pub fn new(doc: &Document) -> Option<Self> {
        let regions = selected_curve_regions(doc);
        let open: Vec<_> = if regions.is_empty() { selected_chains(doc).into_iter().filter(|c| !c.is_closed()).collect() } else { Vec::new() };
        if regions.is_empty() && open.is_empty() {
            return None;
        }
        let edge_axis = doc.selected_edges().into_iter().find_map(|(o, e)| {
            let ed = doc.object(o)?.body.edge(e).clone();
            ed.curve.is_linear().then(|| (ed.start(), (ed.end() - ed.start()).normalize()))
        });
        let axis_choice = if edge_axis.is_some() { 0 } else { 2 };
        Some(Self { regions, open, edge_axis, axis_choice, degrees: 360.0, cache: std::cell::RefCell::new(None) })
    }

    fn axis(&self) -> (DVec3, DVec3) {
        let f = current_cplane().frame;
        match (self.axis_choice, self.edge_axis) {
            (0, Some(a)) => a,
            (1, _) => (f.origin, f.x),
            _ => (f.origin, f.y),
        }
    }

    fn result(&self) -> (Option<Body>, Option<String>) {
        use ductile_brep::ops::boolean::{boolean, BoolOp};
        use ductile_brep::ops::revolve::revolve;
        if let Some((c, d, b, e)) = &*self.cache.borrow() {
            if *c == self.axis_choice && *d == self.degrees {
                return (b.clone(), e.clone());
            }
        }
        let (o, a) = self.axis();
        let ang = self.degrees.to_radians();
        let mut out: Result<Option<Body>, String> = Ok(None);
        if self.regions.is_empty() {
            out = ductile_brep::ops::sheet::revolve_sheet(&self.open, o, a, ang).map(Some).ok_or_else(|| "could not revolve the curves".into());
        }
        'outer: for r in &self.regions {
            let Some(mut solid) = revolve(&r.outer, o, a, ang) else {
                out = Err("the profile must lie on one side of an axis in its plane".into());
                break;
            };
            for h in &r.holes {
                match revolve(h, o, a, ang).map(|hs| boolean(&solid, &hs, BoolOp::Difference)) {
                    Some(Ok(b)) => solid = b,
                    _ => {
                        out = Err("could not cut a hole of the profile".into());
                        break 'outer;
                    }
                }
            }
            out = match out {
                Ok(None) => Ok(Some(solid)),
                Ok(Some(p)) => boolean(&p, &solid, BoolOp::Union).map(Some).map_err(|e| e.to_string()),
                e => e,
            };
        }
        let res = match out {
            Ok(b) => (b, None),
            Err(e) => (None, Some(e)),
        };
        *self.cache.borrow_mut() = Some((self.axis_choice, self.degrees, res.0.clone(), res.1.clone()));
        res
    }
}

impl Tool for RevolveTool {
    fn name(&self) -> &'static str {
        "Revolve"
    }
    fn hint(&self) -> String {
        match self.cache.borrow().as_ref().and_then(|c| c.3.clone()) {
            Some(e) => format!("Revolve failed: {e}"),
            None => format!("Revolve: type an angle ({:.1}°), choose the axis in the panel, click or Enter to finish", self.degrees),
        }
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        if p.clicked {
            ToolOutcome::Finished
        } else {
            ToolOutcome::Continue
        }
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        self.result().0
    }
    fn guides(&self, out: &mut Vec<LineInstance>) {
        let (o, a) = self.axis();
        let l = self.regions.iter().map(|r| &r.outer).chain(self.open.iter()).flat_map(|c| c.polygon(1e-2)).map(|p| (p - o).length()).fold(1.0, f64::max) * 1.5;
        out.push(LineInstance::new(o - a * l, o + a * l, [0.3, 0.9, 1.0, 1.0], 1.5));
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut ch = drag_value(ui, "Angle", &mut self.degrees, 1.0);
        ui.horizontal(|ui| {
            ui.label("Axis");
            if self.edge_axis.is_some() {
                ch |= ui.radio_value(&mut self.axis_choice, 0, "Edge").changed();
            }
            ch |= ui.radio_value(&mut self.axis_choice, 1, "X").changed();
            ch |= ui.radio_value(&mut self.axis_choice, 2, "Y").changed();
        });
        ch
    }
    fn typed_value(&mut self, v: Option<f64>) {
        self.degrees = v.unwrap_or(360.0);
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let Some(b) = self.result().0 else { return false };
        let id = doc.add_body("Revolve", b);
        doc.selection.clear();
        doc.selection.insert(Pick::Object(id));
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A click straight down onto the construction plane at `p`.
    fn click(t: &mut dyn Tool, doc: &Document, p: DVec3, clicked: bool) -> bool {
        let info = PointerInfo { ray: Ray { origin: p + DVec3::Z * 10.0, dir: -DVec3::Z }, point: Some(p), plane_point: Some(p), clicked, world_per_px: 0.01 };
        matches!(t.pointer(doc, &CPlane::default(), &info), ToolOutcome::Finished)
    }

    fn volume(b: &Body) -> f64 {
        ductile_brep::tessellate(b, &ductile_brep::TessOptions { chord_tol: 1e-4, angle_tol: 0.05 }).volume()
    }

    #[test]
    fn shell_opens_the_selected_face() {
        let mut doc = Document::default();
        let id = doc.add_body("Box", ductile_brep::primitives::make_box(&ductile_geom::Frame::WORLD, DVec3::new(4.0, 3.0, 2.0)));
        let b = doc.object(id).unwrap().body.clone();
        let top = b.face_ids().find(|f| b.face_edges(*f).iter().all(|e| b.edge(*e).start().z > 1.9 && b.edge(*e).end().z > 1.9)).unwrap();
        doc.selection.insert(Pick::Face(id, top));
        let mut t = ShellTool::new(&doc, OffsetMode::Shell).unwrap();
        t.typed_value(Some(0.25));
        assert!(t.commit(&mut doc));
        let v = ductile_brep::tessellate(&doc.object(id).unwrap().body, &ductile_brep::TessOptions::default()).volume();
        assert!((v - (24.0 - 3.5 * 2.5 * 1.75)).abs() < 1e-6, "{v}");
        // Offset the result outward.
        doc.selection.insert(Pick::Object(id));
        let mut o = ShellTool::new(&doc, OffsetMode::Solid).unwrap();
        o.typed_value(Some(0.1));
        assert!(o.commit(&mut doc));
    }

    #[test]
    fn fillet_and_offset_a_drawn_rectangle() {
        set_current_cplane(CPlane::default());
        let mut doc = Document::default();
        let mut r = CurveTool::new(CurveKind::Rectangle);
        click(&mut r, &doc, DVec3::ZERO, true);
        assert!(click(&mut r, &doc, DVec3::new(4.0, 3.0, 0.0), true));
        assert!(r.commit(&mut doc));
        let id = doc.objects[0].id;
        doc.selection.insert(Pick::Object(id));
        let mut f = CurveEditTool::new(&doc, false).unwrap();
        f.typed_value(Some(0.5));
        assert!(f.commit(&mut doc));
        assert_eq!(doc.object(id).unwrap().body.edges.len(), 8);
        doc.selection.insert(Pick::Object(id));
        let mut o = CurveEditTool::new(&doc, true).unwrap();
        o.typed_value(Some(0.25));
        assert!(o.commit(&mut doc));
        assert_eq!(doc.objects.len(), 2);
        // The offset grows the loop: it extrudes to a larger area.
        doc.selection.clear();
        doc.selection.insert(Pick::Object(doc.objects[1].id));
        let a = extrude_selected(&mut doc);
        // Steiner: area + perimeter * d + pi d^2 for an outward offset.
        let pi = std::f64::consts::PI;
        let (area, perimeter) = (12.0 - (4.0 - pi) * 0.25, 10.0 + pi);
        let expect = area + perimeter * 0.25 + pi * 0.0625;
        assert!((a - expect).abs() < 2e-2, "{a} vs {expect}");
    }

    #[test]
    fn instances_share_edits_but_not_placement() {
        set_current_cplane(CPlane::default());
        let mut doc = Document::default();
        let id = doc.add_body("Box", ductile_brep::primitives::make_box(&ductile_geom::Frame::WORLD, DVec3::new(2.0, 2.0, 2.0)));
        doc.selection.insert(Pick::Object(id));
        let copy = doc.instance_selected()[0];
        assert_eq!(doc.instance_count(id), 2);
        let base = doc.object(id).unwrap().body.bbox().min;
        // Move the copy away: the original stays.
        doc.selection.clear();
        doc.selection.insert(Pick::Object(copy));
        let mut t = TransformTool::new(TransformKind::Move, &doc);
        t.typed_value(Some(5.0));
        t.constrain_axis(0);
        let m = t.matrix();
        assert!(t.commit(&mut doc));
        let shift = m.transform_point3(DVec3::ZERO);
        assert!(shift.length() > 1.0, "{shift}");
        let bb = |d: &Document, i| d.object(i).unwrap().body.bbox();
        assert!(bb(&doc, id).min.distance(base) < 1e-9);
        assert!(bb(&doc, copy).min.distance(base + shift) < 1e-9);
        // Knife the original's top: the copy gets the cut too, where it is.
        let mut c = CurveTool::new(CurveKind::Polyline);
        c.command_key(egui::Key::K, egui::Modifiers::NONE);
        click(&mut c, &doc, DVec3::new(1.0, 0.0, 2.0), true);
        click(&mut c, &doc, DVec3::new(1.0, 2.0, 2.0), true);
        c.accept();
        assert!(c.commit(&mut doc));
        assert_eq!(doc.object(id).unwrap().body.faces.len(), 7);
        assert_eq!(doc.object(copy).unwrap().body.faces.len(), 7);
        assert!(bb(&doc, copy).min.distance(base + shift) < 1e-9);
        // Unique again: edits stay put.
        doc.selection.clear();
        doc.selection.insert(Pick::Object(copy));
        assert_eq!(doc.make_unique_selected(), 1);
        assert_eq!(doc.instance_count(id), 0);
    }

    #[test]
    fn deforms_curves_from_one_face_onto_another() {
        set_current_cplane(CPlane::default());
        let mut doc = Document::default();
        doc.add_body("Box", ductile_brep::primitives::make_box(&ductile_geom::Frame::WORLD, DVec3::new(4.0, 3.0, 2.0)));
        let at = ductile_geom::Frame { origin: DVec3::new(20.0, 0.0, 0.0), ..ductile_geom::Frame::WORLD };
        doc.add_body("Big box", ductile_brep::primitives::make_box(&at, DVec3::new(8.0, 6.0, 1.0)));
        let line = doc.add_body("Curve", ductile_brep::profile::wire_from_segments(&[ductile_brep::profile::Segment::line(DVec3::new(1.0, 0.0, 2.0), DVec3::new(1.0, 3.0, 2.0))]));
        doc.selection.insert(Pick::Object(line));
        let mut t = DeformCurveTool::new(&doc).unwrap();
        click(&mut t, &doc, DVec3::new(2.0, 1.5, 2.0), true);
        click(&mut t, &doc, DVec3::new(24.0, 3.0, 1.0), true);
        assert!(t.preview(&doc).is_some());
        assert!(t.commit(&mut doc));
        let b = &doc.object(line).unwrap().body;
        let e = &b.edges[0];
        let (p, q) = (e.curve.point(e.t0), e.curve.point(e.t1));
        assert!((p.z - 1.0).abs() < 1e-6 && (q.z - 1.0).abs() < 1e-6, "{p} {q}");
        let len = p.distance(q);
        assert!((len - 6.0).abs() < 1e-4 || (len - 8.0).abs() < 1e-4, "{len}");
        assert!(p.x > 19.9 && q.x < 28.1);
    }

    #[test]
    fn slides_control_points_along_the_polygon() {
        let mut doc = Document::default();
        let n = ductile_geom::NurbsCurve::uniform(3, &[DVec3::ZERO, DVec3::new(1.0, 1.0, 0.0), DVec3::new(2.0, 1.0, 0.0), DVec3::new(3.0, 0.0, 0.0)]);
        let (a, b) = n.domain();
        let id = doc.add_body("Curve", ductile_brep::profile::wire_from_segments(&[ductile_brep::profile::Segment::new(ductile_geom::Curve3::Nurbs(n), a, b)]));
        doc.selection.insert(Pick::Control(id, EdgeId(0), 1));
        let mut t = SlideCvTool::new(&doc).unwrap();
        t.typed_value(Some(0.5));
        assert!(t.commit(&mut doc));
        let ductile_geom::Curve3::Nurbs(n) = &doc.object(id).unwrap().body.edges[0].curve else { panic!() };
        // Along the line from the previous to the next control point.
        let d = DVec3::new(2.0, 1.0, 0.0).normalize() * 0.5;
        assert!(n.control_points()[1].distance(DVec3::new(1.0, 1.0, 0.0) + d) < 1e-12);
        assert!(n.control_points()[2].distance(DVec3::new(2.0, 1.0, 0.0)) < 1e-12);
    }

    #[test]
    fn select_previous_walks_back_through_inputs() {
        let mut doc = Document::default();
        let a = doc.add_body("A", ductile_brep::primitives::make_box(&ductile_geom::Frame::WORLD, DVec3::ONE));
        let b = doc.add_body("B", ductile_brep::primitives::make_box(&ductile_geom::Frame::WORLD, DVec3::ONE));
        doc.selection.insert(Pick::Object(a));
        doc.checkpoint("one");
        doc.selection.clear();
        doc.selection.insert(Pick::Object(b));
        doc.checkpoint("two");
        doc.selection.clear();
        assert_eq!(doc.select_previous(), 1);
        assert!(doc.selection.contains(&Pick::Object(b)));
        assert_eq!(doc.select_previous(), 1);
        assert!(doc.selection.contains(&Pick::Object(a)));
        assert_eq!(doc.select_previous(), 0);
    }

    #[test]
    fn offset_edge_imprints_beside_the_edge() {
        let mut doc = Document::default();
        let id = doc.add_body("Box", ductile_brep::primitives::make_box(&ductile_geom::Frame::WORLD, DVec3::new(4.0, 3.0, 2.0)));
        let b = doc.object(id).unwrap().body.clone();
        let e = b.edge_ids().find(|e| { let m = b.edge(*e).mid(); m.z > 1.9 && m.y < 0.1 }).unwrap();
        doc.selection.insert(Pick::Edge(id, e));
        let mut t = OffsetEdgeTool::new(&doc).unwrap();
        t.typed_value(Some(0.4));
        assert!(t.commit(&mut doc));
        assert_eq!(doc.object(id).unwrap().body.faces.len(), 8);
    }

    #[test]
    fn constrained_surface_from_clicks() {
        let mut doc = Document::default();
        let mut t = ConstrainedSurfaceTool::new();
        for (x, y, z) in [(0.0, 0.0, 0.0), (2.0, 0.0, 0.5), (2.0, 2.0, 0.0), (0.0, 2.0, 0.5), (1.0, 1.0, 0.8)] {
            click(&mut t, &doc, DVec3::new(x, y, z), true);
        }
        assert!(t.command_key(egui::Key::Backspace, egui::Modifiers::NONE));
        click(&mut t, &doc, DVec3::new(1.0, 1.0, 0.6), true);
        assert!(t.commit(&mut doc));
        assert_eq!(doc.objects.len(), 1);
        assert_eq!(doc.objects[0].body.kind, ductile_brep::BodyKind::Sheet);
    }

    #[test]
    fn knife_splits_the_face_drawn_on() {
        set_current_cplane(CPlane::default());
        let mut doc = Document::default();
        let id = doc.add_body("Box", ductile_brep::primitives::make_box(&ductile_geom::Frame::WORLD, DVec3::new(4.0, 3.0, 2.0)));
        let mut c = CurveTool::new(CurveKind::Polyline);
        assert!(c.command_key(egui::Key::K, egui::Modifiers::NONE));
        assert!(c.hint().starts_with("Knife"));
        click(&mut c, &doc, DVec3::new(1.0, 0.0, 2.0), true);
        click(&mut c, &doc, DVec3::new(1.0, 3.0, 2.0), true);
        c.accept();
        assert!(c.commit(&mut doc));
        // No curve object: the box's top is now two faces.
        assert_eq!(doc.objects.len(), 1);
        let b = &doc.object(id).unwrap().body;
        assert_eq!(b.faces.len(), 7);
        assert!((volume(b) - 24.0).abs() < 1e-6);
    }

    #[test]
    fn splits_curves_and_inserts_knots() {
        set_current_cplane(CPlane::default());
        let mut doc = Document::default();
        let mut r = CurveTool::new(CurveKind::Rectangle);
        click(&mut r, &doc, DVec3::ZERO, true);
        assert!(click(&mut r, &doc, DVec3::new(4.0, 3.0, 0.0), true));
        assert!(r.commit(&mut doc));
        // Open the rectangle on its bottom side, then cut off the top.
        let mut t = SplitCurveTool::new(&doc, false).unwrap();
        click(&mut t, &doc, DVec3::new(1.0, 0.0, 0.0), true);
        click(&mut t, &doc, DVec3::new(2.0, 3.0, 0.0), true);
        assert!(t.commit(&mut doc));
        assert_eq!(doc.objects.len(), 2);
        let lens: Vec<f64> = doc.objects.iter().map(|o| o.body.edges.iter().map(|e| e.curve.to_nurbs(e.t0, e.t1).length()).sum()).collect();
        assert!((lens.iter().sum::<f64>() - 14.0).abs() < 1e-6, "{lens:?}");
        assert!(lens.iter().any(|l| (l - 8.0).abs() < 1e-6) && lens.iter().any(|l| (l - 6.0).abs() < 1e-6), "{lens:?}");
        // A knot in a circle: one more control point, same circle.
        let mut c = CurveTool::new(CurveKind::Circle);
        click(&mut c, &doc, DVec3::new(10.0, 0.0, 0.0), true);
        c.typed_value(Some(1.0));
        assert!(c.commit(&mut doc));
        let id = doc.objects.last().unwrap().id;
        let mut t = SplitCurveTool::new(&doc, true).unwrap();
        click(&mut t, &doc, DVec3::new(10.0 + 0.6, 0.8, 0.0), true);
        assert!(t.commit(&mut doc));
        let b = &doc.object(id).unwrap().body;
        let ductile_geom::Curve3::Nurbs(n) = &b.edges[0].curve else { panic!("{:?}", b.edges[0].curve) };
        let base = ductile_geom::Curve3::Circle { frame: ductile_geom::Frame::new(DVec3::new(10.0, 0.0, 0.0), DVec3::Z, DVec3::X), radius: 1.0 }.to_nurbs(0.0, std::f64::consts::TAU);
        assert_eq!(n.ctrl.len(), base.ctrl.len() + 1);
        for i in 0..=16 {
            let p = n.point(b.edges[0].t0 + (b.edges[0].t1 - b.edges[0].t0) * i as f64 / 16.0);
            assert!((p.distance(DVec3::new(10.0, 0.0, 0.0)) - 1.0).abs() < 1e-9);
        }
    }

    #[test]
    fn trims_a_circle_against_a_rectangle() {
        set_current_cplane(CPlane::default());
        let mut doc = Document::default();
        let mut r = CurveTool::new(CurveKind::Rectangle);
        click(&mut r, &doc, DVec3::ZERO, true);
        assert!(click(&mut r, &doc, DVec3::new(4.0, 3.0, 0.0), true));
        assert!(r.commit(&mut doc));
        let mut c = CurveTool::new(CurveKind::Circle);
        click(&mut c, &doc, DVec3::new(4.0, 3.0, 0.0), true);
        c.typed_value(Some(1.0));
        assert!(c.commit(&mut doc));
        let mut t = TrimTool::new(&doc).unwrap();
        // Remove the circle's arc outside the rectangle and the
        // rectangle's corner inside the circle.
        let ray = |x: f64, y: f64| PointerInfo { ray: Ray { origin: DVec3::new(x, y, 10.0), dir: -DVec3::Z }, point: None, plane_point: None, clicked: true, world_per_px: 0.01 };
        t.pointer(&doc, &CPlane::default(), &ray(5.0, 3.0));
        t.pointer(&doc, &CPlane::default(), &ray(4.0, 2.5));
        t.pointer(&doc, &CPlane::default(), &ray(3.5, 3.0));
        assert!(t.commit(&mut doc));

        // What is left closes into one region: the rectangle with a bite.
        doc.selection.clear();
        for o in doc.objects.iter() {
            doc.selection.insert(Pick::Object(o.id));
        }
        let a = extrude_selected(&mut doc);
        let expect = 12.0 - std::f64::consts::PI / 4.0;
        assert!((a - expect).abs() < 2e-2, "{a} vs {expect}");
    }

    #[test]
    fn move_tool_drags_surface_control_points() {
        use ductile_brep::profile::{Loop3, Segment};
        let q = [DVec3::ZERO, DVec3::new(2.0, 0.0, 0.0), DVec3::new(2.0, 2.0, 1.0), DVec3::new(0.0, 2.0, 0.0)];
        let lp = Loop3 { segments: (0..4).map(|i| Segment::line(q[i], q[(i + 1) % 4])).collect() };
        let mut doc = Document::default();
        let id = doc.add_body("Patch", ductile_brep::ops::sheet::patch(&lp, &[]).unwrap());
        let cps = ductile_brep::ops::sheet::surface_control_points(&doc.object(id).unwrap().body);
        let c = cps.iter().find(|c| c.2.distance(q[2]) < 1e-12).unwrap();
        doc.selection.insert(Pick::SurfaceControl(id, c.0, c.1));
        let mut t = TransformTool::new(TransformKind::Move, &doc);
        t.axis = Some(2);
        t.typed_value(Some(1.5));
        assert!(t.commit(&mut doc));
        let b = &doc.object(id).unwrap().body;
        assert!(b.vertices.iter().any(|v| v.point.distance(q[2] + DVec3::Z * 1.5) < 1e-9));
        assert!(b.vertices.iter().any(|v| v.point.distance(q[0]) < 1e-12));
    }

    #[test]
    fn deletes_and_duplicates_faces_and_edges() {
        let mut doc = Document::default();
        let id = doc.add_body("Box", ductile_brep::primitives::make_box(&ductile_geom::Frame::WORLD, DVec3::ONE));
        doc.mode = SelectMode::Edge;
        doc.selection.insert(Pick::Edge(id, ductile_brep::EdgeId(0)));
        let made = duplicate_selected(&mut doc);
        assert_eq!(made.len(), 1);
        assert_eq!(doc.object(made[0]).unwrap().body.kind, ductile_brep::BodyKind::Wire);
        doc.selection.clear();
        doc.mode = SelectMode::Face;
        doc.selection.insert(Pick::Face(id, ductile_brep::FaceId(0)));
        assert!(delete_selected(&mut doc).unwrap().contains("open sheet"));
        assert_eq!(doc.object(id).unwrap().body.faces.len(), 5);
        doc.undo();
        assert_eq!(doc.object(id).unwrap().body.faces.len(), 6);
    }

    #[test]
    fn joins_curves_end_to_end() {
        use ductile_brep::profile::{wire_chains, wire_from_segments, Segment};
        let line = |a: DVec3, b: DVec3| wire_from_segments(&[Segment::new(ductile_geom::Curve3::line_through(a, b), 0.0, a.distance(b))]);
        let mut doc = Document::default();
        let a = doc.add_body("A", line(DVec3::ZERO, DVec3::X));
        let b = doc.add_body("B", line(DVec3::X, DVec3::new(1.0, 1.0, 0.0)));
        doc.selection.insert(Pick::Object(a));
        assert!(join_selected(&mut doc).is_err());
        doc.selection.insert(Pick::Object(b));
        let id = join_selected(&mut doc).unwrap();
        assert_eq!(doc.objects.len(), 1);
        let body = &doc.object(id).unwrap().body;
        assert_eq!((body.edges.len(), body.vertices.len()), (2, 3));
        assert_eq!(wire_chains(body).len(), 1);
    }

    #[test]
    fn mirrors_a_half_across_its_cut_face_into_one_solid() {
        let mut doc = Document::default();
        let id = doc.add_body("Half", ductile_brep::primitives::make_box(&ductile_geom::Frame::WORLD, DVec3::new(1.0, 2.0, 1.0)));
        let b = doc.object(id).unwrap().body.clone();
        // The x = 1 face.
        let f = b.face_ids().find(|f| b.face_normal(*f, 0.5, 0.5).x > 0.9).unwrap();
        doc.mode = SelectMode::Face;
        doc.selection.insert(Pick::Face(id, f));
        mirror_selected(&mut doc, &ductile_geom::Frame::WORLD, 0).unwrap();
        assert_eq!(doc.objects.len(), 1);
        let whole = &doc.object(id).unwrap().body;
        let bb = whole.bbox();
        assert!((bb.max.x - 2.0).abs() < 1e-2 && bb.min.x.abs() < 1e-2, "{bb:?}");
        // The cut face is gone: a plain box again.
        assert_eq!(whole.faces.len(), 6);
        // Without a face: a copy mirrored across the construction plane.
        doc.mode = SelectMode::Object;
        doc.selection.insert(Pick::Object(id));
        mirror_selected(&mut doc, &ductile_geom::Frame::WORLD, 0).unwrap();
        assert_eq!(doc.objects.len(), 2);
        assert!((doc.objects[1].body.bbox().min.x + 2.0).abs() < 1e-2);
    }

    #[test]
    fn pipes_a_circle_into_a_ring() {
        use ductile_brep::profile::{wire_from_segments, Segment};
        let mut doc = Document::default();
        let c = doc.add_body("Circle", wire_from_segments(&[Segment::new(circle_at(DVec3::ZERO, 2.0), 0.0, std::f64::consts::TAU)]));
        doc.selection.insert(Pick::Object(c));
        let mut t = PipeTool::new(&doc).unwrap();
        t.typed_value(Some(0.25));
        assert!(t.commit(&mut doc));
        let ring = &doc.objects.last().unwrap().body;
        assert_eq!(ring.kind, ductile_brep::BodyKind::Solid);
        assert_eq!(ductile_brep::ops::boolean::open_edges(ring), 0);
    }

    #[test]
    fn drafts_box_sides_by_a_typed_angle() {
        let mut doc = Document::default();
        let id = doc.add_body("Box", ductile_brep::primitives::make_box(&ductile_geom::Frame::WORLD, DVec3::new(2.0, 2.0, 1.0)));
        let b = doc.object(id).unwrap().body.clone();
        doc.mode = SelectMode::Face;
        for f in b.face_ids().filter(|f| b.face_normal(*f, 0.5, 0.5).z.abs() < 0.5) {
            doc.selection.insert(Pick::Face(id, f));
        }
        let mut t = DraftTool::new(&doc, DVec3::Z).unwrap();
        t.typed_value(Some(5.0));
        assert!(t.commit(&mut doc));
        let d = &doc.object(id).unwrap().body;
        // The base stays, the top shrinks by tan 5 deg on each side.
        let top = d.vertices.iter().map(|v| v.point).filter(|p| p.z > 0.5).map(|p| p.x).fold(f64::MIN, f64::max);
        let bottom = d.vertices.iter().map(|v| v.point).filter(|p| p.z < 0.5).map(|p| p.x).fold(f64::MIN, f64::max);
        assert!((top - (2.0 - 5f64.to_radians().tan())).abs() < 1e-9, "{top}");
        assert!((bottom - 2.0).abs() < 1e-9);
    }

    #[test]
    fn construction_plane_sits_on_a_face() {
        let b = ductile_brep::primitives::make_box(&ductile_geom::Frame::WORLD, DVec3::new(4.0, 2.0, 3.0));
        let top = b.face_ids().find(|f| b.face_normal(*f, 0.5, 0.5).z > 0.9).unwrap();
        let fr = cplane_on_face(&b, top).unwrap();
        assert!((fr.origin - DVec3::new(2.0, 1.0, 3.0)).length() < 1e-9);
        assert!((fr.z - DVec3::Z).length() < 1e-9 && fr.x.y.abs() < 1e-9);
        let side = b.face_ids().find(|f| b.face_normal(*f, 0.5, 0.5).x < -0.9).unwrap();
        assert!((cplane_on_face(&b, side).unwrap().z + DVec3::X).length() < 1e-9);
    }

    #[test]
    fn draws_on_a_face_construction_plane() {
        let b = ductile_brep::primitives::make_box(&ductile_geom::Frame::WORLD, DVec3::new(4.0, 2.0, 3.0));
        let side = b.face_ids().find(|f| b.face_normal(*f, 0.5, 0.5).x < -0.9).unwrap();
        let cp = CPlane { frame: cplane_on_face(&b, side).unwrap() };
        set_current_cplane(cp);
        let mut doc = Document::default();
        let mut r = CurveTool::new(CurveKind::Rectangle);
        let at = |x: f64, y: f64| cp.frame.to_world(DVec3::new(x, y, 0.0));
        for (p, last) in [(at(-0.5, -0.5), false), (at(0.5, 0.5), true)] {
            let info = PointerInfo { ray: Ray { origin: p - DVec3::X * 10.0, dir: DVec3::X }, point: Some(p), plane_point: Some(p), clicked: true, world_per_px: 0.01 };
            let done = matches!(r.pointer(&doc, &cp, &info), ToolOutcome::Finished);
            assert_eq!(done, last);
        }
        assert!(r.commit(&mut doc));
        let w = &doc.objects[0].body;
        // The rectangle lies on the face, at x = 0.
        assert!(w.vertices.iter().all(|v| v.point.x.abs() < 1e-9), "{:?}", w.vertices);
        set_current_cplane(CPlane::default());
    }

    #[test]
    fn move_tool_pushes_a_face() {
        let mut doc = Document::default();
        let id = doc.add_body("Box", ductile_brep::primitives::make_box(&ductile_geom::Frame::WORLD, DVec3::new(2.0, 2.0, 1.0)));
        let b = doc.object(id).unwrap().body.clone();
        let top = b.face_ids().find(|f| b.face_normal(*f, 0.5, 0.5).z > 0.9).unwrap();
        doc.mode = SelectMode::Face;
        doc.selection.insert(Pick::Face(id, top));
        let mut t = TransformTool::new(TransformKind::Move, &doc);
        t.axis = Some(2);
        t.typed = Some(0.5);
        assert!(t.commit(&mut doc));
        let v = volume(&doc.object(id).unwrap().body);
        assert!((v - 6.0).abs() < 1e-9, "{v}");
        // The top of a cylinder rides up its curved side.
        let mut doc = Document::default();
        let id = doc.add_body("Cyl", ductile_brep::primitives::make_cylinder(&ductile_geom::Frame::WORLD, 1.0, 2.0));
        let b = doc.object(id).unwrap().body.clone();
        let top = b.face_ids().find(|f| matches!(b.face(*f).surface, ductile_geom::Surface::Plane { .. }) && b.face_normal(*f, 0.0, 0.0).z > 0.9).unwrap();
        doc.mode = SelectMode::Face;
        doc.selection.insert(Pick::Face(id, top));
        let mut t = TransformTool::new(TransformKind::Move, &doc);
        t.axis = Some(2);
        t.typed = Some(1.0);
        assert!(t.commit(&mut doc));
        let v = volume(&doc.object(id).unwrap().body);
        assert!((v - 3.0 * std::f64::consts::PI).abs() < 5e-3, "{v}");
    }

    #[test]
    fn cuts_a_box_with_a_drawn_line() {
        set_current_cplane(CPlane::default());
        let mut doc = Document::default();
        let b = doc.add_body("Box", ductile_brep::primitives::make_box(&ductile_geom::Frame::WORLD, DVec3::new(4.0, 3.0, 2.0)));
        let line = ductile_brep::profile::wire_from_segments(&[ductile_brep::profile::Segment::line(DVec3::new(1.0, -1.0, 0.0), DVec3::new(3.0, 4.0, 0.0))]);
        let l = doc.add_body("Line", line);
        doc.selection.insert(Pick::Object(b));
        doc.selection.insert(Pick::Object(l));
        let (ids, pieces) = cut_selected(&doc).unwrap();
        assert_eq!(ids, vec![b]);
        assert_eq!(pieces[0].len(), 2);
        let total: f64 = pieces[0].iter().map(|p| ductile_brep::tessellate(p, &ductile_brep::TessOptions::default()).volume()).sum();
        assert!((total - 24.0).abs() < 1e-9, "{total}");
    }

    #[test]
    fn patch_caps_a_tube_tangentially() {
        use ductile_brep::profile::{Loop3, Segment};
        let circle = Loop3 { segments: vec![Segment::new(ductile_geom::Curve3::Circle { frame: ductile_geom::Frame::WORLD, radius: 1.0 }, 0.0, std::f64::consts::TAU)] };
        let tube = ductile_brep::ops::sheet::extrude_sheet(&[circle], DVec3::new(0.0, 0.0, 2.0)).unwrap();
        let mut doc = Document::default();
        let id = doc.add_body("Tube", tube);
        let b = doc.object(id).unwrap().body.clone();
        let rim = b.edge_ids().find(|e| b.edge(*e).mid().z > 1.9 && b.edge(*e).is_closed()).unwrap();
        doc.selection.insert(Pick::Edge(id, rim));
        let cap = patch_selected(&doc).unwrap();
        let ductile_geom::Surface::Nurbs(s) = &cap.faces[0].surface else { panic!() };
        // A dome over the rim, meeting the tube along it.
        assert!(s.point(0.5, 0.5).z > 2.2, "{}", s.point(0.5, 0.5));
        // (The four corners split a smooth circle, so the patch's normal is
        // undefined there.)
        for k in 1..8 {
            let t = k as f64 / 8.0;
            for (u, v) in [(t, 0.0), (1.0, t), (t, 1.0), (0.0, t)] {
                let d = s.derivs(u, v, 1);
                let p = d[0][0];
                let n = d[1][0].cross(d[0][1]).normalize();
                let radial = DVec3::new(p.x, p.y, 0.0).normalize();
                assert!((p.z - 2.0).abs() < 1e-6 && n.dot(radial).abs() > 0.9999, "{p} {n}");
            }
        }
    }

    #[test]
    fn radial_and_linear_arrays() {
        let mut doc = Document::default();
        let id = doc.add_body("Box", ductile_brep::primitives::make_box(&ductile_geom::Frame { origin: DVec3::new(3.0, 0.0, 0.0), ..ductile_geom::Frame::WORLD }, DVec3::ONE));
        doc.selection.insert(Pick::Object(id));
        let mut r = ArrayTool::new(&doc, true).unwrap();
        r.count = 4;
        assert!(r.commit(&mut doc));
        assert_eq!(doc.objects.len(), 4);
        // The copy a quarter turn round sits on +Y.
        let c = doc.objects[1].body.bbox().center();
        assert!((c - DVec3::new(-0.5, 3.5, 0.5)).length() < 1e-9, "{c}");
        doc.selection.clear();
        doc.selection.insert(Pick::Object(id));
        let mut l = ArrayTool::new(&doc, false).unwrap();
        l.count = 3;
        l.typed_value(Some(2.0));
        assert!(l.commit(&mut doc));
        assert_eq!(doc.objects.len(), 6);
        let c = doc.objects[5].body.bbox().center();
        assert!((c - DVec3::new(7.5, 0.5, 0.5)).length() < 1e-9, "{c}");
    }

    #[test]
    fn offset_a_cylinder_side() {
        let mut doc = Document::default();
        let id = doc.add_body("Cyl", ductile_brep::primitives::make_cylinder(&ductile_geom::Frame::WORLD, 1.0, 2.0));
        let b = doc.object(id).unwrap().body.clone();
        let side = b.face_ids().find(|f| matches!(b.face(*f).surface, ductile_geom::Surface::Cylinder { .. })).unwrap();
        doc.selection.insert(Pick::Face(id, side));
        assert!(ExtrudeTool::new(&doc).is_none());
        let mut t = ShellTool::new(&doc, OffsetMode::Faces).unwrap();
        t.typed_value(Some(0.5));
        assert!(t.commit(&mut doc));
        let v = ductile_brep::tessellate(&doc.object(id).unwrap().body, &ductile_brep::TessOptions { chord_tol: 1e-4, angle_tol: 0.05 }).volume();
        assert!((v - std::f64::consts::PI * 2.25 * 2.0).abs() < 2e-3, "{v}");
    }

    #[test]
    fn text_extrudes_into_letters_with_holes() {
        set_current_cplane(CPlane::default());
        let mut doc = Document::default();
        let mut t = TextTool::new();
        t.text = "Ob".into();
        t.height = 2.0;
        click(&mut t, &doc, DVec3::ZERO, true);
        assert!(t.commit(&mut doc));
        // Two letters, each a region with a hole.
        assert_eq!(selected_curve_regions(&doc).len(), 2);
        let mut e = ExtrudeTool::new(&doc).unwrap();
        e.typed_value(Some(0.5));
        assert!(e.commit(&mut doc));
        let solids: Vec<_> = doc.objects.iter().filter(|o| o.body.kind == ductile_brep::BodyKind::Solid).collect();
        assert!(!solids.is_empty());
        let v: f64 = solids.iter().map(|o| volume(&o.body).abs()).sum();
        assert!(v > 0.3 && v < 3.0, "{v}");
    }

    #[test]
    fn rectangle_with_circle_hole_extrudes() {
        set_current_cplane(CPlane::default());
        let mut doc = Document::default();
        let mut r = CurveTool::new(CurveKind::Rectangle);
        click(&mut r, &doc, DVec3::ZERO, true);
        assert!(click(&mut r, &doc, DVec3::new(4.0, 3.0, 0.0), true));
        assert!(r.commit(&mut doc));
        let rect = doc.objects[0].id;
        let mut c = CurveTool::new(CurveKind::Circle);
        click(&mut c, &doc, DVec3::new(2.0, 1.5, 0.0), true);
        c.typed_value(Some(1.0));
        assert!(c.commit(&mut doc));
        doc.selection.insert(Pick::Object(rect));
        assert_eq!(selected_curve_regions(&doc).len(), 1);
        let mut e = ExtrudeTool::new(&doc).unwrap();
        e.typed_value(Some(2.0));
        assert!(e.commit(&mut doc));
        let solid = doc.objects.last().unwrap();
        let expect = (12.0 - std::f64::consts::PI) * 2.0;
        assert!((volume(&solid.body).abs() - expect).abs() < 1e-2, "{}", volume(&solid.body));
    }

    #[test]
    fn extruding_a_curve_on_a_face_joins_or_cuts_the_solid() {
        use ductile_brep::profile::{wire_from_segments, Segment};
        let circle = circle_at(DVec3::new(2.0, 1.5, 1.0), 1.0);
        for (d, expect) in [(0.5, 12.0 + 0.5 * std::f64::consts::PI), (-0.5, 12.0 - 0.5 * std::f64::consts::PI)] {
            let mut doc = Document::default();
            let bx = doc.add_body("Box", ductile_brep::primitives::make_box(&ductile_geom::Frame::WORLD, DVec3::new(4.0, 3.0, 1.0)));
            let c = doc.add_body("Circle", wire_from_segments(&[Segment::new(circle.clone(), 0.0, std::f64::consts::TAU)]));
            doc.selection.insert(Pick::Object(c));
            let mut e = ExtrudeTool::new(&doc).unwrap();
            e.typed_value(Some(d));
            assert!(e.commit(&mut doc));
            assert_eq!(doc.objects.len(), 2, "no new body");
            let v = volume(&doc.object(bx).unwrap().body);
            assert!((v - expect).abs() < 1e-2, "{d}: {v} vs {expect}");
        }
    }

    #[test]
    fn bridges_extends_and_joins_surfaces() {
        use ductile_brep::ops::bridge::{bridge_surface, Continuity, Side};
        use ductile_brep::profile::{Loop3, Segment};
        let p = |x: f64, z: f64| DVec3::new(x, 0.0, z);
        let sheet = |pts: &[DVec3]| {
            let c = ductile_geom::NurbsCurve::interpolate(pts, 3);
            let (t0, t1) = c.domain();
            ductile_brep::ops::sheet::extrude_sheet(&[Loop3 { segments: vec![Segment::new(ductile_geom::Curve3::Nurbs(c), t0, t1)] }], DVec3::Y * 3.0).unwrap()
        };
        let (a, b) = (sheet(&[p(0.0, 0.0), p(1.5, 1.2), p(3.0, 1.6)]), sheet(&[p(8.0, 0.6), p(9.5, 0.9), p(11.0, 0.0)]));
        let edge_at = |s: &Body, x: f64| s.edge_ids().find(|e| s.edge(*e).curve.is_linear() && (s.edge(*e).mid().x - x).abs() < 1e-6).unwrap();
        let mut doc = Document::default();
        let ia = doc.add_body("A", a.clone());
        let ib = doc.add_body("B", b.clone());
        // Extend the first sheet's far edge, then bridge the gap with G2.
        doc.selection.insert(Pick::Edge(ia, edge_at(&a, 3.0)));
        let mut ext = ExtendSheetTool::new(&doc).unwrap();
        ext.typed_value(Some(1.0));
        assert!(ext.commit(&mut doc));
        let a2 = doc.object(ia).unwrap().body.clone();
        let far = a2.edge_ids().filter(|e| a2.edge(*e).curve.is_linear() || a2.edge_faces(*e).len() == 1).max_by(|x, y| a2.edge(*x).mid().x.total_cmp(&a2.edge(*y).mid().x)).unwrap();
        assert!(a2.edge(far).mid().x > 3.5);
        let sa = Side::of_edge(&a2, far, DVec3::new(8.0, 1.5, 0.6), Continuity::G2, 1.0);
        let sb = Side::of_edge(&b, edge_at(&b, 8.0), a2.edge(far).mid(), Continuity::G2, 1.0);
        let bridge = doc.add_body("Bridge", bridge_surface(&sa, &sb).unwrap());
        doc.selection.clear();
        for id in [ia, ib, bridge] {
            doc.selection.insert(Pick::Object(id));
        }
        let (id, solid) = join_sheets_selected(&mut doc).unwrap();
        assert!(!solid);
        let j = &doc.object(id).unwrap().body;
        assert_eq!(j.faces.len(), 3);
        assert_eq!(j.edge_ids().filter(|e| j.edge_faces(*e).len() == 2).count(), 2, "sewn along both bridged edges");
    }

    #[test]
    fn boolean_command_keys_and_slice() {
        let mut doc = Document::default();
        let a = doc.add_body("A", ductile_brep::primitives::make_box(&Frame::WORLD, DVec3::new(2.0, 2.0, 2.0)));
        let b = doc.add_body("B", ductile_brep::primitives::make_box(&Frame { origin: DVec3::new(1.0, 0.5, 0.5), ..Frame::WORLD }, DVec3::new(2.0, 1.0, 1.0)));
        doc.selection.insert(Pick::Object(a));
        doc.selection.insert(Pick::Object(b));
        let mut t = BooleanTool::new(&doc).unwrap();
        assert!(t.command_key(egui::Key::Q, egui::Modifiers::SHIFT));
        assert!(t.commit(&mut doc));
        assert!(doc.object(b).is_none(), "tool consumed");
        let vols: Vec<f64> = doc.objects.iter().map(|o| volume(&o.body)).collect();
        assert_eq!(vols.len(), 2);
        assert!((vols[0] - 7.0).abs() < 1e-6 && (vols[1] - 1.0).abs() < 1e-6, "{vols:?}");
    }

    #[test]
    fn offsets_faces_up_to_another() {
        let mut doc = Document::default();
        let a = doc.add_body("A", ductile_brep::primitives::make_box(&Frame::WORLD, DVec3::new(4.0, 3.0, 2.0)));
        let _ = doc.add_body("B", ductile_brep::primitives::make_box(&Frame { origin: DVec3::new(6.0, 0.0, 0.0), ..Frame::WORLD }, DVec3::new(1.0, 1.0, 3.0)));
        let b = doc.object(a).unwrap().body.clone();
        let top = b.face_ids().find(|f| b.face_normal(*f, 0.5, 0.5).z > 0.9).unwrap();
        doc.selection.insert(Pick::Face(a, top));
        let mut t = OffsetFaceToTool::new(&doc).unwrap();
        assert!(click(&mut t, &doc, DVec3::new(6.5, 0.5, 3.0), true));
        assert!(t.commit(&mut doc));
        assert!((volume(&doc.object(a).unwrap().body) - 36.0).abs() < 1e-6);
    }

    #[test]
    fn boolean_region_mode() {
        let mut doc = Document::default();
        let a = doc.add_body("A", ductile_brep::primitives::make_box(&Frame::WORLD, DVec3::new(2.0, 2.0, 2.0)));
        let b = doc.add_body("B", ductile_brep::primitives::make_box(&Frame { origin: DVec3::new(1.0, 0.5, 0.5), ..Frame::WORLD }, DVec3::new(2.0, 1.0, 1.0)));
        doc.selection.insert(Pick::Object(a));
        doc.selection.insert(Pick::Object(b));
        let mut t = BooleanTool::new(&doc).unwrap();
        assert!(t.command_key(egui::Key::R, egui::Modifiers::SHIFT));
        assert!(t.commit(&mut doc));
        let mut vols: Vec<f64> = doc.objects.iter().map(|o| volume(&o.body)).collect();
        vols.sort_by(f64::total_cmp);
        assert_eq!(vols.len(), 3, "{vols:?}");
        assert!((vols[0] - 1.0).abs() < 1e-6 && (vols[1] - 1.0).abs() < 1e-6 && (vols[2] - 7.0).abs() < 1e-6, "{vols:?}");
    }

    #[test]
    fn sketch_variants_close_up() {
        use std::f64::consts::PI;
        let v = |x: f64, y: f64| DVec3::new(x, y, 0.0);
        let f = Frame::WORLD;
        let ends = |s: &ductile_brep::profile::Segment| {
            let (a, b) = (s.curve.point(s.t0), s.curve.point(s.t1));
            if s.reversed { (b, a) } else { (a, b) }
        };
        for (kind, pts, typed, length) in [
            (CurveKind::Slot, vec![v(0.0, 0.0), v(4.0, 0.0), v(2.0, 1.0)], None, 8.0 + 2.0 * PI),
            (CurveKind::Slot, vec![v(0.0, 0.0), v(4.0, 0.0), v(9.0, 9.0)], Some(0.5), 8.0 + PI),
            (CurveKind::ThreePointCircle, vec![v(1.0, 0.0), v(-1.0, 0.0), v(0.0, 1.0)], None, 2.0 * PI),
            (CurveKind::TwoPointCircle, vec![v(1.0, 1.0), v(3.0, 1.0)], None, 2.0 * PI),
            (CurveKind::ThreePointRectangle, vec![v(0.0, 0.0), v(3.0, 4.0), v(0.0, 9.0)], Some(2.0), 14.0),
        ] {
            let mut t = CurveTool::new(kind);
            t.typed = typed;
            let segs = t.point_segments(&pts, &f);
            let total: f64 = segs.iter().map(|s| s.curve.length(s.t0, s.t1)).sum();
            assert!((total - length).abs() < 1e-6, "{kind:?}: {total} vs {length}");
            for (i, s) in segs.iter().enumerate() {
                let next = &segs[(i + 1) % segs.len()];
                assert!(ends(s).1.distance(ends(next).0) < 1e-9, "{kind:?}: gap after segment {i}");
            }
        }
        // A quarter arc counter-clockwise from +X about the center.
        let t = CurveTool::new(CurveKind::CenterArc);
        let segs = t.point_segments(&[v(0.0, 0.0), v(2.0, 0.0), v(0.0, 5.0)], &f);
        let s = &segs[0];
        assert!((s.curve.length(s.t0, s.t1) - PI).abs() < 1e-9 && s.curve.point(s.t1).distance(v(0.0, 2.0)) < 1e-9);
    }

    #[test]
    fn boolean_keys_inside_extrude() {
        use ductile_brep::profile::{wire_from_segments, Segment};
        use std::f64::consts::{PI, TAU};
        let circle = circle_at(DVec3::new(2.0, 1.5, 1.0), 1.0);
        // Pulled down into the box: Q joins (no change), W cuts, Shift+E
        // keeps the overlap, B leaves the box alone and adds the cylinder.
        for (key, shift, expect, objects) in [(egui::Key::Q, false, 12.0, 2), (egui::Key::W, false, 12.0 - 0.5 * PI, 2), (egui::Key::E, true, 0.5 * PI, 2), (egui::Key::B, false, 12.0, 3)] {
            let mut doc = Document::default();
            let bx = doc.add_body("Box", ductile_brep::primitives::make_box(&ductile_geom::Frame::WORLD, DVec3::new(4.0, 3.0, 1.0)));
            let c = doc.add_body("Circle", wire_from_segments(&[Segment::new(circle.clone(), 0.0, TAU)]));
            doc.selection.insert(Pick::Object(c));
            let mut e = ExtrudeTool::new(&doc).unwrap();
            let m = if shift { egui::Modifiers::SHIFT } else { egui::Modifiers::NONE };
            assert!(e.command_key(key, m));
            e.typed_value(crate::expr::eval("-1/2"));
            assert!(e.commit(&mut doc));
            assert_eq!(doc.objects.len(), objects, "{key:?}");
            let v = volume(&doc.object(bx).unwrap().body);
            assert!((v - expect).abs() < 1e-2, "{key:?}: {v} vs {expect}");
        }
    }

    fn circle_at(c: DVec3, r: f64) -> ductile_geom::Curve3 {
        ductile_geom::Curve3::Circle { frame: ductile_geom::Frame { origin: c, ..ductile_geom::Frame::WORLD }, radius: r }
    }

    #[test]
    fn closed_polyline_revolves_about_y() {
        set_current_cplane(CPlane::default());
        let mut doc = Document::default();
        let mut l = CurveTool::new(CurveKind::Polyline);
        for p in [DVec3::new(1.0, 0.0, 0.0), DVec3::new(2.0, 0.0, 0.0), DVec3::new(2.0, 3.0, 0.0), DVec3::new(1.0, 3.0, 0.0)] {
            assert!(!click(&mut l, &doc, p, true));
        }
        assert!(click(&mut l, &doc, DVec3::new(1.0, 0.0, 0.0), true));
        assert!(l.commit(&mut doc));
        let mut r = RevolveTool::new(&doc).unwrap();
        r.typed_value(Some(180.0));
        assert!(r.commit(&mut doc));
        let expect = std::f64::consts::PI / 2.0 * 3.0 * 3.0;
        assert!((volume(&doc.objects.last().unwrap().body) - expect).abs() < 1e-2);
    }

    /// Extrudes the selected curve objects by 1 and returns the volume.
    fn extrude_selected(doc: &mut Document) -> f64 {
        let mut e = ExtrudeTool::new(doc).unwrap();
        e.typed_value(Some(1.0));
        assert!(e.commit(doc));
        volume(&doc.objects.last().unwrap().body).abs()
    }

    #[test]
    fn ellipse_polygon_and_arc_profiles_extrude() {
        use std::f64::consts::PI;
        set_current_cplane(CPlane::default());
        let mut doc = Document::default();
        let mut el = CurveTool::new(CurveKind::Ellipse);
        click(&mut el, &doc, DVec3::ZERO, true);
        assert!(click(&mut el, &doc, DVec3::new(3.0, 1.0, 0.0), true));
        assert!(el.commit(&mut doc));
        assert!((extrude_selected(&mut doc) - 3.0 * PI).abs() < 1e-2);

        let mut pg = CurveTool::new(CurveKind::Polygon);
        click(&mut pg, &doc, DVec3::new(10.0, 0.0, 0.0), true);
        pg.typed_value(Some(2.0));
        assert!(pg.commit(&mut doc));
        assert!((extrude_selected(&mut doc) - 1.5 * 3f64.sqrt() * 4.0).abs() < 1e-6);

        // A half disk from an arc and a line drawn as separate curves.
        let mut arc = CurveTool::new(CurveKind::Arc);
        assert!(!click(&mut arc, &doc, DVec3::new(20.0, 0.0, 0.0), true));
        assert!(!click(&mut arc, &doc, DVec3::new(24.0, 0.0, 0.0), true));
        assert!(click(&mut arc, &doc, DVec3::new(22.0, 2.0, 0.0), true));
        assert!(arc.commit(&mut doc));
        let a = doc.objects.last().unwrap().id;
        let mut line = CurveTool::new(CurveKind::Polyline);
        click(&mut line, &doc, DVec3::new(24.0, 0.0, 0.0), true);
        click(&mut line, &doc, DVec3::new(20.0, 0.0, 0.0), true);
        line.accept();
        assert!(line.commit(&mut doc));
        doc.selection.insert(Pick::Object(a));
        assert!((extrude_selected(&mut doc) - 2.0 * PI).abs() < 1e-2);
    }

    #[test]
    fn loft_and_sweep_from_drawn_curves() {
        use std::f64::consts::PI;
        set_current_cplane(CPlane::default());
        let mut doc = Document::default();
        // Two squares drawn on the plane, the second lifted 3 up.
        for (c, h) in [(DVec3::new(0.0, 0.0, 3.0), 1.0), (DVec3::ZERO, 2.0)] {
            let mut r = CurveTool::new(CurveKind::Rectangle);
            click(&mut r, &doc, c - DVec3::new(h, h, 0.0), true);
            assert!(click(&mut r, &doc, c + DVec3::new(h, h, 0.0), true));
            assert!(r.commit(&mut doc));
        }
        doc.selection = doc.objects.iter().map(|o| Pick::Object(o.id)).collect();
        let b = loft_selected(&doc).unwrap();
        assert!((volume(&b) - 28.0).abs() < 1e-6, "{}", volume(&b));

        // A circle swept along a polyline leg.
        let mut doc = Document::default();
        let mut c = CurveTool::new(CurveKind::Circle);
        click(&mut c, &doc, DVec3::ZERO, true);
        c.typed_value(Some(0.5));
        assert!(c.commit(&mut doc));
        // The path runs off the construction plane, so build it directly.
        let path = ductile_brep::profile::wire_from_segments(&[ductile_brep::profile::Segment::line(DVec3::new(0.0, 0.0, 4.0), DVec3::ZERO)]);
        doc.add_body("Path", path);
        doc.selection = doc.objects.iter().map(|o| Pick::Object(o.id)).collect();
        let b = sweep_selected(&doc).unwrap();
        assert!((volume(&b) - PI * 0.25 * 4.0).abs() < 1e-2, "{}", volume(&b));
    }

    #[test]
    fn move_tool_drags_selected_control_points() {
        set_current_cplane(CPlane::default());
        let mut doc = Document::default();
        let mut sp = CurveTool::new(CurveKind::Spline);
        for p in [DVec3::ZERO, DVec3::new(2.0, 1.0, 0.0), DVec3::new(4.0, 0.0, 0.0)] {
            click(&mut sp, &doc, p, true);
        }
        sp.accept();
        assert!(sp.commit(&mut doc));
        let id = doc.objects[0].id;
        let e = ductile_brep::EdgeId(0);
        doc.selection.clear();
        doc.selection.insert(Pick::Control(id, e, 1));
        let mut t = TransformTool::new(TransformKind::Move, &doc);
        t.axis = Some(2);
        t.typed_value(Some(1.5));
        assert!(t.commit(&mut doc));
        let b = &doc.objects[0].body;
        let ductile_geom::Curve3::Nurbs(n) = &b.edge(e).curve else { panic!() };
        let before = ductile_geom::NurbsCurve::interpolate(&[DVec3::ZERO, DVec3::new(2.0, 1.0, 0.0), DVec3::new(4.0, 0.0, 0.0)], 3);
        assert!((n.control_points()[1] - before.control_points()[1] - DVec3::Z * 1.5).length() < 1e-9);
        assert!((n.control_points()[0] - before.control_points()[0]).length() < 1e-12);
    }

    #[test]
    fn closed_spline_extrudes_to_its_area() {
        set_current_cplane(CPlane::default());
        let mut doc = Document::default();
        let mut sp = CurveTool::new(CurveKind::Spline);
        let pts = [DVec3::new(0.0, 0.0, 0.0), DVec3::new(4.0, -1.0, 0.0), DVec3::new(5.0, 3.0, 0.0), DVec3::new(1.0, 4.0, 0.0), DVec3::new(-1.0, 2.0, 0.0)];
        for p in pts {
            assert!(!click(&mut sp, &doc, p, true));
        }
        assert!(click(&mut sp, &doc, pts[0], true));
        assert!(sp.commit(&mut doc));
        let c = ductile_geom::NurbsCurve::interpolate_closed(&pts);
        let n = 20000;
        let poly: Vec<DVec3> = (0..n).map(|i| c.point(i as f64 / n as f64)).collect();
        let area: f64 = (0..n).map(|i| poly[i].truncate().perp_dot(poly[(i + 1) % n].truncate()) / 2.0).sum::<f64>().abs();
        let v = extrude_selected(&mut doc);
        assert!((v - area).abs() < 1e-3 * area, "{v} vs {area}");
    }

    #[test]
    fn tangent_arc_leaves_a_line_smoothly() {
        let mut doc = Document::default();
        doc.add_body("Line", ductile_brep::profile::wire_from_segments(&[ductile_brep::profile::Segment::line(DVec3::ZERO, DVec3::X * 2.0)]));
        let mut t = CurveTool::new(CurveKind::TangentArc);
        // Click near the line's end: the arc starts there, leaving along +X.
        assert!(!click(&mut t, &doc, DVec3::new(2.02, 0.01, 0.0), true));
        assert!(click(&mut t, &doc, DVec3::new(3.0, 1.0, 0.0), true));
        let segs = t.segments();
        assert_eq!(segs.len(), 1);
        let s = &segs[0];
        assert!(s.start().distance(DVec3::X * 2.0) < 1e-9 && s.end().distance(DVec3::new(3.0, 1.0, 0.0)) < 1e-9);
        assert!(s.tangent_at(s.t0).normalize().distance(DVec3::X) < 1e-9, "{}", s.tangent_at(s.t0));
        // A quarter circle of radius 1.
        let ductile_geom::Curve3::Circle { radius, .. } = s.curve else { panic!() };
        assert!((radius - 1.0).abs() < 1e-9 && (s.t1 - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
    }

    #[test]
    fn curve_array_follows_and_turns_with_a_circle() {
        let mut doc = Document::default();
        // A small box at the circle's start (radius 5, starting at +X).
        let bx = doc.add_body("Box", ductile_brep::primitives::make_box(&ductile_geom::Frame { origin: DVec3::new(4.9, -0.1, 0.0), ..ductile_geom::Frame::WORLD }, DVec3::splat(0.2)));
        let c = doc.add_body("Circle", ductile_brep::profile::wire_from_segments(&[ductile_brep::profile::Segment::new(ductile_geom::Curve3::Circle { frame: ductile_geom::Frame::WORLD, radius: 5.0 }, 0.0, std::f64::consts::TAU)]));
        doc.selection = [Pick::Object(bx), Pick::Object(c)].into_iter().collect();
        let mut t = CurveArrayTool::new(&doc).unwrap();
        t.count = 4;
        assert!(t.commit(&mut doc));
        assert_eq!(doc.objects.len(), 5);
        // Copies a quarter turn apart, at the circle's +Y, -X and -Y.
        let centers: Vec<DVec3> = doc.objects[2..].iter().map(|o| o.body.bbox().center()).collect();
        for want in [DVec3::new(0.0, 5.0, 0.1), DVec3::new(-5.0, 0.0, 0.1), DVec3::new(0.0, -5.0, 0.1)] {
            assert!(centers.iter().any(|c| c.distance(want) < 1e-2), "{centers:?}");
        }
    }

    #[test]
    fn tangent_circle_between_two_lines_and_inside_a_triangle() {
        use ductile_brep::profile::{wire_from_segments, Segment};
        let mut doc = Document::default();
        // An L of two lines meeting at the origin, and a diagonal closing it.
        doc.add_body("X", wire_from_segments(&[Segment::line(DVec3::ZERO, DVec3::new(10.0, 0.0, 0.0))]));
        doc.add_body("Y", wire_from_segments(&[Segment::line(DVec3::ZERO, DVec3::new(0.0, 10.0, 0.0))]));
        let mut t = TangentCircleTool::new(&doc);
        click(&mut t, &doc, DVec3::new(5.0, 0.0, 0.0), true);
        click(&mut t, &doc, DVec3::new(0.0, 5.0, 0.0), true);
        assert_eq!(t.picked.len(), 2);
        // The cursor at (3, 2) sizes the circle to the bisector's (2.5, 2.5).
        click(&mut t, &doc, DVec3::new(3.0, 2.0, 0.0), false);
        assert!((t.radius - 2.5).abs() < 1e-6, "{}", t.radius);
        t.typed_value(Some(1.0));
        assert!(click(&mut t, &doc, DVec3::new(3.0, 2.0, 0.0), true));
        assert!(t.commit(&mut doc));
        let ductile_geom::Curve3::Circle { frame, radius } = &doc.objects.last().unwrap().body.edges[0].curve else { panic!() };
        assert!(frame.origin.distance(DVec3::new(1.0, 1.0, 0.0)) < 1e-6 && (*radius - 1.0).abs() < 1e-12, "{}", frame.origin);
        // The incircle of the 3-4-5 triangle has radius 1 at (1, 1).
        let ts = [
            Tangency::from_edge(&ductile_geom::Curve3::Line { origin: DVec3::ZERO, dir: DVec3::X }, 0.0, 4.0),
            Tangency::from_edge(&ductile_geom::Curve3::Line { origin: DVec3::ZERO, dir: DVec3::Y }, 0.0, 3.0),
            Tangency::from_edge(&ductile_geom::Curve3::Line { origin: DVec3::new(4.0, 0.0, 0.0), dir: DVec3::new(-4.0, 3.0, 0.0).normalize() }, 0.0, 5.0),
        ];
        let (c, r) = tangent_circle(&ts, DVec3::new(1.5, 0.8, 0.0), None, &Frame::WORLD).unwrap();
        assert!((r - 1.0).abs() < 1e-9 && c.distance(DVec3::new(1.0, 1.0, 0.0)) < 1e-9, "{c} {r}");
        // Tangent to a circle of radius 3 and the X axis, radius 1, outside.
        let ts = [
            Tangency::from_edge(&ductile_geom::Curve3::Circle { frame: Frame { origin: DVec3::new(0.0, 5.0, 0.0), ..Frame::WORLD }, radius: 3.0 }, 0.0, std::f64::consts::TAU),
            ts[0].clone(),
        ];
        let (c, r) = tangent_circle(&ts, DVec3::new(2.0, 1.0, 0.0), Some(1.0), &Frame::WORLD).unwrap();
        assert!((r - 1.0).abs() < 1e-12 && (c.y - 1.0).abs() < 1e-9 && (c.distance(DVec3::new(0.0, 5.0, 0.0)) - 4.0).abs() < 1e-9, "{c}");
    }

    #[test]
    fn place_stands_copies_on_the_face_under_the_cursor() {
        let mut doc = Document::default();
        // A tall box on the ground, and a cube to place on its +X side.
        let wall = doc.add_body("Wall", ductile_brep::primitives::make_box(&ductile_geom::Frame { origin: DVec3::new(-10.0, -1.0, 0.0), ..ductile_geom::Frame::WORLD }, DVec3::new(10.0, 2.0, 4.0)));
        let cube = doc.add_body("Cube", ductile_brep::primitives::make_box(&ductile_geom::Frame { origin: DVec3::new(5.0, 5.0, 0.0), ..ductile_geom::Frame::WORLD }, DVec3::splat(1.0)));
        doc.selection = [Pick::Object(cube)].into_iter().collect();
        let mut t = PlaceTool::new(&doc).unwrap();
        // Looking along -X at the wall's side, at (0, 0, 2).
        let info = PointerInfo { ray: Ray { origin: DVec3::new(10.0, 0.0, 2.0), dir: -DVec3::X }, point: None, plane_point: None, clicked: true, world_per_px: 0.01 };
        t.pointer(&doc, &CPlane::default(), &info);
        // And once on the top.
        click(&mut t, &doc, DVec3::new(-5.0, 0.0, 4.0), true);
        assert!(t.commit(&mut doc));
        assert_eq!(doc.objects.len(), 4);
        let side = doc.objects[2].body.bbox();
        assert!(side.min.x.abs() < 2e-3 && (side.max.x - 1.0).abs() < 2e-3 && (side.center().z - 2.0).abs() < 1e-9, "{side:?}");
        let top = doc.objects[3].body.bbox();
        assert!((top.min.z - 4.0).abs() < 2e-3 && (top.center().x + 5.0).abs() < 1e-9, "{top:?}");
        assert!((doc.objects[3].body.vertices.iter().map(|v| v.point.z).fold(f64::MAX, f64::min) - 4.0).abs() < 1e-9);
        let _ = wall;
    }

    #[test]
    fn dimension_places_a_line_off_two_points_and_survives_undo_and_save() {
        let mut doc = Document::default();
        let mut t = DimensionTool::new();
        click(&mut t, &doc, DVec3::ZERO, true);
        click(&mut t, &doc, DVec3::new(3.0, 4.0, 0.0), true);
        assert!(click(&mut t, &doc, DVec3::new(-4.0, 3.0, 0.0) + DVec3::new(1.5, 2.0, 0.0), true));
        doc.checkpoint("Dimension");
        assert!(t.commit(&mut doc));
        let d = doc.dimensions[0];
        assert!(d.offset.distance(DVec3::new(-4.0, 3.0, 0.0)) < 1e-9, "{:?}", d.offset);
        assert_eq!(dimension_label(&d).1, "5");
        let path = std::env::temp_dir().join("ductile_dimension_test.ductile");
        crate::commands::write_native(&doc, &path).unwrap();
        let mut back = Document::default();
        crate::commands::load_into(&mut back, &path).unwrap();
        assert_eq!(back.dimensions, doc.dimensions);
        doc.undo();
        assert!(doc.dimensions.is_empty());
    }

    #[test]
    fn full_round_rounds_two_rib_tops() {
        let mut doc = Document::default();
        let a = doc.add_body("Rib", ductile_brep::primitives::make_box(&ductile_geom::Frame::WORLD, DVec3::new(6.0, 2.0, 3.0)));
        let b = doc.object(a).unwrap().body.clone();
        let top = b.face_ids().find(|f| b.face_normal(*f, 0.5, 0.5).z > 0.9).unwrap();
        doc.selection = [Pick::Face(a, top)].into_iter().collect();
        assert_eq!(full_round_selected(&mut doc), Ok(1));
        assert!(doc.object(a).unwrap().body.faces.iter().any(|f| matches!(f.surface, ductile_geom::Surface::Cylinder { .. })));
    }

    #[test]
    fn align_curve_meets_a_line_end_with_g2() {
        use ductile_brep::profile::{wire_from_segments, Segment};
        let mut doc = Document::default();
        doc.add_body("Line", wire_from_segments(&[Segment::line(DVec3::ZERO, DVec3::new(4.0, 0.0, 0.0))]));
        let pts = [DVec3::new(5.0, 1.0, 0.0), DVec3::new(6.0, 2.0, 0.0), DVec3::new(6.5, 4.0, 0.0), DVec3::new(6.0, 6.0, 0.0)];
        let sp = ductile_geom::NurbsCurve::uniform(3, &pts);
        let id = doc.add_body("Spline", wire_from_segments(&[Segment::new(ductile_geom::Curve3::Nurbs(sp), 0.0, 1.0)]));
        let mut t = AlignCurveTool::new(&doc);
        // Click the spline's start, then the line.
        click(&mut t, &doc, DVec3::new(5.0, 1.0, 0.0), true);
        assert!(t.moving.is_some());
        assert!(click(&mut t, &doc, DVec3::new(3.0, 0.0, 0.0), true));
        assert!(t.commit(&mut doc));
        let e = &doc.object(id).unwrap().body.edges[0];
        let c = e.curve.to_nurbs(e.t0, e.t1);
        let d = c.derivs(c.domain().0, 2);
        assert!(d[0].distance(DVec3::new(4.0, 0.0, 0.0)) < 1e-9);
        assert!(d[1].normalize().distance(DVec3::X) < 1e-9);
        assert!(d[1].cross(d[2]).length() < 1e-9, "curvature zero, as the line's");
    }

    #[test]
    fn raise_and_rebuild_selected_faces() {
        let mut doc = Document::default();
        let id = doc.add_body("Cyl", ductile_brep::primitives::make_cylinder(&ductile_geom::Frame::WORLD, 1.0, 2.0));
        let b = doc.object(id).unwrap().body.clone();
        let side = b.face_ids().find(|f| !b.face(*f).surface.is_planar()).unwrap();
        doc.selection = [Pick::Face(id, side)].into_iter().collect();
        assert_eq!(raise_selected_face_degree(&mut doc), Ok(1));
        assert!(matches!(doc.object(id).unwrap().body.face(side).surface, ductile_geom::Surface::Nurbs(_)));
        doc.selection = [Pick::Face(id, side)].into_iter().collect();
        let mut t = RebuildFaceTool::new(&doc).unwrap();
        t.nu = 10;
        t.nv = 3;
        assert!(t.commit(&mut doc));
        let ductile_geom::Surface::Nurbs(n) = &doc.object(id).unwrap().body.face(side).surface else { panic!() };
        assert_eq!((n.nu, n.nv), (10, 3));
        assert!(doc.object(id).unwrap().mesh.triangle_count() > 0);
    }

    #[test]
    fn isoparam_untrim_unjoin_and_boundary_edges() {
        let mut doc = Document::default();
        let id = doc.add_body("Box", ductile_brep::primitives::make_box(&ductile_geom::Frame::WORLD, DVec3::new(4.0, 2.0, 1.0)));
        // Click the top at x = 1: split it there along the surface's u.
        let mut t = IsoparamTool::new();
        assert!(click(&mut t, &doc, DVec3::new(1.0, 1.0, 1.0), true));
        assert!(t.commit(&mut doc));
        let b = &doc.object(id).unwrap().body;
        assert_eq!(b.faces.len(), 7);
        assert!(b.edge_ids().any(|e| { let ed = b.edge(e); (ed.start().x - 1.0).abs() < 1e-6 && (ed.end().x - 1.0).abs() < 1e-6 && ed.start().z > 0.99 }));
        // Pull the two top halves out: the box becomes an open sheet with a gap.
        let tops: Vec<FaceId> = b.face_ids().filter(|f| b.face_normal(*f, 0.5, 0.5).z > 0.9).collect();
        assert_eq!(tops.len(), 2);
        doc.selection = tops.iter().map(|f| Pick::Face(id, *f)).collect();
        let n = untrim_selected(&mut doc).unwrap();
        assert_eq!(n, 2);
        doc.selection = tops.iter().map(|f| Pick::Face(id, *f)).collect();
        unjoin_selected(&mut doc).unwrap();
        assert_eq!(doc.object(id).unwrap().body.kind, ductile_brep::BodyKind::Sheet);
        doc.selection = [Pick::Object(id)].into_iter().collect();
        assert_eq!(find_boundary_edges(&mut doc), 6, "the long top edges were split at the isoparam");
        doc.selection = [Pick::Object(id)].into_iter().collect();
        assert_eq!(reverse_selected_sheets(&mut doc), 1);
        doc.selection = [Pick::Object(id)].into_iter().collect();
        unjoin_selected(&mut doc).unwrap();
        assert!(doc.object(id).is_none());
        assert_eq!(doc.objects.len(), 4 + 5);
    }
}

// ------------------------------------------------------------- Isoparam ----

/// Isoparam: hover a face to see where its surface's iso-parametric curve
/// under the cursor runs; Tab swaps u and v; click splits the face there.
pub struct IsoparamTool {
    /// The face under the cursor, whether the curve is at constant u, and
    /// where across the face it sits (0 to 1).
    target: Option<(ObjectId, FaceId)>,
    along_u: bool,
    param: f64,
    typed: Option<f64>,
    cut: Option<ductile_brep::profile::Segment>,
    error: Option<String>,
}

impl IsoparamTool {
    pub fn new() -> Self {
        Self { target: None, along_u: true, param: 0.5, typed: None, cut: None, error: None }
    }

    fn fraction(&self) -> f64 {
        self.typed.unwrap_or(self.param).clamp(1e-4, 1.0 - 1e-4)
    }

    /// The split body and its new edge, for the current face and place.
    fn split(&self, doc: &Document) -> Result<(Body, ductile_brep::profile::Segment), String> {
        let (o, f) = self.target.ok_or("hover a face")?;
        let ob = doc.object(o).ok_or("the object is gone")?;
        let (lo, hi) = ductile_brep::ops::isoparam::face_param_range(&ob.body, f, self.along_u).ok_or("the face has no extent")?;
        ductile_brep::ops::isoparam::split_face_iso_cut(&ob.body, f, self.along_u, lo + (hi - lo) * self.fraction())
    }

    fn refresh(&mut self, doc: &Document) {
        match self.split(doc) {
            Ok((_, seg)) => {
                self.cut = Some(seg);
                self.error = None;
            }
            Err(e) => {
                self.cut = None;
                self.error = self.target.map(|_| e);
            }
        }
    }
}

impl Tool for IsoparamTool {
    fn name(&self) -> &'static str {
        "Isoparam"
    }
    fn hint(&self) -> String {
        let dir = if self.along_u { "u" } else { "v" };
        match &self.error {
            Some(e) => format!("Isoparam: {e}"),
            None => format!("Isoparam: hover a face, click to split it along {dir} = {:.3} of the way across; Tab swaps u and v, type a value from 0 to 1", self.fraction()),
        }
    }
    fn pointer(&mut self, doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        let hit = crate::picking::pick_face(doc, &p.ray).filter(|h| !doc.is_locked(h.object) && doc.object(h.object).is_some_and(|o| o.body.mesh.is_none()));
        if let Some(h) = &hit {
            if let Some(ob) = doc.object(h.object) {
                let s = &ob.body.face(h.face).surface;
                if let Some((lo, hi)) = ductile_brep::ops::isoparam::face_param_range(&ob.body, h.face, self.along_u) {
                    // Project near the face's middle so periodic surfaces land in its chart.
                    let other = ductile_brep::ops::isoparam::face_param_range(&ob.body, h.face, !self.along_u).unwrap_or((lo, hi));
                    let mid = if self.along_u { ductile_geom::DVec2::new(0.5 * (lo + hi), 0.5 * (other.0 + other.1)) } else { ductile_geom::DVec2::new(0.5 * (other.0 + other.1), 0.5 * (lo + hi)) };
                    let uv = s.align_periodic(s.project(h.point, Some(mid)), mid);
                    let k = if self.along_u { uv.x } else { uv.y };
                    if self.typed.is_none() {
                        self.param = ((k - lo) / (hi - lo)).clamp(0.0, 1.0);
                    }
                }
            }
            self.target = Some((h.object, h.face));
            self.refresh(doc);
        }
        if p.clicked && self.cut.is_some() {
            return ToolOutcome::Finished;
        }
        ToolOutcome::Continue
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        None
    }
    fn guides(&self, out: &mut Vec<LineInstance>) {
        if let Some(seg) = &self.cut {
            let pts = seg.sample(1e-3);
            for w in pts.windows(2) {
                out.push(LineInstance::new(w[0], w[1], [1.0, 0.85, 0.3, 1.0], 2.5));
            }
        }
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut ch = false;
        ui.horizontal(|ui| {
            ui.label("Direction");
            ch |= ui.radio_value(&mut self.along_u, true, "U").changed();
            ch |= ui.radio_value(&mut self.along_u, false, "V").changed();
        });
        let mut v = self.fraction();
        if drag_value(ui, "Param", &mut v, 0.005) {
            self.typed = Some(v.clamp(0.0, 1.0));
            ch = true;
        }
        ch
    }
    fn typed_value(&mut self, v: Option<f64>) {
        self.typed = v.map(|x| x.clamp(0.0, 1.0));
    }
    fn command_key(&mut self, key: egui::Key, _m: egui::Modifiers) -> bool {
        if key == egui::Key::Tab {
            self.along_u = !self.along_u;
            self.cut = None;
            return true;
        }
        false
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let Ok((b, _)) = self.split(doc) else { return false };
        let Some((o, _)) = self.target else { return false };
        doc.replace_body(o, b);
        true
    }
}

// ------------------------------------------------ Untrim, unjoin, reverse ----

/// Untrim: a new sheet of the whole surface under each selected face.
pub fn untrim_selected(doc: &mut Document) -> Result<usize, String> {
    let faces = doc.selected_faces();
    if faces.is_empty() {
        return Err("Select faces to untrim (face mode: 3)".into());
    }
    let sheets: Vec<Body> = faces.iter().filter_map(|(o, f)| ductile_brep::ops::untrim::untrim(&doc.object(*o)?.body, *f)).collect();
    if sheets.is_empty() {
        return Err("Could not untrim those faces".into());
    }
    doc.checkpoint("Untrim");
    doc.selection.clear();
    let n = sheets.len();
    for s in sheets {
        let id = doc.add_body("Untrimmed", s);
        doc.selection.insert(Pick::Object(id));
    }
    doc.mode = SelectMode::Object;
    Ok(n)
}

/// Full round: each selected rib top replaced by a half cylinder tangent to
/// the rib's sides.
pub fn full_round_selected(doc: &mut Document) -> Result<usize, String> {
    let faces = doc.selected_faces();
    if faces.is_empty() {
        return Err("Select the top faces of ribs to round (face mode: 3)".into());
    }
    // Faces are found again by a point on them after each round changes the body.
    let marks: Vec<(ObjectId, DVec3)> = faces
        .iter()
        .filter_map(|(o, f)| {
            let m = &doc.object(*o)?.mesh.faces.get(f.idx())?.positions;
            (!m.is_empty()).then(|| (*o, m.iter().copied().sum::<DVec3>() / m.len() as f64))
        })
        .collect();
    let mut bodies: std::collections::BTreeMap<ObjectId, Body> = Default::default();
    let mut n = 0;
    let mut err = None;
    for (o, p) in marks {
        let body = bodies.entry(o).or_insert_with(|| (*doc.object(o).unwrap().body).clone());
        let Some(f) = body.face_ids().filter(|f| body.face(*f).surface.is_planar()).min_by(|a, b| {
            let d = |f: &FaceId| { let s = &body.face(*f).surface; let uv = s.project(p, None); s.point(uv.x, uv.y).distance(p) };
            d(a).total_cmp(&d(b))
        }) else { continue };
        match ductile_brep::ops::fillet::full_round(body, f) {
            Ok(b) => {
                *body = b;
                n += 1;
            }
            Err(e) => err = Some(e.to_string()),
        }
    }
    if n == 0 {
        return Err(format!("Could not round those faces: {}", err.unwrap_or_default()));
    }
    doc.checkpoint("Full round");
    doc.selection.clear();
    for (o, b) in bodies {
        doc.replace_body(o, b);
        doc.selection.insert(Pick::Object(o));
    }
    doc.mode = SelectMode::Object;
    Ok(n)
}

/// Unjoin: selected faces pulled out of their bodies as sheets; selected
/// solids or sheets exploded into one sheet per face; selected curves
/// split into one curve per segment.
pub fn unjoin_selected(doc: &mut Document) -> Result<String, String> {
    use ductile_brep::ops::delete::{extract_edges, extract_faces};
    let faces = doc.selected_faces();
    let objects: Vec<ObjectId> = doc.selection.iter().filter_map(|p| if let Pick::Object(o) = p { Some(*o) } else { None }).collect();
    if faces.is_empty() && objects.is_empty() {
        return Err("Select faces, bodies or curves to unjoin".into());
    }
    doc.checkpoint("Unjoin");
    doc.selection.clear();
    let mut made = 0;
    let mut by_object: std::collections::BTreeMap<ObjectId, Vec<FaceId>> = Default::default();
    for (o, f) in faces {
        by_object.entry(o).or_default().push(f);
    }
    for (o, fs) in by_object {
        let Some(ob) = doc.object(o).cloned() else { continue };
        let rest: Vec<FaceId> = ob.body.face_ids().filter(|f| !fs.contains(f)).collect();
        for f in &fs {
            let id = doc.add_body("Face", extract_faces(&ob.body, &[*f]));
            doc.selection.insert(Pick::Object(id));
            made += 1;
        }
        if rest.is_empty() {
            doc.remove(o);
        } else {
            doc.replace_body(o, extract_faces(&ob.body, &rest));
        }
    }
    for o in objects {
        let Some(ob) = doc.object(o).cloned() else { continue };
        let parts: Vec<Body> = match ob.body.kind {
            ductile_brep::BodyKind::Wire => ob.body.edge_ids().map(|e| extract_edges(&ob.body, &[e])).collect(),
            _ => ob.body.face_ids().map(|f| extract_faces(&ob.body, &[f])).collect(),
        };
        if parts.len() < 2 {
            continue;
        }
        doc.remove(o);
        for p in parts {
            let id = doc.add_body(&ob.name, p);
            if let Some(x) = doc.objects.iter_mut().find(|x| x.id == id) {
                (x.group, x.color) = (ob.group, ob.color);
            }
            doc.selection.insert(Pick::Object(id));
            made += 1;
        }
    }
    doc.mode = SelectMode::Object;
    Ok(format!("Unjoined into {made} piece(s)"))
}

/// Find Boundary Edges: selects the open edges of the selected bodies (all
/// visible bodies when none is selected), the gaps keeping a body from
/// being a closed solid.
pub fn find_boundary_edges(doc: &mut Document) -> usize {
    let mut ids = doc.selected_objects();
    if ids.is_empty() {
        ids = doc.objects.iter().filter(|o| o.visible && o.body.kind != ductile_brep::BodyKind::Wire).map(|o| o.id).collect();
    }
    let mut picks = vec![];
    for id in ids {
        let Some(o) = doc.object(id) else { continue };
        if o.body.kind == ductile_brep::BodyKind::Wire {
            continue;
        }
        picks.extend(o.body.edge_ids().filter(|e| o.body.edge_faces(*e).len() == 1).map(|e| Pick::Edge(id, e)));
    }
    doc.selection.clear();
    doc.mode = SelectMode::Edge;
    let n = picks.len();
    doc.selection.extend(picks);
    n
}

/// Reverse Sheet: swaps the front and back of the selected sheets.
pub fn reverse_selected_sheets(doc: &mut Document) -> usize {
    let ids: Vec<ObjectId> = doc.selected_objects().into_iter().filter(|id| doc.object(*id).is_some_and(|o| o.body.kind == ductile_brep::BodyKind::Sheet)).collect();
    if ids.is_empty() {
        return 0;
    }
    doc.checkpoint("Reverse sheet");
    for id in &ids {
        let b = ductile_brep::ops::untrim::reverse_sheet(&doc.object(*id).unwrap().body);
        doc.replace_body(*id, b);
    }
    ids.len()
}

// ----------------------------------------------------------------- Text ----

/// Text: click where the baseline starts, type the text in the panel; the
/// letters become closed curves on the construction plane.
pub struct TextTool {
    origin: Option<DVec3>,
    cursor: Option<DVec3>,
    pub text: String,
    pub height: f64,
    /// A font file, or None for the built-in one.
    font: Option<std::path::PathBuf>,
    font_bytes: Option<std::sync::Arc<Vec<u8>>>,
    fonts: Option<Vec<std::path::PathBuf>>,
    cache: std::cell::RefCell<Option<(String, f64, DVec3, Vec<ductile_brep::profile::Segment>)>>,
}

impl TextTool {
    pub fn new() -> Self {
        Self { origin: None, cursor: None, text: "Ductile".into(), height: 1.0, font: None, font_bytes: None, fonts: None, cache: std::cell::RefCell::new(None) }
    }

    fn segments(&self) -> Vec<ductile_brep::profile::Segment> {
        let Some(o) = self.origin.or(self.cursor) else { return vec![] };
        if let Some((t, h, at, segs)) = &*self.cache.borrow() {
            if *t == self.text && *h == self.height && *at == o {
                return segs.clone();
            }
        }
        let font: &[u8] = match &self.font_bytes {
            Some(b) => b.as_slice(),
            None => crate::text::default_font(),
        };
        let frame = Frame { origin: o, ..current_cplane().frame };
        let segs: Vec<_> = crate::text::text_loops(font, &self.text, &frame, self.height.abs().max(1e-6)).unwrap_or_default().into_iter().flatten().collect();
        *self.cache.borrow_mut() = Some((self.text.clone(), self.height, o, segs.clone()));
        segs
    }
}

impl Tool for TextTool {
    fn name(&self) -> &'static str {
        "Text"
    }
    fn hint(&self) -> String {
        match self.origin {
            None => "Text: click where the text starts".into(),
            Some(_) => "Text: type the text and its height in the panel; click or Enter to finish".into(),
        }
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        self.cursor = p.point;
        if p.clicked {
            if self.origin.is_some() {
                return ToolOutcome::Finished;
            }
            self.origin = p.point;
        }
        ToolOutcome::Continue
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        let segs = self.segments();
        (!segs.is_empty()).then(|| ductile_brep::profile::wire_from_segments(&segs))
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut ch = ui.add(egui::TextEdit::multiline(&mut self.text).desired_rows(2).desired_width(180.0)).changed();
        ch |= drag_value(ui, "Height", &mut self.height, 0.05);
        let name = |p: &Option<std::path::PathBuf>| p.as_ref().and_then(|p| p.file_stem()).map_or("Ubuntu Light (built in)".to_string(), |s| s.to_string_lossy().into_owned());
        let mut pick: Option<Option<std::path::PathBuf>> = None;
        egui::ComboBox::from_label("Font").selected_text(name(&self.font)).width(160.0).show_ui(ui, |ui| {
            if ui.selectable_label(self.font.is_none(), name(&None)).clicked() {
                pick = Some(None);
            }
            for f in self.fonts.get_or_insert_with(crate::text::system_fonts).iter() {
                if ui.selectable_label(self.font.as_ref() == Some(f), name(&Some(f.clone()))).clicked() {
                    pick = Some(Some(f.clone()));
                }
            }
        });
        if let Some(f) = pick {
            self.font_bytes = f.as_ref().and_then(|p| std::fs::read(p).ok()).map(std::sync::Arc::new);
            self.font = if self.font_bytes.is_some() { f } else { None };
            *self.cache.borrow_mut() = None;
            ch = true;
        }
        ch
    }
    fn typed_value(&mut self, v: Option<f64>) {
        if let Some(v) = v {
            self.height = v.abs();
        }
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        self.origin = self.origin.or(self.cursor);
        let segs = self.segments();
        if segs.is_empty() {
            return false;
        }
        let id = doc.add_body("Text", ductile_brep::profile::wire_from_segments(&segs));
        doc.selection.clear();
        doc.selection.insert(Pick::Object(id));
        true
    }
}

// ---------------------------------------------------------- Curve array ----

/// Copies of the selected objects spaced evenly along a selected curve
/// (the longest selected curve is the path), turning with it.
pub struct CurveArrayTool {
    targets: Vec<(ObjectId, std::sync::Arc<Body>)>,
    /// The path as points by arc length with their tangents.
    path: Vec<(f64, DVec3, DVec3)>,
    closed: bool,
    pub count: usize,
    pub align: bool,
}

impl CurveArrayTool {
    pub fn new(doc: &Document) -> Option<Self> {
        let ids = doc.selected_objects();
        let length = |id: &ObjectId| doc.object(*id).map_or(0.0, |o| o.body.edges.iter().map(|e| e.length()).sum::<f64>());
        let path_id = ids.iter().filter(|id| doc.object(**id).is_some_and(|o| o.body.kind == ductile_brep::BodyKind::Wire)).max_by(|a, b| length(a).total_cmp(&length(b))).copied()?;
        let targets: Vec<_> = ids.iter().filter(|id| **id != path_id).filter_map(|id| doc.object(*id).map(|o| (*id, o.body.clone()))).collect();
        if targets.is_empty() {
            return None;
        }
        let chain = ductile_brep::profile::wire_chains(&doc.object(path_id)?.body).into_iter().next()?;
        let mut path: Vec<(f64, DVec3, DVec3)> = vec![];
        for seg in &chain.segments {
            let n = 64;
            for i in 0..=n {
                let f = i as f64 / n as f64;
                let t = if seg.reversed { seg.t1 + (seg.t0 - seg.t1) * f } else { seg.t0 + (seg.t1 - seg.t0) * f };
                let p = seg.curve.point(t);
                let tan = seg.tangent_at(t).normalize_or_zero();
                let s = path.last().map_or(0.0, |l| l.0 + l.1.distance(p));
                if path.last().is_some_and(|l| l.1.distance(p) < 1e-12) {
                    continue;
                }
                path.push((s, p, tan));
            }
        }
        if path.len() < 2 {
            return None;
        }
        let closed = chain.is_closed();
        Some(Self { targets, path, closed, count: 5, align: true })
    }

    /// Point and tangent at arc length `s`.
    fn at(&self, s: f64) -> (DVec3, DVec3) {
        let i = self.path.partition_point(|x| x.0 < s).clamp(1, self.path.len() - 1);
        let (a, b) = (self.path[i - 1], self.path[i]);
        let f = if b.0 > a.0 { ((s - a.0) / (b.0 - a.0)).clamp(0.0, 1.0) } else { 0.0 };
        (a.1.lerp(b.1, f), a.2.lerp(b.2, f).normalize_or(a.2))
    }

    fn copies(&self) -> Vec<(ObjectId, Body)> {
        let total = self.path.last().unwrap().0;
        let n = self.count.max(2);
        let step = if self.closed { total / n as f64 } else { total / (n - 1) as f64 };
        let (p0, t0) = self.at(0.0);
        (1..n)
            .flat_map(|k| {
                let (p, t) = self.at(step * k as f64);
                let r = if self.align { ductile_geom::DQuat::from_rotation_arc(t0, t) } else { ductile_geom::DQuat::IDENTITY };
                let m = ductile_geom::DMat4::from_translation(p) * ductile_geom::DMat4::from_quat(r) * ductile_geom::DMat4::from_translation(-p0);
                self.targets.iter().map(move |(id, b)| (*id, b.transformed(&m)))
            })
            .collect()
    }
}

impl Tool for CurveArrayTool {
    fn name(&self) -> &'static str {
        "Curve array"
    }
    fn hint(&self) -> String {
        format!("Curve array: {} copies along the curve; set the count in the panel or type it, Enter to finish", self.count)
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        if p.clicked {
            return ToolOutcome::Finished;
        }
        ToolOutcome::Continue
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        let mut out: Option<Body> = None;
        for (_, b) in self.copies() {
            match &mut out {
                None => out = Some(b),
                Some(o) => o.append(&b),
            }
        }
        out
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut n = self.count as f64;
        let mut ch = false;
        if drag_value(ui, "Count", &mut n, 0.1) {
            self.count = (n.round() as usize).clamp(2, 500);
            ch = true;
        }
        ch |= ui.checkbox(&mut self.align, "Turn with the curve").changed();
        ch
    }
    fn typed_value(&mut self, v: Option<f64>) {
        if let Some(v) = v {
            self.count = (v.round() as usize).clamp(2, 500);
        }
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let copies = self.copies();
        if copies.is_empty() {
            return false;
        }
        doc.selection.clear();
        for (id, b) in copies {
            let (name, group, color) = doc.object(id).map(|o| (o.name.clone(), o.group, o.color)).unwrap_or(("Copy".into(), None, crate::document::DEFAULT_COLOR));
            let new = doc.add_body(&name, b);
            if let Some(o) = doc.objects.iter_mut().find(|o| o.id == new) {
                (o.group, o.color) = (group, color);
            }
            doc.selection.insert(Pick::Object(new));
        }
        true
    }
}

// ------------------------------------------------------- Tangent circle ----

/// One curve a tangent circle touches: lines count as infinite, other
/// curves over their own range.
#[derive(Clone)]
struct Tangency {
    curve: ductile_geom::Curve3,
    t0: f64,
    t1: f64,
}

impl Tangency {
    fn from_edge(curve: &ductile_geom::Curve3, t0: f64, t1: f64) -> Self {
        match curve {
            ductile_geom::Curve3::Line { .. } => Self { curve: curve.clone(), t0: -1e7, t1: 1e7 },
            _ => Self { curve: curve.clone(), t0, t1 },
        }
    }

    /// Signed distance from `c` (on the plane with normal `z`) and the
    /// direction it grows in.
    fn signed(&self, c: DVec3, z: DVec3) -> (f64, DVec3) {
        let t = self.curve.closest_param_in(c, self.t0, self.t1);
        let foot = self.curve.point(t);
        let d = c - foot;
        let d = d - z * d.dot(z);
        let n = z.cross(self.curve.tangent(t)).try_normalize().unwrap_or(DVec3::X);
        let n = n - z * n.dot(z);
        // Past the curve's ends the distance is to the end point.
        let s = d.length() * if d.dot(n) < 0.0 { -1.0 } else { 1.0 };
        let g = if d.length() > 1e-12 { d.normalize() * s.signum() } else { n };
        (s, g)
    }
}

/// The circle tangent to two (or three) curves on the construction plane,
/// on the sides of them the cursor is on: with a radius, the one whose
/// center is nearest `near`; without, the one whose center is nearest
/// `near`'s projection onto the curves' equidistance line; with three
/// curves, the one touching them all.
fn tangent_circle(ts: &[Tangency], near: DVec3, radius: Option<f64>, f: &Frame) -> Option<(DVec3, f64)> {
    let z = f.z;
    let near = near - z * (near - f.origin).dot(z);
    let sig: Vec<f64> = ts.iter().map(|t| if t.signed(near, z).0 < 0.0 { -1.0 } else { 1.0 }).collect();
    let mut c = near;
    let mut r = radius.unwrap_or_else(|| ts.iter().zip(&sig).map(|(t, s)| s * t.signed(near, z).0).sum::<f64>() / ts.len() as f64);
    let (x, y) = (f.x, f.y);
    for _ in 0..60 {
        let v: Vec<(f64, DVec3)> = ts.iter().zip(&sig).map(|(t, s)| { let (d, g) = t.signed(c, z); (s * d, g * *s) }).collect();
        let step = if ts.len() >= 3 && radius.is_none() {
            // Three equations d_i(c) = r in (cx, cy, r).
            let m = ductile_geom::DMat3::from_cols(
                DVec3::new(v[0].1.dot(x), v[1].1.dot(x), v[2].1.dot(x)),
                DVec3::new(v[0].1.dot(y), v[1].1.dot(y), v[2].1.dot(y)),
                DVec3::splat(-1.0),
            );
            if m.determinant().abs() < 1e-12 {
                return None;
            }
            let s = m.inverse() * DVec3::new(v[0].0 - r, v[1].0 - r, v[2].0 - r);
            r -= s.z;
            x * s.x + y * s.y
        } else if let Some(r) = radius {
            // Two equations d_i(c) = r.
            let (a, b, cc, d) = (v[0].1.dot(x), v[0].1.dot(y), v[1].1.dot(x), v[1].1.dot(y));
            let det = a * d - b * cc;
            if det.abs() < 1e-12 {
                return None;
            }
            let (e0, e1) = (v[0].0 - r, v[1].0 - r);
            x * ((d * e0 - b * e1) / det) + y * ((a * e1 - cc * e0) / det)
        } else {
            // Onto d_0 = d_1 along its gradient.
            let g = v[0].1 - v[1].1;
            if g.length_squared() < 1e-18 {
                return None;
            }
            g * ((v[0].0 - v[1].0) / g.length_squared())
        };
        c -= step;
        if step.length() < 1e-12 * (1.0 + c.length()) {
            break;
        }
    }
    let ds: Vec<f64> = ts.iter().zip(&sig).map(|(t, s)| s * t.signed(c, z).0).collect();
    let r = radius.unwrap_or(if ts.len() >= 3 { r } else { ds[0] });
    let ok = r > 1e-9 && ds.iter().all(|d| (d - r).abs() < 1e-6 * (1.0 + r));
    ok.then_some((c, r))
}

/// Tangent circle: click two curves, then move to size the circle between
/// them (or click a third curve for the circle touching all three); Tab
/// types the radius.
pub struct TangentCircleTool {
    edges: Vec<(ductile_geom::Curve3, f64, f64, Vec<DVec3>)>,
    picked: Vec<usize>,
    hover: Option<usize>,
    cursor: Option<DVec3>,
    pub radius: f64,
    pub fixed: bool,
}

impl TangentCircleTool {
    pub fn new(doc: &Document) -> Self {
        let edges = doc
            .objects
            .iter()
            .filter(|o| o.visible && !doc.is_locked(o.id))
            .flat_map(|o| o.body.edges.iter().map(|e| (e.curve.clone(), e.t0, e.t1, e.curve.tessellate(e.t0, e.t1, 1e-3).into_iter().map(|q| q.1).collect::<Vec<_>>())))
            .collect();
        Self { edges, picked: vec![], hover: None, cursor: None, radius: 1.0, fixed: false }
    }

    fn pick(&self, p: &PointerInfo) -> Option<usize> {
        let mut best = (8.0 * p.world_per_px, None);
        for (i, e) in self.edges.iter().enumerate() {
            for w in e.3.windows(2) {
                let d = ray_segment_distance(&p.ray, w[0], w[1]);
                if d < best.0 {
                    best = (d, Some(i));
                }
            }
        }
        best.1
    }

    fn circle(&self) -> Option<(DVec3, f64)> {
        if self.picked.len() < 2 {
            return None;
        }
        let mut ids = self.picked.clone();
        if let Some(h) = self.hover.filter(|h| !ids.contains(h)) {
            if !self.fixed {
                ids.push(h);
            }
        }
        let ts: Vec<Tangency> = ids.iter().map(|i| Tangency::from_edge(&self.edges[*i].0, self.edges[*i].1, self.edges[*i].2)).collect();
        let f = current_cplane().frame;
        let near = self.cursor?;
        tangent_circle(&ts, near, self.fixed.then_some(self.radius), &f).or_else(|| (ts.len() > 2).then(|| tangent_circle(&ts[..2], near, self.fixed.then_some(self.radius), &f)).flatten())
    }

    fn segment(&self) -> Option<ductile_brep::profile::Segment> {
        let (c, r) = self.circle()?;
        let frame = Frame { origin: c, ..current_cplane().frame };
        Some(ductile_brep::profile::Segment::new(ductile_geom::Curve3::Circle { frame, radius: r }, 0.0, std::f64::consts::TAU))
    }
}

impl Tool for TangentCircleTool {
    fn name(&self) -> &'static str {
        "Tangent circle"
    }
    fn hint(&self) -> String {
        match self.picked.len() {
            0 => "Tangent circle: click the first curve".into(),
            1 => "Tangent circle: click the second curve".into(),
            _ => "Tangent circle: move to size it, or hover a third curve; click to place; Tab types the radius".into(),
        }
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        self.hover = self.pick(p);
        self.cursor = p.plane_point.or(p.point);
        if self.picked.len() >= 2 && !self.fixed {
            if let Some((_, r)) = self.circle() {
                self.radius = r;
            }
        }
        if p.clicked {
            if self.picked.len() >= 2 {
                return if self.circle().is_some() { ToolOutcome::Finished } else { ToolOutcome::Continue };
            }
            if let Some(h) = self.hover.filter(|h| !self.picked.contains(h)) {
                self.picked.push(h);
            }
        }
        ToolOutcome::Continue
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        self.segment().map(|s| ductile_brep::profile::wire_from_segments(&[s]))
    }
    fn guides(&self, out: &mut Vec<LineInstance>) {
        for (i, e) in self.edges.iter().enumerate() {
            let color = if self.picked.contains(&i) {
                [0.3, 0.7, 1.0, 1.0]
            } else if self.hover == Some(i) {
                [1.0, 0.8, 0.3, 1.0]
            } else {
                continue;
            };
            for w in e.3.windows(2) {
                out.push(LineInstance::new(w[0], w[1], color, 3.0));
            }
        }
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut ch = ui.checkbox(&mut self.fixed, "Fixed radius").changed();
        if drag_value(ui, "Radius", &mut self.radius, 0.05) {
            self.fixed = true;
            ch = true;
        }
        ch
    }
    fn typed_value(&mut self, v: Option<f64>) {
        match v {
            Some(v) if v.abs() > 1e-9 => {
                self.radius = v.abs();
                self.fixed = true;
            }
            _ => self.fixed = false,
        }
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let Some(s) = self.segment() else { return false };
        let id = doc.add_body("Circle", ductile_brep::profile::wire_from_segments(&[s]));
        doc.selection.clear();
        doc.selection.insert(Pick::Object(id));
        true
    }
}

// ---------------------------------------------------------------- Place ----

/// Place: copies of the selection stood on the face under the cursor (or the
/// construction plane), their bottom centre on the point and their up axis
/// along the surface normal; each click places one.
pub struct PlaceTool {
    targets: Vec<(ObjectId, std::sync::Arc<Body>)>,
    /// The selection's bottom centre, which lands on the surface.
    base: DVec3,
    hover: Option<(DVec3, DVec3)>,
    placed: Vec<DMat4>,
    /// Turn about the normal, in degrees.
    pub angle: f64,
    pub scale: f64,
}

impl PlaceTool {
    pub fn new(doc: &Document) -> Option<Self> {
        let targets: Vec<_> = doc.selected_objects().iter().filter_map(|id| doc.object(*id).map(|o| (*id, o.body.clone()))).collect();
        let bb = targets.iter().map(|(_, b)| b.bbox()).reduce(|a, b| a.union(&b))?;
        let c = bb.center();
        // The lowest point of the shaded surface (the box has some slack).
        let low = doc.selected_objects().iter().filter_map(|id| doc.object(*id)).flat_map(|o| o.mesh.faces.iter().flat_map(|f| f.positions.iter().map(|p| p.z)).chain(o.body.vertices.iter().map(|v| v.point.z))).fold(f64::INFINITY, f64::min);
        let low = if low.is_finite() { low } else { bb.min.z };
        Some(Self { targets, base: DVec3::new(c.x, c.y, low), hover: None, placed: vec![], angle: 0.0, scale: 1.0 })
    }

    fn matrix(&self, p: DVec3, n: DVec3) -> DMat4 {
        let r = DQuat::from_rotation_arc(DVec3::Z, n.normalize()) * DQuat::from_rotation_z(self.angle.to_radians());
        DMat4::from_translation(p) * DMat4::from_quat(r) * DMat4::from_scale(DVec3::splat(self.scale.max(1e-6))) * DMat4::from_translation(-self.base)
    }

    fn matrices(&self) -> Vec<DMat4> {
        self.placed.iter().copied().chain(self.hover.map(|(p, n)| self.matrix(p, n))).collect()
    }
}

impl Tool for PlaceTool {
    fn name(&self) -> &'static str {
        "Place"
    }
    fn hint(&self) -> String {
        format!("Place: click a face to stand a copy on it ({} placed); turn and scale in the panel; Enter to finish", self.placed.len())
    }
    fn pointer(&mut self, doc: &Document, cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        let hit = crate::picking::pick_face(doc, &p.ray).filter(|h| doc.object(h.object).is_some_and(|o| o.body.mesh.is_none()));
        self.hover = match hit {
            Some(h) => {
                let b = &doc.object(h.object).unwrap().body;
                let f = b.face(h.face);
                let uv = f.surface.project(h.point, None);
                let n = b.face_normal(h.face, uv.x, uv.y);
                // A bad projection falls back to the facet's normal.
                Some((h.point, if n.dot(h.normal) > 0.5 { n } else { h.normal }))
            }
            None => p.point.map(|q| (q, cplane.frame.z)),
        };
        if p.clicked {
            if let Some((q, n)) = self.hover {
                self.placed.push(self.matrix(q, n));
            }
        }
        ToolOutcome::Continue
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        let mut out: Option<Body> = None;
        for m in self.matrices() {
            for (_, b) in &self.targets {
                let c = b.transformed(&m);
                match &mut out {
                    None => out = Some(c),
                    Some(o) => o.append(&c),
                }
            }
        }
        out
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut ch = drag_value(ui, "Angle", &mut self.angle, 1.0);
        ch |= drag_value(ui, "Scale", &mut self.scale, 0.01);
        ui.label(format!("{} placed", self.placed.len()));
        ch
    }
    fn typed_value(&mut self, v: Option<f64>) {
        if let Some(v) = v {
            self.angle = v;
        }
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        if self.placed.is_empty() {
            return false;
        }
        doc.selection.clear();
        for m in self.placed.clone() {
            for (id, b) in &self.targets {
                let (name, group, color, metal, rough) = doc.object(*id).map(|o| (o.name.clone(), o.group, o.color, o.metalness, o.roughness)).unwrap_or(("Copy".into(), None, crate::document::DEFAULT_COLOR, 0.0, 0.5));
                let new = doc.add_body(&name, b.transformed(&m));
                if let Some(o) = doc.objects.iter_mut().find(|o| o.id == new) {
                    (o.group, o.color, o.metalness, o.roughness) = (group, color, metal, rough);
                }
                doc.selection.insert(Pick::Object(new));
            }
        }
        true
    }
}

// ------------------------------------------------------------ Dimension ----

/// The lines of a dimension: extension lines from the two points, the
/// dimension line between them with arrowheads, in `color`.
pub fn dimension_lines(d: &crate::document::Dimension, color: [f32; 4], out: &mut Vec<LineInstance>) {
    let (a, b, o) = (d.a, d.b, d.offset);
    let over = o.normalize_or_zero() * (b - a).length().max(1e-9) * 0.03;
    out.push(LineInstance::new(a, a + o + over, color, 1.0));
    out.push(LineInstance::new(b, b + o + over, color, 1.0));
    let (p, q) = (a + o, b + o);
    out.push(LineInstance::new(p, q, color, 1.5));
    let dir = (q - p).normalize_or_zero();
    let side = o.normalize_or_zero();
    let s = p.distance(q) * 0.04;
    for (end, along) in [(p, dir), (q, -dir)] {
        out.push(LineInstance::new(end, end + along * s + side * s * 0.35, color, 1.5));
        out.push(LineInstance::new(end, end + along * s - side * s * 0.35, color, 1.5));
    }
}

/// The dimension's text and where it goes.
pub fn dimension_label(d: &crate::document::Dimension) -> (DVec3, String) {
    let s = format!("{:.4}", d.a.distance(d.b));
    let s = s.trim_end_matches('0').trim_end_matches('.').to_string();
    ((d.a + d.b) * 0.5 + d.offset, s)
}

/// Dimension: click two points, then move the line off them and click to
/// place it; it stays in the scene until deleted.
pub struct DimensionTool {
    a: Option<DVec3>,
    b: Option<DVec3>,
    cursor: Option<DVec3>,
}

impl DimensionTool {
    pub fn new() -> Self {
        Self { a: None, b: None, cursor: None }
    }

    fn dimension(&self) -> Option<crate::document::Dimension> {
        let (a, c) = (self.a?, self.cursor?);
        let b = self.b.unwrap_or(c);
        let dir = (b - a).try_normalize()?;
        let offset = match self.b {
            Some(_) => {
                let v = c - a;
                v - dir * v.dot(dir)
            }
            None => DVec3::ZERO,
        };
        Some(crate::document::Dimension { a, b, offset })
    }
}

impl Tool for DimensionTool {
    fn name(&self) -> &'static str {
        "Dimension"
    }
    fn hint(&self) -> String {
        let d = self.dimension().map(|d| dimension_label(&d).1).unwrap_or_default();
        match (self.a, self.b) {
            (None, _) => "Dimension: click the first point".into(),
            (Some(_), None) => format!("Dimension: click the second point ({d})"),
            _ => format!("Dimension {d}: move the line off the points and click to place it"),
        }
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        self.cursor = p.point.or(p.plane_point);
        if p.clicked {
            match (self.a, self.b) {
                (None, _) => self.a = self.cursor,
                (Some(a), None) => self.b = self.cursor.filter(|c| c.distance(a) > 1e-9),
                _ => return ToolOutcome::Finished,
            }
        }
        ToolOutcome::Continue
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        None
    }
    fn guides(&self, out: &mut Vec<LineInstance>) {
        if let Some(d) = self.dimension() {
            dimension_lines(&d, [0.45, 0.95, 0.6, 1.0], out);
        }
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        if let Some(d) = self.dimension() {
            ui.label(format!("Distance {}", dimension_label(&d).1));
        }
        false
    }
    fn anchor(&self) -> Option<DVec3> {
        self.b.or(self.a)
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        if self.b.is_none() {
            return false;
        }
        match self.dimension() {
            Some(d) => {
                doc.dimensions.push(d);
                true
            }
            None => false,
        }
    }
}

// --------------------------------------------------------- Align curve ----

/// Align curve: click the end of a curve to change, then the curve to
/// align it to; the end moves onto the other curve (its nearest end, or
/// the nearest point along it) and leaves it with G0 to G3 continuity.
pub struct AlignCurveTool {
    /// Every visible edge: object, edge, curve, range, samples.
    edges: Vec<(ObjectId, EdgeId, ductile_geom::Curve3, f64, f64, Vec<DVec3>)>,
    /// The edge to change, and whether its start moves.
    moving: Option<(usize, bool)>,
    hover: Option<usize>,
    cursor_ray: Option<Ray>,
    pub order: usize,
}

impl AlignCurveTool {
    pub fn new(doc: &Document) -> Self {
        let edges = doc
            .objects
            .iter()
            .filter(|o| o.visible && !doc.is_locked(o.id))
            .flat_map(|o| o.body.edges.iter().enumerate().map(move |(i, e)| (o.id, EdgeId(i as u32), e.curve.clone(), e.t0, e.t1, e.curve.tessellate(e.t0, e.t1, 1e-3).into_iter().map(|q| q.1).collect::<Vec<_>>())))
            .filter(|e| e.5.len() > 1)
            .collect();
        Self { edges, moving: None, hover: None, cursor_ray: None, order: 2 }
    }

    fn pick(&self, p: &PointerInfo, wires_only: Option<&Document>) -> Option<usize> {
        let mut best = (8.0 * p.world_per_px, None);
        for (i, e) in self.edges.iter().enumerate() {
            if wires_only.is_some_and(|d| d.object(e.0).is_none_or(|o| o.body.kind != ductile_brep::BodyKind::Wire)) {
                continue;
            }
            for w in e.5.windows(2) {
                let d = ray_segment_distance(&p.ray, w[0], w[1]);
                if d < best.0 {
                    best = (d, Some(i));
                }
            }
        }
        best.1
    }

    /// The changed curve, aligned to edge `target`.
    fn aligned(&self, target: usize) -> Option<ductile_geom::NurbsCurve> {
        let (mi, at_start) = self.moving?;
        if mi == target {
            return None;
        }
        let m = &self.edges[mi];
        let a = m.2.to_nurbs(m.3, m.4);
        let (a0, a1) = a.domain();
        let end = a.point(if at_start { a0 } else { a1 });
        let t = &self.edges[target];
        let b = t.2.to_nurbs(t.3, t.4);
        let (b0, b1) = b.domain();
        let tb = b.closest_param(end);
        let span = b.length().max(1e-12);
        let pt = b.point(tb);
        // Near an end of the other curve: meet that end.
        let (tb, rev) = if b.point(b1).distance(pt) < 0.15 * span || b.point(b1).distance(end) < 0.15 * span {
            (b1, false)
        } else if b.point(b0).distance(pt) < 0.15 * span || b.point(b0).distance(end) < 0.15 * span {
            (b0, true)
        } else {
            // Along the curve: go the way the changed curve already leaves.
            let d = a.derivs(if at_start { a0 } else { a1 }, 1)[1] * if at_start { 1.0 } else { -1.0 };
            (tb, d.dot(b.derivs(tb, 1)[1]) < 0.0)
        };
        let d = b.derivs(tb, 3);
        let s = if rev { -1.0 } else { 1.0 };
        let target = [d[0], d[1] * s, d[2], d[3] * s];
        Some(ductile_brep::curves::align_curve_end(&a, at_start, target, self.order))
    }
}

impl Tool for AlignCurveTool {
    fn name(&self) -> &'static str {
        "Align curve"
    }
    fn hint(&self) -> String {
        match self.moving {
            None => "Align curve: click near the end of the curve to change".into(),
            Some(_) => format!("Align curve (G{}): click the curve to align to; 0–3 set the continuity", self.order),
        }
    }
    fn pointer(&mut self, doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        self.cursor_ray = Some(p.ray);
        self.hover = self.pick(p, self.moving.is_none().then_some(doc));
        if p.clicked {
            if let Some(h) = self.hover {
                match self.moving {
                    None => {
                        let e = &self.edges[h];
                        let (s, t) = (e.2.point(e.3), e.2.point(e.4));
                        let near = |q: DVec3| {
                            let v = q - p.ray.origin;
                            (v - p.ray.dir * v.dot(p.ray.dir)).length()
                        };
                        self.moving = Some((h, near(s) < near(t)));
                    }
                    Some(_) if self.aligned(h).is_some() => return ToolOutcome::Finished,
                    _ => {}
                }
            }
        }
        ToolOutcome::Continue
    }
    fn preview(&self, _doc: &Document) -> Option<Body> {
        let c = self.aligned(self.hover?)?;
        let (t0, t1) = c.domain();
        Some(ductile_brep::profile::wire_from_segments(&[ductile_brep::profile::Segment::new(ductile_geom::Curve3::Nurbs(c), t0, t1)]))
    }
    fn guides(&self, out: &mut Vec<LineInstance>) {
        for (i, color) in [(self.moving.map(|m| m.0), [0.3, 0.7, 1.0, 1.0]), (self.hover, [1.0, 0.8, 0.3, 1.0])] {
            if let Some(i) = i {
                for w in self.edges[i].5.windows(2) {
                    out.push(LineInstance::new(w[0], w[1], color, 3.0));
                }
            }
        }
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut ch = false;
        ui.horizontal(|ui| {
            for k in 0..=3 {
                ch |= ui.selectable_value(&mut self.order, k, format!("G{k}")).changed();
            }
        });
        ch
    }
    fn command_key(&mut self, key: egui::Key, m: egui::Modifiers) -> bool {
        let k = match key {
            egui::Key::Num0 => 0,
            egui::Key::Num1 => 1,
            egui::Key::Num2 => 2,
            egui::Key::Num3 => 3,
            _ => return false,
        };
        if m.any() {
            return false;
        }
        self.order = k;
        true
    }
    fn hidden_objects(&self) -> Vec<ObjectId> {
        vec![]
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let (Some((mi, _)), Some(h)) = (self.moving, self.hover) else { return false };
        let Some(c) = self.aligned(h) else { return false };
        let (oid, eid) = (self.edges[mi].0, self.edges[mi].1);
        let Some(o) = doc.object(oid) else { return false };
        let (t0, t1) = c.domain();
        let new = ductile_brep::profile::Segment::new(ductile_geom::Curve3::Nurbs(c), t0, t1);
        let segs: Vec<_> = o.body.edges.iter().enumerate().map(|(i, e)| if i == eid.idx() { new.clone() } else { ductile_brep::profile::Segment::new(e.curve.clone(), e.t0, e.t1) }).collect();
        doc.replace_body(oid, ductile_brep::profile::wire_from_segments(&segs));
        true
    }
}

// ------------------------------------------------- Rebuild face / raise ----

/// Each object with its selected faces' surfaces changed by `f`.
fn edit_faces(doc: &Document, faces: &[(ObjectId, FaceId)], f: &dyn Fn(&Body, FaceId) -> Option<Body>) -> Vec<(ObjectId, Body)> {
    let mut out: Vec<(ObjectId, Body)> = Vec::new();
    for (o, face) in faces {
        let i = match out.iter().position(|x| x.0 == *o) {
            Some(i) => i,
            None => {
                let Some(ob) = doc.object(*o) else { continue };
                out.push((*o, (*ob.body).clone()));
                out.len() - 1
            }
        };
        if let Some(b) = f(&out[i].1, *face) {
            out[i].1 = b;
        }
    }
    out
}

/// Raise surface degree: the selected faces' surfaces one degree higher in
/// u and v, the shape unchanged, for finer control point editing.
pub fn raise_selected_face_degree(doc: &mut Document) -> Result<usize, String> {
    let faces = doc.selected_faces();
    if faces.is_empty() {
        return Err("Select faces to raise the degree of (face mode: 3)".into());
    }
    let bodies = edit_faces(doc, &faces, &ductile_brep::ops::untrim::raise_face_degree);
    doc.checkpoint("Raise surface degree");
    for (o, b) in bodies {
        doc.replace_body(o, b);
    }
    Ok(faces.len())
}

/// Rebuild face: the selected faces' surfaces refitted as smooth NURBS
/// surfaces with the chosen number of control points.
pub struct RebuildFaceTool {
    faces: Vec<(ObjectId, FaceId)>,
    pub nu: usize,
    pub nv: usize,
    pub degree: usize,
    cache: std::cell::RefCell<Option<((usize, usize, usize), Vec<(ObjectId, Body)>)>>,
}

impl RebuildFaceTool {
    pub fn new(doc: &Document) -> Option<Self> {
        let faces = doc.selected_faces();
        (!faces.is_empty()).then(|| Self { faces, nu: 6, nv: 6, degree: 3, cache: Default::default() })
    }

    fn bodies(&self, doc: &Document) -> Vec<(ObjectId, Body)> {
        let key = (self.nu, self.nv, self.degree);
        if let Some((k, b)) = &*self.cache.borrow() {
            if *k == key {
                return b.clone();
            }
        }
        let (nu, nv, d) = key;
        let b = edit_faces(doc, &self.faces, &|b, f| ductile_brep::ops::untrim::rebuild_face(b, f, nu, nv, d));
        *self.cache.borrow_mut() = Some((key, b.clone()));
        b
    }
}

impl Tool for RebuildFaceTool {
    fn name(&self) -> &'static str {
        "Rebuild face"
    }
    fn hint(&self) -> String {
        format!("Rebuild face: {} × {} control points, degree {}; set them in the panel, Enter to finish", self.nu, self.nv, self.degree)
    }
    fn pointer(&mut self, _doc: &Document, _cplane: &CPlane, p: &PointerInfo) -> ToolOutcome {
        if p.clicked {
            return ToolOutcome::Finished;
        }
        ToolOutcome::Continue
    }
    fn preview(&self, doc: &Document) -> Option<Body> {
        let mut out: Option<Body> = None;
        for (_, b) in self.bodies(doc) {
            match &mut out {
                None => out = Some(b),
                Some(o) => o.append(&b),
            }
        }
        out
    }
    fn hidden_objects(&self) -> Vec<ObjectId> {
        let mut ids: Vec<ObjectId> = self.faces.iter().map(|f| f.0).collect();
        ids.dedup();
        ids
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut ch = false;
        for (label, v, lo) in [("U points", &mut self.nu, 2usize), ("V points", &mut self.nv, 2), ("Degree", &mut self.degree, 1)] {
            let mut x = *v as f64;
            if drag_value(ui, label, &mut x, 0.1) {
                *v = (x.round() as usize).clamp(lo, 64);
                ch = true;
            }
        }
        ch
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let bodies = self.bodies(doc);
        for (o, b) in bodies {
            doc.replace_body(o, b);
        }
        true
    }
}
