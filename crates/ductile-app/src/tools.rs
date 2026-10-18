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
        Self { kind, targets, controls, surface_controls, faces, pivot, start: None, current: None, axis: None, typed: None, value: 0.0 }
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
            return ductile_brep::ops::sheet::move_surface_controls(body, &on_surface, m).unwrap_or_else(|| body.clone());
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
                doc.replace_body(id, b);
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
            made.push(doc.add_body(&o.name, (*o.body).clone()));
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
}

impl CurveTool {
    pub fn new(kind: CurveKind) -> Self {
        Self { kind, points: vec![], cursor: None, size: DVec3::ZERO, angle: 0.0, sides: 6, closed: false, typed: None }
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
        }
    }
    fn hint(&self) -> String {
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
        let Some(q) = pt else { return ToolOutcome::Continue };
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
            CurveKind::Polyline | CurveKind::Spline | CurveKind::Arc | CurveKind::ThreePointCircle => {
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
                CurveKind::Polyline | CurveKind::Spline | CurveKind::Arc | CurveKind::ThreePointCircle => {}
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
        let id = doc.add_body(self.name(), ductile_brep::profile::wire_from_segments(&segs));
        doc.selection.clear();
        doc.selection.insert(Pick::Object(id));
        true
    }
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
    typed: Option<f64>,
    bg: Background<(f64, BlendShape, bool, f64), (Option<Body>, Option<String>)>,
}

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
        Some(Self { target, body: o.body.clone(), edges, base, dir, start_t: None, radius: 0.0, shape, variable: false, end_radius: 1.0, typed: None, bg: Background::default() })
    }

    fn value(&self) -> f64 {
        self.typed.unwrap_or(self.radius)
    }

    fn key(&self) -> (f64, BlendShape, bool, f64) {
        (self.value(), self.shape, self.variable, self.end_radius)
    }

    /// The blend computation for the current values, owning its inputs.
    fn job(&self) -> Box<dyn FnOnce() -> (Option<Body>, Option<String>) + Send> {
        use ductile_brep::ops::fillet::{blend_edges, blend_edges_variable, Blend};
        let (body, edges, (r, shape, variable, end_radius)) = (self.body.clone(), self.edges.clone(), self.key());
        Box::new(move || {
            if r <= 1e-9 {
                (None, None)
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
}
