// SPDX-License-Identifier: GPL-3.0-only
//! Interactive modeling commands.
//!
//! Each command is a small state machine fed with viewport events. Commands
//! show a live preview, accept typed values in the command panel, and commit
//! a single undoable change to the document when finished.

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
fn current_cplane() -> CPlane {
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
}

pub struct RoundTool {
    kind: RoundKind,
    center: Option<DVec3>,
    radius: f64,
    radius2: f64,
    height: f64,
    stage: u8,
}

impl RoundTool {
    pub fn new(kind: RoundKind) -> Self {
        Self { kind, center: None, radius: 0.0, radius2: 0.0, height: 0.0, stage: 0 }
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
        let mut bb = ductile_geom::BBox3::EMPTY;
        for (o, f, i) in &surface_controls {
            if let Some(ductile_geom::Surface::Nurbs(s)) = doc.object(*o).map(|ob| ob.body.face(*f).surface.clone()) {
                if let Some(h) = s.ctrl.get(*i as usize) {
                    bb = bb.union(&ductile_geom::BBox3::from_points([h.truncate() / h.w]));
                }
            }
        }
        if controls.is_empty() && surface_controls.is_empty() {
            for id in &targets {
                if let Some(o) = doc.object(*id) {
                    bb = bb.union(&o.body.bbox());
                }
            }
        } else {
            for (o, e, i) in &controls {
                if let Some(ductile_geom::Curve3::Nurbs(n)) = doc.object(*o).map(|ob| ob.body.edge(*e).curve.clone()) {
                    if let Some(p) = n.control_points().get(*i as usize) {
                        bb = bb.union(&ductile_geom::BBox3::from_points([*p]));
                    }
                }
            }
        }
        let pivot = if bb.is_empty() { DVec3::ZERO } else { bb.center() };
        Self { kind, targets, controls, surface_controls, pivot, start: None, current: None, axis: None, typed: None, value: 0.0 }
    }

    /// The object's body after applying `m` to it, or to its selected
    /// control points.
    fn moved(&self, id: ObjectId, body: &Body, m: &DMat4) -> Body {
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
    cache: std::cell::RefCell<Option<(f64, Option<Body>, Option<String>)>>,
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
    let (solids, chains, dir) = cut_inputs(doc, "Cut")?;
    let mut pieces = Vec::new();
    for id in &solids {
        let b = &doc.object(*id).ok_or("Cut: missing object")?.body;
        let parts = ductile_brep::ops::cut::cut_with_curves(b, &chains, dir).map_err(|e| format!("Cut failed: {e}"))?;
        pieces.push(parts);
    }
    Ok((solids, pieces))
}

/// Imprints the selected curves onto the selected solids, splitting the
/// faces they cross; returns the solids and their imprinted bodies.
pub fn imprint_selected(doc: &Document) -> Result<(Vec<ObjectId>, Vec<Body>), String> {
    let (solids, chains, dir) = cut_inputs(doc, "Imprint")?;
    let mut out = Vec::new();
    for id in &solids {
        let b = &doc.object(*id).ok_or("Imprint: missing object")?.body;
        out.push(ductile_brep::ops::cut::imprint_curves(b, &chains, dir).map_err(|e| format!("Imprint failed: {e}"))?);
    }
    Ok((solids, out))
}

/// The selected solids, the selected curve chains, and the direction to
/// sweep them through the solids: square to the curves' plane, or else
/// along the construction plane's normal.
fn cut_inputs(doc: &Document, what: &str) -> Result<(Vec<ObjectId>, Vec<ductile_brep::profile::Loop3>, DVec3), String> {
    let solids: Vec<ObjectId> = doc.selected_objects().into_iter().filter(|id| doc.object(*id).is_some_and(|o| o.body.kind == ductile_brep::BodyKind::Solid)).collect();
    let chains = selected_chains(doc);
    if solids.is_empty() || chains.is_empty() {
        return Err(format!("{what}: select solids and the curves to {} them with", what.to_lowercase()));
    }
    let dir = ductile_brep::profile::loops_plane(&chains).filter(|_| chains.iter().any(|c| c.segments.len() > 1 || !c.segments[0].curve.is_linear())).map(|f| f.z).unwrap_or(current_cplane().frame.z);
    Ok((solids, chains, dir))
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
                    return Some(Self { target: None, body: None, regions, open, normal, base, start_t: None, distance: 0.0, typed: None, cache: std::cell::RefCell::new(None) });
                }
                (None, None, regions)
            }
        };
        let poly = regions[0].outer.polygon(1e-3);
        let base = poly.iter().copied().sum::<DVec3>() / poly.len().max(1) as f64;
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
            cache: std::cell::RefCell::new(None),
        })
    }

    fn value(&self) -> f64 {
        self.typed.unwrap_or(self.distance)
    }

    fn result(&self) -> (Option<Body>, Option<String>) {
        use ductile_brep::ops::boolean::{boolean, BoolOp};
        let d = self.value();
        if let Some((cd, b, e)) = &*self.cache.borrow() {
            if *cd == d {
                return (b.clone(), e.clone());
            }
        }
        let out = if d.abs() < 1e-9 {
            (None, None)
        } else if self.regions.is_empty() {
            match ductile_brep::ops::sheet::extrude_sheet(&self.open, self.normal * d) {
                Some(b) => (Some(b), None),
                None => (None, Some("could not extrude the curves".into())),
            }
        } else {
            let mut acc: Result<Option<Body>, String> = Ok(None);
            for r in &self.regions {
                let Ok(prev) = acc else { break };
                acc = match ductile_brep::ops::extrude::extrude_region(r, self.normal * d) {
                    None => Err("could not build the extrusion".into()),
                    Some(prism) => match prev {
                        None => Ok(Some(prism)),
                        Some(p) => boolean(&p, &prism, BoolOp::Union).map(Some).map_err(|e| e.to_string()),
                    },
                };
            }
            match (acc, &self.body) {
                (Err(e), _) => (None, Some(e)),
                (Ok(None), _) => (None, None),
                (Ok(Some(prism)), None) => (Some(prism), None),
                (Ok(Some(prism)), Some(body)) => {
                    let op = if d > 0.0 { BoolOp::Union } else { BoolOp::Difference };
                    match boolean(body, &prism, op) {
                        Ok(b) => (Some(b), None),
                        Err(e) => (None, Some(e.to_string())),
                    }
                }
            }
        };
        *self.cache.borrow_mut() = Some((d, out.0.clone(), out.1.clone()));
        out
    }
}

impl Tool for ExtrudeTool {
    fn name(&self) -> &'static str {
        "Extrude"
    }
    fn hint(&self) -> String {
        match self.cache.borrow().as_ref().and_then(|c| c.2.clone()) {
            Some(e) => format!("Extrude failed: {e}"),
            None => format!("Extrude: move along the normal or type a distance ({:.4}), click or Enter to finish", self.value()),
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
        self.result().0
    }
    fn guides(&self, out: &mut Vec<LineInstance>) {
        let tip = self.base + self.normal * self.value();
        out.push(LineInstance::new(self.base, tip, [1.0, 0.85, 0.3, 1.0], 2.0));
        out.push(LineInstance::point(tip, [1.0, 0.85, 0.3, 1.0], 7.0));
    }
    fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        let mut d = self.value();
        let ch = drag_value(ui, "Distance", &mut d, 0.05);
        if ch {
            self.typed = Some(d);
        }
        ch
    }
    fn hidden_objects(&self) -> Vec<ObjectId> {
        match self.target {
            Some(t) if self.result().0.is_some() => vec![t],
            _ => vec![],
        }
    }
    fn typed_value(&mut self, v: Option<f64>) {
        self.typed = v;
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let Some(b) = self.result().0 else { return false };
        doc.selection.clear();
        match self.target {
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
}

impl CurveTool {
    pub fn new(kind: CurveKind) -> Self {
        Self { kind, points: vec![], cursor: None, size: DVec3::ZERO, angle: 0.0, sides: 6, closed: false }
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
        }
    }
}

impl Tool for CurveTool {
    fn name(&self) -> &'static str {
        match self.kind {
            CurveKind::Polyline => "Curve",
            CurveKind::Spline => "Spline",
            CurveKind::Arc => "Arc",
            CurveKind::Rectangle => "Rectangle",
            CurveKind::Circle => "Circle",
            CurveKind::Ellipse => "Ellipse",
            CurveKind::Polygon => "Polygon",
        }
    }
    fn hint(&self) -> String {
        match (self.kind, self.points.len()) {
            (CurveKind::Polyline, 0) => "Curve: click the first point".into(),
            (CurveKind::Polyline, _) => "Curve: click points; click the first point to close, Enter to finish open".into(),
            (CurveKind::Rectangle, 0) => "Rectangle: click the first corner".into(),
            (CurveKind::Rectangle, _) => "Rectangle: click the opposite corner or type the size".into(),
            (CurveKind::Circle, 0) => "Circle: click the center".into(),
            (CurveKind::Circle, _) => "Circle: click to set the radius or type it".into(),
            (CurveKind::Spline, 0) => "Spline: click the first point".into(),
            (CurveKind::Spline, _) => "Spline: click points it passes through; click the first point to close, Enter to finish open".into(),
            (CurveKind::Arc, 0) => "Arc: click the start".into(),
            (CurveKind::Arc, 1) => "Arc: click the end".into(),
            (CurveKind::Arc, _) => "Arc: click a point the arc passes through".into(),
            (CurveKind::Ellipse, 0) => "Ellipse: click the center".into(),
            (CurveKind::Ellipse, _) => "Ellipse: click a corner of its bounding box or type the radii".into(),
            (CurveKind::Polygon, 0) => "Polygon: click the center".into(),
            (CurveKind::Polygon, _) => "Polygon: click a corner or type the radius; set the sides in the panel".into(),
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
                CurveKind::Polyline | CurveKind::Spline | CurveKind::Arc => {}
            }
        }
        if !p.clicked {
            return ToolOutcome::Continue;
        }
        let Some(q) = pt else { return ToolOutcome::Continue };
        match self.kind {
            CurveKind::Arc => {
                self.points.push(q);
                if self.points.len() == 3 {
                    ToolOutcome::Finished
                } else {
                    ToolOutcome::Continue
                }
            }
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
            CurveKind::Polyline | CurveKind::Spline | CurveKind::Arc => {
                ui.label(format!("{} points", self.points.len()));
            }
        }
        ch
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
                CurveKind::Polyline | CurveKind::Spline | CurveKind::Arc => {}
            }
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
    cache: std::cell::RefCell<Option<((f64, BlendShape, bool, f64), Option<Body>, Option<String>)>>,
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
        Some(Self { target, body: o.body.clone(), edges, base, dir, start_t: None, radius: 0.0, shape, variable: false, end_radius: 1.0, typed: None, cache: std::cell::RefCell::new(None) })
    }

    fn value(&self) -> f64 {
        self.typed.unwrap_or(self.radius)
    }

    fn result(&self) -> (Option<Body>, Option<String>) {
        use ductile_brep::ops::fillet::{blend_edges, blend_edges_variable, Blend};
        let r = self.value();
        let key = (r, self.shape, self.variable, self.end_radius);
        if let Some((k, b, e)) = &*self.cache.borrow() {
            if *k == key {
                return (b.clone(), e.clone());
            }
        }
        let out = if r <= 1e-9 {
            (None, None)
        } else if self.variable && self.shape != BlendShape::Chamfer {
            match blend_edges_variable(&self.body, &self.edges, r, self.end_radius.max(1e-6), self.shape == BlendShape::Smooth) {
                Ok(b) => (Some(b), None),
                Err(e) => (None, Some(e.to_string())),
            }
        } else {
            let blend = match self.shape {
                BlendShape::Round => Blend::Fillet(r),
                BlendShape::Smooth => Blend::Smooth(r),
                BlendShape::Chamfer => Blend::Chamfer(r),
            };
            match blend_edges(&self.body, &self.edges, blend) {
                Ok(b) => (Some(b), None),
                Err(e) => (None, Some(e.to_string())),
            }
        };
        *self.cache.borrow_mut() = Some((key, out.0.clone(), out.1.clone()));
        out
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
        match self.cache.borrow().as_ref().and_then(|c| c.2.clone()) {
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
        self.result().0
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
        if self.result().0.is_some() {
            vec![self.target]
        } else {
            vec![]
        }
    }
    fn typed_value(&mut self, v: Option<f64>) {
        self.typed = v.map(|x| x.abs());
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let Some(b) = self.result().0 else { return false };
        doc.replace_body(self.target, b);
        doc.selection.clear();
        true
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
    cache: std::cell::RefCell<Option<(f64, OffsetMode, Option<Body>, Option<String>)>>,
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
        Some(Self { target, body: o.body.clone(), open: if mode == OffsetMode::Solid { vec![] } else { open }, base, dir, start_t: None, value: 0.0, mode, typed: None, cache: std::cell::RefCell::new(None) })
    }

    fn amount(&self) -> f64 {
        self.typed.unwrap_or(self.value)
    }

    fn result(&self) -> (Option<Body>, Option<String>) {
        use ductile_brep::ops::offset::{offset_body, shell};
        let t = self.amount();
        if let Some((ct, co, b, e)) = &*self.cache.borrow() {
            if *ct == t && *co == self.mode {
                return (b.clone(), e.clone());
            }
        }
        let out = if t.abs() <= 1e-9 || (self.mode == OffsetMode::Shell && t < 0.0) {
            (None, None)
        } else {
            let r = match self.mode {
                OffsetMode::Shell => shell(&self.body, &self.open, t),
                OffsetMode::Solid => offset_body(&self.body, &|_| t),
                OffsetMode::Faces => offset_body(&self.body, &|f| if self.open.contains(&f) { t } else { 0.0 }),
            };
            match r {
                Ok(b) => (Some(b), None),
                Err(e) => (None, Some(e.to_string())),
            }
        };
        *self.cache.borrow_mut() = Some((t, self.mode, out.0.clone(), out.1.clone()));
        out
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
        match self.cache.borrow().as_ref().and_then(|c| c.3.clone()) {
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
        self.result().0
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
        if self.result().0.is_some() {
            vec![self.target]
        } else {
            vec![]
        }
    }
    fn typed_value(&mut self, v: Option<f64>) {
        self.typed = v.map(|x| if self.mode != OffsetMode::Shell { x } else { x.abs() });
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let Some(b) = self.result().0 else { return false };
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
