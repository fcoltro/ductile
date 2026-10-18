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
use ductile_brep::Body;
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
        let targets = doc.selected_objects();
        let mut bb = ductile_geom::BBox3::EMPTY;
        for id in &targets {
            if let Some(o) = doc.object(*id) {
                bb = bb.union(&o.body.bbox());
            }
        }
        let pivot = if bb.is_empty() { DVec3::ZERO } else { bb.center() };
        Self { kind, targets, pivot, start: None, current: None, axis: None, typed: None, value: 0.0 }
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
                let b = o.body.transformed(&m);
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
                let b = o.body.transformed(&m);
                doc.replace_body(id, b);
            }
        }
        true
    }
}


// -------------------------------------------------------------- Extrude ----

/// Push/pull of a planar face: pulling out adds material, pushing in cuts.
pub struct ExtrudeTool {
    target: ObjectId,
    body: std::sync::Arc<Body>,
    region: ductile_brep::profile::Region,
    normal: DVec3,
    base: DVec3,
    start_t: Option<f64>,
    pub distance: f64,
    typed: Option<f64>,
    cache: std::cell::RefCell<Option<(f64, Option<Body>, Option<String>)>>,
}

impl ExtrudeTool {
    /// Starts on the first selected planar face, if any.
    pub fn new(doc: &Document) -> Option<Self> {
        let (id, f) = doc.selection.iter().find_map(|p| match p {
            Pick::Face(o, f) => Some((*o, *f)),
            _ => None,
        })?;
        let o = doc.object(id)?;
        let region = ductile_brep::profile::Region::from_face(&o.body, f)?;
        let poly = region.outer.polygon(1e-3);
        let base = poly.iter().copied().sum::<DVec3>() / poly.len().max(1) as f64;
        Some(Self {
            target: id,
            body: o.body.clone(),
            normal: region.plane.z,
            region,
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
        let d = self.value();
        if let Some((cd, b, e)) = &*self.cache.borrow() {
            if *cd == d {
                return (b.clone(), e.clone());
            }
        }
        let out = if d.abs() < 1e-9 {
            (None, None)
        } else {
            match ductile_brep::ops::extrude::extrude_region(&self.region, self.normal * d) {
                None => (None, Some("could not build the extrusion".into())),
                Some(prism) => {
                    let op = if d > 0.0 { ductile_brep::ops::boolean::BoolOp::Union } else { ductile_brep::ops::boolean::BoolOp::Difference };
                    match ductile_brep::ops::boolean::boolean(&self.body, &prism, op) {
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
        if self.result().0.is_some() {
            vec![self.target]
        } else {
            vec![]
        }
    }
    fn typed_value(&mut self, v: Option<f64>) {
        self.typed = v;
    }
    fn commit(&mut self, doc: &mut Document) -> bool {
        let Some(b) = self.result().0 else { return false };
        doc.replace_body(self.target, b);
        doc.selection.clear();
        true
    }
}

/// Fillet or chamfer of the selected edges of one solid, with the radius
/// dragged along the edge's bisector.
pub struct FilletTool {
    target: ObjectId,
    body: std::sync::Arc<Body>,
    edges: Vec<ductile_brep::EdgeId>,
    base: DVec3,
    dir: DVec3,
    start_t: Option<f64>,
    pub radius: f64,
    pub chamfer: bool,
    typed: Option<f64>,
    cache: std::cell::RefCell<Option<(f64, bool, Option<Body>, Option<String>)>>,
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
        Some(Self { target, body: o.body.clone(), edges, base, dir, start_t: None, radius: 0.0, chamfer, typed: None, cache: std::cell::RefCell::new(None) })
    }

    fn value(&self) -> f64 {
        self.typed.unwrap_or(self.radius)
    }

    fn result(&self) -> (Option<Body>, Option<String>) {
        use ductile_brep::ops::fillet::{blend_edges, Blend};
        let r = self.value();
        if let Some((cr, cc, b, e)) = &*self.cache.borrow() {
            if *cr == r && *cc == self.chamfer {
                return (b.clone(), e.clone());
            }
        }
        let out = if r <= 1e-9 {
            (None, None)
        } else {
            let blend = if self.chamfer { Blend::Chamfer(r) } else { Blend::Fillet(r) };
            match blend_edges(&self.body, &self.edges, blend) {
                Ok(b) => (Some(b), None),
                Err(e) => (None, Some(e.to_string())),
            }
        };
        *self.cache.borrow_mut() = Some((r, self.chamfer, out.0.clone(), out.1.clone()));
        out
    }
}

impl Tool for FilletTool {
    fn name(&self) -> &'static str {
        if self.chamfer {
            "Chamfer"
        } else {
            "Fillet"
        }
    }
    fn hint(&self) -> String {
        match self.cache.borrow().as_ref().and_then(|c| c.3.clone()) {
            Some(e) => format!("{} failed: {e}", self.name()),
            None => format!("{}: drag or type a {} ({:.4}), click or Enter to finish", self.name(), if self.chamfer { "distance" } else { "radius" }, self.value()),
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
        let mut ch = drag_value(ui, if self.chamfer { "Distance" } else { "Radius" }, &mut r, 0.01);
        if ch {
            self.typed = Some(r.max(0.0));
        }
        ch |= ui.checkbox(&mut self.chamfer, "Chamfer").changed();
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
