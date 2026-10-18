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

/// Push/pull of a planar face (pulling out adds material, pushing in cuts),
/// or extrusion of the regions bounded by selected closed planar curves
/// into a new solid.
pub struct ExtrudeTool {
    /// Solid whose face is pushed or pulled; `None` when extruding curves.
    target: Option<ObjectId>,
    body: Option<std::sync::Arc<Body>>,
    regions: Vec<ductile_brep::profile::Region>,
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
                    return None;
                }
                (None, None, regions)
            }
        };
        let poly = regions[0].outer.polygon(1e-3);
        let base = poly.iter().copied().sum::<DVec3>() / poly.len().max(1) as f64;
        Some(Self {
            target,
            body,
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

// -------------------------------------------------------------- Revolve ----

/// Revolves the regions of the selected closed curves about an axis in
/// their plane: a selected straight edge, or the construction plane's Y
/// (or X) axis.
pub struct RevolveTool {
    regions: Vec<ductile_brep::profile::Region>,
    edge_axis: Option<(DVec3, DVec3)>,
    /// 0: selected edge (if any), 1: construction plane X, 2: Y.
    pub axis_choice: usize,
    pub degrees: f64,
    cache: std::cell::RefCell<Option<(usize, f64, Option<Body>, Option<String>)>>,
}

impl RevolveTool {
    pub fn new(doc: &Document) -> Option<Self> {
        let regions = selected_curve_regions(doc);
        if regions.is_empty() {
            return None;
        }
        let edge_axis = doc.selected_edges().into_iter().find_map(|(o, e)| {
            let ed = doc.object(o)?.body.edge(e).clone();
            ed.curve.is_linear().then(|| (ed.start(), (ed.end() - ed.start()).normalize()))
        });
        let axis_choice = if edge_axis.is_some() { 0 } else { 2 };
        Some(Self { regions, edge_axis, axis_choice, degrees: 360.0, cache: std::cell::RefCell::new(None) })
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
        let l = self.regions.iter().flat_map(|r| r.outer.polygon(1e-2)).map(|p| (p - o).length()).fold(1.0, f64::max) * 1.5;
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
