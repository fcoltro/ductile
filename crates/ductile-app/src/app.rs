// SPDX-License-Identifier: GPL-3.0-only
//! The application: layout, input handling and glue between the document,
//! tools and the viewport renderer.

use crate::camera::{Camera, Projection};
use crate::commands::{self, Command};
use crate::document::{mesh_body, Document, ObjectId, Pick, SelectMode};
use crate::picking;
use crate::render::{LineInstance, ObjectStyle, ViewportRenderer};
use crate::theme;
use crate::tools::{self, CPlane, PointerInfo, Tool, ToolOutcome, TransformKind, TransformTool};
use ductile_brep::{Body, BodyMesh};
use glam::{DVec2, DVec3};
use std::collections::HashMap;
use std::sync::Arc;

const PREVIEW_ID: u64 = u64::MAX;

pub struct Options {
    pub screenshot: Option<std::path::PathBuf>,
    pub open: Option<std::path::PathBuf>,
    pub demo: bool,
    pub zebra: bool,
    /// Camera override: target x, y, z, distance, yaw and pitch in degrees.
    pub camera: Option<[f64; 6]>,
    /// Index of an object to select at start (for screenshots).
    pub select: Option<usize>,
}

struct BoxSelect {
    start: egui::Pos2,
    current: egui::Pos2,
}

pub struct DuctileApp {
    pub doc: Document,
    pub camera: Camera,
    pub cplane: CPlane,
    renderer: Option<ViewportRenderer>,
    uploaded: HashMap<ObjectId, u64>,
    style_key: u64,
    pub tool: Option<Box<dyn Tool>>,
    tool_from_key: bool,
    preview_shown: bool,
    /// The body last uploaded as the preview, to skip re-meshing it.
    preview_body: Option<Body>,
    pub status: String,
    box_select: Option<BoxSelect>,
    typed: String,
    pub show_grid: bool,
    pub show_edges: bool,
    pub palette_open: bool,
    pub palette_query: String,
    pub show_help: bool,
    screenshot: Option<(std::path::PathBuf, u32)>,
    viewport_rect: egui::Rect,
    last_cursor: Option<DVec2>,
    pub pending: Vec<Command>,
    pub snap_enabled: bool,
    snap_marker: Option<DVec3>,
    screenshot_done: Arc<std::sync::atomic::AtomicBool>,
    pub pending_finish: bool,
    pub pending_cancel: bool,
    /// The transform-gizmo handle under the cursor.
    gizmo_hover: Option<(TransformKind, usize)>,
    /// A transform started by dragging a gizmo handle; it ends on release.
    gizmo_drag: bool,
}

/// One draggable handle of the transform gizmo, as a screen polyline.
struct GizmoHandle {
    kind: TransformKind,
    axis: usize,
    pts: Vec<egui::Pos2>,
}

const AXIS_COLORS: [egui::Color32; 3] = [egui::Color32::from_rgb(220, 80, 80), egui::Color32::from_rgb(110, 200, 80), egui::Color32::from_rgb(80, 130, 230)];

fn dist_to_polyline(p: egui::Pos2, pts: &[egui::Pos2]) -> f32 {
    pts.windows(2)
        .map(|w| {
            let (a, b) = (w[0], w[1]);
            let ab = b - a;
            let t = ((p - a).dot(ab) / ab.length_sq().max(1e-6)).clamp(0.0, 1.0);
            p.distance(a + ab * t)
        })
        .fold(f32::INFINITY, f32::min)
}

impl DuctileApp {
    pub fn new(cc: &eframe::CreationContext<'_>, opts: Options) -> Self {
        theme::apply(&cc.egui_ctx);
        let renderer = cc.wgpu_render_state.as_ref().map(|rs| ViewportRenderer::new(rs.device.clone(), rs.queue.clone()));
        let mut app = Self {
            doc: Document::default(),
            camera: Camera::default(),
            cplane: CPlane::default(),
            renderer,
            uploaded: HashMap::new(),
            style_key: 0,
            tool: None,
            tool_from_key: false,
            gizmo_hover: None,
            gizmo_drag: false,
            preview_shown: false,
            preview_body: None,
            status: "Welcome to Ductile. Press Space for the command palette, F1 for help.".into(),
            box_select: None,
            typed: String::new(),
            show_grid: true,
            show_edges: true,
            palette_open: false,
            palette_query: String::new(),
            show_help: false,
            screenshot: opts.screenshot.map(|p| (p, 0)),
            viewport_rect: egui::Rect::NOTHING,
            last_cursor: None,
            pending: vec![],
            snap_enabled: true,
            snap_marker: None,
            screenshot_done: Default::default(),
            pending_finish: false,
            pending_cancel: false,
        };
        if let Some(path) = opts.open {
            commands::open_file(&mut app, &path);
        } else if opts.demo {
            commands::demo_scene(&mut app.doc);
            let bb = app.doc.bbox();
            app.camera.frame(bb.center(), bb.diagonal() * 0.5);
        }
        if opts.zebra {
            app.toggle_zebra();
        }
        if let Some(id) = opts.select.and_then(|i| app.doc.objects.get(i)).map(|o| o.id) {
            app.doc.selection.insert(Pick::Object(id));
        }
        if let Some(c) = opts.camera {
            app.camera.target = DVec3::new(c[0], c[1], c[2]);
            app.camera.distance = c[3];
            app.camera.yaw = c[4].to_radians();
            app.camera.pitch = c[5].to_radians();
        }
        app
    }

    pub fn start_tool(&mut self, t: Box<dyn Tool>) {
        self.tool = Some(t);
        self.typed.clear();
        self.tool_from_key = false;
    }

    pub fn cancel_tool(&mut self) {
        self.tool = None;
        self.typed.clear();
    }

    fn finish_tool(&mut self) {
        if let Some(mut t) = self.tool.take() {
            self.doc.checkpoint(t.name());
            if !t.commit(&mut self.doc) {
                // Nothing changed: drop the checkpoint.
                self.doc.undo();
                self.doc.dirty = true;
            } else {
                self.status = format!("{} done", t.name());
            }
        }
        self.typed.clear();
    }

    // ----------------------------------------------------------- frame ----

    fn sync_renderer(&mut self) {
        let Some(r) = self.renderer.as_mut() else { return };
        let ids: std::collections::HashSet<ObjectId> = self.doc.objects.iter().map(|o| o.id).collect();
        self.uploaded.retain(|id, _| ids.contains(id));
        r.retain_objects(|id| ids.contains(&id) || id == PREVIEW_ID);
        let mut changed = false;
        for o in &self.doc.objects {
            if self.uploaded.get(&o.id) != Some(&o.revision) {
                r.upload_object(o.id, &o.mesh);
                self.uploaded.insert(o.id, o.revision);
                changed = true;
            }
        }
        // Styles depend on selection/hover; recompute when they change.
        let hidden: Vec<ObjectId> = self.tool.as_ref().map(|t| t.hidden_objects()).unwrap_or_default();
        let key = {
            use std::hash::{Hash, Hasher};
            let mut h = std::collections::hash_map::DefaultHasher::new();
            self.doc.selection.hash(&mut h);
            self.doc.hover.hash(&mut h);
            hidden.hash(&mut h);
            self.show_edges.hash(&mut h);
            for o in &self.doc.objects {
                o.visible.hash(&mut h);
                o.revision.hash(&mut h);
                o.color.map(|c| c.to_bits()).hash(&mut h);
            }
            h.finish()
        };
        if key == self.style_key && !changed {
            return;
        }
        self.style_key = key;
        for o in &self.doc.objects {
            let mut st = ObjectStyle::default();
            st.color = [o.color[0], o.color[1], o.color[2], 1.0];
            st.visible = o.visible && !hidden.contains(&o.id);
            if self.doc.selection.contains(&Pick::Object(o.id)) {
                st.state = 2;
            } else if self.doc.hover == Some(Pick::Object(o.id)) {
                st.state = 1;
            }
            let mut faces = vec![0u32; o.mesh.faces.len()];
            for p in &self.doc.selection {
                if let Pick::Face(oid, f) = p {
                    if *oid == o.id && f.idx() < faces.len() {
                        faces[f.idx()] = 2;
                    }
                }
            }
            if let Some(Pick::Face(oid, f)) = self.doc.hover {
                if oid == o.id && f.idx() < faces.len() && faces[f.idx()] == 0 {
                    faces[f.idx()] = 1;
                }
            }
            r.set_style(o.id, &st, &faces);
            let lines = edge_lines(&o.mesh, &o.body, o.id, &self.doc, self.show_edges);
            r.set_edges(o.id, &lines);
        }
    }

    fn update_preview(&mut self) {
        let Some(r) = self.renderer.as_mut() else { return };
        let body = self.tool.as_ref().and_then(|t| t.preview(&self.doc));
        if body.is_some() && body == self.preview_body {
            return;
        }
        self.preview_body = body.clone();
        match body {
            Some(b) => {
                let mesh = mesh_body(&b);
                r.upload_object(PREVIEW_ID, &mesh);
                let st = ObjectStyle { color: [0.55, 0.62, 0.78, 1.0], ..Default::default() };
                r.set_style(PREVIEW_ID, &st, &[]);
                let lines: Vec<LineInstance> = mesh
                    .edges
                    .iter()
                    .flat_map(|p| p.windows(2).map(|w| LineInstance::new(w[0], w[1], [0.85, 0.9, 1.0, 1.0], 1.5)))
                    .collect();
                r.set_edges(PREVIEW_ID, &lines);
                self.preview_shown = true;
            }
            None => {
                if self.preview_shown {
                    r.remove_object(PREVIEW_ID);
                    self.preview_shown = false;
                }
            }
        }
    }

    fn grid_lines(&self, out: &mut Vec<LineInstance>) {
        if !self.show_grid {
            return;
        }
        let f = self.cplane.frame;
        let wpp = self.camera.world_per_pixel(self.camera.target, self.viewport_rect.height().max(1.0) as f64);
        let step = tools::nice_step(wpp * 40.0);
        let major = step * 5.0;
        let n = 30i64;
        let c = f.to_local(self.camera.target);
        let cx = (c.x / major).round() * major;
        let cy = (c.y / major).round() * major;
        let ext = step * n as f64;
        let fade_r = ext * 0.9;
        let mut push = |a: DVec3, b: DVec3, col: [f32; 4]| {
            // Split into pieces whose alpha fades with distance from the view target.
            let segs = 12;
            for k in 0..segs {
                let p0 = a.lerp(b, k as f64 / segs as f64);
                let p1 = a.lerp(b, (k + 1) as f64 / segs as f64);
                let d = (((p0 + p1) * 0.5) - f.to_world(DVec3::new(cx, cy, 0.0))).length();
                let fade = (1.0 - d / fade_r).clamp(0.0, 1.0) as f32;
                if fade <= 0.01 {
                    continue;
                }
                let c = [col[0], col[1], col[2], col[3] * fade * fade];
                out.push(LineInstance { style: [1.0, 0.0, -1.0, 0.0], ..LineInstance::new(p0, p1, c, 1.0) });
            }
        };
        for i in -n..=n {
            let x = cx + step * i as f64;
            let y = cy + step * i as f64;
            let is_major_x = ((x / major).round() * major - x).abs() < step * 0.01;
            let is_major_y = ((y / major).round() * major - y).abs() < step * 0.01;
            let minor_col = [0.20, 0.21, 0.24, 0.5];
            let major_col = [0.28, 0.29, 0.33, 0.85];
            let cx_col = if x.abs() < step * 0.01 { [0.30, 0.60, 0.25, 1.0] } else if is_major_x { major_col } else { minor_col };
            let cy_col = if y.abs() < step * 0.01 { [0.70, 0.25, 0.25, 1.0] } else if is_major_y { major_col } else { minor_col };
            push(f.to_world(DVec3::new(x, cy - ext, 0.0)), f.to_world(DVec3::new(x, cy + ext, 0.0)), cx_col);
            push(f.to_world(DVec3::new(cx - ext, y, 0.0)), f.to_world(DVec3::new(cx + ext, y, 0.0)), cy_col);
        }
    }

    /// Snapped pointer position: vertices, edge midpoints, face centres and the grid.
    fn snap(&self, cursor: DVec2, size: DVec2, ray: &crate::camera::Ray, ctrl: bool) -> (Option<DVec3>, Option<DVec3>) {
        let f = self.cplane.frame;
        let plane_pt = ray.hit_plane(f.origin, f.z);
        if self.snap_enabled {
            if let Some((o, v)) = picking::pick_vertex(&self.doc, &self.camera, cursor, size, 10.0, None) {
                return (self.doc.object(o).map(|ob| ob.body.vertex(v)), plane_pt);
            }
            if let Some((o, e)) = picking::pick_edge(&self.doc, &self.camera, cursor, size, 8.0, None) {
                if let Some(ob) = self.doc.object(o) {
                    let edge = ob.body.edge(e);
                    let mid = edge.mid();
                    let to_s = |p: DVec3| {
                        self.camera.project(p).map(|n| DVec2::new((n.x * 0.5 + 0.5) * size.x, (0.5 - n.y * 0.5) * size.y))
                    };
                    if let Some(ms) = to_s(mid) {
                        if ms.distance(cursor) < 12.0 {
                            return (Some(mid), plane_pt);
                        }
                    }
                    // Closest point on the edge to the view ray.
                    let t = edge.curve.closest_param_in(ray.at((mid - ray.origin).dot(ray.dir)), edge.t0, edge.t1);
                    return (Some(edge.curve.point(t)), plane_pt);
                }
            }
            if let Some(h) = picking::pick_face(&self.doc, ray) {
                return (Some(h.point), plane_pt);
            }
        }
        let grid = plane_pt.map(|p| {
            if ctrl || self.snap_enabled {
                let wpp = self.camera.world_per_pixel(p, size.y);
                let step = tools::nice_step(wpp * 40.0) / if ctrl { 1.0 } else { 4.0 };
                let l = f.to_local(p);
                f.to_world(DVec3::new((l.x / step).round() * step, (l.y / step).round() * step, 0.0))
            } else {
                p
            }
        });
        (grid, plane_pt)
    }

    fn viewport(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        let rect = ui.available_rect_before_wrap();
        self.viewport_rect = rect;
        let resp = ui.allocate_rect(rect, egui::Sense::click_and_drag());
        let size = DVec2::new(rect.width() as f64, rect.height() as f64);
        self.camera.aspect = (size.x / size.y.max(1.0)).max(0.01);
        let (mods, scroll, pointer) = ui.input(|i| (i.modifiers, i.smooth_scroll_delta.y, i.pointer.clone()));

        // Navigation.
        let nav_drag = pointer.button_down(egui::PointerButton::Middle) || pointer.button_down(egui::PointerButton::Secondary);
        if resp.dragged_by(egui::PointerButton::Middle) || resp.dragged_by(egui::PointerButton::Secondary) {
            let d = resp.drag_delta();
            if mods.shift {
                self.camera.pan(d.x as f64, d.y as f64, size.y);
            } else if mods.ctrl {
                self.camera.zoom((1.0 + d.y as f64 * 0.01).max(0.1), None);
            } else {
                self.camera.orbit(d.x as f64, d.y as f64);
            }
        }
        let hover_pos = resp.hover_pos();
        let cursor = hover_pos.map(|p| DVec2::new((p.x - rect.min.x) as f64, (p.y - rect.min.y) as f64));
        if resp.hovered() && scroll.abs() > 0.0 {
            let toward = cursor.and_then(|c| {
                let ndc = DVec2::new(c.x / size.x * 2.0 - 1.0, 1.0 - c.y / size.y * 2.0);
                let ray = self.camera.ray(ndc);
                picking::pick_face(&self.doc, &ray).map(|h| h.point).or_else(|| ray.hit_plane(DVec3::ZERO, DVec3::Z))
            });
            self.camera.zoom((-scroll as f64 * 0.0015).exp(), toward);
        }

        // Tool / selection interaction.
        let gizmo = if self.tool.is_none() { self.gizmo_handles(rect) } else { None };
        if self.gizmo_drag && (self.tool.is_none() || !pointer.primary_down()) {
            // Released (or the tool was cancelled): a gizmo drag confirms on release.
            self.gizmo_drag = false;
            if self.tool.is_some() {
                self.finish_tool();
            }
        }
        if let Some(c) = cursor {
            let ndc = DVec2::new(c.x / size.x * 2.0 - 1.0, 1.0 - c.y / size.y * 2.0);
            let ray = self.camera.ray(ndc);
            let moved = self.last_cursor.map(|l| l.distance(c) > 0.5).unwrap_or(true);
            self.last_cursor = Some(c);
            let clicked = resp.clicked_by(egui::PointerButton::Primary);
            if self.tool.is_some() {
                let wants = self.tool.as_ref().unwrap().wants_selection();
                if let Some(mode) = wants {
                    // Tools that pick entities first behave like selection.
                    if moved {
                        self.doc.hover = picking::pick(&self.doc, &self.camera, c, size, mode);
                    }
                }
                let (pt, plane_pt) = self.snap(c, size, &ray, mods.ctrl);
                let info = PointerInfo {
                    ray,
                    point: pt,
                    plane_point: plane_pt,
                    clicked,
                    world_per_px: self.camera.world_per_pixel(pt.unwrap_or(self.camera.target), size.y),
                };
                tools::set_current_cplane(self.cplane);
                let out = {
                    let doc = &self.doc;
                    let cplane = self.cplane;
                    self.tool.as_mut().unwrap().pointer(doc, &cplane, &info)
                };
                if wants.is_some() && clicked {
                    self.click_select(c, size, mods);
                }
                match out {
                    ToolOutcome::Finished => self.finish_tool(),
                    ToolOutcome::Cancelled => self.cancel_tool(),
                    ToolOutcome::Continue => {}
                }
                if let Some(p) = pt {
                    self.snap_marker = Some(p);
                }
            } else {
                let cp = egui::pos2(rect.min.x + c.x as f32, rect.min.y + c.y as f32);
                // Hover picks the handle under the press point while a drag begins.
                let at = if resp.drag_started_by(egui::PointerButton::Primary) { pointer.press_origin().unwrap_or(cp) } else { cp };
                self.gizmo_hover = if self.box_select.is_none() && !nav_drag {
                    gizmo
                        .iter()
                        .flatten()
                        .map(|h| (dist_to_polyline(at, &h.pts), h))
                        .filter(|(d, _)| *d < 7.0)
                        .min_by(|a, b| a.0.total_cmp(&b.0))
                        .map(|(_, h)| (h.kind, h.axis))
                } else {
                    None
                };
                if moved && !nav_drag && self.box_select.is_none() && self.gizmo_hover.is_none() {
                    self.doc.hover = picking::pick(&self.doc, &self.camera, c, size, self.doc.mode);
                }
                if clicked && self.gizmo_hover.is_none() {
                    self.click_select(c, size, mods);
                }
                if let (Some((kind, axis)), true) = (self.gizmo_hover, resp.drag_started_by(egui::PointerButton::Primary)) {
                    let mut t = TransformTool::new(kind, &self.doc);
                    t.axis = Some(axis);
                    self.start_tool(Box::new(t));
                    self.gizmo_drag = true;
                    self.gizmo_hover = None;
                } else if resp.drag_started_by(egui::PointerButton::Primary) {
                    if let Some(p) = resp.interact_pointer_pos() {
                        self.box_select = Some(BoxSelect { start: p, current: p });
                    }
                }
                if let Some(bs) = &mut self.box_select {
                    if let Some(p) = resp.interact_pointer_pos() {
                        bs.current = p;
                    }
                    if resp.drag_stopped_by(egui::PointerButton::Primary) {
                        let a = DVec2::new((bs.start.x - rect.min.x) as f64, (bs.start.y - rect.min.y) as f64);
                        let b = DVec2::new((bs.current.x - rect.min.x) as f64, (bs.current.y - rect.min.y) as f64);
                        let picks = picking::pick_box(&self.doc, &self.camera, a, b, size, self.doc.mode);
                        if !mods.shift && !mods.ctrl {
                            self.doc.selection.clear();
                        }
                        for p in picks {
                            if mods.ctrl {
                                self.doc.selection.remove(&p);
                            } else {
                                self.doc.selection.insert(p);
                            }
                        }
                        self.box_select = None;
                    }
                }
            }
        } else {
            self.doc.hover = None;
        }

        // Render.
        self.sync_renderer();
        self.update_preview();
        let mut lines = Vec::new();
        self.grid_lines(&mut lines);
        let mut overlay = Vec::new();
        if let Some(t) = &self.tool {
            t.guides(&mut overlay);
        }
        if let Some(p) = self.snap_marker.take() {
            if self.tool.is_some() {
                overlay.push(LineInstance::point(p, [0.3, 0.9, 1.0, 1.0], 6.0));
            }
        }
        vertex_overlay(&self.doc, &mut overlay);
        let ppp = ui.ctx().pixels_per_point();
        let px = [(rect.width() * ppp).round() as u32, (rect.height() * ppp).round() as u32];
        if let (Some(r), Some(rs)) = (self.renderer.as_mut(), frame.wgpu_render_state()) {
            let mut egui_renderer = rs.renderer.write();
            let tex = r.render(&mut egui_renderer, &self.camera, px, &lines, &overlay);
            drop(egui_renderer);
            ui.painter().image(tex, rect, egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)), egui::Color32::WHITE);
        } else {
            ui.painter().rect_filled(rect, 0.0, egui::Color32::from_gray(20));
            ui.painter().text(rect.center(), egui::Align2::CENTER_CENTER, "wgpu renderer unavailable", egui::FontId::proportional(16.0), egui::Color32::GRAY);
        }
        if let Some(bs) = &self.box_select {
            let r = egui::Rect::from_two_pos(bs.start, bs.current);
            ui.painter().rect_filled(r, 0.0, egui::Color32::from_rgba_unmultiplied(120, 150, 255, 25));
            ui.painter().rect_stroke(r, 0.0, egui::Stroke::new(1.0, egui::Color32::from_rgb(120, 150, 255)), egui::StrokeKind::Inside);
        }
        if let Some(h) = &gizmo {
            if self.tool.is_none() {
                self.draw_gizmo(ui, rect, h);
            }
        }
        self.view_gizmo(ui, rect);
        if !self.typed.is_empty() {
            ui.painter().text(
                rect.left_bottom() + egui::vec2(12.0, -12.0),
                egui::Align2::LEFT_BOTTOM,
                format!("= {}", self.typed),
                egui::FontId::monospace(16.0),
                theme::ACCENT,
            );
        }
    }

    /// The move arrows and rotate rings drawn at the selection, sized to stay
    /// the same on screen. None when nothing selected can be transformed.
    fn gizmo_handles(&self, rect: egui::Rect) -> Option<Vec<GizmoHandle>> {
        if self.doc.selection.is_empty() {
            return None;
        }
        let t = TransformTool::new(TransformKind::Move, &self.doc);
        if !t.has_targets() {
            return None;
        }
        let pivot = t.pivot();
        let to_screen = |p: DVec3| {
            self.camera.project(p).map(|n| egui::pos2(rect.min.x + ((n.x + 1.0) * 0.5) as f32 * rect.width(), rect.min.y + ((1.0 - n.y) * 0.5) as f32 * rect.height()))
        };
        let o = to_screen(pivot)?;
        let wpp = self.camera.world_per_pixel(pivot, rect.height() as f64);
        let axes = [DVec3::X, DVec3::Y, DVec3::Z];
        let mut out = Vec::new();
        for (i, a) in axes.iter().enumerate() {
            // Arrows start clear of the rings so the two never overlap.
            if let (Some(p0), Some(p1)) = (to_screen(pivot + *a * 60.0 * wpp), to_screen(pivot + *a * 110.0 * wpp)) {
                if p0.distance(p1) > 12.0 && o.distance(p0) > 4.0 {
                    out.push(GizmoHandle { kind: TransformKind::Move, axis: i, pts: vec![p0, p1] });
                }
            }
            let (u, v) = (axes[(i + 1) % 3], axes[(i + 2) % 3]);
            let pts: Option<Vec<egui::Pos2>> = (0..=48)
                .map(|k| {
                    let t = k as f64 / 48.0 * std::f64::consts::TAU;
                    to_screen(pivot + (u * t.cos() + v * t.sin()) * 45.0 * wpp)
                })
                .collect();
            if let Some(pts) = pts {
                out.push(GizmoHandle { kind: TransformKind::Rotate, axis: i, pts });
            }
        }
        Some(out)
    }

    fn draw_gizmo(&self, ui: &egui::Ui, rect: egui::Rect, handles: &[GizmoHandle]) {
        let painter = ui.painter_at(rect);
        for h in handles {
            let hot = self.gizmo_hover == Some((h.kind, h.axis));
            let col = if hot { egui::Color32::from_rgb(255, 220, 90) } else { AXIS_COLORS[h.axis] };
            match h.kind {
                TransformKind::Move => {
                    let (a, b) = (h.pts[0], h.pts[1]);
                    painter.line_segment([a, b], egui::Stroke::new(if hot { 3.5 } else { 2.5 }, col));
                    let d = (b - a).normalized();
                    let n = egui::vec2(-d.y, d.x);
                    painter.add(egui::Shape::convex_polygon(vec![b + d * 12.0, b + n * 6.0, b - n * 6.0], col, egui::Stroke::NONE));
                }
                _ => {
                    painter.add(egui::Shape::line(h.pts.clone(), egui::Stroke::new(if hot { 3.0 } else { 1.8 }, col)));
                }
            }
        }
    }

    fn click_select(&mut self, c: DVec2, size: DVec2, mods: egui::Modifiers) {
        let mode = self.tool.as_ref().and_then(|t| t.wants_selection()).unwrap_or(self.doc.mode);
        let hit = picking::pick(&self.doc, &self.camera, c, size, mode);
        match hit {
            Some(p) => {
                if mods.shift {
                    if !self.doc.selection.remove(&p) {
                        self.doc.selection.insert(p);
                    }
                } else if mods.ctrl {
                    self.doc.selection.remove(&p);
                } else if self.tool.is_some() {
                    // Inside a picking tool, clicks accumulate.
                    if !self.doc.selection.remove(&p) {
                        self.doc.selection.insert(p);
                    }
                } else {
                    self.doc.selection.clear();
                    self.doc.selection.insert(p);
                }
            }
            None => {
                if !mods.shift && !mods.ctrl && self.tool.is_none() {
                    self.doc.selection.clear();
                }
            }
        }
    }

    /// Small orientation widget in the viewport corner; click an axis to view along it.
    fn view_gizmo(&mut self, ui: &mut egui::Ui, rect: egui::Rect) {
        let center = rect.right_top() + egui::vec2(-60.0, 60.0);
        let painter = ui.painter_at(rect);
        let view = self.camera.view();
        let axes = [
            (DVec3::X, "X", egui::Color32::from_rgb(220, 80, 80)),
            (DVec3::Y, "Y", egui::Color32::from_rgb(110, 200, 80)),
            (DVec3::Z, "Z", egui::Color32::from_rgb(80, 130, 230)),
        ];
        let mut items = Vec::new();
        for (a, name, col) in axes {
            for sign in [1.0, -1.0] {
                let v = view.transform_vector3(a * sign);
                items.push((v.z, v, name, col, sign));
            }
        }
        items.sort_by(|a, b| a.0.total_cmp(&b.0));
        painter.circle_filled(center, 48.0, egui::Color32::from_rgba_unmultiplied(255, 255, 255, 8));
        let click = ui.input(|i| i.pointer.primary_clicked().then(|| i.pointer.interact_pos()).flatten());
        for (_, v, name, col, sign) in items {
            let p = center + egui::vec2(v.x as f32, -v.y as f32) * 38.0;
            if sign > 0.0 {
                painter.line_segment([center, p], egui::Stroke::new(2.0, col));
                painter.circle_filled(p, 9.0, col);
                painter.text(p, egui::Align2::CENTER_CENTER, name, egui::FontId::proportional(11.0), egui::Color32::BLACK);
            } else {
                painter.circle_filled(p, 7.0, col.gamma_multiply(0.45));
            }
            if let Some(cp) = click {
                if cp.distance(p) < 9.0 {
                    let dir = match name {
                        "X" => DVec3::X,
                        "Y" => DVec3::Y,
                        _ => DVec3::Z,
                    } * sign;
                    self.camera.set_view(dir);
                }
            }
        }
    }

    fn handle_keys(&mut self, ctx: &egui::Context) {
        if ctx.egui_wants_keyboard_input() {
            return;
        }
        let events = ctx.input(|i| i.events.clone());
        for ev in events {
            let egui::Event::Key { key, pressed: true, modifiers, .. } = ev else { continue };
            // Modal typing for active tools.
            if self.tool.is_some() {
                let digit = match key {
                    egui::Key::Num0 => Some('0'),
                    egui::Key::Num1 => Some('1'),
                    egui::Key::Num2 => Some('2'),
                    egui::Key::Num3 => Some('3'),
                    egui::Key::Num4 => Some('4'),
                    egui::Key::Num5 => Some('5'),
                    egui::Key::Num6 => Some('6'),
                    egui::Key::Num7 => Some('7'),
                    egui::Key::Num8 => Some('8'),
                    egui::Key::Num9 => Some('9'),
                    egui::Key::Period => Some('.'),
                    egui::Key::Minus => Some('-'),
                    _ => None,
                };
                if let Some(d) = digit {
                    self.typed.push(d);
                    let v = self.typed.parse::<f64>().ok();
                    self.tool.as_mut().unwrap().typed_value(v);
                    continue;
                }
                match key {
                    egui::Key::Backspace => {
                        self.typed.pop();
                        let v = self.typed.parse::<f64>().ok();
                        self.tool.as_mut().unwrap().typed_value(v);
                        continue;
                    }
                    egui::Key::Escape => {
                        self.cancel_tool();
                        self.status = "Cancelled".into();
                        continue;
                    }
                    egui::Key::Enter => {
                        match self.tool.as_mut().unwrap().accept() {
                            ToolOutcome::Finished => self.finish_tool(),
                            ToolOutcome::Cancelled => self.cancel_tool(),
                            ToolOutcome::Continue => {}
                        }
                        continue;
                    }
                    egui::Key::X | egui::Key::Y | egui::Key::Z if !modifiers.ctrl => {
                        let a = match key {
                            egui::Key::X => 0,
                            egui::Key::Y => 1,
                            _ => 2,
                        };
                        self.tool.as_mut().unwrap().constrain_axis(a);
                        continue;
                    }
                    _ => {}
                }
            }
            if let Some(cmd) = commands::shortcut(key, modifiers) {
                self.pending.push(cmd);
            }
        }
    }
}

/// Edge display lines with selection/hover colours.
fn edge_lines(mesh: &BodyMesh, body: &ductile_brep::Body, id: ObjectId, doc: &Document, show: bool) -> Vec<LineInstance> {
    let mut out = Vec::new();
    for (ei, poly) in mesh.edges.iter().enumerate() {
        let e = ductile_brep::EdgeId(ei as u32);
        if is_seam(body, e) {
            continue;
        }
        // Curves are always drawn and highlight with their object.
        let wire = body.kind == ductile_brep::BodyKind::Wire;
        let sel = doc.selection.contains(&Pick::Edge(id, e)) || (wire && doc.selection.contains(&Pick::Object(id)));
        let hov = doc.hover == Some(Pick::Edge(id, e)) || (wire && doc.hover == Some(Pick::Object(id)));
        let (col, w) = if sel {
            ([1.0, 0.75, 0.2, 1.0], 3.0)
        } else if hov {
            ([1.0, 0.9, 0.55, 1.0], 3.0)
        } else if wire {
            ([0.75, 0.82, 0.95, 1.0], 2.0)
        } else if show {
            ([0.05, 0.05, 0.06, 1.0], 1.4)
        } else {
            continue;
        };
        for w2 in poly.windows(2) {
            out.push(LineInstance::new(w2[0], w2[1], col, w));
        }
    }
    out
}

/// Seam edges (an edge used twice by the same face) are not drawn.
pub fn is_seam(body: &ductile_brep::Body, e: ductile_brep::EdgeId) -> bool {
    let uses = body.edge_faces(e);
    uses.len() == 2 && uses[0].0 == uses[1].0
}

fn vertex_overlay(doc: &Document, out: &mut Vec<LineInstance>) {
    let show_all = doc.mode == SelectMode::Vertex;
    for o in doc.objects.iter().filter(|o| o.visible) {
        for (vi, v) in o.body.vertices.iter().enumerate() {
            let p = Pick::Vertex(o.id, ductile_brep::VertexId(vi as u32));
            let (col, s) = if doc.selection.contains(&p) {
                ([1.0, 0.75, 0.2, 1.0], 9.0)
            } else if doc.hover == Some(p) {
                ([1.0, 0.9, 0.55, 1.0], 9.0)
            } else if show_all {
                ([0.85, 0.85, 0.9, 1.0], 5.0)
            } else {
                continue;
            };
            out.push(LineInstance::point(v.point, col, s));
        }
        // Curve control polygons, editable with Move in vertex mode.
        if show_all && o.body.kind == ductile_brep::BodyKind::Wire {
            let cps = ductile_brep::profile::control_points(&o.body);
            for w in cps.windows(2).filter(|w| w[0].0 == w[1].0) {
                out.push(LineInstance::new(w[0].2, w[1].2, [0.55, 0.6, 0.75, 0.8], 1.0));
            }
            for (e, i, p) in cps {
                let pk = Pick::Control(o.id, e, i);
                let (col, s) = if doc.selection.contains(&pk) {
                    ([1.0, 0.75, 0.2, 1.0], 9.0)
                } else if doc.hover == Some(pk) {
                    ([1.0, 0.9, 0.55, 1.0], 9.0)
                } else {
                    ([0.55, 0.75, 1.0, 1.0], 6.0)
                };
                out.push(LineInstance::point(p, col, s));
            }
        }
        // Surface control nets, likewise.
        if show_all && o.body.kind == ductile_brep::BodyKind::Sheet {
            let cps = ductile_brep::ops::sheet::surface_control_points(&o.body);
            for f in o.body.face_ids() {
                let ductile_geom::Surface::Nurbs(s) = &o.body.face(f).surface else { continue };
                let pts: Vec<DVec3> = cps.iter().filter(|c| c.0 == f).map(|c| c.2).collect();
                if pts.len() != s.nu * s.nv {
                    continue;
                }
                for i in 0..s.nu {
                    for j in 0..s.nv {
                        let p = pts[i * s.nv + j];
                        if j + 1 < s.nv {
                            out.push(LineInstance::new(p, pts[i * s.nv + j + 1], [0.55, 0.6, 0.75, 0.8], 1.0));
                        }
                        if i + 1 < s.nu {
                            out.push(LineInstance::new(p, pts[(i + 1) * s.nv + j], [0.55, 0.6, 0.75, 0.8], 1.0));
                        }
                    }
                }
            }
            for (f, i, p) in cps {
                let pk = Pick::SurfaceControl(o.id, f, i);
                let (col, s) = if doc.selection.contains(&pk) {
                    ([1.0, 0.75, 0.2, 1.0], 9.0)
                } else if doc.hover == Some(pk) {
                    ([1.0, 0.9, 0.55, 1.0], 9.0)
                } else {
                    ([0.55, 0.75, 1.0, 1.0], 6.0)
                };
                out.push(LineInstance::point(p, col, s));
            }
        }
    }
}

impl eframe::App for DuctileApp {
    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        self.handle_keys(&ctx);
        let cmds = std::mem::take(&mut self.pending);
        for c in cmds {
            commands::run(self, c);
        }
        if std::mem::take(&mut self.pending_finish) {
            self.finish_tool();
        }
        if std::mem::take(&mut self.pending_cancel) {
            self.cancel_tool();
        }
        crate::ui::top_bar(self, ui);
        crate::ui::status_bar(self, ui);
        crate::ui::toolbar(self, ui);
        crate::ui::side_panel(self, ui);
        egui::CentralPanel::default().frame(egui::Frame::NONE).show(ui, |ui| self.viewport(ui, frame));
        crate::ui::palette(self, &ctx);
        crate::ui::help_window(self, &ctx);
        let title = format!(
            "Ductile — {}{}",
            self.doc.path.as_ref().and_then(|p| p.file_name()).map(|s| s.to_string_lossy().to_string()).unwrap_or("Untitled".into()),
            if self.doc.dirty { " •" } else { "" }
        );
        ctx.send_viewport_cmd(egui::ViewportCommand::Title(title));
        self.handle_screenshot(&ctx);
        if self.tool.is_some() || self.screenshot.is_some() {
            ctx.request_repaint();
        }
    }
}

impl DuctileApp {
    fn handle_screenshot(&mut self, ctx: &egui::Context) {
        let Some((path, frames)) = self.screenshot.as_mut() else { return };
        *frames += 1;
        if *frames == 8 {
            let path = path.clone();
            let done = self.screenshot_done.clone();
            ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(egui::ScreenshotCallback::new(move |img| {
                match crate::screenshot::save_png(&path, &img) {
                    Ok(()) => eprintln!("saved screenshot to {}", path.display()),
                    Err(e) => eprintln!("screenshot failed: {e}"),
                }
                done.store(true, std::sync::atomic::Ordering::SeqCst);
            })));
        }
        if self.screenshot_done.load(std::sync::atomic::Ordering::SeqCst) || *frames > 300 {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
    }

    pub fn frame_selection(&mut self) {
        let mut bb = ductile_geom::BBox3::EMPTY;
        for id in self.doc.selected_objects() {
            if let Some(o) = self.doc.object(id) {
                bb = bb.union(&o.body.bbox());
            }
        }
        if bb.is_empty() {
            bb = self.doc.bbox();
        }
        if !bb.is_empty() {
            self.camera.frame(bb.center(), (bb.diagonal() * 0.5).max(0.5));
        }
    }

    pub fn xray(&self) -> bool {
        self.renderer.as_ref().map(|r| r.xray).unwrap_or(false)
    }

    pub fn zebra(&self) -> bool {
        self.renderer.as_ref().map(|r| r.zebra).unwrap_or(false)
    }

    pub fn toggle_zebra(&mut self) {
        if let Some(r) = self.renderer.as_mut() {
            r.zebra = !r.zebra;
        }
    }

    pub fn toggle_xray(&mut self) {
        if let Some(r) = self.renderer.as_mut() {
            r.xray = !r.xray;
        }
    }

    pub fn toggle_projection(&mut self) {
        self.camera.projection = match self.camera.projection {
            Projection::Perspective => Projection::Orthographic,
            Projection::Orthographic => Projection::Perspective,
        };
    }

    pub fn set_mode(&mut self, m: SelectMode) {
        if self.doc.mode != m {
            self.doc.mode = m;
            self.doc.selection.clear();
            self.doc.hover = None;
        }
    }

    pub fn replace_object_body(&mut self, id: ObjectId, body: ductile_brep::Body) {
        self.doc.replace_body(id, body);
    }

    pub fn arc_mesh(&self, id: ObjectId) -> Option<Arc<BodyMesh>> {
        self.doc.object(id).map(|o| o.mesh.clone())
    }
}
