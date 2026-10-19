// SPDX-License-Identifier: GPL-3.0-only
//! The application: layout, input handling and glue between the document,
//! tools and the viewport renderer.

use crate::camera::{Camera, Projection};
use crate::commands::{self, Command};
use crate::document::{mesh_body, Document, ObjectId, Pick, SelectMode};
use crate::picking;
use crate::render::{Analysis, LineInstance, ObjectStyle, ViewportRenderer};
use crate::theme;
use crate::tools::{self, CPlane, PointerInfo, Tool, ToolOutcome, TransformKind, TransformTool};
use ductile_brep::{Body, BodyMesh};
use glam::{DVec2, DVec3};
use std::collections::HashMap;
use std::sync::Arc;

const PREVIEW_ID: u64 = u64::MAX;
/// Renderer ids of curves' region fills: the curve's id with this bit set.
const REGION_BIT: u64 = 1 << 62;

/// The fill of a curve object's closed planar loops, for display.
fn region_mesh(body: &ductile_brep::Body) -> Option<BodyMesh> {
    use ductile_brep::profile::{loops_plane, wire_chains, Region};
    let loops: Vec<_> = wire_chains(body).into_iter().filter(|l| l.is_closed()).collect();
    if loops.is_empty() {
        return None;
    }
    let plane = loops_plane(&loops)?;
    let mut faces = Vec::new();
    for r in Region::from_loops(plane, loops) {
        let sheet = ductile_brep::ops::sheet::patch(&r.outer, &r.holes)?;
        faces.extend(crate::document::mesh_body(&sheet).faces);
    }
    (!faces.is_empty()).then(|| BodyMesh { faces, ..Default::default() })
}

pub struct Options {
    pub screenshot: Option<std::path::PathBuf>,
    pub open: Option<std::path::PathBuf>,
    pub demo: bool,
    pub analysis: Option<Analysis>,
    pub look: Option<crate::render::Look>,
    pub curvature: bool,
    pub section: bool,
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
    /// Revisions of the curves whose region fills are uploaded (None: no fill).
    regions: HashMap<ObjectId, (u64, bool)>,
    style_key: u64,
    pub tool: Option<Box<dyn Tool>>,
    tool_from_key: bool,
    preview_shown: bool,
    /// The body last uploaded as the preview, to skip re-meshing it.
    preview_body: Option<Body>,
    pub status: String,
    box_select: Option<BoxSelect>,
    /// The typed-value field, open while Some (Tab or a digit opens it).
    typed: Option<String>,
    /// A character already taken from its key event, so its text event is skipped.
    typed_echo: Option<char>,
    pub show_grid: bool,
    pub show_edges: bool,
    pub show_curves: bool,
    pub show_faces: bool,
    /// Curvature combs on the selected curves and edges.
    pub show_curvature: bool,
    /// Section analysis: a plane through a point, with its normal, and an
    /// offset along it; the scene beyond it is cut away.
    pub section: Option<(DVec3, DVec3, f64)>,
    /// Length of the longer comb teeth, as a fraction of the curves' size.
    pub comb_scale: f64,
    /// The last measurement, shown while the selection it measured stays.
    pub measurement: Option<(crate::measure::Measurement, std::collections::BTreeSet<Pick>)>,
    pub palette_open: bool,
    /// Where command search opened (the mouse position).
    pub palette_at: Option<egui::Pos2>,
    /// Where the Shift+F menu of suggested commands is open.
    pub suggest_at: Option<egui::Pos2>,
    /// Quit was asked for; the window closes on the next frame.
    pub quit_requested: bool,
    /// Points drawn on the construction plane round to the grid.
    pub grid_snap: bool,
    /// Grid spacing; 0 follows the zoom.
    pub grid_step: f64,
    /// Every how many lines a major line falls.
    pub grid_major: u32,
    /// Lines drawn each way from the centre.
    pub grid_count: u32,
    pub snap_kinds: crate::snap::Kinds,
    /// Whether this run set the running marker (not headless screenshots).
    session_marked: bool,
    /// The pointer over the viewport, for menus that open at the cursor.
    pub pointer_pos: Option<egui::Pos2>,
    pub palette_query: String,
    pub show_help: bool,
    pub show_prefs: bool,
    pub show_left: bool,
    pub show_right: bool,
    /// The left panel's tab: 0 Outliner, 1 Assets.
    pub left_tab: u8,
    pub outliner_filter: String,
    /// The status message last shown and since when, so it fades.
    status_seen: (String, std::time::Instant),
    pub prefs: crate::prefs::Prefs,
    /// Set while the running tool was started by selecting a face or edge
    /// (auto-invoke): whether a drag is driving it right now. Such a tool
    /// only follows the pointer while dragged, and a click before any
    /// change goes back to selecting.
    auto_tool: Option<bool>,
    screenshot: Option<(std::path::PathBuf, u32)>,
    pub viewport_rect: egui::Rect,
    last_cursor: Option<DVec2>,
    pub pending: Vec<Command>,
    /// The last modeling command run, for Shift+R.
    pub last_command: Option<Command>,
    /// When the document was last autosaved.
    last_autosave: std::time::Instant,
    /// The command whose shortcut the next key press sets.
    pub assigning: Option<Command>,
    pub snap_enabled: bool,
    snap_marker: Option<(DVec3, Option<&'static str>)>,
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
            regions: HashMap::new(),
            style_key: 0,
            tool: None,
            tool_from_key: false,
            gizmo_hover: None,
            gizmo_drag: false,
            preview_shown: false,
            preview_body: None,
            status: "Welcome to Ductile. Press F for the command palette, F1 for help.".into(),
            box_select: None,
            typed: None,
            typed_echo: None,
            show_grid: true,
            show_edges: true,
            show_curves: true,
            show_faces: true,
            show_curvature: false,
            comb_scale: 0.1,
            section: None,
            measurement: None,
            palette_open: false,
            palette_at: None,
            suggest_at: None,
            quit_requested: false,
            grid_snap: true,
            grid_step: 0.0,
            grid_major: 5,
            grid_count: 30,
            snap_kinds: Default::default(),
            session_marked: false,
            pointer_pos: None,
            palette_query: String::new(),
            show_help: false,
            show_prefs: false,
            show_left: true,
            show_right: true,
            left_tab: 0,
            outliner_filter: String::new(),
            status_seen: (String::new(), std::time::Instant::now()),
            prefs: if opts.screenshot.is_some() { crate::prefs::Prefs::default() } else { crate::prefs::Prefs::load() },
            auto_tool: None,
            screenshot: opts.screenshot.map(|p| (p, 0)),
            viewport_rect: egui::Rect::NOTHING,
            last_cursor: None,
            pending: vec![],
            last_command: None,
            last_autosave: std::time::Instant::now(),
            assigning: None,
            snap_enabled: true,
            snap_marker: None,
            screenshot_done: Default::default(),
            pending_finish: false,
            pending_cancel: false,
        };
        let last = crate::recent::autosave_path().filter(|p| p.exists());
        app.session_marked = app.screenshot.is_none();
        if app.session_marked && crate::backup::start_session() && last.is_some() {
            app.status = "Ductile did not close cleanly last time; File > Recover autosave brings back the last autosave".into();
        }
        if let Some(path) = opts.open {
            commands::open_file(&mut app, &path);
        } else if let (Some(p), true, false) = (last, app.prefs.load_last_session, opts.demo) {
            commands::open_file(&mut app, &p);
            app.doc.path = None;
        } else if let (Some(p), false, true) = (commands::startup_scene_path().filter(|p| p.exists()), opts.demo, app.session_marked) {
            if commands::load_into(&mut app.doc, &p).is_ok() {
                app.doc.path = None;
                app.doc.dirty = false;
            }
        } else if opts.demo {
            commands::demo_scene(&mut app.doc);
            let bb = app.doc.bbox();
            app.camera.frame(bb.center(), bb.diagonal() * 0.5);
        }
        if let Some(a) = opts.analysis {
            app.toggle_analysis(a);
        }
        if let Some(l) = opts.look {
            app.set_look(l);
        }
        app.show_curvature = opts.curvature;
        if opts.section {
            app.toggle_section();
        }
        // Screenshots show the model unselected unless --select names one.
        if app.screenshot.is_some() {
            app.doc.selection.clear();
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

    /// The aligned view the camera looks along, if any (Top, Front, …).
    pub fn named_view(&self) -> Option<&'static str> {
        let b = self.camera.back();
        [(DVec3::Z, "Top"), (-DVec3::Z, "Bottom"), (-DVec3::Y, "Front"), (DVec3::Y, "Back"), (DVec3::X, "Right"), (-DVec3::X, "Left")].into_iter().find(|(d, _)| b.dot(*d) > 1.0 - 1e-6).map(|(_, n)| n)
    }

    /// The status message while it is fresh (a few seconds after it changed).
    pub fn recent_status(&self) -> Option<&str> {
        (!self.status.is_empty() && self.status_seen.0 == self.status && self.status_seen.1.elapsed() < std::time::Duration::from_secs(5)).then_some(self.status.as_str())
    }

    pub fn start_tool(&mut self, t: Box<dyn Tool>) {
        self.tool = Some(t);
        self.auto_tool = None;
        self.typed = None;
        self.tool_from_key = false;
    }

    pub fn cancel_tool(&mut self) {
        self.tool = None;
        self.auto_tool = None;
        self.typed = None;
    }

    fn finish_tool(&mut self) {
        self.auto_tool = None;
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
        self.typed = None;
    }

    // ----------------------------------------------------------- frame ----

    fn sync_renderer(&mut self) {
        let Some(r) = self.renderer.as_mut() else { return };
        let images: Vec<crate::render::ImageDraw> = self
            .doc
            .images
            .iter()
            .filter(|i| i.visible)
            .map(|i| crate::render::ImageDraw { id: i.id, pixels: i.pixels.clone(), size: i.size, corners: i.corners(), opacity: i.opacity })
            .collect();
        r.set_images(&images);
        let ids: std::collections::HashSet<ObjectId> = self.doc.objects.iter().map(|o| o.id).collect();
        self.uploaded.retain(|id, _| ids.contains(id));
        r.retain_objects(|id| ids.contains(&(id & !REGION_BIT)) || id == PREVIEW_ID);
        self.regions.retain(|id, _| ids.contains(id));
        let mut changed = false;
        for o in &self.doc.objects {
            if self.uploaded.get(&o.id) != Some(&o.revision) {
                r.upload_object(o.id, &o.mesh);
                self.uploaded.insert(o.id, o.revision);
                changed = true;
            }
            if o.body.kind == ductile_brep::BodyKind::Wire && self.regions.get(&o.id).map(|x| x.0) != Some(o.revision) {
                let fill = region_mesh(&o.body);
                match &fill {
                    Some(m) => r.upload_object(o.id | REGION_BIT, m),
                    None => r.remove_object(o.id | REGION_BIT),
                }
                self.regions.insert(o.id, (o.revision, fill.is_some()));
                changed = true;
            }
        }
        self.doc.hide_curves = !self.show_curves;
        crate::units::set(self.prefs.unit);
        // Styles depend on selection/hover; recompute when they change.
        let hidden: Vec<ObjectId> = self.tool.as_ref().map(|t| t.hidden_objects()).unwrap_or_default();
        let key = {
            use std::hash::{Hash, Hasher};
            let mut h = std::collections::hash_map::DefaultHasher::new();
            self.doc.selection.hash(&mut h);
            self.doc.hover.hash(&mut h);
            hidden.hash(&mut h);
            self.show_edges.hash(&mut h);
            self.show_curves.hash(&mut h);
            self.show_faces.hash(&mut h);
            (r.analysis == Analysis::Render).hash(&mut h);
            for o in &self.doc.objects {
                o.visible.hash(&mut h);
                o.revision.hash(&mut h);
                o.color.map(|c| c.to_bits()).hash(&mut h);
                [o.metalness, o.roughness].map(|c| c.to_bits()).hash(&mut h);
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
            st.visible = o.visible && !hidden.contains(&o.id) && (self.show_curves || o.body.kind != ductile_brep::BodyKind::Wire);
            st.faces = self.show_faces;
            st.solid = o.body.kind == ductile_brep::BodyKind::Solid;
            (st.metalness, st.roughness) = (o.metalness, o.roughness);
            if self.doc.selection.contains(&Pick::Object(o.id)) {
                st.state = 2;
            } else if self.doc.hover == Some(Pick::Object(o.id)) {
                st.state = 1;
            }
            if self.regions.get(&o.id).is_some_and(|x| x.1) {
                let fill = ObjectStyle { color: [0.45, 0.62, 0.95, 0.16], hover_color: [0.55, 0.72, 1.0, 1.0], select_color: [0.95, 0.75, 0.35, 1.0], flat: true, ..st.clone() };
                r.set_style(o.id | REGION_BIT, &fill, &[]);
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
            // The rendered view shows surfaces only, as a finished part.
            let edges = self.show_edges && r.analysis != Analysis::Render;
            let lines = edge_lines(&o.mesh, &o.body, o.id, &self.doc, edges);
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
        let step = if self.grid_step > 0.0 { self.grid_step } else { tools::nice_step(wpp * 40.0) };
        let major = step * self.grid_major.max(1) as f64;
        let n = self.grid_count.max(1) as i64;
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
        let minor_col = [0.205, 0.207, 0.215, 1.0];
        let major_col = [0.235, 0.237, 0.246, 1.0];
        for i in -n..=n {
            let x = cx + step * i as f64;
            let y = cy + step * i as f64;
            let is_major_x = ((x / major).round() * major - x).abs() < step * 0.01;
            let is_major_y = ((y / major).round() * major - y).abs() < step * 0.01;
            // The axes are drawn on their own below.
            if x.abs() >= step * 0.01 {
                push(f.to_world(DVec3::new(x, cy - ext, 0.0)), f.to_world(DVec3::new(x, cy + ext, 0.0)), if is_major_x { major_col } else { minor_col });
            }
            if y.abs() >= step * 0.01 {
                push(f.to_world(DVec3::new(cx - ext, y, 0.0)), f.to_world(DVec3::new(cx + ext, y, 0.0)), if is_major_y { major_col } else { minor_col });
            }
        }
        // The plane's axes run right across the view, as in Plasticity.
        let far = ext * 6.0;
        let axis = |out: &mut Vec<LineInstance>, d: DVec3, col: [f32; 4]| {
            let k = 48;
            for i in 0..k {
                let (a, b) = (-far + 2.0 * far * i as f64 / k as f64, -far + 2.0 * far * (i + 1) as f64 / k as f64);
                out.push(LineInstance { style: [1.2, 0.0, -1.0, 0.0], ..LineInstance::new(f.to_world(d * a), f.to_world(d * b), col, 1.2) });
            }
        };
        axis(out, DVec3::X, [0.62, 0.16, 0.17, 1.0]);
        axis(out, DVec3::Y, [0.18, 0.52, 0.20, 1.0]);
        axis(out, DVec3::Z, [0.20, 0.36, 0.72, 1.0]);
    }

    /// Snapped pointer position (with the snap's label): object snaps,
    /// then the grid.
    fn snap(&self, cursor: DVec2, size: DVec2, ray: &crate::camera::Ray, ctrl: bool) -> (Option<DVec3>, Option<DVec3>, Option<&'static str>) {
        let f = self.cplane.frame;
        let plane_pt = ray.hit_plane(f.origin, f.z);
        // Ctrl suspends snapping while it is held.
        if self.snap_enabled && !ctrl {
            let anchor = self.tool.as_ref().and_then(|t| t.anchor());
            if let Some(s) = crate::snap::find(&self.doc, &self.camera, &f, cursor, size, ray, anchor, &self.snap_kinds) {
                return (Some(s.point), plane_pt, Some(s.label));
            }
        }
        let grid = plane_pt.map(|p| {
            if self.grid_snap && !ctrl {
                let wpp = self.camera.world_per_pixel(p, size.y);
                let step = if self.grid_step > 0.0 { self.grid_step } else { tools::nice_step(wpp * 40.0) / 4.0 };
                let l = f.to_local(p);
                f.to_world(DVec3::new((l.x / step).round() * step, (l.y / step).round() * step, 0.0))
            } else {
                p
            }
        });
        (grid, plane_pt, None)
    }

    fn viewport(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        let rect = ui.available_rect_before_wrap();
        self.viewport_rect = rect;
        let resp = ui.allocate_rect(rect, egui::Sense::click_and_drag());
        let size = DVec2::new(rect.width() as f64, rect.height() as f64);
        self.camera.aspect = (size.x / size.y.max(1.0)).max(0.01);
        let (mods, scroll, pointer) = ui.input(|i| (i.modifiers, i.smooth_scroll_delta.y, i.pointer.clone()));

        // Navigation.
        let nav = self.prefs.nav;
        let held = |b| pointer.button_down(b);
        let nav_now = nav.drag(held(egui::PointerButton::Primary), held(egui::PointerButton::Middle), held(egui::PointerButton::Secondary), mods);
        let nav_drag = nav_now.is_some();
        if resp.dragged() {
            let d = resp.drag_delta();
            match nav_now {
                Some(crate::prefs::Nav::Pan) => self.camera.pan(d.x as f64, d.y as f64, size.y),
                Some(crate::prefs::Nav::Zoom) => self.camera.zoom((1.0 + d.y as f64 * 0.01).max(0.1), None),
                Some(crate::prefs::Nav::Orbit) => self.camera.orbit(d.x as f64, d.y as f64),
                None => {}
            }
        }
        // A left drag the camera takes is not a click, box or tool drag.
        let left_nav = nav.takes_left(mods) && held(egui::PointerButton::Primary);
        let hover_pos = resp.hover_pos();
        self.pointer_pos = hover_pos;
        let cursor = hover_pos.map(|p| DVec2::new((p.x - rect.min.x) as f64, (p.y - rect.min.y) as f64));
        if resp.hovered() && scroll.abs() > 0.0 {
            let toward = cursor.filter(|_| self.prefs.zoom_to_cursor).and_then(|c| {
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
            let clicked = resp.clicked_by(egui::PointerButton::Primary) && !left_nav;
            // An auto-invoked tool follows the pointer only while dragged.
            let passive = match self.auto_tool {
                Some(_) if self.tool.is_some() => {
                    let dragging = resp.dragged_by(egui::PointerButton::Primary) && !left_nav;
                    self.auto_tool = Some(dragging);
                    !dragging
                }
                _ => false,
            };
            if passive {
                if moved && !nav_drag {
                    self.doc.hover = self.drill_pick(c, size);
                }
                if clicked {
                    let touched = self.tool.as_ref().is_some_and(|t| t.preview(&self.doc).is_some());
                    if touched {
                        self.finish_tool();
                    } else {
                        self.cancel_tool();
                        self.click_select(c, size, mods);
                        self.auto_invoke();
                    }
                }
            } else if self.tool.is_some() {
                let wants = self.tool.as_ref().unwrap().wants_selection();
                if let Some(mode) = wants {
                    // Tools that pick entities first behave like selection.
                    if moved {
                        self.doc.hover = picking::pick(&self.doc, &self.camera, c, size, mode);
                    }
                }
                let (pt, plane_pt, label) = self.snap(c, size, &ray, mods.ctrl);
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
                    // A command waiting for its input starts on the pick.
                    if let Some(cmd) = self.tool.as_ref().and_then(|t| t.then_run()).filter(|_| !self.doc.selection.is_empty()) {
                        let waiting = self.tool.take();
                        commands::run(self, cmd);
                        if self.tool.is_none() {
                            self.tool = waiting;
                        }
                    }
                }
                if wants == Some(SelectMode::Edge) && resp.double_clicked() {
                    self.chain_select(c, size);
                }
                match out {
                    ToolOutcome::Finished => self.finish_tool(),
                    ToolOutcome::Cancelled => self.cancel_tool(),
                    ToolOutcome::Continue => {}
                }
                if let Some(p) = pt {
                    self.snap_marker = Some((p, label));
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
                    self.doc.hover = self.drill_pick(c, size);
                }
                if clicked && self.gizmo_hover.is_none() {
                    self.click_select(c, size, mods);
                    self.auto_invoke();
                }
                if resp.double_clicked() && self.gizmo_hover.is_none() {
                    self.chain_select(c, size);
                }
                if let (Some((kind, axis)), true) = (self.gizmo_hover, resp.drag_started_by(egui::PointerButton::Primary)) {
                    let mut t = TransformTool::new(kind, &self.doc);
                    t.axis = Some(axis);
                    self.start_tool(Box::new(t));
                    self.gizmo_drag = true;
                    self.gizmo_hover = None;
                } else if resp.drag_started_by(egui::PointerButton::Primary) && !left_nav {
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
                        // Right to left: anything the box touches; left to right: only what it holds.
                        let crossing = b.x < a.x;
                        let picks = picking::pick_box_in(&self.doc, &self.camera, a, b, size, &self.doc.modes(), crossing);
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
        let mut labels: Vec<(DVec3, String)> = Vec::new();
        if let Some((p, label)) = self.snap_marker.take().filter(|_| self.tool.is_some()) {
            overlay.push(LineInstance::point(p, [0.3, 0.9, 1.0, 1.0], 6.0));
            let tag = self.tool.as_ref().and_then(|t| t.inference());
            let text = match (label, tag) {
                (Some(l), Some(t)) => Some(format!("{l},{t}")),
                (l, t) => l.map(str::to_string).or(t),
            };
            labels.extend(text.map(|l| (p, l)));
        }
        if let Some(t) = &self.tool {
            labels.extend(t.labels());
        }
        if self.show_curvature {
            self.curvature_overlay(&mut overlay, &mut labels);
        }
        if self.measurement.as_ref().is_some_and(|(_, sel)| *sel != self.doc.selection) {
            self.measurement = None;
        }
        if let Some((m, _)) = &self.measurement {
            match m.span {
                Some((p, q)) => {
                    let c = [0.45, 0.95, 0.6, 1.0];
                    overlay.push(LineInstance::new(p, q, c, 2.0));
                    overlay.push(LineInstance::point(p, c, 6.0));
                    overlay.push(LineInstance::point(q, c, 6.0));
                    labels.push(((p + q) * 0.5, m.text.clone()));
                }
                None => {
                    if let Some(p) = self.doc.selection.iter().next().and_then(|p| self.pick_point(p)) {
                        labels.push((p, m.text.clone()));
                    }
                }
            }
        }
        for d in &self.doc.dimensions {
            crate::tools::dimension_lines(d, [0.85, 0.85, 0.9, 1.0], &mut overlay);
            labels.push(crate::tools::dimension_label(d));
        }
        vertex_overlay(&self.doc, &mut overlay);
        let ppp = ui.ctx().pixels_per_point();
        let px = [(rect.width() * ppp).round() as u32, (rect.height() * ppp).round() as u32];
        if let (Some(r), Some(rs)) = (self.renderer.as_mut(), frame.wgpu_render_state()) {
            let mut egui_renderer = rs.renderer.write();
            r.section = self.section.map(|(o, n, d)| {
                let n = n.normalize();
                [n.x as f32, n.y as f32, n.z as f32, -(n.dot(o) + d) as f32]
            });
            let tex = r.render(&mut egui_renderer, &self.camera, px, &lines, &overlay);
            drop(egui_renderer);
            ui.painter().image(tex, rect, egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)), egui::Color32::WHITE);
        } else {
            ui.painter().rect_filled(rect, 0.0, egui::Color32::from_gray(20));
            ui.painter().text(rect.center(), egui::Align2::CENTER_CENTER, "wgpu renderer unavailable", egui::FontId::proportional(16.0), egui::Color32::GRAY);
        }
        // Labels beside their points: the snap's, continuity markers.
        for (p, label) in labels {
            if let Some(n) = self.camera.project(p) {
                let at = egui::pos2(rect.min.x + (n.x as f32 * 0.5 + 0.5) * rect.width() + 10.0, rect.min.y + (0.5 - n.y as f32 * 0.5) * rect.height() - 10.0);
                let painter = ui.painter();
                let galley = painter.layout_no_wrap(label, egui::FontId::proportional(12.0), egui::Color32::from_gray(235));
                let bg = egui::Rect::from_min_size(at, galley.size()).expand(3.0);
                painter.rect_filled(bg, 3.0, egui::Color32::from_black_alpha(190));
                painter.galley(at, galley, egui::Color32::WHITE);
            }
        }
        if let Some(bs) = &self.box_select {
            let r = egui::Rect::from_two_pos(bs.start, bs.current);
            // Crossing boxes green, window boxes purple, as in Plasticity.
            let (r8, g8, b8) = if bs.current.x < bs.start.x { (110, 210, 130) } else { (160, 120, 255) };
            ui.painter().rect_filled(r, 0.0, egui::Color32::from_rgba_unmultiplied(r8, g8, b8, 25));
            ui.painter().rect_stroke(r, 0.0, egui::Stroke::new(1.0, egui::Color32::from_rgb(r8, g8, b8)), egui::StrokeKind::Inside);
        }
        if let Some(h) = &gizmo {
            if self.tool.is_none() {
                self.draw_gizmo(ui, rect, h);
            }
        }
        self.view_gizmo(ui, rect);
        if let Some(t) = &self.typed {
            let length = self.tool.as_ref().is_some_and(|t| t.typed_length());
            let shown = match crate::expr::eval(t) {
                Some(_) if length && t.parse::<f64>().is_err() => format!("= {t}  ({})", crate::units::len(crate::expr::eval_length(t).unwrap_or(0.0))),
                Some(v) if t.parse::<f64>().is_err() => format!("= {t}  ({})", fmt_value(v)),
                None if !t.is_empty() => format!("= {t}  ?"),
                _ => format!("= {t}"),
            };
            ui.painter().text(
                rect.left_bottom() + egui::vec2(12.0, -12.0),
                egui::Align2::LEFT_BOTTOM,
                format!("{shown}\u{258f}"),
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

    /// What a click in the viewport picks with no tool running: in object
    /// mode, a second click on the solid already selected (drill-down)
    /// takes the edge or face under the cursor.
    fn drill_pick(&self, c: DVec2, size: DVec2) -> Option<Pick> {
        let base = picking::pick_in(&self.doc, &self.camera, c, size, &self.doc.modes());
        if !self.doc.mode_on(SelectMode::Object) || !self.prefs.drill_down {
            return base;
        }
        let Some(Pick::Object(o)) = base else { return base };
        let solid = self.doc.object(o).is_some_and(|x| x.body.kind != ductile_brep::BodyKind::Wire && x.body.mesh.is_none());
        let focused = !self.doc.selection.is_empty() && self.doc.selection.iter().all(|p| p.object() == o);
        if !solid || !focused {
            return base;
        }
        let mine = |p: Option<Pick>| p.filter(|p| p.object() == o);
        mine(picking::pick(&self.doc, &self.camera, c, size, SelectMode::Edge)).or_else(|| mine(picking::pick(&self.doc, &self.camera, c, size, SelectMode::Face))).or(base)
    }

    /// Auto-invoke (Preferences): selected faces of one solid start Push
    /// face, selected edges Fillet. The tool waits for a drag or a typed
    /// value; a click before that goes back to selecting.
    fn auto_invoke(&mut self) {
        if !self.prefs.auto_invoke || self.tool.is_some() || self.doc.selection.is_empty() {
            return;
        }
        let o = self.doc.selection.iter().next().unwrap().object();
        if !self.doc.object(o).is_some_and(|x| x.body.kind == ductile_brep::BodyKind::Solid) || self.doc.selection.iter().any(|p| p.object() != o) {
            return;
        }
        let cmd = if self.doc.selection.iter().all(|p| matches!(p, Pick::Face(..))) {
            commands::Command::Extrude
        } else if self.doc.selection.iter().all(|p| matches!(p, Pick::Edge(..))) {
            commands::Command::Fillet
        } else {
            return;
        };
        let status = self.status.clone();
        commands::run(self, cmd);
        if self.tool.is_some() {
            self.auto_tool = Some(false);
        } else {
            self.status = status;
        }
    }

    fn click_select(&mut self, c: DVec2, size: DVec2, mods: egui::Modifiers) {
        let modes = self.tool.as_ref().and_then(|t| t.wants_selection()).map_or_else(|| self.doc.modes(), |m| vec![m]);
        let hit = if self.tool.is_none() && !mods.alt { self.drill_pick(c, size) } else { picking::pick_in(&self.doc, &self.camera, c, size, &modes) };
        if mods.alt {
            // Alt: the edge's loop or the face's tangent run; with Ctrl the
            // edge's ring or the faces parallel to it.
            let view = (self.camera.eye() - self.camera.target).normalize();
            let grown: Vec<Pick> = match hit {
                Some(Pick::Edge(o, e)) => self.doc.object(o).map_or(vec![], |ob| {
                    let es = if mods.ctrl { crate::select::edge_ring(&ob.body, e) } else { crate::select::edge_loop(&ob.body, e, view) };
                    es.into_iter().map(|x| Pick::Edge(o, x)).collect()
                }),
                Some(Pick::Face(o, f)) => self.doc.object(o).map_or(vec![], |ob| {
                    let fs = if mods.ctrl { crate::select::parallel_faces(&ob.body, f) } else { crate::select::tangent_faces(&ob.body, f) };
                    fs.into_iter().map(|x| Pick::Face(o, x)).collect()
                }),
                // A grouped object: everything in its group.
                Some(Pick::Object(o)) => match self.doc.object(o).and_then(|x| x.group) {
                    Some(g) => self.doc.group_members(g).into_iter().filter(|id| !self.doc.is_locked(*id) && self.doc.object(*id).is_some_and(|x| x.visible)).map(Pick::Object).collect(),
                    None => vec![Pick::Object(o)],
                },
                Some(p) => vec![p],
                None => vec![],
            };
            if !grown.is_empty() {
                if !mods.shift && self.tool.is_none() {
                    self.doc.selection.clear();
                }
                self.doc.selection.extend(grown);
            }
            return;
        }
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

    /// Double-click on an edge: adds the edges running on tangentially from
    /// it (a filleted outline, a spline profile's edges) to the selection.
    fn chain_select(&mut self, c: DVec2, size: DVec2) {
        let modes = self.tool.as_ref().and_then(|t| t.wants_selection()).map_or_else(|| self.doc.modes(), |m| vec![m]);
        if let Some(Pick::Edge(o, e)) = picking::pick_in(&self.doc, &self.camera, c, size, &modes) {
            let Some(ob) = self.doc.object(o) else { return };
            for x in ductile_brep::ops::chain::grow_tangent_chain(&ob.body, e, &|_| true) {
                self.doc.selection.insert(Pick::Edge(o, x));
            }
        }
    }

    /// Small orientation widget in the viewport corner; click an axis to view along it.
    fn view_gizmo(&mut self, ui: &mut egui::Ui, rect: egui::Rect) {
        let center = rect.right_top() + egui::vec2(-60.0, 110.0);
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

    /// Curvature combs on the selected curves and edges (all of a selected
    /// object's), and G0 / G1 / G2 where two of them meet.
    fn curvature_overlay(&self, out: &mut Vec<LineInstance>, labels: &mut Vec<(DVec3, String)>) {
        // Picked edges and whole curves are checked for continuity where
        // they meet; a selected solid's edges only get combs.
        let mut edges: Vec<(&ductile_brep::topo::Edge, bool)> = self.doc.selected_edges().into_iter().filter_map(|(o, e)| Some((self.doc.object(o)?.body.edge(e), true))).collect();
        for id in self.doc.selected_objects() {
            if let Some(ob) = self.doc.object(id) {
                let wire = ob.body.kind == ductile_brep::BodyKind::Wire;
                edges.extend((0..ob.body.edges.len() as u32).map(ductile_brep::EdgeId).filter(|e| !is_seam(&ob.body, *e)).map(|e| (ob.body.edge(e), wire)));
            }
        }
        if edges.is_empty() {
            return;
        }
        let mut bb = ductile_geom::BBox3::EMPTY;
        for (e, _) in &edges {
            bb = bb.union(&e.curve.bbox(e.t0, e.t1));
        }
        let size = bb.diagonal().max(1e-9);
        let combs: Vec<_> = edges.iter().map(|(e, _)| crate::analysis::comb(&e.curve, e.t0, e.t1, 1.0)).collect();
        // Scaled so the longer teeth come to `comb_scale` of the curves'
        // size, the same scale for all of them so joins compare.
        let mut k: Vec<f64> = combs.iter().flat_map(|(teeth, _)| teeth.iter().map(|(p, q)| p.distance(*q))).collect();
        k.sort_by(f64::total_cmp);
        let high = k[(k.len() * 9 / 10).min(k.len() - 1)];
        if high < 1e-12 {
            return;
        }
        let scale = self.comb_scale * size / high;
        let (tooth, hull) = ([0.95, 0.35, 0.55, 0.55], [0.95, 0.45, 0.65, 0.95]);
        for (teeth, _) in &combs {
            let tips: Vec<DVec3> = teeth.iter().map(|(p, q)| *p + (*q - *p) * scale).collect();
            out.extend(teeth.iter().zip(&tips).map(|((p, _), q)| LineInstance::new(*p, *q, tooth, 1.0)));
            out.extend(tips.windows(2).map(|w| LineInstance::new(w[0], w[1], hull, 1.5)));
        }
        let tol = 1e-6 * (1.0 + size);
        let joined: Vec<&ductile_brep::topo::Edge> = edges.iter().filter(|(_, j)| *j).map(|(e, _)| *e).collect();
        for (i, a) in joined.iter().enumerate() {
            for b in &joined[i + 1..] {
                for ta in [a.t0, a.t1] {
                    for tb in [b.t0, b.t1] {
                        if let Some(g) = crate::analysis::continuity(&a.curve, ta, &b.curve, tb, tol) {
                            labels.push((a.curve.point(ta), g.to_string()));
                        }
                    }
                }
            }
        }
    }

    /// A point to label a pick at.
    fn pick_point(&self, p: &Pick) -> Option<DVec3> {
        let ob = self.doc.object(p.object())?;
        Some(match *p {
            Pick::Vertex(_, v) => ob.body.vertex(v),
            Pick::Edge(_, e) => ob.body.edge(e).mid(),
            Pick::Face(_, f) => {
                let m = ob.mesh.faces.get(f.idx())?;
                m.positions.iter().copied().sum::<DVec3>() / m.positions.len().max(1) as f64
            }
            _ => ob.body.bbox().center(),
        })
    }

    /// Hands the typed field's value to the tool (None while it doesn't parse).
    fn apply_typed(&mut self) {
        let length = self.tool.as_ref().is_some_and(|t| t.typed_length());
        let eval = if length { crate::expr::eval_length } else { crate::expr::eval };
        let v = self.typed.as_deref().and_then(eval);
        if let Some(t) = self.tool.as_mut() {
            t.typed_value(v);
        }
    }

    fn handle_keys(&mut self, ctx: &egui::Context) {
        if ctx.egui_wants_keyboard_input() {
            return;
        }
        let events = ctx.input(|i| i.events.clone());
        for ev in events {
            if std::env::var_os("DUCTILE_DEBUG_KEYS").is_some() {
                eprintln!("{ev:?}");
            }
            if let egui::Event::Text(text) = &ev {
                if let (Some(t), true) = (self.typed.as_mut(), self.tool.is_some()) {
                    let mut chars = text.chars();
                    if let (Some(c), Some(e)) = (chars.clone().next(), self.typed_echo.take()) {
                        if c == e {
                            chars.next();
                        }
                    }
                    t.extend(chars);
                    self.apply_typed();
                }
                continue;
            }
            let egui::Event::Key { key, pressed: true, modifiers, .. } = ev else { continue };
            if let Some(cmd) = self.assigning.take() {
                self.status = if key == egui::Key::Escape {
                    "Shortcut unchanged".into()
                } else {
                    let chord = crate::keymap::Chord::new(key, modifiers);
                    match crate::keymap::keymap_mut().assign(cmd, chord) {
                        Some(old) => format!("{} is now {} (taken from {})", chord.text(), cmd.label(), old.label()),
                        None => format!("{} is now {}", chord.text(), cmd.label()),
                    }
                };
                continue;
            }
            if key == egui::Key::Escape && self.suggest_at.take().is_some() {
                continue;
            }
            // Modal typing for active tools.
            if self.tool.is_some() && self.typed.is_some() {
                match key {
                    egui::Key::Backspace => {
                        self.typed.as_mut().unwrap().pop();
                        self.apply_typed();
                    }
                    egui::Key::Escape => {
                        // Back to the pointer's value; Escape again cancels.
                        self.typed = None;
                        self.tool.as_mut().unwrap().typed_value(None);
                    }
                    egui::Key::Tab => self.typed = None,
                    egui::Key::Enter => {
                        self.typed = None;
                        match self.tool.as_mut().unwrap().accept() {
                            ToolOutcome::Finished => self.finish_tool(),
                            ToolOutcome::Cancelled => self.cancel_tool(),
                            ToolOutcome::Continue => {}
                        }
                    }
                    // Characters arrive as text.
                    _ => {}
                }
                continue;
            }
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
                if key == egui::Key::Tab {
                    // Tools with options to cycle take Tab; others open the typed field.
                    if self.tool.as_mut().unwrap().command_key(key, modifiers) {
                        continue;
                    }
                    self.typed = Some(String::new());
                    continue;
                }
                if self.auto_tool.is_some() && self.typed.is_none() {
                    // An auto-invoked tool leaves the digit keys to selection
                    // modes; D opens its value for typing.
                    if key == egui::Key::D && !modifiers.any() {
                        self.auto_tool = None;
                        self.typed = Some(String::new());
                        continue;
                    }
                    if digit.is_some() {
                        self.cancel_tool();
                        if let Some(cmd) = commands::shortcut(key, modifiers) {
                            self.pending.push(cmd);
                        }
                        continue;
                    }
                }
                if let (Some(d), false) = (digit, modifiers.ctrl || modifiers.alt) {
                    self.typed = Some(d.to_string());
                    self.typed_echo = Some(d);
                    self.apply_typed();
                    continue;
                }
                if self.tool.as_mut().unwrap().command_key(key, modifiers) {
                    continue;
                }
                match key {
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
    let show_all = doc.mode_on(SelectMode::Vertex);
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
    fn on_exit(&mut self) {
        if self.session_marked {
            crate::backup::end_session();
        }
    }
    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        self.handle_keys(&ctx);
        if self.status_seen.0 != self.status {
            self.status_seen = (self.status.clone(), std::time::Instant::now());
            ctx.request_repaint_after(std::time::Duration::from_secs(5));
        }
        // Every couple of minutes with unsaved changes, keep a copy to
        // recover from (File > Recover autosave).
        if self.doc.dirty && self.tool.is_none() && self.prefs.autosave_minutes > 0 && self.last_autosave.elapsed() > std::time::Duration::from_secs(60 * self.prefs.autosave_minutes as u64) {
            self.last_autosave = std::time::Instant::now();
            if let Some(p) = crate::recent::autosave_path() {
                if let Err(e) = commands::write_native(&self.doc, &p) {
                    self.status = format!("Autosave failed: {e}");
                }
            }
        }
        let cmds = std::mem::take(&mut self.pending);
        for c in cmds {
            commands::run(self, c);
        }
        if std::mem::take(&mut self.quit_requested) {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
        if std::mem::take(&mut self.pending_finish) {
            self.finish_tool();
        }
        if std::mem::take(&mut self.pending_cancel) {
            self.cancel_tool();
        }
        crate::ui::top_bar(self, ui);
        crate::ui::left_panel(self, ui);
        crate::ui::right_panel(self, ui);
        egui::CentralPanel::default().frame(egui::Frame::new().fill(crate::theme::FRAME).inner_margin(egui::Margin { left: 0, right: 0, top: 0, bottom: 6 })).show(ui, |ui| self.viewport(ui, frame));
        crate::ui::viewport_overlays(self, &ctx);
        crate::ui::palette(self, &ctx);
        crate::ui::help_window(self, &ctx);
        crate::ui::prefs_window(self, &ctx);
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

    pub fn look(&self) -> crate::render::Look {
        self.renderer.as_ref().map(|r| r.look).unwrap_or_default()
    }

    pub fn set_look(&mut self, l: crate::render::Look) {
        if let Some(r) = self.renderer.as_mut() {
            r.look = l;
        }
    }

    pub fn analysis(&self) -> Analysis {
        self.renderer.as_ref().map(|r| r.analysis).unwrap_or_default()
    }

    /// Switches to analysis shading `a`, or back to plain shading when it
    /// is already on. Draft analysis pulls along the construction plane's normal.
    pub fn toggle_analysis(&mut self, a: Analysis) {
        let z = self.cplane.frame.z.as_vec3();
        if let Some(r) = self.renderer.as_mut() {
            r.analysis = if r.analysis == a { Analysis::None } else { a };
            r.pull.0 = z.into();
        }
    }

    /// Minimum draft angle in degrees for draft analysis.
    pub fn draft_angle(&mut self) -> Option<&mut f32> {
        self.renderer.as_mut().filter(|r| r.analysis == Analysis::Draft).map(|r| &mut r.pull.1)
    }

    /// Section analysis on or off: the plane starts through the scene's
    /// centre, facing along the construction plane's normal.
    pub fn toggle_section(&mut self) {
        self.section = match self.section {
            Some(_) => None,
            None => Some((self.doc.bbox().center(), self.cplane.frame.z, 0.0)),
        };
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
        if self.doc.mode != m || self.doc.also != 0 {
            self.doc.mode = m;
            self.doc.also = 0;
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

/// A computed typed value, without trailing zeros.
fn fmt_value(v: f64) -> String {
    let s = format!("{v:.4}");
    s.trim_end_matches('0').trim_end_matches('.').to_string()
}
