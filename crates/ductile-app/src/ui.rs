// SPDX-License-Identifier: GPL-3.0-only
//! Interface panels around the viewport.

use crate::app::DuctileApp;
use crate::render::Analysis;
use crate::commands::Command;
use crate::document::{GroupId, ObjectId, SelectMode};
use crate::theme;
use egui::{Color32, RichText};

pub fn top_bar(app: &mut DuctileApp, ui: &mut egui::Ui) {
    egui::Panel::top("top").frame(egui::Frame::new().fill(theme::PANEL_DARK).inner_margin(egui::Margin::symmetric(8, 4))).show(ui, |ui| {
        ui.horizontal(|ui| {
            ui.label(RichText::new("Ductile").strong().color(theme::ACCENT));
            ui.separator();
            ui.menu_button("File", |ui| {
                for c in [Command::New, Command::Open] {
                    menu_item(app, ui, c);
                }
                let recent = crate::recent::list();
                ui.add_enabled_ui(!recent.is_empty(), |ui| {
                    ui.menu_button("Open recent", |ui| {
                        for p in recent {
                            let name = p.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
                            if ui.button(name).on_hover_text(p.display().to_string()).clicked() {
                                crate::commands::open_file(app, &p);
                                ui.close();
                            }
                        }
                    });
                });
                if let Some(p) = crate::recent::autosave_path().filter(|p| p.exists()) {
                    if ui.button("Recover autosave").on_hover_text("Open the copy kept while you work").clicked() {
                        crate::commands::open_file(app, &p);
                        // Not the document's own file: Save asks where.
                        app.doc.path = None;
                        app.doc.dirty = true;
                        ui.close();
                    }
                }
                ui.separator();
                for c in [Command::Save, Command::SaveAs, Command::ImportStep, Command::ImportSvg, Command::ImportMesh, Command::ExportStep, Command::ExportStl, Command::ExportObj, Command::ExportHiddenLine] {
                    menu_item(app, ui, c);
                }
            });
            ui.menu_button("Edit", |ui| {
                for c in [Command::Undo, Command::Redo, Command::Duplicate, Command::Instance, Command::MakeUnique, Command::Delete, Command::SelectAll, Command::DeselectAll, Command::Hide, Command::Isolate, Command::UnhideAll] {
                    menu_item(app, ui, c);
                }
            });
            ui.menu_button("Create", |ui| {
                for c in [Command::Box, Command::Cylinder, Command::Cone, Command::Sphere, Command::Torus] {
                    menu_item(app, ui, c);
                }
            });
            ui.menu_button("Curve", |ui| {
                for c in [Command::Curve, Command::Spline, Command::Arc, Command::TangentArc, Command::Rectangle, Command::Circle, Command::TangentCircle, Command::Ellipse, Command::Polygon, Command::Slot, Command::Helix, Command::Text, Command::OffsetCurve, Command::JoinCurves, Command::Trim, Command::SplitCurve, Command::InsertKnot, Command::BridgeCurve, Command::AlignCurve, Command::ExtendCurve, Command::ReverseCurve, Command::RaiseDegree, Command::RebuildCurve, Command::IntersectionCurve, Command::ProjectCurve] {
                    menu_item(app, ui, c);
                }
            });
            ui.menu_button("Solid", |ui| {
                for c in [Command::Extrude, Command::Revolve, Command::Loft, Command::Sweep, Command::Pipe, Command::Patch, Command::BridgeSurface, Command::MatchSurface, Command::Isoparam, Command::RaiseSurfaceDegree, Command::RebuildFace, Command::Untrim, Command::Unjoin, Command::ReverseSheet, Command::FindBoundaryEdges, Command::Thicken, Command::Cut, Command::Imprint, Command::Draft, Command::Fillet, Command::FullRound, Command::Chamfer, Command::Shell, Command::OffsetSolid, Command::Union, Command::Subtract, Command::Intersect] {
                    menu_item(app, ui, c);
                }
            });
            ui.menu_button("Transform", |ui| {
                for c in [Command::Move, Command::Rotate, Command::Scale, Command::MirrorX, Command::MirrorY, Command::MirrorZ, Command::LinearArray, Command::RadialArray, Command::CurveArray, Command::Place] {
                    menu_item(app, ui, c);
                }
            });
            ui.menu_button("View", |ui| {
                for c in [
                    Command::ViewTop,
                    Command::ViewFront,
                    Command::ViewRight,
                    Command::ViewIso,
                    Command::ToggleOrtho,
                    Command::FrameSelection,
                    Command::ToggleGrid,
                    Command::ToggleEdges,
                    Command::ToggleXray,
                    Command::ToggleZebra,
                    Command::ToggleIsophote,
                    Command::ToggleDraft,
                    Command::ToggleFaceDirection,
                    Command::ToggleCurvature,
                    Command::ToggleSection,
                    Command::Measure,
                    Command::Dimension,
                    Command::DeleteDimensions,
                    Command::ToggleSnap,
                ] {
                    menu_item(app, ui, c);
                }
            });
            ui.menu_button("Help", |ui| {
                menu_item(app, ui, Command::Palette);
                menu_item(app, ui, Command::Help);
            });
            ui.separator();
            // Selection mode switcher.
            for (m, label, cmd) in [
                (SelectMode::Vertex, "Vertex", Command::ModeVertex),
                (SelectMode::Edge, "Edge", Command::ModeEdge),
                (SelectMode::Face, "Face", Command::ModeFace),
                (SelectMode::Object, "Solid", Command::ModeObject),
            ] {
                let sel = app.doc.mode == m;
                if ui
                    .add(egui::Button::new(label).selected(sel))
                    .on_hover_text(format!("{} ({})", cmd.label(), cmd.shortcut_text()))
                    .clicked()
                {
                    app.pending.push(cmd);
                }
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let proj = match app.camera.projection {
                    crate::camera::Projection::Perspective => "Perspective",
                    crate::camera::Projection::Orthographic => "Orthographic",
                };
                if ui.button(proj).clicked() {
                    app.pending.push(Command::ToggleOrtho);
                }
                if ui.add(egui::Button::new("Snap").selected(app.snap_enabled)).clicked() {
                    app.pending.push(Command::ToggleSnap);
                }
                if app.show_curvature {
                    ui.add(egui::DragValue::new(&mut app.comb_scale).speed(0.002).range(0.001..=5.0).max_decimals(3)).on_hover_text("Comb scale");
                }
                if ui.add(egui::Button::new("Curvature").selected(app.show_curvature)).on_hover_text("Curvature combs on the selected curves and edges").clicked() {
                    app.pending.push(Command::ToggleCurvature);
                }
                if let Some((_, n, d)) = app.section.as_mut() {
                    if ui.small_button("Flip").on_hover_text("Cut away the other side").clicked() {
                        *n = -*n;
                        *d = -*d;
                    }
                    ui.add(egui::DragValue::new(d).speed(0.01).max_decimals(3)).on_hover_text("Section offset");
                }
                if ui.add(egui::Button::new("Section").selected(app.section.is_some())).on_hover_text("Cut the scene away beyond a plane along the construction plane's normal").clicked() {
                    app.pending.push(Command::ToggleSection);
                }
                if let Some(a) = app.draft_angle() {
                    ui.add(egui::DragValue::new(a).speed(0.1).range(0.0..=45.0).suffix("°")).on_hover_text("Minimum draft angle");
                }
                let current = app.analysis();
                egui::ComboBox::from_id_salt("analysis").selected_text(current.label()).width(110.0).show_ui(ui, |ui| {
                    for a in Analysis::ALL {
                        if ui.selectable_label(current == a, a.label()).clicked() && current != a {
                            app.toggle_analysis(a);
                        }
                    }
                });
                if ui.add(egui::Button::new("X-ray").selected(app.xray())).clicked() {
                    app.pending.push(Command::ToggleXray);
                }
            });
        });
    });
}

fn menu_item(app: &mut DuctileApp, ui: &mut egui::Ui, c: Command) {
    let b = egui::Button::new(c.label()).shortcut_text(c.shortcut_text());
    if ui.add(b).clicked() {
        app.pending.push(c);
        ui.close();
    }
}

pub fn status_bar(app: &mut DuctileApp, ui: &mut egui::Ui) {
    egui::Panel::bottom("status").frame(egui::Frame::new().fill(theme::PANEL_DARK).inner_margin(egui::Margin::symmetric(10, 4))).show(ui, |ui| {
        ui.horizontal(|ui| {
            let hint = app.tool.as_ref().map(|t| t.hint()).unwrap_or_else(|| app.status.clone());
            if app.tool.as_ref().is_some_and(|t| t.busy()) {
                ui.add(egui::Spinner::new().size(12.0).color(theme::TEXT_DIM));
            }
            ui.label(RichText::new(hint).color(theme::TEXT));
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let n = app.doc.selection.len();
                ui.label(RichText::new(format!("{} objects · {} selected", app.doc.objects.len(), n)).color(theme::TEXT_DIM));
            });
        });
    });
}

pub fn toolbar(app: &mut DuctileApp, ui: &mut egui::Ui) {
    use Command::*;
    let groups: [(&str, &[Command]); 4] = [
        ("Solid", &[Box, Cylinder, Cone, Sphere, Torus]),
        ("Curve", &[Curve, Spline, Arc, Rectangle, Circle, Ellipse, Polygon]),
        ("Model", &[Extrude, Revolve, Loft, Sweep, Pipe, Fillet, Chamfer, Shell, Union, Subtract, Intersect]),
        ("Edit", &[Move, Rotate, Scale, MirrorX, RadialArray, Duplicate, Delete]),
    ];
    egui::Panel::left("tools").resizable(false).exact_size(64.0).frame(egui::Frame::new().fill(theme::PANEL).inner_margin(egui::Margin::same(6))).show(ui, |ui| {
        egui::ScrollArea::vertical().scroll_bar_visibility(egui::scroll_area::ScrollBarVisibility::AlwaysHidden).show(ui, |ui| {
            ui.vertical_centered(|ui| {
                for (title, cmds) in groups {
                    section(ui, title);
                    for pair in cmds.chunks(2) {
                        ui.horizontal(|ui| {
                            ui.spacing_mut().item_spacing.x = 4.0;
                            for c in pair {
                                icon_button(app, ui, *c);
                            }
                        });
                    }
                }
            });
        });
    });
}

/// A square toolbar button showing the command's icon, its name and
/// shortcut on hover.
fn icon_button(app: &mut DuctileApp, ui: &mut egui::Ui, cmd: Command) {
    let tip = match cmd.shortcut_text().as_str() {
        "" => cmd.label().to_string(),
        s => format!("{}   ({s})", cmd.label()),
    };
    let active = app.tool.as_ref().map(|t| t.name() == cmd.label()).unwrap_or(false);
    let resp = ui.add(egui::Button::new("").selected(active).min_size(egui::vec2(24.0, 24.0)));
    let color = if active || resp.hovered() { theme::TEXT } else { theme::TEXT_DIM };
    crate::icons::paint(cmd, ui.painter(), resp.rect.shrink(3.0), color);
    if resp.on_hover_text(tip).clicked() {
        app.pending.push(cmd);
    }
}

fn section(ui: &mut egui::Ui, title: &str) {
    ui.add_space(4.0);
    ui.label(RichText::new(title).size(10.0).color(theme::TEXT_DIM));
}

pub fn side_panel(app: &mut DuctileApp, ui: &mut egui::Ui) {
    egui::Panel::right("side").resizable(true).default_size(250.0).frame(egui::Frame::new().fill(theme::PANEL).inner_margin(egui::Margin::same(10))).show(ui, |ui| {
        if let Some(tool) = app.tool.as_mut() {
            ui.label(RichText::new(tool.name()).strong().size(15.0));
            ui.add_space(4.0);
            tool.panel(ui);
            ui.add_space(6.0);
            ui.horizontal(|ui| {
                if ui.button("OK").clicked() {
                    app.pending_finish = true;
                }
                if ui.button("Cancel").clicked() {
                    app.pending_cancel = true;
                }
            });
            ui.separator();
        }
        ui.label(RichText::new("Outliner").strong());
        egui::ScrollArea::vertical().max_height(ui.available_height() * 0.6).show(ui, |ui| {
            let mut acts = vec![];
            outliner_level(app, ui, None, 0, &mut acts);
            let shift = ui.input(|i| i.modifiers.shift);
            for a in acts {
                outliner_act(app, a, shift);
            }
        });
        ui.separator();
        properties(app, ui);
    });
}

/// Something done in the Outliner, applied after drawing it.
enum OutlinerAct {
    ToggleVisible(ObjectId),
    ToggleLock(ObjectId),
    Select(ObjectId),
    GroupVisible(GroupId),
    GroupLock(GroupId),
    GroupOpen(GroupId),
    SelectGroup(GroupId),
    Activate(Option<GroupId>),
    Dissolve(GroupId),
    DeleteGroup(GroupId),
    AddSelection(GroupId),
    ToRoot(ObjectId),
    Rename(GroupId, String),
}

fn row_icon(ui: &mut egui::Ui, tip: &str, paint: impl FnOnce(&egui::Painter, egui::Rect, Color32)) -> bool {
    let r = ui.add(egui::Button::new("").frame(false).min_size(egui::vec2(18.0, 18.0)));
    let col = if r.hovered() { theme::TEXT } else { theme::TEXT_DIM };
    paint(ui.painter(), r.rect.shrink2(egui::vec2(1.0, 3.0)), col);
    r.on_hover_text(tip).clicked()
}

/// The groups and objects directly inside `parent`, indented by `depth`.
fn outliner_level(app: &DuctileApp, ui: &mut egui::Ui, parent: Option<GroupId>, depth: usize, acts: &mut Vec<OutlinerAct>) {
    let doc = &app.doc;
    let indent = depth as f32 * 14.0;
    for g in doc.groups.iter().filter(|g| g.parent == parent) {
        let active = doc.active_group == Some(g.id);
        ui.horizontal(|ui| {
            ui.add_space(indent);
            let (vis, locked) = (g.visible, g.locked);
            if row_icon(ui, if vis { "Hide group" } else { "Show group" }, |p, r, c| crate::icons::eye(p, r, if vis { c } else { c.gamma_multiply(0.5) }, vis)) {
                acts.push(OutlinerAct::GroupVisible(g.id));
            }
            if row_icon(ui, if locked { "Unlock group" } else { "Lock group" }, |p, r, c| crate::icons::lock(p, r, if locked { c } else { c.gamma_multiply(0.45) }, locked)) {
                acts.push(OutlinerAct::GroupLock(g.id));
            }
            let open = g.open;
            if row_icon(ui, if open { "Collapse" } else { "Expand" }, |p, r, c| {
                let r = r.shrink(2.0);
                let pts = if open { vec![r.left_top(), r.right_top(), egui::pos2(r.center().x, r.bottom())] } else { vec![r.left_top(), egui::pos2(r.right(), r.center().y), r.left_bottom()] };
                p.add(egui::Shape::convex_polygon(pts, c, egui::Stroke::NONE));
            }) {
                acts.push(OutlinerAct::GroupOpen(g.id));
            }
            let folder = ui.add(egui::Button::new("").frame(false).min_size(egui::vec2(16.0, 18.0)));
            crate::icons::folder(ui.painter(), folder.rect.shrink2(egui::vec2(0.0, 2.0)), if active { theme::ACCENT } else { theme::TEXT_DIM });
            let mut text = RichText::new(g.name.as_str()).color(if vis { theme::TEXT } else { theme::TEXT_DIM });
            if active {
                text = text.color(theme::ACCENT).strong();
            }
            let r = ui.selectable_label(false, text).on_hover_text("Click: select its objects · double-click: make it the active group (new objects go in it)");
            if r.double_clicked() {
                acts.push(OutlinerAct::Activate(if active { None } else { Some(g.id) }));
            } else if r.clicked() {
                acts.push(OutlinerAct::SelectGroup(g.id));
            }
            r.context_menu(|ui| {
                let mut name = g.name.clone();
                if ui.text_edit_singleline(&mut name).changed() {
                    acts.push(OutlinerAct::Rename(g.id, name));
                }
                if ui.button(if active { "Stop being the active group" } else { "Make active group" }).clicked() {
                    acts.push(OutlinerAct::Activate(if active { None } else { Some(g.id) }));
                    ui.close();
                }
                if ui.button("Select all").clicked() {
                    acts.push(OutlinerAct::SelectGroup(g.id));
                    ui.close();
                }
                if ui.button("Add current selection to group").clicked() {
                    acts.push(OutlinerAct::AddSelection(g.id));
                    ui.close();
                }
                if ui.button("Dissolve (keep children)").clicked() {
                    acts.push(OutlinerAct::Dissolve(g.id));
                    ui.close();
                }
                if ui.button("Delete recursively").clicked() {
                    acts.push(OutlinerAct::DeleteGroup(g.id));
                    ui.close();
                }
            });
        });
        if g.open {
            outliner_level(app, ui, Some(g.id), depth + 1, acts);
        }
    }
    for o in doc.objects.iter().filter(|o| o.group == parent || (parent.is_none() && o.group.is_some_and(|g| doc.group(g).is_none()))) {
        let selected = doc.selection.contains(&crate::document::Pick::Object(o.id));
        ui.horizontal(|ui| {
            ui.add_space(indent);
            let (vis, locked) = (o.visible, o.locked);
            if row_icon(ui, if vis { "Hide" } else { "Show" }, |p, r, c| crate::icons::eye(p, r, if vis { c } else { c.gamma_multiply(0.5) }, vis)) {
                acts.push(OutlinerAct::ToggleVisible(o.id));
            }
            if row_icon(ui, if locked { "Unlock" } else { "Lock (shows, but can't be selected)" }, |p, r, c| crate::icons::lock(p, r, if locked { c } else { c.gamma_multiply(0.45) }, locked)) {
                acts.push(OutlinerAct::ToggleLock(o.id));
            }
            let text = RichText::new(o.name.as_str()).color(if vis && !locked { theme::TEXT } else { theme::TEXT_DIM });
            let r = ui.selectable_label(selected, text);
            if r.clicked() && !doc.is_locked(o.id) {
                acts.push(OutlinerAct::Select(o.id));
            }
            if o.group.is_some() {
                r.context_menu(|ui| {
                    if ui.button("Move to root").clicked() {
                        acts.push(OutlinerAct::ToRoot(o.id));
                        ui.close();
                    }
                });
            }
        });
    }
}

fn outliner_act(app: &mut DuctileApp, a: OutlinerAct, shift: bool) {
    use crate::document::Pick;
    let doc = &mut app.doc;
    match a {
        OutlinerAct::ToggleVisible(id) => {
            if let Some(o) = doc.objects.iter_mut().find(|o| o.id == id) {
                o.visible = !o.visible;
            }
        }
        OutlinerAct::ToggleLock(id) => {
            if let Some(o) = doc.objects.iter_mut().find(|o| o.id == id) {
                o.locked = !o.locked;
            }
            if doc.is_locked(id) {
                doc.selection.retain(|p| p.object() != id);
            }
        }
        OutlinerAct::Select(id) => {
            if !shift {
                doc.selection.clear();
            }
            doc.mode = SelectMode::Object;
            doc.selection.insert(Pick::Object(id));
        }
        OutlinerAct::GroupVisible(g) => {
            let vis = doc.group(g).is_some_and(|x| x.visible);
            doc.set_group_visible(g, !vis);
        }
        OutlinerAct::GroupLock(g) => {
            if let Some(x) = doc.groups.iter_mut().find(|x| x.id == g) {
                x.locked = !x.locked;
            }
            let members = doc.group_members(g);
            let locked: Vec<ObjectId> = members.into_iter().filter(|id| doc.is_locked(*id)).collect();
            doc.selection.retain(|p| !locked.contains(&p.object()));
        }
        OutlinerAct::GroupOpen(g) => {
            if let Some(x) = doc.groups.iter_mut().find(|x| x.id == g) {
                x.open = !x.open;
            }
        }
        OutlinerAct::SelectGroup(g) => {
            if !shift {
                doc.selection.clear();
            }
            doc.mode = SelectMode::Object;
            let ids: Vec<ObjectId> = doc.group_members(g).into_iter().filter(|id| !doc.is_locked(*id) && doc.object(*id).is_some_and(|o| o.visible)).collect();
            doc.selection.extend(ids.into_iter().map(Pick::Object));
        }
        OutlinerAct::Activate(g) => doc.active_group = g,
        OutlinerAct::Dissolve(g) => {
            doc.checkpoint("Dissolve group");
            doc.ungroup(g);
        }
        OutlinerAct::DeleteGroup(g) => {
            doc.checkpoint("Delete group");
            for id in doc.group_members(g) {
                doc.remove(id);
            }
            let inner: Vec<GroupId> = doc.groups.iter().filter(|x| doc.within(Some(x.id), g)).map(|x| x.id).collect();
            doc.groups.retain(|x| !inner.contains(&x.id));
            if doc.active_group.is_some_and(|a| inner.contains(&a)) {
                doc.active_group = None;
            }
        }
        OutlinerAct::AddSelection(g) => {
            doc.checkpoint("Add to group");
            let ids = doc.selected_objects();
            for o in doc.objects.iter_mut().filter(|o| ids.contains(&o.id)) {
                o.group = Some(g);
            }
        }
        OutlinerAct::ToRoot(id) => {
            doc.checkpoint("Move to root");
            if let Some(o) = doc.objects.iter_mut().find(|o| o.id == id) {
                o.group = None;
            }
        }
        OutlinerAct::Rename(g, name) => {
            if let Some(x) = doc.groups.iter_mut().find(|x| x.id == g) {
                x.name = name;
            }
            doc.dirty = true;
        }
    }
}

fn properties(app: &mut DuctileApp, ui: &mut egui::Ui) {
    let ids = app.doc.selected_objects();
    if ids.len() != 1 {
        return;
    }
    let id = ids[0];
    let Some(o) = app.doc.objects.iter_mut().find(|o| o.id == id) else { return };
    ui.label(RichText::new("Properties").strong());
    ui.horizontal(|ui| {
        ui.label("Name");
        ui.text_edit_singleline(&mut o.name);
    });
    ui.horizontal(|ui| {
        ui.label("Color");
        ui.color_edit_button_rgb(&mut o.color);
    });
    ui.horizontal(|ui| {
        ui.label("Metal");
        ui.add(egui::Slider::new(&mut o.metalness, 0.0..=1.0).show_value(false));
        ui.label("Rough");
        ui.add(egui::Slider::new(&mut o.roughness, 0.0..=1.0).show_value(false));
    });
    ui.horizontal(|ui| {
        for (name, c, m, r) in [("Plastic", [0.62, 0.63, 0.65], 0.0, 0.45), ("Steel", [0.78, 0.78, 0.80], 1.0, 0.3), ("Chrome", [0.95, 0.95, 0.96], 1.0, 0.05), ("Gold", [1.0, 0.78, 0.34], 1.0, 0.2), ("Rubber", [0.12, 0.12, 0.13], 0.0, 0.9)] {
            if ui.small_button(name).clicked() {
                (o.color, o.metalness, o.roughness) = (c, m, r);
            }
        }
    });
    let b = &o.body;
    ui.label(
        RichText::new(match &b.mesh {
            Some(m) => format!("Mesh · {} triangles · {} points", m.triangles.len(), m.positions.len()),
            None => format!("{} faces · {} edges · {} vertices", b.faces.len(), b.edges.len(), b.vertices.len()),
        })
            .color(theme::TEXT_DIM),
    );
    if o.link.is_some() {
        ui.label(RichText::new("Instance: edits apply to every linked copy").color(theme::TEXT_DIM));
    }
    let bb = b.bbox();
    let s = bb.size();
    ui.label(RichText::new(format!("Size {:.3} × {:.3} × {:.3}", s.x, s.y, s.z)).color(theme::TEXT_DIM));
    if b.kind == ductile_brep::BodyKind::Solid {
        ui.label(RichText::new(format!("Volume ≈ {:.4}", o.mesh.volume())).color(theme::TEXT_DIM));
    }
    ui.label(RichText::new(format!("Area ≈ {:.4}", o.mesh.area())).color(theme::TEXT_DIM));
}

pub fn palette(app: &mut DuctileApp, ctx: &egui::Context) {
    if !app.palette_open {
        return;
    }
    let mut open = true;
    let mut chosen = None;
    egui::Window::new("Command palette")
        .title_bar(false)
        .anchor(egui::Align2::CENTER_TOP, egui::vec2(0.0, 80.0))
        .fixed_size(egui::vec2(420.0, 360.0))
        .show(ctx, |ui| {
            let r = ui.add(egui::TextEdit::singleline(&mut app.palette_query).hint_text("Type a command…").desired_width(f32::INFINITY));
            r.request_focus();
            let q = app.palette_query.to_lowercase();
            let matches: Vec<Command> = Command::ALL
                .iter()
                .copied()
                .filter(|c| *c != Command::Palette && (q.is_empty() || fuzzy(&c.label().to_lowercase(), &q)))
                .collect();
            if ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                chosen = matches.first().copied();
            }
            if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                open = false;
            }
            egui::ScrollArea::vertical().show(ui, |ui| {
                for c in matches {
                    let b = egui::Button::new(c.label()).shortcut_text(c.shortcut_text()).frame(false);
                    let r = ui.add_sized([ui.available_width(), 22.0], b).on_hover_text("Right-click to change the shortcut");
                    if r.clicked() {
                        chosen = Some(c);
                    }
                    r.context_menu(|ui| {
                        if ui.button("Assign shortcut…").clicked() {
                            app.assigning = Some(c);
                            app.status = format!("Press the new shortcut for {} (Esc keeps the old one)", c.label());
                            open = false;
                            ui.close();
                        }
                        if ui.button("Reset to default").clicked() {
                            crate::keymap::keymap_mut().reset(c);
                            ui.close();
                        }
                    });
                }
            });
        });
    if let Some(c) = chosen {
        app.pending.push(c);
        open = false;
    }
    app.palette_open = open;
}

fn fuzzy(hay: &str, needle: &str) -> bool {
    let mut it = hay.chars();
    needle.chars().all(|c| it.any(|h| h == c))
}

pub fn help_window(app: &mut DuctileApp, ctx: &egui::Context) {
    if !app.show_help {
        return;
    }
    let mut open = true;
    egui::Window::new("Ductile help").open(&mut open).default_width(420.0).show(ctx, |ui| {
        ui.label(RichText::new("Navigation").strong());
        ui.label("Orbit: right or middle mouse drag\nPan: Shift + right/middle drag\nZoom: mouse wheel (zooms toward the cursor)\nFrame selection: / · Command palette: F · Construction plane from selection: Space");
        ui.add_space(6.0);
        ui.label(RichText::new("Selection").strong());
        ui.label("1 vertices · 2 edges · 3 faces · 4 solids\nClick to select, Shift+click to toggle, Ctrl+click to remove, drag for box select");
        ui.add_space(6.0);
        ui.label(RichText::new("Commands").strong());
        egui::Grid::new("keys").striped(true).show(ui, |ui| {
            for c in Command::ALL {
                if !c.shortcut_text().is_empty() {
                    ui.label(c.label());
                    ui.label(RichText::new(c.shortcut_text()).color(Color32::from_rgb(170, 180, 255)));
                    ui.end_row();
                }
            }
        });
        ui.add_space(6.0);
        ui.label("While a command runs: type a number to enter a value, X/Y/Z to constrain, Enter to confirm, Esc to cancel.");
        ui.label("Shortcuts follow Plasticity's. Right-click a command in the palette (F) to change its shortcut.");
    });
    app.show_help = open;
}
