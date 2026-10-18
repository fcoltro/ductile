// SPDX-License-Identifier: GPL-3.0-only
//! Interface panels around the viewport.

use crate::app::DuctileApp;
use crate::commands::Command;
use crate::document::SelectMode;
use crate::theme;
use egui::{Color32, RichText};

pub fn top_bar(app: &mut DuctileApp, ui: &mut egui::Ui) {
    egui::Panel::top("top").frame(egui::Frame::new().fill(theme::PANEL_DARK).inner_margin(egui::Margin::symmetric(8, 4))).show(ui, |ui| {
        ui.horizontal(|ui| {
            ui.label(RichText::new("Ductile").strong().color(theme::ACCENT));
            ui.separator();
            ui.menu_button("File", |ui| {
                for c in [Command::New, Command::Open, Command::Save, Command::SaveAs, Command::ImportStep, Command::ExportStep, Command::ExportStl, Command::ExportObj] {
                    menu_item(app, ui, c);
                }
            });
            ui.menu_button("Edit", |ui| {
                for c in [Command::Undo, Command::Redo, Command::Duplicate, Command::Delete, Command::SelectAll, Command::DeselectAll, Command::Hide, Command::Isolate, Command::UnhideAll] {
                    menu_item(app, ui, c);
                }
            });
            ui.menu_button("Create", |ui| {
                for c in [Command::Box, Command::Cylinder, Command::Cone, Command::Sphere, Command::Torus] {
                    menu_item(app, ui, c);
                }
            });
            ui.menu_button("Curve", |ui| {
                for c in [Command::Curve, Command::Spline, Command::Arc, Command::Rectangle, Command::Circle, Command::Ellipse, Command::Polygon, Command::Helix, Command::OffsetCurve, Command::JoinCurves, Command::Trim, Command::BridgeCurve, Command::ExtendCurve, Command::IntersectionCurve, Command::ProjectCurve] {
                    menu_item(app, ui, c);
                }
            });
            ui.menu_button("Solid", |ui| {
                for c in [Command::Extrude, Command::Revolve, Command::Loft, Command::Sweep, Command::Patch, Command::Thicken, Command::Cut, Command::Imprint, Command::Draft, Command::Fillet, Command::Chamfer, Command::Shell, Command::OffsetSolid, Command::Union, Command::Subtract, Command::Intersect] {
                    menu_item(app, ui, c);
                }
            });
            ui.menu_button("Transform", |ui| {
                for c in [Command::Move, Command::Rotate, Command::Scale, Command::MirrorX, Command::MirrorY, Command::MirrorZ, Command::LinearArray, Command::RadialArray] {
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
                if ui.add(egui::Button::new("Zebra").selected(app.zebra())).clicked() {
                    app.pending.push(Command::ToggleZebra);
                }
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
        ("Model", &[Extrude, Revolve, Loft, Sweep, Fillet, Chamfer, Shell, Union, Subtract, Intersect]),
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
    let tip = match cmd.shortcut_text() {
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
            let mut toggle_vis = None;
            let mut select = None;
            for o in &app.doc.objects {
                let selected = app.doc.selection.contains(&crate::document::Pick::Object(o.id));
                ui.horizontal(|ui| {
                    let r = ui.add(egui::Button::new("").frame(false).min_size(egui::vec2(18.0, 18.0)));
                    let col = if r.hovered() { theme::TEXT } else if o.visible { theme::TEXT_DIM } else { theme::TEXT_DIM.gamma_multiply(0.5) };
                    crate::icons::eye(ui.painter(), r.rect.shrink2(egui::vec2(1.0, 3.0)), col, o.visible);
                    if r.on_hover_text(if o.visible { "Hide" } else { "Show" }).clicked() {
                        toggle_vis = Some(o.id);
                    }
                    let text = RichText::new(o.name.as_str()).color(if o.visible { theme::TEXT } else { theme::TEXT_DIM });
                    if ui.selectable_label(selected, text).clicked() {
                        select = Some(o.id);
                    }
                });
            }
            if let Some(id) = toggle_vis {
                if let Some(o) = app.doc.objects.iter_mut().find(|o| o.id == id) {
                    o.visible = !o.visible;
                }
            }
            if let Some(id) = select {
                let shift = ui.input(|i| i.modifiers.shift);
                if !shift {
                    app.doc.selection.clear();
                }
                app.doc.mode = SelectMode::Object;
                app.doc.selection.insert(crate::document::Pick::Object(id));
            }
        });
        ui.separator();
        properties(app, ui);
    });
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
    let b = &o.body;
    ui.label(
        RichText::new(format!("{} faces · {} edges · {} vertices", b.faces.len(), b.edges.len(), b.vertices.len()))
            .color(theme::TEXT_DIM),
    );
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
                    if ui.add_sized([ui.available_width(), 22.0], b).clicked() {
                        chosen = Some(c);
                    }
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
        ui.label("Orbit: right or middle mouse drag\nPan: Shift + right/middle drag\nZoom: mouse wheel (zooms toward the cursor)\nFrame selection: F · Top view: T · Orthographic: O");
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
    });
    app.show_help = open;
}
