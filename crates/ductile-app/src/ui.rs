// SPDX-License-Identifier: GPL-3.0-only
//! Interface panels around the viewport.

use crate::app::DuctileApp;
use crate::render::Analysis;
use crate::commands::Command;
use crate::document::{GroupId, ObjectId, SelectMode};
use crate::theme;
use egui::{Color32, RichText};

/// The menus behind the app button at the left of the title row.
fn app_menu(app: &mut DuctileApp, ui: &mut egui::Ui) {
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
        for c in [Command::Save, Command::SaveAs, Command::QuickSaveVersion, Command::SaveStartup] {
            menu_item(app, ui, c);
        }
        ui.separator();
        for c in [Command::ImportAppend, Command::ImportImage, Command::ExportStep, Command::ExportStl, Command::ExportObj, Command::ExportHiddenLine] {
            menu_item(app, ui, c);
        }
        ui.separator();
        for c in [Command::NewWindow, Command::Quit] {
            menu_item(app, ui, c);
        }
    });
    ui.menu_button("Edit", |ui| {
        for c in [Command::Undo, Command::Redo, Command::Duplicate, Command::Instance, Command::MakeUnique, Command::DeleteRedundant, Command::CompleteEdge, Command::UnwrapFace, Command::PolySplines, Command::Delete, Command::SelectAll, Command::DeselectAll, Command::SelectPrevious, Command::Hide, Command::Isolate, Command::UnhideAll] {
            menu_item(app, ui, c);
        }
    });
    ui.menu_button("Create", |ui| {
        for c in [Command::Box, Command::Cylinder, Command::Cone, Command::Sphere, Command::Torus] {
            menu_item(app, ui, c);
        }
    });
    ui.menu_button("Curve", |ui| {
        for c in [Command::Curve, Command::Spline, Command::Arc, Command::TangentArc, Command::Rectangle, Command::Circle, Command::TangentCircle, Command::Ellipse, Command::Polygon, Command::Slot, Command::Helix, Command::Text, Command::OffsetCurve, Command::JoinCurves, Command::Trim, Command::SplitCurve, Command::InsertKnot, Command::ConvertVertex, Command::DeformCurve, Command::SlideCv, Command::BridgeCurve, Command::AlignCurve, Command::ExtendCurve, Command::ReverseCurve, Command::RaiseDegree, Command::RebuildCurve, Command::IntersectionCurve, Command::ProjectCurve] {
            menu_item(app, ui, c);
        }
    });
    ui.menu_button("Solid", |ui| {
        for c in [Command::Extrude, Command::Revolve, Command::Loft, Command::Sweep, Command::Pipe, Command::Patch, Command::ConstrainedSurface, Command::BridgeSurface, Command::MatchSurface, Command::Isoparam, Command::RaiseSurfaceDegree, Command::RebuildFace, Command::Untrim, Command::Unjoin, Command::ReverseSheet, Command::FindBoundaryEdges, Command::Thicken, Command::Cut, Command::Imprint, Command::OffsetEdge, Command::OffsetFaceTo, Command::Draft, Command::Fillet, Command::FullRound, Command::RemoveFillets, Command::Chamfer, Command::Shell, Command::OffsetSolid, Command::Union, Command::Subtract, Command::Intersect] {
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
    ui.separator();
    menu_item(app, ui, Command::Preferences);
    menu_item(app, ui, Command::Palette);
    menu_item(app, ui, Command::Help);
}

/// A frameless square button painted by `paint`, with a tooltip.
fn glyph_button(ui: &mut egui::Ui, size: f32, on: bool, tip: &str, paint: impl FnOnce(&egui::Painter, egui::Rect, Color32)) -> egui::Response {
    let (rect, resp) = ui.allocate_exact_size(egui::vec2(size, size), egui::Sense::click());
    let p = ui.painter();
    if on {
        p.rect_filled(rect, 4.0, theme::ACCENT);
    } else if resp.hovered() {
        p.rect_filled(rect, 4.0, theme::HOVER);
    }
    let col = if on || resp.hovered() { theme::TEXT } else { theme::TEXT_DIM };
    paint(p, rect.shrink(size * 0.2), col);
    resp.on_hover_text(tip)
}

fn toggle_command(m: SelectMode) -> Command {
    match m {
        SelectMode::Vertex => Command::ToggleVertexMode,
        SelectMode::Edge => Command::ToggleEdgeMode,
        SelectMode::Face => Command::ToggleFaceMode,
        SelectMode::Object => Command::ToggleObjectMode,
    }
}

fn tip(c: Command) -> String {
    match c.shortcut_text().as_str() {
        "" => c.label().to_string(),
        s => format!("{}   {s}", c.label()),
    }
}

/// The title row: the app menu, the panel toggles and whether the
/// document has unsaved changes.
pub fn top_bar(app: &mut DuctileApp, ui: &mut egui::Ui) {
    egui::Panel::top("top").exact_size(32.0).frame(egui::Frame::new().fill(theme::FRAME).inner_margin(egui::Margin::symmetric(10, 4))).show(ui, |ui| {
        ui.horizontal_centered(|ui| {
            ui.menu_button(RichText::new("Ductile").strong().color(theme::TEXT), |ui| app_menu(app, ui)).response.on_hover_text("Menu");
            ui.add_space(6.0);
            if glyph_button(ui, 22.0, false, "Left panel   Ctrl+B", |p, r, c| crate::icons::sidebar(p, r, c, true)).clicked() {
                app.show_left = !app.show_left;
            }
            ui.add_space(10.0);
            let name = app.doc.path.as_ref().and_then(|p| p.file_name()).map(|n| n.to_string_lossy().into_owned()).unwrap_or_else(|| "UNSAVED".into());
            ui.label(RichText::new(name).size(12.0).color(theme::TEXT_DIM));
            // A hollow dot when saved, filled once changed.
            let (r, _) = ui.allocate_exact_size(egui::vec2(10.0, 10.0), egui::Sense::hover());
            if app.doc.dirty {
                ui.painter().circle_filled(r.center(), 3.5, theme::TEXT_DIM);
            } else {
                ui.painter().circle_stroke(r.center(), 3.0, egui::Stroke::new(1.2, theme::TEXT_DIM));
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if glyph_button(ui, 22.0, false, "Right panel   Ctrl+Shift+B", |p, r, c| crate::icons::sidebar(p, r, c, false)).clicked() {
                    app.show_right = !app.show_right;
                }
                if app.tool.as_ref().is_some_and(|t| t.busy()) {
                    ui.add(egui::Spinner::new().size(14.0).color(theme::TEXT_DIM));
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

/// A side-panel card: a heading in capitals that folds the body away.
fn card(ui: &mut egui::Ui, title: &str, body: impl FnOnce(&mut egui::Ui)) {
    egui::Frame::new().fill(theme::CARD).corner_radius(6.0).inner_margin(egui::Margin::symmetric(10, 8)).show(ui, |ui| {
        ui.set_width(ui.available_width());
        let id = ui.make_persistent_id(("card", title));
        let mut open = ui.data_mut(|d| *d.get_persisted_mut_or(id, true));
        let head = ui.horizontal(|ui| {
            ui.label(RichText::new(title).size(11.5).strong().color(theme::TEXT_DIM));
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let (r, _) = ui.allocate_exact_size(egui::vec2(14.0, 14.0), egui::Sense::hover());
                crate::icons::chevron(ui.painter(), r, theme::TEXT_DIM, open);
            });
        });
        if ui.interact(head.response.rect, id.with("head"), egui::Sense::click()).clicked() {
            open = !open;
            ui.data_mut(|d| d.insert_persisted(id, open));
        }
        if open {
            ui.add_space(4.0);
            body(ui);
        }
    });
    ui.add_space(6.0);
}

/// The left panel: the Outliner and the Assets (reference images).
pub fn left_panel(app: &mut DuctileApp, ui: &mut egui::Ui) {
    if !app.show_left {
        return;
    }
    egui::Panel::left("outliner").resizable(true).default_size(230.0).frame(egui::Frame::new().fill(theme::FRAME).inner_margin(egui::Margin { left: 8, right: 4, top: 4, bottom: 8 })).show(ui, |ui| {
        egui::Frame::new().fill(theme::CARD).corner_radius(6.0).inner_margin(3.0).show(ui, |ui| {
            ui.columns(2, |cols| {
                for (k, name) in [(0u8, "OUTLINER"), (1, "ASSETS")] {
                    let on = app.left_tab == k;
                    let b = egui::Button::new(RichText::new(name).size(11.5).color(if on { theme::TEXT } else { theme::TEXT_DIM })).fill(if on { theme::HOVER } else { Color32::TRANSPARENT }).frame(on);
                    if cols[k as usize].add_sized([cols[k as usize].available_width(), 22.0], b).clicked() {
                        app.left_tab = k;
                    }
                }
            });
        });
        ui.add_space(6.0);
        if app.left_tab == 1 {
            ui.horizontal(|ui| {
                if ui.button("Import image…").clicked() {
                    app.pending.push(Command::ImportImage);
                }
            });
            ui.add_space(4.0);
            if app.doc.images.is_empty() {
                ui.label(RichText::new("No reference images yet").color(theme::TEXT_DIM));
            } else {
                reference_images(app, ui);
            }
            return;
        }
        ui.horizontal(|ui| {
            if glyph_button(ui, 20.0, false, "Group selected   Ctrl+G", |p, r, c| crate::icons::folder(p, r, c)).clicked() {
                app.pending.push(Command::Group);
            }
            ui.add(egui::TextEdit::singleline(&mut app.outliner_filter).hint_text("Search").desired_width(f32::INFINITY));
        });
        ui.add_space(4.0);
        egui::ScrollArea::vertical().show(ui, |ui| {
            let mut acts = vec![];
            egui::CollapsingHeader::new(RichText::new("Scene").color(theme::TEXT)).id_salt("scene").default_open(true).show(ui, |ui| {
                outliner_level(app, ui, None, 1, &mut acts, false);
                outliner_kinds(app, ui, &mut acts);
            });
            let shift = ui.input(|i| i.modifiers.shift);
            for a in acts {
                outliner_act(app, a, shift);
            }
        });
    });
}

/// Ungrouped objects under a heading per kind, as Plasticity's Scene tree.
fn outliner_kinds(app: &DuctileApp, ui: &mut egui::Ui, acts: &mut Vec<OutlinerAct>) {
    use ductile_brep::BodyKind;
    let doc = &app.doc;
    let q = app.outliner_filter.to_lowercase();
    let loose: Vec<&crate::document::Object> = doc.objects.iter().filter(|o| (o.group.is_none() || o.group.is_some_and(|g| doc.group(g).is_none())) && (q.is_empty() || o.name.to_lowercase().contains(&q))).collect();
    for (title, kind) in [("Solids", Some(BodyKind::Solid)), ("Surfaces", Some(BodyKind::Sheet)), ("Curves", Some(BodyKind::Wire)), ("Meshes", None)] {
        let these: Vec<&&crate::document::Object> = loose.iter().filter(|o| if kind.is_some() { o.body.mesh.is_none() && Some(o.body.kind) == kind } else { o.body.mesh.is_some() }).collect();
        if these.is_empty() {
            continue;
        }
        egui::CollapsingHeader::new(RichText::new(title).color(theme::TEXT_DIM)).id_salt(("kind", title)).default_open(true).show(ui, |ui| {
            for o in these {
                object_row(app, ui, o, 0.0, acts);
            }
        });
    }
}

/// The floating command dialog, the hint strip, the tool strip, the
/// command bar and the toggles over the viewport.
pub fn viewport_overlays(app: &mut DuctileApp, ctx: &egui::Context) {
    let vr = app.viewport_rect;
    if !vr.is_positive() {
        return;
    }
    let pill = |ui: &mut egui::Ui, body: &mut dyn FnMut(&mut egui::Ui)| {
        egui::Frame::new().fill(theme::CARD).corner_radius(6.0).inner_margin(3.0).show(ui, |ui| body(ui));
    };
    // The value being dragged, next to the handle.
    if let Some((p, text)) = app.tool.as_ref().and_then(|t| t.readout()) {
        if let Some(n) = app.camera.project(p) {
            let at = egui::pos2(vr.min.x + ((n.x + 1.0) * 0.5) as f32 * vr.width(), vr.min.y + ((1.0 - n.y) * 0.5) as f32 * vr.height()) + egui::vec2(14.0, -14.0);
            if vr.contains(at) {
                let painter = ctx.layer_painter(egui::LayerId::new(egui::Order::Foreground, egui::Id::unique("readout")));
                let galley = painter.layout_no_wrap(text, egui::FontId::monospace(12.0), theme::TEXT);
                let r = egui::Rect::from_min_size(at - egui::vec2(0.0, galley.size().y), galley.size()).expand2(egui::vec2(6.0, 3.0));
                painter.rect_filled(r, 4.0, theme::CARD.gamma_multiply(0.92));
                painter.galley(r.min + egui::vec2(6.0, 3.0), galley, theme::TEXT);
            }
        }
    }
    // Selection modes, top left.
    egui::Area::new(egui::Id::unique("modes")).fixed_pos(vr.min + egui::vec2(10.0, 10.0)).order(egui::Order::Foreground).show(ctx, |ui| {
        pill(ui, &mut |ui| {
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 3.0;
                for (m, cmd) in [(SelectMode::Vertex, Command::ModeVertex), (SelectMode::Edge, Command::ModeEdge), (SelectMode::Face, Command::ModeFace), (SelectMode::Object, Command::ModeObject)] {
                    let r = glyph_button(ui, 26.0, app.doc.mode_on(m), &format!("{}\nShift+click to toggle", tip(cmd)), |p, r, c| {
                        crate::icons::paint(cmd, p, r, c);
                    });
                    if r.clicked() {
                        // Shift adds or removes the mode instead of switching to it.
                        app.pending.push(if ui.input(|i| i.modifiers.shift) { toggle_command(m) } else { cmd });
                    }
                }
            });
        });
    });
    // The named view, top centre.
    if let Some(name) = app.named_view() {
        egui::Area::new(egui::Id::unique("view chip")).pivot(egui::Align2::CENTER_TOP).fixed_pos(egui::pos2(vr.center().x, vr.min.y + 10.0)).order(egui::Order::Foreground).show(ctx, |ui| {
            pill(ui, &mut |ui| {
                ui.horizontal(|ui| {
                    if glyph_button(ui, 22.0, false, "Leave the aligned view", |p, r, c| crate::icons::cross(p, r, c)).clicked() {
                        app.pending.push(Command::ViewIso);
                    }
                    ui.label(RichText::new(name).color(theme::TEXT));
                    ui.add_space(4.0);
                });
            });
        });
    }
    // View toggles, top right.
    egui::Area::new(egui::Id::unique("view toggles")).pivot(egui::Align2::RIGHT_TOP).fixed_pos(egui::pos2(vr.max.x - 10.0, vr.min.y + 10.0)).order(egui::Order::Foreground).show(ctx, |ui| {
        ui.horizontal(|ui| {
            pill(ui, &mut |ui| {
                if glyph_button(ui, 26.0, app.section.is_some(), &tip(Command::ToggleSection), |p, r, c| {
                    crate::icons::paint(Command::ToggleSection, p, r, c);
                })
                .clicked()
                {
                    app.pending.push(Command::ToggleSection);
                }
            });
            pill(ui, &mut |ui| {
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 3.0;
                    let ortho = app.camera.projection == crate::camera::Projection::Orthographic;
                    for (cmd, on) in [(Command::ToggleOrtho, ortho), (Command::ToggleEdges, app.show_edges), (Command::ToggleXray, app.xray())] {
                        if glyph_button(ui, 26.0, on, &tip(cmd), |p, r, c| {
                            crate::icons::paint(cmd, p, r, c);
                        })
                        .clicked()
                        {
                            app.pending.push(cmd);
                        }
                    }
                    let current = app.analysis();
                    let r = glyph_button(ui, 26.0, current != Analysis::ALL[0], &format!("Render mode: {}", current.label()), |p, r, c| {
                        p.circle_stroke(r.center(), r.width() * 0.45, egui::Stroke::new(1.4, c));
                        p.circle_filled(r.center() + egui::vec2(-r.width() * 0.12, -r.width() * 0.12), r.width() * 0.16, c);
                    });
                    egui::Popup::menu(&r).show(|ui| {
                        for a in Analysis::ALL {
                            if ui.selectable_label(current == a, a.label()).clicked() && current != a {
                                app.toggle_analysis(a);
                            }
                        }
                    });
                });
            });
        });
    });
    // The tool strip, left.
    {
        use Command::*;
        let strip = [Curve, Spline, Circle, Polygon, Helix, Rectangle, Arc, BridgeCurve, Trim, SplitCurve, Sphere, Box, Cylinder, Measure];
        egui::Area::new(egui::Id::unique("tool strip")).pivot(egui::Align2::LEFT_CENTER).fixed_pos(egui::pos2(vr.min.x + 10.0, vr.center().y)).order(egui::Order::Foreground).show(ctx, |ui| {
            pill(ui, &mut |ui| {
                ui.vertical(|ui| {
                    ui.spacing_mut().item_spacing.y = 3.0;
                    for c in strip {
                        let active = app.tool.as_ref().is_some_and(|t| t.name() == c.label());
                        if glyph_button(ui, 28.0, active, &tip(c), |p, r, col| {
                            crate::icons::paint(c, p, r, col);
                        })
                        .clicked()
                        {
                            app.pending.push(c);
                        }
                    }
                });
            });
        });
    }
    // The command bar, bottom centre.
    {
        use Command::*;
        egui::Area::new(egui::Id::unique("command bar")).pivot(egui::Align2::CENTER_BOTTOM).fixed_pos(egui::pos2(vr.center().x, vr.max.y - 10.0)).order(egui::Order::Foreground).show(ctx, |ui| {
            pill(ui, &mut |ui| {
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 3.0;
                    for (k, c) in [Palette, Move, Rotate, Scale, Duplicate, Boolean, LinearArray, RadialArray].into_iter().enumerate() {
                        if k == 1 {
                            ui.add_space(6.0);
                        }
                        if glyph_button(ui, 28.0, false, &tip(c), |p, r, col| {
                            crate::icons::paint(c, p, r, col);
                        })
                        .clicked()
                        {
                            app.pending.push(c);
                        }
                    }
                    let more = glyph_button(ui, 28.0, false, "More commands   Shift+F", |p, r, c| crate::icons::dots(p, r, c));
                    egui::Popup::menu(&more).show(|ui| suggested(app, ui, false));
                });
            });
        });
    }
    // Suggested commands at the cursor (Shift+F).
    if let Some(at) = app.suggest_at {
        let resp = egui::Area::new(egui::Id::unique("suggested at cursor")).fixed_pos(at).constrain(true).order(egui::Order::Foreground).show(ctx, |ui| {
            egui::Frame::menu(ui.style()).show(ui, |ui| {
                ui.set_min_width(200.0);
                ui.label(RichText::new("SUGGESTED").size(11.0).color(theme::TEXT_DIM));
                let before = app.pending.len();
                ui.visuals_mut().widgets.inactive.weak_bg_fill = Color32::TRANSPARENT;
                ui.with_layout(egui::Layout::top_down_justified(egui::Align::LEFT), |ui| suggested(app, ui, true));
                if app.pending.len() != before {
                    app.suggest_at = None;
                }
            });
        });
        if resp.response.clicked_elsewhere() {
            app.suggest_at = None;
        }
    }
    // The command dialog, bottom left.
    if app.tool.is_some() {
        egui::Area::new(egui::Id::unique("dialog")).pivot(egui::Align2::LEFT_BOTTOM).fixed_pos(egui::pos2(vr.min.x + 52.0, vr.max.y - 10.0)).order(egui::Order::Foreground).show(ctx, |ui| {
            egui::Frame::new().fill(theme::CARD).corner_radius(8.0).stroke(egui::Stroke::new(1.0, Color32::from_rgb(44, 44, 49))).inner_margin(egui::Margin::same(10)).show(ui, |ui| {
                ui.set_width(270.0);
                let tool = app.tool.as_mut().unwrap();
                let id = ui.make_persistent_id("dialog open");
                let mut open = ui.data_mut(|d| *d.get_persisted_mut_or(id, true));
                ui.horizontal(|ui| {
                    ui.label(RichText::new(tool.name().to_uppercase()).size(12.0).strong().color(theme::TEXT));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if glyph_button(ui, 18.0, false, if open { "Fold" } else { "Unfold" }, |p, r, c| crate::icons::chevron(p, r, c, open)).clicked() {
                            open = !open;
                            ui.data_mut(|d| d.insert_persisted(id, open));
                        }
                    });
                });
                if open {
                    ui.add_space(4.0);
                    for slot in tool.slots() {
                        slot_row(ui, &slot);
                    }
                    tool.panel(ui);
                }
                ui.add_space(6.0);
                ui.separator();
                ui.horizontal(|ui| {
                    if key_button(ui, "Esc", "Cancel").clicked() {
                        app.pending_cancel = true;
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if key_button(ui, "Enter", "OK").clicked() {
                            app.pending_finish = true;
                        }
                    });
                });
            });
        });
    }
    // The command's keys, bottom right; else the last message, bottom left.
    let keys = app.tool.as_ref().map(|t| t.keys()).unwrap_or_default();
    if !keys.is_empty() {
        egui::Area::new(egui::Id::unique("hints")).pivot(egui::Align2::RIGHT_BOTTOM).fixed_pos(egui::pos2(vr.max.x - 14.0, vr.max.y - 12.0)).order(egui::Order::Foreground).interactable(false).show(ctx, |ui| {
            ui.horizontal_top(|ui| {
                for col in keys.chunks(4) {
                    ui.vertical(|ui| {
                        for (k, what) in col {
                            ui.horizontal(|ui| {
                                key_chip(ui, k);
                                ui.label(RichText::new(*what).size(11.5).color(theme::TEXT_DIM));
                            });
                        }
                    });
                    ui.add_space(10.0);
                }
            });
        });
    }
    let message = app.tool.as_ref().map(|t| t.hint()).or_else(|| app.recent_status().map(str::to_string));
    if let Some(m) = message {
        let at = egui::pos2(vr.center().x, vr.min.y + if app.named_view().is_some() { 48.0 } else { 12.0 });
        egui::Area::new(egui::Id::unique("message")).pivot(egui::Align2::CENTER_TOP).fixed_pos(at).order(egui::Order::Foreground).interactable(false).show(ctx, |ui| {
            egui::Frame::new().fill(theme::CARD.gamma_multiply(0.9)).corner_radius(5.0).inner_margin(egui::Margin::symmetric(10, 4)).show(ui, |ui| {
                ui.set_max_width(520.0);
                ui.add(egui::Label::new(RichText::new(m).size(12.0).color(theme::TEXT)).wrap());
            });
        });
    }
}

/// A key in a small chip, as in the key hints.
fn key_chip(ui: &mut egui::Ui, key: &str) {
    egui::Frame::new().fill(theme::FIELD).corner_radius(3.0).inner_margin(egui::Margin::symmetric(4, 1)).show(ui, |ui| {
        ui.label(RichText::new(key).size(11.0).color(theme::TEXT_DIM));
    });
}

/// A footer button of the command dialog: label and its key chip.
fn key_button(ui: &mut egui::Ui, key: &str, label: &str) -> egui::Response {
    let r = ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 4.0;
        key_chip(ui, key);
        ui.label(RichText::new(label).color(if label == "OK" { theme::TEXT } else { theme::TEXT_DIM }));
    });
    ui.interact(r.response.rect, ui.id().with(label), egui::Sense::click()).on_hover_cursor(egui::CursorIcon::PointingHand)
}

/// A command input at the top of its dialog: a green check once filled,
/// what to pick, and what fills it.
fn slot_row(ui: &mut egui::Ui, slot: &crate::tools::Slot) {
    egui::Frame::new().fill(theme::PANEL_DARK).corner_radius(5.0).inner_margin(egui::Margin::symmetric(8, 5)).show(ui, |ui| {
        ui.set_width(ui.available_width());
        ui.horizontal(|ui| {
            let (r, _) = ui.allocate_exact_size(egui::vec2(16.0, 16.0), egui::Sense::hover());
            let p = ui.painter();
            if slot.filled.is_some() {
                p.circle_filled(r.center(), 8.0, theme::ACCENT);
                let c = r.center();
                p.line(vec![c + egui::vec2(-3.5, 0.0), c + egui::vec2(-1.0, 2.5), c + egui::vec2(3.5, -2.5)], egui::Stroke::new(1.6, Color32::WHITE));
            } else {
                p.circle_filled(r.center(), 8.0, theme::FIELD);
            }
            ui.vertical(|ui| {
                ui.spacing_mut().item_spacing.y = 0.0;
                ui.label(RichText::new(slot.title).size(12.0).color(theme::TEXT));
                ui.label(RichText::new(slot.detail).size(11.0).color(theme::TEXT_DIM));
            });
            if let Some(f) = &slot.filled {
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
                    ui.label(RichText::new(f).size(11.0).color(theme::TEXT_DIM));
                });
            }
        });
    });
    ui.add_space(2.0);
}

/// The More menu (Shift+F): one fixed list, with what does not apply to
/// the selection greyed out so places stay put.
fn suggested(app: &mut DuctileApp, ui: &mut egui::Ui, only_applicable: bool) {
    use crate::document::Pick;
    use Command::*;
    let sel = &app.doc.selection;
    let objs = app.doc.selected_objects();
    let kinds: Vec<ductile_brep::BodyKind> = objs.iter().filter_map(|id| app.doc.object(*id)).map(|o| o.body.kind).collect();
    let curves = kinds.contains(&ductile_brep::BodyKind::Wire);
    let solids = kinds.contains(&ductile_brep::BodyKind::Solid);
    let sheets = kinds.contains(&ductile_brep::BodyKind::Sheet);
    let faces = sel.iter().any(|p| matches!(p, Pick::Face(..)));
    let edges = sel.iter().any(|p| matches!(p, Pick::Edge(..)));
    let any = !sel.is_empty();
    if only_applicable && !any {
        ui.label(RichText::new("Select something to see what applies").color(theme::TEXT_DIM));
        return;
    }
    for (c, ok) in [
        (CurveArray, any),
        (MirrorX, any),
        (Cut, solids || sheets),
        (JoinCurves, curves || sheets),
        (Unjoin, solids || sheets),
        (Shell, solids || faces),
        (Thicken, sheets || faces),
        (Pipe, curves || edges),
        (Loft, curves || edges || faces),
        (Sweep, curves),
        (Revolve, curves || faces),
        (Extrude, curves || faces),
        (Patch, curves || edges),
        (OffsetCurve, curves),
        (RaiseDegree, curves),
        (RebuildCurve, curves),
        (ProjectCurve, curves),
        (Imprint, curves),
        (CompleteEdge, edges),
        (Fillet, edges || curves),
        (Draft, faces),
        (MatchSurface, edges),
    ] {
        if only_applicable && !ok {
            continue;
        }
        let b = egui::Button::new(c.label()).shortcut_text(c.shortcut_text());
        if ui.add_enabled(ok, b).clicked() {
            app.pending.push(c);
            ui.close();
        }
    }
}

/// The right panel: cards for how the scene shows, the camera, the grid,
/// snapping, construction planes, curvature combs and the selection.
pub fn right_panel(app: &mut DuctileApp, ui: &mut egui::Ui) {
    if !app.show_right {
        return;
    }
    egui::Panel::right("cards").resizable(false).exact_size(230.0).frame(egui::Frame::new().fill(theme::FRAME).inner_margin(egui::Margin { left: 4, right: 8, top: 4, bottom: 8 })).show(ui, |ui| {
        egui::ScrollArea::vertical().scroll_bar_visibility(egui::scroll_area::ScrollBarVisibility::AlwaysHidden).show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.spacing_mut().slider_width = 60.0;
            if app.doc.selected_objects().len() == 1 {
                card(ui, "SELECTION", |ui| properties(app, ui));
            }
            card(ui, "SHADER MODE", |ui| {
                let current = app.analysis();
                egui::Grid::new("shader modes").num_columns(3).spacing([10.0, 8.0]).show(ui, |ui| {
                    for (k, a) in Analysis::ALL.into_iter().enumerate() {
                        let (rect, resp) = ui.allocate_exact_size(egui::vec2(52.0, 40.0), egui::Sense::click());
                        let ball = egui::Rect::from_center_size(rect.center(), egui::vec2(34.0, 34.0));
                        crate::icons::swatch(ui.painter(), ball, a);
                        if current == a {
                            ui.painter().circle_stroke(ball.center(), 19.5, egui::Stroke::new(1.5, theme::ACCENT));
                        } else if resp.hovered() {
                            ui.painter().circle_stroke(ball.center(), 19.5, egui::Stroke::new(1.0, theme::TEXT_DIM));
                        }
                        if resp.on_hover_text(a.label()).clicked() && current != a {
                            app.toggle_analysis(a);
                        }
                        if k % 3 == 2 {
                            ui.end_row();
                        }
                    }
                });
                if current == Analysis::None {
                    // Looks for plain shading, our own matcap-like studies.
                    let look = app.look();
                    ui.horizontal(|ui| {
                        ui.spacing_mut().item_spacing.x = 6.0;
                        for l in crate::render::Look::ALL {
                            let (rect, resp) = ui.allocate_exact_size(egui::vec2(28.0, 28.0), egui::Sense::click());
                            let ball = rect.shrink(3.0);
                            crate::icons::look_swatch(ui.painter(), ball, l);
                            if look == l {
                                ui.painter().circle_stroke(ball.center(), 12.5, egui::Stroke::new(1.5, theme::ACCENT));
                            } else if resp.hovered() {
                                ui.painter().circle_stroke(ball.center(), 12.5, egui::Stroke::new(1.0, theme::TEXT_DIM));
                            }
                            if resp.on_hover_text(l.label()).clicked() {
                                app.set_look(l);
                            }
                        }
                    });
                    ui.label(RichText::new(format!("Shaded · {}", look.label())).size(11.0).color(theme::TEXT_DIM));
                } else {
                    ui.label(RichText::new(current.label()).size(11.0).color(theme::TEXT_DIM));
                }
                ui.add_space(4.0);
                let mut edges = app.show_edges;
                if theme::switch_row(ui, "Show edges", &mut edges) {
                    app.pending.push(Command::ToggleEdges);
                }
                let mut xray = app.xray();
                theme::switch_row(ui, "Show curves", &mut app.show_curves);
                theme::switch_row(ui, "Show faces", &mut app.show_faces);
                if theme::switch_row(ui, "X-ray", &mut xray) {
                    app.pending.push(Command::ToggleXray);
                }
                if let Some(a) = app.draft_angle() {
                    ui.horizontal(|ui| {
                        ui.label("Minimum draft");
                        ui.add(egui::DragValue::new(a).speed(0.1).range(0.0..=45.0).suffix("°"));
                    });
                }
            });
            card(ui, "CAMERA", |ui| {
                let mut ortho = app.camera.projection == crate::camera::Projection::Orthographic;
                if theme::switch_row(ui, "Orthographic", &mut ortho) {
                    app.pending.push(Command::ToggleOrtho);
                }
                ui.add_enabled_ui(!ortho, |ui| {
                    ui.horizontal(|ui| {
                        ui.label("FOV");
                        let mut deg = app.camera.fov_y.to_degrees();
                        if ui.add(egui::DragValue::new(&mut deg).range(10.0..=120.0).speed(0.5).suffix("°")).changed() {
                            app.camera.fov_y = deg.to_radians();
                        }
                    });
                });
                if ui.add_sized([ui.available_width(), 22.0], egui::Button::new("Reset to default")).clicked() {
                    app.pending.push(Command::ViewIso);
                }
            });
            card(ui, "GRID", |ui| {
                let mut grid = app.show_grid;
                if theme::switch_row(ui, "Show grid", &mut grid) {
                    app.pending.push(Command::ToggleGrid);
                }
                theme::switch_row(ui, "Snap to grid", &mut app.grid_snap);
                // The spacing, halved or doubled by the buttons; 0 follows the zoom.
                ui.horizontal(|ui| {
                    if ui.small_button("−").on_hover_text("Halve the spacing").clicked() {
                        app.grid_step = if app.grid_step > 0.0 { app.grid_step / 2.0 } else { 1.0 };
                    }
                    let w = ui.available_width() - 32.0;
                    let text = if app.grid_step > 0.0 { None } else { Some("auto") };
                    match text {
                        Some(t) => {
                            if ui.add_sized([w, 20.0], egui::Button::new(t).frame(false)).on_hover_text("Spacing follows the zoom; click for a fixed spacing").clicked() {
                                app.grid_step = 1.0;
                            }
                        }
                        None => {
                            ui.add_sized([w, 20.0], egui::DragValue::new(&mut app.grid_step).speed(0.01).range(0.0..=1e4).max_decimals(4));
                        }
                    }
                    if ui.small_button("+").on_hover_text("Double the spacing").clicked() {
                        app.grid_step = if app.grid_step > 0.0 { app.grid_step * 2.0 } else { 1.0 };
                    }
                });
                egui::Grid::new("grid sizes").num_columns(2).show(ui, |ui| {
                    ui.label("Grid size");
                    ui.add(egui::DragValue::new(&mut app.grid_count).range(4..=200));
                    ui.end_row();
                    ui.label("Line every");
                    ui.add(egui::DragValue::new(&mut app.grid_major).range(1..=50));
                    ui.end_row();
                });
            });
            card(ui, "SNAPPING", |ui| {
                let mut snap = app.snap_enabled;
                if theme::switch_row(ui, "Object snapping", &mut snap) {
                    app.pending.push(Command::ToggleSnap);
                }
                egui::CollapsingHeader::new(RichText::new("OBJECT SNAPS").size(11.0).color(theme::TEXT_DIM)).id_salt("object snaps").default_open(false).show(ui, |ui| {
                    let k = &mut app.snap_kinds;
                    theme::switch_row(ui, "Points", &mut k.points);
                    theme::switch_row(ui, "Edges", &mut k.edges);
                    theme::switch_row(ui, "Curves", &mut k.curves);
                    theme::switch_row(ui, "Faces", &mut k.faces);
                    theme::switch_row(ui, "Axes", &mut k.axes);
                });
                ui.add(egui::Label::new(RichText::new("Hold Ctrl to suspend snapping while you draw").size(11.0).color(theme::TEXT_DIM)).wrap());
            });
            card(ui, "PLANES", |ui| {
                ui.columns(3, |cols| {
                    for (k, (name, z, x)) in [("XY", ductile_geom::DVec3::Z, ductile_geom::DVec3::X), ("YZ", ductile_geom::DVec3::X, ductile_geom::DVec3::Y), ("XZ", -ductile_geom::DVec3::Y, ductile_geom::DVec3::X)].into_iter().enumerate() {
                        if cols[k].add_sized([cols[k].available_width(), 22.0], egui::Button::new(name)).on_hover_text(format!("Construction plane {name}")).clicked() {
                            app.cplane = crate::tools::CPlane { frame: ductile_geom::Frame::new(ductile_geom::DVec3::ZERO, z, x) };
                        }
                    }
                });
                for (label, c) in [("Plane from selection", Command::SetCPlaneOnly), ("Plane from camera", Command::CPlaneView)] {
                    if ui.add_sized([ui.available_width(), 22.0], egui::Button::new(label)).on_hover_text(tip(c)).clicked() {
                        app.pending.push(c);
                    }
                }
            });
            card(ui, "CURVATURE", |ui| {
                let mut combs = app.show_curvature;
                if theme::switch_row(ui, "Combs on selection", &mut combs) {
                    app.pending.push(Command::ToggleCurvature);
                }
                ui.horizontal(|ui| {
                    ui.label("Comb scale");
                    ui.add(egui::DragValue::new(&mut app.comb_scale).speed(0.002).range(0.001..=5.0).max_decimals(3));
                });
            });
            if let Some((_, n, d)) = app.section.as_mut() {
                card(ui, "SECTION", |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Offset");
                        ui.add(egui::DragValue::new(d).speed(0.01).max_decimals(3));
                        if ui.small_button("Flip").on_hover_text("Cut away the other side").clicked() {
                            *n = -*n;
                            *d = -*d;
                        }
                    });
                });
            }
        });
    });
}

/// The reference images: shown, opacity, size and placement.
fn reference_images(app: &mut DuctileApp, ui: &mut egui::Ui) {
    let mut remove = None;
    let cplane = crate::tools::current_cplane().frame;
    for (k, img) in app.doc.images.iter_mut().enumerate() {
        egui::CollapsingHeader::new(&img.name).id_salt(("refimg", img.id)).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.checkbox(&mut img.visible, "Shown");
                if ui.small_button("Delete").clicked() {
                    remove = Some(k);
                }
            });
            ui.horizontal(|ui| {
                ui.label("Opacity");
                ui.add(egui::Slider::new(&mut img.opacity, 0.0..=1.0));
            });
            ui.horizontal(|ui| {
                ui.label("Width");
                ui.add(egui::DragValue::new(&mut img.width).speed(0.05).range(1e-3..=1e6));
            });
            ui.horizontal(|ui| {
                ui.label("Centre");
                let o = &mut img.frame.origin;
                for c in [&mut o.x, &mut o.y, &mut o.z] {
                    ui.add(egui::DragValue::new(c).speed(0.05));
                }
            });
            ui.horizontal(|ui| {
                if ui.small_button("Turn 90°").clicked() {
                    img.frame = ductile_geom::Frame::new(img.frame.origin, img.frame.z, img.frame.y);
                }
                if ui.small_button("Flip").clicked() {
                    img.frame = ductile_geom::Frame::new(img.frame.origin, -img.frame.z, -img.frame.x);
                }
                if ui.small_button("Onto cplane").clicked() {
                    img.frame = ductile_geom::Frame::new(img.frame.origin, cplane.z, cplane.x);
                }
            });
        });
    }
    if let Some(k) = remove {
        app.doc.images.remove(k);
        app.doc.dirty = true;
    }
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
fn outliner_level(app: &DuctileApp, ui: &mut egui::Ui, parent: Option<GroupId>, depth: usize, acts: &mut Vec<OutlinerAct>, objects: bool) {
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
            outliner_level(app, ui, Some(g.id), depth + 1, acts, true);
        }
    }
    if !objects {
        return;
    }
    for o in doc.objects.iter().filter(|o| o.group == parent) {
        object_row(app, ui, o, indent, acts);
    }
}

/// A small glyph for what an object is: a cube for solids, a sheet for
/// surfaces, a stroke for curves, a triangle for meshes.
fn kind_icon(p: &egui::Painter, r: egui::Rect, o: &crate::document::Object, c: Color32) {
    use ductile_brep::BodyKind;
    let s = egui::Stroke::new(1.2, c);
    let (cx, cy, h) = (r.center().x, r.center().y, r.width() * 0.42);
    let pt = |x: f32, y: f32| egui::pos2(cx + x * h, cy + y * h);
    if o.body.mesh.is_some() {
        p.add(egui::Shape::closed_line(vec![pt(0.0, -0.9), pt(0.9, 0.7), pt(-0.9, 0.7)], s));
        return;
    }
    match o.body.kind {
        BodyKind::Solid => {
            // An isometric cube: hexagon outline and the three inner edges.
            let hex = [pt(0.0, -1.0), pt(0.87, -0.5), pt(0.87, 0.5), pt(0.0, 1.0), pt(-0.87, 0.5), pt(-0.87, -0.5)];
            p.add(egui::Shape::closed_line(hex.to_vec(), s));
            for q in [hex[1], hex[3], hex[5]] {
                p.line_segment([pt(0.0, 0.0), q], s);
            }
        }
        BodyKind::Sheet => {
            p.add(egui::Shape::closed_line(vec![pt(-0.5, -0.8), pt(0.95, -0.8), pt(0.5, 0.8), pt(-0.95, 0.8)], s));
        }
        _ => {
            p.add(egui::Shape::line(vec![pt(-0.9, 0.8), pt(-0.3, 0.1), pt(0.3, -0.2), pt(0.9, -0.9)], s));
        }
    }
}

fn object_row(app: &DuctileApp, ui: &mut egui::Ui, o: &crate::document::Object, indent: f32, acts: &mut Vec<OutlinerAct>) {
    let doc = &app.doc;
    let selected = doc.selection.contains(&crate::document::Pick::Object(o.id));
    // Eye and lock show on the hovered row, and stay while they are off.
    let row = egui::Rect::from_min_size(ui.cursor().min, egui::vec2(ui.available_width(), 20.0));
    let hovered = ui.rect_contains_pointer(row);
    ui.horizontal(|ui| {
        ui.add_space(indent);
        let (vis, locked) = (o.visible, o.locked);
        let (ir, _) = ui.allocate_exact_size(egui::vec2(14.0, 14.0), egui::Sense::hover());
        kind_icon(ui.painter(), ir, o, if vis { theme::ACCENT } else { theme::TEXT_DIM });
        let text = RichText::new(o.name.as_str()).color(if vis && !locked { theme::TEXT } else { theme::TEXT_DIM });
        let r = ui.selectable_label(selected, text);
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if (hovered || locked) && row_icon(ui, if locked { "Unlock" } else { "Lock (shows, but can't be selected)" }, |p, r, c| crate::icons::lock(p, r, if locked { c } else { c.gamma_multiply(0.45) }, locked)) {
                acts.push(OutlinerAct::ToggleLock(o.id));
            }
            if (hovered || !vis) && row_icon(ui, if vis { "Hide" } else { "Show" }, |p, r, c| crate::icons::eye(p, r, if vis { c } else { c.gamma_multiply(0.5) }, vis)) {
                acts.push(OutlinerAct::ToggleVisible(o.id));
            }
        });
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
    ui.horizontal(|ui| {
        ui.label("Name");
        ui.add(egui::TextEdit::singleline(&mut o.name).desired_width(f32::INFINITY));
    });
    ui.horizontal(|ui| {
        ui.label("Color");
        ui.color_edit_button_rgb(&mut o.color);
    });
    ui.horizontal(|ui| {
        ui.label("Metal");
        ui.add(egui::Slider::new(&mut o.metalness, 0.0..=1.0).show_value(false));
    });
    ui.horizontal(|ui| {
        ui.label("Rough");
        ui.add(egui::Slider::new(&mut o.roughness, 0.0..=1.0).show_value(false));
    });
    ui.horizontal_wrapped(|ui| {
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
    // Opens at the mouse, as Plasticity's command search.
    let at = app.palette_at.unwrap_or(egui::pos2(ctx.content_rect().center().x - 210.0, 80.0));
    let favourites = app.prefs.favourites.clone();
    let mut prefs_changed = false;
    egui::Window::new("Command palette")
        .title_bar(false)
        .fixed_pos(at)
        .constrain(true)
        .fixed_size(egui::vec2(420.0, 360.0))
        .show(ctx, |ui| {
            let r = ui.add(egui::TextEdit::singleline(&mut app.palette_query).hint_text("Search commands").desired_width(f32::INFINITY));
            if app.assigning.is_none() {
                r.request_focus();
            }
            let q = app.palette_query.to_lowercase();
            let mut matches: Vec<Command> = Command::ALL
                .iter()
                .copied()
                .filter(|c| *c != Command::Palette && (q.is_empty() || fuzzy(&c.label().to_lowercase(), &q)))
                .collect();
            // Favourites first, then A to Z.
            matches.sort_by_key(|c| (!favourites.contains(c), c.label().to_lowercase()));
            if ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                chosen = matches.first().copied();
            }
            if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                open = false;
            }
            // A solid scroll bar with its own room, clear of the row menus.
            ui.spacing_mut().scroll = egui::style::ScrollStyle::solid();
            egui::ScrollArea::vertical().show(ui, |ui| {
                let mut heading = None;
                for c in matches {
                    let fav = favourites.contains(&c);
                    let h = if fav { "FAVOURITES" } else { "ALL COMMANDS" };
                    if !favourites.is_empty() && heading != Some(h) {
                        heading = Some(h);
                        ui.label(RichText::new(h).size(11.0).color(theme::TEXT_DIM));
                    }
                    ui.horizontal(|ui| {
                        // The row's own menu, as the … on each search result.
                        let menu = ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            let dots = glyph_button(ui, 20.0, false, "Shortcut and favourite", |p, r, col| crate::icons::dots(p, r, col));
                            egui::Popup::menu(&dots).show(|ui| {
                                if ui.button("Documentation").clicked() {
                                    app.show_help = true;
                                    ui.close();
                                }
                                if ui.button("Assign shortcut…").clicked() {
                                    app.assigning = Some(c);
                                    app.status = format!("Press the new shortcut for {} (Esc keeps the old one)", c.label());
                                    open = false;
                                    ui.close();
                                }
                                if ui.add_enabled(!c.shortcut_text().is_empty(), egui::Button::new("Unset shortcut")).clicked() {
                                    crate::keymap::keymap_mut().unset(c);
                                    ui.close();
                                }
                                if ui.button("Reset to default").clicked() {
                                    crate::keymap::keymap_mut().reset(c);
                                    ui.close();
                                }
                                if ui.button(if fav { "Remove from favourites" } else { "Add to favourites" }).clicked() {
                                    if fav {
                                        app.prefs.favourites.retain(|x| *x != c);
                                    } else {
                                        app.prefs.favourites.push(c);
                                    }
                                    prefs_changed = true;
                                    ui.close();
                                }
                            });
                            let b = egui::Button::new(c.label()).shortcut_text(c.shortcut_text()).frame(false);
                            ui.add_sized([ui.available_width(), 22.0], b)
                        });
                        if menu.inner.clicked() {
                            chosen = Some(c);
                        }
                    });
                }
            });
        });
    if prefs_changed {
        app.prefs.save();
    }
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

pub fn prefs_window(app: &mut DuctileApp, ctx: &egui::Context) {
    if !app.show_prefs {
        return;
    }
    let mut open = true;
    let before = app.prefs.clone();
    let p = &mut app.prefs;
    egui::Window::new("Preferences").open(&mut open).default_width(360.0).show(ctx, |ui| {
        ui.label(RichText::new("Selection").strong());
        ui.checkbox(&mut p.auto_invoke, "Selecting a face starts Push face, an edge Fillet");
        ui.checkbox(&mut p.drill_down, "Second click on a selected solid picks its face or edge");
        ui.add_space(6.0);
        ui.label(RichText::new("Navigation").strong());
        egui::ComboBox::from_label("Mouse preset").selected_text(p.nav.label()).show_ui(ui, |ui| {
            for n in crate::prefs::NavPreset::ALL {
                ui.selectable_value(&mut p.nav, n, n.label());
            }
        });
        ui.checkbox(&mut p.zoom_to_cursor, "Zoom toward the cursor");
        ui.add_space(6.0);
        ui.label(RichText::new("Units").strong());
        egui::ComboBox::from_label("Lengths").selected_text(p.unit.name()).show_ui(ui, |ui| {
            for u in crate::units::Unit::ALL {
                ui.selectable_value(&mut p.unit, u, u.name());
            }
        });
        ui.label(RichText::new("The model stays in millimetres; this sets how lengths are shown and how a bare typed number is read").color(theme::TEXT_DIM));
        ui.add_space(6.0);
        ui.label(RichText::new("Files").strong());
        ui.horizontal(|ui| {
            ui.label("Autosave every");
            ui.add(egui::DragValue::new(&mut p.autosave_minutes).range(0..=120).suffix(" min"));
        });
        ui.checkbox(&mut p.load_last_session, "Open the last session on startup");
        ui.add_space(6.0);
        ui.label(RichText::new("Backups").strong());
        ui.label(RichText::new("Saving over a file keeps its old copy").color(theme::TEXT_DIM));
        egui::Grid::new("backup counts").num_columns(2).show(ui, |ui| {
            for (label, n) in [("Newest to keep", &mut p.backups.recent), ("Hourly", &mut p.backups.hourly), ("Daily", &mut p.backups.daily), ("Weekly", &mut p.backups.weekly)] {
                ui.label(label);
                ui.add(egui::DragValue::new(n).range(0..=50));
                ui.end_row();
            }
        });
        ui.horizontal(|ui| {
            ui.label("Folder");
            ui.add(egui::TextEdit::singleline(&mut p.backup_dir).hint_text("next to the file").desired_width(160.0));
        });
    });
    app.show_prefs = open;
    if app.prefs != before {
        app.prefs.save();
    }
}

pub fn help_window(app: &mut DuctileApp, ctx: &egui::Context) {
    if !app.show_help {
        return;
    }
    let mut open = true;
    egui::Window::new("Ductile help").open(&mut open).default_width(420.0).show(ctx, |ui| {
        ui.label(RichText::new("Navigation").strong());
        ui.label("Orbit: middle mouse drag\nPan: right drag or Shift + middle drag\nZoom: mouse wheel (zooms toward the cursor)\nOther presets (Blender, Maya, Rhino…) are in Preferences (Ctrl+,)\nFrame selection: / · Command search: F · Construction plane from selection: Space");
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
        ui.label("Shortcuts follow Plasticity's. In command search (F), each command's … menu assigns, unsets or resets its shortcut and pins it to favourites.");
    });
    app.show_help = open;
}
