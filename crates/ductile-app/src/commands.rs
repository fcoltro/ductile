// SPDX-License-Identifier: GPL-3.0-only
//! Named commands: what the toolbar, menus, palette and shortcuts invoke.

use crate::app::DuctileApp;
use crate::document::{Document, Pick, SelectMode};
use crate::tools::{BoxTool, CurveKind, CurveTool, ExtrudeTool, FilletTool, RevolveTool, RoundKind, RoundTool, TransformKind, TransformTool};
use ductile_brep::ops::boolean::{boolean, BoolError, BoolOp};
use ductile_brep::primitives::*;
use ductile_geom::{DMat4, DVec3, Frame};
use egui::{Key, Modifiers};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    New,
    Open,
    Save,
    SaveAs,
    ImportStep,
    ExportStep,
    ExportStl,
    ExportObj,
    Undo,
    Redo,
    Delete,
    Duplicate,
    SelectAll,
    DeselectAll,
    Hide,
    UnhideAll,
    ModeVertex,
    ModeEdge,
    ModeFace,
    ModeObject,
    Box,
    Cylinder,
    Cone,
    Sphere,
    Torus,
    Curve,
    Rectangle,
    Circle,
    Extrude,
    Revolve,
    Fillet,
    Chamfer,
    Union,
    Subtract,
    Intersect,
    Move,
    Rotate,
    Scale,
    MirrorX,
    MirrorY,
    MirrorZ,
    ViewTop,
    ViewFront,
    ViewRight,
    ViewIso,
    ToggleOrtho,
    FrameSelection,
    ToggleGrid,
    ToggleEdges,
    ToggleXray,
    ToggleSnap,
    Palette,
    Help,
}

impl Command {
    pub const ALL: &'static [Command] = &[
        Command::New,
        Command::Open,
        Command::Save,
        Command::SaveAs,
        Command::ImportStep,
        Command::ExportStep,
        Command::ExportStl,
        Command::ExportObj,
        Command::Undo,
        Command::Redo,
        Command::Delete,
        Command::Duplicate,
        Command::SelectAll,
        Command::DeselectAll,
        Command::Hide,
        Command::UnhideAll,
        Command::ModeVertex,
        Command::ModeEdge,
        Command::ModeFace,
        Command::ModeObject,
        Command::Box,
        Command::Cylinder,
        Command::Cone,
        Command::Sphere,
        Command::Torus,
        Command::Curve,
        Command::Rectangle,
        Command::Circle,
        Command::Extrude,
        Command::Revolve,
        Command::Fillet,
        Command::Chamfer,
        Command::Union,
        Command::Subtract,
        Command::Intersect,
        Command::Move,
        Command::Rotate,
        Command::Scale,
        Command::MirrorX,
        Command::MirrorY,
        Command::MirrorZ,
        Command::ViewTop,
        Command::ViewFront,
        Command::ViewRight,
        Command::ViewIso,
        Command::ToggleOrtho,
        Command::FrameSelection,
        Command::ToggleGrid,
        Command::ToggleEdges,
        Command::ToggleXray,
        Command::ToggleSnap,
        Command::Palette,
        Command::Help,
    ];

    pub fn label(self) -> &'static str {
        use Command::*;
        match self {
            New => "New document",
            Open => "Open…",
            Save => "Save",
            SaveAs => "Save as…",
            ImportStep => "Import STEP…",
            ExportStep => "Export STEP…",
            ExportStl => "Export STL…",
            ExportObj => "Export OBJ…",
            Undo => "Undo",
            Redo => "Redo",
            Delete => "Delete",
            Duplicate => "Duplicate",
            SelectAll => "Select all",
            DeselectAll => "Deselect all",
            Hide => "Hide selected",
            UnhideAll => "Unhide all",
            ModeVertex => "Select vertices",
            ModeEdge => "Select edges",
            ModeFace => "Select faces",
            ModeObject => "Select solids",
            Box => "Box",
            Cylinder => "Cylinder",
            Cone => "Cone",
            Sphere => "Sphere",
            Torus => "Torus",
            Curve => "Curve (polyline)",
            Rectangle => "Rectangle",
            Circle => "Circle",
            Extrude => "Extrude face or curves",
            Revolve => "Revolve curves",
            Fillet => "Fillet edges",
            Chamfer => "Chamfer edges",
            Union => "Boolean union",
            Subtract => "Boolean subtract",
            Intersect => "Boolean intersect",
            Move => "Move",
            Rotate => "Rotate",
            Scale => "Scale",
            MirrorX => "Mirror X",
            MirrorY => "Mirror Y",
            MirrorZ => "Mirror Z",
            ViewTop => "View top",
            ViewFront => "View front",
            ViewRight => "View right",
            ViewIso => "View isometric",
            ToggleOrtho => "Toggle orthographic",
            FrameSelection => "Frame selection",
            ToggleGrid => "Toggle grid",
            ToggleEdges => "Toggle edges",
            ToggleXray => "Toggle x-ray",
            ToggleSnap => "Toggle snapping",
            Palette => "Command palette",
            Help => "Help",
        }
    }

    pub fn shortcut_text(self) -> &'static str {
        use Command::*;
        match self {
            New => "Ctrl+N",
            Open => "Ctrl+O",
            Save => "Ctrl+S",
            SaveAs => "Ctrl+Shift+S",
            Undo => "Ctrl+Z",
            Redo => "Ctrl+Shift+Z",
            Delete => "Del / X",
            Duplicate => "Ctrl+D",
            SelectAll => "Ctrl+A",
            DeselectAll => "Esc",
            Hide => "H",
            UnhideAll => "Alt+H",
            ModeVertex => "1",
            ModeEdge => "2",
            ModeFace => "3",
            ModeObject => "4",
            Box => "B",
            Cylinder => "C",
            Cone => "Shift+C",
            Sphere => "Shift+S",
            Torus => "Shift+T",
            Curve => "L",
            Rectangle => "Shift+R",
            Circle => "Shift+O",
            Extrude => "E",
            Revolve => "V",
            Fillet => "Shift+F",
            Chamfer => "Shift+K",
            Union => "Shift+U",
            Subtract => "Shift+D",
            Intersect => "Shift+I",
            Move => "G",
            Rotate => "R",
            Scale => "S",
            ViewTop => "Numpad 7 / T",
            ViewFront => "Numpad 1",
            ViewRight => "Numpad 3",
            ViewIso => "Numpad 0",
            ToggleOrtho => "Numpad 5 / O",
            FrameSelection => "F",
            ToggleXray => "Alt+Z",
            Palette => "Space",
            Help => "F1",
            _ => "",
        }
    }
}

pub fn shortcut(key: Key, m: Modifiers) -> Option<Command> {
    use Command::*;
    let ctrl = m.ctrl || m.command;
    Some(match (key, ctrl, m.shift, m.alt) {
        (Key::N, true, _, _) => New,
        (Key::O, true, _, _) => Open,
        (Key::S, true, true, _) => SaveAs,
        (Key::S, true, false, _) => Save,
        (Key::Z, true, true, _) => Redo,
        (Key::Y, true, _, _) => Redo,
        (Key::Z, true, false, _) => Undo,
        (Key::D, true, _, _) => Duplicate,
        (Key::A, true, _, _) => SelectAll,
        (Key::Delete, _, _, _) | (Key::X, false, false, false) => Delete,
        (Key::Escape, _, _, _) => DeselectAll,
        (Key::H, false, _, true) => UnhideAll,
        (Key::H, false, _, false) => Hide,
        (Key::Num1, false, _, _) => ModeVertex,
        (Key::Num2, false, _, _) => ModeEdge,
        (Key::Num3, false, _, _) => ModeFace,
        (Key::Num4, false, _, _) => ModeObject,
        (Key::B, false, false, false) => Box,
        (Key::C, false, true, false) => Cone,
        (Key::C, false, false, false) => Cylinder,
        (Key::S, false, true, false) => Sphere,
        (Key::T, false, true, false) => Torus,
        (Key::E, false, false, false) => Extrude,
        (Key::L, false, false, false) => Curve,
        (Key::R, false, true, false) => Rectangle,
        (Key::O, false, true, false) => Circle,
        (Key::V, false, false, false) => Revolve,
        (Key::F, false, true, false) => Fillet,
        (Key::K, false, true, false) => Chamfer,
        (Key::U, false, true, false) => Union,
        (Key::D, false, true, false) => Subtract,
        (Key::I, false, true, false) => Intersect,
        (Key::G, false, false, false) => Move,
        (Key::R, false, false, false) => Rotate,
        (Key::S, false, false, false) => Scale,
        (Key::T, false, false, false) => ViewTop,
        (Key::O, false, false, false) => ToggleOrtho,
        (Key::F, false, false, false) => FrameSelection,
        (Key::Z, false, false, true) => ToggleXray,
        (Key::Space, false, false, false) => Palette,
        (Key::F1, _, _, _) => Help,
        _ => return None,
    })
}

pub fn run(app: &mut DuctileApp, cmd: Command) {
    use Command::*;
    match cmd {
        New => {
            app.doc.clear();
            app.cancel_tool();
            app.status = "New document".into();
        }
        Open => {
            if let Some(path) = rfd::FileDialog::new().add_filter("Ductile", &["ductile"]).add_filter("STEP", &["step", "stp"]).pick_file() {
                open_file(app, &path);
            }
        }
        Save => match app.doc.path.clone() {
            Some(p) => save_file(app, &p),
            None => run(app, SaveAs),
        },
        SaveAs => {
            if let Some(mut path) = rfd::FileDialog::new().add_filter("Ductile", &["ductile"]).save_file() {
                if path.extension().is_none() {
                    path.set_extension("ductile");
                }
                save_file(app, &path);
            }
        }
        ImportStep => {
            if let Some(path) = rfd::FileDialog::new().add_filter("STEP", &["step", "stp"]).pick_file() {
                app.doc.checkpoint("Import STEP");
                app.status = match import_step(&mut app.doc, &path) {
                    Ok(msg) => {
                        app.frame_selection();
                        msg
                    }
                    Err(e) => format!("STEP import failed: {e}"),
                };
            }
        }
        ExportStep => {
            if let Some(mut path) = rfd::FileDialog::new().add_filter("STEP", &["step", "stp"]).save_file() {
                if path.extension().is_none() {
                    path.set_extension("step");
                }
                app.status = match export_step(&app.doc, &path) {
                    Ok(n) => format!("Exported {n} solid(s) to {}", path.display()),
                    Err(e) => format!("STEP export failed: {e}"),
                };
            }
        }
        ExportStl | ExportObj => {
            let ext = if cmd == ExportStl { "stl" } else { "obj" };
            if let Some(mut path) = rfd::FileDialog::new().add_filter(ext.to_uppercase(), &[ext]).save_file() {
                if path.extension().is_none() {
                    path.set_extension(ext);
                }
                app.status = match export_mesh(&app.doc, &path, cmd == ExportStl) {
                    Ok(()) => format!("Exported {}", path.display()),
                    Err(e) => format!("Export failed: {e}"),
                };
            }
        }
        Undo => {
            app.cancel_tool();
            app.status = match app.doc.undo() {
                Some(l) => format!("Undo {l}"),
                None => "Nothing to undo".into(),
            }
        }
        Redo => {
            app.status = match app.doc.redo() {
                Some(l) => format!("Redo {l}"),
                None => "Nothing to redo".into(),
            }
        }
        Delete => {
            let ids = app.doc.selected_objects();
            if !ids.is_empty() && app.doc.mode == SelectMode::Object {
                app.doc.checkpoint("Delete");
                for id in ids {
                    app.doc.remove(id);
                }
            }
        }
        Duplicate => {
            let ids = app.doc.selected_objects();
            if !ids.is_empty() {
                app.doc.checkpoint("Duplicate");
                app.doc.selection.clear();
                for id in ids {
                    if let Some(o) = app.doc.object(id).cloned() {
                        let nid = app.doc.add_body(&o.name, (*o.body).clone());
                        app.doc.selection.insert(Pick::Object(nid));
                    }
                }
                app.start_tool(std::boxed::Box::new(TransformTool::new(TransformKind::Move, &app.doc)));
            }
        }
        SelectAll => {
            let mode = app.doc.mode;
            let all: Vec<Pick> = app
                .doc
                .objects
                .iter()
                .filter(|o| o.visible)
                .flat_map(|o| -> Vec<Pick> {
                    match mode {
                        SelectMode::Object => vec![Pick::Object(o.id)],
                        SelectMode::Face => o.body.face_ids().map(|f| Pick::Face(o.id, f)).collect(),
                        SelectMode::Edge => o.body.edge_ids().map(|e| Pick::Edge(o.id, e)).collect(),
                        SelectMode::Vertex => (0..o.body.vertices.len() as u32).map(|v| Pick::Vertex(o.id, ductile_brep::VertexId(v))).collect(),
                    }
                })
                .collect();
            app.doc.selection.extend(all);
        }
        DeselectAll => {
            if app.palette_open {
                app.palette_open = false;
            } else {
                app.doc.selection.clear();
            }
        }
        Hide => {
            for id in app.doc.selected_objects() {
                if let Some(o) = app.doc.objects.iter_mut().find(|o| o.id == id) {
                    o.visible = false;
                }
            }
            app.doc.selection.clear();
        }
        UnhideAll => {
            for o in app.doc.objects.iter_mut() {
                o.visible = true;
            }
        }
        ModeVertex => app.set_mode(SelectMode::Vertex),
        ModeEdge => app.set_mode(SelectMode::Edge),
        ModeFace => app.set_mode(SelectMode::Face),
        ModeObject => app.set_mode(SelectMode::Object),
        Box => app.start_tool(std::boxed::Box::new(BoxTool::new())),
        Cylinder => app.start_tool(std::boxed::Box::new(RoundTool::new(RoundKind::Cylinder))),
        Cone => app.start_tool(std::boxed::Box::new(RoundTool::new(RoundKind::Cone))),
        Sphere => app.start_tool(std::boxed::Box::new(RoundTool::new(RoundKind::Sphere))),
        Torus => app.start_tool(std::boxed::Box::new(RoundTool::new(RoundKind::Torus))),
        Extrude => match ExtrudeTool::new(&app.doc) {
            Some(t) => app.start_tool(std::boxed::Box::new(t)),
            None => app.status = "Select a planar face (face mode: 3) or closed planar curves to extrude".into(),
        },
        Curve => app.start_tool(std::boxed::Box::new(CurveTool::new(CurveKind::Polyline))),
        Rectangle => app.start_tool(std::boxed::Box::new(CurveTool::new(CurveKind::Rectangle))),
        Circle => app.start_tool(std::boxed::Box::new(CurveTool::new(CurveKind::Circle))),
        Revolve => match RevolveTool::new(&app.doc) {
            Some(t) => app.start_tool(std::boxed::Box::new(t)),
            None => app.status = "Select closed planar curves to revolve".into(),
        },
        Fillet | Chamfer => match FilletTool::new(&app.doc, cmd == Chamfer) {
            Some(t) => app.start_tool(std::boxed::Box::new(t)),
            None => app.status = "Select edges to blend (edge mode: 2)".into(),
        },
        Union => boolean_selected(app, BoolOp::Union),
        Subtract => boolean_selected(app, BoolOp::Difference),
        Intersect => boolean_selected(app, BoolOp::Intersection),
        Move | Rotate | Scale => {
            let kind = match cmd {
                Move => TransformKind::Move,
                Rotate => TransformKind::Rotate,
                _ => TransformKind::Scale,
            };
            let t = TransformTool::new(kind, &app.doc);
            if t.has_targets() {
                app.start_tool(std::boxed::Box::new(t));
            } else {
                app.status = "Select something to transform first".into();
            }
        }
        MirrorX | MirrorY | MirrorZ => {
            let ids = app.doc.selected_objects();
            if ids.is_empty() {
                app.status = "Select something to mirror first".into();
                return;
            }
            let s = match cmd {
                MirrorX => DVec3::new(-1.0, 1.0, 1.0),
                MirrorY => DVec3::new(1.0, -1.0, 1.0),
                _ => DVec3::new(1.0, 1.0, -1.0),
            };
            let o = app.cplane.frame.origin;
            let m = DMat4::from_translation(o) * DMat4::from_scale(s) * DMat4::from_translation(-o);
            app.doc.checkpoint("Mirror");
            app.doc.selection.clear();
            for id in ids {
                if let Some(ob) = app.doc.object(id).cloned() {
                    let nid = app.doc.add_body(&ob.name, ob.body.transformed(&m));
                    app.doc.selection.insert(Pick::Object(nid));
                }
            }
        }
        ViewTop => app.camera.set_view(DVec3::Z),
        ViewFront => app.camera.set_view(-DVec3::Y),
        ViewRight => app.camera.set_view(DVec3::X),
        ViewIso => app.camera.set_view(DVec3::new(1.0, -1.0, 0.8)),
        ToggleOrtho => app.toggle_projection(),
        FrameSelection => app.frame_selection(),
        ToggleGrid => app.show_grid = !app.show_grid,
        ToggleEdges => app.show_edges = !app.show_edges,
        ToggleXray => app.toggle_xray(),
        ToggleSnap => app.snap_enabled = !app.snap_enabled,
        Palette => {
            app.palette_open = true;
            app.palette_query.clear();
        }
        Help => app.show_help = !app.show_help,
    }
}

/// Combines the selected solids: the oldest one is the target, the others
/// are tools consumed by the operation.
fn boolean_selected(app: &mut DuctileApp, op: BoolOp) {
    let ids = app.doc.selected_objects();
    if ids.len() < 2 {
        app.status = "Select a target solid and at least one tool solid".into();
        return;
    }
    let target = ids[0];
    let Some(mut body) = app.doc.object(target).map(|o| (*o.body).clone()) else { return };
    let name = match op {
        BoolOp::Union => "Union",
        BoolOp::Difference => "Subtract",
        BoolOp::Intersection => "Intersect",
    };
    for id in &ids[1..] {
        let Some(tool) = app.doc.object(*id).map(|o| o.body.clone()) else { continue };
        match boolean(&body, &tool, op) {
            Ok(b) => body = b,
            Err(BoolError::Empty) => {
                app.status = format!("{name}: the result is empty");
                return;
            }
            Err(e) => {
                app.status = format!("{name} failed: {e}");
                return;
            }
        }
    }
    app.doc.checkpoint(name);
    app.doc.replace_body(target, body);
    for id in &ids[1..] {
        app.doc.remove(*id);
    }
    app.doc.selection.clear();
    app.doc.selection.insert(Pick::Object(target));
    app.status = format!("{name} done");
}

fn is_step(path: &std::path::Path) -> bool {
    path.extension().and_then(|e| e.to_str()).is_some_and(|e| e.eq_ignore_ascii_case("step") || e.eq_ignore_ascii_case("stp"))
}

/// Adds the solids of a STEP file to the document and selects them;
/// returns a status line that mentions anything that was skipped.
pub fn import_step(doc: &mut Document, path: &std::path::Path) -> Result<String, String> {
    let s = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
    let imp = ductile_io::step_read::read_step(&s).map_err(|e| e.to_string())?;
    if imp.bodies.is_empty() {
        return Err(imp.warnings.first().cloned().unwrap_or_else(|| "no solids found".into()));
    }
    doc.selection.clear();
    let n = imp.bodies.len();
    for b in imp.bodies {
        let id = doc.add_body(&b.name, b.body);
        doc.selection.insert(Pick::Object(id));
    }
    let mut msg = format!("Imported {n} body(ies) from {}", path.display());
    if let Some(w) = imp.warnings.first() {
        msg += &format!(" ({} warning(s): {w})", imp.warnings.len());
    }
    Ok(msg)
}

/// Replaces the document's objects with those of a native or STEP file.
pub fn load_into(doc: &mut Document, path: &std::path::Path) -> Result<(), String> {
    if is_step(path) {
        doc.clear();
        return import_step(doc, path).map(|_| ());
    }
    let s = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
    let nd = ductile_io::native::NativeDocument::from_json(&s).map_err(|e| e.to_string())?;
    doc.clear();
    for o in nd.objects {
        let id = doc.add_body(&o.name, o.body);
        if let Some(ob) = doc.objects.iter_mut().find(|x| x.id == id) {
            ob.name = o.name;
            ob.visible = o.visible;
            if let Some(c) = o.color {
                ob.color = c;
            }
        }
    }
    Ok(())
}

pub fn open_file(app: &mut DuctileApp, path: &std::path::Path) {
    match load_into(&mut app.doc, path) {
        Ok(()) => {
            app.doc.path = (!is_step(path)).then(|| path.to_path_buf());
            app.doc.dirty = false;
            app.frame_selection();
            app.status = format!("Opened {}", path.display());
        }
        Err(e) => app.status = format!("Could not open {}: {e}", path.display()),
    }
}

pub fn save_file(app: &mut DuctileApp, path: &std::path::Path) {
    let nd = ductile_io::native::NativeDocument::new(
        app.doc
            .objects
            .iter()
            .map(|o| ductile_io::native::NativeObject {
                name: o.name.clone(),
                body: (*o.body).clone(),
                color: Some(o.color),
                visible: o.visible,
            })
            .collect(),
    );
    let res = nd.to_json().map_err(|e| e.to_string()).and_then(|s| std::fs::write(path, s).map_err(|e| e.to_string()));
    match res {
        Ok(()) => {
            app.doc.path = Some(path.to_path_buf());
            app.doc.dirty = false;
            app.status = format!("Saved {}", path.display());
        }
        Err(e) => app.status = format!("Save failed: {e}"),
    }
}

/// Writes the visible solids to a STEP file; returns how many were written.
pub fn export_step(doc: &Document, path: &std::path::Path) -> Result<usize, String> {
    let objs: Vec<(&str, &ductile_brep::Body)> = doc.objects.iter().filter(|o| o.visible).map(|o| (o.name.as_str(), o.body.as_ref())).collect();
    let name = path.file_stem().and_then(|s| s.to_str()).unwrap_or("Ductile part");
    let mut f = std::io::BufWriter::new(std::fs::File::create(path).map_err(|e| e.to_string())?);
    ductile_io::step::write_step(&mut f, name, &objs).map_err(|e| e.to_string())
}

fn export_mesh(doc: &Document, path: &std::path::Path, stl: bool) -> Result<(), String> {
    let objs: Vec<_> = doc.objects.iter().filter(|o| o.visible).collect();
    let mut f = std::io::BufWriter::new(std::fs::File::create(path).map_err(|e| e.to_string())?);
    if stl {
        let meshes: Vec<_> = objs.iter().map(|o| o.mesh.as_ref()).collect();
        ductile_io::mesh::write_stl(&mut f, &meshes).map_err(|e| e.to_string())
    } else {
        let meshes: Vec<_> = objs.iter().map(|o| (o.name.as_str(), o.mesh.as_ref())).collect();
        ductile_io::mesh::write_obj(&mut f, &meshes).map_err(|e| e.to_string())
    }
}

/// A small scene used for screenshots and first launch demos.
pub fn demo_scene(doc: &mut Document) {
    use ductile_brep::ops::fillet::{blend_edges, Blend};
    let at = |x: f64, y: f64, z: f64| Frame { origin: DVec3::new(x, y, z), ..Frame::WORLD };
    let op = |a: ductile_brep::Body, b: ductile_brep::Body, o: BoolOp| boolean(&a, &b, o).unwrap_or(a);
    // A machined block: boss, through hole, flush pocket and a spherical notch.
    let mut part = make_box(&Frame::WORLD, DVec3::new(8.0, 6.0, 3.0));
    part = op(part, make_cylinder(&at(2.2, 3.0, 3.0), 1.6, 1.2), BoolOp::Union);
    part = op(part, make_cylinder(&at(2.2, 3.0, -1.0), 0.8, 6.0), BoolOp::Difference);
    part = op(part, make_box(&at(4.5, 1.0, 2.0), DVec3::new(2.5, 4.0, 1.0)), BoolOp::Difference);
    part = op(part, make_sphere(DVec3::new(8.0, 0.0, 3.0), 1.4), BoolOp::Difference);
    // Round the boss rim and the block's vertical corners.
    let pick = |b: &ductile_brep::Body, f: &dyn Fn(&ductile_brep::topo::Edge) -> bool| b.edge_ids().filter(|e| f(b.edge(*e))).collect::<Vec<_>>();
    let rim = pick(&part, &|e| matches!(e.curve, ductile_geom::Curve3::Circle { .. }) && e.start().z > 4.0);
    part = blend_edges(&part, &rim, Blend::Fillet(0.3)).unwrap_or(part);
    let verticals = pick(&part, &|e| e.curve.is_linear() && (e.start() - e.end()).normalize().z.abs() > 0.999 && [0.0, 8.0].contains(&e.start().x) && [0.0, 6.0].contains(&e.start().y) && e.start().distance(e.end()) > 2.9);
    part = blend_edges(&part, &verticals, Blend::Fillet(0.6)).unwrap_or(part);
    doc.add_body("Block", part);
    // A fully rounded box.
    let all = |b: &ductile_brep::Body| b.edge_ids().collect::<Vec<_>>();
    let rb = make_box(&at(-7.0, 11.0, 0.0), DVec3::new(5.0, 3.5, 2.5));
    let rb = blend_edges(&rb, &all(&rb), Blend::Fillet(0.6)).unwrap_or(rb);
    doc.add_body("Rounded box", rb);
    // Capsule end: cylinder with a hemispherical cap.
    let cap = op(make_cylinder(&at(13.0, 3.0, 0.0), 2.5, 4.0), make_sphere(DVec3::new(13.0, 3.0, 4.0), 2.5), BoolOp::Union);
    doc.add_body("Capsule", cap);
    // Sphere with a cylindrical bore and a flat.
    let mut ball = make_sphere(DVec3::new(4.0, 12.0, 3.0), 3.0);
    ball = op(ball, make_cylinder(&Frame::new(DVec3::new(0.0, 12.0, 3.0), DVec3::X, DVec3::Y), 1.0, 8.0), BoolOp::Difference);
    ball = op(ball, make_box(&at(0.0, 8.0, 5.0), DVec3::new(8.0, 8.0, 2.0)), BoolOp::Difference);
    doc.add_body("Ball", ball);
    doc.add_body("Torus", make_torus(&at(14.0, 12.0, 1.0), 3.0, 1.0));
    doc.add_body("Cone", make_frustum(&at(-6.0, 4.0, 0.0), 2.5, 0.0, 5.0));
    // A pulley revolved three quarters of a turn from its profile curve.
    {
        use ductile_brep::profile::{wire_chains, wire_from_segments, Segment};
        let c = DVec3::new(-8.0, -3.0, 0.0);
        let pts = [(0.6, 0.0), (3.0, 0.0), (3.0, 0.4), (2.2, 1.0), (3.0, 1.6), (3.0, 2.0), (0.6, 2.0)];
        let q: Vec<DVec3> = pts.iter().map(|(r, z)| c + DVec3::new(*r, 0.0, *z)).collect();
        let segs: Vec<Segment> = (0..q.len()).map(|i| Segment::line(q[i], q[(i + 1) % q.len()])).collect();
        let wire = wire_from_segments(&segs);
        if let Some(l) = wire_chains(&wire).into_iter().next() {
            if let Some(b) = ductile_brep::ops::revolve::revolve(&l, c, DVec3::Z, 1.5 * std::f64::consts::PI) {
                doc.add_body("Pulley", b);
            }
        }
        doc.add_body("Pulley profile", wire.transformed(&ductile_geom::DMat4::from_translation(DVec3::new(0.0, -0.5, 0.0))));
    }
    doc.dirty = false;
}
