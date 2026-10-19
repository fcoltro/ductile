// SPDX-License-Identifier: GPL-3.0-only
//! Named commands: what the toolbar, menus, palette and shortcuts invoke.

use crate::app::DuctileApp;
use crate::render::Analysis;
use crate::document::{Document, Pick, SelectMode};
use crate::tools::{ArrayTool, BoxTool, CurveEditTool, TrimTool, delete_selected, duplicate_selected, join_selected, BridgeTool, ExtendTool, ThickenTool, CurveKind, CurveTool, ExtrudeTool, FilletTool, RevolveTool, RoundKind, RoundTool, ShellTool, OffsetMode, TransformKind, TransformTool};
use ductile_brep::ops::boolean::{boolean, BoolError, BoolOp};
use ductile_brep::primitives::*;
use ductile_geom::{DVec3, Frame};
use egui::{Key, Modifiers};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    New,
    Open,
    Save,
    SaveAs,
    ImportStep,
    ImportSvg,
    ImportMesh,
    ImportImage,
    ExportStep,
    ExportStl,
    ExportObj,
    ExportHiddenLine,
    Undo,
    Redo,
    Delete,
    Duplicate,
    Instance,
    DeleteRedundant,
    CompleteEdge,
    RemoveFillets,
    OffsetEdge,
    ConstrainedSurface,
    SelectPrevious,
    UnwrapFace,
    MakeUnique,
    SelectAll,
    DeselectAll,
    Hide,
    Isolate,
    UnhideAll,
    Group,
    Ungroup,
    LockSelected,
    UnlockAll,
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
    Spline,
    Arc,
    Rectangle,
    Circle,
    Ellipse,
    Polygon,
    Slot,
    TangentArc,
    TangentCircle,
    Text,
    CenterRectangle,
    ThreePointRectangle,
    TwoPointCircle,
    ThreePointCircle,
    CenterArc,
    Helix,
    Extrude,
    Revolve,
    Loft,
    Patch,
    Thicken,
    BridgeSurface,
    MatchSurface,
    Cut,
    Imprint,
    Draft,
    Sweep,
    Pipe,
    Fillet,
    Chamfer,
    Shell,
    OffsetSolid,
    OffsetFaces,
    OffsetCurve,
    JoinCurves,
    SplitCurve,
    ConvertVertex,
    DeformCurve,
    SlideCv,
    InsertKnot,
    Trim,
    Isoparam,
    Untrim,
    FullRound,
    RaiseSurfaceDegree,
    RebuildFace,
    Unjoin,
    FindBoundaryEdges,
    ReverseSheet,
    BridgeCurve,
    AlignCurve,
    ExtendCurve,
    ReverseCurve,
    RaiseDegree,
    RebuildCurve,
    IntersectionCurve,
    ProjectCurve,
    Union,
    Subtract,
    Intersect,
    Move,
    Rotate,
    Scale,
    LinearArray,
    RadialArray,
    CurveArray,
    Place,
    Dimension,
    DeleteDimensions,
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
    ToggleZebra,
    ToggleIsophote,
    ToggleDraft,
    ToggleFaceDirection,
    ToggleCurvature,
    ToggleSection,
    Measure,
    ToggleSnap,
    SetCPlane,
    SetCPlaneOnly,
    CPlaneView,
    Palette,
    Help,
    Repeat,
    InvertSelection,
    Boolean,
    Offset,
    Mirror,
}

impl Command {
    pub const ALL: &'static [Command] = &[
        Command::New,
        Command::Open,
        Command::Save,
        Command::SaveAs,
        Command::ImportStep,
        Command::ImportSvg,
        Command::ImportMesh,
        Command::ImportImage,
        Command::ExportStep,
        Command::ExportStl,
        Command::ExportObj,
        Command::ExportHiddenLine,
        Command::Undo,
        Command::Redo,
        Command::Delete,
        Command::Duplicate,
        Command::Instance,
        Command::DeleteRedundant,
        Command::CompleteEdge,
        Command::RemoveFillets,
        Command::OffsetEdge,
        Command::ConstrainedSurface,
        Command::SelectPrevious,
        Command::UnwrapFace,
        Command::MakeUnique,
        Command::SelectAll,
        Command::DeselectAll,
        Command::Hide,
        Command::Isolate,
        Command::UnhideAll,
        Command::Group,
        Command::Ungroup,
        Command::LockSelected,
        Command::UnlockAll,
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
        Command::Spline,
        Command::Arc,
        Command::Rectangle,
        Command::Circle,
        Command::Ellipse,
        Command::Polygon,
        Command::Slot,
        Command::TangentArc,
        Command::TangentCircle,
        Command::Text,
        Command::CenterRectangle,
        Command::ThreePointRectangle,
        Command::TwoPointCircle,
        Command::ThreePointCircle,
        Command::CenterArc,
        Command::Helix,
        Command::Extrude,
        Command::Revolve,
        Command::Loft,
        Command::Patch,
        Command::Thicken,
        Command::BridgeSurface,
        Command::MatchSurface,
        Command::Cut,
        Command::Imprint,
        Command::Draft,
        Command::Sweep,
        Command::Pipe,
        Command::Fillet,
        Command::Chamfer,
        Command::Shell,
        Command::OffsetSolid,
        Command::OffsetFaces,
        Command::OffsetCurve,
        Command::JoinCurves,
        Command::Trim,
        Command::SplitCurve,
        Command::ConvertVertex,
        Command::DeformCurve,
        Command::SlideCv,
        Command::InsertKnot,
        Command::Isoparam,
        Command::Untrim,
        Command::FullRound,
        Command::RaiseSurfaceDegree,
        Command::RebuildFace,
        Command::Unjoin,
        Command::FindBoundaryEdges,
        Command::ReverseSheet,
        Command::BridgeCurve,
        Command::AlignCurve,
        Command::ExtendCurve,
        Command::ReverseCurve,
        Command::RaiseDegree,
        Command::RebuildCurve,
        Command::IntersectionCurve,
        Command::ProjectCurve,
        Command::Union,
        Command::Subtract,
        Command::Intersect,
        Command::Move,
        Command::Rotate,
        Command::Scale,
        Command::LinearArray,
        Command::RadialArray,
        Command::CurveArray,
        Command::Place,
        Command::Dimension,
        Command::DeleteDimensions,
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
        Command::ToggleZebra,
        Command::ToggleIsophote,
        Command::ToggleDraft,
        Command::ToggleFaceDirection,
        Command::ToggleCurvature,
        Command::ToggleSection,
        Command::Measure,
        Command::ToggleSnap,
        Command::SetCPlane,
        Command::SetCPlaneOnly,
        Command::CPlaneView,
        Command::Palette,
        Command::Help,
        Command::Repeat,
        Command::InvertSelection,
        Command::Boolean,
        Command::Offset,
        Command::Mirror,
    ];

    pub fn label(self) -> &'static str {
        use Command::*;
        match self {
            New => "New document",
            Open => "Open…",
            Save => "Save",
            SaveAs => "Save as…",
            ImportStep => "Import STEP…",
            ImportSvg => "Import SVG curves…",
            ImportMesh => "Import mesh (OBJ, STL, 3MF)…",
            ImportImage => "Reference image (PNG, JPEG)…",
            ExportStep => "Export STEP…",
            ExportStl => "Export STL…",
            ExportObj => "Export OBJ…",
            ExportHiddenLine => "Export hidden line (SVG)…",
            Undo => "Undo",
            Redo => "Redo",
            Delete => "Delete",
            Duplicate => "Duplicate",
            Instance => "Instance (linked copy)",
            DeleteRedundant => "Delete redundant topology",
            CompleteEdge => "Complete edge",
            RemoveFillets => "Remove fillets from solid",
            OffsetEdge => "Offset edge (imprint)",
            ConstrainedSurface => "Constrained surface (through points)",
            SelectPrevious => "Select previous (inputs of the last operation)",
            UnwrapFace => "Unwrap face (lay flat)",
            MakeUnique => "Make instance unique",
            SelectAll => "Select all",
            DeselectAll => "Deselect all",
            Hide => "Hide selected",
            Isolate => "Isolate selected",
            UnhideAll => "Unhide all",
            Group => "Group selected",
            Ungroup => "Ungroup selected",
            LockSelected => "Lock selected",
            UnlockAll => "Unlock all",
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
            Spline => "Spline",
            Arc => "Arc (3 points)",
            Ellipse => "Ellipse",
            Polygon => "Polygon",
            Slot => "Slot",
            TangentArc => "Tangent arc",
            TangentCircle => "Tangent circle (touching two or three curves)",
            Text => "Text",
            CenterRectangle => "Rectangle from center",
            ThreePointRectangle => "Rectangle by three points",
            TwoPointCircle => "Circle by two points",
            ThreePointCircle => "Circle by three points",
            CenterArc => "Arc from center",
            Helix => "Helix",
            Extrude => "Extrude face or curves",
            Revolve => "Revolve curves",
            Loft => "Loft",
            Patch => "Patch surface",
            Thicken => "Thicken surface",
            BridgeSurface => "Bridge surface",
            MatchSurface => "Match surface",
            Cut => "Cut solid with curves",
            Imprint => "Imprint curves onto faces",
            Draft => "Draft faces",
            Sweep => "Sweep",
            Pipe => "Pipe",
            Fillet => "Fillet edges",
            Chamfer => "Chamfer edges",
            Shell => "Shell (hollow, open selected faces)",
            OffsetSolid => "Offset solid",
            OffsetFaces => "Offset faces",
            OffsetCurve => "Offset curve",
            JoinCurves => "Join curves or surfaces",
            Trim => "Trim curves",
            SplitCurve => "Split curve at a point",
            ConvertVertex => "Convert corners to control points",
            DeformCurve => "Deform curves onto a face",
            SlideCv => "Slide control points",
            InsertKnot => "Insert knot (add a control point)",
            Isoparam => "Isoparam (split a face along its surface's u or v)",
            Untrim => "Untrim (the whole surface under a face)",
            FullRound => "Full round fillet (rib tops)",
            RaiseSurfaceDegree => "Raise surface degree",
            RebuildFace => "Rebuild face (refit its surface)",
            Unjoin => "Unjoin (faces, bodies or curves into pieces)",
            FindBoundaryEdges => "Find boundary edges",
            ReverseSheet => "Reverse sheet",
        BridgeCurve => "Bridge curves",
            AlignCurve => "Align curve (G0 to G3 to another curve)",
            ExtendCurve => "Extend curve or surface",
            ReverseCurve => "Reverse curve",
            RaiseDegree => "Raise curve degree",
            RebuildCurve => "Rebuild curve",
            IntersectionCurve => "Intersection curves",
            ProjectCurve => "Project curves onto solids",
            Union => "Boolean union",
            Subtract => "Boolean subtract",
            Intersect => "Boolean intersect",
            Move => "Move",
            Rotate => "Rotate",
            Scale => "Scale",
            LinearArray => "Linear array",
            RadialArray => "Radial array",
            CurveArray => "Curve array (copies along a curve)",
            Place => "Place (copies stood on faces)",
            Dimension => "Dimension (distance between two points)",
            DeleteDimensions => "Delete all dimensions",
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
            ToggleZebra => "Toggle zebra stripes",
            ToggleIsophote => "Toggle isophote analysis",
            ToggleDraft => "Toggle draft analysis",
            ToggleFaceDirection => "Toggle face direction",
            ToggleCurvature => "Toggle curvature combs",
            ToggleSection => "Toggle section analysis",
            Measure => "Measure",
            ToggleSnap => "Toggle snapping",
            SetCPlane => "Construction plane from selection, and look at it",
            SetCPlaneOnly => "Construction plane from selection",
            CPlaneView => "Construction plane facing the view",
            Palette => "Command palette",
            Help => "Help",
            Repeat => "Repeat last command",
            InvertSelection => "Invert selection",
            Boolean => "Boolean (union, subtract, intersect or slice)",
            Offset => "Offset (curves, faces or solid)",
            Mirror => "Mirror across the construction plane's YZ",
        }
    }

    /// Stable name, as used in the keymap file.
    pub fn id(self) -> String {
        format!("{self:?}")
    }

    pub fn from_id(id: &str) -> Option<Command> {
        Command::ALL.iter().copied().find(|c| c.id().eq_ignore_ascii_case(id))
    }

    pub fn shortcut_text(self) -> String {
        crate::keymap::keymap().text(self)
    }
}

pub fn shortcut(key: Key, m: Modifiers) -> Option<Command> {
    crate::keymap::keymap().lookup(key, m)
}

pub fn run(app: &mut DuctileApp, cmd: Command) {
    use Command::*;
    // Shift+R repeats modeling commands, not navigation or selection.
    let fleeting = matches!(
        cmd,
        Repeat | Palette | Help | Undo | Redo | New | Open | Save | SaveAs | SelectAll | InvertSelection | DeselectAll | Hide | Isolate | UnhideAll | Group | Ungroup | LockSelected | UnlockAll | ModeVertex | ModeEdge | ModeFace | ModeObject | ViewTop | ViewFront | ViewRight | ViewIso | ToggleOrtho | FrameSelection | ToggleGrid | ToggleEdges | ToggleXray | ToggleZebra | ToggleIsophote | ToggleDraft | ToggleFaceDirection | ToggleCurvature | ToggleSection | Measure | ToggleSnap | SetCPlane | SetCPlaneOnly | CPlaneView
    );
    if !fleeting {
        app.last_command = Some(cmd);
    }
    match cmd {
        Repeat => match app.last_command {
            Some(c) => run(app, c),
            None => app.status = "Nothing to repeat yet".into(),
        },
        Boolean => match crate::tools::BooleanTool::new(&app.doc) {
            Some(t) => app.start_tool(std::boxed::Box::new(t)),
            None => app.status = "Select a target solid and at least one tool solid".into(),
        },
        Mirror => run(app, MirrorX),
        Offset => {
            let curves = app.doc.selected_objects().iter().any(|id| app.doc.object(*id).is_some_and(|o| o.body.kind == ductile_brep::BodyKind::Wire));
            let next = if curves {
                OffsetCurve
            } else if !app.doc.selected_faces().is_empty() {
                OffsetFaces
            } else {
                OffsetSolid
            };
            run(app, next);
        }
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
        ImportMesh => {
            if let Some(path) = rfd::FileDialog::new().add_filter("Mesh", &["obj", "stl", "3mf"]).pick_file() {
                app.status = match import_mesh(&mut app.doc, &path) {
                    Ok(n) => {
                        app.frame_selection();
                        format!("Imported {n} triangle(s) from {}", path.display())
                    }
                    Err(e) => format!("Mesh import failed: {e}"),
                };
            }
        }
        ImportImage => {
            if let Some(path) = rfd::FileDialog::new().add_filter("Image", &["png", "jpg", "jpeg"]).pick_file() {
                app.status = match add_reference_image(&mut app.doc, &path) {
                    Ok(()) => format!("Placed {} on the construction plane", path.display()),
                    Err(e) => format!("Could not open {}: {e}", path.display()),
                };
            }
        }
        ImportSvg => {
            if let Some(path) = rfd::FileDialog::new().add_filter("SVG", &["svg"]).pick_file() {
                app.status = match import_svg(&mut app.doc, &path) {
                    Ok(n) => {
                        app.frame_selection();
                        format!("Imported {n} curve(s) from {}", path.display())
                    }
                    Err(e) => format!("SVG import failed: {e}"),
                };
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
                    Ok(n) => format!("Exported {n} shape(s) to {}", path.display()),
                    Err(e) => format!("STEP export failed: {e}"),
                };
            }
        }
        ExportHiddenLine => {
            if let Some(mut path) = rfd::FileDialog::new().add_filter("SVG", &["svg"]).save_file() {
                if path.extension().is_none() {
                    path.set_extension("svg");
                }
                app.status = match export_hidden_line(&app.doc, &app.camera, &path, true) {
                    Ok(()) => format!("Exported the view's lines to {}", path.display()),
                    Err(e) => format!("Export failed: {e}"),
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
            if let Some(msg) = delete_selected(&mut app.doc) {
                app.status = msg;
            }
        }
        Duplicate => {
            let made = duplicate_selected(&mut app.doc);
            if !made.is_empty() {
                app.set_mode(SelectMode::Object);
                app.doc.selection.clear();
                app.doc.selection.extend(made.into_iter().map(Pick::Object));
                app.start_tool(std::boxed::Box::new(TransformTool::new(TransformKind::Move, &app.doc)));
            }
        }
        Instance => {
            if app.doc.selected_objects().is_empty() {
                app.status = "Instance: select objects to place again".into();
            } else {
                app.doc.checkpoint("Instance");
                let made = app.doc.instance_selected();
                app.set_mode(SelectMode::Object);
                app.doc.selection.clear();
                app.doc.selection.extend(made.into_iter().map(Pick::Object));
                app.start_tool(std::boxed::Box::new(TransformTool::new(TransformKind::Move, &app.doc)));
            }
        }
        UnwrapFace => {
            let faces = app.doc.selected_faces();
            if faces.is_empty() {
                app.status = "Unwrap face: select faces (face mode)".into();
            } else {
                // Laid out on the construction plane, beside the model.
                let cp = crate::tools::current_cplane().frame;
                let bb = app.doc.bbox();
                let mut x = if bb.is_empty() { 0.0 } else { (bb.max - cp.origin).dot(cp.x) + 0.1 * bb.diagonal() };
                let mut made = Vec::new();
                for (o, f) in faces {
                    let Some(b) = app.doc.object(o).map(|ob| ob.body.clone()) else { continue };
                    let onto = Frame { origin: cp.origin + cp.x * x, ..cp };
                    if let Some(flat) = ductile_brep::ops::unwrap::unwrap_face(&b, f, &onto) {
                        // Shift so pieces sit side by side.
                        let fb = flat.bbox();
                        let lo = (fb.min - onto.origin).dot(cp.x);
                        let flat = flat.transformed(&ductile_geom::DMat4::from_translation(-cp.x * lo));
                        x += (fb.max - fb.min).dot(cp.x).abs() + 0.05 * bb.diagonal().max(1.0);
                        made.push(flat);
                    }
                }
                if made.is_empty() {
                    app.status = "Unwrap face: could not lay the faces flat".into();
                } else {
                    app.doc.checkpoint("Unwrap face");
                    app.doc.selection.clear();
                    let n = made.len();
                    for b in made {
                        let id = app.doc.add_body("Unwrapped", b);
                        app.doc.selection.insert(Pick::Object(id));
                    }
                    app.status = format!("Laid {n} face(s) flat");
                }
            }
        }
        SelectPrevious => {
            let n = app.doc.select_previous();
            app.status = if n == 0 { "Nothing earlier to select".into() } else { format!("Selected {n} previous item(s)") };
        }
        ConstrainedSurface => app.start_tool(std::boxed::Box::new(crate::tools::ConstrainedSurfaceTool::new())),
        OffsetEdge => match crate::tools::OffsetEdgeTool::new(&app.doc) {
            Some(t) => app.start_tool(std::boxed::Box::new(t)),
            None => app.status = "Offset edge: select edges of a solid".into(),
        },
        RemoveFillets => match crate::tools::RemoveFilletsTool::new(&app.doc) {
            Some(t) => app.start_tool(std::boxed::Box::new(t)),
            None => app.status = "Remove fillets: select solids".into(),
        },
        CompleteEdge => {
            let mut by_obj: std::collections::BTreeMap<crate::document::ObjectId, Vec<ductile_brep::EdgeId>> = Default::default();
            for (o, e) in app.doc.selected_edges() {
                by_obj.entry(o).or_default().push(e);
            }
            if by_obj.is_empty() {
                app.status = "Complete edge: select straight edges that stop partway across a face".into();
            } else {
                app.doc.checkpoint("Complete edge");
                let mut done = 0;
                for (id, edges) in by_obj {
                    let b = app.doc.object(id).unwrap().body.clone();
                    if let Ok(c) = ductile_brep::ops::cut::complete_edges(&b, &edges) {
                        if c.faces.len() > b.faces.len() {
                            done += c.faces.len() - b.faces.len();
                            app.doc.replace_body(id, c);
                        }
                    }
                }
                if done == 0 {
                    app.doc.undo();
                    app.status = "Complete edge: nothing to extend".into();
                } else {
                    app.status = format!("Completed edges: split {done} face(s)");
                }
            }
        }
        DeleteRedundant => {
            let ids: Vec<crate::document::ObjectId> = app.doc.selected_objects().into_iter().filter(|id| app.doc.object(*id).is_some_and(|o| o.body.kind != ductile_brep::BodyKind::Wire && o.body.mesh.is_none())).collect();
            if ids.is_empty() {
                app.status = "Delete redundant topology: select solids or sheets".into();
            } else {
                app.doc.checkpoint("Delete redundant topology");
                let mut gone = 0;
                for id in ids {
                    let b = app.doc.object(id).unwrap().body.clone();
                    let s = ductile_brep::ops::heal::simplify(&b);
                    if s.faces.len() + s.edges.len() < b.faces.len() + b.edges.len() {
                        gone += b.faces.len() + b.edges.len() - s.faces.len() - s.edges.len();
                        app.doc.replace_body(id, s);
                    }
                }
                if gone == 0 {
                    app.doc.undo();
                    app.status = "Nothing redundant to remove".into();
                } else {
                    app.status = format!("Removed {gone} redundant faces and edges");
                }
            }
        }
        MakeUnique => {
            app.doc.checkpoint("Make unique");
            let n = app.doc.make_unique_selected();
            if n == 0 {
                app.doc.undo();
                app.status = "Make unique: select instances".into();
            } else {
                app.status = format!("{n} instance(s) made unique");
            }
        }
        SelectAll | InvertSelection => {
            let mode = app.doc.mode;
            let all: Vec<Pick> = app
                .doc
                .objects
                .iter()
                .filter(|o| o.visible && !app.doc.is_locked(o.id))
                .flat_map(|o| -> Vec<Pick> {
                    match mode {
                        SelectMode::Object => vec![Pick::Object(o.id)],
                        SelectMode::Face => o.body.face_ids().map(|f| Pick::Face(o.id, f)).collect(),
                        SelectMode::Edge => o.body.edge_ids().map(|e| Pick::Edge(o.id, e)).collect(),
                        SelectMode::Vertex => (0..o.body.vertices.len() as u32).map(|v| Pick::Vertex(o.id, ductile_brep::VertexId(v))).collect(),
                    }
                })
                .collect();
            if cmd == InvertSelection {
                let was = std::mem::take(&mut app.doc.selection);
                app.doc.selection.extend(all.into_iter().filter(|p| !was.contains(p)));
            } else {
                app.doc.selection.extend(all);
            }
        }
        DeselectAll => {
            if app.palette_open {
                app.palette_open = false;
            } else {
                app.doc.selection.clear();
            }
        }
        Isolate => {
            let keep = app.doc.selected_objects();
            if !keep.is_empty() {
                for o in app.doc.objects.iter_mut() {
                    o.visible = keep.contains(&o.id);
                }
                app.status = format!("Isolated; {} shows everything again", UnhideAll.shortcut_text());
            }
        }
        JoinCurves if app.doc.selected_objects().iter().filter(|id| app.doc.object(**id).is_some_and(|o| o.body.kind == ductile_brep::BodyKind::Sheet)).count() >= 2 => {
            app.status = match crate::tools::join_sheets_selected(&mut app.doc) {
                Ok((id, solid)) => {
                    app.doc.selection.clear();
                    app.doc.selection.insert(Pick::Object(id));
                    if solid { "Surfaces joined into a closed solid".into() } else { "Surfaces joined".into() }
                }
                Err(e) => e,
            }
        }
        JoinCurves => {
            app.status = match join_selected(&mut app.doc) {
                Ok(id) => {
                    app.doc.selection.clear();
                    app.doc.selection.insert(Pick::Object(id));
                    "Curves joined".into()
                }
                Err(e) => e,
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
            for g in app.doc.groups.iter_mut() {
                g.visible = true;
            }
        }
        Group => {
            app.doc.checkpoint("Group");
            match app.doc.group_selected() {
                Some(g) => app.status = format!("Grouped as {}", app.doc.group(g).map_or("", |g| g.name.as_str())),
                None => {
                    app.doc.undo();
                    app.status = "Select objects to group".into();
                }
            }
        }
        Ungroup => {
            let mut gs: Vec<_> = app.doc.selected_objects().iter().filter_map(|id| app.doc.object(*id)?.group).collect();
            gs.sort();
            gs.dedup();
            if gs.is_empty() {
                app.status = "Select grouped objects to ungroup".into();
            } else {
                app.doc.checkpoint("Ungroup");
                for g in gs {
                    app.doc.ungroup(g);
                }
            }
        }
        LockSelected => {
            let ids = app.doc.selected_objects();
            for o in app.doc.objects.iter_mut().filter(|o| ids.contains(&o.id)) {
                o.locked = true;
            }
            app.doc.selection.clear();
            app.status = format!("Locked {} object(s); they show but can't be selected", ids.len());
        }
        UnlockAll => {
            for o in app.doc.objects.iter_mut() {
                o.locked = false;
            }
            for g in app.doc.groups.iter_mut() {
                g.locked = false;
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
        Helix => app.start_tool(std::boxed::Box::new(RoundTool::new(RoundKind::Helix))),
        Extrude => match ExtrudeTool::new(&app.doc) {
            Some(t) => app.start_tool(std::boxed::Box::new(t)),
            // Curved faces push and pull by offsetting.
            None => match ShellTool::new(&app.doc, OffsetMode::Faces) {
                Some(t) => app.start_tool(std::boxed::Box::new(t)),
                None => app.status = "Select a face (face mode: 3) or closed planar curves to extrude".into(),
            },
        },
        Curve => app.start_tool(std::boxed::Box::new(CurveTool::new(CurveKind::Polyline))),
        Rectangle => app.start_tool(std::boxed::Box::new(CurveTool::new(CurveKind::Rectangle))),
        Circle => app.start_tool(std::boxed::Box::new(CurveTool::new(CurveKind::Circle))),
        Spline => app.start_tool(std::boxed::Box::new(CurveTool::new(CurveKind::Spline))),
        Arc => app.start_tool(std::boxed::Box::new(CurveTool::new(CurveKind::Arc))),
        Ellipse => app.start_tool(std::boxed::Box::new(CurveTool::new(CurveKind::Ellipse))),
        Polygon => app.start_tool(std::boxed::Box::new(CurveTool::new(CurveKind::Polygon))),
        Slot => app.start_tool(std::boxed::Box::new(CurveTool::new(CurveKind::Slot))),
        Text => app.start_tool(std::boxed::Box::new(crate::tools::TextTool::new())),
        TangentArc => app.start_tool(std::boxed::Box::new(CurveTool::new(CurveKind::TangentArc))),
        TangentCircle => app.start_tool(std::boxed::Box::new(crate::tools::TangentCircleTool::new(&app.doc))),
        CenterRectangle => app.start_tool(std::boxed::Box::new(CurveTool::new(CurveKind::CenterRectangle))),
        ThreePointRectangle => app.start_tool(std::boxed::Box::new(CurveTool::new(CurveKind::ThreePointRectangle))),
        TwoPointCircle => app.start_tool(std::boxed::Box::new(CurveTool::new(CurveKind::TwoPointCircle))),
        ThreePointCircle => app.start_tool(std::boxed::Box::new(CurveTool::new(CurveKind::ThreePointCircle))),
        CenterArc => app.start_tool(std::boxed::Box::new(CurveTool::new(CurveKind::CenterArc))),
        Revolve => match RevolveTool::new(&app.doc) {
            Some(t) => app.start_tool(std::boxed::Box::new(t)),
            None => app.status = "Select closed planar curves to revolve".into(),
        },
        Fillet | Chamfer => match FilletTool::new(&app.doc, cmd == Chamfer) {
            Some(t) => app.start_tool(std::boxed::Box::new(t)),
            // With curves selected, Fillet rounds their corners.
            None => match (cmd == Fillet).then(|| CurveEditTool::new(&app.doc, false)).flatten() {
                Some(t) => app.start_tool(std::boxed::Box::new(t)),
                None => app.status = "Select edges to blend (edge mode: 2), or curves to round".into(),
            },
        },
        Isoparam => app.start_tool(std::boxed::Box::new(crate::tools::IsoparamTool::new())),
        RaiseSurfaceDegree => {
            app.status = match crate::tools::raise_selected_face_degree(&mut app.doc) {
                Ok(n) => format!("Raised the degree of {n} face(s)"),
                Err(e) => e,
            }
        }
        RebuildFace => match crate::tools::RebuildFaceTool::new(&app.doc) {
            Some(t) => app.start_tool(std::boxed::Box::new(t)),
            None => app.status = "Select faces to rebuild (face mode: 3)".into(),
        },
        FullRound => {
            app.status = match crate::tools::full_round_selected(&mut app.doc) {
                Ok(n) => format!("Rounded {n} face(s)"),
                Err(e) => e,
            }
        }
        Untrim => {
            app.status = match crate::tools::untrim_selected(&mut app.doc) {
                Ok(n) => format!("Untrimmed {n} face(s) into new sheets"),
                Err(e) => e,
            }
        }
        Unjoin => {
            app.status = crate::tools::unjoin_selected(&mut app.doc).unwrap_or_else(|e| e);
        }
        FindBoundaryEdges => {
            let n = crate::tools::find_boundary_edges(&mut app.doc);
            app.status = if n == 0 { "No open edges: every body is closed".into() } else { format!("{n} open edge(s) selected") };
        }
        ReverseSheet => {
            let n = crate::tools::reverse_selected_sheets(&mut app.doc);
            app.status = if n == 0 { "Select sheets to reverse".into() } else { format!("Reversed {n} sheet(s)") };
        }
        SlideCv => match crate::tools::SlideCvTool::new(&app.doc) {
            Some(t) => app.start_tool(std::boxed::Box::new(t)),
            None => app.status = "Slide CV: select curve control points (vertex mode)".into(),
        },
        DeformCurve => match crate::tools::DeformCurveTool::new(&app.doc) {
            Some(t) => app.start_tool(std::boxed::Box::new(t)),
            None => app.status = "Deform curve: select the curves first".into(),
        },
        SplitCurve | InsertKnot => match crate::tools::SplitCurveTool::new(&app.doc, cmd == InsertKnot) {
            Some(t) => app.start_tool(std::boxed::Box::new(t)),
            None => app.status = "There are no curves to split".into(),
        },
        Trim => match TrimTool::new(&app.doc) {
            Some(t) => app.start_tool(std::boxed::Box::new(t)),
            None => app.status = "There are no curves to trim".into(),
        },
        Cut => match crate::tools::cut_selected(&app.doc) {
            Ok((ids, pieces)) => {
                app.doc.checkpoint("Cut");
                app.doc.selection.clear();
                for (id, parts) in ids.into_iter().zip(pieces) {
                    let name = app.doc.object(id).map(|o| o.name.clone()).unwrap_or_else(|| "Solid".into());
                    app.doc.remove(id);
                    for (k, p) in parts.into_iter().enumerate() {
                        let nid = app.doc.add_body(&format!("{name} {}", k + 1), p);
                        app.doc.selection.insert(Pick::Object(nid));
                    }
                }
                app.status = "Cut done".into();
            }
            Err(e) => app.status = e,
        },
        Draft => match crate::tools::DraftTool::new(&app.doc, crate::tools::current_cplane().frame.z) {
            Some(t) => app.start_tool(std::boxed::Box::new(t)),
            None => app.status = "Draft: select planar faces of a solid (face mode, 3)".into(),
        },
        ProjectCurve => match crate::tools::project_selected(&app.doc) {
            Ok(wires) => {
                app.doc.checkpoint("Project curves");
                app.doc.selection.clear();
                for w in wires {
                    let id = app.doc.add_body("Projected curve", w);
                    app.doc.selection.insert(Pick::Object(id));
                }
                app.set_mode(SelectMode::Object);
                app.status = "Projected curves added".into();
            }
            Err(e) => app.status = e,
        },
        IntersectionCurve => match crate::tools::intersection_curves_selected(&app.doc) {
            Ok(wires) => {
                app.doc.checkpoint("Intersection curves");
                app.doc.selection.clear();
                for w in wires {
                    let id = app.doc.add_body("Intersection", w);
                    app.doc.selection.insert(Pick::Object(id));
                }
                app.set_mode(SelectMode::Object);
                app.status = "Intersection curves added".into();
            }
            Err(e) => app.status = e,
        },
        Imprint => match crate::tools::imprint_selected(&app.doc) {
            Ok((ids, bodies)) => {
                app.doc.checkpoint("Imprint");
                for (id, b) in ids.into_iter().zip(bodies) {
                    app.doc.replace_body(id, b);
                }
                app.status = "Imprinted; the split faces can now be picked on their own".into();
            }
            Err(e) => app.status = e,
        },
        Pipe => match crate::tools::PipeTool::new(&app.doc) {
            Some(t) => app.start_tool(std::boxed::Box::new(t)),
            None => app.status = "Select curves to run a pipe along".into(),
        },
        MatchSurface => match crate::tools::MatchSurfaceTool::new(&app.doc) {
            Some(t) => app.start_tool(std::boxed::Box::new(t)),
            None => app.status = "Select an open edge of a surface and the edge it should meet".into(),
        },
        BridgeSurface => match crate::tools::BridgeSurfaceTool::new(&app.doc) {
            Some(t) => app.start_tool(std::boxed::Box::new(t)),
            None => app.status = "Select two edges to bridge with a surface".into(),
        },
        Thicken => match ThickenTool::new(&app.doc) {
            Some(t) => app.start_tool(std::boxed::Box::new(t)),
            None => app.status = "Select surfaces to thicken".into(),
        },
        AlignCurve => app.start_tool(std::boxed::Box::new(crate::tools::AlignCurveTool::new(&app.doc))),
        BridgeCurve => match BridgeTool::new(&app.doc) {
            Some(t) => app.start_tool(std::boxed::Box::new(t)),
            None => app.status = "Select two curves (or two curve ends in vertex mode) to bridge".into(),
        },
        ConvertVertex => {
            let convert = |s: &[ductile_brep::profile::Segment]| {
                let closed = s.first().zip(s.last()).is_some_and(|(a, b)| a.start().distance(b.end()) < 1e-9);
                ductile_brep::curves::convert_corners(s, closed)
            };
            app.status = match crate::tools::edit_selected_curves(&mut app.doc, "Convert vertex", &convert) {
                Ok(n) => format!("Smoothed the corners of {n} curve(s)"),
                Err(e) => e,
            }
        }
        ReverseCurve => {
            app.status = match crate::tools::edit_selected_curves(&mut app.doc, "Reverse curve", &ductile_brep::curves::reverse_chain) {
                Ok(n) => format!("Reversed {n} curve(s)"),
                Err(e) => e,
            }
        }
        RaiseDegree => {
            app.status = match crate::tools::edit_selected_curves(&mut app.doc, "Raise degree", &|s| s.iter().map(ductile_brep::curves::raise_degree).collect()) {
                Ok(n) => format!("Raised the degree of {n} curve(s)"),
                Err(e) => e,
            }
        }
        RebuildCurve => match crate::tools::RebuildCurveTool::new(&app.doc) {
            Some(t) => app.start_tool(std::boxed::Box::new(t)),
            None => app.status = "Select curves to rebuild".into(),
        },
        ExtendCurve if crate::tools::ExtendSheetTool::new(&app.doc).is_some() => app.start_tool(std::boxed::Box::new(crate::tools::ExtendSheetTool::new(&app.doc).unwrap())),
        ExtendCurve => match ExtendTool::new(&app.doc) {
            Some(t) => app.start_tool(std::boxed::Box::new(t)),
            None => app.status = "Select open curves (or their ends in vertex mode), or open edges of surfaces, to extend".into(),
        },
        OffsetCurve => match CurveEditTool::new(&app.doc, true) {
            Some(t) => app.start_tool(std::boxed::Box::new(t)),
            None => app.status = "Select curves to offset".into(),
        },
        Shell | OffsetSolid | OffsetFaces => {
            let mode = match cmd {
                Shell => OffsetMode::Shell,
                OffsetSolid => OffsetMode::Solid,
                _ => OffsetMode::Faces,
            };
            match ShellTool::new(&app.doc, mode) {
                Some(t) => app.start_tool(std::boxed::Box::new(t)),
                None if mode == OffsetMode::Faces => app.status = "Select the faces to offset (face mode: 3)".into(),
                None => app.status = "Select a solid, or the faces to open (face mode: 3)".into(),
            }
        }
        Dimension => app.start_tool(std::boxed::Box::new(crate::tools::DimensionTool::new())),
        DeleteDimensions => {
            if app.doc.dimensions.is_empty() {
                app.status = "There are no dimensions".into();
            } else {
                app.doc.checkpoint("Delete dimensions");
                app.doc.dimensions.clear();
            }
        }
        Place => match crate::tools::PlaceTool::new(&app.doc) {
            Some(t) => app.start_tool(std::boxed::Box::new(t)),
            None => app.status = "Select the objects to place".into(),
        },
        CurveArray => match crate::tools::CurveArrayTool::new(&app.doc) {
            Some(t) => app.start_tool(std::boxed::Box::new(t)),
            None => app.status = "Select the objects to copy and the curve to copy them along".into(),
        },
        LinearArray | RadialArray => match ArrayTool::new(&app.doc, cmd == RadialArray) {
            Some(t) => app.start_tool(std::boxed::Box::new(t)),
            None => app.status = "Select objects to copy in an array".into(),
        },
        Union => boolean_selected(app, BoolOp::Union),
        Loft | Sweep | Patch => {
            let (name, res) = match cmd {
                Loft => ("Loft", crate::tools::loft_selected(&app.doc)),
                Patch => ("Patch", crate::tools::patch_selected(&app.doc)),
                _ => ("Sweep", crate::tools::sweep_selected(&app.doc)),
            };
            match res {
                Ok(b) => {
                    app.doc.checkpoint(name);
                    let id = app.doc.add_body(name, b);
                    app.doc.selection.clear();
                    app.doc.selection.insert(Pick::Object(id));
                    app.status = format!("{name} done");
                }
                Err(e) => app.status = e,
            }
        }
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
            let axis = match cmd {
                MirrorX => 0,
                MirrorY => 1,
                _ => 2,
            };
            if app.doc.selection.is_empty() {
                app.status = "Select something to mirror first".into();
                return;
            }
            app.doc.checkpoint("Mirror");
            match crate::tools::mirror_selected(&mut app.doc, &app.cplane.frame, axis) {
                Ok(msg) => app.status = msg,
                Err(e) => {
                    app.doc.undo();
                    app.status = e;
                }
            }
        }
        ViewTop => app.camera.set_view(DVec3::Z),
        ViewFront => app.camera.set_view(-DVec3::Y),
        ViewRight => app.camera.set_view(DVec3::X),
        ViewIso => app.camera.set_view(DVec3::new(1.0, -1.0, 0.8)),
        ToggleOrtho => app.toggle_projection(),
        SetCPlane | SetCPlaneOnly => {
            let view = (app.camera.eye() - app.camera.target).normalize();
            match crate::cplane::from_selection(&app.doc, view) {
                Some((fr, how)) => {
                    app.cplane.frame = fr;
                    if cmd == SetCPlane {
                        app.camera.target = fr.origin;
                        app.camera.set_view(fr.z);
                    }
                    app.status = format!("Construction plane {how}; {} with nothing selected goes back to the world plane", SetCPlane.shortcut_text());
                }
                None if app.doc.selection.is_empty() => {
                    app.cplane.frame = Frame::WORLD;
                    app.status = "Construction plane back on the world XY plane".into();
                }
                None => app.status = "Select a planar face, a face and an edge, two parallel faces, points or a flat curve".into(),
            }
        }
        CPlaneView => {
            app.cplane.frame = crate::cplane::facing(app.camera.eye() - app.camera.target, DVec3::ZERO);
            app.status = "Construction plane square to the view through the origin".into();
        }
        FrameSelection => app.frame_selection(),
        ToggleGrid => app.show_grid = !app.show_grid,
        ToggleEdges => app.show_edges = !app.show_edges,
        ToggleXray => app.toggle_xray(),
        ToggleZebra => app.toggle_analysis(Analysis::Zebra),
        ToggleIsophote => app.toggle_analysis(Analysis::Isophote),
        ToggleDraft => app.toggle_analysis(Analysis::Draft),
        ToggleFaceDirection => app.toggle_analysis(Analysis::Direction),
        Measure => match crate::measure::measure(&app.doc) {
            Some(m) => {
                app.status = m.text.clone();
                app.measurement = Some((m, app.doc.selection.clone()));
            }
            None => app.status = "Select one thing to measure its size, or two to measure between them".into(),
        },
        ToggleCurvature => {
            app.show_curvature = !app.show_curvature;
            if app.show_curvature && app.doc.selection.is_empty() {
                app.status = "Curvature combs show on the selected curves and edges".into();
            }
        }
        ToggleSection => app.toggle_section(),
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

/// Adds an SVG drawing's outlines as curves, one per subpath, in a group
/// named after the file and selected.
pub fn import_svg(doc: &mut Document, path: &std::path::Path) -> Result<usize, String> {
    let s = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
    let chains = ductile_io::svg::read_svg(&s).map_err(|e| e.to_string())?;
    if chains.is_empty() {
        return Err("the drawing has no shapes".into());
    }
    doc.checkpoint("Import SVG");
    doc.selection.clear();
    for c in &chains {
        let id = doc.add_body("Curve", ductile_brep::profile::wire_from_segments(c));
        doc.selection.insert(Pick::Object(id));
    }
    if let Some(g) = doc.group_selected() {
        let name = path.file_stem().map_or("SVG".to_string(), |s| s.to_string_lossy().into_owned());
        if let Some(g) = doc.groups.iter_mut().find(|x| x.id == g) {
            g.name = name;
        }
    }
    doc.mode = SelectMode::Object;
    Ok(chains.len())
}

/// Adds an OBJ, STL or 3MF file's meshes as reference mesh objects (an OBJ's
/// objects grouped under the file's name); returns the triangle count.
pub fn import_mesh(doc: &mut Document, path: &std::path::Path) -> Result<usize, String> {
    let stem = path.file_stem().map_or("Mesh".to_string(), |s| s.to_string_lossy().into_owned());
    let meshes = if is_ext(path, "stl") {
        vec![(stem.clone(), ductile_io::mesh::read_stl(&std::fs::read(path).map_err(|e| e.to_string())?).map_err(|e| e.to_string())?)]
    } else if is_ext(path, "3mf") {
        ductile_io::mesh::read_3mf(&std::fs::read(path).map_err(|e| e.to_string())?).map_err(|e| e.to_string())?
    } else {
        ductile_io::mesh::read_obj(&std::fs::read_to_string(path).map_err(|e| e.to_string())?).map_err(|e| e.to_string())?
    };
    doc.checkpoint("Import mesh");
    doc.selection.clear();
    let mut n = 0;
    for (name, m) in &meshes {
        n += m.triangles.len();
        let id = doc.add_body(if name.is_empty() { &stem } else { name }, ductile_brep::Body::from_mesh(m.clone()));
        doc.selection.insert(Pick::Object(id));
    }
    if meshes.len() > 1 {
        if let Some(g) = doc.group_selected() {
            if let Some(g) = doc.groups.iter_mut().find(|x| x.id == g) {
                g.name = stem;
            }
        }
    }
    doc.mode = SelectMode::Object;
    Ok(n)
}

fn is_ext(path: &std::path::Path, ext: &str) -> bool {
    path.extension().and_then(|e| e.to_str()).is_some_and(|e| e.eq_ignore_ascii_case(ext))
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
    if path.extension().is_some_and(|e| e.eq_ignore_ascii_case("svg")) {
        doc.clear();
        return import_svg(doc, path).map(|_| ());
    }
    if is_ext(path, "stl") || is_ext(path, "obj") || is_ext(path, "3mf") {
        doc.clear();
        return import_mesh(doc, path).map(|_| ());
    }
    let s = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
    let nd = ductile_io::native::NativeDocument::from_json(&s).map_err(|e| e.to_string())?;
    doc.clear();
    // Group ids are renumbered past the objects'.
    let mut gid = std::collections::HashMap::new();
    for g in &nd.groups {
        gid.insert(g.id, doc.new_group_id());
    }
    for g in &nd.groups {
        doc.groups.push(crate::document::Group {
            id: gid[&g.id],
            name: g.name.clone(),
            parent: g.parent.and_then(|p| gid.get(&p).copied()),
            visible: g.visible,
            locked: g.locked,
            open: true,
        });
    }
    doc.dimensions = nd.dimensions.iter().map(|[a, b, o]| crate::document::Dimension { a: DVec3::from_array(*a), b: DVec3::from_array(*b), offset: DVec3::from_array(*o) }).collect();
    for im in &nd.images {
        // Pictures are kept by path; a missing one is skipped.
        let path = std::path::PathBuf::from(&im.path);
        if add_reference_image(doc, &path).is_ok() {
            if let Some(i) = doc.images.last_mut() {
                let [o, x, z] = im.frame.map(DVec3::from_array);
                (i.frame, i.width, i.opacity, i.visible) = (Frame::new(o, z, x), im.width, im.opacity, im.visible);
            }
        }
    }
    for o in nd.objects {
        let id = doc.add_body(&o.name, o.body);
        if let Some(ob) = doc.objects.iter_mut().find(|x| x.id == id) {
            ob.name = o.name;
            ob.visible = o.visible;
            ob.locked = o.locked;
            if let Some([m, r]) = o.material {
                (ob.metalness, ob.roughness) = (m, r);
            }
            ob.group = o.group.and_then(|g| gid.get(&g).copied());
            if let Some(c) = o.color {
                ob.color = c;
            }
            ob.link = o.link.map(|(id, m)| crate::document::Link { id, placement: ductile_geom::DMat4::from_cols_array(&m) });
        }
    }
    Ok(())
}

pub fn open_file(app: &mut DuctileApp, path: &std::path::Path) {
    match load_into(&mut app.doc, path) {
        Ok(()) => {
            let native = !is_step(path) && !path.extension().is_some_and(|e| e.eq_ignore_ascii_case("svg"));
            app.doc.path = native.then(|| path.to_path_buf());
            app.doc.dirty = false;
            crate::recent::push(path);
            app.frame_selection();
            app.status = format!("Opened {}", path.display());
        }
        Err(e) => app.status = format!("Could not open {}: {e}", path.display()),
    }
}

/// Places a picture on the construction plane, centred on its origin,
/// sized to the scene (10 units across for an empty one).
pub fn add_reference_image(doc: &mut Document, path: &std::path::Path) -> Result<(), String> {
    let (px, size) = crate::refimage::load_rgba(path)?;
    let bb = doc.bbox();
    let width = if bb.is_empty() { 10.0 } else { bb.diagonal().max(1.0) };
    let id = doc.images.iter().map(|i| i.id).max().unwrap_or(0) + 1;
    let name = path.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_else(|| "Image".into());
    doc.images.push(crate::refimage::RefImage { id, name, path: path.to_path_buf(), pixels: std::sync::Arc::new(px), size, frame: crate::tools::current_cplane().frame, width, opacity: 0.6, visible: true });
    doc.dirty = true;
    Ok(())
}

/// Writes the document as a native file.
pub fn write_native(doc: &Document, path: &std::path::Path) -> Result<(), String> {
    let nd = ductile_io::native::NativeDocument::new(
        doc.objects
            .iter()
            .map(|o| ductile_io::native::NativeObject {
                name: o.name.clone(),
                body: (*o.body).clone(),
                color: Some(o.color),
                visible: o.visible,
                locked: o.locked,
                group: o.group,
                material: Some([o.metalness, o.roughness]),
                link: o.link.map(|l| (l.id, l.placement.to_cols_array())),
            })
            .collect(),
    );
    let mut nd = nd;
    nd.groups = doc
        .groups
        .iter()
        .map(|g| ductile_io::native::NativeGroup { id: g.id, name: g.name.clone(), parent: g.parent, visible: g.visible, locked: g.locked })
        .collect();
    nd.dimensions = doc.dimensions.iter().map(|d| [d.a.to_array(), d.b.to_array(), d.offset.to_array()]).collect();
    nd.images = doc
        .images
        .iter()
        .map(|i| ductile_io::native::NativeImage { path: i.path.to_string_lossy().into_owned(), frame: [i.frame.origin.to_array(), i.frame.x.to_array(), i.frame.z.to_array()], width: i.width, opacity: i.opacity, visible: i.visible })
        .collect();
    if let Some(dir) = path.parent().filter(|d| !d.as_os_str().is_empty()) {
        std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    }
    nd.to_json().map_err(|e| e.to_string()).and_then(|s| std::fs::write(path, s).map_err(|e| e.to_string()))
}

pub fn save_file(app: &mut DuctileApp, path: &std::path::Path) {
    match write_native(&app.doc, path) {
        Ok(()) => {
            app.doc.path = Some(path.to_path_buf());
            app.doc.dirty = false;
            crate::recent::push(path);
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

/// Writes the visible bodies' edges and silhouettes as seen by `camera` to an
/// SVG file, with lines behind surfaces dashed (when `hidden`) or left out.
pub fn export_hidden_line(doc: &Document, camera: &crate::camera::Camera, path: &std::path::Path, hidden: bool) -> Result<(), String> {
    let objs: Vec<_> = doc.objects.iter().filter(|o| o.visible).collect();
    let meshes: Vec<&ductile_brep::BodyMesh> = objs.iter().map(|o| o.mesh.as_ref()).collect();
    let diag = objs.iter().map(|o| o.body.bbox()).reduce(|a, b| a.union(&b)).map_or(1.0, |b| b.diagonal());
    let width = 1600.0;
    let view = ductile_io::hidden_line::View {
        view_proj: camera.view_proj(),
        eye: (camera.projection == crate::camera::Projection::Perspective).then(|| camera.eye()),
        forward: -camera.back(),
        size: ductile_geom::DVec2::new(width, (width / camera.aspect.max(1e-3)).round()),
    };
    let d = ductile_io::hidden_line::hidden_lines(&meshes, &view, 1e-3 * diag);
    std::fs::write(path, ductile_io::hidden_line::to_svg(&d, view.size, hidden)).map_err(|e| e.to_string())
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
    doc.dimensions.push(crate::document::Dimension { a: DVec3::ZERO, b: DVec3::new(8.0, 0.0, 0.0), offset: DVec3::new(0.0, -1.5, 0.0) });
    // A fully rounded box with curvature-continuous (G2) edges and corners.
    let all = |b: &ductile_brep::Body| b.edge_ids().collect::<Vec<_>>();
    let rb = make_box(&at(-7.0, 11.0, 0.0), DVec3::new(5.0, 3.5, 2.5));
    let rb = blend_edges(&rb, &all(&rb), Blend::Smooth(0.6)).unwrap_or(rb);
    doc.add_body("Rounded box", rb);
    // A soap bar: round the corners, then blend the top and bottom rims as
    // tangent chains in one go, all with curvature-continuous (G2) blends.
    let soap = make_box(&at(4.0, -11.0, 0.0), DVec3::new(6.0, 4.0, 2.0));
    let soap = blend_edges(&soap, &pick(&soap, &|e| (e.start() - e.end()).z.abs() > 1.0), Blend::Smooth(1.2)).map(|b| ductile_brep::ops::heal::compact(&b)).unwrap_or(soap);
    let rims = pick(&soap, &|e| (e.start().z - e.end().z).abs() < 1e-9 && (e.start().z.abs() < 1e-9 || (e.start().z - 2.0).abs() < 1e-9));
    let soap = blend_edges(&soap, &rims, Blend::Smooth(0.5)).unwrap_or(soap);
    doc.add_body("Soap", soap);
    // Variable-radius blends: a smooth one widening along the top front
    // edge, a round one tapering down the back corner.
    let wedge = make_box(&at(13.0, -12.0, 0.0), DVec3::new(5.0, 3.0, 2.5));
    let top_front = pick(&wedge, &|e| e.start().z > 2.4 && e.end().z > 2.4 && (e.start().y + 12.0).abs() < 1e-9 && (e.end().y + 12.0).abs() < 1e-9);
    let wedge = ductile_brep::ops::fillet::blend_edges_variable(&wedge, &top_front, 0.3, 1.4, true).unwrap_or(wedge);
    let back = pick(&wedge, &|e| (e.start() - e.end()).z.abs() > 2.4 && [e.start(), e.end()].iter().all(|p| (p.x - 18.0).abs() < 1e-9 && (p.y + 9.0).abs() < 1e-9));
    let wedge = ductile_brep::ops::fillet::blend_edges_variable(&wedge, &back, 0.2, 1.0, false).unwrap_or(wedge);
    doc.add_body("Variable blends", wedge);
    // Two curves joined by a curvature-continuous bridge, one of them
    // extended along its own shape.
    {
        use ductile_brep::curves::{bridge_curve, curve_end, extend_segment, Extension};
        use ductile_brep::profile::{wire_from_segments, Segment};
        use ductile_geom::{Curve3, NurbsCurve};
        let arc = Segment::new(Curve3::Circle { frame: at(-4.0, -12.0, 0.0), radius: 2.0 }, 3.14, 5.0);
        let pts = [(3.0, -15.0), (4.0, -13.0), (6.0, -14.5), (8.0, -13.0)].map(|(x, y)| DVec3::new(x, y, 0.0));
        let sp = NurbsCurve::interpolate(&pts, 3);
        let (t0, t1) = sp.domain();
        let spline = Segment::new(Curve3::Nurbs(sp), t0, t1);
        let bridge = bridge_curve(&curve_end(&arc, true), &curve_end(&spline, false), 2, [1.0, 1.0]);
        let (b0, b1) = bridge.domain();
        doc.add_body("Arc", wire_from_segments(&[arc]));
        doc.add_body("Bridge", wire_from_segments(&[Segment::new(Curve3::Nurbs(bridge), b0, b1)]));
        doc.add_body("Spline", wire_from_segments(&extend_segment(&spline, true, 1.2, Extension::Natural)));
    }
    // Surfaces: an extruded wave, a revolved bowl, a lofted sheet and a
    // Coons patch on a curved boundary.
    {
        use ductile_brep::ops::sheet::{extrude_sheet, loft_sheet, patch, revolve_sheet};
        use ductile_brep::profile::{Loop3, Segment};
        use ductile_geom::{Curve3, NurbsCurve};
        let spline = |pts: &[DVec3]| {
            let c = NurbsCurve::interpolate(pts, 3);
            let (t0, t1) = c.domain();
            Loop3 { segments: vec![Segment::new(Curve3::Nurbs(c), t0, t1)] }
        };
        let p = |x: f64, y: f64, z: f64| DVec3::new(x, y, z);
        let wave = spline(&[p(-14.0, -24.0, 0.0), p(-12.5, -22.5, 0.0), p(-11.0, -24.0, 0.0), p(-9.5, -22.5, 0.0)]);
        if let Some(b) = extrude_sheet(&[wave], p(0.0, 0.0, 3.0)) {
            doc.add_body("Wave surface", b);
        }
        let prof = spline(&[p(-4.0, -23.0, 0.2), p(-2.0, -23.0, 0.6), p(-1.0, -23.0, 2.0), p(-0.6, -23.0, 3.2)]);
        if let Some(b) = revolve_sheet(&[prof], p(-4.0, -23.0, 0.0), DVec3::Z, std::f64::consts::TAU) {
            doc.add_body("Bowl surface", b);
        }
        let l0 = spline(&[p(3.0, -26.0, 0.0), p(5.0, -25.5, 0.5), p(7.0, -26.0, 0.0)]);
        let l1 = spline(&[p(3.0, -23.5, 1.5), p(5.0, -23.0, 2.5), p(7.0, -23.5, 1.5)]);
        let l2 = spline(&[p(3.0, -21.0, 0.5), p(5.0, -21.5, 0.0), p(7.0, -21.0, 0.5)]);
        if let Some(b) = loft_sheet(&[l0, l1, l2]) {
            // Thickened into a solid shell.
            match ductile_brep::ops::sheet::thicken(&b, 0.25) {
                Ok(s) => doc.add_body("Thick loft", s),
                Err(_) => doc.add_body("Lofted surface", b),
            };
        }
        let c = |pts: &[DVec3]| spline(pts).segments.remove(0);
        let boundary = Loop3 {
            segments: vec![
                c(&[p(10.0, -26.0, 0.0), p(12.0, -26.5, 1.0), p(14.0, -26.0, 0.0)]),
                c(&[p(14.0, -26.0, 0.0), p(14.5, -24.0, 1.5), p(14.0, -22.0, 0.0)]),
                c(&[p(14.0, -22.0, 0.0), p(12.0, -21.5, 2.0), p(10.0, -22.0, 0.0)]),
                c(&[p(10.0, -22.0, 0.0), p(9.5, -24.0, 0.5), p(10.0, -26.0, 0.0)]),
            ],
        };
        if let Some(b) = patch(&boundary, &[]) {
            doc.add_body("Patch", b);
        }
        // A tube capped by a patch that meets it tangentially.
        let rim = Loop3 { segments: vec![Segment::new(Curve3::Circle { frame: at(18.0, -24.0, 0.0), radius: 1.5 }, 0.0, std::f64::consts::TAU)] };
        if let Some(tube) = extrude_sheet(&[rim], p(0.0, 0.0, 2.5)) {
            let top = tube.edge_ids().find(|e| tube.edge(*e).mid().z > 2.4 && tube.edge(*e).is_closed());
            let cap = top.and_then(|e| {
                let ed = tube.edge(e);
                let f = tube.edge_faces(e)[0].0;
                let lp = Loop3 { segments: vec![Segment::new(ed.curve.clone(), ed.t0, ed.t1)] };
                ductile_brep::ops::sheet::patch_g1(&lp, &[Some((tube.face(f).surface.clone(), p(18.0, -22.5, 1.0)))])
            });
            doc.add_body("Tube", tube);
            if let Some(c) = cap {
                doc.add_body("Tangent cap", c);
            }
        }
        // Two curved sheets joined by a curvature-continuous bridge.
        let a = spline(&[p(20.0, -32.0, 0.0), p(21.5, -32.0, 1.2), p(23.0, -32.0, 1.6)]);
        let b = spline(&[p(28.0, -32.0, 0.6), p(29.5, -32.0, 0.9), p(31.0, -32.0, 0.0)]);
        if let (Some(a), Some(b)) = (extrude_sheet(&[a], p(0.0, 3.0, 0.0)), extrude_sheet(&[b], p(0.0, 3.0, 0.0))) {
            use ductile_brep::ops::bridge::{bridge_surface, Continuity, Side};
            let edge_at = |s: &ductile_brep::Body, x: f64| s.edge_ids().find(|e| s.edge(*e).curve.is_linear() && (s.edge(*e).mid().x - x).abs() < 1e-6);
            if let (Some(ea), Some(eb)) = (edge_at(&a, 23.0), edge_at(&b, 28.0)) {
                let sa = Side::of_edge(&a, ea, p(28.0, -30.5, 0.6), Continuity::G2, 1.0);
                let sb = Side::of_edge(&b, eb, p(23.0, -30.5, 1.6), Continuity::G2, 1.0);
                if let Some(br) = bridge_surface(&sa, &sb) {
                    doc.add_body("Bridge surface", br);
                }
            }
            doc.add_body("Bridged sheet", a);
            doc.add_body("Bridged sheet", b);
        }
    }
    // A drafted block with a spline imprinted across its top.
    {
        use ductile_brep::profile::{Loop3, Segment};
        let bx = make_box(&at(26.0, -24.0, 0.0), DVec3::new(6.0, 4.0, 3.0));
        let sides: Vec<ductile_brep::FaceId> = bx.face_ids().filter(|f| bx.face_normal(*f, 0.5, 0.5).z.abs() < 0.5).collect();
        let drafted = ductile_brep::ops::draft::draft_faces(&bx, &sides, DVec3::new(0.0, 0.0, 0.0), DVec3::Z, 8f64.to_radians()).unwrap_or(bx);
        let n = ductile_geom::NurbsCurve::interpolate(&[DVec3::new(27.0, -25.0, 3.0), DVec3::new(28.0, -21.2, 3.0), DVec3::new(30.0, -23.2, 3.0), DVec3::new(31.0, -19.0, 3.0)], 3);
        let (t0, t1) = n.domain();
        let wave = Loop3 { segments: vec![Segment::new(ductile_geom::Curve3::Nurbs(n), t0, t1)] };
        let imprinted = ductile_brep::ops::cut::imprint_curves(&drafted, &[wave], DVec3::Z).unwrap_or(drafted);
        doc.add_body("Drafted block", imprinted);
    }
    // A spring: a circle swept along a helix.
    {
        use ductile_brep::profile::{Loop3, Segment};
        let axis = at(40.0, -22.0, 0.0);
        let h = ductile_brep::curves::helix(&axis, 2.0, 6.0, 3.0);
        let (t0, t1) = h.domain();
        let start = h.point(t0);
        let tan = h.derivs(t0, 1)[1].normalize();
        let ring = Loop3 { segments: vec![Segment::new(ductile_geom::Curve3::Circle { frame: Frame::new(start, tan, DVec3::X), radius: 0.4 }, 0.0, std::f64::consts::TAU)] };
        if let Some(b) = ductile_brep::ops::sweep::sweep(&ring, &[Segment::new(ductile_geom::Curve3::Nurbs(h), t0, t1)]) {
            doc.add_body("Spring", b);
        }
    }
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
    // A vase revolved from a spline profile: an exact NURBS surface of revolution.
    {
        use ductile_brep::profile::{Loop3, Segment};
        let c = DVec3::new(20.0, -6.0, 0.0);
        let pts: Vec<DVec3> = [(1.6, 0.0), (2.4, 1.5), (1.2, 4.0), (1.0, 5.2), (1.5, 6.5)].iter().map(|(r, z)| c + DVec3::new(*r, 0.0, *z)).collect();
        let sp = ductile_geom::NurbsCurve::interpolate(&pts, 3);
        let (t0, t1) = sp.domain();
        let top = c + DVec3::Z * 6.5;
        let l = Loop3 {
            segments: vec![
                Segment::line(c, pts[0]),
                Segment::new(ductile_geom::Curve3::Nurbs(sp), t0, t1),
                Segment::line(pts[4], top),
                Segment::line(top, c),
            ],
        };
        if let Some(b) = ductile_brep::ops::revolve::revolve_loop(&l, c, DVec3::Z) {
            // Hollow it out through the mouth, walls 0.2 thick, then round
            // both lip edges with rolling-ball fillets.
            let mouth = b.face_ids().find(|f| matches!(b.face(*f).surface, ductile_geom::Surface::Plane { .. }) && b.face_edges(*f).iter().all(|e| b.edge(*e).start().z > c.z + 6.0));
            let b = mouth.and_then(|f| ductile_brep::ops::offset::shell(&b, &[f], 0.2).ok()).unwrap_or(b);
            let lips: Vec<_> = b.edge_ids().filter(|e| b.edge(*e).is_closed() && b.edge(*e).start().z > c.z + 6.4).collect();
            let b = blend_edges(&b, &lips, Blend::Fillet(0.08)).unwrap_or(b);
            doc.add_body("Vase", b);
        }
    }
    // A loft from a circle to a rounded square, and a tube swept along a spline.
    {
        use ductile_brep::profile::{Loop3, Segment};
        use ductile_geom::Curve3;
        let c = DVec3::new(26.0, 4.0, 0.0);
        let circle = Loop3 { segments: vec![Segment::new(Curve3::Circle { frame: Frame { origin: c, ..Frame::WORLD }, radius: 2.5 }, 0.0, std::f64::consts::TAU)] };
        let mid = Loop3 { segments: vec![Segment::new(Curve3::Circle { frame: Frame { origin: c + DVec3::Z * 2.5, ..Frame::WORLD }, radius: 1.6 }, 0.0, std::f64::consts::TAU)] };
        let top = c + DVec3::Z * 5.0;
        let q = [top + DVec3::new(-2.0, -2.0, 0.0), top + DVec3::new(2.0, -2.0, 0.0), top + DVec3::new(2.0, 2.0, 0.0), top + DVec3::new(-2.0, 2.0, 0.0)];
        let square = Loop3 { segments: (0..4).map(|i| Segment::line(q[i], q[(i + 1) % 4])).collect() };
        if let Some(b) = ductile_brep::ops::loft::loft(&[circle, mid, square]) {
            let foot = b.edge_ids().find(|e| b.edge(*e).is_closed() && b.edge(*e).start().z < 0.1);
            let b = foot.and_then(|e| blend_edges(&b, &[e], Blend::Fillet(0.4)).ok()).unwrap_or(b);
            doc.add_body("Loft", b);
        }
        let pts = [DVec3::new(-14.0, 6.0, 0.6), DVec3::new(-10.0, 10.0, 1.5), DVec3::new(-6.0, 7.0, 3.0), DVec3::new(-2.0, 12.0, 2.0)];
        let path = ductile_geom::NurbsCurve::interpolate(&pts, 3);
        let (t0, t1) = path.domain();
        let tube = Loop3 { segments: vec![Segment::new(Curve3::Circle { frame: Frame::new(pts[0], path.tangent(t0), DVec3::Z), radius: 0.6 }, 0.0, std::f64::consts::TAU)] };
        if let Some(b) = ductile_brep::ops::sweep::sweep(&tube, &[Segment::new(Curve3::Nurbs(path), t0, t1)]) {
            doc.add_body("Tube", b);
        }
        // A pipe bent through sharp corners, mitred at each one.
        let bends = [DVec3::new(-16.0, -4.0, 0.6), DVec3::new(-16.0, -10.0, 0.6), DVec3::new(-10.0, -10.0, 0.6), DVec3::new(-10.0, -10.0, 5.0)];
        let pipe = Loop3 { segments: vec![Segment::new(Curve3::Circle { frame: Frame::new(bends[0], bends[1] - bends[0], DVec3::Z), radius: 0.5 }, 0.0, std::f64::consts::TAU)] };
        let legs: Vec<Segment> = bends.windows(2).map(|w| Segment::line(w[0], w[1])).collect();
        if let Some(b) = ductile_brep::ops::sweep::sweep(&pipe, &legs) {
            doc.add_body("Pipe", b);
        }
    }
    // A few materials for the rendered view.
    for (name, c, m, r) in [("Block", [0.78, 0.78, 0.80], 1.0, 0.3), ("Soap", [0.85, 0.35, 0.30], 0.0, 0.25), ("Rounded box", [1.0, 0.78, 0.34], 1.0, 0.2), ("Tube", [0.95, 0.95, 0.96], 1.0, 0.05)] {
        if let Some(o) = doc.objects.iter_mut().find(|o| o.name == name) {
            (o.color, o.metalness, o.roughness) = (c, m, r);
        }
    }
    // The surfaces in a group, as an Outliner example.
    let sheets: Vec<crate::document::ObjectId> = doc.objects.iter().filter(|o| o.body.kind == ductile_brep::BodyKind::Sheet).map(|o| o.id).collect();
    if sheets.len() > 1 {
        doc.selection = sheets.into_iter().map(Pick::Object).collect();
        if let Some(g) = doc.group_selected() {
            if let Some(g) = doc.groups.iter_mut().find(|x| x.id == g) {
                g.name = "Surfaces".into();
            }
        }
        doc.selection.clear();
    }
    doc.dirty = false;
}

#[cfg(test)]
mod native_tests {
    use super::*;

    #[test]
    fn groups_and_locks_survive_save_and_open() {
        let mut doc = Document::default();
        demo_scene(&mut doc);
        let g = doc.groups[0].id;
        let member = doc.group_members(g)[0];
        doc.objects.iter_mut().find(|o| o.id == member).unwrap().locked = true;
        let path = std::env::temp_dir().join(format!("ductile-groups-{}.ductile", std::process::id()));
        write_native(&doc, &path).unwrap();
        let mut back = Document::default();
        load_into(&mut back, &path).unwrap();
        std::fs::remove_file(&path).ok();
        assert_eq!(back.groups.len(), 1);
        assert_eq!(back.groups[0].name, "Surfaces");
        let g2 = back.groups[0].id;
        assert_eq!(back.group_members(g2).len(), doc.group_members(g).len());
        assert_eq!(back.objects.iter().filter(|o| o.locked).count(), 1);
    }
}
