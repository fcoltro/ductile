# Ductile

Ductile is a free and open-source 3D CAD modeler for Linux, written in Rust,
whose goal is to match the workflow, modeling power and feel of
[Plasticity](https://www.plasticity.xyz/): fast, keyboard-driven, NURBS-based
solid and surface modeling for artists.

It ships its own geometric kernel. The kernel's design is informed by the
public literature around industrial kernels (Parasolid, xNURBS) and by
classic references such as *The NURBS Book* (Piegl & Tiller), but contains no
code from them.

![Ductile screenshot](docs/screenshot.png)

## What works today

- Primitives: box, cylinder, cone, sphere, torus (exact analytic surfaces).
- Exact booleans: union, subtract, intersect and slice (both pieces). Q
  previews the other selected solids subtracted from the first; inside it
  Q unions, W subtracts, Shift+E intersects, Shift+Q slices and T keeps
  the tools,
  including coplanar, tangent and pole cases. Cut (C) splits solids
  along curves swept straight through them: a closed curve cuts out a
  plug, an open one splits the solid in two. Imprint (Shift+I) sweeps curves
  the same way but only splits the faces they cross, so the pieces of a
  face can be picked, offset or extruded on their own. Both also take a
  surface (sheet) that reaches across the solid in place of curves. Draft tilts
  selected planar faces about the line where they meet a neutral plane
  through their base, by a dragged or typed angle. Intersection curves
  adds the curves where selected solids cross; Project (I)
  traces curves onto solids along the construction plane's normal.
- Move faces: G in face mode pushes or tilts planar faces and re-trims the
  faces around them (curved neighbours follow a straight push).
- Delete faces (Delete in face mode) heals the solid where it can: holes
  and bosses close up, and fillets or chamfers between planes grow back to
  a sharp edge. Otherwise the solid becomes an open sheet. Duplicate
  (Shift+D) in face or edge mode copies the selected faces out as a sheet
  or the selected edges out as curves.
- Construction planes from the selection, as in Plasticity: Space sets one
  and turns the view to it, Shift+Space sets it without moving the view.
  A planar face gives its plane; a face and an edge, the plane standing on
  the face along the edge; two opposing faces, the plane midway; two
  points, the plane through them seen edge-on; three or more points or a
  flat curve, the plane they lie in. Ctrl+Space faces the view; Space with
  nothing selected goes back to the world XY plane.
- Curves on the construction plane: polyline (Shift+A), spline through points
  (open or closed C2), arc (three points or from the center), rectangle
  (from a corner, the center or three points), circle (from the center,
  two or three points), slot, ellipse, regular polygon,
  helix (radius, height and turns; sweep a circle along it for a spring).
  C inside the rectangle, circle and arc commands switches between their
  variants, as in Plasticity.
  Curves drawn separately that meet end to end form one profile. In vertex
  mode (1) spline control points show and can be selected and moved (G).
  Fillet (B) with curves selected rounds their corners (or just the
  selected vertices'); Offset (O) offsets them in their plane,
  rounding outside corners; Trim (T) deletes the pieces you click
  between curve crossings; Join (J) merges curves that meet end to end
  into one. Bridge (Alt+B) joins two curve ends with a new
  curve that meets them with G0 to G3 continuity and adjustable tension;
  Extend lengthens open curves past their ends, naturally (lines
  and arcs stay lines and arcs, splines continue their own polynomial) or
  straight along the end tangent.
- Extrude (E): push/pull planar faces, or turn closed planar curves (with
  holes) into solids, with live preview. Curves drawn on a solid's face
  join it when pulled out and cut it when pushed in (or pick New, Join or
  Cut in the panel).
- Revolve (V) closed curves by any angle about a selected edge or the
  construction plane axes; exact planes, cylinders, cones, spheres, tori,
  and NURBS surfaces of revolution for splines.
- Loft (L) through two or more closed curves, keeping corners as edges
  when the profiles match; Sweep (Shift+P) a closed curve along an open path
  with a rotation-minimizing frame (exact for straight and circular paths),
  mitred where the path turns through a sharp corner. Pipe (P) runs a
  round tube of a dragged radius along curves, closing into a ring along a
  closed curve.
- Surfaces: Extrude, Revolve and Loft turn open curves into surfaces
  (exact planes, cylinders, cones, spheres and tori where the curves allow);
  Patch (Alt+P) fills closed curves, with a trimmed plane when they are
  flat (inner curves become holes) or a Coons patch through any four-sided
  (or split into four) boundary. Patch on a loop of selected edges meets
  the faces beyond them tangentially (G1), faired for even curvature: a
  tube's rim gets a smooth dome. In vertex mode (1) the control nets of
  untrimmed spline surfaces show and their points move with G, rotate and
  scale; neighbouring faces stay joined. Thicken (Alt+K) turns surfaces into
  solids of a given thickness on either side. Bridge surface spans two
  selected edges with a sheet meeting the faces beyond them with G0, G1 or
  G2 continuity, chosen per side, with a tension per side. Extend with
  open surface edges selected grows the surface past them, naturally (its
  own shape continued) or straight along the tangent. Join (J) with
  surfaces selected sews them along the edges they share, turning them to
  face the same way, and makes a solid when they close up. Match surface
  reshapes a surface along its open edge so it lands on another edge and
  continues the face beyond it with G0, G1 or G2. Back faces show tinted.
- Fillets (B) and chamfers (Shift+K) on edges between planes, on
  circular plane/cylinder edges (boss and hole rims), with spherical corner
  patches where three filleted edges meet. Closed edges between any two
  surfaces (a vase lip, a loft's foot, a sphere cut flat) get rolling-ball
  NURBS fillets and chamfers with exact circular sections, and so do
  chains of tangent edges: round a box's corners, then fillet its whole
  top rim in one go; or a side edge between a spline face and a plane,
  closed off by the caps it runs into. Blends can be round, smooth (G2:
  curvature-continuous sections that leave the faces with zero curvature,
  for class-A reflections) or chamfered. Smooth blends get faired,
  curvature-continuous corner patches where three of them meet, between
  faces at any angle (a box, a hexagonal prism), and meet in clean mitres
  where a corner's third edge stays sharp.
  Straight edges between planes can also take a radius varying linearly
  along the edge (Variable radius in the Fillet panel): the round blend is
  then an exact cone, the smooth one its G2 counterpart.
- Shell (Alt+S): hollow a solid into walls of a given thickness, opening
  the selected faces, or leaving an inner void when none are selected.
  Offset (O) grows or shrinks a whole solid, on selected faces
  (or E on a curved face) it pushes and pulls any faces. Faces move along
  their normals with exact offset surfaces for analytic faces, sharp edges
  stay sharp, and tangent faces (fillets) move together.
- STEP (AP214) export of exact solids and surfaces, checked against
  OpenCascade; several objects are written as an assembly with one named part each. STL and
  OBJ export. Headless: `ductile --demo --export-step out.step`.
- Files: File > Open recent lists the last files opened or saved, and the
  open document is autosaved every two minutes while it has unsaved
  changes (File > Recover autosave brings it back after a crash).
- STEP import (File > Import STEP, or `ductile part.step`): planes,
  cylinders, cones, spheres, tori, B-spline and rational surfaces, surfaces
  of extrusion and revolution, in any length unit. Parts written by
  OpenCascade import with the volumes OpenCascade reports for them
  (`crates/ductile-io/tests/occ_import.rs`). Assemblies are flattened:
  every placed instance of a part becomes its own object, named after the
  part.
- Zebra stripes (Alt+R, or the Zebra button) to judge surface continuity:
  stripes stay unbroken across tangent (G1) joins and keep their direction
  across curvature-continuous (G2) ones. Headless: `--zebra`.
- Curvature combs (the Curvature button) on the selected curves, edges and
  solids, all at one scale so joins compare, with G0 / G1 / G2 marked where
  picked curves meet. Headless: `--curvature`.
- Move (G) / rotate (R) / scale (S) / mirror (Alt+X), linear and radial
  arrays, undo/redo. Mirror reflects across the construction plane,
  or across a selected planar face; a solid mirrored across its own face is
  joined to its reflection, so half a symmetric part becomes the whole.
- Hide (H), isolate the selection (Shift+H or .), show everything (Alt+H).
  Double-click an edge to add the edges running on tangentially from it
  (a filleted outline, a spline profile) to the selection. As in
  Plasticity, Alt+click an edge picks its loop and Ctrl+Alt+click its ring;
  Alt+click a face picks the faces running on tangentially from it and
  Ctrl+Alt+click the faces parallel to it (coaxial for cylinders). A box
  dragged right to left picks anything it touches, left to right only what
  it holds.
- Transform gizmo: selecting something shows axis arrows and rotation rings
  at it; drag an arrow to move along that axis or a ring to turn about it
  (type a value while dragging for an exact one). Headless: `--select N`.
- Snapping while drawing and placing, labelled beside the marker as in
  Plasticity: End, Middle, 1/4, 3/4, Center, Intersection, Perpendicular
  and Tangent to the last point, the X / Y / Z lines through it, then
  curves, edges and faces. Snaps hidden behind a solid are ignored.

In progress: more of Plasticity's curve and surface tools.

## Shortcuts

The default keys follow Plasticity's, one key per intent: F opens the
command palette, B fillets (edges, or curve corners), O offsets (curves,
faces or a whole solid), Q runs a boolean, C cuts, E extrudes, L lofts, P
makes a pipe, G / R / S move, rotate and scale, 1–4 pick vertices, edges,
faces or solids, A selects all, Shift+R repeats the last command, / frames
the selection and Space sets the construction plane from it. Every
command, with or without a key, is in the palette; right-click one there
to give it a new shortcut. Inside a command, Tab (or any digit) opens a
typed value that takes arithmetic and units (`10+2.5`, `3*4mm`, `1in`,
`45deg`); Escape drops it back to the pointer. Extrude, like Plasticity's,
takes Q to join, W to cut, Shift+E to intersect and B for a new body. Changes are kept in
`~/.config/ductile/keymap.txt` as lines like `Box = Shift+B`.

## Layout

| Crate | What it is |
| --- | --- |
| `ductile-geom` | Geometry: frames, NURBS curves/surfaces, analytic curves and surfaces, projection |
| `ductile-brep` | Topology and modeling: B-rep bodies, primitives, tessellation, extrude/revolve/boolean/fillet |
| `ductile-io` | File formats: native `.ductile`, STEP, OBJ, STL |
| `ductile-app` | The application: wgpu viewport + egui interface |

## Building

```sh
cargo run --release -p ductile-app
```

Requires a Vulkan or OpenGL capable GPU driver (Mesa works).

## Installing on Linux

```sh
packaging/linux/install.sh                      # into ~/.local
PREFIX=/usr/local sudo -E packaging/linux/install.sh
```

This builds a release binary and installs it with a desktop entry, icon,
AppStream metadata and the `.ductile` file type, so Ductile shows up in the
application menu and opens `.ductile` and `.step` files. A Debian package
can be built with [`cargo deb -p ductile-app`](https://github.com/kornelski/cargo-deb).

## License

GPL-3.0-only. See [LICENSE](LICENSE).
