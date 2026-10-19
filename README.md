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
  Q unions, W subtracts, Shift+E intersects, Shift+Q slices, Shift+R
  makes every enclosed region its own solid (split further by selected
  sheets) and T keeps the tools, including coplanar, tangent and pole cases. Cut (C) splits solids
  along curves swept straight through them: a closed curve cuts out a
  plug, an open one splits the solid in two. Imprint (Shift+I) sweeps curves
  the same way but only splits the faces they cross, so the pieces of a
  face can be picked, offset or extruded on their own. Both also take a
  surface (sheet) that reaches across the solid in place of curves. Knife
  mode (K while drawing any curve) splits just the face the curve is drawn
  on; earlier imprints survive later booleans. Delete redundant topology merges
  them away again. Offset edge imprints new edges at a distance from the
  selected edges, inside the flat faces beside them. Complete edge runs
  straight edges that stop partway on across the flat face beyond them, to
  the next edge, finishing a partial split. Offset face up to a face
  moves the selected flat faces until they reach the face clicked next
  (out along their normals, or onto its plane when it is tilted). Draft tilts
  selected planar faces about the line where they meet a neutral plane
  through their base, by a dragged or typed angle. Intersection curves
  adds the curves where selected solids cross; Project (I)
  traces curves onto solids along the construction plane's normal.
- Move faces: G in face mode pushes or tilts planar faces and re-trims the
  faces around them (curved neighbours follow a straight push).
- Move edges: G with straight edges of a solid selected tilts the planar
  faces along them about their far sides, so pulling a block's edge in
  makes a slope.
- Delete faces (Delete in face mode) heals the solid where it can: holes
  and bosses close up, and fillets or chamfers between planes grow back to
  a sharp edge, as do round fillets on circular edges (the top or foot of
  a boss). Otherwise the solid becomes an open sheet. Remove fillets
  finds every round up to a radius (convex, concave or both) and takes
  them off together. Duplicate
  (Shift+D) in face or edge mode copies the selected faces out as a sheet
  or the selected edges out as curves. Instance (Alt+D) places linked
  copies: each moves on its own, but editing one (fillet, knife, boolean...)
  edits them all, until Make unique unlinks it. Links are saved.
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
  helix (radius, height and turns; sweep a circle along it for a spring),
  spiral (center and outer radius; start radius, turns and an optional rise).
  C inside the rectangle, circle and arc commands switches between their
  variants, as in Plasticity.
  Tangent arc leaves the end of a curve tangentially (D flips it). Tangent
  circle touches two curves (sized by the cursor, or a typed radius) or
  three, on the sides the cursor is on. Text
  turns letters into closed curves ready to extrude, in the built-in font or
  any TrueType/OpenType font installed on the system, with a set height.
  Curves drawn separately that meet end to end form one profile. In vertex
  mode (1) spline control points show and can be selected and moved (G).
  Reverse curve flips a curve's direction, Raise degree lifts it exactly
  one degree, and Rebuild curve refits it with a chosen number of control
  points and degree, showing how far it moved.
  Fillet (B) with curves selected rounds their corners (or just the
  selected vertices'); Offset (O) offsets them in their plane,
  rounding outside corners; Trim (T) deletes the pieces you click
  between curve crossings; Join (J) merges curves that meet end to end
  into one. Split curve cuts a curve at the points you click (a closed
  one opens there); Insert knot (K to switch) adds a control point
  without changing the shape; Convert corners turns a polyline's corners
  into control points of one smooth spline. Deform curve lays curves drawn over a flat
  face onto another face (curved too), with scale, offset and flips in its
  u, v and normal (Unwrap face lays a face flat first, exactly for planes,
  cylinders and cones, as a reference). Slide CV (Shift+G) moves selected control points along
  their control polygon. Bridge (Alt+B) joins two curve ends with a new
  curve that meets them with G0 to G3 continuity and adjustable tension;
  Extend lengthens open curves past their ends, naturally (lines
  and arcs stay lines and arcs, splines continue their own polynomial) or
  straight along the end tangent. Align curve moves the end of a curve onto another
  curve and leaves it with G0 to G3 continuity, changing only the control
  points next to that end.
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
  (or split into four) boundary. Curved boundaries with five or more
  sides, or with inner curves, get one smooth fitted surface trimmed by
  them. Constrained surface makes a smooth sheet through points you
  click. Patch on a loop of selected edges meets
  the faces beyond them tangentially (G1), faired for even curvature: a
  tube's rim gets a smooth dome. Holes of three or more than four sides
  are filled with one piece per corner, curvature continuous with each
  other across their seams and with the faces around (G2; a corner where
  three fillets meet fills like a sphere). In vertex mode (1) the control nets of
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
- Isoparam (Ctrl+R): hover a face to see the surface's iso-parametric
  curve under the cursor, Tab swaps u and v, click splits the face there
  (neighbouring edges are split to match, so solids stay closed). Untrim
  (Alt+T) makes a sheet of the whole surface under the selected faces.
  Unjoin (Alt+J) pulls selected faces out as sheets, explodes bodies into
  one sheet per face, and curves into one curve per segment. Find boundary
  edges selects the open edges keeping a body from being solid; Reverse
  sheet swaps a sheet's front and back.
- Raise surface degree lifts the selected faces' surfaces one degree in u
  and v without changing their shape; Rebuild face refits them as smooth
  NURBS surfaces with the number of control points you choose.
- Moving surface control points (G/R/S) can drag their neighbours along
  with a smooth falloff (Falloff, in grid steps) and move the mirrored
  points across the surface's u middle (Mirror) for symmetric shaping.
- Full round: select the top face of a rib (between two parallel sides)
  and it becomes a half cylinder tangent to both.
- Fillets (B) and chamfers (Shift+K) on edges between planes, on
  circular plane/cylinder edges (boss and hole rims), with spherical corner
  patches where three filleted edges meet. Closed edges between any two
  surfaces (a vase lip, a loft's foot, a sphere cut flat) get rolling-ball
  NURBS fillets and chamfers with exact circular sections, and so do
  chains of tangent edges: round a box's corners, then fillet its whole
  top rim in one go; or a side edge between a spline face and a plane,
  closed off by the caps it runs into. Blends can be round, smooth (G2:
  curvature-continuous sections that leave the faces with zero curvature,
  for class-A reflections, fuller or flatter with a tension) or chamfered. Smooth blends get faired,
  curvature-continuous corner patches where three of them meet, between
  faces at any angle (a box, a hexagonal prism), and meet in clean mitres
  where a corner's third edge stays sharp.
  Straight edges between planes can also take a radius varying linearly
  along the edge (Variable radius in the Fillet panel): the round blend is
  then an exact cone, the smooth one its G2 counterpart. Chordal sizes a
  round blend by the width of its chord instead of its radius; along one
  edge whose faces open up, the radius varies to keep the chord.
  Picking one edge of a tangent run (a filleted outline) blends the whole
  run, as in Plasticity; T or the panel's Add tangent edges turns that off.
  Limit points (L, one straight edge between planes) blend only part of
  the edge, between two fractions along it, ending square where the sharp
  edge carries on.
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
- SVG import (File > Import SVG curves, or `ductile logo.svg`): every
  shape's outline (paths, rectangles, circles, ellipses, polygons, with
  their transforms) becomes curves in a group named after the file, one
  SVG unit to one model unit, ready to extrude.
- Hidden-line export (File > Export hidden line): the current view's edges
  and silhouettes as an SVG drawing, with the lines behind surfaces dashed
  in their own group. Headless: `--export-hidden-line OUT.svg` (with
  `--camera` to choose the view).
- Reference images (File > Reference image): a PNG or JPEG placed on the
  construction plane to model over, with opacity, size, centre, turn and
  flip in the side panel; saved with the document by path.
- Mesh import (File > Import mesh, or `ductile scan.stl`): OBJ, STL
  (binary or ASCII) and 3MF come in as reference meshes, shaded smooth across
  gentle bends and sharp at creases, that move, hide, snap and measure
  like any object. An OBJ's objects and a 3MF's build items become a group.
- PolySplines (Edit menu or palette) turns a selected mesh, best a quad
  cage from a subdivision modeller, into a smooth solid (or sheet, when
  open) of untrimmed spline patches: one exact bicubic patch per regular
  quad, curvature continuous with its neighbours, and a curvature
  continuous N-sided fill round each vertex where three, five or more quads meet. Triangles
  and other polygons are subdivided first. Open meshes carry on smoothly
  past their edges (a ghost ring reflected outside), so boundary vertices
  where three or more faces meet work too. Rectangles of regular quads
  merge into single B-spline surfaces, the same shape with fewer faces.
  The mesh is hidden.
- STEP import (File > Import STEP, or `ductile part.step`): planes,
  cylinders, cones, spheres, tori, B-spline and rational surfaces, surfaces
  of extrusion and revolution, in any length unit. Parts written by
  OpenCascade import with the volumes OpenCascade reports for them
  (`crates/ductile-io/tests/occ_import.rs`). Assemblies are flattened:
  every placed instance of a part becomes its own object, named after the
  part.
- Shaded looks: plain shading comes in five lighting studies of our own
  (Clay, Ceramic, Graphite, Chrome, Wax), picked under the shader balls
  or with `--look NAME`; they light from the camera as matcaps do.
- Topography shading (`--topography`): height bands and contour lines up
  the construction plane's normal, a heavier line every fifth.
- Zebra stripes (Alt+R, or the Zebra button) to judge surface continuity:
  stripes stay unbroken across tangent (G1) joins and keep their direction
  across curvature-continuous (G2) ones. Headless: `--zebra`.
- Rendered view (shading menu, or `--render`): each object's material
  (colour, metalness, roughness, set in Properties with plastic, steel,
  chrome, gold and rubber presets) lit by a studio of soft boxes, without
  edges, for judging a finished part. Materials are saved in `.ductile`.
- More analysis shading from the shading menu in the top bar: isophotes
  (lines of equal angle to the view every 10°, smooth over G2 surfaces,
  kinked at G1 joins), draft against the construction plane's normal (green
  pulls, red undercut, yellow under the minimum angle you set) and face
  direction (front blue, back red). Headless: `--isophote`, `--draft`,
  `--direction`.
- Curvature combs (the Curvature button) on the selected curves, edges and
  solids, all at one scale so joins compare, with G0 / G1 / G2 marked where
  picked curves meet. Headless: `--curvature`.
- Dimension: click two points (snapped to geometry), move the line off
  them and click; dimensions stay in the scene, undo and save with it, and
  Delete all dimensions clears them.
- Measure (palette or View menu): with two things selected, the shortest
  distance between them drawn as a dimension (the angle too for two
  planes); with one, its length, radius, area or volume.
  Measuring one edge between two faces also reports how they meet: not
  G0 (a gap), G0 (an angle), G1 (tangent, with the curvature jump) or G2.
- Section analysis (the Section button): cuts the scene away beyond a
  plane along the construction plane's normal, through the scene's centre;
  solids show their cut inside as a cap. Drag the offset, or Flip to keep
  the other side. Headless: `--section`.
- Move (G) / rotate (R) / scale (S) / mirror (Alt+X), linear and radial
  arrays, undo/redo. Mirror reflects across the construction plane,
  or across a selected planar face; a solid mirrored across its own face is
  joined to its reflection, so half a symmetric part becomes the whole.
  Freestyle move (F while moving): click the point to move from, then where
  it goes; both snap.
- Place (Ctrl+D) stands copies of the selection on the faces you click,
  bottom centre on the point and up along the surface normal, turned and
  scaled from the panel.
- Curve array copies the selection evenly along the longest selected curve,
  turning each copy with the curve's tangent (Align off keeps them upright).
- Hide (H), isolate the selection (Shift+H or .), show everything (Alt+H).
- Outliner groups: Ctrl+G groups the selection (nested inside the group it
  shares), Alt+G ungroups. Each row has show/hide and lock toggles; locked
  objects stay visible but can't be selected. Click a group to select its
  objects, Alt+click an object in the viewport for its whole group, and
  double-click a group to make it the active group that new objects go into.
  Right-click a group to rename, dissolve, delete it with its contents or
  add the selection to it. Groups and locks are saved in `.ductile` files.
  Double-click an edge to add the edges running on tangentially from it
  (a filleted outline, a spline profile) to the selection. As in
  Plasticity, Alt+click an edge picks its loop and Ctrl+Alt+click its ring;
  Alt+click a face picks the faces running on tangentially from it and
  Ctrl+Alt+click the faces parallel to it (coaxial for cylinders). A box
  dragged right to left picks anything it touches, left to right only what
  it holds.
- The window follows Plasticity's layout: a slim title row (the app menu,
  panel toggles, the document's saved state), the Outliner and Assets on
  the left (Ctrl+B), cards for shading, camera, grid, snapping, planes and
  curvature on the right (Ctrl+Shift+B), and over the viewport the
  selection modes, the view toggles, a drawing tool strip, a bottom
  command bar with a More menu (inapplicable commands greyed, never
  hidden), the floating command dialog and a strip of the command's keys.
- Camera card: orthographic switch, FOV (35° by default) and Squish, a
  dolly zoom that flattens or deepens perspective while the target stays
  framed.
- Selection modes combine: 1 to 4 switch to vertices, edges, faces or
  solids, 5 turns them all on, and Shift+click on a mode button (or the
  unbound Toggle … mode commands) adds or removes one. With several on, a
  click takes the most specific thing under the cursor (a vertex, then an
  edge, a face, the solid) and a box takes them all.
- Regions: clicking inside a closed planar curve picks it, as Plasticity
  picks the region, and the region shows a faint fill that lights up on
  hover and selection. Extrude, Revolve, Fillet and Chamfer started with
  nothing selected open their dialog and wait for the pick.
- Click drill-down and auto-invoke, as in Plasticity: in solid mode the
  first click selects the solid and a second click the face or edge under
  the cursor. Selecting faces starts Push face and edges start Fillet;
  drag to set the value (or press D, or Tab, and type it), click or Enter
  to finish. A click before any change just selects something else.
- Preferences (Ctrl+,), kept in `~/.config/ductile/prefs.txt`: auto-invoke
  and drill-down on or off, mouse navigation presets (Plasticity: middle
  drag orbits and right drag pans; Blender,
  Maya, Rhino, Moi3D, 3ds Max, Fusion 360), zoom toward the cursor, the autosave interval and opening
  the last session on startup, and the display unit (mm, cm, m, in, ft):
  the model stays in millimetres, while readouts, measurements and
  dimensions show the chosen unit and a bare typed length is read in it.
- File menu as in Plasticity: Import/Append (Ctrl+Shift+O) adds a
  Ductile, STEP, SVG or mesh file to the open scene; Quick save version
  writes `name_v2.ductile`, `name_v3.ductile`, … beside the file; Save as
  startup scene makes new documents (and launches) start from the current
  one; New window; Quit (Ctrl+Q).
- Backups: saving over a file first keeps the old copy in `<name>.backups`
  beside it (or a folder set in Preferences), pruned to the newest few plus
  one per hour, day and week as configured. If Ductile did not close
  cleanly, the next start says so and File > Recover autosave brings the
  work back.
- While dragging a value, the number shows beside the handle (five
  decimals) and push and fillet drags draw an endless guide along their
  direction. Shift+F opens the commands that apply to the selection at
  the cursor.
- Transform gizmo: selecting something shows axis arrows and rotation rings
  at it; drag an arrow to move along that axis or a ring to turn about it
  (type a value while dragging for an exact one). Headless: `--select N`.
- Snapping while drawing and placing, labelled beside the marker as in
  Plasticity: End, Middle, 1/4, 3/4, Center, Intersection, Perpendicular
  and Tangent to the last point, the X / Y / Z lines through it, then
  curves, edges and faces. Snaps hidden behind a solid are ignored.
  The Snapping card turns each kind of object snap on or off; the Grid
  card sets snapping to the grid, a fixed spacing (or one that follows the
  zoom), the grid's size and its major lines. Ctrl suspends all snapping.
  While sketching, the tag also names the plane drawn on ("End,XY"); a
  rectangle shows its width and height and locks square near the
  diagonal ("square,XY", with a magenta guide).

In progress: more of Plasticity's curve and surface tools.

## Shortcuts

The default keys follow Plasticity's, one key per intent: F opens the
command palette, B fillets (edges, or curve corners), O offsets (curves,
faces or a whole solid), Q runs a boolean, C cuts, E extrudes, L lofts, P
makes a pipe, G / R / S move, rotate and scale, 1–4 pick vertices, edges,
faces or solids (Ctrl+1–4 turn the current selection into those), A selects all, Shift+R repeats the last command, Ctrl+T reselects the last operation's inputs (again to walk back), / frames
the selection and Space sets the construction plane from it. Every
command, with or without a key, is in command search (F, opened at the
mouse); each row's … menu assigns, unsets or resets its shortcut and adds
it to the favourites pinned at the top. Inside a command, Tab (or any digit) opens a
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
