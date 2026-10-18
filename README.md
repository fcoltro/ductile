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
- Exact booleans: union, subtract, intersect (Shift+U / Shift+D / Shift+I),
  including coplanar, tangent and pole cases.
- Curves on the construction plane: polyline (L), spline through points
  (Shift+L, open or closed C2), three-point arc (Shift+A), rectangle
  (Shift+R), circle (Shift+O), ellipse (Shift+E), regular polygon (Shift+P).
  Curves drawn separately that meet end to end form one profile.
- Extrude (E): push/pull planar faces, or turn closed planar curves (with
  holes) into solids, with live preview.
- Revolve (V) closed curves by any angle about a selected edge or the
  construction plane axes; exact planes, cylinders, cones, spheres, tori,
  and NURBS surfaces of revolution for splines.
- Loft (Alt+L) through two or more closed curves, keeping corners as edges
  when the profiles match; Sweep (Alt+W) a closed curve along an open path
  with a rotation-minimizing frame (exact for straight and circular paths).
- Fillets (Shift+F) and chamfers (Shift+K) on edges between planes, on
  circular plane/cylinder edges (boss and hole rims), with spherical corner
  patches where three filleted edges meet.
- STEP (AP214) export of exact solids, checked against OpenCascade; STL and
  OBJ export. Headless: `ductile --demo --export-step out.step`.
- STEP import (File > Import STEP, or `ductile part.step`): planes,
  cylinders, cones, spheres, tori, B-spline and rational surfaces, surfaces
  of extrusion and revolution, in any length unit. Parts written by
  OpenCascade import with the volumes OpenCascade reports for them
  (`crates/ductile-io/tests/occ_import.rs`).
- Move / rotate / scale / mirror, undo/redo.

In progress: curve editing, mitred sweeps around sharp path corners, general
rolling-ball NURBS fillets, STEP assemblies.

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

## License

GPL-3.0-only. See [LICENSE](LICENSE).
