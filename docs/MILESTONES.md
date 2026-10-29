# Milestones

## M0: Foundation

- Cargo workspace (`gummiband-core`, `gummiband-kicad`, `gummiband-cli`).
- KiCad 9/10 board reader: outline, footprints, pads (all shapes), tracks, arcs, vias, zones,
  keepouts, locked flags, net classes from `.kicad_pro`.
- Board model with obstacles per layer; `gummiband info` prints a summary.
- `gummiband svg` renders layers, obstacles and the triangulation for debugging.
- Done when: the Lichtschnitzel controller loads, the summary matches KiCad (pads, nets,
  locked copper), the SVG looks right.

## M1: Two layer topological router

- Triangulation substrate with per layer blocking, capacities and via sites.
- Topological A* with slots (planar per layer), vias, MST decomposition, rip-up and reroute.
- Realization: portals, funnel, arcs around obstacles.
- Writer: new segments, arcs and vias into a copy of the board.
- Plane nets: fan-out vias into the inner planes.
- Net class widths (current tracks), clearances, via sizes.
- Portfolio parallelism over orders and seeds.
- Done when: synthetic boards route completely without clearance errors, and the Lichtschnitzel
  controller, stripped of all tracks, vias and outer pours, routes completely with a DRC that
  has no clearance or short errors.

## M2: KiCad plugin and manual control

- IPC API plugin with dialog: net or net class selection, priorities, layers.
- Incremental routing: route selected nets only, keep everything else.
- `gummiband.toml` per board (priorities, widths, skipped nets).
- Done when: Daniel routes the controller from inside KiCad with pre-routed power lanes locked.

## M3: Quality

- Elastic optimizer: parallel relaxation of all routes as rubber bands (see architecture).
- Spreading (even spacing where there is room), track smoothing, via minimization.
- Net class rules from `.kicad_dru` (widths, clearances per class pair).
- Differential pairs (coupled routing), length matching.
- More than two signal layers.

## M4: Performance and comparison

- Benchmarks against Freerouting on the controller and the injector board.
- Profile and tune: parallel congestion rounds, incremental triangulation.
