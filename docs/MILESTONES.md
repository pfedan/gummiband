# Milestones

Status as of 2026-10-29 (night 1). Benchmark: Lichtschnitzel controller, all tracks, vias and
outer pours removed, routed completely by Gummiband (`bench/run-lichtschnitzel.sh`).

## M0: Foundation (done)

- Cargo workspace (`gummiband-core`, `gummiband-kicad`, `gummiband-cli`).
- KiCad 9/10 board reader: outline, footprints, pads (all shapes incl. custom), tracks, arcs,
  vias, zones, keepouts, locked flags, net classes from `.kicad_pro`, custom rules from
  `.kicad_dru` (clearances between nets, minimum widths with courtyard exemptions).
- Writer that patches the original file (segments, arcs, vias).
- `gummiband info`, `gummiband strip`, `gummiband route --svg`.

## M1: Two layer topological router (mostly done)

- Per layer constrained Delaunay mesh of the real obstacles, coarse Steiner grid against
  slivers, vias inserted dynamically (triangle split with re-threading of existing routes).
- Topological A* with slots (planar per layer), apex constraints, vias, via in pad as last
  resort, plane fan-out, MST connections, rip-up and reroute, overlapping pads as one pin.
- Track necks near pins where the rules allow it.
- Realization: portals and funnel, then the elastic optimizer (relaxation with movable vias),
  a taut pass with clearance-checked shortcuts and arc fillets at the corners.
- Portfolio and evolutionary order search on all cores.
- Open: 1 connection (eFuse VBUS_IN pad), about 10 to 40 DRC clearance violations depending on
  the run, a few topological crossings.

## M2: KiCad plugin and manual control

- IPC API plugin (`plugin/`, KiCad 10) with dialog, CLI runner and one-commit import. Built and
  tested against a fake API; live test in KiCad pending (needs the KiCad API enabled).
- Next: incremental routing of selected nets, `gummiband.toml` per board (priorities, widths,
  skipped nets).
- **Follow-me routing:** the user draws the start of a track, Gummiband proposes the rest to
  the target (single connection search from the end of the drawn copper, shown as a preview).
  Done without preview: the plugin action Finish selected nets (`route --keep-all --nets`).
  Kept copper without a pad becomes a pin, pins on one piece of copper count as connected,
  the search rides kept copper for free. Not yet tested inside KiCad.

## M3: Quality and degrees of freedom

- **Variable track width:** neck down in tight spots, not only near pins (needs a width per
  crossing in the capacity model; the geometry already supports per piece widths).
- **Movable parts:** parts that are not locked may shift a little when that frees a channel
  (search over small placement offsets, evaluated by the router).
- Differential pairs with impedance and length matching.
- Rule engine: more custom rule conditions (positions, footprints, layers).
- More than two signal layers (pulled forward with the synthetic 4 layer benchmarks).
- Alternative placements as portfolio variants (smaller, square, round outlines with the
  connectors and the ESP32 module fixed), see `crates/gummiband-place`.
- Synthetic benchmark suite of increasing difficulty (`bench/gen_boards.py`).

## M3b: Interface for AI agents

- Machine readable everything: `gummiband route --report out.json` (stats, unrouted
  connections, violations with nets and positions, portfolio) is the first step.
- Next: an MCP server (`gummiband mcp`) with tools to load a board, list nets and classes,
  lock or prioritize nets, set per net options, route (all or selected nets), get DRC results
  and quality metrics as JSON, and render PNG images of regions for visual inspection.
- Stable JSON schemas, deterministic results for the same seed, incremental calls on an open
  session so an agent can iterate quickly.
- Datasheet-guided pre-routing (idea from Daniel): the controlling agent reads the datasheet of
  a critical part (layout recommendation of an eFuse, a buck converter, a crystal), places that
  copper through MCP tools as locked tracks, vias and zones, checks it, and then lets the
  autorouter do the rest around it. Done: `part_info`, `add_locked_copper` (tracks, vias).
  Still open: locked zones, checking a region.
- Blocks (fixed blocks): a KiCad group or placed design block (parts plus their pre-routed locked copper, e.g. the datasheet
  layout of a converter) moves only as a whole under --move-parts; inside it nothing changes.

## M4: Search and performance

- Evolutionary search with a richer genome (routing order, via cost, layer preference, part
  offsets) and a fitness that includes the final DRC result.
- Benchmarks against Freerouting on the controller and the injector board.
- Profiling: incremental capacity checks, cheaper relaxation.

## M4b: GPU experiment

- Relaxation, clearance check and candidate screening fit a compute shader (wgpu): uniform
  Jacobi rounds over many points with grid neighbourhoods. Topological A* does not. Decide after
  the CPU micro-optimisation, from the share of run time that relaxation still takes. The gain
  would be search breadth (hundreds of screened finalists), not only speed.

## M5: User documentation

- A user guide (requested by Daniel): installation (binary, plugin environment, `rsvg-convert`
  and ImageMagick for animations), the KiCad plugin dialog, the CLI with all options, the MCP
  server for AI agents, and typical workflows (pre-routing and locking power lanes, routing by
  priority, judging results with the KiCad DRC, placement variants).
