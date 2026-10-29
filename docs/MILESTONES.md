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

## M3: Quality and degrees of freedom

- **Variable track width:** neck down in tight spots, not only near pins (needs a width per
  crossing in the capacity model; the geometry already supports per piece widths).
- **Movable parts:** parts that are not locked may shift a little when that frees a channel
  (search over small placement offsets, evaluated by the router).
- Differential pairs with impedance and length matching.
- Rule engine: more custom rule conditions (positions, footprints, layers).
- More than two signal layers.

## M4: Search and performance

- Evolutionary search with a richer genome (routing order, via cost, layer preference, part
  offsets) and a fitness that includes the final DRC result.
- Benchmarks against Freerouting on the controller and the injector board.
- Profiling: incremental capacity checks, cheaper relaxation.
