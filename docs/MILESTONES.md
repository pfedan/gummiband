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
- Differential pairs: experimental option (`route --pairs`, see `docs/ARCHITECTURE.md`). Width and gap
  from net classes and custom rules per layer, skew tuning inside a pair, impedance estimate and
  `--pair-zdiff`. Measured on 2026-10-30: `diff_pairs` 99% coupled and clean in the KiCad DRC, Jetson
  93% coupled instead of 30%, but 81 instead of 51 connections open and 1954 instead of 493
  violations, Lichtschnitzel 347/348 instead of 348/348. Open: the KiCad DRC is not clean yet (`diff_pairs` 1 short at a via, Lichtschnitzel near J1), the
  lost connections on dense boards, more than one wide track per pair
  (branches, USB-C with both connector rows), necks for fine pitch, a crossover without a via,
  matching between pairs (`length` rules), the router knowing about polarity.
- Rule engine: more custom rule conditions (positions, footprints, layers).
- More than two signal layers (pulled forward with the synthetic 4 layer benchmarks).
- Alternative placements as portfolio variants (smaller, square, round outlines with the
  connectors and the ESP32 module fixed), see `crates/gummiband-place`.
- Placement from the netlist as a step of its own (`gummiband place`, plugin button, MCP
  `place_parts`): all unlocked, selected, or outside the board, with fixed parts respected.
- Placements as a search around routing (`route --placements N`), judged by full routes. Open: a
  local placement mutator that re-places parts around problem spots inside the generations.
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

## Open: fine pitch fan-out (Corne)

Measured on 2026-10-30, none of these solved it: straight pin stubs (twice: worse on
`fine_pitch_mixed`, arty, Lichtschnitzel), a larger router margin, extra gap outside the fine
pitch parts. A router margin of 0.06 mm brings Corne to 14 violations but only 392/482 routed:
the violations are the price of the last connections through the pin rows. Original widths
(0.2032 mm) did not change it either; the original design is clean under the same rules.

## Next: search quality

- Negotiated routing: done as an option, `route --negotiate N` (see ARCHITECTURE). Measured on
  2026-10-30 with `--runs 14 --generations 1 --negotiate 20`, KiCad DRC against the plain run:
  `fine_pitch_mixed` 0 open and 0 errors instead of 8 and 7, `four_signal_layers` 0 open instead
  of 4, `bga_dense` 8 instead of 12, Arty 2 open and 2 errors instead of 3 and 11, Corne 1 open
  instead of 12 and 669 errors instead of 702 (own check 1269 instead of 2289). Lichtschnitzel
  `--geometry-generations 3`: 348/348, 287 vias and DRC clean instead of 294 vias and 4 errors.
  The other synthetic boards stay clean. The run takes both searches, about twice as long. Open:
  ULX3S does not converge (the original also routes on In2): 160 open instead of 162, own check
  42 instead of 79, but 186 DRC errors instead of 133 and 1865 s instead of 438 s, so a board with
  more connections than room on its layers should be recognised early. Corne's clearance errors
  at the fine pitch parts remain.
- A post pass that removes via pairs when the hop also fits on the other layer (found in the MCP
  test: 23 short B.Cu hops, via cost 5 gives 175 instead of 208 vias without DRC errors).
- Via count per net in the route result.
- Follow-me with give: existing unlocked tracks yield. Soft guides (a cost pull towards the old
  route) failed on 2026-10-30: at most 65% of the length stays near the old way, and vias grow
  from 278 to 755 at high weight. Done instead: `route --keep-all --nets ... --shove` imports the
  tracks into the topology (`gummiband-core::import`, see ARCHITECTURE), the new connection
  threads in and relaxation pushes aside only what it touches. On the Lichtschnitzel follow-me
  test LED_D3 goes from 0/1 to 1/1 with 0 clearance errors, 18 tracks moved by at most 0.32 mm,
  the other copper unchanged in the file. 11 of 12 other nets, stripped and finished one by one,
  are clean as well. Open: in the tightest spots the relaxation can still pull a track over a
  neighbour (USB_DN at the USB-C pins, Arty /~{MR}: the clearance check reports it); a dense
  board with no room in the topology stays unrouted (ULX3S, `--give` is the fallback there); a
  new connection may leave kept copper a little before its open end, which leaves a stub.
- Adaptive track width: as wide as the surroundings allow, instead of one neck width per zone.

## M4b: GPU experiment

- Opt-in: `--features gpu` at build time and `GB_GPU=1` at run time, Metal through wgpu. Without
  them nothing changes, the default run stays byte-identical. Sleeping copper and vias of one layer
  stay on the CPU.
- Share of the relaxation in the work (core seconds, 4 threads, 2026-10-30): Lichtschnitzel 31 %,
  with `--geometry-generations 3 --move-parts` 63 %, Arty 43 %. Corne, ULX3S and bga_dense 1 to 5 %
  (measured before the counters of PR #8 were removed, which only lowers these). On the hard
  boards topological A* decides the run time, and it does not fit a shader.
- A way to the GPU and back costs about 1.5 ms, a round of 27000 points about 0.1 ms. So the GPU
  runs ten rounds at a time, from one resampling to the next, and the vias take one step of ten
  times the size per batch. In one round per way the GPU only reaches the speed of 4 CPU cores.
- With everything on the GPU more violations stay. So the last quarter (clearances only) stays on
  the CPU. One relaxation of Lichtschnitzel on 4 threads: 0.73 instead of 1.8 core seconds, 0.40
  instead of 0.66 s, with more violations on average.
- Lichtschnitzel `--geometry-generations 3 --move-parts`, seeds 1 to 4, 4 threads: 996 core
  seconds on the CPU, 593 with the GPU, all 348/348; violations 15, 4, 0, 2 against 5, 0, 0, 11.
- The same on all 14 cores, alone on the machine: 110 s on the CPU, 71 s with the GPU (1175 and 669
  core seconds), same violations. One relaxation: 0.52 s against 0.38 s, six in parallel 1.17 s
  against 0.57 s, with 15 % more violations on Lichtschnitzel and 0.4 % more length. Default run
  7.3 s against 6.0 s. The search takes different paths, so the seeds do not compare one to one.
  On the CPU the serial piece grids take 0.49 ms of a round of about 1 ms: the next cheap gain.
- Atomic counters that all threads touch per point made the relaxation three times slower
  (PR #8, removed in PR #10). Timings from builds with them are worthless.
- The relaxation is chaotic: a change of 1e-12 mm turns 1 short into 16 on `four_signal_layers`.
  Compare distributions (`relax_bench --ensemble`), not single runs.
- Results repeat bit by bit on one machine. Metal may compile with fast math, so another GPU or
  driver can give another result: this path breaks the rule that results do not depend on the
  machine, which is why it stays opt-in.
- Tried and dropped, 2026-10-30: taking the best of several slightly different relaxations
  (margins 1e-7 mm apart). Eight tries of the polish step: Lichtschnitzel seeds 15, 4, 0, 2 become
  12, 4, 0, 2. Eight tries of the full relaxation per finalist: 9, 4, 0, 9 at up to four times the
  core seconds, and fine_pitch_mixed gets worse (25 instead of 19). The violations that remain are
  mostly real bottlenecks, more relaxation does not remove them. They need another topology.
- So search breadth through the GPU is not worth building now: it would pay only for many
  finalists, and the search that makes them is what limits the hard boards.

## M5: User documentation

- A user guide (requested by Daniel): installation (binary, plugin environment, `rsvg-convert`
  and ImageMagick for animations), the KiCad plugin dialog, the CLI with all options, the MCP
  server for AI agents, and typical workflows (pre-routing and locking power lanes, routing by
  priority, judging results with the KiCad DRC, placement variants).
