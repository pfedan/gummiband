# Milestones

Status as of 2026-10-29 (night 1). Benchmark: LED controller, all tracks, vias and
outer pours removed, routed completely by Gummiband (`bench/run-ledctl.sh`).

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
- Moving a part with its copper (rubber band part shove): the part moves in steps no longer than
  the smallest clearance, its own tracks stretch, other nets give way, it stops where nothing can
  give way any more. `gummiband move-part`, MCP `move_part` (default), and the shifts of
  `--move-parts`, which before jumped up to 0.5 mm at once and relaxed only once, and whose next
  round after a rejected new search started from a placement that did not match the copper.
  Since 2026-11-01 on by default (`--no-move-parts` switches it off), also in the plugin and in
  MCP `route`, and open connections ask for room as well: the neighbours of their pins step
  aside. Open: rotation with following copper, parts that shove other parts along.
- Differential pairs: experimental option (`route --pairs`, see `docs/ARCHITECTURE.md`). Width and gap
  from net classes and custom rules per layer, skew tuning inside a pair, impedance estimate and
  `--pair-zdiff`. Measured on 2026-10-30: `diff_pairs` 99% coupled and clean in the KiCad DRC, Jetson
  93% coupled instead of 30%, but 81 instead of 51 connections open and 1954 instead of 493
  violations, LED controller 347/348 instead of 348/348. Open: the KiCad DRC is not clean yet (`diff_pairs` 1 short at a via, LED controller near J1), the
  lost connections on dense boards, more than one wide track per pair
  (branches, USB-C with both connector rows), necks for fine pitch, a crossover without a via,
  the router knowing about polarity. Since 2026-11-01 pairs are coupled only where a
  `diff_pair_gap` rule or `--pair-all` asks for it, and the uncoupled length is reported.
- Length matching (2026-11-01): `length`, `skew` and `diff_pair_uncoupled` rules read as KiCad
  groups them, lengths measured as the KiCad DRC does (equal to 0.0001 mm on the DDR5 testbed),
  `gummiband lengths`, meanders for nets and pairs that are too short, written as KiCad tuning
  patterns. Measured on `matching` and Antmicro's DDR5 testbed (see `docs/guide/benchmarks.md`):
  the clock reaches its window, but on DDR5 only 7 of 22 nets reach their length. Room for
  meanders (2026-11-01): before the meanders the nets that are too short are widened to the
  width of their meander and push their neighbours aside, undone in halves where that would add
  a clearance error; DDR5 10 instead of 16 of 29 out of range, 14 of 23 nets reach their length,
  KiCad skew errors 9 instead of 14, pair gap 6 instead of 22, 0 clearance. Pairs and 45 degrees
  (2026-11-01): with `--pairs --angles 45` the wide track goes onto the grid before the split,
  so both tracks stay parallel; a pair end whose repair finds no way gives up the pair (single
  tracks) before it keeps crossed tracks. `diff_pairs` 8 instead of 13 KiCad errors (pair gap 1
  instead of 11, 99 % instead of 95 % coupled), LED controller 0 instead of 12 errors and 1
  instead of 3 open, but its USB pair then runs as single tracks. Breakout zones
  (2026-11-01): with `--pairs` the wide track ends just outside the pads, the pads block it and
  the room for the two short tracks to them stays free; which track takes which side is chosen
  so that they do not cross. `diff_pairs` without clearance errors and shorts on all seeds
  (before 3 of 6 runs with them), seed 0 2 instead of 6 KiCad errors (45 degrees 3 instead of
  8), `matching` 7 instead of 13, LED controller unchanged (its USB-C end stays at the pads).
  On DDR5 they were worse over 6 seeds (more pair gap and length errors, a pair whose repair
  breaks it into single nets gets meandered as two nets), so they are opt-in (`--breakouts`)
  until that is understood; ends between the pads of a part (BGA) stay at their pads. A repair
  whose single track takes a long way round now counts as failed (on DDR5 one 31 mm detour set
  the aim of the whole command address group): with `--breakouts` DDR5 has 10 to 16 instead of
  13 to 22 of 29 out of range over 6 seeds, about as many as without; more pair gap errors
  remain near the breakouts. Via pairs (2026-11-01, opt-in `--via-pairs`): two vias side by
  side are one capsule that turns with the wide track, in check and relaxation, instead of the
  circle that leaves room for vias in line too. Over seeds 1 to 3 (DDR5 1 to 4) KiCad errors
  `diff_pairs` 11 instead of 16, DDR5 185 instead of 206 (clearance and shorts 26 instead of 31
  on seed 1), `matching` 56 instead of 53 (2 length errors more on one seed), lengths out of
  range on DDR5 53 instead of 52. Few joints qualify (1 to 5 on DDR5); most pair errors sit at
  vias in line, where a pair turns back or changes sides.
  Vias in line (2026-11-02): a pair that turns back at a via now takes its vias side by side
  there and changes sides; at most one joint per pair still has vias in line, only where the
  polarity asks for it (before: every joint where the band turned back). With `--pairs` over
  seeds 1 to 3 (DDR5 1 to 4) KiCad errors `diff_pairs` 10 instead of 16 (no clearance errors or
  shorts left), `matching` 28 instead of 53, DDR5 151 instead of 206 (clearance and shorts 8
  instead of 44, lengths out of range 48 instead of 52), LED controller unchanged.
  Skew bumps without a flat top (2026-11-02): most remaining pair gap errors on DDR5 were the
  tops of the bumps, which KiCad takes as coupled at a gap of up to 0.66 mm. As points the
  bumps gain as much and only count as uncoupled length: DDR5 107 instead of 151 KiCad errors
  over seeds 1 to 4 (pair gap 21 instead of 65), `diff_pairs` 5 instead of 10, `matching`
  unchanged.
  USB-C pins without partner (2026-11-02): with breakout zones the LED controller's USB pair is
  routed as a pair, but the last pass joined the second pads of the socket (B6/B7) with a 70 mm
  track along the pair and onto its partner (502 KiCad errors). Such a pair now gives up its
  wide track: LED controller `--pairs --breakouts` 3 instead of 502 KiCad errors; DDR5,
  `diff_pairs` and `matching` unchanged (byte for byte). The pair itself still needs a way for
  those pins next to the socket.
  Pads at the pair's gap and crossed pairs (2026-11-02): where the pads lie farther apart than
  the pair (an edge connector), the two tracks ran out of them at the pad pitch, which KiCad
  takes as coupled at a gap out of range; each now steps over towards its partner within its
  pad. A pair whose polarity is crossed and whose wide track has no via (DDR5 CLK) no longer
  falls apart into two single tracks with 20 mm uncoupled: the tracks cross just before the far
  pads, where the repair reroutes them (and gives the pair up rather than keep crossed tracks).
  DDR5 over seeds 1 to 8, KiCad errors: `--pairs` 183 instead of 243 (pair gap 19 instead of
  41, clearance and shorts 12 instead of 26), `--pairs --breakouts` 156 instead of 175 (pair
  gap 29 instead of 51); CLK coupled in most seeds with 4.5 to 8 mm uncoupled. `diff_pairs` and
  `matching` unchanged in KiCad errors. A wide track repaired at both ends no longer looks
  joined in the last pass when the tracks of its pins without partner join among themselves
  (LED controller with `--pairs --breakouts`: the USB-C pair kept its wide track with the second
  pad row open, 2 unconnected); that copper is now a pin of its net there.
  Open: room in the router itself, phase matching near the source, carrying the pair to its
  end, the open end with pairs.
- Rule engine: more custom rule conditions (positions, footprints, layers).
- More than two signal layers (pulled forward with the synthetic 4 layer benchmarks).
- Alternative placements as portfolio variants (smaller, square, round outlines with the
  connectors and the ESP32 module fixed), see `bench/place` (the `variants` example of
  `crates/gummiband-place`).
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
- Good practice around the router (idea from Daniel: the controlling agent sets the rules, the
  router does the heavy lifting, the agent finishes): `power_plan` (currents to widths, pours,
  planes, vias, `.kicad_dru` rules, widths as net options), `current_check` (narrowest band and
  total cross-section of every current path, vias included), `power_copper` (pours over the
  pads of high current paths and via fields into planes before routing, kept by the router as
  "gb:" zones), `ground_pour` (ground zones and stitching vias) and the prompt `good_practice` that runs a board end to end.
- The agent stays the layouter (idea from Daniel): auto-place and auto-route complement manual
  placement, manual routing and planes, and the agent refines after them. Done: `lock` (parts by
  reference, nets, all copper, and unlock), the section "Working with the tools" in MCP.md, the
  server instructions and tool descriptions, and `guide`, which returns the documentation through
  MCP (the workflow, every tool's reference, concepts, options).
- Gaps from a strict MCP run (the 4 port LED controller from its schematic only, 2026-11-06),
  closed: `create_board` overwrite removes the stale working copy and `parts_outside` keeps the
  board empty for hand placement and copper drawn first (84 parking moves before); `move_part`
  `side`; `place_parts` keeps a locked part named in `on_front`/`on_back` on its side with a warning
  instead of failing; `remove` (zones, tracks, vias, footprints); `set_rules` for `.kicad_dru`
  rules, and Gummiband's own check honours clearance rules that hold in courtyards;
  `power_copper` options for crowded power parts (`via_rings`, `via_in_pad`,
  `vias_in_courtyards`, `via_margin`: 25 of 26 vias where the defaults found 5);
  `current_check` adds the currents of parallel vias instead of their widths (two false flags
  gone); `place_silk` for reference texts; `try_seeds` over placement seeds; `add_mounting_hole`.
- Blocks (fixed blocks): a KiCad group or placed design block (parts plus their pre-routed locked copper, e.g. the datasheet
  layout of a converter) moves only as a whole under --move-parts; inside it nothing changes.

## M4: Search and performance

- Evolutionary search with a richer genome (routing order, via cost, layer preference, part
  offsets) and a fitness that includes the final DRC result.
- Benchmarks against Freerouting on the controller and the injector board.
- Profiling: incremental capacity checks, cheaper relaxation.
- Micro-optimisation and an open end that keeps the cores busy, 2026-10-31, on 556edbd, 4 cores in
  the cloud. Without an open end the results stay byte for byte the same: default run LED controller
  100 s instead of 133 s, Arty 88 s instead of 112 s. The open end makes 549 instead of 211 real
  mutants per minute on the LED controller and 474 instead of 263 on Arty: every core takes the next
  move until the generation has its 14 mutants, and the relaxation of a mutant takes about half the
  time. Stopped after 270 s, seeds 1 and 2, open end cost (2 per via, 0.05 per mm, plus the flaw):
  LED controller 968 and 961 instead of 991 and 1007 (246 and 246 vias instead of 258 and 271), Arty
  365 and 405 instead of 371 and 448 (111 and 128 vias instead of 114 and 151). Generations of 4
  mutants were not better (986, 976, 396, 400).
- Board caches, 2026-11-01, results byte for byte the same: every via site keeps its distance to the
  board edge, and the board grids of the relaxation (edge mask, obstacle cells, near obstacles) are
  built once per board instead of in every relaxation. Corne has an edge of 449 pieces in 74
  outlines: completing the two open connections of a finished run (`--keep-all --nets`, one thread)
  takes 283 s instead of more than 1200 s; in the open end (Mac, seed 1, 4 threads) generation 90
  instead of 7 after 20 minutes, 480/482 after 705 s instead of 1909 s. LED controller and Arty: no
  measurable change (the grids were 0.2 % of a run).
- Profiling and a proof that a search fails, 2026-11-04, on 9634088, Mac M4 Pro, 14 threads,
  default `route`. Results byte for byte the same on all 21 synthetic boards, the LED controller,
  Arty, Corne and ULX3S (SHA-1 of the routed file):

  | Board | before | first round | second round |
  | --- | --- | --- | --- |
  | LED controller | 53.6 s | 36.0 s | 18.8 s |
  | Arty | 36.3 s | 36.0 s | 31.8 s |
  | Corne | 581 s | 322 s | 304 s |
  | ULX3S | 366 s | 67 s | 39 s |
  | fine_pitch_mixed | 37.2 s | 28.1 s | 24.2 s |
  | four_signal_layers | 18.8 s | 12.4 s | 11.4 s |

  - A failing search walked every state it could reach before it gave up: in a short run on ULX3S
    58 000 of 204 000 searches took 93 % of all expansions (35 000 each against 1 000 for a
    successful one), on the LED controller 51 %. After 1000 expansions a search now walks from its goal (`route/shut.rs`):
    gaps between the crossings of an edge that have room with the loosest sides, via vertices and
    sites where the search may place a via, pads and holes of the net, joined where a track would
    not cross the chord of another net. Whatever the search can reach is reached there too, so a
    goal side without a start proves the failure. A goal shut in took 78 places in the middle, the
    largest of 108 000 on ULX3S 19 000 (budget 1000 places). A walk without chords proved almost
    nothing, one without the search's own via checks 14 % of the failures on the LED controller:
    the tracks and the vias that do not fit are what shut a goal in.
  - Corne: the relaxation and the clearance check looked at all 449 edges of its 74 outlines for
    every point and piece near the board edge (28 % of the run). Now per cell the edges in reach,
    per row of cells the edges of the even-odd test, a grid of the edges in the check.
  - Smaller: a four-ary heap of integer keys for the open list and stale flags instead of a map
    lookup per pop, vias from the sweep of the relaxation, the sweep list in key order, polish and
    via pass side by side.
  - Tried and dropped: only the points near what moved in the tension rounds (exact, but about
    65 % of the points creep by 1e-9 to 1e-3 mm every round, only 4 % by more than 1 um, so it was
    slower); tension across the band only (the creep stays); the route of the via pass ahead,
    next to the last geometry generation (the last generation finds a new winner in three of
    four searches); small changes in the search (u64 keys of the edge views, side lookups): not
    measurable.
  - Second round (review by three skeptics, four finders; all byte for byte the same again):
    a geometry candidate with fewer connections than the most of its generation never wins, so
    its relaxation is skipped or stopped (LED controller 36 to 20 s; its portfolio entry has no
    violation numbers); the router keeps every pin's pitch that `Board::pin_room` scanned all
    pins for in every search (ULX3S 18 % of the busy time); sweep buffers from a pool per thread;
    a goal found shut in is asked about at once in the next search of that connection; the via
    pass of a winner from the geometry generations is the end of a run on a copy of its router.
    The review found two cases where the proof could have been too strict (pad obstacles on a
    layer the pin does not name, via sites in a triangle of another net's mask opening); both are
    closed.
  - Dropped in the second round: relaxing equal geometries once (Arty had none); running the
    next geometry generation ahead on free cores as if the best stayed (the same files, Corne
    304 to 277-293 s, but ULX3S 39 to 44 s unless gated by rules fitted to these boards).
  - Where the time goes now: Corne 77 % of the busy time in negotiated searches of the second
    search the run starts on its own, whose geometry generations (about 80-100 s each) never
    produce a candidate as complete as the best; the LED controller mostly relaxation.
  - Experiment `GB_GEO_SCREEN=k`: a geometry generation relaxes all candidates with a quarter of
    the rounds and only the best k fully. With k = 4 the same file in all six runs (LED controller,
    Arty, Corne, seeds 1 and 2), LED controller and Arty about 12 % faster, Corne not. Off by
    default until more boards agree.

## Open: fine pitch fan-out (Corne)

Measured on 2026-10-30, none of these solved it: straight pin stubs (twice: worse on
`fine_pitch_mixed`, arty, LED controller), a larger router margin, extra gap outside the fine
pitch parts. A router margin of 0.06 mm brings Corne to 14 violations but only 392/482 routed:
the violations are the price of the last connections through the pin rows. Original widths
(0.2032 mm) did not change it either; the original design is clean under the same rules.

## Solder mask openings

KiCad reports `solder_mask_bridge` when a mask opening shows copper of two nets. The router did not
know openings, and on Corne the DRC stopped at its limit of 199 per type. Measured on 2026-10-31
with kicad-cli 10.0.7 and test boards:

- A pad's opening is the pad plus its margin (pad, then footprint, then `pad_to_mask_clearance`; a
  margin of 0 counts as not set). Copper of another net at the exact margin passes, 0.001 mm less
  is a bridge, for tracks and vias alike. `solder_mask_to_copper_clearance` adds to the margin.
- Openings of `allow_soldermask_bridges` footprints do not count, and neither does a bare drill.
  A non-plated pad wider than its drill opens the mask, a pad without net does as well.
- A shape on a mask layer is a bridge only with two nets in it. For the check it reaches as far as
  the board's margin beyond its edge, whatever the margin of its footprint; a pad without copper
  does not reach beyond its shape. A zone on a mask layer gave no bridge.
- Vias are tented on Corne and LED controller, so only their copper counts, like a track.

On Corne an opening is only that: pads with `pad_to_mask_clearance` 0.2 mm, seven footprints with a
margin of their own (0.09 to 0.19 mm), no shapes on the mask layers. The clearance is 0.19 mm, so a
rubber band that hugs a pad lies 0.01 mm inside its opening, and almost all bridges were that
shallow. Now a pad's margin plus 0.01 mm counts as its clearance where that is more, and the shapes
are keepouts on the outer layer. A shape with one net in it stays free for that net: such a net
draws real track there, which an own-net obstacle would not give it (`Mesh::opening`).

Corne, `--geometry-generations 1`, default seed and seeds 1 to 4 (`main` is the state before):

| | open connections | `solder_mask_bridge` | of these from routing | spots of the own check | DRC `shorting_items` |
| --- | --- | --- | --- | --- | --- |
| main | 8, 10, 8, 7, 6 | 199 (the limit) | 1388, 1462, 1599, 1396, 1509 | 57, 85, 84, 63, 79 | 4, 5, 9, 0, 9 |
| now | 9, 11, 12, 13, 9 | 20, 17, 14, 16, 18 | 16, 13, 10, 12, 14 | 54, 62, 54, 72, 58 | 0, 0, 0, 1, 0 |

- The bridges from routing are counted from the geometry for main, as the DRC stops at 199; the
  count matches the DRC on all runs below the limit. The unrouted board has 4 of its own (a pad
  without net beside pad S2 of EXSW1, EXSW2 and their right hand twins), the router cannot change
  them.
- The price is 3 connections on average (7.8 to 10.8 of 482 open). What stays in the openings are
  spots the own check reports as well: bottlenecks at Y1 and vias next to capacitors.
- Room of 0.003 mm instead of 0.01 keeps the connections (7.4 open), at 70 instead of 60 spots and
  nearly twice the shorts of main; 0.005 mm has the open connections of 0.01 with 78 spots. The
  search is too chaotic for more than that from five seeds. Bridges stay at 10 to 13 per run in
  all three.
- LED controller has a margin of 0: byte-identical (`50c4f3cfb765`).
- A synthetic board with shapes, text and a pad without copper in the way: 12 bridges before, 0
  after, all connected.

## Next: search quality

- Open end: `route --until-stop` (see ARCHITECTURE). Single runs on 2026-10-30, 150 s in all,
  against the order mutation it replaced, which re-routed the whole board for every mutant:
  LED controller seeds 24301, 1, 2 reach 219 to 224 vias against 272 and 293 of that mutation (DRC:
  0 open, 3 to 10 clearance errors against 3). Arty 323/323 and 126 vias against 322/323 and 214.
  The original Arty has 147 vias, 3073 mm, tracks of 0.182 and 0.2 mm, and 217 mm on inner layers;
  we route it with 0.15 mm in 2884 mm on two layers. With the original widths as a hard rule: 321/323,
  161 vias, 2970 mm. With `--prefer-width 0.25` instead: 323/323, 126 vias, 2904 mm, 91 % of the
  length at 0.2 mm or more, 72 % at 0.25 mm. No run so far on Corne and ULX3S. Starting without
  geometry generations leaves more violations for the open end to work off.
- Text on copper is an obstacle since 2026-10-30, as a box. Arty then has no routing errors in the
  DRC; the last connection `/CRVL_IO1` depends on the run: 323/323 after 5 minutes in one run,
  322/323 in others. The box is a little larger than the letters.
- Spots instead of pieces in the open end, 2026-10-30. Arty, 180 s with `--prefer-width 0.25`:
  323/323, 123 vias, 2952 mm, 73 % at 0.25 mm, KiCad DRC clean but for the 4 errors of the empty
  board. LED controller, 180 s: 348/348, 236 vias, 2591 mm, 1 spot, 2 DRC errors.
- Corne with tracks on the middle of their fine pitch pads and solder mask openings as obstacles,
  `--geometry-generations 1`, 45 minutes, not converged: 477/482, 371 vias, 8106 mm, against 452
  vias and 8352 mm of the complete original. DRC: no clearance errors, shorts or crossings, the 4
  solder mask bridges of the unrouted board, 1 at the edge. Before: at least 378 clearance errors,
  at least 199 bridges. Nearly all hits had been within 0.5 mm of a pad centre: neighbours pushed a
  track 20 um off the middle of its own pad, next to the 10 um that a 0.2 mm track has at 0.4 mm
  pitch. Rails 0.8 mm beyond the pad were worse, they take the room to fan out. Neck extensions, more
  reserve and leaving out the tightening did little. Open: the own check reports 4 spots at
  `D+`/`D-` against GND that KiCad accepts, not understood.
- All of it together, 2026-10-31, all cores, one board after the other, default seed:
  Corne `--geometry-generations 1`: 482/482, 391 vias, 8526 mm, KiCad DRC 0 open, 0 clearance, 0
  shorts, 1 edge error, the 4 mask bridges of the unrouted board, stopped at 90 minutes at level
  10 of 12. The original has 452 vias and 8352 mm. Arty `--geometry-generations 3 --prefer-width 0.25`:
  323/323, 106 vias, 2927 mm, 69 % at 0.25 mm, DRC clean but for the 4 errors of the empty board,
  converged in 9 minutes. The original has 147 vias and 3073 mm. LED controller
  `--geometry-generations 3`: 348/348, 200 vias, 2523 mm, converged in 9 minutes, 2 clearance hits at
  one spot. The original has 294 vias and 2633 mm. Each also leaves results that differ in vias and
  length to choose from.
- Tracks that end on a track of their net, 2026-11-01. Arty until convergence, with the width of
  0.25 mm preferred, no routing errors in the DRC in all three: without them 106 vias, 2927 mm; with
  them at 40 mm per via 71 vias, 3203 mm; at 20 mm per via 80 vias, 3083 mm. Same-net tracks next to
  each other went from 176 to 24 mm (measured on the 40 mm run), mostly plane fan-outs of GND.
  LED controller: 200 vias, 2523 mm; 122 vias, 2864 mm; 123 vias, 2848 mm, one spot in each. Single
  runs. Fewer vias cost length: the results on offer reach 125 vias with 2945 mm on Arty and 236
  vias with 2460 mm on the LED controller.
- KiCad's DRC stops at 199 or 499 hits per type, so those are limits and not results.
- Idea to keep: a strict convergence, where every gain in vias or length starts the boldness over
  as well. Arty then still lost a via every few minutes after 18 minutes and never converged, but
  for a long overnight run that may be what one wants, as an option.
- Corne, 400 s with `--geometry-generations 1`: 476/482, 388 vias, 8086 mm, 41 spots, against 452
  vias and 8352 mm of the complete original. Nearly all hits are tracks of 0.2 mm against tracks of
  0.2 mm or pads, with a median of 0.010 mm missing against 0.19 mm. The QFN has a pitch of 0.4 mm,
  which leaves 0.2 mm next to a track of 0.2 mm, and tracks that leave it in parallel lose that with
  the angle, from about 13 degrees below the rule. Not proven yet; the idea is a straight exit from
  the pin row before the fan-out. Longer necks and more reserve did not help, neither did leaving
  out the tightening in that pass. KiCad's DRC stops at 199 or 499 hits per type, so those are
  limits and not results. Solder mask openings are no obstacles for the router.
- Aimed moves for open connections in the open end, 2026-10-31 (see ARCHITECTURE). The search did
  find topologies with one more connection: on Corne 288 of 791 moves in one run, but only 3 of
  84 in a closer look beat the result, the others brought 4 to 20 shorts. So most of the gain
  comes from the start of the relaxation (new tracks between their real neighbours, new vias past
  the tracks they lie behind) and from a side branch that works the shorts off, the move only
  supplies the candidates. Measured with `--geometry-generations 1`, 4 threads, three runs side by
  side, until convergence or 1500 s, before and after, routed, vias, spots:
  - Corne seed 1: 473, 361, 24 and 479, 405, 39. Seed 2: 476, 383, 35 and 481, 391, 34. Seed 3:
    481, 404, 36. KiCad reports 9 and 6 unconnected before, 3 and 1 after.
  - Arty seed 1: 322, 137, 0 and 323, 117, 0 (one build before the last fixes). Seed 2: 322, 123,
    0 (150 s, all cores) and 323, 119, 0. KiCad: 0 unconnected instead of 1, the 4 hole clearance
    errors of the empty board stay.
  - LED controller seed 1: 348, 201, 1 and 348, 209, 2. Seed 2: 348, 191, 1 and 348, 176, 0, DRC
    clean instead of 10 errors.
  - Generations per second on Corne 0.25 to 0.41 instead of 0.42 to 0.46, because more mutants get
    as far as the relaxation; Arty and LED controller unchanged within the noise.
  - Not proven, single seeds: the cheaper via (without it Corne seeds 1, 2: 482 and 480, with 433
    vias, against 479 and 481 with about 400 vias) and the chain of `clear_ways` (without it
    481 and 481). A run without the immediate side branch reached 480 on seed 2, but 846 s into
    the open end against 38 s. The one complete Corne, 482/482 with 0 unconnected, came from the
    run without the cheaper via; its clearance errors are as many as in the other runs.
  - Open: Corne keeps its clearance errors (KiCad's limit of 499 is reached in most runs), and the
    last connection on most seeds is `/right/D+` from `rR4.1`, whose pad lies in a pocket: it has
    to leave through a via in the pad. A site that had a via once stays split, so a via in a pad
    that was ripped up cannot come back to that site.
    Measured on 2026-11-01 (cloud: normal search, 2 threads; Mac: open end of 20 minutes, 4 threads,
    KiCad DRC; seed 1 unless named): allowing the vertices of a split pad site on the other layers
    again (`GB_PAD_REUSE=1`, off by default) routes `/right/D+` through a via in the pad as in the
    original, 478/482 instead of 476/482 in the cloud; on the Mac the open end then loses
    `/right/QSPI_SD2` instead (480/482). On seeds 2 and 3 (one thread) `/right/D+` is routed without
    it, and it changes nothing. Naming the net (`GB_VIP_NETS=/right/D+`: a via in the pad at any
    time) gives the first complete Corne of the series, 482/482 on the Mac, KiCad 0 unconnected and
    12 errors. `GB_VIP_LEARN=1` (a net that needed the last resort once may take it at any time)
    learns 40 nets on its way, ends the normal search with 472 instead of 474, and the open end does
    not start: the learned nets outlive the run, so it does not repeat. `GB_VIP_RETRY=1` learns from
    the result instead: the winner again, in its own order, with a via in a pad at any time for the
    nets it left open. Cloud: 477/482 instead of 476/482; seed 2: 475 either way (`/right/D+` routed
    without it); seed 3: 475 either way (`/right/D+` routed without it). Mac: 480/482 (open
    `/right/KEY4`, `/right/USB_VBUS_PIN`), KiCad 2 unconnected and 116 errors against 15 without the
    switch, 67 instead of 90 generations in 20 minutes. `GB_VIP_RETRY=each` first routes the winner
    once per open net, with a via in a pad for that net alone, and keeps only the nets that then
    route more. It keeps two of the eight, and the run is the same as with all of them, byte for
    byte in the cloud and generation by generation on the Mac: the other nets never took a via in a
    pad. What costs is the late decision; `GB_VIP_NETS` decides before the search and completes
    Corne. `GB_VIP_EARLY` (9ce567f) looks for the net before the generations and finds none, in the
    cloud with seed 1: on the empty board no connection between pins needs a via in a pad, D+
    included (`=empty`), so the tracks of its neighbours close the pocket, not fixed copper; after
    the portfolio 14 nets are open, D+ among them, and none routes more on its own with one (`=1`).
    Open: the via in a pad as a gene of the search, which the mutations give and take. Measured
    again on 2026-11-01, once the search negotiates by itself (PR #55): no bench board needs it.
    Corne is complete through the negotiation, and `bga_dense`, which keeps 8 of 100 connections
    open, gives the same file byte for byte with a via in the pad at any time for those eight
    nets (`GB_VIP_NETS`, seed 1). So it waits for a board that shows the case.
- Negotiated routing: done as an option, `route --negotiate N` (see ARCHITECTURE). Measured on
  2026-10-30 with `--runs 14 --generations 1 --negotiate 20`, KiCad DRC against the plain run:
  `fine_pitch_mixed` 0 open and 0 errors instead of 8 and 7, `four_signal_layers` 0 open instead
  of 4, `bga_dense` 8 instead of 12, Arty 2 open and 2 errors instead of 3 and 11, Corne 1 open
  instead of 12 and 669 errors instead of 702 (own check 1269 instead of 2289). LED controller
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
  threads in and relaxation pushes aside only what it touches. On the LED controller follow-me
  test LED_D3 goes from 0/1 to 1/1 with 0 clearance errors, 18 tracks moved by at most 0.32 mm,
  the other copper unchanged in the file. 11 of 12 other nets, stripped and finished one by one,
  are clean as well. Open: in the tightest spots the relaxation can still pull a track over a
  neighbour (USB_DN at the USB-C pins, Arty /~{MR}: the clearance check reports it); a dense
  board with no room in the topology stays unrouted (ULX3S, `--give` is the fallback there); a
  new connection may leave kept copper a little before its open end, which leaves a stub.
- Finishing nets uses shove and give by default (2026-11-01, `gummiband_core::finish_nets` for
  the CLI and the MCP server; `--no-shove` and `--give 0`, MCP `shove: false` and `give: 0`
  switch them off). Give is a fallback: every circle is a try, and the try with the fewest open
  connections, then violations wins, the earlier one on a tie. A circle that takes in no new net
  is not routed again. Night series data (one net stripped and laid again on the main result of
  seed 1, 2 threads): plain left LED_D0, LED_D3 and BUCK_EN open on the LED controller and UART_EN on
  Arty, shove laid all 16 nets, near the old length, three to four times faster on Arty. Give
  alone within 3 mm completed three of the four but broke four connections around BUCK_EN; the
  new rule keeps the try before in such a case. With the new defaults all 16 nets give the same
  file as `--shove` before, byte for byte.
- Finishing nets open ended (2026-11-01; `--until-stop` with `--keep-all --nets`, MCP
  `open_end_seconds` with `keep_all` and `nets`): shove and give no longer exclude the open end.
  The tries of give run without it, and the one kept runs again, the same search first, so the
  open end goes on from where that try ended; if it ends worse (a stop during its search part),
  the try stays. The open end works on the soft copper (its rebuild repeats the router run on
  it, `soft_tagged`), its own passes let soft copper sleep until routed copper pushes it, and the
  air reserve stays 0, as it would move tracks of other nets that the new ones do not touch (270
  on Arty). Night series data (the 16 nets above, 30 generations of open end, 2 threads, KiCad
  DRC with zones refilled): today's open end, which keeps the other copper where it is, leaves
  LED_D0, LED_D3, BUCK_EN and UART_EN half open (12 of 16 complete); open ended with shove all 16
  are complete, KiCad 0 unconnected everywhere, LED controller 0 errors and Arty only the 66 the
  board brings along, the alternatives as well. Against finishing without the open end: 22
  instead of 25 vias on the nets laid, 596 instead of 576 mm (BTN_BOOT trades two vias for 22 mm,
  OUT1_DATA one for 4 mm, five nets get shorter), 218 instead of 189 tracks of other nets moved.
  A normal open ended run (Arty seed 1, 20 generations) gives the same file byte for byte.
  A first version wrote the soft copper that the passes of the open end had put to sleep where
  the file had it, not where it lay: KiCad found clearance errors in 12 of the 16 files and
  shorts in three, which the own check could not see, as it reads the geometry. Now the place
  decides what is written again (`import::rest_in_place`), and an alternative gets its own list
  of what moved. With that, a resting track that the taper narrowed is written as well: VBUS_IN
  at R61 on the LED controller kept KiCad's three errors of 1.5 µm in every finished file, also
  without the open end, now 0. The files of finishing without the open end change only there;
  Arty stays byte for byte.
- Tapering at tight spots is on by default (2026-11-01, `improve::taper_spots`, `GB_TAPER=0`
  switches it off): a line or arc that misses its clearance by at most 50 µm gets narrower, never
  below the minimum width of its class and never wider anywhere. LED controller is the only bench
  board with minimum width rules (USB pair, input and output paths); seeds 1 to 4 on the main of
  that morning (before parts moved by default), own check: plain runs 10 instead of 20
  violations, the open end with 60 generations 0 instead of 10 (seed 3 had nothing to taper). The
  KiCad numbers are those of the current paths below, which came in the same pull request. The
  first version narrowed everything in a
  square of 0.4 mm around the spot, and a whole arc whose chord reached it got that width: on
  seed 3 a neck of VBUS_F at the USB chip went from 0.25 to 0.72 mm and came 0.15 mm too close to
  a pin, which only KiCad saw, because the own check measures the pieces of an arc and the file
  has one width per arc. Now the width goes to whole elements. Open: what remains are clearances
  of classes without a minimum width (VBUS_P at U6 pad 10, `PowerStage`) and copper slivers of
  zones.
- Current paths keep their width (2026-11-01, Daniel's question about the neck of VBUS_F). A neck
  that a minimum width rule allows in a courtyard is for the pin there: it ends at the edge of
  the courtyard, half the track width outside it, because KiCad tests the shape of the track
  against the courtyard. The own check measures the widest way through the copper between two
  pins that take the minimum width, vias counted by IPC-2221 (`current`), and a layer change on
  such a way gets as many vias as carry its current (`relax::stitch_vias`, `GB_STITCH=0` switches
  it off): in a row on the straight end of a track, or around the via with a piece of copper on
  each layer. The router counts those vias in the price of the layer change. Nets of a
  differential pair are left out: their width is a matter of impedance. LED controller, seeds 1
  to 4 plain and 1, 2, 4 with the open end (60 generations), the main before PR #45 against the
  same main with tapering and these changes, KiCad DRC with zones refilled: 2 errors instead of
  15, no unconnected item in either series. Main: the VBUS_P neck at U6 pad 10 in one run (10
  errors, 19 µm), four copper slivers, a dangling track. Branch: a clearance of 1.6 µm between a
  pad of R61 and a via, and a crossing that KiCad reports for a track of no length on a via
  (from the open end, also in main's files; such tracks are no longer written). The own check
  finds a current path short of its width in 4 of the 7 branch runs: a layer change with a
  single via where nothing fits next to it (USB_DP, the CC1 vias and R112 around it). Open:
  reserve the room for those vias in the search. On the way the router lost a run to a panic: a
  via of a chain split the triangle that another via of the same chain leaves through; such a
  chain is refused now, and a run that panics all the same counts as lost instead of ending the
  search.
- The search negotiates when the plain one leaves a few connections open (2026-11-01,
  `route_fixed`, `GB_AUTO_NEGOTIATE=0` switches it off): at most 15 % open, a second search with 20
  rounds of negotiation, and the better result counts, as with `--negotiate 20`. Seeds 1 to 3 in
  the cloud, KiCad's DRC on the Mac with zones refilled, unconnected items and errors with the
  second search against without: `fine_pitch_mixed` 0 and 0 in every seed instead of 4 to 7 and 23
  to 32, `four_signal_layers` 0 and 0 instead of 3 and 0, `bga_dense` 8 instead of 10 or 11 open
  (no errors in either) with 45 to 49 vias instead of 62 to 67. Corne, seed 1: 482/482 with 313
  instead of 475/482 with 397 vias; KiCad 0 unconnected and 501 errors (478 clearance) instead of 7
  and 576 (clearance at KiCad's limit of 499, 15 shorts, 4 crossings, 32 hole clearances). The
  Corne figure comes from the negotiated search alone (`--negotiate 20`, `GB_NEG_ONLY=1`), which is
  the one the default keeps there. A search that completes the board does not negotiate: the ten
  other synthetic boards, Arty and LED controller seeds 1 to 4 give the same files byte for byte.
  The first version negotiated as soon as the order generations left a connection open, before the
  geometry generations, and took the negotiated generations from there; LED controller seed 3 then
  came out complete with 328 instead of 265 vias, because the geometry generations of the plain
  search find the last connection themselves. Open: ULX3S and other boards with more than 15 %
  open, an open ended run (the open end works on what is open), and Corne's clearance errors at its
  fine pitch parts.
- Adaptive track width: as wide as the surroundings allow, instead of one neck width per zone.
- A new via in the plane of its net stays in it (open end, 2026-11-01, default since 60ab967,
  `GB_PLANE_HOLD=0` switches it off). `place_new_vias` moved such a via along its track past a
  sleeping track and out of the plane; the connection still counted as made through the plane, KiCad
  found it open and the via dangling. In 4 of 49 finished runs of the night series without the hold
  that reported every connection, on the LED controller (J10 pad 2, `+5V_A`) and Arty (U2 pad 24,
  `/FTDI_3V3`), also with the settings of main. With the hold: 0 of 5; the cost after 20 minutes
  against A without it, seed by seed: LED controller +0.9 %; Arty +3.5 %, +3.1 %, +3.8 %; on Arty at
  the same generation of the open end (700): -0.8 %, +0.5 %, +4.3 %, so part of the price is speed
  (the runs reached 0.61 to 0.79 generations per second). Open: count a plane connection as made
  only when its via lies in the plane, checked in the judgement of every mutant and at the end of a
  run; that would keep the moves that leave the net connected another way, which the hold forbids as
  well.
- Night series of open end ideas, 2026-11-01: Mac, 20 minutes per run, three runs of 4 threads side
  by side, cost 2 per via, 0.05 per mm and 10000 per open connection, KiCad DRC at the end.
  Combination A is on by default since then (`GB_BANDIT=0`, `GB_AIM_COST=0`, `GB_PRUNE=0` switch
  it off); the other switches stay off (`improve::Ideas`).
  - Combination A (`GB_BANDIT=1 GB_AIM_COST=0.25 GB_PRUNE=1`) against main, median over the seeds:
    LED controller -12 % (4 and 4 seeds), Arty -15 % (3 and 3); fewer vias, a little more length;
    KiCad DRC clean in 5 of 7 runs against 3 of 7. Seed by seed: LED controller -13.3 %, -11.5 %,
    -5.1 %, -13.8 %; Arty -9.7 %, -17.6 %, -15.5 %.
  - A without geometry generations (`--geometry-generations 0`) against A: LED controller -4.9 %,
    -6.1 %, -3.5 %; Arty -4.2 %, +3.3 %, +0.2 %; the open end starts at about 17 s instead of 42 s.
  - B (A plus `GB_MERGE`, `GB_SPOT_FADE`, `GB_TAPER`) against A: LED controller -1.7 %, -3.9 %; Arty
    +1.0 %, -0.3 %.
  - Single ideas against main, seed 1, `GB_VIA_PRICE`: LED controller -7.5 %, Arty -2.5 %;
    `GB_BANDIT`: LED controller -7.9 %, Arty -6.3 %; `GB_AIM_COST=0.25`: LED controller -8.7 %, Arty
    +7.5 %; `GB_PRUNE`: LED controller +2.9 %, Arty -1.5 %; `GB_MERGE`: LED controller +1.5 %, Arty
    +3.7 %; `GB_SPOT_FADE`: LED controller -1.9 %; `GB_TAPER`: LED controller 0.0 % (KiCad DRC clean
    against 3 errors: the spot at R61 is gone).
  - `--via-cost 10` on main: LED controller -6.5 %, -16.8 %; Arty +2.5 %, -11.6 %; on top of A:
    LED controller 0.0 %; Arty +5.4 %.
  - 12 threads in one run (LED controller, main, seed 1): cost 468.7 after 20 minutes with 1.6 times
    the generations of 4 threads; the best of three seeds with 4 threads: 454.0.
  - Mutant trace of A (LED controller, seed 1, `GB_MUTANT_LOG`): moves at random places bring 78 %
    of the saved cost in 63 % of the time; moves at clearance spots mostly remove the flaw penalty;
    93 % of the saving is in after 10 minutes.
  - ULX3S, 40 minutes: main and A both 655/796, A with 305 instead of 370 vias. The open connections
    are no search problem: the original routes signals on In2.
- Levels of the open end, cloud, 2026-11-01 and 02: 4 cores, one thread per run, 45 or 80 minutes,
  Gummiband's own check (no `kicad-cli` for KiCad 10 boards there), cost as in the night series.
  - `GB_PRUNE` dropped the starts of side branches too. With clearance spots on the board every
    start came to a mutant (51 of 51), without spots nearly none did (4 of 151): a mutant counts
    as hopeless when it adds vias away from every spot. The level then climbed by one per
    generation, since stale is not reset by a failed start: Arty, seed 1, level 1 at generation
    228 and 12 ("converged") at 274, on to 23; on a71bdaa Arty seed 2 went from 1 to 4 in four
    generations. Since 2026-11-02 the starts are not pruned (`GB_PRUNE_STARTS=1` prunes them
    again). Cost after 45 minutes against pruned starts, on bcca477: LED controller +1.7 %,
    -0.8 %, +1.9 %, Arty -0.4 %, +0.9 %, +0.7 % (seeds 1 to 3); on a71bdaa: LED controller the
    same bytes on seeds 1 and 2, Arty the same on seed 1 and +0.9 % on seed 2. With an open
    connection it helped once: LED controller, open end alone with short branches
    (`GB_BRANCH_AFTER=8 GB_BRANCH_LIFE=8`), seed 1, found the last connection after 4393 s
    through a side branch, without it not in 80 minutes.
  - Measured and not taken (bcca477): `--geometry-generations 0` before the open end,
    LED controller -9 %, -12 %, +7 %, +4 %, Arty +11 %, -1 %, +8 %, -1 % (seeds 1 to 4), Corne
    480 instead of 481 of 482 (seed 1, 2 threads, 95 minutes); the share of aimed moves following
    how often they came to a mutant (at least a fifth), LED controller open end alone: the last
    connection after 2553, 3564, 1011, 1689 s against 4181, 1890, 2789, 567 s, cost at the end
    higher on 3 of 4 seeds, Corne 479 instead of 481 of 482. A longer ladder while a connection is
    open (16 levels, the areas growing on beyond a quarter) gave the same boards, but the pruned
    starts hid it: above level 12 no start came to a mutant (0 of 117), all on boards without
    spots. With the starts unpruned they did (30 of 30, up to level 26), so the ladder is still
    open to measure.

## Local patterns

`gummiband_core::patterns` (see ARCHITECTURE) finds pin rows, pad grids, decoupling capacitors,
pins of a plane net that could share a via, and buses, and `route --patterns` turns three of them
into hints, all off by default. Measured on 2026-11-01 in the cloud on 4 cores, on main before
PR #38 (no `--move-parts`), with Gummiband's own clearance check (no `kicad-cli` there): cost as
in the open end, 2 per via, 0.05 per mm, 10000 per open connection. Detection on the boards:
LED controller 11 rows in 6 parts, 14 decaps, 8 groups that share a via; Arty 38 rows, 41 decaps,
16 groups; Corne 20 rows, 10 decaps, no planes; ULX3S a 20 x 20 grid at 0.8 mm with 379 pins;
`bus` one kept bus of 16 nets, `crossing` one of 16 nets in reversed order.

Normal search (default settings, seeds 1 to 3), mean:

| Board | none | `decaps` | `shares` | `shares,decaps` |
| --- | --- | --- | --- | --- |
| LED controller | 288 vias, 2610 mm, 1.3 spots, cost 707 | 267 vias, 2524 mm, 2.0 spots, cost 660 | 289 vias, 2565 mm, 2.7 spots, cost 706 | 269 vias, 2562 mm, 3.0 spots, cost 667 |
| Arty | 0.67 open, 181 vias, 2934 mm | 0.33 open, 185 vias, 3033 mm | 0.33 open, 256 vias, 3205 mm | 0.67 open, 228 vias, 3218 mm |
| Corne (seed 1) | 6 open, 385 vias, 8190 mm, 82 spots | 6 open, 529 vias, 8538 mm, 119 spots | | |
| `qfn_fanout` | 46 vias | 46 vias | 47 vias | 47 vias |
| `bga_dense` | 10.7 open | same | same | same |

- `decaps` is the only one that gains on the LED controller (-7 %), at a spot more; on Arty it is
  even, on Corne much worse in the one seed measured. Not a default.
- `shares` costs Arty 70 vias, mostly on signal nets near the bridged pins: it is the bridges
  between neighbouring pins of one part (`GB_SHARE_KIND=neighbours`: 251, 247, 245 vias), not the
  ones between an IC pin and its capacitor (`GB_SHARE_KIND=decap`: 196, 177, 138 vias, but 3374
  instead of 2934 mm). The router already shares a via that lies close, at 0.05 mm of track, so a
  bridge brings little and takes room at the pins. A bridge that takes a via or a detour falls
  back to a via of its own (`Router::detour_bridge`); that changed nothing measurable.
- Pockets were left out: the via in a pad experiments above show that the empty board does not
  show them.

Open end with and without `rows` (`--until-stop`, `GB_OPEN_GENERATIONS` 300 on the LED controller, 150
on Corne with `--geometry-generations 1`), same work per run:

| Board, seed | none | `rows` |
| --- | --- | --- |
| LED controller, 1 | 166 vias, 3089 mm, 1 spot, cost 486.5 | 160 vias, 3067 mm, 1 spot, cost 473.4 |
| LED controller, 2 | 177 vias, 2976 mm, 1 spot, cost 502.8 | 177 vias, 2932 mm, 1 spot, cost 500.6 |
| Corne, 1 | 481/482, 365 vias, 8590 mm, 4 spots | 482/482, 347 vias, 8704 mm, 5 spots |

A small gain on every run, Corne complete; three runs are not proof, the search is chaotic. Next:
`rows` in the night series on the Mac with the KiCad DRC and more seeds, before it becomes a
default. Not built yet from the concept: pad grid via sites and quadrant directions, local
preferred directions per tile, buses as bundles (after the differential pair concept).

## Dog-bone via sites in ball fields

On the DDR5 testbed CA5 (and in other seeds CA7, DQ5) stayed open: a via between the balls sat
wherever the 0.4 mm raster had a site, mostly off the diagonal, and a GND track to it closed the
channel of the ball next to it. `patterns::grids` now finds ball fields from the lattice of SMD
pads (also fields of short rows, like the two 3 column blocks of a DDR package; the pad grids of
`patterns::find` need rows of 4), and `build_sites` adds a via site in the middle between every
four balls and half a pitch outside the outer ones. Measured on 2026-11-02 in the cloud on 4
cores against main (e92bba4), KiCad DRC:

- DDR5 testbed, `--pairs --no-move-parts`, seeds 1 to 8: every net routed in every seed (main:
  11 open nets in 8 runs); errors 155 instead of 144, all of them more skew errors: with every
  net routed, single nets far too long (CLK_N 48 mm in seed 8) set the aim of their group, and
  tuning stretches the others towards it.
- Synthetic boards, default and `--pairs`: `bga_dense` 0 instead of 8 open connections, in 12
  instead of 304 s (392 s with pairs), every other board the same.
- Dog-bone sites only (the raster removed within 0.75 pitch of a ball): DDR5 seeds 1 to 4 had
  118 instead of 67 errors, with clearance errors where the search led two tracks between two
  neighbouring dog-bone vias. Not kept.

Next: the overlong nets of a length group, CA and CLK first (a CA/CLK skew costs more than a DQ
one, see the JLCPCB DDR5 layout guidelines).

## Skew need in the search cost

A single net routed the long way round sets the aim of its whole skew group, and tuning then
has to stretch every other net of the group towards it, often without room. `RunStats::skew_need`
is what tuning would have to add: per skew rule (not within pairs), the mm each net lacks to the
longest net of the rule beyond the allowed skew, track lengths only. `cost()` adds it with weight
0.5 (`GB_SKEW_WEIGHT` sets another one), so the search prefers routes of similar length. Measured
on 2026-11-03 in the cloud on 4 cores against main (337ff24), KiCad DRC:

- DDR5 testbed, `--pairs --no-move-parts`, seeds 1 to 8: 147 instead of 155 errors, skew errors
  73 instead of 108, every net routed. Weight 0.2 on seeds 1 to 4: skew 47 instead of 31; 1.0:
  the same as 0.5.
- Synthetic boards: `matching` 3 instead of 6 errors (default) and 6 instead of 12 with
  `--pairs` (own check 0 instead of 17), every other board the same.

Tried and not kept:

- The skew need only in the choice among relaxed candidates, not in the search: seeds 1 to 4
  75 errors, skew 50; the effect comes from the early choices.
- Vias on dog-bone places pinned in the relaxation: on a copy of the testbed with the DDR
  package 5 mm further from the connector 64 instead of 76 errors (seeds 1 to 4), on the testbed
  itself 363 instead of 147 (seeds 1 to 8), clearance errors below the field where the
  capacitors leave no room.
- Room for meanders with no new clearance spot inside a ball field (the push squeezed two
  tracks between dog-bone vias in one seed): 182 instead of 147, because in two seeds every push
  then failed and the meanders had no room.
- Return vias (a GND via next to every signal via that changes the reference plane): the
  testbed has one GND plane, so they only dangled. Set aside.

## Routing order: pairs and part to part groups

The first topology routes the connections in the order of `Router::plan()`: priority, then the
jittered length. Two opt-in hints (`--patterns pairs,groups`, also in the MCP and the plugin
patterns) change that order. Without them the plan and the result stay byte-identical (Mac,
seed 1, all 16 bench boards).

- `pairs`: pairs come first, the longest first, the partners next to each other. With `--pairs`
  a pair is one wide net, so it moves only as a pair.
- `groups`: connections between two parts with 4 or more pins (chips and connectors) that share
  at least 2 nets form a group, routed as a block, the members in order across the band so they
  do not cross. `GB_GROUP_ORDER` picks the order of the blocks: `long` (longest first, the
  default of the hint), `short`, or `inline` (blocks in place, only the members together).
- New synthetic boards in `bench/gen_boards.py`: `pair_wall`, `pair_traffic`, and part to part in
  rising steps `parts_easy`, `parts_medium`, `parts_hard`, `parts_multi` (MCU and PHY with a 12
  net bus, peripherals, three pairs), `parts_multi_feed` (40 scattered 0603 parts in the way),
  `parts_xhard` (the same on a board 25 % smaller in each direction).

KiCad DRC on the Mac, seeds 1 to 3, means (2026-11-02, 210 runs):

| Board | none | `groups` long | `groups` short | `pairs` |
| --- | --- | --- | --- | --- |
| parts_hard | 56.3 vias, 1106 mm | 48.7 vias, 1070 mm | 54.3 vias, 1070 mm | |
| parts_multi | 66.7 vias, 1399 mm | 58.7 vias, 1424 mm | 84.0 vias, 1330 mm | 71.0 vias |
| parts_multi_feed | 126.7 vias, 1902 mm | 98.0 vias, 1897 mm, 1 pair gap error | 115.0 vias | 119.7 vias |
| parts_xhard | 103.0 vias, 1324 mm | 101.3 vias, 1393 mm | 115.7 vias | 104.7 vias |
| crossing | 26.0 vias | 14.7 vias | 14.7 vias | |
| matching | 28.0 vias, 2.7 errors | 5.3 vias, 11.0 length and skew errors | same | |
| pair_traffic | 2.0 vias, 581 mm, 4.0 mm uncoupled | | | 6.7 vias, 503 mm, 2.2 mm uncoupled |
| diff_pairs | 1.7 errors, 43 mm uncoupled | | | 5.3 errors, 21 mm uncoupled |
| LED controller | 0 clearance | 0.3 clearance | 5.3 clearance | 0 clearance, 61 % coupled (42) |
| Arty | 161.3 vias | 165.0 vias | 175.3 vias | 159.7 vias; `pairs,groups` 141.0 |
| DDR5 | 23.3 errors | 20.7 errors | 19.0 errors, 145 vias (125) | 24.3 errors |
| Corne (seed 1) | 181 errors | 171 errors | | `pairs,groups` 323 errors |

- `groups long` is the best order on boards with several chips: fewer vias, a few percent more
  length. `short` hurts LED controller, `inline` is the weakest overall. The small boards stay
  neutral (plus or minus one via).
- `pairs` shortens and couples the pairs better, but costs vias and DRC errors on diff_pairs.
- No hint wins everywhere, so both stay opt-in.

## Placement: room for track groups

- `place --corridors`: the annealer knows the track groups (parts with 4 or more pads sharing 2
  or more short signal nets) and keeps a band between the two parts free: other parts in it pay
  depth times length. Bands between two fixed parts count too.
- `place --escape-halo`: instead of the uniform 0.3 mm halo, each side of a part with 5 or more
  pads gets a skirt as deep as its signal pins need to leave (sum of track and clearance per
  routing layer, at most 3 mm). Plane pins do not count.
- `route --clear-corridors`: before routing, movable parts in a band that are not on the group's
  nets are placed again (`Scope::Only`, corridors on). Decoupling capacitors (two pads, both on
  supply nets) stay at their IC.

Placement from scratch, then `route --patterns groups` (long) seed 1, placement seeds 1 to 3,
no DRC (cloud):

| Board | none | `--corridors` | `--escape-halo` | both |
| --- | --- | --- | --- | --- |
| parts_hard | 0.33 open, 46.7 vias, 562 mm | 0.67 open, 39.3 vias, 535 mm | 0 open, 42.0 vias, 659 mm | 0 open, 38.7 vias, 643 mm |
| parts_multi_feed | 1.33 open, 92.0 vias, 844 mm | 1.00 open, 86.7 vias, 808 mm | 0 open, 86.7 vias, 899 mm | 0 open, 93.7 vias, 928 mm, 1 violation |

- The annealer packs the parts tight for short wires, so the router misses a connection now and
  then. The escape halo removes that at about 15 % more length; the corridor term saves vias.
- Both together are best on parts_hard; on parts_multi_feed one seed (110 vias) pulls the mean up.
- `route --move-parts` changes nothing on these placements: it reacts only to violations and open
  connections of the geometry generations.

`route --clear-corridors --patterns groups` (long), seeds 1 to 3, no DRC (cloud), all complete:

| Board | none | `groups` | `groups`, `--clear-corridors` | parts moved |
| --- | --- | --- | --- | --- |
| parts_hard | 60.3 vias, 1084 mm | 48.7 vias, 1071 mm | 48.7 vias, 1017 mm | 3 |
| parts_multi | 69.3 vias, 1372 mm | 58.7 vias, 1424 mm | 56.7 vias, 1403 mm | 1 |
| parts_multi_feed | 126.7 vias, 1902 mm | 98.0 vias, 1897 mm | 112.7 vias, 1692 mm | 9 |
| parts_xhard | 103.0 vias, 1324 mm | 101.3 vias, 1392 mm | 83.0 vias, 1279 mm | 11 |

- The tighter the board, the more it gives: parts_xhard 18 % fewer vias and 8 % less length.
  On parts_multi_feed the tracks get 11 % shorter at 15 more vias.
- Moving the decoupling capacitors of parts_hard out of the band gave 39 vias, but they belong
  at their IC; a capacitor between a ground and a supply (a plane, a power class, or a net with 8
  or more pads) now stays, and so does any capacitor with a pad on a ground or supply.

LED controller (KiCad DRC on the Mac, seeds 1 to 3, `--no-move-parts`, 2026-11-02):

| Variant | Vias | mm | Clearance errors |
| --- | --- | --- | --- |
| A original placement, no hints | 261/263/267 | 2612/2553/2638 | 0/0/0 |
| A, `pairs,groups` | 279/286/263 | 2708/2508/2575 | 1/2/14 |
| B `--clear-corridors`, `pairs,groups` (before the leash) | 265/259/235 | 2606/2582/2693 | 40/33/4 |
| C `place`, `pairs,groups` | 267/227/218 | 2199/2201/2119 | 0/0/1 |
| D `place --semantic --corridors --escape-halo`, no hints | 242/256/200 | 2398/2161/2351 | 0/0/0 |
| D, `pairs,groups` | 289/236/220 | 2271/2236/2263 | 0/0/0 |

- D is the cleanest: no clearance error and no short in 9 runs, 12 % shorter than A. It is a new
  placement though (only connectors and mechanics stay), the power stages sit elsewhere.
- With the placement fixed the hints do not help on the LED controller.
- B moved the power stage U5 21 mm: its place next to its neighbours is tighter than the gap and
  halo of the placement, so the legalisation found no room for it at home. More gap and halo made
  it worse; `--air` 0.03 or 0.05 removed only the smallest deficits.
- Fix: `place --leash MM`. A part with 4 or more pads pays for every mm beyond its leash (squared),
  starts at home, gets no halo and keeps the room it has where its neighbours sit closer than the
  gap. `--clear-corridors` leashes ICs and connectors at 1.5 mm (`GB_CC_LEASH`): U5 moves 0.1 mm,
  U22 1.5 mm. With move rounds, seeds 1 to 3, Gummiband's own clearance check: 272/276/300 vias,
  2461/2543/2552 mm, 0/2/1 violations. The KiCad DRC of this state is still open.
- With move rounds (the router's own moves, KiCad DRC, before the leash): little change at about
  twice the run time; D stays without a clearance error or short in all 18 runs. In B, U22 may
  move 6.9 mm and gives the shortest result (down to 2292 mm); holding U5 alone is enough for 0 to
  1 clearance errors. So the leash, not a fixed IC, is the right default; a switching regulator
  with its power nets may need a shorter one.

## M4b: GPU experiment

- Opt-in: `--features gpu` at build time and `GB_GPU=1` at run time, Metal through wgpu. Without
  them nothing changes, the default run stays byte-identical. Sleeping copper and vias of one layer
  stay on the CPU.
- Share of the relaxation in the work (core seconds, 4 threads, 2026-10-30): LED controller 31 %,
  with `--geometry-generations 3 --move-parts` 63 %, Arty 43 %. Corne, ULX3S and bga_dense 1 to 5 %
  (measured before the counters of PR #8 were removed, which only lowers these). On the hard
  boards topological A* decides the run time, and it does not fit a shader.
- A way to the GPU and back costs about 1.5 ms, a round of 27000 points about 0.1 ms. So the GPU
  runs ten rounds at a time, from one resampling to the next, and the vias take one step of ten
  times the size per batch. In one round per way the GPU only reaches the speed of 4 CPU cores.
- With everything on the GPU more violations stay. So the last quarter (clearances only) stays on
  the CPU. One relaxation of the LED controller on 4 threads: 0.73 instead of 1.8 core seconds, 0.40
  instead of 0.66 s, with more violations on average.
- LED controller `--geometry-generations 3 --move-parts`, seeds 1 to 4, 4 threads: 996 core
  seconds on the CPU, 593 with the GPU, all 348/348; violations 15, 4, 0, 2 against 5, 0, 0, 11.
- The same on all 14 cores, alone on the machine: 110 s on the CPU, 71 s with the GPU (1175 and 669
  core seconds), same violations. One relaxation: 0.52 s against 0.38 s, six in parallel 1.17 s
  against 0.57 s, with 15 % more violations on the LED controller and 0.4 % more length. Default run
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
  (margins 1e-7 mm apart). Eight tries of the polish step: LED controller seeds 15, 4, 0, 2 become
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
- Pictures for the guide (done, 2026-11-03): a feature tour (`docs/guide/tour.md`), a chapter on
  working with an AI agent (`docs/guide/ai-agents.md`), result images from the benchmark boards and
  drawings of the principles in every chapter; the drawings come from
  `docs/guide/img/src/schematics.py`.

## Consolidation: environment variables

- 2026-11-10: experiment switches whose measurements above are concluded were removed with their
  code: the via in a pad experiments `GB_VIP_NETS`, `GB_VIP_LEARN`, `GB_VIP_RETRY`, `GB_VIP_EARLY`
  and `GB_PAD_REUSE` (the via in a pad stays the last resort of a run),
  `GB_PRUNE_STARTS`, and the routing order `groups-inline` with `GB_GROUP_ORDER` (`groups`,
  `groups-long` and `groups-short` stay). The texts above describe them as they were; the code is
  in the git history up to 3a9d573. What is left is listed in `docs/ENVIRONMENT.md`.

## Consolidation, phase 2a: one behaviour for the CLI and the MCP server

- 2026-11-10: after the shared route pipeline (`gummiband-app`), the places where
  `gummiband route` and the MCP tools `route` and `try_seeds` still behaved differently moved
  into the pipeline, so neither front end carries a rule of its own:
  - The checks of the options against each other are `RouteOptions::validate`. Contradictions
    are errors that say what to change, in both front ends in the same words; the CLI no longer
    turns `shove` off silently under `--move-parts`, the MCP server no longer takes the default
    for `runs: 0`.
  - `air` is left to the pipeline by both, so an open ended route of the whole board gets its
    0.03 mm through the MCP server too.
  - Options per net are one type, `NetOptions` (width, minimum width, parts to neck at,
    priority, skip). The file of `--net-widths` takes an object per net next to the plain
    width, and `set_net_options` has the same fields; its `width` no longer doubles as the
    minimum width.
  - The MCP server routes through `route_file`, as the CLI does: one way of reading the board
    and collecting the movable parts, and `plane_layers` for `route` and `try_seeds`.
  - The pipeline writes the output, the alternatives of the open end
    (`<output>.altN.kicad_pcb`, now also from the MCP server, which lists them) and the project
    files next to the output (now also from the CLI). An output that is the input is refused.
  - The open end with a time budget is an option of the pipeline (`open_end_seconds`, new as
    `--open-end-seconds`), and its clock starts with the open end, not with the route.
    `seconds` is the wall time of the whole route in both reports; `seed` names the seed of
    the search, as does the line `best: seed N`.
  - `gummiband_app::OPTIONS` lists every option with name, default and one sentence. Tests hold
    the fields of `RouteOptions`, the flags of the CLI and the arguments of the MCP tool to it.
    New on the CLI: `--reroute`, `--tune`, `--open-end-seconds`. New in MCP `route`:
    `plane_layers`, `placements`, `placement_seed`, `net_widths`, `net_priorities`,
    `teardrops`, `frames`; `pair_breakouts` and `pair_via_pairs` are `breakouts` and
    `via_pairs`; an argument that does not exist is refused.
  - The CLI removes its folder `gummiband-corridors-<pid>` from the temp directory.
  The route hashes did not change. Still to come: help texts generated from the table (2f).

## Consolidation, phase 2b: one option for differential pairs

- 2026-11-11: `pairs` is one option with three words in the CLI and the MCP tool `route`:
  `off` (the default, as before), `pull` (what `pair_pull` above 0 did) and `band` (what the
  switch `pairs` did). `pair_pull` is the weight of the pull, 1 unless given, and with `band`
  the pull for the pairs that get no wide track (0 unless given). `breakouts`, `via_pairs`,
  `pair_all` and `pair_zdiff` belong to `band` and are refused without it, with a message that
  says what to write. The words and their sentences are in `gummiband_app::OPTIONS`
  (`Kind::Choice`); `--help` and the MCP schema take them from there. The old spellings still
  work and say what to write now (`gummiband_app::old`): `--pairs` alone is `--pairs band`,
  `--pair-pull W` alone is `--pairs pull --pair-pull W`, `pairs: true` is `pairs: "band"`.
  Measured: 12 routes of `diff_pairs` in old and new spelling give the files of the build
  before the change, byte for byte; `bench/hashes.sh` now passes `--pairs band` and `--pairs
  pull`, and both hash lists are unchanged.

## Consolidation, phase 2c: one option for what stays on the board

- 2026-11-11: `scope` is one option with four words in the CLI and the MCP tool `route`, in
  place of the switches `keep_locked`, `keep_all` and `reroute`: `fresh` (no copper stays),
  `keep_locked` (the locked copper stays), `finish` (all copper stays, what is open is laid) and
  `reroute` (the unlocked copper of `nets` goes and they are laid again). The sentences of the
  words say which one is for "I laid these nets by hand, finish the rest" (`finish`) and for
  "lay net X again" (`reroute`); the MCP schema has them from `gummiband_app::OPTIONS`. The
  defaults of `move_parts`, `shove`, `give` and `air` hang on the scope and name it in their
  one line. Of the eight combinations of the three switches four had a meaning and map onto a
  word; `reroute` without `keep_all` had none, was refused and still is. The old switches still
  work and say what to write now. The default stays what it was: `fresh` on the command line,
  in an MCP session what the board was loaded with.
  Measured: 15 routes of a half routed `medium` (three nets open, two locked) in every old
  combination and in the new spelling give the files of the build before the change, byte for
  byte, and so do the 12 routes of the pair options; both hash lists are unchanged.
