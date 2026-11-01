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
- Micro-optimisation and an open end that keeps the cores busy, 2026-10-31, on 556edbd, 4 cores in
  the cloud. Without an open end the results stay byte for byte the same: default run Lichtschnitzel
  100 s instead of 133 s, Arty 88 s instead of 112 s. The open end makes 549 instead of 211 real
  mutants per minute on Lichtschnitzel and 474 instead of 263 on Arty: every core takes the next
  move until the generation has its 14 mutants, and the relaxation of a mutant takes about half the
  time. Stopped after 270 s, seeds 1 and 2, open end cost (2 per via, 0.05 per mm, plus the flaw):
  Lichtschnitzel 968 and 961 instead of 991 and 1007 (246 and 246 vias instead of 258 and 271), Arty
  365 and 405 instead of 371 and 448 (111 and 128 vias instead of 114 and 151). Generations of 4
  mutants were not better (986, 976, 396, 400).
- Board caches, 2026-11-01, results byte for byte the same: every via site keeps its distance to the
  board edge, and the board grids of the relaxation (edge mask, obstacle cells, near obstacles) are
  built once per board instead of in every relaxation. Corne has an edge of 449 pieces in 74
  outlines: completing the two open connections of a finished run (`--keep-all --nets`, one thread)
  takes 283 s instead of more than 1200 s; in the open end (Mac, seed 1, 4 threads) generation 90
  instead of 7 after 20 minutes, 480/482 after 705 s instead of 1909 s. Lichtschnitzel and Arty: no
  measurable change (the grids were 0.2 % of a run).

## Open: fine pitch fan-out (Corne)

Measured on 2026-10-30, none of these solved it: straight pin stubs (twice: worse on
`fine_pitch_mixed`, arty, Lichtschnitzel), a larger router margin, extra gap outside the fine
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
- Vias are tented on Corne and Lichtschnitzel, so only their copper counts, like a track.

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
- Lichtschnitzel has a margin of 0: byte-identical (`50c4f3cfb765`).
- A synthetic board with shapes, text and a pad without copper in the way: 12 bridges before, 0
  after, all connected.

## Next: search quality

- Open end: `route --until-stop` (see ARCHITECTURE). Single runs on 2026-10-30, 150 s in all,
  against the order mutation it replaced, which re-routed the whole board for every mutant:
  Lichtschnitzel seeds 24301, 1, 2 reach 219 to 224 vias against 272 and 293 of that mutation (DRC:
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
  board. Lichtschnitzel, 180 s: 348/348, 236 vias, 2591 mm, 1 spot, 2 DRC errors.
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
  converged in 9 minutes. The original has 147 vias and 3073 mm. Lichtschnitzel
  `--geometry-generations 3`: 348/348, 200 vias, 2523 mm, converged in 9 minutes, 2 clearance hits at
  one spot. The original has 294 vias and 2633 mm. Each also leaves results that differ in vias and
  length to choose from.
- Tracks that end on a track of their net, 2026-11-01. Arty until convergence, with the width of
  0.25 mm preferred, no routing errors in the DRC in all three: without them 106 vias, 2927 mm; with
  them at 40 mm per via 71 vias, 3203 mm; at 20 mm per via 80 vias, 3083 mm. Same-net tracks next to
  each other went from 176 to 24 mm (measured on the 40 mm run), mostly plane fan-outs of GND.
  Lichtschnitzel: 200 vias, 2523 mm; 122 vias, 2864 mm; 123 vias, 2848 mm, one spot in each. Single
  runs. Fewer vias cost length: the results on offer reach 125 vias with 2945 mm on Arty and 236
  vias with 2460 mm on Lichtschnitzel.
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
  - Lichtschnitzel seed 1: 348, 201, 1 and 348, 209, 2. Seed 2: 348, 191, 1 and 348, 176, 0, DRC
    clean instead of 10 errors.
  - Generations per second on Corne 0.25 to 0.41 instead of 0.42 to 0.46, because more mutants get
    as far as the relaxation; Arty and Lichtschnitzel unchanged within the noise.
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
    Open: the via in a pad as a gene of the search, which the mutations give and take.
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
- Finishing nets uses shove and give by default (2026-11-01, `gummiband_core::finish_nets` for
  the CLI and the MCP server; `--no-shove` and `--give 0`, MCP `shove: false` and `give: 0`
  switch them off). Give is a fallback: every circle is a try, and the try with the fewest open
  connections, then violations wins, the earlier one on a tie. A circle that takes in no new net
  is not routed again. Night series data (one net stripped and laid again on the main result of
  seed 1, 2 threads): plain left LED_D0, LED_D3 and BUCK_EN open on Lichtschnitzel and UART_EN on
  Arty, shove laid all 16 nets, near the old length, three to four times faster on Arty. Give
  alone within 3 mm completed three of the four but broke four connections around BUCK_EN; the
  new rule keeps the try before in such a case. With the new defaults all 16 nets give the same
  file as `--shove` before, byte for byte.
- Adaptive track width: as wide as the surroundings allow, instead of one neck width per zone.
- A new via in the plane of its net stays in it (open end, 2026-11-01, default since 60ab967,
  `GB_PLANE_HOLD=0` switches it off). `place_new_vias` moved such a via along its track past a
  sleeping track and out of the plane; the connection still counted as made through the plane, KiCad
  found it open and the via dangling. In 4 of 49 finished runs of the night series without the hold
  that reported every connection, on Lichtschnitzel (J10 pad 2, `+5V_A`) and Arty (U2 pad 24,
  `/FTDI_3V3`), also with the settings of main. With the hold: 0 of 5; the cost after 20 minutes
  against A without it, seed by seed: Lichtschnitzel +0.9 %; Arty +3.5 %, +3.1 %, +3.8 %; on Arty at
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
    Lichtschnitzel -12 % (4 and 4 seeds), Arty -15 % (3 and 3); fewer vias, a little more length;
    KiCad DRC clean in 5 of 7 runs against 3 of 7. Seed by seed: Lichtschnitzel -13.3 %, -11.5 %,
    -5.1 %, -13.8 %; Arty -9.7 %, -17.6 %, -15.5 %.
  - A without geometry generations (`--geometry-generations 0`) against A: Lichtschnitzel -4.9 %,
    -6.1 %, -3.5 %; Arty -4.2 %, +3.3 %, +0.2 %; the open end starts at about 17 s instead of 42 s.
  - B (A plus `GB_MERGE`, `GB_SPOT_FADE`, `GB_TAPER`) against A: Lichtschnitzel -1.7 %, -3.9 %; Arty
    +1.0 %, -0.3 %.
  - Single ideas against main, seed 1, `GB_VIA_PRICE`: Lichtschnitzel -7.5 %, Arty -2.5 %;
    `GB_BANDIT`: Lichtschnitzel -7.9 %, Arty -6.3 %; `GB_AIM_COST=0.25`: Lichtschnitzel -8.7 %, Arty
    +7.5 %; `GB_PRUNE`: Lichtschnitzel +2.9 %, Arty -1.5 %; `GB_MERGE`: Lichtschnitzel +1.5 %, Arty
    +3.7 %; `GB_SPOT_FADE`: Lichtschnitzel -1.9 %; `GB_TAPER`: Lichtschnitzel 0.0 % (KiCad DRC clean
    against 3 errors: the spot at R61 is gone).
  - `--via-cost 10` on main: Lichtschnitzel -6.5 %, -16.8 %; Arty +2.5 %, -11.6 %; on top of A:
    Lichtschnitzel 0.0 %; Arty +5.4 %.
  - 12 threads in one run (Lichtschnitzel, main, seed 1): cost 468.7 after 20 minutes with 1.6 times
    the generations of 4 threads; the best of three seeds with 4 threads: 454.0.
  - Mutant trace of A (Lichtschnitzel, seed 1, `GB_MUTANT_LOG`): moves at random places bring 78 %
    of the saved cost in 63 % of the time; moves at clearance spots mostly remove the flaw penalty;
    93 % of the saving is in after 10 minutes.
  - ULX3S, 40 minutes: main and A both 655/796, A with 305 instead of 370 vias. The open connections
    are no search problem: the original routes signals on In2.

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
