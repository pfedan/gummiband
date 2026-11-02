# Command line

`gummiband` is the binary the plugin calls; everything the plugin can do, and more, is a flag
here. Build it with `cargo build --release`; it is then at `target/release/gummiband`. The
examples below assume it is on your `PATH`.

```text
gummiband [--threads N] [--plane-layers L1,L2] <command>

Commands:
  info   Print a summary of a board
  route  Route a board and write the result
  strip  Remove tracks, vias and outer copper zones (benchmark preparation)
```

`gummiband <command> --help` prints the options of a command.

## Input files

Every command takes a `.kicad_pcb` (KiCad 9 or 10). Two more files are read if they sit next to
it with the same base name:

- `.kicad_pro`: the net classes (track width, clearance, via size, priority) and the net to class
  assignment, plus the board design rules.
- `.kicad_dru`: custom design rules. Supported are clearances between tracks of two nets or net
  classes and minimum track widths with courtyard exemptions, see [concepts](concepts.md#rules).

Without the `.kicad_pro` every net is in the default class with KiCad's defaults.

## Global options

| Option | What it does |
| --- | --- |
| `--threads N` | Worker threads for the whole run. Default: `RAYON_NUM_THREADS` or all cores. |
| `--plane-layers In1.Cu,In2.Cu` | Treat these copper layers as planes: they are not routed and their zones stay. Layers of KiCad type *power* are planes anyway. Useful for boards whose inner layers are plain copper layers that happen to carry only a pour. |

## `gummiband info`

```bash
gummiband info board.kicad_pcb [--keep-all]
```

Prints what the router sees: outline size, copper layers with the plane flag, obstacle counts by
kind, pins, nets, connections, planes, the net classes with their nets, the board rules and the
number of net pairs with custom clearances. By default only locked tracks and vias are counted as
kept copper; `--keep-all` counts every track and via.

Run this first on a new board. If the layer list, the planes or the class widths are not what you
expect, fix the project before routing.

## `gummiband route`

```bash
gummiband route board.kicad_pcb -o routed.kicad_pcb [options]
```

Writes a copy of the board with the routed tracks, arcs and vias. Nothing but tracks and vias
changes in the file but the footprints that `--move-parts` shifts (on by default, see below).

### What is kept and what is replaced

| Option | Default | Effect |
| --- | --- | --- |
| `--keep-locked` | off | Keep locked tracks and vias as fixed copper and route around them; their ends count as connected. Without it **every** track and via is removed first, locked or not. The plugin passes this flag by default. |
| `--keep-all` | off | Experimental. Keep every existing track and via, locked or not, and add to them: for completing a partly routed board or finishing a track you started. Pins that sit on one piece of kept copper count as connected, and the search rides along kept copper without drawing a new track there. New, so check the result with the DRC. |
| `--nets A,B` | all | Experimental. Route only these nets (exact names, comma separated); the other nets stay as they are. Meant together with `--keep-all`. |
| `--shove` | on with `--keep-all --nets` | The unlocked tracks of other nets become part of the topology, so a new connection is threaded between them and they shift aside a little instead of being rerouted. What the new track does not touch stays exactly as it was. Vias that moved lose their teardrop settings, and in very tight spots a short is still possible, so run the DRC. On by default when you finish nets with `--keep-all --nets`, except with `--move-parts`; the flag insists on it (and fails where it cannot work). The plugin's *Finish selected nets* passes it too. |
| `--no-shove` | off | With `--keep-all --nets`: the tracks of other nets stay where they are, a new connection only goes around them. |
| `--give R` | 8 with `--keep-all --nets`, else 0 | When a connection finds no way, unlocked tracks of other nets within R mm of its pins give way and are routed again, in growing steps (2, 4, 8 ... mm) up to R. Every step is a try of its own, and the try with the fewest open connections, then the fewest clearance violations is written; on a tie the earlier one, which moves less. So a net that gave way and found no way back costs nothing: that try loses. Locked copper and plane nets such as GND never move. `--give 0` keeps everything in place. With `--until-stop` the tries run without the open end, and the one kept runs again and goes on open ended (if it ends worse, after an early stop, the try stays). |

Zones are never removed by `route`. Refill them in KiCad afterwards.

### Search effort

| Option | Default | Effect |
| --- | --- | --- |
| `--runs N` | 14 | Portfolio size: routing attempts with different connection orders (and via costs) in every phase, spread over the worker threads. A constant by design, so results do not depend on the machine. |
| `--generations N` | 3 | Rounds of the evolutionary order search after the portfolio. Each round mutates the best order (failed connections move forward, a few random moves) and tries `--runs` mutants. 0 means portfolio only. The search stops early when everything is routed. |
| `--geometry-generations N` | 3 | Rounds that are judged after the tracks were drawn, by the clearance check. Each round draws and checks every candidate, so 3 rounds take about 2 to 3 times as long as 0, and leave far fewer violations (Arty 32 to 1, ULX3S 54 to 4, Lichtschnitzel 18 to 4). 0 is the quick draft. The plugin and the MCP server use 3 as well. |
| `--seed N` | `GB_EVO_SEED`, then fixed | Seed of the search. The same board, options and seed give the same result; another seed gives another attempt, often with a quite different violation count (Arty 32 or 1, Lichtschnitzel 18 or 14 at `--geometry-generations 0`). |
| `--negotiate N` | 0 (when needed) | Negotiated routing: connections may overfill channels and cross for a price that rises from round to round, until they have settled who goes where. Finds the last connections on dense boards, where the order alone does not. Without the option the search takes it up on its own: when the plain search leaves a few connections open (at most 15 %), a second one negotiates with 20 rounds and the better result counts, so such a board takes the time of both; `GB_AUTO_NEGOTIATE=0` switches that off. With N a negotiated search runs from the start, next to a plain one, and the better result counts, so a run takes about twice as long. 16 to 20 rounds work well; fewer leave conflicts that cost connections. |
| `--until-stop` | off | Keeps improving the result after the search until a line arrives on stdin or stdin closes, then finishes the best result. A generation takes a few connections around one spot out (a violation, or a connection with vias or a long way) and lays them again, everything else stays; the longer nothing helps, the more it takes out, and side branches try bolder changes that may first make the result worse. It never gives up a connection, and it counts a problem as one spot, however many pieces of track it touches. Results that are as complete and clean but differ in vias and length are written next to the output as `.alt1.kicad_pcb`, `.alt2.kicad_pcb`, each with an `alternative:` line in the summary. The same seed stopped at the same generation gives the same result. Finishing nets (`--keep-all --nets`), the tracks of other nets shift aside and give way as without it (see `--shove` and `--give`), and the open end goes on from the try kept. Other limits: no placement rounds with `--move-parts`, no plain search next to `--negotiate`, and it does not go with `--pairs`. |
| `--air MM` | 0, with `--until-stop` 0.03 | Reserve on top of the clearances that the finished copper gets where there is room. A rubber band lies right at the limit, this pushes it off and evens out the gaps in tight bundles. The result stays only if it brings no new violations. Finishing nets (`--keep-all --nets`) it stays 0 with `--until-stop` too, as it would move the tracks of other nets that the new ones do not touch (on Arty 270 of them); give it explicitly to have it there. |
| `--prefer-width W` | 0 (off) | At the very end tracks of the default net class first push aside what can give way, then grow up to W mm wherever there is room, with all clearances kept; tracks in a bundle share the room. Routing still works with the class width, so this costs no connections. They never get narrower than their class width, a short wider stretch is left out, and every change of width is a taper of 0.025 mm per piece. Other classes keep their width unless a `track_width` rule in the `.kicad_dru` gives them a target with `opt` (that also works without this option, see [Wider tracks where there is room](concepts.md#wider-tracks-where-there-is-room)); differential pairs always keep theirs. `--teardrops` can smooth that further. |
| `--angles A` | `any` | `45` turns the result into conventional 45 degree routing as the very last step: straight pieces in the eight directions of the 45 degree grid, corners of 90 and 135 degrees chamfered, no arcs. Vias and track ends stay where they are. The step never adds a clearance violation: where no 45 degree path fits, mostly in tight bundles, a piece keeps its angle, and the output says how many (`45 degrees: ... pieces off the grid`). Tracks get about 4 % longer. Necks stay where the router had them. With `--pairs` the wide track of each pair goes onto the grid before the split (`45 degrees on the wide tracks: ...`), so both tracks stay parallel. The JSON report has the numbers under `angles_45`. See [Conventional 45 degree tracks](concepts.md#conventional-45-degree-tracks). |
| `--via-pass F` | 4 | After the search, connections with vias are routed once more with the via cost multiplied by F, and the result is kept only if completion, violations and shorts do not get worse. 0 switches the pass off. |
| `--via-pitch MM` | 0.5 | Grid pitch of the candidate via positions. Finer grids allow more via places and cost memory and time. |
| `--via-cost MM` | 3.0 | Cost of a via in millimetres of track. Lower gives more vias and shorter tracks; the search varies the value around what you give. |
| `--pair-pull W` | 0 (off) | Experimental. Differential pairs are recognised by net name as KiCad pairs them (`_P`/`_N`, `DP`/`DN`, `+`/`-`). With a weight above 0 the first net of a pair is routed normally and the second gets a cost bonus for staying next to its way on the same layer; afterwards the tracks close up to the pair's gap (width and gap from the class) wherever they run within 1.5 mm of each other and the move keeps all clearances. The output then reports how much of the pair length runs coupled (same layer, at most 1 mm apart). Limits: breakouts to the pins and places with copper between stay apart, there is no impedance control and no length matching, and on small boards the pull can cost many clearance violations. Try 1 on boards with real pairs and compare the DRC. |
| `--pairs` | off | Experimental. Route each differential pair as one wide track and split it into two parallel tracks. A pair is coupled only if a `diff_pair_gap` rule or a net class of its own (not `Default`) marks it as one; a `MOTOR_P`/`MOTOR_N` in the default class without such a rule stays two nets (`--pair-all` couples it anyway). Width and gap come from `diff_pair_gap` and `track_width` rules in the `.kicad_dru` (also per `(layer inner)` or `(layer outer)`), then from the net class. Output and `--report` list every pair. Keeps pairs together, but costs connections on dense boards and can leave clearance errors and shorts next to pads and vias (seen on Lichtschnitzel at J1, on Jetson 81 instead of 51 open connections): always check the result with the KiCad DRC. |
| `--pair-all` | off | With `--pairs`: couple every pair recognised by name, also without a pair rule or net class. |
| `--via-pairs` | off | Experimental, with `--pairs`: two vias side by side are one via pair, a capsule that turns with the wide track in the relaxation, instead of the circle around both that leaves room for vias in line as well. A 0.2 mm track of another net may pass the middle of its long side at 0.5 instead of 0.95 mm (0.6 mm vias, 0.3 mm via gap, 0.1 mm clearance). On DDR5 over 4 seeds 185 instead of 206 KiCad errors. |
| `--breakouts` | off | Experimental, with `--pairs`: end the wide track at a point just outside the pads, with two short single tracks into them and the room for them kept free of other nets. Helps at connector rows (`diff_pairs`, `matching`), but on DDR5 it is still worse than without (more pair gap errors and length errors over 6 seeds). Ends between the pads of a part (BGA) stay at their pads. |
| `--pair-skew MM` | 0.1 | With `--pairs`: length difference left within a pair; the shorter track gets bumps where they fit. A `skew` rule with `(within_diff_pairs)` may ask for less. |
| `--pair-zdiff OHM` | none | With `--pairs`: target differential impedance, sets the track width per layer from the stackup (estimate after IPC-2141), the gap stays. |
| `--no-tune` | off | Leave the lengths as routed. By default, when the `.kicad_dru` has `length` or `skew` rules, the routing first makes room next to the nets that are too short and then gets meanders that bring them into range, see [Length matching](concepts.md#length-matching). |
| `--tune-amplitude MM` | 1 | Highest meander bump (centre line above the original line). |
| `--tune-spacing W` | 2 | Air between neighbouring meander legs, in track widths (at least the clearance). |
| `--patterns LIST` | none | Hints from local patterns of the board (`gummiband info` lists them), comma separated: `decaps` connects a decoupling capacitor to its IC pin before everything else, `rows` lets the open end lay the connections of a pin row again in pin order, `shares` lets pins of a plane net next to each other reach the plane through one via, `pairs` routes differential pairs before the other signal connections (the longest first), `groups` routes the connections between the same two parts (bus, SPI, I2C, connector) as one block in pin order, the longest block first (`groups-long` is the same, `groups-short` puts the shortest first, `groups-inline` leaves each block at the place of its connections; `GB_GROUP_ORDER` still sets the order of a plain `groups`). `all` or `none`. Experimental, none wins on every board: see [Choosing the opt-in options](options.md). |
| `--layer-dirs W` | 0.5 | Weight of preferred directions per layer: the routing layers alternate horizontal and vertical, and running against the grain costs extra. A soft cost, used by every third run of the portfolio, so the search has candidates with and without it. 0 switches it off. |

A good default for a real board with a few hundred connections is the plain `--keep-locked`: 14
runs on all cores, 3 generations, 3 geometry generations, a few minutes. `--geometry-generations 0`
gives a quick draft in about a third of the time. For the final pass add `--until-stop` and stop
when the progress reports `converged`.

### Degrees of freedom

| Option | Default | Effect |
| --- | --- | --- |
| `--move-parts` / `--no-move-parts` | on, off with `--keep-all` | During the geometry generations, parts next to clearance problems and next to the pins of open connections may shift a little to open a channel. At an open connection the part with that pin stays and its neighbours step aside. The routed copper goes along like rubber bands, as in [`move-part`](#gummiband-move-part): the parts move in small steps and the tracks give way; a shift stops where the copper cannot give way any more. Only if that keeps the problems does a new search on the shifted board follow (on boards up to 400 connections). Locked footprints, connectors, modules, mounting holes, switches, jumpers and test points stay (by footprint library name and by reference prefix J, P, CN, USB, H, MH, SW, TP). The output file has the moved footprints at their new positions, with the footprint's own zones (a keepout drawn inside the footprint) moved along, and the console lists every move. Footprints in a KiCad group form a block that moves as a whole with its kept tracks and vias (zones in the group do not move yet); a block with one fixed member stays put. Blocks are new and untested on a real board. |
| `--fixed R1,U3` | none | More references that never move under `--move-parts` and `--placements`. |
| `--placements N` | 0 | Place the board N times from the netlist (see [`gummiband place`](#gummiband-place), default scope), route each result with the settings of this command and keep the best: fewest open connections, then clearance violations, vias and length. Takes N + 1 routes, the last one writes the winner; `--until-stop` applies to that last one only. Not with `--keep-all`, `--nets` or `--pairs`. The JSON report gets a `placement` block with every try. |
| `--placement-seed S` | 1 | First seed of `--placements`. |
| `--clear-corridors` | off | Before the routing, parts that sit in the band of a group of tracks between two parts (ICs, connectors sharing at least two signal nets) and are not on its nets get new positions with `place --corridors`; everything else stays, and so do decoupling capacitors (a capacitor on a ground or supply). ICs and connectors in a band move at most 1.5 mm (`place --leash`, `GB_CC_LEASH` to change it). Pairs with `--patterns groups`. Experimental: `GB_CC_GAP`, `GB_CC_HALO` (mm), `GB_CC_ESCAPE=1` and `GB_CC_SEMANTIC=1` give the moved parts more room. When it pays: [options](options.md#room-for-track-groups). |
| `--teardrops` | off | Switches on KiCad's teardrops for every via and connected pad in the output file, and the track end teardrops (the wedges where a track changes width, for example at a neck) in the output's project. KiCad builds the shapes itself: open the file and fill the zones once (B), then the teardrops appear. The plugin cannot set them through KiCad's API and opens *Edit Teardrops* instead. |
| `--net-priorities prio.json` | none | Routing order per net from a JSON file `{"net name": priority, ...}`; lower numbers route first. Nets not in the file keep their net class priority. The MCP server does the same with `set_net_options`. |
| `--net-widths widths.json` | none | Track width per net from a JSON file `{"net name": width_mm, ...}`, overriding the net class width. `bench/net_widths.py ORIGINAL.kicad_pcb --json widths.json` extracts the widths a designer used in an existing routing. |

### Output and reports

| Option | Effect |
| --- | --- |
| `--report out.json` | Machine readable report, see below. |
| `--progress` | Prints a `progress {json}` line after every phase: `phase`, `routed`, `conns`, `vias`, `length`, `violations` (null before the geometry step; in the open end these are spots, see `--until-stop`), `generation`, `seconds`, and for `--until-stop` `stale` (generations since the last noticeable gain), `level` (side branches that failed since then) `top` (how many have to fail) and `converged` (even bold changes do not help any more). The plugin's progress window reads these lines. |
| `--svg out.svg` | Debug drawing of the result: obstacles, tracks, vias and the violations. |
| `--svg-tris` | Also draw the triangulation into that SVG. |
| `--frames DIR` | One SVG (and PNG, if `rsvg-convert` is available) of the best candidate per generation into `DIR`, and `DIR/evolution.gif` if ImageMagick's `magick` is available. |

### Reading the console output

```text
substrate: 1012 vertices, 2012 triangles, 2402 via sites
  gen  0 seed  0 fanout_first true  via cost 3.0: 9/9 routed, 0 vias, 113 mm
  gen  0 seed  1 fanout_first false via cost 3.0: 9/9 routed, 0 vias, 109 mm, 0 violations (0 shorts)
best: seed 1 9/9 routed, 0 vias, 109 mm, 0.15 s
  length lower bound (signal nets, Steiner): 88 mm; signal length is at most 24% above the optimum
clearance check: 0 violations
  {}
wrote 83 items to routed.kicad_pcb
```

- One line per candidate of the portfolio and the generations: routed connections, vias, length,
  and after the geometry step the violations of the drawn tracks.
- `best`: the winner. Fewest unrouted connections first, then fewest violations, then vias and
  length.
- `unrouted: NET: PAD -> PAD` lines name the connections the router gave up on. Route them by
  hand, or lock the rest and run again.
- `diff pairs: N, X of Y mm coupled (Z%)` appears when the board has nets paired by name: how
  much of the pairs' length runs side by side on one layer, at most 1 mm apart.
- With `--pairs`, one `pair P / N: wide ...` line per pair follows: `wide` or `single` (routed
  as two separate nets, with the reason), width and gap, the estimated differential impedance
  from the stackup (0 if the stackup is unknown), both track lengths, the remaining skew, vias
  and polarity swaps through the pair's vias.
- `room for meanders: N nets and pairs, made|none without new violations`: with `length` or
  `skew` rules, before the meanders, how many nets (pairs as one) wanted room next to them and
  whether pushing their neighbours aside was kept (`made`) or undone.
- `length tuning: N nets, M reached their length`: with `length` or `skew` rules, how many nets
  the meanders were to lengthen and how many got there; one line for each net that did not
  (before, after, aim, `(as a pair)` for a pair meandered as a band).
- `length rules: ...`: every `length`, `skew`, `diff_pair_uncoupled` and `diff_pair_gap` rule
  of the `.kicad_dru` with the nets (or pairs) it holds for, measured as the KiCad DRC measures
  them, and one line per net out of range with its length and, for skew, its distance to the
  longest net of the group. `gummiband lengths` prints the same for any routed file.
- `length lower bound`: an estimate of the shortest possible total signal length (minimum
  spanning trees, Steiner corrected). The percentage says how far the result is above it; plane
  fan-outs are not in the bound, so the real distance is smaller.
- `clearance check`: Gummiband's own check with a histogram by kind: `track` and `via` for two
  routed nets too close to each other, `pad`, `locked-track`, `locked-via`, `keepout`, `zone`,
  `hole`, `graphic` for a track too close to that kind of obstacle, `edge` for the board edge,
  `width` for a piece below the minimum of a `track_width (min ...)` rule out of the courtyards
  the rule exempts, and `current` for a net with such a rule (not of a differential pair) whose
  widest way between two pins that can take that width carries less, vias included (see
  [Track width and necking](concepts.md#track-width-and-necking)). KiCad's DRC is stricter in
  places and the final judge.
- `current paths: N vias added next to their layer changes` appears when layer changes of such
  nets got the vias that carry their current.
- `run of seed N in generation G lost: ...` means that one router run tripped over an
  inconsistency of its own and was dropped; the search goes on with the others. It is worth a bug
  report with the board.

### The JSON report

`--report out.json` writes:

```json
{
  "board": "board.kicad_pcb",
  "output": "routed.kicad_pcb",
  "seconds": 44.7,
  "connections": 348,
  "routed": 346,
  "unrouted": ["VBUS_IN: U6.5 -> C12.1"],
  "vias": 338,
  "length_mm": 2458.1,
  "length_lower_bound_mm": 1980.4,
  "items_written": 3306,
  "violations": [
    {"what": "pad /SDA~GND", "x": 143.21, "y": 121.07,
     "actual_mm": 0.148, "required_mm": 0.15, "nets": ["/SDA", "GND"]}
  ],
  "portfolio": [
    {"generation": 0, "seed": 0, "fanout_first": true, "routed": 346, "connections": 348,
     "vias": 340, "length_mm": 2470.2, "violations": 31, "shorts": 0}
  ]
}
```

With `--pairs` the report also has `pairs`: one entry per differential pair with `p`, `n`,
`wide`, `note` (why a pair was not routed wide, if so), `width`, `gap`, `length_p`, `length_n`,
`skew`, `vias`, `swaps` and `zdiff`.

With length rules the report has `matching` (every rule: `rule`, `kind`, `value` with `min`,
`opt`, `max`, `within_pairs`, and `nets` with `net`, `length`, `skew`, `target`, `ok`) and, unless
`--no-tune`, `length_tuning` (`nets`: `net`, `before`, `after`, `aim`, `coupled`; `patterns`: one
per meander with its nets, layer, base line and settings).

`violations[].nets` is the routed net and the other net (or null for an obstacle without a net,
such as the board edge or a hole). Positions are board millimetres as KiCad shows them. Scripts
and AI agents can act on this; the MCP server returns the same fields.

## `gummiband place`

Gives parts a position from the netlist, as a step of its own before routing. The input file stays
untouched. Where a moving part sits in the input does not matter, so a pile of freshly imported
footprints next to the board works as well as a placed board.

```sh
gummiband place my_board.kicad_pcb -o placed.kicad_pcb
gummiband route placed.kicad_pcb -o routed.kicad_pcb
```

| Option | Default | What |
| --- | --- | --- |
| `--only R1,C2` | | place just these references, everything else stays |
| `--outside` | | place just the parts outside the board outline |
| `--all-unlocked` | | place every unlocked part, connectors, modules and mounting holes too |
| `--fixed U1,U2` | | these stay in any case |
| `--free TP1` | | move these on top of the above (a test point, a button) |
| `--seed N` | 1 | another seed gives another set of starts, none shared with a neighbouring seed |
| `--starts N` | 1 | independent runs in parallel, the one with the fewest crossings is kept. The crossings hardly predict the routed result (correlation about 0 over 12 seeds on Lichtschnitzel), so for another attempt change `--seed`, or let `route --placements N` route several placements |
| `--iters N` | 3000000 | annealing moves per run |
| `--gap MM` | 0.3 | spacing between courtyards |
| `--margin MM` | 0.5 | distance of the courtyards from the outline |
| `--halo MM` | 0.3 | extra room around parts with five or more pads, for their escape |
| `--semantic` | off | use the schematic beyond the netlist, see below |
| `--corridors` | off | experimental: keep a band free for each group of tracks between two parts, see below |
| `--escape-halo` | off | experimental: room for the escape per side of a part, from its pins on that side, see below |
| `--leash MM` | 0 (free) | parts with 4 or more pads (ICs, connectors) stay within this distance of where they sit: a local change instead of a new placement, e.g. with `--only`. A leashed part keeps the room it has where its neighbours sit closer than `--gap`, and gets no halo |
| `--two-sided` | off | place on both sides: each side has its own room, small SMD parts may change sides, see below |
| `--back-max-pads N` | 8 | with `--two-sided`: parts with more pads keep their side |
| `--back-cost MM` | 6 | with `--two-sided`: what a part on the back costs, in mm of wire |
| `--on-front U3,C1` / `--on-back U6` | | with `--two-sided`: these parts go on that side, whatever their pads |
| `--report FILE` | | JSON report: poses, decoupling assignment, crossings, every start, with `--semantic` inductors and groups, with `--two-sided` `back`, `flipped`, `flipped_zones` and `back_free_area` |

Without a scope option, all parts move except locked footprints, connectors, modules, mounting
holes, buttons and test points. Locked footprints never move. A KiCad group moves as a whole; a
group with tracks stays, since tracks do not turn yet.

Locked tracks and vias stay: parts keep the `--gap` distance from them on the layers the moving
parts use, and a part with locked copper of its net on a pad stays where it is (the console and the
report name it). Copper added after placing is not known to the placer, so lock your pre-routed
copper first. Unlocked tracks and vias do not follow; the console says how many there are. Keepouts of a
moved part follow it (a keepout counts as the part's if its courtyard covers at least half of it),
other keepouts that forbid footprints or pads stay free of parts. Without `--two-sided` all parts
share one room as far as spacing goes and keep their side, so a two sided board comes out less
dense than it could be.

Before the search the placer compares the courtyard area of the movable parts with the free room
(of each side, with `--two-sided`). When they cannot fit, the error says so at once, with the area
missing, instead of after a search that has no chance; for a one sided placement it also says
when front and back together would have the room.

What it looks at: wire length per net (wide and differential nets double, switch nodes eightfold,
nets with a plane over most of the board not at all), the pins of a net with a partial plane
staying over that plane, decoupling capacitors next to a supply pin of an IC (bulk capacitors at
the inductor of their rail) and the courtyards.

`--semantic` adds what the schematic says beyond the netlist. It changes results, so it is off by
default; without it the output is the same as before.

- Each schematic sheet instance (from the footprint's `path`, named by `sheetname`) stays
  together, weakly, as a box around its parts.
- Inside a sheet, small signal nets (up to three parts) tie a cluster around one IC: a converter
  with its feedback divider, boot capacitor and mode resistors. A chain of passives does not tie
  two ICs together. Ground, planes and current paths tie nothing.
- An inductor is pulled to the IC with the most pads on one of its nets (the switch node), pad to
  pad.
- Decoupling capacitors prefer an IC of their own sheet.
- Current paths (classes of 0.5 mm and wider, switch nodes) count their spanning tree instead of a
  box, at a weight that grows with the class width. A series chain such as connector, fuse,
  shunt, eFuse then stays in order, and a TVS or input capacitor stays on the path instead of
  anywhere inside its box.

`--corridors` keeps room for groups of tracks: two parts with at least 4 pads on nets (ICs,
connectors) that share at least two signal nets (a bus, SPI, I2C; a pull-up on the way does not
count) get a band between the centres of their pins, as wide as the tracks of the group with
their clearances. Another part pays 2 mm of wire per mm2 of the band it covers; parts on the nets
of the group (series resistors, pull-ups) may sit there. It pairs with `route --patterns groups`,
which routes such a group as one block.

`--escape-halo` replaces the even halo of parts with five or more pads by a strip in front of each
side, as deep as the tracks of the signal pins on that side with their clearances, shared by the
routing layers (at most 3 mm; pins of a plane net leave through a via and do not count). A full
side of a QFN keeps room for its fan-out, an empty side none, and two chips with a bus between
them stay apart by what their pins need.

`--two-sided` gives each side its own room: a part on the front and one on the back may sit on
the same spot. A through hole part keeps its body on its side and takes on the other side only its
holes (a square of the larger pad size plus 0.25 mm around each, the solder joint and the cut
lead), so small parts fit under an electrolytic capacitor or a terminal, between its pins.
Keepouts count on the sides of their layers; kept copper counts on F.Cu for the front and on B.Cu
for the back. SMD parts with at most `--back-max-pads` pads (8: passives, diodes, transistors,
small regulators up to SOIC-8) may change sides; larger parts, through hole parts, blocks and
fixed parts keep theirs. `--on-front` and `--on-back` put a movable SMD part on a side whatever its
pads (a large IC on the back under a module, a part that has to stay on top). A part on the back
pays `--back-cost` mm of wire, and a net with pins on both sides and no through hole pin pays
1.5 mm per unit of net weight for the via it needs, so the back takes what does not fit on the
front or saves more wire there (a decoupling capacitor right under its IC). With `--back-cost 0`
both sides count alike. A part that changes sides is flipped as KiCad flips it: the footprint
mirrored, pads, graphics and texts on the layers of the other side, the texts mirrored; keepouts
that follow it change layers too. Groups of tracks (`--corridors`) count a part on the other side
of both their ends at half weight, `--escape-halo` and `--leash` work per side. Not modelled: the
vias of a BGA fan-out under parts on the back. The console prints the free room per side and how
many parts went to the back.

The console lists the inductors and groups. Connectors and power stages that matter still belong
in `--fixed`. On the Lichtschnitzel controller (only connectors fixed, four seeds) the routed
VBUS_F came out at 18 to 26 mm instead of 58 to 78 mm and every inductor sat 3.2 mm from its switch
pins, at about the same via count and track length.

## `gummiband angles45`

```bash
gummiband angles45 routed.kicad_pcb -o routed-45.kicad_pcb [--nets A,B] [--report r.json]
```

The 45 degree step of `route --angles 45` on a board that is already routed, without routing
again: the unlocked tracks become conventional 45 degree tracks with chamfered corners and no
arcs. Each run of track between two vias, branches or pads is converted on its own; vias, track
ends, pads and locked tracks stay where they are, so connectivity stays as it was. Every new
piece keeps the clearances, and a piece with no room on the grid keeps its angle. Runs of track
that do not change stay in the file byte for byte. Narrow necks below a minimum width rule stay
as they are. `--nets` converts only the tracks of these nets; the others stay and count as
obstacles.

The console shows one line like `45 degrees: 626 tracks, 606 changed, 2348 -> 2446 mm, 31
pieces off the grid (60.4 mm), 1 sharp corners, violations 17 -> 17, 1.1 s`, where the
violations are Gummiband's own check before and after. `--report` writes the same numbers as
JSON under `angles_45`. The plugin button *45 degree tracks with Gummiband* runs this command.

## `gummiband move-part`

```bash
gummiband move-part routed.kicad_pcb C12 152.4 118.0 -o moved.kicad_pcb [--report r.json] [--svg moved.svg]
```

Moves one part on a board that is already routed, and the copper around it follows like rubber
bands; nothing is routed again. The part (with the rest of its KiCad group, if it is in one)
moves to the new position in steps no longer than the smallest clearance, so a pad can never
jump over a track. After every step the track ends and vias on its pads go along, the tracks of
its own nets stretch, and tracks and vias of other nets that the pads now press against give way
within their clearances and push their own neighbours in turn. Copper that nothing touches stays
in the file byte for byte, locked copper stays anyway.

The part stops short, at the last clean step, where the copper cannot give way any more (a
track clamped between the pad and fixed copper or the board edge), where its courtyard runs into
another part, or where it would leave the board. A step counts as clean if Gummiband's own check
finds no clearance spot that was not there before the move: a spot of two nets that had none
nearby, or one where more is missing than before. A problem that only moves from one pair of
nets to another stops the part as well. Keep the `.kicad_pro` (and `.kicad_dru`) next to the
board, otherwise the copper gives way by default rules; the command warns then. The console says how far it got and why it
stopped, for example `moved Q1 from (105.000, 121.000) to (105.000, 119.500), 15 of 15 steps; 8
tracks and 0 vias of 6 nets followed, at most 1.500 mm; clearance spots 91 -> 64`, followed by
`stopped short: ...` if it did. Only translations; to turn a part, use the MCP tool `move_part`
with `rotation` (the tracks do not follow then). `--report` writes the numbers as JSON under
`move_part`, `--svg` draws the result.

## `gummiband strip`

```bash
gummiband strip board.kicad_pcb -o stripped.kicad_pcb [--keep-locked] [--keep-zones]
```

Removes all tracks, arcs and vias and the copper zones on the routing layers (pours on plane
layers stay). This is how the benchmarks make an unrouted board out of a finished one.
`--keep-locked` keeps locked tracks and vias, `--keep-zones` keeps the outer pours.

## `gummiband lengths`

```bash
gummiband lengths routed.kicad_pcb [--nets 'DQ*,CLK*'] [--json]
```

Measures a routed board as the KiCad DRC does and checks the `length`, `skew`,
`diff_pair_uncoupled` and `diff_pair_gap` rules of its `.kicad_dru`: net lengths with the via
heights from the stackup and the pad-to-die lengths, the skew within each group (a rule holds for
the nets it wins, the last matching rule per kind; the target is the longest net), the coupled and
uncoupled length of each differential pair. Tracks of KiCad tuning patterns count for the length
and stay out of the coupling, as in KiCad. `--nets` also lists the lengths of these nets (glob
patterns). On the Antmicro DDR5 testbed the lengths equal those of the DRC to 0.0001 mm; the
uncoupled length can differ, KiCad's coupling test has corner cases this one does not copy.

## Judging a result with KiCad

```bash
cp board.kicad_pro routed.kicad_pro      # and board.kicad_dru if you have one
kicad-cli pcb drc --refill-zones --format json --units mm -o drc.json routed.kicad_pcb
python3 bench/drc_report.py drc.json     # counts per type, worst clearances
```

The project file must have the same base name as the routed board, otherwise `kicad-cli` runs
with default net classes.

## Environment variables

| Variable | Effect |
| --- | --- |
| `RAYON_NUM_THREADS` | Default for `--threads`. |
| `GB_EVO_SEED` | Seed of the evolutionary search when `route --seed` is not given (default fixed). Same board, same options, same seed give the same result; another seed gives another attempt. |
| `GB_GPU=1` | Experimental, macOS only. Runs most of the track relaxation as a compute shader on the GPU (Metal). Needs a binary built with `cargo build --release -p gummiband-cli --features gpu`; without the variable such a binary behaves exactly like the normal one, and a binary without the feature prints a warning and stays on the CPU. Only the relaxation moves to the GPU, so the gain is bounded by its share of the work: about 30 percent of the core seconds in a default run on boards like the Lichtschnitzel controller, about 60 percent with `--geometry-generations 3 --move-parts`, and 1 to 5 percent on boards where the search dominates (Corne, ULX3S, dense BGAs), where the GPU gains nothing. In the quality run on Lichtschnitzel (4 seeds, 4 threads) it needed about 40 percent fewer core seconds than the CPU; when all cores are already busy the gain shrinks. Completion is the same; violation counts are comparable but vary more from seed to seed. Results repeat on one machine but not across GPUs or drivers, so for a final run use the CPU or try several seeds. |

## Exit status

0 on success, including runs with unrouted connections or violations (read the report for
those). Non-zero only when the board could not be read or written.
