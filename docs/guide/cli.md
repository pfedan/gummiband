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
changes in the file (footprints move only with `--move-parts`).

### What is kept and what is replaced

| Option | Default | Effect |
| --- | --- | --- |
| `--keep-locked` | off | Keep locked tracks and vias as fixed copper and route around them; their ends count as connected. Without it **every** track and via is removed first, locked or not. The plugin passes this flag by default. |
| `--keep-all` | off | Experimental. Keep every existing track and via, locked or not, and add to them: for completing a partly routed board or finishing a track you started. Pins that sit on one piece of kept copper count as connected, and the search rides along kept copper without drawing a new track there. New, so check the result with the DRC. |
| `--nets A,B` | all | Experimental. Route only these nets (exact names, comma separated); the other nets stay as they are. Meant together with `--keep-all`. |
| `--shove` | off | With `--keep-all --nets`: the unlocked tracks of other nets become part of the topology, so a new connection is threaded between them and they shift aside a little instead of being rerouted. What the new track does not touch stays exactly as it was. Vias that moved lose their teardrop settings, and in very tight spots a short is still possible, so run the DRC. The plugin's *Finish selected nets* uses it, with `--give` as the fallback. |
| `--give R` | 0 | With `--keep-all --nets`: when a connection finds no way, unlocked tracks of other nets within R mm of its pin give way and are routed again, in growing steps up to R. Locked copper and plane nets such as GND never move. 0 keeps everything in place. The plugin's *Finish selected nets* uses 8. |

Zones are never removed by `route`. Refill them in KiCad afterwards.

### Search effort

| Option | Default | Effect |
| --- | --- | --- |
| `--runs N` | 14 | Portfolio size: routing attempts with different connection orders (and via costs) in every phase, spread over the worker threads. A constant by design, so results do not depend on the machine. |
| `--generations N` | 3 | Rounds of the evolutionary order search after the portfolio. Each round mutates the best order (failed connections move forward, a few random moves) and tries `--runs` mutants. 0 means portfolio only. The search stops early when everything is routed. |
| `--geometry-generations N` | 0 | Rounds that are judged after the tracks were drawn, by the clearance check. Slower (each round draws and checks every candidate) and cleaner. The plugin uses 3. |
| `--via-pass F` | 4 | After the search, connections with vias are routed once more with the via cost multiplied by F, and the result is kept only if completion, violations and shorts do not get worse. 0 switches the pass off. |
| `--via-pitch MM` | 0.5 | Grid pitch of the candidate via positions. Finer grids allow more via places and cost memory and time. |
| `--via-cost MM` | 3.0 | Cost of a via in millimetres of track. Lower gives more vias and shorter tracks; the search varies the value around what you give. |
| `--pair-pull W` | 0 (off) | Experimental. Differential pairs are recognised by net name as KiCad pairs them (`_P`/`_N`, `DP`/`DN`, `+`/`-`). With a weight above 0 the first net of a pair is routed normally and the second gets a cost bonus for staying next to it. The output then reports how much of the pair length runs coupled (same layer, at most 1 mm apart). Limits: the spacing is only roughly kept, there is no impedance control and no length matching, and on small boards the pull can cost many clearance violations. Try 1 on boards with real pairs and compare the DRC. |
| `--pairs` | off | Experimental. Route each differential pair as one wide track and split it into two parallel tracks. Width and gap come from `diff_pair_gap` and `track_width` rules in the `.kicad_dru` (also per `(layer inner)` or `(layer outer)`), then from the net class. Output and `--report` list every pair. Keeps pairs together, but costs connections on dense boards and can leave clearance errors and shorts next to pads and vias (seen on Lichtschnitzel at J1, on `diff_pairs` at a via, on Jetson 81 instead of 51 open connections): always check the result with the KiCad DRC. |
| `--pair-skew MM` | 0.1 | With `--pairs`: length difference left within a pair; the shorter track gets bumps where they fit. A `skew` rule may ask for less. |
| `--pair-zdiff OHM` | none | With `--pairs`: target differential impedance, sets the track width per layer from the stackup (estimate after IPC-2141), the gap stays. |
| `--layer-dirs W` | 0.5 | Weight of preferred directions per layer: the routing layers alternate horizontal and vertical, and running against the grain costs extra. A soft cost, used by every third run of the portfolio, so the search has candidates with and without it. 0 switches it off. |

A good default for a real two layer board with a few hundred connections is the plain
`--keep-locked`: 14 runs on all cores, 3 generations, no geometry generations, a minute or two. Add
`--geometry-generations 3` for the final pass.

### Degrees of freedom

| Option | Default | Effect |
| --- | --- | --- |
| `--move-parts` | off | During the geometry generations, parts next to clearance problems may shift a little to open a channel. Locked footprints, connectors, modules, mounting holes, switches, jumpers and test points stay (by footprint library name and by reference prefix J, P, CN, USB, H, MH, SW, TP). The output file has the moved footprints at their new positions, and the console lists every move. Footprints in a KiCad group form a block that moves as a whole with its kept tracks and vias (zones in the group do not move yet); a block with one fixed member stays put. Blocks are new and untested on a real board. |
| `--fixed R1,U3` | none | More references that never move under `--move-parts`. |
| `--teardrops` | off | Switches on KiCad's teardrops for every via and connected pad in the output file, and the track end teardrops (the wedges where a track changes width, for example at a neck) in the output's project. KiCad builds the shapes itself: open the file and fill the zones once (B), then the teardrops appear. Not in the plugin yet. |
| `--net-priorities prio.json` | none | Routing order per net from a JSON file `{"net name": priority, ...}`; lower numbers route first. Nets not in the file keep their net class priority. The MCP server does the same with `set_net_options`. |
| `--net-widths widths.json` | none | Track width per net from a JSON file `{"net name": width_mm, ...}`, overriding the net class width. `bench/net_widths.py ORIGINAL.kicad_pcb --json widths.json` extracts the widths a designer used in an existing routing. |

### Output and reports

| Option | Effect |
| --- | --- |
| `--report out.json` | Machine readable report, see below. |
| `--progress` | Prints a `progress {json}` line after every phase: `phase`, `routed`, `conns`, `vias`, `length`, `violations` (null before the geometry step) and `seconds`. The plugin's progress dialog reads these lines. |
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
- `length lower bound`: an estimate of the shortest possible total signal length (minimum
  spanning trees, Steiner corrected). The percentage says how far the result is above it; plane
  fan-outs are not in the bound, so the real distance is smaller.
- `clearance check`: Gummiband's own check with a histogram by kind: `track` and `via` for two
  routed nets too close to each other, `pad`, `locked-track`, `locked-via`, `keepout`, `zone`,
  `hole`, `graphic` for a track too close to that kind of obstacle, `edge` for the board edge.
  KiCad's DRC is stricter in places and the final judge.

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

`violations[].nets` is the routed net and the other net (or null for an obstacle without a net,
such as the board edge or a hole). Positions are board millimetres as KiCad shows them. Scripts
and AI agents can act on this; the MCP server returns the same fields.

## `gummiband strip`

```bash
gummiband strip board.kicad_pcb -o stripped.kicad_pcb [--keep-locked] [--keep-zones]
```

Removes all tracks, arcs and vias and the copper zones on the routing layers (pours on plane
layers stay). This is how the benchmarks make an unrouted board out of a finished one.
`--keep-locked` keeps locked tracks and vias, `--keep-zones` keeps the outer pours.

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
| `GB_EVO_SEED` | Seed of the evolutionary search (default fixed). Same board, same options, same seed give the same result; another seed gives another attempt. |
| `GB_GPU=1` | Experimental, macOS only. Runs most of the track relaxation as a compute shader on the GPU (Metal). Needs a binary built with `cargo build --release -p gummiband-cli --features gpu`; without the variable such a binary behaves exactly like the normal one. It saves about 30 percent of the core seconds in a default run on boards like the Lichtschnitzel controller and about 60 percent with `--geometry-generations 3 --move-parts` (measured with 4 threads, one run each); on boards where the search dominates (Corne, ULX3S, dense BGAs) it gains nothing, and when all cores are already busy the gain shrinks. Completion is the same; violation counts are comparable but vary more from seed to seed. Results repeat on one machine but not across GPUs or drivers, so for a final run use the CPU or try several seeds. |

## Exit status

0 on success, including runs with unrouted connections or violations (read the report for
those). Non-zero only when the board could not be read or written.
