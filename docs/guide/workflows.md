# Workflows

Recipes for everyday routing with Gummiband. They assume the binary is built and, for the KiCad
steps, the plugin is installed ([Installation](installation.md)).

## Route a board for the first time

1. Finish the placement. The router moves nothing unless you ask for it.
2. Set up net classes in *Board Setup > Net Classes*: track width, clearance, via size. Assign
   nets to classes. Give the classes that matter most (clocks, high current, USB) a lower
   priority number so they are routed first, see [Route by priority](#route-by-priority).
3. Save the project, so the `.kicad_pro` is current.
4. Run `gummiband info board.kicad_pcb` (or just trust the plugin) and check layers, planes and
   classes.
5. Route from the plugin with the defaults, or on the command line:

   ```bash
   gummiband route board.kicad_pcb -o board.routed.kicad_pcb --keep-locked
   ```

6. Refill zones, run the DRC, look at the unrouted connections.
7. Iterate: lock what is good, fix the rest by hand or route again with more effort
   (`--geometry-generations 3`).

## Pre-route and lock power lanes

The router has no notion of "this lane should be wide and straight": it gives a net the width
of its class and the shortest path. For power you usually know better. So:

1. Route the power paths by hand in KiCad (or let the router do a first pass and keep the lanes
   you like).
2. Select those tracks and vias, right click, *Locking > Lock*.
3. Route with *Keep locked tracks and vias* on (the plugin default) or `--keep-locked`.

Locked copper is an obstacle of its net. Its ends count as pins, so a signal that has to reach a
locked lane ends on the lane instead of on the far pad. The same works for the layout a datasheet
asks for around a converter or an eFuse: draw it, lock it, let the router do the rest around it.

Zones are kept in any case. Keepout areas are honoured as obstacles on their layers.

## Route by priority

Nets are routed in order of their net class priority (lower first); within a class the search
decides the order and reorders failed connections forward. The priority field is in the net class
setup of the `.kicad_pro`. Classes without a priority come last.

On the command line there is no per net override yet. The MCP server has one: `set_net_options`
with a `priority` per net, and a `skip` flag that leaves a net out of routing. An AI agent or a
script driving the server can use it to route the critical nets first, judge, then route the rest.
See [MCP.md](../MCP.md#set_net_options).

## Judge a result

Three sources tell you how good a routing is:

- **The router's summary** at the end of the run (and in the plugin's message box): routed
  connections, vias, length, how far above the length lower bound, and the violations of its own
  clearance check.
- **The JSON report** (`--report out.json`): the same with positions and nets for every
  violation, and every candidate of the search. Scripts and agents read this.
- **KiCad's DRC**: the judge. `kicad-cli pcb drc --refill-zones --format json -o drc.json
  board.routed.kicad_pcb` (with the `.kicad_pro` next to it), then `bench/drc_report.py drc.json`
  for counts per type, a deficit histogram and the worst clearances with positions.

Typical for this early version on a dense board: a handful of clearance violations with deficits
of a few micrometres (nudge a track), occasionally a crossing or short in a BGA fan-out (reroute
that net by hand or lock the fan-out and run again).

## Spend more effort

Effort knobs, from cheap to expensive:

| Want | Do |
| --- | --- |
| more orders tried per phase | `--runs` higher than the core count (each run then takes a turn) |
| better order | `--generations 6` |
| fewer violations in the drawn tracks | `--geometry-generations 3` (the plugin default) or more |
| fewer vias | `--via-cost 5` |
| shorter tracks, more vias | `--via-cost 1.5` |
| tidier two layer boards, one direction per layer | `--layer-dirs 1` (0 switches the preference off) |
| finer via placement in tight spots | `--via-pitch 0.25` |

The search is deterministic: the same board, options and `GB_EVO_SEED` give the same result. Set
a different `GB_EVO_SEED` to get another attempt.

## Let parts move

```bash
gummiband route board.kicad_pcb -o out.kicad_pcb --keep-locked \
    --geometry-generations 3 --move-parts --fixed U1,Y1
```

With `--move-parts` the geometry generations may shift unlocked parts that sit next to clearance
problems by a small amount to open a channel, then rerun the search on the shifted board.
Connectors, modules, mounting holes, switches, jumpers and test points never move, nor do locked
footprints or the references in `--fixed`. The console lists every move
(`moved R8 by 0.30, 0.00 mm`) and the output file has the footprints at the new positions. This
is a command line feature for now; the plugin does not import footprint moves.

Lock a footprint in KiCad (*Locking > Lock* on the part) to keep it from moving.

**Blocks** (preliminary): put the parts of a pre-routed cell, say a converter with its locked
tracks and vias, into a KiCad group (select them, right click, *Grouping > Group*). Under
`--move-parts` the group moves as one block, copper included, so the layout inside stays intact.
A group with a locked or fixed member does not move at all. Zones inside a group are not moved
yet, and moving blocks on a real board is untested.

## Keep the designer's track widths

When you re-route an existing board and want the same widths the designer used per net:

```bash
bench/net_widths.py original.kicad_pcb --json widths.json   # also prints length and via totals
gummiband route stripped.kicad_pcb -o out.kicad_pcb --net-widths widths.json
```

The JSON is plain `{"net name": width_mm}`; you can write it by hand for a few nets.

## Watch the search

```bash
gummiband route board.kicad_pcb -o out.kicad_pcb --frames frames/
open frames/evolution.gif
```

`--frames` writes the best candidate after every phase as SVG, converts it to PNG with
`rsvg-convert` and joins the PNGs into `evolution.gif` with ImageMagick. In the plugin, tick
*Save an animation of the generations*; the images go to `gummiband-evolution/` next to the
project.

`--svg out.svg` draws the final result with its violations, `--svg-tris` adds the triangulation
the router plans on. Both open in any browser.

## Try placement variants

`gummiband-place` (`cargo build --release -p gummiband-place`) rebuilds the Lichtschnitzel
controller in another outline (smaller rectangle, square, circle) with the connectors and the
module fixed and the inner planes regenerated, so the router can be tried on alternatives of the
same design. It is specific to that board's part roles today; the scripts and parameters are in
[bench/place/README.md](../../bench/place/README.md).

## Drive the router from an AI agent

The MCP server exposes `load_board`, `list_nets`, `set_net_options`, `part_info`,
`add_locked_copper`, `route`, `drc` and `render` to Claude Code, Claude Desktop and other MCP
clients. An agent can load a board, give the critical nets priority, route, read the violations
with positions, look at a PNG of the region and iterate.

New and preliminary: datasheet guided pre-routing. The agent asks `part_info` for the pads,
courtyard and neighbours of a critical part, places the copper the datasheet recommends with
`add_locked_copper` (tracks as polylines, vias; written as locked items into a working copy
`<input>.prerouted.kicad_pcb`, your input stays untouched), checks it with `render`, and then
routes the working copy, where the router keeps the pre-routed copper and attaches to it. Setup
and tool reference in [MCP.md](../MCP.md).
