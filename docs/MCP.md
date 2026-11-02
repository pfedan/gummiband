# MCP server

`gummiband-mcp` is a [Model Context Protocol](https://modelcontextprotocol.io) server that
lets AI agents drive the router: load a board, look at nets and classes, set per net
priorities or skip nets, route, run KiCad's DRC on the result and look at PNG renders of board
regions. It is milestone M3b (see [MILESTONES.md](MILESTONES.md)).

- Transport: stdio, one JSON-RPC 2.0 message per line; logs go to stderr.
- Protocol: built on the official Rust SDK [`rmcp`](https://crates.io/crates/rmcp) 3.8. It
  serves both eras of the protocol: the `initialize` handshake (legacy revisions 2024-11-05
  up to 2025-11-25, e.g. 2025-06-18) and the stateless current revision 2026-07-28
  (`server/discover`, protocol version in every request's `_meta`).
- Every tool returns a short text summary, the result as JSON text (for clients that only show
  `content`) and the same JSON as `structuredContent`, which follows the tool's
  `outputSchema`. `render` also returns the PNG as image content. Failures (unknown board id,
  missing file, unknown net, kicad-cli errors) come back as tool results with `isError: true`
  and a message the agent can act on.
- Session state: boards are kept in memory by id (`b1`, `b2`, ...) for the life of the server
  process. Loading the same file again re-reads it and keeps its id and net options.

## Build

```bash
cargo build --release -p gummiband-mcp
# binary: target/release/gummiband-mcp
```

External tools: `kicad-cli` (KiCad 9/10) for `drc` and `render`, and `rsvg-convert` (librsvg,
`brew install librsvg`) for `render`. They are searched in `GUMMIBAND_KICAD_CLI` /
`GUMMIBAND_RSVG_CONVERT`, then `PATH`, then the usual places
(`/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli`, `/opt/homebrew/bin`,
`/usr/local/bin`, `/usr/bin`). MCP clients often start servers with a short `PATH`, so set the
variables if the tools live elsewhere.

Threads: routing runs on the rayon pool. `RAYON_NUM_THREADS` sets its size; without it the
server uses 4 threads (the machine is usually shared). Only one `route` runs at a time; other
tools keep answering while a route runs.

## Register the server

### Claude Code

```bash
claude mcp add gummiband -e RAYON_NUM_THREADS=8 -- /path/to/gummiband/target/release/gummiband-mcp
# for all projects: claude mcp add --scope user gummiband -- /path/to/.../gummiband-mcp
claude mcp list
```

Routing a real board takes from seconds to minutes. If Claude Code gives up on long tool
calls, raise its tool timeout (environment variable `MCP_TOOL_TIMEOUT`, in milliseconds,
when starting `claude`).

### Claude Desktop

`~/Library/Application Support/Claude/claude_desktop_config.json` (macOS):

```json
{
  "mcpServers": {
    "gummiband": {
      "command": "/path/to/gummiband/target/release/gummiband-mcp",
      "args": [],
      "env": {
        "RAYON_NUM_THREADS": "8",
        "GUMMIBAND_KICAD_CLI": "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli",
        "GUMMIBAND_RSVG_CONVERT": "/opt/homebrew/bin/rsvg-convert"
      }
    }
  }
}
```

Restart Claude Desktop after editing the file.

## Tools

Coordinates are millimetres in KiCad board coordinates (y grows downwards), the same numbers
KiCad shows in the status bar. Optional arguments may be left out.

| Tool | Arguments | Result |
| --- | --- | --- |
| `create_board` | `netlist`, `output`, `width`, `height`, `x0?`, `y0?`, `corner_radius?`, `layers?`, `plane_layers?`, `thickness?`, `copper_oz?`, `prepreg?`, `mounting_holes? {diameter?, inset?, footprint?, net?}`, `net_classes? [{name, clearance?, track_width?, via_diameter?, via_drill?, diff_pair_width?, diff_pair_gap?, priority?, patterns}]`, `rules?`, `footprint_dirs?`, `pin_aliases?`, `overwrite?` | id of the new board (already loaded), paths of the `.kicad_pcb` and `.kicad_pro`, file version, footprint, hole, net and pad counts, pads with a net, missing footprints, parts whose pads and pins do not match (with the `pin_aliases` entry to pass), libraries read, parts that did not fit inside the outline, `warnings` |
| `load_board` | `path`, `keep_locked?` | board id, layers (plane flag), outline box and size, pins, nets, routable nets, connections, planes, net classes (priority, width, clearance, via), custom rule count, custom clearance net pairs, kept net options, recognised differential pairs |
| `list_nets` | `board`, `filter?`, `include_single?` | nets with class, pin count, plane flag, effective priority, skip flag |
| `board_patterns` | `board` | local patterns of the board: pin rows (part, pins in order, pitch, the side tracks leave to), pad grids, decoupling capacitors with their IC pin, pins of a plane net that can share a fan-out via, buses between two parts; a summary line per kind |
| `part_info` | `board`, `reference` | value, position, rotation, locked, sheet, pads (name, net, position, size, layers), courtyard box, neighbouring footprints within 3 mm |
| `move_part` | `board`, `reference`, `x`, `y`, `rotation?`, `follow?` | id and path of the working copy with the footprint at its new position, `items_added`, `warnings` |
| `place_parts` | `board`, `only?`, `scope?`, `fixed?`, `seed?`, `starts?`, `gap?`, `halo?`, `semantic?` | id and path of the working copy, the new poses, decoupling assignment, crossings, smallest courtyard distance, pins off their plane, pads without via room, released connectors, with `semantic` inductors, groups and ties, `warnings` |
| `add_zone` | `board`, `net`, `layer`, `points`, `priority?`, `pad_connection?`, `thermal_spoke_width?` | id and path of the working copy with the new zone, `items_added`, `warnings` |
| `add_locked_copper` | `board`, `tracks? [{net, layer, width?, points}]`, `vias? [{net, x, y, diameter?, drill?}]` | id and path of the working copy `<input>.prerouted.kicad_pcb` with the copper added as locked items, `items_added`, `warnings` |
| `set_net_options` | `board`, `net`, `priority?`, `skip?`, `width?`, `reset?` | the net's options and effective priority |
| `route` | `board`, `runs?`, `seed?`, `generations?`, `geometry_generations?`, `keep_locked?`, `keep_all?`, `move_parts?`, `fixed?`, `reroute?`, `output?`, `nets?`, `via_pitch?`, `via_cost?`, `layer_dirs?`, `patterns?`, `pair_pull?`, `pairs?`, `pair_all?`, `pair_breakouts?`, `pair_via_pairs?`, `pair_skew?`, `pair_zdiff?`, `negotiate?`, `via_pass?`, `open_end_seconds?`, `open_via_cost?`, `air?`, `prefer_width?`, `angles?`, `tune?`, `tune_amplitude?`, `tune_spacing?`, `max_violations?` | the fields of `gummiband route --report` plus session fields (see below): per net numbers, the nets in the way of each unrouted connection, with the experimental `pairs` one entry per differential pair |
| `angles_45` | `path`, `output?`, `nets?` | path of the written `<input>.45.kicad_pcb`, runs of track looked at and changed, length before and after, pieces off the grid, sharp corners, Gummiband's violations before and after |
| `power_plan` | `board`, `currents? [{net, amps, short_amps?}]`, `temp_rise?`, `apply?` | per power net the IPC-2221 width per layer, the track width to route with, vias per layer change, planes and zones, advice; board notes (GND plane, reference planes); the widths as `.kicad_dru` rules; with `apply` the widths stored as net options |
| `current_check` | `path`, `paths? [{net, amps, short_amps?, from?, to?, name?}]`, `temp_rise?`, `refill?`, `max_segments?` | per current path the narrow segments, the narrowest single band and the total cross-section between the terminals (pours and vias included) with their position, vias against the current, verdict `ok` or `FLAG` |
| `power_copper` | `board`, `paths [{net, amps, short_amps?, from, to?, layer?, name?}]`, `temp_rise?`, `margin?`, `max_span?`, `via_diameter?`, `via_drill?`, `priority?` | id and path of the working copy with "gb:" power pours over the pads of each path and, for plane nets, locked via fields with a pour into the plane; per pour its outline, area, what cut it, vias needed and placed; a `current_check` of the pours; `warnings` |
| `ground_pour` | `path`, `net?`, `layers?`, `pitch?`, `edge_pitch?`, `edge_inset?`, `avoid_courtyards?`, `via_diameter?`, `via_drill?`, `priority?`, `pad_connection?`, `thermal_spoke_width?`, `output?` | id and path of `<input>.gnd.kicad_pcb` with ground zones on the outer layers and locked stitching vias, `warnings` |
| `drc` | `path`, `max_items?` | KiCad DRC summary |
| `render` | `path`, `layers?`, `region? {x0,y0,x1,y1}`, `width_px?`, `background?`, `output?`, `refill?` | PNG (image content) and its path, region, pixels per mm |

### create_board

Turns a KiCad netlist (s-expression format: Eeschema File > Export > Netlist > KiCad, or
`kicad-cli sch export netlist`) into a `.kicad_pcb` without KiCad or its Python API, and loads
it (the result carries the board id).

- Footprints: each symbol's `Library:Footprint` is read from the KiCad footprint libraries.
  Searched in this order: `footprint_dirs` (`.pretty` directories or directories that contain
  them), the project `fp-lib-table` next to the output, the netlist or the schematic it names
  (`${KIPRJMOD}` and environment variables expanded), `KICAD10_FOOTPRINT_DIR`,
  `KICAD9_FOOTPRINT_DIR`, `KICAD8_FOOTPRINT_DIR`, `KICAD_FOOTPRINT_DIR`, the user's global
  `fp-lib-table`, and the usual install places (`/usr/share/kicad/footprints`,
  `/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints`, ...). A symbol whose
  footprint is not found stays off the board and is listed in `missing_footprints`; `libraries`
  says which directory each library came from. Symbols marked "exclude from board" are left out.
- Fields: Reference, Value, Datasheet, Description, the user fields (LCSC, MPN, ...) as hidden
  properties, DNP and "exclude from BOM" as footprint attributes, and the sheet path, name and
  file that link the footprint to its symbol (so KiCad's Update PCB from Schematic matches them).
- Nets: a pad gets the net of the netlist pin with its number, with pin function and pin type
  (`+no_connect` on `unconnected-(...)` nets). A pad without a netlist pin stays without a net,
  a pin without a pad leaves its net short of a connection: both are listed in `unmatched` and in
  `warnings`. If exactly one pad number and one pin of a part are left, the warning names the
  `pin_aliases` entry, e.g. a USB-C shield whose symbol pin is `S1` and whose pads are `SH`:
  `"pin_aliases": {"J1": {"S1": "SH"}}` gives all `SH` pads the net of `S1`.
- Board: rectangular outline from `x0`, `y0` (default 100, 100) of `width` x `height` mm with an
  optional `corner_radius`; `layers` copper layers (default 2; 4 gives F.Cu, In1.Cu, In2.Cu,
  B.Cu; any even count up to 32), `plane_layers` of KiCad type power (Gummiband does not route
  them, see `load_board`); a stackup of `thickness` (default 1.6 mm) with `copper_oz` per layer
  (one value for all, default 1 oz = 0.035 mm), `prepreg` (default 0.21 mm) next to the outer
  layers and the cores taking the rest, FR4, 0.01 mm masks.
- `mounting_holes`: four locked holes at the corners, `inset` mm (default 4) from both edges,
  plain non-plated holes of `diameter` (default 3.2) or a library `footprint` (with `net` for a
  plated one).
- Placement: the parts sit unlocked inside the outline, the larger ones first, each at the
  topmost then leftmost free place around the holes, with a gap of 1 mm (down to 0 mm if needed,
  see `grid_gap`). What does not fit sits in rows below the board (listed in `outside`), so
  `place_parts` with `scope: "outside"` takes it in. Both are only a start for `move_part` and
  `place_parts`.
- `.kicad_pro`: written next to the board, or merged into an existing one (the schematic's
  project; the rest of it stays). It gets `board.design_settings.rules` (KiCad's defaults for
  edge clearance, hole clearances and via sizes if missing, overridden by `rules`) and
  `net_settings` with the `net_classes` and their patterns; a class named Default sets the
  defaults of the others. Nets that the schematic puts into a class the patterns do not give go
  into `netclass_assignments` if `net_classes` defines that class (a warning otherwise).
- File format: KiCad 9 (version 20241229, with net table) when all footprint files are KiCad 9
  or older, otherwise KiCad 10 (20260206, nets by name), so the result opens in the KiCad that
  made the libraries. An existing output is only replaced with `overwrite: true`.

```json
{"name": "create_board", "arguments": {"netlist": "hw/lichtschnitzel.net",
  "output": "hw/lichtschnitzel.kicad_pcb", "width": 74, "height": 60, "layers": 4,
  "plane_layers": ["In1.Cu", "In2.Cu"], "mounting_holes": {"diameter": 3.2, "inset": 4},
  "rules": {"min_clearance": 0.127, "min_copper_edge_clearance": 0.3},
  "net_classes": [{"name": "Default", "clearance": 0.15, "track_width": 0.2},
    {"name": "Power", "track_width": 0.5, "via_diameter": 0.8, "via_drill": 0.4,
     "patterns": ["+5V_*", "+3V3", "GND"]}],
  "pin_aliases": {"J1": {"S1": "SH"}}}}
```

### load_board

Reads the `.kicad_pcb` with its `.kicad_pro` (net classes) and `.kicad_dru` (custom rules) of
the same base name. `keep_locked` (default false) keeps locked tracks and vias as fixed copper
whose ends join their nets; by default everything is routed from scratch, like
`gummiband route`. `connections` is counted like `gummiband info` (pins - 1 per net with 2 or
more pins); `route` counts plane nets per fan-out, so its total is lower on boards with planes.
`diff_pairs` lists the differential pairs recognised by net name (`..P`/`..N`, `..+`/`..-`) with
their class. They are routed as two independent nets unless `route` gets `pair_pull` or `pairs`;
the summary says so.

```json
{"name": "load_board", "arguments": {"path": "bench/work/lichtschnitzel/lichtschnitzel.kicad_pcb"}}
```

### list_nets

`filter` is a case insensitive substring of the net or class name. Nets with fewer than two
pins are left out unless `include_single` is true.

```json
{"name": "list_nets", "arguments": {"board": "b1", "filter": "gnd"}}
```

### board_patterns

The structure a person sees at a glance, found from the pins: rows of pins of one part at a
regular pitch, pad grids (BGA), two pin parts that decouple an IC pin, pins of a plane net close
enough to share one fan-out via, and buses (runs of neighbouring pins whose nets each go to one
pin of another row, with the flag whether the order is kept). `route` uses them as hints when
`patterns` names them (`decaps`, `rows`, `shares`, `all`), with the syntax of `gummiband route
--patterns`.

```json
{"name": "board_patterns", "arguments": {"board": "b1"}}
```

### part_info

Everything an agent needs to plan copper around one footprint: its value, position, rotation,
lock state and schematic sheet, its pads with name, net,
position, size and layers, the courtyard box `[x0, y0, x1, y1]` and the references of the
footprints whose courtyard lies within 3 mm.

```json
{"name": "part_info", "arguments": {"board": "b1", "reference": "U6"}}
```

### move_part

Preliminary. Moves a footprint to a new position in mm (and rotation in degrees, unchanged if
left out) in the working copy `<input>.prerouted.kicad_pcb`; the input file stays untouched and
the returned board id is the one to route. `warnings` names the footprints whose courtyard
the moved part now overlaps (by outline where the courtyard lines close, by box otherwise), and
an outline it leaves (by box); run `drc` or `render` for the exact answer. Pads, their nets and the footprint's own copper move along. Without
`rotation` only the position changes; with it the pads, the footprint's texts and its own zones
turn with the footprint, so the part arrives as KiCad would place it.

Without `rotation` the tracks around the part follow like rubber bands (`follow`, default
true), as in `gummiband move-part`: the part moves in small steps, its own tracks stretch, tracks
and vias of other nets give way within their clearances, and copper nothing touches stays as it
is. The part stops short where nothing can give way any more, or where its courtyard meets
another part or the edge; a warning then names where it stopped and why, and the message counts
the tracks and vias that followed. `follow: false` only moves the footprint, as before.

```json
{"name": "move_part", "arguments": {"board": "b1", "reference": "C12", "x": 152.4, "y": 118.0, "rotation": 90}}
```

### place_parts

Gives parts a position from the netlist in the working copy, see
[`gummiband place`](guide/cli.md#gummiband-place). An agent with its own plan places the parts it
cares about with `move_part` and leaves the rest to this tool: `fixed` names what stays, `only` what
moves. `scope` is `unlocked` (the default: not locked and not mechanical), `all_unlocked` or
`outside`. Locked footprints never move. `starts` (default 1) is the number of independent
placements, of which the one with the fewest crossings is kept; `gap` (0.3 mm) is the spacing
between courtyards and `halo` (0.3 mm) the extra room around parts with five or more pads. The
result names the working copy to route next, the new pose of every moved part (`placed`), which
decoupling capacitor was assigned to which part, the `crossings` between the nets (a measure of
the vias to expect), the smallest distance between two courtyards and the pins that ended up off
the plane of their net (`off_plane`, routing reaches them with a via). Locked copper stays: parts
keep `gap` away from it, and a part with locked copper of its net on a pad stays (`anchored`), so add
pre-routed copper before this call, not after. A part that still sits in the pile of a fresh import
(its courtyard covers a good part of another one) or off the board is not anchored, whatever copper
touches it. Pads of a net with a plane (small SMD pads, which need a fan-out via) keep a via site
free next to them: kept tracks and vias of other nets on any layer, holes and rule areas without
vias push them off, so that a ground pad does not land over a locked track on the other side.
Pads that found no room anyway are listed in `no_via_room`. When the parts do not fit, the error
names the parts that found no spot (largest first), the courtyard area they and all movable parts
need with `gap`, the free area, and the options with their size: the `gap` that would make the
room, what `halo: 0` frees, about how much more board area it takes, and the area the fixed parts
take.

With the default scope connectors, modules, buttons and test points stay, unless they sit piled up,
on a pad of a larger part (one that was moved onto them) or off the board: then they are placed
with the rest and listed in `released`, with a warning.
Place them yourself with `move_part` and name them in `fixed` for a real layout. `warnings` also
names every part that stays piled up or with its courtyard outside the outline (fixed, locked or
out of scope); the tool never returns such a placement silently. Unlocked tracks and vias do not follow, so call it
before `route`. The tool places, it does not judge: `route` and `drc` tell how good the result is,
another `seed` gives another try, and seeds differ a lot. Neighbouring seeds share no starts. The
crossing count is no forecast of the routed result.

`semantic` (default false) uses the schematic beyond the netlist, as `gummiband place --semantic`
does: sheets and clusters around an IC stay together, an inductor sits at the switch pins of its
converter, decoupling capacitors prefer an IC of their own sheet, and current paths count their
full length, so an input chain stays in order. A two-pin part on a small net is pulled to the part
with the most pads on it (a pull-down to its IC pin, a boot resistor to its converter, a fuse to the
next part of the input chain), and a small regulator with only rails on its pins to the biggest IC
on its output rail (an LDO to the MCU it feeds). The result then lists `coils` (inductor, IC),
`groups` (name, parts) and `ties` (part, the part it was pulled to). Connectors and the power stages you care about still go in `fixed`.

```json
{"name": "place_parts", "arguments": {"board": "b1", "fixed": ["U1", "L1"], "seed": 2}}
{"name": "place_parts", "arguments": {"board": "b1", "only": ["C12", "C13", "R4"]}}
```

### add_zone

Preliminary, tested on small boards only. Adds a copper zone of `net` on `layer` with the
outline `points` (`[x, y]` in mm, at least three) and `priority` (default 0) to the working copy.
On a KiCad power layer the zone becomes a plane the router connects to with vias; on a routing
layer it is kept like any other zone. `pad_connection` sets how the pads of the net join it:
`thermal` (default, thermal reliefs), `solid`, or `thermal_tht` (reliefs on through hole pads,
SMD pads solid); `thermal_spoke_width` (default 0.3 mm) widens the spokes for pins that carry
amperes (four spokes of 0.3 mm carry about what a 1.2 mm track carries). Gummiband does not fill zones: KiCad does it when it fills
the zones, and `drc` runs with a refill, so the result is judged correctly.

```json
{"name": "add_zone", "arguments": {"board": "b1", "net": "GND", "layer": "In1.Cu",
  "points": [[140, 110], [200, 110], [200, 150], [140, 150]]}}
```

### add_locked_copper

Preliminary (new, lightly tested). Adds pre-routed copper as **locked** tracks and vias: the
router keeps it, routes around it, and track ends join their net like pins. Tracks are polylines
of `[x, y]` points in mm on one layer; `width`, `diameter` and `drill` default to the net class,
and a `width` of 0 or less is an error. The input file stays untouched: the copper goes into a
working copy `<input>.prerouted.kicad_pcb` next to it, which is loaded with `keep_locked` and
gets its own board id. Use that id for `route`, `drc` and `render` from then on. Every call adds
to the working copy, whether you pass the input's id or the copy's, so earlier additions are never
lost.

The result has `items_added` and `warnings`: a point off the board, a track on a plane layer,
and every track piece or via closer to the pads, tracks or vias of another net than the
clearance asks for, routed tracks of the file included. Copper that would touch another net is
refused with an error naming both nets, and nothing is written: KiCad would join the two nets
there. Zones are not counted, they fill around the new copper. A `drc` after routing remains the
final check.

```json
{"name": "add_locked_copper", "arguments": {"board": "b1",
  "tracks": [{"net": "VBUS_IN", "layer": "F.Cu", "width": 0.8,
              "points": [[150.0, 120.5], [153.2, 120.5], [153.2, 123.0]]}],
  "vias": [{"net": "GND", "x": 151.0, "y": 122.0}]}}
```

Typical use, the datasheet layout of an eFuse or a converter: `part_info` for the part and its
neighbours, `add_locked_copper` for the recommended copper, `render` of the region to check it,
then `route` with the returned board id.

### set_net_options

Stored in the session by net name and applied on every following `route` of the board:
`priority` overrides the net class priority (lower routes first), `skip: true` leaves the net
out of routing (its pads stay obstacles; its old tracks are not kept, since the output replaces
all unlocked copper). `width` (mm) gives the net its own track width, which is also its minimum
width: it does not neck below it, except inside the courtyards of the parts whose pads are
narrower than the width (fine pitch pins); `width: 0` clears it. Known gap: the router still
lets such a net neck in places outside those courtyards (VBUS_F on the Lichtschnitzel benchmark
kept 0.25 to 0.6 mm pieces); `current_check` finds them. `power_plan` with `apply` sets
the widths of the power nets this way. `reset: true` drops the net's options before applying the
others.

```json
{"name": "set_net_options", "arguments": {"board": "b1", "net": "/MCU/SDA", "priority": -10}}
{"name": "set_net_options", "arguments": {"board": "b1", "net": "VBUS_F", "width": 0.75}}
{"name": "set_net_options", "arguments": {"board": "b1", "net": "VBUS_IN", "skip": true}}
```

### route

Runs `gummiband_core::route_board` on the loaded board with the net options applied and writes
the routed board (existing unlocked tracks and vias replaced; with `keep_locked` the locked ones
stay). Defaults match the CLI: `runs` 14 (a constant, so results do not depend on the machine), `generations` 3,
`geometry_generations` 3, `via_pitch` 0.5 mm, `via_cost` 3.0, `layer_dirs` 0.5 (preferred
directions per layer for every third run, 0 = off), `pair_pull` 0 (experimental pull between the
nets of a differential pair: the search keeps the second net near the first one's way on the same
layer, and the finished tracks close up to the pair's gap wherever they run within 1.5 mm of each
other and nothing lies between; works for every recognised pair, also in the `Default` class), `pairs` false (experimental: a pair as one wide track that is split
in two, see the CLI guide; only pairs with a pair rule or a net class of their own, `pair_all`
true for every pair), `pair_breakouts` false (experimental: the wide track ends just outside the
pads), `pair_via_pairs` false (experimental: vias side by side as one capsule), `pair_skew` 0.1 mm, `pair_zdiff` none, `tune` true (meanders for the
`length` and `skew` rules of the `.kicad_dru`, written as KiCad tuning patterns),
`tune_amplitude` 1 mm, `tune_spacing` 2 track widths between meander legs, `negotiate` 0
(negotiated routing from the start, 20 rounds work well and take about twice the time; with 0
the search negotiates on its own when it leaves a few connections open), `via_pass`
4 (factor on the via cost in the pass that routes connections with vias again and keeps what has
fewer vias, 0 = off), `open_end_seconds` 0 (off; a value keeps improving the result for that many
seconds after the normal search, like `--until-stop` with a time budget, and `open_via_cost`,
default 20 mm, is what a via is worth when it judges its results), `air` 0 mm, `prefer_width` 0
(off), `angles` `"any"` (`"45"` turns the result into conventional 45 degree routing as a last
step, like `--angles 45`). `seed` picks the search seed
(default fixed): the same seed gives the same result, another one a different try. `keep_all`
keeps every track and via of the input and adds to it, like `--keep-all` on the command line;
with `nets` only those nets are routed and everything else stays. Add `reroute` to replace
connected nets: the unlocked copper of `nets` goes first, then they are routed anew. With
`keep_all` and `nets` the unlocked tracks of the other nets shift aside where a connection needs
the room (`shove`, default true), and if a connection still finds no way, those within 8 mm of
its pins give way and are routed again (`give`, default 8, in steps of 2, 4, 8 mm; the try with
the fewest open connections, then the fewest violations is written). `shove: false` and `give: 0`
leave the other nets where they are; locked copper and plane nets never move. With
`open_end_seconds` the tries of `give` run without the open end, and the one kept goes on open
ended for the rest of the time (the budget counts from the start of the call). `output` defaults to
`<input>.routed.kicad_pcb` next to the input; the `.kicad_pro` and `.kicad_dru` are copied next
to the output if missing, so `drc` and `load_board` work on it. `nets` routes only the listed
nets for this call.

Without `keep_all` the geometry generations may shift parts next to clearance problems and open
connections, while the routed copper follows like rubber bands (`move_parts`, default true
without `keep_all`, as `--move-parts` of the CLI). Locked footprints, connectors, modules,
mounting holes, buttons and test points stay, and so do the references in `fixed`. The output
has the footprints at their new positions, and `moved` lists them with their shift in mm.
`move_parts` does not go together with `shove`.

Result fields: `board`, `moved`, `input`, `output`, `seconds`, `threads`, `connections`, `routed`,
`unrouted` (list of `net: pad -> pad`), `vias`, `length_mm`, `items_written`, `skipped_nets`,
`violations_total`, `violations_by_kind`, `violations` (first `max_violations`, default 50:
`what`, `x`, `y`, `actual_mm`, `required_mm`, `nets` [routed net, other net or null]; the kinds
are those of the CLI's `clearance check`, and for `width` and `current` the two values are
widths; `hole_to_hole` holds between all drilled holes, of the same net too, and `width` also
names a track below the project's `min_track_width` or `min_connection`, which KiCad reports
as `track_width` or `connection_width` whatever the net class allows),
`portfolio` (per run: `generation`, `seed`, `fanout_first`, `routed`, `connections`, `vias`,
`length_mm`, `violations`), `pairs` (with `pairs`), `coupling` (every recognised pair, whatever
the settings: `p`, `n`, `coupled_mm` within width + gap + 0.1 mm of the partner on the same layer,
`length_mm` of both nets, `vias_p`, `vias_n`, and with `pair_pull` or `pairs` `uncoupled`: the
longest stretches of 0.5 mm and more that do not run beside the partner, `net`, `layer`, `x`, `y`,
`length_mm`, `why` one of "partner on another layer", "partner farther away" (a breakout to the
pins, or a way of its own), "something between"), `notes` (hints, such as pairs routed without
`pair_pull` or `pairs`, or a pair with `pair_pull` that runs coupled for less than 60 % of its length), `length_tuning` (every net the tuning aimed at: `net`, `before_mm`,
`after_mm`, `aim_mm`, `as_pair`), `length_rules` (every length, skew, uncoupled and gap rule
after the tuning, measured as the KiCad DRC measures: `rule`, `kind`, `min_mm`, `opt_mm`,
`max_mm`, `nets`, `failed` with `net`, `length_mm`, `skew_mm`), `nets` (per routed net, longest first: `net`,
`length_mm`, `vias`, `min_width_mm`, `max_width_mm`), `blocked` (per unrouted connection the
`nets` that run in its way, a neighbourhood estimate rather than an exact analysis, meant for
choosing what to prioritize, skip or move), `copied`, `angles_45` (with `angles` `"45"`:
`length_before_mm`, `length_after_mm`, `off_grid_pieces` and `off_grid_mm` that kept their angle
because no 45 degree path fit, `sharp_corners` left unchamfered).

```json
{"name": "route", "arguments": {"board": "b1", "runs": 2, "generations": 0}}
```

### angles_45

Turns the unlocked tracks of a routed `.kicad_pcb` into conventional 45 degree tracks without
routing again, like `gummiband angles45`: the same step as `route` with `angles: "45"`, on copper
that is already there. Vias, track ends, pads and locked tracks stay; every new piece keeps the
clearances, and a piece with no room on the grid keeps its angle. `nets` limits it to those
nets. The output goes to `<input>.45.kicad_pcb` by default, with the `.kicad_pro` and
`.kicad_dru` copied next to it for `drc`.

```json
{"name": "angles_45", "arguments": {"path": "/abs/path/lichtschnitzel.routed.kicad_pcb"}}
```

### Good practice: power_plan, power_copper, current_check, ground_pour

The router does the heavy lifting; around it go the rules of the craft a careful layouter
applies: plan the power copper before routing, check the current carrying capacity after it,
pour ground and stitch it. These four tools and the prompt `good_practice` (below) cover that,
so an agent can take a board from netlist to DRC in one go. Currents follow IPC-2221
(I = k * dT^0.44 * A^0.725, A in mil², k = 0.048 outer, 0.024 inner, default rise 20 K) with the
copper thickness of each layer from the board's stackup (35 um without one). A via counts with
its barrel (drill plus 25 um plating, inner k): a 0.3 mm via carries about 1.3 A, a 0.4 mm via
about 1.6 A at 20 K.

#### power_plan

Before routing. `currents` names the power nets by name or glob (`*VBUS_P`, `/Power/SW_?`) with
their continuous current and, where a fuse or PTC limits a short, the short term current the
copper must carry until it trips (the larger of both sizes the copper). Without `currents` the
tool takes the nets of clearly wide classes (twice the default width and at least 0.5 mm) and
the current their minimum width (or class width) carries, and lists nets that look like supplies
by name but have a narrow class.

Per net: the required width per copper layer, `track_width_mm` (outer layers, rounded up to
0.05 mm), the planes and zones it has, `vias_per_transition` and `advice`. The advice says to
widen the net, to keep it off thin inner layers, to give each pin into a plane enough vias, or to
draw a pour instead of a track: above 2.5 mm, or above 1 mm into pins narrower than half the
width (a 1.75 mm switch node track into 0.3 mm converter pins blocks the router around them; on
the Lichtschnitzel benchmark it turned 36 clearance violations into 2541). Nets that already
have a plane or a pour on a routing layer are left to `current_check`. `board_notes` cover the
ground plane (missing, or on which layer), plane layers without a zone and, on boards with
differential pairs, routing layers whose nearest plane is not ground. `dru_rules` holds the
widths as KiCad custom rules (`track_width` min and opt, courtyards of fine pitch parts exempt)
to append to the project's `.kicad_dru`: KiCad's DRC then enforces them, and Gummiband reads
them on the next `load_board`. `apply: true` stores the widths as net options (`width`, see
`set_net_options`) for the following routes.

```json
{"name": "power_plan", "arguments": {"board": "b1", "apply": true, "currents": [
  {"net": "*VBUS_RAW", "amps": 4.1}, {"net": "VBUS_F", "amps": 4.1},
  {"net": "*/SW_?", "amps": 8}, {"net": "*OUT?_5V", "amps": 4, "short_amps": 7}]}}
```

#### power_copper

Before routing, for current paths a track carries badly: a converter's switch node from its
pins to the inductor, a big pad that reaches its plane through one via. For each path:

- A net without plane gets one pour over its `from` and `to` pads on the layer they share (outer
  first, or `layer`): the convex hull of the pads grown by `margin` (0.5 mm), cut back by straight
  lines wherever copper, holes, rule areas or reserved zones of other nets come closer than
  their clearance, and along the board edge. Each cut runs across the line from the obstacle to
  the nearest point of the spine between the terminals, so the pour gives way on that side only.
  Terminals further apart than `max_span` (12 mm) get no pour (route them wide); a pad the pour
  no longer reaches gets a warning. Several `from` terminals that one pour cannot join (the
  two VBUS pin groups of a USB-C receptacle with the signal pins between them) get one pour
  each to the `to` pads, with a warning; join them on another layer if they must share the
  current.
- A long narrow pad (at least twice as long as wide: the IN and OUT strips of a DFN/QFN eFuse,
  the drain of a power FET) that one convex pour cannot reach, because its pin row stands
  between the pad's middle and the other terminal or a neighbour beside the strip cuts the
  pour down to the strip, is left through an exit: through a gap in a long side or off an end,
  between its neighbours. The pour is then several zones: a narrow stub over the pad out to the
  exit (as wide as the gap between the neighbours allows, reported in `exits` and flagged by
  the `current_check` when it is a neck) and the pour from there. A via field at such a pad
  gives each via the one pour misses its own pour through an exit.
- The paths are laid out in order: the pours and vias of the earlier paths are in the way of
  the later ones (of other nets).
- A net with a plane gets, at every terminal, locked vias for the current (IPC-2221 per via; the
  `from` terminals share the current, every `to` terminal takes all of it) on rings just off the
  pad (the holes stay out of the pad; vias already touching it count), free by the same rules as
  stitching vias except that the part's own courtyard is allowed, and a small pour joining pad
  and vias. A via only counts where it lands in the copper of a plane of the net: inside the
  plane zone with room for the via and its clearance, and outside the zones of other nets on
  that layer with a higher priority (KiCad fills those first). When vias would fit beside the
  pad but the plane has no copper there, the warning says so (extend the plane zone).

Courtyards count by their outline where the courtyard lines close (an L or T shaped module
courtyard leaves the room beside its stem free), by their box otherwise. The same holds for the
stitching vias of `ground_pour` and the overlap warnings of `move_part`.

Overlapping zones on a plane layer: the router and `power_copper` see a plane only where no
zone of another net with a higher priority covers it, so a via of the lower net does not land
in the higher net's area.

The pours are zones named `gb:...` with priority 20 and solid pad connections; a pour that
overlaps a zone of the same priority on its layer (two via fields of FETs side by side) goes one
priority up, so KiCad reports no `zones_intersect`. Gummiband keeps
zones with that prefix as obstacles for other nets even without `zones_as_obstacles`, and counts
pads under one such pour (and vias in it that reach the plane) as connected, so the router
neither runs a track across the pour nor routes the joined pads again. The tool writes the
working copy `<input>.prerouted.kicad_pcb` (like `add_locked_copper`); route the returned board.
It ends with a `current_check` of the pour paths (the zone outline counts until KiCad fills it).

On the Lichtschnitzel benchmark, buck A: the switch node pour (U4.5 to U4.9 into L1.1, 8 A)
gives way to U4.10 and the exposed GND pad and measures 3.5 mm cross-section against 1.74 mm
required; L1.2 gets 7 vias of 0.3 mm into the `+5V_A` plane instead of one. Routed afterwards:
all connections, 19 instead of 36 Gummiband clearance violations, none near the converter.

```json
{"name": "power_copper", "arguments": {"board": "b1", "paths": [
  {"name": "buck A SW", "net": "/Power/SW_A", "amps": 8,
   "from": ["U4.5+U4.6+U4.7+U4.8+U4.9"], "to": ["L1.1"]},
  {"name": "buck A out", "net": "+5V_A", "amps": 8, "from": ["L1.2"]}]}}
```

#### current_check

After routing, on the written file. A path names a net, its current and optionally its terminals:
`from` (source pads as `REF.PAD`, `REF.*` for every pad of the part on the net, pins in parallel
joined with `+` as one terminal; several terminals share the current evenly) and `to` (sink
pads, any of them may take the current; `+` is allowed there too). A net with two pins gets its terminals automatically; a
glob without terminals expands to its nets. Without `paths` the nets of the wide classes are
checked.

For every path the copper of the net (pads, tracks, vias and its filled zones, on every layer it
has copper) is put on a 0.05 mm raster, and two numbers are measured between the terminals:

- `width_mm`: the narrowest single band, the largest width whose band still joins every source
  terminal to the sinks (the copper eroded by half the width; terminal and via pads are not
  eroded, and a band may end up to 0.3 mm from them). Inner layers count with their width
  divided by the factor IPC-2221 asks for there. This is the measure of the Lichtschnitzel
  `layout/necks.py` and gives its numbers for the VBUS paths of that board (0.70, 0.70,
  0.28 mm). necks.py lets a band end up to half its width from a pad, so where two terminals are
  closer than the width it bridges the gap without copper and reports its search limit (3.99 mm
  for the switch nodes and the outputs); the 0.3 mm limit here measures the copper that is there.
- `cross_section_mm`: the total cross-section, a minimum cut on the raster where parallel copper
  adds up and every via counts with what its barrel carries. Copper that is a band of the full
  width anyway cannot be cut, so the cut runs through narrower copper and vias only; a cut at an
  angle through narrow copper counts a little short (the safe side). On a net with a lot of
  copper (a ground net with a plane and pours) the cut is measured in a window around the
  terminals, smaller until it fits; the copper outside it does not count (the safe side) and a
  note gives the window.

The cross-section decides: a path is `ok` when it reaches the required width (within half a
raster cell), else `FLAG` with the position and layer of the cut, marked `via` when a via barrel
is the limit. `segments` lists the track segments narrower than required (outside fills of the
net), which also covers nets without terminals. Zones without fill are refilled with `kicad-cli
pcb drc --refill-zones --save-board` in a temp copy (KiCad 9 or newer); without kicad-cli their
outline counts, which is optimistic, and a note says so. IPC-2221 describes long conductors: a
neck of a few tenths of a millimetre right at a pad runs cooler than it says, so judge a short
flag by its length.

On the finished Lichtschnitzel board the check finds the known 0.28 mm neck of `VBUS_IN` and
shows that `VBUS_F` and `VBUS_IN` each pass a single 0.4 mm via (about 1.6 A) on their 4.1 A path.

```json
{"name": "current_check", "arguments": {"path": "/abs/lichtschnitzel.routed.kicad_pcb", "paths": [
  {"name": "USB-C to fuse", "net": "/Power/VBUS_RAW", "amps": 4.1,
   "from": ["J1.A4+J1.B9", "J1.A9+J1.B4"], "to": ["F1.1"]},
  {"name": "fuse to shunt", "net": "VBUS_F", "amps": 4.1, "from": ["F1.2"], "to": ["R110.*"]},
  {"name": "buck A switch node", "net": "/Power/SW_A", "amps": 8,
   "from": ["U4.5+U4.6+U4.7+U4.8+U4.9"], "to": ["L1.1"]}]}}
```

A flag is fixed by widening the net (`set_net_options` width, then `route` with `keep_all`,
`nets` and `reroute`), a pour over the neck (`add_zone`, or `power_copper` before the next
route), more vias or locked copper, then another check.

#### ground_pour

After routing: zones of the ground net (default `GND`, or the net with the most pins whose name
holds GND) over the board outline on the outer layers (or `layers`), with `priority` 0 so power
pours with a higher priority win, and locked stitching vias: a row along the edge
(`edge_pitch` 3 mm, `edge_inset` 1 mm from the outline) and a grid (`pitch` 5 mm; a blocked
grid spot tries eight neighbours a quarter pitch away, and spots next to an existing via of the
net are skipped). A via goes only where it keeps the clearance plus 0.3 mm to copper of other
nets on every layer, the hole to hole distance, the edge clearance, out of rule areas that
forbid vias, of other nets' zones on routing layers and of courtyards (`avoid_courtyards`). A
layer that already has a zone of the net is left as it is (the small `gb:` pours of
`power_copper` do not count); `pad_connection` and `thermal_spoke_width` work as for `add_zone`
(`solid` clears KiCad's `starved_thermal` where the spokes find no room, at the cost of harder
soldering for small passives); `warnings` name zones of other nets
whose priority does not beat the pour. The output `<input>.gnd.kicad_pcb` is loaded as a new
board. The zones are filled with kicad-cli before the file is handed back (a warning says so if
that fails), so `render` and KiCad show the pours right away. A via the fill does not reach shows
up in `drc` as dangling, unless an inner plane of the net holds it.

```json
{"name": "ground_pour", "arguments": {"path": "/abs/lichtschnitzel.routed.kicad_pcb"}}
```

#### The prompt good_practice

The server offers one MCP prompt, `good_practice` (arguments `path`, optional `currents` and
`notes`). It asks the agent to take the board through the whole sequence without asking back
unless the design intent changes: look at the board and its power paths, settle the currents,
plan with `power_plan` (planes, pours, datasheet layouts as locked copper, `dru_rules` into the
project), place and route, `current_check` every path and fix the flags, `ground_pour`, `drc`
and `render`, and report a table of the current paths. In Claude Code it appears as
`/mcp__gummiband__good_practice`.

### drc

Runs `kicad-cli pcb drc --refill-zones --format json --units mm` and summarizes it. The
`.kicad_pro` with the board's base name must sit next to the board; if it is missing it is
copied (with the `.kicad_dru`) from a project whose name is a prefix of the board's
(`x.routed.kicad_pcb` takes `x.kicad_pro`) or from the only project in the folder. The full
report is written next to the board as `<name>.drc.json`.

Result: `counts` per violation type without silkscreen (`silk*`) and library (`lib_*`) checks,
`violations` (their sum), `ignored`, `unconnected` and the first `max_items` (default 20)
`unconnected_items`, `deficit_histogram` of the clearance type violations (< 2 um, < 10 um,
< 30 um, < 100 um, more) and the `worst_clearance` violations with type, required, actual,
deficit and the items with positions.

```json
{"name": "drc", "arguments": {"path": "/abs/path/lichtschnitzel.routed.kicad_pcb", "max_items": 5}}
```

### render

Plots the layers with `kicad-cli pcb export svg` (page coordinates equal board coordinates),
crops the SVG to `region` and rasterizes it with `rsvg-convert`. Default layers: the routing
copper layers (no inner planes, which are solid pours) and `Edge.Cuts`; default region: the
board outline plus 1 mm; `width_px` defaults to 1000 (at most 4000), the height follows the
region. The PNG goes to the server's temp folder unless `output` is given. Zones without fill
(after `add_zone` or a pour that was never filled) plot as nothing, so by default they are filled
in a temp copy first (`refill`, kicad-cli 9 or newer); the file itself is not changed.

```json
{"name": "render", "arguments": {"path": "/abs/path/lichtschnitzel.routed.kicad_pcb",
  "region": {"x0": 140, "y0": 120, "x1": 160, "y1": 140}, "width_px": 800}}
```

## Try it without a client

Each line on stdin is one message; wait for an answer before sending the next request
(stdin closing ends the server).

```bash
RAYON_NUM_THREADS=4 target/release/gummiband-mcp <<'EOF'
{"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-06-18","capabilities":{},"clientInfo":{"name":"shell","version":"0"}}}
{"jsonrpc":"2.0","method":"notifications/initialized"}
{"jsonrpc":"2.0","id":2,"method":"tools/list"}
{"jsonrpc":"2.0","id":3,"method":"tools/call","params":{"name":"load_board","arguments":{"path":"bench/work/lichtschnitzel/lichtschnitzel.kicad_pcb"}}}
EOF
```

Shortened session on the Lichtschnitzel benchmark (4 threads):

```text
>> initialize {"protocolVersion":"2025-06-18",...}
<< {"protocolVersion":"2025-06-18","capabilities":{"tools":{}},"serverInfo":{"name":"gummiband-mcp",...},"instructions":"..."}
>> tools/call load_board {"path":".../lichtschnitzel.kicad_pcb"}
<< Loaded lichtschnitzel.kicad_pcb as board b1: 100.0 x 60.0 mm, 4 copper layers (F.Cu, In1.Cu plane,
   In2.Cu plane, B.Cu), 525 pins, 111 nets (92 to route, 414 connections), 11 net classes, 10 custom rules.
>> tools/call list_nets {"board":"b1","filter":"gnd"}
<< {"count":1,"nets":[{"class":"GND","name":"GND","pins":150,"plane":true,"priority":4,"skip":false}]}
>> tools/call route {"board":"b1","runs":2,"generations":0,"output":".../lichtschnitzel.routed.kicad_pcb"}
<< Routed 346/348 connections (2 unrouted), 338 vias, 2458 mm of track, 29 clearance violations
   (Gummiband check), 44.7 s on 4 threads. Wrote 3306 items to .../lichtschnitzel.routed.kicad_pcb.
>> tools/call drc {"path":".../lichtschnitzel.routed.kicad_pcb"}
<< KiCad DRC: 2 unconnected, 23 violations {"clearance": 18, "copper_sliver": 1, "items_not_allowed": 1,
   "shorting_items": 1, "tracks_crossing": 2} (428 silk/lib findings ignored).
   Clearance deficits <2um/<10um/<30um/<100um/more: [4, 2, 10, 2, 0].
>> tools/call render {"path":"...","region":{"x0":140,"y0":120,"x1":160,"y1":140},"width_px":800}
<< Rendered ... (F.Cu,B.Cu,Edge.Cuts) region x 140.00..160.00, y 120.00..140.00 mm at 800x800 px (40.0 px/mm)
   + image/png content
```

`cargo test -p gummiband-mcp` runs the same handshake, `tools/list`, `load_board`,
`list_nets`, `set_net_options` and a small `route` on a two resistor board written by the test.

## Open points

- `move_part` and `add_zone` are new and tested on small boards only; `move_part` checks
  courtyard outlines, not the clearance KiCad's DRC wants between them, and a zone is not filled until KiCad fills it.
- `place_parts` is new. It lacks the `--iters`, `--margin` and `--free` options of
  `gummiband place`, knows no datasheet layouts (place and lock power stages with `move_part`;
  `semantic` keeps an inductor at its converter, but no via arrays or copper),
  and treats both board sides as one space.
- Not reachable through MCP yet: per net widths and priorities from files (`--net-widths`,
  `--net-priorities`; both go through `set_net_options`).
- `create_board` reads rectangular outlines only, places every part on the front side and
  without rotation, and does not read footprint libraries of other types than KiCad (`.pretty`).
- `power_copper` shapes a pour as a convex hull cut by straight lines (plus a narrow stub out of
  a long narrow pad, see above): obstacles between the terminals cost more copper than needed,
  and a terminal behind another net's pad is lost (a warning says so). It does not move parts; the via fields sit on rings around the pad.
- `current_check` measures on a raster (0.05 mm, coarser on very large nets); it does not weigh
  the length of a neck, and the cross-section is skipped for nets of more than three million
  copper cells. `ground_pour` does not yet check that the fill reaches every via (the DRC does).
- Progress notifications during long routes, and cancellation (the core has no cancel hook).
- `route` returns Gummiband's own clearance check; KiCad's verdict needs a `drc` call.
