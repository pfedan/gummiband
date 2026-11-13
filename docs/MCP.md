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

## Working with the tools: by hand, automatic, both

Gummiband is a toolbox for the layouter, not a one-stop shop. The agent stays the designer: it
decides where the parts that matter go, which layers are planes, how the power copper runs and
when a result is good enough. `place_parts` and `route` are helpers for the bulk of the work. They
complement manual placement, manual routing and planes; they do not replace them, and everything
they produce can be edited afterwards. A good board usually mixes all three: the critical parts and
copper by hand, the rest automatic, then a round of fine-tuning by hand again.

The server reads this section back through the tool `guide` (topic `workflow`), together with every
tool section below and the guide chapters on concepts and options.

### What the agent can do by hand

- **Place parts**: `move_part` sets position and rotation of one footprint; on a routed board its
  tracks follow like rubber bands. `part_info` shows pads, courtyard and neighbours first.
- **Route by hand**: `add_locked_copper` adds tracks (polylines on one layer) and vias, for a
  datasheet layout, a power lane, a critical net or a single connection the router did badly. The
  copper is always locked, and copper that would short another net is refused.
- **Planes and pours**: `set_layer_type` makes inner layers plane layers, `add_zone` puts zones on
  them (several side by side for a split plane) or on an outer layer, `power_copper` shapes pours
  and via fields for current paths, `ground_pour` pours ground with stitching vias.
- **Remove**: `remove` takes zones, tracks, vias and footprints away again, `add_mounting_hole`
  adds a hole where it is needed.
- **Lock and unlock**: `lock` locks footprints by reference and the tracks and vias of nets (or all
  copper) in place, `unlock: true` releases them again.
- **Rules per net**: `set_net_options` sets priority, width, minimum width, the parts to neck at
  and skip for the next route.

### What the automatic tools do with it

- **Locked stays.** Locked footprints never move in `place_parts` or in the part shifts of `route`
  (`move_parts`). Locked tracks and vias are kept by `route`, routed around and joined by their
  net, and never shoved or replaced; in `move_part` they neither follow nor give way. The working copies
  of the editing tools are loaded with `keep_locked`; for a file of your own with locked copper,
  pass `keep_locked: true` to `load_board` (or `scope: "keep_locked"` to `route`).
- **Unlocked is fair game, unless the scope keeps it.** `route` with `scope` `"keep_locked"`
  replaces all unlocked tracks and vias of the board. With `"finish"` it keeps them all and lays
  only what is still open, and with `"reroute"` and `nets` it replaces only the unlocked copper
  of those nets while the unlocked tracks of other nets may shift aside (`shove`) or be routed
  again near the pins (`give`). `place_parts` moves every part that is not locked, not in `fixed`
  and in scope.
- **So lock before you hand over**: what you placed (`lock` the parts or name them in `fixed`), what
  you routed (`add_locked_copper` locks by itself), and autorouted nets you are happy with (`lock`
  with `nets`) before you route the rest again.
- **Use the newest board id.** Every editing tool writes a working copy next to the input and
  returns its board id; the input file is never changed.

### Order of work

1. Look: `load_board`, `list_nets`, `board_patterns`, `part_info` on connectors, regulators and
   the main ICs.
2. By hand, what has its place: connectors, mounting holes, power stages and other datasheet
   layouts (`move_part`, then `lock`), plane layers and zones (`set_layer_type`, `add_zone`), power
   copper (`power_plan`, `power_copper`, `add_locked_copper`).
3. Automatic, the rest: `place_parts` for the parts you did not place, `route` for the nets you did
   not route. Compare a few seeds.
4. Fine-tuning by hand and in small automatic steps (below), until `drc`, `current_check` and
   `reference_check` are clean and the renders look right.

The order is no rule. An agent may route a few critical nets by hand after a first automatic run,
or place everything itself and only let `route` finish; `place_parts` with `only` and `route` with
`nets` keep each automatic step as small as wanted.

### Fine-tuning after place_parts and route

- **Move a part on a routed board**: `move_part` on the routed board id; its tracks stretch and
  other nets give way, nothing is routed again. Then `lock` it.
- **Route one net differently**: draw the way you want with `add_locked_copper` (the whole net or
  only the critical stretch), then `route` with `scope: "reroute"` and `nets`: the router's old
  unlocked copper of that net goes, your locked copper stays, and the router joins the rest.
- **Route a few nets again**: `set_net_options` (priority, width), then `route` with
  `scope: "reroute"` and `nets`. `lock` the nets that must not move first.
- **Nets laid by hand, the rest to the router**: `route` with `scope: "finish"`. Every track and
  via on the board stays, locked or not, and only the open connections are laid.
- **Freeze a good result**: `lock` with `all_copper: true`, then route only what is left with
  `scope: "finish"` (and `nets` for single ones), or edit by hand without the router touching
  the rest.
- **Necks and current**: widen with `set_net_options` `width` and `min_width` and route the net
  again, add a pour
  over the neck (`add_zone`), or `power_copper` with `unlocked: "clear"` on the routed board and
  route its `reroute_nets` again.
- **Copper after routing**: `ground_pour`, `add_zone`, `angles_45` for conventional 45 degree
  tracks.
- **Judge**: `drc`, `render` of the region you changed, `current_check`, `reference_check`.

- **Take something away**: `remove` deletes zones (a `power_copper` pour by its `gb:` name),
  tracks and vias of nets or within a box, and footprints such as a mounting hole;
  `add_mounting_hole` adds one.
- **Silk and rules**: `place_silk` moves the reference texts off pads and other silk,
  `set_rules` adds custom KiCad rules (a smaller clearance in the courtyard of a fine pitch part).
- **Compare seeds**: `try_seeds` runs `place_parts` and `route` over several seeds and keeps the
  best.

Not reachable through MCP yet: changing a zone's outline in place (remove it and add it anew) and
editing silk graphics beyond `place_silk`. KiCad itself remains the tool for those, and the server
reads the edited file again with `load_board`.

## Tools

Coordinates are millimetres in KiCad board coordinates (y grows downwards), the same numbers
KiCad shows in the status bar. Optional arguments may be left out.

| Tool | Arguments | Result |
| --- | --- | --- |
| `create_board` | `netlist`, `output`, `width`, `height`, `x0?`, `y0?`, `corner_radius?`, `layers?`, `plane_layers?`, `thickness?`, `copper_oz?`, `prepreg?`, `mounting_holes? {diameter?, inset?, footprint?, net?}`, `net_classes? [{name, clearance?, track_width?, via_diameter?, via_drill?, diff_pair_width?, diff_pair_gap?, priority?, patterns}]`, `rules?`, `footprint_dirs?`, `pin_aliases?`, `overwrite?`, `parts_outside?` | id of the new board (already loaded), paths of the `.kicad_pcb` and `.kicad_pro`, file version, footprint, hole, net and pad counts, pads with a net, missing footprints, parts whose pads and pins do not match (with the `pin_aliases` entry to pass), libraries read, parts that did not fit inside the outline (or all with `parts_outside`), stale working copies removed by `overwrite`, `warnings` |
| `load_board` | `path`, `keep_locked?` | board id, layers (plane flag), outline box and size, pins, nets, routable nets, connections, planes, net classes (priority, width, clearance, via), custom rule count, custom clearance net pairs, kept net options, recognised differential pairs |
| `list_nets` | `board`, `filter?`, `include_single?` | nets with class, pin count, plane flag, effective priority, skip flag |
| `board_patterns` | `board` | local patterns of the board: pin rows (part, pins in order, pitch, the side tracks leave to), pad grids, decoupling capacitors with their IC pin, pins of a plane net that can share a fan-out via, buses between two parts; a summary line per kind |
| `part_info` | `board`, `reference` | value, position, rotation, locked, sheet, pads (name, net, position, size, layers), courtyard box, neighbouring footprints within 3 mm |
| `move_part` | `board`, `reference`, `x`, `y`, `rotation?`, `follow?`, `side?` | id and path of the working copy with the footprint at its new position, `items_added`, `warnings` |
| `track_corridors` | `board`, `fixed?` | groups of tracks between two parts (`a`, `b`, `nets`, `width_mm`, `length_mm`) with the movable parts `in_the_way` of each band, and all of them once; nothing moves |
| `place_parts` | `board`, `only?`, `scope?`, `fixed?`, `seed?`, `starts?`, `gap?`, `halo?`, `semantic?`, `corridors?`, `escape_halo?`, `leash?`, `two_sided?`, `back_max_pads?`, `back_cost?`, `on_front?`, `on_back?` | id and path of the working copy, the new poses (with `back` for a part on the back), decoupling assignment, crossings, smallest courtyard distance, pins off their plane, pads without via room, released connectors, with `semantic` inductors, groups and ties, with `two_sided` `flipped` and `free_mm2` per side, `warnings` |
| `add_zone` | `board`, `net`, `layer`, `points`, `priority?`, `pad_connection?`, `thermal_spoke_width?` | id and path of the working copy with the new zone, `items_added`, `warnings` |
| `set_layer_type` | `board`, `layers [{layer, type}]` | id and path of the working copy with the new KiCad layer types (`signal`, `power`, `mixed`, `jumper`), per change the old type and the nets with a zone on the layer, all copper layers with their plane flag, `warnings` |
| `add_locked_copper` | `board`, `tracks? [{net, layer, width?, points}]`, `vias? [{net, x, y, diameter?, drill?}]` | id and path of the working copy `<input>.prerouted.kicad_pcb` with the copper added as locked items, `items_added`, `warnings` |
| `lock` | `board`, `parts?`, `nets?`, `all_copper?`, `unlock?` | id and path of the working copy with the lock flags changed, `locked`, the `parts` whose state changed, `tracks` and `vias` changed, `warnings` (unknown references, nets without copper) |
| `remove` | `board`, `zones? [{name?, net?, layer?}]`, `nets?`, `area? [x0, y0, x1, y1]`, `layers?`, `copper?`, `only?`, `parts?` | id and path of the working copy, the zones removed, tracks and vias removed, footprints removed, `warnings` |
| `add_mounting_hole` | `board`, `x`, `y`, `diameter?`, `reference?` | id and path of the working copy with the new locked hole, `warnings` (courtyards it overlaps) |
| `place_silk` | `board`, `parts?`, `min_size?`, `margin?`, `edge_margin?`, `reach?`, `hide_unplaced?`, `trim_edge?` | id and path of the working copy, references kept, moved, shrunk, hidden, left, footprints whose edge silk was trimmed |
| `set_rules` | `board`, `rules`, `replace?` | id of the working copy, path of its `.kicad_dru`, rule count, what Gummiband understood of the rules, `warnings` |
| `set_net_options` | `board`, `net`, `priority?`, `skip?`, `width?`, `min_width?`, `neck_refs?`, `reset?` | the net's options and effective priority |
| `try_seeds` | `board`, `seeds?`, `place?`, `route?`, `drc?` | per seed (best first) the placed and routed files, unrouted, Gummiband violations, KiCad DRC errors and unconnected, vias, length; the best seed, the working copy now holding its placement and its routed board |
| `route` | `board`, `output?`, `max_violations?` and every option of the route pipeline under the name `gummiband route` has it (there as `--kebab-case`): `scope?`, `nets?`, `preset?`, `shove?`, `give?`, `move_parts?`, `fixed?`, `plane_layers?`, `areas?`, `pour?`, `area_stitch?`, `runs?`, `via_pitch?`, `via_cost?`, `layer_dirs?`, `patterns?`, `via_pass?`, `pairs?`, `pair_pull?`, `pair_skew?`, `pair_zdiff?`, `pair_all?`, `breakouts?`, `via_pairs?`, `negotiate?`, `generations?`, `geometry_generations?`, `seed?`, `frames?`, `open_end_seconds?`, `open_via_cost?`, `air?`, `prefer_width?`, `angles?`, `tune?`, `tune_amplitude?`, `tune_spacing?`, `placements?`, `placement_seed?`, `clear_corridors?`, `net_widths?`, `net_priorities?`, `teardrops?` | the fields of `gummiband route --report` plus session fields (see below): per net numbers, the nets in the way of each unrouted connection, with `pairs` `"band"` one entry per wide track |
| `angles_45` | `path`, `output?`, `nets?` | path of the written `<input>.45.kicad_pcb`, runs of track looked at and changed, length before and after, pieces off the grid, sharp corners, Gummiband's violations before and after |
| `power_plan` | `board`, `currents? [{net, amps, short_amps?}]`, `temp_rise?`, `apply?` | per power net the IPC-2221 width per layer, the track width to route with, vias per layer change, planes and zones, advice; board notes (GND plane, reference planes); the widths as `.kicad_dru` rules; with `apply` the widths stored as net options |
| `current_check` | `path`, `paths? [{net, amps, short_amps?, from?, to?, name?}]`, `temp_rise?`, `refill?`, `max_segments?` | per current path the narrow segments, the narrowest single band and the total cross-section between the terminals (pours and vias included) with their position, vias against the current, verdict `ok` or `FLAG` |
| `power_copper` | `board`, `paths [{net, amps, short_amps?, from, to?, layer?, name?}]`, `temp_rise?`, `margin?`, `max_span?`, `via_diameter?`, `via_drill?`, `via_rings?`, `via_in_pad?`, `vias_in_courtyards?`, `via_margin?`, `priority?`, `unlocked?` | id and path of the working copy with "gb:" power pours over the pads of each path and, for plane nets, locked via fields with a pour into the plane; per pour its outline, area, what cut it, vias needed and placed; a `current_check` of the pours; the unlocked copper on the board, with `unlocked: "clear"` what was removed (`cleared`) and the nets to route again (`reroute_nets`); `warnings` |
| `reference_check` | `path`, `nets?`, `min_gap?`, `refill?`, `max_items?` | per routing layer its reference layers and their zone nets; signal tracks over no copper (`gap`, with length) or across the border between two zone nets (`split`) on their reference layer, with position; per net the length over no copper; short gaps only counted |
| `ground_pour` | `path`, `net?` or `nets?`, `layers?`, `stitch?`, `via_in_pad?`, `pad_connection?`, `thermal_spoke_width?`, `thermal_gap?`, `zone_clearance?`, `zone_min_width?`, `pitch?`, `edge_pitch?`, `edge_inset?`, `avoid_courtyards?`, `via_diameter?`, `via_drill?`, `priority?`, `output?` | id and path of `<input>.gnd.kicad_pcb` with ground zones on the outer layers and locked stitching vias; per net what the pour reaches and which pads stay unreached and why; `warnings` |
| `area_report` | `path`, `nets?`, `pour?`, `zone_clearance?`, `zone_min_width?`, `thermal_gap?`, `thermal_spoke_width?`, `pad_connection?`, `plane_layers?`, `step?`, `doubtful?` | per net with a zone what its fill reaches, judged by the router's fill model: `summary`, pieces per layer, `unreached` pads with the reason, `islands` with the site of one via each, `site_distance`, `doubtful` pads |
| `drc` | `path`, `max_items?` | KiCad DRC summary |
| `render` | `path`, `layers?`, `region? {x0,y0,x1,y1}`, `width_px?`, `background?`, `output?`, `refill?` | PNG (image content) and its path, region, pixels per mm |
| `guide` | `topic?` | how to use Gummiband: the workflow of manual and automatic steps (default), one section per tool, concepts, options, open points; without a known topic the list of topics |

### Manufacturer limits: JLCPCB as an example

Gummiband routes to the rules of the project (`.kicad_pro` net classes and design rules, the
`.kicad_dru`); it has no fab profile of its own. When a board goes to JLCPCB, an agent should
set the rules of `create_board` (or the project) from the numbers below. JLCPCB's routing guide
(jlcpcb.com/blog/pcb-routing-rules-best-practices, 2026) gives these minimums; start with the
recommended values and go down to the minimum only where it is needed (BGA and fine pitch fan-out,
best as a local `.kicad_dru` rule for the courtyard of that part).

| Rule | Minimum 1 to 2 layers | Minimum 4+ layers | Recommended |
| --- | --- | --- | --- |
| Track width, 1 oz | 0.10 mm (4 mil) | 0.09 mm (3.5 mil), 0.075 mm at extra cost | signals 0.2 to 0.25 mm (8 to 10 mil) |
| Track to track, 1 oz | 0.10 mm | 0.09 mm, 0.075 mm locally in BGA fan-out | wider wherever there is room |
| Track to track, 2 oz | 0.16 mm | 0.15 mm | |
| Power and ground tracks | | | 0.4 to 0.5 mm (15 to 20 mil) and more, sized by `power_plan` |
| Via drill / diameter | 0.15 / 0.25 mm | 0.15 / 0.25 mm | 0.3 / 0.6 mm; 0.2 / 0.4 mm for fast signals (less parasitic) |
| Hole to hole | 0.20 mm | 0.20 mm | |
| Annular ring, plated holes | 0.25 mm (at least 0.18) | 0.20 mm (at least 0.15) | |
| Copper to board edge | 0.2 mm milled, 0.4 mm V-cut | same | |
| Solder mask to track | 0.09 mm | 0.09 mm | |
| Silkscreen to pad, text height | 0.15 mm, 1.0 mm | same | |

The same guide's craft rules and where Gummiband has them: decoupling capacitors at the pins
(`board_patterns`, `place_parts`), critical nets first (net class priority, `set_net_options`),
compact switching loops (`power_copper`, `add_locked_copper`), widths by current (`power_plan`,
`current_check`, IPC-2221), every signal layer next to a solid ground plane (`power_plan` board
notes) and unbroken under its tracks (`reference_check`), 45 degree corners or arcs (the router
draws arcs; `angles_45`), meanders at 3W (`tune_spacing` 2 track widths of air), ground pour with
stitching (`ground_pour`), DRC without errors (`drc`). Spacing against crosstalk (the 3W rule,
centre to centre three track widths) is no router option: give sensitive nets (clocks, analog
inputs, references) a larger clearance with a `.kicad_dru` rule, which the router reads, e.g.
`(rule "3W clocks" (condition "A.NetClass == 'CLK'") (constraint clearance (min 0.4)))` for
0.2 mm tracks.

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
  `place_parts`. With `parts_outside: true` every part but the mounting holes sits in rows
  below the board from the start: the board stays empty for the parts you place by hand and for
  copper drawn before the placement (`add_locked_copper`, `add_zone`), which would otherwise
  meet the pile of parts and be refused as a short. Before this option the 4 port LED controller
  run needed 84 `move_part` calls only to park the parts.
- `.kicad_pro`: written next to the board, or merged into an existing one (the schematic's
  project; the rest of it stays). It gets `board.design_settings.rules` (KiCad's defaults for
  edge clearance, hole clearances and via sizes if missing, overridden by `rules`) and
  `net_settings` with the `net_classes` and their patterns; a class named Default sets the
  defaults of the others. Nets that the schematic puts into a class the patterns do not give go
  into `netclass_assignments` if `net_classes` defines that class (a warning otherwise).
- File format: KiCad 9 (version 20241229, with net table) when all footprint files are KiCad 9
  or older, otherwise KiCad 10 (20260206, nets by name), so the result opens in the KiCad that
  made the libraries. An existing output is only replaced with `overwrite: true`, which also
  removes the working copy of the old board (`<output>.prerouted.kicad_pcb` with its
  `.kicad_pro`, `.kicad_dru` and `.kicad_prl`, listed in `removed`): otherwise the editing tools
  would go on from the stale copy (parts on the old side, zones twice).

```json
{"name": "create_board", "arguments": {"netlist": "hw/ledctl.net",
  "output": "hw/ledctl.kicad_pcb", "width": 74, "height": 60, "layers": 4,
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
their class. They are routed as two independent nets unless `route` gets `pairs` `"pull"` or
`"band"`; the summary says so.

```json
{"name": "load_board", "arguments": {"path": "bench/work/ledctl/ledctl.kicad_pcb"}}
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
`patterns` names them (`decaps`, `rows`, `shares`, `pairs`, `groups`, `groups-short`, `all`), with the syntax of `gummiband route
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

`side` (`"front"` or `"back"`) puts the part on that side: a part on the other side is flipped
as KiCad flips it (layers, pads, graphics and texts mirrored to the other side) and placed at
`x`, `y` with `rotation`. The tracks do not follow a flip; a warning says so. Use it to bring back
a part that `place_parts` with `two_sided` put on the back, or to put one there yourself.

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

`corridors` and `escape_halo` (both default false, experimental) keep room for groups of tracks
between two parts and for the escape of each side of a part, as `gummiband place --corridors` and
`--escape-halo` do.

`leash` (default 0: free) keeps parts with 4 or more pads (ICs, connectors) within that many mm
of where they sit, as `gummiband place --leash`: a local change instead of a new placement. With
`only`, `corridors: true` and `leash: 1.5` it moves the parts that `track_corridors` lists out of
a band while the ICs stay, which is what `route` with `clear_corridors` does for all of them.

`two_sided` (default false: one room for both sides, every part keeps its side) places on both
sides, as `gummiband place --two-sided`: each side gets its own room, and a through hole part takes
only its holes on the other side, so small parts fit under its body. SMD parts with at most
`back_max_pads` pads (default 8) may change sides and are flipped as KiCad flips them (mirrored,
on the layers of the other side); `on_front` and `on_back` put named movable SMD parts on a side
whatever their pads. A part on the back pays `back_cost` mm of wire (default 6) and a net that then
needs a via between the sides pays for it, so the back takes what does not fit on the front or
saves more wire there; 0 treats both sides alike. The result marks parts on the back with `back`
in `placed`, lists the parts that changed sides in `flipped` and the free room of each side in
`free_mm2`. Before searching, the tool checks the courtyard area against the free room of each
side: when the parts cannot fit it fails at once, with the area missing, and a one sided call says
when front and back together have the room. A failed two sided call says when the parts that keep
the front are the problem (raise `back_max_pads` or name them in `on_back`).

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
{"name": "place_parts", "arguments": {"board": "b1", "two_sided": true, "on_back": ["U6"]}}
```

### track_corridors

Lists the groups of tracks between two parts, as `place --corridors` and `route
--clear-corridors` see them: two parts with at least 4 pads on nets (ICs, connectors) that share
at least two short signal nets (a bus, SPI, I2C, a parallel interface). For each group: the two
parts `a` and `b`, the `nets`, the band it needs (`width_mm` from the tracks with their
clearances, `length_mm` between the group's pads on the two parts), and the movable parts
`in_the_way`: parts in the band that are not on its nets. Parts on the group's nets (series
resistors, pull-ups), decoupling capacitors, locked and mechanical parts and the references in
`fixed` are never listed. `in_the_way` at the top level has every listed part once. The tool
reads the working copy if there is one and changes nothing.

Use it to decide yourself what moves: a group of a bus that matters, a part in the way that you
would rather keep (name it in `fixed`), then `place_parts` with `only` set to the parts you chose,
`corridors: true` and `leash: 1.5`, and route with `patterns` `groups`. `route` with
`clear_corridors` is the same step for every listed part at once.

```json
{"name": "track_corridors", "arguments": {"board": "b1"}}
{"name": "place_parts", "arguments": {"board": "b1", "only": ["R13", "C40"], "corridors": true, "leash": 1.5}}
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

### set_layer_type

Sets the KiCad type of inner copper layers in the working copy, so the planes can be planned
through the MCP. `power` makes a layer a plane layer: its zones are planes that the router
reaches with vias, and it routes no tracks there. `signal`, `mixed` and `jumper` make it a
routing layer (Gummiband routes all three alike). Outer layers stay routing layers (they carry
the parts; pour them with `add_zone` or `ground_pour`). Warnings name a new power layer without
a zone (add one with `add_zone`, or several side by side for a split plane), locked tracks left
on a new power layer, and zones that stop being planes when a power layer turns back into a
routing layer. Use the returned board id from now on.

```json
{"name": "set_layer_type", "arguments": {"board": "b1", "layers": [
  {"layer": "In1.Cu", "type": "power"}, {"layer": "In4.Cu", "type": "power"}, {"layer": "In6.Cu", "type": "power"}]}}
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

### lock

Locks (or with `unlock: true` unlocks) footprints by reference and the tracks, arcs and vias of
`nets` in the working copy `<input>.prerouted.kicad_pcb`; `all_copper: true` takes every track,
arc and via of the board. Only the lock flag of each item changes, nothing moves. Locked parts stay
where they are in `place_parts` and in the part shifts of `route`; locked copper is kept by `route`
(the working copy is loaded with `keep_locked`), routed around, and never shoved, given way or
replaced by a route with `scope` `"reroute"`. Unlocking hands an item back to the automatic tools.

Use it to fix what you placed with `move_part` before `place_parts`, to freeze an autorouted net
you are happy with before routing others again, or `all_copper` to keep a whole routed result
while you route the remaining nets with `scope` `"finish"`. `warnings` names references without
a footprint and nets without copper on the board.

```json
{"name": "lock", "arguments": {"board": "b2", "parts": ["J1", "U6", "L1"], "nets": ["VBUS_IN", "SW"]}}
{"name": "lock", "arguments": {"board": "b3", "all_copper": true}}
{"name": "lock", "arguments": {"board": "b3", "nets": ["/LED/DATA1"], "unlock": true}}
```

### remove

Removes what the other editing tools added, in the working copy: zones by `name`, `net` and
`layer` (each filter needs at least one field and a zone goes when it matches every field of one
filter; names and nets are globs with `*` and `?`, so `{"name": "gb:*"}` takes every pour of
`power_copper`), the tracks, arcs and vias of `nets` (globs), or of any net within `area`
(`[x0, y0, x1, y1]`, with `nets` only of those), narrowed by `layers` (a via goes if it spans one),
`copper` (`tracks`, `vias`, `all`) and `only` (`locked`, `unlocked`), and footprints by reference in
`parts` (a mounting hole; a part with pads on a net stays in the netlist, the result warns).
Zones inside footprints stay. It is how a pour, a via field or a hand drawn track is corrected
without building the board again: remove it, then add it anew or route the net again with
`scope: "reroute"` and `nets`.

```json
{"name": "remove", "arguments": {"board": "b4", "zones": [{"name": "gb:vias +5V_A Q2*"}], "nets": ["+5V_A"], "area": [132, 110, 136, 114], "copper": "vias", "only": "locked"}}
{"name": "remove", "arguments": {"board": "b4", "parts": ["H4"]}}
```

### add_mounting_hole

Adds one locked, plain non-plated mounting hole (`diameter`, default 3.2 mm) at `x`, `y`, as
`create_board` makes them, with the next free `H<n>` or `reference`. With `move_part` (which
moves holes too) and `remove` (`parts`) the holes are no longer bound to the four corners of
`create_board`. `warnings` names courtyards the hole overlaps.

```json
{"name": "add_mounting_hole", "arguments": {"board": "b4", "x": 130, "y": 124, "diameter": 2.7}}
```

### place_silk

Places the reference texts on the silk screen, after placement or on a routed board (copper does
not change). Each reference stays where it is if that spot is clear; else it goes to the nearest
clear spot around its courtyard (above, below, left, right, then the corners, stepping out up to
`reach` mm, default 2), first as it is, then turned by 90 degrees, then smaller in steps down to
`min_size` (default 0.8 mm, about the smallest legible text at JLCPCB); a reference with no clear
spot is hidden (`hide_unplaced: false` leaves it). Clear means: `margin` (0.1 mm) away from the
pads of its side, from the silk graphics of other footprints and from the texts placed before,
and `edge_margin` (0.3 mm) inside the board edge. Larger parts choose first. `trim_edge: true`
also removes footprint silk over or near the board edge (a connector body drawn beyond it,
KiCad's `silk_edge_clearance`).

The text size is estimated from KiCad's stroke font (0.9 of the height per character), so a
`drc` afterwards has the last word. On the 4 port LED controller 178 of 190 silk messages were
reference texts on pads or other silk, and 12 connector outlines over the edge.

```json
{"name": "place_silk", "arguments": {"board": "b7", "trim_edge": true}}
```

### set_rules

Adds KiCad custom rules (one or more `(rule ...)` items as in a `.kicad_dru`) to the working
copy, or with `replace: true` replaces them: the file `<input>.prerouted.kicad_dru` is written
and the board loaded again, so KiCad's `drc` and Gummiband both use them. The result lists what
Gummiband itself reads of them: clearance rules by net, class and pair, minimum widths per class
(with the courtyards exempt from them), length and skew rules, and clearance rules that hold only
in courtyards (`A.intersectsCourtyard('U4') || A.intersectsCourtyard('U6')`, e.g. 0.1 mm at
fine pitch power ICs). Gummiband's own check after `route` then accepts the smaller clearance
in those courtyards (and reports the rule's minimum where it is still short); the router itself
keeps the class clearance there, on the safe side.

```json
{"name": "set_rules", "arguments": {"board": "b2", "rules": "(rule \"fine pitch power ICs\" (condition \"A.intersectsCourtyard('U4') || A.intersectsCourtyard('U6')\") (constraint clearance (min 0.1)))"}}
```

### set_net_options

Stored in the session by net name and applied on every following `route` of the board. The
fields are those of an entry in the file of `gummiband route --net-widths`, and both go through
the same function:

- `priority` overrides the net class priority (lower routes first).
- `skip: true` leaves the net out of routing (its pads stay obstacles; its old tracks are not
  kept, since the output replaces all unlocked copper).
- `width` (mm) is the track width the net is routed with; `width: 0` clears it. On its own the
  track may still neck as its class and the rules allow.
- `min_width` (mm) is the minimum width: the track never necks below it, except inside the
  courtyards of `neck_refs`; `min_width: 0` clears it. A larger minimum of the class or the
  custom rules stays. For a track that has to carry its current everywhere, set it to the
  width. It must not be above the width.
- `neck_refs` names the parts in whose courtyard the track may neck below `min_width` (fine
  pitch pins). Left out, these are the parts whose pad on this net is narrower than
  `min_width`; an empty list allows it nowhere.
- `reset: true` drops the net's options before applying the others.

`power_plan` with `apply` sets `width` and `min_width` of the power nets this way. Known gap: the
router still lets such a net neck in places outside those courtyards (VBUS_F on the LED
controller benchmark kept 0.25 to 0.6 mm pieces); `current_check` finds them.

```json
{"name": "set_net_options", "arguments": {"board": "b1", "net": "/MCU/SDA", "priority": -10}}
{"name": "set_net_options", "arguments": {"board": "b1", "net": "VBUS_F", "width": 0.75, "min_width": 0.75}}
{"name": "set_net_options", "arguments": {"board": "b1", "net": "VBUS_IN", "skip": true}}
```

### try_seeds

Runs `place_parts` and `route` over several placement seeds (default 1 to 4) from the same
start, the board as you left it (hand placement, locks, zones, locked copper, power copper):
before each seed the working copy is restored. `place` holds the arguments of `place_parts`
(without `board` and `seed`), `route` those of `route` (without `board` and `output`). Each seed
keeps its placed working copy `<input>.seed<N>.placed.kicad_pcb` and its routed file
`<input>.seed<N>.routed.kicad_pcb`. With `drc` (default when `kicad-cli` is found) KiCad's DRC
runs on each routed file. The ranking: fewest unrouted (with KiCad's unconnected items), then
fewest KiCad errors (Gummiband's violations without `drc`), then fewest vias, then the shortest
length. Afterwards the working copy holds the placement of the best seed, and its routed file is
loaded too (`routed_board`).

On a tight board the same placement rules scatter widely: on the 4 port LED controller (60 x 46
mm, two sided) seeds 1 to 8 gave anything from 0 to 47 KiCad errors, so an agent should compare
a few seeds before it refines one.

```json
{"name": "try_seeds", "arguments": {"board": "b6", "seeds": [1, 2, 3, 4, 5, 6], "place": {"scope": "all_unlocked", "semantic": true, "two_sided": true}, "route": {"pairs": "pull"}}}
```

### route

Runs the route pipeline of `gummiband-app` on the file of the loaded board, the same one
`gummiband route` runs, with the net options of the session applied, and writes the routed board
(what of the copper on the board stays is the `scope`, see below). Every
option has the name, the default and the meaning it has on the command line (there as
`--kebab-case`); a test fails when the two differ in what they offer. One default is the
session's: `scope`. An argument that does not
exist is refused. Options that contradict each other are refused too, with a message that says
what to change (for example `move_parts` with `shove`, `scope` `"reroute"` without `nets`,
an open end with `pairs` `"band"`, `breakouts` without `pairs` `"band"`, `runs` 0); nothing is
adjusted silently. An option with a fixed list of words (`scope`, `preset`, `pairs`, `angles`) has the words as an
`enum` in the tool's schema, and its description says what each word means and when to pick it;
the text comes from the same table as the CLI's `--help`. An argument in the spelling of an
earlier version still works: the result is the same, and `notes` says what to write now
([Old spellings](#old-spellings-of-route)). The defaults: `preset` `"standard"` (how much
search the route gets, see below), `runs` 14 (a constant, so results do not depend on the machine), `generations` 3,
`geometry_generations` 3 (these three and `open_end_seconds` are what the preset says), `via_pitch` 0.5 mm, `via_cost` 3.0, `layer_dirs` 0.5 (preferred
directions per layer for every third run, 0 = off), `pairs` `"off"` (how the two nets of a
differential pair are laid: `"off"` each on its own; `"pull"`, experimental: the search keeps
the second net near the first one's way on the same layer, and the finished tracks close up to
the pair's gap wherever they run within 1.5 mm of each other and nothing lies between, which
works for every recognised pair, also in the `Default` class; `"band"`, experimental: a pair as
one wide track that is split in two, see the CLI guide, only for pairs with a pair rule or a net
class of their own, `pair_all` true for every pair), `pair_pull` (the weight of the pull: 1 with
`"pull"`; with `"band"` 0, and a value pulls the pairs that get no wide track; refused with
`"off"`), and with `pairs` `"band"` only: `breakouts` false (experimental: the wide track ends
just outside the pads), `via_pairs` false (experimental: vias side by side as one capsule),
`pair_skew` 0.1 mm, `pair_zdiff` none. `tune` true (meanders for the
`length` and `skew` rules of the `.kicad_dru`, written as KiCad tuning patterns),
`tune_amplitude` 1 mm, `tune_spacing` 2 track widths between meander legs, `negotiate` 0
(negotiated routing from the start, 20 rounds work well and take about twice the time; with 0
the search negotiates on its own when it leaves a few connections open), `via_pass`
4 (factor on the via cost in the pass that routes connections with vias again and keeps what has
fewer vias, 0 = off), `open_end_seconds` 0 (off; a value keeps improving the result for that many
seconds after the normal search, as `--open-end-seconds`; the clock starts when the open end
starts, so a short budget never cuts the normal search; `open_via_cost`, default 20 mm, is what
a via is worth when it judges its results; the results that differ from the output in vias and
length are written next to it as `<output>.altN.kicad_pcb` and listed in `alternatives`), `air`
(reserve on the clearances: 0.03 mm in an open ended run, else 0; see the scope below),
`prefer_width` 0 (off), `plane_layers` none (layers taken as planes for this route, as
`--plane-layers`), `areas` "tracks" ("fill" leaves the nets with an area on a routed layer, a zone
of the file or a pour named in `pour`, to the fill of that area: the other nets are laid first, and only a group of pads the fill then does not reach gets a
piece of track or a via to the area; `areas` in the result says per net what the fill reaches
on the finished copper and which pads it does not, each with its reason, and a group of pads
that neither fill nor copper joins to the area is listed in `unrouted` like any open
connection; `pour` names nets that get a pour after the route, each "NET" or
"NET:LAYER,LAYER", the outer layers where none is given; `area_stitch`, true with areas
"fill", gives an island of fill that holds pads one via where it joins the area instead of a
track, and `area_vias` in the result counts them; not with `pairs`, the open end or shove), `placements` 0 (a value tries that many placements from the netlist, routes
each and goes on with the best, as `--placements`; first seed `placement_seed`, default 1; the
output has the parts where the best placement put them and `placement` lists the tries; to keep
and compare the placed boards use `try_seeds`), `net_widths` and `net_priorities` none (JSON
files with options and priorities per net, as `--net-widths` and `--net-priorities`; what
`set_net_options` holds for a net wins field by field), `teardrops` false (KiCad's teardrops
switched on in the output and its project), `frames` none (a folder for one SVG of the best
candidate per generation), `angles` `"any"` (`"45"` turns the result into conventional 45 degree routing as a last
step, like `--angles 45`), `patterns` none (routing order hints, see
[Choosing opt-in options](#choosing-opt-in-options)), `clear_corridors` false (experimental:
before routing, the parts that `track_corridors` lists are placed again around the bands into the
working copy, ICs and connectors at most 1.5 mm from where they sit, as `route --clear-corridors`;
`board` of the result is the working copy, `cleared_corridors` names the parts; not with
`scope` `"finish"` or `"reroute"`). `seed` picks the search seed
(default fixed): the same seed gives the same result, another one a different try. `output`
defaults to `<input>.routed.kicad_pcb` next to the input and must not be the input file; the
`.kicad_pro` and `.kicad_dru` are copied next to the output if missing (`copied`), so `drc` and
`load_board` work on it.

`preset` says how much search the route gets. It sets `runs`, `generations`,
`geometry_generations` and `open_end_seconds`, and each of the four wins over it when given;
nothing else changes.

| `preset` | `runs` | `generations` | `geometry_generations` | `open_end_seconds` | Pick it for |
| --- | --- | --- | --- | --- | --- |
| `"draft"` | 4 | 1 | 0 | 0 | the quickest look at whether the board routes and where it is tight, between two tries of a placement; many clearance violations |
| `"quick"` | 14 | 3 | 0 | 0 | a first result on a real board, in about a third of the time of `"standard"`; many more clearance violations |
| `"standard"` | 14 | 3 | 3 | 0 | a usable board; the default |
| `"final"` | 14 | 3 | 3 | 600 | the last run of a board: the open end improves the result for 10 minutes; not with `pairs` `"band"` |

`scope` says what of the copper on the board stays, and what is laid:

| `scope` | What stays | What is laid | Pick it for |
| --- | --- | --- | --- |
| `"fresh"` | nothing: every track and via goes, locked or not | the whole board | a board to start over |
| `"keep_locked"` | locked tracks and vias, as fixed copper that the router connects to | everything else, anew | a board on which the copper that must stay is locked (`add_locked_copper`, `lock`) |
| `"finish"` | every track and via, locked or not | what is still open; with `nets` the open connections of those nets | "I laid these nets by hand, finish the rest", a track that was started, the last open connections of a routed board |
| `"reroute"` | everything but the unlocked copper of `nets`, which it needs | those nets, anew; their locked copper stays | "lay net X again" on a routed board |

Without `scope` a call routes with the scope of its board: `"keep_locked"` for a board that was
loaded with `keep_locked` and for every working copy (the board id of `add_locked_copper`,
`lock`, `move_part`, `place_parts` and the other editing tools), else `"fresh"`. On the command
line the default is `fresh`.

`nets` limits a call to the listed nets. With `"finish"` and `"reroute"` all other nets stay as
they are. With `"fresh"` and `"keep_locked"` only these nets get copper and the unlocked copper
of the others is gone from the output, so single nets on a routed board want `"finish"` or
`"reroute"`.

What hangs on the scope when a call does not say otherwise:

- `shove` and `give`: where single nets are laid on kept copper (`"reroute"`, or `"finish"` with
  `nets`) the unlocked tracks of the other nets shift aside where a connection needs the room
  (`shove`, default true there), and if a connection still finds no way, those within 8 mm of
  its pins give way and are routed again (`give`, default 8 there, in steps of 2, 4, 8 mm; the
  try with the fewest open connections, then the fewest violations is written). `shove: false`
  and `give: 0` leave the other nets where they are; locked copper and plane nets never move.
  Elsewhere both are off and asking for them is refused. With `open_end_seconds` the tries of
  `give` run without the open end, and the one kept goes on open ended for the budget.
- `move_parts`: default true with `"fresh"` and `"keep_locked"`, false with `"finish"` and
  `"reroute"`. The geometry generations may shift parts next to clearance problems and open
  connections, while the routed copper follows like rubber bands (as `--move-parts` of the
  CLI). Locked footprints, connectors, modules, mounting holes, buttons and test points stay,
  and so do the references in `fixed`. The output has the footprints at their new positions,
  and `moved` lists them with their shift in mm. `move_parts` does not go together with `shove`:
  single nets on kept copper with `move_parts: true` need `shove: false`, otherwise the route is
  refused.
- `air`: 0.03 mm in an open ended run, but 0 with `"reroute"` and with `"finish"` and `nets`,
  where it would move tracks the new ones do not touch.
- `placements` and `clear_corridors` place parts before a whole board is routed: not with
  `"finish"` or `"reroute"`.

Result fields: `board`, `cleared_corridors`, `placement` (with `placements`: `seed`, `moved`,
`tried`), `moved`, `input`, `output`, `seed` (the seed the search ran with), `seconds` (wall time
of the whole route, from reading the board to the checked result, the same number as in the
CLI's report), `search_seconds` (the part up to the end of the search), `threads`,
`alternatives` (with `open_end_seconds`: `path`, `routed`, `connections`, `vias`, `length_mm`,
`clearance_spots`), `frames`, `connections`, `routed`,
`unrouted` (list of `net: pad -> pad`), `vias`, `length_mm`, `items_written`, `skipped_nets`,
`violations_total`, `violations_by_kind`, `violations` (first `max_violations`, default 50:
`what`, `x`, `y`, `actual_mm`, `required_mm`, `nets` [routed net, other net or null]; the kinds
are those of the CLI's `clearance check`, and for `width` and `current` the two values are
widths; `hole_to_hole` holds between all drilled holes, of the same net too, and `width` also
names a track below the project's `min_track_width` or `min_connection`, which KiCad reports
as `track_width` or `connection_width` whatever the net class allows),
`portfolio` (per run: `generation`, `seed`, `fanout_first`, `routed`, `connections`, `vias`,
`length_mm`, `violations`), `pairs` (the wide tracks, with `pairs` `"band"`), `coupling` (every recognised pair, whatever
the settings: `p`, `n`, `coupled_mm` within width + gap + 0.1 mm of the partner on the same layer,
`length_mm` of both nets, `vias_p`, `vias_n`, and with `pairs` `"pull"` or `"band"` `uncoupled`: the
longest stretches of 0.5 mm and more that do not run beside the partner, `net`, `layer`, `x`, `y`,
`length_mm`, `why` one of "partner on another layer", "partner farther away" (a breakout to the
pins, or a way of its own), "something between"), `notes` (hints: an argument in an old spelling and what to write now,
pairs routed with `pairs` `"off"`, or a pair that was asked to couple and runs coupled for less
than 60 % of its length, or a `gb:` pour whose fill will not get to a pad that lies on it, judged
by the fill model), `length_tuning` (every net the tuning aimed at: `net`, `before_mm`,
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

### Old spellings of route

Arguments in the spelling of an earlier version still work. They are mapped onto the options of
today, the result is the same, and `notes` of the result has one line `OLD is now NEW` for each.
The schema of the tool no longer lists them.

| Earlier | Now |
| --- | --- |
| `pairs: true` | `pairs: "band"` |
| `pairs: false`, or `pairs` left out | `pairs: "off"`, the default |
| `pair_pull: W` without `pairs: true` | `pairs: "pull"`, `pair_pull: W` (`"pull"` alone pulls with 1) |
| `pairs: true`, `pair_pull: W` | `pairs: "band"`, `pair_pull: W` |
| `keep_locked: true` | `scope: "keep_locked"` |
| `keep_locked: false` | `scope: "fresh"` |
| `keep_all: true`, also with `keep_locked` | `scope: "finish"` |
| `keep_all: true`, `nets` | `scope: "finish"`, `nets` |
| `keep_all: true`, `nets`, `reroute: true`, also with `keep_locked` | `scope: "reroute"`, `nets` |
| `keep_all: false`, `reroute: false` | nothing: the scope of the board stays |

`reroute: true` without `keep_all: true` had no meaning and was refused; it still is, and the
message names `scope` `"reroute"`. `keep_all` and `reroute` without `nets` was refused too:
`scope` `"reroute"` needs `nets`. An old switch next to `scope` is refused. `keep_locked` of
`load_board` is an argument of that tool and stays.

### Choosing opt-in options

Many options of `route` and `place_parts` are off by default because none of them wins on every
board: each helps one kind of board and costs on another. The full table with the measurements is
[Choosing the opt-in options](guide/options.md); this is what an agent needs to pick them.

- **Seeds first.** Results scatter strongly from seed to seed (10 to 20 % in vias, 0 or 14
  clearance errors on the same board). Judge an option over the same 2 or 3 seeds, by the mean
  and the worst run, with `drc`. Placement seeds (`place_parts` `seed`) scatter even more than
  route seeds and are often worth more than any hint.
- **Look before choosing.** `board_patterns` shows rows, decaps, buses and pairs; `track_corridors`
  shows the bands between chips and what sits in them; `load_board` lists the differential pairs.
- **Several chips with buses** (MCU and PHY, SPI, parallel buses): `patterns` `groups`. On a tight
  board add `clear_corridors`, or move the parts of `track_corridors` yourself with `place_parts`
  (`only`, `corridors`, `leash` 1.5). Measured: up to 25 % fewer vias, 18 % on the tightest board
  with the corridors cleared.
- **FPGA or BGA with pairs and buses**: `patterns` `pairs,groups` (best on Arty).
- **DDR fan-out**: `patterns` `groups-short`; for pairs that change layers `pairs` `"band"` with
  `via_pairs`. Leave `breakouts` off there.
- **Placement fixed and the board already routes cleanly**: leave the hints off (they added
  clearance errors on such a board); `decaps` at most. Spend the time on seeds and
  `open_end_seconds` instead.
- **Placement from scratch**: `place_parts` with `semantic`, `corridors` and `escape_halo`, two or
  three seeds, route each without hints, keep the best.
- **Differential pairs**: `pairs` `"pull"` when they should run together and there is room;
  `pairs` `"band"` (one wide track) only for pairs with a rule or class of their own and room
  around them, it costs connections on dense boards; `pair_all` for pairs in the default class.
- `shares` usually costs vias; `rows` acts only in the open end (`open_end_seconds`).
- Keep an option per board only when it beat the plain run; it is no default for the next board.

### angles_45

Turns the unlocked tracks of a routed `.kicad_pcb` into conventional 45 degree tracks without
routing again, like `gummiband angles45`: the same step as `route` with `angles: "45"`, on copper
that is already there. Vias, track ends, pads and locked tracks stay; every new piece keeps the
clearances, and a piece with no room on the grid keeps its angle. `nets` limits it to those
nets. The output goes to `<input>.45.kicad_pcb` by default, with the `.kicad_pro` and
`.kicad_dru` copied next to it for `drc`.

```json
{"name": "angles_45", "arguments": {"path": "/abs/path/ledctl.routed.kicad_pcb"}}
```

### Good practice: power_plan, power_copper, current_check, reference_check, ground_pour, area_report

The router does the heavy lifting; around it go the rules of the craft a careful layouter
applies: plan the power copper before routing, check the current carrying capacity after it,
check that fast signals run over unbroken reference copper, pour ground and stitch it. These five tools and the prompt `good_practice` (below) cover that,
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
The LED controller benchmark it turned 36 clearance violations into 2541). Nets that already
have a plane or a pour on a routing layer are left to `current_check`. `board_notes` cover the
ground plane (missing, or on which layer), plane layers without a zone and, on boards with
differential pairs, routing layers whose nearest plane is not ground. `dru_rules` holds the
widths as KiCad custom rules (`track_width` min and opt, courtyards of fine pitch parts exempt)
to append to the project's `.kicad_dru`: KiCad's DRC then enforces them, and Gummiband reads
them on the next `load_board`. `apply: true` stores the widths as net options (`width` and
`min_width`, see `set_net_options`) for the following routes.

```json
{"name": "power_plan", "arguments": {"board": "b1", "apply": true, "currents": [
  {"net": "*VBUS_RAW", "amps": 4.1}, {"net": "VBUS_F", "amps": 4.1},
  {"net": "*/SW_?", "amps": 8}, {"net": "*OUT?_5V", "amps": 4, "short_amps": 7}]}}
```

#### Planes and wide supply nets

Plan the layers before `power_plan`, so that its advice already sees the planes:

1. Decide which inner layers are planes and give them type `power` with `set_layer_type`; the
   others stay `signal`. A GND plane next to every signal layer that carries fast signals or
   differential pairs, the supplies on the rest. KiCad files often mark every layer `signal`
   (the Jetson baseboard even has `jumper` on In4), so look at the `plane` flags `load_board`
   reports.
2. One `add_zone` per plane net. Several supplies can share one power layer as a split plane:
   one zone per net, side by side with a gap of at least the clearance (0.5 mm is common); a via
   counts only where it lands in the copper of its own net (see the overlap rule below). On the
   Jetson baseboard: In1 and In4 GND, In2 +3V3, In6 VCC on the left and +5V on the right.
3. Then `power_plan` with the currents: a net with a plane gets the vias per pin instead of a
   width, a net without one its width and the advice below.

A supply net that is too wide for a track on an inner layer (VBUS or VIN from a connector, a
battery net, a motor supply) needs a plan of its own. `power_plan` sizes the track for the outer
layers; an inner layer needs about twice the width for the same current (IPC-2221 k 0.024
instead of 0.048), and the router takes a net with its width onto any routing layer it uses. A
1.5 mm VBUS track across an inner signal layer takes the room of four or five signal tracks, and a
long one cuts the layer in two halves that buses no longer cross (on the Jetson baseboard the wide
VBUS nets ran over the inner layers, next to the buses that were already the bottleneck). The
router has no per net layer choice, so leave it no long wide track to route:

- A plane or a share of a split plane on a power layer under the path from the source to the
  loads, best for a supply that spans the board. `power_copper` gives the large pads their via
  fields into it.
- A pour on an outer layer for a short path (connector to fuse to eFuse): `power_copper` with
  `from` and `to` up to `max_span` (12 mm) apart, `add_zone` on F.Cu or B.Cu with a priority
  above the GND pour for a longer one.
- A track pre-routed on the outer layers with `add_locked_copper`, `power_plan`'s width and
  `vias_per_transition` vias wherever it changes layer, then `route` (the working copy keeps the
  locked copper: `scope` `"keep_locked"`).

What is left for the router are short fan-outs from the pins into the plane or pour. After the
route `render` the inner layers and look for wide tracks; `current_check` confirms the paths.

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
- At a crowded power part the field may find too little room. Four options widen it, each
  named in the warning when it would help: `via_rings` (default 4) lets the field reach more
  rings out, `via_in_pad: true` puts vias into the pad itself first (a large pad such as an
  exposed pad or a drain tab; order the vias filled and capped), `vias_in_courtyards: true`
  lets vias sit in the courtyards of neighbouring parts (off their copper; KiCad does not object
  to that), and `via_margin` (default 0.15 mm beyond the clearance, so the pour reaches the via
  with some width) down to 0 packs the vias as tight as the rules allow. On the 4 port LED
  controller the four together found 25 of 26 vias where the defaults found 5.

- On a routed board (or one with any unlocked tracks and vias) `unlocked` decides what
  happens to the unlocked copper of other nets. `avoid` (the default) treats it as an obstacle
  like locked copper: the pours give way and the vias go where it leaves room, which on a
  finished board is often nowhere; a warning then points to `clear`. `clear` lays out the copper
  as on an unrouted board (only locked copper and the earlier paths are in the way) and removes
  the unlocked tracks and vias of other nets the new copper would touch, closer than their
  clearance (`cleared`, per net). Route `reroute_nets` (the cleared nets and the nets of the
  paths) next, with `scope: "reroute"` and `nets`: those nets are routed anew, everything
  else stays. Locked copper is never removed. Before this option the tool saw only locked
  copper and put vias on the routed tracks of other nets without a warning; a reroute from there
  ended on the LED controller with 175 shorts.

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

On the LED controller benchmark, buck A: the switch node pour (U4.5 to U4.9 into L1.1, 8 A)
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
  divided by the factor IPC-2221 asks for there. This is the measure of the LED controller
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

On the finished LED controller board the check finds the known 0.28 mm neck of `VBUS_IN` and
shows that `VBUS_F` and `VBUS_IN` each pass a single 0.4 mm via (about 1.6 A) on their 4.1 A path.

```json
{"name": "current_check", "arguments": {"path": "/abs/ledctl.routed.kicad_pcb", "paths": [
  {"name": "USB-C to fuse", "net": "/Power/VBUS_RAW", "amps": 4.1,
   "from": ["J1.A4+J1.B9", "J1.A9+J1.B4"], "to": ["F1.1"]},
  {"name": "fuse to shunt", "net": "VBUS_F", "amps": 4.1, "from": ["F1.2"], "to": ["R110.*"]},
  {"name": "buck A switch node", "net": "/Power/SW_A", "amps": 8,
   "from": ["U4.5+U4.6+U4.7+U4.8+U4.9"], "to": ["L1.1"]}]}}
```

A flag is fixed by widening the net (`set_net_options` width, then `route` with
`scope: "reroute"` and `nets`), a pour over the neck (`add_zone`), more vias or locked copper, then another
check. For a pour or via field from `power_copper` there are two ways:

- On the routed board: `power_copper` with `unlocked: "clear"`, then `route` with
  `scope: "reroute"` and `nets` set to the returned `reroute_nets`. Only the nets in the way are routed
  again. Without `clear` the new copper gives way to the routed tracks and often finds no room.
- On the unrouted board before the next full route: `power_copper`, then `route` (with
  `scope` `"keep_locked"`, the default of the working copy). Next to fine pitch pins a via field before routing can wall in the neighbours:
  at the eFuse U6 of the LED controller the VBUS_P vias in front of pins 7 to 10 left 6
  connections open in every run. Compare such a run with one without the via field over the same
  seeds before keeping it.

#### reference_check

After routing (and after `ground_pour` on two layer boards, where the pour is the reference).
The return current of a signal flows in the plane right under its track. Where that plane has no
copper (a slot, the edge of a zone, antipads of a via row that merged into one hole) or where the
track passes from the zone of one net into the zone of another (a split plane, such as several
supply zones on one inner layer), the return current detours: loop inductance, ringing, EMI, and
under a differential pair lost common mode rejection. KiCad's DRC does not look at this.

The tool takes the copper layers next to each routing layer in the stackup that carry zones as
its reference layers (F.Cu over In1.Cu, B.Cu over In2.Cu on four layers; on two layers the
other side's pour), rasters their filled zones at 0.05 mm and walks the centre line of every
track of the checked nets over them:

- `gap`: a stretch with no copper on any reference layer. The antipad of the track's own via or
  through hole pin (where it changes layer) does not count. Gaps shorter than `min_gap`
  (0.5 mm), mostly a single antipad of a neighbouring via, are counted in `short_gaps` only.
- `split`: the zone net under the track changes on a reference layer, while no other reference
  layer (an inner layer between two planes) runs on unbroken underneath.

By default every net with tracks is checked except the ground net, nets with a plane and nets of
clearly wide classes (power); `nets` names others (globs allowed). Zones without fill are refilled
with kicad-cli first; without kicad-cli their outline counts, which hides the antipads (a note
says so). The result lists the findings (splits first, then gaps longest first), the nets with
findings worst first, and per routing layer the reference layers with their zone nets.

On the LED controller board (fills as saved): F.Cu over the GND plane on In1.Cu has no finding;
B.Cu runs over In2.Cu with the zones of +5V_A, +5V_B, VBUS_P and VBUS_IN, and 11 nets cross a
border there (the LED data lines at y 130 between +5V_B and +5V_A, USB_DN between VBUS_P and
+5V_A). Fix a split by routing the track over one zone or on the layer next to ground, or with a
stitching capacitor between the two nets next to the crossing; a gap by moving the track, by
moving vias apart so their antipads do not merge, or by closing the slot.

```json
{"name": "reference_check", "arguments": {"path": "/abs/ledctl.routed.kicad_pcb",
  "nets": ["USB_D?", "LED_D*"]}}
```

#### ground_pour

After routing: zones of the ground net (default `GND`, or the net with the most pins whose name
holds GND) over the board outline on the outer layers (or `layers`), with `priority` 0 so power
pours with a higher priority win, and locked stitching vias. A layer that already has a zone of
the net is left as it is (the small `gb:` pours of `power_copper` do not count); `warnings`
name zones of other nets whose priority does not beat the pour.

**Stitching** (`stitch`):

- `islands` (default): a pour over a routed layer falls into pieces, and a pad on a piece that
  nothing joins to the rest is not connected. The router's fill model (see `area_report`)
  finds these islands and, for each, the spot where one via joins it to the area on another
  layer, clear of other nets on every layer and with the most room. That via is set, the model
  runs again on the board with it, and so on until no island has a site left. A via is only
  set where it joins something: a pour that hangs together gets none.
- `grid`: a row along the edge (`edge_pitch` 3 mm, `edge_inset` 1 mm from the outline) and a
  grid (`pitch` 5 mm; a blocked grid spot tries eight neighbours a quarter pitch away, and
  spots next to an existing via of the net are skipped). A via goes wherever it keeps the
  clearance plus 0.3 mm to copper of other nets on every layer, the hole to hole distance, the
  edge clearance, out of rule areas that forbid vias, of other nets' zones on routing layers,
  of courtyards (`avoid_courtyards`) and off the pads of the net. This is for return paths and
  shielding, not for connecting pads; a via the fill does not reach shows up in `drc` as
  dangling, unless an inner plane of the net holds it.
- `both`: the grid first, then the islands that are left. `pitch` or `edge_pitch` given without
  `stitch` mean `both`; with `stitch: islands` or `none` they are an error.
- `none`: the zones only.

**Several nets** (`nets: ["GND", "GNDR"]` instead of `net`): the board is shared by nearest pad.
Every spot goes to the net whose pad is nearest, each net gets a zone per layer around its own
pads (several where its pads lie in separate groups), and the zones of two nets never overlap:
between them a corridor of about 0.5 mm stays free. A pad that lies alone among the pads of
another net gets no zone, since a zone around one pad joins nothing and would cut the other
pour apart; `warnings` name it. The outlines follow the bounding box of the board (KiCad clips
the fill at the edge).

**Fill rules** go into the zone as given: `pad_connection` (`thermal`, `solid`, `thermal_tht`;
`solid` clears KiCad's `starved_thermal` where the spokes find no room, at the cost of harder
soldering for small passives), `thermal_spoke_width` (0.3), `thermal_gap` (0.3),
`zone_clearance` (0.2) and `zone_min_width` (0.2 mm). A smaller minimum width lets the fill
through tighter gaps between tracks, down to what the manufacturer makes.

**Via in pad** (`via_in_pad: true`): a pad of the net that nothing else reaches and that is large
enough to hold the whole via, typically the exposed pad of a QFN boxed in by its pins, gets a
via in the pad where it lands in the area on another layer. Off by default: a via in a pad
takes solder and is a decision for the assembly. Without it such a pad is reported with that
advice.

**The result** says per net (`nets`) what the pour reaches on the board as written, judged by the
fill model: `summary` in one sentence, `island_vias`, `pad_vias`, `grid_vias`, `pads`,
`reached`, and `unreached` with `reason` and `why` in plain words including what would help
(the via that `stitch: islands` would set, `via_in_pad`, a track, other fill rules), and the
`doubtful` pads whose answer hangs on less than a raster cell of the model. This is the
router's judgement, not KiCad's: `drc` on the result is the judge.

The output `<input>.gnd.kicad_pcb` is loaded as a new board. The zones are filled with kicad-cli
before the file is handed back (a warning says so if that fails), so `render` and KiCad show the
pours right away.

```json
{"name": "ground_pour", "arguments": {"path": "/abs/ledctl.routed.kicad_pcb"}}
{"name": "ground_pour", "arguments": {"path": "/abs/board.routed.kicad_pcb",
  "nets": ["GND", "GNDR"], "stitch": "both", "pitch": 5, "zone_min_width": 0.15}}
```

#### area_report

What the copper fill of the nets with a zone reaches, on the copper the file has. A zone is an
outline; KiCad computes the fill later, and what the fill joins depends on every track and via
of the other nets. `area_report` computes that fill with the router's own model (no KiCad
needed) and reads it for you. Per net:

- `summary`: the finding in one sentence. "The area of GND (812.4 mm2 on F.Cu, B.Cu) reaches 48
  of 52 pads; 4 are not reached: 3 on islands, 1 without a spoke."
- `layers`: pieces of fill per layer, how many belong to the area (`of_the_area`) and how many
  hold nothing of the net (`floating`: KiCad removes those where the zone says so, which is
  always unless the zone is set otherwise). `isolated` of them stay all the same: KiCad takes
  no piece out of a zone in which every piece holds nothing, and warns of isolated copper for
  each. A pour on a layer without a pad or via of the net is such a zone.
- `pads`, `reached` (`by_fill`: touched by the fill itself, `by_copper`: joined through tracks,
  vias or other pads of the net), `joins`: the pads the area joins. `groups_apart`: the groups
  of pads that do not hang on the area, and `expected_unconnected`: the unconnected items
  KiCad's DRC would count for them.
- `unreached`: each pad with `reason` and the same in plain words (`why`), and `pad_via` where
  the pad holds a whole via and one in the pad would join it (the exposed pad of a QFN):
  `island` (on a piece of fill that copper of other nets cuts off from the area), `walled` (in
  the outline, but no room for fill next to it), `no_spoke` (a thermal relief none of whose
  spokes ends in fill), `outside` (outside the outline), `other_layer` (the net has no area on
  the pad's layer: it needs a via or a track).
- `islands`: pads, layers, `filled` mm², `widest` spot, and `via`: where one via joins the
  island to the area (`at`, `from` and `to` layer, `spare` room; `second`: the piece it lands on
  is itself apart and needs this second via), or `no_via` with the reason (narrower than a via,
  blocked on some layer under every spot, nothing of the area above or below).
- `site_distance`: mean and largest straight distance from a pad to the nearest spot where a
  via of the net stands clear on every layer and lands in the area, and the `farthest` pad. For
  a plane this is how far the pins are from their via.
- `doubtful`: the pads whose answer changes when every clearance of the fill moves by half a
  raster cell (`reached` now, `with_less`, `with_more`; `spoke`: reached in none of the three,
  but a spoke of the thermal relief stays in one and none in another, so whether the pad hangs
  on fill at all is within the raster). The model works on a raster (`step`, 0.04 to 0.05 mm by
  default); a strip of fill within one cell of the minimum width can come out either way, and
  so can a spoke that ends within a cell of the border of the fill. These are the pads that
  hang on such a spot. Treat them as not reached until KiCad's fill says otherwise.
- `against_file`: where the file holds a fill (KiCad's, as of its last refill), the model
  against it per layer: mm² in both, in one only, and the pads only one of them touches.

"The area" of a net is the group of fill and copper that joins the most fill among those that
join at least two pads. Where no piece joins two pads (a plane before its first via, a pour cut
off from all pads but one), the summary says so ("The fill of GND joins no two pads") and the
largest piece is taken, so the pads are measured against the copper they are meant to reach.

With `nets` and `pour` (layers) the nets given get an area for the report where they have none
yet (one net over the board outline, several nets each around its own pads, as `ground_pour`
lays them), with `zone_clearance`, `zone_min_width`, `thermal_gap`,
`thermal_spoke_width` and `pad_connection` as their settings (defaults 0.2, 0.2, 0.3, 0.3,
`thermal`): what a pour would reach, before one is written. The file is never changed.

The model is not KiCad's filler. It was compared with KiCad's fill of the LED controller planes
(0.08 % of the area differs, the same pads are touched) and with KiCad 10 on routed bench boards
with a pour (`bench/areas-kicad.sh`): a strip of just the minimum width passes, pads in a row
whose thermal gaps meet are joined by their spokes, and a pour that nothing holds stays as
isolated copper, as in KiCad. A pad that hangs on a strip or a spoke within a raster cell is
what `doubtful` is for. `drc` stays the judge.

```json
{"name": "area_report", "arguments": {"path": "/abs/ledctl.routed.gnd.kicad_pcb"}}
{"name": "area_report", "arguments": {"path": "/abs/board.routed.kicad_pcb",
  "nets": ["GND"], "pour": ["F.Cu", "B.Cu"]}}
```

#### The prompt good_practice

The server offers one MCP prompt, `good_practice` (arguments `path`, optional `currents` and
`notes`). It asks the agent to take the board through the whole sequence without asking back
unless the design intent changes: look at the board and its power paths, settle the currents,
plan with `power_plan` (planes, pours, datasheet layouts as locked copper, `dru_rules` into the
project), place and route, `current_check` every path and fix the flags, `ground_pour`,
`reference_check` for the fast signals and pairs, `drc` and `render`, and report a table of the current paths. In Claude Code it appears as
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
{"name": "drc", "arguments": {"path": "/abs/path/ledctl.routed.kicad_pcb", "max_items": 5}}
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
{"name": "render", "arguments": {"path": "/abs/path/ledctl.routed.kicad_pcb",
  "region": {"x0": 140, "y0": 120, "x1": 160, "y1": 140}, "width_px": 800}}
```

### guide

Returns the documentation of the server as Markdown, so an agent can look up how to use a feature
or the whole workflow while it works. Without `topic` it returns `workflow` (the section
[Working with the tools](#working-with-the-tools-by-hand-automatic-both): what to do by hand,
what the automatic tools do with it, the order of work and fine-tuning) followed by the list of
topics. Topics: `workflow`, every tool name (`route`, `place_parts`, `lock`, ...), the other
sections of this reference (`good_practice`, `planes_and_wide_supply_nets`,
`choosing_opt_in_options`, `manufacturer_limits`, `open_points`, ...), `concepts` (how the router
works: topology, rules, necking, vias, priorities) and `options` (the opt-in options with
measurements). An unknown topic returns the list. The text is built into the server, so it matches
its version.

```json
{"name": "guide", "arguments": {"topic": "route"}}
```

## Try it without a client

Each line on stdin is one message; wait for an answer before sending the next request
(stdin closing ends the server).

```bash
RAYON_NUM_THREADS=4 target/release/gummiband-mcp <<'EOF'
{"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-06-18","capabilities":{},"clientInfo":{"name":"shell","version":"0"}}}
{"jsonrpc":"2.0","method":"notifications/initialized"}
{"jsonrpc":"2.0","id":2,"method":"tools/list"}
{"jsonrpc":"2.0","id":3,"method":"tools/call","params":{"name":"load_board","arguments":{"path":"bench/work/ledctl/ledctl.kicad_pcb"}}}
EOF
```

Shortened session on the LED controller benchmark (4 threads):

```text
>> initialize {"protocolVersion":"2025-06-18",...}
<< {"protocolVersion":"2025-06-18","capabilities":{"tools":{}},"serverInfo":{"name":"gummiband-mcp",...},"instructions":"..."}
>> tools/call load_board {"path":".../ledctl.kicad_pcb"}
<< Loaded ledctl.kicad_pcb as board b1: 100.0 x 60.0 mm, 4 copper layers (F.Cu, In1.Cu plane,
   In2.Cu plane, B.Cu), 525 pins, 111 nets (92 to route, 414 connections), 11 net classes, 10 custom rules.
>> tools/call list_nets {"board":"b1","filter":"gnd"}
<< {"count":1,"nets":[{"class":"GND","name":"GND","pins":150,"plane":true,"priority":4,"skip":false}]}
>> tools/call route {"board":"b1","runs":2,"generations":0,"output":".../ledctl.routed.kicad_pcb"}
<< Routed 346/348 connections (2 unrouted), 338 vias, 2458 mm of track, 29 clearance violations
   (Gummiband check), 44.7 s on 4 threads. Wrote 3306 items to .../ledctl.routed.kicad_pcb.
>> tools/call drc {"path":".../ledctl.routed.kicad_pcb"}
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
- The open end can only be given a time budget (`open_end_seconds`); the stop by the user that
  `--until-stop` has needs the cancellation below.
- `create_board` reads rectangular outlines only, places every part on the front side and
  without rotation, and does not read footprint libraries of other types than KiCad (`.pretty`).
- `power_copper` shapes a pour as a convex hull cut by straight lines (plus a narrow stub out of
  a long narrow pad, see above): obstacles between the terminals cost more copper than needed,
  and a terminal behind another net's pad is lost (a warning says so). It does not move parts; the via fields sit on rings around the pad.
- `route` has no per net layer choice: a wide net can only be kept off inner signal layers by a
  plane, a pour or locked copper (see Planes and wide supply nets). Zones can be added and
  removed, not changed in place.
- `place_silk` moves only reference texts; footprint silk over other parts' pads stays (KiCad
  clips silk at solder mask openings when it plots). Its text sizes are estimates.
- Courtyard clearance rules (`set_rules`) relax Gummiband's own check, not the router, which
  keeps the class clearance in those courtyards.
- `current_check` measures on a raster (0.05 mm, coarser on very large nets); it does not weigh
  the length of a neck, and the cross-section is skipped for nets of more than three million
  copper cells. `ground_pour` and `area_report` judge the fill by the router's own model, which has not been
  compared with KiCad's filler at strips of about the minimum width (`doubtful` names the pads
  that hang on such a strip); the outlines of a pour of several nets follow the bounding box
  of the board, not a board outline that is not a rectangle, and a zone outline with a hole
  (one net's pads enclosed by another's) is written with a slit, which KiCad has not been
  seen to read yet.
- Progress notifications during long routes, and cancellation (the core has no cancel hook).
- `route` returns Gummiband's own clearance check; KiCad's verdict needs a `drc` call.
