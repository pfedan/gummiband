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
| `load_board` | `path`, `keep_locked?` | board id, layers (plane flag), outline box and size, pins, nets, routable nets, connections, planes, net classes (priority, width, clearance, via), custom rule count, custom clearance net pairs, kept net options |
| `list_nets` | `board`, `filter?`, `include_single?` | nets with class, pin count, plane flag, effective priority, skip flag |
| `part_info` | `board`, `reference` | pads (name, net, position, size, layers), courtyard box, neighbouring footprints within 3 mm |
| `move_part` | `board`, `reference`, `x`, `y`, `rotation?` | id and path of the working copy with the footprint at its new position, `items_added`, `warnings` |
| `place_parts` | `board`, `only?`, `scope?`, `fixed?`, `seed?`, `starts?`, `gap?`, `halo?` | id and path of the working copy, the new poses, decoupling assignment, crossings, smallest courtyard distance, pins off their plane, `warnings` |
| `add_zone` | `board`, `net`, `layer`, `points`, `priority?` | id and path of the working copy with the new zone, `items_added`, `warnings` |
| `add_locked_copper` | `board`, `tracks? [{net, layer, width?, points}]`, `vias? [{net, x, y, diameter?, drill?}]` | id and path of the working copy `<input>.prerouted.kicad_pcb` with the copper added as locked items, `items_added`, `warnings` |
| `set_net_options` | `board`, `net`, `priority?`, `skip?`, `reset?` | the net's options and effective priority |
| `route` | `board`, `runs?`, `seed?`, `generations?`, `geometry_generations?`, `keep_locked?`, `keep_all?`, `reroute?`, `output?`, `nets?`, `via_pitch?`, `via_cost?`, `layer_dirs?`, `pair_pull?`, `pairs?`, `pair_skew?`, `pair_zdiff?`, `max_violations?` | the fields of `gummiband route --report` plus session fields (see below): per net numbers, the nets in the way of each unrouted connection, with the experimental `pairs` one entry per differential pair |
| `drc` | `path`, `max_items?` | KiCad DRC summary |
| `render` | `path`, `layers?`, `region? {x0,y0,x1,y1}`, `width_px?`, `background?`, `output?` | PNG (image content) and its path, region, pixels per mm |

### load_board

Reads the `.kicad_pcb` with its `.kicad_pro` (net classes) and `.kicad_dru` (custom rules) of
the same base name. `keep_locked` (default false) keeps locked tracks and vias as fixed copper
whose ends join their nets; by default everything is routed from scratch, like
`gummiband route`. `connections` is counted like `gummiband info` (pins - 1 per net with 2 or
more pins); `route` counts plane nets per fan-out, so its total is lower on boards with planes.

```json
{"name": "load_board", "arguments": {"path": "bench/work/lichtschnitzel/lichtschnitzel.kicad_pcb"}}
```

### list_nets

`filter` is a case insensitive substring of the net or class name. Nets with fewer than two
pins are left out unless `include_single` is true.

```json
{"name": "list_nets", "arguments": {"board": "b1", "filter": "gnd"}}
```

### part_info

Everything an agent needs to plan copper around one footprint: its pads with name, net,
position, size and layers, the courtyard box `[x0, y0, x1, y1]` and the references of the
footprints whose courtyard lies within 3 mm.

```json
{"name": "part_info", "arguments": {"board": "b1", "reference": "U6"}}
```

### move_part

Preliminary. Moves a footprint to a new position in mm (and rotation in degrees, unchanged if
left out) in the working copy `<input>.prerouted.kicad_pcb`; the input file stays untouched and
the returned board id is the one to route. Courtyards and overlaps are not checked, so run `drc`
or `render` on the result. Pads, their nets and the footprint's own copper move along.

```json
{"name": "move_part", "arguments": {"board": "b1", "reference": "C12", "x": 152.4, "y": 118.0, "rotation": 90}}
```

### place_parts

Gives parts a position from the netlist in the working copy, see
[`gummiband place`](guide/cli.md#gummiband-place). An agent with its own plan places the parts it
cares about with `move_part` and leaves the rest to this tool: `fixed` names what stays, `only` what
moves. `scope` is `unlocked` (the default: not locked and not mechanical), `all_unlocked` or
`outside`. Locked footprints never move. The tool places, it does not judge: `route` and `drc` tell
how good the result is, another `seed` gives another try.

```json
{"name": "place_parts", "arguments": {"board": "b1", "fixed": ["U1", "L1"], "seed": 2}}
{"name": "place_parts", "arguments": {"board": "b1", "only": ["C12", "C13", "R4"]}}
```

### add_zone

Preliminary, tested on small boards only. Adds a copper zone of `net` on `layer` with the
outline `points` (`[x, y]` in mm, at least three) and `priority` (default 0) to the working copy.
On a KiCad power layer the zone becomes a plane the router connects to with vias; on a routing
layer it is kept like any other zone. Gummiband does not fill zones: KiCad does it when it fills
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

The result has `items_added` and `warnings`: problems found at the given points of the new
copper, such as a point off the board, a point touching copper of another net, or a track on a
plane layer. Only the points are checked, not the track between them, so a `render` of the
region and a `drc` after routing remain the real check.

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
all unlocked copper). `reset: true` drops the net's options before applying the others.

```json
{"name": "set_net_options", "arguments": {"board": "b1", "net": "/MCU/SDA", "priority": -10}}
{"name": "set_net_options", "arguments": {"board": "b1", "net": "VBUS_IN", "skip": true}}
```

### route

Runs `gummiband_core::route_board` on the loaded board with the net options applied and writes
the routed board (existing unlocked tracks and vias replaced; with `keep_locked` the locked ones
stay). Defaults match the CLI: `runs` 14 (a constant, so results do not depend on the machine), `generations` 3,
`geometry_generations` 0, `via_pitch` 0.5 mm, `via_cost` 3.0, `layer_dirs` 0.5 (preferred
directions per layer for every third run, 0 = off), `pair_pull` 0 (experimental pull between the
nets of a differential pair), `pairs` false (experimental: a pair as one wide track that is split
in two, see the CLI guide), `pair_skew` 0.1 mm, `pair_zdiff` none. `seed` picks the search seed
(default fixed): the same seed gives the same result, another one a different try. `keep_all`
keeps every track and via of the input and adds to it, like `--keep-all` on the command line;
with `nets` only those nets are routed and everything else stays. Add `reroute` to replace
connected nets: the unlocked copper of `nets` goes first, then they are routed anew. `output` defaults to
`<input>.routed.kicad_pcb` next to the input; the `.kicad_pro` and `.kicad_dru` are copied next
to the output if missing, so `drc` and `load_board` work on it. `nets` routes only the listed
nets for this call.

Result fields: `board`, `input`, `output`, `seconds`, `threads`, `connections`, `routed`,
`unrouted` (list of `net: pad -> pad`), `vias`, `length_mm`, `items_written`, `skipped_nets`,
`violations_total`, `violations_by_kind`, `violations` (first `max_violations`, default 50:
`what`, `x`, `y`, `actual_mm`, `required_mm`, `nets` [routed net, other net or null]),
`portfolio` (per run: `generation`, `seed`, `fanout_first`, `routed`, `connections`, `vias`,
`length_mm`, `violations`), `pairs` (with `pairs`), `nets` (per routed net, longest first: `net`,
`length_mm`, `vias`, `min_width_mm`, `max_width_mm`), `blocked` (per unrouted connection the
`nets` that run in its way, a neighbourhood estimate rather than an exact analysis, meant for
choosing what to prioritize, skip or move), `copied`.

```json
{"name": "route", "arguments": {"board": "b1", "runs": 2, "generations": 0}}
```

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
region. The PNG goes to the server's temp folder unless `output` is given.

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

- `move_part` and `add_zone` are new and tested on small boards only; `move_part` checks no
  courtyards, and a zone is not filled until KiCad fills it.
- Not reachable through MCP yet: creating a board, `--move-parts`, `--shove` and `--give`, and per net widths and priorities
  from files (`--net-widths`, `--net-priorities`; priorities go through `set_net_options`).
- Progress notifications during long routes, and cancellation (the core has no cancel hook).
- `route` returns Gummiband's own clearance check; KiCad's verdict needs a `drc` call.
