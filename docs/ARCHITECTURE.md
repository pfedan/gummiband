# Gummiband architecture

Gummiband is a topological ("rubber band") PCB autorouter for KiCad. Routes are planned as
topology first (which side of which obstacle a connection passes) and only then turned into
geometry: tight any-angle tracks that wrap around obstacles with arcs, like rubber bands pulled
taut between pins. The core is written in Rust and uses every core of the machine; KiCad talks to
it through a thin plugin.

The design is our own. It follows ideas that are public in the literature (constrained Delaunay
triangulation as routing substrate, topological routing with sketch/"rubber band" representation,
negotiated congestion), not any existing product's code.

## Goals

- Any-angle tracks and arcs instead of a 45 degree grid; shortest paths around obstacles.
- Manual work stays in charge: everything locked in KiCad (pre-routed power lanes, scripted
  copper, critical pairs) is kept as an obstacle and never touched.
- Nets are routed by priority (net class priority, overridable per net).
- Use all cores: independent work (portfolio search over net orders, per layer realization,
  per net checks) runs in parallel with rayon.
- Deterministic: the same input and seed give the same board.
- First target: two signal layers (F.Cu, B.Cu); inner plane layers are only crossed by vias.

## Overview

```
 KiCad (.kicad_pcb + .kicad_pro)
        |  gummiband-kicad: s-expression reader
        v
 Board model (gummiband-core::board)
   layers, outline, pads, locked copper, keepouts, nets, net classes, rules
        |
        v
 Routing substrate (gummiband-core::topo)
   one constrained Delaunay triangulation of all obstacle points of all layers,
   plus free via sites; per layer: blocked triangles, edge capacities
        |
        v
 Topological router (gummiband-core::route)
   nets -> two pin connections (MST), A* over triangle edges with slot positions,
   planar by construction, vias at via sites, rip-up and reroute;
   portfolio of orderings in parallel, best result wins
        |
        v
 Geometric realization (gummiband-core::realize)
   per layer: order of routes on each edge -> portals -> funnel (taut string) ->
   arcs around the inflated obstacles -> tracks, arcs, vias
        |
        v
 Check (gummiband-core::check) and writer (gummiband-kicad)
   clearance check in the core, then new segments/arcs/vias into a copy of the board;
   kicad-cli pcb drc as the final judge
```

## Crates

| Crate | What |
| --- | --- |
| `gummiband-core` | geometry, board model, triangulation substrate, topological router, realization, checks. No I/O. |
| `gummiband-kicad` | KiCad 9/10 s-expression reader and writer, net class and rule import from `.kicad_pro`. |
| `gummiband-place` | placement: `auto` places any board from its netlist (`gummiband place`, MCP `place_parts`), the `gummiband-place` program makes the Lichtschnitzel outline variants. |
| `gummiband-cli` | `gummiband` binary: `info`, `place`, `route`, `svg`. The plugin calls this. |
| `plugin/` | KiCad action plugin (Python, KiCad IPC API): exports the board, runs the CLI, imports the result. |

Units: millimetres as `f64` in the core. The writer rounds to KiCad's nanometre grid.

## Board model

- `Layer`: copper layers by name and index; a routing layer set (default F.Cu and B.Cu).
- `Obstacle`: a polygon on a set of layers with a net (or none), a clearance class and a kind
  (pad, locked track, locked via, keepout, zone, edge). Pads, track capsules and circles are
  converted to polygons with a bounded error (`max_error`, default 5 um outward).
- `Pin`: a pad that a net must connect, with its position, layers and the polygon.
- `Net`: name, net class, pins, priority. Net class gives width, clearance, via size.
- Locked tracks and vias are obstacles of their net; their endpoints join the net's pin set, so a
  pre-routed lane counts as connected copper the router may attach to.
- Unlocked tracks and vias are removed (or kept with `--keep-unlocked`).

## Routing substrate

Each routing layer has its own constrained Delaunay triangulation (`topo::Mesh`) of the
obstacles on that layer. Edges run between real obstacles, so the room on an edge is the room
between the obstacles at its ends (a shared mesh for all layers turned the pads of one layer
into ghost points on the other, and capacities went wrong).

- Vertices: obstacle polygon vertices, board outline, a frame, and a coarse Steiner grid
  (2.5 mm) in empty regions that breaks up long sliver triangles.
- Per triangle: free, outside, or covered by an obstacle (a net's own pads are passable).
- Edge end demands: clearance plus half width from the obstacle at the vertex, and 1/sin(angle)
  for obstacle sides leaving the vertex at a sharp angle.
- Apex constraints: an obstacle or via at the vertex opposite an edge blocks the middle of the
  edge; each crossing knows on which side of that apex it passes.
- Via sites: a 0.4 mm grid of candidate points, plus via-in-pad points for plane nets. Placing a
  via splits the triangle that holds the site in every layer mesh; routes that already cross it
  are threaded through the three new triangles.
- Phantom points: Steiner vertices and via vertices whose via was ripped up keep a half
  clearance, so tracks never bunch up at a free vertex.

## Topological routing

A route on one layer is a sequence of edge crossings. Each crossing has a slot (its position
among the other routes on that edge). Inside a free triangle every route is a chord between two
boundary positions; a new chord must not interleave with an existing chord of another net, so
each layer stays planar by construction.

- Search: A* over (triangle, entry edge, slot, layer), loop free, with via transitions to sites
  inside the triangle or to existing via vertices, through-hole pads as free layer changes,
  plane fan-out goals (a via inside the plane outline or a nearby through-hole pad).
- Capacity: packing of all crossings on an edge with end demands, apex bounds, net class
  clearances and custom rule clearances between net pairs.
- Track width: net class width; near pins it may neck down to 0.25 mm if no minimum width rule
  forbids it (or inside the courtyards a rule exempts).
- Multi pin nets: Prim order, each new pin to the tree; overlapping pads of one net count as
  one pin.
- Rip-up and reroute of connections near a failed one, via-in-pad as a last resort for
  fan-outs.

### Negotiated routing

With `--negotiate N` (off by default) the router does not reserve its space firmly at first, in
the spirit of PathFinder (`Router::negotiate`). In the negotiation phase capacity and planarity
are soft: the search may overfill an edge or cross a track of another net, and pays for it in mm
of track. Cross lists and chords stay as they are, a route with a conflict is an ordinary route
in the state, so the search, the via splits and the rip-up need no second data structure.

- First pass: every connection takes its way in the plain order.
- Then up to N rounds: the connections in a conflict (on an edge that does not pack, or crossing
  another net in a triangle) are ripped up and routed again, one at a time and seeing all the
  others, the worst first. A connection whose conflict has gone by then stays. The prices of a
  conflict rise from round to round, and every edge in a conflict also gets a history price that
  stays, so that the connection with the fewest alternatives ends up keeping the place.
- Any illegal step costs at least the price of a crossing, whatever the overflow: otherwise a
  connection sits on a 0.01 mm overflow for ever instead of taking up the conflict with a
  neighbour.
- A phase ends when no conflict is left, after N rounds, or after five rounds without progress
  (a connection without any legal way, or a board with more connections than room on its layers,
  which no price can settle). What is still in a conflict is then ripped up, worst first, which
  leaves a legal topology. From there on `Router::run` goes on
  as always (via in pad, plane fallback, rip-up passes), with the history as a guide.
- In a conflict only the cheapest slot of an edge goes into the search: its many ways through a
  bundle would use up the expansion limit.
- A crossing that leaves an edge little room pays a small price throughout (`tight`, up to
  3 mm; free from 0.05 mm per gap): tracks take a tight gap only if they must. This is the soft
  form of a larger router margin, which forbids such gaps and loses the connections that need
  them.

A negotiation is a second search next to the plain one (`route_search`), not a replacement: it
completes the dense boards, but on a loose board a lucky order may leave the geometry more room,
and the better result counts, completion first. So the option costs the time of both searches.

Vias that a ripped connection leaves behind stay in the mesh as phantom vertices, and the wraps of
the rubber band must keep to their portals after a negotiation: its routes leave more slack in
the portals, where a wrap around a far pivot could swing past a neighbour.

## Importing existing copper

For finishing nets on a routed board (`--keep-all --nets ... --shove`) the unlocked tracks and
vias of the other nets are no obstacles but part of the topology (`gummiband-core::import`).

- Board side: `Board::soften` builds chains from the file's items: a chain is a run of one width
  between two nodes. Nodes are vias, branches, open ends and changes of width; ends in a pad stay
  fixed. Necks below a minimum width rule stay obstacles, a neck may be allowed only where it is.
- Mesh side: every node becomes a vertex (a via in all layer meshes, a branch point in its own
  layer) with the same splits as a placed via. Each chain is walked through its layer mesh into
  crossings and chords, in the order the tracks really have on each edge, and keeps the real
  crossing points. The router sees ordinary segments: capacity, chords and via threading work as
  always, and a new connection threads in between them.
- Geometry: imported tracks start exactly where they are, a routed track goes through portals
  between its real neighbours. Only where it runs into a bend of an imported track (or a new via
  splits a triangle) that stretch becomes the straight chord the topology assumes.
- Relaxation: imported copper sleeps. It wakes when copper that is awake violates its clearance
  and takes its neighbours along in the same way, so the wave ends where there is room. Sleeping
  copper is not written: the file keeps those items byte for byte, and changed ones are replaced
  (`import::soft_changes`).

## Geometry

1. Portals on every crossed edge (room for the neighbours and the edge end clearances) and the
   funnel algorithm give a first taut polyline.
2. Elastic optimizer (`relax`): every track is a chain of points under tension, obstacles,
   vias and other tracks push with their clearance, vias move as well and drag their tracks.
   All tracks relax in parallel (Jacobi rounds); the last quarter resolves clearances only.
   Experimental: with the `gpu` cargo feature and `GB_GPU=1` the rounds before the last quarter
   run as a compute shader (`relax_gpu.rs`, `relax_gpu.wgsl`, wgpu on Metal), see M4b in
   `MILESTONES.md`.
3. Taut pass: straight shortcuts wherever they keep all clearances (removes the waves).
4. Fillets: every corner is rounded with the largest arc that keeps all clearances; the output
   uses real KiCad arcs.
5. Clearance check in the core; the KiCad DRC is the final judge in the benchmark.

## Differential pairs

With `--pairs` (`gummiband_core::pairs`, experimental, off by default) a pair is routed as one wide
track and split afterwards.

1. Merge: the two nets become one net whose track is as wide as both tracks and their gap, with a
   via that holds both vias of the pair. The pads of an end merge into one pin (their hull, if it
   keeps its distance from foreign copper). A pair has one wide track, between its two ends
   farthest apart; other pins get single tracks in the last pass.
2. The normal search and geometry steps route that net.
3. Split: the centre line is offset to both sides, arcs stay arcs, folds on the inner side of tight
   bends are cut out. The tracks leave the pads straight and join the curves past the pad tips.
4. Polarity: if the P pad lies on different sides at the two ends, the pair changes sides through
   one of its vias (vias in line instead of side by side). Without a via the pair is routed as two
   single nets.
5. Repair: clearance problems within 6 mm of an end take the tracks and vias away up to the last
   problem; the normal router connects pad and track end. If that finds no way, the end stays.
6. Last pass: what is still open is routed around everything as single tracks.
7. Tuning: the shorter track of a path gets 45 degree bumps on its outer side, where they fit,
   down to `--pair-skew` or a `skew` rule.

Width, gap and via gap per layer come from `diff_pair_gap` and `track_width` rules in the
`.kicad_dru`, then the net class. `--pair-zdiff` sets the width per layer from the stackup with the
IPC-2141 formulas (microstrip outside, symmetric stripline inside); the report gives the estimate.

## Search

A portfolio of router runs (different orders, fan-outs first or last) runs on all cores. Then
an evolutionary search improves the routing order: each generation mutates the best order
(failed connections move forward, a few random moves) and evaluates the mutants in parallel.
The best few candidates go through the geometry step and the one with the fewest violations
wins.

## Parallelism

- Portfolio: N independent router runs (different orders and seeds), one per core.
- Within a run: A* candidates for independent connections in the same congestion round, the
  realization per layer and the checks per net run in parallel.
- Data shared between threads is immutable (substrate); per run state is owned by the run.

## KiCad integration

- File based first: `gummiband route board.kicad_pcb -o routed.kicad_pcb`. Reads net classes and
  rules from the `.kicad_pro` next to the board.
- Plugin: KiCad IPC API (KiCad 9 and 10). It saves the board, runs `gummiband route` with the
  options of a small dialog (nets or net classes, priorities, layers), and loads the new
  tracks into the open board as one undo step.
- Manual control: lock tracks in KiCad to keep them; per net options in a `gummiband.toml` next
  to the board (priority, width, skip).

## Testing

- Unit tests for geometry, triangulation substrate, chord planarity and the funnel.
- Small synthetic boards in `testdata/` with expected completion.
- Real board: the Lichtschnitzel controller (4 layers, signals on F.Cu and B.Cu, inner planes).
  The benchmark routes it completely: all tracks and vias are removed first, including the
  locked scripted copper and the outer layer pours; placement, outline, keepouts and the inner
  planes stay. Metrics: completion, vias, length, runtime, `kicad-cli pcb drc` result, compared
  with the Freerouting pipeline of the Lichtschnitzel repo.
