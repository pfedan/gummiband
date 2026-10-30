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
| `gummiband-cli` | `gummiband` binary: `info`, `route`, `svg`. The plugin calls this. |
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
3. Taut pass: straight shortcuts wherever they keep all clearances (removes the waves).
4. Fillets: every corner is rounded with the largest arc that keeps all clearances; the output
   uses real KiCad arcs.
5. Clearance check in the core; the KiCad DRC is the final judge in the benchmark.

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
