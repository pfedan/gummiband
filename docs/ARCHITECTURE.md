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

One triangulation for all layers keeps vias simple: a via is a vertex shared by every layer.

- Vertices: polygon vertices of all obstacles (all layers), pin anchor points, board outline
  vertices and free via sites (a jittered grid plus points on the medial axis of large gaps,
  filtered by via clearance on every layer).
- Constraint edges: obstacle polygon edges and the outline (intersections are split).
- Per layer, each triangle is free, blocked or owned by one net (inside a pad or locked copper of
  that net). A route of net N may enter a triangle that is free or owned by N.
- Per layer, each edge has a capacity: its length minus the space its two end vertices need
  (clearance to the obstacle at that vertex, or the via radius plus clearance for an occupied via
  site). Every route crossing it uses width plus clearance.

## Topological routing

A route on one layer is a sequence of edge crossings between two vertices. Each crossing has a
slot: its position among the other routes crossing that edge. Inside one triangle every route is
a chord between two boundary positions; a new chord is legal if it does not interleave with an
existing chord. So the layout of each layer stays planar by construction and the geometry step
can always realize it if capacities hold.

- Search: A* over states (triangle, entry edge, slot, layer), cost = Euclidean length of the
  guiding path through edge midpoints at the slot position, plus via cost, plus congestion and
  history penalties (negotiated congestion, PathFinder style) for edges near capacity.
- Layer change: at a free via site vertex; the via then occupies the vertex on all layers.
- Multi pin nets: Kruskal MST over pins (and locked copper of the net), routed as two pin
  connections; a connection may end on any copper already routed for the net.
- Rip-up and reroute: a connection that fails rips up the connections that block it (those that
  use the full capacity or the slots it needs), with growing history costs.
- Order: by priority, then by bounding box size (short first). The portfolio runs several orders
  and seeds in parallel (rayon) and keeps the best result (completion, then vias, then length).

## Geometric realization

For each layer and each edge, the slots give the order of routes. Each route gets a portal on
every edge it crosses: an interval that leaves room for the routes on either side and the
clearance to the edge end vertices. The shortest path through the portals is the funnel
(string pulling) algorithm. Where the taut path bends at a portal end, it wraps around the
obstacle or neighbour route there; the bend is replaced with an arc of the inflated radius, so
tracks follow obstacles at exactly their clearance. Later steps spread routes apart where
capacity is generous (even spacing, "gummiband" relaxation).

## Plane nets and current

- Nets with a plane on an inner layer (a copper zone of that net on a non routing layer) are
  connected by fan-out: each pad gets a short track to a via into the plane, as close as the
  rules allow. Only pads that cannot reach a plane are routed on the signal layers.
- Track width comes from the net class (Output 1.5 mm, Input 0.75 mm on the test board); the
  capacity model uses the real width, so wide current tracks take their space in the topology.

## Elastic optimizer (radical track)

Beside the discrete pipeline we build a continuous one: every route is an elastic band (a chain
of points with tension) and every obstacle and foreign route pushes with a clearance potential.
Bands relax in parallel (each band reads the last state of the others, Jacobi style), which
shortens routes, spreads them evenly and resolves small clearance violations without rip-up.
Topology changes (a band snapping over an obstacle when that is shorter) are allowed between
relaxation rounds. The discrete router supplies the start topology; the relaxation turns it into
the final, smooth geometry. Both tracks share the board model and the checks.

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
