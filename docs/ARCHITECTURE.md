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
   clearance check in the core, then new segments/arcs/vias into a copy of the board
   (a track whose ends fall on the same nanometre is left out: KiCad takes it for a crossing);
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
  converted to polygons with a bounded error (`max_error`, default 5 um outward). A solder mask
  opening of its own is a keepout, with the net of the copper in it if there is one: that net
  must be able to draw track there, so the triangles stay free and only close to other nets
  (`Mesh::opening`). A pad's own opening is simply its clearance.
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
  forbids it (or inside the courtyards a rule exempts). Under such a rule a line that leaves the
  courtyard is cut where a track of the class width would no longer reach into it, wherever a
  track gets its shape (`realize::zone_cuts`: the band, the taut pass, the widening), so only the
  piece into the courtyard is narrow and the round end of the wide one stays out of it. On the
  finished geometry the necks widen piece by piece as far as there is room
  (`relax::widen_necks`).
- Layer changes on current paths: a connection between two pins that take the minimum width of
  their class (`Board::current_min`: not for the nets of a differential pair) pays as many vias
  per layer change as carry that width (`Router::via_factor`, `Board::vias_for`, IPC-2221), and
  at the very end (`relax::stitch_vias`, after the placement rounds) the missing vias go next to
  the via, in a row on a straight track with a piece of the other layer over them or around it
  with a piece on each layer, tested with the rules of the check (`check::Copper::clear`).
- A chain whose new vias split triangles is checked before it is committed: consecutive
  triangles must still be neighbours, and a via must still be a corner of the triangles the
  chain arrives from and leaves through, since another via of the same chain may have split one
  of them; otherwise the connection is searched again (`Router::commit`). A run that still trips
  over its own topology is lost and counts as one that routed nothing (`evaluate`), so the
  search goes on.
- Multi pin nets: Prim order, each new pin to the tree; overlapping pads of one net count as
  one pin. Under a minimum width rule with exempt courtyards the pins that cannot take the
  minimum (`Board::narrow_pin`) join last, so that each ends a branch and the current between
  two other pins never passes a neck.
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

For finishing nets on a routed board (`--keep-all --nets`, shove on by default) the unlocked
tracks and vias of the other nets are no obstacles but part of the topology
(`gummiband-core::import`). `finish_nets` drives it for the CLI and the MCP server: when a
connection still finds no way, the nets near its pins give way in growing circles, and the try
with the fewest open connections, then violations is kept.

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
5. Clearance check in the core; for the nets with such a rule the pieces below the minimum out
   of exempt courtyards (`width`) and the widest way through their copper between the pins that
   can take the minimum (`current` where it carries less: copper by layer and thickness, a layer
   change by the barrels of all vias that join the same copper, `Board::current_width`,
   `Board::via_current_width`); the KiCad DRC is the final judge in the benchmark.
6. Optional, `--angles 45` (`octo.rs`): the finished tracks become conventional 45 degree
   routing. Per track a greedy walk along its path: the farthest point reachable with two legs on
   the grid (or a Z of three), else a staircase or the old piece; then 45 degree chamfers on the
   90 and 135 degree corners. Every leg is tested with the clearance rules of `check` against the
   current copper, with its own width (the old width, a neck only where the router had one), so
   no violation is added. Four passes over all tracks, because a tight bundle only gets room from
   the outside in. Vias and track ends stay. `octilinear_copper` runs the same step on the
   copper of a routed file (`gummiband angles45`): `Board::soften` cuts the unlocked tracks into
   runs between vias, branches and pads, each run is one track, and `soft_changes` names the
   file items to replace; runs that did not change stay in the file.

## Moving parts with the copper

`follow::follow_steps` moves parts from one set of footprint offsets to another in steps no
longer than the smallest clearance (0.02 to 0.1 mm), so that no pad can pass a track within one
step. Per step: `follow_parts` takes the track ends and vias on moved pads along (spread over
the first millimetre of the track, so it does not kink), `wake_near` wakes the sleeping copper of
other nets that the moved pads now come too close to, and `relax` lets it give way; the
relaxation wakes what that copper touches in turn. A step that leaves a clearance spot of a net
pair that had none within reach, or a worse one (`is_new`), a courtyard that runs into another part or leaves the board, ends the move at the
last clean step. Copper that stays asleep keeps its exact shape. `move_with_copper` runs this on
the soft copper of a routed file (`gummiband move-part`, MCP `move_part`); `route_board` runs it
for the shifts of `--move-parts`, with all routed copper awake, before it considers a new search.

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
A pair is merged only if a `diff_pair_gap` rule or a net class of its own marks it as a pair
(`Board::coupled_pairs`; `--pair-all` merges every pair recognised by name).

## Length rules and tuning

`gummiband-kicad::dru::match_rules` reads the `length`, `skew`, `diff_pair_uncoupled` and
`diff_pair_gap` constraints as KiCad groups them: per net and kind the last rule whose condition
holds wins (`Board::match_rules`). `matching.rs` measures as KiCad's DRC does
(`LENGTH_DELAY_CALCULATION`, `DRC_TEST_PROVIDER_MATCHED_LENGTH`,
`DRC_TEST_PROVIDER_DIFF_PAIR_COUPLING`): tracks joined into runs where exactly two ends meet, the
part inside a via (and inside a pad, for runs ending in its centre) straightened, vias over the
layers they really connect with the stackup distance (outer copper in full, inner copper half),
pad-to-die lengths; skew against the longest net of a group; coupling of exactly parallel lines
(within KiCad's int32 tolerance) and of arcs about nearly the same centre, if nothing lies between
and the gap fits the rule. Tracks of tuning patterns stay out of the coupling, as in KiCad.

Before that, still inside the geometry step, `tune::make_room` makes room: `room_wanted` gives
every net (with `--pairs` every pair band, keyed by its P net) that is too short an extra width
of twice the bump height its missing length needs, from the gain of one bump per `4r` of line
(`2h + (2 pi - 8) r`), clamped to `[2r, amplitude]`. `push_aside` widens these tracks in steps of
0.15 mm with `relax` (no tension, vias fixed, only the widened tracks and what they touch awake),
keeps the old width as necks at the ends, near pads and zones and at spots that already had a
violation, then pulls the moved tracks taut and restores the widths. `make_room_judged` keeps the
result only if the judge (the router's clearance check, with `--pairs` on the split geometry)
finds no spot that was not there before; otherwise it bisects the set and pushes the halves on
top of what was accepted, at most 12 pushes. In `pairs.rs` this runs on the merged board, before
the split.

`tune.rs` runs after the routing (and after the 45 degree step) when the board has such rules.
It computes an aim per net (length window first, then the skew groups against their longest
net), then places meanders: per straight line of the track, slots of `4r` along the line
(`2r` = track width plus the leg spacing), per slot the side and the highest bump that keeps the
clearance to all foreign copper and the edge (bisection on the height), then one common cap on
the heights, found by bisection, so that the gain matches the need; below the gain of one low
bump, a single bump with a smaller radius. A pair whose two nets both need length is tuned on
the lines where its two tracks run side by side: the bumps are planned for the centre line with
the width of the whole band, and both tracks are its offsets, so both gain about the same (the
plan is lowered until neither net gets past its `ceiling`: window max, longest net of its
group). The meander becomes a track of its own (the original track is split around it) and a
`tune::Pattern`, which the writer turns into a KiCad `tuning_pattern` group (base line, target,
settings, members).

## Search

A portfolio of router runs (different orders, fan-outs first or last) runs on all cores. Then
an evolutionary search improves the routing order: each generation mutates the best order
(failed connections move forward, a few random moves) and evaluates the mutants in parallel.
The best few candidates go through the geometry step and the one with the fewest violations
wins.

`--until-stop` adds an open end (`improve.rs`). It starts from the router state of the winner,
which it gets by repeating that run, and keeps the relaxed geometry next to it, with the segment of
each track and the site of each via. A mutant clones the router, rips up a few connections around
one spot (with those that share a via), routes them in a random order at a varied via cost, and
takes over from the parent what is still there, asleep; only the new copper starts awake, and the
relaxation wakes what it touches. This costs about 1/20 of a full evaluation, so far more mutants
fit in. Many moves come to nothing (the connections find their old ways again), so a generation does
not start a fixed number of mutants and wait for all of them: every core takes the next move as soon
as it is free, until the generation has its mutants (as many as the portfolio has runs, half of them
for the side branch, if there is one). The first ones in the order of the moves count, not the first
ones done, so the result does not depend on the threads; a mutant that is no longer needed stops,
also in the middle of its relaxation.
Every 24 generations the whole board is relaxed again, tension first, which also tightens
what has been asleep. After 24 generations without a noticeable gain a side branch starts from the
result with a bold change, from a handful of connections around a spot up to a quarter of the board
with every branch that failed, that may be worse, gets half of the mutants for 32 generations and
replaces the result if it beats it. A gain means more connections, fewer violations or shorts, or
0.2 % less via and length cost.

While connections are open, half of the moves aim at one of them instead of at a spot:

1. The connection looks for its way as in a negotiation (`Router::way_through`, on a copy of the
   router): it may run through tracks and overfull edges at a price, which the move varies. Only an
   edge with no room for it even when empty stays closed. What it ran into is what really blocks
   it: the tracks it crosses, what lies on the edges that overflow, the vias at those edges and
   what passes its own vias, on every layer. `Router::blockers` can only guess this from the
   straight line, and a spot at a pin of the connection did not even take the connection itself.
2. These connections go, with those that share a via and a few more around the way.
3. The open connection is laid first, at its own via cost, between 0.1 and 1 of the normal one: the
   last connections often need a change of layer that the normal price talks them out of. The
   others follow at a varied via cost, then at the normal one. Where it finds no way at all, for
   instance from a pad in a pocket, a via in the pad helps, the last resort of a run.
4. A connection that lost its way in this clears its own in the same way (`clear_ways`), as long as
   the chain of the move lasts, and never at the expense of one that already cleared its way.

A side branch aims at open connections too, in four of five bold moves; the others take out a whole
area as before. With every branch that failed the moves get bolder: more goes around the way, it
costs less to run into something, the chain gets longer. So far a mutant could not lose a
connection even in a side branch, because a lost connection was also a net in one more piece.
`Router::net_splits` counts the pieces no open connection accounts for, so that a split stays
forbidden and a lost connection does not.

A mutant with one more connection that does not beat the result, because of its shorts, starts a
side branch of its own at once, if it is better than the one running, and a branch lives on as
long as it gains. Two in five spots go to a short while there are any.

The start of the relaxation decides most of this. On Corne only 3 of 84 mutants with one more
connection beat the result, the others brought 4 to 20 shorts, because a track that starts on the
wrong side of a neighbour stays there. The realization places tracks by the topology, the
neighbours asleep lie where the relaxation left them, so a new track now starts between its
neighbours as they really are (`Router::threaded`: on every edge between the real crossings of its
neighbours, a taut polyline through those portals), with the vias at their real places. And the
site of a new via can lie behind a track that has moved over it since: its track then crosses that
track an odd number of times, which no push undoes, so the via goes along its track past the
crossing if that leaves fewer such crossings on all its tracks (`place_new_vias`). A via in the
plane of its net does not go where it would leave that plane: a connection can count as made
through the plane at that via, and outside it KiCad sees the connection open and the via dangling
(`GB_PLANE_HOLD=0` lets it go, as before 2026-11-01).

Three ideas of the night series of 2026-11-01 run by default, combination A (`improve::Ideas`,
`docs/MILESTONES.md`): the via price of a move reaches up to what a via is worth in the cost of the
result and is drawn by what has won on this board so far (`GB_BANDIT`, Thompson sampling); a quarter
of the moves aims at an expensive routed connection (`GB_AIM_COST`, the share); and a mutant that
cannot beat its parent is dropped before its relaxation (`GB_PRUNE`). Against the open end without
them the cost after 20 minutes fell by 12 % on Lichtschnitzel and 15 % on Arty (median of 4 and 3
seeds), with fewer vias and a little more length. `GB_BANDIT=0`, `GB_AIM_COST=0` and `GB_PRUNE=0`
switch them off; runs without an open end are untouched by them.

The open end does not count violations but spots: all violations of one pair of nets within 2 mm
are one spot (`check::spots`), because the check reports every short piece of track, so that one
curve around a pad can make twenty out of one problem. A spot weighs 300 and up to 1000 more for
what is missing, full at 0.03 mm, so a spot that gets better counts before it is gone. A short weighs
5000 on top. Before every tightening a pass works on the spots: the tracks there stay at their
neck width for 1 mm around the spot, where the rules allow it, and only those nets move.

In the open end a connection may end on a track of its net, also a plane fan-out (`End::Tap`):
right after crossing an edge next to such a track the search is done, if that track is not on its
own side of the gap. Before, it could only end on a pad or a via, so a connection to the same place
ran the whole way next to the first one. The end stays on that segment: in the relaxation it slides
along it to where its way is shortest, and whatever rips up a segment rips up what ends on it. Two
fan-outs that end on each other reach no plane, `Router::net_pieces` counts that. Runs without
`--until-stop` have no taps. A via weighs 20 mm of track in the open end, half of its weight in the
search before, or the taps pay for fewer vias with long ways; `--open-via-cost` sets that price.

The search has converged when even bold changes do not help: `TOP_LEVEL` side branches in a row
have failed, each bolder than the last. One more connection or fewer spots start that over, a gain
in vias or length only holds it, because those can go on falling for hours. There is no count of
generations. Meanwhile a front keeps the results at the level of the best one that no other beats
in both vias and length; they are finished like the best one and written next to the output, for
the user to choose, because the cost weighs vias against length once and for all.

`--air` then relaxes the whole board without tension and with a larger margin. Where that leaves
new violations, the nets involved go back to the normal margin. These passes and the open end
also let the board edge push vias and keep a via in the plane it reaches (`RelaxConfig::via_edge`).

`--prefer-width` runs at the very end, on the best result, because the topology keeps counting
with the narrow tracks. First the tracks push as if they had the target width, without tension,
so that neighbours and vias give way in chains. A spot that has no room for that becomes a neck of
1.2 mm at the old width, and the pushing goes on, for a few rounds; one tight spot must not cost a
track its whole length. After that `relax::prefer_width` lets every piece of track grow in steps of
0.025 mm, round by round so that a bundle shares its room, drops wider stretches under 1.5 mm and
ramps every change of width by one step per piece.

The last step tapers (`improve::taper_spots`, on by default since 2026-11-01, `GB_TAPER=0`
switches it off): a spot that misses its clearance by at most 50 µm gets the nearest line or arc
of a net whose class has a minimum width rule narrower as a whole, so that its edge moves back by
the deficit and 2 µm, never below the minimum. The width goes to point zones in the middle of its
pieces and of its chord, as `relax::prefer_width` sets them: the file has one width per arc, and a
zone around the spot would also reach a neck next to it and make it as wide as the taper. Three
rounds; the result counts only with fewer violations.

## Local patterns

`gummiband_core::patterns` finds, once per board and from the pins alone, the situations a
person routes by habit: rows of pins of one part at a regular pitch (with the side tracks leave
to), pad grids of at least 3 x 3 (BGA), decoupling capacitors (a two pin part next to a pin of a
larger part, on the supply net they share, its other pin on a plane or a net with many pins),
pins of a plane net close enough to reach the plane through one via, and buses between two pin
rows. `gummiband info`, `route --report` and the MCP tool `board_patterns` list them.

A pattern is only a hint to the search, never fixed copper: what does not fit a board costs at
most the run that tried it, and nothing is ever given up for it. `route --patterns` switches the
hints on, all off by default (and then byte for byte the same as without them):

- `decaps`: the connection between a capacitor and its IC pin goes first, while the room at the
  pin is free (`Router::plan`).
- `shares`: a pin of a plane net connects to its neighbour or its capacitor (the leader, which
  fans out) instead of taking a via of its own. A connection that takes a via or a detour
  for that falls back to a fan-out of its own (`Router::detour_bridge`).
- `rows`: a move of the open end (`improve::row_move`, 15 % of the moves) takes the connections
  of one pin row out and lays them again in the order of their pins, from one end, the other,
  the middle out or the ends in.

Pockets (a pad that can only be left through a via in it) are not a pattern: the tracks of its
neighbours close the pocket, not the fixed copper, so the empty board does not show them (see
the via in a pad experiments in `MILESTONES.md`).

## Parallelism

- Portfolio: N independent router runs (different orders and seeds), one per core.
- Within a run: A* candidates for independent connections in the same congestion round, the
  realization per layer and the checks per net run in parallel.
- Open end: one loop per core takes the next move of a generation (`improve::first_through`).
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
