# How Gummiband routes

This chapter explains the router's behaviour from the user's side: what it reads from your board,
what it respects, how it decides, and where its limits are today. The algorithms themselves are in
[ARCHITECTURE.md](../ARCHITECTURE.md).

## Topology first, geometry second

A grid router draws tracks as it goes and gets stuck when the grid is full. Gummiband separates
two questions:

1. **Topology**: for every connection, on which side of every pad, via, locked track and keepout
   does it pass, and on which layer? The router plans this on a triangulation of the obstacles,
   layer by layer, and only accepts a plan when every triangle edge still has room for all the
   tracks that cross it with their widths and clearances. Plans never cross each other on a
   layer.
2. **Geometry**: the planned paths are pulled taut like rubber bands. Every track is a chain of
   points under tension; obstacles, vias and neighbouring tracks push it away by their clearance,
   vias move with their tracks. Corners become the largest arcs that still keep clearance. The
   result has real KiCad arcs and any angle.

This is why the tracks look the way they do: they hug the pads and each other with exactly the
clearance, run straight where nothing is in the way, and bend around obstacles in arcs rather
than 45 degree stairs.

## What the router reads from your board

| From | Used for |
| --- | --- |
| Board outline (`Edge.Cuts`) | the routable area, with the edge clearance of the board rules. Open or broken outlines fall back to the bounding box with the edge lines as obstacles. |
| Copper layers | routing happens on all copper layers that are not planes. Layers of KiCad type *power* (and `--plane-layers`) are planes. |
| Footprints and pads | every pad is an obstacle for other nets and a pin for its own net. Overlapping pads of one net count as one pin. Non-plated holes are obstacles with the hole clearance. |
| Locked tracks and vias | kept as fixed copper of their net (`--keep-locked`, the plugin default); their ends are pins. Unlocked ones are removed. |
| Zones | pours on plane layers define the planes; a pin of the plane net is connected by a via into the plane outline (or by a through-hole pad). Keepout zones are obstacles on their layers. Other zones stay in the file and are not refilled by the router. |
| Net classes (`.kicad_pro`) | track width, clearance, via diameter and drill, priority, per net. |
| Board rules (`.kicad_pro`) | minimum clearance, minimum track width, edge clearance, hole to hole and hole clearance, minimum via diameter. |
| Custom rules (`.kicad_dru`) | see below. |

Graphics on copper layers (logos, text) are obstacles too.

## Rules

Gummiband honours a subset of KiCad's custom design rules:

- **Clearance between two nets or net classes**: rules with a `clearance` constraint whose
  condition talks about `A.NetName`, `A.NetClass`, `B.NetName`, `B.NetClass` and `A.Type` /
  `B.Type == 'Track'`, with wildcards. Typical use: a 3W spacing rule for a group of data lines.
  Position dependent parts of the condition (`intersectsCourtyard`, `insideArea`) evaluate to
  false, which keeps a rule like `... && !A.intersectsCourtyard('U1')` in force everywhere. That
  is the safe side.
- **Minimum track width per net class**: `track_width (min ...)` constraints with an
  `A.NetClass == 'X'` condition. They bound how far a track may neck down near fine pitch pins;
  a `!A.intersectsCourtyard('REF')` in the condition exempts that footprint's courtyard, so a
  fan-out under a QFN can still be narrow.

Everything else in the `.kicad_dru` is ignored. Run the KiCad DRC on the result to catch what the
router could not see.

## Track width and necking

A track has the width of its net class. Where the class width does not fit between fine pitch
pins, the router may neck a track down near the pin: to the pad width, or at fine pitch to the
pitch minus the clearance, at most 0.4 mm, and never below the board's minimum track width or what
a custom minimum width rule allows. The neck covers the pin area (the courtyard when a rule exempts
one), so a whole fan-out narrows alike.
`--net-widths` overrides the class width per net.

## Vias

Vias are placed on a grid of candidate positions (`--via-pitch`, 0.5 mm by default) wherever
they fit with their clearance, and may move during the geometry step. Through-hole pads are free
layer changes for their net. A via in a pad is a last resort for nets with big enough pads when a
fan-out has no other way out. The via cost (`--via-cost`, in millimetres of track) decides how
eagerly the router changes layers; the search also tries values around it.

## Priorities and order

Connections are routed in order of their net class priority (lower first); plane fan-outs are
routed first or last depending on the attempt. Every third attempt gives the layers preferred
directions (horizontal and vertical in turn, `--layer-dirs`), which keeps bus like boards tidy;
the other attempts route without that preference, and the best result wins. When a connection finds no room, nearby
connections are ripped up and rerouted; when that fails, the connection is marked unrouted and
the next search round moves it forward in the order. Many attempts with different orders run in
parallel (the portfolio), then an evolutionary search mutates the best order for a few
generations. The best candidate wins: fewest unrouted connections, then fewest violations (after
the geometry step), then fewest vias and shortest length.

Results are deterministic for the same input, options and seed.

## Judging and violations

After the geometry step the router checks every track and via against every obstacle and every
other net with the class and rule clearances, the edge clearance and the hole clearances, and
reports the violations by kind and position. With `--geometry-generations` this check also
drives the search. KiCad's DRC uses the same rules with its own geometry; its numbers are close
but not identical, and it is the final judge.

## Parallelism

Gummiband uses every core: independent routing attempts run side by side, the geometry step
runs per layer in parallel and the checks per net. `--threads` (or `RAYON_NUM_THREADS`) limits
the pool, `--runs` the number of attempts per phase. Memory grows with the number of parallel
attempts on large boards.

## Current limits

Things the router does not do yet. The [milestones](../MILESTONES.md) say what comes next.

- **Partial routing.** Every run replaces all unlocked copper. There is no "route selected nets"
  and no follow-me routing from a half drawn track yet. Lock what you want to keep.
- **Differential pairs and length matching.** Pairs are routed as two independent nets.
- **Teardrops.** Add them with KiCad's own teardrop feature after routing.
- **Per net options from a file.** Priorities come from net classes; a `gummiband.toml` next to
  the board is planned. The MCP server has per net priority and skip today.
- **More rule conditions.** Only the clearance and minimum width rules above; no rules by
  position, footprint or layer.
- **Clean DRC on dense boards.** Expect a few clearance items and the occasional crossing in
  BGA fan-outs; the geometry generations reduce them, hand finishing removes the rest.
- **Plugin features.** Moving parts and per net options are command line and MCP features; the
  plugin dialog covers keep locked, runs, via cost, geometry generations and the animation.
