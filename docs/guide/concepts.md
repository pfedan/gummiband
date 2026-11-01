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
| Solder mask | openings are obstacles for other nets, see [Solder mask openings](#solder-mask-openings). |
| Locked tracks and vias | kept as fixed copper of their net (`--keep-locked`, the plugin default); their ends are pins. Unlocked ones are removed. |
| Zones | pours on plane layers define the planes; a pin of the plane net is connected by a via into the plane outline (or by a through-hole pad). Keepout zones are obstacles on their layers. Other zones stay in the file and are not refilled by the router. |
| Net classes (`.kicad_pro`) | track width, clearance, via diameter and drill, priority, per net. |
| Board rules (`.kicad_pro`) | minimum clearance, minimum track width, edge clearance, hole to hole and hole clearance, minimum via diameter. |
| Custom rules (`.kicad_dru`) | see below. |

Graphics on copper layers (logos, text) are obstacles too. Text counts as a box around it, a little
larger than the letters.

## Rules

Gummiband honours a subset of KiCad's custom design rules:

- **Clearance between two nets or net classes**: rules with a `clearance` constraint whose
  condition talks about `A.NetName`, `A.NetClass`, `B.NetName`, `B.NetClass` and `A.Type` /
  `B.Type == 'Track'`, with wildcards. Typical use: a 3W spacing rule for a group of data lines.
  Position dependent parts of the condition (`intersectsCourtyard`, `insideArea`) evaluate to
  false, which keeps a rule like `... && !A.intersectsCourtyard('U1')` in force everywhere. That
  is the safe side.
- **Minimum and target track width per net class**: `track_width (min ...)` constraints with an
  `A.NetClass == 'X'` condition. They bound how far a track may neck down near fine pitch pins;
  a `!A.intersectsCourtyard('REF')` in the condition exempts that footprint's courtyard, so a
  fan-out under a QFN can still be narrow. An `opt` value in the same constraint is the target
  width of the class: see [Wider tracks where there is room](#wider-tracks-where-there-is-room).
  The `opt` of a rule that also has a `diff_pair_gap` is the width of the pair, not a target.

Everything else in the `.kicad_dru` is ignored. Run the KiCad DRC on the result to catch what the
router could not see.

## Solder mask openings

KiCad's DRC reports `solder_mask_bridge` when a mask opening shows copper of two nets, so nothing
but mask keeps them apart. The router keeps tracks and vias out of the openings of other nets, by
the rule KiCad 10 applies:

- **Pads.** The opening of a pad with a mask layer is the pad plus its margin: the
  `solder_mask_margin` of the pad, else of the footprint, else the board's
  `pad_to_mask_clearance`. KiCad compares copper against this margin and not against the
  clearance, so a pad's margin, plus 0.01 mm against rounding and plus
  `solder_mask_to_copper_clearance`, counts as its clearance wherever it is the larger one. The
  own net gets to its pad as ever. That is the whole story on Corne: no shapes on the mask layers,
  a margin of 0.2 mm around a clearance of 0.19 mm, and every track that hugged a pad lay in its
  opening.
- **Shapes and text on `F.Mask` or `B.Mask`**, in a footprint or on the board, are obstacles on the
  outer copper layer below, and so are pads without copper and non-plated pads wider than their
  drill (a bare drill opens no mask for the check). A drawn shape reaches as far as the board's
  margin beyond its edge, also in a footprint with a margin of its own: that is what KiCad
  measures, although the mask itself does not grow. The net of the pads and copper graphics in an
  opening may enter it and route through it. Over bare board an opening is closed to all nets,
  even though KiCad would accept one net. An opening with several nets in it is a bridge by itself
  and stays out of the routing.
- **Footprints** with the attribute *Allow bridged solder mask apertures between pads*
  (`allow_soldermask_bridges`) have no openings for the router, as for KiCad.

Boards without margin and without such shapes route exactly as before. What the router cannot
change are bridges in the footprints themselves: two pads too close for their openings, four of
them on Corne.

## Track width and necking

A track has the width of its net class. Where the class width does not fit between fine pitch
pins, the router may neck a track down near the pin: to the pad width, or at fine pitch to the
pitch minus the clearance, at most 0.4 mm, and never below the board's minimum track width or what
a custom minimum width rule allows. The neck covers the pin area (the courtyard when a rule exempts
one), so a whole fan-out narrows alike.

Necks stay narrow only where they must. After the geometry is drawn, every narrow piece is
widened step by step (0.05 mm) to the widest width that still keeps all clearances to pads, vias
and other tracks, up to the class width; a net that would gain a violation keeps its old taper.
Where a minimum width rule allows the narrow width only inside a courtyard, the neck ends with
the courtyard: every line that leaves it is cut where a track of the class width would no longer
reach into it, half that width outside its edge, so that the round end of the wide piece stays
out of the courtyard and off the pads in it. The piece outside has the class width, the way
KiCad's rule reads it (a piece whose copper touches the courtyard is exempt, the next one is
not). The geometry steps see that piece at its full width from the start, so the tracks around
keep their room for it. The own check reports a `width` violation for a piece below the minimum
whose copper touches no exempt courtyard.

Such a rule also keeps the current wide. In a net of the class, the pins that cannot take the
minimum width (fine pitch, small pads) are connected last, each as a branch of its own: the way
between two other pins never passes a neck, and a narrow track to a sense pin carries no current.
The own check reports a `current` violation where the widest way between two pins that can take
the width carries less than the minimum anyway. It counts copper by what it carries after
IPC-2221 (the temperature rise drops out of the comparison): a piece on an inner or thinner layer
counts for less than on the outer copper the rule is made for, and a layer change counts with all
vias that join the same copper on both sides, each barrel an inner conductor with 25 µm of
plating. A 0.4 mm via carries what 0.18 mm of 2 oz outer copper carry, so a 4.1 A rule of
0.69 mm needs three of them, five for 1.44 mm. In the search a layer change of a connection
between two such pins costs as many vias as it needs, and at the end the router puts the missing
vias next to the one it placed: in a row on the straight end of one of its tracks with a piece of
the other layer over them, or where that has no room, around it with a piece of copper on each
layer, wherever they keep every clearance, the hole to hole distance and out of the pads
(`GB_STITCH=0` switches that off). The nets of a differential pair are left out: there the width
rule is about the impedance. KiCad's DRC sees neither case: its rule exempts every track that
touches the courtyard, however long it is, and it knows no currents.

The very last step uses the same freedom at tight spots: where a track still misses its
clearance by a few micrometres, at most 50 µm, it gets narrower right there, as much as the
clearance needs and 2 µm more, never below the minimum of a `track_width (min ...)` rule of its
class. A wide power track through a gap that is just too narrow is a matter of width, not of the
way. Only classes with such a rule have this freedom, and the result counts only with fewer
violations. `GB_TAPER=0` switches it off.

`--net-widths` overrides the class width per net.

## Wider tracks where there is room

A net class can have three widths, all set in KiCad:

| Width | Where | What Gummiband does with it |
|---|---|---|
| Class width | Board Setup > Net Classes, *Track Width* | The width it routes with. |
| Target | Custom rule, `track_width (opt ...)` | At the end the tracks grow up to it where there is room. |
| Minimum | Custom rule, `track_width (min ...)` | The floor for necks at fine pitch pins and for tapering at tight spots. |

```
(rule "Power widths"
  (condition "A.NetClass == 'Power'")
  (constraint track_width (min 0.3mm) (opt 0.8mm)))
```

KiCad's DRC reads the same rule, so the minimum is checked there too. `--prefer-width W` (in the
plugin *Preferred track width*) gives the default class a target of W when it has no rule of its
own. Other classes without an `opt` keep their width, and differential pairs always do, because
their width is their impedance.

Growing is the last step: the routing does not see it, so it costs no connections. Tracks first
push their neighbours and vias aside as far as those can go, then they grow, with a taper at
every change of width. A track never gets narrower than its class width this way, and a spot
without room stays at the old width while the rest of the track is wide. KiCad's teardrops
(`--teardrops`) can smooth that further, but the taper does not depend on them.

## Conventional 45 degree tracks

Rubber band tracks are fine for every fab and electrically at least as good as 45 degree routing,
but many reviewers and company guidelines expect 45 degrees, and KiCad's push and shove tools
are made for them. `--angles 45` (in the plugin *45 degree tracks* under *Advanced*, in the MCP
`route` with `angles: "45"`) turns the finished result into conventional routing as the very last
step:

- Every track is walked from its start. From where it stands it reaches as far along its old
  path as one or two straight legs in the eight directions of the 45 degree grid allow (or three,
  as a Z that stays closer to the old line), then goes on from there.
- Corners of 90 and 135 degrees get 45 degree chamfers where they fit; there are no arcs.
- Vias and track ends do not move, so connectivity stays as it was.
- Every new piece is checked against the pads, keepouts, board edge, vias and the current
  tracks of all other nets, with the same rules as Gummiband's own check, so the step never adds
  a clearance violation. Tracks are done one after another, in four passes: a bundle frees its
  room from the outside in, and what got room since the last pass straightens out.
- A piece keeps the width it had, and gets narrower only inside the router's neck zones. Under a
  minimum width rule the narrow pieces at the pins keep their angle or take the full width,
  because the rule allows a neck only where it touches the courtyard.
- Where no 45 degree path fits, typically inside tight bundles at an odd angle, the piece keeps
  its angle. The output counts these pieces and their length.
- With `--pairs` the wide track of a pair goes onto the grid before it is split, together with
  the tracks around it, so the two tracks of the pair stay parallel at their gap (the outer track
  of a corner gets a corner too, not an arc). Near the pads and vias of the pair, where the split
  decides how the tracks leave the wide one, the wide track keeps its direction for about three
  times its width; those pieces stay off the grid. The two tracks are left alone afterwards, only
  what was routed on its own at the ends goes onto the grid in the last step.

The same step also works on a board that is already routed, without routing again:
`gummiband angles45`, the plugin button *45 degree tracks with Gummiband* and the MCP tool
`angles_45`. There every run of track between two vias, branches or pads is one track, and
runs that do not change stay as they are.

Expect tracks about 4 to 5 % longer and a few percent of the length left off the grid on dense
boards (see [benchmarks](benchmarks.md#conventional-45-degree-tracks)). The step takes a second
or less.

## Length matching

The `length`, `skew`, `diff_pair_uncoupled` and `diff_pair_gap` rules of the `.kicad_dru` are read
as KiCad groups them: per net and kind of constraint the last rule in the file whose condition
holds wins, and a `skew` rule holds for the nets it wins (with `(within_diff_pairs)` per pair).
KiCad applies one skew rule per net, so a strobe pair that belongs to a byte lane group has no
within-pair skew rule of its own; Gummiband then keeps the pair's skew below `--pair-skew`.
Lengths are measured as the KiCad DRC measures them: tracks (joined into runs, what lies inside a
via or a pad straightened), each via over the layers it really connects (the stackup distance),
and the pad-to-die length of the pads. `gummiband lengths` prints this for any routed board.

When such rules exist, the routing first makes room for the meanders, then gets them as a last
step (after `--angles 45`):

- Room: every net that is too short (a pair that both nets of want length as one band) gets a
  wider track for the moment, as wide as the meander it needs (twice the bump height, from the
  length it lacks and the room one bump gains), and the geometry step pushes the neighbours aside
  in small steps. Only the widened nets and what they push move; near pads, vias and their ends
  the tracks keep their width, so the breakout stays as it was. The pushed nets are pulled taut
  again and get their widths back. If that leaves a clearance error that was not there before,
  the set is halved and tried again, so the nets that find room keep it. With `--pairs` this
  happens on the wide track of each pair, before it is split into its two nets.

- Aims: a net below the `min` of its `length` rule goes into the window (to `opt`, else the
  middle); in a `skew` group every net more than `max` below the longest one goes to a quarter of
  `max` below it. Nets are only made longer, never shorter.
- A meander is a row of bumps on a straight line of the track, round (semicircles, as KiCad's
  round tuning patterns) or with 45 degree corners with `--angles 45`. Each bump goes to the side
  with more room and as high as its clearance allows (up to `--tune-amplitude`); then all bumps
  are lowered evenly until the gain is what the net needs. Neighbouring legs keep
  `--tune-spacing` track widths of air. The longest lines come first.
- A differential pair whose two nets both need length is meandered as one band: the bumps are
  laid out for the centre line and both tracks follow at their distance, so the pair stays
  coupled. The outer track of a turn gains a little more than the inner one; neither net may get
  past the `max` of its length window or the longest net of its skew group.
- Every meander is written as a KiCad tuning pattern (a `generated` group of type
  `tuning_pattern` with its base line, target and settings). In KiCad it can be selected, edited
  and tuned again like one drawn by hand; the DRC leaves its tracks out of the coupling of a pair,
  as for KiCad's own patterns.

In a tight bundle the room step cannot always push the neighbours aside without a new error, and
a net stays short; the output names every net that did not reach its length (see
[benchmarks](benchmarks.md#length-matching)).

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

Results are deterministic for the same input, options and seed, on any machine and with any
thread count: the portfolio size is a constant (14 runs), and threads only change the run time.

## Judging and violations

After the geometry step the router checks every track and via against every obstacle and every
other net with the class and rule clearances, the edge clearance and the hole clearances, and
reports the violations by kind and position; `mask` is copper in a solder mask opening. With
`--geometry-generations` this check also drives the search. KiCad's DRC uses the same rules with its own geometry; its numbers are close
but not identical, and it is the final judge.

## Parallelism

Gummiband uses every core: independent routing attempts run side by side, the geometry step
runs per layer in parallel and the checks per net. `--threads` (or `RAYON_NUM_THREADS`) limits
the pool, `--runs` the number of attempts per phase. Memory grows with the number of parallel
attempts on large boards.

The GPU is optional and experimental: a binary built with the `gpu` feature and started with
`GB_GPU=1` runs the bulk of the track relaxation as a compute shader (Metal on macOS). The
topological search stays on the CPU, so the gain is limited to boards where relaxation takes a
large share of the time, and the result can differ between GPUs, which is why it is off by
default. See the [environment variables](cli.md#environment-variables).

## Current limits

Things the router does not do yet. The [milestones](../MILESTONES.md) say what comes next.

- **Partial routing is experimental.** By default a run replaces all unlocked copper; lock what
  you want to keep. `--keep-all` with `--nets` adds to existing copper, the MCP `route` with
  `reroute` re-lays named nets on a finished board, and an agent can add locked copper through
  the MCP server before routing. Follow-me routing from a half drawn
  track is the plugin action *Finish selected nets*, not yet tested live in KiCad.
- **Automatic placement is new.** `gummiband place`, the plugin button and the MCP tool
  `place_parts` place parts from the netlist, the outline, the planes and the fixed parts; the
  router is not involved. The placer knows no datasheet layouts, so place and lock a power stage
  yourself; parts on both sides share the same space; tracks and vias do not follow, so place
  before you route; and the seeds differ a lot, so try a few. The button is not yet tested live
  in KiCad. Measured on one board only, see [benchmarks](benchmarks.md#placement-from-the-netlist).
- **Blocks and pre-routing tools are new.** Groups that move as blocks under `--move-parts` and
  the MCP tools `part_info` and `add_locked_copper` exist but have not been tried on a real
  board yet; expect rough edges.
- **Differential pairs are experimental.** Pairs are recognised by name (`_P`/`_N`, `P`/`N`,
  `+`/`-`) and routed as two nets by default. `--pairs` routes a pair as one wide track, splits it
  into two parallel tracks with the gap and width from the rules or an impedance target, and tunes
  the skew within the pair; it keeps pairs together but still costs connections on dense boards
  and can leave DRC errors near pads and vias. `--pair-pull` is the lighter variant: the second
  net is pulled towards the first, spacing only roughly kept. A pair is coupled only if a pair
  rule or a net class of its own marks it as one (`--pair-all` for all). Polarity awareness is
  not there yet.
- **Length matching makes room only after routing.** The router itself does not keep room free
  for meanders; the room comes from pushing neighbours aside afterwards, which a tight bundle
  does not always allow. It does not tune where a mismatch arises (at the bends or the breakout)
  and does not shorten a net that is too long. In tight bundles nets stay short; see
  [Length matching](#length-matching).
- **Solder mask.** A through-hole pad keeps its mask margin on all layers, also where the mask
  does not open. Vias count as tented: a board that leaves them open gets no room around them for
  the openings of pads. Which net may enter an opening is settled when the board is read, also if
  `--move-parts` moves a part below it later. Zones on mask layers are not read.
- **Teardrops.** Gummiband draws none itself. `--teardrops` (command line only) switches on
  KiCad's own teardrop feature for the vias and pads of the output and for the track ends at
  width changes; fill the zones once (B) after opening the file and KiCad builds the shapes.
- **Per net options in one file.** Priorities (`--net-priorities`) and widths (`--net-widths`)
  come from separate JSON files; a single `gummiband.toml` next to the board is planned. The MCP
  server has per net priority and skip.
- **More rule conditions.** Only the clearance and minimum width rules above; no rules by
  position, footprint or layer.
- **Clean DRC on dense boards.** Expect a few clearance items and the occasional crossing in
  BGA fan-outs; the geometry generations reduce them, hand finishing removes the rest.
- **Plugin features.** Moving parts and per net options are command line and MCP features; the
  plugin dialog covers keep locked, runs, via cost, geometry generations and the animation.
