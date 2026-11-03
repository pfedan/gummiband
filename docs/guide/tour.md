# A tour of the features

This page shows what Gummiband does, one picture per feature, and points to the chapter that
explains how to use it. The result images come from the boards of the
[benchmarks](benchmarks.md): the Lichtschnitzel LED controller (a real four layer board), the
synthetic test boards and a DDR5 testbed. The drawings explain a principle and are not to scale.

Top layer copper is red, bottom layer copper is blue in all pictures.

![The Lichtschnitzel controller routed by Gummiband: any-angle tracks with arcs on both outer layers](img/routed-lichtschnitzel.png)

*The Lichtschnitzel controller as Gummiband routes it: every connection made, tracks with any
angle and real arcs. The inner layers are planes and not shown.*

## Rubber band routing

![Topology first: a route is a list of edge crossings in a triangulation. Geometry second: the route is pulled taut around the obstacles at their clearance.](img/topology-geometry.svg)

Gummiband first decides on which side of every pad, via and locked track a connection passes,
and only then pulls the planned path taut like a rubber band. The tracks hug the pads at exactly
the clearance, run straight where nothing is in the way and bend in arcs. More in
[How Gummiband routes](concepts.md#topology-first-geometry-second).

## A search that keeps the best

![The phases of a run: portfolio, generations, geometry, geometry generations, keep improving, finishing](img/search-pipeline.svg)

Many routing attempts with different orders run in parallel, an evolutionary search improves the
best order, and the geometry generations judge the drawn tracks by their clearances. With *Keep
improving* (`--until-stop`) the result gets better until you accept it; the progress window shows
vias, length and violations over time.

![The progress window of the KiCad plugin in the open end: generation, routed connections, vias, length, violations and a chart of how they went](img/plugin-progress.png)

See [Placing and routing in KiCad](kicad-plugin.md#the-dialog) and
[Spend more effort](workflows.md#spend-more-effort).

## Track width: necks, tapers, wider where there is room

![Necking at fine pitch pins inside the courtyard, and a short taper where a wide track passes a tight spot](img/necking.svg)

A track has the width of its net class. At fine pitch pins it may neck down inside the part's
courtyard; at a spot that is a few micrometres too tight it gets narrower right there, never below
the minimum of a `track_width (min ...)` rule.

![A track at its target width, narrowing to the class width in a gap with a taper at each change](img/prefer-width.svg)

With a target width (`track_width (opt ...)` in the `.kicad_dru`, or *Preferred track width* in
the plugin) tracks grow where there is room, with a taper at every change of width. See
[Track width and necking](concepts.md#track-width-and-necking) and
[Wider tracks where there is room](concepts.md#wider-tracks-where-there-is-room).

## Conventional 45 degree tracks

![Any-angle routing with an arc next to the same connection after the 45 degree step with chamfered corners](img/angles45.svg)

If your reviewer wants 45 degrees, `--angles 45` (plugin: *45 degree tracks*, or the separate
button for a finished board) turns the result into straight legs in eight directions with
chamfered corners. Vias and track ends stay where they are, and no new clearance violation is
allowed.

![A part of the Lichtschnitzel controller before and after --angles 45](img/angles45-before-after.png)

See [Conventional 45 degree tracks](concepts.md#conventional-45-degree-tracks).

## Length matching and skew

![A meander: bumps on the straight part of a track, with their height and the air between legs](img/meander.svg)

The `length` and `skew` rules of the `.kicad_dru` are read as KiCad reads them. The router makes
room next to nets that are too short, then adds meanders that KiCad treats as its own tuning
patterns. The search already prefers routes of similar length within a skew group.

![The synthetic matching board without and with length matching](img/length-matching-before-after.png)

See [Length matching](concepts.md#length-matching) and the
[measurements](benchmarks.md#length-matching).

## Differential pairs

![A pair routed as one wide track and split into two tracks at the pair gap, with single tracks into the pads](img/pairs.svg)

Pairs are recognised by name. `--pair-pull` keeps the two nets together while routing them on
their own; `--pairs` routes a pair as one wide track and splits it in two at the gap from the
rules or an impedance target. `--breakouts` ends the wide track before the pads.

![A connector row with --pairs: on the left the wide track ends at the pads and shorts on the right, on the right the breakout zone before the pads](img/pair-breakouts-before-after.png)

See [Route differential pairs](workflows.md#route-differential-pairs-experimental) and
[Differential pairs](options.md#differential-pairs).

## Dog-bone vias in ball fields

![A ball grid with the plain via raster next to the same grid with dog-bone sites between every four balls](img/dogbone.svg)

In a ball field the via raster rarely lines up with the balls. Gummiband finds ball fields by
their pad lattice and adds a via site in the middle of every four balls and half a pitch outside
the outer ones, so the fan-out gets the classic dog-bone.

![The synthetic bga_dense board with raster sites only and with dog-bone sites](img/dogbone-bga-before-after.png)

See [Vias](concepts.md#vias).

## Placement from the netlist

![Two placement terms: a band kept free for a bus between two ICs, and room per side of an IC for its escaping tracks](img/placement-terms.svg)

`gummiband place` (and the plugin button *Place with Gummiband*) places the parts that are not
locked. Opt-in terms use the schematic (`--semantic`), keep bands free for buses
(`--corridors`), give each side of an IC the room its tracks need (`--escape-halo`) and place on
both sides (`--two-sided`).

![Four placements of the Lichtschnitzel controller, seeds 1 and 4, without and with --semantic: the input chain and the switch nodes get much shorter](img/semantic-placement.png)

![A two sided placement of a module carrier: 63 parts on the front, 106 on the back](img/two-sided-som.png)

See [Choosing the opt-in options](options.md#room-for-track-groups) and
[Place the parts](workflows.md#place-the-parts).

## Clearing bands for buses before routing

![parts_xhard with --patterns groups alone and with --clear-corridors: fewer vias and less length](img/clear-corridors-before-after.png)

`--clear-corridors` moves small parts out of the band of a bus before routing, together with the
routing order hint `--patterns groups`. See [Room for track groups](options.md#room-for-track-groups).

## Finishing nets and shoving

![A track started by hand is continued by the router, and an unlocked track of another net gives way](img/finish-net.svg)

*Finish selected nets* (or `--keep-all --nets`) completes the nets you point at, from where a
track you started ends, and pushes unlocked tracks of other nets aside where it needs the room.
See [Finish a partly routed board](workflows.md#finish-a-partly-routed-board-experimental).

## Good practice for power and signals (MCP)

![Buck converter A of the Lichtschnitzel controller, routed without and with power_copper](img/power-copper-buck.png)

![A signal track over its reference plane: a gap at a slot and a split between two zones](img/reference-check.svg)

For AI agents the MCP server adds the checks a careful layouter makes: `power_plan` and
`power_copper` before routing (pours and via fields for current paths), `current_check`,
`ground_pour` and `reference_check` after it. See [Working with an AI agent](ai-agents.md).
