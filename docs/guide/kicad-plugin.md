# Routing in KiCad

The plugin adds one action to the PCB editor: *Route with Gummiband*. It exports a copy of the
open board with its project to a temp folder, runs `gummiband route` on the copy and imports the
result into the open board as **one undo step**. Your file on disk is only changed when you save.

Setup is in [Installation](installation.md).

## Before you click

- **Lock what must stay.** Select the tracks and vias you routed by hand (power lanes, the
  layout a datasheet asked for, a matched pair), right click, *Locking > Lock*. Locked copper is
  kept and treated as an obstacle; its ends count as connected, so the router attaches to it.
- **Check the net classes.** Track width, clearance and via size come from the net classes in
  the project (*Board Setup > Net Classes*). Custom rules in the `.kicad_dru` are honoured for
  clearances between nets and for minimum track widths, see [concepts](concepts.md#rules).
- **Planes.** Inner layers of type *power* are planes: they are not routed, their pours stay, and
  pins of the plane's net get a via to the plane instead of a track.
- **Save the project once** so the `.kicad_pro` with the net classes exists next to the board.

## The dialog

| Option | Default | What it does |
| --- | --- | --- |
| Keep locked tracks and vias | on | Locked copper stays and is routed around. All unlocked tracks and vias are replaced. Off: everything is replaced, locked or not. |
| Parallel runs | 14 | How many routing attempts with different orders are tried in every phase. The default is a constant, not the core count, so the same board gives the same result on every machine; the core count only changes how long it takes. More runs explore more orders. |
| Via cost (mm of track) | 3.0 | How much track a via is worth to the router. Lower values give more vias and shorter tracks, higher values fewer vias. The search also tries values around it. |
| Geometry generations | 3 | Extra search rounds judged by the clearance check of the drawn tracks, not only by the topology. 0 is fastest; each round costs about one full routing pass and removes violations. |
| Save an animation of the generations | off | Writes one image per generation and an `evolution.gif` into `gummiband-evolution/` next to the project. Needs `rsvg-convert` and ImageMagick. |

Press OK and a progress dialog shows the phase (portfolio, generation n, geometry, geometry
generation n, polish), the routed connections, vias, track length and the remaining violations of
the best candidate so far. *Cancel* stops the router and leaves the board as it was.

## After the run

A message box reports what was removed and created, the router's summary (routed connections,
vias, length, violations) and the connections it could not route, if any.

1. **Look at the result.** Tracks are any-angle with arcs; this looks different from grid
   routing and is intended.
2. **Refill zones** (B). The plugin does not refill them, so pours and plane connections show up
   only after the refill.
3. **Run the DRC.** Gummiband checks clearances itself, but KiCad's DRC is the judge. Expect a
   small number of clearance items on dense boards in this early version; the deficits are
   usually a few micrometres and quick to fix by nudging a track.
4. **Finish by hand** what is unrouted, lock what you want to keep, and route again if you like.
   Every run replaces only unlocked copper, so you can iterate: lock the good parts, reroute the
   rest.
5. **Undo** with Ctrl/Cmd+Z takes the whole routing back in one step.

## Finish selected nets (follow-me, experimental)

The second toolbar button, **Finish selected nets with Gummiband**, completes the nets you point
at instead of rerouting the board:

1. Start a track by hand if you like, or just select any track, via or pad of the nets you want
   finished (several nets at once are fine).
2. Click the button. Without a selection the plugin asks for one.
3. All copper on the board stays, locked or not. The router adds only what is missing on the
   selected nets, continuing a started track from where it ends: copper that touches no pad of its
   net counts as a pin, so the new track starts right there.
4. The additions arrive as one undo step. There is no dialog: the action runs with the defaults
   (14 runs, via cost 3.0) and without geometry generations, so it is quick.

This is the plugin's form of `gummiband route --keep-all --nets A,B`. It is new and not yet
tested live in KiCad, and new tracks keep too little clearance to existing tracks for now, so run
the DRC on the result.

## Without a dialog

If the plugin's Python has no wxPython (a Linux system interpreter, for example), the action runs
with the defaults from the table above and prints its report to the status bar (KiCad 10.0.1 and
later). Use the [command line](cli.md) for full control in that case.

## What the plugin does not do yet

- Route a selection of the board. *Route with Gummiband* replaces all unlocked copper; lock the
  copper you want to keep. *Finish selected nets* adds to existing copper but is experimental.
- Set per net priorities or skip nets. Priorities come from the net classes; the MCP server and
  the CLI report offer more control, see [workflows](workflows.md#route-by-priority).
- Move parts. `--move-parts` is a command line option for now, because moved footprints have to
  be written back to the board file.
