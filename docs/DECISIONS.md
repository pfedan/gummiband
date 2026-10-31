# Decisions and working rules

Standing decisions of the project owner, kept here so that they survive any session.

## Goals

- Topological rubber band autorouter for KiCad: Rust core, multicore, KiCad plugin, very good
  interface for AI agents. Own design, inspired by TopoR, no foreign code.
- Smart algorithms and heuristics that get close to the optimum quickly.
- The aim on real boards is parity with the source design at the same track widths
  (`--net-widths` from `bench/net_widths.py`), not outsmarting the source.

## Benchmarks

- The Lichtschnitzel controller is routed completely by Gummiband: all tracks, vias and outer
  pours stripped, also locked ones. Placement, outline, keepouts and inner planes stay.
  Support for locked copper stays as a feature for manual pre-routing.
- Avoid overfitting: judge every change across all boards with `bench/compare.py`, with the
  KiCad DRC as the judge. A change must win in sum and lose nowhere badly.
- Runs must not distort each other: one router run at a time, or split the cores explicitly.
  Timing numbers only from runs alone on the machine.
- Relaxation and search changes are judged over several seeds (`GB_EVO_SEED`).

## Router behaviour

- Placement is a step of its own before routing (`gummiband place`), for people through a plugin
  button with the choice of parts (selected, outside the board, all unlocked), for agents through
  `place_parts` next to their own `move_part`. Locked footprints never move.
- Placement is fixed by default while routing. A search over whole placements (`--placements`) is
  opt-in and judges by full routes. `--move-parts` is opt-in; locked footprints, connectors,
  modules and mounting holes stay. Full placement re-searches are used sparingly.
- Necking goes down to the pad width at most, never below the board minimum or what custom
  rules allow; current paths must stay electrically sensible.
- Completion comes first: the geometry stage never trades a connection for cleaner geometry.
- Results are deterministic: the same input and seed give the same output on every machine, at
  any thread count (fixed default portfolio, no decisions by the clock). An open ended run
  (`--until-stop`) ends when the user says so, the same seed and last generation repeat.
- Late search works on the result, not on the routing order. Moving a connection in the order
  routes the whole board again and loses what is good, so an open ended run takes out a few
  connections around one spot, lays them again and relaxes what they touch. Bolder side branches
  may first get worse; a connection is never given up, and one more counts only if it does not
  bring two shorts.
- Wider tracks are a step after the routing (`--prefer-width`), not a higher class width: the
  topology keeps the narrow capacity, so the search is not harder, and a track only grows where
  the finished copper leaves room. Classes other than the default are left alone.
- Copper does not have to lie at the limit: `--air` gives it a reserve where there is room. On by
  default only in open ended runs, so that plain runs stay as they are.
- Defaults aim at a usable board, not at the quickest run: CLI, MCP server and plugin route
  with 3 geometry generations, which cost about 2 to 3 times the time of none and remove most
  clearance violations. 0 stays available as the quick draft. `place` makes one placement by
  default, because the crossings it selects by say almost nothing about the routed result.
- A block is a group of parts with its locked copper that moves only as a whole.
- Layer preferred directions are a portfolio dimension, not only an option.
- Teardrops come from KiCad's built-in feature.
- Follow-me stays "route the selected nets" for now; no live completion while drawing a track.

## License

- Unlicensed: proprietary, all rights reserved. No LICENSE file, crates are not published.
  MIT may follow later, but only as a deliberate step by the owner.

## Working rules

- All repository content is in English.
- Outside this repository, read or write only with the owner's permission. Known exceptions:
  the Lichtschnitzel KiCad folder (benchmark source), the KiCad plugin folder, toolchain paths,
  scratch folders. Ask for paths instead of searching other folders.
- Order of work: converge the current state, then the roadmap at bird's eye level
  (`docs/MILESTONES.md`), then targeted new benchmarks. Micro-optimisation runs independently,
  also on newly added algorithms, and must preserve results.
- User documentation lives in the main README and `docs/`, user centred, in English.
- Jetson Orin is not a test or benchmark board until the owner explicitly says so.
