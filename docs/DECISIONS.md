# Decisions and working rules

Standing decisions of the project owner, kept here so that they survive any session.

## Goals

- Topological rubber band autorouter for KiCad: Rust core, multicore, KiCad plugin, very good
  interface for AI agents. Own design, inspired by TopoR, no foreign code.
- Smart algorithms and heuristics that get close to the optimum quickly.
- The aim on real boards is parity with the source design at the same track widths
  (`--net-widths` from `bench/net_widths.py`), not outsmarting the source.

## Benchmarks

- the LED controller is routed completely by Gummiband: all tracks, vias and outer
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
  any thread count (fixed default portfolio, no decisions by the clock). This rule holds for runs
  without an open end. For it the workspace has its own maths (2026-11-10): `sin`, `cos`,
  `atan2`, `acos`, `exp`, `ln`, `powf`, `hypot` and their kin come from `gummiband_core::math`
  (the `libm` crate, pure Rust), never from the system's maths library, whose last bits differ
  between glibc versions, musl and macOS and changed the routed file with them. `hypot` is the
  exception: it is `(x * x + y * y).sqrt()`, exactly as written, so that a length agrees with
  the squared distances the router compares elsewhere. A correctly rounded hypot differs from
  that in the last bit at about one point in ten, which was enough to change clearance decisions
  (Corne keyboard, 7 seeds, macOS: median 468 violations with the plain formula's bits, 797
  with the library's). A test keeps
  the methods of `f64` that go to the system out of the sources, `powi` too. Measured: the glibc
  and the musl build on x86_64 Linux give the same files (`bench/README.md`, route hashes);
  macOS on ARM is expected to, and still to be measured. The reference results stayed those of
  the musl build (no hash changed); a new version of the crate that changes a bit may change
  them. An open ended run (`--until-stop`) ends when the user says so; there
  keeping the cores busy comes first, and its results may change with the way the mutants are
  scheduled, also with the timing. Its present scheduling still repeats the same seed and last
  generation at any thread count.
- Late search works on the result, not on the routing order. Moving a connection in the order
  routes the whole board again and loses what is good, so an open ended run takes out a few
  connections around one spot, lays them again and relaxes what they touch. Bolder side branches
  may first get worse; a connection is never given up, and one more counts only if it does not
  bring two shorts.
- Wider tracks are a step after the routing (`--prefer-width`), not a higher class width: the
  topology keeps the narrow capacity, so the search is not harder, and a track only grows where
  the finished copper leaves room. Classes other than the default are left alone.
- In the open end a violation is a spot, weighed by what is missing. Counting pieces of track
  made the search chase the number of pieces in a curve. Plain runs keep their count.
- The open end runs combination A by default (2026-11-01): the via price of a move drawn by a
  bandit, a quarter of the moves aimed at expensive connections, hopeless mutants dropped before
  their relaxation. The night series measured 12 % and 15 % less cost after 20 minutes on
  LED controller and Arty, KiCad DRC clean in 5 of 7 runs against 3 of 7. Each part keeps a switch
  to turn it off; the other ideas of that night stay off.
- The algorithms use the freedoms a board leaves them by default, a switch only turns one off
  (Daniel, 2026-11-01). Everything that is not locked may give way within the rules. First step:
  finishing nets (`--keep-all --nets`, MCP `keep_all` with `nets`) shoves the unlocked tracks of
  the other nets aside, and if a connection still finds no way, the nets near its pins give way
  within 8 mm; the try with the fewest open connections is kept, so giving way never costs a
  connection. In the night series of 2026-11-01 plain left 4 of 16 re-laid nets open, shove none.
  Second step: the last step tapers a track at a spot that misses its clearance by a few
  micrometres, down to the minimum width of its class and never wider elsewhere. A class without a
  minimum width rule keeps its class width there: the spot stays until the search finds another way
  (Daniel, 2026-11-02, about VBUS_P at U6.10, which 0.25 instead of 0.3 mm would have cleared). A
  neck that a minimum width rule allows in a courtyard is for the pin there, not for the current:
  the way between two pins that can take the minimum never passes one (Daniel's question about
  VBUS_F, 2026-11-01), even where KiCad's DRC would let it pass; the neck ends at the edge of the
  courtyard, and a layer change on such a way gets the vias that carry its current (IPC-2221; the
  current check of PR #43 found single 0.4 mm vias on the 4.1 A paths of the finished board).
  Parts move by default since PR #40. Third step: when the whole plain search leaves a few
  connections open (at most 15 %), a second search negotiates and the better result counts; a
  search that completes the board stays as it is. Fourth step: finishing nets open ended, with
  shove and give; the tries of give run without the open end, and the one kept goes on open
  ended. The via in a pad as a gene of the search waits for a board that needs it: once the
  search negotiates, none of the bench boards does (2026-11-01).
- Convergence means that bold changes no longer help, not that some number of rounds went by. When
  it is reached, the user gets the results that differ in vias and length, not one of them.
- Copper does not have to lie at the limit: `--air` gives it a reserve where there is room. On by
  default only in open ended runs, so that plain runs stay as they are.
- Defaults aim at a usable board, not at the quickest run: CLI, MCP server and plugin route
  with 3 geometry generations, which cost about 2 to 3 times the time of none and remove most
  clearance violations. 0 stays available as the quick draft. `place` makes one placement by
  default, because the crossings it selects by say almost nothing about the routed result.
- A block is a group of parts with its locked copper that moves only as a whole.
- Layer preferred directions are a portfolio dimension, not only an option.
- An area (the zones of a net: a pour or a plane) is one model, and its fill is computed, never
  stored. The model holds the outline and the settings of the fill; what the fill is and what
  it joins is computed from the board and its copper each time it is asked for
  (`gummiband_core::fill`), because every track of another net changes it. The fill in a file is
  KiCad's answer for the copper of its last refill and is only ever compared against, never
  taken as the state. The model is the router's own judgement; KiCad's fill and DRC have the
  last word, and a statement of the model says how sure it is (raster step, doubtful pads).
- A pour of the net is copper of the net: a connection that reaches a `gb:` pour ends where it
  enters it. It does not run on to a pad under the pour. Whether the fill of the pour really
  holds its pads is the fill model's to say: for now as a second opinion in the report of a
  route, which changes no count (stage 1); counting such a pad as open comes with the stage in
  which the search itself knows the fill.
- Copper of the file that gave way in a shove and then touches another net goes back to where
  the file has it, and new tracks that cross that place are taken out by the last check. A
  route is not refused because tracks lie inside a pour outline: that is legal in KiCad.
- Teardrops come from KiCad's built-in feature.
- Any-angle tracks with arcs stay the default. Conventional 45 degree routing is an optional
  last step (`--angles 45`, plugin and MCP alike) that never adds a clearance violation and
  leaves a piece at its angle where no 45 degree path fits.
- Follow-me stays "route the selected nets" for now; no live completion while drawing a track.

## Interfaces

- Highest quality standard with maximum usability (Daniel, 2026-11-10). Consolidation merges
  switches that are duplicates or contradict each other; it never removes a way to tune. Each
  knob has one name, one default and one description, the same in the CLI, the MCP server and
  the plugin (`gummiband_app::OPTIONS` for a route; tests hold the front ends to it).
- The MCP server is a toolbox, not an automaton. An agent brings its own knowledge of
  electronics and must be able to do any part of a board by hand: place and lock parts, route
  single nets itself, prescribe widths and currents. It leaves only the rest to the router,
  which leaves existing work untouched when asked to. The KiCad plugin follows the same
  principle for the person at the board.
- Options that contradict each other are errors that say what to change; they are never
  adjusted silently. An option that was left out does not contradict: its default follows from
  the others. The rule lives in one place (`RouteOptions::validate`), so every front end gives
  the same message.

## License

- Unlicensed: proprietary, all rights reserved. No LICENSE file, crates are not published.
  MIT may follow later, but only as a deliberate step by the owner.

## Working rules

- All repository content is in English.
- Outside this repository, read or write only with the owner's permission. Known exceptions:
  the LED controller KiCad folder (benchmark source), the KiCad plugin folder, toolchain paths,
  scratch folders. Ask for paths instead of searching other folders.
- Order of work: converge the current state, then the roadmap at bird's eye level
  (`docs/MILESTONES.md`), then targeted new benchmarks. Micro-optimisation runs independently,
  also on newly added algorithms, and must preserve results byte for byte.
- Refactorings must keep `bench/hashes.txt` unchanged (`bench/hashes.sh --check`, the quick set
  runs in CI). The reference is only rewritten (`--update`) together with a change that is meant
  to alter results, and the commit says so.
- User documentation lives in the main README and `docs/`, user centred, in English.
- Jetson Orin is not a test or benchmark board until the owner explicitly says so.
