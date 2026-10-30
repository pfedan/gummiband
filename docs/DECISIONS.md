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

- Placement is fixed by default. `--move-parts` is opt-in; locked footprints, connectors,
  modules and mounting holes stay. Full placement re-searches are used sparingly.
- Necking goes down to the pad width at most, never below the board minimum or what custom
  rules allow; current paths must stay electrically sensible.
- Completion comes first: the geometry stage never trades a connection for cleaner geometry.
- A block is a group of parts with its locked copper that moves only as a whole.
- Layer preferred directions are a portfolio dimension, not only an option.
- Teardrops come from KiCad's built-in feature.

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
