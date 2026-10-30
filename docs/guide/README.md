# Gummiband user guide

Gummiband routes KiCad boards with any-angle tracks and arcs, planned as topology first and
pulled taut afterwards. This guide is for people who want to route their boards with it. The
[main README](../../README.md) has the short setup and quickstart; the chapters here go into
detail.

| Chapter | Read it when |
| --- | --- |
| [Installation](installation.md) | you set up the binary, the KiCad plugin or the optional tools |
| [Routing in KiCad](kicad-plugin.md) | you route from the PCB editor and want to know what the dialog options do |
| [Command line](cli.md) | you route from a shell or a script, or need the JSON report |
| [Workflows](workflows.md) | you want recipes: locking power lanes, priorities, judging results, movable parts, animations |
| [How Gummiband routes](concepts.md) | you want to understand what the router respects, how it decides, and its current limits |
| [Benchmarks](benchmarks.md) | you want to measure a change or compare against other routers |

For AI agents the router also speaks the Model Context Protocol, see [MCP.md](../MCP.md). The
internals are in [ARCHITECTURE.md](../ARCHITECTURE.md), the roadmap in
[MILESTONES.md](../MILESTONES.md), the standing decisions of the project in
[DECISIONS.md](../DECISIONS.md).

## Vocabulary

A few words this guide uses throughout:

- **Connection**: one pin to pin link the router has to make. A net with n pins has n - 1
  connections; a net that is a plane counts one connection per pin that has to reach the plane
  (a fan-out).
- **Locked copper**: tracks and vias with KiCad's lock flag. Gummiband never changes them and
  routes around them; their ends count as already connected copper the router may attach to.
- **Plane**: an inner copper layer that holds a pour of one net (GND, a supply) and is never
  routed. Layers of KiCad type *power* are planes; others can be declared with `--plane-layers`.
- **Run**: one complete routing attempt with its own order of connections. Gummiband runs many
  in parallel (the portfolio) and keeps the best.
- **Generation**: one round of the evolutionary search that mutates the best routing order and
  tries the mutants in parallel.
- **Geometry generation**: the same, but judged after the tracks were drawn, by the clearance
  check. Slower and cleaner.
- **Violation**: a clearance problem the router's own check finds in its result. KiCad's DRC is
  the final judge; the numbers are close but not identical.
