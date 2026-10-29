# Benchmarks

## Synthetic suite

`gen_boards.py` (plain Python 3) writes small KiCad 10 boards of increasing difficulty into
`boards/`, each as `<name>.kicad_pcb` plus `<name>.kicad_pro` (net classes, rules). The boards are
deterministic (fixed seeds, UUIDs derived from the board name); rerun the generator after changing
it and commit the boards together with it. Every unrouted board passes `kicad-cli pcb drc` with
only unconnected items.

| Board | Layers | What it tests |
| --- | --- | --- |
| `easy` | 2 | a few 2 pin connections between 0603 parts |
| `medium` | 2 | SOIC-14, SOT-23 and 0603 parts, random local netlist, an 8 pin GND net |
| `bus` | 2 | 16 bit bus between two 0.5 mm FPC connectors with a 90 degree turn, GND across the bus, 45 degree parts in the bend |
| `power_mix` | 2 | 1 mm (VIN, GND) and 0.75 mm (VOUT) nets with 0.2 mm signals through a 3.5 mm channel between two THT walls |
| `crossing` | 2 | two 1 mm pitch SMD connectors with reversed pin order |
| `qfn_fanout` | 4 (In1 GND, In2 3V3 planes) | QFN-48 at 0.5 mm to a ring of 40 pull resistors, plane fan-out vias |
| `four_signal_layers` | 4 signal | dense random netlist around two QFN-32 and two SOIC-14 |
| `bga_escape` | 4 (planes) | 8x8 BGA at 0.8 mm, 48 signals escape to four edge connectors, 0.1/0.1 mm rules |
| `bga_dense` | 4 (planes) | 10x10 BGA at 0.8 mm, 3 signal rings (84 signals): the inner ring needs vias on 2 signal layers |

`run_suite.sh [board ...]` builds the CLI, routes every board into `boards/out/`
one board after another (`--runs 4 --generations 1` with `RAYON_NUM_THREADS=4`, override with
`RUNS`, `GENERATIONS`, `RAYON_NUM_THREADS`, `ROUTE_ARGS`, `TIMEOUT`), runs
`kicad-cli pcb drc --refill-zones` on each result (next to a copy of the board's `.kicad_pro`) and
prints a table: connections routed, vias, length, router wall time, Gummiband's own clearance
check, and the KiCad DRC counts (unconnected, clearance, shorts, crossings, other; silk and lib
types ignored). A crash or timeout on one board is reported in the table and the notes and does
not stop the suite. The table is also written to `boards/out/results.txt`; the router output of
each board is in `boards/out/<board>.route.log`.

## Lichtschnitzel

`prepare-lichtschnitzel.sh` strips the real Lichtschnitzel controller board,
`run-lichtschnitzel.sh` routes it and prints the KiCad DRC summary; `drc_report.py` summarizes a
DRC JSON report in more detail.
