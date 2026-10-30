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
| `fine_pitch` | 2 | QFN-56 at 0.4 mm with 0.2/0.19 mm rules, nets in pin order: a clean straight fan-out |
| `fine_pitch_mixed` | 2 | the same part with shuffled nets: vias and crossings right at the pins, the Corne problem in small |
| `diff_pairs` | 2 | 8 differential pairs (`_P`, `_N`) with a ground between them, 0.5 mm connectors at an angle, parts in the way |

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

## Relaxation on the CPU and the GPU

`cargo run --release -p gummiband-cli --features gpu --example relax_bench -- BOARD` routes the
board with a small search and relaxes the same routing on the CPU and the GPU. It prints the time,
the core seconds, a breakdown per round, the violations and a hash of the result. `--ensemble N`
relaxes it N times with a margin changed by next to nothing: the relaxation is chaotic, a single
violation count says little. Compare core seconds when other jobs run on the machine.

## Real boards (external)

`kicad-autorouter-benchmarks.zip` (not in the repository; sources and licenses in its
`QUELLEN.md`) holds four fully routed open hardware boards. `prepare-ext.sh [zip]` unpacks it
into `work/ext/src/` and strips every track, via and pour outside the plane layers into
`work/ext/<name>/`; the plane layers are marked as KiCad `power` layers in the stripped file, so
the board describes itself. `run-ext.sh NAME [route args]` routes one board into
`work/ext/routed/NAME/` and prints the KiCad DRC summary.

| Name | Board | Routing layers | Planes kept | Notes |
| --- | --- | --- | --- | --- |
| `corne` | Corne Cherry hotswap (panel of two halves) | F, B | none | Edge.Cuts with mouse bites and T joints: the outline falls back to the bounding box with the edge lines as obstacles; GND is routed as a net |
| `arty` | Arty MPW Tester | F, B | In1 (GND), In2 (split power) | cut-out in the middle, copper logos |
| `ulx3s` | ULX3S (ECP5 BGA 381, 0.8 mm) | F, B | In1 (split power), In2 (GND) | the original also routes on In2 |
| `jetson` | Jetson Orin baseboard | F, In3, In5, B | In1, In2, In4, In6 | 1927 connections, diff pairs, custom rules |

First results (2026-10-29, `--runs 14 --generations 1`, no geometry generations):

| Board | Routed | Vias | Time | Main problems |
| --- | --- | --- | --- | --- |
| `arty` | 322/323 | 179 | 10 s | two tracks on top of each other along the left edge, FTDI area |
| `corne` | 471/482 | 453 | 136 s | the two RP2040 fan-outs (0.4 mm pitch) |
| `ulx3s` | 651/796 | 377 | 304 s | BGA escape on two signal layers, bundles along the board edge |
