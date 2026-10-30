# Benchmarks

The `bench/` folder holds boards and scripts to measure the router: a synthetic suite of small
boards of increasing difficulty, the Lichtschnitzel controller as a real four layer board, and
four open hardware boards as external real world cases. Every benchmark routes a board that was
stripped of all its tracks and vias and judges the result with the KiCad DRC. Details of the
scripts are in [bench/README.md](../../bench/README.md).

You need `kicad-cli` on `PATH` and Python 3.

## Synthetic suite

Twelve generated boards (`bench/boards/`), from a few 0603 connections to a 10x10 BGA whose inner
ring needs vias on two signal layers. They are committed with their `.kicad_pro`, so they work
out of the box:

```bash
bench/run_suite.sh                 # all boards
bench/run_suite.sh bus qfn_fanout  # some of them
```

The script builds the CLI, routes every board into `bench/boards/out/` (4 runs, 1 generation, 4
threads by default; override with `RUNS`, `GENERATIONS`, `RAYON_NUM_THREADS`, `ROUTE_ARGS`,
`TIMEOUT`), runs the KiCad DRC on each result and prints a table: connections routed, vias,
length, wall time, the router's own violations and the DRC counts (unconnected, clearance,
shorts, crossings, other). The table is also in `bench/boards/out/results.txt`, the router output
per board in `bench/boards/out/<board>.route.log`.

| Board | Layers | What it tests |
| --- | --- | --- |
| `easy` | 2 | a few 2 pin connections between 0603 parts |
| `medium` | 2 | SOIC-14, SOT-23 and 0603 parts, random local netlist, an 8 pin GND net |
| `bus` | 2 | 16 bit bus between two 0.5 mm FPC connectors with a 90 degree turn |
| `power_mix` | 2 | 1 mm and 0.75 mm power nets with 0.2 mm signals through a 3.5 mm channel |
| `crossing` | 2 | two 1 mm pitch SMD connectors with reversed pin order |
| `qfn_fanout` | 4 (2 planes) | QFN-48 at 0.5 mm to a ring of 40 resistors, plane fan-out vias |
| `four_signal_layers` | 4 signal | dense random netlist around two QFN-32 and two SOIC-14 |
| `bga_escape` | 4 (2 planes) | 8x8 BGA at 0.8 mm, 48 signals to four edge connectors, 0.1/0.1 mm rules |
| `bga_dense` | 4 (2 planes) | 10x10 BGA at 0.8 mm, 84 signals, inner ring needs vias on 2 signal layers |
| `fine_pitch` | 2 | QFN-56 at 0.4 mm with 0.2/0.19 mm rules, nets in pin order: a clean straight fan-out |
| `fine_pitch_mixed` | 2 | the same part with shuffled nets: vias and crossings right at the pins |
| `diff_pairs` | 2 | 8 differential pairs with a ground between them, 0.5 mm connectors at an angle, parts in the way |

`bench/gen_boards.py` regenerates the boards deterministically; commit the boards together with
a change to the generator.

## Lichtschnitzel controller

The real reference board: 100 x 60 mm, four layers with signals on the outer layers and GND and
power planes inside, about 350 connections, custom rules. It needs the Lichtschnitzel repository
next to this one (`LICHTSCHNITZEL=/path/to/repo` otherwise).

```bash
bench/prepare-lichtschnitzel.sh   # copies the project and strips all copper into bench/work/lichtschnitzel/
bench/run-lichtschnitzel.sh [route args]   # routes it, writes bench/work/routed/, prints the DRC summary
```

The stripped board keeps placement, outline, keepouts and the inner planes; everything else,
including the locked scripted copper and the outer pours, is routed from scratch.

## External real boards

Four fully routed open hardware boards (Corne keyboard, Arty MPW tester, ULX3S FPGA board,
Jetson Orin baseboard) are stripped and routed the same way. The archive
`bench/kicad-autorouter-benchmarks.zip` with sources and licenses is not in the repository.

```bash
bench/prepare-ext.sh [zip]        # unpacks and strips into bench/work/ext/<name>/
bench/run-ext.sh corne [route args]
```

The plane layers are marked as *power* layers in the stripped files, so the boards describe
themselves.

## Compare two versions

```bash
bench/compare.py OLD_BIN NEW_BIN [--boards a,b,...] [--args "route args"] [--out DIR]
```

Routes every board of the suite, the prepared external boards and the Lichtschnitzel board with
each binary in turn, alone on the machine, then runs the KiCad DRC. Violations the unrouted board
already has (courtyards, padstacks) are subtracted per type. The table shows completion, routing
caused DRC errors and time per board, then the totals. A change should win in sum and lose
nowhere badly. The same binary twice gives a status table of one run per board.

## Ground truth of an existing routing

```bash
bench/net_widths.py ORIGINAL.kicad_pcb [--json widths.json]
```

Prints the total track length, via count and the width the designer used per net (length
weighted). With `--json` it writes the widths for `gummiband route --net-widths`, so a
re-routing can be compared at equal widths.

## Placement variants

`bench/place/make-variants.sh` and `route-variants.sh` build and route the Lichtschnitzel
controller in a smaller rectangle, a square and a circle with `gummiband-place`, as extra
boards for the search. See [bench/place/README.md](../../bench/place/README.md).
