# Benchmarks

The `bench/` folder holds boards and scripts to measure the router: a synthetic suite of small
boards of increasing difficulty, the LED controller as a real four layer board, and
four open hardware boards as external real world cases, and a DDR5 board for length matching. Every benchmark routes a board that was
stripped of all its tracks and vias and judges the result with the KiCad DRC. Details of the
scripts are in [bench/README.md](../../bench/README.md).

You need `kicad-cli` on `PATH` and Python 3.

## Synthetic suite

Thirteen generated boards (`bench/boards/`), from a few 0603 connections to a 10x10 BGA whose inner
ring needs vias on two signal layers. They are committed with their `.kicad_pro`, so they work
out of the box:

```bash
bench/run_suite.sh                 # all boards
bench/run_suite.sh bus qfn_fanout  # some of them
```

The script builds the CLI, routes every board into `bench/boards/out/` (4 runs, 1 generation, no
geometry generations, 4 threads by default; override with `RUNS`, `GENERATIONS`,
`GEOMETRY_GENERATIONS`, `RAYON_NUM_THREADS`, `ROUTE_ARGS`,
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
| `matching` | 2 | a byte lane and a clock pair with length, skew and uncoupled rules, see [Length matching](#length-matching) |

`bench/gen_boards.py` regenerates the boards deterministically; commit the boards together with
a change to the generator.

## LED controller

The real reference board: 100 x 60 mm, four layers with signals on the outer layers and GND and
power planes inside, about 350 connections, custom rules. Its KiCad project is not
public; point `LEDCTL_PCB` at the `.kicad_pcb` if you have it.

```bash
LEDCTL_PCB=/path/to/board.kicad_pcb bench/prepare-ledctl.sh   # copies the project and strips all copper into bench/work/ledctl/
bench/run-ledctl.sh [route args]   # routes it, writes bench/work/routed/, prints the DRC summary
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

## Length matching

Two boards carry `length`, `skew` and `diff_pair_uncoupled` rules in their `.kicad_dru`, so the
lengths can be judged by the KiCad DRC (see [Length matching](concepts.md#length-matching)).

- `matching` in the synthetic suite: a byte lane (8 data nets and a strobe pair) and a clock pair
  between two 0.5 mm connectors at an angle. One skew group of 0.2 mm, the clock 40 to 41 mm long
  with 0.05 mm skew inside the pair, 3 mm uncoupled length per pair.
- Antmicro's DDR5 testbed (one DDR5 x8 DRAM on an SO-DIMM edge connector, 6 layers, Apache-2.0),
  fetched at a fixed commit. The board ships without length rules; `bench/ddr5-testbed.kicad_dru`
  adds what its reference routing achieves: one byte lane (DQ0 to DQ7, DQS, TDQS) with 0.75 mm
  skew, one command/address group with the clock and 0.6 mm skew, 6 mm uncoupled length per pair.
  Antmicro's routing passes all of them.

```bash
bench/run_suite.sh matching
bench/prepare-ddr5.sh              # fetches and strips into bench/work/ddr5/
bench/run-ddr5.sh --pairs --no-move-parts   # lengths of the reference, routing, lengths, KiCad DRC
```

Results of 2026-11-01 (`--pairs --runs 4 --generations 1 --no-move-parts`, so the placement
stays as the reference has it). "Meanders only" is the routing with meanders in the room it left,
"room, then meanders" pushes the neighbours aside first:

| Board | Without tuning | Meanders only | Room, then meanders | KiCad DRC with room |
| --- | --- | --- | --- | --- |
| `matching` | 13 of 16 nets or pairs out of range | 11 of 16, 5 of 11 tuned nets reached their length | 6 of 16: clock length 2 of 2, byte lane 6 of 10; 6 of 11 tuned nets reached their length | 0 unconnected, 0 clearance, 5 skew (10 with meanders only), 1 uncoupled (2) |
| DDR5 testbed | 24 of 29: byte lane 3 of 12, command/address 1 of 14, 22 skew errors in the KiCad DRC | 16 of 29: byte lane 7 of 12, command/address 5 of 14, uncoupled 1 of 3; 7 of 22 tuned nets reached their length | 10 of 29: byte lane 8 of 12, command/address 9 of 14, uncoupled 2 of 3; 14 of 23 tuned nets reached their length | 261 of 263 routed, 0 clearance, 9 skew (14), 6 pair gap (22), 1 uncoupled (3), 0 sliver (1) |

In `bench/compare.py` against main (default arguments, so without `--pairs` and with moving
parts) every board has the same KiCad DRC result; the LED controller file is byte-identical
(the step only runs with length rules). On `matching` it makes room for 6 nets but the meanders
reach the same lengths as before there.

The room step widens each net that is too short to the width of the meander it needs and lets
the geometry step push the neighbours aside; what would add a clearance error is undone, in
halves, so the nets that find room keep it. It takes 45 s on `matching` and 80 s on DDR5 (on a
busy machine). The misses that remain on DDR5 are nets in the middle of a bundle whose
neighbours cannot move further (CA7, CA9, DQ1, DQ3), CA5 that is not routed, and the strobe pair,
which the longest net of the byte lane (pulled long by its detour) leaves far behind. The clock
pair's own skew (0.7 mm on `matching`) is a matter of phase matching, which is not there yet.

## Conventional 45 degree tracks

`--angles 45` on the same routing as without it (`--runs 14 --generations 1`, 2026-10-31, 4
cores). Length is the track length of the drawn copper before and after the step; off the grid is
what kept its angle because no 45 degree path fit (with the necks that a minimum width rule keeps
as they are). KiCad DRC counts without the types that the unrouted board already has.

| Board | Length | Off the grid | Vias | KiCad DRC any-angle | KiCad DRC 45 degrees | Time of the step |
| --- | --- | --- | --- | --- | --- | --- |
| LED controller | 2387 to 2493 mm (+4.4 %) | 127 mm (5 %) | 278, same | 16 clearance | 16 clearance | 0.8 s |
| Arty | 2813 to 2932 mm (+4.2 %) | 148 mm (5 %) | 136, same | 4 hole clearance, 1 edge, 1 sliver | 4 hole clearance, 1 edge | 0.4 s |
| Corne | 7719 to 8082 mm (+4.7 %) | 328 mm (4 %) | 374, same | 499 clearance, 1 crossing, 1 short, 1 hole, 5 slivers | 499 clearance, 1 hole | 1.3 s |

The step adds no DRC error on any of them; where an old piece already had one it can only stay or
go away.

`gummiband angles45` does the same afterwards on the written any-angle files of this table,
without routing again. It sees the copper as the file has it: runs of track between vias,
branches and pads, with the necks below a minimum width left alone, so the numbers differ a
little from the step inside `route`.

| Board | Length | Off the grid | Runs changed | Vias | KiCad DRC any-angle | KiCad DRC afterwards | Time |
| --- | --- | --- | --- | --- | --- | --- | --- |
| LED controller | 2348 to 2446 mm (+4.2 %) | 60 mm (2.5 %) | 606 of 626 | same | 16 clearance | 16 clearance | 1.1 s |
| Arty | 2818 to 2934 mm (+4.1 %) | 148 mm (5 %) | 431 of 444 | same | 4 hole clearance, 1 edge, 1 sliver | 4 hole clearance, 1 edge | 2.0 s |
| Corne | 7846 to 8086 mm (+3.1 %) | 308 mm (4 %) | 2511 of 3139 | same | 499 clearance, 1 crossing, 1 short, 1 hole, 5 slivers | 499 clearance, 1 hole | 2.8 s |

## Compare two versions

```bash
bench/compare.py OLD_BIN NEW_BIN [--boards a,b,...] [--args "route args"] [--out DIR]
```

Routes every board of the suite, the prepared external boards and the LED controller board with
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

## Relaxation on the CPU and the GPU

```bash
cargo run --release -p gummiband-cli --features gpu --example relax_bench -- BOARD [--ensemble N]
```

Routes the board with a small search and relaxes the same routing on the CPU and on the GPU,
printing time, core seconds, a breakdown per round, the violations and a hash of the result.
Relaxation is chaotic (a change far below a nanometre can turn one short into many), so a single
violation count says little: `--ensemble N` relaxes N times with a tiny change each and compares
the distributions. Compare core seconds rather than wall time when other jobs run on the machine.

## Placement from the netlist

`gummiband place` on the LED controller with the connectors, the module, holes, buttons
and test points fixed, 133 parts from nothing, original outline and inner planes. Routed with
`--geometry-generations 3 --move-parts`, judged with the KiCad DRC (measured on 2026-10-30):

| Placement | Routed | Vias | Length | KiCad DRC |
| --- | --- | --- | --- | --- |
| Original (by hand) | 348/348 | 294 | 2633 mm | clean |
| Seed 1 (default) | 348/348 | 232 | 2088 mm | clean |
| Seed 11 | 348/348 | 217 | 2229 mm | clean |
| Seed 31 | 348/348 | 205 | 2077 mm | clean |
| Seed 41 | 348/348 | 234 | 2167 mm | 2 slivers |
| Seed 21 | 347/348 | 240 | 2077 mm | 2 unconnected, 1 sliver, 1 dangling via |
| Seed 51 | 350/350 | 289 | 2044 mm | 8 clearance, 1 short, 1 crossing, 1 mask bridge |

The seeds of this table are from before seeds stopped sharing starts.

Fewer vias and shorter tracks than the hand placement, but the seeds differ, so try a few. The
placer knows no datasheet layouts: switch nodes come out 3.4 to 7.4 mm long against 3.0 mm in the
original. Place a power stage by hand or lock it when that matters.

Twelve seeds on a later version, 4 threads: the quality run takes 39 s on average, 9 of 12 are clean
in the KiCad DRC, one more has only a sliver, the best has 208 vias. A short run (`--runs 4
--generations 1`, about 5 s) ranks the placements only weakly like the quality run (rank correlation
0.52 for the overall result, 0.53 for vias, 0.03 for length), and the crossing count of `place` not
at all (-0.01). So `--placements` judges with the settings of the route itself.

## Placement variants

`bench/place/make-variants.sh` and `route-variants.sh` build and route the LED controller
controller in a smaller rectangle, a square and a circle with the `variants` example of the
`gummiband-place` crate, as extra boards for the search. See
[bench/place/README.md](../../bench/place/README.md).
