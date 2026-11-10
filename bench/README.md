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
| `matching` | 2 | a byte lane and a clock pair between two 0.5 mm connectors at an angle, with `length`, `skew` and `diff_pair_uncoupled` rules |
| `pair_wall` | 2 | a USB pair through the one gap of a wall of THT pins (the gap takes the wide pair or three single tracks, between the pins only one single track fits), six short nets across the wall next to the gap |
| `pair_traffic` | 2 | two long pairs from a connector to a QFN-32 across a field of 0603 parts with random local nets |
| `parts_easy` | 2 | part to part, level 1: QFN-32 and SPI flash with a planar bus, series resistors to a header |
| `parts_medium` | 2 | level 2: SPI flash in its real pinout (bus on both sides of the flash), I2C sensor with pull-ups and an interrupt, 6 bit bus with one swap, random local nets |
| `parts_hard` | 2 | level 3: QFN-48 with shared SPI to two devices, shared I2C with pull-ups to two more, 8 bit bus with two swaps to an edge connector, USB pair, random local nets |
| `parts_multi` | 4 (planes) | level 4: MCU (QFN-48) and PHY (QFN-32) with a 12 net bus with swaps, SPI flash, 6 bit latch bus, I2C shared by both chips with pull-ups, three pairs, a debug header wired to both, random local nets |
| `parts_multi_feed` | 4 (planes) | level 5: `parts_multi` with 40 scattered 0603 parts (chicken feed), half of them with a plane via |
| `parts_xhard` | 4 (planes) | extra hard: `parts_multi_feed` on a board 25 % smaller in each direction (positions scaled, edge connectors at their edge, overlaps spread apart) |

`run_suite.sh [board ...]` builds the CLI, routes every board into `boards/out/`
one board after another (`--runs 4 --generations 1 --geometry-generations 0` with `RAYON_NUM_THREADS=4`, override with
`RUNS`, `GENERATIONS`, `GEOMETRY_GENERATIONS`, `RAYON_NUM_THREADS`, `ROUTE_ARGS`, `TIMEOUT`), runs
`kicad-cli pcb drc --refill-zones` on each result (next to a copy of the board's `.kicad_pro`) and
prints a table: connections routed, vias, length, router wall time, Gummiband's own clearance
check, and the KiCad DRC counts (unconnected, clearance, shorts, crossings, other; silk and lib
types ignored). A crash or timeout on one board is reported in the table and the notes and does
not stop the suite. The table is also written to `boards/out/results.txt`; the router output of
each board is in `boards/out/<board>.route.log`.

## Route hashes (regression test)

`hashes.sh` routes the synthetic boards with fixed settings and prints one line per run: the name
and the SHA-1 of the routed `.kicad_pcb`. Runs without an open end are deterministic (same input
and seed, any thread count), so the list is a regression test: a refactoring or a
micro-optimisation must leave it unchanged. Only the router runs, no `kicad-cli` is needed.

The same machine, that is. The router calls `sin`, `cos`, `atan2`, `acos`, `exp`, `log`, `pow` and
`hypot` of the system's maths library, and their last bits differ between glibc versions and
macOS: the first CI run (glibc of Ubuntu 26.04) changed 12 of 15 hashes of a reference made with
glibc 2.39, with the same compiler. So the reference files hold the hashes of the static musl
build, which brings its own maths library and gives the same hashes on every x86_64 Linux; CI
checks that build. On a Mac, compare two binaries instead.

```sh
# x86_64 Linux: against the reference
rustup target add x86_64-unknown-linux-musl
cargo build --release -p gummiband-cli --target x86_64-unknown-linux-musl
bench/hashes.sh --check             # all boards against hashes.txt, about 5 min at 2 threads
QUICK=1 bench/hashes.sh --check     # the quick set against hashes.quick.txt, what CI runs

# any machine: the build before a change against the build after it
bench/hashes.sh --compare path/to/old/gummiband target/release/gummiband
QUICK=1 bench/hashes.sh --compare OLD NEW

bench/hashes.sh easy bus            # print the hashes of some boards, compare nothing
```

Every board is routed with `--runs 4 --generations 1 --geometry-generations 0 --no-move-parts`.
A few extra runs on small boards cover option paths the plain run does not touch (the defaults,
`--angles 45`, `--pairs`, `--pair-pull`, `--patterns groups`, `--negotiate`); their names carry a
`+`, as in `medium+45`. `--check` prints a diff and exits with 1 when a hash differs, and a run
that fails shows up as `FAILED` in place of its hash. `RAYON_NUM_THREADS` sets the threads
(default 4) and changes the time only.

A change that is meant to alter results (a new heuristic, a changed default) is judged with
`compare.py` and the KiCad DRC as before, and then the reference is updated deliberately, in the
same commit as the change and with the reason in its message:

```sh
bench/hashes.sh --update            # rewrites hashes.txt
QUICK=1 bench/hashes.sh --update    # rewrites hashes.quick.txt
```

Never update the reference to make a refactoring pass: there a changed hash is a bug. A new board
from `gen_boards.py` needs `--update` too, since the full list covers every board in `boards/`.

## Two sided placement

`gen_two_sided.py` writes four synthetic boards into `boards/two_sided/` for `gummiband place
--two-sided` (placement only, no routing): `ts_som` (4 layers, 42 x 30 mm, BGA SoC and memory,
PMIC, about 120 0402 parts, a board to board connector locked on the back; the parts need about
1.4 times the front), `ts_sensor` (34 x 23 mm, a coin cell holder locked on the back, an antenna
keepout on the front: one sided nothing fits, two sided the front holds it all), `ts_roomy` (a
control with room to spare) and `ts_tht` (through hole electrolytics and a terminal block between
SMD converters). `check_sides.py PLACED [--input ORIGINAL]` checks a result from the file alone:
courtyard overlaps per side, the holes of through hole parts against parts on the other side, SMD
pads and courtyards on the layers of their side, mirrored texts on the back, and with `--input`
that every part that changed sides keeps its pad distances and is mirrored. `draw_sides.py PLACED
OUT.svg` draws front and back next to each other.

With `--iters 1000000`, one start, seed 1:

| Board | One sided | Two sided: parts on the back | Wall time |
| --- | --- | --- | --- |
| `ts_som` | fails before the search, 321 mm2 missing | 105 | 20 s |
| `ts_sensor` | fails before the search, 275 mm2 missing | 0 (the front holds it all once the holder only takes the back) | 6 s |
| `ts_roomy` | places | 9 (decoupling capacitors under their IC) | 3 s |
| `ts_tht` | places | 10, some under the electrolytics | 3 s |

`check_sides.py` finds no problem on any of them.

## LED controller

`prepare-ledctl.sh` strips the real LED controller board,
`run-ledctl.sh` routes it and prints the KiCad DRC summary; `drc_report.py` summarizes a
DRC JSON report in more detail.

## Relaxation on the CPU and the GPU

`cargo run --release -p gummiband-cli --features gpu --example relax_bench -- BOARD` routes the
board with a small search and relaxes the same routing on the CPU and the GPU. It prints the time,
the core seconds, a breakdown per round, the violations and a hash of the result. `--ensemble N`
relaxes it N times with a margin changed by next to nothing: the relaxation is chaotic, a single
violation count says little. Compare core seconds when other jobs run on the machine.

## Profiling

`GB_TIMING=1` prints cumulative phase timers of a route (search, relaxation rounds, taut, check;
parallel calls add up, so the shares are of the work, not of the clock). For the functions
behind them, record with [samply](https://github.com/mstange/samply) (`cargo install samply`;
Instruments needs a full Xcode) and summarise with `profile.py`:

```sh
samply record --save-only --unstable-presymbolicate -r 1000 -o run.json.gz -- \
    target/release/gummiband route BOARD -o out.kicad_pcb
bench/profile.py run.json.gz parts      # busy time by part: search, relaxation, taut, check ...
bench/profile.py run.json.gz top 40     # self and total time per function
bench/profile.py run.json.gz lines 'relax_while::\{closure#41\}' relax.rs target/release/gummiband
```

A speed change that should not change results is checked by the SHA-1 of the routed file: the
result of a route is the same for any number of threads.

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

## DDR5 testbed (length matching)

`prepare-ddr5.sh` fetches Antmicro's DDR5 testbed (Apache-2.0, pinned to one commit) into
`work/ddr5/src/`, writes the board with its routing and the real layer types into
`work/ddr5/reference/` and the stripped board into `work/ddr5/ddr5/`. `ddr5-testbed.kicad_dru`
holds the length rules (the board has none of its own). `run-ddr5.sh [route args]` prints the
lengths of the reference routing, routes the stripped board into `work/ddr5/routed/`, prints its
lengths and the KiCad DRC summary.

## Repository housekeeping

`merged-branches.sh [REMOTE] [MAIN]` lists the branches of the remote (default `origin`) whose
head is an ancestor of its main branch, and prints the `git push origin --delete ...` command for
them. It is read-only: it asks the remote for its heads, fetches nothing and deletes nothing. A
head whose commit is not in the local clone is listed as unknown, never as merged.
