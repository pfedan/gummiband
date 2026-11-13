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

The reference holds for every build. The router used to call `sin`, `cos`, `atan2`, `acos`, `exp`,
`log`, `pow` and `hypot` of the system's maths library, whose last bits differ between glibc
versions, musl and macOS: the first CI run (glibc of Ubuntu 26.04) changed 12 of 15 hashes of a
reference made with glibc 2.39, with the same compiler. Since 2026-11-10 these functions come from
`gummiband_core::math` (the `libm` crate, pure Rust, compiled in), and the binary asks the system's
maths library for nothing. Measured on x86_64 Linux: the glibc build and the static musl build give
the same hash on all 27 runs, where before they agreed on 6. The reference did not change with it:
the hashes are the ones the musl build had before. (The crate is a port of musl's maths library; its
`sin`, `cos`, `tan`, `atan2`, `acos` and `hypot` give the bits of today's musl, its `exp`, `ln` and
`powf` are an older version and differ in some last bits, which no run of the list noticed.) Another
processor (macOS on ARM) is expected to give the same hashes, because only operations that IEEE 754
fixes to the last bit are left to the hardware; that is not measured yet. `cargo test -p
gummiband-core math` compares the bits of every function with fixed numbers, on any machine.

```sh
cargo build --release -p gummiband-cli
bench/hashes.sh --check             # all boards against hashes.txt, about 2 min at 4 threads
QUICK=1 bench/hashes.sh --check     # the quick set against hashes.quick.txt, what CI runs

# two binaries against each other: the build before a change and the one after it, or two
# builds of the same source (glibc and musl, two compilers)
bench/hashes.sh --compare path/to/old/gummiband target/release/gummiband
QUICK=1 bench/hashes.sh --compare OLD NEW

bench/hashes.sh easy bus            # print the hashes of some boards, compare nothing
```

Every board is routed with `--runs 4 --generations 1 --geometry-generations 0 --no-move-parts`.
A few extra runs on small boards cover option paths the plain run does not touch (the defaults,
`--angles 45`, `--pairs band`, `--pairs pull`, `--patterns groups`, `--negotiate`); their names carry a
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
The boards for areas in `boards/areas/` are in neither list, see below.

## Areas: pours and planes

`gen_boards.py` also writes five small boards into `boards/areas/`, each with one situation of a
pour or a plane. They are fixtures for the fill model behind `gummiband areas` (what the copper
fill of a net with a zone reaches), not routing problems: each routes in under a second.

| Board | Layers | What it holds | What the fill model says |
| --- | --- | --- | --- |
| `area_strips` | 2 | a GND pour on both layers (minimum width 0.2 mm, clearance 0.2 mm); six pens of locked track around one GND pad each, open only through a channel 4 mm long that leaves a strip of fill 0.10, 0.15, 0.20, 0.25, 0.30 or 0.40 mm wide | the pads behind 0.10 and 0.15 mm sit on islands (one via site each), the others are reached, as in KiCad; the pad behind 0.20 mm is doubtful |
| `area_island` | 2 | a GND pour on both layers; locked tracks from edge to edge cut the front into a region and a band with one GND pad each, strips of fill 0.21 mm wide between them that hold nothing, and the rest with a through hole pin; one GND pad stands in a pen of track that cuts its four spokes | the region and the band are islands with a via site each; the pad in the pen has no spoke; the strips are left out. Routed with `--areas fill --keep-locked`: two vias, one piece of track, all six pads reached (`area_route.rs`) |
| `area_halves` | 2 | GND parts at the left, GNDR parts at the right, three signals across, no zone; `area_halves.nets.json` leaves the two nets out of a route | a pour of both nets gives each a zone per layer around its own pads; no spot lies in two zones; all 6 pads of each are reached |
| `area_far` | 2 | VBUS with an area on B.Cu over the right third of the board, two through hole pins in it, six pins outside it | 2 of 8 pads reached; the farthest pad is 28.0 mm from a spot where a via lands in the area |
| `area_split` | 4 (In1 GND, In2 split) | In2.Cu holds +3V3 at the left and +5V at the right, 0.6 mm between the outlines; each net has one SMD pin over the other half and one through hole pin in it | each half reaches its two through hole pins; the pin in the other half lies outside the outline |

`cargo test -p gummiband-app --test area_boards` holds the model to the last column. The boards
are **not in the hash lists**: `hashes.sh` lists the boards of `boards/` only. They are quick
and their routed files are the same from run to run, but the hash of a routed file does not see
what they are for, the fill, and a board in the lists costs a reference line that every change
of the router has to carry.

`boards/areas/kicad/` holds four routed boards of the suite with a GND pour on both layers,
as KiCad 10 got them from the loop below (`<board>.s<seed>`). They are kept because the model
and KiCad disagreed on them; the tests in `area_boards.rs` pin KiCad's answer on each. They are
not written by `gen_boards.py` and must not be routed again: a new route is a new board that
KiCad has not seen.

### The model against KiCad's fill

The model was held against KiCad's fill on the planes of the LED controller, and against
KiCad 10 on the boards below (see "What KiCad 10 said"). To repeat it on a machine with
`kicad-cli` (KiCad 9 or 10) on the PATH:

```sh
python3 bench/gen_boards.py          # only if bench/boards/areas is missing
bench/areas-kicad.sh                 # the four boards above
```

and, for thin strips as a router leaves them, ten boards of the suite routed without GND, with
a pour of GND on both layers and one via per island:

```sh
for seed in 1 2 3; do
    POUR=GND:F.Cu,B.Cu \
    ROUTE_ARGS="--runs 4 --generations 1 --geometry-generations 0 --no-move-parts --seed $seed" \
    bench/areas-kicad.sh bench/boards/bus.kicad_pcb bench/boards/diff_pairs.kicad_pcb \
        bench/boards/fine_pitch.kicad_pcb bench/boards/fine_pitch_mixed.kicad_pcb \
        bench/boards/matching.kicad_pcb bench/boards/medium.kicad_pcb \
        bench/boards/parts_easy.kicad_pcb bench/boards/parts_hard.kicad_pcb \
        bench/boards/parts_medium.kicad_pcb bench/boards/power_mix.kicad_pcb
    cp bench/boards/out/areas/results.txt bench/boards/out/areas/results.seed$seed.txt
done
```

`areas-kicad.sh` routes a board (and pours it where asked), lets KiCad refill a copy
(`kicad-cli pcb drc --refill-zones --save-board`) and runs `gummiband areas --json` on that
copy, so that the model and KiCad's fill are compared on the same copper. It prints one line per
net and layer, also into `boards/out/areas/results.txt`:

- `mm2 model only / file only`: fill only one of the two has, beyond what the raster explains;
- `pads m/f`: pads only the model's fill touches, and pads only KiCad's fill touches;
- `unconn model/kicad`: the unconnected items the model expects for the net, and those KiCad's
  DRC counts;
- `doubtful`: the pads whose answer changes within one raster cell.

The model holds on a board where both pad lists are empty and the two unconnected counts agree.
A pad in `pads m/f` that is not in the doubtful list is a finding: send the line and the two
files `<name>.areas.json` and `<name>.filled/<name>.kicad_pcb`. On `area_halves` the question
is also whether KiCad reads the outlines as written (a region enclosed by another is cut into
its outline through a slit; this board has none, a board with an island of the other ground
has).

#### What KiCad 10 said

Measured 2026-11-11 with `kicad-cli` 10 and the commands above (the four area boards of that
day, the ten boards with seeds 1 to 3). Where the two disagreed the model now follows KiCad, and a test
pins KiCad's answer on the board where it showed:

| Where | The model said | KiCad 10 | Now |
| --- | --- | --- | --- |
| `area_strips` | the strip of 0.20 mm, the minimum width itself, is shut: 4 pieces on the front, 3 unconnected, `TP3.1` doubtful | passes it: 3 pieces, 2 unconnected | a cell centre at just the clearance is fill, the strip passes; `TP3.1` stays doubtful |
| `fine_pitch_mixed` seed 2, `R50.2` to `R54.2` and `R56.2` (a row at a pitch of 1.6 mm) | none of their spokes ends in fill: 7 unconnected | joins all six: 1 unconnected | a spoke also stays where it and a spoke of another pad hold each other's ends: 1 unconnected (`U1.57`) |
| `fine_pitch_mixed` seed 3, `U1.57` (exposed pad) | an island, 1 piece of fill on the front | unconnected too, but 2 pieces | the spokes are laid and tested as KiCad lays them: the island is a piece of 1 mm2 that stays, 2 pieces |
| `matching` seed 3, `J1.16` | no spoke ends in fill, and not doubtful | joins it | still not reached, but doubtful: the end of its one free spoke lies 0.05 mm from the clearance of a track, one cell outside the fill at the raster as it is, inside with half a cell less |
| `parts_medium` (all seeds), `fine_pitch_mixed` seeds 1 and 3, back layer | nothing of GND holds the pour there: every piece left out, 0 mm2 | keeps all of it (1393 to 1760 mm2 in 1 or 35 to 41 pieces) and warns of isolated copper for each | the pieces stay in the model and the report says so (`isolated`). `ground_pour` and `pour_stitch` still write the zone of such a layer: not writing it is open |
| `parts_medium`, `U2.4` | reached, doubtful | unconnected | as before: this is what doubtful is for |
| `area_halves`, `area_far`, `area_split` | | no difference at any pad, no DRC item; the model has 0.4 to 5 mm2 more fill of 344 to 1375 | as before |

What is assumed about KiCad's filler and not measured case by case: the end of a spoke lies the
thermal gap, half the minimum width and 0.005 mm beyond the box of the pad; a spoke is as wide
as the spoke width or the short side of the pad and is left out where that is under the minimum
width; it stays when its end lies in the fill as it is after the minimum width took its part,
or when it and a spoke of another pad hold each other's ends; other copper then cuts it by its
clearance. A pad whose answer hangs on this within a raster cell is named as doubtful.

On `J1.16` the two are not reconciled. Read from the copper, its spoke ends in a pocket of fill
of about 0.2 mm2 between the tracks of DQ4 and DQ0, which lie 0.16 mm apart where the pocket
would have to open: the pad would hang on that pocket and on nothing else. If KiCad calls the
pad connected, something in this reading is wrong; the filled board at 105.5, 114.7 shows what.

### `areas tracks` against `areas fill`

`areas-route.py` routes a board both ways with the same binary, `--areas tracks` (a net with a
pour is laid as tracks) and `--areas fill` (the other nets first, then only what the fill does
not reach), and prints one line per board, seed and mode: connections, the pads of the area
nets that the router's fill model leaves open on the routed board with its pour, the pieces
and spots of the own clearance check, vias, length, CPU seconds, and the seconds the fill model
took inside the search (summed over the threads) with the number of its runs. It needs no KiCad:

```sh
cargo build --release -p gummiband-cli
bench/areas-route.py                 # ten boards of the suite with GND poured, the four area boards
bench/areas-route.py /path/to/board.kicad_pcb=GND      # another board, GND poured on F.Cu and B.Cu
bench/areas-route.py /path/to/board.kicad_pcb          # the zones of the file are the areas
```

The table is also written to `boards/out/areas-route/results.txt` (and `results.json`); the
routed boards are deleted unless `--keep` is given. `--fill-args=--no-area-stitch` routes the
fill runs without the vias for islands, and `GB_AREA_ESCAPE=0` in the environment without the
price at the pads of the area nets.
`--modes fill` routes one mode only. Write `--args=...` and `--fill-args=...` with the equals
sign: their values begin with a dash.

Measured without KiCad (2026-11-11, the last check counting by fill, the settings of the
suite: `--runs 4 --generations 1 --geometry-generations 1 --no-move-parts`). Every number is
the sum of the seeds 1, 2 and 3; `open pads` is the router's own model, not KiCad. The lines of
`fill` are those of commit 980246a and have **not been measured again** since the fill model
follows KiCad and since the last check tells the cells of the area right (the two commits
after it): the lines of `tracks` cannot change, those of `fill` can. On `area_strips` the
route now sets two vias for islands, not three.

| board | mode | routed | open pads | pieces/spots | vias | of them island vias | mm | cpu s | model s (runs) |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `bus` | tracks | 66/66 | 0 | 0/0 | 86 | 0 | 2174 | 24 |  |
| `bus` | fill | 66/66 | 0 | 0/0 | 44 | 6 | 1944 | 40 | 6 (27) |
| `diff_pairs` | tracks | 93/93 | 0 | 0/0 | 158 | 0 | 3183 | 66 |  |
| `diff_pairs` | fill | 93/93 | 0 | 0/0 | 108 | 6 | 2961 | 74 | 5 (27) |
| `fine_pitch` | tracks | 336/336 | 0 | 0/0 | 6 | 0 | 2388 | 20 |  |
| `fine_pitch` | fill | 336/336 | 0 | 0/0 | 6 | 6 | 2010 | 32 | 5 (24) |
| `fine_pitch_mixed` | tracks | 336/336 | 0 | 27/9 | 420 | 0 | 4551 | 339 |  |
| `fine_pitch_mixed` | fill | 336/336 | 0 | 6/6 | 408 | 0 | 4278 | 477 | 32 (153) |
| `matching` | tracks | 78/78 | 0 | 0/0 | 54 | 0 | 1833 | 168 |  |
| `matching` | fill | 76/78 | 2 | 0/0 | 12 | 6 | 1651 | 195 | 5 (30) |
| `medium` | tracks | 114/114 | 0 | 0/0 | 36 | 0 | 1347 | 7 |  |
| `medium` | fill | 114/114 | 0 | 0/0 | 42 | 6 | 1137 | 14 | 4 (27) |
| `parts_easy` | tracks | 66/66 | 0 | 0/0 | 15 | 0 | 456 | 2 |  |
| `parts_easy` | fill | 66/66 | 0 | 0/0 | 3 | 0 | 396 | 8 | 3 (27) |
| `parts_hard` | tracks | 225/225 | 0 | 0/0 | 195 | 0 | 3191 | 20 |  |
| `parts_hard` | fill | 225/225 | 0 | 0/0 | 150 | 12 | 2751 | 38 | 7 (27) |
| `parts_medium` | tracks | 129/129 | 0 | 0/0 | 60 | 0 | 1568 | 8 |  |
| `parts_medium` | fill | 129/129 | 0 | 0/0 | 27 | 0 | 1584 | 20 | 10 (27) |
| `power_mix` | tracks | 57/57 | 0 | 0/0 | 17 | 0 | 960 | 7 |  |
| `power_mix` | fill | 57/57 | 0 | 0/0 | 15 | 0 | 657 | 12 | 5 (27) |
| **the ten** | tracks | 1500/1500 | 0 | 27/9 | 1047 | 0 | 21651 | 661 |  |
| **the ten** | fill | 1498/1500 | 2 | 6/6 | 815 | 42 | 19369 | 912 | 82 (396) |
| `area_strips` | tracks | 27/27 | 0 | 0/0 | 0 | 0 | 189 | 1 |  |
| `area_strips` | fill | 27/27 | 0 | 0/0 | 0 | 0 | 0 | 3 | 2 (24) |
| `area_far` | tracks | 60/60 | 0 | 0/0 | 0 | 0 | 654 | 3 |  |
| `area_far` | fill | 60/60 | 0 | 0/0 | 6 | 0 | 549 | 7 | 2 (27) |
| `area_split` | tracks | 51/51 | 0 | 0/0 | 30 | 0 | 405 | 2 |  |
| `area_split` | fill | 51/51 | 0 | 0/0 | 30 | 0 | 405 | 2 | 0 (0) |
| `area_halves` | tracks | 69/69 | 0 | 0/0 | 3 | 0 | 678 | 3 |  |
| `area_halves` | fill | 69/69 | 0 | 0/0 | 6 | 0 | 468 | 17 | 8 (27) |
| ledctl, its planes | tracks | 1044/1044 | 0 | 16/6 | 842 | 0 | 7414 | 256 |  |
| ledctl, its planes | fill | 1044/1044 | 0 | 16/6 | 842 | 0 | 7414 | 260 | 0 (0) |
| ledctl, GND poured | tracks | 1044/1044 | 0 | 16/6 | 842 | 0 | 7414 | 255 |  |
| ledctl, GND poured | fill | 1098/1098 | 0 | 5/5 | 782 | 5 | 7754 | 647 | 192 (133) |

What the table does not show:

- The two open pads are `J1.16` and `J2.16` of `matching` on seed 1: pads with a thermal
  relief whose spokes end in no fill on the finished copper. The search had them joined; the
  last check counts them open (24 of 26) and names them.
- A second run of the 42 lines with `fill` gave the same 42 lines.
- Without the vias for islands (`--fill-args=--no-area-stitch`, the ten boards): 18 open pads
  in 12 groups on 13 of the 30 runs, 877 vias, 19291 mm. Looked at on `parts_hard`: the
  search ended the track of `C2.2` in a piece of fill that hung on the area then and is an
  island on the finished copper; the last check takes such a track back. The via for an
  island is set on the finished copper and does not depend on what the search saw. This
  reading is in doubt since: the last check could also take a track back because it did not
  know the cells of the area (see `area_island`), which gives the same picture. Not measured
  again after that repair.
- Without the price at the pads (`GB_AREA_ESCAPE=0`, the ten boards): 13 open pads on 3 of the
  30 runs (`fine_pitch_mixed` 12, `diff_pairs` 1), 110 violating pieces at 24 spots against 6
  at 6, 780 vias, 19022 mm.
- The LED controller as it is has its planes on inner layers and no area on a routed layer:
  the files of `tracks` and `fill` are the same byte for byte. With GND poured on the outer
  layers as well (`ledctl.kicad_pcb=GND`) the route with `fill` counts 366 connections against
  348, because every group of pads of an area net counts and not only the pads that need a
  way to the plane.
- `model s` is wall time summed over the threads of the search. The machine was shared with
  another job, so it is an upper bound on the CPU time of the model.

**With KiCad** (`kicad-cli` of KiCad 9 or 10 on the PATH), `--kicad` lays the pour on every
result (the example `pour_stitch`: the zones, and one via per island that still has none), lets
KiCad refill it and adds three columns from KiCad's DRC: unconnected items (of the area nets /
of all nets), clearance violations, dangling vias and track ends. This is the judgement that
counts; the model's column `open pads` is what the router believed.

```sh
cargo build --release -p gummiband-cli --example pour_stitch
bench/areas-route.py --kicad --keep                    # the fourteen boards, three seeds each
```

The acceptance run on the Corne (`prepare-ext.sh` fetches and strips it into
`work/ext/corne/`), with the default settings of a route, three seeds:

```sh
bench/prepare-ext.sh
cargo build --release -p gummiband-cli --example pour_stitch
bench/areas-route.py --kicad --keep --args="" bench/work/ext/corne/corne.kicad_pcb=GND
```

`areas fill` passes when, on every seed and against the line of `tracks` ("route all, then
pour"): `unconn` of GND is 0; `clear` is no more than the 119 that the Corne had with GND left
out of the route; `vias` is no more than the 226 of that run plus the vias set for islands
(`areas: N vias set for islands of fill` in the log, `area_vias` in the report); `dangling` is
0. For the LED controller with a GND pour on the outer layers
(`bench/areas-route.py --kicad --keep --args="" /path/to/ledctl.kicad_pcb=GND`) nothing may be
worse than its line of `tracks`. With `--keep` the boards KiCad judged are
`boards/out/areas-route/<board>.s<seed>.<mode>.filled.kicad_pcb`, next to `.drc.json`; send
those two files for a pad that KiCad's DRC calls unconnected and the column `open pads` does
not count.

On the Corne with KiCad 10 (2026-11-11, before the two repairs named above) `fill` had more
clearance violations than `tracks` on every seed (573 against 439 over the seeds 1 to 3), most
of them at the two QFNs. Whether the price at the pads of the area nets pushes the signals
aside there is measured with the same seeds and the price off:

```sh
bench/areas-route.py --kicad --keep --modes fill --args="" bench/work/ext/corne/corne.kicad_pcb=GND
GB_AREA_ESCAPE=0 bench/areas-route.py --kicad --keep --modes fill --args="" bench/work/ext/corne/corne.kicad_pcb=GND
```

Compare `clear`, `unconn` and `vias` of the two tables line by line (the second run overwrites
`boards/out/areas-route/results.txt`: copy it first). Without KiCad the column `pieces/spots`
of the own clearance check stands in for `clear`. The price is not to be tuned on the small
boards alone: of the ten only `fine_pitch_mixed` and `diff_pairs` answer to it (see above). One
route of the Corne with the default settings takes more than an hour of CPU time.

## Two sided placement

`gen_two_sided.py` writes four synthetic boards into `boards/two_sided/` for `gummiband place
--two-sided` (placement only, no routing): `ts_som` (4 layers, 42 x 30 mm, BGA SoC and memory,
PMIC, about 120 0402 parts, a board to board connector locked on the back; the parts need about
1.4 times the front), `ts_sensor` (34 x 23 mm, a coin cell holder locked on the back, an antenna
keepout on the front: one sided nothing fits, two sided the front holds nearly all), `ts_roomy` (a
control with room to spare) and `ts_tht` (through hole electrolytics and a terminal block between
SMD converters). `check_sides.py PLACED [--input ORIGINAL]` checks a result from the file alone:
courtyard overlaps per side, the holes of through hole parts against parts on the other side, SMD
pads and courtyards on the layers of their side, mirrored texts on the back, and with `--input`
that every part that changed sides keeps its pad distances and is mirrored. `draw_sides.py PLACED
OUT.svg` draws front and back next to each other.

With `--iters 1000000`, one start, seed 1 (counts of 2026-11-10, with the maths of
`gummiband_core::math`; the system's maths gave 0 and 10 on `ts_sensor` and `ts_tht`):

| Board | One sided | Two sided: parts on the back | Wall time |
| --- | --- | --- | --- |
| `ts_som` | fails before the search, 321 mm2 missing | 105 | 20 s |
| `ts_sensor` | fails before the search, 275 mm2 missing | 1 (the front holds the rest once the holder only takes the back) | 6 s |
| `ts_roomy` | places | 9 (decoupling capacitors under their IC) | 3 s |
| `ts_tht` | places | 11, some under the electrolytics | 3 s |

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

`GB_LOG=timing` prints cumulative phase timers of a route (search, relaxation rounds, taut, check;
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

## Geometry test stand

`geostand/` holds small boards with one basic situation each, where the right copper is known:
one track around a via, a round pad or a corner; one to three tracks between two pads, a pad and a
via, two vias; a row of pads at 0.4 and 0.5 mm whose tracks turn hard behind the pads; bundles of
three and six around a corner; a turn with a neighbour on the inside; a tangent from via to via; a
track and a via next to a cutout of the board; a via between two pads and between two rows of
pads; a pad with a clearance of its own. Rules: clearance 0.2 mm, track 0.2 mm.

Every case comes in five forms, named by the room its channel has beyond what the copper needs:
`loose` (0.3 mm), `tight20` (20 um), `tight` (5 um), `exact` (none: the copper fits to the
micrometre, as boards are often laid out) and `short` (10 um missing: the right answer is "does
not fit"). `gen_geostand.py DIR` writes the 130 boards, and for each a reference solution
(`.ref.kicad_pcb`), the copper as it should be, with the spare room shared between the gaps.
The boards are not in the repository; the generator writes them in a second.

- `check_refs.py DIR` (needs `kicad-cli`) runs KiCad's DRC on the unrouted boards and on the
  references: clean and connected for every form but `short`, at least one entry there. A case
  whose reference fails does not measure what it claims. KiCad 10.0.7 passes 130 of 130.
- `run_geostand.py ROUTER DIR OUT` routes every board (`--seed 1 --no-move-parts`) and judges it
  with KiCad's DRC (`--all-track-errors`, every kind that concerns copper, counted by place);
  `table.py OUT` prints the table.
- `run_own.py ROUTER DIR OUT` does the same with the router's own check and needs no KiCad. On the
  130 boards the two agree: the own check is clean exactly where KiCad reports no place.
- `check.sh [--update] [ROUTER]` generates, routes and compares routed connections and spots per
  board with `expected.txt`: the stand as a regression test (about five minutes on four cores).
- `probe.py ROUTER CASE [slack in um ...]` builds one case with each slack and says from which
  slack on the router lays it (it honours the `GB_` variables of the shell, and writes into
  `probe/` in the current directory).

A board is right if it is fully routed and clean, or for `short` clean and not fully routed.
`expected.txt` records where the router stands, not where it should be: 47 of 130 are right at
the time of writing. A change that moves a board is meant to show up in that file.
