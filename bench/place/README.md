# Placement variants of the LED controller

Extra boards for the router portfolio: the prepared benchmark board
(`bench/work/ledctl`, see `bench/prepare-ledctl.sh`) with a new outline and a new
placement, made by the `variants` example of the placement crate
(`crates/gummiband-place/examples/variants.rs`, a benchmark tool and not an installed program;
by hand: `cargo run --release -p gummiband-place --example variants -- BOARD -o OUT [options]`).

```sh
bench/place/make-variants.sh      # placement + KiCad DRC of the unrouted board
bench/place/route-variants.sh     # route each variant (--runs 4 --generations 1), DRC, PNGs
```

| Variant | Outline | Command line |
| --- | --- | --- |
| `rect-90x55` | 90 x 55 mm (original 100 x 60) | `--shape rect --width 90 --height 55 --gap 0.35 --seed 2` |
| `square-76` | 76 x 76 mm | `--shape square --width 76 --gap 0.6` |
| `round-84` | circle, 84 mm | `--shape circle --diameter 84 --gap 0.6` |

Each folder: `ledctl.kicad_pcb` (+ `.kicad_pro`, `.kicad_dru`), `place.log`,
`drc-placed.json`, `placement.png`, `routed/` (routed board, `route.log`, `drc.json`) and
`routed.png`.

How the placer works:

- Fixed mechanical parts go on the edge by rule: USB-C J1 on the left, the ESP32 module U10 on the
  right with its antenna end at the edge (its courtyard and keepout, 48 mm wide, outside or along
  the edge), plugs J200..J203 (rail A) along the top, J204..J207 (rail B) along the bottom (on the
  circle: radially, on the upper and lower arc), mounting holes in the corners or in the free arcs.
  Each plug takes its PTC and 22 uF output capacitor along, in the arrangement of J202.
- The power stages (SiC437, inductor, input and driver capacitors, feedback, boot and mode
  resistors) stay rigid macros as in the original layout; everything else moves on its own.
- Start: the original placement mapped into the new outline. Then simulated annealing (moves,
  90 degree rotations, swaps) on weighted half perimeter wire length (GND ignored, it is the In1
  plane), the distance of decoupling capacitors to the IC pin they served in the original, and the
  courtyard overlap (soft, growing weight); then legalization (biggest units first, spiral search)
  and an overlap free polish. Deterministic for a given seed.
- In1: one GND zone over the whole outline. In2: +5V_A, +5V_B, VBUS_P and VBUS_IN as regions
  around their pins (nearest pin on a 0.25 mm raster, one connected region per net without holes,
  enclosed regions get a corridor to the outside, 0.5 mm gaps, 1 mm from the edge, not under the
  antenna). Pins outside their net's region are reached by routing.
- Keepouts follow their part (antenna keepout with U10, the B.Cu keepouts under the SW nodes with
  the power stage), as do the silk labels of the plugs.
