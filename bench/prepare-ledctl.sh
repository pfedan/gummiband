#!/bin/sh
# Prepare the LED controller benchmark: copy the project and strip every track, via and outer
# layer pour, so that the board is routed completely by Gummiband.
# Usage: LEDCTL_PCB=/path/to/board.kicad_pcb bench/prepare-ledctl.sh [out dir]
# The .kicad_pro and .kicad_dru with the same base name are copied along.
set -eu
export PATH=/opt/homebrew/opt/rustup/bin:$PATH
PCB=${LEDCTL_PCB:?set LEDCTL_PCB to the .kicad_pcb of the LED controller}
BASE=${PCB%.kicad_pcb}
OUT=${1:-$(dirname "$0")/work/ledctl}
mkdir -p "$OUT"
cp "$BASE.kicad_pro" "$OUT/ledctl.kicad_pro"
cp "$BASE.kicad_dru" "$OUT/ledctl.kicad_dru"
cargo run --release -q -p gummiband-cli -- strip "$PCB" -o "$OUT/ledctl.kicad_pcb"
