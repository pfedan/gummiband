#!/bin/sh
# Prepare the Lichtschnitzel controller benchmark: copy the project and strip every track, via
# and outer layer pour, so that the board is routed completely by Gummiband.
set -eu
export PATH=/opt/homebrew/opt/rustup/bin:$PATH
SRC=${LICHTSCHNITZEL:-$HOME/code/lichtschnitzel}/hardware/kicad
OUT=${1:-$(dirname "$0")/work/lichtschnitzel}
mkdir -p "$OUT"
cp "$SRC/lichtschnitzel.kicad_pro" "$SRC/lichtschnitzel.kicad_dru" "$OUT/"
cargo run --release -q -p gummiband-cli -- strip "$SRC/lichtschnitzel.kicad_pcb" -o "$OUT/lichtschnitzel.kicad_pcb"
