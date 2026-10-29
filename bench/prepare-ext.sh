#!/bin/sh
# Prepare the external benchmark boards (bench/kicad-autorouter-benchmarks.zip, see
# bench/README.md): unpack, then strip every track, via and pour outside the plane layers.
# Placement, outline, keepouts and the inner planes stay.
set -eu
export PATH=/opt/homebrew/opt/rustup/bin:$PATH
B=$(cd "$(dirname "$0")" && pwd)
ZIP=${1:-$B/kicad-autorouter-benchmarks.zip}
W=$B/work/ext
mkdir -p "$W/src"
unzip -q -o "$ZIP" -d "$W/src"
cargo build --release -q -p gummiband-cli
GB=$B/../target/release/gummiband
S=$W/src/kicad-autorouter-benchmarks
# name, source folder, plane layers
while read -r name dir planes; do
    mkdir -p "$W/$name"
    for f in "$S/$dir/"*.kicad_pro "$S/$dir/"*.kicad_dru; do
        [ -f "$f" ] && cp "$f" "$W/$name/$name.${f##*.}"
    done
    "$GB" --plane-layers "$planes" strip "$S/$dir/"*.kicad_pcb -o "$W/$name/$name.kicad_pcb"
    # mark the plane layers as KiCad "power" layers, so the board describes itself
    for l in $(echo "$planes" | tr , ' '); do
        sed -i '' -E "s/^([[:space:]]*\([0-9]+ \"$l\") [a-z]+\)/\1 power)/" "$W/$name/$name.kicad_pcb"
    done
done <<LIST
corne 01_corne-cherry -
arty 02_arty-mpw-tester In1.Cu,In2.Cu
ulx3s 03_ulx3s In1.Cu,In2.Cu
jetson 04_jetson-orin-baseboard In1.Cu,In2.Cu,In4.Cu,In6.Cu
LIST
