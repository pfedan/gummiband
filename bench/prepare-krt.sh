#!/bin/sh
# Prepare the boards of the KiCadRoutingTools test corpus as a second set of real boards
# (see bench/README.md): fetch https://github.com/drandyhaas/KiCadRoutingTools (MIT; the boards
# are open hardware designs with licenses of their own, so they are fetched and not kept in
# this repository) and copy the unrouted boards with their project files into bench/work/krt.
# Usage: bench/prepare-krt.sh [path to an existing clone]
set -eu
B=$(cd "$(dirname "$0")" && pwd)
W=$B/work/krt
SRC=${1:-$W/src}
if [ ! -d "$SRC/kicad_files" ]; then
    mkdir -p "$W"
    git clone --depth 1 https://github.com/drandyhaas/KiCadRoutingTools "$SRC"
fi
# The boards that route with the rules they bring. Left out: boards without project rules
# whose parts need finer ones than KiCad's defaults (orangecrab_ext_pll, haasoscope_pro_max_test)
# and the fixtures of single router features.
for n in esp_prog sonde_u splitflap_driver tigard watchy interf_u_unrouted \
    kit-dev-coldfire-xilinx_5213 glasgow_revC ulx3s; do
    mkdir -p "$W/$n"
    for e in kicad_pcb kicad_pro kicad_dru; do
        [ -f "$SRC/kicad_files/$n.$e" ] && cp "$SRC/kicad_files/$n.$e" "$W/$n/$n.$e"
    done
    echo "$W/$n/$n.kicad_pcb"
done
