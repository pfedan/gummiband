#!/bin/sh
# Prepare the DDR5 matching benchmark: Antmicro's DDR5 testbed (one DDR5 x8 DRAM on an SO-DIMM
# edge connector, 6 layers, Apache-2.0, https://github.com/antmicro/ddr5-testbed), pinned to one
# commit. Writes into work/ddr5/:
#   reference/  the board as Antmicro routed it, with ddr5-testbed.kicad_dru (our rules)
#   ddr5/       the same board stripped of every track, via and pour outside the planes
# The file declares In2.Cu as a power layer although it carries most of the DDR signals, and
# In1/In3/In4 (the planes) partly as signal layers; both copies get the real layer types.
set -eu
export PATH=/opt/homebrew/opt/rustup/bin:$PATH
B=$(cd "$(dirname "$0")" && pwd)
W=$B/work/ddr5
REV=5db9286b45b5ad9b44f2d82230609038476f95b9
mkdir -p "$W"
if [ ! -d "$W/src/.git" ]; then
    rm -rf "$W/src"
    git init -q "$W/src"
    git -C "$W/src" remote add origin https://github.com/antmicro/ddr5-testbed
fi
git -C "$W/src" fetch -q --depth 1 origin "$REV"
git -C "$W/src" -c advice.detachedHead=false checkout -q "$REV"
mkdir -p "$W/reference" "$W/ddr5"
python3 - "$W/src/ddr5-testbed.kicad_pcb" "$W/reference/ddr5-testbed.kicad_pcb" <<'PY'
import re, sys
s = open(sys.argv[1]).read()
types = {"In1.Cu": "power", "In2.Cu": "signal", "In3.Cu": "power", "In4.Cu": "power"}
for name, t in types.items():
    s, n = re.subn(r'(\(\d+ "%s") [a-z]+\)' % re.escape(name), r'\1 %s)' % t, s, count=1)
    assert n == 1, name
open(sys.argv[2], "w").write(s)
PY
for d in reference ddr5; do
    cp "$W/src/ddr5-testbed.kicad_pro" "$B/ddr5-testbed.kicad_dru" "$W/$d/"
done
cargo build --release -q -p gummiband-cli
"$B/../target/release/gummiband" strip "$W/reference/ddr5-testbed.kicad_pcb" -o "$W/ddr5/ddr5-testbed.kicad_pcb"
echo "reference: $W/reference, stripped: $W/ddr5"
