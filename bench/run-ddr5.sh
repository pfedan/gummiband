#!/bin/sh
# Route the prepared DDR5 benchmark (prepare-ddr5.sh), measure it against its length rules as
# KiCad does and judge it with the KiCad DRC, next to the measurements of Antmicro's routing.
# Usage: bench/run-ddr5.sh [extra gummiband route args]     (e.g. --pairs band)
set -eu
export PATH=/opt/homebrew/opt/rustup/bin:$PATH
B=$(cd "$(dirname "$0")" && pwd)
W=$B/work/ddr5
[ -f "$W/ddr5/ddr5-testbed.kicad_pcb" ] || "$B/prepare-ddr5.sh"
cp "$B/ddr5-testbed.kicad_dru" "$W/reference/"
cp "$B/ddr5-testbed.kicad_dru" "$W/ddr5/"
mkdir -p "$W/routed"
cp "$W/ddr5/ddr5-testbed.kicad_pro" "$B/ddr5-testbed.kicad_dru" "$W/routed/"
cargo build --release -q -p gummiband-cli
GB=$B/../target/release/gummiband
echo "== reference (Antmicro)"
"$GB" lengths "$W/reference/ddr5-testbed.kicad_pcb"
echo "== gummiband"
"$GB" route "$W/ddr5/ddr5-testbed.kicad_pcb" -o "$W/routed/ddr5-testbed.kicad_pcb" \
    --svg "$W/routed/routed.svg" "$@"
cd "$W/routed"
kicad-cli pcb drc --refill-zones --format json -o drc.json ddr5-testbed.kicad_pcb >/dev/null
python3 - <<'PY'
import json, collections
d = json.load(open("drc.json"))
c = collections.Counter(v["type"] for v in d["violations"] if not v["type"].startswith(("silk", "lib_")))
print("KiCad DRC: unconnected", len(d["unconnected_items"]), dict(c))
PY
