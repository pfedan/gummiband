#!/bin/sh
# Route one prepared external benchmark board and judge it with the KiCad DRC.
# Usage: bench/run-ext.sh NAME [extra gummiband route args]   (NAME: corne, arty, ulx3s, jetson)
set -eu
B=$(cd "$(dirname "$0")" && pwd)
N=$1
shift
W=$B/work/ext
[ -f "$W/$N/$N.kicad_pcb" ] || "$B/prepare-ext.sh"
mkdir -p "$W/routed/$N"
cp "$W/$N/"*.kicad_pro "$W/$N/"*.kicad_dru "$W/routed/$N/" 2>/dev/null || true
"$B/../target/release/gummiband" route "$W/$N/$N.kicad_pcb" -o "$W/routed/$N/$N.kicad_pcb" \
    --svg "$W/routed/$N/routed.svg" "$@"
cd "$W/routed/$N"
kicad-cli pcb drc --refill-zones --format json -o drc.json "$N.kicad_pcb" >/dev/null
python3 - <<'PY'
import json, collections
d = json.load(open("drc.json"))
c = collections.Counter(v["type"] for v in d["violations"] if not v["type"].startswith(("silk", "lib_")))
print("KiCad DRC: unconnected", len(d["unconnected_items"]), dict(c))
PY
