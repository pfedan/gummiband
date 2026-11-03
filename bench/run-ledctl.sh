#!/bin/sh
# Route the prepared LED controller benchmark and judge it with the KiCad DRC.
# Usage: bench/run-ledctl.sh [extra gummiband route args]
set -eu
export PATH=/opt/homebrew/opt/rustup/bin:$PATH
W=$(cd "$(dirname "$0")" && pwd)/work
[ -f "$W/ledctl/ledctl.kicad_pcb" ] || "$(dirname "$0")/prepare-ledctl.sh"
mkdir -p "$W/routed"
cp "$W/ledctl/ledctl.kicad_pro" "$W/ledctl/ledctl.kicad_dru" "$W/routed/"
cargo build --release -q -p gummiband-cli
"$(dirname "$0")/../target/release/gummiband" route "$W/ledctl/ledctl.kicad_pcb" \
    -o "$W/routed/ledctl.kicad_pcb" --svg "$W/routed/routed.svg" "$@"
cd "$W/routed"
kicad-cli pcb drc --refill-zones --format json -o drc.json ledctl.kicad_pcb >/dev/null
python3 - <<'PY'
import json, collections
d = json.load(open("drc.json"))
c = collections.Counter(v["type"] for v in d["violations"] if not v["type"].startswith(("silk", "lib_")))
print("KiCad DRC: unconnected", len(d["unconnected_items"]), dict(c))
PY
