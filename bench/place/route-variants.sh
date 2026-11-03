#!/bin/sh
# Route every placement variant in bench/place/<variant>/ with Gummiband, one after another, and
# judge the result with the KiCad DRC. Also renders placement and routed board to PNG.
# Usage: bench/place/route-variants.sh [variant ...]
set -eu
export PATH=/opt/homebrew/opt/rustup/bin:$PATH
export RAYON_NUM_THREADS=${RAYON_NUM_THREADS:-4}
HERE=$(cd "$(dirname "$0")" && pwd)
BIN="$HERE/../../target/release/gummiband"
render() { # board png
    kicad-cli pcb export svg --layers F.Cu,B.Cu,Edge.Cuts,F.Courtyard --mode-single --page-size-mode 2 \
        --exclude-drawing-sheet -o "${2%.png}.svg" "$1" >/dev/null
    rsvg-convert -w 1600 -b white "${2%.png}.svg" -o "$2"
    rm -f "${2%.png}.svg"
}
summary() { # drc.json
    python3 - "$1" <<'PY'
import json, collections, sys
d = json.load(open(sys.argv[1]))
c = collections.Counter(v["type"] for v in d["violations"] if not v["type"].startswith(("silk", "lib_")))
print("KiCad DRC: unconnected", len(d["unconnected_items"]), dict(c))
PY
}
[ $# -gt 0 ] || set -- $(cd "$HERE" && ls -d */ | tr -d /)
for v in "$@"; do
    D="$HERE/$v"
    [ -f "$D/ledctl.kicad_pcb" ] || continue
    echo "== $v"
    render "$D/ledctl.kicad_pcb" "$D/placement.png"
    mkdir -p "$D/routed"
    cp "$D/ledctl.kicad_pro" "$D/ledctl.kicad_dru" "$D/routed/"
    start=$(date +%s)
    "$BIN" route "$D/ledctl.kicad_pcb" -o "$D/routed/ledctl.kicad_pcb" --runs 4 --generations 1 \
        > "$D/routed/route.log" 2>&1 || { echo "router failed:"; tail -5 "$D/routed/route.log"; continue; }
    echo "route time $(( $(date +%s) - start )) s"
    tail -4 "$D/routed/route.log"
    (cd "$D/routed" && kicad-cli pcb drc --refill-zones --format json -o drc.json ledctl.kicad_pcb >/dev/null)
    summary "$D/routed/drc.json"
    render "$D/routed/ledctl.kicad_pcb" "$D/routed.png"
done
