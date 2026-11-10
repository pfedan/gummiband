#!/bin/sh
# Generate the placement variants of the LED controller from the prepared benchmark
# board (bench/prepare-ledctl.sh) into bench/place/<variant>/, one after another.
set -eu
export PATH=/opt/homebrew/opt/rustup/bin:$PATH
export RAYON_NUM_THREADS=${RAYON_NUM_THREADS:-4}
HERE=$(cd "$(dirname "$0")" && pwd)
IN="$HERE/../work/ledctl/ledctl.kicad_pcb"
(cd "$HERE/../.." && cargo build --release -q -p gummiband-place --example variants)
BIN="$HERE/../../target/release/examples/variants"
make() { # name args...
    name=$1
    shift
    mkdir -p "$HERE/$name"
    "$BIN" "$IN" -o "$HERE/$name/ledctl.kicad_pcb" "$@" > "$HERE/$name/place.log" 2>&1 \
        || { echo "$name failed:"; tail -3 "$HERE/$name/place.log"; return 0; }
    (cd "$HERE/$name" && kicad-cli pcb drc --refill-zones --format json -o drc-placed.json ledctl.kicad_pcb >/dev/null)
    python3 - "$name" "$HERE/$name/drc-placed.json" <<'PY'
import json, collections, sys
d = json.load(open(sys.argv[2]))
c = collections.Counter(v["type"] for v in d["violations"])
print(sys.argv[1], "placed, KiCad DRC: unconnected", len(d["unconnected_items"]), dict(c))
PY
}
make rect-90x55 --shape rect --width 90 --height 55 --gap 0.35 --seed 2
make square-76 --shape square --width 76 --gap 0.6
make round-84 --shape circle --diameter 84 --gap 0.6
