#!/bin/sh
# The geometry test stand as a regression test: generate the boards, route each one and compare
# routed connections and spots of the own check with expected.txt.
# Usage: bench/geostand/check.sh [--update] [ROUTER]     (default: target/release/gummiband)
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
MODE=--expect
if [ "${1:-}" = --update ]; then MODE=--update; shift; fi
BIN=${1:-$ROOT/target/release/gummiband}
[ -x "$BIN" ] || { echo "no router binary at $BIN (cargo build --release -p gummiband-cli)" >&2; exit 2; }
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
python3 "$HERE/gen_geostand.py" "$TMP/boards" >/dev/null
python3 "$HERE/run_own.py" "$BIN" "$TMP/boards" "$TMP/out" $MODE "$HERE/expected.txt"
