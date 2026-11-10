#!/bin/sh
# Route the synthetic boards with fixed settings and print one SHA-1 per routed file.
# Results without an open end are deterministic (same input and seed, any thread count), so the
# list is a regression test: a refactoring must leave it unchanged.
#
# Usage: bench/hashes.sh [BIN] [board ...]      (default BIN: target/release/gummiband)
#        bench/hashes.sh --check [BIN]          compare with bench/hashes.txt, exit 1 on a difference
#        bench/hashes.sh --update [BIN]         rewrite bench/hashes.txt
# Environment: QUICK=1 routes only the boards that take a few seconds (the CI set).
# No kicad-cli needed: only the router runs.
set -u
BENCH=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$BENCH/.." && pwd)
MODE=print
case "${1:-}" in
    --check) MODE=check; shift ;;
    --update) MODE=update; shift ;;
esac
BIN=${1:-$ROOT/target/release/gummiband}
[ $# -gt 0 ] && shift
[ -x "$BIN" ] || { echo "no router binary at $BIN (cargo build --release -p gummiband-cli)" >&2; exit 2; }
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT
export RAYON_NUM_THREADS=${RAYON_NUM_THREADS:-4}
QUICK_SET="easy medium bus power_mix crossing qfn_fanout diff_pairs parts_easy parts_medium"

sha() {
    if command -v sha1sum >/dev/null 2>&1; then sha1sum "$1" | cut -d' ' -f1; else shasum "$1" | cut -d' ' -f1; fi
}

run() {
    # name, then the route arguments
    name=$1; shift
    board=${name%%+*}
    src=$BENCH/boards/$board.kicad_pcb
    [ -f "$src" ] || { echo "$name missing"; return; }
    # the router reads the .kicad_pro and .kicad_dru next to the board
    if "$BIN" route "$src" -o "$OUT/$name.kicad_pcb" "$@" >"$OUT/$name.log" 2>&1; then
        echo "$name $(sha "$OUT/$name.kicad_pcb")"
    else
        echo "$name FAILED"
    fi
}

list() {
    if [ $# -gt 0 ]; then
        boards="$*"
    elif [ "${QUICK:-0}" = 1 ]; then
        boards=$QUICK_SET
    else
        boards=$(cd "$BENCH/boards" && ls *.kicad_pcb | sed 's/\.kicad_pcb$//' | sort)
    fi
    for b in $boards; do
        run "$b" --runs 4 --generations 1 --geometry-generations 0 --no-move-parts
    done
    # option paths that the plain run does not touch (small boards only)
    if [ $# -eq 0 ]; then
        run "medium+default" --runs 4
        run "medium+45" --runs 4 --generations 1 --geometry-generations 1 --angles 45
        run "diff_pairs+pairs" --runs 4 --generations 1 --geometry-generations 1 --pairs --no-move-parts
        run "diff_pairs+pull" --runs 4 --generations 1 --geometry-generations 0 --pair-pull 1 --no-move-parts
        run "bus+groups" --runs 4 --generations 1 --geometry-generations 0 --patterns groups --no-move-parts
        run "crossing+negotiate" --runs 4 --generations 1 --geometry-generations 0 --negotiate 20 --no-move-parts
    fi
}

REF=$BENCH/hashes.txt
[ "${QUICK:-0}" = 1 ] && REF=$BENCH/hashes.quick.txt
case $MODE in
    print) list "$@" ;;
    update) list >"$REF"; cat "$REF" ;;
    check)
        list >"$OUT/now.txt"
        if diff "$REF" "$OUT/now.txt"; then
            echo "hashes unchanged ($(wc -l <"$REF" | tr -d ' ') runs)"
        else
            echo "hashes differ from $REF" >&2
            exit 1
        fi
        ;;
esac
