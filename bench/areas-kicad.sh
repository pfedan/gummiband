#!/bin/sh
# The fill model of `gummiband areas` against KiCad's filler, on the same copper.
#
# Needs kicad-cli (KiCad 9 or 10) on the PATH. The model has been compared with KiCad's fill on
# the planes of the LED controller only; this script is for thin strips between tracks on routed
# outer layers, where the two may differ.
#
# Usage: bench/areas-kicad.sh                  the four boards of bench/boards/areas, each as
#                                              it is meant (see below)
#        bench/areas-kicad.sh BOARD.kicad_pcb [BOARD.kicad_pcb ...]
# Environment, for boards given by name:
#   ROUTE=1        route each board first (default 1; 0: take the file as it is). A file
#                  <board>.nets.json next to the board is passed as --net-widths.
#   ROUTE_ARGS     arguments for `gummiband route` (default: --runs 4 --generations 1
#                  --geometry-generations 0 --no-move-parts, as the suite)
#   POUR           NETS:LAYERS, for a board without zones: these nets are left out of the
#                  route, and after it a pour of them on these layers is written with one via
#                  per island (the example pour_stitch of gummiband-cli, the same step as the
#                  MCP tool ground_pour), for instance POUR=GND:F.Cu,B.Cu
#   AREAS_ARGS     extra arguments for `gummiband areas`
#
# Without arguments:
#   area_strips    as the file is (ROUTE=0): locked tracks and the pour are in the file
#   area_far       routed, the VBUS area is in the file
#   area_split     routed, the planes are in the file
#   area_halves    routed without GND and GNDR, then POUR=GND,GNDR:F.Cu,B.Cu
#
# For every board, in bench/boards/out/areas/:
#   <name>.kicad_pcb       the board KiCad gets: routed and poured as asked for, or a copy
#   <name>.filled/         a copy that KiCad refilled:
#                          kicad-cli pcb drc --refill-zones --save-board
#   <name>.drc.json        KiCad's DRC of that copy
#   <name>.areas.json      gummiband areas --json of the refilled copy: the model on the same
#                          copper, with "against_file" per net and layer
#   <name>.areas.txt       the same as text
# and a table on stdout and in results.txt, one line per net and layer:
#   model only / file only   mm2 only one of the fills has, beyond what the raster explains
#   pads m/f                 pads only the model's fill touches / only KiCad's
#   unconn model/kicad       unconnected items the model expects for the net / KiCad's DRC counts
#   doubtful                 pads whose answer changes within one raster cell
# The model is right at a board where both "pads" columns are 0 and the unconnected counts
# agree; a pad in "pads m/f" that is not in the doubtful list is a finding.
set -u
BENCH=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$BENCH/.." && pwd)
command -v kicad-cli >/dev/null 2>&1 || { echo "kicad-cli not found: run this where KiCad is installed"; exit 1; }
echo "building gummiband-cli ..."
(cd "$ROOT" && cargo build --release -q -p gummiband-cli) || { echo "build failed"; exit 1; }
(cd "$ROOT" && cargo build --release -q -p gummiband-cli --example pour_stitch) || { echo "build failed"; exit 1; }
exec python3 - "$ROOT" "$@" <<'PY'
import json, os, re, shutil, subprocess, sys

root, boards = sys.argv[1], sys.argv[2:]
target = os.environ.get("CARGO_TARGET_DIR", os.path.join(root, "target"))
gb = os.path.join(target, "release", "gummiband")
pour_stitch = os.path.join(target, "release", "examples", "pour_stitch")
out = os.path.join(root, "bench", "boards", "out", "areas")
os.makedirs(out, exist_ok=True)
route_args = os.environ.get("ROUTE_ARGS", "--runs 4 --generations 1 --geometry-generations 0 --no-move-parts").split()
areas_args = os.environ.get("AREAS_ARGS", "").split()
if boards:
    # (board, route first, pour)
    plan = [(b, os.environ.get("ROUTE", "1") != "0", os.environ.get("POUR", "")) for b in boards]
else:
    own = os.path.join(root, "bench", "boards", "areas")
    plan = [(os.path.join(own, n + ".kicad_pcb"), r, p) for n, r, p in (
        ("area_strips", False, ""), ("area_far", True, ""), ("area_split", True, ""),
        ("area_halves", True, "GND,GNDR:F.Cu,B.Cu"))]

def with_project(src, dst):
    for ext in (".kicad_pro", ".kicad_dru"):
        s = os.path.splitext(src)[0] + ext
        if os.path.isfile(s):
            shutil.copy(s, os.path.splitext(dst)[0] + ext)

rows = []
for board, route, pour in plan:
    if not os.path.isfile(board):
        print(f"{board}: no such file")
        continue
    name = os.path.splitext(os.path.basename(board))[0]
    work = os.path.join(out, name + ".kicad_pcb")
    if os.path.abspath(board) == os.path.abspath(work):
        print(f"{name}: the board lies where the results go, give a file outside {out}")
        continue
    if route:
        nets = os.path.splitext(board)[0] + ".nets.json"
        if pour and not os.path.isfile(nets):
            # the nets of the pour are left out of the route: the pour is to connect them
            nets = os.path.join(out, name + ".nets.json")
            json.dump({n: {"skip": True} for n in pour.partition(":")[0].split(",")}, open(nets, "w"))
        extra = ["--net-widths", nets] if os.path.isfile(nets) else []
        with open(os.path.join(out, name + ".route.log"), "w") as log:
            r = subprocess.run([gb, "route", board, "-o", work] + extra + route_args, stdout=log, stderr=subprocess.STDOUT)
        if r.returncode != 0 or not os.path.isfile(work):
            print(f"{name}: route failed, see {name}.route.log")
            continue
    else:
        shutil.copy(board, work)
    with_project(board, work)
    if pour:
        nets, _, layers = pour.partition(":")
        poured = os.path.join(out, name + ".poured.kicad_pcb")
        with open(os.path.join(out, name + ".pour.log"), "w") as log:
            r = subprocess.run([pour_stitch, work, "--net", nets, "--pour", layers or "F.Cu,B.Cu", "-o", poured],
                               stdout=log, stderr=subprocess.STDOUT)
        if r.returncode != 0 or not os.path.isfile(poured):
            print(f"{name}: the pour failed, see {name}.pour.log")
            continue
        os.replace(poured, work)
        for ext in (".kicad_pro", ".kicad_dru"):
            if os.path.isfile(os.path.splitext(poured)[0] + ext):
                os.remove(os.path.splitext(poured)[0] + ext)
    filled_dir = os.path.join(out, name + ".filled")
    os.makedirs(filled_dir, exist_ok=True)
    filled = os.path.join(filled_dir, name + ".kicad_pcb")
    shutil.copy(work, filled)
    with_project(board, filled)
    drc = os.path.join(out, name + ".drc.json")
    subprocess.run(["kicad-cli", "pcb", "drc", "--refill-zones", "--save-board", "--format", "json", "-o", drc, filled],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    j = subprocess.run([gb, "areas", filled, "--json"] + areas_args, capture_output=True, text=True)
    if j.returncode != 0:
        print(f"{name}: gummiband areas failed: {j.stderr.strip()[:300]}")
        continue
    open(os.path.join(out, name + ".areas.json"), "w").write(j.stdout)
    t = subprocess.run([gb, "areas", filled] + areas_args, capture_output=True, text=True)
    open(os.path.join(out, name + ".areas.txt"), "w").write(t.stdout)
    report = json.loads(j.stdout)
    # KiCad's unconnected items per net: the net name stands in brackets in the item texts
    unconnected = {}
    try:
        for item in json.load(open(drc)).get("unconnected_items", []):
            nets = set()
            for it in item.get("items", []):
                m = re.search(r"\[([^\]]*)\]", it.get("description", ""))
                if m:
                    nets.add(m.group(1))
            for n in nets:
                unconnected[n] = unconnected.get(n, 0) + 1
    except (OSError, ValueError) as e:
        print(f"{name}: no DRC report ({e})")
    for net in report["nets"]:
        doubtful = [d["pad"] for d in (net.get("doubtful") or [])]
        if not net["against_file"]:
            rows.append((name, net["net"], "-", "no fill in the file", "", "", "", ""))
        for c in net["against_file"]:
            pads = f'{len(c["pads_model_only"])}/{len(c["pads_file_only"])}'
            named = " ".join(c["pads_model_only"] + c["pads_file_only"])
            rows.append((name, net["net"], c["layer"],
                         f'{c["model_only_deep"]:.2f} / {c["file_only_deep"]:.2f} of {c["both"]:.0f}',
                         pads + (" " + named if named else ""),
                         f'{net["expected_unconnected"]}/{unconnected.get(net["net"], 0)}',
                         f'{len(doubtful)}' + (" " + " ".join(doubtful) if doubtful else ""),
                         f'{c["pieces_model"]}/{c["pieces_file"]}'))

head = ("board", "net", "layer", "mm2 model only / file only", "pads m/f", "unconn model/kicad", "doubtful", "pieces m/f")
widths = [max(len(str(r[i])) for r in rows + [head]) for i in range(len(head))]
lines = ["  ".join(str(c).ljust(w) for c, w in zip(r, widths)).rstrip() for r in [head] + rows]
text = "\n".join(lines) + "\n"
print(text, end="")
open(os.path.join(out, "results.txt"), "w").write(text)
PY
