#!/usr/bin/env python3
"""`areas tracks` against `areas fill` of `gummiband route`, with the same binary.

With `areas tracks` a net with a pour is laid as tracks like any net. With `areas fill` the other
nets are laid first and only the pads the fill of the pour does not reach get copper. This script
routes each board both ways and prints one line per board, seed and mode.

Usage: bench/areas-route.py                      the ten two layer boards of the suite with a
                                                 GND pour on both outer layers, and the four
                                                 boards of bench/boards/areas
       bench/areas-route.py BOARD[=NETS] ...     these boards; NETS (comma separated) get a
                                                 pour on both outer layers, without NETS the
                                                 zones of the file are the areas
Options:
  --seeds 1,2,3     the seeds of the search (default 1,2,3)
  --modes fill      only these modes (default tracks,fill)
  --args="..."      arguments of `gummiband route` (default: --runs 4 --generations 1
                    --geometry-generations 1 --no-move-parts, as the suite); --args="" routes
                    with the defaults of `gummiband route`
  --fill-args="..." more arguments for the runs with `areas fill` only
                    (write both with `=`: their values begin with a dash)
  --kicad           also judge every result with KiCad: lay the pour (the example pour_stitch
                    of gummiband-cli: the zones, and one via per island that still has none),
                    let KiCad refill it and run its DRC. Needs kicad-cli (KiCad 9 or 10).
  --keep            keep the routed boards and reports in bench/boards/out/areas-route

Columns:
  routed        connections routed / planned; with `fill` what the fill joins counts as both
  open pads     pads of the area nets that neither fill nor copper joins to the area, by the
                router's fill model on the routed board with the pour (`gummiband areas`)
  groups        groups of such pads: the unconnected items KiCad's DRC would count for them
  pieces/spots  Gummiband's own clearance check: violating pieces of copper, and places
  vias, mm      vias and track length written
  island vias   of the vias, those set for islands of fill without a search
  cpu s         CPU seconds of the route (user and system, all threads)
  model s       seconds inside the fill model during the search, summed over the threads
                (wall time of each run, so under load more than its share of the CPU), and
                the number of its runs
With --kicad:
  unconn        unconnected items of KiCad's DRC, of the area nets / of all nets
  clear         clearance violations of KiCad's DRC
  dangling      dangling vias and track ends of KiCad's DRC
"""

import argparse
import json
import os
import re
import resource
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TARGET = os.environ.get("CARGO_TARGET_DIR", os.path.join(ROOT, "target"))
GB = os.path.join(TARGET, "release", "gummiband")
POUR_STITCH = os.path.join(TARGET, "release", "examples", "pour_stitch")
OUT = os.path.join(ROOT, "bench", "boards", "out", "areas-route")
SUITE = ["bus", "diff_pairs", "fine_pitch", "fine_pitch_mixed", "matching", "medium",
         "parts_easy", "parts_hard", "parts_medium", "power_mix"]
AREA_BOARDS = [("area_strips", ""), ("area_far", ""), ("area_split", ""), ("area_halves", "GND,GNDR")]


def cpu():
    r = resource.getrusage(resource.RUSAGE_CHILDREN)
    return r.ru_utime + r.ru_stime


def with_project(src, dst):
    for ext in (".kicad_pro", ".kicad_dru"):
        s = os.path.splitext(src)[0] + ext
        if os.path.isfile(s):
            shutil.copy(s, os.path.splitext(dst)[0] + ext)


def kicad(board, pcb, nets, name):
    """KiCad's DRC of `pcb` with the pour of `nets` laid and refilled."""
    work = pcb
    if nets:
        poured = os.path.join(OUT, name + ".poured.kicad_pcb")
        r = subprocess.run([POUR_STITCH, pcb, "--net", nets, "--pour", "F.Cu,B.Cu", "-o", poured],
                           capture_output=True, text=True)
        if r.returncode != 0 or not os.path.isfile(poured):
            return {"error": "pour failed: " + (r.stderr.strip() or r.stdout.strip())[:200]}
        work = poured
    filled = os.path.join(OUT, name + ".filled.kicad_pcb")
    shutil.copy(work, filled)
    with_project(board, filled)
    drc = os.path.join(OUT, name + ".drc.json")
    subprocess.run(["kicad-cli", "pcb", "drc", "--refill-zones", "--save-board", "--format", "json",
                    "-o", drc, filled], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        d = json.load(open(drc))
    except (OSError, ValueError) as e:
        return {"error": f"no DRC report ({e})"}
    own = set(nets.split(",")) if nets else None
    of_nets = 0
    unconnected = d.get("unconnected_items", [])
    for item in unconnected:
        names = set()
        for it in item.get("items", []):
            m = re.search(r"\[([^\]]*)\]", it.get("description", ""))
            if m:
                names.add(m.group(1))
        if own is None or names & own:
            of_nets += 1
    kinds = {}
    for v in d.get("violations", []):
        kinds[v.get("type", "?")] = kinds.get(v.get("type", "?"), 0) + 1
    return {
        "unconn": f"{of_nets}/{len(unconnected)}",
        "clear": kinds.get("clearance", 0),
        "dangling": kinds.get("via_dangling", 0) + kinds.get("track_dangling", 0),
    }


def run(board, nets, seed, mode, args, use_kicad, keep):
    base = os.path.splitext(os.path.basename(board))[0]
    name = f"{base}.s{seed}.{mode}"
    pcb = os.path.join(OUT, name + ".kicad_pcb")
    report = os.path.join(OUT, name + ".json")
    cmd = [GB, "route", board, "-o", pcb, "--seed", str(seed), "--report", report] + args
    cmd += ["--areas", mode]
    if mode == "fill":
        for n in nets.split(",") if nets else []:
            cmd += ["--pour", n]
    t0 = cpu()
    r = subprocess.run(cmd, capture_output=True, text=True)
    seconds = cpu() - t0
    row = {"board": base, "seed": seed, "mode": mode}
    if r.returncode != 0 or not os.path.isfile(report):
        row["error"] = (r.stderr.strip().splitlines() or ["route failed"])[-1][:120]
        return row
    j = json.load(open(report))
    m = re.search(r"clearance check: (\d+) violations at (\d+) spots", r.stdout)
    row.update({
        "routed": f'{j["routed"]}/{j["connections"]}',
        "pieces/spots": f"{m.group(1)}/{m.group(2)}" if m else "?",
        "vias": j["vias"],
        "island vias": j.get("area_vias", 0),
        "mm": round(j["length_mm"]),
        "cpu s": round(seconds, 1),
        "model s": f'{j.get("area_model_seconds", 0.0):.1f} ({j.get("area_model_runs", 0)})',
    })
    # the model on the routed board, with the pour as it will be laid
    cmd = [GB, "areas", pcb, "--json", "--no-doubtful"]
    if nets:
        cmd += ["--nets", nets, "--pour", "F.Cu,B.Cu"]
    a = subprocess.run(cmd, capture_output=True, text=True)
    if a.returncode == 0:
        found = json.loads(a.stdout)["nets"]
        row["open pads"] = sum(n["pads"] - n["reached"] for n in found)
        row["groups"] = sum(n["expected_unconnected"] for n in found)
        row["not reached"] = " ".join(u["pad"] for n in found for u in n["unreached"])
    else:
        row["open pads"] = row["groups"] = "?"
    if use_kicad:
        row.update(kicad(board, pcb, nets, name))
    if not keep:
        for f in os.listdir(OUT):
            if f.startswith(name + "."):
                os.remove(os.path.join(OUT, f))
    return row


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("boards", nargs="*")
    p.add_argument("--seeds", default="1,2,3")
    p.add_argument("--args", default="--runs 4 --generations 1 --geometry-generations 1 --no-move-parts")
    p.add_argument("--fill-args", default="")
    p.add_argument("--modes", default="tracks,fill")
    p.add_argument("--kicad", action="store_true")
    p.add_argument("--keep", action="store_true")
    o = p.parse_args()
    if set(o.modes.split(",")) - {"tracks", "fill"}:
        sys.exit("--modes takes tracks, fill or both")
    if o.kicad and not shutil.which("kicad-cli"):
        sys.exit("kicad-cli not found: run this where KiCad is installed, or without --kicad")
    for f in [GB] + ([POUR_STITCH] if o.kicad else []):
        if not os.path.isfile(f):
            sys.exit(f"{f} is missing: cargo build --release -p gummiband-cli"
                     + (" --example pour_stitch" if f == POUR_STITCH else ""))
    if o.boards:
        plan = [(b.partition("=")[0], b.partition("=")[2]) for b in o.boards]
    else:
        plan = [(os.path.join(ROOT, "bench", "boards", n + ".kicad_pcb"), "GND") for n in SUITE]
        plan += [(os.path.join(ROOT, "bench", "boards", "areas", n + ".kicad_pcb"), nets) for n, nets in AREA_BOARDS]
    os.makedirs(OUT, exist_ok=True)
    head = ["board", "seed", "mode", "routed", "open pads", "groups", "pieces/spots", "vias", "island vias", "mm", "cpu s",
            "model s"]
    if o.kicad:
        head += ["unconn", "clear", "dangling"]
    head += ["not reached", "error"]
    rows = []
    for board, nets in plan:
        if not os.path.isfile(board):
            print(f"{board}: no such file", file=sys.stderr)
            continue
        for seed in [int(s) for s in o.seeds.split(",")]:
            for mode in o.modes.split(","):
                args = o.args.split() + (o.fill_args.split() if mode == "fill" else [])
                row = run(board, nets, seed, mode, args, o.kicad, o.keep)
                rows.append(row)
                print("  ".join(f"{k}={row[k]}" for k in head if row.get(k) not in (None, "")), flush=True)
    table = [head] + [[str(r.get(k, "")) for k in head] for r in rows]
    widths = [max(len(r[i]) for r in table) for i in range(len(head))]
    text = "\n".join("  ".join(c.ljust(w) for c, w in zip(r, widths)).rstrip() for r in table) + "\n"
    print()
    print(text, end="")
    with open(os.path.join(OUT, "results.txt"), "w") as f:
        f.write(text)
    json.dump(rows, open(os.path.join(OUT, "results.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
