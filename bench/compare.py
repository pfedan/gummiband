#!/usr/bin/env python3
"""Compare two gummiband binaries across the whole benchmark breadth, judged by the KiCad DRC.

Usage: bench/compare.py OLD_BIN NEW_BIN [--boards a,b,...] [--args "route args"] [--out DIR]

Boards: the synthetic suite (bench/boards), the prepared external boards (bench/work/ext:
arty, corne, ulx3s) and the Lichtschnitzel benchmark (bench/work/lichtschnitzel). Every board is
routed by each binary in turn, alone on the machine, then checked with
`kicad-cli pcb drc --refill-zones`. Violations that the unrouted board already has (courtyards,
padstacks, ...) are subtracted per type. The table shows completion, routing caused DRC errors
and time per board, then the totals; a change should win in sum and lose nowhere badly.
"""
import argparse
import re
import collections
import json
import os
import shutil
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BENCH = os.path.join(ROOT, "bench")
IGNORE = ("silk", "lib_", "copper_sliver", "isolated_copper", "padstack", "courtyard", "text_")


def boards():
    out = []
    syn = os.path.join(BENCH, "boards")
    for f in sorted(os.listdir(syn)):
        if f.endswith(".kicad_pcb"):
            out.append((f[:-10], os.path.join(syn, f)))
    for n in ("arty", "corne", "ulx3s"):
        p = os.path.join(BENCH, "work", "ext", n, n + ".kicad_pcb")
        if os.path.exists(p):
            out.append((n, p))
    p = os.path.join(BENCH, "work", "lichtschnitzel", "lichtschnitzel.kicad_pcb")
    if os.path.exists(p):
        out.append(("lichtschnitzel", p))
    return out


def drc(pcb):
    rep = pcb + ".drc.json"
    subprocess.run(["kicad-cli", "pcb", "drc", "--refill-zones", "--format", "json", "-o", rep, pcb],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        d = json.load(open(rep))
    except (OSError, ValueError):
        return None, collections.Counter()
    c = collections.Counter(v["type"] for v in d["violations"] if not v["type"].startswith(IGNORE))
    return len(d["unconnected_items"]), c


def copy_project(src, dst_dir, name):
    os.makedirs(dst_dir, exist_ok=True)
    stem = os.path.splitext(src)[0]
    for ext in (".kicad_pro", ".kicad_dru"):
        if os.path.exists(stem + ext):
            shutil.copy(stem + ext, os.path.join(dst_dir, name + ext))
    return os.path.join(dst_dir, name + ".kicad_pcb")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("old")
    ap.add_argument("new")
    ap.add_argument("--boards", default="")
    ap.add_argument("--args", default="--runs 14 --generations 1")
    ap.add_argument("--out", default=os.path.join(BENCH, "work", "compare"))
    a = ap.parse_args()
    wanted = [b for b in a.boards.split(",") if b]
    todo = [(n, p) for n, p in boards() if not wanted or n in wanted]
    rows = []
    for name, src in todo:
        base_dir = os.path.join(a.out, "base", name)
        base = copy_project(src, base_dir, name)
        shutil.copy(src, base)
        _, pre = drc(base)
        row = {"board": name}
        for tag, gb in (("old", a.old), ("new", a.new)):
            d = os.path.join(a.out, tag, name)
            pcb = copy_project(src, d, name)
            t0 = time.time()
            p = subprocess.run([gb, "route", src, "-o", pcb] + a.args.split(), stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True, errors="replace")
            dt = time.time() - t0
            open(pcb + ".log", "w").write(p.stdout)
            if p.returncode != 0 or not os.path.exists(pcb):
                row[tag] = dict(unconn=None, errors=None, time=dt, kinds={}, core=None)
                continue
            unconn, c = drc(pcb)
            kinds = {k: v - pre.get(k, 0) for k, v in c.items() if v - pre.get(k, 0) > 0}
            # the router's own check (KiCad caps its report at a few hundred per type)
            m = re.search(r"clearance check: (\d+) violations", p.stdout)
            core = int(m.group(1)) if m else None
            row[tag] = dict(unconn=unconn, errors=sum(kinds.values()), time=dt, kinds=kinds, core=core)
            print(f"{name:20s} {tag}: unconnected {unconn}, errors {row[tag]['errors']} {kinds}, "
                  f"own check {core}, {dt:.0f} s", flush=True)
        rows.append(row)
    print()
    print(f"{'board':20s} {'unconn old/new':>15s} {'errors old/new':>15s} {'own old/new':>13s} "
          f"{'time old/new':>14s}")
    tot = collections.Counter()
    for r in rows:
        o, n = r["old"], r["new"]
        fmt = lambda x: "fail" if x is None else str(x)
        print(f"{r['board']:20s} {fmt(o['unconn']) + '/' + fmt(n['unconn']):>15s} "
              f"{fmt(o['errors']) + '/' + fmt(n['errors']):>15s} "
              f"{fmt(o['core']) + '/' + fmt(n['core']):>13s} "
              f"{o['time']:.0f}/{n['time']:.0f} s".rjust(14))
        for tag, x in (("old", o), ("new", n)):
            tot[tag + "_unconn"] += x["unconn"] or 0
            tot[tag + "_errors"] += x["errors"] or 0
            tot[tag + "_core"] += x["core"] or 0
            tot[tag + "_time"] += x["time"]
    print(f"{'total':20s} {tot['old_unconn']}/{tot['new_unconn']:<10} {tot['old_errors']}/{tot['new_errors']:<10} "
          f"{tot['old_core']}/{tot['new_core']:<10} {tot['old_time']:.0f}/{tot['new_time']:.0f} s")


if __name__ == "__main__":
    sys.exit(main())
