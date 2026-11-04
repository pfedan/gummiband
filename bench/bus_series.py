#!/usr/bin/env python3
"""Route synthetic boards with and without a set of route arguments, several seeds each, and
compare completion, vias, length, wall time, Gummiband's own clearance check and the bus
metrics of bus_metrics.py.

Usage: bench/bus_series.py OUTDIR [--boards a,b] [--seeds 1,2,3] [--variant name=args ...]
       [--route-args "--runs 4 --generations 1 --geometry-generations 0"] [--kicad-drc]

Default variants: base (no extra arguments) and buses (--patterns buses). Results go to
OUTDIR/results.jsonl (one line per run, appended, runs already there are skipped) and a table
of the means per board and variant on stdout. Run from the repository root after
`cargo build --release -p gummiband-cli`.
"""
import argparse
import json
import os
import re
import shutil
import statistics
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARDS = os.path.join(ROOT, "bench", "boards")
GB = os.path.join(ROOT, "target", "release", "gummiband")


def run_one(board, variant, args, seed, outdir, base_args, timeout, kicad):
    name = f"{board}.{variant}.s{seed}"
    out = os.path.join(outdir, name + ".kicad_pcb")
    for ext in (".kicad_pro", ".kicad_dru"):
        src = os.path.join(BOARDS, board + ext)
        if os.path.exists(src):
            shutil.copy(src, os.path.join(outdir, name + ext))
    cmd = [GB, "route", os.path.join(BOARDS, board + ".kicad_pcb"), "-o", out, "--seed", str(seed)]
    cmd += base_args + args
    t0 = time.time()
    try:
        p = subprocess.run(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           text=True, errors="replace", timeout=timeout)
        text, ok = p.stdout, p.returncode == 0
    except subprocess.TimeoutExpired:
        text, ok = "", False
    dt = time.time() - t0
    with open(os.path.join(outdir, name + ".log"), "w") as f:
        f.write(" ".join(cmd) + "\n" + text)
    row = dict(board=board, variant=variant, seed=seed, ok=ok, time=round(dt, 2))
    m = re.findall(r"^best: seed \d+ (\d+)/(\d+) routed, (\d+) vias, ([\d.]+) mm", text, re.M)
    if m:
        r, n, v, L = m[-1]
        row.update(routed=int(r), conns=int(n), vias=int(v), length=float(L))
    m = re.findall(r"^clearance check: (\d+) violations at (\d+) spots", text, re.M)
    if m:
        row.update(violations=int(m[-1][0]), spots=int(m[-1][1]))
    if ok and os.path.exists(out):
        bm = subprocess.run([sys.executable, os.path.join(ROOT, "bench", "bus_metrics.py"), out, "--json"],
                            stdout=subprocess.PIPE, text=True)
        try:
            row["buses"] = json.loads(bm.stdout)
        except json.JSONDecodeError:
            pass
        if kicad:
            drc = os.path.join(outdir, name + ".drc.json")
            subprocess.run(["kicad-cli", "pcb", "drc", "--refill-zones", "--format", "json", "-o", drc, out],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            try:
                d = json.load(open(drc))
                kinds = {}
                for v in d.get("violations", []):
                    if v.get("severity") == "error" and not v["type"].startswith(("silk", "lib_")):
                        kinds[v["type"]] = kinds.get(v["type"], 0) + 1
                row["drc"] = kinds
                row["unconnected"] = len(d.get("unconnected_items", []))
            except (OSError, json.JSONDecodeError, KeyError):
                pass
    return row


def bus_of(row):
    """Vias and cohesion over the buses of a run: (vias, mean cohesion)."""
    b = row.get("buses") or {}
    b = {k: v for k, v in b.items() if v["nets"] >= 4}
    if not b:
        return None, None
    vias = sum(v["vias"] for v in b.values())
    coh = [v["cohesion"] for v in b.values() if v["cohesion"] is not None]
    return vias, (sum(coh) / len(coh) if coh else None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("outdir")
    ap.add_argument("--boards", default="")
    ap.add_argument("--seeds", default="1,2,3")
    ap.add_argument("--variant", action="append", default=[])
    ap.add_argument("--route-args", default="--runs 4 --generations 1 --geometry-generations 0")
    ap.add_argument("--timeout", type=float, default=900)
    ap.add_argument("--kicad-drc", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)
    boards = [b for b in a.boards.split(",") if b] or sorted(
        f[:-10] for f in os.listdir(BOARDS) if f.endswith(".kicad_pcb"))
    variants = [v.split("=", 1) for v in a.variant] or [["base", ""], ["buses", "--patterns buses"]]
    seeds = [int(s) for s in a.seeds.split(",")]
    res_path = os.path.join(a.outdir, "results.jsonl")
    done = set()
    if os.path.exists(res_path):
        for line in open(res_path):
            r = json.loads(line)
            done.add((r["board"], r["variant"], r["seed"]))
    for board in boards:
        for seed in seeds:
            for vname, vargs in variants:
                if (board, vname, seed) in done:
                    continue
                row = run_one(board, vname, vargs.split(), seed, a.outdir, a.route_args.split(), a.timeout,
                              a.kicad_drc)
                with open(res_path, "a") as f:
                    f.write(json.dumps(row) + "\n")
                print(json.dumps({k: v for k, v in row.items() if k != "buses"}), flush=True)
    rows = [json.loads(line) for line in open(res_path)]
    table(rows, boards, [v[0] for v in variants])


def table(rows, boards, variants):
    def mean(xs):
        xs = [x for x in xs if x is not None]
        return statistics.mean(xs) if xs else None

    def f(x, nd=1):
        return "-" if x is None else f"{x:.{nd}f}"

    print("| Board | Variant | Open | Vias | Length | Spots | Time s | Bus vias | Cohesion |")
    print("| --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    for b in boards:
        for v in variants:
            rs = [r for r in rows if r["board"] == b and r["variant"] == v]
            if not rs:
                continue
            opn = mean([r["conns"] - r["routed"] if "routed" in r else None for r in rs])
            bus = [bus_of(r) for r in rs]
            print(f"| {b} | {v} | {f(opn, 2)} | {f(mean([r.get('vias') for r in rs]))} | "
                  f"{f(mean([r.get('length') for r in rs]), 0)} | {f(mean([r.get('spots') for r in rs]), 2)} | "
                  f"{f(mean([r['time'] for r in rs]))} | {f(mean([x[0] for x in bus]))} | "
                  f"{f(mean([x[1] for x in bus]), 2)} |")


if __name__ == "__main__":
    main()
