#!/usr/bin/env python3
"""Route every board of the geometry test stand and judge it with KiCad's DRC.

Usage: run_geostand.py ROUTER_BINARY BOARDS_DIR OUT_DIR [--seed N] [--only SUBSTRING] [--env NAME=VALUE ...]
For each board: `ROUTER route BOARD -o OUT --seed N --no-move-parts`, then
`kicad-cli pcb drc --all-track-errors --refill-zones`. Writes OUT_DIR/results.json and prints a table.
Places: KiCad entries of one kind with the same pair of items (a via or pad counts as one item,
tracks by net) merged when they lie within 0.5 mm. Standard library only.
"""
import json, math, os, re, shutil, subprocess, sys, time, collections

COPPER = {"clearance": "clr", "copper_edge_clearance": "edge", "hole_clearance": "hole", "hole_to_hole": "h2h",
          "solder_mask_bridge": "mask", "shorting_items": "short", "tracks_crossing": "cross"}


def items_of(pcb):
    t = open(pcb).read()
    n = collections.Counter(m.group(1) for m in re.finditer(r"^\s*\((segment|arc|via)\b", t, re.M))
    return n["segment"], n["arc"], n["via"]


def places(drc):
    ents = []
    for v in drc["violations"]:
        kind = COPPER.get(v["type"])
        if not kind or not v["items"]:
            continue
        m = re.search(r"; actual ([\d.]+)", v["description"]); r = re.search(r"(?:clearance|constraint) ([\d.]+) mm", v["description"])
        short = (float(r.group(1)) - float(m.group(1))) * 1000 if (m and r) else None
        key = [kind]
        for i in v["items"]:
            d = i["description"]; net = re.search(r"\[([^\]]*)\]", d)
            word = d.split(" ")[0]
            key.append((net.group(1) if net else "-") + ("#" + i.get("uuid", "") if word in ("Via", "Pad", "PTH", "Segment") else ""))
        ents.append(dict(key=tuple([key[0]] + sorted(key[1:])), p=(v["items"][0]["pos"]["x"], v["items"][0]["pos"]["y"]),
                         short=short, desc=[i["description"][:60] for i in v["items"]]))
    groups = collections.defaultdict(list)
    for e in ents:
        groups[e["key"]].append(e)
    out = []
    for g in groups.values():
        left = list(g)
        while left:
            cl = [left.pop()]; i = 0
            while i < len(cl):
                near = [e for e in left if math.dist(e["p"], cl[i]["p"]) <= 0.5 or any("#" in k for k in e["key"][1:])]
                for e in near:
                    left.remove(e)
                cl += near; i += 1
            out.append(cl)
    return out


def main():
    router, boards, out = sys.argv[1], sys.argv[2], sys.argv[3]
    seed = sys.argv[sys.argv.index("--seed") + 1] if "--seed" in sys.argv else "1"
    only = sys.argv[sys.argv.index("--only") + 1] if "--only" in sys.argv else ""
    env = dict(os.environ)
    for i, a in enumerate(sys.argv):
        if a == "--env":
            k, _, v = sys.argv[i + 1].partition("="); env[k] = v
    os.makedirs(out, exist_ok=True)
    cases = [c for c in json.load(open(os.path.join(boards, "cases.json"))) if only in c["board"]]
    # only cases whose reference solution passed check_refs.py
    refcheck = os.path.join(boards, "refcheck.json")
    if os.path.isfile(refcheck):
        ok = json.load(open(refcheck))
        skipped = [c["board"] for c in cases if not ok.get(c["board"], {}).get("ok")]
        if skipped:
            print("skipped, reference not clean:", " ".join(skipped))
        cases = [c for c in cases if ok.get(c["board"], {}).get("ok")]
    else:
        print("no refcheck.json: run check_refs.py first; all cases taken")
    rows = []
    for c in cases:
        src = os.path.join(boards, c["board"] + ".kicad_pcb")
        dst = os.path.join(out, c["board"] + ".kicad_pcb")
        shutil.copy(os.path.join(boards, c["board"] + ".kicad_pro"), os.path.join(out, c["board"] + ".kicad_pro"))
        t0 = time.time()
        r = subprocess.run([router, "route", src, "-o", dst, "--seed", seed, "--no-move-parts"], capture_output=True, text=True, env=env)
        wall = time.time() - t0
        open(os.path.join(out, c["board"] + ".route.log"), "w").write(r.stdout + "\n--- stderr\n" + r.stderr)
        best = re.search(r"best: seed \d+ (\d+)/(\d+) routed", r.stdout)
        routed = f"{best.group(1)}/{best.group(2)}" if best else ("fail" if r.returncode else "?")
        unrouted = re.findall(r"unrouted: (.*)", r.stdout)
        row = dict(c, routed=routed, unrouted=unrouted, wall_s=round(wall, 1), exit=r.returncode)
        if os.path.isfile(dst):
            drcf = os.path.join(out, c["board"] + ".drc.json")
            subprocess.run(["kicad-cli", "pcb", "drc", "--all-track-errors", "--refill-zones", "--format", "json", "-o", drcf, dst],
                           capture_output=True)
            drc = json.load(open(drcf))
            pl = places(drc)
            row["places"] = len(pl)
            row["by_kind"] = dict(collections.Counter(p[0]["key"][0] for p in pl))
            sh = [e["short"] for p in pl for e in p if e["short"] is not None]
            row["worst_um"] = round(max(sh), 1) if sh else 0.0
            row["unconnected"] = len(drc.get("unconnected_items", []))
            row["lines"], row["arcs"], row["vias"] = items_of(dst)
            row["where"] = [dict(kind=p[0]["key"][0], at=[round(x, 3) for x in max(p, key=lambda e: e["short"] or 0)["p"]],
                                 short_um=round(max((e["short"] or 0) for e in p), 1), items=max(p, key=lambda e: e["short"] or 0)["desc"],
                                 entries=len(p)) for p in pl]
        rows.append(row)
        ri = c.get("reference_items", {})
        print(f"{c['board']:34s} need {c['need_mm']:.3f} avail {c['available_mm']:.3f} ref {ri.get('lines', '-')}L/{ri.get('arcs', '-')}A {routed:>6s} unconn {row.get('unconnected', '-')!s:>2} places {row.get('places', '-')!s:>2} "
              f"{row.get('by_kind', {})} worst {row.get('worst_um', '-')} um  lines {row.get('lines', '-')} arcs {row.get('arcs', '-')} "
              f"vias {row.get('vias', '-')}  {wall:.1f}s", flush=True)
    json.dump(rows, open(os.path.join(out, "results.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
