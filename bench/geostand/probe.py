#!/usr/bin/env python3
"""Find the least slack at which the router lays a case. Usage: probe.py ROUTER CASE [slacks in um ...]"""
import sys, os, json, subprocess, re
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_geostand as g
router, case = sys.argv[1], sys.argv[2]
slacks = [float(s) for s in sys.argv[3:]] or [0, 2, 5, 8, 10, 12, 16, 20, 25, 30, 40, 60, 100]
os.makedirs("probe", exist_ok=True)
for um in slacks:
    b, desc, need, conns = g.CASES[case](um / 1000)
    base = f"probe/{case}.{um:g}"
    open(base + ".kicad_pcb", "w").write(b.text())
    json.dump(b.project(os.path.basename(base)), open(base + ".kicad_pro", "w"))
    r = subprocess.run([router, "route", base + ".kicad_pcb", "-o", base + ".out.kicad_pcb", "--seed", "1", "--no-move-parts"],
                       capture_output=True, text=True, env=dict(os.environ))
    best = re.search(r"(\d+/\d+) routed", r.stdout)
    chk = re.search(r"clearance check: (.*)", r.stdout)
    print(f"{case} +{um:g} um: {best.group(1) if best else '?'}  {chk.group(1) if chk else ''}")
