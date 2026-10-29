"""Summarize a KiCad DRC JSON report: counts per type and the worst clearance violations."""
import collections
import json
import re
import sys

d = json.load(open(sys.argv[1]))
n = int(sys.argv[2]) if len(sys.argv) > 2 else 12
c = collections.Counter(v["type"] for v in d["violations"] if not v["type"].startswith(("silk", "lib_")))
print("unconnected", len(d["unconnected_items"]), dict(c))
rows = []
for v in d["violations"]:
    m = re.search(r"clearance ([0-9.]+) mm; actual ([0-9.]+)", v["description"])
    if not m:
        continue
    deficit = float(m.group(1)) - float(m.group(2))
    rows.append((deficit, v["type"], [(i["description"][:60], round(i["pos"]["x"], 3), round(i["pos"]["y"], 3)) for i in v["items"]]))
rows.sort(reverse=True)
buckets = collections.Counter(next(i for i, x in enumerate([0.002, 0.01, 0.03, 0.1, 9]) if r[0] < x) for r in rows)
print("deficit <2um <10um <30um <100um more:", [buckets[i] for i in range(5)])
for r in rows[:n]:
    print(round(r[0], 3), r[1], r[2])
for u in d["unconnected_items"][:n]:
    print("unconnected:", [i["description"][:60] for i in u["items"]])
