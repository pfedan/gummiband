#!/usr/bin/env python3
"""Route the boards of the geometry test stand and judge them with the router's own check.

Usage: run_own.py ROUTER BOARDS_DIR OUT_DIR [--only SUBSTRING] [--env NAME=VALUE ...] [--refs RECHECK]
                  [--expect FILE | --update FILE]
Prints one line per case: per form the routed connections and, if any, the number of spots of the
check; a routed and clean board also shows its lines and arcs. A cell that is not the right answer
ends with "!". The right answer: fully routed and clean for loose, tight20, tight and exact; for
short not fully routed and clean. The last line is the score.
--refs RECHECK: judge the reference solutions with the `recheck` example instead of routing.
--expect FILE: compare routed connections and spots per board with FILE, exit 1 on a difference.
--update FILE: write FILE.
No KiCad needed. Standard library only.
"""
import json, os, re, subprocess, sys, time
from concurrent.futures import ThreadPoolExecutor

def main():
    router, boards, out = sys.argv[1:4]
    only = sys.argv[sys.argv.index("--only") + 1] if "--only" in sys.argv else ""
    refs = sys.argv[sys.argv.index("--refs") + 1] if "--refs" in sys.argv else None
    env = dict(os.environ)
    for i, a in enumerate(sys.argv):
        if a == "--env":
            k, _, v = sys.argv[i + 1].partition("="); env[k] = v
    os.makedirs(out, exist_ok=True)
    cases = [c for c in json.load(open(os.path.join(boards, "cases.json"))) if only in c["board"]]
    def one(c):
        src = os.path.join(boards, c["board"] + ".kicad_pcb"); dst = os.path.join(out, c["board"] + ".kicad_pcb")
        t0 = time.time()
        if refs:
            r = subprocess.run([refs, os.path.join(boards, c["board"] + ".ref.kicad_pcb")], capture_output=True, text=True, env=env)
            m = re.search(r"(\d+) violations at (\d+) spots", r.stdout)
            miss = [float(x) * 1000 for x in re.findall(r": ([\d.]+) missing", r.stdout)]
            return c, dict(routed="ref", viol=int(m.group(1)) if m else -1, spots=int(m.group(2)) if m else -1,
                           worst=max(miss) if miss else 0.0, wall=time.time() - t0)
        try:
            r = subprocess.run([router, "route", src, "-o", dst, "--seed", "1", "--no-move-parts"], capture_output=True, text=True, env=env, timeout=300)
            txt = r.stdout
        except subprocess.TimeoutExpired:
            txt = ""
        open(os.path.join(out, c["board"] + ".log"), "w").write(txt)
        b = re.search(r"best: seed \d+ (\d+/\d+) routed", txt); m = re.search(r"clearance check: (\d+) violations at (\d+) spots", txt)
        items = open(dst).read() if os.path.isfile(dst) else ""
        return c, dict(routed=b.group(1) if b else "?", viol=int(m.group(1)) if m else -1, spots=int(m.group(2)) if m else -1,
                       lines=len(re.findall(r"\(segment\b", items)), arcs=len(re.findall(r"\(arc\b", items)), wall=time.time() - t0)
    with ThreadPoolExecutor(2) as ex:
        res = list(ex.map(one, cases))
    json.dump([dict(c, **r) for c, r in res], open(os.path.join(out, "results.json"), "w"), indent=1)
    names = []
    for c, _ in res:
        if c["case"] not in names: names.append(c["case"])
    by = {c["board"]: (c, r) for c, r in res}
    forms = ["loose", "tight20", "tight", "exact", "short"]
    print(f"{'case':24s} " + " ".join(f"{f:>16s}" for f in forms))
    good = 0; total = 0
    for n in names:
        cells = []
        for f in forms:
            if f"{n}.{f}" not in by: cells.append("-"); continue
            c, r = by[f"{n}.{f}"]
            full = r["routed"] == f"{c['connections']}/{c['connections']}"
            if refs:
                s = f"v{r['viol']} {r['worst']:.1f}um" if r["viol"] else "clean"
            else:
                s = r["routed"] + (f" p{r['spots']}" if r["spots"] else "") + (f" {r.get('lines')}L{r.get('arcs')}A" if full and not r["spots"] else "")
                total += 1
                ok = (full and r["spots"] == 0) if f != "short" else (not full and r["spots"] == 0)
                good += ok; s += "" if ok else " !"
            cells.append(s)
        print(f"{n:24s} " + " ".join(f"{s:>16s}" for s in cells))
    if refs:
        return
    print(f"right: {good} of {total}")
    lines = [f"{c['board']} {r['routed']} {r['spots']}" for c, r in res]
    if "--update" in sys.argv:
        open(sys.argv[sys.argv.index("--update") + 1], "w").write("\n".join(lines) + "\n")
    if "--expect" in sys.argv:
        want = open(sys.argv[sys.argv.index("--expect") + 1]).read().split("\n")
        want = {w.split(" ")[0]: w for w in want if w and only in w.split(" ")[0]}
        diff = [(want.get(l.split(" ")[0], "(new)"), l) for l in lines if want.get(l.split(" ")[0]) != l]
        for w, l in diff:
            print(f"expected: {w}\n     got: {l}")
        print("as expected" if not diff else f"{len(diff)} boards differ")
        sys.exit(1 if diff else 0)

if __name__ == "__main__":
    main()
