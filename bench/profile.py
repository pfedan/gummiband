#!/usr/bin/env python3
"""Summarise a samply profile of a gummiband run (macOS, no Xcode needed).

Record:  samply record --save-only --unstable-presymbolicate -r 1000 -o run.json.gz -- \\
             target/release/gummiband route BOARD -o out.kicad_pcb
Then:    bench/profile.py run.json.gz top [N]          self and total time per function
         bench/profile.py run.json.gz parts             busy time by part (search, relaxation, ...)
         bench/profile.py run.json.gz lines PAT FILE BIN [N]
                                                        self time of functions matching PAT by
                                                        source line of FILE (atos, inlined frames)

Shares are of all samples (`top`, idle threads included) or of the busy ones (`parts`, `lines`).
"""
import bisect
import collections
import gzip
import json
import re
import subprocess
import sys

IDLE = ("__psynch_cvwait", "swtch_pri", "cthread_yield", "__psynch_mutexwait", "thread_switch")
PARTS = [("taut", "relax::taut"), ("relaxation", "relax::relax_while"), ("follow", "follow::"),
         ("search", "route::Router"), ("check", "check::"), ("substrate", "topo::Substrate"),
         ("realize", "realize::"), ("tune", "tune::"), ("improve", "improve::"),
         ("place", "gummiband_place"), ("kicad io", "gummiband_kicad")]


def load(path):
    p = json.load(gzip.open(path))
    s = json.load(open(path.replace(".json.gz", ".json.syms.json")))
    st = s["string_table"]
    syms = {}
    for d in s["data"]:
        tab = sorted(d["symbol_table"], key=lambda e: e["rva"])
        syms[d["debug_name"]] = ([e["rva"] for e in tab], [st[e["symbol"]] for e in tab])
    return p, syms


def frames(p, syms, t):
    """Per frame of thread t: (library, address, function name)."""
    ft, fu, rt = t["frameTable"], t["funcTable"], t["resourceTable"]
    out = []
    for i in range(ft["length"]):
        res = fu["resource"][ft["func"][i]]
        lib = p["libs"][rt["lib"][res]]["debugName"] if res is not None and res >= 0 else "?"
        rv, nm = syms.get(lib, ([], []))
        k = bisect.bisect_right(rv, ft["address"][i]) - 1
        name = re.sub(r"::h[0-9a-f]{16}$", "", nm[k]) if k >= 0 else f"{lib}+{ft['address'][i]:#x}"
        out.append((lib, ft["address"][i], name))
    return out


def stacks(p, syms):
    """Yields (frames leaf first, weight) per sample of every thread (an idle thread's samples
    come merged, with the number of ticks as weight)."""
    for t in p["threads"]:
        fr = frames(p, syms, t)
        stk = t["stackTable"]
        memo = {}
        weights = t["samples"].get("weight") or [1] * t["samples"]["length"]
        for si, w in zip(t["samples"]["stack"], weights):
            if si is None:
                continue
            if si not in memo:
                c, y = [], si
                while y is not None:
                    c.append(fr[stk["frame"][y]])
                    y = stk["prefix"][y]
                memo[si] = c
            yield memo[si], w or 1


def short(n):
    return re.sub(r"<[^<>]*>", "", n)[:150] if len(n) > 150 else n


def top(p, syms, n):
    own, total, all_ = collections.Counter(), collections.Counter(), 0
    for c, w in stacks(p, syms):
        all_ += w
        own[c[0][2]] += w
        for f in {x[2] for x in c}:
            total[f] += w
    print(f"samples {all_}\n--- self")
    for f, k in own.most_common(n):
        print(f"{100 * k / all_:6.2f} % {short(f)}")
    print("--- total")
    for f, k in total.most_common(n):
        print(f"{100 * k / all_:6.2f} % {short(f)}")


def parts(p, syms):
    c, busy, idle = collections.Counter(), 0, 0
    for s, w in stacks(p, syms):
        if any(x in s[0][2] for x in IDLE):
            idle += w
            continue
        busy += w
        c[next((name for f in s for name, pat in PARTS if pat in f[2]), "other")] += w
    print(f"busy {busy}, idle {idle} ({100 * idle / max(busy + idle, 1):.0f} % of the samples)")
    for k, v in c.most_common():
        print(f"{100 * v / max(busy, 1):6.1f} % {k}")


def lines(p, syms, pat, file, binary, n):
    bn = binary.split("/")[-1]
    addrs, busy = collections.Counter(), 0
    for s, w in stacks(p, syms):
        if any(x in s[0][2] for x in IDLE):
            continue
        busy += w
        lib, a, name = s[0]
        if lib == bn and re.search(pat, name):
            addrs[a] += w
    items = list(addrs.items())
    out = subprocess.run(["atos", "-i", "-o", binary, "-l", "0x100000000"] +
                         [hex(0x100000000 + a) for a, _ in items], capture_output=True, text=True).stdout
    agg = collections.Counter()
    for (a, k), b in zip(items, out.strip("\n").split("\n\n")):
        ls = [m.group(1) for m in (re.search(r"\(([^()]*:\d+)\)\s*$", x) for x in b.split("\n") if x) if m]
        agg[next((x for x in ls if x.startswith(file)), ls[0] if ls else "?")] += k
    sel = sum(addrs.values())
    print(f"{sel} samples, {100 * sel / max(busy, 1):.1f} % of the busy ones")
    for k, v in agg.most_common(n):
        print(f"{100 * v / max(sel, 1):6.2f} % {k}")


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    p, syms = load(sys.argv[1])
    what = sys.argv[2]
    if what == "top":
        top(p, syms, int(sys.argv[3]) if len(sys.argv) > 3 else 40)
    elif what == "parts":
        parts(p, syms)
    elif what == "lines":
        lines(p, syms, sys.argv[3], sys.argv[4], sys.argv[5], int(sys.argv[6]) if len(sys.argv) > 6 else 40)
    else:
        print(__doc__)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
