#!/usr/bin/env python3
"""Bus metrics of a routed board: per bus the vias of its lines and how well the lines stay
together (cohesion).

Usage: bench/bus_metrics.py ROUTED.kicad_pcb [--pitch 0.35] [--json]

A bus is a set of nets named <prefix><number> with the same prefix (D0..D15, A0..A11, X0..X15),
at least 3 of them. Cohesion is the share of the track length of the lines that runs next to
another line of the same bus on the same layer (centre to centre within 1.6 track pitches),
leaving out the first and last 2 mm of every line, where the lines fan out to their pads.
"""
import argparse
import collections
import json
import math
import re


def items(text):
    for m in re.finditer(r"\((segment|arc|via)\s((?:[^()]|\((?:[^()]|\([^()]*\))*\))*)\)", text):
        yield m.group(1), m.group(2)


def pts(body, key):
    m = re.search(r"\(%s ([\d.\-]+) ([\d.\-]+)\)" % key, body)
    return (float(m.group(1)), float(m.group(2))) if m else None


def arc_points(a, m, b, step=0.2):
    ax, ay = a
    bx, by = m
    cx, cy = b
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12:
        return [a, b]
    ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d
    uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d
    r = math.dist((ux, uy), a)
    t0 = math.atan2(ay - uy, ax - ux)
    tm = math.atan2(by - uy, bx - ux)
    t1 = math.atan2(cy - uy, cx - ux)

    def norm(t):
        return (t - t0) % (2 * math.pi)

    sweep = norm(t1)
    if norm(tm) > sweep:  # the arc runs the other way round
        sweep -= 2 * math.pi
    n = max(2, int(abs(sweep) * r / step) + 1)
    return [(ux + r * math.cos(t0 + sweep * k / (n - 1)), uy + r * math.sin(t0 + sweep * k / (n - 1))) for k in range(n)]


def seg_dist(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / l2))
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("board")
    ap.add_argument("--pitch", type=float, default=0.35)
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()
    text = open(a.board).read()
    polys = collections.defaultdict(list)  # net -> [(layer, [points])]
    vias = collections.Counter()
    for kind, body in items(text):
        net = re.search(r'\(net "([^"]*)"\)', body)
        if not net:
            continue
        net = net.group(1)
        if kind == "via":
            vias[net] += 1
            continue
        layer = re.search(r'\(layer "([^"]+)"\)', body).group(1)
        s, e = pts(body, "start"), pts(body, "end")
        line = arc_points(s, pts(body, "mid"), e) if kind == "arc" else [s, e]
        polys[net].append((layer, line))
    buses = collections.defaultdict(list)
    for net in set(polys) | set(vias):
        m = re.fullmatch(r"([A-Za-z_]+?)(\d+)", net)
        if m:
            buses[m.group(1)].append(net)
    out = {}
    for prefix, nets in sorted(buses.items()):
        if len(nets) < 3:
            continue
        nets.sort(key=lambda n: int(re.search(r"\d+$", n).group()))
        total = near = 0.0
        for n in nets:
            others = [(l, seg) for o in nets if o != n for (l, line) in polys[o] for seg in zip(line, line[1:])]
            # samples every 0.1 mm
            samples = []
            for layer, line in polys[n]:
                for p, q in zip(line, line[1:]):
                    L = math.dist(p, q)
                    k = max(1, int(L / 0.1))
                    for i in range(k):
                        t = (i + 0.5) / k
                        samples.append((layer, (p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t), L / k))
            # the pieces come unordered: leave out the samples within 2 mm of the open ends of
            # the net's copper (its pads) instead of the first and last 2 mm of a walk
            ends = []
            for layer, line in polys[n]:
                ends += [line[0], line[-1]]
            cnt = collections.Counter((round(x, 3), round(y, 3)) for x, y in ends)
            tips = [p for p, c in cnt.items() if c == 1]
            for layer, p, dl in samples:
                if any(math.dist(p, t) < 2.0 for t in tips):
                    continue
                total += dl
                if any(l == layer and seg_dist(p, s0, s1) < 1.6 * a.pitch for l, (s0, s1) in others):
                    near += dl
        out[prefix] = dict(nets=len(nets), vias=sum(vias[n] for n in nets),
                           length=round(sum(math.dist(p, q) for n in nets for _, line in polys[n]
                                            for p, q in zip(line, line[1:])), 1),
                           cohesion=round(near / total, 3) if total else None)
    if a.json:
        print(json.dumps(out))
    else:
        for k, v in out.items():
            print(f"{k:6s} {v['nets']:3d} nets  {v['vias']:3d} vias  {v['length']:8.1f} mm  cohesion {v['cohesion']}")


if __name__ == "__main__":
    main()
