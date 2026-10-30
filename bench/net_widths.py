#!/usr/bin/env python3
"""Ground truth of a routed KiCad board: per net the track width the designer used most
(length weighted), plus the totals to compare a Gummiband result against.

Usage: bench/net_widths.py ORIGINAL.kicad_pcb [--json widths.json]
Prints total track length, via count and the width table; with --json writes
{"net": width_mm} for `gummiband route --net-widths`.
"""
import argparse
import collections
import json
import math
import re


def items(text):
    """(kind, body) of the root level segments, arcs and vias (KiCad 7 to 10 layouts)."""
    for m in re.finditer(r"\((segment|arc|via)\s((?:[^()]|\([^()]*\))*)\)", text):
        yield m.group(1), m.group(2)


def net_names(text):
    """Net number -> name for files with a root net table (KiCad up to 9)."""
    return {n: name for n, name in re.findall(r'^\s*\(net (\d+) "([^"]*)"\)', text, re.M)}


def length(kind, body):
    p = lambda k: tuple(map(float, re.search(r"\(%s ([\d.\-]+) ([\d.\-]+)\)" % k, body).groups()))
    s, e = p("start"), p("end")
    if kind == "segment":
        return math.dist(s, e)
    m = p("mid")
    # arc through three points
    ax, ay = s
    bx, by = m
    cx, cy = e
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12:
        return math.dist(s, e)
    ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d
    uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d
    r = math.dist((ux, uy), s)
    chord = lambda p, q: 2 * math.asin(min(1.0, math.dist(p, q) / (2 * r)))
    return r * (chord(s, m) + chord(m, e))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("board")
    ap.add_argument("--json")
    a = ap.parse_args()
    text = open(a.board, encoding="utf-8").read()
    names = net_names(text)
    per_net = collections.defaultdict(collections.Counter)
    total, vias = 0.0, 0
    for kind, body in items(text):
        net = re.search(r'\(net (?:(\d+)|"([^"]*)")(?: "([^"]*)")?\)', body)
        if not net:
            continue
        name = net.group(3) or net.group(2) or names.get(net.group(1), net.group(1))
        if kind == "via":
            vias += 1
            continue
        w = float(re.search(r"\(width ([\d.]+)\)", body).group(1))
        L = length(kind, body)
        total += L
        per_net[name][w] += L
    widths = {n: c.most_common(1)[0][0] for n, c in per_net.items() if n}
    print(f"tracks {total:.0f} mm, vias {vias}, nets with tracks {len(widths)}")
    for w, n in sorted(collections.Counter(widths.values()).items()):
        print(f"  width {w} mm: {n} nets")
    if a.json:
        json.dump(widths, open(a.json, "w"), indent=1, sort_keys=True)


if __name__ == "__main__":
    main()
