#!/usr/bin/env python3
"""Draws the schematic SVG figures of the user guide.

Run from anywhere: python3 docs/guide/img/src/schematics.py
The SVGs are written next to this folder, into docs/guide/img/.
"""
import math
import os

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")

TOP = "#c8373a"      # copper on the top layer
BOT = "#3f72c0"      # copper on the bottom layer
PAD = "#c8373a"
GREY = "#9aa0a6"
LIGHT = "#e8eaed"
INK = "#202124"
MUTED = "#5f6368"
ACCENT = "#1a9a6c"
WARN = "#e8710a"
FONT = "font-family=\"DejaVu Sans, Helvetica, Arial, sans-serif\""


class Svg:
    def __init__(self, w, h):
        self.w, self.h = w, h
        self.items = []

    def add(self, s):
        self.items.append(s)

    def text(self, x, y, s, size=13, color=INK, anchor="start", weight="normal", italic=False):
        style = ' font-style="italic"' if italic else ""
        self.add(f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" fill="{color}" '
                 f'text-anchor="{anchor}" font-weight="{weight}"{style}>{s}</text>')

    def line(self, x1, y1, x2, y2, color=INK, w=1.0, dash=None, cap="round", marker=None):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        m = f' marker-end="url(#{marker})"' if marker else ""
        self.add(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{color}" '
                 f'stroke-width="{w}" stroke-linecap="{cap}"{d}{m}/>')

    def arrow(self, x1, y1, x2, y2, color=MUTED, w=1.2, both=False):
        m1 = ' marker-start="url(#arr-s)"' if both else ""
        self.add(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{color}" '
                 f'stroke-width="{w}" marker-end="url(#arr)"{m1}/>')

    def path(self, d, color=TOP, w=6, fill="none", dash=None, opacity=1.0, join="round"):
        ds = f' stroke-dasharray="{dash}"' if dash else ""
        op = f' opacity="{opacity}"' if opacity != 1.0 else ""
        self.add(f'<path d="{d}" stroke="{color}" stroke-width="{w}" fill="{fill}" '
                 f'stroke-linecap="round" stroke-linejoin="{join}"{ds}{op}/>')

    def poly(self, pts, color=TOP, w=6, fill="none", closed=False, dash=None, opacity=1.0):
        d = "M " + " L ".join(f"{x:.1f} {y:.1f}" for x, y in pts) + (" Z" if closed else "")
        self.path(d, color, w, fill, dash, opacity)

    def circle(self, x, y, r, fill=PAD, stroke="none", w=0, dash=None, opacity=1.0):
        ds = f' stroke-dasharray="{dash}"' if dash else ""
        op = f' opacity="{opacity}"' if opacity != 1.0 else ""
        self.add(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{r:.1f}" fill="{fill}" stroke="{stroke}" '
                 f'stroke-width="{w}"{ds}{op}/>')

    def rect(self, x, y, w, h, fill=PAD, stroke="none", sw=0, rx=0, dash=None, opacity=1.0):
        ds = f' stroke-dasharray="{dash}"' if dash else ""
        op = f' opacity="{opacity}"' if opacity != 1.0 else ""
        self.add(f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" rx="{rx}" '
                 f'fill="{fill}" stroke="{stroke}" stroke-width="{sw}"{ds}{op}/>')

    def via(self, x, y, r=7, color=BOT):
        self.circle(x, y, r, fill="#ffffff", stroke=color, w=r * 0.55)

    def title(self, x, y, s, size=14):
        self.text(x, y, s, size=size, weight="bold")

    def save(self, name):
        head = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w}" height="{self.h}" '
                f'viewBox="0 0 {self.w} {self.h}" {FONT}>\n'
                '<defs>'
                f'<marker id="arr" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
                f'<path d="M0 0 L10 5 L0 10 z" fill="{MUTED}"/></marker>'
                f'<marker id="arr-s" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
                f'<path d="M0 0 L10 5 L0 10 z" fill="{MUTED}"/></marker>'
                '</defs>\n'
                f'<rect x="0.5" y="0.5" width="{self.w - 1}" height="{self.h - 1}" rx="10" fill="#ffffff" stroke="#dadce0"/>\n')
        with open(os.path.join(OUT, name), "w") as f:
            f.write(head + "\n".join(self.items) + "\n</svg>\n")
        print("wrote", name)


# ---------------------------------------------------------------- geometry helpers

def taut(start, circles, end):
    """Path from start to end that wraps around circles.

    circles: list of (x, y, r, side), side +1 passes the circle on the left of the travel
    direction, -1 on the right. Returns an SVG path string with lines and arcs, the shortest
    path of that topology (a rubber band).
    """
    nodes = [(start[0], start[1], 0.0, 1)] + list(circles) + [(end[0], end[1], 0.0, 1)]
    tangents = []
    for a, b in zip(nodes, nodes[1:]):
        (x1, y1, r1, s1), (x2, y2, r2, s2) = a, b
        # signed radii: positive = circle lies to the right of the line, i.e. the path keeps it
        # on its left when side is +1 ... measured in SVG coordinates (y down)
        p1, p2 = -s1 * r1, -s2 * r2
        dx, dy = x2 - x1, y2 - y1
        L = math.hypot(dx, dy)
        c = (p1 - p2) / L
        c = max(-1.0, min(1.0, c))
        th = math.atan2(dy, dx)
        best = None
        for sgn in (1, -1):
            ang = th + sgn * math.acos(c)
            nx, ny = math.cos(ang), math.sin(ang)
            t1 = (x1 + p1 * nx, y1 + p1 * ny)
            t2 = (x2 + p2 * nx, y2 + p2 * ny)
            ux, uy = t2[0] - t1[0], t2[1] - t1[1]
            # the normal must point to the left of the travel direction (SVG: left = (uy, -ux))
            if (nx * uy - ny * ux) > 0:
                best = (t1, t2)
        tangents.append(best)
    d = f"M {tangents[0][0][0]:.1f} {tangents[0][0][1]:.1f} L {tangents[0][1][0]:.1f} {tangents[0][1][1]:.1f}"
    for i, (x, y, r, s) in enumerate(circles):
        a = tangents[i][1]
        b = tangents[i + 1][0]
        a0 = math.atan2(a[1] - y, a[0] - x)
        a1 = math.atan2(b[1] - y, b[0] - x)
        sweep = 0 if s > 0 else 1
        delta = (a1 - a0) % (2 * math.pi)
        if sweep == 0:
            delta = 2 * math.pi - delta
        large = 1 if delta > math.pi else 0
        d += f" A {r:.1f} {r:.1f} 0 {large} {sweep} {b[0]:.1f} {b[1]:.1f}"
        d += f" L {tangents[i + 1][1][0]:.1f} {tangents[i + 1][1][1]:.1f}"
    return d


# ---------------------------------------------------------------- figures

def topology_geometry():
    s = Svg(820, 330)
    pads = [(150, 175), (250, 110), (260, 240), (340, 170)]
    pin_a, pin_b = (40, 200), (420, 140)
    for ox, label in ((0, "1  Topology: which side of each obstacle"),
                      (410, "2  Geometry: pulled taut like a rubber band")):
        s.title(20 + ox, 30, label)
    # panel 1: triangulation
    pts = pads + [pin_a, pin_b, (40, 70), (420, 70), (40, 300), (420, 300), (200, 300), (200, 60), (330, 60), (330, 300)]
    edges = [(0, 1), (0, 2), (1, 2), (1, 3), (2, 3), (4, 0), (4, 6), (4, 8), (0, 6), (0, 10), (0, 8), (8, 10), (2, 10),
             (11, 6), (11, 1), (0, 11), (1, 12), (12, 11), (12, 3), (3, 5), (5, 7), (12, 7), (3, 7), (3, 13), (2, 13),
             (13, 9), (5, 9), (3, 9), (10, 13), (2, 8)]
    for a, b in edges:
        (x1, y1), (x2, y2) = pts[a], pts[b]
        if x1 > 420 or x2 > 420:
            continue
        s.line(x1 * 0.9 + 0, y1, x2 * 0.9, y2, color=GREY, w=1, dash="4 3")
    for x, y in pads:
        s.circle(x * 0.9, y, 16)
    for x, y in (pin_a, pin_b):
        s.rect(x * 0.9 - 9, y - 9, 18, 18, fill=PAD)
    # crossings of the planned route (above pad 0, below pad 1 ... )
    cross = [(pin_a[0], pin_a[1]), (95, 150), (150, 140), (200, 175), (255, 175), (300, 140), (pin_b[0], pin_b[1])]
    s.poly([(x * 0.9, y) for x, y in cross], color=TOP, w=2.5, dash="7 5")
    for x, y in cross[1:-1]:
        s.circle(x * 0.9, y, 4.5, fill="#ffffff", stroke=TOP, w=2)
    s.text(20, 318, "Obstacles are triangulated; a route is a list of edge crossings.", 12, MUTED)
    # panel 2: taut path with clearance rings
    ox = 410
    clr = 30
    for x, y in pads:
        s.circle(ox + x * 0.9, y, clr, fill="none", stroke=GREY, w=1, dash="3 3")
        s.circle(ox + x * 0.9, y, 16)
    for x, y in (pin_a, pin_b):
        s.rect(ox + x * 0.9 - 9, y - 9, 18, 18, fill=PAD)
    circles = [(ox + 150 * 0.9, 175, clr, -1), (ox + 250 * 0.9, 110, clr, +1), (ox + 260 * 0.9, 240, clr, -1), (ox + 340 * 0.9, 170, clr, -1)]
    d = taut((ox + pin_a[0] * 0.9, pin_a[1]), [circles[0], circles[3]], (ox + pin_b[0] * 0.9, pin_b[1]))
    s.path(d, color=TOP, w=7)
    s.text(ox + 20, 318, "Straight where free, arcs at the clearance around obstacles.", 12, MUTED)
    s.text(ox + 150 * 0.9 - 30, 222, "clearance", 11, MUTED)
    s.save("topology-geometry.svg")


def search_pipeline():
    s = Svg(860, 250)
    boxes = [
        ("Portfolio", "14 runs, each with", "its own order"),
        ("Generations", "mutate the best", "order, judged by topology"),
        ("Geometry", "pull taut, arcs,", "clearance check"),
        ("Geometry gens.", "search judged by", "the drawn tracks"),
        ("Keep improving", "optional open end", "until you accept"),
        ("Finishing", "widen, 45 degrees,", "length tuning"),
    ]
    w, h, gap = 122, 92, 18
    x0 = 18
    for i, (t, a, b) in enumerate(boxes):
        x = x0 + i * (w + gap)
        fill = "#fbe9e9" if i < 2 else ("#e8f0fb" if i < 4 else "#e6f4ee")
        s.rect(x, 60, w, h, fill=fill, stroke="#c4c7c5", sw=1, rx=8)
        s.text(x + w / 2, 85, t, 13, INK, "middle", "bold")
        s.text(x + w / 2, 108, a, 11, MUTED, "middle")
        s.text(x + w / 2, 124, b, 11, MUTED, "middle")
        if i:
            s.arrow(x - gap + 2, 106, x - 3, 106)
    s.title(18, 32, "From netlist to tracks: the phases you see in the progress window")
    s.text(x0 + w, 180, "topology", 12, TOP, "middle", "bold")
    s.line(x0, 168, x0 + 2 * w + gap, 168, TOP, 2)
    s.text(x0 + 2 * (w + gap) + w + gap / 2, 180, "geometry", 12, BOT, "middle", "bold")
    s.line(x0 + 2 * (w + gap), 168, x0 + 4 * (w + gap) - gap, 168, BOT, 2)
    s.text(x0 + 4 * (w + gap) + w + gap / 2, 180, "after the search", 12, ACCENT, "middle", "bold")
    s.line(x0 + 4 * (w + gap), 168, x0 + 6 * (w + gap) - gap, 168, ACCENT, 2)
    s.text(18, 220, "Best candidate: fewest open connections, then fewest violations, then fewest vias and the shortest length.", 12, MUTED)
    s.save("search-pipeline.svg")


def necking():
    s = Svg(820, 300)
    s.title(20, 30, "Necking at fine pitch pins, and the taper at a tight spot")
    # pin row of a QFN, pitch 26 px
    s.rect(40, 70, 210, 190, fill="none", stroke=GREY, sw=1.2, dash="6 4")
    s.text(48, 88, "courtyard", 11, MUTED)
    for i in range(6):
        s.rect(200, 100 + i * 26, 40, 12, fill=PAD)
    s.rect(70, 110, 110, 130, fill="#d0d3d6")
    s.text(125, 180, "QFN", 12, MUTED, "middle")
    # three tracks leaving the pins: narrow inside the courtyard, class width outside
    for i, ty in enumerate((106, 132, 158)):
        y = ty
        s.poly([(240, y), (250, y)], TOP, 5)
        s.poly([(250, y), (400, y + (i - 1) * 40 + 0)], TOP, 5) if False else None
    ys = [106, 132, 158, 184]
    outs = [70, 130, 190, 250]
    for y, yo in zip(ys, outs):
        s.poly([(236, y), (262, y)], TOP, 5)
        s.poly([(262, y), (300, y + (yo - y) * 0.3), (430, yo)], TOP, 11)
        s.poly([(262, y), (300, y + (yo - y) * 0.3)], TOP, 5)
    s.arrow(300, 272, 262, 194)
    s.text(270, 288, "neck inside the courtyard", 12, MUTED)
    s.text(330, 52, "class width", 12, MUTED)
    s.arrow(370, 56, 400, 70)
    # right panel: taper between two pads
    ox = 470
    s.circle(ox + 150, 110, 30, fill=PAD)
    s.circle(ox + 150, 230, 30, fill=PAD)
    s.circle(ox + 150, 110, 52, fill="none", stroke=GREY, w=1, dash="3 3")
    s.circle(ox + 150, 230, 52, fill="none", stroke=GREY, w=1, dash="3 3")
    s.add(f'<path d="M {ox+10} 162 L {ox+110} 162 L {ox+130} 167 L {ox+170} 167 L {ox+190} 162 L {ox+320} 162 '
          f'L {ox+320} 178 L {ox+190} 178 L {ox+170} 173 L {ox+130} 173 L {ox+110} 178 L {ox+10} 178 Z" fill="{TOP}"/>')
    s.text(ox + 20, 205, "wide track", 12, MUTED)
    s.text(ox + 160, 288, "a few micrometres narrower where the clearance is short", 12, MUTED, "middle")
    s.text(800, 60, "never below the", 11, MUTED, "end")
    s.text(800, 74, "track_width (min ...) rule", 11, MUTED, "end")
    s.save("necking.svg")


def prefer_width():
    s = Svg(820, 240)
    s.title(20, 30, "Wider tracks where there is room: track_width (opt ...) or --prefer-width")
    # obstacles
    s.rect(300, 70, 60, 50, fill=PAD)
    s.rect(300, 168, 60, 50, fill=PAD)
    s.circle(560, 172, 18, fill=PAD, opacity=0.25)
    s.circle(560, 196, 18, fill=PAD)
    s.arrow(560, 172, 560, 190)
    y = 144
    # wide track with tapers around the gap
    pts_top = [(40, y - 12), (270, y - 12), (290, y - 5), (370, y - 5), (390, y - 12), (780, y - 12)]
    pts_bot = [(780, y + 12), (390, y + 12), (370, y + 5), (290, y + 5), (270, y + 12), (40, y + 12)]
    d = "M " + " L ".join(f"{a} {b}" for a, b in pts_top + pts_bot) + " Z"
    s.add(f'<path d="{d}" fill="{TOP}"/>')
    s.line(40, y, 780, y, "#ffffff", 1, dash="5 4")
    s.text(140, 110, "target width", 12, MUTED, "middle")
    s.arrow(140, 116, 140, 130)
    s.text(330, 64 - 8, "class width in the gap", 12, MUTED, "middle")
    s.text(280, 232, "taper at every change of width", 12, MUTED, "middle")
    s.arrow(280, 220, 282, 162)
    s.text(660, 110, "neighbours are pushed aside first", 12, MUTED, "middle")
    s.save("prefer-width.svg")


def angles45():
    s = Svg(820, 280)
    s.title(20, 30, "Any-angle (default)")
    s.title(430, 30, "After --angles 45")
    for ox in (0, 410):
        s.via(ox + 50, 230, 9)
        s.via(ox + 360, 70, 9)
        s.circle(ox + 200, 150, 22, fill=PAD)
        s.circle(ox + 200, 150, 44, fill="none", stroke=GREY, w=1, dash="3 3")
    d = taut((50, 230), [(200, 150, 44, -1)], (360, 70))
    s.path(d, TOP, 7)
    ox = 410
    pts = [(50, 230), (120, 230), (160, 190), (160, 120), (190, 90), (190, 70), (360, 70)]
    pts = [(50, 230), (110, 230), (150, 190), (150, 135), (215, 70), (360, 70)]
    s.poly([(ox + x, y) for x, y in pts], TOP, 7)
    for ox2 in (0, 410):
        s.via(ox2 + 50, 230, 9)
        s.via(ox2 + 360, 70, 9)
    s.text(20, 268, "Tracks hug the obstacle in an arc at the clearance.", 12, MUTED)
    s.text(430, 268, "Legs in eight directions, chamfered corners; vias stay put.", 12, MUTED)
    s.save("angles45.svg")


def meander():
    s = Svg(820, 300)
    s.title(20, 30, "Length tuning: a meander on the straight part of a track")
    y0 = 150
    # base
    s.poly([(30, y0), (180, y0)], TOP, 6)
    d = f"M 180 {y0}"
    x = 180
    amp, step = 46, 30
    up = True
    for i in range(8):
        yy = y0 - amp if up else y0 + amp
        d += f" L {x} {yy - (0 if up else 0)}"
        d = d  # keep straight legs
        x2 = x + step
        sweep = 1 if up else 0
        d += f" A {step/2} {step/2} 0 0 {sweep} {x2} {yy}"
        d += f" L {x2} {y0}"
        x = x2
        up = not up
    d2 = f"M 180 {y0}"
    x = 180
    up = True
    for i in range(8):
        r = step / 2
        if up:
            d2 += f" L {x} {y0 - amp + r} A {r} {r} 0 0 1 {x + step} {y0 - amp + r} L {x + step} {y0}"
        else:
            d2 += f" L {x} {y0 + amp - r} A {r} {r} 0 0 0 {x + step} {y0 + amp - r} L {x + step} {y0}"
        x += step
        up = not up
    s.path(d2, TOP, 6)
    s.poly([(x, y0), (x + 160, y0)], TOP, 6)
    s.line(160, y0, x + 20, y0, MUTED, 1, dash="5 4")
    s.text(x + 30, y0 - 8, "base line", 11, MUTED)
    s.arrow(150, y0, 150, y0 - amp - 0)
    s.text(140, y0 - amp / 2, "height", 11, MUTED, "end")
    s.text(140, y0 - amp / 2 + 14, "(--tune-amplitude)", 11, MUTED, "end")
    s.arrow(180 + step, y0 + amp + 22, 180 + 2 * step, y0 + amp + 22, both=True)
    s.text(180 + 1.5 * step, y0 + amp + 42, "air between legs (--tune-spacing)", 11, MUTED, "start")
    s.text(30, 270, "Bumps go to the side with more room and are lowered evenly until the net gains exactly what it lacks.", 12, MUTED)
    s.text(30, 288, "Written as a KiCad tuning pattern, so it can be edited and tuned again in KiCad.", 12, MUTED)
    s.save("meander.svg")


def dogbone():
    s = Svg(820, 330)
    p = 44
    for ox, title in ((0, "Raster sites only"), (410, "Plus dog-bone sites in ball fields")):
        s.title(20 + ox, 30, title)
        x0, y0 = ox + 115, 80
        for i in range(5):
            for j in range(5):
                s.circle(x0 + i * p, y0 + j * p, 11, fill=PAD)
    # raster sites (left): a regular grid of 0.4 mm that does not line up with the balls
    ox = 0
    x0, y0 = 115, 80
    g = 35
    for i in range(-1, 8):
        for j in range(-1, 8):
            x, y = 95 + i * g, 62 + j * g
            if x < 70 or x > 340 or y < 50 or y > 290:
                continue
            ok = all(math.hypot(x - (x0 + a * p), y - (y0 + b * p)) > 20 for a in range(5) for b in range(5))
            if ok:
                s.circle(x, y, 3.5, fill=GREY)
    s.text(20, 312, "A raster that ignores the balls leaves few sites inside the field.", 12, MUTED)
    # right: dog-bone sites in the middle of four balls and half a pitch outside
    ox = 410
    x0, y0 = ox + 115, 80
    for i in range(-1, 5):
        for j in range(-1, 5):
            x, y = x0 + i * p + p / 2, y0 + j * p + p / 2
            s.circle(x, y, 4, fill=ACCENT)
    for (i, j), (di, dj) in [((0, 0), (-1, -1)), ((1, 1), (1, -1)), ((3, 3), (1, 1)), ((2, 3), (-1, 1)), ((4, 1), (1, 1))]:
        bx, by = x0 + i * p, y0 + j * p
        vx, vy = bx + di * p / 2, by + dj * p / 2
        s.poly([(bx, by), (vx, vy)], TOP, 5)
        s.via(vx, vy, 7)
    s.text(ox + 20, 312, "Sites between every four balls and half a pitch outside.", 12, MUTED)
    s.save("dogbone.svg")


def placement_terms():
    s = Svg(820, 320)
    s.title(20, 30, "--corridors: keep a band free for a bus")
    # two ICs with a bus
    s.rect(40, 90, 80, 150, fill="#d0d3d6")
    s.rect(300, 110, 80, 110, fill="#d0d3d6")
    s.text(80, 170, "U1", 12, MUTED, "middle")
    s.text(340, 170, "U2", 12, MUTED, "middle")
    s.rect(120, 120, 180, 90, fill=ACCENT, opacity=0.13)
    for k in range(6):
        y = 128 + k * 15
        s.rect(118, y - 3, 10, 6, fill=PAD)
        s.rect(292, y - 3, 10, 6, fill=PAD)
        s.line(128, y, 292, y, TOP, 2.5)
    s.rect(196, 150, 26, 14, fill=WARN)
    s.arrow(209, 148, 209, 100)
    s.rect(196, 80, 26, 14, fill=WARN, opacity=0.45)
    s.text(232, 92, "a part in the band pays", 11, MUTED)
    s.text(210, 232, "band of the track group", 11, ACCENT, "middle")
    s.text(20, 290, "Parts that are not on the bus's nets move out of its band.", 12, MUTED)
    ox = 420
    s.title(ox, 30, "--escape-halo: room per side for the tracks")
    for k, (title, halo) in enumerate((("same halo all round", (18, 18, 18, 18)), ("room where tracks leave", (40, 8, 22, 8)))):
        cx = ox + 95 + k * 200
        cy = 165
        w = 70
        l, t, r, b = halo[0], halo[1], halo[2], halo[3]
        s.rect(cx - w / 2 - l, cy - w / 2 - t, w + l + r, w + t + b, fill=ACCENT, opacity=0.13, stroke=ACCENT, sw=1, dash="4 3")
        s.rect(cx - w / 2, cy - w / 2, w, w, fill="#d0d3d6")
        for i in range(5):
            s.line(cx - w / 2 - 6, cy - 25 + i * 14, cx - w / 2 - 6 - (34 if k else 14), cy - 25 + i * 14, TOP, 2.5)
        for i in range(2):
            s.line(cx + w / 2 + 6, cy - 11 + i * 28, cx + w / 2 + 6 + (18 if k else 14), cy - 11 + i * 28, TOP, 2.5)
        for i in range(5):
            s.rect(cx - w / 2 - 6, cy - 28 + i * 14, 8, 6, fill=PAD)
            s.rect(cx + w / 2 - 2, cy - 28 + i * 14, 8, 6, fill=PAD)
        s.text(cx, 255, title, 11, MUTED, "middle")
    s.text(ox, 290, "Sides with many escaping tracks get more room than quiet ones.", 12, MUTED)
    s.save("placement-terms.svg")


def pairs():
    s = Svg(820, 260)
    s.title(20, 30, "--pairs: one wide track, split in two at the gap")
    y = 130
    s.rect(30, y - 30, 16, 12, fill=PAD)
    s.rect(30, y + 18, 16, 12, fill=PAD)
    # wide track
    s.path(f"M 120 {y} L 330 {y} A 80 80 0 0 0 386 {y-23} L 430 {y-67}", "#e4a3a4", 26)
    s.text(230, y - 26, "routed as one wide track", 11, MUTED, "middle")
    # after the split, drawn further right
    ox = 440
    s.path(f"M {ox+20} {y-6} L {ox+230} {y-6} A 74 74 0 0 0 {ox+282} {y-27.7} L {ox+330} {y-76}", TOP, 6)
    s.path(f"M {ox+20} {y+6} L {ox+230} {y+6} A 86 86 0 0 0 {ox+290} {y-19} L {ox+338} {y-67}", TOP, 6)
    s.arrow(400, 190, 450, 190)
    s.text(425, 210, "split", 11, MUTED, "middle")
    s.text(ox + 120, y + 40, "two tracks at the pair gap", 11, MUTED, "middle")
    # breakouts: single tracks into the pads
    s.path(f"M 46 {y-24} L 90 {y-24} L 120 {y-4}", TOP, 5)
    s.path(f"M 46 {y+24} L 90 {y+24} L 120 {y+4}", TOP, 5)
    s.text(70, y + 60, "--breakouts: single", 11, MUTED, "middle")
    s.text(70, y + 74, "tracks into the pads", 11, MUTED, "middle")
    s.text(20, 248, "--pair-pull is the lighter way: two nets routed on their own, the second pulled to the way of the first.", 12, MUTED)
    s.save("pairs.svg")


def power_copper():
    s = Svg(820, 300)
    s.title(20, 30, "power_copper: a pour for the switch node, a via field into the plane")
    # converter pins
    for i in range(5):
        s.rect(60, 90 + i * 22, 30, 12, fill=PAD)
    s.text(40, 210, "U4 pins 5 to 9", 11, MUTED)
    # inductor pads
    s.rect(300, 80, 90, 120, fill=PAD)
    s.rect(560, 80, 90, 120, fill=PAD)
    s.text(345, 220, "L1.1", 11, MUTED, "middle")
    s.text(605, 220, "L1.2", 11, MUTED, "middle")
    # obstacle of another net
    s.circle(200, 210, 14, fill=BOT)
    s.text(220, 250, "other net: the pour is cut back", 11, MUTED)
    # pour: hull grown by margin, cut by a straight line below the obstacle
    s.poly([(50, 80), (300, 66), (402, 70), (402, 210), (300, 212), (230, 186), (150, 182), (50, 190)],
           color=ACCENT, w=1.5, fill="#1a9a6c33", closed=True)
    s.line(140, 186, 250, 188, WARN, 1.5, dash="5 3")
    # via field on rings around L1.2
    for k in range(7):
        a = math.pi * (0.95 + k * 0.18)
        s.via(605 + 70 * math.cos(a) * 1.0, 140 + 78 * math.sin(a) * 1.0, 8)
    s.text(560, 262, "vias for the current, on rings just off the pad", 11, MUTED, "middle")
    s.text(20, 288, "The hull of the pads, grown by the margin and cut back wherever another net comes closer than its clearance.", 12, MUTED)
    s.save("power-copper.svg")


def reference_check():
    s = Svg(820, 300)
    s.title(20, 30, "reference_check: what lies under a fast signal")
    # reference layer zones
    s.rect(40, 60, 360, 200, fill="#dfe8f6", stroke="#9fb6db", sw=1)
    s.rect(400, 60, 380, 200, fill="#e6f1e3", stroke="#a9c9a0", sw=1)
    s.text(60, 80, "In2.Cu zone +5V_A", 11, MUTED)
    s.text(760, 80, "In2.Cu zone +5V_B", 11, MUTED, "end")
    # slot in the plane
    s.rect(190, 120, 18, 120, fill="#ffffff", stroke="#9fb6db", sw=1)
    s.text(199, 255, "slot", 11, MUTED, "middle")
    # track on top
    s.poly([(40, 170), (760, 170)], TOP, 6)
    s.text(600, 162, "signal track on B.Cu", 11, MUTED)
    # markers
    s.circle(199, 170, 16, fill="none", stroke=WARN, w=2.5)
    s.text(199, 112, "gap", 12, WARN, "middle", "bold")
    s.circle(400, 170, 16, fill="none", stroke=WARN, w=2.5)
    s.text(400, 112, "split", 12, WARN, "middle", "bold")
    # return current detour
    s.path("M 150 196 C 175 250, 225 250, 248 196", MUTED, 1.4, dash="4 3")
    s.text(20, 288, "The return current flows under the track; at a gap or a split it detours. The tool reports both with position.", 12, MUTED)
    s.save("reference-check.svg")


def finish_net():
    s = Svg(820, 210)
    s.title(20, 30, "Finish selected nets: continue a started track, shove what is in the way")
    s.rect(40, 120, 20, 20, fill=PAD)
    s.rect(740, 120, 20, 20, fill=PAD)
    s.poly([(60, 130), (230, 130)], TOP, 6)
    s.text(140, 115, "started by hand", 11, MUTED, "middle")
    s.path("M 230 130 L 300 130 C 330 130, 340 145, 370 145 L 640 145 L 740 130", TOP, 6, dash="10 6")
    s.text(560, 170, "added by the router", 11, MUTED, "middle")
    # an unlocked track of another net on the same layer, pushed up out of the way
    s.rect(330, 52, 18, 18, fill=PAD)
    s.rect(470, 52, 18, 18, fill=PAD)
    s.path("M 339 70 L 339 150 Q 339 175 364 175 L 454 175 Q 479 175 479 150 L 479 70", TOP, 5, opacity=0.25)
    s.path("M 339 70 L 339 88 Q 339 108 359 108 L 459 108 Q 479 108 479 88 L 479 70", TOP, 5)
    s.arrow(409, 168, 409, 116)
    s.text(500, 100, "another net gives way", 11, MUTED)
    s.save("finish-net.svg")


if __name__ == "__main__":
    topology_geometry()
    search_pipeline()
    necking()
    prefer_width()
    angles45()
    meander()
    dogbone()
    placement_terms()
    pairs()
    power_copper()
    reference_check()
    finish_net()
