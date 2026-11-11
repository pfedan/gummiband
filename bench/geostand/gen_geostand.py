#!/usr/bin/env python3
"""Geometry test stand: small unrouted KiCad boards, one basic routing situation each.

Every case comes in five forms:
  loose   the channel has plenty of room (LOOSE_SLACK above what the copper needs)
  tight20 the channel is what the copper needs plus TIGHT20_SLACK (more than the router's own margin)
  tight   the channel is what the copper needs plus TIGHT_SLACK: a legal answer exists
  exact   the channel is exactly what the copper needs (EXACT_SLACK, 0): the copper fits to the
          micrometre by the rules, as boards are often designed; KiCad accepts clearance + 0
  short   the channel is SHORT_SLACK too narrow: the right answer is "does not fit"

The room a channel needs is computed from the rules at the top of this file. Obstacles and walls
are pads of nets with a single pin, so both KiCad and the router treat them as copper of another
net. B.Cu is blocked by a rule area unless a case needs it, so the tracks stay on F.Cu. Starts and
goals of tracks sit in line with the lanes of the channel, so that no funnel is part of a case.

For every case and form the generator also writes a reference solution: the copper as it should be
(lines and arcs, the slack shared equally between the gaps of the channel). Its KiCad DRC must be
clean and fully connected for loose, tight20, tight and exact, and show at least one entry for short; a
case whose reference fails that does not measure what it claims.

Usage: gen_geostand.py OUT_DIR
Writes OUT_DIR/<case>.<form>.kicad_pcb (unrouted), <case>.<form>.ref.kicad_pcb (reference),
the .kicad_pro of both, and OUT_DIR/cases.json.
Standard library only.
"""
import json
import math
import os
import sys
import uuid

# ---- rules and parameters (mm) -------------------------------------------------------------
CLEARANCE = 0.2          # net class clearance
TRACK = 0.2              # net class track width
VIA_D, VIA_DRILL = 0.6, 0.3
EDGE_CLEARANCE = 0.3     # min_copper_edge_clearance
HOLE_CLEARANCE = 0.2
MAX_ERROR = 0.005
LOOSE_SLACK = 0.3
TIGHT20_SLACK = 0.020    # 20 um more than needed: above the router's own relaxation margin (8 um)
TIGHT_SLACK = 0.005      # 5 um more than needed
EXACT_SLACK = 0.0        # exactly what is needed
SHORT_SLACK = -0.010     # 10 um less than needed
BLOCK_GAP = 0.3          # gap that blocks a path (less than one track with its clearances)
LANE_PAD = 0.2           # size of start and goal pads that sit in a lane (not wider than the track)
FORMS = {"loose": LOOSE_SLACK, "tight20": TIGHT20_SLACK, "tight": TIGHT_SLACK, "exact": EXACT_SLACK,
         "short": SHORT_SLACK}


def uid():
    return str(uuid.uuid4())


def add(p, d, s=1.0):
    return (p[0] + d[0] * s, p[1] + d[1] * s)


def unit(v):
    n = math.hypot(*v)
    return (v[0] / n, v[1] / n)


class Board:
    """A board under construction: outline, footprints with one pad each, rule areas, and the
    reference copper (tracks, arcs, vias)."""

    def __init__(self, w, h, b_cu_open=False, track=TRACK, min_track=TRACK):
        self.w, self.h = w, h
        self.nets = [""]
        self.footprints = []
        self.cutouts = []
        self.b_cu_open = b_cu_open
        self.track, self.min_track = track, min_track
        self.extra_areas = []
        self.ref = []

    def net(self, name):
        if name not in self.nets:
            self.nets.append(name)
        return self.nets.index(name)

    def _fp(self, x, y, pads, local_clearance=None):
        ref = f"P{len(self.footprints) + 1}"
        cl = f"\n\t\t(clearance {local_clearance})" if local_clearance is not None else ""
        self.footprints.append(f'''\t(footprint "geostand:{ref}" (layer "F.Cu") (uuid "{uid()}") (at {x:.6f} {y:.6f}){cl}
\t\t(property "Reference" "{ref}" (at 0 0 0) (layer "F.SilkS") (hide yes) (uuid "{uid()}") (effects (font (size 0.4 0.4) (thickness 0.08))))
\t\t(property "Value" "geo" (at 0 0 0) (layer "F.Fab") (hide yes) (uuid "{uid()}") (effects (font (size 0.4 0.4) (thickness 0.08))))
\t\t(attr smd)
{pads}
\t)''')
        return ref

    def _pad_line(self, num, shape, dx, dy, size, net, layers='"F.Cu"', rot=0.0, extra=""):
        n = self.net(net)
        return (f'\t\t(pad "{num}" {shape} (at {dx:.6f} {dy:.6f} {rot}) (size {size[0]:.6f} {size[1]:.6f}) '
                f'(layers {layers}) (net {n} "{net}"){extra} (uuid "{uid()}"))')

    # ---- pads
    def rect(self, x, y, w, h, net, rot=0.0, layer="F.Cu", local_clearance=None):
        return self._fp(x, y, self._pad_line("1", "smd rect", 0, 0, (w, h), net, f'"{layer}"', rot), local_clearance)

    def wall(self, x0, y0, x1, y1, name=None):
        """A rectangle of copper of a net of its own (or of `name`), from (x0, y0) to (x1, y1)."""
        name = name or f"WALL{len(self.footprints) + 1}"
        return self.rect((x0 + x1) / 2, (y0 + y1) / 2, abs(x1 - x0), abs(y1 - y0), name)

    def circle(self, x, y, d, net, layer="F.Cu"):
        return self._fp(x, y, self._pad_line("1", "smd circle", 0, 0, (d, d), net, f'"{layer}"'))

    def via_like(self, x, y, net, d=VIA_D, drill=VIA_DRILL):
        """A through hole pad with the size of a via, of a net of its own."""
        line = (f'\t\t(pad "1" thru_hole circle (at 0 0) (size {d} {d}) (drill {drill}) (layers "*.Cu") '
                f'(net {self.net(net)} "{net}") (uuid "{uid()}"))')
        return self._fp(x, y, line)

    def poly(self, pts, net, anchor):
        """A custom pad whose copper is the polygon pts (absolute coordinates); anchor: a point inside it."""
        ax, ay = anchor
        rel = " ".join(f"(xy {x - ax:.6f} {y - ay:.6f})" for x, y in pts)
        line = (f'\t\t(pad "1" smd custom (at 0 0) (size 0.05 0.05) (layers "F.Cu") (net {self.net(net)} "{net}") '
                f'(options (clearance outline) (anchor circle)) (primitives (gr_poly (pts {rel}) (width 0) (fill yes))) '
                f'(uuid "{uid()}"))')
        return self._fp(ax, ay, line)

    def row(self, x0, y, pitch, n, pw, ph, nets, layer="F.Cu"):
        """One footprint: n pads in a row along x, pad size pw x ph, centred at (x0 + i*pitch, y)."""
        cx = x0 + (n - 1) * pitch / 2
        lines = "\n".join(self._pad_line(str(i + 1), "smd rect", x0 + i * pitch - cx, 0, (pw, ph), nets[i], f'"{layer}"')
                          for i in range(n))
        return self._fp(cx, y, lines)

    def terminal(self, p, net, layer="F.Cu", size=0.5):
        """A start or goal of a track: a small square pad at p."""
        return self.rect(p[0], p[1], size, size, net, layer=layer)

    def lane_terminal(self, p, net, layer="F.Cu"):
        """A start or goal that sits in a lane: a round pad no wider than the track."""
        return self.circle(p[0], p[1], LANE_PAD, net, layer=layer)

    def cutout(self, x0, y0, x1, y1):
        self.cutouts.append((x0, y0, x1, y1))

    def keepout(self, layer, x0, y0, x1, y1):
        self.extra_areas.append((layer, x0, y0, x1, y1))

    # ---- reference copper
    def ref_line(self, a, b, net, width=TRACK, layer="F.Cu"):
        if math.dist(a, b) > 1e-9:
            self.ref.append(f'\t(segment (start {a[0]:.6f} {a[1]:.6f}) (end {b[0]:.6f} {b[1]:.6f}) (width {width}) '
                            f'(layer "{layer}") (net {self.net(net)}) (uuid "{uid()}"))')

    def ref_arc(self, c, r, a, b, net, width=TRACK, layer="F.Cu", ccw=None):
        """An arc around c of radius r from the point a to the point b (both on the circle). ccw: True
        for increasing angle (board coordinates), False for decreasing, None for the shorter way."""
        t0 = math.atan2(a[1] - c[1], a[0] - c[0]); t1 = math.atan2(b[1] - c[1], b[0] - c[0])
        d = (t1 - t0 + math.pi) % (2 * math.pi) - math.pi
        if ccw is True and d < 0:
            d += 2 * math.pi
        if ccw is False and d > 0:
            d -= 2 * math.pi
        if abs(d) < 1e-9:
            return
        m = (c[0] + r * math.cos(t0 + d / 2), c[1] + r * math.sin(t0 + d / 2))
        self.ref.append(f'\t(arc (start {a[0]:.6f} {a[1]:.6f}) (mid {m[0]:.6f} {m[1]:.6f}) (end {b[0]:.6f} {b[1]:.6f}) '
                        f'(width {width}) (layer "{layer}") (net {self.net(net)}) (uuid "{uid()}"))')

    def ref_via(self, p, net):
        self.ref.append(f'\t(via (at {p[0]:.6f} {p[1]:.6f}) (size {VIA_D}) (drill {VIA_DRILL}) (layers "F.Cu" "B.Cu") '
                        f'(net {self.net(net)}) (uuid "{uid()}"))')

    def ref_items(self):
        return {"lines": sum(1 for r in self.ref if r.startswith("\t(segment")),
                "arcs": sum(1 for r in self.ref if r.startswith("\t(arc")),
                "vias": sum(1 for r in self.ref if r.startswith("\t(via"))}

    def text(self, with_ref=False):
        def area(layer, x0, y0, x1, y1):
            return (f'\t(zone (net 0) (net_name "") (layers "{layer}") (uuid "{uid()}") (name "block") (hatch edge 0.5) '
                    f'(connect_pads (clearance 0)) (min_thickness 0.25) (filled_areas_thickness no) '
                    f'(keepout (tracks not_allowed) (vias not_allowed) (pads allowed) (copperpour not_allowed) (footprints allowed)) '
                    f'(fill (thermal_gap 0.5) (thermal_bridge_width 0.5)) '
                    f'(polygon (pts (xy {x0} {y0}) (xy {x1} {y0}) (xy {x1} {y1}) (xy {x0} {y1}))))')
        areas = [area(*a) for a in self.extra_areas]
        if not self.b_cu_open:
            areas.append(area("B.Cu", -1, -1, self.w + 1, self.h + 1))
        edges = [f'\t(gr_rect (start 0 0) (end {self.w} {self.h}) (stroke (width 0.05) (type default)) (fill no) (layer "Edge.Cuts") (uuid "{uid()}"))']
        edges += [f'\t(gr_rect (start {a} {b}) (end {c} {d}) (stroke (width 0.05) (type default)) (fill no) (layer "Edge.Cuts") (uuid "{uid()}"))'
                  for a, b, c, d in self.cutouts]
        nets = "\n".join(f'\t(net {i} "{n}")' for i, n in enumerate(self.nets))
        body = self.footprints + edges + areas + (self.ref if with_ref else [])
        return f'''(kicad_pcb (version 20240108) (generator "geostand") (generator_version "8.0")
\t(general (thickness 1.6) (legacy_teardrops no))
\t(paper "A4")
\t(layers (0 "F.Cu" signal) (31 "B.Cu" signal) (36 "B.SilkS" user "B.Silkscreen") (37 "F.SilkS" user "F.Silkscreen") (38 "B.Mask" user) (39 "F.Mask" user) (44 "Edge.Cuts" user) (49 "F.Fab" user))
\t(setup (pad_to_mask_clearance 0))
{nets}
''' + "\n".join(body) + "\n)\n"

    def project(self, name):
        return {
            "board": {"design_settings": {
                "defaults": {},
                "rules": {"max_error": MAX_ERROR, "min_clearance": 0.0, "min_track_width": self.min_track,
                          "min_copper_edge_clearance": EDGE_CLEARANCE, "min_hole_clearance": HOLE_CLEARANCE,
                          "min_via_diameter": VIA_D, "min_through_hole_diameter": VIA_DRILL},
                "rule_severities": {}}},
            "meta": {"filename": name + ".kicad_pro", "version": 1},
            "net_settings": {"classes": [{"name": "Default", "clearance": CLEARANCE, "track_width": self.track,
                                          "via_diameter": VIA_D, "via_drill": VIA_DRILL}], "meta": {"version": 3}},
        }


def need_tracks(n, width=TRACK, c_left=CLEARANCE, c_right=CLEARANCE):
    """Room between two copper edges for n parallel tracks of other nets."""
    return c_left + n * width + (n - 1) * CLEARANCE + c_right


def lanes(edge_a, n, slack, width=TRACK):
    """Centre lines of n tracks in a channel that starts at the copper edge edge_a and runs towards
    larger values; the slack is shared equally between the n + 1 gaps. A negative slack (the short form)
    gives no share: the lanes keep exactly the clearance between them, and the last one comes too near the
    far side of the channel, so that the starts and goals themselves stay legal."""
    share = max(slack, 0.0) / (n + 1)
    return [edge_a + CLEARANCE + share + width / 2 + k * (width + CLEARANCE + share) for k in range(n)]


def tangent_point(p, c, r, upper):
    """The point where a line from p touches the circle (c, r); upper: the one with the smaller y."""
    d = math.dist(p, c)
    phi = math.atan2(p[1] - c[1], p[0] - c[0]); alpha = math.acos(r / d)
    cands = [(c[0] + r * math.cos(phi + s * alpha), c[1] + r * math.sin(phi + s * alpha)) for s in (1, -1)]
    return min(cands, key=lambda q: q[1]) if upper else max(cands, key=lambda q: q[1])


def ref_over(b, s, g, c, r, net):
    """Reference: from s over the top of the circle (c, r) to g (two tangents and the arc between)."""
    t1 = tangent_point(s, c, r, True); t2 = tangent_point(g, c, r, True)
    b.ref_line(s, t1, net); b.ref_arc(c, r, t1, t2, net); b.ref_line(t2, g, net)


# ---- cases ---------------------------------------------------------------------------------
# each builder takes the slack and returns (Board, description, needed room in mm, connections);
# the board carries its reference copper

def around(kind, slack):
    """1: one track from left to right around an obstacle; the only way is the channel above it."""
    b = Board(12, 8)
    ox, oy = 6, 4
    if kind == "via":
        b.via_like(ox, oy, "OBST"); top = oy - VIA_D / 2; bottom = oy + VIA_D / 2; r_obst = VIA_D / 2
    elif kind == "round_pad":
        b.circle(ox, oy, 1.0, "OBST"); top = oy - 0.5; bottom = oy + 0.5; r_obst = 0.5
    else:  # a square pad turned by 45 degrees: a corner points up; the track turns around that corner
        b.rect(ox, oy, 1.0, 1.0, "OBST", rot=45); top = oy - 0.5 * math.sqrt(2); bottom = oy + 0.5 * math.sqrt(2); r_obst = 0.0
    need = need_tracks(1)
    b.wall(3, 0.3, 9, top - need - slack)                 # above: the channel
    b.wall(3, bottom + BLOCK_GAP, 9, b.h - 0.3)            # below: closed
    s, g = (1.5, oy), (10.5, oy)
    b.terminal(s, "SIG"); b.terminal(g, "SIG")
    centre = (ox, oy) if r_obst > 0 else (ox, top)
    ref_over(b, s, g, centre, r_obst + CLEARANCE + TRACK / 2 + slack / 2, "SIG")
    return b, f"one track around a {kind.replace('_', ' ')}", need, 1


def between(a_kind, b_kind, n, slack):
    """2: n tracks from left to right through the gap between two obstacles."""
    b = Board(12, 9)
    cy = 4.5
    need = need_tracks(n)
    def obstacle(kind, y_edge, up):
        # place an obstacle whose near edge lies at y_edge; up: it lies above the gap
        if kind == "pad":
            h = 1.0; y = y_edge - h / 2 if up else y_edge + h / 2
            b.rect(6, y, 1.0, h, f"OBST{len(b.footprints)}"); return y - h / 2 if up else y + h / 2
        r = VIA_D / 2; y = y_edge - r if up else y_edge + r
        b.via_like(6, y, f"OBST{len(b.footprints)}"); return y - r if up else y + r
    top_edge = cy - (need + slack) / 2
    far_a = obstacle(a_kind, top_edge, True)
    far_b = obstacle(b_kind, cy + (need + slack) / 2, False)
    b.wall(5.2, 0.3, 6.8, far_a - BLOCK_GAP)               # close above and below the obstacles
    b.wall(5.2, far_b + BLOCK_GAP, 6.8, b.h - 0.3)
    for i, y in enumerate(lanes(top_edge, n, slack)):
        s, g = (1.5, y), (10.5, y)
        b.lane_terminal(s, f"SIG{i + 1}"); b.lane_terminal(g, f"SIG{i + 1}")
        b.ref_line(s, g, f"SIG{i + 1}")
    return b, f"{n} track(s) between {a_kind} and {b_kind}", need, n


def fanout(pitch, wide, slack, n=6, closed_below=True):
    """3: a row of pads (one part) whose tracks leave upwards and turn hard to the left, under a wall.
    The track of the leftmost pad turns left at once and passes above no other pad; the other n - 1
    pass above pads and stack between the pad tops and the wall: c + (n - 1) tracks + c.
    closed_below: a wall under the row as well, so that every track must take the bundle above it."""
    track = 0.25 if wide else TRACK
    b = Board(14, 9, track=track, min_track=TRACK)
    pw, ph = (0.2 if pitch < 0.45 else 0.3), 0.8   # 0.4 mm pitch: 0.2 wide pads leave exactly the clearance
    y = 6.0
    nets = [f"S{i + 1}" for i in range(n)]
    b.row(6.0, y, pitch, n, pw, ph, nets)
    x_last = 6.0 + (n - 1) * pitch
    need = need_tracks(n - 1, width=TRACK)
    pad_top = y - ph / 2
    wall_bottom = pad_top - need - slack
    b.wall(3.0, 0.3, x_last + 1.5, wall_bottom, "WALL")
    b.wall(x_last + pw / 2 + BLOCK_GAP, 0.3, b.w - 0.3, b.h - 0.3, "WALL")   # closed on the right (one blob with the wall above)
    if closed_below:
        b.wall(3.0, y + ph / 2 + BLOCK_GAP, b.w - 0.3, b.h - 0.3, "WALL")   # and below the row
    # lanes from the wall down; the highest lane belongs to the rightmost pad. Reference: up, then left
    hs = list(reversed(lanes(wall_bottom, n - 1, slack)))      # heights of S2 .. Sn, lowest first
    for i in range(n):
        x = 6.0 + i * pitch
        h = y if i == 0 else hs[i - 1]
        goal = (1.0, h)
        b.lane_terminal(goal, nets[i])
        if i > 0:
            b.ref_line((x, y), (x, h), nets[i])
        b.ref_line((x, h), goal, nets[i])
    desc = (f"row of {n} at {pitch} mm pitch fanning out left{' (class 0.25 > neck 0.2)' if wide else ''}"
            f"{'' if closed_below else ', open below the row'}")
    return b, desc, need, n


def clip_to(poly, x0, y0, x1, y1):
    """Sutherland-Hodgman: poly clipped to the rectangle (x0, y0)-(x1, y1)."""
    def clip(pts, inside, cut):
        out = []
        for i, cur in enumerate(pts):
            prev = pts[i - 1]
            if inside(cur):
                if not inside(prev):
                    out.append(cut(prev, cur))
                out.append(cur)
            elif inside(prev):
                out.append(cut(prev, cur))
        return out
    def at_x(x):
        return lambda p, q: (x, p[1] + (q[1] - p[1]) * (x - p[0]) / (q[0] - p[0]))
    def at_y(y):
        return lambda p, q: (p[0] + (q[0] - p[0]) * (y - p[1]) / (q[1] - p[1]), y)
    pts = list(poly)
    pts = clip(pts, lambda p: p[0] >= x0, at_x(x0))
    pts = clip(pts, lambda p: p[0] <= x1, at_x(x1))
    pts = clip(pts, lambda p: p[1] >= y0, at_y(y0))
    pts = clip(pts, lambda p: p[1] <= y1, at_y(y1))
    return pts


def bundle(n, corner, slack, angle=90):
    """4/5: n tracks turning around a corner between an inner and an outer obstacle (concentric arcs).
    The inner obstacle (the corner with its two legs) and the outer one (an arc band, its legs, and
    walls from them to the board edge) leave exactly one way from the start region to the goal region.
    Starts and goals sit in the lanes, just beyond the outer legs."""
    b = Board(14, 14)
    cx, cy = 7.0, 7.0
    c = (cx, cy)
    r_in = {"via": VIA_D / 2, "round": 0.5, "sharp": 0.0}[corner]
    need = need_tracks(n)
    r_out = r_in + need + slack
    a0 = math.pi                          # from the centre to the incoming side (left of the corner)
    a1 = math.pi + math.radians(angle)    # to the outgoing side; y down: the turn goes over the top
    n0 = (math.cos(a0), math.sin(a0)); n1 = (math.cos(a1), math.sin(a1))          # outer normals of the legs
    d0 = (math.cos(a0 - math.pi / 2), math.sin(a0 - math.pi / 2))                 # incoming leg, away from the corner
    d1 = (math.cos(a1 + math.pi / 2), math.sin(a1 + math.pi / 2))                 # outgoing leg, away from the corner
    def arc_pts(r, t0, t1):
        k = max(8, int(abs(t1 - t0) * max(r, 0.01) / 0.01))
        return [(cx + r * math.cos(t0 + (t1 - t0) * i / k), cy + r * math.sin(t0 + (t1 - t0) * i / k)) for i in range(k + 1)]
    inset = (0.4, 0.4, b.w - 0.4, b.h - 0.4)
    far = 30.0
    # inner obstacle: the corner, its legs to the board edge, the wedge between them
    p_in0, p_in1 = add(c, n0, r_in), add(c, n1, r_in)
    inner = arc_pts(r_in, a0, a1) if r_in > 0 else [c]
    inner_poly = clip_to([add(p_in0, d0, far)] + inner + [add(p_in1, d1, far), add(add(c, d0, far), d1, far)], *inset)
    b.poly(inner_poly, "INNER", add(add(c, d0, 0.6), d1, 0.6))
    # outer obstacle: band beyond r_out along the turn, legs of length Lo, and from them to the board edge
    t, Lo = 0.5, 3.0
    q0, q1 = add(c, n0, r_out), add(c, n1, r_out)
    band = arc_pts(r_out, a0, a1) + arc_pts(r_out + t, a1, a0)
    b.poly(band, "OUTER", add(c, (math.cos((a0 + a1) / 2), math.sin((a0 + a1) / 2)), r_out + t / 2))
    for q, d, nrm in ((q0, d0, n0), (q1, d1, n1)):
        back = r_out + t
        side = clip_to([add(q, d, -back), add(q, d, Lo), add(add(q, d, Lo), nrm, far), add(add(q, d, -back), nrm, far)], *inset)
        b.poly(side, "OUTER", add(add(q, d, Lo / 2), nrm, 0.3))
    # lanes: radius of each track around the corner; starts and goals in the lanes, just beyond the ends
    # of the outer legs (still in line with their lanes, so no funnel lies in front of the channel)
    for i, rad in enumerate(lanes(r_in, n, slack)):
        net = f"B{i + 1}"
        s = add(add(c, n0, rad), d0, Lo + 0.4)
        g = add(add(c, n1, rad), d1, Lo + 0.4)
        b.lane_terminal(s, net); b.lane_terminal(g, net)
        e0, e1 = add(c, n0, rad), add(c, n1, rad)
        b.ref_line(s, e0, net); b.ref_arc(c, rad, e0, e1, net, ccw=True); b.ref_line(e1, g, net)
    return b, f"{n} track(s) turning {angle} deg around a {corner} corner", need, n


def tangent(slack):
    """6: a track wraps one via-sized pad from above and the next from below, in a corridor along the
    inner tangent of the two (bars of the vias' own nets on both sides). Each bar runs on beyond the
    tangent point at the other via, so that it also closes the way over (under) that via: the gap
    between a via and the bar of the other net is the needed room plus the slack, as in the corridor."""
    b = Board(14, 9)
    v1, v2 = (5.0, 4.9), (9.0, 4.1)
    b.via_like(*v1, "V1"); b.via_like(*v2, "V2")
    rho = VIA_D / 2 + CLEARANCE + TRACK / 2 + slack / 2       # radius of the track centre around each
    dx, dy = v2[0] - v1[0], v2[1] - v1[1]; d = math.hypot(dx, dy)
    ang = math.atan2(dy, dx); beta = math.asin(2 * rho / d)
    nrm = (math.cos(ang - math.pi / 2 + beta), math.sin(ang - math.pi / 2 + beta))   # normal towards the track from v1
    t1 = (v1[0] + rho * nrm[0], v1[1] + rho * nrm[1]); t2 = (v2[0] - rho * nrm[0], v2[1] - rho * nrm[1])
    mid = ((t1[0] + t2[0]) / 2, (t1[1] + t2[1]) / 2)
    u = unit((t2[0] - t1[0], t2[1] - t1[1])); side = (-u[1], u[0])
    off = TRACK / 2 + CLEARANCE + slack / 2
    L = math.dist(t1, t2)
    ext = 1.0                                                 # how far a bar runs on past the other via
    for net, v, a0, a1 in (("V1", v1, 0.0, L + ext), ("V2", v2, -ext, L)):
        sgn = 1 if (side[0] * (v[0] - mid[0]) + side[1] * (v[1] - mid[1])) > 0 else -1
        def p(a, o):
            return (t1[0] + u[0] * a + side[0] * sgn * o, t1[1] + u[1] * a + side[1] * sgn * o)
        b.poly([p(a0, off), p(a1, off), p(a1, off + 0.4), p(a0, off + 0.4)], net, p((a0 + a1) / 2, off + 0.2))
    b.wall(4.4, v1[1] + VIA_D / 2 + BLOCK_GAP, 5.6, b.h - 0.3)    # v1 closed below
    b.wall(8.4, 0.3, 9.6, v2[1] - VIA_D / 2 - BLOCK_GAP)          # v2 closed above
    s, g = (1.5, 4.5), (12.5, 4.5)
    b.terminal(s, "SIG"); b.terminal(g, "SIG")
    a = tangent_point(s, v1, rho, True); z = tangent_point(g, v2, rho, False)
    b.ref_line(s, a, "SIG"); b.ref_arc(v1, rho, a, t1, "SIG"); b.ref_line(t1, t2, "SIG")
    b.ref_arc(v2, rho, t2, z, "SIG"); b.ref_line(z, g, "SIG")
    return b, "tangent from via to via, in a corridor along the tangent", 2 * CLEARANCE + TRACK, 1


def edge(kind, slack):
    """7: a track around an inner cutout of the outline (edge clearance); or with its goal on B.Cu,
    so that a via must find room next to the cutout."""
    b = Board(14, 9, b_cu_open=(kind == "via"))
    x0, y0, x1, y1 = 5.0, 3.5, 9.0, 8.7
    b.cutout(x0, y0, x1, y1)
    s, g = (1.5, 6.0), (12.5, 6.0)
    if kind == "track":
        need = EDGE_CLEARANCE + TRACK + CLEARANCE
        b.wall(4.0, 0.3, 10.0, y0 - need - slack)
        b.terminal(s, "SIG"); b.terminal(g, "SIG")
        rho = EDGE_CLEARANCE + TRACK / 2 + slack / 2          # the track centre around each top corner
        a = tangent_point(s, (x0, y0), rho, True); z = tangent_point(g, (x1, y0), rho, True)
        b.ref_line(s, a, "SIG"); b.ref_arc((x0, y0), rho, a, (x0, y0 - rho), "SIG")
        b.ref_line((x0, y0 - rho), (x1, y0 - rho), "SIG"); b.ref_arc((x1, y0), rho, (x1, y0 - rho), z, "SIG")
        b.ref_line(z, g, "SIG")
        return b, "track around an inner cutout", need, 1
    need = EDGE_CLEARANCE + VIA_D + CLEARANCE
    wall_bottom = y0 - need - slack
    b.wall(4.0, 0.3, 10.0, wall_bottom, "WALL")
    b.keepout("B.Cu", 0, 0, x0, b.h)                                    # no via left of the cutout
    b.wall(x1 + BLOCK_GAP, 0.3, b.w - 0.3, b.h - 0.3, "WALL")           # F.Cu ends with the cutout: the via must sit above it
    b.terminal(s, "SIG"); b.terminal(g, "SIG", layer="B.Cu")
    yv = y0 - EDGE_CLEARANCE - VIA_D / 2 - slack / 2                    # via centre: the slack shared by edge and wall
    via = (7.0, yv)
    b.ref_line(s, (x0 - 0.5, yv), "SIG"); b.ref_line((x0 - 0.5, yv), via, "SIG"); b.ref_via(via, "SIG")
    b.ref_line(via, (x1 + 0.6, yv), "SIG", layer="B.Cu"); b.ref_line((x1 + 0.6, yv), g, "SIG", layer="B.Cu")
    return b, "via needed next to an inner cutout (goal on B.Cu)", need, 1


def via_between_pads(slack):
    """8: one track must change layer between two pads of other nets that have copper on both layers
    (through hole); the via has to sit in the gap between them, the slack shared on both sides."""
    b = Board(12, 9, b_cu_open=True)
    cy, px, pw = 4.5, 6.0, 1.0
    need = CLEARANCE + VIA_D + CLEARANCE
    gap = need + slack
    for y, name in ((cy - gap / 2 - pw / 2, "PADA"), (cy + gap / 2 + pw / 2, "PADB")):
        line = (f'\t\t(pad "1" thru_hole rect (at 0 0) (size {pw} {pw}) (drill 0.4) (layers "*.Cu") '
                f'(net {b.net(name)} "{name}") (uuid "{uid()}"))')
        b._fp(px, y, line)
    x0, x1 = px - pw / 2, px + pw / 2
    # F.Cu: closed above and below the pads, and right of them (the track can go no further on F.Cu)
    b.wall(x0 - 0.3, 0.3, x1 + 0.3, cy - gap / 2 - pw - BLOCK_GAP, "WALL")
    b.wall(x0 - 0.3, cy + gap / 2 + pw + BLOCK_GAP, x1 + 0.3, b.h - 0.3, "WALL")
    b.wall(x1 + BLOCK_GAP, 0.3, b.w - 0.3, b.h - 0.3, "WALL")
    # B.Cu: no via left of the pads
    b.keepout("B.Cu", 0, 0, x0, b.h)
    s, g = (1.5, cy), (10.5, cy)
    b.terminal(s, "SIG"); b.terminal(g, "SIG", layer="B.Cu")
    via = (px, cy)
    b.ref_line(s, via, "SIG"); b.ref_via(via, "SIG"); b.ref_line(via, g, "SIG", layer="B.Cu")
    return b, "via between two through hole pads (layer change)", need, 1


def via_between_rows(slack):
    """8: a pad of a fine pitch row must change layer; the only room for the via is between two rows."""
    b = Board(12, 10, b_cu_open=True)
    pitch, pw, ph, n = 0.5, 0.3, 0.8, 7
    need = CLEARANCE + VIA_D + CLEARANCE
    gap = need + slack
    y_top = 5.0 - gap / 2 - ph / 2; y_bot = 5.0 + gap / 2 + ph / 2
    top = [f"T{i}" for i in range(n)]; top[n // 2] = "SIG"
    b.row(4.5, y_top, pitch, n, pw, ph, top)
    b.row(4.5, y_bot, pitch, n, pw, ph, [f"U{i}" for i in range(n)])
    # B.Cu only between the rows and on the way to the goal; F.Cu closed around the rows
    b.keepout("B.Cu", 0, 0, b.w, 5.0 - gap / 2); b.keepout("B.Cu", 0, 5.0 + gap / 2, 9.5, b.h)
    x_end = 4.5 + (n - 1) * pitch
    # one blob of wall copper (one net, the pieces overlap) closes F.Cu around both rows
    b.wall(1.0, 0.3, x_end + 0.8, y_top - ph / 2 - BLOCK_GAP, "WALL")
    b.wall(1.0, 0.3, 4.5 - pw / 2 - BLOCK_GAP, b.h - 0.3, "WALL")
    b.wall(x_end + pw / 2 + BLOCK_GAP, 0.3, x_end + 0.8, b.h - 0.3, "WALL")
    b.wall(1.0, y_bot + ph / 2 + BLOCK_GAP, x_end + 0.8, b.h - 0.3, "WALL")
    goal = (11.0, 8.5)
    b.terminal(goal, "SIG", layer="B.Cu")
    xs = 4.5 + (n // 2) * pitch
    via = (xs, 5.0)
    b.ref_line((xs, y_top), via, "SIG"); b.ref_via(via, "SIG")
    b.ref_line(via, (9.9, 5.0), "SIG", layer="B.Cu"); b.ref_line((9.9, 5.0), goal, "SIG", layer="B.Cu")
    return b, "via between two 0.5 mm pad rows (layer change)", need, 1


def local_clearance(slack):
    """9: a channel next to a pad whose footprint sets a clearance below the class clearance (0.15).
    KiCad takes the local clearance; the room is computed with it."""
    b = Board(12, 8)
    local = 0.15
    need = local + TRACK + CLEARANCE
    b.rect(6, 4, 2.0, 1.0, "LOCAL", local_clearance=local)
    b.wall(3, 0.3, 9, 3.5 - need - slack)
    b.wall(3, 4.5 + BLOCK_GAP, 9, b.h - 0.3)
    s, g = (1.5, 4.0), (10.5, 4.0)
    b.terminal(s, "SIG"); b.terminal(g, "SIG")
    yc = 3.5 - local - TRACK / 2 - slack / 2
    b.ref_line(s, (4.4, yc), "SIG"); b.ref_line((4.4, yc), (7.6, yc), "SIG"); b.ref_line((7.6, yc), g, "SIG")
    return b, "channel next to a pad with footprint clearance 0.15 (class 0.2)", need, 1


CASES = {
    "c1_around_via": lambda s: around("via", s),
    "c1_around_round_pad": lambda s: around("round_pad", s),
    "c1_around_rect_corner": lambda s: around("rect_corner", s),
    "c2_1_between_pads": lambda s: between("pad", "pad", 1, s),
    "c2_2_between_pads": lambda s: between("pad", "pad", 2, s),
    "c2_3_between_pads": lambda s: between("pad", "pad", 3, s),
    "c2_1_pad_via": lambda s: between("pad", "via", 1, s),
    "c2_1_via_via": lambda s: between("via", "via", 1, s),
    "c3_row_0p4": lambda s: fanout(0.4, False, s),
    "c3_row_0p5": lambda s: fanout(0.5, False, s),
    "c3_row_0p4_wide": lambda s: fanout(0.4, True, s),
    "c3_row_0p5_wide": lambda s: fanout(0.5, True, s),
    "c3_row_0p4_open": lambda s: fanout(0.4, False, s, closed_below=False),
    "c3_row_0p5_open": lambda s: fanout(0.5, False, s, closed_below=False),
    "c4_bundle3_round": lambda s: bundle(3, "round", s),
    "c4_bundle6_round": lambda s: bundle(6, "round", s),
    "c4_bundle3_via": lambda s: bundle(3, "via", s),
    "c4_bundle6_sharp": lambda s: bundle(6, "sharp", s),
    "c5_turn90_neighbour": lambda s: bundle(2, "sharp", s, 90),
    "c5_turn45_neighbour": lambda s: bundle(2, "sharp", s, 45),
    "c6_tangent": tangent,
    "c7_edge_track": lambda s: edge("track", s),
    "c7_edge_via": lambda s: edge("via", s),
    "c8_via_between_pads": via_between_pads,
    "c8_via_between_rows": via_between_rows,
    "c9_local_clearance": local_clearance,
}


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "boards"
    os.makedirs(out, exist_ok=True)
    meta = []
    for name, build in CASES.items():
        for form, slack in FORMS.items():
            b, desc, need, conns = build(slack)
            base = f"{name}.{form}"
            open(os.path.join(out, base + ".kicad_pcb"), "w").write(b.text())
            json.dump(b.project(base), open(os.path.join(out, base + ".kicad_pro"), "w"), indent=1)
            open(os.path.join(out, base + ".ref.kicad_pcb"), "w").write(b.text(with_ref=True))
            json.dump(b.project(base + ".ref"), open(os.path.join(out, base + ".ref.kicad_pro"), "w"), indent=1)
            meta.append(dict(board=base, case=name, form=form, slack_mm=slack, need_mm=round(need, 6),
                             available_mm=round(need + slack, 6), connections=conns, description=desc,
                             reference_items=b.ref_items()))
    json.dump(meta, open(os.path.join(out, "cases.json"), "w"), indent=1)
    print(f"{len(meta)} boards in {out}")


if __name__ == "__main__":
    main()
