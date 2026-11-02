#!/usr/bin/env python3
"""Generate the synthetic Gummiband benchmark boards (KiCad 10 format).

Writes <name>.kicad_pcb and <name>.kicad_pro per board into bench/boards/ (or the directory
given as first argument). Plain Python 3, no third party packages. Deterministic: fixed seeds
and UUIDs derived from the board name, so a rerun produces identical files.

Boards, roughly by difficulty:
  easy                2 layers, a handful of 2 pin connections between 0603 parts
  medium              2 layers, SOIC/SOT-23/0603 with a random local netlist and a GND net
  bus                 2 layers, 16 bit bus between two 0.5 mm connectors with a 90 degree turn
  power_mix           2 layers, 1 mm and 0.75 mm nets plus signals through a channel between THT walls
  crossing            2 layers, two 1 mm SMD connectors with reversed pin order (needs vias)
  qfn_fanout          4 layers (In1 GND, In2 3V3 planes), QFN-48 to a ring of pull resistors
  four_signal_layers  4 signal layers, dense random netlist (for >2 layer support)
  bga_escape          4 layers (planes), 8x8 BGA at 0.8 mm, 2 signal rings escape to four edge connectors
  bga_dense           4 layers (planes), 10x10 BGA at 0.8 mm, 3 signal rings (84 signals), needs vias
  pair_wall           2 layers, USB pair through the one gap of a THT wall, short nets across it
  pair_traffic        2 layers, two long pairs across a field of random local nets
  parts_easy          2 layers, MCU and SPI flash with a planar bus, local nets
  parts_medium        2 layers, MCU with SPI flash (real pinout), I2C with pull-ups, 6 bit bus with a swap
  parts_hard          2 layers, QFN-48 with shared SPI and I2C, 8 bit bus with swaps, USB pair, local nets
  parts_multi         4 layers (planes), MCU and PHY with a 12 net bus, peripherals, shared I2C, 3 pairs, debug header
  parts_multi_feed    the same with 40 scattered 0603 parts (chicken feed), half of them with a plane via
  parts_xhard         parts_multi_feed on a board 25 % smaller in each direction
"""
import json
import math
import os
import random
import sys
import uuid

NS = uuid.UUID("0b6f5c3e-3c1d-4a8e-9f2a-6d5e8b7c1a90")
ORIGIN = (100.0, 100.0)


def fmt(v):
    s = f"{v:.6f}".rstrip("0").rstrip(".")
    return "0" if s in ("", "-0") else s


def rot(p, deg):
    """KiCad rotation (y down, positive angle counter-clockwise on screen)."""
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return (p[0] * c + p[1] * s, -p[0] * s + p[1] * c)


# ---------------------------------------------------------------- footprints


class Pad:
    def __init__(self, num, kind, shape, at, size, layers, net=None, drill=None, rratio=None):
        self.num, self.kind, self.shape, self.at, self.size = num, kind, shape, at, size
        self.layers, self.net, self.drill, self.rratio = layers, net, drill, rratio


SMD = ["F.Cu", "F.Mask", "F.Paste"]
THT = ["*.Cu", "*.Mask"]


class Footprint:
    def __init__(self, lib, ref, value, pads, crtyd, tht=False):
        self.lib, self.ref, self.value, self.pads, self.crtyd, self.tht = lib, ref, value, pads, crtyd, tht
        self.x = self.y = self.rot = 0.0

    def place(self, x, y, r=0.0):
        self.x, self.y, self.rot = x, y, r
        return self

    def pad(self, num):
        return next(p for p in self.pads if p.num == str(num))

    def world(self, pad):
        d = rot(pad.at, self.rot)
        return (self.x + d[0], self.y + d[1])

    def bbox(self):
        x0, y0, x1, y1 = self.crtyd
        pts = [rot(p, self.rot) for p in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]
        xs = [self.x + p[0] for p in pts]
        ys = [self.y + p[1] for p in pts]
        return (min(xs), min(ys), max(xs), max(ys))


def two_terminal(prefix, ref, value="10k"):
    """0603 resistor or capacitor (KiCad R_0603_1608Metric dimensions)."""
    pads = [Pad(str(i + 1), "smd", "roundrect", (sx * 0.825, 0), (0.8, 0.95), SMD, rratio=0.25)
            for i, sx in enumerate((-1, 1))]
    return Footprint(f"Bench:{prefix}_0603", ref, value, pads, (-1.48, -0.73, 1.48, 0.73))


def soic(ref, n, value="IC"):
    """SOIC-n, 1.27 mm pitch. Pin 1 top left, counter-clockwise."""
    half = n // 2
    pads = []
    for i in range(half):
        y = (i - (half - 1) / 2) * 1.27
        pads.append(Pad(str(i + 1), "smd", "roundrect", (-2.475, y), (1.95, 0.6), SMD, rratio=0.25))
    for i in range(half):
        y = ((half - 1) / 2 - i) * 1.27
        pads.append(Pad(str(half + i + 1), "smd", "roundrect", (2.475, y), (1.95, 0.6), SMD, rratio=0.25))
    h = half * 1.27 / 2 + 0.6
    return Footprint(f"Bench:SOIC-{n}", ref, value, pads, (-3.7, -h, 3.7, h))


def sot23(ref, value="Q"):
    pads = [
        Pad("1", "smd", "roundrect", (-0.9375, -0.95), (1.475, 0.6), SMD, rratio=0.25),
        Pad("2", "smd", "roundrect", (-0.9375, 0.95), (1.475, 0.6), SMD, rratio=0.25),
        Pad("3", "smd", "roundrect", (0.9375, 0), (1.475, 0.6), SMD, rratio=0.25),
    ]
    return Footprint("Bench:SOT-23", ref, value, pads, (-1.92, -1.7, 1.92, 1.7))


def qfn(ref, per_side, pitch, center, length, width, ep, value="MCU"):
    """QFN with per_side pins per side; pin 1 at the top of the left side, counter-clockwise.
    The exposed pad is the last pin number."""
    pads = []
    c = (per_side - 1) / 2 * pitch
    n = 0
    for side in range(4):
        for i in range(per_side):
            t = -c + i * pitch
            n += 1
            if side == 0:  # left, top to bottom
                at, size = (-center, t), (length, width)
            elif side == 1:  # bottom, left to right
                at, size = (t, center), (width, length)
            elif side == 2:  # right, bottom to top
                at, size = (center, -t), (length, width)
            else:  # top, right to left
                at, size = (-t, -center), (width, length)
            pads.append(Pad(str(n), "smd", "roundrect", at, size, SMD, rratio=0.25))
    pads.append(Pad(str(n + 1), "smd", "rect", (0, 0), (ep, ep), SMD))
    e = center + length / 2 + 0.25
    return Footprint(f"Bench:QFN-{4 * per_side}", ref, value, pads, (-e, -e, e, e))


def bga(ref, n, pitch, dia, value="FPGA"):
    rows = "ABCDEFGHJKLMNPRTUVWY"
    pads = []
    c = (n - 1) / 2 * pitch
    for r in range(n):
        for k in range(n):
            pads.append(Pad(f"{rows[r]}{k + 1}", "smd", "circle", (-c + k * pitch, -c + r * pitch), (dia, dia), SMD))
    e = c + dia / 2 + 0.6
    return Footprint(f"Bench:BGA-{n * n}", ref, value, pads, (-e, -e, e, e))


def fpc(ref, n, pitch, pad=(0.3, 1.2), value="FPC"):
    """SMD connector: n signal pads along local x (pad 1 at -x), two mounting pads without net behind
    them (-y). Tracks leave towards +y: rotation 0 faces down, 90 right, 180 up, 270 left."""
    pads = []
    c = (n - 1) / 2 * pitch
    for i in range(n):
        pads.append(Pad(str(i + 1), "smd", "rect", (-c + i * pitch, 0), pad, SMD))
    mx = c + pitch / 2 + 1.0
    for i, sx in enumerate((-1, 1)):
        pads.append(Pad(f"MP{i + 1}", "smd", "rect", (sx * mx, -1.6), (1.0, 1.8), SMD))
    e = mx + 0.75
    return Footprint(f"Bench:FPC-{n}P-{pitch}mm", ref, value, pads, (-e, -2.75, e, pad[1] / 2 + 0.5))


def header(ref, n, rows=1, value="Conn"):
    """THT pin header, 2.54 mm, pins along local x."""
    pads = []
    k = 0
    for i in range(n):
        for r in range(rows):
            k += 1
            shape = "rect" if k == 1 else "oval"
            pads.append(Pad(str(k), "thru_hole", shape, (i * 2.54, r * 2.54), (1.7, 1.7), THT, drill=1.0))
    return Footprint(f"Bench:PinHeader_{rows}x{n:02d}", ref, value, pads,
                     (-1.77, -1.77, (n - 1) * 2.54 + 1.77, (rows - 1) * 2.54 + 1.77), tht=True)


def wall(ref, offsets, value="NC"):
    """A row of THT pins without nets at the given offsets along local x (mm), as one part."""
    pads = [Pad(str(i + 1), "thru_hole", "circle", (x, 0), (1.7, 1.7), THT, drill=1.0) for i, x in enumerate(offsets)]
    return Footprint(f"Bench:Wall_{len(offsets)}", ref, value, pads,
                     (min(offsets) - 1.0, -1.0, max(offsets) + 1.0, 1.0), tht=True)


def radial_cap(ref, value="470u"):
    pads = [
        Pad("1", "thru_hole", "rect", (-2.5, 0), (2.0, 2.0), THT, drill=1.0),
        Pad("2", "thru_hole", "circle", (2.5, 0), (2.0, 2.0), THT, drill=1.0),
    ]
    return Footprint("Bench:CP_Radial_D8.0mm_P5.00mm", ref, value, pads, (-4.25, -4.25, 4.25, 4.25), tht=True)


def power_terminal(ref, value="Load"):
    """Two big SMD pads (e.g. a solder tab connector), along local y."""
    pads = [
        Pad("1", "smd", "rect", (0, -2.0), (2.5, 3.0), SMD),
        Pad("2", "smd", "rect", (0, 2.0), (2.5, 3.0), SMD),
    ]
    return Footprint("Bench:PowerTab_2P", ref, value, pads, (-1.75, -4.0, 1.75, 4.0))


# ---------------------------------------------------------------- board


SEVERITIES = {
    "annular_width": "error", "clearance": "error", "connection_width": "warning",
    "copper_edge_clearance": "error", "copper_sliver": "warning", "courtyards_overlap": "error",
    "creepage": "error", "diff_pair_gap_out_of_range": "error",
    "diff_pair_uncoupled_length_too_long": "error", "drill_out_of_range": "error",
    "duplicate_footprints": "warning", "extra_footprint": "warning", "footprint": "error",
    "footprint_filters_mismatch": "ignore", "footprint_symbol_field_mismatch": "warning",
    "footprint_symbol_mismatch": "warning", "footprint_type_mismatch": "ignore",
    "hole_clearance": "error", "hole_to_hole": "warning", "holes_co_located": "warning",
    "invalid_outline": "error", "isolated_copper": "warning", "item_on_disabled_layer": "error",
    "items_not_allowed": "error", "length_out_of_range": "error", "lib_footprint_issues": "ignore",
    "lib_footprint_mismatch": "ignore", "malformed_courtyard": "error",
    "microvia_drill_out_of_range": "error", "mirrored_text_on_front_layer": "warning",
    "missing_courtyard": "ignore", "missing_footprint": "warning",
    "missing_tuning_profile": "warning", "net_conflict": "warning",
    "nonmirrored_text_on_back_layer": "warning", "npth_inside_courtyard": "error",
    "padstack": "warning", "pth_inside_courtyard": "error", "shorting_items": "error",
    "silk_edge_clearance": "warning", "silk_over_copper": "warning", "silk_overlap": "warning",
    "skew_out_of_range": "error", "solder_mask_bridge": "error", "starved_thermal": "error",
    "text_height": "warning", "text_on_edge_cuts": "error", "text_thickness": "warning",
    "through_hole_pad_without_hole": "error", "too_many_vias": "error", "track_angle": "error",
    "track_dangling": "warning", "track_not_centered_on_via": "ignore",
    "track_on_post_machined_layer": "error", "track_segment_length": "error",
    "track_width": "error", "tracks_crossing": "error", "unconnected_items": "error",
    "unresolved_variable": "error", "via_dangling": "warning", "zones_intersect": "error",
}


class Board:
    def __init__(self, name, w, h, inner=None, title=""):
        """inner: None (2 layers), "power" (In1/In2 planes), "signal" (4 routing layers) or a list
        of the inner layer types from In1 down ("power" or "signal")."""
        self.name, self.w, self.h, self.inner, self.title = name, w, h, inner, title
        self.fps = []
        self.zones = []  # (net, layer)
        self.classes = [("Default", 0.2, 0.15, 0.6, 0.3)]  # name, width, clearance, via dia, drill
        self.patterns = []  # (pattern, class)
        self.dru = None  # custom rules (.kicad_dru), if any
        self.rules = dict(min_clearance=0.15, min_track_width=0.1, min_hole_clearance=0.2,
                          min_hole_to_hole=0.25, min_via_diameter=0.45, min_via_annular_width=0.1)
        self._uid = 0
        self.scale = 1.0  # positions given to add() are scaled by this (a tighter variant of a board)

    def uid(self):
        self._uid += 1
        return str(uuid.uuid5(NS, f"{self.name}/{self._uid}"))

    def add(self, fp, x, y, r=0.0):
        """Place a footprint; x, y relative to the board origin (top left corner). With a `scale`
        other than 1 (on a board `scale` times the size `w / scale`), a position within 6 mm of an
        edge keeps its distance to that edge, so edge connectors stay at the edge; the others
        scale."""
        if self.scale != 1.0:
            def sc(v, size):
                full = size / self.scale
                if v <= 6.0:
                    return v
                if v >= full - 6.0:
                    return size - (full - v)
                return v * self.scale
            x, y = sc(x, self.w), sc(y, self.h)
        self.fps.append(fp.place(ORIGIN[0] + x, ORIGIN[1] + y, r))
        return fp

    def net_class(self, name, width, clearance, via_dia, via_drill, patterns):
        self.classes.append((name, width, clearance, via_dia, via_drill))
        self.patterns += [(p, name) for p in patterns]

    def spread(self, gap=0.3, step=0.2, max_steps=400):
        """Move parts that overlap a part placed before them (or stick out of the board) away from
        it in small steps: what a tighter `scale` packs on top of each other ends up side by side."""
        x0b, y0b = ORIGIN[0] + 0.3, ORIGIN[1] + 0.3
        x1b, y1b = ORIGIN[0] + self.w - 0.3, ORIGIN[1] + self.h - 0.3

        def hit(a, b):
            return a[0] - gap < b[2] and b[0] - gap < a[2] and a[1] - gap < b[3] and b[1] - gap < a[3]

        for i, fp in enumerate(self.fps):
            for _ in range(max_steps):
                bb = fp.bbox()
                others = [o for o in self.fps[:i] if hit(bb, o.bbox())]
                dx = dy = 0.0
                for o in others:
                    ob = o.bbox()
                    vx = (bb[0] + bb[2] - ob[0] - ob[2]) / 2
                    vy = (bb[1] + bb[3] - ob[1] - ob[3]) / 2
                    n = math.hypot(vx, vy) or 1.0
                    dx, dy = dx + vx / n, dy + vy / n
                dx += (bb[0] < x0b) - (bb[2] > x1b)
                dy += (bb[1] < y0b) - (bb[3] > y1b)
                if not others and dx == 0 and dy == 0:
                    break
                n = math.hypot(dx, dy) or 1.0
                fp.x += step * dx / n
                fp.y += step * dy / n
            fp.x, fp.y = round(fp.x, 2), round(fp.y, 2)

    # -- checks
    def check(self):
        bx0, by0 = ORIGIN
        bx1, by1 = bx0 + self.w, by0 + self.h
        boxes = [(fp.ref, fp.bbox()) for fp in self.fps]
        refs = [r for r, _ in boxes]
        assert len(refs) == len(set(refs)), f"{self.name}: duplicate references"
        for r, (x0, y0, x1, y1) in boxes:
            assert x0 >= bx0 + 0.3 and y0 >= by0 + 0.3 and x1 <= bx1 - 0.3 and y1 <= by1 - 0.3, \
                f"{self.name}: {r} courtyard outside the board"
        for i, (ra, a) in enumerate(boxes):
            for rb, b in boxes[i + 1:]:
                if a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]:
                    raise AssertionError(f"{self.name}: courtyards of {ra} and {rb} overlap")

    def nets(self):
        out = {}
        for fp in self.fps:
            for p in fp.pads:
                if p.net:
                    out.setdefault(p.net, []).append(f"{fp.ref}.{p.num}")
        return out

    # -- writers
    def pcb(self):
        u = self.uid
        L = []
        w = L.append
        w("(kicad_pcb")
        w("\t(version 20260206)")
        w('\t(generator "gummiband_bench")')
        w('\t(generator_version "10.0")')
        w("\t(general\n\t\t(thickness 1.6)\n\t\t(legacy_teardrops no)\n\t)")
        w('\t(paper "A4")')
        w(f'\t(title_block\n\t\t(title "Gummiband bench: {self.name}")\n\t\t(comment 1 "{self.title}")\n\t)')
        w("\t(layers")
        w('\t\t(0 "F.Cu" signal)')
        inner = [self.inner] * 2 if isinstance(self.inner, str) else (self.inner or [])
        for i, kind in enumerate(inner):
            w(f'\t\t({4 + 2 * i} "In{i + 1}.Cu" {kind})')
        w('\t\t(2 "B.Cu" signal)')
        for line in ('(13 "F.Paste" user)', '(15 "B.Paste" user)', '(5 "F.SilkS" user "F.Silkscreen")',
                     '(7 "B.SilkS" user "B.Silkscreen")', '(1 "F.Mask" user)', '(3 "B.Mask" user)',
                     '(17 "Dwgs.User" user "User.Drawings")', '(19 "Cmts.User" user "User.Comments")',
                     '(25 "Edge.Cuts" user)', '(27 "Margin" user)', '(31 "F.CrtYd" user "F.Courtyard")',
                     '(29 "B.CrtYd" user "B.Courtyard")', '(35 "F.Fab" user)', '(33 "B.Fab" user)'):
            w("\t\t" + line)
        w("\t)")
        w("\t(setup\n\t\t(pad_to_mask_clearance 0)\n\t\t(allow_soldermask_bridges_in_footprints no)"
          "\n\t\t(tenting\n\t\t\t(front yes)\n\t\t\t(back yes)\n\t\t)"
          f"\n\t\t(aux_axis_origin {fmt(ORIGIN[0])} {fmt(ORIGIN[1] + self.h)})"
          f"\n\t\t(grid_origin {fmt(ORIGIN[0])} {fmt(ORIGIN[1])})\n\t)")
        for fp in self.fps:
            self._footprint(fp, w)
        x0, y0 = ORIGIN
        x1, y1 = x0 + self.w, y0 + self.h
        w(f"\t(gr_rect\n\t\t(start {fmt(x0)} {fmt(y0)})\n\t\t(end {fmt(x1)} {fmt(y1)})"
          f'\n\t\t(stroke\n\t\t\t(width 0.1)\n\t\t\t(type default)\n\t\t)\n\t\t(fill no)'
          f'\n\t\t(layer "Edge.Cuts")\n\t\t(uuid "{u()}")\n\t)')
        for net, layer in self.zones:
            m = 0.5
            pts = " ".join(f"(xy {fmt(x)} {fmt(y)})" for x, y in
                           ((x0 + m, y0 + m), (x1 - m, y0 + m), (x1 - m, y1 - m), (x0 + m, y1 - m)))
            w(f'\t(zone\n\t\t(net "{net}")\n\t\t(layer "{layer}")\n\t\t(uuid "{u()}")'
              f'\n\t\t(name "{net} plane")\n\t\t(hatch edge 0.5)\n\t\t(connect_pads\n\t\t\t(clearance 0.2)\n\t\t)'
              f"\n\t\t(min_thickness 0.25)\n\t\t(fill yes\n\t\t\t(thermal_gap 0.3)\n\t\t\t(thermal_bridge_width 0.4)"
              f"\n\t\t\t(island_removal_mode 0)\n\t\t)\n\t\t(polygon\n\t\t\t(pts\n\t\t\t\t{pts}\n\t\t\t)\n\t\t)\n\t)")
        w(")")
        return "\n".join(L) + "\n"

    def _footprint(self, fp, w):
        u = self.uid
        r = fp.rot
        ra = f" {fmt(r)}" if r else ""
        w(f'\t(footprint "{fp.lib}"\n\t\t(layer "F.Cu")\n\t\t(uuid "{u()}")\n\t\t(at {fmt(fp.x)} {fmt(fp.y)}{ra})')
        ty = fp.crtyd[1] - 0.4
        for prop, val, y, hide in (("Reference", fp.ref, ty, False), ("Value", fp.value, -ty, True)):
            w(f'\t\t(property "{prop}" "{val}"\n\t\t\t(at 0 {fmt(y)}{ra})\n\t\t\t(layer "F.Fab")'
              + ("\n\t\t\t(hide yes)" if hide else "")
              + f'\n\t\t\t(uuid "{u()}")\n\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 0.5 0.5)'
              f"\n\t\t\t\t\t(thickness 0.08)\n\t\t\t\t)\n\t\t\t)\n\t\t)")
        w("\t\t(attr through_hole)" if fp.tht else "\t\t(attr smd)")
        x0, y0, x1, y1 = fp.crtyd
        w(f"\t\t(fp_rect\n\t\t\t(start {fmt(x0)} {fmt(y0)})\n\t\t\t(end {fmt(x1)} {fmt(y1)})"
          f'\n\t\t\t(stroke\n\t\t\t\t(width 0.05)\n\t\t\t\t(type solid)\n\t\t\t)\n\t\t\t(fill no)'
          f'\n\t\t\t(layer "F.CrtYd")\n\t\t\t(uuid "{u()}")\n\t\t)')
        for p in fp.pads:
            s = [f'\t\t(pad "{p.num}" {p.kind} {p.shape}', f"\t\t\t(at {fmt(p.at[0])} {fmt(p.at[1])}{ra})",
                 f"\t\t\t(size {fmt(p.size[0])} {fmt(p.size[1])})"]
            if p.drill:
                s.append(f"\t\t\t(drill {fmt(p.drill)})")
            s.append("\t\t\t(layers " + " ".join(f'"{l}"' for l in p.layers) + ")")
            if p.kind == "thru_hole":
                s.append("\t\t\t(remove_unused_layers no)")
            if p.rratio is not None:
                s.append(f"\t\t\t(roundrect_rratio {fmt(p.rratio)})")
            if p.net:
                s.append(f'\t\t\t(net "{p.net}")')
            s.append(f'\t\t\t(uuid "{u()}")')
            w("\n".join(s) + "\n\t\t)")
        w("\t)")

    def pro(self):
        r = self.rules
        classes = []
        for i, (name, width, clr, vd, vdr) in enumerate(self.classes):
            classes.append({
                "bus_width": 12, "clearance": clr, "diff_pair_gap": 0.25, "diff_pair_via_gap": 0.25,
                "diff_pair_width": 0.2, "line_style": 0, "microvia_diameter": 0.3, "microvia_drill": 0.1,
                "name": name, "pcb_color": "rgba(0, 0, 0, 0.000)",
                "priority": 2147483647 if name == "Default" else i,
                "schematic_color": "rgba(0, 0, 0, 0.000)", "track_width": width, "tuning_profile": "",
                "via_diameter": vd, "via_drill": vdr, "wire_width": 6,
            })
        widths = sorted({0.0} | {c[1] for c in self.classes})
        vias = [{"diameter": 0.0, "drill": 0.0}] + [
            {"diameter": d, "drill": dr} for d, dr in sorted({(c[3], c[4]) for c in self.classes})]
        return {
            "board": {
                "3dviewports": [],
                "design_settings": {
                    "defaults": {"board_outline_line_width": 0.1, "copper_line_width": 0.2,
                                 "courtyard_line_width": 0.05},
                    "diff_pair_dimensions": [{"gap": 0.0, "via_gap": 0.0, "width": 0.0}],
                    "drc_exclusions": [],
                    "meta": {"version": 2},
                    "rule_severities": SEVERITIES,
                    "rules": {
                        "max_error": 0.005, "min_clearance": r["min_clearance"], "min_connection": 0.0,
                        "min_copper_edge_clearance": 0.3, "min_groove_width": 0.0,
                        "min_hole_clearance": r["min_hole_clearance"], "min_hole_to_hole": r["min_hole_to_hole"],
                        "min_microvia_diameter": 0.2, "min_microvia_drill": 0.1, "min_resolved_spokes": 1,
                        "min_silk_clearance": 0.0, "min_text_height": 0.5, "min_text_thickness": 0.08,
                        "min_through_hole_diameter": 0.2, "min_track_width": r["min_track_width"],
                        "min_via_annular_width": r["min_via_annular_width"],
                        "min_via_diameter": r["min_via_diameter"], "solder_mask_to_copper_clearance": 0.0,
                        "use_height_for_length_calcs": True,
                    },
                    "track_widths": widths,
                    "via_dimensions": vias,
                    "zones_allow_external_fillets": False,
                },
                "layer_pairs": [], "layer_presets": [], "viewports": [],
            },
            "boards": [],
            "libraries": {"pinned_footprint_libs": [], "pinned_symbol_libs": []},
            "meta": {"filename": f"{self.name}.kicad_pro", "version": 3},
            "net_settings": {
                "classes": classes,
                "meta": {"version": 5},
                "net_colors": {},
                "netclass_assignments": {},
                "netclass_patterns": [{"netclass": c, "pattern": p} for p, c in self.patterns],
            },
            "pcbnew": {"page_layout_descr_file": ""},
            "sheets": [],
            "text_variables": {},
        }

    def write(self, outdir):
        self.check()
        with open(os.path.join(outdir, f"{self.name}.kicad_pcb"), "w") as f:
            f.write(self.pcb())
        with open(os.path.join(outdir, f"{self.name}.kicad_pro"), "w") as f:
            json.dump(self.pro(), f, indent=2, sort_keys=True)
            f.write("\n")
        if self.dru:
            with open(os.path.join(outdir, f"{self.name}.kicad_dru"), "w") as f:
                f.write("(version 1)\n" + self.dru.strip() + "\n")
        nets = self.nets()
        pins = sum(len(v) for v in nets.values())
        conns = sum(len(v) - 1 for v in nets.values())
        print(f"{self.name:20s} {len(self.fps):3d} parts {len(nets):4d} nets {pins:4d} pins "
              f"{conns:4d} connections (pins - nets)")


# ---------------------------------------------------------------- helpers


def match_sorted(a_pads, b_pads, key_a, key_b, names, reverse=False):
    """Give the pads of two lists the same nets after sorting each by a key (planar bus)."""
    a = sorted(a_pads, key=key_a)
    b = sorted(b_pads, key=key_b, reverse=reverse)
    for (fa, pa), (fb, pb), n in zip(a, b, names):
        pa.net = pb.net = n


def random_nets(rng, items, prefix, near, sizes=(2, 2, 2, 3, 3, 4)):
    """items: (footprint, pad) without net. Random local nets: each net takes a random pad and
    1..3 random pads among the `near` nearest free pads of other parts."""
    free = [it for it in items if it[1].net is None]
    rng.shuffle(free)
    pos = {id(p): fp.world(p) for fp, p in free}
    taken = set()
    k = 0
    for fp, p in free:
        if id(p) in taken:
            continue
        size = rng.choice(sizes)
        q = pos[id(p)]
        cand = [(fp2, p2) for fp2, p2 in free if id(p2) not in taken and fp2 is not fp]
        cand.sort(key=lambda it: math.dist(q, pos[id(it[1])]))
        cand = cand[:near]
        if not cand:
            continue
        pick = rng.sample(cand, min(size - 1, len(cand)))
        k += 1
        name = f"{prefix}{k}"
        p.net = name
        taken.add(id(p))
        for _, p2 in pick:
            p2.net = name
            taken.add(id(p2))


def all_pads(fps):
    return [(fp, p) for fp in fps for p in fp.pads if not p.num.startswith("MP")]


# ---------------------------------------------------------------- boards


def board_easy():
    b = Board("easy", 30, 20, title="2 layers, 2 pin connections between 0603 parts")
    left = [b.add(two_terminal("R", f"R{i + 1}"), 6, 4 + 4 * i) for i in range(4)]
    right = [b.add(two_terminal("R", f"R{i + 5}"), 24, 4 + 4 * i, 90) for i in range(4)]
    mid = [b.add(two_terminal("C", f"C{i + 1}", "100n"), 15, 6 + 8 * i, 0) for i in range(2)]
    for i in range(4):
        left[i].pad(2).net = f"N{i + 1}"
        right[i].pad(1).net = f"N{i + 1}"
    left[0].pad(1).net = left[1].pad(1).net = "A"
    left[2].pad(1).net = left[3].pad(1).net = "B"
    right[0].pad(2).net = mid[0].pad(2).net = "C"
    right[3].pad(2).net = mid[1].pad(2).net = "D"
    mid[0].pad(1).net = mid[1].pad(1).net = "E"
    return b


def board_medium():
    rng = random.Random(2)
    b = Board("medium", 40, 30, title="2 layers, SOIC/SOT-23/0603, random local netlist and a GND net")
    b.net_class("Power", 0.3, 0.15, 0.6, 0.3, ["GND"])
    fps = [b.add(soic(f"U{i + 1}", 14), x, 9, 0) for i, x in enumerate((10, 30))]
    fps += [b.add(sot23(f"Q{i + 1}"), 5 + 10 * i, 21, rng.choice((0, 90, 180, 270))) for i in range(4)]
    k = 0
    for row, y in enumerate((17.0, 25.5)):
        for col in range(8):
            x = 3.5 + col * 4.6
            if row == 0 and col in (0, 2, 4, 6):
                continue  # room for the SOT-23
            k += 1
            pre = "C" if k % 3 == 0 else "R"
            fps.append(b.add(two_terminal(pre, f"{pre}{k}"), x, y, rng.choice((0, 90))))
    for fp in fps:
        if fp.ref.startswith("C"):
            fp.pad(2).net = "GND"
        if fp.ref.startswith("U"):
            fp.pad(7).net = "GND"
    random_nets(rng, all_pads(fps), "N", near=5)
    return b


def board_bus():
    b = Board("bus", 45, 35, title="2 layers, 16 bit bus between 0.5 mm connectors, 90 degree turn")
    j1 = b.add(fpc("J1", 18, 0.5), 5.5, 22, 90)  # vertical at the left, pads face right
    j2 = b.add(fpc("J2", 18, 0.5), 30, 4.5, 0)  # horizontal at the top, pads face down
    names = [f"D{i}" for i in range(16)]
    sig1 = [(j1, p) for p in j1.pads if p.num.isdigit() and 2 <= int(p.num) <= 17]
    sig2 = [(j2, p) for p in j2.pads if p.num.isdigit() and 2 <= int(p.num) <= 17]
    match_sorted(sig1, sig2, lambda it: j1.world(it[1])[1], lambda it: j2.world(it[1])[0], names)
    for j in (j1, j2):
        j.pad(1).net = j.pad(18).net = "GND"
    # a few parts inside the bend the bus has to flow around
    for i, (x, y) in enumerate(((16, 16), (20, 12), (24, 20))):
        r = b.add(two_terminal("R", f"R{i + 1}"), x, y, 45 * (i % 2))
        r.pad(1).net = f"P{i + 1}"
    b.add(two_terminal("R", "R4"), 38, 30).pad(1).net = "P1"
    b.add(two_terminal("R", "R5"), 10, 31).pad(1).net = "P2"
    b.add(two_terminal("R", "R6"), 40, 15, 90).pad(1).net = "P3"
    return b


def board_power_mix():
    b = Board("power_mix", 50, 30, title="2 layers, 1 mm and 0.75 mm nets with 0.2 mm signals through a channel")
    b.net_class("Power", 1.0, 0.2, 0.8, 0.4, ["VIN", "GND"])
    b.net_class("Power075", 0.75, 0.2, 0.8, 0.4, ["VOUT"])
    j1 = b.add(header("J1", 8), 4, 24, 90)  # pins go up from y=24
    for n, net in zip(range(1, 9), ("VIN", "VIN", "GND", "GND", "S1", "S2", "S3", "S4")):
        j1.pad(n).net = net
    # two THT walls (unconnected 2x5 headers) leave a 3.5 mm copper channel in the middle
    gap = 3.5 + 1.7
    b.add(header("J3", 5, 2, "NC"), 25, 15 - gap / 2, 90)  # pins from 15-gap/2 upwards
    b.add(header("J4", 5, 2, "NC"), 25, 15 + gap / 2 + 4 * 2.54, 90)
    u1 = b.add(soic("U1", 8, "Buck"), 37, 15)
    for n, net in zip(range(1, 9), ("VIN", "S1", "S2", "GND", "S3", "S4", "VOUT", "VIN")):
        u1.pad(n).net = net
    c1 = b.add(radial_cap("C1"), 39, 5.5)
    c1.pad(1).net, c1.pad(2).net = "VIN", "GND"
    c2 = b.add(radial_cap("C2"), 39, 24.5)
    c2.pad(1).net, c2.pad(2).net = "VOUT", "GND"
    j2 = b.add(power_terminal("J2"), 47, 15)
    j2.pad(1).net, j2.pad(2).net = "VOUT", "GND"
    for i, y in enumerate((11, 19)):
        r = b.add(two_terminal("R", f"R{i + 1}"), 31, y, 90)
        r.pad(1).net, r.pad(2).net = f"S{i + 1}", "GND"
    return b


def board_crossing():
    b = Board("crossing", 40, 26, title="2 layers, two SMD connectors with reversed pin order")
    j1 = b.add(fpc("J1", 16, 1.0, pad=(0.6, 1.6)), 6, 13, 90)
    j2 = b.add(fpc("J2", 16, 1.0, pad=(0.6, 1.6)), 34, 13, 270)
    sig1 = [(j1, p) for p in j1.pads if p.num.isdigit()]
    sig2 = [(j2, p) for p in j2.pads if p.num.isdigit()]
    match_sorted(sig1, sig2, lambda it: j1.world(it[1])[1], lambda it: j2.world(it[1])[1],
                 [f"X{i}" for i in range(16)], reverse=True)
    return b


def board_qfn_fanout():
    b = Board("qfn_fanout", 32, 32, inner="power",
              title="4 layers, QFN-48 0.5 mm to pull resistors, GND/3V3 planes on In1/In2")
    b.net_class("Power", 0.3, 0.15, 0.6, 0.3, ["GND", "+3V3"])
    b.zones = [("GND", "In1.Cu"), ("+3V3", "In2.Cu")]
    cx = cy = 16.0
    u = b.add(qfn("U1", 12, 0.5, 3.4375, 0.875, 0.25, 5.15), cx, cy)
    u.pad(49).net = "GND"
    sides = {0: [], 1: [], 2: [], 3: []}
    for i in range(48):
        p = u.pad(i + 1)
        side, k = divmod(i, 12)
        if k == 5:
            p.net = "+3V3"
        elif k == 6:
            p.net = "GND"
        else:
            sides[side].append((u, p))
    # decoupling caps next to the power pins, inside the resistor ring
    for side, (dx, dy, r) in enumerate(((-5.6, 0, 90), (0, 5.6, 0), (5.6, 0, 90), (0, -5.6, 0))):
        c = b.add(two_terminal("C", f"C{side + 1}", "100n"), cx + dx, cy + dy, r)
        c.pad(1).net, c.pad(2).net = "+3V3", "GND"
    # ring of resistors: pad 1 faces the QFN, pad 2 pulls to GND or 3V3
    k = 0
    for side in range(4):
        rs = []
        for i in range(10):
            t = -7.2 + i * 1.6
            k += 1
            x, y, r = ((cx - 9.5, cy + t, 180), (cx + t, cy + 9.5, 270),
                       (cx + 9.5, cy + t, 0), (cx + t, cy - 9.5, 90))[side]
            res = b.add(two_terminal("R", f"R{k}"), x, y, r)
            res.pad(2).net = "GND" if k % 2 else "+3V3"
            rs.append((res, res.pad(1)))
        axis = 1 if side in (0, 2) else 0
        match_sorted(sides[side], rs, lambda it: it[0].world(it[1])[axis], lambda it: it[0].world(it[1])[axis],
                     [f"S{side * 10 + i + 1}" for i in range(10)])
    # header that feeds the planes; two of its pins take over pull nets of two resistors
    j = b.add(header("J1", 4), 3, 29.5)
    j.pad(1).net, j.pad(2).net, j.pad(3).net, j.pad(4).net = "GND", "+3V3", "TX", "RX"
    b.fps[[f.ref for f in b.fps].index("R11")].pad(2).net = "TX"
    b.fps[[f.ref for f in b.fps].index("R13")].pad(2).net = "RX"
    return b


def board_four_signal_layers():
    rng = random.Random(4)
    b = Board("four_signal_layers", 40, 30, inner="signal",
              title="4 signal layers, dense random netlist (QFN-32, SOIC-14, 0603)")
    b.net_class("Power", 0.3, 0.15, 0.6, 0.3, ["GND"])
    fps = [b.add(qfn(f"U{i + 1}", 8, 0.5, 2.4375, 0.875, 0.25, 3.3), x, 8, 0) for i, x in enumerate((8, 32))]
    fps += [b.add(soic(f"U{i + 3}", 14), x, 22, 90) for i, x in enumerate((8, 32))]
    k = 0
    for y in (4.0, 8.0, 12.0, 15.5, 19.0, 23.0, 26.5):
        for x in (15.0, 20.0, 25.0):
            k += 1
            pre = "C" if k % 4 == 0 else "R"
            fps.append(b.add(two_terminal(pre, f"{pre}{k}"), x, y, rng.choice((0, 90)) if y < 15 or y > 16 else 0))
    for x in (3.0, 37.0):
        for y in (15.5,):
            k += 1
            fps.append(b.add(two_terminal("R", f"R{k}"), x, y, 90))
    for fp in fps:
        if fp.ref.startswith("C"):
            fp.pad(2).net = "GND"
    fps[0].pad(33).net = fps[1].pad(33).net = "GND"
    random_nets(rng, all_pads(fps), "N", near=10)
    return b


def _bga_board(name, n, rings, size, title, inner="power", zones=(("GND", "In1.Cu"), ("+3V3", "In2.Cu"))):
    """n x n BGA at 0.8 mm in the middle: the outer `rings` rings are signals that escape to four
    0.5 mm edge connectors (one per side), the inner balls alternate GND and 3V3 (planes)."""
    b = Board(name, size, size, inner=inner, title=title)
    b.classes = [("Default", 0.1, 0.1, 0.45, 0.2)]
    b.net_class("Power", 0.2, 0.1, 0.45, 0.2, ["GND", "+3V3"])
    b.rules.update(min_clearance=0.1, min_hole_clearance=0.15, min_via_diameter=0.45, min_via_annular_width=0.1)
    b.zones = list(zones)
    c = size / 2
    u = b.add(bga("U1", n, 0.8, 0.4), c, c)
    letters = "ABCDEFGHJKLMNPRTUVWY"
    grid = {}
    for p in u.pads:
        grid[(letters.index(p.num[0]), int(p.num[1:]) - 1)] = p
    for (r, k), p in grid.items():
        if rings <= r < n - rings and rings <= k < n - rings:
            p.net = "GND" if (r + k) % 2 == 0 else "+3V3"

    def ring(d):
        m = n - 2 * d
        top = [(d, d + i) for i in range(m - 1)]
        right = [(d + i, d + m - 1) for i in range(m - 1)]
        bottom = [(d + m - 1, d + m - 1 - i) for i in range(m - 1)]
        left = [(d + m - 1 - i, d) for i in range(m - 1)]
        return [top, right, bottom, left]

    per_side = [[], [], [], []]
    for d in range(rings):
        for s, cells in enumerate(ring(d)):
            per_side[s] += [(u, grid[cell]) for cell in cells]
    pins = len(per_side[0])
    e = size - 3.5
    conns = [
        (b.add(fpc("J1", pins, 0.5), c, 3.5, 0), 0),  # top: pads face down
        (b.add(fpc("J2", pins, 0.5), e, c, 270), 1),  # right
        (b.add(fpc("J3", pins, 0.5), c, e, 180), 0),  # bottom
        (b.add(fpc("J4", pins, 0.5), 3.5, c, 90), 1),  # left
    ]
    for s, (j, axis) in enumerate(conns):
        jp = [(j, p) for p in j.pads if p.num.isdigit()]
        match_sorted(per_side[s], jp, lambda it: it[0].world(it[1])[axis], lambda it: it[0].world(it[1])[axis],
                     [f"B{s}_{i}" for i in range(pins)])
    # power entry that feeds both planes
    j = b.add(header("J5", 2), 4, size - 4)
    j.pad(1).net, j.pad(2).net = "GND", "+3V3"
    return b


def board_bga_escape():
    return _bga_board("bga_escape", 8, 2, 30,
                      "4 layers, 8x8 BGA 0.8 mm escaping to four 0.5 mm edge connectors, GND/3V3 planes")


def board_bga_dense():
    return _bga_board("bga_dense", 10, 3, 36,
                      "4 layers, 10x10 BGA 0.8 mm with 3 signal rings (84 signals) to four edge connectors")


def board_return_vias():
    """bga_dense on 6 layers with a GND plane next to every signal layer (F.Cu, In1 GND, In2, In3,
    In4 GND, B.Cu): a signal via from the top half to the bottom half changes its reference from
    In1 to In4, a GND via next to it joins the two. The inner balls are all GND, and a skew rule
    with room to spare makes the signals fast nets (`route --return-vias`)."""
    b = _bga_board("return_vias", 10, 3, 36,
                   "6 layers, 10x10 BGA 0.8 mm, GND planes on In1 and In4 next to every signal layer",
                   inner=["power", "signal", "signal", "power"], zones=(("GND", "In1.Cu"), ("GND", "In4.Cu")))
    for fp in b.fps:
        for p in fp.pads:
            if p.net == "+3V3":
                p.net = "GND"
    b.dru = """
(rule "fast"
  (condition "A.NetName == 'B*'")
  (constraint skew (max 30mm)))
"""
    return b


def board_fine_pitch():
    """The fan-out of an RP2040 like part: 0.4 mm pitch, 0.2 mm pads, 0.19 mm clearance on two
    layers, so that a 0.2 mm track only fits straight between the pins."""
    b = Board("fine_pitch", 42, 42, title="2 layers, QFN-56 0.4 mm, 0.2/0.19 mm rules, no planes")
    b.net_class("Fine", 0.2, 0.19, 0.6, 0.3, ["F*", "GND"])
    cx = cy = 21.0
    u = b.add(qfn("U1", 14, 0.4, 3.4375, 0.875, 0.2, 3.2), cx, cy)
    u.pad(57).net = "GND"
    sides = {0: [], 1: [], 2: [], 3: []}
    for i in range(56):
        side, k = divmod(i, 14)
        sides[side].append((u, u.pad(i + 1)))
    k = 0
    for side in range(4):
        rs = []
        for i in range(14):
            t = -10.4 + i * 1.6
            k += 1
            x, y, r = ((cx - 15.0, cy + t, 180), (cx + t, cy + 15.0, 270),
                       (cx + 15.0, cy + t, 0), (cx + t, cy - 15.0, 90))[side]
            res = b.add(two_terminal("R", f"R{k}"), x, y, r)
            res.pad(2).net = "GND"
            rs.append((res, res.pad(1)))
        axis = 1 if side in (0, 2) else 0
        match_sorted(sides[side], rs, lambda it: it[0].world(it[1])[axis], lambda it: it[0].world(it[1])[axis],
                     [f"F{side * 14 + i + 1}" for i in range(14)])
    return b


def board_fine_pitch_mixed():
    """The same part with the nets shuffled: the fan-out needs vias and crossings right at the
    pins, as on a keyboard controller."""
    b = Board("fine_pitch_mixed", 42, 42, title="2 layers, QFN-56 0.4 mm with shuffled nets")
    b.net_class("Fine", 0.2, 0.19, 0.6, 0.3, ["F*", "GND"])
    cx = cy = 21.0
    u = b.add(qfn("U1", 14, 0.4, 3.4375, 0.875, 0.2, 3.2), cx, cy)
    u.pad(57).net = "GND"
    rng = random.Random(56)
    order = list(range(1, 57))
    rng.shuffle(order)
    k = 0
    for side in range(4):
        for i in range(14):
            t = -10.4 + i * 1.6
            k += 1
            x, y, r = ((cx - 15.0, cy + t, 180), (cx + t, cy + 15.0, 270),
                       (cx + 15.0, cy + t, 0), (cx + t, cy - 15.0, 90))[side]
            res = b.add(two_terminal("R", f"R{k}"), x, y, r)
            res.pad(2).net = "GND"
            res.pad(1).net = u.pad(order[k - 1]).net = f"F{k}"
    return b


def board_diff_pairs():
    """Eight differential pairs between two connectors that sit at an angle, around parts."""
    b = Board("diff_pairs", 50, 36, title="2 layers, 8 differential pairs (_P, _N) between 0.5 mm connectors")
    j1 = b.add(fpc("J1", 26, 0.5), 5.5, 18, 90)
    j2 = b.add(fpc("J2", 26, 0.5), 36, 4.5, 0)
    names = []
    for i in range(8):
        names += [f"L{i}_P", f"L{i}_N", f"G{i}"]
    sig1 = [(j1, p) for p in j1.pads if p.num.isdigit() and 2 <= int(p.num) <= 25]
    sig2 = [(j2, p) for p in j2.pads if p.num.isdigit() and 2 <= int(p.num) <= 25]
    match_sorted(sig1, sig2, lambda it: j1.world(it[1])[1], lambda it: j2.world(it[1])[0], names)
    for j in (j1, j2):
        j.pad(1).net = j.pad(26).net = "GND"
    for i, (x, y) in enumerate(((20, 14), (26, 22), (32, 16), (16, 26))):
        r = b.add(two_terminal("R", f"R{i + 1}"), x, y, 45 * (i % 2))
        r.pad(1).net = f"P{i + 1}"
        q = b.add(two_terminal("R", f"R{i + 5}"), 42 - i * 6, 31)
        q.pad(1).net = f"P{i + 1}"
    # the pairs as KiCad knows them: width and gap of the net class, coupled within 0.1 mm skew
    b.dru = """
(rule "pairs"
  (condition "A.inDiffPair('L*')")
  (constraint track_width (opt 0.2mm))
  (constraint diff_pair_gap (min 0.2mm) (opt 0.25mm)))
(rule "pair skew"
  (condition "A.inDiffPair('L*')")
  (constraint skew (max 0.1mm) (within_diff_pairs)))
"""
    return b


def board_matching():
    """A byte lane (8 data nets and a strobe pair) and a clock pair between two connectors at an
    angle, with the length rules of a memory interface: one skew group, a length window for the
    clock, little skew within the clock pair and little uncoupled length."""
    b = Board("matching", 46, 34, title="2 layers, byte lane and clock pair with length, skew and uncoupled rules")
    j1 = b.add(fpc("J1", 16, 0.5), 5.5, 19, 90)
    j2 = b.add(fpc("J2", 16, 0.5), 30, 4.5, 0)
    names = ["DQ0", "DQ1", "DQ2", "DQ3", "DQS_P", "DQS_N", "DQ4", "DQ5", "DQ6", "DQ7", "G1", "CLK_P", "CLK_N",
             "G2"]
    sig1 = [(j1, p) for p in j1.pads if p.num.isdigit() and 2 <= int(p.num) <= 15]
    sig2 = [(j2, p) for p in j2.pads if p.num.isdigit() and 2 <= int(p.num) <= 15]
    match_sorted(sig1, sig2, lambda it: j1.world(it[1])[1], lambda it: j2.world(it[1])[0], names)
    for j in (j1, j2):
        j.pad(1).net = j.pad(16).net = "GND"
    for i, (x, y) in enumerate(((18, 18), (24, 24), (34, 20))):
        r = b.add(two_terminal("C", f"C{i + 1}", "100n"), x, y, 45 * (i % 2))
        r.pad(1).net = "GND"
        r.pad(2).net = f"V{i + 1}"
        q = b.add(two_terminal("C", f"C{i + 4}", "100n"), 40 - i * 7, 30)
        q.pad(1).net = "GND"
        q.pad(2).net = f"V{i + 1}"
    b.dru = """
(rule "pairs"
  (condition "A.inDiffPair('*')")
  (constraint track_width (opt 0.15mm))
  (constraint diff_pair_gap (min 0.15mm) (opt 0.15mm)))
(rule "uncoupled"
  (condition "A.inDiffPair('*')")
  (constraint diff_pair_uncoupled (max 3mm)))
(rule "byte lane"
  (condition "A.NetName == 'DQ?' || A.inDiffPair('DQS')")
  (constraint skew (max 0.2mm)))
(rule "clock"
  (condition "A.inDiffPair('CLK')")
  (constraint length (min 40mm) (max 41mm))
  (constraint skew (max 0.05mm) (within_diff_pairs)))
"""
    return b


PAIR_DRU = """
(rule "pairs"
  (condition "A.inDiffPair('*')")
  (constraint track_width (opt 0.2mm))
  (constraint diff_pair_gap (min 0.2mm) (opt 0.25mm)))
(rule "pair skew"
  (condition "A.inDiffPair('*')")
  (constraint skew (max 0.1mm) (within_diff_pairs)))
"""


def board_pair_wall():
    """A USB pair from a connector to a QFN through the one gap of a wall of THT pins. The gap
    takes the pair as one wide track or three single tracks; between the pins of the wall only one
    single track fits. Six short connections cross the wall right next to the gap: routed first,
    the two nearest take the gap and the pair has to change layers."""
    gap_y = 13.76
    b = Board("pair_wall", 50, 27.6, title="2 layers, USB pair through the one gap of a THT wall, short nets across")
    j1 = b.add(fpc("J1", 6, 0.5), 4, gap_y, 90)
    u1 = b.add(qfn("U1", 8, 0.5, 2.4375, 0.875, 0.25, 3.3), 43, gap_y)
    jp = sorted([j1.pad(3), j1.pad(4)], key=lambda p: j1.world(p)[1])
    up = sorted([u1.pad(4), u1.pad(5)], key=lambda p: u1.world(p)[1])
    for a, c, n in zip(jp, up, ("USB_P", "USB_N")):
        a.net = c.net = n
    # the wall: pin rows above and below the gap, 1.3 mm between the pads of the gap
    offs = [-1.5 - 2.54 * i for i in range(5)] + [1.5 + 2.54 * i for i in range(5)]
    b.add(wall("W1", offs), 25, gap_y, 270)
    for i, dy in enumerate((-5.0, -3.2, -1.4, 1.4, 3.2, 5.0)):
        a = b.add(two_terminal("R", f"R{i + 1}"), 21.5, gap_y + dy)
        c = b.add(two_terminal("R", f"R{i + 7}"), 28.5, gap_y + dy)
        a.pad(2).net = c.pad(1).net = f"S{i + 1}"
    b.dru = PAIR_DRU
    return b


def board_pair_traffic():
    """Two long pairs from a connector to a QFN across a field of 0603 parts with random local
    nets, some of which cross the band the pairs run in."""
    rng = random.Random(7)
    b = Board("pair_traffic", 56, 36, title="2 layers, two long pairs across a field of local nets")
    j1 = b.add(fpc("J1", 8, 0.5), 4, 18, 90)
    u1 = b.add(qfn("U1", 8, 0.5, 2.4375, 0.875, 0.25, 3.3), 49, 18)
    jp = sorted([j1.pad(n) for n in (2, 3, 6, 7)], key=lambda p: j1.world(p)[1])
    up = sorted([u1.pad(n) for n in (2, 3, 6, 7)], key=lambda p: u1.world(p)[1])
    for a, c, n in zip(jp, up, ("USB_P", "USB_N", "LVDS_P", "LVDS_N")):
        a.net = c.net = n
    fps = []
    k = 0
    for y in (4.0, 8.0, 12.5, 23.5, 28.0, 32.0):
        for x in (13.0, 18.0, 23.0, 28.0, 33.0, 38.0):
            k += 1
            fps.append(b.add(two_terminal("R", f"R{k}"), x, y, rng.choice((0, 90))))
    random_nets(rng, all_pads(fps), "N", near=5, sizes=(2, 2, 2, 3))
    b.dru = PAIR_DRU
    return b


def wire(a, pa, c, pc, name):
    a.pad(pa).net = c.pad(pc).net = name


def board_parts_easy():
    """Part to part, level 1: an MCU and an SPI flash whose pins face each other in order, and a
    few local connections around the MCU."""
    b = Board("parts_easy", 40, 30, title="2 layers, QFN-32 and SOIC-8 with a planar SPI bus, local nets")
    u1 = b.add(qfn("U1", 8, 0.5, 2.4375, 0.875, 0.25, 3.3), 13, 15)
    u2 = b.add(soic("U2", 8, "Flash"), 30, 15)
    a = sorted([u1.pad(n) for n in (19, 20, 21, 22)], key=lambda p: u1.world(p)[1])
    c = sorted([u2.pad(n) for n in (1, 2, 3, 4)], key=lambda p: u2.world(p)[1])
    for pa, pc, n in zip(a, c, ("CS", "MISO", "MOSI", "SCK")):
        pa.net = pc.net = n
    u2.pad(5).net = "GND"
    u2.pad(8).net = "+3V3"
    u1.pad(33).net = "GND"
    u1.pad(17).net = "+3V3"
    c1 = b.add(two_terminal("C", "C1", "100n"), 30, 21)
    c1.pad(1).net, c1.pad(2).net = "+3V3", "GND"
    c2 = b.add(two_terminal("C", "C2", "100n"), 19, 21, 90)
    c2.pad(1).net, c2.pad(2).net = "+3V3", "GND"
    # local: four series resistors from the left pins of the MCU to a header
    j1 = b.add(header("J1", 4), 3, 9, 270)
    for i, n in enumerate((2, 4, 6, 8)):
        r = b.add(two_terminal("R", f"R{i + 1}"), 7.5, 10.5 + 1.8 * i)
        r.pad(2).net = u1.pad(n).net = f"L{i + 1}"
        r.pad(1).net = j1.pad(i + 1).net = f"H{i + 1}"
    # two more pins of the flash to a header at the right edge
    j2 = b.add(header("J2", 4), 37, 11, 270)
    wire(u2, 6, j2, 1, "WP")
    wire(u2, 7, j2, 2, "HOLD")
    j2.pad(3).net, j2.pad(4).net = "+3V3", "GND"
    return b


def board_parts_medium():
    """Part to part, level 2: an MCU with an SPI flash in its real pinout (the bus leaves on both
    sides of the flash), an I2C sensor with pull-ups and an interrupt, and a 6 bit parallel bus
    to a SOIC-14 with one swap, plus random local nets."""
    rng = random.Random(12)
    b = Board("parts_medium", 46, 40, title="2 layers, MCU with SPI flash, I2C sensor with pull-ups, 6 bit bus, local nets")
    u1 = b.add(qfn("U1", 8, 0.5, 2.4375, 0.875, 0.25, 3.3), 16, 17)
    u2 = b.add(soic("U2", 8, "Flash"), 36, 8)
    u3 = b.add(soic("U3", 8, "Sensor"), 36, 27)
    u4 = b.add(soic("U4", 14, "Latch"), 15, 33, 90)
    # SPI flash: 1 CS, 2 MISO, 5 MOSI, 6 SCK (pins 5 and 6 on the far side)
    wire(u1, 24, u2, 1, "CS")
    wire(u1, 23, u2, 2, "MISO")
    wire(u1, 22, u2, 5, "MOSI")
    wire(u1, 21, u2, 6, "SCK")
    u2.pad(4).net = u3.pad(4).net = u4.pad(7).net = u1.pad(33).net = "GND"
    u2.pad(8).net = u3.pad(8).net = u4.pad(14).net = u1.pad(25).net = "+3V3"
    # I2C sensor: SDA, SCL with pull-ups to 3V3, and an interrupt
    wire(u1, 18, u3, 1, "SDA")
    wire(u1, 17, u3, 2, "SCL")
    wire(u1, 19, u3, 3, "INT")
    for i, n in enumerate(("SDA", "SCL")):
        r = b.add(two_terminal("R", f"R{i + 1}", "4k7"), 28, 21 + 2 * i)
        r.pad(1).net, r.pad(2).net = "+3V3", n
    # 6 bit bus from the bottom pins of the MCU to the latch, bits 2 and 3 swapped
    a = sorted([u1.pad(n) for n in range(10, 16)], key=lambda p: u1.world(p)[0])
    c = sorted([p for p in u4.pads if int(p.num) in (8, 9, 10, 11, 12, 13)], key=lambda p: u4.world(p)[0])
    names = ["Q0", "Q1", "Q3", "Q2", "Q4", "Q5"]
    for pa, pc, n in zip(a, c, names):
        pa.net = pc.net = n
    # decoupling and local nets on the left
    for i, (x, y, r) in enumerate(((36, 13, 0), (36, 32, 0), (24, 33, 90))):
        cap = b.add(two_terminal("C", f"C{i + 1}", "100n"), x, y, r)
        cap.pad(1).net, cap.pad(2).net = "+3V3", "GND"
    loc = []
    for i, (x, y) in enumerate(((4, 6), (4, 10), (4, 14), (4, 18), (4, 22), (9, 3), (14, 3), (19, 3))):
        loc.append(b.add(two_terminal("R", f"R{i + 3}"), x, y, 90 if x == 4 else 0))
    items = all_pads(loc) + [(u1, u1.pad(n)) for n in (1, 2, 3, 4, 5, 6, 7, 8, 26, 27, 28, 29, 30, 31, 32)]
    random_nets(rng, items, "N", near=4, sizes=(2, 2, 3))
    return b


def board_parts_hard():
    """Part to part, level 3: a QFN-48 MCU in the middle with a shared SPI bus to two devices, a
    shared I2C bus with pull-ups to two more, an 8 bit bus with two swaps to an edge connector, a
    USB pair to the other edge and random local nets, on two layers."""
    rng = random.Random(48)
    b = Board("parts_hard", 54, 44, title="2 layers, QFN-48 MCU with shared SPI and I2C, 8 bit bus, USB pair, local nets")
    cx, cy = 26.0, 22.0
    u1 = b.add(qfn("U1", 12, 0.5, 3.4375, 0.875, 0.25, 5.15), cx, cy)
    u2 = b.add(soic("U2", 8, "Flash"), 10, 7)
    u3 = b.add(soic("U3", 8, "ADC"), 40, 7)
    u4 = b.add(soic("U4", 8, "Sensor"), 10, 37)
    u5 = b.add(soic("U5", 8, "EEPROM"), 40, 37)
    gnd, vcc = "GND", "+3V3"
    for u in (u2, u3, u4, u5):
        u.pad(4).net, u.pad(8).net = gnd, vcc
    u1.pad(49).net = gnd
    for n in (6, 30):
        u1.pad(n).net = vcc
    for n in (7, 31):
        u1.pad(n).net = gnd
    # shared SPI from the top pins (37..48 run right to left): SCK, MOSI, MISO to both, CS each
    wire(u1, 43, u2, 6, "SCK")
    u3.pad(6).net = "SCK"
    wire(u1, 42, u2, 5, "MOSI")
    u3.pad(5).net = "MOSI"
    wire(u1, 44, u2, 2, "MISO")
    u3.pad(2).net = "MISO"
    wire(u1, 46, u2, 1, "CS0")
    wire(u1, 39, u3, 1, "CS1")
    # shared I2C from the bottom pins (13..24 run left to right) with pull-ups
    wire(u1, 17, u4, 5, "SDA")
    u5.pad(5).net = "SDA"
    wire(u1, 18, u4, 6, "SCL")
    u5.pad(6).net = "SCL"
    wire(u1, 15, u4, 1, "INT0")
    wire(u1, 21, u5, 1, "WC")
    for i, n in enumerate(("SDA", "SCL")):
        r = b.add(two_terminal("R", f"R{i + 1}", "4k7"), 24 + 4 * i, 33)
        r.pad(1).net, r.pad(2).net = vcc, n
    # 8 bit bus from the right pins (25..36 run bottom to top) to an edge connector, two swaps
    j1 = b.add(fpc("J1", 12, 0.5), 50, cy, 270)
    a = sorted([u1.pad(n) for n in range(27, 35)], key=lambda p: u1.world(p)[1])
    c = sorted([p for p in j1.pads if p.num.isdigit() and 3 <= int(p.num) <= 10], key=lambda p: j1.world(p)[1])
    for pa, pc, n in zip(a, c, ["D0", "D2", "D1", "D3", "D4", "D6", "D5", "D7"]):
        pa.net = pc.net = n
    j1.pad(1).net = j1.pad(12).net = gnd
    j1.pad(2).net = j1.pad(11).net = vcc
    # USB pair from the left pins (1..12 top to bottom) to the left edge
    j2 = b.add(fpc("J2", 6, 0.5), 3.5, cy, 90)
    jp = sorted([j2.pad(3), j2.pad(4)], key=lambda p: j2.world(p)[1])
    up = sorted([u1.pad(9), u1.pad(10)], key=lambda p: u1.world(p)[1])
    for pa, pc, n in zip(jp, up, ("USB_P", "USB_N")):
        pa.net = pc.net = n
    j2.pad(1).net = j2.pad(6).net = gnd
    # decoupling at the MCU and local nets
    for i, (x, y, r) in enumerate(((cx - 6.5, cy - 6.5, 45), (cx + 6.5, cy + 6.5, 45), (10, 12, 0), (40, 12, 0),
                                   (10, 32, 0), (40, 32, 0))):
        cap = b.add(two_terminal("C", f"C{i + 1}", "100n"), x, y, r)
        cap.pad(1).net, cap.pad(2).net = vcc, gnd
    loc = []
    for i, (x, y, r) in enumerate(((16, 12, 90), (22, 12, 90), (30, 12, 90), (36, 12, 90), (16, 30, 90),
                                   (35, 30, 90), (12, 18, 0), (12, 26, 0), (40, 16, 0), (40, 28, 0))):
        loc.append(b.add(two_terminal("R", f"R{i + 3}"), x, y, r))
    free = [(u1, p) for p in u1.pads if p.net is None]
    random_nets(rng, all_pads(loc) + free, "N", near=4, sizes=(2, 2, 3))
    b.dru = PAIR_DRU
    return b


def board_parts_multi(feed=False, name=None, scale=1.0, title=None):
    """Part to part, level 4: two big chips (QFN-48 MCU, QFN-32 PHY) with a 12 net bus between
    them, each with its own peripherals and connectors: SPI flash and a 6 bit bus at the MCU, an
    I2C bus shared by both chips with pull-ups, a USB pair to one edge, two pairs from the PHY to a
    connector at the other edge, a debug header wired to both chips, random local nets. Four layers, GND and 3V3 planes inside."""
    rng = random.Random(64)
    b = Board(name or ("parts_multi_feed" if feed else "parts_multi"), round(74 * scale, 2), round(52 * scale, 2),
              inner="power",
              title=title or "4 layers, MCU and PHY with a 12 net bus, peripherals, shared I2C, three pairs, debug header")
    b.scale = scale
    b.net_class("Power", 0.3, 0.15, 0.6, 0.3, ["GND", "+3V3"])
    b.zones = [("GND", "In1.Cu"), ("+3V3", "In2.Cu")]
    gnd, vcc = "GND", "+3V3"
    u1 = b.add(qfn("U1", 12, 0.5, 3.4375, 0.875, 0.25, 5.15), 22, 26)
    u2 = b.add(qfn("U2", 8, 0.5, 2.4375, 0.875, 0.25, 3.3), 52, 26)
    u3 = b.add(soic("U3", 8, "Flash"), 10, 9)
    u4 = b.add(soic("U4", 14, "Latch"), 22, 44, 90)
    u5 = b.add(soic("U5", 8, "EEPROM"), 37, 44)
    u6 = b.add(soic("U6", 8, "Sensor"), 52, 44)
    u1.pad(49).net = u2.pad(33).net = gnd
    for n in (6, 30):
        u1.pad(n).net = vcc
    for n in (7, 31):
        u1.pad(n).net = gnd
    u2.pad(1).net, u2.pad(32).net = vcc, gnd
    for u in (u3, u5, u6):
        u.pad(4).net, u.pad(8).net = gnd, vcc
    u4.pad(7).net, u4.pad(14).net = gnd, vcc
    # the bus between the chips: right side of U1 (25..36 bottom to top) to the left side of U2
    # (1..8 top to bottom) and its bottom side (9..16 left to right), three nets crossed
    a = sorted([u1.pad(n) for n in range(26, 36) if n != 30] + [u1.pad(36), u1.pad(25)], key=lambda p: u1.world(p)[1])
    c = sorted([u2.pad(n) for n in range(2, 9)], key=lambda p: u2.world(p)[1]) + \
        sorted([u2.pad(n) for n in range(9, 14)], key=lambda p: u2.world(p)[0])
    names = ["A0", "A1", "A2", "A3", "A5", "A4", "A6", "A7", "WR", "RD", "CS", "IRQ"]
    for pa, pc, n in zip(a, c, names):
        pa.net = pc.net = n
    # SPI flash at the top left, from the top pins of U1 (37..48 right to left)
    wire(u1, 46, u3, 1, "F_CS")
    wire(u1, 45, u3, 2, "F_MISO")
    wire(u1, 44, u3, 5, "F_MOSI")
    wire(u1, 43, u3, 6, "F_SCK")
    # 6 bit bus to the latch below, from the bottom pins (13..24 left to right), one swap
    a = sorted([u1.pad(n) for n in range(14, 20)], key=lambda p: u1.world(p)[0])
    c = sorted([p for p in u4.pads if int(p.num) in (8, 9, 10, 11, 12, 13)], key=lambda p: u4.world(p)[0])
    for pa, pc, n in zip(a, c, ["Q0", "Q2", "Q1", "Q3", "Q4", "Q5"]):
        pa.net = pc.net = n
    # I2C shared by both chips, with pull-ups, to the EEPROM and the sensor
    wire(u1, 21, u5, 5, "SDA")
    u6.pad(5).net = u2.pad(15).net = "SDA"
    wire(u1, 22, u5, 6, "SCL")
    u6.pad(6).net = u2.pad(16).net = "SCL"
    wire(u2, 14, u6, 1, "S_INT")
    for i, n in enumerate(("SDA", "SCL")):
        r = b.add(two_terminal("R", f"R{i + 1}", "4k7"), 44, 36 + 2 * i)
        r.pad(1).net, r.pad(2).net = vcc, n
    # USB pair at the left edge
    j1 = b.add(fpc("J1", 6, 0.5), 3.5, 26, 90)
    jp = sorted([j1.pad(3), j1.pad(4)], key=lambda p: j1.world(p)[1])
    up = sorted([u1.pad(5), u1.pad(4)], key=lambda p: u1.world(p)[1])
    for pa, pc, n in zip(jp, up, ("USB_P", "USB_N")):
        pa.net = pc.net = n
    j1.pad(1).net = j1.pad(6).net = gnd
    # two pairs and two LED lines from the PHY to a connector at the right edge
    j2 = b.add(fpc("J2", 10, 0.5), 70, 26, 270)
    jp = sorted([j2.pad(n) for n in (2, 3, 5, 6, 8, 9)], key=lambda p: j2.world(p)[1])
    up = sorted([u2.pad(n) for n in (19, 20, 21, 22, 23, 24)], key=lambda p: u2.world(p)[1])
    for pa, pc, n in zip(jp, up, ("TX_P", "TX_N", "LED0", "LED1", "RX_P", "RX_N")):
        pa.net = pc.net = n
    j2.pad(1).net = j2.pad(10).net = gnd
    j2.pad(4).net = j2.pad(7).net = gnd
    # debug header at the top, wired to both chips
    j3 = b.add(header("J3", 5, 2), 34, 4)
    wire(j3, 1, u1, 39, "SWDIO")
    wire(j3, 3, u1, 40, "SWCLK")
    wire(j3, 5, u1, 41, "NRST")
    wire(j3, 7, u2, 28, "TMS")
    wire(j3, 9, u2, 27, "TCK")
    wire(j3, 2, u2, 26, "TDI")
    wire(j3, 4, u2, 25, "TDO")
    j3.pad(6).net, j3.pad(8).net, j3.pad(10).net = gnd, vcc, gnd
    # decoupling next to the chips
    for i, (x, y, r) in enumerate(((15.5, 19.5, 45), (28.5, 32.5, 45), (47, 21, 0), (57, 31, 0),
                                   (10, 14, 0), (37, 39.5, 0), (52, 39.5, 0), (29, 44, 90))):
        cap = b.add(two_terminal("C", f"C{i + 1}", "100n"), x, y, r)
        cap.pad(1).net, cap.pad(2).net = vcc, gnd
    # local parts with random nets around the MCU and the PHY
    loc = []
    for i, (x, y, r) in enumerate(((10, 18, 0), (10, 22, 0), (10, 31, 0), (10, 35, 0), (22, 14, 90), (26, 14, 90),
                                   (60, 14, 90), (64, 14, 90), (60, 38, 90), (64, 38, 90), (44, 14, 90),
                                   (40, 30, 0))):
        loc.append(b.add(two_terminal("R", f"R{i + 3}"), x, y, r))
    free = [(u, p) for u in (u1, u2) for p in u.pads if p.net is None]
    free += [(u4, p) for p in u4.pads if p.net is None] + [(u3, u3.pad(3)), (u3, u3.pad(7))]
    random_nets(rng, all_pads(loc) + free, "N", near=4, sizes=(2, 2, 3))
    if b.scale != 1.0:
        b.spread()
    if feed:
        # chicken feed: 0603 parts wherever there is room, half of them on a plane net with one
        # pad (a via each), the other pads in small random nets with their neighbours
        add_feed(b, rng, gnd, vcc)
    b.dru = PAIR_DRU
    return b


def add_feed(b, rng, gnd, vcc):
    k = b.scale
    feed = scatter(b, rng, "F", 40, keep_out=[(14 * k, 18 * k, 30 * k, 34 * k), (47 * k, 21 * k, 57 * k, 31 * k)])
    for f in feed:
        if rng.random() < 0.5:
            f.pad(1).net = rng.choice((gnd, vcc))
    random_nets(rng, all_pads(feed), "M", near=3, sizes=(2, 2, 3))


def board_parts_multi_feed():
    """Part to part, level 5: parts_multi with 40 scattered 0603 parts (chicken feed) in the way."""
    return board_parts_multi(feed=True)


def board_parts_xhard():
    """Part to part, extra hard: parts_multi_feed on a board 25 % smaller in each direction, the
    same parts and nets closer together (the chicken feed takes what room is left)."""
    return board_parts_multi(feed=True, name="parts_xhard", scale=0.75,
                             title="4 layers, parts_multi_feed on a board 25 % smaller in each direction")


def scatter(b, rng, prefix, n, keep_out=(), tries=4000):
    """Up to n 0603 parts at random places and angles (0, 90) where their courtyard keeps 0.4 mm
    from every part, the board edge and the keep-out boxes (x0, y0, x1, y1 relative to the board)."""
    placed = []
    boxes = [fp.bbox() for fp in b.fps]
    ko = [(ORIGIN[0] + x0, ORIGIN[1] + y0, ORIGIN[0] + x1, ORIGIN[1] + y1) for x0, y0, x1, y1 in keep_out]
    k = 0
    for _ in range(tries):
        if len(placed) == n:
            break
        x, y = round(rng.uniform(2.5, b.w - 2.5), 1), round(rng.uniform(2.5, b.h - 2.5), 1)
        fp = two_terminal("C" if rng.random() < 0.5 else "R", f"{prefix}{k + 1}").place(ORIGIN[0] + x, ORIGIN[1] + y,
                                                                                      rng.choice((0, 90)))
        x0, y0, x1, y1 = fp.bbox()
        if x0 < ORIGIN[0] + 0.7 or y0 < ORIGIN[1] + 0.7 or x1 > ORIGIN[0] + b.w - 0.7 or y1 > ORIGIN[1] + b.h - 0.7:
            continue
        m = 0.4
        if any(x0 - m < c[2] and c[0] < x1 + m and y0 - m < c[3] and c[1] < y1 + m for c in boxes + ko):
            continue
        k += 1
        b.fps.append(fp)
        boxes.append(fp.bbox())
        placed.append(fp)
    return placed


BOARDS = [board_easy, board_medium, board_bus, board_power_mix, board_crossing, board_qfn_fanout,
          board_four_signal_layers, board_bga_escape, board_bga_dense, board_return_vias, board_fine_pitch, board_fine_pitch_mixed,
          board_diff_pairs, board_matching, board_pair_wall, board_pair_traffic, board_parts_easy,
          board_parts_medium, board_parts_hard, board_parts_multi,
          board_parts_multi_feed, board_parts_xhard]


def main():
    outdir = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "boards")
    os.makedirs(outdir, exist_ok=True)
    for make in BOARDS:
        make().write(outdir)


if __name__ == "__main__":
    main()
