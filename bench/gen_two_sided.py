#!/usr/bin/env python3
"""Generate the synthetic boards for two sided placement (KiCad 10 format).

Writes <name>.kicad_pcb and <name>.kicad_pro per board into bench/boards/two_sided/ (or the
directory given as first argument). Plain Python 3, reuses the footprints and the writer of
gen_boards.py. Deterministic like gen_boards.py.

These boards test the placement only (`gummiband place --two-sided`), not the routing: the movable
parts start in rows below the board, the parts that stay (connectors, mounting holes, locked
parts) sit where they belong, some of them on the back.

  ts_som     4 layers, a small system on module: SoC BGA, memory BGA, PMIC with four inductors,
             two QFN-32, ~120 0402 decoupling capacitors and resistors; a board to board connector
             locked on the back, four mounting holes. The parts need about 1.4 times the front.
  ts_sensor  2 layers, a battery powered radio sensor: QFN-32 radio with crystal and match, an
             antenna keepout on the front, a coin cell holder locked on the back over most of it,
             a micro USB connector at the edge, sensors, a display driver and passives. One sided
             the holder takes the front too and nothing fits; two sided the front holds it all.
  ts_roomy   2 layers, MCU, flash, sensors and passives at about half the room of the front: a
             control, two sided placement should leave the back (nearly) empty.
  ts_tht     2 layers, a power board with through hole electrolytics and a terminal block between
             SMD converters and their passives: on the back the through hole parts take only their
             holes, small parts may sit under their bodies.
"""
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_boards as gb  # noqa: E402
from gen_boards import ORIGIN, SMD, THT, Board, Footprint, Pad, fmt  # noqa: E402


def chip(prefix, ref, size, value):
    """Two terminal SMD chip in the KiCad metric sizes: 0402, 0603, 0805, 1210."""
    dims = {  # pad offset, pad size, courtyard half extents
        "0402": (0.48, (0.56, 0.62), (0.93, 0.47)),
        "0603": (0.825, (0.8, 0.95), (1.48, 0.73)),
        "0805": (0.95, (1.0, 1.45), (1.68, 0.95)),
        "1210": (1.4, (1.25, 2.65), (2.28, 1.6)),
    }
    off, pad, (cx, cy) = dims[size]
    pads = [Pad(str(i + 1), "smd", "roundrect", (s * off, 0), pad, SMD, rratio=0.25) for i, s in enumerate((-1, 1))]
    return Footprint(f"Bench:{prefix}_{size}", ref, value, pads, (-cx, -cy, cx, cy))


def power_inductor(ref, value="2u2"):
    """4x4 mm shielded power inductor."""
    pads = [Pad(str(i + 1), "smd", "rect", (s * 1.45, 0), (1.1, 3.6), SMD) for i, s in enumerate((-1, 1))]
    return Footprint("Bench:L_4040", ref, value, pads, (-2.3, -2.3, 2.3, 2.3))


def crystal(ref, value="32M"):
    """3.2 x 2.5 mm crystal, four pads."""
    pads = [Pad(str(i + 1), "smd", "rect", (x, y), (1.2, 1.0), SMD)
            for i, (x, y) in enumerate(((-1.1, 0.8), (1.1, 0.8), (1.1, -0.8), (-1.1, -0.8)))]
    return Footprint("Bench:Crystal_3225", ref, value, pads, (-2.1, -1.75, 2.1, 1.75))


def mounting_hole(ref):
    pads = [Pad("", "np_thru_hole", "circle", (0, 0), (3.2, 3.2), ["*.Cu", "*.Mask"], drill=3.2)]
    return Footprint("Bench:MountingHole_3.2mm_M3", ref, "MH", pads, (-3.45, -3.45, 3.45, 3.45), tht=True)


def b2b(ref, n, value="B2B"):
    """Board to board connector, 2 rows of n/2 pads at 0.5 mm, pads along local x."""
    half = n // 2
    c = (half - 1) / 2 * 0.5
    pads = []
    for r, y in enumerate((-1.6, 1.6)):
        for i in range(half):
            pads.append(Pad(str(r * half + i + 1), "smd", "rect", (-c + i * 0.5, y), (0.3, 1.0), SMD))
    for i, s in enumerate((-1, 1)):
        pads.append(Pad(f"MP{i + 1}", "smd", "rect", (s * (c + 1.4), 0), (1.0, 2.0), SMD))
    e = c + 2.2
    return Footprint(f"Bench:B2B-{n}", ref, value, pads, (-e, -2.6, e, 2.6))


def coin_holder(ref):
    """CR2032 holder, SMD: two tabs and a ground pad in the middle."""
    pads = [Pad("1", "smd", "rect", (-12.5, 0), (3.0, 5.0), SMD), Pad("2", "smd", "rect", (12.5, 0), (3.0, 5.0), SMD),
            Pad("3", "smd", "circle", (0, 0), (8.0, 8.0), SMD)]
    return Footprint("Bench:BatteryHolder_CR2032", ref, "CR2032", pads, (-14.5, -10.5, 14.5, 10.5))


def usb_micro(ref):
    """Micro USB receptacle, SMD only: five pins and four shield tabs."""
    pads = [Pad(str(i + 1), "smd", "rect", (-1.3 + i * 0.65, -2.7), (0.4, 1.35), SMD) for i in range(5)]
    for i, (x, y) in enumerate(((-3.1, -2.55), (3.1, -2.55), (-1.2, 0), (1.2, 0))):
        pads.append(Pad(f"S{i + 1}", "smd", "rect", (x, y), (2.1 if y == 0 else 1.6, 1.9 if y == 0 else 1.4), SMD))
    return Footprint("Bench:USB_Micro-B_SMD", ref, "USB", pads, (-4.6, -3.9, 4.6, 1.6))


def usb_c(ref):
    """USB-C receptacle: a row of SMD pads and four through hole pegs of the shield."""
    pads = []
    for i in range(12):
        pads.append(Pad(str(i + 1), "smd", "rect", (-2.75 + i * 0.5, -1.0), (0.3, 1.1), SMD))
    for i, (x, y) in enumerate(((-4.32, 0.0), (4.32, 0.0), (-4.32, 4.2), (4.32, 4.2))):
        pads.append(Pad(f"S{i + 1}", "thru_hole", "oval", (x, y), (1.0, 1.6), THT, drill=0.6))
    return Footprint("Bench:USB_C_Receptacle", ref, "USB-C", pads, (-5.3, -2.1, 5.3, 5.5), tht=True)


def radial(ref, d, value):
    """Radial electrolytic with diameter d, pitch d / 2.5."""
    p = round(d / 2.5, 1)
    pads = [Pad("1", "thru_hole", "rect", (-p / 2, 0), (1.6, 1.6), THT, drill=0.8),
            Pad("2", "thru_hole", "circle", (p / 2, 0), (1.6, 1.6), THT, drill=0.8)]
    e = d / 2 + 0.25
    return Footprint(f"Bench:CP_Radial_D{d}mm", ref, value, pads, (-e, -e, e, e), tht=True)


def terminal(ref, n):
    """Screw terminal, 5.08 mm pitch, through hole."""
    pads = [Pad(str(i + 1), "thru_hole", "rect" if i == 0 else "circle", (i * 5.08, 0), (2.4, 2.4), THT, drill=1.3)
            for i in range(n)]
    return Footprint(f"Bench:TerminalBlock_1x{n:02d}_P5.08mm", ref, "Term", pads,
                     (-2.8, -4.3, (n - 1) * 5.08 + 2.8, 4.3), tht=True)


class TwoSided(Board):
    """A board whose footprints may sit on the back (fp.back) or be locked (fp.locked), with rule
    areas per layer (keepouts: points relative to the board, layers, what they bar)."""

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.keepouts = []
        self.parked = 0

    def park(self, fp):
        """A movable part in the rows below the board, where a fresh import puts it."""
        x = 2 + (self.parked % 12) * 6.5
        y = self.h + 8 + (self.parked // 12) * 6.5
        self.parked += 1
        return self.add(fp, x, y)

    def pcb(self):
        text = super().pcb()
        u = self.uid
        extra = []
        for pts, layers, bars in self.keepouts:
            xy = " ".join(f"(xy {fmt(ORIGIN[0] + x)} {fmt(ORIGIN[1] + y)})" for x, y in pts)
            ls = " ".join(f'"{l}"' for l in layers)
            keep = " ".join(f"({k} {'not_allowed' if k in bars else 'allowed'})"
                            for k in ("tracks", "vias", "pads", "copperpour", "footprints"))
            extra.append(f'\t(zone\n\t\t(net 0)\n\t\t(net_name "")\n\t\t(layers {ls})\n\t\t(uuid "{u()}")'
                         f'\n\t\t(name "keepout")\n\t\t(hatch edge 0.5)\n\t\t(connect_pads\n\t\t\t(clearance 0)\n\t\t)'
                         f"\n\t\t(min_thickness 0.25)\n\t\t(keepout {keep})\n\t\t(fill\n\t\t\t(thermal_gap 0.5)"
                         f"\n\t\t\t(thermal_bridge_width 0.5)\n\t\t)\n\t\t(polygon\n\t\t\t(pts\n\t\t\t\t{xy}\n\t\t\t)\n\t\t)\n\t)")
        i = text.rstrip().rfind(")")
        return text[:i] + "\n".join(extra) + ("\n" if extra else "") + ")\n"

    def _footprint(self, fp, w):
        back, locked = getattr(fp, "back", False), getattr(fp, "locked", False)
        if not back and not locked:
            return super()._footprint(fp, w)
        if back:
            # KiCad's flip: the footprint frame mirrored at its x axis, layers to the back
            assert not fp.rot, "back footprints are written unrotated"
            saved = ([p.at for p in fp.pads], fp.crtyd)
            for p in fp.pads:
                p.at = (p.at[0], -p.at[1])
            x0, y0, x1, y1 = fp.crtyd
            fp.crtyd = (x0, -y1, x1, -y0)
        lines = []
        super()._footprint(fp, lines.append)
        s = "\n".join(lines)
        if back:
            for p, at in zip(fp.pads, saved[0]):
                p.at = at
            fp.crtyd = saved[1]
            for f, k in (("Cu", "Cu"), ("Mask", "Mask"), ("Paste", "Paste"), ("Fab", "Fab"), ("CrtYd", "CrtYd")):
                s = s.replace(f'"F.{f}"', f'"B.{k}"')
            s = s.replace("\t\t\t\t)\n\t\t\t)\n\t\t)", "\t\t\t\t)\n\t\t\t\t(justify mirror)\n\t\t\t)\n\t\t)")
        if locked:
            s = s.replace('\t\t(layer "B.Cu")\n' if back else '\t\t(layer "F.Cu")\n',
                          ('\t\t(layer "B.Cu")\n' if back else '\t\t(layer "F.Cu")\n') + "\t\t(locked yes)\n", 1)
        w(s)

    def write(self, outdir):
        nets = self.nets()
        with open(os.path.join(outdir, f"{self.name}.kicad_pcb"), "w") as f:
            f.write(self.pcb())
        with open(os.path.join(outdir, f"{self.name}.kicad_pro"), "w") as f:
            import json
            json.dump(self.pro(), f, indent=2, sort_keys=True)
            f.write("\n")
        movable = sum(court(fp) for fp in self.fps if not stays(fp))
        print(f"{self.name:12s} {len(self.fps):3d} parts {len(nets):4d} nets, board {self.w * self.h:.0f} mm2, "
              f"movable courtyards {movable:.0f} mm2")


def stays(fp):
    return getattr(fp, "locked", False) or fp.ref[0] in "JH" or fp.ref.startswith("BT")


def court(fp):
    x0, y0, x1, y1 = fp.crtyd
    return (x1 - x0) * (y1 - y0)


def decouple(b, ic, caps, rail, gnd, size="0402", prefix="C", start=1, value="100n"):
    out = []
    for i in range(caps):
        c = b.park(chip(prefix, f"{prefix}{start + i}", size, value))
        c.pad(1).net, c.pad(2).net = rail, gnd
        out.append(c)
    return out


def bga_power(u, rails, gnd, share=0.25):
    """Every 4th ball ground, a share of the rest on the rails in rings, the rest stays free."""
    for k, p in enumerate(u.pads):
        if k % 4 == 0:
            p.net = gnd
        elif k % 4 == 2 and rails:
            p.net = rails[(k // 4) % len(rails)]


# ---------------------------------------------------------------- boards


def board_som():
    rng = random.Random(201)
    b = TwoSided("ts_som", 42, 30, inner="power", title="4 layers, a small system on module, B2B on the back")
    b.zones = [("GND", "In1.Cu"), ("+1V8", "In2.Cu")]
    gnd = "GND"
    # what stays: the board to board connector on the back, four mounting holes
    j1 = b.add(b2b("J1", 80), 21, 25)
    j1.back = True
    for i, (x, y) in enumerate(((3.6, 3.6), (38.4, 3.6), (3.6, 26.4), (38.4, 26.4))):
        b.add(mounting_hole(f"H{i + 1}"), x, y)
    j2 = b.add(gb.fpc("J2", 24, 0.5), 21, 3.4, 180)
    # chips
    soc = b.park(gb.bga("U1", 14, 0.8, 0.4, "SoC"))
    ram = b.park(gb.bga("U2", 10, 0.8, 0.4, "LPDDR"))
    emmc = b.park(gb.bga("U3", 8, 0.8, 0.4, "eMMC"))
    pmic = b.park(gb.qfn("U4", 12, 0.5, 3.4375, 0.875, 0.25, 5.15, "PMIC"))
    phy = b.park(gb.qfn("U5", 8, 0.5, 2.4375, 0.875, 0.25, 3.3, "PHY"))
    hub = b.park(gb.qfn("U6", 8, 0.5, 2.4375, 0.875, 0.25, 3.3, "HUB"))
    rails = ["VDD_CORE", "VDD_DDR", "+3V3", "+1V8"]
    bga_power(soc, rails, gnd)
    bga_power(ram, ["VDD_DDR", "+1V8"], gnd)
    bga_power(emmc, ["+3V3", "+1V8"], gnd)
    for u, rs in ((pmic, ["VIN", "+3V3"]), (phy, ["+3V3", "+1V8"]), (hub, ["+3V3"])):
        u.pads[-1].net = gnd
        for k, r in enumerate(rs):
            u.pads[3 + 7 * k].net = r
    # buses: SoC to memory, eMMC, PHY, hub, and the B2B connector
    free = lambda u: [p for p in u.pads if p.net is None]
    for u, n, name in ((ram, 30, "DQ"), (emmc, 10, "MMC"), (phy, 8, "RGMII"), (hub, 6, "ULPI")):
        a, c = free(soc), free(u)
        rng.shuffle(a)
        for k in range(n):
            a[k].net = c[k].net = f"{name}{k}"
    jp = [p for p in j1.pads if not p.num.startswith("MP")]
    for k, p in enumerate(jp):
        if k % 5 == 0:
            p.net = gnd
        elif k % 7 == 1:
            p.net = "VIN"
    sigs = [p for p in jp if p.net is None]
    targets = free(soc)[:20] + free(phy)[:6] + free(hub)[:8] + free(pmic)[:4]
    for k, (p, q) in enumerate(zip(sigs, targets)):
        p.net = q.net = f"B2B{k}"
    fp_sigs = [p for p in j2.pads if not p.num.startswith("MP")]
    for k, p in enumerate(fp_sigs):
        if k % 6 == 0:
            p.net = gnd
        else:
            q = free(soc)[0] if free(soc) else None
            if q:
                p.net = q.net = f"CSI{k}"
    # PMIC: four bucks with inductor and output capacitors, input capacitors
    sw_pins = [1, 13, 25, 37]
    outs = ["VDD_CORE", "VDD_DDR", "+3V3", "+1V8"]
    k = 1
    for pin, out in zip(sw_pins, outs):
        sw = f"SW_{out.strip('+')}"
        pmic.pad(pin).net = pmic.pad(pin + 1).net = sw
        l = b.park(power_inductor(f"L{k}"))
        l.pad(1).net, l.pad(2).net = sw, out
        for c in decouple(b, pmic, 2, out, gnd, "0805", "C", 200 + 2 * k, "22u"):
            pass
        k += 1
    decouple(b, pmic, 3, "VIN", gnd, "0805", "C", 250, "10u")
    # decoupling: many small capacitors on the rails of the chips
    n = 1
    for rail, count in (("VDD_CORE", 30), ("VDD_DDR", 22), ("+1V8", 22), ("+3V3", 24)):
        decouple(b, None, count, rail, gnd, "0402", "C", n)
        n += count
    # resistors: pull-ups and straps on free chip pins, series resistors on the bus to the PHY
    rs = []
    for i in range(36):
        r = b.park(chip("R", f"R{i + 1}", "0402", "10k"))
        rs.append(r)
    pins = free(soc) + free(pmic) + free(phy) + free(hub) + free(emmc)
    rng.shuffle(pins)
    for i, r in enumerate(rs):
        if i < len(pins):
            pins[i].net = r.pad(1).net = f"STRAP{i}"
            r.pad(2).net = "+1V8" if i % 3 else gnd
    # a few transistors and level shifters
    for i in range(6):
        q = b.park(gb.sot23(f"Q{i + 1}"))
        q.pad(1).net, q.pad(2).net, q.pad(3).net = f"EN{i}", gnd, f"LS{i}"
        p = free(soc)
        if p:
            p[0].net = f"EN{i}"
    for i in range(2):
        u = b.park(gb.soic(f"U{7 + i}", 8, "EEPROM"))
        u.pad(4).net, u.pad(8).net = gnd, "+3V3"
        for k, q in zip((5, 6), free(soc)[:2]):
            q.net = u.pad(k).net = f"I2C{i}_{k}"
    b.net_class("Power", 0.4, 0.15, 0.6, 0.3, ["VIN", "SW_*", "VDD_CORE"])
    return b


def board_sensor():
    rng = random.Random(202)
    b = TwoSided("ts_sensor", 34, 23, title="2 layers, radio sensor, coin cell holder on the back, antenna keepout")
    gnd = "GND"
    bt = b.add(coin_holder("BT1"), 17, 11.5)
    bt.back = True
    bt.locked = True
    bt.pad(1).net = bt.pad(2).net = "VBAT"
    bt.pad(3).net = gnd
    j1 = b.add(usb_micro("J1"), 8, 20.6, 180)
    for k, n in enumerate(("VBUS", "USB5", "USB6", "ID", gnd)):
        j1.pads[k].net = n
    for p in j1.pads[5:]:
        p.net = gnd
    # antenna at the top right: no parts on the front, copper free on both sides
    b.keepouts.append((((24, 0.5), (33.5, 0.5), (33.5, 8), (24, 8)), ["F.Cu"], ("footprints", "tracks", "vias",
                                                                             "copperpour", "pads")))
    sw = b.add(gb.two_terminal("R", "SW1", "BTN"), 30, 20.5)  # a button as a small mechanical part
    sw.pad(1).net, sw.pad(2).net = "BTN", gnd
    mcu = b.park(gb.qfn("U1", 8, 0.5, 2.4375, 0.875, 0.25, 3.3, "Radio"))
    mcu.pads[-1].net = gnd
    for n in (1, 9, 25):
        mcu.pad(n).net = "+3V3"
    y = b.park(crystal("Y1"))
    y.pad(1).net, y.pad(3).net = "XI", "XO"
    y.pad(2).net = y.pad(4).net = gnd
    mcu.pad(5).net, mcu.pad(6).net = "XI", "XO"
    for i, n in enumerate(("XI", "XO")):
        c = b.park(chip("C", f"C{i + 1}", "0402", "12p"))
        c.pad(1).net, c.pad(2).net = n, gnd
    # radio match: series and shunt parts from the RF pin towards the antenna corner
    mcu.pad(17).net = "RF0"
    for i in range(4):
        s = b.park(chip("L" if i % 2 == 0 else "C", f"{'L' if i % 2 == 0 else 'C'}{10 + i}", "0402", "2n2"))
        s.pad(1).net, s.pad(2).net = f"RF{i}", f"RF{i + 1}"
        sh = b.park(chip("C", f"C{20 + i}", "0402", "1p"))
        sh.pad(1).net, sh.pad(2).net = f"RF{i + 1}", gnd
    # regulator from VBUS or VBAT to 3V3 with its capacitors, a diode or-ing
    reg = b.park(gb.sot23("U2", "LDO"))
    reg.pad(1).net, reg.pad(2).net, reg.pad(3).net = gnd, "VIN", "+3V3"
    for i, (n, net) in enumerate((("D1", "VBUS"), ("D2", "VBAT"))):
        d = b.park(chip("D", n, "0603", "BAT54"))
        d.pad(1).net, d.pad(2).net = net, "VIN"
    caps = [("VIN", "1u"), ("VIN", "1u"), ("+3V3", "1u"), ("+3V3", "10u")] + [("+3V3", "100n")] * 6
    for i, (n, v) in enumerate(caps):
        c = b.park(chip("C", f"C{30 + i}", "0603" if v == "10u" else "0402", v))
        c.pad(1).net, c.pad(2).net = n, gnd
    # sensors: a pressure sensor and an accelerometer (small QFN), a SOIC flash, with their caps
    sensors = [b.park(gb.qfn("U3", 3, 0.5, 1.2, 0.6, 0.25, 0.8, "BME")),
               b.park(gb.qfn("U4", 4, 0.5, 1.45, 0.6, 0.25, 1.1, "ACC")),
               b.park(gb.soic("U5", 8, "Flash"))]
    for s in sensors:
        s.pads[-1].net = gnd if s.ref != "U5" else s.pads[-1].net
    sensors[2].pad(4).net, sensors[2].pad(8).net = gnd, "+3V3"
    for s in sensors[:2]:
        s.pads[0].net = "+3V3"
    free = lambda u: [p for p in u.pads if p.net is None]
    for k, (a, c) in enumerate(zip(free(mcu), free(sensors[0])[:2] + free(sensors[1])[:4] + free(sensors[2])[:4])):
        a.net = c.net = f"BUS{k}"
    for i in range(3):
        c = b.park(chip("C", f"C{50 + i}", "0402", "100n"))
        c.pad(1).net, c.pad(2).net = "+3V3", gnd
    # USB data with series resistors and ESD, LEDs with resistors, pull-ups, a divider for VBAT
    for k, n in enumerate(("USB5", "USB6")):
        r = b.park(chip("R", f"R{k + 1}", "0402", "22"))
        r.pad(1).net, r.pad(2).net = n, f"D{k}"
        free(mcu)[0].net = f"D{k}"
    esd = b.park(gb.sot23("U6", "ESD"))
    esd.pad(1).net, esd.pad(2).net, esd.pad(3).net = "USB5", "USB6", gnd
    for i in range(3):
        led = b.park(chip("D", f"D{10 + i}", "0603", "LED"))
        r = b.park(chip("R", f"R{10 + i}", "0402", "1k"))
        led.pad(2).net, led.pad(1).net, r.pad(1).net = gnd, f"LED{i}", f"LED{i}"
        r.pad(2).net = f"LEDC{i}"
        free(mcu)[0].net = f"LEDC{i}"
    for i in range(6):
        r = b.park(chip("R", f"R{20 + i}", "0402", "10k"))
        r.pad(1).net = "+3V3"
        p = free(mcu)
        r.pad(2).net = p[0].net = f"PU{i}" if p else gnd
    for i, (a, c) in enumerate((("VBAT", "VDIV"), ("VDIV", gnd))):
        r = b.park(chip("R", f"R{30 + i}", "0402", "1M"))
        r.pad(1).net, r.pad(2).net = a, c
    p = free(mcu)
    if p:
        p[0].net = "VDIV"
    mcu_btn = free(mcu)
    if mcu_btn:
        mcu_btn[0].net = "BTN"
    # a display driver with its charge pump capacitors and an FPC to the display
    drv = b.park(gb.qfn("U7", 6, 0.5, 1.95, 0.75, 0.25, 2.5, "EPD"))
    drv.pads[-1].net = gnd
    drv.pad(1).net = "+3V3"
    for i in range(10):
        c = b.park(chip("C", f"C{60 + i}", "0402", "1u"))
        p = [q for q in drv.pads if q.net is None]
        c.pad(1).net, c.pad(2).net = (p[0].net if p and p[0].net else None), gnd
        if p:
            p[0].net = c.pad(1).net = f"CP{i}"
    for k, (a, c) in enumerate(zip(free(mcu), free(drv))):
        a.net = c.net = f"SPI{k}"
    for i in range(6):
        r = b.park(chip("R", f"R{40 + i}", "0402", "47k"))
        p = free(drv)
        r.pad(1).net = "+3V3"
        r.pad(2).net = p[0].net = f"DP{i}" if p else gnd
    rng.random()
    return b


def board_roomy():
    rng = random.Random(203)
    b = TwoSided("ts_roomy", 60, 45, title="2 layers, a control: about half the front is enough")
    gnd, vcc = "GND", "+3V3"
    j1 = b.add(gb.header("J1", 6), 3, 22, 90)
    for k, p in enumerate(j1.pads):
        p.net = (vcc, gnd, "TX", "RX", "SWD", "SWC")[k]
    for i, (x, y) in enumerate(((4, 4), (56, 4), (4, 41), (56, 41))):
        b.add(mounting_hole(f"H{i + 1}"), x, y)
    mcu = b.park(gb.qfn("U1", 12, 0.5, 3.4375, 0.875, 0.25, 5.15))
    mcu.pads[-1].net = gnd
    for n in (6, 18, 30, 42):
        mcu.pad(n).net = vcc
        mcu.pad(n + 1).net = gnd
    flash = b.park(gb.soic("U2", 8, "Flash"))
    sens = b.park(gb.soic("U3", 8, "Sensor"))
    for u in (flash, sens):
        u.pad(4).net, u.pad(8).net = gnd, vcc
    free = lambda u: [p for p in u.pads if p.net is None]
    for k, (a, c) in enumerate(zip(free(mcu), free(flash) + free(sens))):
        a.net = c.net = f"B{k}"
    for k, n in enumerate(("TX", "RX", "SWD", "SWC")):
        free(mcu)[0].net = n
    for i in range(8):
        c = b.park(gb.two_terminal("C", f"C{i + 1}", "100n"))
        c.pad(1).net, c.pad(2).net = vcc, gnd
    loc = [b.park(gb.two_terminal("R", f"R{i + 1}")) for i in range(16)]
    gb.random_nets(rng, gb.all_pads(loc) + [(mcu, p) for p in free(mcu)], "N", near=6, sizes=(2, 2, 3))
    reg = b.park(gb.sot23("U4", "LDO"))
    reg.pad(1).net, reg.pad(2).net, reg.pad(3).net = gnd, "VIN", vcc
    j1.pads[0].net = "VIN"
    for i, n in enumerate(("VIN", vcc)):
        c = b.park(gb.two_terminal("C", f"C{20 + i}", "10u"))
        c.pad(1).net, c.pad(2).net = n, gnd
    return b


def board_tht():
    rng = random.Random(204)
    b = TwoSided("ts_tht", 50, 32, title="2 layers, power board: through hole electrolytics between SMD converters")
    gnd = "GND"
    t1 = b.add(terminal("J1", 2), 5.0, 24.0, 90)
    t1.pad(1).net, t1.pad(2).net = "VIN", gnd
    t2 = b.add(terminal("J2", 3), 45.0, 24.0, 90)
    for k, n in enumerate(("+5V", "+12V", gnd)):
        t2.pad(k + 1).net = n
    for i, (x, y) in enumerate(((3.6, 3.6), (46.4, 3.6))):
        b.add(mounting_hole(f"H{i + 1}"), x, y)
    k = 1
    for out in ("+5V", "+12V"):
        ic = b.park(gb.soic(f"U{k}", 8, "BUCK"))
        sw = f"SW{k}"
        ic.pad(1).net, ic.pad(2).net, ic.pad(8).net, ic.pad(4).net = "VIN", sw, sw, gnd
        ic.pad(5).net, ic.pad(3).net, ic.pad(6).net = f"FB{k}", f"BST{k}", f"EN{k}"
        l = b.park(power_inductor(f"L{k}", "10u"))
        l.pad(1).net, l.pad(2).net = sw, out
        boot = b.park(chip("C", f"C{10 * k}", "0603", "100n"))
        boot.pad(1).net, boot.pad(2).net = f"BST{k}", sw
        for j, (a, c) in enumerate(((out, f"FB{k}"), (f"FB{k}", gnd))):
            r = b.park(chip("R", f"R{10 * k + j}", "0603", "10k"))
            r.pad(1).net, r.pad(2).net = a, c
        en = b.park(chip("R", f"R{10 * k + 5}", "0603", "100k"))
        en.pad(1).net, en.pad(2).net = "VIN", f"EN{k}"
        for j in range(3):
            c = b.park(chip("C", f"C{10 * k + 1 + j}", "1210", "22u"))
            c.pad(1).net, c.pad(2).net = out if j else "VIN", gnd
        for j, d in enumerate((10, 8)):
            e = b.park(radial(f"C{10 * k + 5 + j}", d, "220u"))
            e.pad(1).net, e.pad(2).net = (out if j else "VIN"), gnd
        k += 1
    for i in range(10):
        c = b.park(chip("C", f"C{40 + i}", "0603", "100n"))
        c.pad(1).net, c.pad(2).net = rng.choice(("VIN", "+5V", "+12V")), gnd
    b.net_class("Power", 0.8, 0.2, 0.8, 0.4, ["VIN", "+5V", "+12V", "SW*", "GND"])
    return b


BOARDS = [board_som, board_sensor, board_roomy, board_tht]


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    outdir = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "boards", "two_sided")
    os.makedirs(outdir, exist_ok=True)
    for make in BOARDS:
        make().write(outdir)


if __name__ == "__main__":
    main()
