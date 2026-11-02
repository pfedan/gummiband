#!/usr/bin/env python3
"""Check a placed board side by side, from the file alone (no KiCad needed).

    check_sides.py BOARD.kicad_pcb [--input ORIGINAL.kicad_pcb]

Per footprint: its side (layer F.Cu or B.Cu), whether it goes through the board (a pad with a
hole), its courtyard (fp_rect or the bounding box of the courtyard lines) in board coordinates
with KiCad's rotation. Reports courtyard overlaps between parts on the same side and, on the
other side of a through hole part, overlaps with its hole pads (a square of the larger pad size
plus 0.25 mm around each, as the placer keeps them), parts whose pads are not on the copper of their side, back
parts whose texts are not mirrored, and, with --input, how the pads of every footprint that
changed sides moved: a correct flip keeps the pad numbers and mirrors the footprint frame, so
the pad distances within the part stay the same. Prints a summary per side (parts, courtyard
area) and exits with 1 on a problem.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "plugin"))
from gb_plugin import sexpr as sx  # noqa: E402


def rot(p, deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return (p[0] * c + p[1] * s, -p[0] * s + p[1] * c)


def footprints(path):
    with open(path, encoding="utf-8") as f:
        root = sx.parse(f.read())
    out = {}
    for fp in sx.children(root, "footprint"):
        layer = sx.atoms(sx.child(fp, "layer"))[0]
        at = sx.child(fp, "at")
        x, y = float(at[1]), float(at[2])
        r = float(at[3]) if len(at) > 3 and not isinstance(at[3], list) else 0.0
        ref = next(str(p[2]) for p in sx.children(fp, "property") if str(p[1]) == "Reference")
        pads = {}
        holes = []
        through = False
        for p in sx.children(fp, "pad"):
            pa = sx.child(p, "at")
            local = (float(pa[1]), float(pa[2]))
            d = rot(local, r)
            layers = [str(v) for v in sx.atoms(sx.child(p, "layers"))]
            hole = str(p[2]) in ("thru_hole", "np_thru_hole")
            through |= hole
            pads[str(p[1])] = ((x + d[0], y + d[1]), layers, str(p[2]))
            if hole:
                sz = sx.child(p, "size")
                h = max(float(v) for v in sz[1:3]) / 2 + 0.25 if sz else 0.75
                holes.append((x + d[0] - h, y + d[1] - h, x + d[0] + h, y + d[1] + h))
        xs, ys = [], []
        for g in fp[1:]:
            if not isinstance(g, list) or sx.head(g) not in ("fp_rect", "fp_line", "fp_poly"):
                continue
            gl = sx.child(g, "layer")
            if not gl or not str(gl[1]).endswith("CrtYd"):
                continue
            pts = []
            for k in ("start", "end"):
                c = sx.child(g, k)
                if c:
                    pts.append((float(c[1]), float(c[2])))
            if sx.head(g) == "fp_rect" and len(pts) == 2:
                pts += [(pts[0][0], pts[1][1]), (pts[1][0], pts[0][1])]
            pl = sx.child(g, "pts")
            if pl:
                pts += [(float(q[1]), float(q[2])) for q in sx.children(pl, "xy")]
            for q in pts:
                d = rot(q, r)
                xs.append(x + d[0])
                ys.append(y + d[1])
            crt_layer = str(gl[1])
        box = (min(xs), min(ys), max(xs), max(ys)) if xs else None
        mirrored = []
        for prop in sx.children(fp, "property"):
            eff = sx.child(prop, "effects")
            j = sx.child(eff, "justify") if eff else None
            mirrored.append(bool(j) and "mirror" in [str(v) for v in j[1:]])
        out[ref] = dict(layer=layer, at=(x, y), rot=r, pads=pads, through=through, box=box, holes=holes,
                        crtyd=crt_layer if xs else None, mirrored=mirrored)
    return out


def main():
    args = sys.argv[1:]
    original = None
    if "--input" in args:
        i = args.index("--input")
        original = args[i + 1]
        del args[i:i + 2]
    fps = footprints(args[0])
    bad = []
    refs = sorted(fps)
    for a_i, a in enumerate(refs):
        fa = fps[a]
        if not fa["box"]:
            continue
        for b in refs[a_i + 1:]:
            fb = fps[b]
            if not fb["box"]:
                continue
            # rotated courtyards are boxes here; only report real overlaps of 90 degree parts
            if not all(abs(f["rot"] % 90) < 1e-6 for f in (fa, fb)):
                continue
            if fa["layer"] == fb["layer"]:
                pairs = [(fa["box"], fb["box"], "courtyards")]
            else:
                pairs = [(hb, fb["box"], f"a hole of {a} and the courtyard") for hb in fa["holes"]]
                pairs += [(fa["box"], hb, f"the courtyard and a hole of {b}") for hb in fb["holes"]]
            for p, q, what in pairs:
                w = min(p[2], q[2]) - max(p[0], q[0])
                h = min(p[3], q[3]) - max(p[1], q[1])
                if w > 1e-6 and h > 1e-6:
                    bad.append(f"{what} of {a} and {b} overlap ({w:.2f} x {h:.2f} mm, {fa['layer']}/{fb['layer']})")
                    break
    side = {"F.Cu": "F.", "B.Cu": "B."}
    for ref, f in fps.items():
        s = side[f["layer"]]
        for num, (_, layers, kind) in f["pads"].items():
            if kind == "smd" and not any(l.startswith(s) for l in layers):
                bad.append(f"{ref}.{num}: SMD pad on {layers} but the part is on {f['layer']}")
        if f["crtyd"] and not f["crtyd"].startswith(s):
            bad.append(f"{ref}: courtyard on {f['crtyd']} but the part is on {f['layer']}")
        if f["layer"] == "B.Cu" and not all(f["mirrored"]):
            bad.append(f"{ref}: text on the back not mirrored")
    flipped = 0
    if original:
        before = footprints(original)
        for ref, f in fps.items():
            g = before.get(ref)
            if not g or g["layer"] == f["layer"]:
                continue
            flipped += 1
            nums = sorted(set(f["pads"]) & set(g["pads"]))
            for i, m in enumerate(nums):
                for n in nums[i + 1:]:
                    d0 = math.dist(g["pads"][m][0], g["pads"][n][0])
                    d1 = math.dist(f["pads"][m][0], f["pads"][n][0])
                    if abs(d0 - d1) > 1e-3:
                        bad.append(f"{ref}: pads {m}-{n} {d0:.3f} mm apart before, {d1:.3f} after")
            # mirrored: the turn from pad 1 to pad 2 to pad 3 changes its sense
            if len(nums) >= 3:
                def turn(fp_):
                    a, b, c = (fp_["pads"][k][0] for k in nums[:3])
                    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
                t0, t1 = turn(g), turn(f)
                if abs(t0) > 1e-6 and t0 * t1 > 0:
                    bad.append(f"{ref}: changed sides but its pads are not mirrored")
    for s in ("F.Cu", "B.Cu"):
        on = [f for f in fps.values() if f["layer"] == s]
        area = sum((f["box"][2] - f["box"][0]) * (f["box"][3] - f["box"][1]) for f in on if f["box"])
        print(f"{s}: {len(on)} parts, courtyards {area:.0f} mm2")
    print(f"through the board: {sum(f['through'] for f in fps.values())} parts" +
          (f"; changed sides: {flipped}" if original else ""))
    for b in bad:
        print("PROBLEM:", b)
    print("ok" if not bad else f"{len(bad)} problems")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
