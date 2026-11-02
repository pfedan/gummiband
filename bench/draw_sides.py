#!/usr/bin/env python3
"""Draw a board front and back side by side as SVG (no KiCad needed).

    draw_sides.py BOARD.kicad_pcb OUT.svg [--title TEXT]

Front left, back right, the back seen from the front (not mirrored, so a part on the back sits
under the same spot as on the front). Courtyards of front parts blue, back parts red, through hole
parts grey on their side; on the other side a through hole part shows its outline dashed and the
squares its holes take; pads dark; keepouts hatched; the board outline from Edge.Cuts rectangles.
Parts outside the outline (not placed yet) are left out.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_sides import footprints, rot  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "plugin"))
from gb_plugin import sexpr as sx  # noqa: E402


def outline(path):
    with open(path, encoding="utf-8") as f:
        root = sx.parse(f.read())
    for g in sx.children(root, "gr_rect"):
        if str(sx.child(g, "layer")[1]) == "Edge.Cuts":
            a, b = sx.child(g, "start"), sx.child(g, "end")
            return (float(a[1]), float(a[2]), float(b[1]), float(b[2]))
    raise SystemExit("no Edge.Cuts rectangle")


def keepouts(path):
    with open(path, encoding="utf-8") as f:
        root = sx.parse(f.read())
    out = []
    for z in sx.children(root, "zone"):
        if not sx.child(z, "keepout"):
            continue
        layers = [str(v) for v in sx.atoms(sx.child(z, "layers") or sx.child(z, "layer"))]
        pts = [(float(q[1]), float(q[2])) for q in sx.children(sx.child(sx.child(z, "polygon"), "pts"), "xy")]
        out.append((layers, pts))
    return out


def main():
    args = sys.argv[1:]
    title = ""
    if "--title" in args:
        i = args.index("--title")
        title = args[i + 1]
        del args[i:i + 2]
    path, out = args
    x0, y0, x1, y1 = outline(path)
    w, h = x1 - x0, y1 - y0
    k = 640 / max(w, 1)  # px per mm
    pad = 30
    W = 2 * w * k + 3 * pad
    H = h * k + 2 * pad + 30
    fps = footprints(path)
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W:.0f}" height="{H:.0f}" '
           f'viewBox="0 0 {W:.0f} {H:.0f}" font-family="sans-serif">',
           '<defs><pattern id="hatch" width="6" height="6" patternUnits="userSpaceOnUse" '
           'patternTransform="rotate(45)"><line x1="0" y1="0" x2="0" y2="6" stroke="#b58900" '
           'stroke-width="2"/></pattern></defs>',
           f'<rect width="{W:.0f}" height="{H:.0f}" fill="#fafafa"/>',
           f'<text x="{pad}" y="20" font-size="15" fill="#222">{title}</text>']
    for side, ox in (("F.Cu", pad), ("B.Cu", 2 * pad + w * k)):
        oy = pad + 20
        X = lambda x: ox + (x - x0) * k  # noqa: E731
        Y = lambda y: oy + (y - y0) * k  # noqa: E731
        n = sum(1 for f in fps.values() if f["layer"] == side and f["box"] and x0 <= f["at"][0] <= x1
                and y0 <= f["at"][1] <= y1)
        svg.append(f'<text x="{ox}" y="{oy - 6}" font-size="13" fill="#444">'
                   f'{"front" if side == "F.Cu" else "back (seen from the front)"}: {n} parts</text>')
        svg.append(f'<rect x="{X(x0):.1f}" y="{Y(y0):.1f}" width="{w * k:.1f}" height="{h * k:.1f}" '
                   'fill="#e9f0e4" stroke="#333" stroke-width="1.5"/>')
        for layers, pts in keepouts(path):
            if any(l in (side, "*.Cu", "F&B.Cu") for l in layers):
                d = " ".join(f"{X(x):.1f},{Y(y):.1f}" for x, y in pts)
                svg.append(f'<polygon points="{d}" fill="url(#hatch)" stroke="#b58900"/>')
        for ref, f in sorted(fps.items()):
            if not f["box"] or not (x0 <= f["at"][0] <= x1 and y0 <= f["at"][1] <= y1):
                continue
            mine = f["layer"] == side
            if not (mine or f["through"]):
                continue
            color = "#9e9e9e" if f["through"] else ("#2f6db5" if side == "F.Cu" else "#c0392b")
            bx = f["box"]
            if not mine:
                svg.append(f'<rect x="{X(bx[0]):.1f}" y="{Y(bx[1]):.1f}" width="{(bx[2] - bx[0]) * k:.1f}" '
                           f'height="{(bx[3] - bx[1]) * k:.1f}" fill="none" stroke="{color}" '
                           'stroke-width="1" stroke-dasharray="4 3"/>')
                for hb in f["holes"]:
                    svg.append(f'<rect x="{X(hb[0]):.1f}" y="{Y(hb[1]):.1f}" width="{(hb[2] - hb[0]) * k:.1f}" '
                               f'height="{(hb[3] - hb[1]) * k:.1f}" fill="{color}" fill-opacity="0.35" '
                               f'stroke="{color}" stroke-width="1"/>')
                continue
            svg.append(f'<rect x="{X(bx[0]):.1f}" y="{Y(bx[1]):.1f}" width="{(bx[2] - bx[0]) * k:.1f}" '
                       f'height="{(bx[3] - bx[1]) * k:.1f}" fill="{color}" fill-opacity="0.18" '
                       f'stroke="{color}" stroke-width="1"/>')
            for num, (at, layers, kind) in f["pads"].items():
                if kind == "smd" and not mine:
                    continue
                svg.append(f'<circle cx="{X(at[0]):.1f}" cy="{Y(at[1]):.1f}" r="{max(1.2, 0.18 * k):.1f}" '
                           f'fill="{"#555" if kind != "smd" else color}"/>')
            if (bx[2] - bx[0]) * k > 26:
                cx, cy = (bx[0] + bx[2]) / 2, (bx[1] + bx[3]) / 2
                svg.append(f'<text x="{X(cx):.1f}" y="{Y(cy) + 4:.1f}" font-size="10" text-anchor="middle" '
                           f'fill="#222">{ref}</text>')
    svg.append("</svg>")
    with open(out, "w", encoding="utf-8") as fo:
        fo.write("\n".join(svg))


if __name__ == "__main__":
    main()
