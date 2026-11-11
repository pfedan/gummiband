#!/usr/bin/env python3
"""Headless dry run of the plugin's import step, without KiCad.

    python3 dev_run.py BOARD.kicad_pcb --routed ROUTED.kicad_pcb [--keep-locked]
    python3 dev_run.py BOARD.kicad_pcb --route [--keep-locked] [--runs N] [--via-cost X]

BOARD is the board as the plugin would hand it to the router. With --route the gummiband
binary is located like in the plugin (GUMMIBAND_BIN, settings.json, PATH) and run first.
Prints what the plugin would remove from and create in the open board. If kicad-python is
importable, every item is also converted to a kipy object (nothing is sent to KiCad).
"""

from __future__ import annotations

import argparse
import sys
import tempfile
import time
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from gb_plugin import board_file as bf  # noqa: E402
from gb_plugin import runner  # noqa: E402


def describe(it) -> str:
    lock = " locked" if it.locked else ""
    if isinstance(it, bf.Segment):
        return (f"segment {it.layer} w={it.width:g} ({it.start[0]:g}, {it.start[1]:g}) -> "
                f"({it.end[0]:g}, {it.end[1]:g}) net={it.net!r}{lock}")
    if isinstance(it, bf.Arc):
        return (f"arc     {it.layer} w={it.width:g} ({it.start[0]:g}, {it.start[1]:g}) via "
                f"({it.mid[0]:g}, {it.mid[1]:g}) -> ({it.end[0]:g}, {it.end[1]:g}) "
                f"net={it.net!r}{lock}")
    return (f"via     {it.layers[0]}-{it.layers[1]} d={it.size:g}/{it.drill:g} "
            f"({it.at[0]:g}, {it.at[1]:g}) net={it.net!r}{lock}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("board", type=Path)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--routed", type=Path, help="routed board written by gummiband route")
    src.add_argument("--route", action="store_true", help="run gummiband route first")
    ap.add_argument("--keep-locked", action="store_true")
    ap.add_argument("--runs", type=int, default=runner.RouteOptions().runs)
    ap.add_argument("--via-cost", type=float, default=1.5)
    ap.add_argument("-v", "--verbose", action="store_true", help="list every item")
    ap.add_argument("--no-net-check", action="store_true",
                    help="skip checking net names against the pads (full parse)")
    args = ap.parse_args(argv)

    routed = args.routed
    if args.route:
        opts = runner.RouteOptions(scope="keep_locked" if args.keep_locked else "fresh",
                                   runs=args.runs, via_cost=args.via_cost)
        work = Path(tempfile.mkdtemp(prefix="gummiband-dev-"))
        routed = work / f"{args.board.stem}-routed.kicad_pcb"
        cmd = runner.build_command(runner.find_binary(), args.board.resolve(), routed, opts)
        print("running:", " ".join(cmd))
        t0 = time.monotonic()
        try:
            res = runner.run_route(cmd, routed, work)
        except runner.RouteFailed as e:
            print(e, file=sys.stderr)
            return 1
        print(f"router: {time.monotonic() - t0:.1f} s, {res.best_line()}, "
              f"{len(res.unrouted())} unrouted listed, {res.violations()} violations")
        print(f"routed file: {routed}")

    original_text = args.board.read_text(encoding="utf-8")
    routed_text = routed.read_text(encoding="utf-8")
    t0 = time.monotonic()
    original = bf.read_copper(original_text)
    routed_copper = bf.read_copper(routed_text)
    plan = bf.plan_import(original, routed_copper, args.keep_locked)
    dt = time.monotonic() - t0

    s = plan.summary()
    print(f"parsed original ({len(original.items)} copper items) and routed "
          f"({len(routed_copper.items)}) in {dt:.2f} s")
    print(f"mode: {'keep locked' if args.keep_locked else 'replace all'} tracks and vias")
    print(f"would remove: {bf.format_counts(s['remove'])}")
    print(f"would keep:   {bf.format_counts(s['keep'])}")
    print(f"would create: {bf.format_counts(s['create'])}")

    by_layer = Counter(getattr(it, "layer", None) or "-".join(it.layers) for it in plan.create)
    print("create per layer:", ", ".join(f"{k} {v}" for k, v in sorted(by_layer.items())))
    widths = Counter(it.width for it in plan.create if not isinstance(it, bf.Via))
    print("track widths:", ", ".join(f"{w:g} mm x{n}" for w, n in sorted(widths.items())))
    vias = Counter((it.size, it.drill) for it in plan.create if isinstance(it, bf.Via))
    print("via sizes:", ", ".join(f"{d:g}/{h:g} x{n}" for (d, h), n in sorted(vias.items())))
    nets = Counter(it.net for it in plan.create)
    print(f"nets: {len(nets)}; most items:",
          ", ".join(f"{n or '<no net>'} {c}" for n, c in nets.most_common(5)))
    if not args.no_net_check:
        pad_nets = bf.pad_net_names(original_text)
        unknown = sorted(n for n in nets if n and n not in pad_nets)
        print(f"net check: {len(nets) - len(unknown)}/{len(nets)} nets exist on pads"
              + (f"; unknown: {', '.join(unknown[:10])}" if unknown else ""))
    for w in plan.warnings:
        print("warning:", w)

    shown = plan.create if args.verbose else plan.create[:3] + [
        it for it in plan.create if isinstance(it, (bf.Arc, bf.Via))][:3]
    print("items to create" + ("" if args.verbose else " (sample)") + ":")
    for it in shown:
        print("  " + describe(it))

    try:
        from gb_plugin import kicad_apply
        from kipy.board_types import Net
    except ImportError:
        print("kicad-python not importable here; skipped the kipy conversion check")
    else:
        resolve = kicad_apply.NetResolver({n: Net(name=n) for n in nets if n})
        t0 = time.monotonic()
        objs = kicad_apply.build_items(plan, resolve)
        print(f"kipy conversion: {len(objs)} objects in {time.monotonic() - t0:.2f} s, "
              f"e.g. {objs[0] if objs else None}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
