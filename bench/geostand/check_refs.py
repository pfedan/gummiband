#!/usr/bin/env python3
"""Check the boards of the geometry test stand with KiCad's DRC before any routing.
Usage: check_refs.py BOARDS_DIR
For every case and form: the unrouted board must have no copper entry; the reference solution must be
clean and fully connected for loose, tight20, tight and exact, and have at least one copper entry for short.
Writes BOARDS_DIR/refcheck.json ({board: {"ok": bool, ...}}) and prints the failures. Standard library only."""
import collections, json, os, subprocess, sys
COPPER = ("clearance", "copper_edge_clearance", "hole_clearance", "hole_to_hole", "solder_mask_bridge",
          "shorting_items", "tracks_crossing", "padstack_invalid", "copper_sliver")


def drc(path):
    out = path + ".drc.json"
    subprocess.run(["kicad-cli", "pcb", "drc", "--all-track-errors", "--format", "json", "-o", out, path], capture_output=True)
    d = json.load(open(out))
    entries = [v for v in d["violations"] if v["type"] in COPPER]
    return entries, len(d.get("unconnected_items", []))


def main():
    boards = sys.argv[1]
    cases = json.load(open(os.path.join(boards, "cases.json")))
    res = {}
    for c in cases:
        base = os.path.join(boards, c["board"])
        un, _ = drc(base + ".kicad_pcb")
        ref, unconn = drc(base + ".ref.kicad_pcb")
        if c["form"] == "short":
            ok = not un and len(ref) > 0
        else:
            ok = not un and not ref and unconn == 0
        res[c["board"]] = dict(ok=ok, unrouted_entries=len(un), ref_entries=len(ref), ref_unconnected=unconn,
                               ref_kinds=dict(collections.Counter(v["type"] for v in ref)),
                               first=[[i["description"][:50] for i in v["items"]] + [v["description"][-40:]] for v in ref[:2]])
        if not ok:
            print(c["board"], res[c["board"]])
    json.dump(res, open(os.path.join(boards, "refcheck.json"), "w"), indent=1)
    print(f"{sum(r['ok'] for r in res.values())} of {len(res)} boards pass")


if __name__ == "__main__":
    main()
