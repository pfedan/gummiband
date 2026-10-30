"""Copper items (segments, arcs, vias) of a .kicad_pcb file and the import plan.

Pure Python: used by the KiCad action and by ``dev_run.py`` without KiCad.

The plan mirrors ``gummiband route``: without ``--keep-locked`` every top level track, arc and
via of the board is replaced; with it, locked ones stay and only unlocked ones are replaced.
The items to create are those of the routed file that are not in the original file (matched by
UUID, so locked copper copied through by the CLI is not created twice).
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Tuple

from . import sexpr

Point = Tuple[float, float]

COPPER_HEADS = ("segment", "arc", "via")


@dataclass(frozen=True)
class Segment:
    start: Point
    end: Point
    width: float
    layer: str
    net: str
    locked: bool = False
    uuid: Optional[str] = None
    kind: str = field(default="segment", init=False)

    def key(self) -> object:
        return self.uuid or (self.kind, self.start, self.end, self.width, self.layer, self.net)


@dataclass(frozen=True)
class Arc:
    start: Point
    mid: Point
    end: Point
    width: float
    layer: str
    net: str
    locked: bool = False
    uuid: Optional[str] = None
    kind: str = field(default="arc", init=False)

    def key(self) -> object:
        return self.uuid or (self.kind, self.start, self.mid, self.end, self.width, self.layer,
                             self.net)


@dataclass(frozen=True)
class Via:
    at: Point
    size: float
    drill: float
    layers: Tuple[str, str]
    net: str
    locked: bool = False
    uuid: Optional[str] = None
    kind: str = field(default="via", init=False)

    def key(self) -> object:
        return self.uuid or (self.kind, self.at, self.size, self.drill, self.layers, self.net)

    @property
    def is_through(self) -> bool:
        return self.layers == ("F.Cu", "B.Cu") or self.layers == ("B.Cu", "F.Cu")


CopperItem = object  # Segment | Arc | Via (kept loose for Python 3.9)


class BoardFileError(ValueError):
    pass


@dataclass
class BoardCopper:
    items: List[CopperItem]
    # KiCad 9 and older: net number -> name from the top level net table. Empty for KiCad 10.
    net_table: Dict[int, str]
    warnings: List[str]


def _uuid(node: sexpr.Node) -> Optional[str]:
    for name in ("uuid", "tstamp"):
        c = sexpr.child(node, name)
        if c is not None:
            args = sexpr.atoms(c)
            if args:
                return str(args[0])
    return None


def _net(node: sexpr.Node, table: Dict[int, str], warnings: List[str]) -> str:
    c = sexpr.child(node, "net")
    if c is None:
        return ""
    args = sexpr.atoms(c)
    if not args:
        return ""
    ref = args[0]
    if isinstance(ref, sexpr.QStr):
        return str(ref)  # KiCad 10: (net "name")
    try:
        code = int(ref)
    except ValueError:
        return str(ref)
    if code == 0:
        return ""
    if code not in table:
        warnings.append(f"net number {code} not in the net table")
        return ""
    return table[code]


def _point(node: sexpr.Node, name: str) -> Point:
    p = sexpr.floats(sexpr.child(node, name), 2)
    if p is None:
        raise BoardFileError(f"{node[0]} without ({name} x y): {_uuid(node)}")
    return (p[0], p[1])


def _scalar(node: sexpr.Node, name: str) -> float:
    v = sexpr.floats(sexpr.child(node, name), 1)
    if v is None:
        raise BoardFileError(f"{node[0]} without ({name} ...): {_uuid(node)}")
    return v[0]


def _layer(node: sexpr.Node) -> str:
    c = sexpr.child(node, "layer")
    args = sexpr.atoms(c) if c is not None else []
    if not args:
        raise BoardFileError(f"{node[0]} without (layer ...): {_uuid(node)}")
    return str(args[0])


def item_from_node(node: sexpr.Node, net_table: Dict[int, str],
                   warnings: List[str]) -> CopperItem:
    kind = node[0]
    net = _net(node, net_table, warnings)
    locked = sexpr.has_flag(node, "locked")
    uuid = _uuid(node)
    if kind == "segment":
        return Segment(_point(node, "start"), _point(node, "end"), _scalar(node, "width"),
                       _layer(node), net, locked, uuid)
    if kind == "arc":
        return Arc(_point(node, "start"), _point(node, "mid"), _point(node, "end"),
                   _scalar(node, "width"), _layer(node), net, locked, uuid)
    if kind == "via":
        c = sexpr.child(node, "layers")
        layers = [str(a) for a in sexpr.atoms(c)] if c is not None else []
        if len(layers) < 2:
            layers = ["F.Cu", "B.Cu"]
        return Via(_point(node, "at"), _scalar(node, "size"), _scalar(node, "drill"),
                   (layers[0], layers[-1]), net, locked, uuid)
    raise BoardFileError(f"not a copper item: {kind}")


def read_copper(text: str) -> BoardCopper:
    """Top level segments, arcs and vias of a board file (footprint internals are ignored)."""
    if not text.lstrip().startswith("(kicad_pcb"):
        raise BoardFileError("not a KiCad board file (expected '(kicad_pcb')")
    net_table: Dict[int, str] = {}
    nodes: List[sexpr.Node] = []
    for node in sexpr.iter_top_level(text, ("net",) + COPPER_HEADS):
        if node[0] == "net":
            args = sexpr.atoms(node)
            if len(args) >= 2:
                try:
                    net_table[int(args[0])] = str(args[1])
                except ValueError:
                    pass
        else:
            nodes.append(node)
    warnings: List[str] = []
    items = [item_from_node(n, net_table, warnings) for n in nodes]
    return BoardCopper(items, net_table, warnings)


def pad_net_names(text: str) -> set:
    """Names of all nets used by footprint pads (for sanity checks; parses the whole file)."""
    root = sexpr.parse(text)
    table: Dict[int, str] = {}
    for n in sexpr.children(root, "net"):
        args = sexpr.atoms(n)
        if len(args) >= 2:
            table[int(args[0])] = str(args[1])
    names = set(table.values())
    warnings: List[str] = []
    for fp in sexpr.children(root, "footprint"):
        for pad in sexpr.children(fp, "pad"):
            name = _net(pad, table, warnings)
            if name:
                names.add(name)
    return names


@dataclass
class ImportPlan:
    keep_locked: bool
    remove: List[CopperItem]  # original items that are replaced
    keep: List[CopperItem]  # original items that stay (locked, with keep_locked)
    create: List[CopperItem]  # routed items to create
    warnings: List[str]
    add_only: bool = False  # nothing on the board is removed

    def summary(self) -> Dict[str, Counter]:
        return {
            "remove": Counter(i.kind for i in self.remove),
            "keep": Counter(i.kind for i in self.keep),
            "create": Counter(i.kind for i in self.create),
        }


def should_remove(locked: bool, keep_locked: bool) -> bool:
    """Same rule as the CLI: unlocked copper always goes, locked copper only without
    --keep-locked."""
    return not (locked and keep_locked)


def plan_import(original: BoardCopper, routed: BoardCopper, keep_locked: bool) -> ImportPlan:
    remove, keep = [], []
    for it in original.items:
        (remove if should_remove(it.locked, keep_locked) else keep).append(it)
    known = {it.key() for it in original.items}
    create = [it for it in routed.items if it.key() not in known]
    warnings = list(original.warnings) + list(routed.warnings)
    # Kept items should be copied through by the CLI; flag it if the routed file lost some.
    routed_keys = {it.key() for it in routed.items}
    missing = [it for it in keep if it.key() not in routed_keys]
    if missing:
        warnings.append(f"{len(missing)} kept (locked) items are missing in the routed file")
    return ImportPlan(keep_locked, remove, keep, create, warnings)


def plan_addition(original: BoardCopper, routed: BoardCopper) -> ImportPlan:
    """What the router added; only tracks that moved or gave way leave the board."""
    known = {it.key() for it in original.items}
    create = [it for it in routed.items if it.key() not in known]
    # tracks that moved or gave way are gone from the routed file
    kept = {it.key() for it in routed.items}
    remove = [it for it in original.items if it.key() not in kept]
    keep = [it for it in original.items if it.key() in kept]
    return ImportPlan(True, remove, keep, create,
                      list(original.warnings) + list(routed.warnings), add_only=True)


def plan_from_texts(original_text: str, routed_text: str, keep_locked: bool) -> ImportPlan:
    return plan_import(read_copper(original_text), read_copper(routed_text), keep_locked)


@dataclass
class PartMoves:
    """Footprints that ``route --move-parts`` shifted."""
    poses: List[Tuple[str, float, float]]  # reference and new position
    stale_copper: int  # kept tracks and vias that moved along with a footprint group


def _reference(fp: sexpr.Node) -> Optional[str]:
    for prop in sexpr.children(fp, "property"):
        words = sexpr.atoms(prop)
        if len(words) >= 2 and words[0] == "Reference":
            return words[1]
    for text in sexpr.children(fp, "fp_text"):
        words = sexpr.atoms(text)
        if len(words) >= 2 and words[0] == "reference":
            return words[1]
    return None


def _positions(text: str) -> Dict[str, Point]:
    out = {}
    for fp in sexpr.iter_top_level(text, ("footprint", "module")):
        ref, at = _reference(fp), sexpr.floats(sexpr.child(fp, "at"), 2)
        if ref and at:
            out[ref] = at
    return out


def plan_moves(original_text: str, routed_text: str) -> PartMoves:
    """Compare the footprints of the routed file with those of the file that was routed."""
    before = _positions(original_text)
    poses = []
    for ref, at in sorted(_positions(routed_text).items()):
        old = before.get(ref)
        if old and max(abs(at[0] - old[0]), abs(at[1] - old[1])) > 1e-6:
            poses.append((ref, at[0], at[1]))
    old_copper = {it.uuid: it for it in read_copper(original_text).items}
    stale = sum(1 for it in read_copper(routed_text).items
                if it.uuid in old_copper and it != old_copper[it.uuid])
    return PartMoves(poses, stale)


def format_counts(c: Counter, kinds: Iterable[str] = COPPER_HEADS) -> str:
    names = {"segment": "tracks", "arc": "arcs", "via": "vias"}
    return ", ".join(f"{c.get(k, 0)} {names[k]}" for k in kinds)
