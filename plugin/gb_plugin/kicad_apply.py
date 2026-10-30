"""Turn planned copper items into KiCad IPC API objects and apply them as one undo step.

Needs the ``kicad-python`` package (module ``kipy``). Building the objects needs no running
KiCad, so ``dev_run.py`` uses ``to_kipy`` to check the conversion headless.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
from typing import List, Mapping, Optional, Sequence

from kipy.board_types import ArcTrack, BoardLayer, Net, Track, Via, ViaType
from kipy.geometry import Angle, PolygonWithHoles, PolyLine, PolyLineNode, Vector2
from kipy.util.board_layer import layer_from_canonical_name

from . import board_file as bf

COMMIT_MESSAGE = "Route with Gummiband"
PLACE_MESSAGE = "Place with Gummiband"
CHUNK = 500  # items per create/delete request, keeps each request well below the timeout


def nm(v: float) -> int:
    return int(round(v * 1_000_000))


def vec(p: bf.Point) -> Vector2:
    return Vector2.from_xy(nm(p[0]), nm(p[1]))


def layer_id(name: str) -> int:
    lid = layer_from_canonical_name(name)
    if lid == BoardLayer.BL_UNKNOWN:
        raise ValueError(f"unknown layer {name!r}")
    return lid


@dataclass
class NetResolver:
    """Net objects by name from the open board; unknown names are counted."""

    nets: Mapping[str, Net]
    unknown: Counter = field(default_factory=Counter)

    def __call__(self, name: str) -> Optional[Net]:
        if not name:
            return None
        n = self.nets.get(name)
        if n is None:
            self.unknown[name] += 1
            n = Net(name=name)
        return n


def to_kipy(item, net: Optional[Net]):
    """One Segment/Arc/Via of the plan as a new (unlocked) kipy board item."""
    if isinstance(item, bf.Segment):
        t = Track()
        t.start = vec(item.start)
        t.end = vec(item.end)
        t.width = nm(item.width)
        t.layer = layer_id(item.layer)
    elif isinstance(item, bf.Arc):
        t = ArcTrack()
        t.start = vec(item.start)
        t.mid = vec(item.mid)
        t.end = vec(item.end)
        t.width = nm(item.width)
        t.layer = layer_id(item.layer)
    elif isinstance(item, bf.Via):
        t = Via()  # through via with a normal padstack by default
        t.position = vec(item.at)
        t.diameter = nm(item.size)
        t.drill_diameter = nm(item.drill)
        if not item.is_through:
            a, b = layer_id(item.layers[0]), layer_id(item.layers[1])
            outer = {BoardLayer.BL_F_Cu, BoardLayer.BL_B_Cu}
            t.type = ViaType.VT_BLIND if (a in outer or b in outer) else ViaType.VT_BURIED
            t.padstack.drill.start_layer = a
            t.padstack.drill.end_layer = b
    else:
        raise TypeError(f"not a copper item: {item!r}")
    t.locked = False
    if net is not None:
        t.net = net
    return t


def build_items(plan: bf.ImportPlan, resolve: NetResolver) -> List[object]:
    return [to_kipy(it, resolve(it.net)) for it in plan.create]


@dataclass
class ApplyResult:
    removed: Counter
    created: Counter
    unknown_nets: Counter


def _kind(item) -> str:
    if isinstance(item, ArcTrack):
        return "arc"
    if isinstance(item, Via):
        return "via"
    return "segment"


def _chunks(seq: Sequence, n: int):
    for i in range(0, len(seq), n):
        yield seq[i:i + n]


def apply_plan(board, plan: bf.ImportPlan) -> ApplyResult:
    """Remove the replaced tracks/vias of the open board and create the routed ones, all in one
    commit (one undo step). ``board`` is a ``kipy.board.Board``."""
    resolve = NetResolver({n.name: n for n in board.get_nets()})
    new_items = build_items(plan, resolve)

    old = list(board.get_tracks()) + list(board.get_vias())
    if plan.add_only:
        ids = {it.uuid for it in plan.remove}
        remove = [it for it in old if it.id.value in ids]
    else:
        remove = [it for it in old if bf.should_remove(bool(it.locked), plan.keep_locked)]

    commit = board.begin_commit()
    created: List[object] = []
    try:
        for chunk in _chunks(remove, CHUNK):
            board.remove_items(list(chunk))
        for chunk in _chunks(new_items, CHUNK):
            created += board.create_items(list(chunk))
    except BaseException:
        try:
            board.drop_commit(commit)
        except Exception:
            pass  # report the original error, not the failed rollback
        raise
    board.push_commit(commit, COMMIT_MESSAGE)
    return ApplyResult(
        removed=Counter(_kind(it) for it in remove),
        created=Counter(_kind(it) for it in created),
        unknown_nets=resolve.unknown,
    )



@dataclass
class PlaceResult:
    moved: int
    zones: int
    missing: List[str]


def set_outline(zone, points: Sequence[bf.Point]) -> None:
    """Replace the outline of a zone."""
    line = PolyLine()
    line.closed = True
    for p in points:
        line.append(PolyLineNode.from_point(vec(p)))
    poly = PolygonWithHoles()
    poly.outline = line
    zone.outline.polygons.clear()
    zone.outline.polygons.append(poly)


def apply_placement(board, poses: Sequence, zones: Sequence[dict]) -> PlaceResult:
    """Move footprints to the poses of ``gummiband place`` and the keepouts of moved parts to
    their new outlines, all in one commit (one undo step)."""
    by_ref = {fp.reference_field.text.value: fp for fp in board.get_footprints()}
    changed = []
    missing = []
    for ref, x, y, rot in poses:
        fp = by_ref.get(ref)
        if fp is None:
            missing.append(ref)
            continue
        fp.position = vec((x, y))
        fp.orientation = Angle.from_degrees(rot)
        changed.append(fp)
    moved = len(changed)
    outlines = {z["id"]: z["points"] for z in zones}
    followers = [z for z in board.get_zones() if z.id.value in outlines] if outlines else []
    for z in followers:
        set_outline(z, outlines[z.id.value])
    commit = board.begin_commit()
    try:
        for chunk in _chunks(changed + followers, CHUNK):
            board.update_items(list(chunk))
    except BaseException:
        try:
            board.drop_commit(commit)
        except Exception:
            pass  # report the original error, not the failed rollback
        raise
    board.push_commit(commit, PLACE_MESSAGE)
    return PlaceResult(moved, len(followers), missing)
