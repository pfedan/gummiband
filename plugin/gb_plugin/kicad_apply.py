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
    moved: int = 0
    missing: Sequence[str] = ()


def _kind(item) -> str:
    if isinstance(item, ArcTrack):
        return "arc"
    if isinstance(item, Via):
        return "via"
    return "segment"


def _chunks(seq: Sequence, n: int):
    for i in range(0, len(seq), n):
        yield seq[i:i + n]


def move_footprints(board, poses: Sequence) -> tuple:
    """Set the poses ``(reference, x, y[, rotation])`` on the footprints of the board, which
    still has to take them with ``update_items``. Returns those footprints and the references
    not found."""
    by_ref = {fp.reference_field.text.value: fp for fp in board.get_footprints()}
    changed = []
    missing = []
    for ref, x, y, *rot in poses:
        fp = by_ref.get(ref)
        if fp is None:
            missing.append(ref)
            continue
        fp.position = vec((x, y))
        if rot:
            fp.orientation = Angle.from_degrees(rot[0])
        changed.append(fp)
    return changed, missing


def apply_plan(board, plan: bf.ImportPlan, poses: Sequence = (),
               zones: Sequence[dict] = ()) -> ApplyResult:
    """Remove the replaced tracks/vias of the open board and create the routed ones, all in one
    commit (one undo step), together with the footprints the router moved to ``poses`` and the
    keepouts that followed them (``zones``). ``board`` is a ``kipy.board.Board``."""
    resolve = NetResolver({n.name: n for n in board.get_nets()})
    new_items = build_items(plan, resolve)

    old = list(board.get_tracks()) + list(board.get_vias())
    if plan.add_only:
        ids = {it.uuid for it in plan.remove}
        remove = [it for it in old if it.id.value in ids]
    else:
        remove = [it for it in old if bf.should_remove(bool(it.locked), plan.keep_locked)]

    moved, missing = move_footprints(board, poses) if poses else ([], [])
    followers = reshape_zones(board, zones)
    commit = board.begin_commit()
    created: List[object] = []
    try:
        for chunk in _chunks(moved + followers, CHUNK):
            board.update_items(list(chunk))
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
        moved=len(moved),
        missing=missing,
    )



@dataclass
class PlaceResult:
    moved: int
    zones: int
    missing: List[str]
    flipped: int = 0


def other_side(layer: int) -> int:
    """The layer on the other side of the board (``F.Cu`` for ``B.Cu`` ...); inner and board wide
    layers stay."""
    name = BoardLayer.Name(layer)
    for a, b in (("BL_F_", "BL_B_"), ("BL_B_", "BL_F_")):
        if name.startswith(a):
            return BoardLayer.Value(b + name[len(a):])
    return layer


def flip_footprints(board, refs: Sequence[str]) -> tuple:
    """Put the footprints ``refs`` on the other side of the board, mirrored as KiCad's flip
    does (their pose is set afterwards). Returns how many it flipped and the references not
    found."""
    if not refs:
        return 0, []
    if not hasattr(board, "flip_items"):
        raise RuntimeError("placing on both sides needs kicad-python 0.8 and KiCad 10.0.6 or newer "
                           "(the API to flip footprints)")
    wanted = set(refs)
    fps = [fp for fp in board.get_footprints() if fp.reference_field.text.value in wanted]
    done = 0
    for chunk in _chunks(fps, CHUNK):
        done += len(board.flip_items(list(chunk)))
    found = {fp.reference_field.text.value for fp in fps}
    return done, sorted(wanted - found)


def set_outline(zone, points: Sequence[bf.Point]) -> None:
    """Replace the outline of a zone."""
    line = PolyLine()
    line.closed = True
    for p in points:
        line.append(PolyLineNode.from_point(vec(p)))
    poly = PolygonWithHoles()
    poly.outline = line
    zone.outline = poly


def reshape_zones(board, zones: Sequence[dict], flipped: Sequence[str] = ()) -> List[object]:
    """Set the outlines of ``zones`` on the zones of the board, which still has to take them with
    ``update_items``; the zones ``flipped`` (ids) also go to the layers of the other side.
    Returns those zones."""
    outlines = {z["id"]: z["points"] for z in zones}
    followers = [z for z in board.get_zones() if z.id.value in outlines] if outlines else []
    to_flip = set(flipped)
    for z in followers:
        set_outline(z, outlines[z.id.value])
        if z.id.value in to_flip:
            z.layers = [other_side(layer) for layer in z.layers]
    return followers


def apply_placement(board, poses: Sequence, zones: Sequence[dict], flipped: Sequence[str] = (),
                    flipped_zones: Sequence[str] = ()) -> PlaceResult:
    """Put the footprints ``flipped`` on the other side of the board, move footprints to the
    poses of ``gummiband place`` and the keepouts of moved parts to their new outlines (and
    sides), all in one commit (one undo step)."""
    commit = board.begin_commit()
    try:
        turned, missing = flip_footprints(board, flipped)
        changed, not_found = move_footprints(board, poses)
        missing += [r for r in not_found if r not in missing]
        moved = len(changed)
        followers = reshape_zones(board, zones, flipped_zones)
        for chunk in _chunks(changed + followers, CHUNK):
            board.update_items(list(chunk))
    except BaseException:
        try:
            board.drop_commit(commit)
        except Exception:
            pass  # report the original error, not the failed rollback
        raise
    board.push_commit(commit, PLACE_MESSAGE)
    return PlaceResult(moved, len(followers), missing, turned)
