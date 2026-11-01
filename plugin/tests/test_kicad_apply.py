"""Conversion to kipy objects and the commit sequence against a fake board.

Needs the kicad-python package (skipped otherwise), but no running KiCad.
"""

import types
import unittest

from gb_plugin import board_file as bf
from tests import fixtures

try:
    from kipy.board_types import ArcTrack, BoardLayer, Net, Track, Via, ViaType

    from gb_plugin import kicad_apply
except ImportError:  # pragma: no cover - depends on the environment
    kicad_apply = None


@unittest.skipIf(kicad_apply is None, "kicad-python (kipy) not installed")
class ToKipyTest(unittest.TestCase):
    def test_segment(self):
        s = bf.Segment((1.5, -2.0), (3.0000004, 4.5), 0.25, "B.Cu", "GND")
        t = kicad_apply.to_kipy(s, Net(name="GND"))
        self.assertIsInstance(t, Track)
        self.assertEqual((t.start.x, t.start.y), (1_500_000, -2_000_000))
        self.assertEqual((t.end.x, t.end.y), (3_000_000, 4_500_000))
        self.assertEqual(t.width, 250_000)
        self.assertEqual(t.layer, BoardLayer.BL_B_Cu)
        self.assertEqual(t.net.name, "GND")
        self.assertFalse(t.locked)

    def test_arc(self):
        a = bf.Arc((0, 0), (0.7071, 0.2929), (1, 1), 0.2, "F.Cu", "")
        t = kicad_apply.to_kipy(a, None)
        self.assertIsInstance(t, ArcTrack)
        self.assertEqual((t.mid.x, t.mid.y), (707_100, 292_900))
        self.assertEqual(t.net.name, "")

    def test_vias(self):
        v = kicad_apply.to_kipy(bf.Via((7, 8), 0.6, 0.3, ("F.Cu", "B.Cu"), "GND"), None)
        self.assertIsInstance(v, Via)
        self.assertEqual(v.type, ViaType.VT_THROUGH)
        self.assertEqual((v.diameter, v.drill_diameter), (600_000, 300_000))
        self.assertEqual((v.position.x, v.position.y), (7_000_000, 8_000_000))
        blind = kicad_apply.to_kipy(bf.Via((0, 0), 0.6, 0.3, ("F.Cu", "In1.Cu"), ""), None)
        self.assertEqual(blind.type, ViaType.VT_BLIND)
        self.assertEqual(blind.padstack.drill.end_layer, BoardLayer.BL_In1_Cu)
        buried = kicad_apply.to_kipy(bf.Via((0, 0), 0.6, 0.3, ("In1.Cu", "In2.Cu"), ""), None)
        self.assertEqual(buried.type, ViaType.VT_BURIED)

    def test_unknown_layer(self):
        with self.assertRaises(ValueError):
            kicad_apply.to_kipy(bf.Segment((0, 0), (1, 1), 0.2, "Nope.Cu", ""), None)


@unittest.skipIf(kicad_apply is None, "kicad-python (kipy) not installed")
class ZoneOutlineTest(unittest.TestCase):
    def points(self, zone):
        return [(n.point.x, n.point.y) for n in zone.outline.outline]

    def test_a_new_outline_replaces_the_old_one(self):
        from kipy.board_types import Zone

        zone = Zone()
        kicad_apply.set_outline(zone, [(0, 0), (1, 0), (1, 1)])
        kicad_apply.set_outline(zone, [(2, 2), (3, 2), (3, 3), (2, 3)])
        self.assertEqual(self.points(zone), [(2_000_000, 2_000_000), (3_000_000, 2_000_000),
                                             (3_000_000, 3_000_000), (2_000_000, 3_000_000)])
        self.assertTrue(zone.outline.outline.closed)
        self.assertEqual(len(zone.proto.outline.polygons), 1)


class FakeBoard:
    """Records the API calls of apply_plan."""

    def __init__(self, existing):
        self.existing = existing
        self.calls = []

    def get_nets(self):
        return [Net(name="GND"), Net(name='/sheet/SIG "A"')]

    def get_tracks(self):
        return [i for i in self.existing if not isinstance(i, Via)]

    def get_vias(self):
        return [i for i in self.existing if isinstance(i, Via)]

    def begin_commit(self):
        self.calls.append(("begin",))
        return "commit-1"

    def remove_items(self, items):
        self.calls.append(("remove", len(items)))

    def create_items(self, items):
        self.calls.append(("create", len(items)))
        return items

    def push_commit(self, commit, message=""):
        self.calls.append(("push", commit, message))

    def drop_commit(self, commit):
        self.calls.append(("drop", commit))

    def get_footprints(self):
        field = types.SimpleNamespace(text=types.SimpleNamespace(value="U1"))
        self.footprint = types.SimpleNamespace(reference_field=field, position=None)
        return [self.footprint]

    def update_items(self, items):
        self.calls.append(("update", len(items)))


@unittest.skipIf(kicad_apply is None, "kicad-python (kipy) not installed")
class ApplyPlanTest(unittest.TestCase):
    def test_moved_parts_share_the_commit(self):
        board = FakeBoard(self.existing())
        plan = bf.plan_from_texts(fixtures.BOARD_V10, fixtures.routed_v10(True), True)
        res = kicad_apply.apply_plan(board, plan, [("U1", 1.5, 2.0), ("U9", 0.0, 0.0)])
        self.assertEqual((res.moved, res.missing), (1, ["U9"]))
        self.assertEqual((board.footprint.position.x, board.footprint.position.y),
                         (1_500_000, 2_000_000))
        kinds = [c[0] for c in board.calls]
        self.assertEqual(kinds[:2], ["begin", "update"])
        self.assertEqual(kinds.count("begin"), 1)
        self.assertEqual(kinds.count("push"), 1)

    def existing(self):
        # the live board as KiCad would report it: same items as BOARD_V10
        c = bf.read_copper(fixtures.BOARD_V10)
        out = []
        for it in c.items:
            obj = kicad_apply.to_kipy(it, None)
            obj.locked = it.locked
            out.append(obj)
        return out

    def test_keep_locked_one_commit(self):
        board = FakeBoard(self.existing())
        plan = bf.plan_from_texts(fixtures.BOARD_V10, fixtures.routed_v10(True), True)
        res = kicad_apply.apply_plan(board, plan)
        self.assertEqual(board.calls, [("begin",), ("remove", 3), ("create", 3),
                                       ("push", "commit-1", kicad_apply.COMMIT_MESSAGE)])
        self.assertEqual(dict(res.removed), {"segment": 1, "arc": 1, "via": 1})
        self.assertEqual(dict(res.created), {"segment": 1, "arc": 1, "via": 1})
        self.assertEqual(dict(res.unknown_nets), {})

    def test_replace_all_and_chunks(self):
        board = FakeBoard(self.existing())
        plan = bf.plan_from_texts(fixtures.BOARD_V10, fixtures.routed_v10(False), False)
        plan.create = plan.create * 400  # 1200 items -> 3 create requests
        kicad_apply.apply_plan(board, plan)
        self.assertEqual(board.calls[1], ("remove", 5))
        self.assertEqual([c for c in board.calls if c[0] == "create"],
                         [("create", 500), ("create", 500), ("create", 200)])

    def test_error_drops_the_commit(self):
        board = FakeBoard(self.existing())

        def boom(items):
            raise RuntimeError("api error")

        board.create_items = boom
        plan = bf.plan_from_texts(fixtures.BOARD_V10, fixtures.routed_v10(True), True)
        with self.assertRaises(RuntimeError):
            kicad_apply.apply_plan(board, plan)
        self.assertEqual(board.calls[-1], ("drop", "commit-1"))
        self.assertNotIn("push", [c[0] for c in board.calls])

    def test_unknown_nets_are_counted(self):
        board = FakeBoard([])
        board.get_nets = lambda: []
        plan = bf.plan_from_texts(fixtures.BOARD_V10, fixtures.routed_v10(True), True)
        res = kicad_apply.apply_plan(board, plan)
        self.assertEqual(sum(res.unknown_nets.values()), 3)


if __name__ == "__main__":
    unittest.main()
