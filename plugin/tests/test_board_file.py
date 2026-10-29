import unittest

from gb_plugin import board_file as bf
from tests import fixtures


class ReadCopperTest(unittest.TestCase):
    def test_kicad10_items(self):
        c = bf.read_copper(fixtures.BOARD_V10)
        self.assertEqual([i.kind for i in c.items], ["segment", "segment", "arc", "via", "via"])
        s0, s1, arc, v0, v1 = c.items
        self.assertEqual(s0, bf.Segment((1.0, 2.0), (3.0, 4.5), 0.25, "F.Cu", "GND", False,
                                        "a0000000-0000-0000-0000-000000000001"))
        self.assertTrue(s1.locked)
        self.assertEqual(s1.net, '/sheet/SIG "A"')
        self.assertEqual(arc.mid, (0.7071, 0.2929))
        self.assertTrue(v0.locked)
        self.assertTrue(v0.is_through)
        self.assertEqual((v0.size, v0.drill, v0.at), (0.6, 0.3, (7.0, 8.0)))
        self.assertEqual(v1.layers, ("F.Cu", "In1.Cu"))
        self.assertFalse(v1.is_through)
        self.assertEqual(c.net_table, {})
        self.assertEqual(c.warnings, [])

    def test_kicad9_net_numbers_and_bare_locked(self):
        c = bf.read_copper(fixtures.BOARD_V9)
        self.assertEqual(c.net_table, {0: "", 1: "GND", 2: "VCC"})
        self.assertEqual([(i.kind, i.net, i.locked) for i in c.items], [
            ("segment", "VCC", True),
            ("segment", "GND", False),
            ("segment", "", False),
            ("via", "GND", False),
        ])

    def test_unknown_net_number_warns(self):
        c = bf.read_copper('(kicad_pcb (segment (start 0 0) (end 1 0) (width 0.2) '
                           '(layer "F.Cu") (net 7) (uuid "u")))')
        self.assertEqual(c.items[0].net, "")
        self.assertEqual(len(c.warnings), 1)

    def test_not_a_board(self):
        with self.assertRaises(bf.BoardFileError):
            bf.read_copper("(kicad_sch (version 1))")

    def test_missing_geometry(self):
        with self.assertRaises(bf.BoardFileError):
            bf.read_copper('(kicad_pcb (segment (start 0 0) (width 0.2) (layer "F.Cu")))')

    def test_pad_nets(self):
        self.assertEqual(bf.pad_net_names(fixtures.BOARD_V10), {"GND", '/sheet/SIG "A"'})
        self.assertEqual(bf.pad_net_names(fixtures.BOARD_V9), {"", "GND", "VCC"})


class PlanTest(unittest.TestCase):
    def plan(self, keep_locked):
        return bf.plan_from_texts(fixtures.BOARD_V10, fixtures.routed_v10(keep_locked),
                                  keep_locked)

    def test_keep_locked(self):
        p = self.plan(True)
        self.assertEqual([i.uuid[-1] for i in p.remove], ["1", "3", "5"])
        self.assertEqual([i.uuid[-1] for i in p.keep], ["2", "4"])
        self.assertEqual([i.kind for i in p.create], ["segment", "arc", "via"])
        self.assertTrue(all(i.uuid.startswith("c") for i in p.create))
        self.assertEqual(p.warnings, [])
        self.assertEqual(bf.format_counts(p.summary()["create"]), "1 tracks, 1 arcs, 1 vias")

    def test_replace_all(self):
        p = self.plan(False)
        self.assertEqual(len(p.remove), 5)
        self.assertEqual(p.keep, [])
        self.assertEqual(len(p.create), 3)

    def test_lost_locked_items_are_reported(self):
        # routed without the locked copper although keep_locked was asked for
        p = bf.plan_from_texts(fixtures.BOARD_V10, fixtures.routed_v10(False), True)
        self.assertEqual(len(p.create), 3)
        self.assertEqual(len(p.warnings), 1)

    def test_items_without_uuid_match_by_geometry(self):
        a = '(kicad_pcb (segment (start 0 0) (end 1 0) (width 0.2) (layer "F.Cu") (net "N")))'
        b = a[:-1] + ' (segment (start 1 0) (end 2 0) (width 0.2) (layer "F.Cu") (net "N")))'
        p = bf.plan_from_texts(a, b, True)
        self.assertEqual([i.start for i in p.create], [(1.0, 0.0)])

    def test_should_remove_matches_cli(self):
        self.assertTrue(bf.should_remove(False, True))
        self.assertTrue(bf.should_remove(False, False))
        self.assertTrue(bf.should_remove(True, False))
        self.assertFalse(bf.should_remove(True, True))


if __name__ == "__main__":
    unittest.main()
