import unittest

from gb_plugin import sexpr
from gb_plugin.sexpr import QStr, SexprError


class ParseTest(unittest.TestCase):
    def test_nested_lists_and_atoms(self):
        root = sexpr.parse('(a 1 -2.5 (b "x y" c) ())')
        self.assertEqual(root, ["a", "1", "-2.5", ["b", "x y", "c"], []])
        self.assertIsInstance(root[3][1], QStr)
        self.assertNotIsInstance(root[3][2], QStr)

    def test_escapes_and_specials_in_strings(self):
        root = sexpr.parse(r'(net "/a \"b\" \\ (c)" "line\nnext" "")')
        self.assertEqual(root[1], '/a "b" \\ (c)')
        self.assertEqual(root[2], "line\nnext")
        self.assertEqual(root[3], "")
        self.assertIsInstance(root[3], QStr)

    def test_whitespace_and_newlines(self):
        self.assertEqual(sexpr.parse("\n\t( a\n\t\t(b 1)\r\n)\n"), ["a", ["b", "1"]])

    def test_errors(self):
        for bad in ["", "(a", "(a))", "a", '(a "x)', "(a) (b)"]:
            with self.subTest(bad=bad), self.assertRaises(SexprError):
                sexpr.parse(bad)


class TopLevelTest(unittest.TestCase):
    TEXT = '''(kicad_pcb (version 1)
      (net 1 "GND")
      (footprint "X" (pad "1" (net "GND")) (segment (start 9 9)))
      (segment (start 0 0) (end 1 1) (net "GND"))
      (zone (net "GND") (polygon (pts (xy 0 0))))
      (via (at 2 2) (net "a)b"))
    )'''

    def test_only_wanted_top_level_children(self):
        got = list(sexpr.iter_top_level(self.TEXT, ["segment", "via", "net"]))
        self.assertEqual([g[0] for g in got], ["net", "segment", "via"])
        self.assertEqual(got[1], ["segment", ["start", "0", "0"], ["end", "1", "1"],
                                  ["net", "GND"]])
        self.assertEqual(got[2][2], ["net", "a)b"])

    def test_same_result_as_full_parse(self):
        root = sexpr.parse(self.TEXT)
        heads = {"segment", "via", "zone"}
        full = [c for c in root[1:] if isinstance(c, list) and c[0] in heads]
        self.assertEqual(list(sexpr.iter_top_level(self.TEXT, heads)), full)

    def test_errors(self):
        for bad in ["(a (b)", "(a))", '(a (segment "x))']:
            with self.subTest(bad=bad), self.assertRaises(SexprError):
                list(sexpr.iter_top_level(bad, ["segment"]))


class AccessorTest(unittest.TestCase):
    def test_child_children_atoms(self):
        n = sexpr.parse('(via (at 1 2) (layers "F.Cu" "B.Cu") (net 3) (x) (x 1))')
        self.assertEqual(sexpr.head(n), "via")
        self.assertEqual(sexpr.child(n, "at"), ["at", "1", "2"])
        self.assertIsNone(sexpr.child(n, "size"))
        self.assertEqual(len(list(sexpr.children(n, "x"))), 2)
        self.assertEqual(sexpr.atoms(sexpr.child(n, "layers")), ["F.Cu", "B.Cu"])
        self.assertEqual(sexpr.floats(sexpr.child(n, "at"), 2), (1.0, 2.0))
        self.assertIsNone(sexpr.floats(sexpr.child(n, "layers"), 1))
        self.assertIsNone(sexpr.floats(None, 1))
        self.assertIsNone(sexpr.head("atom"))

    def test_locked_flag_variants(self):
        cases = {
            "(segment locked (start 0 0))": True,
            "(segment (locked yes) (start 0 0))": True,
            "(segment (locked) (start 0 0))": True,
            "(segment (locked no) (start 0 0))": False,
            '(segment "locked" (start 0 0))': False,
            "(segment (start 0 0))": False,
        }
        for text, want in cases.items():
            with self.subTest(text=text):
                self.assertEqual(sexpr.has_flag(sexpr.parse(text), "locked"), want)


if __name__ == "__main__":
    unittest.main()
