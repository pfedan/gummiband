import dataclasses
import unittest

import gummiband_action as action
from gb_plugin import fields, runner


@unittest.skipIf(action.wx is None, "no wxPython")
class DialogTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = action.wx.App(False)

    def build(self, tabs, opts):
        dlg, form = action.options_dialog("t", tabs, opts, type(opts)(), "note")
        self.addCleanup(dlg.Destroy)
        return form

    def test_values_come_back_unchanged(self):
        route = runner.RouteOptions(runs=8, via_pitch=0.25, plane_layers=["In1.Cu", "In2.Cu"],
                                    net_widths="/p/w.json", pairs=True)
        self.assertEqual(dataclasses.replace(route, **self.build(fields.ROUTE_TABS, route).values()),
                         route)
        place = runner.PlaceOptions(seed=9, free=["TP1"], iters=1_000_000)
        self.assertEqual(dataclasses.replace(place, **self.build(fields.PLACE_TABS, place).values()),
                         place)

    def test_pair_fields_follow_the_check_box(self):
        form = self.build(fields.ROUTE_TABS, runner.RouteOptions())
        self.assertFalse(form.controls["pair_skew"].IsEnabled())
        self.assertTrue(form.controls["pair_pull"].IsEnabled())
        form.controls["pairs"].SetValue(True)
        form.update_enabled()
        self.assertTrue(form.controls["pair_skew"].IsEnabled())

    def test_fixed_parts_follow_the_check_box(self):
        form = self.build(fields.ROUTE_TABS, runner.RouteOptions())
        self.assertFalse(form.controls["fixed"].IsEnabled())

    def test_defaults_button_values(self):
        form = self.build(fields.ROUTE_TABS, runner.RouteOptions(runs=8, plane_layers=["In1.Cu"]))
        form.set_values(runner.RouteOptions())
        self.assertEqual(dataclasses.replace(runner.RouteOptions(), **form.values()),
                         runner.RouteOptions())


if __name__ == "__main__":
    unittest.main()
