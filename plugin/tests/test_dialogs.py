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

    def test_open_via_cost_follows_the_open_end(self):
        form = self.build(fields.ROUTE_TABS, runner.RouteOptions())
        self.assertEqual(form.controls["open_via_cost"].GetValue(), 20.0)
        self.assertTrue(form.controls["open_via_cost"].IsEnabled())
        form.controls["until_stop"].SetValue(False)
        form.update_enabled()
        self.assertFalse(form.controls["open_via_cost"].IsEnabled())

    def test_opt_in_values_come_back_unchanged(self):
        route = runner.RouteOptions(hint_groups=True, group_order="inline", hint_rows=True,
                                    clear_corridors=True, seed=77, placements=2,
                                    placement_seed=5, pairs=True, via_pairs=True, air=0.05)
        self.assertEqual(dataclasses.replace(route, **self.build(fields.ROUTE_TABS, route).values()),
                         route)
        place = runner.PlaceOptions(semantic=True, corridors=True, escape_halo=True, leash=1.5,
                                    two_sided=True, back_cost=2.5, back_max_pads=12)
        self.assertEqual(dataclasses.replace(place, **self.build(fields.PLACE_TABS, place).values()),
                         place)

    def test_group_order_follows_the_groups(self):
        form = self.build(fields.ROUTE_TABS, runner.RouteOptions())
        self.assertFalse(form.controls["group_order"].IsEnabled())
        self.assertFalse(form.labels["group_order"].IsEnabled())
        form.controls["hint_groups"].SetValue(True)
        form.update_enabled()
        self.assertTrue(form.controls["group_order"].IsEnabled())

    def test_placement_seed_follows_the_placements(self):
        form = self.build(fields.ROUTE_TABS, runner.RouteOptions())
        self.assertFalse(form.controls["placement_seed"].IsEnabled())
        self.assertFalse(form.buttons["placement_seed"].IsEnabled())
        form.controls["placements"].SetValue(3)
        form.update_enabled()
        self.assertTrue(form.controls["placement_seed"].IsEnabled())
        self.assertTrue(form.buttons["placement_seed"].IsEnabled())

    def test_new_seed_button(self):
        form = self.build(fields.ROUTE_TABS, runner.RouteOptions())
        self.assertEqual(form.controls["seed"].GetValue(), 0)
        button = form.buttons["seed"]
        button.GetEventHandler().ProcessEvent(
            action.wx.CommandEvent(action.wx.wxEVT_BUTTON, button.GetId()))
        self.assertGreater(form.controls["seed"].GetValue(), 0)

    def test_defaults_button_values(self):
        form = self.build(fields.ROUTE_TABS, runner.RouteOptions(runs=8, plane_layers=["In1.Cu"]))
        form.set_values(runner.RouteOptions())
        self.assertEqual(dataclasses.replace(runner.RouteOptions(), **form.values()),
                         runner.RouteOptions())


if __name__ == "__main__":
    unittest.main()
