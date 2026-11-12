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
                                    net_widths="/p/w.json", pairs="band",
                                    scope="reroute", nets=["/SDA", "Net-(U1-Pad 3)"])
        self.assertEqual(dataclasses.replace(route, **self.build(fields.ROUTE_TABS, route).values()),
                         route)
        place = runner.PlaceOptions(seed=9, free=["TP1"], iters=1_000_000)
        self.assertEqual(dataclasses.replace(place, **self.build(fields.PLACE_TABS, place).values()),
                         place)

    def choose(self, form, name, word):
        """Pick a word of a choice as the user does."""
        form.set_value(form.by_name[name], word)
        form.on_choice(form.by_name[name])

    def test_pair_fields_follow_the_mode(self):
        form = self.build(fields.ROUTE_TABS, runner.RouteOptions())
        self.assertFalse(form.controls["pair_skew"].IsEnabled())
        self.assertFalse(form.controls["pair_pull"].IsEnabled())
        self.choose(form, "pairs", "pull")
        self.assertTrue(form.controls["pair_pull"].IsEnabled())
        self.assertFalse(form.controls["pair_skew"].IsEnabled())
        self.choose(form, "pairs", "band")
        self.assertTrue(form.controls["pair_pull"].IsEnabled())
        self.assertTrue(form.controls["pair_skew"].IsEnabled())
        self.assertEqual(form.values()["pairs"], "band")

    def test_nets_and_parts_follow_the_scope(self):
        form = self.build(fields.ROUTE_TABS, runner.RouteOptions(nets=["A"]))
        self.assertEqual(form.values()["scope"], "keep_locked")
        self.assertFalse(form.controls["nets"].IsEnabled())
        self.assertTrue(form.controls["move_parts"].IsEnabled())
        self.choose(form, "scope", "reroute")
        self.assertTrue(form.controls["nets"].IsEnabled())
        self.assertFalse(form.controls["move_parts"].IsEnabled())
        self.assertFalse(form.controls["placements"].IsEnabled())
        self.assertEqual(form.values()["nets"], ["A"])

    def test_a_preset_sets_its_fields(self):
        form = self.build(fields.ROUTE_TABS, runner.RouteOptions())
        preset = form.by_name["preset"]
        # the plugin's defaults are the router's final
        self.assertEqual(form.chosen(preset), "final")
        self.assertNotIn("preset", form.values())
        self.choose(form, "preset", "draft")
        self.assertEqual((form.controls["runs"].GetValue(), form.controls["generations"].GetValue(),
                          form.controls["geometry_generations"].GetValue(),
                          form.controls["until_stop"].GetValue()), (4, 1, 0, False))
        self.assertFalse(form.controls["open_via_cost"].IsEnabled())
        # a value of one's own makes it custom
        form.controls["runs"].SetValue(8)
        form.update_enabled()
        self.assertEqual(form.chosen(preset), runner.CUSTOM)
        self.choose(form, "preset", "final")
        self.assertEqual(form.controls["runs"].GetValue(), 14)
        self.assertTrue(form.controls["until_stop"].GetValue())

    def test_open_via_cost_follows_the_open_end(self):
        form = self.build(fields.ROUTE_TABS, runner.RouteOptions())
        self.assertEqual(form.controls["open_via_cost"].GetValue(), 20.0)
        self.assertTrue(form.controls["open_via_cost"].IsEnabled())
        form.controls["until_stop"].SetValue(False)
        form.update_enabled()
        self.assertFalse(form.controls["open_via_cost"].IsEnabled())

    def test_opt_in_values_come_back_unchanged(self):
        route = runner.RouteOptions(hint_groups=True, group_order="short", hint_rows=True,
                                    clear_corridors=True, seed=77, placements=2,
                                    placement_seed=5, pairs="band", via_pairs=True, air=0.05)
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
