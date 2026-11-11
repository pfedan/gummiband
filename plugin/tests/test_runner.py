import dataclasses
import json
import os
import stat
import sys
import tempfile
import unittest
from pathlib import Path

from gb_plugin import fields, runner, table

STDOUT = """substrate: 1 vertices, 2 triangles, 3 via sites
  run seed  0 fanout_first  true: 9/10 routed, 4 vias, 120 mm
best: seed 0 9/10 routed, 4 vias, 120 mm, 0.50 s
  unrouted: GND from U1.3
clearance check: 2 violations
wrote 12 items to out.kicad_pcb
"""


def make_exe(path: Path, body: str = "") -> Path:
    path.write_text(f"#!{sys.executable}\n{body}\n")
    path.chmod(path.stat().st_mode | stat.S_IXUSR)
    return path


class FindBinaryTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.settings = self.dir / "settings.json"
        self.env_bin = make_exe(self.dir / "env-gummiband")
        self.set_bin = make_exe(self.dir / "set-gummiband")
        self.path_dir = self.dir / "bin"
        self.path_dir.mkdir()
        self.path_bin = make_exe(self.path_dir / "gummiband")

    def tearDown(self):
        self.tmp.cleanup()

    def find(self, env):
        return runner.find_binary(env, self.settings, extra_dirs=())

    def test_precedence(self):
        self.settings.write_text(json.dumps({"gummiband_bin": str(self.set_bin)}))
        env = {"GUMMIBAND_BIN": str(self.env_bin), "PATH": str(self.path_dir)}
        self.assertEqual(self.find(env), str(self.env_bin))
        del env["GUMMIBAND_BIN"]
        self.assertEqual(self.find(env), str(self.set_bin))
        self.settings.unlink()
        self.assertEqual(self.find(env), str(self.path_bin))

    def test_bad_entries_fall_through(self):
        self.settings.write_text(json.dumps({"gummiband_bin": str(self.dir / "nope")}))
        env = {"GUMMIBAND_BIN": str(self.dir), "PATH": str(self.path_dir)}
        self.assertEqual(self.find(env), str(self.path_bin))

    def test_not_found_lists_what_was_tried(self):
        with self.assertRaises(runner.RunnerError) as cm:
            self.find({"GUMMIBAND_BIN": "/does/not/exist", "PATH": str(self.dir / "empty")})
        self.assertIn("/does/not/exist", str(cm.exception))
        self.assertIn("settings.json", str(cm.exception))

    def test_broken_settings(self):
        self.settings.write_text("{not json")
        with self.assertRaises(runner.RunnerError):
            self.find({"PATH": ""})


class CommandTest(unittest.TestCase):
    def test_build_command(self):
        opts = runner.RouteOptions(scope="keep_locked", runs=8, via_cost=2,
                                   geometry_generations=0, until_stop=False)
        self.assertEqual(
            runner.build_command("gb", Path("/a/b.kicad_pcb"), Path("/t/o.kicad_pcb"), opts),
            ["gb", "route", "/a/b.kicad_pcb", "-o", "/t/o.kicad_pcb", "--progress",
             "--scope", "keep_locked",
             "--runs", "8", "--via-cost", "2.0", "--geometry-generations", "0"])
        opts.scope = "fresh"
        cmd = runner.build_command("gb", Path("b"), Path("o"), opts)
        self.assertEqual(cmd[cmd.index("--scope") + 1], "fresh")
        # the spellings of earlier versions are gone from the command
        for old in ("--keep-locked", "--keep-all", "--reroute"):
            self.assertNotIn(old, cmd)

    def test_finish_nets(self):
        # what the Finish action runs: all copper stays, the nets are finished, quickly
        opts = runner.with_preset(runner.RouteOptions(scope="finish", nets=["A", "B"]), "quick")
        cmd = runner.build_command("gb", Path("b"), Path("o"), opts)
        self.assertEqual(cmd[cmd.index("--scope") + 1], "finish")
        self.assertEqual(cmd[cmd.index("--nets") + 1], "A,B")
        self.assertEqual(cmd[cmd.index("--geometry-generations") + 1], "0")
        # shove and give are the router's defaults there, and there is no window to accept in
        for flag in ("--shove", "--give", "--keep-all", "--until-stop"):
            self.assertNotIn(flag, cmd)

    def test_scopes(self):
        cmd = runner.build_command("gb", Path("b"), Path("o"),
                                   runner.RouteOptions(scope="reroute", nets=["/SDA"]))
        self.assertEqual(cmd[cmd.index("--scope") + 1], "reroute")
        self.assertEqual(cmd[cmd.index("--nets") + 1], "/SDA")
        # finish without nets lays everything that is open
        cmd = runner.build_command("gb", Path("b"), Path("o"), runner.RouteOptions(scope="finish"))
        self.assertNotIn("--nets", cmd)
        # nets narrow only where the copper stays: elsewhere the others would lose theirs
        cmd = runner.build_command("gb", Path("b"), Path("o"),
                                   runner.RouteOptions(scope="keep_locked", nets=["A"]))
        self.assertNotIn("--nets", cmd)
        self.assertIsNone(runner.problem(runner.RouteOptions(scope="finish")))
        self.assertIsNone(runner.problem(runner.RouteOptions(scope="reroute", nets=["A"])))
        self.assertIn("needs the nets", runner.problem(runner.RouteOptions(scope="reroute")))
        self.assertEqual(runner.parse_nets(" /SDA, Net-(U1-Pad 3) ,,"), ["/SDA", "Net-(U1-Pad 3)"])

    def test_settled(self):
        asked = runner.RouteOptions(scope="finish", nets=["A"], placements=3, clear_corridors=True)
        run = runner.settled(asked)
        self.assertEqual((run.move_parts, run.placements, run.clear_corridors, run.nets),
                         (False, 0, False, ["A"]))
        run = runner.settled(runner.RouteOptions(scope="keep_locked", nets=["A"], placements=3))
        self.assertEqual((run.move_parts, run.placements, run.nets), (True, 3, None))

    def test_quality_and_animation(self):
        opts = runner.RouteOptions(geometry_generations=3, frames_dir="/p/anim", until_stop=False)
        cmd = runner.build_command("gb", Path("b"), Path("o"), opts)
        self.assertEqual(cmd[-4:], ["--geometry-generations", "3", "--frames", "/p/anim"])

    def test_progress(self):
        line = ('progress {"phase":"geometry generation 1","routed":347,"conns":348,"vias":261,'
                '"length":2585.4,"violations":3,"seconds":61.2}')
        p = runner.parse_progress(line)
        self.assertEqual(p["routed"], 347)
        self.assertEqual(runner.progress_text(p),
                         "geometry generation 1: 347/348 routed, 261 vias, 2585 mm, "
                         "3 clearance violations")
        p["violations"] = None
        self.assertNotIn("violations", runner.progress_text(p))
        self.assertIsNone(runner.parse_progress("best: seed 0"))
        self.assertIsNone(runner.parse_progress("progress {broken"))
        self.assertEqual(runner.progress_text(None), "Routing...")

    def test_tool_env_adds_install_dirs(self):
        env = runner.tool_env({"PATH": "/usr/bin"}, extra_dirs=("/opt/x/bin", "/usr/bin"))
        self.assertEqual(env["PATH"], os.pathsep.join(["/usr/bin", "/opt/x/bin"]))

    def test_defaults(self):
        o = runner.RouteOptions()
        self.assertEqual(o.scope, "keep_locked")
        self.assertEqual(o.pairs, "off")
        self.assertEqual(o.runs, 14)
        self.assertEqual(o.via_cost, 3.0)
        self.assertEqual(o.geometry_generations, 3)
        self.assertIsNone(o.frames_dir)

    def test_parse_output(self):
        r = runner.RouteResult([], 0, STDOUT, "")
        self.assertEqual(r.best_line(), "best: seed 0 9/10 routed, 4 vias, 120 mm, 0.50 s")
        self.assertEqual(r.unrouted(), ["GND from U1.3"])
        self.assertEqual(r.violations(), 2)
        self.assertIsNone(r.animation())
        r = runner.RouteResult([], 0, STDOUT + "frames: 5 images, animation /p/a/evolution.gif\n", "")
        self.assertEqual(r.animation(), "/p/a/evolution.gif")
        self.assertIsNone(runner.RouteResult([], 0, "", "").best_line())


class RunRouteTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.out = self.dir / "out.kicad_pcb"

    def tearDown(self):
        self.tmp.cleanup()

    def test_success(self):
        exe = make_exe(self.dir / "ok", f"""import sys
open(sys.argv[sys.argv.index('-o') + 1], 'w').write('(kicad_pcb)')
print({STDOUT!r})""")
        r = runner.run_route([str(exe), "route", "b", "-o", str(self.out)], self.out, self.dir)
        self.assertEqual(r.returncode, 0)
        self.assertEqual(r.violations(), 2)
        self.assertIsNone(r.animation())
        r = runner.RouteResult([], 0, STDOUT + "frames: 5 images, animation /p/a/evolution.gif\n", "")
        self.assertEqual(r.animation(), "/p/a/evolution.gif")

    def test_failure_shows_output(self):
        exe = make_exe(self.dir / "bad", """import sys
print('partial output')
print('thread main panicked', file=sys.stderr)
sys.exit(101)""")
        with self.assertRaises(runner.RouteFailed) as cm:
            runner.run_route([str(exe)], self.out, self.dir)
        msg = str(cm.exception)
        self.assertIn("exit code 101", msg)
        self.assertIn("partial output", msg)
        self.assertIn("panicked", msg)

    def test_missing_output_is_a_failure(self):
        exe = make_exe(self.dir / "noout", "pass")
        with self.assertRaises(runner.RouteFailed) as cm:
            runner.run_route([str(exe)], self.out, self.dir)
        self.assertIn("no output file", str(cm.exception))

    def test_cancel_kills_the_router(self):
        exe = make_exe(self.dir / "slow", "import time\ntime.sleep(30)")
        calls = []

        def keep_going(progress=None):
            calls.append(1)
            return len(calls) < 3

        with self.assertRaises(runner.RouteCancelled):
            runner.run_route([str(exe)], self.out, self.dir, keep_going)
        self.assertEqual(len(calls), 3)

    def test_progress_reaches_keep_going(self):
        exe = make_exe(self.dir / "prog", f"""import sys, time, pathlib
print('progress {{"phase":"portfolio","routed":1,"conns":2,"vias":0,"length":3}}', flush=True)
time.sleep(0.5)
print('best: seed 0')
pathlib.Path({str(self.out)!r}).write_text('x')""")
        seen = []

        def keep_going(progress=None):
            if progress:
                seen.append(progress["phase"])
            return True

        res = runner.run_route([str(exe)], self.out, self.dir, keep_going)
        self.assertIn("portfolio", seen)
        self.assertNotIn("progress", res.stdout)
        self.assertEqual(res.best_line(), "best: seed 0")

    def test_cannot_start(self):
        with self.assertRaises(runner.RunnerError):
            runner.run_route([str(self.dir / "missing")], self.out, self.dir)


class AcceptTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.out = self.dir / "o.kicad_pcb"

    def tearDown(self):
        self.tmp.cleanup()

    def test_accept_ends_the_run_and_the_result_counts(self):
        # waits for the end of stdin like --until-stop, then writes its result
        exe = make_exe(self.dir / "gb", f"""
import sys
print('progress {{"phase": "geometry generation 1", "generation": 1, "converged": true}}', flush=True)
sys.stdin.read()
open({str(self.out)!r}, "w").write("done")
""")
        seen = []
        def keep_going(p):
            seen.append(p)
            return True
        res = runner.run_route([str(exe)], self.out, self.dir, keep_going,
                               lambda: bool(seen and seen[-1]))
        self.assertEqual(res.returncode, 0)
        self.assertEqual(self.out.read_text(), "done")
        self.assertTrue(runner.converged(seen[-1]))

    def test_command_flag(self):
        opts = runner.RouteOptions(until_stop=True)
        self.assertIn("--until-stop", runner.build_command("gb", Path("b"), Path("o"), opts))
        off = runner.RouteOptions(until_stop=False)
        self.assertNotIn("--until-stop", runner.build_command("gb", Path("b"), Path("o"), off))
        # nets can be finished open ended from the dialog
        finish = runner.RouteOptions(scope="finish", nets=["A"])
        self.assertIn("--until-stop", runner.build_command("gb", Path("b"), Path("o"), finish))
        # the wide track of pairs has no open end, the pull has
        pairs = runner.RouteOptions(until_stop=True, pairs="band")
        self.assertNotIn("--until-stop", runner.build_command("gb", Path("b"), Path("o"), pairs))
        pull = runner.RouteOptions(until_stop=True, pairs="pull")
        self.assertIn("--until-stop", runner.build_command("gb", Path("b"), Path("o"), pull))

    def test_open_via_cost_goes_with_the_open_end(self):
        cmd = runner.build_command("gb", Path("b"), Path("o"), runner.RouteOptions())
        self.assertEqual(cmd[cmd.index("--open-via-cost") + 1], "20.0")
        cheap = runner.RouteOptions(open_via_cost=12.5)
        cmd = runner.build_command("gb", Path("b"), Path("o"), cheap)
        self.assertEqual(cmd[cmd.index("--open-via-cost") + 1], "12.5")
        off = runner.RouteOptions(until_stop=False)
        self.assertNotIn("--open-via-cost", runner.build_command("gb", Path("b"), Path("o"), off))

    def test_angles_flag(self):
        cmd = runner.build_command("gb", Path("b"), Path("o"), runner.RouteOptions(angles_45=True))
        self.assertEqual(cmd[cmd.index("--angles") + 1], "45")
        plain = runner.build_command("gb", Path("b"), Path("o"), runner.RouteOptions())
        self.assertNotIn("--angles", plain)

    def test_angles_command(self):
        cmd = runner.build_angles_command("gb", Path("b"), Path("o"), ["GND", "VCC"],
                                          runner.RouteOptions(threads=2))
        self.assertEqual(cmd[:5], ["gb", "angles45", "b", "-o", "o"])
        self.assertEqual(cmd[cmd.index("--nets") + 1], "GND,VCC")
        self.assertEqual(cmd[cmd.index("--threads") + 1], "2")
        self.assertNotIn("--nets", runner.build_angles_command("gb", Path("b"), Path("o")))
        line = "45 degrees: 3 tracks, 2 changed, 10 -> 11 mm"
        self.assertEqual(runner.angles_line(f"x\n{line}\nreplaced 4 items\n"), line)

    def test_alternatives(self):
        res = runner.RouteResult([], 0, "best: x\nalternative: /t/o.alt1.kicad_pcb: 5/5 routed, 9 vias, "
                                 "20 mm, 0 clearance spots\nwrote 3 items\n", "")
        self.assertEqual(res.alternatives(), [("/t/o.alt1.kicad_pcb",
                                               "5/5 routed, 9 vias, 20 mm, 0 clearance spots")])

    def test_prefer_width_flag(self):
        opts = runner.RouteOptions(prefer_width=0.25)
        cmd = runner.build_command("gb", Path("b"), Path("o"), opts)
        self.assertEqual(cmd[cmd.index("--prefer-width") + 1], "0.25")


class GpuTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.out = self.dir / "o.kicad_pcb"
        self.exe = make_exe(self.dir / "gb", f"""
import os
open({str(self.out)!r}, "w").write(os.environ.get("GB_GPU", "none"))
""")

    def tearDown(self):
        self.tmp.cleanup()

    def test_environment_of_the_router(self):
        runner.run_route([str(self.exe)], self.out, self.dir, gpu=True)
        self.assertEqual(self.out.read_text(), "1")
        runner.run_route([str(self.exe)], self.out, self.dir)
        self.assertEqual(self.out.read_text(), "none")

    def test_binary_without_gpu_is_reported(self):
        line = "warning: GB_GPU has no effect, this binary is built without the gpu feature\n"
        self.assertTrue(runner.RouteResult([], 0, STDOUT + line, "").gpu_missing())
        self.assertFalse(runner.RouteResult([], 0, STDOUT, "").gpu_missing())


class TailTest(unittest.TestCase):
    def test_tail(self):
        self.assertEqual(runner.tail(""), "(empty)")
        self.assertEqual(runner.tail("a\nb\n", 5), "a\nb")
        self.assertEqual(runner.tail("1\n2\n3\n4", 2), "...\n3\n4")


class AdvancedCommandTest(unittest.TestCase):
    def test_defaults_add_nothing(self):
        cmd = runner.build_command("gb", Path("b"), Path("o"),
                                   runner.RouteOptions(geometry_generations=0, scope="fresh",
                                                       until_stop=False))
        self.assertEqual(cmd, ["gb", "route", "b", "-o", "o", "--progress", "--scope", "fresh",
                               "--runs", "14", "--via-cost", "3.0", "--geometry-generations", "0"])

    def test_changed_values_reach_the_router(self):
        opts = runner.RouteOptions(generations=5, via_pitch=0.25, negotiate=20, threads=4,
                                   plane_layers=["In1.Cu", "In2.Cu"], net_widths="/p/w.json",
                                   net_priorities="/p/p.json")
        cmd = runner.build_command("gb", Path("b"), Path("o"), opts)
        for flag, value in (("--generations", "5"), ("--via-pitch", "0.25"), ("--negotiate", "20"),
                            ("--threads", "4"), ("--plane-layers", "In1.Cu,In2.Cu"),
                            ("--net-widths", "/p/w.json"), ("--net-priorities", "/p/p.json")):
            self.assertEqual(cmd[cmd.index(flag) + 1], value)

    def test_move_parts(self):
        # on by default: the router's default, only the fixed parts are passed
        opts = runner.RouteOptions(fixed=["U1", "U2"])
        cmd = runner.build_command("gb", Path("b"), Path("o"), opts)
        self.assertNotIn("--no-move-parts", cmd)
        self.assertEqual(cmd[cmd.index("--fixed") + 1], "U1,U2")
        opts.move_parts = False
        cmd = runner.build_command("gb", Path("b"), Path("o"), opts)
        self.assertIn("--no-move-parts", cmd)
        self.assertNotIn("--fixed", cmd)
        # finishing nets keeps the parts in place, the router's default there
        opts = runner.RouteOptions(scope="finish", nets=["A"], fixed=["U1"])
        cmd = runner.build_command("gb", Path("b"), Path("o"), opts)
        self.assertNotIn("--no-move-parts", cmd)
        self.assertNotIn("--move-parts", cmd)
        self.assertNotIn("--fixed", cmd)

    def test_pair_options_need_their_mode(self):
        # off: nothing of the pairs reaches the router, whatever the fields hold
        opts = runner.RouteOptions(pair_skew=0.2, pair_zdiff=90.0, pair_pull=1.5)
        cmd = runner.build_command("gb", Path("b"), Path("o"), opts)
        for flag in ("--pairs", "--pair-skew", "--pair-zdiff", "--pair-pull"):
            self.assertNotIn(flag, cmd)
        # pull: the mode and its weight, none of the options of the wide track
        opts.pairs = "pull"
        cmd = runner.build_command("gb", Path("b"), Path("o"), opts)
        self.assertEqual(cmd[cmd.index("--pairs") + 1], "pull")
        self.assertEqual(cmd[cmd.index("--pair-pull") + 1], "1.5")
        self.assertNotIn("--pair-zdiff", cmd)
        # the weight 0 leaves the pull to the router
        cmd = runner.build_command("gb", Path("b"), Path("o"), runner.RouteOptions(pairs="pull"))
        self.assertNotIn("--pair-pull", cmd)
        # band: the wide track with its options, and a weight for the pairs that get none
        opts.pairs = "band"
        cmd = runner.build_command("gb", Path("b"), Path("o"), opts)
        self.assertEqual(cmd[cmd.index("--pairs") + 1], "band")
        self.assertEqual(cmd[cmd.index("--pair-zdiff") + 1], "90.0")
        self.assertEqual(cmd[cmd.index("--pair-pull") + 1], "1.5")

    def test_place(self):
        opts = runner.PlaceOptions(seed=7, iters=1_000_000, margin=1.0, free=["TP1", "SW1"],
                                   threads=2, plane_layers=["In1.Cu"])
        cmd = runner.build_place_command("gb", Path("a"), Path("b"), Path("r"), opts)
        for flag, value in (("--seed", "7"), ("--iters", "1000000"), ("--margin", "1.0"),
                            ("--free", "TP1,SW1"), ("--threads", "2"),
                            ("--plane-layers", "In1.Cu")):
            self.assertEqual(cmd[cmd.index(flag) + 1], value)


class OptInCommandTest(unittest.TestCase):
    def cmd(self, **kw):
        return runner.build_command("gb", Path("b"), Path("o"), runner.RouteOptions(**kw))

    def test_hints_are_off_by_default(self):
        c = self.cmd()
        for flag in ("--patterns", "--clear-corridors", "--seed", "--placement-seed", "--air",
                     "--pair-all", "--via-pairs", "--breakouts"):
            self.assertNotIn(flag, c)

    def test_patterns(self):
        c = self.cmd(hint_groups=True, hint_pairs=True)
        self.assertEqual(c[c.index("--patterns") + 1], "groups,pairs")
        c = self.cmd(hint_groups=True, group_order="short", hint_decaps=True, hint_shares=True)
        self.assertEqual(c[c.index("--patterns") + 1], "groups-short,decaps,shares")
        # the order alone is no hint
        self.assertNotIn("--patterns", self.cmd(group_order="short"))
        # an order from older settings that is gone falls back to the default
        c = self.cmd(hint_groups=True, group_order="inline")
        self.assertEqual(c[c.index("--patterns") + 1], "groups")
        # rows acts only in the open end
        c = self.cmd(hint_rows=True)
        self.assertEqual(c[c.index("--patterns") + 1], "rows")
        self.assertNotIn("--patterns", self.cmd(hint_rows=True, until_stop=False))

    def test_clear_corridors(self):
        self.assertIn("--clear-corridors", self.cmd(clear_corridors=True))
        self.assertNotIn("--clear-corridors",
                         self.cmd(clear_corridors=True, scope="finish", nets=["A"]))

    def test_seeds(self):
        c = self.cmd(seed=42)
        self.assertEqual(c[c.index("--seed") + 1], "42")
        self.assertNotIn("--placement-seed", self.cmd(placement_seed=7))
        c = self.cmd(placements=3, placement_seed=7)
        self.assertEqual(c[c.index("--placement-seed") + 1], "7")
        # placements route the whole board: not where its copper stays
        c = self.cmd(placements=3, scope="reroute", nets=["A"])
        self.assertNotIn("--placements", c)
        self.assertNotIn("--placement-seed", c)

    def test_next_placement_seed(self):
        opts = runner.RouteOptions(placements=3, placement_seed=7)
        self.assertEqual(runner.next_placement_seed(opts).placement_seed, 10)
        self.assertEqual(opts.placement_seed, 7)
        same = runner.RouteOptions(placement_seed=7)
        self.assertIs(runner.next_placement_seed(same), same)
        top = runner.RouteOptions(placements=2, placement_seed=runner.SEED_MAX)
        self.assertEqual(runner.next_placement_seed(top).placement_seed, 1)
        self.assertTrue(1 <= runner.random_seed() <= runner.SEED_MAX)

    def test_pair_switches_need_pairs(self):
        c = self.cmd(pair_all=True, via_pairs=True, breakouts=True, until_stop=False)
        self.assertNotIn("--pair-all", c)
        c = self.cmd(pairs="pull", pair_all=True, via_pairs=True, breakouts=True)
        self.assertNotIn("--pair-all", c)
        c = self.cmd(pairs="band", pair_all=True, via_pairs=True, breakouts=True)
        for flag in ("--pairs", "--pair-all", "--via-pairs", "--breakouts"):
            self.assertIn(flag, c)

    def test_air_goes_with_the_open_end(self):
        self.assertNotIn("--air", self.cmd(air=0.03))
        c = self.cmd(air=0.05)
        self.assertEqual(c[c.index("--air") + 1], "0.05")
        self.assertNotIn("--air", self.cmd(air=0.05, until_stop=False))

    def test_place_switches(self):
        c = runner.build_place_command("gb", Path("a"), Path("b"), Path("r"),
                                       runner.PlaceOptions(semantic=True, corridors=True,
                                                           escape_halo=True, leash=1.5))
        for flag in ("--semantic", "--corridors", "--escape-halo"):
            self.assertIn(flag, c)
        self.assertEqual(c[c.index("--leash") + 1], "1.5")
        c = runner.build_place_command("gb", Path("a"), Path("b"), Path("r"), runner.PlaceOptions())
        for flag in ("--semantic", "--corridors", "--escape-halo", "--leash"):
            self.assertNotIn(flag, c)

    def test_two_sided_takes_its_options(self):
        opts = runner.PlaceOptions(two_sided=True, back_cost=2.0, back_max_pads=16)
        c = runner.build_place_command("gb", Path("a"), Path("b"), Path("r"), opts)
        self.assertIn("--two-sided", c)
        self.assertEqual(c[c.index("--back-cost") + 1], "2.0")
        self.assertEqual(c[c.index("--back-max-pads") + 1], "16")
        one = dataclasses.replace(opts, two_sided=False)
        c = runner.build_place_command("gb", Path("a"), Path("b"), Path("r"), one)
        for flag in ("--two-sided", "--back-cost", "--back-max-pads"):
            self.assertNotIn(flag, c)

    def test_parse_seed_and_corridors(self):
        r = runner.RouteResult([], 0, "corridors: 2 parts in the way get new positions: R1 C4\n"
                                      "best placement: seed 9\n", "")
        self.assertEqual(r.cleared_corridors(), ["R1", "C4"])
        self.assertEqual(r.placement_seed(), 9)
        empty = runner.RouteResult([], 0, "corridors: no part in the way\n", "")
        self.assertEqual(empty.cleared_corridors(), [])
        self.assertIsNone(empty.placement_seed())


class SavedOptionsTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "settings.json"

    def tearDown(self):
        self.tmp.cleanup()

    def load(self):
        return runner.load_options(runner.RouteOptions, "route", runner.ROUTE_SAVED, self.path)

    def test_round_trip_keeps_the_binary_path(self):
        self.path.write_text('{"gummiband_bin": "/x/gb"}')
        opts = runner.RouteOptions(runs=8, via_cost=2.5, pairs="band", plane_layers=["In1.Cu"],
                                   scope="fresh")
        self.assertTrue(runner.save_options("route", opts, runner.ROUTE_SAVED, self.path))
        stored = runner.load_settings(self.path)
        self.assertEqual(stored["gummiband_bin"], "/x/gb")
        self.assertNotIn("plane_layers", stored["route"])
        loaded = runner.RouteOptions(**self.load())
        self.assertEqual((loaded.runs, loaded.via_cost, loaded.pairs, loaded.scope),
                         (8, 2.5, "band", "fresh"))
        self.assertIsNone(loaded.plane_layers)

    def test_no_file(self):
        self.assertEqual(self.load(), {})

    def test_bad_values_are_ignored(self):
        self.path.write_text(json.dumps({"route": {
            "runs": "many", "pairs": 1, "via_cost": 2, "scope": "everything", "gone": 1}}))
        self.assertEqual(self.load(), {"via_cost": 2.0})

    def test_settings_of_earlier_versions(self):
        # keep_locked was a switch, pairs the switch for the wide track, and the pull was on
        # whenever its weight was above 0: the same mapping as the router's old spellings
        cases = (
            ({"keep_locked": False}, {"scope": "fresh"}),
            ({"keep_locked": True}, {"scope": "keep_locked"}),
            ({"pairs": True}, {"pairs": "band"}),
            ({"pairs": True, "pair_pull": 1.5}, {"pairs": "band", "pair_pull": 1.5}),
            ({"pairs": False, "pair_pull": 2.0}, {"pairs": "pull", "pair_pull": 2.0}),
            ({"pairs": False, "pair_pull": 0.0}, {"pairs": "off", "pair_pull": 0.0}),
            ({"pairs": False}, {"pairs": "off"}),
            # the names of today win over a key that an earlier version left in the file
            ({"scope": "fresh", "keep_locked": True}, {"scope": "fresh"}),
        )
        for stored, want in cases:
            self.path.write_text(json.dumps({"route": stored}))
            self.assertEqual(self.load(), want, stored)

    def test_a_scope_for_single_nets_is_not_remembered(self):
        for scope in ("finish", "reroute"):
            opts = runner.RouteOptions(scope=scope, nets=["A"])
            self.assertTrue(runner.save_options("route", opts, runner.ROUTE_SAVED, self.path))
            self.assertNotIn("scope", self.load())
            self.assertNotIn("nets", runner.load_settings(self.path)["route"])

    def test_broken_file_stays(self):
        self.path.write_text("{not json")
        self.assertEqual(self.load(), {})
        self.assertFalse(runner.save_options("route", runner.RouteOptions(),
                                             runner.ROUTE_SAVED, self.path))
        self.assertEqual(self.path.read_text(), "{not json")


class FieldTableTest(unittest.TestCase):
    def test_fields_match_the_options(self):
        for cls, tabs, saved in ((runner.RouteOptions, fields.ROUTE_TABS, runner.ROUTE_SAVED),
                                 (runner.PlaceOptions, fields.PLACE_TABS, runner.PLACE_SAVED)):
            dialog = fields.all_fields(tabs)
            names = [f.name for f in dialog]
            self.assertEqual(len(names), len(set(names)))
            self.assertLessEqual(set(saved), set(names))
            opts = cls()
            for f in dialog:
                self.assertTrue(f.tip, f.name)
                if f.kind in ("int", "float"):
                    self.assertTrue(f.low <= getattr(opts, f.name) <= f.high, f.name)
                if f.requires is not None:
                    name, _, words = f.requires.partition("=")
                    self.assertIn(name, names)
                    if words:
                        other = next(o for o in dialog if o.name == name)
                        self.assertLessEqual(set(words.split("|")),
                                             {v for v, _label in other.choices}, f.name)
                if f.kind in ("refs", "nets", "file") or f.derived:
                    self.assertNotIn(f.name, saved)
                if f.derived:
                    self.assertFalse(hasattr(opts, f.name), f.name)
                elif f.kind == "choice":
                    self.assertIn(getattr(opts, f.name), [v for v, _label in f.choices])


class RouterTableTest(unittest.TestCase):
    """The choices and presets of the dialog are those of the router's own table
    (gummiband_app::OPTIONS, in gb_plugin/route_options.json)."""

    def field(self, name):
        return next(f for f in fields.all_fields(fields.ROUTE_TABS) if f.name == name)

    def test_choices_have_the_words_of_the_router(self):
        for name in ("scope", "pairs"):
            self.assertEqual([v for v, _label in self.field(name).choices],
                             list(table.words(name)), name)
            for word in table.words(name):
                self.assertIn(table.meaning(name, word), self.field(name).tip)
        self.assertEqual([v for v, _label in self.field("preset").choices],
                         list(table.words("preset")) + [runner.CUSTOM])
        # the flags the command gets are words of the router
        self.assertIn(runner.RouteOptions().scope, table.words("scope"))
        self.assertIn(runner.RouteOptions().pairs, table.words("pairs"))
        self.assertEqual(set(runner.KEEP_ALL) | {"fresh", "keep_locked"}, set(table.words("scope")))

    def test_presets_are_the_router_s(self):
        self.assertEqual(list(table.PRESETS), list(table.words("preset")))
        self.assertEqual(runner.preset_values("draft"),
                         {"runs": 4, "generations": 1, "geometry_generations": 0,
                          "until_stop": False})
        self.assertEqual(runner.preset_values("final"),
                         {"runs": 14, "generations": 3, "geometry_generations": 3,
                          "until_stop": True})
        # the sentence of a preset starts with what it sets
        for name, sets in table.PRESETS.items():
            self.assertTrue(table.meaning("preset", name).startswith(
                f"runs {sets['runs']}, generations {sets['generations']}, "
                f"geometry_generations {sets['geometry_generations']}, "), name)
        # the plugin's own defaults are the router's standard, and the open end on top: final
        d = runner.RouteOptions()
        self.assertEqual(runner.preset_of(d), "final")
        self.assertEqual(runner.preset_of(dataclasses.replace(d, until_stop=False)), "standard")
        self.assertEqual(runner.preset_of(dataclasses.replace(d, runs=8)), runner.CUSTOM)
        for name in table.PRESETS:
            self.assertEqual(runner.preset_of(runner.with_preset(d, name)), name)
        # the Finish action runs the router's quick
        quick = runner.with_preset(d, "quick")
        self.assertEqual((quick.runs, quick.generations, quick.geometry_generations,
                          quick.until_stop), (14, 3, 0, False))

    def test_a_preset_sets_its_fields_and_follows_them(self):
        self.assertEqual(fields.follows("preset", "draft"), runner.preset_values("draft"))
        self.assertEqual(fields.follows("preset", runner.CUSTOM), {})
        self.assertEqual(fields.follows("scope", "finish"), {})
        values = dataclasses.asdict(runner.RouteOptions())
        self.assertEqual(fields.derived(values), {"preset": "final"})
        values.update(fields.follows("preset", "quick"))
        self.assertEqual(fields.derived(values), {"preset": "quick"})
        values["geometry_generations"] = 2
        self.assertEqual(fields.derived(values), {"preset": runner.CUSTOM})

    def test_fields_are_on_where_they_mean_something(self):
        def on(name, **values):
            return fields.enabled(self.field(name), values)
        for name in ("pair_skew", "pair_zdiff", "pair_all", "via_pairs", "breakouts"):
            self.assertFalse(on(name, pairs="off"), name)
            self.assertFalse(on(name, pairs="pull"), name)
            self.assertTrue(on(name, pairs="band"), name)
        self.assertFalse(on("pair_pull", pairs="off"))
        self.assertTrue(on("pair_pull", pairs="pull"))
        self.assertTrue(on("pair_pull", pairs="band"))
        for scope, single in (("fresh", False), ("keep_locked", False), ("finish", True),
                              ("reroute", True)):
            self.assertEqual(on("nets", scope=scope), single, scope)
            for name in ("move_parts", "placements", "clear_corridors"):
                self.assertEqual(on(name, scope=scope), not single, f"{name} {scope}")
        # as before: a check box or a number that is not 0
        self.assertTrue(on("open_via_cost", until_stop=True))
        self.assertFalse(on("placement_seed", placements=0))
        self.assertTrue(on("scope"))


class PlacementsCommandTest(unittest.TestCase):
    def cmd(self, **kw):
        return runner.build_command("/bin/gb", Path("a.kicad_pcb"), Path("b.kicad_pcb"),
                                    runner.RouteOptions(**kw))

    def test_off_by_default(self):
        self.assertNotIn("--placements", self.cmd())

    def test_placements_with_fixed_parts_and_report(self):
        c = self.cmd(placements=6, fixed=["U1"], report="r.json")
        self.assertEqual(c[c.index("--placements") + 1], "6")
        self.assertEqual(c[c.index("--fixed") + 1], "U1")
        self.assertEqual(c[c.index("--report") + 1], "r.json")
        self.assertNotIn("--move-parts", c)


class PlaceCommandTest(unittest.TestCase):
    def cmd(self, **kw):
        return runner.build_place_command("/bin/gb", Path("a.kicad_pcb"), Path("b.kicad_pcb"),
                                          Path("r.json"), runner.PlaceOptions(**kw))

    def test_default_scope_has_no_flag(self):
        c = self.cmd()
        self.assertEqual(c[:6], ["/bin/gb", "place", "a.kicad_pcb", "-o", "b.kicad_pcb", "--report"])
        for flag in ("--only", "--outside", "--all-unlocked", "--fixed"):
            self.assertNotIn(flag, c)

    def test_scopes(self):
        c = self.cmd(scope="selected", only=["C1", "R2"])
        self.assertEqual(c[c.index("--only") + 1], "C1,R2")
        self.assertIn("--outside", self.cmd(scope="outside"))
        self.assertIn("--all-unlocked", self.cmd(scope="all"))

    def test_fixed(self):
        c = self.cmd(fixed=["U1", "U2"])
        self.assertEqual(c[c.index("--fixed") + 1], "U1,U2")

    def test_parse_references(self):
        self.assertEqual(runner.parse_references(" U1, C2  R3,"), ["U1", "C2", "R3"])
        self.assertEqual(runner.parse_references(""), [])


if __name__ == "__main__":
    unittest.main()
