import json
import os
import stat
import sys
import tempfile
import unittest
from pathlib import Path

from gb_plugin import fields, runner

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
        opts = runner.RouteOptions(keep_locked=True, runs=8, via_cost=2, geometry_generations=0,
                                   until_stop=False)
        self.assertEqual(
            runner.build_command("gb", Path("/a/b.kicad_pcb"), Path("/t/o.kicad_pcb"), opts),
            ["gb", "route", "/a/b.kicad_pcb", "-o", "/t/o.kicad_pcb", "--progress", "--keep-locked",
             "--runs", "8", "--via-cost", "2.0", "--geometry-generations", "0"])
        opts.keep_locked = False
        self.assertNotIn("--keep-locked", runner.build_command("gb", Path("b"), Path("o"), opts))

    def test_finish_nets(self):
        opts = runner.RouteOptions(geometry_generations=0, finish_nets=["A", "B"])
        cmd = runner.build_command("gb", Path("b"), Path("o"), opts)
        self.assertIn("--keep-all", cmd)
        self.assertIn("--shove", cmd)
        self.assertEqual(cmd[cmd.index("--nets") + 1], "A,B")
        self.assertNotIn("--keep-locked", cmd)

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
        self.assertTrue(o.keep_locked)
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
        finish = runner.RouteOptions(finish_nets=["A"])
        self.assertNotIn("--until-stop", runner.build_command("gb", Path("b"), Path("o"), finish))
        pairs = runner.RouteOptions(until_stop=True, pairs=True)
        self.assertNotIn("--until-stop", runner.build_command("gb", Path("b"), Path("o"), pairs))

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
                                   runner.RouteOptions(geometry_generations=0, keep_locked=False,
                                                       until_stop=False))
        self.assertEqual(cmd, ["gb", "route", "b", "-o", "o", "--progress",
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
        opts = runner.RouteOptions(fixed=["U1", "U2"])
        self.assertNotIn("--fixed", runner.build_command("gb", Path("b"), Path("o"), opts))
        opts.move_parts = True
        cmd = runner.build_command("gb", Path("b"), Path("o"), opts)
        self.assertIn("--move-parts", cmd)
        self.assertEqual(cmd[cmd.index("--fixed") + 1], "U1,U2")

    def test_pair_options_need_pairs(self):
        opts = runner.RouteOptions(pair_skew=0.2, pair_zdiff=90.0, pair_pull=1.0)
        cmd = runner.build_command("gb", Path("b"), Path("o"), opts)
        self.assertNotIn("--pairs", cmd)
        self.assertNotIn("--pair-skew", cmd)
        self.assertNotIn("--pair-zdiff", cmd)
        self.assertIn("--pair-pull", cmd)
        opts.pairs = True
        cmd = runner.build_command("gb", Path("b"), Path("o"), opts)
        self.assertIn("--pairs", cmd)
        self.assertEqual(cmd[cmd.index("--pair-zdiff") + 1], "90.0")

    def test_place(self):
        opts = runner.PlaceOptions(seed=7, iters=1_000_000, margin=1.0, free=["TP1", "SW1"],
                                   threads=2, plane_layers=["In1.Cu"])
        cmd = runner.build_place_command("gb", Path("a"), Path("b"), Path("r"), opts)
        for flag, value in (("--seed", "7"), ("--iters", "1000000"), ("--margin", "1.0"),
                            ("--free", "TP1,SW1"), ("--threads", "2"),
                            ("--plane-layers", "In1.Cu")):
            self.assertEqual(cmd[cmd.index(flag) + 1], value)


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
        opts = runner.RouteOptions(runs=8, via_cost=2.5, pairs=True, plane_layers=["In1.Cu"])
        self.assertTrue(runner.save_options("route", opts, runner.ROUTE_SAVED, self.path))
        stored = runner.load_settings(self.path)
        self.assertEqual(stored["gummiband_bin"], "/x/gb")
        self.assertNotIn("plane_layers", stored["route"])
        loaded = runner.RouteOptions(**self.load())
        self.assertEqual((loaded.runs, loaded.via_cost, loaded.pairs), (8, 2.5, True))
        self.assertIsNone(loaded.plane_layers)

    def test_no_file(self):
        self.assertEqual(self.load(), {})

    def test_bad_values_are_ignored(self):
        self.path.write_text(json.dumps({"route": {
            "runs": "many", "pairs": 1, "via_cost": 2, "keep_locked": False, "gone": 1}}))
        self.assertEqual(self.load(), {"via_cost": 2.0, "keep_locked": False})

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
            table = fields.all_fields(tabs)
            names = [f.name for f in table]
            self.assertEqual(len(names), len(set(names)))
            self.assertLessEqual(set(saved), set(names))
            opts = cls()
            for f in table:
                self.assertTrue(f.tip, f.name)
                if f.kind in ("int", "float"):
                    self.assertTrue(f.low <= getattr(opts, f.name) <= f.high, f.name)
                if f.requires is not None:
                    self.assertIn(f.requires, names)
                if f.kind in ("refs", "file"):
                    self.assertNotIn(f.name, saved)


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
