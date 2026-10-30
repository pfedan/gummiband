import json
import os
import stat
import sys
import tempfile
import unittest
from pathlib import Path

from gb_plugin import runner

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
        opts = runner.RouteOptions(keep_locked=True, runs=8, via_cost=2, geometry_generations=0)
        self.assertEqual(
            runner.build_command("gb", Path("/a/b.kicad_pcb"), Path("/t/o.kicad_pcb"), opts),
            ["gb", "route", "/a/b.kicad_pcb", "-o", "/t/o.kicad_pcb", "--progress", "--keep-locked",
             "--runs", "8", "--via-cost", "2.0"])
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
        opts = runner.RouteOptions(geometry_generations=3, frames_dir="/p/anim")
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


class TailTest(unittest.TestCase):
    def test_tail(self):
        self.assertEqual(runner.tail(""), "(empty)")
        self.assertEqual(runner.tail("a\nb\n", 5), "a\nb")
        self.assertEqual(runner.tail("1\n2\n3\n4", 2), "...\n3\n4")


if __name__ == "__main__":
    unittest.main()
