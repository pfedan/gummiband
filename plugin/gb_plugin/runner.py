"""Locate and run the ``gummiband`` command line router."""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import tempfile
import threading
from dataclasses import MISSING, dataclass, fields
from pathlib import Path
from typing import Callable, List, Mapping, Optional, Sequence, Tuple

PLUGIN_DIR = Path(__file__).resolve().parent.parent
SETTINGS_FILE = PLUGIN_DIR / "settings.json"
ENV_VAR = "GUMMIBAND_BIN"

# KiCad started from the macOS Finder gets a minimal PATH; look in the usual places too.
EXTRA_DIRS = ("~/.cargo/bin", "/opt/homebrew/bin", "/usr/local/bin")


class RunnerError(RuntimeError):
    pass


@dataclass
class RouteOptions:
    keep_locked: bool = True
    runs: int = 14  # fixed, so that results do not depend on the machine
    via_cost: float = 3.0
    # generations that select by the clearance check after the geometry step (0 = fastest)
    geometry_generations: int = 3
    # folder for one image per generation and an animated evolution.gif (None = no images)
    frames_dir: Optional[str] = None
    # finish started tracks: route only these nets and add to the copper that is there
    finish_nets: Optional[List[str]] = None
    # whether the dialog asks for the animation; frames_dir is what the router gets
    animate: bool = False
    # let the geometry generations shift parts (the router's default); the plugin takes the
    # moves over
    move_parts: bool = True
    fixed: Optional[List[str]] = None  # references that never move
    # try this many placements from the netlist, each routed; the best one is kept
    placements: int = 0
    # the router's JSON report; the plugin reads the placement from it
    report: Optional[str] = None
    # open KiCad's Edit Teardrops after the run: the API cannot set them on vias and pads
    teardrops: bool = False
    # the defaults of the advanced options are those of the command line
    generations: int = 3
    via_pitch: float = 0.5
    layer_dirs: float = 0.5
    via_pass: float = 4.0
    negotiate: int = 0
    threads: int = 0  # 0 = all cores
    plane_layers: Optional[List[str]] = None
    net_widths: Optional[str] = None  # JSON file
    net_priorities: Optional[str] = None  # JSON file
    pairs: bool = False
    pair_pull: float = 0.0
    pair_skew: float = 0.1
    pair_zdiff: float = 0.0  # 0 = keep the track widths of the net class
    # keep improving after the normal search until the user accepts the result
    until_stop: bool = True
    prefer_width: float = 0.0  # tracks of the default class grow up to this where there is room
    gpu: bool = False  # sets GB_GPU for the router, needs a binary built with the gpu feature
    # turn the result into conventional 45 degree routing as a last step
    angles_45: bool = False


@dataclass
class PlaceOptions:
    # which parts move: "unlocked", "all", "outside" or "selected"
    scope: str = "unlocked"
    # the references of the scope "selected"
    only: Optional[List[str]] = None
    # references that stay in any case
    fixed: Optional[List[str]] = None
    starts: int = 1  # the crossings hardly predict the routed result: change the seed instead
    gap: float = 0.3
    halo: float = 0.3
    # the defaults of the advanced options are those of the command line
    seed: int = 1
    iters: int = 3_000_000
    margin: float = 0.5
    free: Optional[List[str]] = None  # references released although they count as mechanical
    threads: int = 0  # 0 = all cores
    plane_layers: Optional[List[str]] = None


# Options the dialogs remember in settings.json. What only makes sense on one board (references,
# layer names, net files) starts empty every time.
ROUTE_SAVED = ("keep_locked", "runs", "via_cost", "geometry_generations", "animate", "move_parts", "placements",
               "teardrops", "generations", "via_pitch", "layer_dirs", "via_pass", "negotiate", "threads", "pairs", "pair_pull",
               "pair_skew", "pair_zdiff", "until_stop", "prefer_width", "gpu",
               "angles_45")
PLACE_SAVED = ("starts", "gap", "halo", "seed", "iters", "margin", "threads")

# Options of the command line that are left out at their default value.
ROUTE_FLAGS = (("generations", "--generations"), ("via_pitch", "--via-pitch"),
               ("layer_dirs", "--layer-dirs"), ("via_pass", "--via-pass"),
               ("negotiate", "--negotiate"), ("pair_pull", "--pair-pull"),
               ("placements", "--placements"), ("prefer_width", "--prefer-width"))
PAIR_FLAGS = (("pair_skew", "--pair-skew"), ("pair_zdiff", "--pair-zdiff"))
PLACE_FLAGS = (("seed", "--seed"), ("iters", "--iters"), ("margin", "--margin"))


def load_settings(path: Path = SETTINGS_FILE) -> dict:
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
    except FileNotFoundError:
        return {}
    except (OSError, ValueError) as e:
        raise RunnerError(f"cannot read {path}: {e}") from e
    if not isinstance(data, dict):
        raise RunnerError(f"{path}: expected a JSON object")
    return data


def _default(cls, name: str):
    return next(f.default for f in fields(cls) if f.name == name and f.default is not MISSING)


def load_options(cls, section: str, saved: tuple, path: Path = SETTINGS_FILE) -> dict:
    """The values of ``saved`` that the dialog of ``cls`` stored under ``section``, as keyword
    arguments. Unknown keys, values of another type and an unreadable file are ignored: a
    bad setting must not keep the dialog from opening."""
    try:
        stored = load_settings(path).get(section)
    except RunnerError:
        return {}
    if not isinstance(stored, dict):
        return {}
    out = {}
    for name in saved:
        value = stored.get(name)
        kind = type(_default(cls, name))
        if kind is float and type(value) is int:
            value = float(value)
        if type(value) is kind:
            out[name] = value
    return out


def save_options(section: str, opts, saved: tuple, path: Path = SETTINGS_FILE) -> bool:
    """Store the values of ``saved`` under ``section`` of the settings file and leave the rest
    of it alone. A file that cannot be read or written stays as it is: losing the remembered
    values is no reason to stop a run."""
    try:
        data = load_settings(path)
    except RunnerError:
        return False
    data[section] = {name: getattr(opts, name) for name in saved}
    try:
        with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=path.parent, suffix=".tmp",
                                         delete=False) as f:
            json.dump(data, f, indent=4)
            f.write("\n")
        os.replace(f.name, path)
    except OSError:
        return False
    return True


def _executable(p: str) -> Optional[str]:
    p = os.path.expanduser(p)
    if os.path.isfile(p) and os.access(p, os.X_OK):
        return p
    return None


def find_binary(env: Mapping[str, str] = os.environ, settings_path: Path = SETTINGS_FILE,
                extra_dirs=EXTRA_DIRS) -> str:
    """$GUMMIBAND_BIN, then "gummiband_bin" in settings.json, then PATH (and a few usual
    install directories)."""
    tried = []
    v = env.get(ENV_VAR)
    if v:
        found = _executable(v)
        if found:
            return found
        tried.append(f"${ENV_VAR}={v} (not an executable file)")
    s = load_settings(settings_path).get("gummiband_bin")
    if s:
        found = _executable(str(s))
        if found:
            return found
        tried.append(f"{settings_path}: gummiband_bin={s} (not an executable file)")
    found = shutil.which("gummiband", path=env.get("PATH", os.defpath))
    if found:
        return found
    for d in extra_dirs:
        found = _executable(os.path.join(os.path.expanduser(d), "gummiband"))
        if found:
            return found
    tried.append("PATH and " + ", ".join(extra_dirs))
    raise RunnerError(
        "gummiband binary not found. Set GUMMIBAND_BIN, or put "
        f'{{"gummiband_bin": "/path/to/gummiband"}} into {settings_path}.\nTried: '
        + "; ".join(tried))


def _changed(opts, flags: tuple) -> List[str]:
    """The flags whose option differs from its default."""
    cmd: List[str] = []
    for name, flag in flags:
        value = getattr(opts, name)
        if value != _default(type(opts), name):
            cmd += [flag, repr(value) if isinstance(value, float) else str(value)]
    return cmd


def open_ended(opts: RouteOptions) -> bool:
    """Whether the run goes on until the user accepts it: not with the wide tracks of pairs, and
    not for finishing tracks, where only the nets asked for may change."""
    return opts.until_stop and not opts.pairs and not opts.finish_nets


def _global_flags(opts) -> List[str]:
    cmd: List[str] = []
    if opts.threads > 0:
        cmd += ["--threads", str(int(opts.threads))]
    if opts.plane_layers:
        cmd += ["--plane-layers", ",".join(opts.plane_layers)]
    return cmd


def build_command(binary: str, board: Path, output: Path, opts: RouteOptions) -> List[str]:
    cmd = [binary, "route", str(board), "-o", str(output), "--progress"]
    if opts.finish_nets:
        cmd += ["--keep-all", "--nets", ",".join(opts.finish_nets), "--shove", "--give", "8"]
    elif opts.keep_locked:
        cmd.append("--keep-locked")
    # always passed, so that 0 stays 0 whatever the default of the router is
    cmd += ["--runs", str(int(opts.runs)), "--via-cost", repr(float(opts.via_cost)),
            "--geometry-generations", str(int(opts.geometry_generations))]
    cmd += _changed(opts, ROUTE_FLAGS)
    # parts move by default when the whole board is routed; finishing nets keeps them in place
    if not opts.move_parts and not opts.finish_nets:
        cmd.append("--no-move-parts")
    if ((opts.move_parts and not opts.finish_nets) or opts.placements) and opts.fixed:
        cmd += ["--fixed", ",".join(opts.fixed)]
    if opts.pairs:
        cmd.append("--pairs")
        cmd += _changed(opts, PAIR_FLAGS)
    if opts.net_widths:
        cmd += ["--net-widths", str(opts.net_widths)]
    if opts.net_priorities:
        cmd += ["--net-priorities", str(opts.net_priorities)]
    if opts.angles_45:
        cmd += ["--angles", "45"]
    if open_ended(opts):
        cmd.append("--until-stop")
    if opts.frames_dir:
        cmd += ["--frames", str(opts.frames_dir)]
    if opts.report:
        cmd += ["--report", str(opts.report)]
    return cmd + _global_flags(opts)


def build_angles_command(binary: str, board: Path, output: Path, nets: Sequence[str] = (),
                         opts: Optional[RouteOptions] = None) -> List[str]:
    """``gummiband angles45``: the unlocked tracks of ``board`` (of ``nets``, or all) on the 45
    degree grid, without routing again. ``opts`` gives the global flags (threads, planes)."""
    cmd = [binary, "angles45", str(board), "-o", str(output)]
    if nets:
        cmd += ["--nets", ",".join(nets)]
    return cmd + _global_flags(opts or RouteOptions())


def angles_line(stdout: str) -> Optional[str]:
    """The summary line of ``gummiband angles45``."""
    m = re.search(r"^45 degrees: .*$", stdout, re.M)
    return m.group(0) if m else None


def build_place_command(binary: str, board: Path, output: Path, report: Path,
                        opts: PlaceOptions) -> List[str]:
    cmd = [binary, "place", str(board), "-o", str(output), "--report", str(report),
           "--starts", str(int(opts.starts)), "--gap", repr(float(opts.gap)),
           "--halo", repr(float(opts.halo))]
    if opts.scope == "selected":
        cmd += ["--only", ",".join(opts.only or [])]
    elif opts.scope == "outside":
        cmd.append("--outside")
    elif opts.scope == "all":
        cmd.append("--all-unlocked")
    if opts.fixed:
        cmd += ["--fixed", ",".join(opts.fixed)]
    if opts.free:
        cmd += ["--free", ",".join(opts.free)]
    return cmd + _changed(opts, PLACE_FLAGS) + _global_flags(opts)


def parse_references(text: str) -> List[str]:
    """References from a text field: separated by commas or spaces."""
    return [r for r in re.split(r"[,\s]+", text.strip()) if r]


def tool_env(env: Mapping[str, str] = os.environ, extra_dirs=EXTRA_DIRS) -> dict:
    """The environment for the router: PATH extended by the usual install directories, so
    that it finds rsvg-convert and magick for the animation when KiCad was started from the
    Finder."""
    out = dict(env)
    parts = [p for p in out.get("PATH", os.defpath).split(os.pathsep) if p]
    for d in extra_dirs:
        d = os.path.expanduser(d)
        if d not in parts:
            parts.append(d)
    out["PATH"] = os.pathsep.join(parts)
    return out


@dataclass
class RouteResult:
    cmd: List[str]
    returncode: int
    stdout: str
    stderr: str

    def best_line(self) -> Optional[str]:
        m = re.search(r"^best: .*$", self.stdout, re.M)
        return m.group(0) if m else None

    def unrouted(self) -> List[str]:
        return re.findall(r"^\s*unrouted: (.*)$", self.stdout, re.M)

    def gpu_missing(self) -> bool:
        return re.search(r"^warning: GB_GPU", self.stdout, re.M) is not None

    def animation(self) -> Optional[str]:
        m = re.search(r"^frames: \d+ images, animation (.*)$", self.stdout, re.M)
        return m.group(1).strip() if m else None

    def alternatives(self) -> List[Tuple[str, str]]:
        """(file, summary) of the results the open end leaves to choose from."""
        return re.findall(r"^alternative: (.*): (\d+/\d+ routed.*)$", self.stdout, re.M)

    def violations(self) -> Optional[int]:
        m = re.search(r"^clearance check: (\d+) violations", self.stdout, re.M)
        return int(m.group(1)) if m else None


def parse_progress(line: str) -> Optional[dict]:
    """The JSON of a ``progress {...}`` line of the router, or None."""
    if not line.startswith("progress "):
        return None
    try:
        p = json.loads(line[len("progress "):])
    except ValueError:
        return None
    return p if isinstance(p, dict) else None


def progress_text(p: Optional[dict]) -> str:
    """One line for a progress dialog, e.g. "geometry generation 2: 347/348 routed, 261 vias,
    2585 mm, 3 clearance violations"."""
    if not p:
        return "Routing..."
    text = (f"{p.get('phase', '')}: {p.get('routed', 0)}/{p.get('conns', 0)} routed, "
            f"{p.get('vias', 0)} vias, {p.get('length', 0):.0f} mm")
    if p.get("violations") is not None:
        text += f", {p['violations']} clearance violations"
    return text


def converged(p: Optional[dict]) -> bool:
    """The open end has found its level, see ``--until-stop``."""
    return bool(p) and bool(p.get("converged"))


def run_route(cmd: List[str], output: Path, cwd: Optional[Path] = None,
              keep_going: Optional[Callable[[Optional[dict]], bool]] = None,
              accept: Optional[Callable[[], bool]] = None, gpu: bool = False) -> RouteResult:
    """Run the router and wait for it. ``keep_going(progress)`` is polled about ten times a
    second while the router runs, with the latest progress report (see ``parse_progress``);
    returning False kills the router. With ``--until-stop`` the router runs until ``accept()``
    returns True, then it finishes its best result. ``gpu`` sets ``GB_GPU`` for the router."""
    try:
        proc = subprocess.Popen(cmd, cwd=cwd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, text=True, errors="replace",
                                env=dict(tool_env(), GB_GPU="1") if gpu else tool_env())
    except OSError as e:
        raise RunnerError(f"cannot start {cmd[0]}: {e}") from e
    out: dict = {"stdout": [], "progress": None}

    def read_stdout():
        for line in proc.stdout:
            p = parse_progress(line)
            if p is not None:
                out["progress"] = p
            else:
                out["stdout"].append(line)

    def read_stderr():
        out["stderr"] = proc.stderr.read()

    readers = [threading.Thread(target=f, daemon=True) for f in (read_stdout, read_stderr)]
    for t in readers:
        t.start()
    cancelled = False
    accepted = False
    while proc.poll() is None:
        readers[0].join(0.1)
        if keep_going is not None and not cancelled and not keep_going(out["progress"]):
            cancelled = True
            proc.kill()
        if accept is not None and not accepted and accept():
            accepted = True
            try:
                proc.stdin.close()  # the router reads this as the end of the run
            except OSError:
                pass
    for t in readers:
        t.join()
    proc.wait()
    if cancelled:
        raise RouteCancelled("routing cancelled")
    res = RouteResult(cmd, proc.returncode, "".join(out["stdout"]), out.get("stderr", ""))
    if res.returncode != 0 or not output.is_file():
        why = (f"exit code {res.returncode}" if res.returncode != 0
               else f"no output file {output}")
        raise RouteFailed(res, why)
    return res


class RouteCancelled(RunnerError):
    pass


class RouteFailed(RunnerError):
    def __init__(self, result: RouteResult, why: str):
        self.result = result
        super().__init__(
            f"gummiband failed ({why}).\n\nCommand: {' '.join(result.cmd)}\n\n"
            f"stdout:\n{tail(result.stdout)}\n\nstderr:\n{tail(result.stderr)}")


def tail(text: str, lines: int = 40) -> str:
    rows = text.rstrip().splitlines()
    if len(rows) <= lines:
        return "\n".join(rows) or "(empty)"
    return "...\n" + "\n".join(rows[-lines:])
