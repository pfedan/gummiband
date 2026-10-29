"""Locate and run the ``gummiband`` command line router."""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, List, Mapping, Optional

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
    runs: int = os.cpu_count() or 1
    via_cost: float = 3.0
    # generations that select by the clearance check after the geometry step (0 = fastest)
    geometry_generations: int = 3
    # folder for one image per generation and an animated evolution.gif (None = no images)
    frames_dir: Optional[str] = None


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


def build_command(binary: str, board: Path, output: Path, opts: RouteOptions) -> List[str]:
    cmd = [binary, "route", str(board), "-o", str(output)]
    if opts.keep_locked:
        cmd.append("--keep-locked")
    cmd += ["--runs", str(int(opts.runs)), "--via-cost", repr(float(opts.via_cost))]
    if opts.geometry_generations > 0:
        cmd += ["--geometry-generations", str(int(opts.geometry_generations))]
    if opts.frames_dir:
        cmd += ["--frames", str(opts.frames_dir)]
    return cmd


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

    def animation(self) -> Optional[str]:
        m = re.search(r"^frames: \d+ images, animation (.*)$", self.stdout, re.M)
        return m.group(1).strip() if m else None

    def violations(self) -> Optional[int]:
        m = re.search(r"^clearance check: (\d+) violations", self.stdout, re.M)
        return int(m.group(1)) if m else None


def run_route(cmd: List[str], output: Path, cwd: Optional[Path] = None,
              keep_going: Optional[Callable[[], bool]] = None) -> RouteResult:
    """Run the router and wait for it. ``keep_going`` is polled about ten times a second while
    the router runs (to update a progress dialog); returning False kills the router."""
    try:
        proc = subprocess.Popen(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                text=True, errors="replace", env=tool_env())
    except OSError as e:
        raise RunnerError(f"cannot start {cmd[0]}: {e}") from e
    out: dict = {}

    def wait():
        out["stdout"], out["stderr"] = proc.communicate()

    waiter = threading.Thread(target=wait, daemon=True)
    waiter.start()
    cancelled = False
    while waiter.is_alive():
        waiter.join(0.1)
        if keep_going is not None and not cancelled and not keep_going():
            cancelled = True
            proc.kill()
    waiter.join()
    if cancelled:
        raise RouteCancelled("routing cancelled")
    res = RouteResult(cmd, proc.returncode, out.get("stdout", ""), out.get("stderr", ""))
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
