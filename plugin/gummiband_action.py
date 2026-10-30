"""KiCad IPC API action "Route with Gummiband".

KiCad starts this script with the plugin's Python environment and sets KICAD_API_SOCKET and
KICAD_API_TOKEN. Steps: save a copy of the open board (with its project) into a temp folder,
ask for options, run ``gummiband route`` on the copy, then replace the tracks and vias of the open
board with the routed ones in one commit (one undo step).
"""

from __future__ import annotations

import dataclasses
import shutil
import sys
import tempfile
import time
import traceback
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path(__file__).resolve().parent))

from gb_plugin import board_file as bf  # noqa: E402
from gb_plugin import runner  # noqa: E402

try:
    import wx
except ImportError:  # e.g. Linux system Python without wxPython: run with defaults
    wx = None

TITLE = "Route with Gummiband"
API_TIMEOUT_MS = 120_000  # creating thousands of items in one commit takes a while


# User interface ----------------------------------------------------------------------------


class ConsoleUi:
    """No wx: defaults, messages on stdout/stderr (KiCad shows plugin output in its status
    bar since 10.0.1)."""

    def ask_options(self, defaults: runner.RouteOptions) -> Optional[runner.RouteOptions]:
        return dataclasses.replace(defaults, frames_dir=None)

    def run_with_progress(self, fn, busy: str = "Routing..."):
        return fn(None)

    def info(self, text: str):
        print(text)

    def error(self, text: str):
        print(text, file=sys.stderr)


class WxUi:
    def __init__(self, title: str = TITLE):
        self.app = wx.App(False)
        self.title = title

    def ask_options(self, defaults: runner.RouteOptions) -> Optional[runner.RouteOptions]:
        """``defaults.frames_dir`` is where the animation would go; the check box decides
        whether it is made."""
        dlg = wx.Dialog(None, title=TITLE)
        keep = wx.CheckBox(dlg, label="Keep locked tracks and vias")
        keep.SetValue(defaults.keep_locked)
        runs = wx.SpinCtrl(dlg, min=1, max=1024, initial=int(defaults.runs))
        cost = wx.SpinCtrlDouble(dlg, min=0.0, max=1000.0, initial=float(defaults.via_cost),
                                 inc=0.1)
        cost.SetDigits(2)
        quality = wx.SpinCtrl(dlg, min=0, max=20, initial=int(defaults.geometry_generations))
        frames = wx.CheckBox(dlg, label="Save an animation of the generations")
        frames.SetToolTip(f"One image per generation and evolution.gif in {defaults.frames_dir}")
        note = wx.StaticText(dlg, label=(
            "All unlocked tracks and vias are replaced by the routed ones.\n"
            "Without \"keep locked\", locked ones are replaced as well."))
        grid = wx.FlexGridSizer(3, 2, 8, 12)
        grid.Add(wx.StaticText(dlg, label="Parallel runs:"), 0, wx.ALIGN_CENTER_VERTICAL)
        grid.Add(runs, 0, wx.EXPAND)
        grid.Add(wx.StaticText(dlg, label="Via cost (mm of track):"), 0, wx.ALIGN_CENTER_VERTICAL)
        grid.Add(cost, 0, wx.EXPAND)
        grid.Add(wx.StaticText(dlg, label="Geometry generations:"), 0, wx.ALIGN_CENTER_VERTICAL)
        grid.Add(quality, 0, wx.EXPAND)
        box = wx.BoxSizer(wx.VERTICAL)
        box.Add(keep, 0, wx.ALL, 12)
        box.Add(grid, 0, wx.LEFT | wx.RIGHT | wx.EXPAND, 12)
        box.Add(frames, 0, wx.LEFT | wx.RIGHT | wx.TOP, 12)
        box.Add(note, 0, wx.ALL, 12)
        box.Add(dlg.CreateStdDialogButtonSizer(wx.OK | wx.CANCEL), 0, wx.ALL | wx.EXPAND, 12)
        dlg.SetSizerAndFit(box)
        dlg.CentreOnScreen()
        try:
            if dlg.ShowModal() != wx.ID_OK:
                return None
            return runner.RouteOptions(keep.GetValue(), runs.GetValue(), cost.GetValue(),
                                       quality.GetValue(),
                                       defaults.frames_dir if frames.GetValue() else None)
        finally:
            dlg.Destroy()

    def run_with_progress(self, fn, busy: str = "Routing..."):
        """Call ``fn(keep_going)`` while a progress dialog pulses; Cancel stops the router."""
        # the first message sets the dialog width: as long as a typical progress line
        dlg = wx.ProgressDialog(self.title, "geometry generation 3: 1000/1000 routed, 1000 vias, "
                                "10000 mm, 100 clearance violations\n0 s", maximum=100,
                                style=wx.PD_APP_MODAL | wx.PD_CAN_ABORT | wx.PD_ELAPSED_TIME)
        t0 = time.monotonic()

        def keep_going(progress=None) -> bool:
            line = runner.progress_text(progress) if progress else busy
            cont, _skip = dlg.Pulse(f"{line}\n"
                                    f"{time.monotonic() - t0:.0f} s")
            return cont

        try:
            return fn(keep_going)
        finally:
            dlg.Destroy()

    def info(self, text: str):
        wx.MessageBox(text, self.title, wx.OK | wx.ICON_INFORMATION)

    def error(self, text: str):
        if text.count("\n") < 12:
            wx.MessageBox(text, self.title, wx.OK | wx.ICON_ERROR)
            return
        # long output (router stdout/stderr): scrollable, selectable text
        dlg = wx.Dialog(None, title=self.title, style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER)
        txt = wx.TextCtrl(dlg, value=text, size=(760, 460),
                          style=wx.TE_MULTILINE | wx.TE_READONLY | wx.HSCROLL)
        txt.SetFont(wx.Font(wx.FontInfo(11).Family(wx.FONTFAMILY_TELETYPE)))
        box = wx.BoxSizer(wx.VERTICAL)
        box.Add(txt, 1, wx.ALL | wx.EXPAND, 12)
        box.Add(dlg.CreateStdDialogButtonSizer(wx.OK), 0, wx.ALL | wx.EXPAND, 12)
        dlg.SetSizerAndFit(box)
        dlg.CentreOnScreen()
        dlg.ShowModal()
        dlg.Destroy()


# Steps -------------------------------------------------------------------------------------


def save_copy(board, folder: Path) -> Path:
    """Save the open board (unsaved edits included) and its project into ``folder``; the open
    document and its files stay untouched."""
    stem = Path(board.name).stem or "board"
    dst = folder / f"{stem}.kicad_pcb"
    board.save_as(str(dst), overwrite=True, include_project=True)
    if not dst.is_file():
        raise RuntimeError(f"KiCad did not write the board copy {dst}")
    # The router reads net classes from the .kicad_pro next to the board. KiCad writes it with
    # include_project; fall back to copying the project's files if it did not.
    proj = board.document.project
    for ext in (".kicad_pro", ".kicad_dru"):
        want = folder / f"{stem}{ext}"
        src = Path(proj.path) / f"{proj.name}{ext}"
        if not want.exists() and proj.path and src.is_file():
            shutil.copyfile(src, want)
    return dst


def report(plan: bf.ImportPlan, applied, res: runner.RouteResult) -> str:
    lines = [
        f"Removed: {bf.format_counts(applied.removed)}",
        f"Created: {bf.format_counts(applied.created)}",
    ]
    if plan.keep and not plan.add_only:
        lines.append(f"Kept locked: {bf.format_counts(plan.summary()['keep'])}")
    best = res.best_line()
    if best:
        lines.append("")
        lines.append("Router " + best)
    viol = res.violations()
    if viol:
        lines.append(f"Clearance check of the router: {viol} violations (run DRC)")
    unrouted = res.unrouted()
    if unrouted:
        lines.append(f"Unrouted ({len(unrouted)}{'+' if len(unrouted) >= 40 else ''}):")
        lines += [f"  {u}" for u in unrouted[:10]]
        if len(unrouted) > 10:
            lines.append("  ...")
    gif = res.animation()
    if gif:
        lines.append(f"Animation of the generations: {gif}")
    if applied.unknown_nets:
        lines.append(f"Nets not found on the board: {', '.join(sorted(applied.unknown_nets))}")
    lines += [f"Warning: {w}" for w in plan.warnings[:5]]
    return "\n".join(lines)


def main() -> int:
    ui = WxUi() if wx is not None else ConsoleUi()
    try:
        from kipy import KiCad
        from gb_plugin import kicad_apply
    except ImportError as e:
        ui.error(f"kicad-python is missing in the plugin environment ({e}).\n"
                 "Use \"Recreate Plugin Environment\" in Preferences > Plugins.")
        return 1

    try:
        kicad = KiCad(timeout_ms=API_TIMEOUT_MS)
        board = kicad.get_board()
    except Exception as e:
        ui.error(f"Cannot reach the open board through the KiCad API: {e}\n\n"
                 "Is the API enabled (Preferences > Plugins) and a board open in the PCB "
                 "editor?")
        return 1

    # the animation goes next to the project (the router itself works in a temp folder)
    frames_default = None
    try:
        proj = board.document.project
        if proj.path:
            frames_default = str(Path(proj.path) / "gummiband-evolution")
    except Exception:
        pass
    opts = ui.ask_options(runner.RouteOptions(
        frames_dir=frames_default or str(Path(tempfile.gettempdir()) / "gummiband-evolution")))
    if opts is None:
        return 0

    work = Path(tempfile.mkdtemp(prefix="gummiband-"))
    try:
        binary = runner.find_binary()
        src = save_copy(board, work)
        out = work / f"{src.stem}-routed.kicad_pcb"
        cmd = runner.build_command(binary, src, out, opts)
        res = ui.run_with_progress(lambda keep_going: runner.run_route(cmd, out, work, keep_going))
        plan = bf.plan_from_texts(src.read_text(encoding="utf-8"),
                                  out.read_text(encoding="utf-8"), opts.keep_locked)
        applied = kicad_apply.apply_plan(board, plan)
    except runner.RouteCancelled:
        shutil.rmtree(work, ignore_errors=True)
        return 0
    except runner.RunnerError as e:
        ui.error(f"{e}\n\nFiles kept in {work}")
        return 1
    except Exception:
        ui.error(f"Routing failed:\n\n{traceback.format_exc()}\nFiles kept in {work}")
        return 1
    shutil.rmtree(work, ignore_errors=True)
    ui.info(report(plan, applied, res))
    return 0


if __name__ == "__main__":
    sys.exit(main())
