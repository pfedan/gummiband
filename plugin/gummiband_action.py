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
from gb_plugin import fields, runner  # noqa: E402

try:
    import wx
except ImportError:  # e.g. Linux system Python without wxPython: run with defaults
    wx = None

TITLE = "Route with Gummiband"
API_TIMEOUT_MS = 120_000  # creating thousands of items in one commit takes a while


# User interface ----------------------------------------------------------------------------


class ConsoleUi:
    """No wx: the given options, messages on stdout/stderr (KiCad shows plugin output in its
    status bar since 10.0.1)."""

    def ask_options(self, defaults: runner.RouteOptions) -> Optional[runner.RouteOptions]:
        return dataclasses.replace(defaults, frames_dir=None)

    def run_with_progress(self, fn, busy: str = "Routing..."):
        return fn(None)

    def info(self, text: str):
        print(text)

    def error(self, text: str):
        print(text, file=sys.stderr)


class OptionForm:
    """The controls of a table from ``fields`` on the tabs of a notebook. ``header(page, box)``
    can add its own controls on top of the first tab."""

    def __init__(self, dlg, tabs, opts, header=None):
        self.fields = fields.all_fields(tabs)
        self.controls = {}
        self.book = wx.Notebook(dlg)
        for i, (title, sections) in enumerate(tabs):
            page = wx.Panel(self.book)
            box = wx.BoxSizer(wx.VERTICAL)
            if i == 0 and header is not None:
                header(page, box)
            for heading, group in sections:
                if heading is None:
                    box.Add(self.grid(page, group), 0, wx.ALL | wx.EXPAND, 12)
                    continue
                frame = wx.StaticBoxSizer(wx.VERTICAL, page, heading)
                frame.Add(self.grid(frame.GetStaticBox(), group), 0, wx.ALL | wx.EXPAND, 8)
                box.Add(frame, 0, wx.LEFT | wx.RIGHT | wx.TOP | wx.EXPAND, 8)
            box.AddSpacer(8)
            page.SetSizer(box)
            self.book.AddPage(page, title)
        self.set_values(opts)

    def grid(self, parent, group):
        grid = wx.GridBagSizer(8, 12)
        for row, f in enumerate(group):
            ctrl = self.control(parent, f)
            ctrl.SetToolTip(f.tip)
            self.controls[f.name] = ctrl
            if f.kind == "check":
                grid.Add(ctrl, (row, 0), (1, 2))
                continue
            label = wx.StaticText(parent, label=f.label)
            label.SetToolTip(f.tip)
            grid.Add(label, (row, 0), flag=wx.ALIGN_CENTER_VERTICAL)
            grid.Add(ctrl, (row, 1), flag=wx.EXPAND)
        grid.AddGrowableCol(1)
        return grid

    def control(self, parent, f: fields.Field):
        if f.kind == "int":
            ctrl = wx.SpinCtrl(parent, min=int(f.low), max=int(f.high))
            ctrl.SetIncrement(int(f.step))
            return ctrl
        if f.kind == "float":
            ctrl = wx.SpinCtrlDouble(parent, min=f.low, max=f.high, inc=f.step)
            ctrl.SetDigits(f.digits)
            return ctrl
        if f.kind == "check":
            ctrl = wx.CheckBox(parent, label=f.label)
            ctrl.Bind(wx.EVT_CHECKBOX, lambda _event: self.update_enabled())
            return ctrl
        if f.kind == "refs":
            return wx.TextCtrl(parent, size=(320, -1))
        return wx.FilePickerCtrl(parent, wildcard="JSON files (*.json)|*.json",
                                 style=wx.FLP_OPEN | wx.FLP_FILE_MUST_EXIST | wx.FLP_USE_TEXTCTRL)

    def set_values(self, opts):
        for f in self.fields:
            ctrl, value = self.controls[f.name], getattr(opts, f.name)
            if f.kind == "refs":
                ctrl.SetValue(", ".join(value or []))
            elif f.kind == "file":
                ctrl.SetPath(value or "")
            else:
                ctrl.SetValue(value)
        self.update_enabled()

    def values(self) -> dict:
        out = {}
        for f in self.fields:
            ctrl = self.controls[f.name]
            if f.kind == "refs":
                out[f.name] = runner.parse_references(ctrl.GetValue()) or None
            elif f.kind == "file":
                out[f.name] = ctrl.GetPath() or None
            else:
                out[f.name] = ctrl.GetValue()
        return out

    def update_enabled(self):
        for f in self.fields:
            if f.requires is not None:
                self.controls[f.name].Enable(self.controls[f.requires].GetValue())


def options_dialog(title: str, tabs, opts, factory, note: str, header=None):
    """The dialog and its form. ``factory`` holds the values of the Defaults button."""
    dlg = wx.Dialog(None, title=title)
    form = OptionForm(dlg, tabs, opts, header)
    reset = wx.Button(dlg, label="Defaults")
    reset.SetToolTip("Back to the values of a fresh installation.")
    reset.Bind(wx.EVT_BUTTON, lambda _event: form.set_values(factory))
    buttons = wx.BoxSizer(wx.HORIZONTAL)
    buttons.Add(reset, 0, wx.ALIGN_CENTER_VERTICAL)
    buttons.AddStretchSpacer()
    buttons.Add(dlg.CreateStdDialogButtonSizer(wx.OK | wx.CANCEL), 0, wx.ALIGN_CENTER_VERTICAL)
    box = wx.BoxSizer(wx.VERTICAL)
    box.Add(form.book, 0, wx.ALL | wx.EXPAND, 12)
    box.Add(wx.StaticText(dlg, label=note), 0, wx.LEFT | wx.RIGHT, 12)
    box.Add(buttons, 0, wx.ALL | wx.EXPAND, 12)
    dlg.SetSizerAndFit(box)
    dlg.CentreOnScreen()
    return dlg, form


class WxUi:
    def __init__(self, title: str = TITLE):
        self.app = wx.App(False)
        self.title = title

    def ask_options(self, defaults: runner.RouteOptions) -> Optional[runner.RouteOptions]:
        """``defaults.frames_dir`` is where the animation would go; the check box decides
        whether it is made."""
        dlg, form = options_dialog(TITLE, fields.ROUTE_TABS, defaults, runner.RouteOptions(), (
            "All unlocked tracks and vias are replaced by the routed ones.\n"
            "Without \"keep locked\", locked ones are replaced as well."))
        form.controls["animate"].SetToolTip(
            f"One image per generation and evolution.gif in {defaults.frames_dir}")
        try:
            if dlg.ShowModal() != wx.ID_OK:
                return None
            opts = dataclasses.replace(defaults, **form.values())
            return dataclasses.replace(opts, frames_dir=defaults.frames_dir if opts.animate
                                       else None)
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
    saved = runner.load_options(runner.RouteOptions, "route", runner.ROUTE_SAVED)
    opts = ui.ask_options(runner.RouteOptions(
        frames_dir=frames_default or str(Path(tempfile.gettempdir()) / "gummiband-evolution"),
        **saved))
    if opts is None:
        return 0
    runner.save_options("route", opts, runner.ROUTE_SAVED)

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
