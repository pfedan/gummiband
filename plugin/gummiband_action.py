"""KiCad IPC API action "Route with Gummiband".

KiCad starts this script with the plugin's Python environment and sets KICAD_API_SOCKET and
KICAD_API_TOKEN. Steps: save a copy of the open board (with its project) into a temp folder,
ask for options, run ``gummiband route`` on the copy, then replace the tracks and vias of the open
board with the routed ones in one commit (one undo step).
"""

from __future__ import annotations

import dataclasses
import json
import shutil
import sys
import tempfile
import time
import traceback
from pathlib import Path
from typing import List, Optional

sys.path.insert(0, str(Path(__file__).resolve().parent))

from gb_plugin import board_file as bf  # noqa: E402
from gb_plugin import fields, runner  # noqa: E402

try:
    import wx
except ImportError:  # e.g. Linux system Python without wxPython: run with defaults
    wx = None

TITLE = "Route with Gummiband"
API_TIMEOUT_MS = 120_000  # creating thousands of items in one commit takes a while
TEARDROP_ACTION = "pcbnew.GlobalEdit.editTeardrops"


# User interface ----------------------------------------------------------------------------


class ConsoleUi:
    """No wx: the given options, messages on stdout/stderr (KiCad shows plugin output in its
    status bar since 10.0.1)."""

    def ask_options(self, defaults: runner.RouteOptions) -> Optional[runner.RouteOptions]:
        return dataclasses.replace(defaults, frames_dir=None)

    def run_with_progress(self, fn, busy: str = "Routing..."):
        return fn(None)

    def run_with_metrics(self, fn, open_end: bool = False):
        return fn(None, None)

    def choose_result(self, best: str, others):
        return None

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
        self.pages = []
        for i, (title, sections) in enumerate(tabs):
            # a page scrolls when the dialog is made lower than its fields
            page = wx.ScrolledWindow(self.book, style=wx.VSCROLL)
            page.SetScrollRate(0, 12)
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
            # a scrolled page asks for little room of its own: this one makes the dialog open
            # with every field in sight
            page.SetMinSize(box.GetMinSize())
            self.book.AddPage(page, title)
            self.pages.append(page)
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
    """The dialog and its form. ``factory`` holds the values of the Defaults button. The dialog
    can be resized: the fields grow with its width, and the pages scroll when it is lower than
    they are."""
    dlg = wx.Dialog(None, title=title, style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER)
    form = OptionForm(dlg, tabs, opts, header)
    reset = wx.Button(dlg, label="Defaults")
    reset.SetToolTip("Back to the values of a fresh installation.")
    reset.Bind(wx.EVT_BUTTON, lambda _event: form.set_values(factory))
    buttons = wx.BoxSizer(wx.HORIZONTAL)
    buttons.Add(reset, 0, wx.ALIGN_CENTER_VERTICAL)
    buttons.AddStretchSpacer()
    buttons.Add(dlg.CreateStdDialogButtonSizer(wx.OK | wx.CANCEL), 0, wx.ALIGN_CENTER_VERTICAL)
    box = wx.BoxSizer(wx.VERTICAL)
    box.Add(form.book, 1, wx.ALL | wx.EXPAND, 12)
    box.Add(wx.StaticText(dlg, label=note), 0, wx.LEFT | wx.RIGHT, 12)
    box.Add(buttons, 0, wx.ALL | wx.EXPAND, 12)
    dlg.SetSizerAndFit(box)
    fitted = dlg.GetSize()
    # not higher than the screen; from here on the pages may shrink and scroll instead
    for page in form.pages:
        page.SetMinSize((page.GetMinSize().width, 120))
    screen = wx.Display(max(wx.Display.GetFromWindow(dlg), 0)).GetClientArea()
    size = wx.Size(fitted.width, min(fitted.height, screen.height))
    dlg.SetMinSize((fitted.width, min(fitted.height, 360)))
    dlg.SetSize(size)

    # GTK opens a window at its minimum whatever size it had before, so it gets it once more
    def on_show(event):
        if event.IsShown():
            dlg.Unbind(wx.EVT_SHOW)
            wx.CallAfter(lambda: dlg and dlg.SetSize(size))
        event.Skip()

    dlg.Bind(wx.EVT_SHOW, on_show)
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

    def run_with_metrics(self, fn, open_end: bool = False):
        """Call ``fn(keep_going, accept)`` while a window shows the state of the search."""
        window = MetricsWindow(self.title, open_end)
        window.Show()
        try:
            return fn(window.keep_going, window.accepted)
        finally:
            window.Destroy()

    def choose_result(self, best: str, others):
        """The file of the result the user prefers over the best one, or None."""
        labels = [f"Best: {best}"] + [summary for _, summary in others]
        dlg = wx.SingleChoiceDialog(None, "These results are all as complete and as clean as the "
                                    "best one. Which one do you want?", self.title, labels)
        try:
            if dlg.ShowModal() != wx.ID_OK or dlg.GetSelection() == 0:
                return None
            return others[dlg.GetSelection() - 1][0]
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


METRICS = (("Generation", "generation"), ("Routed", "routed"), ("Vias", "vias"),
           ("Track length", "length"), ("Clearance violations", "violations"),
           ("Without noticeable gain", "stale"), ("Failed side branches", "level"),
           ("Time", "time"))
COURSES = (("vias", wx.Colour(60, 120, 200)), ("length", wx.Colour(70, 160, 110)),
           ("violations", wx.Colour(210, 90, 60))) if wx is not None else ()


class MetricsWindow(wx.Frame if wx is not None else object):
    """Metrics of the running search, their course and the choice to take the result."""

    def __init__(self, title: str, open_end: bool):
        super().__init__(None, title=title, style=wx.DEFAULT_FRAME_STYLE & ~wx.CLOSE_BOX
                         | wx.STAY_ON_TOP)
        self.t0 = time.monotonic()
        self.seen = None
        self.history: List[dict] = []
        self.go_on = True
        self.take = False
        panel = wx.Panel(self)
        self.phase = wx.StaticText(panel, label="Routing...")
        font = self.phase.GetFont()
        self.phase.SetFont(font.Bold())
        grid = wx.FlexGridSizer(len(METRICS), 2, 4, 16)
        self.values = {}
        for label, key in METRICS:
            grid.Add(wx.StaticText(panel, label=label + ":"))
            self.values[key] = wx.StaticText(panel, label="-")
            grid.Add(self.values[key], 0, wx.EXPAND)
        grid.AddGrowableCol(1)
        self.course = wx.Panel(panel, size=(420, 90))
        self.course.SetBackgroundStyle(wx.BG_STYLE_PAINT)
        self.course.Bind(wx.EVT_PAINT, self.paint)
        # the course is drawn for its size: a larger window gets a larger chart
        self.course.Bind(wx.EVT_SIZE, lambda event: (self.course.Refresh(), event.Skip()))
        self.hint_text, self.hint_width = "", 0
        self.hint = wx.StaticText(panel, label="")
        self.accept_button = wx.Button(panel, label="Accept current result")
        self.accept_button.Disable()
        self.accept_button.Show(open_end)
        self.accept_button.Bind(wx.EVT_BUTTON, self.on_accept)
        cancel = wx.Button(panel, label="Cancel")
        cancel.Bind(wx.EVT_BUTTON, self.on_cancel)
        buttons = wx.BoxSizer(wx.HORIZONTAL)
        buttons.AddStretchSpacer()
        buttons.Add(cancel, 0, wx.RIGHT, 8)
        buttons.Add(self.accept_button)
        box = wx.BoxSizer(wx.VERTICAL)
        box.Add(self.phase, 0, wx.ALL, 12)
        box.Add(grid, 0, wx.LEFT | wx.RIGHT | wx.EXPAND, 12)
        box.Add(self.course, 1, wx.ALL | wx.EXPAND, 12)
        box.Add(self.hint, 0, wx.LEFT | wx.RIGHT | wx.EXPAND, 12)
        box.Add(buttons, 0, wx.ALL | wx.EXPAND, 12)
        panel.SetSizer(box)
        box.SetSizeHints(self)
        panel.Bind(wx.EVT_SIZE, self.on_size)
        self.CenterOnScreen()

    def set_hint(self, text: str):
        """The hint below the course, wrapped to the width of the window."""
        width = max(self.course.GetSize().width, 200)
        if (text, width) == (self.hint_text, self.hint_width):
            return
        self.hint_text, self.hint_width = text, width
        self.hint.SetLabel(text)
        self.hint.Wrap(width)
        self.Layout()

    def on_size(self, event):
        event.Skip()
        # wrap again after the sizer has given the course its new width
        wx.CallAfter(lambda: self and self.set_hint(self.hint_text))

    def on_accept(self, _event):
        self.take = True
        self.accept_button.Disable()
        self.set_hint("Finishing the best result...")

    def on_cancel(self, _event):
        self.go_on = False

    def accepted(self) -> bool:
        return self.take

    def keep_going(self, progress=None) -> bool:
        if progress is not None and progress is not self.seen:
            self.seen = progress
            self.show(progress)
        self.values["time"].SetLabel(f"{time.monotonic() - self.t0:.0f} s")
        wx.YieldIfNeeded()
        return self.go_on

    def show(self, p: dict):
        self.history.append(p)
        self.phase.SetLabel(p.get("phase", ""))
        v = self.values
        if p.get("generation", 0) > 0:
            v["generation"].SetLabel(str(p["generation"]))
        v["routed"].SetLabel(f"{p.get('routed', 0)} of {p.get('conns', 0)}")
        v["vias"].SetLabel(str(p.get("vias", 0)))
        v["length"].SetLabel(f"{p.get('length', 0):.0f} mm")
        count = p.get("violations")
        v["violations"].SetLabel("-" if count is None else str(count))
        v["stale"].SetLabel(f"{p.get('stale', 0)} generations")
        v["level"].SetLabel(f"{p.get('level', 0)} of {p.get('top', 0)}" if p.get("top") else "0")
        if p.get("generation", 0) > 0 and not self.take:
            self.accept_button.Enable()
            self.set_hint("Even bold changes do not help any more. Accepting the result is "
                               "recommended."
                               if runner.converged(p) else "")
        self.course.Refresh()

    def paint(self, _event):
        """Vias, length and violations over the reports, each scaled to its own range."""
        dc = wx.AutoBufferedPaintDC(self.course)
        dc.SetBackground(wx.Brush(self.GetBackgroundColour()))
        dc.Clear()
        w, h = self.course.GetClientSize()
        rows = [p for p in self.history if p.get("violations") is not None]
        rows = rows[::len(rows) // w + 1]
        x = 0
        for key, colour in COURSES:
            ys = [p[key] for p in rows]
            if len(ys) > 1:
                lo, hi = min(ys), max(ys)
                dc.SetPen(wx.Pen(colour, 2))
                dc.DrawLines([wx.Point(i * (w - 1) // (len(ys) - 1),
                                       h - 18 - int((y - lo) * (h - 22) / max(hi - lo, 1)))
                              for i, y in enumerate(ys)])
            dc.SetTextForeground(colour)
            dc.DrawText(key, x, h - 16)
            x += dc.GetTextExtent(key)[0] + 12


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


def report(plan: bf.ImportPlan, applied, res: runner.RouteResult,
           moves: Optional[bf.PartMoves] = None) -> str:
    lines = [
        f"Removed: {bf.format_counts(applied.removed)}",
        f"Created: {bf.format_counts(applied.created)}",
    ]
    if moves is not None:
        lines.append(f"Moved parts: {applied.moved}"
                     + (f" ({' '.join(p[0] for p in moves.poses)})" if moves.poses else ""))
        if applied.missing:
            lines.append(f"Not found on the board: {' '.join(applied.missing)}")
        if moves.stale_copper:
            lines.append(f"Warning: {moves.stale_copper} kept tracks and vias moved with a "
                         "footprint group in the router, here they stayed where they were")
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
    if res.gpu_missing():
        lines.append("GPU: this gummiband binary has no GPU support, the run stayed on the CPU. "
                     "Build it with --features gpu.")
    gif = res.animation()
    if gif:
        lines.append(f"Animation of the generations: {gif}")
    if applied.unknown_nets:
        lines.append(f"Nets not found on the board: {', '.join(sorted(applied.unknown_nets))}")
    lines += [f"Warning: {w}" for w in plan.warnings[:5]]
    return "\n".join(lines)


def open_teardrops(kicad) -> None:
    """Last step: KiCad answers only after its dialog closes, and a timeout then means nothing."""
    try:
        kicad.run_action(TEARDROP_ACTION)
    except Exception as e:
        print(f"Edit Teardrops: {e}", file=sys.stderr)


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
        report_file = work / "route.json"
        cmd = runner.build_command(binary, src, out, dataclasses.replace(
            opts, report=str(report_file) if opts.placements else None))
        res = ui.run_with_metrics(lambda keep_going, accept: runner.run_route(
            cmd, out, work, keep_going, accept, gpu=opts.gpu), runner.open_ended(opts))
        chosen = res.alternatives() and ui.choose_result(res.best_line() or "", res.alternatives())
        if chosen:
            out = Path(chosen)
        before, after = src.read_text(encoding="utf-8"), out.read_text(encoding="utf-8")
        plan = bf.plan_from_texts(before, after, opts.keep_locked)
        moves = bf.plan_moves(before, after) if opts.move_parts or opts.placements else None
        zones = []
        if opts.placements:
            zones = json.loads(report_file.read_text(encoding="utf-8"))["placement"]["zones"]
        applied = kicad_apply.apply_plan(board, plan, moves.poses if moves else (), zones)
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
    ui.info(report(plan, applied, res, moves))
    if opts.teardrops:
        open_teardrops(kicad)
    return 0


if __name__ == "__main__":
    sys.exit(main())
