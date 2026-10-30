"""KiCad IPC API action "Place with Gummiband".

Saves a copy of the open board, runs ``gummiband place`` on it and moves the footprints of the
open board to the new positions in one commit (one undo step). The dialog chooses which parts
move: the selected ones, those outside the board, or all unlocked ones. Locked footprints never
move. Route afterwards with "Route with Gummiband".
"""

from __future__ import annotations

import json
import shutil
import sys
import tempfile
import traceback
from pathlib import Path
from typing import List, Optional

sys.path.insert(0, str(Path(__file__).resolve().parent))

import gummiband_action as action  # noqa: E402
from gb_plugin import runner  # noqa: E402

TITLE = "Place with Gummiband"


SCOPES = (
    ("selected", "Selected parts"),
    ("outside", "Parts outside the board"),
    ("unlocked", "All unlocked parts, connectors, modules, holes, buttons and test points stay"),
    ("all", "All unlocked parts, those too"),
)


def default_scope(selected: List[str]) -> str:
    return "selected" if selected else "unlocked"


def ask_options(defaults: runner.PlaceOptions) -> Optional[runner.PlaceOptions]:
    """The dialog; without wxPython the defaults."""
    wx = action.wx
    if wx is None:
        return defaults
    dlg = wx.Dialog(None, title=TITLE)
    count = len(defaults.only or [])
    buttons = {}
    box = wx.BoxSizer(wx.VERTICAL)
    for i, (key, label) in enumerate(SCOPES):
        if key == "selected":
            label = f"{label} ({count})"
        buttons[key] = wx.RadioButton(dlg, label=label, style=wx.RB_GROUP if i == 0 else 0)
        box.Add(buttons[key], 0, wx.LEFT | wx.RIGHT | wx.TOP, 12)
    buttons["selected"].Enable(count > 0)
    buttons[defaults.scope].SetValue(True)
    fixed = wx.TextCtrl(dlg, value=", ".join(defaults.fixed or []), size=(320, -1))
    starts = wx.SpinCtrl(dlg, min=1, max=64, initial=int(defaults.starts))
    gap = wx.SpinCtrlDouble(dlg, min=0.0, max=5.0, initial=float(defaults.gap), inc=0.05)
    gap.SetDigits(2)
    halo = wx.SpinCtrlDouble(dlg, min=0.0, max=5.0, initial=float(defaults.halo), inc=0.05)
    halo.SetDigits(2)
    note = wx.StaticText(dlg, label=(
        "Locked footprints never move. The current position of a moving part does not matter.\n"
        "Tracks and vias do not follow: use this before routing."))
    grid = wx.FlexGridSizer(4, 2, 8, 12)
    grid.AddGrowableCol(1)
    for label, ctrl in (("Also keep (references):", fixed), ("Parallel runs:", starts),
                        ("Courtyard spacing (mm):", gap), ("Room around ICs (mm):", halo)):
        grid.Add(wx.StaticText(dlg, label=label), 0, wx.ALIGN_CENTER_VERTICAL)
        grid.Add(ctrl, 0, wx.EXPAND)
    box.Add(grid, 0, wx.ALL | wx.EXPAND, 12)
    box.Add(note, 0, wx.LEFT | wx.RIGHT, 12)
    box.Add(dlg.CreateStdDialogButtonSizer(wx.OK | wx.CANCEL), 0, wx.ALL | wx.EXPAND, 12)
    dlg.SetSizerAndFit(box)
    dlg.CentreOnScreen()
    try:
        if dlg.ShowModal() != wx.ID_OK:
            return None
        scope = next(k for k, b in buttons.items() if b.GetValue())
        return runner.PlaceOptions(scope, defaults.only, runner.parse_references(fixed.GetValue()),
                                   starts.GetValue(), gap.GetValue(), halo.GetValue())
    finally:
        dlg.Destroy()


def selected_references(board) -> List[str]:
    refs = set()
    for item in board.get_selection():
        field = getattr(item, "reference_field", None)
        if field is not None:
            refs.add(field.text.value)
    return sorted(refs)


def summary(placed: dict, applied) -> str:
    lines = [
        f"Moved {applied.moved} parts, {len(placed['fixed'])} stayed.",
        f"{placed['crossings']} crossings between the nets, seed {placed['seed']}.",
    ]
    if applied.zones:
        lines.append(f"{applied.zones} keepouts followed their parts.")
    if placed["off_plane"]:
        lines.append(f"Pins off their plane (reached by routing): {' '.join(placed['off_plane'])}")
    if placed["stale_copper"]:
        lines.append(f"{placed['stale_copper']} tracks and vias stayed where they were.")
    if applied.missing:
        lines.append(f"Not found on the board: {' '.join(applied.missing)}")
    return "\n".join(lines)


def main() -> int:
    ui = action.WxUi(TITLE) if action.wx is not None else action.ConsoleUi()
    try:
        from kipy import KiCad
        from gb_plugin import kicad_apply
    except ImportError as e:
        ui.error(f"kicad-python is missing in the plugin environment ({e}).")
        return 1
    try:
        board = KiCad(timeout_ms=action.API_TIMEOUT_MS).get_board()
        selected = selected_references(board)
    except Exception as e:
        ui.error(f"Cannot reach the open board through the KiCad API: {e}")
        return 1
    opts = ask_options(runner.PlaceOptions(default_scope(selected), selected))
    if opts is None:
        return 0
    work = Path(tempfile.mkdtemp(prefix="gummiband-"))
    try:
        src = action.save_copy(board, work)
        out = work / f"{src.stem}-placed.kicad_pcb"
        report = work / "place.json"
        cmd = runner.build_place_command(runner.find_binary(), src, out, report, opts)
        ui.run_with_progress(lambda keep_going: runner.run_route(cmd, out, work, keep_going),
                             busy="Placing...")
        placed = json.loads(report.read_text(encoding="utf-8"))
        applied = kicad_apply.apply_placement(board, placed["poses"], placed["zones"])
    except runner.RouteCancelled:
        shutil.rmtree(work, ignore_errors=True)
        return 0
    except runner.RunnerError as e:
        ui.error(f"{e}\n\nFiles kept in {work}")
        return 1
    except Exception:
        ui.error(f"Placing failed:\n\n{traceback.format_exc()}\nFiles kept in {work}")
        return 1
    shutil.rmtree(work, ignore_errors=True)
    ui.info(summary(placed, applied))
    return 0


if __name__ == "__main__":
    sys.exit(main())
