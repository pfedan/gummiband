"""KiCad IPC API action "Place with Gummiband".

Saves a copy of the open board, runs ``gummiband place`` on it and moves the footprints of the
open board to the new positions in one commit (one undo step). The dialog chooses which parts
move: the selected ones, those outside the board, or all unlocked ones. Locked footprints never
move. Route afterwards with "Route with Gummiband".
"""

from __future__ import annotations

import dataclasses
import json
import shutil
import sys
import tempfile
import traceback
from pathlib import Path
from typing import List, Optional

sys.path.insert(0, str(Path(__file__).resolve().parent))

import gummiband_action as action  # noqa: E402
from gb_plugin import fields, runner  # noqa: E402

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
    count = len(defaults.only or [])
    buttons = {}

    def scope_buttons(page, box):
        for i, (key, label) in enumerate(SCOPES):
            if key == "selected":
                label = f"{label} ({count})"
            buttons[key] = wx.RadioButton(page, label=label, style=wx.RB_GROUP if i == 0 else 0)
            box.Add(buttons[key], 0, wx.LEFT | wx.RIGHT | wx.TOP, 12)
        buttons["selected"].Enable(count > 0)
        buttons[defaults.scope].SetValue(True)

    dlg, form = action.options_dialog(
        TITLE, fields.PLACE_TABS, defaults, runner.PlaceOptions(),
        "Locked footprints never move. The current position of a moving part does not matter.\n"
        "Tracks and vias do not follow: use this before routing.", scope_buttons)
    try:
        if dlg.ShowModal() != wx.ID_OK:
            return None
        scope = next(k for k, b in buttons.items() if b.GetValue())
        return dataclasses.replace(defaults, scope=scope, **form.values())
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
    if placed.get("back") or applied.flipped:
        lines.append(f"{len(placed.get('back', []))} moved parts on the back, "
                     f"{applied.flipped} changed sides.")
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
    saved = runner.load_options(runner.PlaceOptions, "place", runner.PLACE_SAVED)
    opts = ask_options(runner.PlaceOptions(default_scope(selected), selected, **saved))
    if opts is None:
        return 0
    runner.save_options("place", opts, runner.PLACE_SAVED)
    work = Path(tempfile.mkdtemp(prefix="gummiband-"))
    try:
        src = action.save_copy(board, work)
        out = work / f"{src.stem}-placed.kicad_pcb"
        report = work / "place.json"
        cmd = runner.build_place_command(runner.find_binary(), src, out, report, opts)
        ui.run_with_progress(lambda keep_going: runner.run_route(cmd, out, work, keep_going),
                             busy="Placing...")
        placed = json.loads(report.read_text(encoding="utf-8"))
        applied = kicad_apply.apply_placement(board, placed["poses"], placed["zones"],
                                              placed.get("flipped", []),
                                              placed.get("flipped_zones", []))
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
