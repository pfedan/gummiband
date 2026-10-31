"""KiCad IPC API action "45 degree tracks with Gummiband".

Turns the unlocked tracks of the open board into conventional 45 degree tracks without routing
again: with a selection only the nets of the selected tracks, vias or pads, otherwise all nets.
Vias, track ends, pads and locked tracks stay where they are; tracks that do not change stay on
the board as they are. Everything changes in one commit (one undo step).
"""

from __future__ import annotations

import shutil
import sys
import tempfile
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import gummiband_action as action  # noqa: E402
import gummiband_finish as finish  # noqa: E402
from gb_plugin import board_file as bf  # noqa: E402
from gb_plugin import runner  # noqa: E402

TITLE = "45 degree tracks with Gummiband"


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
        nets = finish.selected_nets(board)
    except Exception as e:
        ui.error(f"Cannot reach the open board through the KiCad API: {e}")
        return 1
    saved = runner.load_options(runner.RouteOptions, "route", runner.ROUTE_SAVED)
    opts = runner.RouteOptions(**saved)
    work = Path(tempfile.mkdtemp(prefix="gummiband-"))
    try:
        src = action.save_copy(board, work)
        out = work / f"{src.stem}-45.kicad_pcb"
        cmd = runner.build_angles_command(runner.find_binary(), src, out, nets, opts)
        res = ui.run_with_progress(lambda keep_going: runner.run_route(cmd, out, work, keep_going),
                                   busy="Turning tracks to 45 degrees...")
        plan = bf.plan_addition(bf.read_copper(src.read_text(encoding="utf-8")),
                                bf.read_copper(out.read_text(encoding="utf-8")))
        applied = kicad_apply.apply_plan(board, plan)
    except runner.RouteCancelled:
        shutil.rmtree(work, ignore_errors=True)
        return 0
    except runner.RunnerError as e:
        ui.error(f"{e}\n\nFiles kept in {work}")
        return 1
    except Exception:
        ui.error(f"The 45 degree step failed:\n\n{traceback.format_exc()}\nFiles kept in {work}")
        return 1
    shutil.rmtree(work, ignore_errors=True)
    lines = [f"Nets: {', '.join(nets) if nets else 'all'}",
             "",
             f"Removed: {bf.format_counts(applied.removed)}",
             f"Created: {bf.format_counts(applied.created)}"]
    summary = runner.angles_line(res.stdout)
    if summary:
        lines += ["", summary[0].upper() + summary[1:]]
    if applied.unknown_nets:
        lines.append(f"Nets not found on the board: {', '.join(sorted(applied.unknown_nets))}")
    lines += [f"Warning: {w}" for w in plan.warnings[:5]]
    lines += ["", "Undo with Ctrl+Z. Run DRC to check the result."]
    ui.info("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
