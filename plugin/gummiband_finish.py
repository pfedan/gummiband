"""KiCad IPC API action "Finish selected nets with Gummiband" (follow-me routing).

Select a started track (or any track, via or pad of a net): all copper stays, Gummiband routes
what is missing on the selected nets from where the copper ends, and the new tracks are added to
the open board in one commit (one undo step).
"""

from __future__ import annotations

import shutil
import sys
import tempfile
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import gummiband_action as action  # noqa: E402
from gb_plugin import board_file as bf  # noqa: E402
from gb_plugin import runner  # noqa: E402


# the nets of the selection, as the route dialog takes them
selected_nets = action.selected_nets


def main() -> int:
    ui = action.WxUi() if action.wx is not None else action.ConsoleUi()
    try:
        from kipy import KiCad
        from gb_plugin import kicad_apply
    except ImportError as e:
        ui.error(f"kicad-python is missing in the plugin environment ({e}).")
        return 1
    try:
        board = KiCad(timeout_ms=action.API_TIMEOUT_MS).get_board()
        nets = selected_nets(board)
    except Exception as e:
        ui.error(f"Cannot reach the open board through the KiCad API: {e}")
        return 1
    if not nets:
        ui.info("Select a track, via or pad of the net to finish first.")
        return 0
    work = Path(tempfile.mkdtemp(prefix="gummiband-"))
    try:
        src = action.save_copy(board, work)
        out = work / f"{src.stem}-routed.kicad_pcb"
        # all copper stays and the nets are finished, with the router's preset "quick" and
        # without the open end: there is no window to accept a result in
        opts = runner.with_preset(runner.RouteOptions(scope="finish", nets=nets), "quick")
        cmd = runner.build_command(runner.find_binary(), src, out, opts)
        res = ui.run_with_progress(lambda keep_going: runner.run_route(cmd, out, work, keep_going))
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
        ui.error(f"Routing failed:\n\n{traceback.format_exc()}\nFiles kept in {work}")
        return 1
    shutil.rmtree(work, ignore_errors=True)
    ui.info(f"Nets: {', '.join(nets)}\n\n" + action.report(plan, applied, res))
    return 0


if __name__ == "__main__":
    sys.exit(main())
