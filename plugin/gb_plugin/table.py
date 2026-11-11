"""The options of ``gummiband route`` as the router's own table has them
(``gummiband_app::OPTIONS`` and its presets). They come from route_options.json next to this
file, which a test of gummiband-app keeps equal to the table: the words of a choice, what each
means and what a preset sets are the same in the plugin, on the command line and for an agent."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, Tuple

_DATA = json.loads(Path(__file__).with_name("route_options.json").read_text(encoding="utf-8"))
OPTIONS: Dict[str, dict] = {o["name"]: o for o in _DATA["options"]}
# name -> {"runs", "generations", "geometry_generations", "open_end_seconds"}
PRESETS: Dict[str, dict] = {p["name"]: p["sets"] for p in _DATA["presets"]}


def words(name: str) -> Tuple[str, ...]:
    """The words of a choice, in the order of the table."""
    return tuple(c["value"] for c in OPTIONS[name]["choices"])


def meaning(name: str, word: str) -> str:
    """What a word of a choice means."""
    return next(c["help"] for c in OPTIONS[name]["choices"] if c["value"] == word)


def describe(name: str, labels: Dict[str, str]) -> str:
    """The text of a choice for a tooltip: what it is, then one paragraph per word under the
    label the dialog shows for it."""
    option = OPTIONS[name]
    return "\n\n".join([option["help"]] + [f'{labels[c["value"]]}: {c["help"]}'
                                           for c in option["choices"]])
