"""What the dialogs of the plugin ask for: tabs, sections and fields, each field named after its
attribute of ``runner.RouteOptions`` or ``runner.PlaceOptions``. No wx in here."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class Field:
    name: str
    label: str
    tip: str
    kind: str  # "int", "float", "check", "refs" or "file"
    low: float = 0
    high: float = 0
    step: float = 1
    digits: int = 2
    # the field is greyed out unless this check box is on
    requires: Optional[str] = None


def int_field(name, label, tip, low, high, step=1) -> Field:
    return Field(name, label, tip, "int", low, high, step)


def float_field(name, label, tip, low, high, step, digits=2, requires=None) -> Field:
    return Field(name, label, tip, "float", low, high, step, digits, requires)


THREADS = int_field("threads", "Worker threads:",
                    "How many cores the run may use, 0 = all. The result does not depend on it.",
                    0, 256)
PLANE_LAYERS = Field("plane_layers", "Plane layers:",
                     "Copper layers to treat as planes, separated by commas or spaces, e.g. In1.Cu. "
                     "They are not routed and their zones stay. Layers of KiCad type power are "
                     "planes anyway. Not remembered.", "refs")

# A tab is (title, sections), a section is (title or None, fields).
ROUTE_TABS: Tuple = (
    ("Basic", ((None, (
        Field("keep_locked", "Keep locked tracks and vias",
              "Locked copper stays and the router connects to it. Off: locked tracks and vias are "
              "replaced as well.", "check"),
        int_field("runs", "Parallel runs:",
                  "Routing attempts with different orders in every phase. A constant, so that the "
                  "result does not depend on the machine. More runs explore more orders.",
                  1, 1024),
        float_field("via_cost", "Via cost (mm of track):",
                    "How much track a via is worth. Lower gives more vias and shorter tracks, "
                    "higher fewer vias. The search also tries values around it.", 0, 1000, 0.1),
        int_field("generations", "Generations:",
                  "Rounds of the order search after the first runs, judged by routed connections, "
                  "vias and length.", 0, 50),
        int_field("geometry_generations", "Geometry generations:",
                  "Search rounds judged by the clearance check of the drawn tracks. 0 is fastest, "
                  "more is cleaner; each costs about one routing pass.", 0, 20),
        Field("until_stop", "Keep improving until I accept the result",
              "After the search the router keeps working on the result: fewer vias, fewer "
              "violations, sometimes the last connections. A window shows how it goes and "
              "recommends when to accept. Not together with routing pairs as one wide track.",
              "check"),
        float_field("prefer_width", "Preferred track width (mm):",
                    "Tracks of the default net class push aside what can give way and grow up to "
                    "this width where there is room, with a taper at every change. Routing still "
                    "works with the class width, so it costs no connections. 0 = off. Other "
                    "classes grow to the opt of a track_width custom rule.",
                    0, 5, 0.05),
        Field("move_parts", "Let parts shift to open channels",
              "The geometry generations may shift parts next to clearance problems, so it needs "
              "at least one of them. Locked footprints, connectors, modules, mounting holes, "
              "switches, jumpers and test points stay. The moves arrive in the same undo step.",
              "check"),
        int_field("placements", "Placements to try:",
                  "Places the parts that are not locked or mechanical from the netlist this many "
                  "times, routes each result with these settings and keeps the best. Each one "
                  "costs a full route. 0 keeps the placement.", 0, 64),
        Field("fixed", "Parts that never move:",
              "References that stay in addition, separated by commas or spaces. Not remembered.",
              "refs"),
        Field("teardrops", "Open Edit Teardrops after routing",
              "KiCad's API cannot switch teardrops on for vias and pads, so its Edit Teardrops "
              "dialog opens after the run: enable them there. Wedges at track width changes are "
              "a setting in Board Setup.", "check"),
        Field("animate", "Save an animation of the generations",
              "One image per generation and evolution.gif in gummiband-evolution next to the "
              "project. Needs rsvg-convert and ImageMagick.", "check"),
    )),)),
    ("Advanced", (
        ("Search", (
            float_field("via_pitch", "Via grid pitch (mm):",
                        "Spacing of the sites where vias may go. Finer finds more ways through "
                        "tight spots and takes longer.", 0.1, 5, 0.05),
            float_field("layer_dirs", "Layer direction weight:",
                        "How strongly every third run prefers horizontal and vertical tracks on "
                        "alternating layers. 0 = off.", 0, 10, 0.1),
            float_field("via_pass", "Via removal factor:",
                        "Factor on the via cost in the pass that routes connections with vias "
                        "again and keeps what has fewer vias. 0 = off.", 0, 50, 0.5),
            int_field("negotiate", "Negotiation rounds:",
                      "Experimental: a second search in which connections compete for room at a "
                      "rising price. 20 rounds work well and take about twice the time. 0 = off.",
                      0, 200),
            THREADS,
        )),
        ("Board", (
            PLANE_LAYERS,
            Field("net_widths", "Net widths file:",
                  "JSON file {\"net name\": width in mm}. Nets not in the file use the width of "
                  "their class. Not remembered.", "file"),
            Field("net_priorities", "Net priorities file:",
                  "JSON file {\"net name\": priority}. Lower routes first, nets not in the file "
                  "keep the priority of their class. Not remembered.", "file"),
        )),
        ("Differential pairs", (
            Field("pairs", "Route pairs as one wide track (experimental)",
                  "Routes each pair as one wide track and splits it into two parallel tracks. "
                  "Costs connections on dense boards, so check the result with the DRC.", "check"),
            float_field("pair_skew", "Length difference (mm):",
                        "Length difference left within a pair; the shorter track gets bumps down "
                        "to this.", 0, 10, 0.05, requires="pairs"),
            float_field("pair_zdiff", "Differential impedance (ohm):",
                        "Sets the track width per layer from the stackup, the gap stays. 0 keeps "
                        "the widths of the net class.", 0, 500, 5, requires="pairs"),
            float_field("pair_pull", "Pair pull:",
                        "How strongly the two nets of a pair attract each other while routing, "
                        "also without the wide track. 0 = off.", 0, 20, 0.1),
        )),
    )),
)

PLACE_TABS: Tuple = (
    ("Basic", ((None, (
        Field("fixed", "Also keep (references):",
              "Parts that stay in addition to the locked ones, separated by commas or spaces. "
              "Not remembered.", "refs"),
        int_field("starts", "Parallel runs:",
                  "Independent placements; the one with the fewest crossings wins. The crossings "
                  "hardly predict how well the board routes, so one run is the default: for "
                  "another attempt, change the seed.", 1, 64),
        float_field("gap", "Courtyard spacing (mm):", "Spacing between courtyards.", 0, 5, 0.05),
        float_field("halo", "Room around ICs (mm):",
                    "Extra room around parts with five or more pads, for their escape.",
                    0, 5, 0.05),
        int_field("seed", "Seed:",
                  "First seed, the parallel runs count up from it. Placements differ a lot from "
                  "seed to seed: for another attempt, change it.", 0, 2_000_000_000),
    )),)),
    ("Advanced", ((None, (
        Field("free", "Release (references):",
              "Parts that may move although they count as mechanical, separated by commas or "
              "spaces. Locked footprints still stay. Not remembered.", "refs"),
        float_field("margin", "Distance to the outline (mm):",
                    "Courtyards keep this distance from the board outline.", 0, 20, 0.1),
        int_field("iters", "Annealing moves per run:",
                  "More moves take longer and find fewer crossings.",
                  100_000, 2_000_000_000, 500_000),
        THREADS,
        PLANE_LAYERS,
    )),)),
)


def all_fields(tabs) -> list:
    return [f for _title, sections in tabs for _heading, group in sections for f in group]
