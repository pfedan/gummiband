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
    # "int", "float", "check", "refs", "file", "choice" (one of ``choices``) or "seed" (an int
    # with a button that draws a new one)
    kind: str
    low: float = 0
    high: float = 0
    step: float = 1
    digits: int = 2
    # the field is greyed out unless this check box is on (or this number is not 0)
    requires: Optional[str] = None
    # kind "choice": (value, label) pairs
    choices: Tuple[Tuple[str, str], ...] = ()


def int_field(name, label, tip, low, high, step=1) -> Field:
    return Field(name, label, tip, "int", low, high, step)


def float_field(name, label, tip, low, high, step, digits=2, requires=None) -> Field:
    return Field(name, label, tip, "float", low, high, step, digits, requires)


def seed_field(name, label, tip, low=0, requires=None) -> Field:
    return Field(name, label, tip, "seed", low, SEED_MAX, 1, requires=requires)


def check(name, label, tip, requires=None) -> Field:
    return Field(name, label, tip, "check", requires=requires)


SEED_MAX = 2_000_000_000

# The opt-in options are off because none wins on every board; a tooltip says what the option
# does, where it helped and where it hurt in the measurements (docs/guide/options.md).
COMPARE = ("Compare with a run without it over the same two or three seeds: results scatter a "
           "lot from seed to seed.")


THREADS = int_field("threads", "Worker threads:",
                    "How many cores the run may use, 0 = all. The result does not depend on it.",
                    0, 256)
PLANE_LAYERS = Field("plane_layers", "Plane layers:",
                     "Copper layers to treat as planes, separated by commas or spaces, e.g. In1.Cu. "
                     "They are not routed and their zones stay. Layers of KiCad type power are "
                     "planes anyway. Not remembered.", "refs")

# A tab is (title, sections), a section is (title or None, fields) or (title or None, fields,
# intro): the intro is a short text above the fields, for advice that should not hide in a
# tooltip.
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
                    "How much track a via is worth while routing. Lower gives more vias and "
                    "shorter tracks, higher fewer vias. The search also tries values around it. "
                    "The improvement after the search judges with its own via price.",
                    0, 1000, 0.1),
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
        float_field("open_via_cost", "Via price while improving (mm of track):",
                    "How much track a via is worth when the improvement judges its results. Lower "
                    "gives more vias and shorter tracks, higher fewer vias and longer tracks. "
                    "The default is 20.", 0, 1000, 1, digits=1, requires="until_stop"),
        float_field("prefer_width", "Preferred track width (mm):",
                    "Tracks of the default net class push aside what can give way and grow up to "
                    "this width where there is room, with a taper at every change. Routing still "
                    "works with the class width, so it costs no connections. 0 = off. Other "
                    "classes grow to the opt of a track_width custom rule.",
                    0, 5, 0.05),
        Field("move_parts", "Let parts shift to open channels",
              "The geometry generations may shift parts next to clearance problems and open "
              "connections, so it needs at least one of them; the routed tracks follow like "
              "rubber bands. On by default. Locked footprints, connectors, modules, mounting "
              "holes, switches, jumpers and test points stay. The moves arrive in the same undo "
              "step.",
              "check"),
        int_field("placements", "Placements to try:",
                  "Places the parts that are not locked or mechanical from the netlist this many "
                  "times, routes each result with these settings and keeps the best. Each one "
                  "costs a full route. 0 keeps the placement.", 0, 64),
        seed_field("placement_seed", "First placement seed:",
                   "The placements to try use this seed and the ones after it. After every run "
                   "it moves on by the number of placements, so the next run tries placements "
                   "it has not seen yet; the report names the seed of the winner. Placements "
                   "differ a lot from seed to seed.", low=1, requires="placements"),
        seed_field("seed", "Search seed:",
                   "Seed of the routing search. The same board, settings and seed give the same "
                   "result; another seed is another attempt, often with quite different vias "
                   "and violations. 0 = the router's fixed seed. \"New\" draws another one: try "
                   "two or three before you judge a setting."),
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
    ("Hints", (
        ("Routing order (all off: none wins on every board)", (
            check("hint_groups", "Route buses between parts as blocks (groups)",
                  "The connections between two parts with 4 or more pins that share at least two "
                  "nets (a bus, SPI, I2C, a connector) are routed as one block in pin order, so "
                  "they do not cross each other.\n\n"
                  "Helps on boards with several chips: up to 23 % fewer vias on the test boards "
                  "with an MCU, a PHY and buses, at a few percent more length.\n"
                  "Hurts a board whose placement is final and already routes cleanly: there it "
                  "added clearance violations.\n\n" + COMPARE),
            Field("group_order", "Order of the blocks:",
                  "Longest first: the default and the best order on most boards.\n"
                  "Shortest first: best on a DDR memory fan-out, worse elsewhere (more vias, "
                  "more violations).\n"
                  "In place: each block keeps the place of its connections in the normal order; "
                  "the weakest overall, for experiments.", "choice", requires="hint_groups",
                  choices=(("long", "Longest first"), ("short", "Shortest first"),
                           ("inline", "In place"))),
            check("hint_pairs", "Differential pairs first (pairs)",
                  "Differential pairs are routed before the other signals, the longest first, "
                  "both nets next to each other.\n\n"
                  "Helps pairs: shorter and better coupled. Together with the blocks above the "
                  "best result on an FPGA board with many pairs (12 % fewer vias).\n"
                  "Hurts when the pairs are few and the rest is dense: more vias and DRC errors."
                  "\n\n" + COMPARE),
            check("hint_decaps", "Decoupling capacitors first (decaps)",
                  "A decoupling capacitor is connected to its IC pin before everything else, and "
                  "the via serves both.\n\n"
                  "Helps boards with many capacitors at their ICs (7 % on the Lichtschnitzel "
                  "controller).\nHurts fine pitch boards without planes.\n\n" + COMPARE),
            check("hint_rows", "Lay pin rows again in pin order (rows)",
                  "While improving after the search, the connections of a pin row are laid "
                  "again in pin order. Acts only with \"Keep improving\" on the Basic tab.\n\n"
                  "A small gain on every board measured so far, but few runs.",
                  requires="until_stop"),
            check("hint_shares", "Neighbouring plane pins share a via (shares)",
                  "Pins of a plane net next to each other reach the plane through one via.\n\n"
                  "Rarely helps: the router already shares a via that lies close. Cost 70 vias "
                  "on an FPGA board. For experiments."),
        ), "Off by default because none wins on every board. Several chips with buses: the "
           "blocks. FPGA or BGA with many pairs: blocks and pairs. DDR fan-out: blocks, "
           "shortest first. A placement that already routes cleanly: leave them off. Hover "
           "for what each did in the measurements, and compare with a plain run over the "
           "same seeds."),
        ("Room for buses", (
            check("clear_corridors", "Move small parts out of the way of buses (experimental)",
                  "Before routing, movable parts that sit in the band of a bus between two parts "
                  "(see the blocks above) and are not on its nets get new positions next to it. "
                  "ICs and connectors move at most 1.5 mm, decoupling capacitors stay at their IC. "
                  "The moves arrive in the same undo step.\n\n"
                  "Helps tight boards with several chips, together with the blocks: up to 18 % "
                  "fewer vias and 8 % less length.\n"
                  "Not for a board whose placement you have settled: it moves parts."),
        ), "For tight boards with several chips, together with the blocks above."),
    )),
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
                      "A second search from the start in which connections compete for room at a "
                      "rising price. 20 rounds work well and take about twice the time. With 0 "
                      "the search negotiates on its own when it leaves a few connections open.",
                      0, 200),
            THREADS,
            Field("gpu", "Relax on the GPU (experimental, macOS)",
                  "Runs most of the track relaxation as a compute shader. Needs a gummiband binary "
                  "built with the gpu feature. Saves time where the relaxation dominates, not on "
                  "boards where the search does. Results differ between GPUs, so use the CPU for a "
                  "final run.", "check"),
        )),
        ("Result", (
            float_field("air", "Extra air over the clearance (mm):",
                        "The finished copper gets this reserve on top of the clearances where "
                        "there is room: a rubber band lies right at the limit, this pushes it off "
                        "and evens out tight bundles. A result with new violations is dropped. "
                        "Applies with \"Keep improving\"; 0.03 is the router's value there.",
                        0, 0.5, 0.01, digits=2, requires="until_stop"),
            Field("angles_45", "45 degree tracks (conventional look)",
                  "Turns the any-angle tracks and arcs into straight pieces in steps of 45 degrees "
                  "with chamfered corners, as a last step. Tracks get a few percent longer. It "
                  "never adds a clearance violation: where no 45 degree path fits, a piece keeps "
                  "its angle.", "check"),
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
            check("pair_all", "Also pairs without a pair rule or class",
                  "Couples every pair recognised by name (_P/_N, +/-). Without it only pairs "
                  "that a diff_pair_gap rule or a net class of their own marks.",
                  requires="pairs"),
            check("via_pairs", "Vias of a pair as one capsule (experimental)",
                  "Two vias side by side are one via pair that turns with the wide track, "
                  "instead of the circle around both. Helps pairs that change layers often "
                  "(DDR5: 10 % fewer DRC errors).", requires="pairs"),
            check("breakouts", "Single tracks into the pads (experimental)",
                  "The wide track ends just outside the pads, two short single tracks go in, "
                  "and their room stays free. Helps at connector rows, still worse on a DDR "
                  "fan-out.", requires="pairs"),
            float_field("pair_pull", "Pair pull:",
                        "How strongly the two nets of a pair attract each other while routing, "
                        "also without the wide track. 0 = off, 1 is a good start. The gentle way "
                        "to couple pairs on a board that is not dense; the wide track keeps them "
                        "closer but costs connections there.", 0, 20, 0.1),
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
        seed_field("seed", "Seed:",
                   "First seed, the parallel runs count up from it. Placements differ a lot "
                   "from seed to seed: for another attempt, press \"New\". A good seed is "
                   "often worth more than any other option."),
    )),)),
    ("Strategy", ((None, (
        check("semantic", "Use the schematic",
              "Beyond the netlist: parts of a sheet and clusters around an IC stay together, an "
              "inductor sits at the switch pins of its converter, decoupling capacitors prefer "
              "an IC of their own sheet, current paths count their full length.\n\n"
              "Helps boards with power stages and several sheets: with the two options below "
              "the cleanest placements of the Lichtschnitzel controller (no clearance error in 9 "
              "runs, 12 % shorter)."),
        check("corridors", "Keep bands free for buses (experimental)",
              "Two parts with 4 or more pads that share at least two signal nets (a bus, SPI, "
              "I2C) get a free band between them: other parts in it pay for the area they cover."
              "\n\nHelps boards with several chips: fewer vias. Route the result with "
              "\"Route buses between parts as blocks\"."),
        check("escape_halo", "Room for the escape per side (experimental)",
              "Instead of the same room all round, each side of a part with five or more pads "
              "gets as much as the tracks leaving it need (at most 3 mm), and the room setting "
              "is not used for it.\n\n"
              "Helps tightly packed ICs: no more missed connections next to them, at about 15 % "
              "more length."),
        float_field("leash", "Leash for ICs and connectors (mm):",
                    "Parts with 4 or more pads stay within this distance of where they sit: a "
                    "local change instead of a new placement, for example with \"Selected parts\". "
                    "0 = free.", 0, 50, 0.5, digits=1),
    ), "For a placement from scratch, the first three together gave the cleanest results. "
       "Try two or three seeds and route each: a good seed is worth more than any option."),)),
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
    return [f for _title, sections in tabs for section in sections for f in section[1]]
