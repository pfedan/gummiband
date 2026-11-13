"""What the dialogs of the plugin ask for: tabs, sections and fields, each field named after its
attribute of ``runner.RouteOptions`` or ``runner.PlaceOptions``. No wx in here."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from . import runner, table


@dataclass(frozen=True)
class Field:
    name: str
    label: str
    tip: str
    # "int", "float", "check", "refs", "nets" (names separated by commas), "file", "choice" (one
    # of ``choices``) or "seed" (an int with a button that draws a new one)
    kind: str
    low: float = 0
    high: float = 0
    step: float = 1
    digits: int = 2
    # the field is greyed out unless this check box is on (or this number is not 0); for a
    # choice "name=word|word": unless it has one of these words
    requires: Optional[str] = None
    # kind "choice": (value, label) pairs
    choices: Tuple[Tuple[str, str], ...] = ()
    # the field is no option of its own: its value follows from other fields (``derived``),
    # and choosing one sets them (``follows``)
    derived: bool = False


def enabled(f: Field, values: dict) -> bool:
    """Whether a field means something with these values of the dialog."""
    if f.requires is None:
        return True
    name, _, words = f.requires.partition("=")
    return values.get(name) in words.split("|") if words else bool(values.get(name))


def follows(name: str, value) -> dict:
    """The fields a choice sets when the user picks ``value``: a preset sets its four."""
    if name == "preset" and value in table.PRESETS:
        return runner.preset_values(value)
    return {}


def derived(values: dict) -> dict:
    """The fields whose value follows from the others: the preset these values are."""
    return {"preset": runner.preset_of(values)}


def worded(name: str, labels: dict) -> Tuple[Tuple[str, str], ...]:
    """The choices of an option of the router: its words in the order of the router's table,
    each under the label the dialog shows. A word without a label is an error at import, so a
    new word of the router cannot go missing here."""
    return tuple((word, labels[word]) for word in table.words(name))


def int_field(name, label, tip, low, high, step=1, requires=None) -> Field:
    return Field(name, label, tip, "int", low, high, step, requires=requires)


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

SCOPE_LABELS = {
    "fresh": "Nothing: locked tracks and vias are replaced too",
    "keep_locked": "Locked tracks and vias; the rest is laid anew",
    "finish": "Everything; lay what is still open",
    "reroute": "Everything but the nets below, which are laid again",
}
PRESET_LABELS = {"draft": "Draft", "quick": "Quick", "standard": "Standard", "final": "Final"}
PAIRS_LABELS = {
    "off": "Each net on its own",
    "pull": "Pull the two nets together (experimental)",
    "band": "One wide track per pair (experimental)",
}
# the scopes that lay the board anew: parts may move and placements can be tried
LAID_ANEW = "scope=fresh|keep_locked"

# A tab is (title, sections), a section is (title or None, fields) or (title or None, fields,
# intro): the intro is a short text above the fields, for advice that should not hide in a
# tooltip.
ROUTE_TABS: Tuple = (
    ("Basic", ((None, (
        Field("scope", "Copper that stays:",
              table.describe("scope", SCOPE_LABELS), "choice",
              choices=worded("scope", SCOPE_LABELS)),
        Field("nets", "Nets:",
              "The nets to lay, separated by commas. Filled in from the tracks, vias and pads "
              "that are selected when the dialog opens. Laying nets again needs them; with "
              "\"lay what is still open\" an empty field means every open connection. The "
              "unlocked tracks of other nets shift aside for them and give way within 8 mm. "
              "Not remembered.", "nets", requires="scope=finish|reroute"),
        Field("preset", "Preset:",
              table.describe("preset", PRESET_LABELS) + "\n\nA preset sets the four fields "
              "below it; change any of them afterwards and it reads Custom. Final keeps "
              "improving until you accept the result, where the command line stops after 10 "
              "minutes.", "choice",
              choices=worded("preset", PRESET_LABELS) + ((runner.CUSTOM, "Custom"),),
              derived=True),
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
              "step. Not where all copper stays: there the parts stay too.",
              "check", requires=LAID_ANEW),
        int_field("placements", "Placements to try:",
                  "Places the parts that are not locked or mechanical from the netlist this many "
                  "times, routes each result with these settings and keeps the best. Each one "
                  "costs a full route. 0 keeps the placement. Not where all copper stays.",
                  0, 64, requires=LAID_ANEW),
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
                  "more violations).", "choice", requires="hint_groups",
                  choices=(("long", "Longest first"), ("short", "Shortest first"))),
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
                  "Helps boards with many capacitors at their ICs (7 % on the LED controller "
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
                  "Not for a board whose placement you have settled: it moves parts.",
                  requires=LAID_ANEW),
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
            Field("areas_fill", "Leave nets with a zone to its fill (experimental)",
                  "A net with a zone on a routed layer (a ground pour) gets no tracks: the fill "
                  "of the zone connects it, and the result names the pads the fill does not "
                  "reach. The other nets get the room. The run then ends on its own instead of "
                  "improving until accepted.", "check"),
            Field("pour", "Planned pours (nets):",
                  "Nets that get a pour on both outer layers after the route, separated by "
                  "commas or spaces, e.g. GND. The route takes the pour as the zone of the net; "
                  "lay it afterwards. Not remembered.", "refs", requires="areas_fill"),
            Field("net_widths", "Net widths file:",
                  "JSON file {\"net name\": width in mm}. Nets not in the file use the width of "
                  "their class. Not remembered.", "file"),
            Field("net_priorities", "Net priorities file:",
                  "JSON file {\"net name\": priority}. Lower routes first, nets not in the file "
                  "keep the priority of their class. Not remembered.", "file"),
        )),
        ("Differential pairs", (
            Field("pairs", "Pairs:", table.describe("pairs", PAIRS_LABELS), "choice",
                  choices=worded("pairs", PAIRS_LABELS)),
            float_field("pair_pull", "Pair pull (0 = the router's):",
                        "How strongly the two nets of a pair attract each other while routing. "
                        "0 leaves it to the router: 1 with the pull, a good start. With the wide "
                        "track a weight pulls the pairs that get none (those without a pair "
                        "rule or class of their own), and 0 leaves them alone.", 0, 20, 0.1,
                        requires="pairs=pull|band"),
            float_field("pair_skew", "Length difference (mm):",
                        "With the wide track: length difference left within a pair; the shorter "
                        "track gets bumps down to this.", 0, 10, 0.05, requires="pairs=band"),
            float_field("pair_zdiff", "Differential impedance (ohm):",
                        "With the wide track: sets the track width per layer from the stackup, "
                        "the gap stays. 0 keeps the widths of the net class.", 0, 500, 5,
                        requires="pairs=band"),
            check("pair_all", "Also pairs without a pair rule or class",
                  "With the wide track: one for every pair recognised by name (_P/_N, +/-). "
                  "Without it only pairs that a diff_pair_gap rule or a net class of their own "
                  "marks.", requires="pairs=band"),
            check("via_pairs", "Vias of a pair as one capsule (experimental)",
                  "With the wide track: two vias side by side are one via pair that turns with "
                  "it, instead of the circle around both. Helps pairs that change layers often "
                  "(DDR5: 10 % fewer DRC errors).", requires="pairs=band"),
            check("breakouts", "Single tracks into the pads (experimental)",
                  "With the wide track: it ends just outside the pads, two short single tracks "
                  "go in, and their room stays free. Helps at connector rows, still worse on a "
                  "DDR fan-out.", requires="pairs=band"),
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
              "the cleanest placements of the LED controller (no clearance error in 9 "
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
        check("two_sided", "Place on both sides",
              "Each side gets its own room; through hole parts take their holes on the other "
              "side, the space under their body stays free. Small SMD parts (up "
              "to the pads below) may go to the back: what does not fit on the front, and what "
              "saves more wire there than the cost below (a decoupling capacitor right under its "
              "IC). Off: one room for both sides, every part keeps its side.\n\n"
              "For dense boards: when a one sided placement fails, its message says whether "
              "front and back together have the room."),
        float_field("back_cost", "Cost of a part on the back (mm):",
                    "With both sides: a part goes to the back if the front has no room for it or "
                    "the back saves at least this much wire. 0 = both sides alike; large = the "
                    "back only for what does not fit.", 0, 100, 1, digits=1,
                    requires="two_sided"),
        int_field("back_max_pads", "Largest part for the back (pads):",
                  "With both sides: parts with more pads keep their side (the big ICs).", 2, 1000,
                  requires="two_sided"),
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
