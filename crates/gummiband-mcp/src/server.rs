//! MCP tools of the Gummiband server and the session state behind them.

use std::collections::{BTreeMap, HashMap};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

use anyhow::{Context, Result, bail};
use base64::Engine as _;
use gummiband_app::old::Respelled;
use gummiband_app::{Angles, NetOptions, Pairs, Preset, RouteOptions, Scope};
use gummiband_core::board::Board;
use gummiband_kicad::{KeepCopper, KicadBoard, ReadOptions, read_board, write};
use rmcp::handler::server::router::tool::ToolRouter;
use rmcp::handler::server::tool::schema_for_output;
use rmcp::handler::server::wrapper::Parameters;
use rmcp::model::{CallToolResult, ContentBlock, Implementation, ServerCapabilities, ServerConfig};
use rmcp::schemars::{self, JsonSchema};
use rmcp::{ErrorData, ServerHandler, tool, tool_handler, tool_router};
use serde::{Deserialize, Serialize};

use crate::external;
use crate::guide;
use crate::practice;
use crate::reference;

const INSTRUCTIONS: &str = "Gummiband is a topological (any-angle, rubber band) autorouter for \
KiCad boards, and a toolbox for the layouter, not a one-stop shop: you stay the designer. Auto-place \
(place_parts) and auto-route (route) complement manual work, they do not replace it. Place the parts \
that matter yourself (move_part), route critical copper by hand (add_locked_copper: tracks and vias, \
always locked), lay out planes and pours (set_layer_type, add_zone, power_copper, ground_pour) and \
lock what must stay (lock: parts by reference, nets, or all copper; unlock: true releases them). \
Locked parts never move in place_parts or route, locked copper is kept and routed around. What a \
route replaces is its scope: \"keep_locked\" lays all unlocked copper anew, \"finish\" keeps all \
copper and lays what is still open (you laid some nets by hand: finish the rest), \"reroute\" with \
nets lays those nets again and keeps the rest (lay net X again), \"fresh\" keeps nothing. After \
place_parts and route keep refining: move_part on the routed board (tracks follow), \
add_locked_copper then route scope \"reroute\" with nets to redraw a net, lock good nets and route \
the rest again, add pours, angles_45. remove takes zones, tracks, vias and footprints away again (a \
power_copper pour by its gb: name), add_mounting_hole adds a hole, move_part side flips a part, \
set_rules adds .kicad_dru rules, place_silk moves the reference texts off pads and other silk, and \
try_seeds compares place_parts + route over several seeds (results scatter widely on tight boards). \
create_board parts_outside keeps the board empty for hand placement and copper drawn first. \
The tool guide returns the documentation: guide without topic for \
this workflow and the topic list, guide with a tool name for its full reference. \
From a schematic only: create_board turns a KiCad netlist into a board (footprints, nets, outline, stackup, net classes). The automatic loop: load_board (gives a board id) -> list_nets -> optionally \
set_net_options (priority: lower routes first; skip: leave a net unrouted; width, min_width and neck_refs prescribe the copper of a net) -> route (writes a \
routed .kicad_pcb and returns stats, unrouted connections and clearance violations) -> drc on \
the written file (KiCad's own DRC) -> render a region (x0,y0,x1,y1 in mm, board coordinates) to \
look at problems. To pre-route a critical part by hand (datasheet layout): part_info -> \
add_locked_copper -> route the returned board id. move_part and add_zone edit the same working \
copy. To change one net on a routed board: load the routed file -> route with scope \"reroute\" and \
nets (the unlocked tracks of other nets shift aside or give way where needed, see shove and give). To straighten a finished any-angle result to 45 degrees without routing again: angles_45 on the routed file. To place parts: move_part for connectors and power stages, place_parts for the rest (semantic: true keeps sheets and IC clusters together and current paths short; two_sided: true when the parts do not fit on one side), set_layer_type (power) and add_zone for planes. load_board lists the differential pairs it recognised; route couples them only with pairs \"pull\" (gentle, every pair) or \"band\" (one wide track, experimental). Good practice around routing (the prompt good_practice walks through a whole board): \
power_plan before routing (currents -> widths, planes, vias; apply stores the widths), \
current_check after routing (narrowest cross-section of every current path), ground_pour (GND \
pours and stitching vias), area_report (which pads the fill of a pour or plane reaches, which it \
leaves on islands, and where one via joins each island), reference_check (signal tracks over gaps or splits of their reference plane), then drc. Opt-in options (route patterns, clear_corridors, \
pairs pull and band, place_parts semantic, corridors, escape_halo, leash) are off because none wins on every board; \
results scatter strongly between seeds, so compare a setting with the plain run over the same 2 or 3 \
seeds (mean and worst, judged by drc). Rough guide: several chips with buses -> patterns groups, on a \
tight board also clear_corridors, or track_corridors and place_parts only + corridors + leash 1.5 to \
decide yourself; FPGA/BGA with pairs -> pairs,groups; DDR -> groups-short; a fixed placement that \
already routes cleanly -> no hints, try seeds and preset final (the open end); placement from scratch -> \
place_parts semantic + corridors + escape_halo over a few seeds. Coordinates are millimetres in KiCad's board coordinates (y grows downwards). \
Routing is CPU heavy (seconds to minutes); routes run one at a time.";

// ------------------------------------------------------------------------------ session

#[derive(Debug, Serialize, JsonSchema)]
pub struct PadOut {
    pub name: String,
    pub net: String,
    pub x: f64,
    pub y: f64,
    pub width_mm: f64,
    pub height_mm: f64,
    /// Copper layers of the pad.
    pub layers: Vec<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct PartInfoOut {
    pub reference: String,
    pub value: String,
    /// Position and rotation of the footprint in mm and degrees.
    pub x: f64,
    pub y: f64,
    pub rotation: f64,
    pub locked: bool,
    /// Schematic sheet of the part, empty on the root sheet.
    pub sheet: String,
    /// Courtyard box [x0, y0, x1, y1] in mm, if the footprint has one.
    pub courtyard: Option<[f64; 4]>,
    pub pads: Vec<PadOut>,
    /// Footprints whose courtyard is within 3 mm.
    pub neighbours: Vec<String>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct BoardPatternsIn {
    /// Board id from load_board.
    pub board: String,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct BoardPatternsOut {
    /// One line per kind of pattern.
    pub summary: Vec<String>,
    /// Pin rows (part, pins in order, pitch, side the tracks leave to), pad grids (BGA),
    /// decoupling capacitors (IC pin, capacitor pins), shared plane fan-outs (leader pin with the
    /// via, the pins that connect to it) and buses (two parts, nets, order kept).
    pub patterns: serde_json::Value,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct AddLockedOut {
    /// Id of the working copy with the locked copper; route this one.
    pub board: String,
    /// The working copy (<input>.prerouted.kicad_pcb); the input file stays untouched.
    pub path: String,
    pub items_added: usize,
    /// Problems of the new copper: points off the board, tracks on a plane layer, and pieces
    /// closer to the copper of other nets than the clearance (routed tracks included). Copper
    /// that would touch another net is refused, and nothing is written.
    pub warnings: Vec<String>,
}

/// The options of one net as the tools return them (`gummiband_app::NetOptions`).
#[derive(Debug, Clone, Default, Serialize, JsonSchema)]
pub struct NetOption {
    /// Routing priority override (lower routes first).
    pub priority: Option<i32>,
    /// The net is left out of routing.
    pub skip: bool,
    /// Track width in mm the net is routed with.
    pub width: Option<f64>,
    /// Minimum width in mm: no necking below it, except in the courtyards of neck_refs.
    pub min_width: Option<f64>,
    /// Parts in whose courtyard the track may neck below the minimum width; left out: the
    /// parts whose pad on this net is narrower than min_width.
    pub neck_refs: Option<Vec<String>>,
}

impl From<&NetOptions> for NetOption {
    fn from(o: &NetOptions) -> Self {
        NetOption {
            priority: o.priority,
            skip: o.skip,
            width: o.width,
            min_width: o.min_width,
            neck_refs: o.neck_refs.clone(),
        }
    }
}

struct Entry {
    path: PathBuf,
    keep_locked: bool,
    kb: Arc<KicadBoard>,
    /// Per net options by net name (survive reloads).
    options: BTreeMap<String, NetOptions>,
}

#[derive(Clone)]
pub struct Gummiband {
    boards: Arc<Mutex<HashMap<String, Entry>>>,
    next_id: Arc<Mutex<usize>>,
    /// One route at a time: every route already uses all rayon threads.
    route_lock: Arc<tokio::sync::Mutex<()>>,
    tool_router: ToolRouter<Self>,
}

fn read(path: &Path, keep_locked: bool) -> Result<KicadBoard> {
    read_board(
        path,
        &ReadOptions {
            keep: if keep_locked {
                KeepCopper::Locked
            } else {
                KeepCopper::None
            },
            ..Default::default()
        },
    )
}

fn connections(b: &Board) -> usize {
    b.routable_nets().map(|n| b.nets[n as usize].pins.len() - 1).sum()
}

fn is_plane_net(b: &Board, net: usize) -> bool {
    !b.nets[net].planes.is_empty() || b.planes.iter().any(|p| p.net as usize == net)
}

// --------------------------------------------------------------------------- parameters

#[derive(Debug, Deserialize, JsonSchema)]
pub struct HolesIn {
    /// Drill diameter in mm (default 3.2, for M3).
    pub diameter: Option<f64>,
    /// Distance of each hole centre from the two edges of its corner in mm (default 4).
    pub inset: Option<f64>,
    /// Library footprint instead of a plain non-plated hole, e.g.
    /// "MountingHole:MountingHole_3.2mm_M3_Pad_Via".
    pub footprint: Option<String>,
    /// Net for the pads of a plated hole footprint, e.g. "GND".
    pub net: Option<String>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct NetClassIn {
    pub name: String,
    /// Clearance in mm (default: that of Default, 0.2).
    pub clearance: Option<f64>,
    /// Track width in mm (default: that of Default, 0.2).
    pub track_width: Option<f64>,
    /// Via pad diameter in mm (default: that of Default, 0.6).
    pub via_diameter: Option<f64>,
    /// Via drill in mm (default: that of Default, 0.3).
    pub via_drill: Option<f64>,
    pub diff_pair_width: Option<f64>,
    pub diff_pair_gap: Option<f64>,
    /// KiCad net class priority, lower wins when several patterns match (default: the order
    /// given, from 0). Default always has the lowest.
    pub priority: Option<i32>,
    /// Net name patterns with * and ?, e.g. "+5V*", "/Power/VBUS_*".
    #[serde(default)]
    pub patterns: Vec<String>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct CreateBoardIn {
    /// KiCad netlist in the s-expression format (Eeschema: File > Export > Netlist > KiCad,
    /// or `kicad-cli sch export netlist`).
    pub netlist: String,
    /// The .kicad_pcb to write; the .kicad_pro next to it is written or, if it exists (the
    /// schematic's project), merged.
    pub output: String,
    /// Replace an existing output file (default false). The working copy of the old board
    /// (<output>.prerouted.kicad_pcb with its project files) is removed, so the new board starts
    /// clean.
    pub overwrite: Option<bool>,
    /// Put every part (but the mounting holes) in rows below the board instead of inside the
    /// outline (default false): the board stays empty for the parts you place by hand and for
    /// copper you draw before the placement (add_locked_copper, add_zone), which would
    /// otherwise meet the pile of parts. place_parts with scope "outside" (or "all_unlocked")
    /// takes them in afterwards.
    pub parts_outside: Option<bool>,
    /// Board outline in mm (a rectangle).
    pub width: f64,
    pub height: f64,
    /// Top left corner of the outline in mm (default 100, 100).
    pub x0: Option<f64>,
    pub y0: Option<f64>,
    /// Corner radius of the outline in mm (default 0).
    pub corner_radius: Option<f64>,
    /// Copper layers (default 2; 4 gives F.Cu, In1.Cu, In2.Cu, B.Cu; any even count to 32).
    pub layers: Option<usize>,
    /// Inner layers of KiCad type "power", e.g. ["In1.Cu", "In2.Cu"]: Gummiband treats them as
    /// planes (not routed). Default: all signal.
    #[serde(default)]
    pub plane_layers: Vec<String>,
    /// Board thickness in mm (default 1.6), split over copper, prepreg and core.
    pub thickness: Option<f64>,
    /// Copper weight in oz per layer from F.Cu to B.Cu, or one value for all (default 1 oz =
    /// 0.035 mm).
    #[serde(default)]
    pub copper_oz: Vec<f64>,
    /// Prepreg thickness in mm between the outer and the next inner layer on boards with 4 or
    /// more layers (default 0.21); the cores take the rest.
    pub prepreg: Option<f64>,
    /// Four mounting holes at the corners, locked.
    pub mounting_holes: Option<HolesIn>,
    /// Net classes with their net name patterns, written to the .kicad_pro. A class named
    /// Default sets the defaults.
    #[serde(default)]
    pub net_classes: Vec<NetClassIn>,
    /// Design rules for the .kicad_pro (board.design_settings.rules), e.g.
    /// {"min_clearance": 0.127, "min_copper_edge_clearance": 0.3}.
    #[serde(default)]
    pub rules: BTreeMap<String, f64>,
    /// Footprint libraries: .pretty directories or directories that contain them. Searched
    /// first, then the project fp-lib-table (${KIPRJMOD}), KICAD10_FOOTPRINT_DIR,
    /// KICAD9_FOOTPRINT_DIR, KICAD_FOOTPRINT_DIR, the global fp-lib-table and the usual
    /// install places.
    #[serde(default)]
    pub footprint_dirs: Vec<String>,
    /// Netlist pins whose pad has another number, by reference: {"J1": {"S1": "SH"}} gives
    /// the pads SH of J1 the net of its symbol pin S1. The warnings suggest entries.
    #[serde(default)]
    pub pin_aliases: BTreeMap<String, BTreeMap<String, String>>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct MissingFootprintOut {
    pub reference: String,
    pub footprint: String,
    pub reason: String,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct UnmatchedOut {
    pub reference: String,
    pub footprint: String,
    /// Pad numbers without a netlist pin: they stay without a net.
    pub pads: Vec<String>,
    /// Netlist pins without a pad, "pin (net)": the net misses that connection.
    pub pins: Vec<String>,
    /// The pin_aliases entry to pass if the one pin left is the one pad left.
    pub suggested_alias: Option<serde_json::Value>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct CreateBoardOut {
    /// Id of the new board in this session (already loaded).
    pub board: String,
    pub path: String,
    pub project: String,
    /// The .kicad_pro existed (the schematic's project) and was merged.
    pub project_merged: bool,
    /// File format version: 20241229 (KiCad 9) or, with KiCad 10 footprint libraries, 20260206.
    pub version: u64,
    /// Footprints from the netlist on the board, plus mounting holes.
    pub footprints: usize,
    pub mounting_holes: usize,
    pub nets: usize,
    /// Numbered pads, and those with a net.
    pub pads: usize,
    pub pads_with_net: usize,
    /// Parts left off the board: their footprint was not found.
    pub missing_footprints: Vec<MissingFootprintOut>,
    /// Parts whose pad numbers and netlist pins do not all match.
    pub unmatched: Vec<UnmatchedOut>,
    /// Library nickname -> directory read.
    pub libraries: BTreeMap<String, String>,
    /// Symbols marked "exclude from board", not placed.
    pub excluded: Vec<String>,
    /// Parts that did not fit in rows inside the outline: they sit in rows below the board
    /// (place_parts with scope "outside" takes them in).
    pub outside: Vec<String>,
    /// Stale working copies of an earlier board under the same name that overwrite removed.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub removed: Vec<String>,
    /// Gap between the parts in the rows, mm.
    pub grid_gap: f64,
    pub warnings: Vec<String>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct LoadBoardIn {
    /// Path of a .kicad_pcb file (KiCad 9 or 10). Net classes come from the .kicad_pro and
    /// custom rules from the .kicad_dru with the same base name, if present.
    pub path: String,
    /// Keep locked tracks and vias as fixed copper (default false: everything is removed and
    /// routed from scratch, like `gummiband route`). Locked track ends join their net's pins.
    pub keep_locked: Option<bool>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct PartInfoIn {
    /// Board id from load_board.
    pub board: String,
    /// Footprint reference, e.g. U6.
    pub reference: String,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct LockedTrackIn {
    /// Exact net name.
    pub net: String,
    /// Copper layer name, e.g. F.Cu.
    pub layer: String,
    /// Track width in mm (default: the net class width).
    pub width: Option<f64>,
    /// Polyline as [x, y] points in mm, at least two.
    pub points: Vec<[f64; 2]>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct LockedViaIn {
    /// Exact net name.
    pub net: String,
    pub x: f64,
    pub y: f64,
    /// Diameter and drill in mm (default: the net class via).
    pub diameter: Option<f64>,
    pub drill: Option<f64>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct AddLockedIn {
    /// Board id from load_board.
    pub board: String,
    #[serde(default)]
    pub tracks: Vec<LockedTrackIn>,
    #[serde(default)]
    pub vias: Vec<LockedViaIn>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct MovePartIn {
    /// Board id from load_board.
    pub board: String,
    /// Footprint reference, e.g. C12.
    pub reference: String,
    /// New position in mm.
    pub x: f64,
    pub y: f64,
    /// New rotation in degrees (default: unchanged).
    pub rotation: Option<f64>,
    /// Let the tracks around the part follow like rubber bands (default true): its own tracks
    /// stretch, tracks of other nets give way within their clearances, nothing is routed again.
    /// The part stops short where the copper cannot give way any more or its courtyard meets
    /// another part or the edge; the message says where. Only for a move without rotation.
    pub follow: Option<bool>,
    /// "front" or "back": the side the part goes to (default: it keeps its side). A part on
    /// the other side is flipped as KiCad flips it (pads, graphics, texts mirrored, layers
    /// swapped); the tracks do not follow a flip. Use it to bring back a part that place_parts
    /// two_sided put on the back, or to put one there yourself.
    pub side: Option<String>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct GuideIn {
    /// A topic: "workflow" (default: how manual and automatic steps fit together), a tool name
    /// ("route", "place_parts", "lock", ...), a section such as "good_practice",
    /// "choosing_opt_in_options" or "open_points", "concepts" or "options". "index" or an unknown
    /// topic lists them all.
    pub topic: Option<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct GuideOut {
    /// The topic answered ("index" for the list).
    pub topic: String,
    /// Every topic the tool knows.
    pub topics: Vec<String>,
    /// The documentation as Markdown.
    pub text: String,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct ZoneFilterIn {
    /// Zone name, a glob with * and ? (power_copper's pours are named "gb: ...").
    pub name: Option<String>,
    /// Net name, a glob.
    pub net: Option<String>,
    /// Copper layer, e.g. In2.Cu.
    pub layer: Option<String>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct RemoveIn {
    /// Board id from load_board (a routed board too).
    pub board: String,
    /// Zones to remove; a zone goes if it matches every field of one filter (at least one
    /// field per filter). Zones inside footprints stay.
    #[serde(default)]
    pub zones: Vec<ZoneFilterIn>,
    /// Nets (globs) whose tracks, arcs and vias go, e.g. the via field power_copper laid out
    /// for a net. With `area` only those within it.
    #[serde(default)]
    pub nets: Vec<String>,
    /// [x0, y0, x1, y1] in mm: copper within this box goes (of every net, or of `nets`).
    pub area: Option<[f64; 4]>,
    /// Only copper on these layers (a via goes if it spans one of them).
    #[serde(default)]
    pub layers: Vec<String>,
    /// "tracks", "vias" or "all" (default).
    pub copper: Option<String>,
    /// "locked" or "unlocked": only that copper (default: both).
    pub only: Option<String>,
    /// Footprints to remove by reference, e.g. a mounting hole (H3). A part with pads on a
    /// net stays in the netlist; the result warns.
    #[serde(default)]
    pub parts: Vec<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct RemoveOut {
    /// Id of the working copy; use this one from now on.
    pub board: String,
    pub path: String,
    /// Removed zones as "name (net on layers)".
    pub zones: Vec<String>,
    pub tracks: usize,
    pub vias: usize,
    pub parts: Vec<String>,
    pub warnings: Vec<String>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct MountingHoleIn {
    /// Board id from load_board.
    pub board: String,
    /// Centre in mm.
    pub x: f64,
    pub y: f64,
    /// Hole diameter in mm (default 3.2, M3).
    pub diameter: Option<f64>,
    /// Reference (default: the next free H<n>).
    pub reference: Option<String>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct PlaceSilkIn {
    /// Board id from load_board (a routed board too).
    pub board: String,
    /// Only the references of these parts (default: all).
    #[serde(default)]
    pub parts: Vec<String>,
    /// Smallest text height in mm a reference may shrink to (default 0.8, JLCPCB's minimum
    /// for legible text).
    pub min_size: Option<f64>,
    /// Room to pads, other silk and texts in mm (default 0.1).
    pub margin: Option<f64>,
    /// Room to the board edge in mm (default 0.3).
    pub edge_margin: Option<f64>,
    /// How far from its courtyard a reference may go, mm (default 2).
    pub reach: Option<f64>,
    /// Hide a reference that finds no clear spot (default true; false leaves it where it is).
    pub hide_unplaced: Option<bool>,
    /// Also remove the silk graphics of footprints that run over or near the board edge
    /// (connector bodies drawn beyond it; KiCad's silk_edge_clearance). Default false.
    pub trim_edge: Option<bool>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct PlaceSilkOut {
    /// Id of the working copy; use this one from now on.
    pub board: String,
    pub path: String,
    /// Clear where they were.
    pub kept: usize,
    pub moved: Vec<String>,
    /// Moved with a smaller text: reference and new height in mm.
    pub shrunk: Vec<(String, f64)>,
    pub hidden: Vec<String>,
    /// No clear spot, left where they were (hide_unplaced false).
    pub left: Vec<String>,
    /// Footprints whose silk at the board edge was removed (trim_edge).
    pub trimmed: Vec<String>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct SetRulesIn {
    /// Board id from load_board.
    pub board: String,
    /// KiCad custom rules, one or more (rule ...) as in a .kicad_dru, e.g.
    /// (rule "fine pitch" (condition "A.intersectsCourtyard('U4')") (constraint clearance (min 0.1))).
    pub rules: String,
    /// Replace the rules of the working copy instead of adding to them (default false).
    pub replace: Option<bool>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct SetRulesOut {
    /// Id of the working copy; use this one from now on.
    pub board: String,
    /// The .kicad_dru written next to the working copy (KiCad's DRC reads it too).
    pub path: String,
    /// Rules in the file now.
    pub rules: usize,
    /// What Gummiband itself makes of them (routing and its own check); KiCad's drc applies
    /// all of them.
    pub understood: Vec<String>,
    pub warnings: Vec<String>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct TrySeedsIn {
    /// Board id from load_board: the state every seed starts from (parts placed by hand,
    /// locked copper, zones); it is restored before each seed.
    pub board: String,
    /// Placement seeds to try (default 1 to 4).
    #[serde(default)]
    pub seeds: Vec<u64>,
    /// Arguments of place_parts for every seed (without board and seed), e.g.
    /// {"scope": "all_unlocked", "semantic": true, "two_sided": true}.
    #[serde(default)]
    pub place: serde_json::Map<String, serde_json::Value>,
    /// Arguments of power_copper (without board) to lay out after each placement and before
    /// its route, e.g. {"paths": [...]}; leave it out for none.
    pub power_copper: Option<serde_json::Map<String, serde_json::Value>>,
    /// Arguments of route for every seed (without board and output), e.g. {"pairs": "pull"}.
    #[serde(default)]
    pub route: serde_json::Map<String, serde_json::Value>,
    /// Run KiCad's DRC on every result and rank by its errors (default true when kicad-cli is
    /// there; Gummiband's own check otherwise).
    pub drc: Option<bool>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct SeedOut {
    pub seed: u64,
    /// The placed working copy of this seed (kept, to go on from another seed).
    pub placed: String,
    /// Its routed file.
    pub routed: String,
    pub unrouted: usize,
    /// Gummiband's clearance violations.
    pub violations: usize,
    /// KiCad's DRC errors (silk and library findings left out) and unconnected items.
    pub drc_errors: Option<usize>,
    pub drc_unconnected: Option<usize>,
    pub vias: usize,
    pub length_mm: f64,
    /// Why the seed failed, if it did.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    /// Why KiCad's DRC did not run on it.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub drc_failed: Option<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct TrySeedsOut {
    /// Best first: fewest unrouted, then fewest DRC errors (or Gummiband violations), then
    /// fewest vias, then shortest.
    pub seeds: Vec<SeedOut>,
    pub best_seed: Option<u64>,
    /// Id of the working copy, now the placement of the best seed.
    pub board: Option<String>,
    /// Id of the routed file of the best seed.
    pub routed_board: Option<String>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct LockIn {
    /// Board id from load_board (a routed board too).
    pub board: String,
    /// Footprint references to lock: place_parts, route (move_parts) and move_part's rubber bands
    /// leave them where they are.
    #[serde(default)]
    pub parts: Vec<String>,
    /// Nets whose tracks, arcs and vias to lock: route keeps them as they are and routes the rest
    /// around them, shove and give do not move them.
    #[serde(default)]
    pub nets: Vec<String>,
    /// Every track, arc and via of the board (default false), e.g. to freeze a routed result
    /// before routing more nets or editing by hand.
    pub all_copper: Option<bool>,
    /// Unlock instead of lock (default false), so that the router may move or replace them again.
    pub unlock: Option<bool>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct LockOut {
    /// Id of the working copy; use this one from now on.
    pub board: String,
    /// The working copy (<input>.prerouted.kicad_pcb); the input file stays untouched.
    pub path: String,
    /// true: locked, false: unlocked.
    pub locked: bool,
    /// References whose lock state changed.
    pub parts: Vec<String>,
    /// Tracks and arcs whose lock state changed.
    pub tracks: usize,
    pub vias: usize,
    /// Unknown references, nets without copper.
    pub warnings: Vec<String>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct PlacePartsIn {
    /// Board id from load_board.
    pub board: String,
    /// Place only these references; everything else stays. Takes precedence over `scope`.
    #[serde(default)]
    pub only: Vec<String>,
    /// Without `only`: "unlocked" (default: all but locked parts, connectors, modules, mounting
    /// holes, buttons and test points), "all_unlocked" (those too, locked parts stay) or
    /// "outside" (the parts outside the board outline).
    pub scope: Option<String>,
    /// References that stay in any case, e.g. the parts you placed yourself with move_part.
    #[serde(default)]
    pub fixed: Vec<String>,
    /// Default 1; another seed gives another set of starts, none shared with a neighbouring seed.
    pub seed: Option<u64>,
    /// Independent runs (default 1); the one with the fewest crossings is kept. The crossings
    /// hardly predict the routed result: for another attempt change the seed instead.
    pub starts: Option<usize>,
    /// Spacing between courtyards in mm (default 0.3).
    pub gap: Option<f64>,
    /// Extra room around parts with five or more pads in mm (default 0.3).
    pub halo: Option<f64>,
    /// Use the schematic beyond the netlist (default false): parts of a sheet and clusters
    /// around an IC stay together, an inductor sits at the switch pins of its converter,
    /// decoupling capacitors prefer an IC of their own sheet, and wide nets and switch nodes
    /// count their full length, so an input chain stays in order. Connectors and power stages
    /// you care about still belong in `fixed`.
    pub semantic: Option<bool>,
    /// Experimental: keep a band free for every group of tracks between two parts (ICs and
    /// connectors that share at least two signal nets, a bus, SPI, I2C); other parts pay for the
    /// area of the band they cover. Default false.
    pub corridors: Option<bool>,
    /// Experimental: room for the escape per side of a part with five or more pads, from its
    /// signal pins on that side, instead of the same halo all round. Default false.
    pub escape_halo: Option<bool>,
    /// Parts with 4 or more pads (ICs, connectors) stay within this many mm of where they sit
    /// (default 0: free). A local change instead of a new placement: with `only` and
    /// `corridors` it moves the parts that track_corridors lists out of a band while the ICs
    /// keep their place; route's clear_corridors uses 1.5.
    pub leash: Option<f64>,
    /// Place on both sides (default false: one room for both sides, every part keeps its
    /// side). Each side gets its own room; a through hole part takes its holes on the other
    /// side (the room under its body stays free). SMD parts with at most `back_max_pads` pads
    /// may change sides. They pay a little for the back and for a net that
    /// then needs a via, so the back takes what does not fit on the front or sits better there
    /// (a decoupling capacitor under its IC). For a dense board whose parts do not fit on one
    /// side: the error of a one sided placement says when the back has the room.
    pub two_sided: Option<bool>,
    /// With `two_sided`: parts with more pads keep their side (default 8: passives, diodes,
    /// transistors, small regulators up to SOIC-8).
    pub back_max_pads: Option<usize>,
    /// With `two_sided`: cost of a part on the back in mm of wire (default 6). A part goes to
    /// the back if the front has no room for it or the back saves at least this much; 0 treats
    /// both sides alike, a large value keeps the back for what does not fit.
    pub back_cost: Option<f64>,
    /// With `two_sided`: movable SMD parts that go on the front, or on the back, whatever their
    /// pads (e.g. a large IC on the back under a module, or a part that has to stay on top).
    #[serde(default)]
    pub on_front: Vec<String>,
    #[serde(default)]
    pub on_back: Vec<String>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct TrackCorridorsIn {
    /// Board id from load_board.
    pub board: String,
    /// References that count as fixed in addition to the locked and mechanical parts: they are
    /// never listed as in the way.
    #[serde(default)]
    pub fixed: Vec<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct CorridorOut {
    /// References of the two parts the group of tracks connects.
    pub a: String,
    pub b: String,
    /// The signal nets of the group.
    pub nets: Vec<String>,
    /// Width of the band in mm: the tracks of the group with their clearances.
    pub width_mm: f64,
    /// Length of the band in mm, between the pads of the group on the two parts.
    pub length_mm: f64,
    /// Movable parts in the band that are not on its nets.
    pub in_the_way: Vec<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct TrackCorridorsOut {
    pub corridors: Vec<CorridorOut>,
    /// Every part in the way of some band, once: what route with clear_corridors would place
    /// again.
    pub in_the_way: Vec<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct PartPose {
    pub reference: String,
    pub x: f64,
    pub y: f64,
    pub rotation: f64,
    /// The part sits on the back (B.Cu).
    #[serde(skip_serializing_if = "std::ops::Not::not")]
    pub back: bool,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct PlacePartsOut {
    /// Id of the working copy; route this one.
    pub board: String,
    /// The working copy (<input>.prerouted.kicad_pcb); the input file stays untouched.
    pub path: String,
    pub placed: Vec<PartPose>,
    /// Decoupling capacitors with the part they were assigned to.
    pub decoupling: Vec<(String, String)>,
    /// Crossings between the spanning trees of the nets: a measure of the vias to expect.
    pub crossings: usize,
    /// Smallest distance between two courtyards in mm.
    pub min_separation: f64,
    /// Pins outside the plane of their net; routing reaches them.
    pub off_plane: Vec<String>,
    /// Parts that stayed because locked copper ends on one of their pads.
    pub anchored: Vec<String>,
    /// With `semantic`: inductors with the converter they were pulled to.
    pub coils: Vec<(String, String)>,
    /// With `semantic`: groups kept together (sheet, or sheet/anchor IC) with their parts.
    pub groups: Vec<GroupOut>,
    /// With `semantic`: two-pin parts and small regulators with the part they were pulled to.
    pub ties: Vec<(String, String)>,
    /// Connectors, buttons and test points outside the scope that were placed anyway because
    /// they sat piled up, on a pad of a larger part or off the board.
    pub released: Vec<String>,
    /// Pads of a plane net without room for their fan-out via (kept copper, a hole or a rule
    /// area around them on some layer).
    pub no_via_room: Vec<String>,
    /// Released parts, parts that stay piled up or with a courtyard off the board, pads without
    /// room for their via.
    pub warnings: Vec<String>,
    /// With `two_sided`: placed parts that changed sides.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub flipped: Vec<String>,
    /// With `two_sided`: free room in mm2 on the front and on the back.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub free_mm2: Option<SidesOut>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct SidesOut {
    pub front: f64,
    pub back: f64,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct GroupOut {
    pub name: String,
    pub parts: Vec<String>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct AddZoneIn {
    /// Board id from load_board.
    pub board: String,
    pub net: String,
    /// Copper layer name, e.g. In1.Cu. A plane on an inner layer.
    pub layer: String,
    /// Outline as [x, y] points in mm, at least three.
    pub points: Vec<[f64; 2]>,
    /// Zone priority (default 0).
    pub priority: Option<u32>,
    /// How the pads of the net join the zone: "thermal" (default), "solid" or "thermal_tht"
    /// (reliefs on through hole pads, SMD pads solid).
    pub pad_connection: Option<String>,
    /// Width of the thermal spokes in mm (default 0.3); wider for pins that carry amperes.
    pub thermal_spoke_width: Option<f64>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct LayerTypeIn {
    /// Copper layer name, e.g. In2.Cu.
    pub layer: String,
    /// KiCad layer type: "signal", "power" (a plane layer: its zones are planes the router
    /// reaches with vias, no tracks on it), "mixed" or "jumper" (both routed like signal).
    #[serde(rename = "type")]
    pub kind: String,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct SetLayerTypeIn {
    /// Board id from load_board.
    pub board: String,
    pub layers: Vec<LayerTypeIn>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct LayerTypeOut {
    pub layer: String,
    pub old_type: String,
    #[serde(rename = "type")]
    pub kind: String,
    /// Nets with a zone on the layer (their planes when the type is power).
    pub zone_nets: Vec<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct SetLayerTypeOut {
    /// Id of the working copy; use it from now on.
    pub board: String,
    pub path: String,
    pub changed: Vec<LayerTypeOut>,
    /// All copper layers after the change.
    pub layers: Vec<LayerOut>,
    /// A power layer without zone, tracks left on a new power layer, zones that stop being
    /// planes.
    pub warnings: Vec<String>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct ListNetsIn {
    /// Board id from load_board.
    pub board: String,
    /// Case insensitive substring of the net or net class name.
    pub filter: Option<String>,
    /// Include nets with fewer than 2 pins (nothing to route). Default false.
    pub include_single: Option<bool>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct SetNetOptionsIn {
    /// Board id from load_board.
    pub board: String,
    /// Exact net name as list_nets returns it.
    pub net: String,
    /// Routing priority (lower routes first; the default is the net class priority).
    pub priority: Option<i32>,
    /// true: leave this net unrouted (it keeps no tracks in the output); false: route it again.
    pub skip: Option<bool>,
    /// true: drop all options of this net first.
    pub reset: Option<bool>,
    /// Track width in mm the net is routed with; 0 clears it. On its own the track may still
    /// neck as its class allows: for a current path give min_width too.
    pub width: Option<f64>,
    /// Minimum width in mm: the track never necks below it, except in the courtyards of
    /// neck_refs (a larger minimum of the class or the custom rules stays); 0 clears it. Set it
    /// to the width for a track that must carry its current everywhere.
    pub min_width: Option<f64>,
    /// References of the parts in whose courtyard the track may neck below min_width. Left out:
    /// the parts whose pad on this net is narrower than min_width. An empty list: nowhere.
    pub neck_refs: Option<Vec<String>>,
}

/// An option with a fixed list of words: the word, or the true or false of an earlier version
/// that had a switch under the same name (`gummiband_app::old`). The schema has the words only.
#[derive(Debug, Clone, Deserialize)]
#[serde(untagged)]
pub enum WordOrSwitch {
    Switch(bool),
    Word(String),
}

/// The schema of an option with a fixed list of words, from the table of gummiband-app: its
/// words, and a description with one line per word and the default.
fn choice_schema(name: &str) -> schemars::Schema {
    let doc = gummiband_app::option_doc(name).expect("an option of the table");
    let words: Vec<&str> = doc.choices.iter().map(|c| c.value).collect();
    schemars::json_schema!({"type": "string", "enum": words, "description": doc.describe()})
}

fn pairs_schema(_: &mut schemars::SchemaGenerator) -> schemars::Schema {
    choice_schema("pairs")
}

fn preset_schema(_: &mut schemars::SchemaGenerator) -> schemars::Schema {
    choice_schema("preset")
}

/// The default of `scope` in a session: the board was loaded with or without its locked copper.
const SCOPE_DEFAULT: &str = "keep_locked for a board that was loaded with keep_locked and for a working copy (the board id of add_locked_copper, lock, move_part, place_parts and the other tools that edit), else fresh";

fn scope_schema(_: &mut schemars::SchemaGenerator) -> schemars::Schema {
    let doc = gummiband_app::option_doc("scope").expect("an option of the table");
    let words: Vec<&str> = doc.choices.iter().map(|c| c.value).collect();
    schemars::json_schema!({"type": "string", "enum": words, "description": doc.describe_with(SCOPE_DEFAULT)})
}

/// The arguments of `route`: the options of the pipeline (`gummiband_app::OPTIONS`, each under
/// its name) and what the session adds (`board`, `output`, `max_violations`). An argument that
/// does not exist is an error, not a typo that goes unnoticed.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RouteIn {
    /// Board id from load_board.
    pub board: String,
    // the description and the words come from the table of gummiband-app
    #[serde(default)]
    #[schemars(schema_with = "preset_schema")]
    pub preset: Option<String>,
    /// Portfolio size: router runs with different orders (default: what `preset` says, 14 with
    /// "standard"; fixed so that results do not depend on the machine).
    pub runs: Option<usize>,
    // the description and the words come from the table of gummiband-app
    #[serde(default)]
    #[schemars(schema_with = "scope_schema")]
    pub scope: Option<String>,
    /// The nets this call lays; all others are left out. With scope "finish": only the open
    /// connections of these nets. With scope "reroute" (which needs them): the nets to lay
    /// again. With scope "fresh" and "keep_locked": only these nets get copper, the unlocked
    /// copper of all others is gone from the output.
    pub nets: Option<Vec<String>>,
    /// With scope "reroute", and with scope "finish" and `nets`: the unlocked tracks of the
    /// other nets are part of the topology, so a connection passes between them and they shift
    /// aside; what it does not touch stays as it is in the file (default true there; false keeps
    /// them where they are). Refused elsewhere, and together with move_parts.
    pub shove: Option<bool>,
    /// With scope "reroute", and with scope "finish" and `nets`: if a connection finds no way,
    /// the unlocked tracks of other nets within this many mm of its pins give way and are routed
    /// again, in growing steps up to this radius; the try with the fewest open connections is
    /// kept (default 8 there; 0: nothing gives way). Locked copper and plane nets never move.
    pub give: Option<f64>,
    /// Seed of the search. The same seed gives the same result; another one a different try.
    pub seed: Option<u64>,
    // the description and the words come from the table of gummiband-app
    #[serde(default)]
    #[schemars(schema_with = "pairs_schema")]
    pub pairs: Option<WordOrSwitch>,
    /// With pairs "pull": weight of the pull between the two nets of a pair (default 1, a good
    /// start; up to about 4; it can cost clearance on tight boards). `coupling` says where and
    /// why a pair stays apart. With pairs "band": the same pull for the pairs that get no wide
    /// track (default 0). Not with pairs "off".
    pub pair_pull: Option<f64>,
    /// With pairs "band": length difference left within a pair in mm (default 0.1).
    pub pair_skew: Option<f64>,
    /// With pairs "band": target differential impedance in ohm; sets the track width per layer
    /// from the stackup, the gap stays.
    pub pair_zdiff: Option<f64>,
    /// With pairs "band": a wide track for every pair recognised by name (default false: only
    /// pairs with a `diff_pair_gap` rule or a net class of their own).
    pub pair_all: Option<bool>,
    /// With pairs "band", experimental: the wide track ends just outside the pads, two single
    /// tracks connect them (default false).
    pub breakouts: Option<bool>,
    /// With pairs "band", experimental: two vias side by side are one via pair, a capsule that
    /// turns with the wide track (default false).
    pub via_pairs: Option<bool>,
    /// Weight of the preferred layer directions in every third run (default 0.5, 0: off).
    pub layer_dirs: Option<f64>,
    /// Hints from the local patterns of board_patterns, comma separated (default none), as
    /// `gummiband route --patterns`. All are opt-in because none wins on every board:
    /// `groups` (connections between two chips or a chip and a connector as one block in pin
    /// order, the longest block first; `groups-short` shortest first) saves vias on boards with several chips and buses; `pairs` (differential pairs
    /// first, longest first) couples pairs better at a few vias; `pairs,groups` was best on an
    /// FPGA board; `groups-short` suited a DDR fan-out; `decaps` (a decoupling capacitor to its IC
    /// pin first) helps boards with many decaps on one side; `rows` (the open end re-lays pin
    /// rows in pin order) only acts with open_end_seconds; `shares` (neighbouring plane pins
    /// share a via) usually costs vias. On a board whose placement is final and already
    /// routes cleanly, hints rarely help. `all` or `none`.
    pub patterns: Option<String>,
    /// Experimental: before routing, movable parts that sit in the band of a group of tracks
    /// between two parts (see track_corridors) and are not on its nets are placed again around
    /// it, ICs and connectors at most 1.5 mm from where they sit; decoupling capacitors stay.
    /// The result goes into the working copy <input>.prerouted.kicad_pcb, and the route uses it
    /// (`board` of the result). Pairs with patterns groups; it gives most on tight boards with
    /// several chips. To decide yourself what moves, use track_corridors and place_parts
    /// instead. Not with scope "finish" or "reroute".
    pub clear_corridors: Option<bool>,
    /// Generations of the evolutionary order search after the portfolio (default: what
    /// `preset` says, 3 with "standard").
    pub generations: Option<usize>,
    /// Generations that select by clearance violations after the geometry step (default: what
    /// `preset` says, 3 with "standard", as the CLI and the plugin; 0 is about three times
    /// faster and leaves many more violations).
    pub geometry_generations: Option<usize>,
    /// Let parts next to clearance problems and open connections shift a little while the
    /// routed copper follows like rubber bands (default true with scope "fresh" and
    /// "keep_locked", false with "finish" and "reroute", where the kept copper would not follow).
    /// Locked footprints, connectors, modules, mounting holes, buttons and test points stay, as
    /// do the references in `fixed`. The output has the footprints at their new positions, and
    /// `moved` lists them.
    pub move_parts: Option<bool>,
    /// References that never move under move_parts and clear_corridors.
    #[serde(default)]
    pub fixed: Vec<String>,
    /// Copper layers treated as planes besides those of KiCad type "power": not routed, their
    /// zones stay (as `gummiband --plane-layers`). To make it lasting, use set_layer_type.
    #[serde(default)]
    pub plane_layers: Vec<String>,
    /// How nets with an area (a zone of the file, or a pour of `pour`) on a routed layer are
    /// connected: "tracks" (default) lays them like any net and does not look at the area,
    /// "fill" does not lay them as tracks: the fill connects them, and `areas` in the result
    /// says per net which pads it does not reach.
    pub areas: Option<String>,
    /// With areas "fill": nets that get a pour after the route (ground_pour), each "NET" or
    /// "NET:LAYER,LAYER" (the outer layers where none is given). The route takes the pour as
    /// an area of the net; it does not write the zone.
    pub pour: Option<Vec<String>>,
    /// With areas "fill": an island of fill that holds pads gets one via where it joins the
    /// area, set on the finished copper where the fill model finds the spot (default true with
    /// areas "fill"). false: such a pad gets a track to the area instead.
    pub area_stitch: Option<bool>,
    /// Output .kicad_pcb (default: `<input>.routed.kicad_pcb` next to the input). The
    /// .kicad_pro and .kicad_dru are copied next to it if missing.
    pub output: Option<String>,
    /// Via site grid pitch in mm (default 0.5).
    pub via_pitch: Option<f64>,
    /// Via cost in mm of track (default 3, as the CLI).
    pub via_cost: Option<f64>,
    /// Rounds of negotiated routing from the start, in a second search (20 works well and takes
    /// about twice the time). With the default 0 the search negotiates on its own when the plain
    /// one leaves a few connections open.
    pub negotiate: Option<usize>,
    /// Factor on the via cost in the pass that routes connections with vias again and keeps
    /// what has fewer vias (default 4; 0: off), as `gummiband route --via-pass`.
    pub via_pass: Option<f64>,
    /// Open end with a time budget: after the normal search keep improving for this many
    /// seconds, then finish the best result (default: what `preset` says, 0 = off with
    /// "standard", 600 with "final"), as `gummiband route --open-end-seconds`. The budget starts when the open end starts, so it never cuts the
    /// normal search short. Mutants take a few connections around one spot out and lay them
    /// again; when that stops helping, bolder side branches take over. No placement rounds
    /// while it runs. Results that differ in vias and length come back in `alternatives`.
    pub open_end_seconds: Option<f64>,
    /// With open_end_seconds: what a via is worth in mm of track when the open end judges its
    /// results (default 20). Lower gives more vias and shorter tracks, higher fewer vias and
    /// longer ones.
    pub open_via_cost: Option<f64>,
    /// Reserve in mm on top of the clearances that the finished copper gets where there is room
    /// (default: 0.03 with open_end_seconds, but 0 with scope "reroute" and with scope "finish"
    /// and `nets`, where the kept copper stays as it is; else 0). The result stays only if it
    /// brings no new violation.
    pub air: Option<f64>,
    /// Tracks of the default net class grow up to this width in mm where there is room, at the
    /// end (default 0: off; a track_width rule with opt sets it per class).
    pub prefer_width: Option<f64>,
    /// Track angles of the result: "any" (default) keeps the any-angle tracks with arcs, "45"
    /// turns them into conventional 45 degree routing as a last step (straight pieces in eight
    /// directions, chamfered corners, a few percent more length). It never adds a clearance
    /// violation; where no 45 degree path fits, a piece keeps its angle (see `angles_45`).
    pub angles: Option<String>,
    /// Meanders for the length and skew rules of the .kicad_dru (default true; without such
    /// rules nothing is tuned). Each meander is written as a KiCad tuning pattern.
    pub tune: Option<bool>,
    /// Highest meander bump in mm (default 1).
    pub tune_amplitude: Option<f64>,
    /// Air between neighbouring meander legs in track widths, at least the clearance (default 2).
    pub tune_spacing: Option<f64>,
    /// Try this many placements from the netlist (as place_parts makes them), route each with
    /// these settings and go on with the best: fewest open connections, then clearance
    /// violations, vias and length (default 0: the placement stays). The output has the parts
    /// where the best placement put them, `placement` lists every try; the working copy is not
    /// touched. To keep and compare the placements themselves, use try_seeds. Not with
    /// scope "finish" or "reroute", nets or pairs "band".
    pub placements: Option<usize>,
    /// First seed of placements, and the seed of clear_corridors (default 1).
    pub placement_seed: Option<u64>,
    /// Path of a JSON file with options per net, one entry per net name: a track width in mm
    /// ({"GND": 0.5}) or an object with width, min_width, neck_refs, priority and skip, the
    /// fields of set_net_options (as `gummiband route --net-widths`). What set_net_options
    /// holds for a net wins field by field. Nets the board lacks are passed over.
    pub net_widths: Option<String>,
    /// Path of a JSON file with a routing priority per net, {"net name": priority}; lower
    /// routes first. Wins over a priority in net_widths, set_net_options over both.
    pub net_priorities: Option<String>,
    /// Switch on KiCad's teardrops for vias and pads in the output and for track ends in its
    /// project (default false); KiCad builds them when it fills the zones.
    pub teardrops: Option<bool>,
    /// Folder for one SVG of the best candidate per generation (a PNG of each if rsvg-convert
    /// is there, and evolution.gif if ImageMagick is), to look at how the search went.
    pub frames: Option<String>,
    /// Maximum number of violations listed (default 50; the total is always given).
    pub max_violations: Option<usize>,
    // The three switches of an earlier version that `scope` replaces. They are still taken
    // and mapped onto it (`gummiband_app::old::scope`), and are in no schema.
    #[serde(default)]
    #[schemars(skip)]
    pub keep_locked: Option<bool>,
    #[serde(default)]
    #[schemars(skip)]
    pub keep_all: Option<bool>,
    #[serde(default)]
    #[schemars(skip)]
    pub reroute: Option<bool>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct Angles45In {
    /// Path of a routed .kicad_pcb (for example the output of route).
    pub path: String,
    /// Output .kicad_pcb (default: `<input>.45.kicad_pcb` next to the input).
    pub output: Option<String>,
    /// Only the tracks of these nets (default: all nets).
    pub nets: Option<Vec<String>>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct Angles45Out {
    pub output: String,
    /// Runs of track between vias, branches and pads that were looked at, and those that changed.
    pub tracks: usize,
    pub tracks_changed: usize,
    #[serde(flatten)]
    pub angles: AnglesOut,
    /// Gummiband's own clearance check before and after (drc has the exact answer).
    pub violations_before: usize,
    pub violations_after: usize,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct DrcIn {
    /// Path of a .kicad_pcb (for example the output of route). A .kicad_pro with the same
    /// base name is needed; it is copied from the original project if missing.
    pub path: String,
    /// Maximum number of unconnected pairs and worst clearance violations listed (default 20).
    pub max_items: Option<usize>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct Region {
    pub x0: f64,
    pub y0: f64,
    pub x1: f64,
    pub y1: f64,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct RenderIn {
    /// Path of a .kicad_pcb.
    pub path: String,
    /// KiCad layer names to plot (default: the routing copper layers, i.e. no inner planes,
    /// and Edge.Cuts).
    pub layers: Option<Vec<String>>,
    /// Region in board coordinates (mm). Default: the board outline plus 1 mm.
    pub region: Option<Region>,
    /// Image width in pixels (default 1000, at most 4000). The height follows the region.
    pub width_px: Option<u32>,
    /// Background colour for rsvg-convert (default "white").
    pub background: Option<String>,
    /// Where to write the PNG (default: a file in the server's temp folder).
    pub output: Option<String>,
    /// Fill the zones with kicad-cli before plotting (default true when the file has a zone
    /// without fill, e.g. after add_zone or ground_pour); the file itself stays as it is.
    pub refill: Option<bool>,
}

// ------------------------------------------------------------------------------ results

#[derive(Debug, Serialize, JsonSchema)]
pub struct LayerOut {
    pub index: usize,
    pub name: String,
    /// Inner plane layer: only crossed by vias.
    pub plane: bool,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct OutlineOut {
    pub x0: f64,
    pub y0: f64,
    pub x1: f64,
    pub y1: f64,
    pub width_mm: f64,
    pub height_mm: f64,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct ClassOut {
    pub name: String,
    pub priority: i32,
    pub track_width: f64,
    pub clearance: f64,
    pub via_diameter: f64,
    pub via_drill: f64,
    /// Nets of this class with 2 or more pins.
    pub nets: usize,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct PlaneOut {
    pub net: String,
    pub layer: String,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct LoadBoardOut {
    /// Board id for the other tools.
    pub board: String,
    pub path: String,
    pub keep_locked: bool,
    pub layers: Vec<LayerOut>,
    pub outline: OutlineOut,
    pub pins: usize,
    pub nets: usize,
    /// Nets with 2 or more pins.
    pub routable_nets: usize,
    /// Two pin connections as `gummiband info` counts them (pins - 1 per routable net); route
    /// counts plane nets per fan-out instead, so its total is lower on boards with planes.
    pub connections: usize,
    pub planes: Vec<PlaneOut>,
    pub net_classes: Vec<ClassOut>,
    /// `(rule ...)` entries in the .kicad_dru.
    pub custom_rules: usize,
    /// Net pairs with a custom track clearance from those rules.
    pub custom_clearance_pairs: usize,
    /// Net options kept from an earlier load of the same file.
    pub net_options: BTreeMap<String, NetOption>,
    /// Differential pairs recognised by net name (`..P`/`..N`, `..+`/`..-`). route couples
    /// them only with `pairs` "pull" or "band".
    pub diff_pairs: Vec<DiffPairOut>,
    /// Nets that are a ground by name (GND, AGND, PGND, VSS, ...).
    pub ground_nets: Vec<String>,
    /// Things to look at before routing: more than one ground net.
    pub warnings: Vec<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct DiffPairOut {
    pub p: String,
    pub n: String,
    pub class: String,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct NetOut {
    pub name: String,
    pub class: String,
    pub pins: usize,
    /// Connected through an inner plane (pins fan out to vias).
    pub plane: bool,
    /// Effective routing priority (lower first).
    pub priority: i32,
    pub skip: bool,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct ListNetsOut {
    pub board: String,
    /// Matching nets (all nets if no filter).
    pub count: usize,
    pub nets: Vec<NetOut>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct SetNetOptionsOut {
    pub board: String,
    pub net: String,
    pub options: NetOption,
    /// Effective priority now.
    pub priority: i32,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct ViolationOut {
    /// Kind and items, e.g. "track-pad".
    pub what: String,
    pub x: f64,
    pub y: f64,
    pub actual_mm: f64,
    pub required_mm: f64,
    /// The routed item's net first; the second is null for obstacles without a net.
    pub nets: (String, Option<String>),
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct PortfolioOut {
    pub generation: usize,
    pub seed: u64,
    pub fanout_first: bool,
    pub routed: usize,
    pub connections: usize,
    pub vias: usize,
    pub length_mm: f64,
    /// Clearance violations after the geometry step (null if this run was not realized).
    pub violations: Option<usize>,
    /// Of those, copper of two nets touching or crossing (null if not realized).
    pub shorts: Option<usize>,
}

/// A differential pair after `route` with `pairs`.
#[derive(Debug, Serialize, JsonSchema)]
pub struct PairOut {
    pub p: String,
    pub n: String,
    /// False if the pair was routed as two separate nets.
    pub wide: bool,
    /// Why the pair or a part of it was not routed as one wide track.
    pub note: String,
    pub width_mm: f64,
    pub gap_mm: f64,
    /// Differential impedance from the stackup in ohm, 0 if unknown.
    pub zdiff_ohm: f64,
    pub length_p_mm: f64,
    pub length_n_mm: f64,
    /// Largest length difference over the pair's paths from pad to pad.
    pub skew_mm: f64,
    pub vias: usize,
    /// Polarity swaps through the pair's vias.
    pub swaps: usize,
}

/// A length, skew, uncoupled or gap rule of the .kicad_dru and how the result keeps it, measured
/// as KiCad's DRC measures.
#[derive(Debug, Serialize, JsonSchema)]
pub struct LengthRuleOut {
    pub rule: String,
    /// "length", "skew", "skew within pairs", "uncoupled" or "gap".
    pub kind: String,
    pub min_mm: Option<f64>,
    pub opt_mm: Option<f64>,
    pub max_mm: Option<f64>,
    /// Nets (pairs for uncoupled) the rule holds for.
    pub nets: usize,
    /// Those out of range.
    pub failed: Vec<LengthNetOut>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct LengthNetOut {
    pub net: String,
    /// Length as KiCad measures it (for uncoupled: the uncoupled length).
    pub length_mm: f64,
    /// Difference to the longest net of the group, for skew rules.
    pub skew_mm: Option<f64>,
}

/// A net the length tuning lengthened.
#[derive(Debug, Serialize, JsonSchema)]
pub struct TunedOut {
    pub net: String,
    pub before_mm: f64,
    pub after_mm: f64,
    pub aim_mm: f64,
    /// Meandered together with its pair partner.
    pub as_pair: bool,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct CouplingOut {
    pub p: String,
    pub n: String,
    /// Track of both nets within width + gap + 0.1 mm of the partner on the same layer.
    pub coupled_mm: f64,
    /// Track of both nets.
    pub length_mm: f64,
    pub vias_p: usize,
    pub vias_n: usize,
    /// With `pairs` "pull" or "band": the longest stretches (0.5 mm and more, at most 5) that do
    /// not run beside the partner, with where and why.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub uncoupled: Vec<UncoupledOut>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct UncoupledOut {
    pub net: String,
    pub layer: String,
    /// Middle of the stretch.
    pub x: f64,
    pub y: f64,
    pub length_mm: f64,
    /// "partner on another layer", "partner farther away" (a breakout to the pins, or a way of
    /// its own), "something between" (copper, a pad or a via keeps the two apart).
    pub why: String,
}

/// The area of one net after a route with areas "fill".
#[derive(Debug, Serialize, JsonSchema)]
pub struct RouteAreaOut {
    pub net: String,
    /// The finding in one sentence.
    pub summary: String,
    pub pads: usize,
    pub reached: usize,
    /// The pads the fill does not reach, each with the reason: island, walled, no_spoke,
    /// outside or other_layer.
    pub unreached: Vec<String>,
}

/// The fields of `gummiband route --report`, plus a few for the session.
#[derive(Debug, Serialize, JsonSchema)]
pub struct BlockedOut {
    pub connection: String,
    pub nets: Vec<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct NetStatOut {
    pub net: String,
    pub length_mm: f64,
    pub vias: usize,
    pub min_width_mm: f64,
    pub max_width_mm: f64,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct RouteOut {
    pub board: String,
    /// Footprints that move_parts shifted, and by how much.
    pub moved: Vec<MovedOut>,
    pub input: String,
    pub output: String,
    /// The seed the search ran with (`seed`, or the default): the same seed on the same board
    /// gives this result again.
    pub seed: u64,
    /// Wall time of the whole route in seconds, from reading the board to the checked result
    /// (writing the files not included); the same number as in the report of `gummiband route`.
    pub seconds: f64,
    /// The part of `seconds` up to the end of the search, without the steps after it (room for
    /// meanders, 45 degrees, length tuning, checks).
    pub search_seconds: f64,
    pub threads: usize,
    pub connections: usize,
    pub routed: usize,
    /// Unrouted connections ("net: pad -> pad").
    pub unrouted: Vec<String>,
    /// For each unrouted connection the nets running in its way, to prioritize or move them.
    pub blocked: Vec<BlockedOut>,
    pub vias: usize,
    pub length_mm: f64,
    pub items_written: usize,
    /// Nets left out by set_net_options or the `nets` argument.
    pub skipped_nets: Vec<String>,
    /// With areas "fill": per net left to its fill what the fill reaches on the finished
    /// copper, judged by the router's fill model (area_report on the written file has the
    /// details).
    pub areas: Vec<RouteAreaOut>,
    /// Of `vias`, those set for islands of fill (area_stitch).
    pub area_vias: usize,
    /// Gummiband's own clearance check on the result.
    pub violations_total: usize,
    pub violations_by_kind: BTreeMap<String, usize>,
    /// The first max_violations violations.
    pub violations: Vec<ViolationOut>,
    pub portfolio: Vec<PortfolioOut>,
    /// The pairs laid as a wide track, with pairs "band".
    pub pairs: Vec<PairOut>,
    /// Every recognised differential pair, whatever the settings: how much of it runs next to
    /// its partner.
    pub coupling: Vec<CouplingOut>,
    /// Hints on the settings: an argument in the spelling of an earlier version and what to write
    /// now, pairs that were not asked to couple.
    pub notes: Vec<String>,
    /// Per routed net: track length, vias and widths, longest first.
    pub nets: Vec<NetStatOut>,
    /// The .kicad_pro and .kicad_dru of the input, copied next to the output where they were
    /// missing.
    pub copied: Vec<String>,
    /// With open_end_seconds: the other results the open end leaves to choose from, those that
    /// differ from the output in vias and length, written next to it as
    /// <output>.altN.kicad_pcb. load_board one of them to go on with it.
    pub alternatives: Vec<AlternativeOut>,
    /// What the 45 degree step did, with `angles` "45".
    pub angles_45: Option<AnglesOut>,
    /// The length tuning: every net it aimed at.
    pub length_tuning: Vec<TunedOut>,
    /// The length, skew, uncoupled and gap rules of the .kicad_dru after the tuning.
    pub length_rules: Vec<LengthRuleOut>,
    /// With clear_corridors: the parts that were placed again before the routing.
    pub cleared_corridors: Vec<String>,
    /// With placements: the seed of the placement kept and every placement tried.
    pub placement: Option<PlacementOut>,
    /// With frames: the folder, the number of pictures and the animation, if one was made.
    pub frames: Option<FramesOut>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct PlacementOut {
    /// Seed of the placement the result is routed on.
    pub seed: u64,
    /// The parts it placed.
    pub moved: Vec<String>,
    pub tried: Vec<PlacementTriedOut>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct PlacementTriedOut {
    pub seed: u64,
    pub routed: usize,
    pub connections: usize,
    pub violations: usize,
    pub vias: usize,
    pub length_mm: f64,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct FramesOut {
    pub folder: String,
    pub pictures: usize,
    pub animation: Option<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct AlternativeOut {
    pub path: String,
    pub routed: usize,
    pub connections: usize,
    pub vias: usize,
    pub length_mm: f64,
    /// Spots with a clearance violation, as the open end counts them.
    pub clearance_spots: usize,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct MovedOut {
    pub reference: String,
    pub dx_mm: f64,
    pub dy_mm: f64,
}

#[derive(Debug, Clone, Serialize, JsonSchema)]
pub struct AnglesOut {
    /// Track length before and after the step.
    pub length_before_mm: f64,
    pub length_after_mm: f64,
    /// Pieces that kept an angle off the 45 degree grid because nothing else fit, and their length.
    pub off_grid_pieces: usize,
    pub off_grid_mm: f64,
    /// Corners of 90 or 135 degrees that could not be chamfered.
    pub sharp_corners: usize,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct RenderOut {
    pub png: String,
    pub width_px: u32,
    pub height_px: u32,
    pub layers: Vec<String>,
    /// The plotted region in mm: [x0, y0, x1, y1].
    pub region: [f64; 4],
    /// Pixels per mm.
    pub scale: f64,
}

// ------------------------------------------------------------------------------ helpers

/// Short text summary, the JSON (for clients that only show `content`) and the structured
/// result.
fn ok<T: Serialize>(summary: String, out: &T) -> Result<CallToolResult, ErrorData> {
    let value = serde_json::to_value(out).map_err(|e| ErrorData::internal_error(e.to_string(), None))?;
    let mut r = CallToolResult::success(vec![
        ContentBlock::text(summary),
        ContentBlock::text(serde_json::to_string(&value).unwrap_or_default()),
    ]);
    r.structured_content = Some(value);
    Ok(r)
}

fn fail(e: impl std::fmt::Display) -> Result<CallToolResult, ErrorData> {
    Ok(CallToolResult::error(vec![ContentBlock::text(format!("error: {e}"))]))
}

fn abs_path(p: &str) -> PathBuf {
    let p = PathBuf::from(p);
    let p = if p.is_absolute() {
        p
    } else {
        std::env::current_dir().map(|d| d.join(&p)).unwrap_or(p)
    };
    p.canonicalize().unwrap_or(p)
}

fn r3(x: f64) -> f64 {
    (x * 1000.0).round() / 1000.0
}

async fn blocking<T: Send + 'static>(f: impl FnOnce() -> Result<T> + Send + 'static) -> Result<T> {
    tokio::task::spawn_blocking(f).await.context("worker thread panicked")?
}

impl Default for Gummiband {
    fn default() -> Self {
        Self::new()
    }
}

impl Gummiband {
    fn with_entry<T>(&self, id: &str, f: impl FnOnce(&mut Entry) -> Result<T>) -> Result<T> {
        let mut boards = self.boards.lock().unwrap();
        let known: Vec<String> = boards.keys().cloned().collect();
        match boards.get_mut(id) {
            Some(e) => f(e),
            None => bail!(
                "unknown board id {id:?} (loaded: {}); call load_board first",
                if known.is_empty() {
                    "none".into()
                } else {
                    known.join(", ")
                }
            ),
        }
    }

    fn create(&self, p: CreateBoardIn) -> Result<(String, CreateBoardOut)> {
        use gummiband_kicad::create::{ClassSpec, CreateOptions, HoleSpec, create_board_files};
        let output = {
            let o = PathBuf::from(&p.output);
            if o.is_absolute() {
                o
            } else {
                std::env::current_dir().map(|d| d.join(&o)).unwrap_or(o)
            }
        };
        let d = CreateOptions::default();
        let opts = CreateOptions {
            width: p.width,
            height: p.height,
            x0: p.x0.unwrap_or(d.x0),
            y0: p.y0.unwrap_or(d.y0),
            corner_radius: p.corner_radius.unwrap_or(0.0),
            layers: p.layers.unwrap_or(2),
            plane_layers: p.plane_layers,
            thickness: p.thickness.unwrap_or(d.thickness),
            copper_oz: if p.copper_oz.is_empty() {
                d.copper_oz
            } else {
                p.copper_oz
            },
            prepreg: p.prepreg.unwrap_or(d.prepreg),
            mounting_holes: p.mounting_holes.map(|h| HoleSpec {
                diameter: h.diameter.unwrap_or(3.2),
                inset: h.inset.unwrap_or(4.0),
                footprint: h.footprint,
                net: h.net,
            }),
            net_classes: p
                .net_classes
                .into_iter()
                .map(|c| ClassSpec {
                    name: c.name,
                    clearance: c.clearance,
                    track_width: c.track_width,
                    via_diameter: c.via_diameter,
                    via_drill: c.via_drill,
                    diff_pair_width: c.diff_pair_width,
                    diff_pair_gap: c.diff_pair_gap,
                    priority: c.priority,
                    patterns: c.patterns,
                })
                .collect(),
            rules: p.rules,
            footprint_dirs: p.footprint_dirs.iter().map(|d| abs_path(d)).collect(),
            pin_aliases: p.pin_aliases,
            system_libraries: true,
            parts_outside: p.parts_outside.unwrap_or(false),
        };
        let rep = create_board_files(&abs_path(&p.netlist), &output, &opts, p.overwrite.unwrap_or(false))?;
        let (_, loaded) = self.load(LoadBoardIn {
            path: rep.pcb_path.display().to_string(),
            keep_locked: None,
        })?;
        let out = CreateBoardOut {
            board: loaded.board.clone(),
            path: loaded.path.clone(),
            project: rep.pro_path.display().to_string(),
            project_merged: rep.pro_merged,
            version: rep.version,
            footprints: rep.footprints + rep.holes,
            mounting_holes: rep.holes,
            nets: rep.nets,
            pads: rep.pads,
            pads_with_net: rep.pads_with_net,
            missing_footprints: rep
                .missing_footprints
                .iter()
                .map(|m| MissingFootprintOut {
                    reference: m.reference.clone(),
                    footprint: m.footprint.clone(),
                    reason: m.reason.clone(),
                })
                .collect(),
            unmatched: rep
                .unmatched
                .iter()
                .map(|u| UnmatchedOut {
                    reference: u.reference.clone(),
                    footprint: u.footprint.clone(),
                    pads: u.pads.clone(),
                    pins: u.pins.iter().map(|(p, n)| format!("{p} ({n})")).collect(),
                    suggested_alias: u.suggested_alias.as_ref().map(|(pin, pad)| {
                        let mut m = serde_json::Map::new();
                        m.insert(pin.clone(), serde_json::Value::from(pad.clone()));
                        let mut r = serde_json::Map::new();
                        r.insert(u.reference.clone(), serde_json::Value::Object(m));
                        serde_json::Value::Object(r)
                    }),
                })
                .collect(),
            libraries: rep.libraries.clone(),
            excluded: rep.excluded.clone(),
            outside: if rep.parked.is_empty() {
                rep.overflow.clone()
            } else {
                rep.parked.clone()
            },
            removed: rep.removed.iter().map(|f| f.display().to_string()).collect(),
            grid_gap: rep.grid_gap,
            warnings: rep.warnings.clone(),
        };
        let summary = format!(
            "Created {} as board {}: {} x {} mm, {} copper layers, {} footprints ({} mounting holes), {} nets, {} of {} pads with a net{}{}. {}",
            rep.pcb_path.file_name().and_then(|s| s.to_str()).unwrap_or(""),
            out.board,
            r3(p.width),
            r3(p.height),
            loaded.layers.len(),
            out.footprints,
            out.mounting_holes,
            out.nets,
            out.pads_with_net,
            out.pads,
            if out.missing_footprints.is_empty() {
                String::new()
            } else {
                format!(", {} footprints missing", out.missing_footprints.len())
            },
            if out.unmatched.is_empty() {
                String::new()
            } else {
                format!(
                    ", {} parts with unmatched pads or pins (see warnings)",
                    out.unmatched.len()
                )
            },
            if !rep.parked.is_empty() {
                format!(
                    "All {} parts sit in rows below the board (parts_outside), unlocked: place them with move_part / place_parts scope \"outside\".",
                    rep.parked.len()
                )
            } else if out.outside.is_empty() {
                "Parts sit in rows inside the outline, unlocked: place them with move_part / place_parts.".to_string()
            } else {
                format!(
                    "{} parts sit in rows inside the outline, {} did not fit and sit below the board; all unlocked: place them with move_part / place_parts.",
                    rep.footprints - out.outside.len(),
                    out.outside.len()
                )
            }
        );
        Ok((summary, out))
    }

    fn load(&self, p: LoadBoardIn) -> Result<(String, LoadBoardOut)> {
        let path = abs_path(&p.path);
        let keep_locked = p.keep_locked.unwrap_or(false);
        let kb = read(&path, keep_locked)?;
        let dru = std::fs::read_to_string(path.with_extension("kicad_dru")).unwrap_or_default();
        let custom_rules = dru
            .lines()
            .map(|l| l.split('#').next().unwrap_or(""))
            .map(|l| l.matches("(rule ").count())
            .sum();
        let mut boards = self.boards.lock().unwrap();
        // the same file keeps its id and net options
        let existing = boards.iter().find(|(_, e)| e.path == path).map(|(k, _)| k.clone());
        let id = existing.unwrap_or_else(|| {
            let mut n = self.next_id.lock().unwrap();
            *n += 1;
            format!("b{n}")
        });
        let options = boards.remove(&id).map(|e| e.options).unwrap_or_default();
        let b = &kb.board;
        let bb = b.bbox();
        let out = LoadBoardOut {
            board: id.clone(),
            path: path.display().to_string(),
            keep_locked,
            layers: b
                .layers
                .iter()
                .enumerate()
                .map(|(i, l)| LayerOut {
                    index: i,
                    name: l.name.clone(),
                    plane: l.is_plane,
                })
                .collect(),
            outline: OutlineOut {
                x0: r3(bb.min.x),
                y0: r3(bb.min.y),
                x1: r3(bb.max.x),
                y1: r3(bb.max.y),
                width_mm: r3(bb.width()),
                height_mm: r3(bb.height()),
            },
            pins: b.pins.len(),
            nets: b.nets.len(),
            routable_nets: b.routable_nets().count(),
            connections: connections(b),
            planes: b
                .planes
                .iter()
                .map(|p| PlaneOut {
                    net: b.nets[p.net as usize].name.clone(),
                    layer: b.layers[p.layer as usize].name.clone(),
                })
                .collect(),
            net_classes: b
                .classes
                .iter()
                .enumerate()
                .map(|(i, c)| ClassOut {
                    name: c.name.clone(),
                    priority: c.priority,
                    track_width: c.track_width,
                    clearance: c.clearance,
                    via_diameter: c.via_diameter,
                    via_drill: c.via_drill,
                    nets: b
                        .nets
                        .iter()
                        .filter(|n| n.class as usize == i && n.pins.len() > 1)
                        .count(),
                })
                .collect(),
            custom_rules,
            custom_clearance_pairs: b.pair_clearance.len(),
            net_options: options.iter().map(|(n, o)| (n.clone(), o.into())).collect(),
            diff_pairs: b
                .diff_pairs()
                .into_iter()
                .map(|(p, n)| DiffPairOut {
                    p: b.nets[p as usize].name.clone(),
                    n: b.nets[n as usize].name.clone(),
                    class: b.class_of(p).name.clone(),
                })
                .collect(),
            ground_nets: b
                .ground_nets()
                .iter()
                .map(|&n| b.nets[n as usize].name.clone())
                .collect(),
            warnings: b.ground_warning().into_iter().collect(),
        };
        let summary = format!(
            "Loaded {} as board {id}: {:.1} x {:.1} mm, {} copper layers ({}), {} pins, {} nets ({} to route, {} connections), {} net classes, {} custom rules.",
            path.file_name().and_then(|s| s.to_str()).unwrap_or(""),
            bb.width(),
            bb.height(),
            b.layers.len(),
            b.layers
                .iter()
                .map(|l| if l.is_plane {
                    format!("{} plane", l.name)
                } else {
                    l.name.clone()
                })
                .collect::<Vec<_>>()
                .join(", "),
            b.pins.len(),
            b.nets.len(),
            out.routable_nets,
            out.connections,
            b.classes.len(),
            custom_rules
        ) + &pairs_note(&out.diff_pairs)
            + &out
                .warnings
                .iter()
                .map(|w| format!(" Warning: {w}."))
                .collect::<String>();
        boards.insert(
            id,
            Entry {
                path,
                keep_locked,
                kb: Arc::new(kb),
                options,
            },
        );
        Ok((summary, out))
    }

    fn nets(&self, p: ListNetsIn) -> Result<(String, ListNetsOut)> {
        self.with_entry(&p.board, |e| {
            let b = &e.kb.board;
            let filter = p.filter.as_deref().unwrap_or("").to_lowercase();
            let single = p.include_single.unwrap_or(false);
            let nets: Vec<NetOut> = b
                .nets
                .iter()
                .enumerate()
                .filter(|(_, n)| single || n.pins.len() >= 2)
                .filter(|(_, n)| {
                    filter.is_empty()
                        || n.name.to_lowercase().contains(&filter)
                        || b.classes[n.class as usize].name.to_lowercase().contains(&filter)
                })
                .map(|(i, n)| {
                    let o = e.options.get(&n.name).cloned().unwrap_or_default();
                    NetOut {
                        name: n.name.clone(),
                        class: b.classes[n.class as usize].name.clone(),
                        pins: n.pins.len(),
                        plane: is_plane_net(b, i),
                        priority: o.priority.unwrap_or(n.priority),
                        skip: o.skip,
                    }
                })
                .collect();
            let summary = format!(
                "{} nets{} on board {}{}.",
                nets.len(),
                if filter.is_empty() {
                    String::new()
                } else {
                    format!(" matching {:?}", filter)
                },
                p.board,
                {
                    let skipped = nets.iter().filter(|n| n.skip).count();
                    if skipped > 0 {
                        format!(", {skipped} skipped")
                    } else {
                        String::new()
                    }
                }
            );
            Ok((
                summary,
                ListNetsOut {
                    board: p.board.clone(),
                    count: nets.len(),
                    nets,
                },
            ))
        })
    }

    fn patterns_of(&self, p: BoardPatternsIn) -> Result<(String, BoardPatternsOut)> {
        self.with_entry(&p.board, |e| {
            let b = &e.kb.board;
            let found = gummiband_core::patterns::find(b);
            let summary = found.summary(b);
            let out = BoardPatternsOut {
                patterns: serde_json::to_value(found.named(b))?,
                summary: summary.clone(),
            };
            Ok((summary.join("\n"), out))
        })
    }

    fn part(&self, p: PartInfoIn) -> Result<(String, PartInfoOut)> {
        self.with_entry(&p.board, |e| {
            let b = &e.kb.board;
            let pads: Vec<PadOut> = b
                .pins
                .iter()
                .filter(|q| q.footprint == p.reference)
                .map(|q| PadOut {
                    name: q.name.clone(),
                    net: b.nets[q.net as usize].name.clone(),
                    x: r3(q.pos.x),
                    y: r3(q.pos.y),
                    width_mm: r3(q.size.0),
                    height_mm: r3(q.size.1),
                    layers: q.layers.iter().map(|l| b.layers[l as usize].name.clone()).collect(),
                })
                .collect();
            let court = b.courtyards.get(&p.reference);
            if pads.is_empty() && court.is_none() {
                bail!("no part {:?} with pads or courtyard", p.reference);
            }
            let mut neighbours: Vec<String> = match court {
                Some(c) => {
                    let g = c.grow(3.0);
                    b.courtyards
                        .iter()
                        .filter(|(r, o)| **r != p.reference && g.overlaps(o))
                        .map(|(r, _)| r.clone())
                        .collect()
                }
                None => Vec::new(),
            };
            neighbours.sort();
            let d = gummiband_place::model::Design::from_root(&e.kb.root);
            let part = d.by_ref.get(&p.reference).map(|&i| &d.parts[i]);
            let out = PartInfoOut {
                reference: p.reference.clone(),
                value: part.map(|q| q.value.clone()).unwrap_or_default(),
                x: part.map_or(0.0, |q| r3(q.pos.x)),
                y: part.map_or(0.0, |q| r3(q.pos.y)),
                rotation: part.map_or(0.0, |q| q.rot),
                locked: part.is_some_and(|q| q.locked),
                sheet: part.map(|q| q.sheet_name.clone()).unwrap_or_default(),
                courtyard: court.map(|c| [r3(c.min.x), r3(c.min.y), r3(c.max.x), r3(c.max.y)]),
                pads,
                neighbours,
            };
            Ok((format!("{}: {} pads", out.reference, out.pads.len()), out))
        })
    }

    /// Write the locked copper into a working copy and load that one, keeping locked copper.
    fn add_locked(&self, p: AddLockedIn) -> Result<(String, AddLockedOut)> {
        let (path, kb, options) =
            self.with_entry(&p.board, |e| Ok((e.path.clone(), e.kb.clone(), e.options.clone())))?;
        // a working copy is the current state, whatever id the call names
        let stem = path.file_stem().and_then(|s| s.to_str()).unwrap_or("board");
        let work = if stem.ends_with(".prerouted") {
            path.clone()
        } else {
            path.with_file_name(format!("{stem}.prerouted.kicad_pcb"))
        };
        let kb = if work.is_file() {
            Arc::new(read(&work, true)?)
        } else {
            kb
        };
        let b = &kb.board;
        let mut warnings = Vec::new();
        // all copper of the file, the routed tracks included: a new via on a track of another net
        // is a short, whether that track is locked or not
        let from = if work.is_file() { work.clone() } else { path.clone() };
        let all = read_board(
            &from,
            &ReadOptions {
                keep: KeepCopper::All,
                ..Default::default()
            },
        )?;
        let ab = &all.board;
        let mut shorts = Vec::new();
        // the nearest copper of another net to the piece a..b of radius r: (net name, gap, needed gap)
        let foreign = |a: gummiband_core::geom::Point,
                       bb: gummiband_core::geom::Point,
                       r: f64,
                       layer: Option<u8>,
                       net: u32|
         -> Option<(String, f64, f64)> {
            let own = b.class_of(net).clearance.max(b.rules.min_clearance);
            ab.obstacles
                .iter()
                .filter(|o| {
                    o.net
                        .is_some_and(|n| ab.nets[n as usize].name != b.nets[net as usize].name)
                })
                .filter(|o| layer.is_none_or(|l| o.layers.has(l)))
                // zones are filled around the new copper; pads, tracks and vias stay where they are
                .filter(|o| {
                    matches!(
                        o.kind,
                        gummiband_core::board::ObstacleKind::Pad
                            | gummiband_core::board::ObstacleKind::Track
                            | gummiband_core::board::ObstacleKind::Via
                    )
                })
                .filter_map(|o| {
                    let gap = o.shape.dist_to_seg(a, bb) - r;
                    let need = own.max(o.clearance);
                    let n = o.net?;
                    (gap < need).then(|| (ab.nets[n as usize].name.clone(), gap, need))
                })
                .min_by(|x, y| x.1.total_cmp(&y.1))
        };
        let judge =
            |warnings: &mut Vec<String>, shorts: &mut Vec<String>, what: String, hit: Option<(String, f64, f64)>| {
                if let Some((other, gap, need)) = hit {
                    if gap <= 0.0 {
                        shorts.push(format!("{what} touches {other}"));
                    } else {
                        warnings.push(format!(
                            "{what} is {gap:.3} mm from {other}, the clearance asks for {need:.3} mm"
                        ));
                    }
                }
            };
        let net_of = |name: &str| b.net_id(name).with_context(|| format!("no net named {name:?}"));
        let mut items = Vec::new();
        for t in &p.tracks {
            let net = net_of(&t.net)?;
            let layer = b
                .layer_id(&t.layer)
                .with_context(|| format!("no layer {:?}", t.layer))?;
            if t.points.len() < 2 {
                bail!("a track needs at least two points");
            }
            let width = t.width.unwrap_or(b.class_of(net).track_width);
            if width <= 0.0 {
                bail!("track of {} has width {width}", t.net);
            }
            if b.layers[layer as usize].is_plane {
                warnings.push(format!("track of {} on plane layer {}", t.net, t.layer));
            }
            for q in &t.points {
                let at = gummiband_core::geom::pt(q[0], q[1]);
                if !b.inside(at) {
                    warnings.push(format!("track of {} leaves the board at {}, {}", t.net, q[0], q[1]));
                }
            }
            for w in t.points.windows(2) {
                let (pa, pb) = (
                    gummiband_core::geom::pt(w[0][0], w[0][1]),
                    gummiband_core::geom::pt(w[1][0], w[1][1]),
                );
                judge(
                    &mut warnings,
                    &mut shorts,
                    format!(
                        "track of {} from {}, {} to {}, {}",
                        t.net, w[0][0], w[0][1], w[1][0], w[1][1]
                    ),
                    foreign(pa, pb, width / 2.0, Some(layer), net),
                );
            }
            for w in t.points.windows(2) {
                items.push(write::Item::Track {
                    a: gummiband_core::geom::pt(w[0][0], w[0][1]),
                    b: gummiband_core::geom::pt(w[1][0], w[1][1]),
                    width,
                    layer,
                    net,
                });
            }
        }
        for v in &p.vias {
            let net = net_of(&v.net)?;
            let c = b.class_of(net);
            let at = gummiband_core::geom::pt(v.x, v.y);
            if !b.inside(at) {
                warnings.push(format!("via of {} off the board at {}, {}", v.net, v.x, v.y));
            }
            judge(
                &mut warnings,
                &mut shorts,
                format!("via of {} at {}, {}", v.net, v.x, v.y),
                foreign(at, at, v.diameter.unwrap_or(c.via_diameter) / 2.0, None, net),
            );
            items.push(write::Item::Via {
                at: gummiband_core::geom::pt(v.x, v.y),
                diameter: v.diameter.unwrap_or(c.via_diameter),
                drill: v.drill.unwrap_or(c.via_drill),
                net,
            });
        }
        if items.is_empty() {
            bail!("nothing to add");
        }
        if !shorts.is_empty() {
            bail!(
                "nothing added: the new copper would short other nets (KiCad would join them): {}",
                shorts.join("; ")
            );
        }
        std::fs::write(&work, write::add_locked(&kb, &items)).with_context(|| format!("writing {}", work.display()))?;
        if work != path {
            external::copy_project_files(&path, &work);
        }
        let (_, loaded) = self.load(LoadBoardIn {
            path: work.display().to_string(),
            keep_locked: Some(true),
        })?;
        let id = loaded.board;
        self.with_entry(&id, |e| {
            e.options = options;
            Ok(())
        })?;
        let out = AddLockedOut {
            board: id,
            path: work.display().to_string(),
            items_added: items.len(),
            warnings,
        };
        Ok((format!("{} locked items in {}", out.items_added, out.path), out))
    }

    /// The working copy of a board: the file itself if it is one already.
    fn working_copy(path: &Path) -> PathBuf {
        let stem = path.file_stem().and_then(|s| s.to_str()).unwrap_or("board");
        if stem.ends_with(".prerouted") {
            path.to_path_buf()
        } else {
            path.with_file_name(format!("{stem}.prerouted.kicad_pcb"))
        }
    }

    /// Change the text of the working copy and load it; returns the board id.
    fn edit_copy(
        &self,
        board: &str,
        edit: impl FnOnce(&KicadBoard, &str) -> Result<String>,
    ) -> Result<(String, String)> {
        let (path, keep, options) =
            self.with_entry(board, |e| Ok((e.path.clone(), e.keep_locked, e.options.clone())))?;
        let work = Self::working_copy(&path);
        let from = if work.is_file() { work.clone() } else { path.clone() };
        let kb = read(&from, true)?;
        let text = edit(&kb, &std::fs::read_to_string(&from)?)?;
        std::fs::write(&work, text).with_context(|| format!("writing {}", work.display()))?;
        if work != path {
            external::copy_project_files(&path, &work);
        }
        let (_, loaded) = self.load(LoadBoardIn {
            path: work.display().to_string(),
            keep_locked: Some(keep || work.is_file()),
        })?;
        let id = loaded.board;
        self.with_entry(&id, |e| {
            e.options = options;
            Ok(())
        })?;
        Ok((id, work.display().to_string()))
    }

    fn do_lock(&self, p: LockIn) -> Result<(String, LockOut)> {
        let all = p.all_copper.unwrap_or(false);
        if p.parts.is_empty() && p.nets.is_empty() && !all {
            bail!("nothing to lock: name parts, nets or set all_copper");
        }
        let lock = !p.unlock.unwrap_or(false);
        let mut counts = None;
        let (id, path) = self.edit_copy(&p.board, |kb, _| {
            let (text, c) = write::set_locked(kb, &p.parts, &p.nets, all, lock);
            counts = Some(c);
            Ok(text)
        })?;
        let c = counts.unwrap_or_default();
        let mut warnings: Vec<String> = c
            .unknown_parts
            .iter()
            .map(|r| format!("no footprint {r} on the board"))
            .collect();
        warnings.extend(
            c.nets_without_copper
                .iter()
                .map(|n| format!("net {n} has no tracks or vias on the board (unknown, or not routed yet)")),
        );
        let verb = if lock { "Locked" } else { "Unlocked" };
        let s = format!(
            "{verb} {} parts, {} tracks and {} vias in {path} (board {id}).{}",
            c.parts.len(),
            c.tracks,
            c.vias,
            if warnings.is_empty() {
                String::new()
            } else {
                format!(" Warnings: {}", warnings.join("; "))
            }
        );
        Ok((
            s,
            LockOut {
                board: id,
                path,
                locked: lock,
                parts: c.parts,
                tracks: c.tracks,
                vias: c.vias,
                warnings,
            },
        ))
    }

    fn do_remove(&self, p: RemoveIn) -> Result<(String, RemoveOut)> {
        use gummiband_kicad::edit::{RemoveSpec, ZoneFilter, remove_items};
        let locked = match p.only.as_deref() {
            None | Some("all") => None,
            Some("locked") => Some(true),
            Some("unlocked") => Some(false),
            Some(o) => bail!("only must be \"locked\" or \"unlocked\", not {o:?}"),
        };
        if let Some(c) = p.copper.as_deref()
            && !matches!(c, "tracks" | "vias" | "all")
        {
            bail!("copper must be \"tracks\", \"vias\" or \"all\", not {c:?}");
        }
        let zones: Vec<ZoneFilter> = p
            .zones
            .iter()
            .map(|z| ZoneFilter {
                name: z.name.clone(),
                net: z.net.clone(),
                layer: z.layer.clone(),
            })
            .collect();
        if zones
            .iter()
            .any(|z| z.name.is_none() && z.net.is_none() && z.layer.is_none())
        {
            bail!("a zone filter needs name, net or layer");
        }
        if zones.is_empty() && p.nets.is_empty() && p.area.is_none() && p.parts.is_empty() {
            bail!("nothing to remove: give zones, nets, area or parts");
        }
        let spec = RemoveSpec {
            zones,
            nets: p.nets.clone(),
            area: p.area.map(|a| gummiband_core::geom::Rect {
                min: gummiband_core::geom::pt(a[0].min(a[2]), a[1].min(a[3])),
                max: gummiband_core::geom::pt(a[0].max(a[2]), a[1].max(a[3])),
            }),
            layers: p.layers.clone(),
            copper: p.copper.clone(),
            locked,
            parts: p.parts.clone(),
        };
        let mut counts = None;
        let (id, path) = self.edit_copy(&p.board, |kb, _| {
            let (text, c) = remove_items(kb, &spec);
            counts = Some(c);
            Ok(text)
        })?;
        let c = counts.unwrap_or_default();
        let mut warnings: Vec<String> = c
            .unknown_parts
            .iter()
            .map(|r| format!("no footprint {r} on the board"))
            .collect();
        if !c.parts_with_nets.is_empty() {
            warnings.push(format!(
                "{} had pads on nets: the netlist still has them, KiCad's DRC reports them as missing",
                c.parts_with_nets.join(", ")
            ));
        }
        if c.zones.is_empty() && c.tracks == 0 && c.vias == 0 && c.parts.is_empty() {
            warnings.push("nothing matched".into());
        }
        let s = format!(
            "Removed {} zones, {} tracks, {} vias and {} footprints in {path} (board {id}).{}",
            c.zones.len(),
            c.tracks,
            c.vias,
            c.parts.len(),
            if warnings.is_empty() {
                String::new()
            } else {
                format!(" Warnings: {}", warnings.join("; "))
            }
        );
        Ok((
            s,
            RemoveOut {
                board: id,
                path,
                zones: c.zones,
                tracks: c.tracks,
                vias: c.vias,
                parts: c.parts,
                warnings,
            },
        ))
    }

    fn do_mounting_hole(&self, p: MountingHoleIn) -> Result<(String, AddLockedOut)> {
        let d = p.diameter.unwrap_or(3.2);
        let mut name = String::new();
        let (id, path) = self.edit_copy(&p.board, |kb, _| {
            let (text, r) = gummiband_kicad::edit::add_mounting_hole(kb, p.x, p.y, d, p.reference.as_deref())?;
            name = r;
            Ok(text)
        })?;
        let mut warnings = Vec::new();
        self.with_entry(&id, |e| {
            let b = &e.kb.board;
            let me = b.courtyards.get(&name).copied();
            if let Some(c) = me {
                for (r, o) in &b.courtyards {
                    if *r != name && o.overlaps(&c) {
                        warnings.push(format!("the hole's courtyard overlaps {r}"));
                    }
                }
            }
            Ok(())
        })?;
        warnings.sort();
        Ok((
            format!(
                "Added mounting hole {name} ({d} mm) at {}, {} in {path} (board {id}), locked.",
                p.x, p.y
            ),
            AddLockedOut {
                board: id,
                path,
                items_added: 1,
                warnings,
            },
        ))
    }

    fn do_place_silk(&self, p: PlaceSilkIn) -> Result<(String, PlaceSilkOut)> {
        use gummiband_kicad::edit::{SilkOptions, place_references, trim_edge_silk};
        let d = SilkOptions::default();
        let o = SilkOptions {
            parts: p.parts.clone(),
            min_size: p.min_size.unwrap_or(d.min_size),
            margin: p.margin.unwrap_or(d.margin),
            edge_margin: p.edge_margin.unwrap_or(d.edge_margin),
            hide_unplaced: p.hide_unplaced.unwrap_or(true),
            reach: p.reach.unwrap_or(d.reach),
        };
        let mut trimmed = Vec::new();
        if p.trim_edge == Some(true) {
            let margin = o.edge_margin;
            self.edit_copy(&p.board, |kb, _| {
                let (text, refs) = trim_edge_silk(kb, margin);
                trimmed = refs;
                Ok(text)
            })?;
        }
        let mut rep = None;
        let (id, path) = self.edit_copy(&p.board, |kb, _| {
            let (text, r) = place_references(kb, &o);
            rep = Some(r);
            Ok(text)
        })?;
        let r = rep.unwrap_or_default();
        let s = format!(
            "References: {} clear where they were, {} moved, {} moved smaller, {} hidden{}{} in {path} (board {id}).",
            r.kept.len(),
            r.moved.len(),
            r.shrunk.len(),
            r.hidden.len(),
            if r.left.is_empty() {
                String::new()
            } else {
                format!(", {} without a clear spot left as they were", r.left.len())
            },
            if trimmed.is_empty() {
                String::new()
            } else {
                format!("; edge silk removed from {}", trimmed.join(", "))
            }
        );
        Ok((
            s,
            PlaceSilkOut {
                board: id,
                path,
                kept: r.kept.len(),
                moved: r.moved,
                shrunk: r.shrunk,
                hidden: r.hidden,
                left: r.left,
                trimmed,
            },
        ))
    }

    fn do_set_rules(&self, p: SetRulesIn) -> Result<(String, SetRulesOut)> {
        let new = p.rules.trim();
        let parsed = gummiband_kicad::sexpr::parse(&format!("(rules {new})"))
            .map_err(|e| anyhow::anyhow!("rules do not parse: {e}"))?;
        let n_new = parsed.children("rule").count();
        if n_new == 0 {
            bail!("no (rule ...) in rules");
        }
        if let Some(o) = parsed
            .args()
            .iter()
            .find(|x| !matches!(x.head(), Some("rule") | Some("version")))
        {
            bail!(
                "rules may only hold (rule ...) items, found {:?}",
                o.head().unwrap_or("an atom")
            );
        }
        // the working copy carries the rules from now on (with the project files of the input)
        let (_, work) = self.edit_copy(&p.board, |_, text| Ok(text.to_string()))?;
        let work = PathBuf::from(work);
        let dru = work.with_extension("kicad_dru");
        let old = if p.replace == Some(true) {
            String::new()
        } else {
            std::fs::read_to_string(&dru).unwrap_or_default()
        };
        let mut text = if old.trim().is_empty() {
            "(version 1)\n".to_string()
        } else {
            old.trim_end().to_string() + "\n"
        };
        let body: String = new
            .lines()
            .filter(|l| !l.trim_start().starts_with("(version"))
            .collect::<Vec<_>>()
            .join("\n");
        text.push_str(body.trim());
        text.push('\n');
        std::fs::write(&dru, &text).with_context(|| format!("writing {}", dru.display()))?;
        let keep = self.with_entry(&p.board, |e| Ok(e.options.clone()))?;
        let (_, loaded) = self.load(LoadBoardIn {
            path: work.display().to_string(),
            keep_locked: Some(true),
        })?;
        let id = loaded.board;
        self.with_entry(&id, |e| {
            e.options = keep;
            Ok(())
        })?;
        let (understood, total) = self.with_entry(&id, |e| {
            let b = &e.kb.board;
            let mut u = Vec::new();
            for (r, m) in &b.courtyard_clearance {
                u.push(format!("clearance {m} mm in the courtyard of {r} (own check)"));
            }
            if !b.pair_clearance.is_empty() || !b.item_clearance.is_empty() {
                u.push(format!(
                    "clearance between {} net pairs",
                    b.pair_clearance.len() + b.item_clearance.len()
                ));
            }
            for c in &b.classes {
                if let Some(w) = c.min_width {
                    u.push(format!("minimum width {w} mm for class {}", c.name));
                }
            }
            if !b.match_rules.is_empty() {
                u.push(format!("{} length or skew rules", b.match_rules.len()));
            }
            let total = gummiband_kicad::sexpr::parse(&format!("(rules {text})"))
                .map(|r| r.children("rule").count())
                .unwrap_or(0);
            Ok((u, total))
        })?;
        let mut warnings = Vec::new();
        if !b_has_kicad_pro(&work) {
            warnings.push(format!(
                "no {} next to the rules: KiCad needs the project for them",
                work.with_extension("kicad_pro").display()
            ));
        }
        Ok((
            format!(
                "{n_new} rules written to {} ({total} in the file), board {id}. Gummiband reads: {}.",
                dru.display(),
                if understood.is_empty() {
                    "none of them (KiCad's drc still applies them)".to_string()
                } else {
                    understood.join("; ")
                }
            ),
            SetRulesOut {
                board: id,
                path: dru.display().to_string(),
                rules: total,
                understood,
                warnings,
            },
        ))
    }

    /// Route a loaded board: the tool `route`, and every seed of `try_seeds`. The pipeline of
    /// gummiband-app reads the file and does all of it; the session adds the options per net
    /// and keeps the placement of `clear_corridors` in the working copy.
    fn route_entry(&self, p: RouteIn) -> Result<(String, RouteOut)> {
        let (options, loaded_keep) = self.with_entry(&p.board, |e| Ok((e.options.clone(), e.keep_locked)))?;
        let (mut opts, respelled) = route_options(&p, loaded_keep)?;
        opts.net_options = options;
        opts.validate(false).map_err(anyhow::Error::msg)?;
        let (mut id, mut cleared) = (p.board.clone(), Vec::new());
        if opts.clear_corridors {
            (id, cleared) = self.clear_corridors(&p.board, &opts.place_options())?;
            opts.clear_corridors = false;
        }
        let path = self.with_entry(&id, |e| Ok(e.path.clone()))?;
        let (mut summary, mut out) = route_blocking(id, path, opts, &respelled, p)?;
        if !cleared.is_empty() {
            summary = format!(
                "placed again out of the track corridors: {}\n{summary}",
                cleared.join(" ")
            );
        }
        if !cleared.is_empty() {
            out.cleared_corridors = cleared;
        }
        Ok((summary, out))
    }

    fn do_try_seeds(&self, p: TrySeedsIn) -> Result<(String, TrySeedsOut)> {
        let seeds: Vec<u64> = if p.seeds.is_empty() {
            vec![1, 2, 3, 4]
        } else {
            p.seeds.clone()
        };
        for k in ["board", "seed"] {
            if p.place.contains_key(k) {
                bail!("place: leave out {k}, try_seeds sets it");
            }
        }
        for k in ["board", "output", "clear_corridors"] {
            if p.route.contains_key(k) {
                bail!("route: leave out {k}");
            }
        }
        let (path, keep, options) =
            self.with_entry(&p.board, |e| Ok((e.path.clone(), e.keep_locked, e.options.clone())))?;
        let work = Self::working_copy(&path);
        let start: Option<String> = if work.is_file() {
            Some(std::fs::read_to_string(&work)?)
        } else {
            None
        };
        let stem = work
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("board")
            .trim_end_matches(".prerouted")
            .to_string();
        let restore = || -> Result<()> {
            match &start {
                Some(t) => std::fs::write(&work, t)?,
                None => {
                    if work.is_file() {
                        std::fs::remove_file(&work)?;
                    }
                }
            }
            Ok(())
        };
        let use_drc = p.drc.unwrap_or_else(|| external::kicad_cli().is_ok());
        let mut results: Vec<SeedOut> = Vec::new();
        for &seed in &seeds {
            restore()?;
            let placed = work.with_file_name(format!("{stem}.seed{seed}.placed.kicad_pcb"));
            let routed = work.with_file_name(format!("{stem}.seed{seed}.routed.kicad_pcb"));
            let mut out = SeedOut {
                seed,
                placed: placed.display().to_string(),
                routed: routed.display().to_string(),
                unrouted: 0,
                violations: 0,
                drc_errors: None,
                drc_unconnected: None,
                vias: 0,
                length_mm: 0.0,
                error: None,
                drc_failed: None,
            };
            let run = || -> Result<RouteOut> {
                let mut a = p.place.clone();
                a.insert("board".into(), p.board.clone().into());
                a.insert("seed".into(), seed.into());
                let pin: PlacePartsIn = serde_json::from_value(serde_json::Value::Object(a)).context("place")?;
                let (_, po) = self.do_place_parts(pin)?;
                let mut board = po.board;
                if let Some(pc) = &p.power_copper {
                    let mut a = pc.clone();
                    a.insert("board".into(), board.clone().into());
                    let pin: practice::PowerCopperIn =
                        serde_json::from_value(serde_json::Value::Object(a)).context("power_copper")?;
                    board = self.do_power_copper(&pin)?.board;
                }
                std::fs::copy(&work, &placed)?;
                external::copy_project_files(&work, &placed);
                let mut r = p.route.clone();
                r.insert("board".into(), board.clone().into());
                r.insert("output".into(), routed.display().to_string().into());
                let rin: RouteIn = serde_json::from_value(serde_json::Value::Object(r)).context("route")?;
                let (_, ro) = self.route_entry(rin)?;
                Ok(ro)
            };
            match run() {
                Ok(ro) => {
                    out.unrouted = ro.unrouted.len();
                    out.violations = ro.violations_total;
                    out.vias = ro.vias;
                    out.length_mm = r3(ro.length_mm);
                    if use_drc {
                        match external::drc(&routed, 0) {
                            Ok(d) => {
                                out.drc_errors = Some(d.violations);
                                out.drc_unconnected = Some(d.unconnected);
                            }
                            Err(e) => out.drc_failed = Some(format!("{e:#}")),
                        }
                    }
                }
                Err(e) => out.error = Some(format!("{e:#}")),
            }
            results.push(out);
        }
        let key = |r: &SeedOut| {
            (
                r.error.is_some(),
                r.unrouted + r.drc_unconnected.unwrap_or(0),
                r.drc_errors.unwrap_or(r.violations),
                r.vias,
                (r.length_mm * 1000.0) as i64,
            )
        };
        results.sort_by_key(key);
        let best = results.first().filter(|r| r.error.is_none());
        let (mut board, mut routed_board) = (None, None);
        restore()?;
        if let Some(b) = best {
            std::fs::copy(&b.placed, &work)?;
            external::copy_project_files(&path, &work);
            let (_, l) = self.load(LoadBoardIn {
                path: work.display().to_string(),
                keep_locked: Some(keep || work.is_file()),
            })?;
            self.with_entry(&l.board, |e| {
                e.options = options.clone();
                Ok(())
            })?;
            board = Some(l.board);
            let (_, r) = self.load(LoadBoardIn {
                path: b.routed.clone(),
                keep_locked: None,
            })?;
            routed_board = Some(r.board);
        }
        let mut s = format!("{} seeds, best first:", results.len());
        for r in &results {
            s += &format!(
                "\nseed {}: {} unrouted, {}{} Gummiband violations, {} vias, {:.0} mm{}",
                r.seed,
                r.unrouted,
                match (r.drc_errors, r.drc_unconnected) {
                    (Some(e), Some(u)) => format!("KiCad {e} errors and {u} unconnected, "),
                    _ => String::new(),
                },
                r.violations,
                r.vias,
                r.length_mm,
                r.error
                    .as_ref()
                    .or(r.drc_failed.as_ref())
                    .map(|e| format!(" ({e})"))
                    .unwrap_or_default()
            );
        }
        if let (Some(b), Some(id), Some(rid)) = (best, &board, &routed_board) {
            s += &format!(
                "\nThe working copy (board {id}) now holds the placement of seed {}; its routed file is board {rid}.",
                b.seed
            );
        }
        Ok((
            s,
            TrySeedsOut {
                best_seed: best.map(|b| b.seed),
                seeds: results,
                board,
                routed_board,
            },
        ))
    }

    fn do_power_copper(&self, p: &practice::PowerCopperIn) -> Result<practice::PowerCopperOut> {
        practice::Unlocked::parse(p.unlocked.as_deref())?;
        // the board with all its copper: unlocked tracks of a routed board are in the way too
        let all = {
            let path = self.with_entry(&p.board, |e| Ok(e.path.clone()))?;
            let work = Self::working_copy(&path);
            read_board(
                if work.is_file() { &work } else { &path },
                &ReadOptions {
                    keep: KeepCopper::All,
                    ..Default::default()
                },
            )?
        };
        let mut out = None;
        let (id, path) = self.edit_copy(&p.board, |kb, _| {
            let (text, o) = practice::power_copper(&all, kb, p)?;
            out = Some(o);
            Ok(text)
        })?;
        let mut out = out.context("power_copper made no report")?;
        out.board = id;
        out.path = path.clone();
        // the pours carry their current already: measure them before routing
        let pour_paths: Vec<practice::PathIn> = p
            .paths
            .iter()
            .filter(|q| out.pours.iter().any(|x| x.kind == "pour" && x.net == q.net))
            .cloned()
            .collect();
        if !pour_paths.is_empty() {
            let check = practice::current_check(&practice::CurrentCheckIn {
                path: path.clone(),
                paths: Some(pour_paths),
                temp_rise: p.temp_rise,
                refill: None,
                max_segments: Some(3),
            });
            match check {
                Ok(c) => out.check = Some(c),
                Err(e) => out.warnings.push(format!("current_check of the pours failed: {e:#}")),
            }
        }
        Ok(out)
    }

    fn do_move_part(&self, p: MovePartIn) -> Result<(String, AddLockedOut)> {
        let back = match p.side.as_deref() {
            None => None,
            Some("front") | Some("top") | Some("F.Cu") => Some(false),
            Some("back") | Some("bottom") | Some("B.Cu") => Some(true),
            Some(o) => bail!("side must be \"front\" or \"back\", not {o:?}"),
        };
        // a flip is a change of side, not a move: the tracks stay where they are
        let flips = match back {
            Some(b) => {
                let text = std::fs::read_to_string(self.current_file(&p.board)?)?;
                match write::footprint_on_back(&text, &p.reference) {
                    Some(on_back) => on_back != b,
                    None => bail!("no footprint {:?}", p.reference),
                }
            }
            None => false,
        };
        let follow = p.follow.unwrap_or(true) && p.rotation.is_none() && !flips;
        let mut no_rules = None;
        // all copper, not only the locked: the tracks are what follows
        let all = if follow {
            let path = self.with_entry(&p.board, |e| Ok(e.path.clone()))?;
            let work = Self::working_copy(&path);
            let from = if work.is_file() { work } else { path };
            if !from.with_extension("kicad_pro").is_file() {
                no_rules = Some(from.with_extension("kicad_pro").display().to_string());
            }
            Some(read_board(
                &from,
                &ReadOptions {
                    keep: KeepCopper::All,
                    ..Default::default()
                },
            )?)
        } else {
            None
        };
        let mut followed = None;
        let (id, path) = self.edit_copy(&p.board, |_, text| {
            let Some(all) = &all else {
                return write::place_footprint_on(text, &p.reference, p.x, p.y, p.rotation, back);
            };
            let (text, r) = gummiband_kicad::follow_write::move_part_following(all, &p.reference, p.x, p.y)?;
            followed = Some(r);
            Ok(text)
        })?;
        // courtyard boxes after the move: a quick look, drc has the exact answer
        let warnings = self.with_entry(&id, |e| {
            let b = &e.kb.board;
            let Some(c) = b.courtyards.get(&p.reference) else {
                return Ok(Vec::new());
            };
            let c = c.grow(-0.05);
            let mut hits: Vec<String> = b
                .courtyards
                .keys()
                .filter(|r| **r != p.reference && b.courtyards_overlap(&p.reference, r, 0.05))
                .cloned()
                .collect();
            hits.sort();
            let mut w: Vec<String> = hits
                .into_iter()
                .map(|r| format!("courtyard of {} overlaps {r}", p.reference))
                .collect();
            let bb = b.bbox();
            if c.min.x < bb.min.x || c.min.y < bb.min.y || c.max.x > bb.max.x || c.max.y > bb.max.y {
                w.push(format!("courtyard of {} leaves the board outline", p.reference));
            }
            Ok(w)
        })?;
        let mut warnings = warnings;
        if let Some(pro) = no_rules {
            warnings.push(format!(
                "no {pro} next to the board: the tracks gave way by default clearances and widths"
            ));
        }
        if flips {
            warnings.push(format!(
                "{} changed side: its tracks did not follow, route or redraw them",
                p.reference
            ));
        }
        let msg = match &followed {
            None if flips => format!(
                "flipped {} to the {} at {}, {}",
                p.reference,
                if back == Some(true) { "back" } else { "front" },
                p.x,
                p.y
            ),
            None => format!("moved {} to {}, {}", p.reference, p.x, p.y),
            Some(r) => {
                if let Some(why) = &r.stats.stopped {
                    warnings.push(format!(
                        "{} stopped at {:.3}, {:.3} instead of {}, {}: {why}",
                        p.reference, r.to.0, r.to.1, p.x, p.y
                    ));
                }
                format!(
                    "moved {} to {:.3}, {:.3}; {} tracks and {} vias of {} nets followed (at most {:.3} mm), clearance spots {} -> {}",
                    p.reference,
                    r.to.0,
                    r.to.1,
                    r.stats.tracks_moved,
                    r.stats.vias_moved,
                    r.stats.nets_moved,
                    r.stats.max_shift,
                    r.stats.spots_before,
                    r.stats.spots_after
                )
            }
        };
        let out = AddLockedOut {
            board: id,
            path,
            items_added: followed.as_ref().map_or(1, |r| r.items_written),
            warnings,
        };
        Ok((msg, out))
    }

    /// The file a placement starts from: the working copy if there is one.
    fn current_file(&self, board: &str) -> Result<PathBuf> {
        let path = self.with_entry(board, |e| Ok(e.path.clone()))?;
        let work = Self::working_copy(&path);
        Ok(if work.is_file() { work } else { path })
    }

    fn corridors_of(&self, p: TrackCorridorsIn) -> Result<(String, TrackCorridorsOut)> {
        use gummiband_place::auto::{PlaceOptions, corridor_groups};
        let from = self.current_file(&p.board)?;
        let opts = PlaceOptions {
            fixed: p.fixed.clone(),
            ..Default::default()
        };
        let groups = corridor_groups(&from, &opts)?;
        let mut all: Vec<String> = groups.iter().flat_map(|g| g.intruders.iter().cloned()).collect();
        all.sort();
        all.dedup();
        let mut text = format!("{} track corridors, {} parts in the way", groups.len(), all.len());
        for g in &groups {
            text += &format!(
                "\n{} - {}: {} nets, {:.1} x {:.1} mm, in the way: {}",
                g.a,
                g.b,
                g.nets.len(),
                g.length,
                g.width,
                if g.intruders.is_empty() {
                    "none".to_string()
                } else {
                    g.intruders.join(" ")
                }
            );
        }
        let out = TrackCorridorsOut {
            corridors: groups
                .into_iter()
                .map(|g| CorridorOut {
                    a: g.a,
                    b: g.b,
                    nets: g.nets,
                    width_mm: g.width,
                    length_mm: g.length,
                    in_the_way: g.intruders,
                })
                .collect(),
            in_the_way: all,
        };
        Ok((text, out))
    }

    /// `route --clear-corridors`: place the parts in the way of a track corridor again, into
    /// the working copy. Returns the board id to route and the parts that moved.
    fn clear_corridors(
        &self,
        board: &str,
        place: &gummiband_place::auto::PlaceOptions,
    ) -> Result<(String, Vec<String>)> {
        let from = self.current_file(board)?;
        let Some(cleared) = gummiband_app::clear_corridors(&from, place, &Default::default())? else {
            return Ok((board.to_string(), Vec::new()));
        };
        let (id, _) = self.edit_copy(board, |_, _| Ok(cleared.text))?;
        Ok((id, cleared.parts))
    }

    fn do_place_parts(&self, p: PlacePartsIn) -> Result<(String, PlacePartsOut)> {
        use gummiband_place::auto::{PlaceOptions, Scope, place_board};
        let scope = if !p.only.is_empty() {
            Scope::Only(p.only.clone())
        } else {
            match p.scope.as_deref() {
                None | Some("unlocked") => Scope::Unlocked,
                Some("all_unlocked") => Scope::AllUnlocked,
                Some("outside") => Scope::Outside,
                Some(o) => bail!("unknown scope {o:?}: use unlocked, all_unlocked or outside"),
            }
        };
        let defaults = PlaceOptions::default();
        let opts = PlaceOptions {
            scope,
            fixed: p.fixed.clone(),
            seed: p.seed.unwrap_or(defaults.seed),
            starts: p.starts.unwrap_or(defaults.starts),
            gap: p.gap.unwrap_or(defaults.gap),
            halo: p.halo.unwrap_or(defaults.halo),
            semantic: p.semantic.unwrap_or(false),
            corridors: p.corridors.unwrap_or(false),
            escape_halo: p.escape_halo.unwrap_or(false),
            leash: p.leash.unwrap_or(defaults.leash),
            two_sided: p.two_sided.unwrap_or(false),
            back_max_pads: p.back_max_pads.unwrap_or(defaults.back_max_pads),
            back_cost: p.back_cost.unwrap_or(defaults.back_cost),
            on_front: p.on_front.clone(),
            on_back: p.on_back.clone(),
            ..defaults
        };
        let two_sided = opts.two_sided;
        let path = self.with_entry(&p.board, |e| Ok(e.path.clone()))?;
        let work = Self::working_copy(&path);
        let from = if work.is_file() { work } else { path };
        let mut report = None;
        let (id, path) = self.edit_copy(&p.board, |_, _| {
            let placed = place_board(&from, &opts)?;
            report = Some(placed.report);
            Ok(placed.text)
        })?;
        let r = report.context("placement did not run")?;
        let mut text = format!("placed {} parts, {} crossings", r.moved.len(), r.crossings);
        if two_sided {
            text += &format!("; {} on the back, {} changed sides", r.back.len(), r.flipped.len());
        }
        if !r.warnings.is_empty() {
            text += &format!("; {} warnings: {}", r.warnings.len(), r.warnings.join("; "));
        }
        let out = PlacePartsOut {
            board: id,
            path,
            placed: r
                .poses
                .into_iter()
                .map(|(reference, x, y, rotation)| PartPose {
                    back: r.back.contains(&reference),
                    reference,
                    x,
                    y,
                    rotation,
                })
                .collect(),
            decoupling: r.decoupling,
            crossings: r.crossings,
            min_separation: r.min_separation,
            off_plane: r.off_plane,
            anchored: r.anchored,
            coils: r.coils,
            groups: r
                .groups
                .into_iter()
                .map(|(name, parts)| GroupOut { name, parts })
                .collect(),
            ties: r.ties,
            released: r.released,
            no_via_room: r.no_via_room,
            warnings: r.warnings,
            flipped: r.flipped,
            free_mm2: two_sided.then_some(SidesOut {
                front: r.free_area - r.back_free_area,
                back: r.back_free_area,
            }),
        };
        Ok((text, out))
    }

    fn do_add_zone(&self, p: AddZoneIn) -> Result<(String, AddLockedOut)> {
        if p.points.len() < 3 {
            bail!("a zone needs at least three points");
        }
        let (id, path) = self.edit_copy(&p.board, |kb, text| {
            let b = &kb.board;
            let net = b.net_id(&p.net).with_context(|| format!("no net named {:?}", p.net))?;
            let layer = b
                .layer_id(&p.layer)
                .with_context(|| format!("no layer {:?}", p.layer))?;
            let pts: Vec<_> = p.points.iter().map(|q| gummiband_core::geom::pt(q[0], q[1])).collect();
            let connect = match p.pad_connection.as_deref() {
                None => write::PadConnect::Thermal,
                Some(s) => write::PadConnect::parse(s)
                    .with_context(|| format!("pad_connection {s:?}: use thermal, solid or thermal_tht"))?,
            };
            let spoke = p.thermal_spoke_width.unwrap_or(write::DEFAULT_SPOKE);
            if spoke <= 0.0 {
                bail!("thermal_spoke_width must be positive");
            }
            let spec = write::ZoneSpec {
                priority: p.priority.unwrap_or(0),
                connect,
                spoke,
                ..write::ZoneSpec::plain(net, layer)
            };
            Ok(write::add_zone_named(kb, text, &spec, &pts, 1))
        })?;
        let out = AddLockedOut {
            board: id,
            path,
            items_added: 1,
            warnings: Vec::new(),
        };
        Ok((format!("zone {} on {}", p.net, p.layer), out))
    }

    fn do_set_layer_type(&self, p: SetLayerTypeIn) -> Result<(String, SetLayerTypeOut)> {
        if p.layers.is_empty() {
            bail!("give at least one layer");
        }
        let mut changed = Vec::new();
        let mut warnings = Vec::new();
        let (id, path) = self.edit_copy(&p.board, |kb, text| {
            let b = &kb.board;
            let zf = gummiband_kicad::power::zones(kb);
            let mut changes = Vec::new();
            for l in &p.layers {
                let lid = b
                    .layer_id(&l.layer)
                    .with_context(|| format!("no copper layer {:?}", l.layer))?;
                let outer = gummiband_kicad::power::is_outer(b, lid);
                if l.kind == "power" && outer {
                    bail!(
                        "{} is an outer layer: it carries the parts and stays a routing layer; pour it with add_zone or ground_pour instead",
                        l.layer
                    );
                }
                let mut zone_nets: Vec<String> = Vec::new();
                for z in zf.iter().filter(|z| z.layer == lid) {
                    if let Some(n) = z.net {
                        let name = b.nets[n as usize].name.clone();
                        if !zone_nets.contains(&name) {
                            zone_nets.push(name);
                        }
                    }
                }
                let was_plane = b.layers[lid as usize].is_plane;
                if l.kind == "power" && !was_plane {
                    if zone_nets.is_empty() {
                        warnings.push(format!(
                            "{} has no zone yet: add_zone gives it its plane (one net, or several nets side by side as a split plane)",
                            l.layer
                        ));
                    }
                    let tracks = b
                        .copper
                        .iter()
                        .filter(|c| match c.kind {
                            gummiband_core::board::CopperKind::Line { layer, .. }
                            | gummiband_core::board::CopperKind::Arc { layer, .. } => layer == lid,
                            _ => false,
                        })
                        .count();
                    if tracks > 0 {
                        warnings.push(format!(
                            "{tracks} locked tracks on {}: they stay, but the router no longer routes on a power layer",
                            l.layer
                        ));
                    }
                } else if l.kind != "power" && was_plane && !zone_nets.is_empty() {
                    warnings.push(format!(
                        "the zones of {} on {} are no longer planes: the router routes on the layer and those nets need tracks or a plane elsewhere",
                        zone_nets.join(", "),
                        l.layer
                    ));
                }
                changes.push((l.layer.clone(), l.kind.clone()));
                changed.push(LayerTypeOut {
                    layer: l.layer.clone(),
                    old_type: String::new(),
                    kind: l.kind.clone(),
                    zone_nets,
                });
            }
            let (t, old) = write::set_layer_types(text, &changes)?;
            for (c, o) in changed.iter_mut().zip(old) {
                c.old_type = o;
            }
            Ok(t)
        })?;
        let layers = self.with_entry(&id, |e| {
            Ok(e.kb
                .board
                .layers
                .iter()
                .enumerate()
                .map(|(i, l)| LayerOut {
                    index: i,
                    name: l.name.clone(),
                    plane: l.is_plane,
                })
                .collect::<Vec<_>>())
        })?;
        let s = format!(
            "Layer types in {path} (board {id}): {}. {}",
            changed
                .iter()
                .map(|c| format!("{} {} -> {}", c.layer, c.old_type, c.kind))
                .collect::<Vec<_>>()
                .join(", "),
            warnings.join(" ")
        );
        Ok((
            s,
            SetLayerTypeOut {
                board: id,
                path,
                changed,
                layers,
                warnings,
            },
        ))
    }

    fn set_options(&self, p: SetNetOptionsIn) -> Result<(String, SetNetOptionsOut)> {
        self.with_entry(&p.board, |e| {
            let b = &e.kb.board;
            let Some(id) = b.net_id(&p.net) else {
                let close: Vec<&str> = b
                    .nets
                    .iter()
                    .map(|n| n.name.as_str())
                    .filter(|n| n.to_lowercase().contains(&p.net.to_lowercase()))
                    .take(10)
                    .collect();
                if close.is_empty() {
                    bail!("no net named {:?} (see list_nets)", p.net);
                }
                bail!("no net named {:?}; similar: {}", p.net, close.join(", "))
            };
            let mut o = if p.reset.unwrap_or(false) {
                NetOptions::default()
            } else {
                e.options.get(&p.net).cloned().unwrap_or_default()
            };
            if let Some(pr) = p.priority {
                o.priority = Some(pr);
            }
            if let Some(s) = p.skip {
                o.skip = s;
            }
            if let Some(w) = p.width {
                o.width = (w > 0.0).then_some(w);
            }
            if let Some(m) = p.min_width {
                o.min_width = (m > 0.0).then_some(m);
            }
            if let Some(refs) = &p.neck_refs {
                o.neck_refs = Some(refs.clone());
            }
            // the same check as the route, now: on a copy of the board
            let mut probe = b.clone();
            gummiband_app::apply_net_options(&mut probe, &BTreeMap::from([(p.net.clone(), o.clone())]), &[])?;
            let priority = o.priority.unwrap_or(b.nets[id as usize].priority);
            if o.is_empty() {
                e.options.remove(&p.net);
            } else {
                e.options.insert(p.net.clone(), o.clone());
            }
            let summary = format!(
                "Net {} on board {}: priority {}{}{}.",
                p.net,
                p.board,
                priority,
                if o.priority.is_some() {
                    " (override)"
                } else {
                    " (class default)"
                },
                if o.skip { ", skipped" } else { "" }
            ) + &o.width.map(|w| format!(" Width {w} mm.")).unwrap_or_default()
                + &o.min_width
                    .map(|m| format!(" Minimum width {m} mm."))
                    .unwrap_or_default();
            Ok((
                summary,
                SetNetOptionsOut {
                    board: p.board.clone(),
                    net: p.net.clone(),
                    options: (&o).into(),
                    priority,
                },
            ))
        })
    }
}

/// The options of a route call: what the caller gave, and the defaults of the pipeline for
/// the rest; with them a note for every spelling of an earlier version among the arguments.
/// `loaded_keep`: the board was loaded with its locked copper, which is then the scope of a call
/// that names none.
fn route_options(p: &RouteIn, loaded_keep: bool) -> Result<(RouteOptions, Vec<Respelled>)> {
    let d = RouteOptions::default();
    let mut respelled = Vec::new();
    let old_scope = [
        ("keep_locked", p.keep_locked),
        ("keep_all", p.keep_all),
        ("reroute", p.reroute),
    ];
    let scope = match &p.scope {
        Some(word) => {
            if let Some((name, _)) = old_scope.iter().find(|(_, given)| given.is_some()) {
                bail!("{name} is the spelling of an earlier version and scope says the same: leave {name} out");
            }
            Scope::parse(word).map_err(anyhow::Error::msg)?
        }
        None => {
            let session = if loaded_keep { Scope::KeepLocked } else { Scope::Fresh };
            let (scope, note) =
                gummiband_app::old::scope(p.keep_locked, p.keep_all, p.reroute, session).map_err(anyhow::Error::msg)?;
            respelled.extend(note);
            scope
        }
    };
    let pairs = match &p.pairs {
        Some(WordOrSwitch::Word(word)) => Pairs::parse(word).map_err(anyhow::Error::msg)?,
        switch => {
            let switch = match switch {
                Some(WordOrSwitch::Switch(on)) => Some(*on),
                _ => None,
            };
            let (pairs, note) = gummiband_app::old::pairs(switch, p.pair_pull);
            respelled.extend(note);
            pairs
        }
    };
    let opts = RouteOptions {
        scope,
        nets: p.nets.clone().unwrap_or_default(),
        shove: p.shove,
        give: p.give,
        move_parts: p.move_parts,
        fixed: p.fixed.clone(),
        plane_layers: p.plane_layers.clone(),
        areas: match p.areas.as_deref() {
            None => d.areas,
            Some(word) => gummiband_app::Areas::parse(word).map_err(anyhow::Error::msg)?,
        },
        pour: p.pour.clone().unwrap_or_default(),
        area_stitch: p.area_stitch,
        preset: match &p.preset {
            Some(word) => Preset::parse(word).map_err(anyhow::Error::msg)?,
            None => d.preset,
        },
        runs: p.runs,
        via_pitch: p.via_pitch.unwrap_or(d.via_pitch),
        via_cost: p.via_cost.unwrap_or(d.via_cost),
        layer_dirs: p.layer_dirs.unwrap_or(d.layer_dirs),
        patterns: p.patterns.clone().unwrap_or(d.patterns),
        via_pass: p.via_pass.unwrap_or(d.via_pass),
        pairs,
        pair_pull: p.pair_pull,
        pair_skew: p.pair_skew.unwrap_or(d.pair_skew),
        pair_zdiff: p.pair_zdiff,
        pair_all: p.pair_all.unwrap_or(d.pair_all),
        breakouts: p.breakouts.unwrap_or(d.breakouts),
        via_pairs: p.via_pairs.unwrap_or(d.via_pairs),
        negotiate: p.negotiate.unwrap_or(d.negotiate),
        generations: p.generations,
        geometry_generations: p.geometry_generations,
        seed: p.seed,
        open_end_seconds: p.open_end_seconds,
        open_via_cost: p.open_via_cost.unwrap_or(d.open_via_cost),
        air: p.air,
        prefer_width: p.prefer_width.unwrap_or(d.prefer_width),
        angles: match p.angles.as_deref() {
            None | Some("any") => Angles::Any,
            Some("45") => Angles::Deg45,
            Some(other) => bail!("angles must be \"any\" or \"45\", not {other:?}"),
        },
        tune: p.tune.unwrap_or(d.tune),
        tune_amplitude: p.tune_amplitude.unwrap_or(d.tune_amplitude),
        tune_spacing: p.tune_spacing.unwrap_or(d.tune_spacing),
        placements: p.placements.unwrap_or(d.placements),
        placement_seed: p.placement_seed.unwrap_or(d.placement_seed),
        clear_corridors: p.clear_corridors.unwrap_or(d.clear_corridors),
        net_widths: p.net_widths.as_deref().map(abs_path),
        net_priorities: p.net_priorities.as_deref().map(abs_path),
        // the options per net of the session, filled in by the caller
        net_options: d.net_options,
        teardrops: p.teardrops.unwrap_or(d.teardrops),
        frames: p.frames.as_deref().map(abs_path),
    };
    Ok((opts, respelled))
}

/// Route the file of a loaded board and write the result. `respelled` are the arguments given
/// in the spelling of an earlier version: the result says what to write now.
fn route_blocking(
    id: String,
    path: PathBuf,
    opts: RouteOptions,
    respelled: &[Respelled],
    p: RouteIn,
) -> Result<(String, RouteOut)> {
    let output = match &p.output {
        Some(o) => abs_path(o),
        None => {
            let stem = path.file_stem().and_then(|s| s.to_str()).unwrap_or("board");
            path.with_file_name(format!("{stem}.routed.kicad_pcb"))
        }
    };
    let outcome = gummiband_app::route_file(&path, &output, &opts, &Default::default())?;
    let routed = &outcome.routed;
    let seconds = routed.seconds;
    let (b, res, viol, pair_reports) = (&routed.board, &routed.res, &routed.violations, &routed.pairs);
    let angles_45 = routed.angles_45.as_ref().map(|a| AnglesOut {
        length_before_mm: r3(a.stats.length_before),
        length_after_mm: r3(a.stats.length_after),
        off_grid_pieces: a.stats.any_angle,
        off_grid_mm: r3(a.stats.any_angle_length),
        sharp_corners: a.stats.sharp_corners,
    });
    let tuned = routed.tuned.as_ref().map(|t| &t.report);
    let written = outcome.write()?;
    let items = written.items;
    let mut moved: Vec<MovedOut> = res
        .offsets
        .iter()
        .map(|(r, d)| MovedOut {
            reference: r.clone(),
            dx_mm: r3(d.x),
            dy_mm: r3(d.y),
        })
        .collect();
    moved.sort_by(|a, b| a.reference.cmp(&b.reference));

    let net = |n: u32| b.nets[n as usize].name.clone();
    let mut per_net: BTreeMap<u32, NetStatOut> = BTreeMap::new();
    let mut stat = |n: u32| {
        per_net.entry(n).or_insert_with(|| NetStatOut {
            net: b.nets[n as usize].name.clone(),
            length_mm: 0.0,
            vias: 0,
            min_width_mm: f64::INFINITY,
            max_width_mm: 0.0,
        }) as *mut NetStatOut
    };
    for t in &res.geometry.tracks {
        let e = unsafe { &mut *stat(t.net) };
        for w in t.pts.windows(2) {
            let width = t.piece_width(w[0], w[1]);
            e.length_mm += w[0].dist(w[1]);
            e.min_width_mm = e.min_width_mm.min(width);
            e.max_width_mm = e.max_width_mm.max(width);
        }
    }
    for v in &res.geometry.vias {
        unsafe { (*stat(v.net)).vias += 1 };
    }
    let mut nets_out: Vec<NetStatOut> = per_net.into_values().collect();
    for n in &mut nets_out {
        n.length_mm = r3(n.length_mm);
        if !n.min_width_mm.is_finite() {
            n.min_width_mm = 0.0;
        }
    }
    nets_out.sort_by(|a, b| b.length_mm.total_cmp(&a.length_mm));
    let mut by_kind = BTreeMap::new();
    for v in viol {
        *by_kind
            .entry(v.what.split(' ').next().unwrap_or("").to_string())
            .or_insert(0) += 1;
    }
    let mut coupling = Vec::new();
    for (a, c) in b.diff_pairs() {
        let (w, g, _) = b.pair_shape(a, c, 0);
        let (coupled, total) = gummiband_core::pair_coupling_of(&res.geometry, a, c, w + g + 0.1);
        let vias = |n| res.geometry.vias.iter().filter(|v| v.net == n).count();
        let asked = opts.pairs != Pairs::Off;
        let uncoupled = if asked {
            gummiband_core::uncoupled_stretches(&res.geometry, a, c, w + g + 0.1, 0.5)
                .into_iter()
                .take(5)
                .map(|u| UncoupledOut {
                    net: net(u.net),
                    layer: b.layers[u.layer as usize].name.clone(),
                    x: r3(u.at.x),
                    y: r3(u.at.y),
                    length_mm: r3(u.length),
                    why: u.why.to_string(),
                })
                .collect()
        } else {
            Vec::new()
        };
        coupling.push(CouplingOut {
            p: net(a),
            n: net(c),
            coupled_mm: r3(coupled),
            length_mm: r3(total),
            vias_p: vias(a),
            vias_n: vias(c),
            uncoupled,
        });
    }
    let mut notes: Vec<String> = respelled.iter().map(Respelled::mcp).collect();
    for x in coupling
        .iter()
        .filter(|x| !x.uncoupled.is_empty() && x.coupled_mm < 0.6 * x.length_mm)
    {
        notes.push(format!(
            "differential pair {}/{} runs coupled for only {:.1} of {:.1} mm: coupling lists the longest uncoupled stretches with where and why",
            x.p, x.n, x.coupled_mm, x.length_mm
        ));
    }
    if !coupling.is_empty() && opts.pairs == Pairs::Off {
        let names: Vec<String> = coupling.iter().map(|x| format!("{}/{}", x.p, x.n)).collect();
        notes.push(format!(
            "differential pairs {} were routed as independent nets: set pairs to \"pull\" or \"band\" to couple them",
            names.join(", ")
        ));
    }
    notes.extend(routed.tight_pads().iter().map(|t| t.to_string()));
    // the second opinion of the fill model on the pours of the nets routed
    for p in &routed.legal.pours {
        notes.push(format!(
            "{p}: the pads count as connected through the pour, but its fill will not get to them; route a track to them or give the pour room (area_report on the written file has the details)"
        ));
    }
    let length_tuning: Vec<TunedOut> = tuned
        .iter()
        .flat_map(|r| &r.nets)
        .map(|n| TunedOut {
            net: n.net.clone(),
            before_mm: r3(n.before),
            after_mm: r3(n.after),
            aim_mm: r3(n.aim),
            as_pair: n.coupled,
        })
        .collect();
    let length_rules: Vec<LengthRuleOut> = routed
        .matching
        .iter()
        .map(|r| {
            use gummiband_core::board::MatchKind;
            LengthRuleOut {
                kind: match (r.kind, r.within_pairs) {
                    (MatchKind::Length, _) => "length",
                    (MatchKind::Skew, false) => "skew",
                    (MatchKind::Skew, true) => "skew within pairs",
                    (MatchKind::Uncoupled, _) => "uncoupled",
                    (MatchKind::Gap, _) => "gap",
                }
                .to_string(),
                min_mm: r.value.min,
                opt_mm: r.value.opt,
                max_mm: r.value.max,
                nets: r.nets.len(),
                failed: r
                    .nets
                    .iter()
                    .filter(|n| !n.ok)
                    .map(|n| LengthNetOut {
                        net: n.net.clone(),
                        length_mm: r3(n.length),
                        skew_mm: n.skew.map(r3),
                    })
                    .collect(),
                rule: r.rule.clone(),
            }
        })
        .collect();
    let st = &res.stats;
    let out = RouteOut {
        cleared_corridors: outcome.cleared_corridors.clone(),
        placement: outcome.placement.as_ref().map(|p| PlacementOut {
            seed: p.seed,
            moved: p.report.moved.clone(),
            tried: p
                .trials
                .iter()
                .map(|t| PlacementTriedOut {
                    seed: t.seed,
                    routed: t.routed,
                    connections: t.connections,
                    violations: t.violations,
                    vias: t.vias,
                    length_mm: r3(t.length),
                })
                .collect(),
        }),
        frames: written.frames.as_ref().map(|f| FramesOut {
            folder: f.dir.display().to_string(),
            pictures: f.svgs,
            animation: f.animation.as_ref().map(|a| a.display().to_string()),
        }),
        moved,
        angles_45,
        length_tuning,
        length_rules,
        coupling,
        notes,
        nets: nets_out,
        board: id,
        input: path.display().to_string(),
        output: output.display().to_string(),
        seed: routed.seed,
        seconds,
        search_seconds: routed.search_seconds,
        threads: rayon::current_num_threads(),
        connections: st.conns,
        routed: st.routed,
        unrouted: st.unrouted.clone(),
        blocked: st
            .blocked
            .iter()
            .map(|(connection, nets)| BlockedOut {
                connection: connection.clone(),
                nets: nets.clone(),
            })
            .collect(),
        vias: st.vias,
        length_mm: st.length,
        items_written: items,
        skipped_nets: outcome.skipped_nets.clone(),
        area_vias: routed.area_vias,
        areas: routed
            .areas
            .iter()
            .map(|a| RouteAreaOut {
                net: a.net.clone(),
                summary: a.summary.clone(),
                pads: a.pads,
                reached: a.reached,
                unreached: a
                    .unreached
                    .iter()
                    .map(|u| format!("{} ({})", u.pad, u.reason))
                    .collect(),
            })
            .collect(),
        alternatives: written
            .alternatives
            .iter()
            .map(|a| AlternativeOut {
                path: a.path.display().to_string(),
                routed: a.stats.routed,
                connections: a.stats.conns,
                vias: a.stats.vias,
                length_mm: r3(a.stats.length),
                clearance_spots: a.stats.violations.unwrap_or(0),
            })
            .collect(),
        pairs: pair_reports
            .iter()
            .map(|r| PairOut {
                p: r.p.clone(),
                n: r.n.clone(),
                wide: r.wide,
                note: r.note.clone(),
                width_mm: r.width,
                gap_mm: r.gap,
                zdiff_ohm: r.zdiff,
                length_p_mm: r.length_p,
                length_n_mm: r.length_n,
                skew_mm: r.skew,
                vias: r.vias,
                swaps: r.swaps,
            })
            .collect(),
        violations_total: viol.len(),
        violations_by_kind: by_kind,
        violations: viol
            .iter()
            .take(p.max_violations.unwrap_or(50))
            .map(|v| ViolationOut {
                what: v.what.clone(),
                x: v.at.x,
                y: v.at.y,
                actual_mm: v.actual,
                required_mm: v.required,
                nets: (net(v.nets.0), v.nets.1.map(net)),
            })
            .collect(),
        portfolio: res
            .portfolio
            .iter()
            .map(|r| PortfolioOut {
                generation: r.generation,
                seed: r.seed,
                fanout_first: r.fanout_first,
                routed: r.routed,
                connections: r.conns,
                vias: r.vias,
                length_mm: r.length,
                violations: r.violations,
                shorts: r.shorts,
            })
            .collect(),
        copied: written.copied.iter().map(|p| p.display().to_string()).collect(),
    };
    let summary = format!(
        "Routed {}/{} connections ({} unrouted), {} vias, {:.0} mm of track, {} clearance violations (Gummiband check), {:.1} s on {} threads. Wrote {} items to {}.{}",
        st.routed,
        st.conns,
        st.conns - st.routed,
        st.vias,
        st.length,
        viol.len(),
        seconds,
        out.threads,
        items,
        output.display(),
        if out.skipped_nets.is_empty() {
            String::new()
        } else {
            format!(" Skipped nets: {}.", out.skipped_nets.len())
        }
    ) + &match out.alternatives.len() {
        0 => String::new(),
        n => format!(
            " The open end left {n} more results that differ in vias and length: {}.",
            out.alternatives
                .iter()
                .map(|a| format!("{} ({} vias, {:.0} mm)", a.path, a.vias, a.length_mm))
                .collect::<Vec<_>>()
                .join(", ")
        ),
    } + &out
        .placement
        .as_ref()
        .map(|p| format!(" Placement of seed {} kept, of {} tried.", p.seed, p.tried.len()))
        .unwrap_or_default()
        + &out
            .frames
            .as_ref()
            .map(|f| format!(" {} pictures of the search in {}.", f.pictures, f.folder))
            .unwrap_or_default()
        + &out.notes.iter().map(|n| format!(" Note: {n}.")).collect::<String>();
    Ok((summary, out))
}

/// Sentence on the recognised pairs for the summaries, empty without pairs.
fn pairs_note(pairs: &[DiffPairOut]) -> String {
    if pairs.is_empty() {
        return String::new();
    }
    let names: Vec<String> = pairs.iter().map(|x| format!("{}/{}", x.p, x.n)).collect();
    format!(
        " Differential pairs: {}; route couples them only with pairs \"pull\" or \"band\".",
        names.join(", ")
    )
}

fn render_blocking(p: RenderIn) -> Result<(String, RenderOut, Vec<u8>)> {
    let path = abs_path(&p.path);
    if !path.is_file() {
        bail!("{} does not exist", path.display());
    }
    // the board tells the copper layers and the outline
    let kb = read_board(
        &path,
        &ReadOptions {
            keep: KeepCopper::All,
            ..Default::default()
        },
    )?;
    let layers = p.layers.clone().unwrap_or_else(|| {
        // routing layers only: plane layers are solid pours that hide everything else
        let mut l: Vec<String> = kb
            .board
            .layers
            .iter()
            .filter(|l| !l.is_plane)
            .map(|l| l.name.clone())
            .collect();
        l.push("Edge.Cuts".into());
        l
    });
    let region = match &p.region {
        Some(r) => (r.x0.min(r.x1), r.y0.min(r.y1), r.x0.max(r.x1), r.y0.max(r.y1)),
        None => {
            let bb = kb.board.bbox();
            (bb.min.x - 1.0, bb.min.y - 1.0, bb.max.x + 1.0, bb.max.y + 1.0)
        }
    };
    let width_px = p.width_px.unwrap_or(1000).clamp(16, 4000);
    // unfilled zones plot as nothing: fill them in a temp copy first
    let unfilled = gummiband_kicad::power::zones(&kb).iter().any(|z| !z.filled);
    let mut note = String::new();
    let plotted = if p.refill.unwrap_or(unfilled) {
        match external::refilled(&path) {
            Ok(f) => f,
            Err(e) => {
                note = format!(" Zones not filled ({e:#}).");
                path.clone()
            }
        }
    } else {
        path.clone()
    };
    let png = match &p.output {
        Some(o) => abs_path(o),
        None => {
            let stem = path.file_stem().and_then(|s| s.to_str()).unwrap_or("board");
            external::temp_file(&format!("{stem}-render"), "png")?
        }
    };
    let r = external::render(
        &plotted,
        &layers,
        region,
        width_px,
        p.background.as_deref().unwrap_or("white"),
        &png,
    )?;
    let data = std::fs::read(&r.png)?;
    let out = RenderOut {
        png: r.png.display().to_string(),
        width_px: r.width_px,
        height_px: r.height_px,
        layers,
        region: [region.0, region.1, region.2, region.3],
        scale: r.width_px as f64 / (region.2 - region.0),
    };
    let summary = format!(
        "Rendered {} ({}) region x {:.2}..{:.2}, y {:.2}..{:.2} mm at {}x{} px ({:.1} px/mm) to {}.{note}",
        path.file_name().and_then(|s| s.to_str()).unwrap_or(""),
        out.layers.join(","),
        region.0,
        region.2,
        region.1,
        region.3,
        r.width_px,
        r.height_px,
        out.scale,
        out.png
    );
    Ok((summary, out, data))
}

fn angles_45_blocking(p: Angles45In) -> Result<(String, Angles45Out)> {
    let path = abs_path(&p.path);
    let output = match &p.output {
        Some(o) => abs_path(o),
        None => {
            let stem = path.file_stem().and_then(|s| s.to_str()).unwrap_or("board");
            path.with_file_name(format!("{stem}.45.kicad_pcb"))
        }
    };
    let kb = read_board(
        &path,
        &ReadOptions {
            keep: KeepCopper::All,
            ..Default::default()
        },
    )?;
    let (text, r) = gummiband_kicad::octo_write::angles_45(&kb, p.nets.as_deref().unwrap_or(&[]))?;
    std::fs::write(&output, text).with_context(|| format!("cannot write {}", output.display()))?;
    external::copy_project_files(&path, &output);
    let st = &r.stats;
    let out = Angles45Out {
        output: output.display().to_string(),
        tracks: st.tracks,
        tracks_changed: r.tracks_changed,
        angles: AnglesOut {
            length_before_mm: r3(st.length_before),
            length_after_mm: r3(st.length_after),
            off_grid_pieces: st.any_angle,
            off_grid_mm: r3(st.any_angle_length),
            sharp_corners: st.sharp_corners,
        },
        violations_before: r.violations_before,
        violations_after: r.violations_after,
    };
    let s = format!(
        "45 degrees: {} of {} tracks changed, {:.0} -> {:.0} mm, {} pieces off the grid ({:.1} mm), {} sharp corners, Gummiband violations {} -> {}. Wrote {}.",
        out.tracks_changed,
        out.tracks,
        st.length_before,
        st.length_after,
        st.any_angle,
        st.any_angle_length,
        st.sharp_corners,
        r.violations_before,
        r.violations_after,
        out.output
    );
    Ok((s, out))
}

// -------------------------------------------------------------------------------- tools

#[tool_router]
impl Gummiband {
    pub fn new() -> Self {
        Gummiband {
            boards: Arc::new(Mutex::new(HashMap::new())),
            next_id: Arc::new(Mutex::new(0)),
            route_lock: Arc::new(tokio::sync::Mutex::new(())),
            tool_router: Self::tool_router(),
        }
    }

    #[tool(
        description = "Create a board from a KiCad netlist without KiCad: footprints from the KiCad footprint libraries with the nets on their pads (by pin number), symbol fields (LCSC, ...), DNP flags and sheet links; a rectangular outline (optional corner radius), 2 or 4 (or more) copper layers with stackup, optional inner power planes, optional mounting holes at the corners, and the .kicad_pro with design rules and net classes with patterns. Parts sit in rows inside the outline, unlocked, for move_part / place_parts. Pads without a netlist pin and pins without a pad are reported in warnings, with the pin_aliases entry to pass when it is clear. The new board is loaded; use the returned id.",
        output_schema = schema_for_output::<CreateBoardOut>(),
        annotations(read_only_hint = false, destructive_hint = false, idempotent_hint = true, open_world_hint = false)
    )]
    async fn create_board(&self, Parameters(p): Parameters<CreateBoardIn>) -> Result<CallToolResult, ErrorData> {
        let me = self.clone();
        match blocking(move || me.create(p)).await {
            Ok((s, out)) => ok(s, &out),
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "Load a KiCad .kicad_pcb (with its .kicad_pro and .kicad_dru) into the session. Returns a board id for the other tools, the copper layers, outline size, pin/net/connection counts, net classes and custom rule count. Loading the same file again re-reads it and keeps its id and net options.",
        output_schema = schema_for_output::<LoadBoardOut>(),
        annotations(read_only_hint = true, open_world_hint = false)
    )]
    async fn load_board(&self, Parameters(p): Parameters<LoadBoardIn>) -> Result<CallToolResult, ErrorData> {
        let me = self.clone();
        match blocking(move || me.load(p)).await {
            Ok((s, out)) => ok(s, &out),
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "List the nets of a loaded board with net class, pin count, plane flag, effective priority (lower routes first) and skip flag. By default only nets with 2 or more pins.",
        output_schema = schema_for_output::<ListNetsOut>(),
        annotations(read_only_hint = true, open_world_hint = false)
    )]
    async fn list_nets(&self, Parameters(p): Parameters<ListNetsIn>) -> Result<CallToolResult, ErrorData> {
        match self.nets(p) {
            Ok((s, out)) => ok(s, &out),
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "Local patterns of a loaded board that the router can use as hints: pin rows of a part, pad grids (BGA), decoupling capacitors with the IC pin they serve, pins of a plane net that can share one fan-out via, buses between two parts. Use it to understand the structure of a board before routing or pre-routing; `route` takes `patterns` to use them.",
        output_schema = schema_for_output::<BoardPatternsOut>(),
        annotations(read_only_hint = true, open_world_hint = false)
    )]
    async fn board_patterns(&self, Parameters(p): Parameters<BoardPatternsIn>) -> Result<CallToolResult, ErrorData> {
        match self.patterns_of(p) {
            Ok((s, out)) => ok(s, &out),
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "Groups of tracks between two parts (ICs and connectors sharing at least two short signal nets: a bus, SPI, I2C, a parallel interface) with the band they need and the movable parts that sit in it without being on its nets. Nothing moves: decide yourself, e.g. place_parts with only = some of in_the_way, corridors true and leash 1.5, then route with patterns groups. Decoupling capacitors and the parts on the group's nets are never listed. route with clear_corridors does the same for every listed part at once.",
        output_schema = schema_for_output::<TrackCorridorsOut>(),
        annotations(read_only_hint = true, open_world_hint = false)
    )]
    async fn track_corridors(&self, Parameters(p): Parameters<TrackCorridorsIn>) -> Result<CallToolResult, ErrorData> {
        let me = self.clone();
        match blocking(move || me.corridors_of(p)).await {
            Ok((s, out)) => ok(s, &out),
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "Pads (name, net, position, size, layers), courtyard and neighbouring parts of one footprint. Use it to plan a pre-routed layout, for example a datasheet recommendation, around a critical part.",
        output_schema = schema_for_output::<PartInfoOut>(),
        annotations(read_only_hint = true, open_world_hint = false)
    )]
    async fn part_info(&self, Parameters(p): Parameters<PartInfoIn>) -> Result<CallToolResult, ErrorData> {
        match self.part(p) {
            Ok((s, out)) => ok(s, &out),
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "Give parts a position from the netlist in the working copy <input>.prerouted.kicad_pcb; the input file stays untouched. Use the returned board id from now on. By default all parts move except locked ones, connectors, modules, mounting holes, buttons and test points (those move too while they sit piled up, on a pad of a larger part or off the board, listed in `released`); `only`, `scope` and `fixed` narrow that, so you can place the key parts yourself with move_part (and lock them) and leave the small parts to this tool; afterwards move_part refines any part. Run it before route: tracks do not follow. Read `warnings`: parts that stay piled up or off the board, pads of a plane net without room for their via (`no_via_room`).",
        output_schema = schema_for_output::<PlacePartsOut>(),
        annotations(read_only_hint = false, destructive_hint = false, idempotent_hint = true, open_world_hint = false)
    )]
    async fn place_parts(&self, Parameters(p): Parameters<PlacePartsIn>) -> Result<CallToolResult, ErrorData> {
        let me = self.clone();
        match blocking(move || me.do_place_parts(p)).await {
            Ok((s, out)) => ok(s, &out),
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "Move a part to a new position (and rotation) in the working copy <input>.prerouted.kicad_pcb; the input file stays untouched. Use the returned board id from now on. Without rotation the tracks around it follow like rubber bands (follow: false switches that off): its own tracks stretch, other nets give way within their clearances, and the part stops short where nothing can give way any more (a warning says where). `warnings` also names courtyards it now overlaps (by their outlines; drc has the exact answer). Works before and after place_parts and route, for manual placement and fine-tuning; lock the part afterwards so that place_parts and route leave it there. Locked copper neither follows nor gives way.",
        output_schema = schema_for_output::<AddLockedOut>(),
        annotations(read_only_hint = false, destructive_hint = false, idempotent_hint = true, open_world_hint = false)
    )]
    async fn move_part(&self, Parameters(p): Parameters<MovePartIn>) -> Result<CallToolResult, ErrorData> {
        let me = self.clone();
        match blocking(move || me.do_move_part(p)).await {
            Ok((s, out)) => ok(s, &out),
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "Lock (or with unlock: true unlock) parts by reference and the tracks and vias of nets, or all copper, in the working copy <input>.prerouted.kicad_pcb; only the lock flag changes. Locked parts stay in place_parts and route; locked copper is kept by route and never shoved or replaced. Use it to fix what you placed or routed yourself, or to freeze a good autorouted net before you route or edit the rest. Use the returned board id from now on.",
        output_schema = schema_for_output::<LockOut>(),
        annotations(read_only_hint = false, destructive_hint = false, idempotent_hint = true, open_world_hint = false)
    )]
    async fn lock(&self, Parameters(p): Parameters<LockIn>) -> Result<CallToolResult, ErrorData> {
        let me = self.clone();
        match blocking(move || me.do_lock(p)).await {
            Ok((s, out)) => ok(s, &out),
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "Remove zones (by name, net or layer, e.g. a pour of power_copper named \"gb: ...\" or a plane), tracks and vias (of nets, within a box, on layers, locked or unlocked only) and footprints (a mounting hole) from the working copy. Use it to correct power_copper, add_locked_copper, add_zone or a routed net without building the board again; then add the copper anew or route scope \"reroute\" with nets. Use the returned board id from now on.",
        output_schema = schema_for_output::<RemoveOut>(),
        annotations(read_only_hint = false, destructive_hint = true, idempotent_hint = true, open_world_hint = false)
    )]
    async fn remove(&self, Parameters(p): Parameters<RemoveIn>) -> Result<CallToolResult, ErrorData> {
        let me = self.clone();
        match blocking(move || me.do_remove(p)).await {
            Ok((s, out)) => ok(s, &out),
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "Add one locked, plain non-plated mounting hole (diameter, default 3.2 mm) at x, y to the working copy, besides or instead of create_board's corner holes; move_part moves it, remove (parts) takes one away. Warns where its courtyard overlaps a part. Use the returned board id from now on.",
        output_schema = schema_for_output::<AddLockedOut>(),
        annotations(read_only_hint = false, destructive_hint = false, idempotent_hint = false, open_world_hint = false)
    )]
    async fn add_mounting_hole(&self, Parameters(p): Parameters<MountingHoleIn>) -> Result<CallToolResult, ErrorData> {
        let me = self.clone();
        match blocking(move || me.do_mounting_hole(p)).await {
            Ok((s, out)) => ok(s, &out),
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "Place the reference texts on the silk screen after placement (also on a routed board, copper does not change): each stays if its spot is clear, else goes to the nearest clear spot around its courtyard, else smaller (down to min_size), else it is hidden. Clear: off the pads of its side, off other footprints' silk and the texts placed before, inside the board edge. This removes most of KiCad's silk_over_copper and silk_overlap messages. trim_edge also removes footprint silk over the board edge. Use the returned board id from now on; check with drc.",
        output_schema = schema_for_output::<PlaceSilkOut>(),
        annotations(read_only_hint = false, destructive_hint = false, idempotent_hint = true, open_world_hint = false)
    )]
    async fn place_silk(&self, Parameters(p): Parameters<PlaceSilkIn>) -> Result<CallToolResult, ErrorData> {
        let me = self.clone();
        match blocking(move || me.do_place_silk(p)).await {
            Ok((s, out)) => ok(s, &out),
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "Add KiCad custom rules ((rule ...) items as in a .kicad_dru) to the working copy, or replace them: the file <input>.prerouted.kicad_dru is written and the board reloaded, so KiCad's drc and Gummiband use them. Gummiband reads clearance rules by net, class and pair, minimum widths per class, length and skew rules, and clearance rules that hold in courtyards (A.intersectsCourtyard('U4') || ..., e.g. 0.1 mm at a fine pitch power IC), which its own check applies. The result lists what Gummiband understood. Use the returned board id from now on.",
        output_schema = schema_for_output::<SetRulesOut>(),
        annotations(read_only_hint = false, destructive_hint = false, idempotent_hint = false, open_world_hint = false)
    )]
    async fn set_rules(&self, Parameters(p): Parameters<SetRulesIn>) -> Result<CallToolResult, ErrorData> {
        let me = self.clone();
        match blocking(move || me.do_set_rules(p)).await {
            Ok((s, out)) => ok(s, &out),
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "Add a copper zone (net, layer, polygon, priority) to the working copy. On a KiCad power layer it becomes a plane the router connects to with vias; on a routing layer it is a pour the router keeps other nets out of. Planes and pours are yours to lay out, before or after route. KiCad fills it; drc refills zones.",
        output_schema = schema_for_output::<AddLockedOut>(),
        annotations(read_only_hint = false, destructive_hint = false, idempotent_hint = false, open_world_hint = false)
    )]
    async fn add_zone(&self, Parameters(p): Parameters<AddZoneIn>) -> Result<CallToolResult, ErrorData> {
        let me = self.clone();
        match blocking(move || me.do_add_zone(p)).await {
            Ok((s, out)) => ok(s, &out),
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "Set the KiCad type of inner copper layers in the working copy: power makes a layer a plane layer (its zones become planes the router reaches with vias, and it routes no tracks there), signal, mixed or jumper make it a routing layer. Use it to plan the planes before routing (a ground plane, a supply plane, a split plane for several supplies with add_zone). Use the returned board id from now on.",
        output_schema = schema_for_output::<SetLayerTypeOut>(),
        annotations(read_only_hint = false, destructive_hint = false, idempotent_hint = true, open_world_hint = false)
    )]
    async fn set_layer_type(&self, Parameters(p): Parameters<SetLayerTypeIn>) -> Result<CallToolResult, ErrorData> {
        let me = self.clone();
        match blocking(move || me.do_set_layer_type(p)).await {
            Ok((s, out)) => ok(s, &out),
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "Add pre-routed copper (tracks as polylines, vias) as locked items: the router keeps it and routes around it, and track ends join their net. Use it for manual routing before or after route: datasheet layouts, power lanes, critical nets, or to redraw a net the router did badly (then route scope \"reroute\" with nets for that net). The input file stays untouched: the copper goes into a working copy <input>.prerouted.kicad_pcb, which is loaded with keep_locked, so a route of it keeps the locked copper (scope \"keep_locked\") unless it names another scope; use the returned board id from now on. Check the result with drc and render.",
        output_schema = schema_for_output::<AddLockedOut>(),
        annotations(read_only_hint = false, destructive_hint = false, idempotent_hint = false, open_world_hint = false)
    )]
    async fn add_locked_copper(&self, Parameters(p): Parameters<AddLockedIn>) -> Result<CallToolResult, ErrorData> {
        let me = self.clone();
        match blocking(move || me.add_locked(p)).await {
            Ok((s, out)) => ok(s, &out),
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "Set per net options in the session, applied on the next route: priority (lower routes first, overrides the net class priority), skip (leave the net unrouted), width (the track width to route with), min_width (no necking below it; set it to the width for a current path) and neck_refs (the parts in whose courtyard the track may neck all the same). The same fields as an entry of the CLI's --net-widths file. reset=true clears the net's options first.",
        output_schema = schema_for_output::<SetNetOptionsOut>(),
        annotations(read_only_hint = false, destructive_hint = false, idempotent_hint = true, open_world_hint = false)
    )]
    async fn set_net_options(&self, Parameters(p): Parameters<SetNetOptionsIn>) -> Result<CallToolResult, ErrorData> {
        match self.set_options(p) {
            Ok((s, out)) => ok(s, &out),
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "Route a loaded board with Gummiband (topological any-angle router, portfolio plus evolutionary order search on all routing threads) and write the routed .kicad_pcb (default <input>.routed.kicad_pcb next to the input). `scope` says what of the copper on the board stays: \"keep_locked\" keeps the locked tracks and vias and lays all unlocked copper anew; \"finish\" keeps all copper and lays only what is still open, which is the scope for \"I laid these nets by hand, finish the rest\" (your nets stay whether they are locked or not; with `nets` only those are finished); \"reroute\" with `nets` takes the unlocked copper of those nets out and lays them again while everything else stays, which is the scope for \"lay net X again\"; \"fresh\" keeps nothing, locked copper goes too. `preset` says how much search the route gets: \"draft\" for a quick look at whether the board routes, \"quick\" for a first result, \"standard\" (the default) for a usable board, \"final\" for the last run (the open end improves the result for 10 minutes); runs, generations, geometry_generations and open_end_seconds win over it when given. It complements manual routing: locked tracks and vias (add_locked_copper, lock) are kept and routed around, locked parts do not move, and the result can be refined afterwards with move_part, add_locked_copper, lock and another route of single nets. Every option has the name, default and meaning of gummiband route (there as --kebab-case; only the default of scope is the session's: a board loaded with keep_locked and every working copy keeps its locked copper); options that contradict each other are refused with a message that says what to change, and so is an argument that does not exist. Returns routed/total connections, unrouted connections, vias, track length, Gummiband's clearance violations with nets and positions, the portfolio, and with open_end_seconds the alternatives written next to the output. CPU heavy: seconds to minutes; one route at a time. Check the result with drc.",
        output_schema = schema_for_output::<RouteOut>(),
        annotations(read_only_hint = false, destructive_hint = true, idempotent_hint = true, open_world_hint = false)
    )]
    async fn route(&self, Parameters(p): Parameters<RouteIn>) -> Result<CallToolResult, ErrorData> {
        let _guard = self.route_lock.lock().await;
        let me = self.clone();
        match blocking(move || me.route_entry(p)).await {
            Ok((s, out)) => ok(s, &out),
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "Run place_parts (then power_copper, if given) and route over several placement seeds from the same start (your hand placement, locks, zones and copper) and rank the results: fewest unrouted, then fewest KiCad DRC errors (Gummiband's check without kicad-cli), then fewest vias and the shortest length. The same placement rules can give anything from 0 to dozens of errors on a tight board, so try a few seeds before refining one. Each seed's placed working copy and routed file are kept (<input>.seed<N>.*); the working copy becomes the placement of the best seed, and its routed board is loaded too. CPU heavy: one route per seed.",
        output_schema = schema_for_output::<TrySeedsOut>(),
        annotations(read_only_hint = false, destructive_hint = true, idempotent_hint = true, open_world_hint = false)
    )]
    async fn try_seeds(&self, Parameters(p): Parameters<TrySeedsIn>) -> Result<CallToolResult, ErrorData> {
        let _guard = self.route_lock.lock().await;
        let me = self.clone();
        match blocking(move || me.do_try_seeds(p)).await {
            Ok((s, out)) => ok(s, &out),
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "Turn the unlocked tracks of a routed .kicad_pcb into conventional 45 degree tracks without routing again (the same step as route with angles \"45\", on copper that is already there). Vias, track ends, pads and locked tracks stay; every new piece keeps the clearances, a piece with no room on the grid keeps its angle, and tracks that do not change stay in the file as they are. Writes <input>.45.kicad_pcb by default. Check the result with drc.",
        output_schema = schema_for_output::<Angles45Out>(),
        annotations(read_only_hint = false, destructive_hint = false, idempotent_hint = true, open_world_hint = false)
    )]
    async fn angles_45(&self, Parameters(p): Parameters<Angles45In>) -> Result<CallToolResult, ErrorData> {
        match blocking(move || angles_45_blocking(p)).await {
            Ok((s, out)) => ok(s, &out),
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "Plan the power copper before routing (IPC-2221): for the given currents per net (glob patterns; left out: estimated from the class widths) the required width per copper layer from the stackup, the track width to route with, vias per layer change, the planes and zones the net has, and advice (widen, pour instead of track, plane vias, necking rules). board_notes cover the GND plane, plane layers and reference planes; dru_rules has the widths as KiCad custom rules. apply=true stores the widths as net options (width and minimum width) for the next route.",
        output_schema = schema_for_output::<practice::PowerPlanOut>(),
        annotations(read_only_hint = false, destructive_hint = false, idempotent_hint = true, open_world_hint = false)
    )]
    async fn power_plan(&self, Parameters(p): Parameters<practice::PowerPlanIn>) -> Result<CallToolResult, ErrorData> {
        let res = self.with_entry(&p.board, |e| {
            let (out, applied) = practice::power_plan(&e.kb, &p.board, &p)?;
            for (net, w) in &applied {
                // a current path: the width is its minimum too
                let o = e.options.entry(net.clone()).or_default();
                o.width = Some(*w);
                o.min_width = Some(*w);
            }
            Ok(out)
        });
        match res {
            Ok(out) => {
                let flagged = out.nets.iter().filter(|n| n.action).count();
                let s = format!(
                    "Power plan for board {} at {} K rise: {} nets, {} need a change, {} widths applied. {}",
                    out.board,
                    out.temp_rise,
                    out.nets.len(),
                    flagged,
                    out.applied.len(),
                    out.board_notes.join("; ")
                );
                ok(s, &out)
            }
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "Check the return paths of a routed .kicad_pcb: walks every signal track over the zones of the copper layers next to its layer (its reference planes) and reports gaps (stretches with no copper underneath: a slot, the edge of a zone, merged antipads) and splits (the track passes from a zone of one net into a zone of another). Fast signals and differential pairs need unbroken copper under them. Refills zones with kicad-cli when needed. By default all nets except ground, plane nets and wide power classes; gaps shorter than min_gap (0.5 mm) are only counted.",
        output_schema = schema_for_output::<reference::ReferenceCheckOut>(),
        annotations(read_only_hint = false, destructive_hint = false, idempotent_hint = true, open_world_hint = false)
    )]
    async fn reference_check(
        &self,
        Parameters(mut p): Parameters<reference::ReferenceCheckIn>,
    ) -> Result<CallToolResult, ErrorData> {
        p.path = abs_path(&p.path).display().to_string();
        match blocking(move || reference::reference_check(&p)).await {
            Ok(out) => {
                let worst: Vec<String> = out
                    .nets
                    .iter()
                    .take(5)
                    .map(|n| {
                        format!(
                            "{} ({} splits, {} gaps, {:.1} mm over no copper)",
                            n.net, n.splits, n.gaps, n.gap_mm
                        )
                    })
                    .collect();
                let s = format!(
                    "Reference check of {}: {} nets, {:.0} mm of track, {} findings ({} short gaps only counted). {}{}",
                    out.path,
                    out.nets_checked,
                    out.track_mm,
                    out.findings,
                    out.short_gaps,
                    if worst.is_empty() {
                        "No net crosses a gap or a split.".to_string()
                    } else {
                        format!("Worst: {}. ", worst.join("; "))
                    },
                    out.notes.join(" ")
                );
                ok(s, &out)
            }
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "Check the current carrying capacity of a routed .kicad_pcb (IPC-2221): per current path the track segments below the required width, and with from/to pads (automatic for two pin nets) the narrowest cross-section between them, measured on a raster over tracks, pours and vias of the net together, with the position of the neck. Also vias against the current. Refills zones with kicad-cli when needed. Verdict ok or FLAG per path.",
        output_schema = schema_for_output::<practice::CurrentCheckOut>(),
        annotations(read_only_hint = false, destructive_hint = false, idempotent_hint = true, open_world_hint = false)
    )]
    async fn current_check(
        &self,
        Parameters(mut p): Parameters<practice::CurrentCheckIn>,
    ) -> Result<CallToolResult, ErrorData> {
        p.path = abs_path(&p.path).display().to_string();
        match blocking(move || practice::current_check(&p)).await {
            Ok(out) => {
                let lines: Vec<String> = out
                    .paths
                    .iter()
                    .map(|q| format!("{} {} A: {}", q.name, q.amps, q.verdict))
                    .collect();
                let s = format!(
                    "Current check of {}: {} paths, {} flagged. {}",
                    out.path,
                    out.paths.len(),
                    out.flagged,
                    lines.join("; ")
                );
                ok(s, &out)
            }
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "Lay out power copper before routing (or, with unlocked: \"clear\", on a routed board: the unlocked tracks and vias of other nets in the way are removed and reroute_nets lists what to route again with scope \"reroute\" and nets; by default the new copper gives way to them), for current paths that a track carries badly (a converter's switch node, a pad into a plane through one via): for a net without plane one pour over its from and to pads on their layer (convex hull grown by margin, cut back from other nets' copper, holes and the edge; a long narrow pad between neighbours, like the strips of a DFN eFuse, is left through a narrow stub off an end or through a gap in its side), for a plane net at each terminal a field of locked vias for the current (IPC-2221 per via) and a small pour joining them to the pad. The pours are zones named \"gb: ...\": the router keeps other nets out and treats the pads under one pour as connected. Writes the working copy <input>.prerouted.kicad_pcb; route the returned board id. Reports the pours, what cut them and a current_check of the pours.",
        output_schema = schema_for_output::<practice::PowerCopperOut>(),
        annotations(read_only_hint = false, destructive_hint = false, idempotent_hint = false, open_world_hint = false)
    )]
    async fn power_copper(
        &self,
        Parameters(p): Parameters<practice::PowerCopperIn>,
    ) -> Result<CallToolResult, ErrorData> {
        let me = self.clone();
        let res = blocking(move || me.do_power_copper(&p)).await;
        match res {
            Ok(out) => {
                let lines: Vec<String> = out
                    .pours
                    .iter()
                    .map(|x| {
                        if x.kind == "pour" {
                            format!("{} pour on {} ({} mm²)", x.net, x.layer, x.area_mm2)
                        } else {
                            format!(
                                "{} {} of {} vias at {}",
                                x.net,
                                x.vias_added,
                                x.vias_needed,
                                x.terminals.join(", ")
                            )
                        }
                    })
                    .collect();
                let verdicts = out
                    .check
                    .as_ref()
                    .map(|c| {
                        c.paths
                            .iter()
                            .map(|q| format!("{}: {}", q.name, q.verdict))
                            .collect::<Vec<_>>()
                            .join("; ")
                    })
                    .unwrap_or_default();
                let s = format!(
                    "Power copper in {} (board {}): {}. {} {}",
                    out.path,
                    out.board,
                    lines.join("; "),
                    verdicts,
                    out.warnings.join(" ")
                );
                ok(s, &out)
            }
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "Pour ground after routing: adds zones of the ground net (default GND) over the board outline on the outer layers (default), with the fill rules given (pad_connection, thermal_spoke_width, thermal_gap, zone_clearance, zone_min_width), and locked stitching vias: by default one via for every island of fill with pads, at the spot the router's fill model finds, and never a via that joins nothing (stitch: islands, grid, both or none; the grid sets vias on a pitch and along the board edge where they are clear). nets pours several nets at once (GND and GNDR): the board is shared by nearest pad, each net gets zones of its own and they never overlap. via_in_pad also sets a via in an exposed pad that nothing else reaches. The result names per net what the pour reaches and which pads stay unreached and why. Writes <input>.gnd.kicad_pcb and loads it; KiCad fills the zones (drc refills them).",
        output_schema = schema_for_output::<practice::GroundPourOut>(),
        annotations(read_only_hint = false, destructive_hint = false, idempotent_hint = false, open_world_hint = false)
    )]
    async fn ground_pour(
        &self,
        Parameters(mut p): Parameters<practice::GroundPourIn>,
    ) -> Result<CallToolResult, ErrorData> {
        p.path = abs_path(&p.path).display().to_string();
        let me = self.clone();
        let res = blocking(move || {
            let (path, mut out) = practice::ground_pour(&p)?;
            let (_, loaded) = me.load(LoadBoardIn {
                path: path.display().to_string(),
                keep_locked: Some(true),
            })?;
            out.board = loaded.board;
            Ok(out)
        })
        .await;
        match res {
            Ok(out) => {
                let found: Vec<&str> = out.nets.iter().map(|n| n.summary.as_str()).collect();
                let s = format!(
                    "{} pours on {} and {} stitching vias ({} along the edge) in {} (board {}). By the fill model: {} {}",
                    out.net,
                    if out.zones_added.is_empty() {
                        "no layer".to_string()
                    } else {
                        out.zones_added.join(", ")
                    },
                    out.vias_added,
                    out.edge_vias,
                    out.path,
                    out.board,
                    found.join(" "),
                    out.warnings.join(" ")
                );
                ok(s, &out)
            }
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "What the copper fill of the nets with a zone (pours, planes) reaches on the copper a .kicad_pcb has, judged by the router's own fill model (no KiCad needed; KiCad's fill has the last word): per net the pieces of fill, the pads the area reaches, the pads it does not reach with the reason in plain words, the islands with their pads and the site of one via that joins each to the area (or why none fits), the distance from the pads to the nearest spot where a via lands in the area, and the pads whose answer changes within one raster cell. With nets and pour it answers what a pour over the board outline would reach before one is written. The file stays as it is.",
        output_schema = schema_for_output::<practice::AreaReportOut>(),
        annotations(read_only_hint = true, destructive_hint = false, idempotent_hint = true, open_world_hint = false)
    )]
    async fn area_report(
        &self,
        Parameters(mut p): Parameters<practice::AreaReportIn>,
    ) -> Result<CallToolResult, ErrorData> {
        p.path = abs_path(&p.path).display().to_string();
        match blocking(move || practice::area_report(&p)).await {
            Ok(out) => {
                let s = if out.summary.is_empty() {
                    format!("No net with an area in {}. {}", out.path, out.notes.join(" "))
                } else {
                    format!(
                        "Areas of {}: {} {}",
                        out.path,
                        out.summary.join(" "),
                        out.notes.join(" ")
                    )
                };
                ok(s, &out)
            }
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "Run KiCad's DRC (kicad-cli pcb drc --refill-zones) on a .kicad_pcb and summarize it: violation counts per type (silkscreen and library checks left out), unconnected items, a deficit histogram and the worst clearance violations with positions. The full report is written next to the board as <name>.drc.json.",
        output_schema = schema_for_output::<external::DrcOut>(),
        annotations(read_only_hint = false, destructive_hint = false, idempotent_hint = true, open_world_hint = false)
    )]
    async fn drc(&self, Parameters(p): Parameters<DrcIn>) -> Result<CallToolResult, ErrorData> {
        let path = abs_path(&p.path);
        let max = p.max_items.unwrap_or(20);
        match blocking(move || external::drc(&path, max)).await {
            Ok(out) => {
                let s = format!(
                    "KiCad DRC of {}: {} unconnected, {} violations {:?} ({} silk/lib findings ignored). Clearance deficits <2um/<10um/<30um/<100um/more: {:?}.",
                    out.path, out.unconnected, out.violations, out.counts, out.ignored, out.deficit_histogram
                );
                ok(s, &out)
            }
            Err(e) => fail(format!("{e:#}")),
        }
    }

    #[tool(
        description = "How to use Gummiband, from the server's own documentation (Markdown). Without topic: the workflow (Gummiband complements manual work: what to place, route, pour and lock by hand, what place_parts and route do with it, the order of work and fine-tuning afterwards) and the list of topics. With topic: the reference of one tool (e.g. route, place_parts, lock), a section (good_practice, choosing_opt_in_options, open_points, ...), concepts or options. Read it before a whole board and whenever a tool's arguments or results are unclear.",
        output_schema = schema_for_output::<GuideOut>(),
        annotations(read_only_hint = true, destructive_hint = false, idempotent_hint = true, open_world_hint = false)
    )]
    async fn guide(&self, Parameters(p): Parameters<GuideIn>) -> Result<CallToolResult, ErrorData> {
        let (topic, text, topics) = guide::lookup(p.topic.as_deref());
        let out = GuideOut { topic, topics, text };
        let value = serde_json::to_value(&out).map_err(|e| ErrorData::internal_error(e.to_string(), None))?;
        // the Markdown once as text, for the reader; the structured copy for clients that read only that
        let mut r = CallToolResult::success(vec![ContentBlock::text(out.text)]);
        r.structured_content = Some(value);
        Ok(r)
    }

    #[tool(
        description = "Render a .kicad_pcb (or a region of it, in board mm) to PNG via kicad-cli SVG export and rsvg-convert. Returns the image and the PNG path. Default layers: the routing copper layers (no inner planes) and Edge.Cuts.",
        output_schema = schema_for_output::<RenderOut>(),
        annotations(read_only_hint = false, destructive_hint = false, idempotent_hint = true, open_world_hint = false)
    )]
    async fn render(&self, Parameters(p): Parameters<RenderIn>) -> Result<CallToolResult, ErrorData> {
        match blocking(move || render_blocking(p)).await {
            Ok((s, out, png)) => {
                let mut r = ok(s, &out)?;
                r.content.push(ContentBlock::image(
                    base64::engine::general_purpose::STANDARD.encode(png),
                    "image/png",
                ));
                Ok(r)
            }
            Err(e) => fail(format!("{e:#}")),
        }
    }
}

#[tool_handler(router = self.tool_router)]
impl ServerHandler for Gummiband {
    fn get_info(&self) -> ServerConfig {
        ServerConfig::new(ServerCapabilities::builder().enable_tools().enable_prompts().build())
            .with_server_info(
                Implementation::new("gummiband-mcp", env!("CARGO_PKG_VERSION")).with_title("Gummiband autorouter"),
            )
            .with_instructions(INSTRUCTIONS)
    }

    #[allow(clippy::field_reassign_with_default)]
    async fn list_prompts(
        &self,
        _request: Option<rmcp::model::PaginatedRequestParams>,
        _context: rmcp::service::RequestContext<rmcp::RoleServer>,
    ) -> Result<rmcp::model::ListPromptsResult, ErrorData> {
        let arg = |name: &str, desc: &str, required: bool| {
            let mut a = rmcp::model::PromptArgument::new(name);
            a.description = Some(desc.into());
            a.required = Some(required);
            a
        };
        let mut r = rmcp::model::ListPromptsResult::default();
        r.prompts = vec![rmcp::model::Prompt::new(
            practice::PROMPT_NAME,
            Some(practice::PROMPT_DESCRIPTION),
            Some(vec![
                arg("path", "Path of the .kicad_pcb to route", true),
                arg(
                    "currents",
                    "Known currents, e.g. 'VBUS 4.1 A from J1 to U6, +5V 8 A'",
                    false,
                ),
                arg(
                    "notes",
                    "Anything else the router should know (keep-outs, parts that stay, impedances)",
                    false,
                ),
            ]),
        )];
        Ok(r)
    }

    async fn get_prompt(
        &self,
        request: rmcp::model::GetPromptRequestParams,
        _context: rmcp::service::RequestContext<rmcp::RoleServer>,
    ) -> Result<rmcp::model::GetPromptResponse, ErrorData> {
        if request.name != practice::PROMPT_NAME {
            return Err(ErrorData::invalid_params(format!("no prompt {:?}", request.name), None));
        }
        let args = request.arguments.unwrap_or_default();
        let get = |k: &str| args.get(k).and_then(|v| v.as_str()).map(str::to_string);
        let path = get("path").unwrap_or_else(|| "<path of the .kicad_pcb>".into());
        let text = practice::prompt_text(&path, get("currents").as_deref(), get("notes").as_deref());
        Ok(
            rmcp::model::GetPromptResult::new(vec![rmcp::model::PromptMessage::new_text(
                rmcp::model::Role::User,
                text,
            )])
            .with_description(practice::PROMPT_DESCRIPTION)
            .into(),
        )
    }
}

fn b_has_kicad_pro(pcb: &std::path::Path) -> bool {
    pcb.with_extension("kicad_pro").is_file()
}

#[cfg(test)]
mod tests {
    use super::*;
    use gummiband_app::OPTIONS;

    /// Arguments of `route` that are the session's own. Everything else has to be an option of
    /// the pipeline.
    const OWN: &[&str] = &["board", "output", "max_violations"];

    /// Arguments of an earlier version that `route` still takes and maps onto the options of
    /// today (`gummiband_app::old`). None of them is in the schema or in a description.
    const OLD: &[&str] = &["keep_locked", "keep_all", "reroute"];

    /// Every option of the pipeline is an argument of `route` under its name, and `route` has no
    /// option of the pipeline that the table lacks. The CLI has the same test against the same
    /// table, so the two front ends offer the same.
    #[test]
    fn route_offers_the_options_of_the_pipeline() {
        let schema = serde_json::to_value(schemars::schema_for!(RouteIn)).unwrap();
        let args: Vec<&String> = schema["properties"].as_object().expect("properties").keys().collect();
        for o in OPTIONS {
            assert!(args.iter().any(|a| *a == o.name), "route lacks {}", o.name);
        }
        for a in args {
            assert!(
                OWN.contains(&a.as_str()) || gummiband_app::option_doc(a).is_some(),
                "{a} is neither in gummiband_app::OPTIONS nor one of the session's own arguments"
            );
        }
        // an argument that does not exist is refused
        assert_eq!(schema["additionalProperties"], serde_json::json!(false));
        assert!(serde_json::from_value::<RouteIn>(serde_json::json!({"board": "b1", "pair_breakouts": true})).is_err());
    }

    /// The same for `area_report` and the options of the area report: the tool offers every
    /// entry of the table under its name and nothing of its own but the file.
    #[test]
    fn area_report_offers_the_options_of_the_report() {
        use practice::AreaReportIn;
        const OWN: &[&str] = &["path"];
        let schema = serde_json::to_value(schemars::schema_for!(AreaReportIn)).unwrap();
        let args: Vec<&String> = schema["properties"].as_object().expect("properties").keys().collect();
        for o in gummiband_app::AREA_OPTIONS {
            assert!(args.iter().any(|a| *a == o.name), "area_report lacks {}", o.name);
        }
        for a in args {
            assert!(
                OWN.contains(&a.as_str()) || gummiband_app::area_option_doc(a).is_some(),
                "{a} is neither in gummiband_app::AREA_OPTIONS nor the tool's own argument"
            );
        }
        assert_eq!(schema["additionalProperties"], serde_json::json!(false));
        // with nothing given the report runs with the defaults of the table
        let args = |v: serde_json::Value| serde_json::from_value::<AreaReportIn>(v).unwrap().options();
        assert_eq!(
            args(serde_json::json!({"path": "b.kicad_pcb"})),
            gummiband_app::AreaOptions::default()
        );
        let o = args(serde_json::json!({
            "path": "b.kicad_pcb", "nets": ["GND"], "pour": ["F.Cu", "B.Cu"], "zone_min_width": 0.15,
            "pad_connection": "solid", "doubtful": false, "plane_layers": ["In1.Cu"],
        }));
        assert_eq!(
            (o.nets, o.pour),
            (vec!["GND".to_string()], vec!["F.Cu".to_string(), "B.Cu".into()])
        );
        assert_eq!(
            (o.zone_min_width, o.pad_connection.as_str(), o.doubtful),
            (0.15, "solid", false)
        );
        assert!(serde_json::from_value::<AreaReportIn>(serde_json::json!({"path": "b", "net": "GND"})).is_err());
    }

    /// The options of a call of `route` and the notes about old spellings in it.
    fn route_call(v: serde_json::Value) -> (RouteOptions, Vec<String>) {
        let (opts, notes) = route_options(&serde_json::from_value::<RouteIn>(v).unwrap(), false).unwrap();
        (opts, notes.iter().map(Respelled::mcp).collect())
    }

    /// With nothing given the tool routes with the defaults of the pipeline, and what is given
    /// arrives under its name.
    #[test]
    fn arguments_become_the_options_of_the_pipeline() {
        let args = |v: serde_json::Value| -> RouteOptions {
            let (opts, notes) = route_call(v);
            assert!(notes.is_empty(), "{notes:?}");
            opts
        };
        let opts = args(serde_json::json!({"board": "b1"}));
        assert_eq!(format!("{opts:?}"), format!("{:?}", RouteOptions::default()));
        let opts = args(serde_json::json!({
            "board": "b1", "scope": "reroute", "nets": ["A", "B"], "shove": false,
            "give": 2, "tune": false, "open_end_seconds": 30, "plane_layers": ["In1.Cu"],
            "breakouts": true, "placements": 3, "teardrops": true, "angles": "45",
        }));
        let fill = args(serde_json::json!({"board": "b1", "areas": "fill", "pour": ["GND:F.Cu,B.Cu", "A"]}));
        assert!(fill.areas == gummiband_app::Areas::Fill);
        assert_eq!(fill.pour, ["GND:F.Cu,B.Cu", "A"]);
        assert_eq!(fill.validate(false), Ok(()));
        let bad = serde_json::from_value::<RouteIn>(serde_json::json!({"board": "b1", "areas": "pour"})).unwrap();
        assert!(
            route_options(&bad, false)
                .unwrap_err()
                .to_string()
                .contains("areas must be")
        );
        assert!(opts.scope == Scope::Reroute && !opts.tune && opts.breakouts && opts.teardrops);
        assert_eq!(opts.nets, ["A", "B"]);
        assert_eq!((opts.shove, opts.give, opts.move_parts), (Some(false), Some(2.0), None));
        assert_eq!((opts.open_end_seconds, opts.placements), (Some(30.0), 3));
        assert_eq!(opts.plane_layers, ["In1.Cu"]);
        assert_eq!(opts.angles, Angles::Deg45);
        // contradictions are the pipeline's to refuse, in its words
        assert!(
            opts.validate(false)
                .unwrap_err()
                .contains("breakouts acts on the wide track of pairs band only")
        );
    }

    /// An option with a fixed list of words has them in its schema, with the sentence of each
    /// from the table of gummiband-app: an agent that reads only the schema knows what to pick.
    #[test]
    fn a_choice_has_the_words_of_the_table_in_its_schema() {
        let schema = serde_json::to_value(schemars::schema_for!(RouteIn)).unwrap();
        let required: Vec<&str> = schema["required"]
            .as_array()
            .expect("required")
            .iter()
            .filter_map(|v| v.as_str())
            .collect();
        assert_eq!(required, ["board"]);
        for name in ["pairs", "scope", "preset"] {
            let doc = gummiband_app::option_doc(name).unwrap();
            let prop = &schema["properties"][name];
            let words: Vec<&str> = doc.choices.iter().map(|c| c.value).collect();
            assert_eq!(prop["enum"], serde_json::json!(words), "{name}");
            let text = prop["description"].as_str().expect("description");
            if name == "scope" {
                // the default is the session's: the copper the board was loaded with
                assert_eq!(text, doc.describe_with(SCOPE_DEFAULT));
            } else {
                assert_eq!(text, doc.describe(), "{name}");
            }
            for c in doc.choices {
                assert!(
                    text.contains(&format!("\n{}: {}", c.value, c.help)),
                    "{name} {}",
                    c.value
                );
            }
        }
        // a word that is none of them is refused with the words there are
        let bad = serde_json::from_value::<RouteIn>(serde_json::json!({"board": "b1", "pairs": "wide"})).unwrap();
        let e = route_options(&bad, false).unwrap_err().to_string();
        assert_eq!(e, "pairs must be off, pull or band, not \"wide\"");
        let (opts, notes) = route_call(serde_json::json!({"board": "b1", "pairs": "pull"}));
        assert!(opts.pairs == Pairs::Pull && opts.pair_pull() == 1.0 && notes.is_empty());
        let bad = serde_json::from_value::<RouteIn>(serde_json::json!({"board": "b1", "scope": "all"})).unwrap();
        let e = route_options(&bad, false).unwrap_err().to_string();
        assert_eq!(e, "scope must be fresh, keep_locked, finish or reroute, not \"all\"");
    }

    /// An agent that reads only the schema of `route` knows which scope to pick: for nets it
    /// laid by hand and for a net to lay again. The tool's description says the same.
    #[test]
    fn the_schema_says_which_scope_to_pick() {
        let schema = serde_json::to_value(schemars::schema_for!(RouteIn)).unwrap();
        let text = schema["properties"]["scope"]["description"].as_str().unwrap();
        let line = |word: &str| {
            text.lines()
                .find(|l| l.starts_with(&format!("{word}: ")))
                .unwrap_or_else(|| panic!("no line for {word}"))
        };
        assert!(line("finish").contains("\"I laid these nets by hand, finish the rest\""));
        assert!(line("reroute").contains("\"lay net X again\"") && line("reroute").contains("Needs nets"));
        assert!(text.ends_with(&format!("Default: {SCOPE_DEFAULT}.")));
        // the options that hang on the scope name it
        for name in [
            "nets",
            "shove",
            "give",
            "move_parts",
            "air",
            "placements",
            "clear_corridors",
        ] {
            let text = schema["properties"][name]["description"].as_str().unwrap();
            assert!(
                text.contains("scope \"") || text.contains("scope\n\""),
                "{name}: {text}"
            );
        }
    }

    /// A preset sets four options, and each of them wins over it when given. The schema says
    /// what each preset sets.
    #[test]
    fn a_preset_sets_what_was_not_given() {
        use serde_json::json;
        let effort = |v: serde_json::Value| {
            let (o, notes) = route_call(v);
            assert!(notes.is_empty());
            (
                o.runs(),
                o.generations(),
                o.geometry_generations(),
                o.open_end_seconds(),
            )
        };
        assert_eq!(effort(json!({"board": "b1"})), (14, 3, 3, 0.0));
        assert_eq!(effort(json!({"board": "b1", "preset": "draft"})), (4, 1, 0, 0.0));
        assert_eq!(effort(json!({"board": "b1", "preset": "quick"})), (14, 3, 0, 0.0));
        assert_eq!(effort(json!({"board": "b1", "preset": "final"})), (14, 3, 3, 600.0));
        assert_eq!(
            effort(json!({"board": "b1", "preset": "final", "open_end_seconds": 30, "runs": 6})),
            (6, 3, 3, 30.0)
        );
        let schema = serde_json::to_value(schemars::schema_for!(RouteIn)).unwrap();
        let text = schema["properties"]["preset"]["description"].as_str().unwrap();
        assert!(text.contains("\ndraft: runs 4, generations 1, geometry_generations 0, open_end_seconds 0: "));
        assert!(text.ends_with("Default: standard."));
        for name in ["runs", "generations", "geometry_generations", "open_end_seconds"] {
            let text = schema["properties"][name]["description"].as_str().unwrap();
            assert!(text.contains("`preset`"), "{name}: {text}");
        }
        let bad = serde_json::from_value::<RouteIn>(json!({"board": "b1", "preset": "fast"})).unwrap();
        assert_eq!(
            route_options(&bad, false).unwrap_err().to_string(),
            "preset must be draft, quick, standard or final, not \"fast\""
        );
    }

    /// Without a scope a call routes with the scope of its session: a board loaded with its
    /// locked copper keeps it, and a word says otherwise.
    #[test]
    fn the_scope_of_a_call_is_its_word_or_the_session_s() {
        use serde_json::json;
        let call = |v: serde_json::Value, loaded_keep: bool| {
            let (opts, notes) = route_options(&serde_json::from_value::<RouteIn>(v).unwrap(), loaded_keep).unwrap();
            assert!(notes.is_empty());
            opts.scope
        };
        assert_eq!(call(json!({"board": "b1"}), false), Scope::Fresh);
        assert_eq!(call(json!({"board": "b1"}), true), Scope::KeepLocked);
        assert_eq!(call(json!({"board": "b1", "scope": "fresh"}), true), Scope::Fresh);
        assert_eq!(call(json!({"board": "b1", "scope": "finish"}), true), Scope::Finish);
        assert_eq!(
            call(json!({"board": "b1", "scope": "keep-locked"}), false),
            Scope::KeepLocked
        );
    }

    /// The spellings of earlier versions give the options of the new spelling, with a note that
    /// says what to write now. Nothing in the schema names them.
    #[test]
    fn old_spellings_are_mapped_onto_the_new_ones() {
        use serde_json::json;
        let cases = [
            (
                json!({"pairs": true}),
                json!({"pairs": "band"}),
                "pairs: true is now pairs: \"band\"",
            ),
            (
                json!({"pairs": true, "breakouts": true, "pair_pull": 1}),
                json!({"pairs": "band", "breakouts": true, "pair_pull": 1}),
                "pairs: true is now pairs: \"band\"",
            ),
            (
                json!({"pair_pull": 2}),
                json!({"pairs": "pull", "pair_pull": 2}),
                "pair_pull: 2 is now pairs: \"pull\", pair_pull: 2",
            ),
            (
                json!({"pairs": false, "pair_pull": 1}),
                json!({"pairs": "pull"}),
                "pairs: false, pair_pull: 1 is now pairs: \"pull\", pair_pull: 1",
            ),
            (
                json!({"pairs": false}),
                json!({"pairs": "off"}),
                "pairs: false is now pairs: \"off\"",
            ),
            (
                json!({"keep_locked": true}),
                json!({"scope": "keep_locked"}),
                "keep_locked: true is now scope: \"keep_locked\"",
            ),
            (
                json!({"keep_all": true}),
                json!({"scope": "finish"}),
                "keep_all: true is now scope: \"finish\"",
            ),
            (
                json!({"keep_all": true, "nets": ["A"], "give": 2}),
                json!({"scope": "finish", "nets": ["A"], "give": 2}),
                "keep_all: true is now scope: \"finish\"",
            ),
            (
                json!({"keep_locked": true, "keep_all": true, "nets": ["A"], "reroute": true, "shove": false}),
                json!({"scope": "reroute", "nets": ["A"], "shove": false}),
                "keep_locked: true, keep_all: true, reroute: true is now scope: \"reroute\"",
            ),
            (
                json!({"keep_all": true, "reroute": false}),
                json!({"scope": "finish"}),
                "keep_all: true, reroute: false is now scope: \"finish\"",
            ),
        ];
        // what the router is told, which is what decides the result
        let told = |o: &RouteOptions| {
            let mut o = o.clone();
            o.pair_pull = Some(o.pair_pull());
            format!("{o:?}")
        };
        let with_board = |mut v: serde_json::Value| {
            v["board"] = json!("b1");
            v
        };
        for (old, new, note) in cases {
            let (old_opts, old_notes) = route_call(with_board(old.clone()));
            let (new_opts, new_notes) = route_call(with_board(new));
            assert_eq!(told(&old_opts), told(&new_opts), "{old}");
            assert_eq!(old_notes, [note], "{old}");
            assert!(new_notes.is_empty(), "{new_notes:?}");
            assert_eq!(old_opts.validate(false), Ok(()), "{old}");
        }
        // the schema knows the word only, and none of the arguments that went
        let schema = serde_json::to_value(schemars::schema_for!(RouteIn)).unwrap();
        assert_eq!(schema["properties"]["pairs"]["type"], json!("string"));
        // keep_locked and reroute are words of scope today; keep_all is nowhere
        assert!(!schema.to_string().contains("keep_all"));
        for old in OLD {
            assert!(schema["properties"].get(old).is_none(), "{old} is in the schema");
            // and each is still taken
            let v = json!({"board": "b1", "keep_all": true, "nets": ["A"], *old: true});
            assert!(serde_json::from_value::<RouteIn>(v).is_ok(), "{old}");
        }
        // in a session that keeps the locked copper, false takes it out and nothing leaves it in
        let session = |v: serde_json::Value| {
            let (opts, notes) = route_options(&serde_json::from_value::<RouteIn>(v).unwrap(), true).unwrap();
            (opts.scope, notes.iter().map(Respelled::mcp).collect::<Vec<_>>())
        };
        assert_eq!(
            session(json!({"board": "b1", "keep_locked": false})),
            (
                Scope::Fresh,
                vec!["keep_locked: false is now scope: \"fresh\"".to_string()]
            )
        );
        assert_eq!(
            session(json!({"board": "b1", "keep_all": false})),
            (
                Scope::KeepLocked,
                vec!["keep_all: false is now scope: \"keep_locked\"".to_string()]
            )
        );
        // what had no meaning is refused, as it was, and so is an old switch next to the word
        let refused = |v: serde_json::Value| {
            route_options(&serde_json::from_value::<RouteIn>(v).unwrap(), false)
                .unwrap_err()
                .to_string()
        };
        assert!(
            refused(json!({"board": "b1", "reroute": true, "nets": ["A"]}))
                .starts_with("reroute without keep_all has no meaning: write scope reroute with nets")
        );
        assert_eq!(
            refused(json!({"board": "b1", "scope": "finish", "keep_all": true})),
            "keep_all is the spelling of an earlier version and scope says the same: leave keep_all out"
        );
        let (opts, _) = route_call(json!({"board": "b1", "keep_all": true, "reroute": true}));
        assert!(
            opts.validate(false)
                .unwrap_err()
                .starts_with("scope reroute needs nets")
        );
    }
}
