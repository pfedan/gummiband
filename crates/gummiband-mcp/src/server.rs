//! MCP tools of the Gummiband server and the session state behind them.

use std::collections::{BTreeMap, HashMap};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

use anyhow::{Context, Result, bail};
use base64::Engine as _;
use gummiband_core::board::Board;
use gummiband_core::realize::Elem;
use gummiband_core::route::RouterConfig;
use gummiband_core::topo::SubstrateConfig;
use gummiband_kicad::{KeepCopper, KicadBoard, ReadOptions, read_board, write};
use rmcp::handler::server::router::tool::ToolRouter;
use rmcp::handler::server::tool::schema_for_output;
use rmcp::handler::server::wrapper::Parameters;
use rmcp::model::{CallToolResult, ContentBlock, Implementation, ServerCapabilities, ServerConfig};
use rmcp::schemars::{self, JsonSchema};
use rmcp::{ErrorData, ServerHandler, tool, tool_handler, tool_router};
use serde::{Deserialize, Serialize};

use crate::external;
use crate::practice;

const INSTRUCTIONS: &str = "Gummiband is a topological (any-angle, rubber band) autorouter for \
KiCad boards. From a schematic only: create_board turns a KiCad netlist into a board (footprints, nets, outline, stackup, net classes). Typical loop: load_board (gives a board id) -> list_nets -> optionally \
set_net_options (priority: lower routes first; skip: leave a net unrouted) -> route (writes a \
routed .kicad_pcb and returns stats, unrouted connections and clearance violations) -> drc on \
the written file (KiCad's own DRC) -> render a region (x0,y0,x1,y1 in mm, board coordinates) to \
look at problems. To pre-route a critical part by hand (datasheet layout): part_info -> \
add_locked_copper -> route the returned board id. move_part and add_zone edit the same working \
copy. To change one net on a routed board: load the routed file -> route with keep_all, nets and \
reroute (the unlocked tracks of other nets shift aside or give way where needed, see shove and give). To straighten a finished any-angle result to 45 degrees without routing again: angles_45 on the routed file. To place parts: move_part for connectors and power stages, place_parts for the rest (semantic: true keeps sheets and IC clusters together and current paths short), add_zone for planes. load_board lists the differential pairs it recognised; route couples them only with pair_pull or pairs. Good practice around routing (the prompt good_practice walks through a whole board): \
power_plan before routing (currents -> widths, planes, vias; apply stores the widths), \
current_check after routing (narrowest cross-section of every current path), ground_pour (GND \
pours and stitching vias), then drc. Coordinates are millimetres in KiCad's board coordinates (y grows downwards). \
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
    /// Problems found at the points of the new copper (only those: the track between two
    /// points is not checked): off board, touching other nets, on a plane layer.
    pub warnings: Vec<String>,
}

#[derive(Debug, Clone, Default, Serialize, JsonSchema)]
pub struct NetOption {
    /// Routing priority override (lower routes first).
    pub priority: Option<i32>,
    /// The net is left out of routing.
    pub skip: bool,
    /// Track width in mm for this net, also its minimum width (no necking below it outside the
    /// courtyards the custom rules name), from set_net_options or power_plan with apply.
    pub width: Option<f64>,
}

struct Entry {
    path: PathBuf,
    keep_locked: bool,
    kb: Arc<KicadBoard>,
    /// Per net options by net name (survive reloads).
    options: BTreeMap<String, NetOption>,
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

/// The board as the router sees it: net options applied (priorities, skipped nets have no
/// pins left, so they get no connections; their pads stay obstacles).
fn apply_options(b: &Board, options: &BTreeMap<String, NetOption>, only: &[String]) -> (Board, Vec<String>) {
    let mut b = b.clone();
    let mut skipped = Vec::new();
    for (name, o) in options {
        if let (Some(w), Some(id)) = (o.width, b.net_id(name)) {
            set_width(&mut b, id, w);
        }
    }
    for n in b.nets.iter_mut() {
        let o = options.get(&n.name);
        if let Some(p) = o.and_then(|o| o.priority) {
            n.priority = p;
        }
        let skip = o.is_some_and(|o| o.skip) || (!only.is_empty() && !only.contains(&n.name));
        if skip && n.pins.len() >= 2 {
            n.pins.clear();
            skipped.push(n.name.clone());
        }
    }
    (b, skipped)
}

/// Give a net its own class with track width and minimum width `w`.
fn set_width(b: &mut Board, net: gummiband_core::board::NetId, w: f64) {
    let ci = b.nets[net as usize].class as usize;
    let mut c = b.classes[ci].clone();
    c.name = format!("{}@{w}", c.name);
    c.track_width = w;
    c.min_width = Some(c.min_width.map_or(w, |m| m.max(w)));
    c.target_width = c.target_width.filter(|t| *t > w);
    // pads narrower than the track: the track may neck inside their part's courtyard
    for &pin in &b.nets[net as usize].pins {
        let p = &b.pins[pin];
        if p.size.0.min(p.size.1) < w && !c.neck_refs.contains(&p.footprint) {
            c.neck_refs.push(p.footprint.clone());
        }
    }
    let i = match b.classes.iter().position(|k| k.name == c.name) {
        Some(i) => i,
        None => {
            b.classes.push(c);
            b.classes.len() - 1
        }
    };
    b.nets[net as usize].class = i as gummiband_core::board::ClassId;
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
    /// Replace an existing output file (default false).
    pub overwrite: Option<bool>,
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
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct PartPose {
    pub reference: String,
    pub x: f64,
    pub y: f64,
    pub rotation: f64,
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
    /// they sat piled up or off the board.
    pub released: Vec<String>,
    /// Pads of a plane net without room for their fan-out via (kept copper, a hole or a rule
    /// area around them on some layer).
    pub no_via_room: Vec<String>,
    /// Released parts, parts that stay piled up or with a courtyard off the board, pads without
    /// room for their via.
    pub warnings: Vec<String>,
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
    /// Track width in mm for this net, also used as its minimum width (no necking); 0 clears it.
    pub width: Option<f64>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct RouteIn {
    /// Board id from load_board.
    pub board: String,
    /// Portfolio size: router runs with different orders (default 14, fixed so that results
    /// do not depend on the machine).
    pub runs: Option<usize>,
    /// Keep all tracks and vias of the input and add to them; with `nets` only those nets are
    /// routed and everything else stays.
    pub keep_all: Option<bool>,
    /// With `keep_all` and `nets`: remove the unlocked tracks and vias of those nets first and
    /// route them anew. Without it a net that is already connected stays as it is.
    pub reroute: Option<bool>,
    /// With `keep_all` and `nets`: the unlocked tracks of the other nets are part of the topology,
    /// so a connection passes between them and they shift aside; what it does not touch stays as
    /// it is in the file (default true; false keeps them where they are).
    pub shove: Option<bool>,
    /// With `keep_all` and `nets`: if a connection finds no way, the unlocked tracks of other nets
    /// within this many mm of its pins give way and are routed again, in growing steps up to this
    /// radius; the try with the fewest open connections is kept (default 8; 0: nothing gives
    /// way). Locked copper and plane nets never move.
    pub give: Option<f64>,
    /// Seed of the search. The same seed gives the same result; another one a different try.
    pub seed: Option<u64>,
    /// Weight of the pull between the nets of differential pairs (default 0: off; 1 is a good
    /// start, it can cost clearance on tight boards). The finished tracks then close up to the
    /// pair's gap where nothing lies between; `coupling` says where and why a pair stays apart.
    pub pair_pull: Option<f64>,
    /// Experimental: route differential pairs as one wide track and split it into two parallel
    /// tracks, width and gap from net classes and custom rules (default false). Check the result
    /// with drc; on dense boards it costs connections.
    pub pairs: Option<bool>,
    /// With pairs: length difference left within a pair in mm (default 0.1).
    pub pair_skew: Option<f64>,
    /// With pairs: target differential impedance in ohm; sets the track width per layer from
    /// the stackup, the gap stays.
    pub pair_zdiff: Option<f64>,
    /// With pairs: couple every pair recognised by name (default false: only pairs with a
    /// `diff_pair_gap` rule or a net class of their own).
    pub pair_all: Option<bool>,
    /// With pairs, experimental: the wide track ends just outside the pads, two single tracks
    /// connect them (default false).
    pub pair_breakouts: Option<bool>,
    /// With pairs, experimental: two vias side by side are one via pair, a capsule that turns
    /// with the wide track (default false).
    pub pair_via_pairs: Option<bool>,
    /// Weight of the preferred layer directions in every third run (default 0.5, 0: off).
    pub layer_dirs: Option<f64>,
    /// Hints from the local patterns of board_patterns, comma separated: shares, decaps, rows,
    /// all or none (default none), as `gummiband route --patterns`.
    pub patterns: Option<String>,
    /// Generations of the evolutionary order search after the portfolio (default 3).
    pub generations: Option<usize>,
    /// Generations that select by clearance violations after the geometry step (default 3, as
    /// the CLI and the plugin; 0 is about three times faster and leaves many more violations).
    pub geometry_generations: Option<usize>,
    /// Keep locked tracks and vias as fixed copper (default: as given to load_board).
    pub keep_locked: Option<bool>,
    /// Let parts next to clearance problems and open connections shift a little while the
    /// routed copper follows like rubber bands (default true without keep_all, false with it).
    /// Locked footprints, connectors, modules, mounting holes, buttons and test points stay, as
    /// do the references in `fixed`. The output has the footprints at their new positions, and
    /// `moved` lists them.
    pub move_parts: Option<bool>,
    /// References that never move under move_parts.
    #[serde(default)]
    pub fixed: Vec<String>,
    /// Output .kicad_pcb (default: `<input>.routed.kicad_pcb` next to the input). The
    /// .kicad_pro and .kicad_dru are copied next to it if missing.
    pub output: Option<String>,
    /// Route only these nets (all others are skipped for this call).
    pub nets: Option<Vec<String>>,
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
    /// Open end: after the normal search keep improving for this many seconds, then finish the
    /// best result (default 0: off), as `--until-stop` with a time budget instead of a stop line.
    /// Mutants take a few connections around one spot out and lay them again; when that stops
    /// helping, bolder side branches take over. No placement rounds while it runs.
    pub open_end_seconds: Option<f64>,
    /// With open_end_seconds: what a via is worth in mm of track when the open end judges its
    /// results (default 20). Lower gives more vias and shorter tracks, higher fewer vias and
    /// longer ones.
    pub open_via_cost: Option<f64>,
    /// Reserve in mm on top of the clearances that the finished copper gets where there is room
    /// (default 0). The result stays only if it brings no new violation.
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
    /// Maximum number of violations listed (default 50; the total is always given).
    pub max_violations: Option<usize>,
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
    /// them only with `pair_pull` or `pairs`.
    pub diff_pairs: Vec<DiffPairOut>,
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
    /// With `pair_pull` or `pairs`: the longest stretches (0.5 mm and more, at most 5) that do
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
    pub seconds: f64,
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
    /// Gummiband's own clearance check on the result.
    pub violations_total: usize,
    pub violations_by_kind: BTreeMap<String, usize>,
    /// The first max_violations violations.
    pub violations: Vec<ViolationOut>,
    pub portfolio: Vec<PortfolioOut>,
    /// Differential pairs, with `pairs` set.
    pub pairs: Vec<PairOut>,
    /// Every recognised differential pair, whatever the settings: how much of it runs next to
    /// its partner.
    pub coupling: Vec<CouplingOut>,
    /// Hints on the settings, e.g. pairs that were not asked to couple.
    pub notes: Vec<String>,
    /// Per routed net: track length, vias and widths, longest first.
    pub nets: Vec<NetStatOut>,
    /// Project files copied next to the output.
    pub copied: Vec<String>,
    /// What the 45 degree step did, with `angles` "45".
    pub angles_45: Option<AnglesOut>,
    /// The length tuning: every net it aimed at.
    pub length_tuning: Vec<TunedOut>,
    /// The length, skew, uncoupled and gap rules of the .kicad_dru after the tuning.
    pub length_rules: Vec<LengthRuleOut>,
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
            outside: rep.overflow.clone(),
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
            if out.outside.is_empty() {
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
            net_options: options.clone(),
            diff_pairs: b
                .diff_pairs()
                .into_iter()
                .map(|(p, n)| DiffPairOut {
                    p: b.nets[p as usize].name.clone(),
                    n: b.nets[n as usize].name.clone(),
                    class: b.class_of(p).name.clone(),
                })
                .collect(),
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
        ) + &pairs_note(&out.diff_pairs);
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
        let foreign = |at: gummiband_core::geom::Point, r: f64, layer: Option<u8>, net: u32| -> Option<String> {
            b.obstacles
                .iter()
                .find(|o| {
                    o.net.is_some_and(|n| n != net)
                        && layer.is_none_or(|l| o.layers.has(l))
                        && o.shape.dist_to_point(at) < r
                })
                .and_then(|o| o.net)
                .map(|n| b.nets[n as usize].name.clone())
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
                if let Some(o) = foreign(at, width / 2.0, Some(layer), net) {
                    warnings.push(format!("track of {} touches {o} at {}, {}", t.net, q[0], q[1]));
                }
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
            if let Some(o) = foreign(at, v.diameter.unwrap_or(c.via_diameter) / 2.0, None, net) {
                warnings.push(format!("via of {} touches {o} at {}, {}", v.net, v.x, v.y));
            }
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

    fn do_move_part(&self, p: MovePartIn) -> Result<(String, AddLockedOut)> {
        let follow = p.follow.unwrap_or(true) && p.rotation.is_none();
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
                return write::place_footprint(text, &p.reference, p.x, p.y, p.rotation);
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
        let msg = match &followed {
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
            ..defaults
        };
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
                net,
                layer,
                priority: p.priority.unwrap_or(0),
                name: "",
                connect,
                spoke,
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
                NetOption::default()
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
            let priority = o.priority.unwrap_or(b.nets[id as usize].priority);
            if o.priority.is_none() && !o.skip && o.width.is_none() {
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
            ) + &o.width.map(|w| format!(" Width {w} mm.")).unwrap_or_default();
            Ok((
                summary,
                SetNetOptionsOut {
                    board: p.board.clone(),
                    net: p.net.clone(),
                    options: o,
                    priority,
                },
            ))
        })
    }
}

/// The nets whose copper goes before routing.
fn reroute_nets(p: &RouteIn) -> Vec<String> {
    match p.reroute {
        Some(true) => p.nets.clone().unwrap_or_default(),
        _ => Vec::new(),
    }
}

fn route_blocking(
    id: String,
    path: PathBuf,
    kb: Arc<KicadBoard>,
    options: BTreeMap<String, NetOption>,
    keep_locked: bool,
    p: RouteIn,
) -> Result<(String, RouteOut)> {
    let output = match &p.output {
        Some(o) => abs_path(o),
        None => {
            let stem = path.file_stem().and_then(|s| s.to_str()).unwrap_or("board");
            path.with_file_name(format!("{stem}.routed.kicad_pcb"))
        }
    };
    if output == path {
        bail!("output must differ from the input file");
    }
    let only = p.nets.clone().unwrap_or_default();
    for n in &only {
        if kb.board.net_id(n).is_none() {
            bail!("no net named {n:?} (see list_nets)");
        }
    }
    // finishing nets on a routed board: the unlocked tracks of the other nets shift aside and, if
    // that is not enough, give way
    let finishing = p.keep_all == Some(true) && !only.is_empty();
    if !finishing && (p.shove == Some(true) || p.give.is_some_and(|g| g > 0.0)) {
        bail!("shove and give need keep_all and nets");
    }
    let shove = finishing && p.shove != Some(false);
    let give = if finishing {
        p.give.unwrap_or(gummiband_core::DEFAULT_GIVE)
    } else {
        0.0
    };
    // the nets left out of this call, for the result
    let (_, skipped) = apply_options(&kb.board, &options, &only);
    let (mut board, _) = apply_options(&kb.board, &options, &[]);
    // parts move by default when the whole board is routed
    let move_parts = p.move_parts.unwrap_or(p.keep_all != Some(true));
    if move_parts {
        if finishing && shove {
            bail!("move_parts does not work with shove: set shove false, or leave move_parts out");
        }
        let movable = read_board(
            &path,
            &ReadOptions {
                keep: if keep_locked {
                    KeepCopper::Locked
                } else {
                    KeepCopper::None
                },
                move_parts: true,
                fixed: p.fixed.clone(),
                ..Default::default()
            },
        )?;
        board.movable = movable.board.movable;
    }
    let runs = p.runs.filter(|&r| r > 0).unwrap_or(gummiband_core::DEFAULT_RUNS);
    let t0 = std::time::Instant::now();
    if let (Some(z), Some(true)) = (p.pair_zdiff, p.pairs) {
        gummiband_core::pairs::apply_zdiff(&mut board, z);
    }
    let angles_45 = match p.angles.as_deref() {
        None | Some("any") => false,
        Some("45") => true,
        Some(other) => bail!("angles must be \"any\" or \"45\", not {other:?}"),
    };
    let sub_cfg = SubstrateConfig {
        via_pitch: p.via_pitch.unwrap_or(0.5),
        ..Default::default()
    };
    let hints = gummiband_core::patterns::hints(p.patterns.as_deref().unwrap_or("none")).map_err(anyhow::Error::msg)?;
    let router_cfg = RouterConfig {
        via_cost: p.via_cost.unwrap_or(3.0),
        pair_pull: p.pair_pull.unwrap_or(0.0),
        layer_dirs: p.layer_dirs.unwrap_or(0.5),
        negotiate: p.negotiate.unwrap_or(0),
        via_pass: p.via_pass.unwrap_or(4.0),
        ..Default::default()
    }
    .with_hints(&hints);
    // the open end of the CLI stops on a line on stdin; here a time budget sets the stop flag
    let open_end = p.open_end_seconds.filter(|&s| s > 0.0).map(|secs| {
        let flag = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let set = flag.clone();
        std::thread::spawn(move || {
            std::thread::sleep(std::time::Duration::from_secs_f64(secs));
            set.store(true, std::sync::atomic::Ordering::Relaxed);
        });
        flag
    });
    let search_cfg = gummiband_core::SearchConfig {
        population: runs,
        generations: p.generations.unwrap_or(3),
        geometry_generations: p.geometry_generations.unwrap_or(3),
        frames: false,
        progress: None,
        seed: p.seed,
        open_end,
        air: p.air.unwrap_or(0.0),
        prefer_width: p.prefer_width.unwrap_or(0.0),
        open_via_cost: p.open_via_cost.unwrap_or(gummiband_core::DEFAULT_OPEN_VIA_COST),
    };
    let tune_cfg = gummiband_core::tune::TuneConfig {
        style: if p.angles.as_deref() == Some("45") {
            gummiband_core::tune::Style::Chamfer
        } else {
            gummiband_core::tune::Style::Round
        },
        amplitude: p.tune_amplitude.unwrap_or(1.0),
        spacing: p.tune_spacing.unwrap_or(2.0),
    };
    let tune = p.tune != Some(false);
    let finished = gummiband_core::finish_nets(&board, &only, shove, give, &search_cfg, |b, search| {
        if p.pairs == Some(true) {
            let cfg = gummiband_core::pairs::PairConfig {
                skew: p.pair_skew.unwrap_or(0.1),
                zdiff: p.pair_zdiff,
                all: p.pair_all.unwrap_or(false),
                room: tune.then(|| tune_cfg.clone()),
                angles45: angles_45,
                breakouts: p.pair_breakouts.unwrap_or(false),
                via_pairs: p.pair_via_pairs.unwrap_or(false),
            };
            gummiband_core::pairs::route_pairs(b, &sub_cfg, &router_cfg, search, &cfg)
        } else {
            let (sub, res) = gummiband_core::route_board(b, &sub_cfg, &router_cfg, search);
            (sub, res, Vec::new())
        }
    });
    let gummiband_core::Finished {
        board: routed,
        sub,
        res,
        extra: pair_reports,
        given,
        ..
    } = finished;
    // the copper was routed on the board with the parts where they ended up
    let shifted;
    let b = if res.offsets.is_empty() {
        &routed
    } else {
        shifted = routed.with_offsets(&res.offsets);
        &shifted
    };
    let mut res = res;
    // room for the meanders (the pairs make it while they are one wide track)
    if tune && p.pairs != Some(true) && !b.match_rules.is_empty() {
        let want = gummiband_core::tune::room_wanted(b, &res.geometry, &tune_cfg, &[]);
        if let Some(g) = gummiband_core::tune::make_room(b, &sub, &res.geometry, &want) {
            res.geometry = g;
        }
    }
    let angles_45 = angles_45.then(|| {
        let wide: Vec<gummiband_core::board::NetId> = pair_reports
            .iter()
            .filter(|r| r.wide)
            .flat_map(|r| [b.net_id(&r.p), b.net_id(&r.n)])
            .flatten()
            .collect();
        let st = gummiband_core::octo::octilinear_only(
            b,
            &mut res.geometry,
            &|t| !(wide.contains(&t.net) && t.seg == u32::MAX),
            &|_, _| false,
        );
        AnglesOut {
            length_before_mm: r3(st.length_before),
            length_after_mm: r3(st.length_after),
            off_grid_pieces: st.any_angle,
            off_grid_mm: r3(st.any_angle_length),
            sharp_corners: st.sharp_corners,
        }
    });
    let tuned =
        (tune && !b.match_rules.is_empty()).then(|| gummiband_core::tune::tune(b, &mut res.geometry, &tune_cfg));
    let seconds = t0.elapsed().as_secs_f64();
    let viol = gummiband_core::check::check(b, &res.geometry);

    // with shove: what rests of the other nets stays in the file as it is
    let changes = gummiband_core::import::soft_changes(b, &res.geometry);
    let rests = |origin: Option<u32>, list: &[bool]| origin.is_some_and(|o| !list[o as usize]);
    let mut items = Vec::new();
    let mut of_track: Vec<std::ops::Range<usize>> = Vec::with_capacity(res.geometry.tracks.len());
    for t in &res.geometry.tracks {
        let start = items.len();
        if rests(t.origin, &changes.chains) {
            of_track.push(start..start);
            continue;
        }
        for e in &t.elems {
            items.push(match *e {
                Elem::Line(a, bb) => write::Item::Track {
                    width: t.piece_width(a, bb),
                    a,
                    b: bb,
                    layer: t.layer,
                    net: t.net,
                },
                Elem::Arc(a, mid, bb) => write::Item::Arc {
                    width: t.piece_width(a, bb),
                    a,
                    mid,
                    b: bb,
                    layer: t.layer,
                    net: t.net,
                },
            });
        }
        of_track.push(start..items.len());
    }
    for pat in tuned.iter().flat_map(|r| &r.patterns) {
        let members: Vec<usize> = pat
            .tracks
            .iter()
            .filter_map(|&i| of_track.get(i))
            .flat_map(|r| r.clone())
            .collect();
        if members.is_empty() {
            continue;
        }
        items.push(write::Item::Pattern(Box::new(write::PatternItem {
            layer: pat.layer,
            net: pat.nets[0],
            base: pat.base[0],
            base_coupled: pat.base.get(1).copied(),
            rounded: pat.rounded,
            width: pat.width,
            gap: pat.gap,
            amplitude: pat.amplitude,
            spacing: pat.spacing,
            target: pat.aim,
            length: gummiband_core::matching::net_length(b, &res.geometry, pat.nets[0]).total(),
            members,
        })));
    }
    for v in &res.geometry.vias {
        if v.drill == 0.0 || rests(v.origin, &changes.nodes) {
            continue;
        }
        items.push(write::Item::Via {
            at: v.at,
            diameter: v.diameter,
            drill: v.drill,
            net: v.net,
        });
    }
    let keep_all = p.keep_all == Some(true);
    let mut strip_nets = reroute_nets(&p);
    strip_nets.extend(given.iter().cloned());
    let opts = write::StripOptions {
        unlocked: !keep_all,
        locked: !keep_locked && !keep_all,
        zone_layers: vec![],
        nets: strip_nets,
        items: changes.strip.clone(),
    };
    let ranges = write::strip_ranges(&kb, &opts);
    let text = write::patch(&kb, &ranges, &items);
    let text = if res.offsets.is_empty() {
        text
    } else {
        write::move_footprints(&text, &res.offsets, &kb.board.blocks)?
    };
    std::fs::write(&output, text).with_context(|| format!("writing {}", output.display()))?;
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
    let copied = external::copy_project_files(&path, &output);

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
    for v in &viol {
        *by_kind
            .entry(v.what.split(' ').next().unwrap_or("").to_string())
            .or_insert(0) += 1;
    }
    let mut coupling = Vec::new();
    for (a, c) in b.diff_pairs() {
        let (w, g, _) = b.pair_shape(a, c, 0);
        let (coupled, total) = gummiband_core::pair_coupling_of(&res.geometry, a, c, w + g + 0.1);
        let vias = |n| res.geometry.vias.iter().filter(|v| v.net == n).count();
        let asked = p.pair_pull.unwrap_or(0.0) > 0.0 || p.pairs == Some(true);
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
    let mut notes = Vec::new();
    for x in coupling
        .iter()
        .filter(|x| !x.uncoupled.is_empty() && x.coupled_mm < 0.6 * x.length_mm)
    {
        notes.push(format!(
            "differential pair {}/{} runs coupled for only {:.1} of {:.1} mm: coupling lists the longest uncoupled stretches with where and why",
            x.p, x.n, x.coupled_mm, x.length_mm
        ));
    }
    if !coupling.is_empty() && p.pair_pull.unwrap_or(0.0) == 0.0 && p.pairs != Some(true) {
        let names: Vec<String> = coupling.iter().map(|x| format!("{}/{}", x.p, x.n)).collect();
        notes.push(format!(
            "differential pairs {} were routed as independent nets: set pair_pull (1 to 4) or pairs to couple them",
            names.join(", ")
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
    let length_rules: Vec<LengthRuleOut> = gummiband_core::matching::check(b, &res.geometry)
        .into_iter()
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
                rule: r.rule,
            }
        })
        .collect();
    let st = &res.stats;
    let out = RouteOut {
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
        seconds,
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
        items_written: items.len(),
        skipped_nets: skipped,
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
        copied: copied.iter().map(|p| p.display().to_string()).collect(),
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
        items.len(),
        output.display(),
        if out.skipped_nets.is_empty() {
            String::new()
        } else {
            format!(" Skipped nets: {}.", out.skipped_nets.len())
        }
    ) + &out.notes.iter().map(|n| format!(" Note: {n}.")).collect::<String>();
    Ok((summary, out))
}

/// Sentence on the recognised pairs for the summaries, empty without pairs.
fn pairs_note(pairs: &[DiffPairOut]) -> String {
    if pairs.is_empty() {
        return String::new();
    }
    let names: Vec<String> = pairs.iter().map(|x| format!("{}/{}", x.p, x.n)).collect();
    format!(
        " Differential pairs: {}; route couples them only with pair_pull or pairs.",
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
        description = "Give parts a position from the netlist in the working copy <input>.prerouted.kicad_pcb; the input file stays untouched. Use the returned board id from now on. By default all parts move except locked ones, connectors, modules, mounting holes, buttons and test points (those move too while they sit piled up or off the board, listed in `released`); `only`, `scope` and `fixed` narrow that, so you can place the key parts yourself with move_part and leave the small parts to this tool. Run it before route: tracks do not follow. Read `warnings`: parts that stay piled up or off the board, pads of a plane net without room for their via (`no_via_room`).",
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
        description = "Move a part to a new position (and rotation) in the working copy <input>.prerouted.kicad_pcb; the input file stays untouched. Use the returned board id from now on. Without rotation the tracks around it follow like rubber bands (follow: false switches that off): its own tracks stretch, other nets give way within their clearances, and the part stops short where nothing can give way any more (a warning says where). `warnings` also names courtyards it now overlaps (by their outlines; drc has the exact answer).",
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
        description = "Add a copper zone (net, layer, polygon, priority) to the working copy. On a KiCad power layer it becomes a plane the router connects to with vias. KiCad fills it; drc refills zones.",
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
        description = "Add pre-routed copper (tracks as polylines, vias) as locked items: the router keeps it and routes around it, and track ends join their net. The input file stays untouched: the copper goes into a working copy <input>.prerouted.kicad_pcb, which is loaded with keep_locked; use the returned board id from now on. Check the result with drc and render.",
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
        description = "Set per net options in the session, applied on the next route: priority (lower routes first, overrides the net class priority) and skip (leave the net unrouted). reset=true clears the net's options first.",
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
        description = "Route a loaded board with Gummiband (topological any-angle router, portfolio plus evolutionary order search on all routing threads) and write the routed .kicad_pcb (default <input>.routed.kicad_pcb next to the input; tracks and vias of the input are replaced). Returns routed/total connections, unrouted connections, vias, track length, Gummiband's clearance violations with nets and positions, and the portfolio. CPU heavy: seconds to minutes; one route at a time. Check the result with drc.",
        output_schema = schema_for_output::<RouteOut>(),
        annotations(read_only_hint = false, destructive_hint = true, idempotent_hint = true, open_world_hint = false)
    )]
    async fn route(&self, Parameters(p): Parameters<RouteIn>) -> Result<CallToolResult, ErrorData> {
        let _guard = self.route_lock.lock().await;
        let keep =
            |e: &mut Entry| -> Result<_> { Ok((e.path.clone(), e.kb.clone(), e.options.clone(), e.keep_locked)) };
        let (path, kb, options, loaded_keep) = match self.with_entry(&p.board, keep) {
            Ok(x) => x,
            Err(e) => return fail(format!("{e:#}")),
        };
        let keep_locked = p.keep_locked.unwrap_or(loaded_keep);
        let id = p.board.clone();
        let res = blocking(move || {
            if p.reroute == Some(true) && (p.keep_all != Some(true) || p.nets.as_ref().is_none_or(Vec::is_empty)) {
                bail!("reroute needs keep_all and nets");
            }
            // the copper to keep changes the board model (locked track ends become pins)
            let kb = if p.keep_all == Some(true) {
                Arc::new(read_board(
                    &path,
                    &ReadOptions {
                        keep: KeepCopper::All,
                        ignore_nets: reroute_nets(&p),
                        ..Default::default()
                    },
                )?)
            } else if keep_locked == loaded_keep {
                kb
            } else {
                Arc::new(read(&path, keep_locked)?)
            };
            route_blocking(id, path, kb, options, keep_locked, p)
        })
        .await;
        match res {
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
                e.options.entry(net.clone()).or_default().width = Some(*w);
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
        description = "Lay out power copper before routing, for current paths that a track carries badly (a converter's switch node, a pad into a plane through one via): for a net without plane one pour over its from and to pads on their layer (convex hull grown by margin, cut back from other nets' copper, holes and the edge), for a plane net at each terminal a field of locked vias for the current (IPC-2221 per via) and a small pour joining them to the pad. The pours are zones named \"gb: ...\": the router keeps other nets out and treats the pads under one pour as connected. Writes the working copy <input>.prerouted.kicad_pcb; route the returned board id. Reports the pours, what cut them and a current_check of the pours.",
        output_schema = schema_for_output::<practice::PowerCopperOut>(),
        annotations(read_only_hint = false, destructive_hint = false, idempotent_hint = false, open_world_hint = false)
    )]
    async fn power_copper(
        &self,
        Parameters(p): Parameters<practice::PowerCopperIn>,
    ) -> Result<CallToolResult, ErrorData> {
        let me = self.clone();
        let res = blocking(move || {
            let mut out = None;
            let (id, path) = me.edit_copy(&p.board, |kb, _| {
                let (text, o) = practice::power_copper(kb, &p)?;
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
        })
        .await;
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
        description = "Pour ground after routing: adds zones of the ground net (default GND) over the board outline on the outer layers (default) and locked stitching vias on a grid and along the board edge where they are clear of other nets, holes, rule areas, other nets' pours and courtyards. Writes <input>.gnd.kicad_pcb and loads it; KiCad fills the zones (drc refills them).",
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
                let s = format!(
                    "{} pours on {} and {} stitching vias ({} along the edge) in {} (board {}). {}",
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
                    out.warnings.join(" ")
                );
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
