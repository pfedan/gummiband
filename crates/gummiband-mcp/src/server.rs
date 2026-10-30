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

const INSTRUCTIONS: &str = "Gummiband is a topological (any-angle, rubber band) autorouter for \
KiCad boards. Typical loop: load_board (gives a board id) -> list_nets -> optionally \
set_net_options (priority: lower routes first; skip: leave a net unrouted) -> route (writes a \
routed .kicad_pcb and returns stats, unrouted connections and clearance violations) -> drc on \
the written file (KiCad's own DRC) -> render a region (x0,y0,x1,y1 in mm, board coordinates) to \
look at problems. To pre-route a critical part by hand (datasheet layout): part_info -> \
add_locked_copper -> route the returned board id. move_part and add_zone edit the same working \
copy. To change one net on a routed board: load the routed file -> route with keep_all, nets and \
reroute. Coordinates are millimetres in KiCad's board coordinates (y grows downwards). \
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
    /// Courtyard box [x0, y0, x1, y1] in mm, if the footprint has one.
    pub courtyard: Option<[f64; 4]>,
    pub pads: Vec<PadOut>,
    /// Footprints whose courtyard is within 3 mm.
    pub neighbours: Vec<String>,
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

fn connections(b: &Board) -> usize {
    b.routable_nets().map(|n| b.nets[n as usize].pins.len() - 1).sum()
}

fn is_plane_net(b: &Board, net: usize) -> bool {
    !b.nets[net].planes.is_empty() || b.planes.iter().any(|p| p.net as usize == net)
}

// --------------------------------------------------------------------------- parameters

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
    /// Seed of the search. The same seed gives the same result; another one a different try.
    pub seed: Option<u64>,
    /// Weight of the pull between the nets of differential pairs (default 0: off; 1 is a good
    /// start, it can cost clearance on tight boards).
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
    /// Weight of the preferred layer directions in every third run (default 0.5, 0: off).
    pub layer_dirs: Option<f64>,
    /// Generations of the evolutionary order search after the portfolio (default 3).
    pub generations: Option<usize>,
    /// Generations that select by clearance violations after the geometry step (default 0,
    /// slower).
    pub geometry_generations: Option<usize>,
    /// Keep locked tracks and vias as fixed copper (default: as given to load_board).
    pub keep_locked: Option<bool>,
    /// Output .kicad_pcb (default: `<input>.routed.kicad_pcb` next to the input). The
    /// .kicad_pro and .kicad_dru are copied next to it if missing.
    pub output: Option<String>,
    /// Route only these nets (all others are skipped for this call).
    pub nets: Option<Vec<String>>,
    /// Via site grid pitch in mm (default 0.5).
    pub via_pitch: Option<f64>,
    /// Via cost in mm of track (default 3, as the CLI).
    pub via_cost: Option<f64>,
    /// Maximum number of violations listed (default 50; the total is always given).
    pub max_violations: Option<usize>,
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
    /// Per routed net: track length, vias and widths, longest first.
    pub nets: Vec<NetStatOut>,
    /// Project files copied next to the output.
    pub copied: Vec<String>,
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
        );
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
            let out = PartInfoOut {
                reference: p.reference.clone(),
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
        let (id, path) = self.edit_copy(&p.board, |_, text| {
            write::place_footprint(text, &p.reference, p.x, p.y, p.rotation)
        })?;
        let out = AddLockedOut {
            board: id,
            path,
            items_added: 1,
            warnings: Vec::new(),
        };
        Ok((format!("moved {} to {}, {}", p.reference, p.x, p.y), out))
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
            Ok(write::add_zone(kb, text, net, layer, &pts, p.priority.unwrap_or(0), 1))
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
            let priority = o.priority.unwrap_or(b.nets[id as usize].priority);
            if o.priority.is_none() && !o.skip {
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
            );
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
    let (board, skipped) = apply_options(&kb.board, &options, &only);
    let b = &board;
    let runs = p.runs.filter(|&r| r > 0).unwrap_or(gummiband_core::DEFAULT_RUNS);
    let t0 = std::time::Instant::now();
    let tuned;
    let b = match p.pair_zdiff {
        Some(z) if p.pairs == Some(true) => {
            let mut c = b.clone();
            gummiband_core::pairs::apply_zdiff(&mut c, z);
            tuned = c;
            &tuned
        }
        _ => b,
    };
    let sub_cfg = SubstrateConfig {
        via_pitch: p.via_pitch.unwrap_or(0.5),
        ..Default::default()
    };
    let router_cfg = RouterConfig {
        via_cost: p.via_cost.unwrap_or(3.0),
        pair_pull: p.pair_pull.unwrap_or(0.0),
        layer_dirs: p.layer_dirs.unwrap_or(0.5),
        ..Default::default()
    };
    let search_cfg = gummiband_core::SearchConfig {
        population: runs,
        generations: p.generations.unwrap_or(3),
        geometry_generations: p.geometry_generations.unwrap_or(0),
        frames: false,
        progress: None,
        seed: p.seed,
    };
    let (res, pair_reports) = if p.pairs == Some(true) {
        let cfg = gummiband_core::pairs::PairConfig {
            skew: p.pair_skew.unwrap_or(0.1),
            zdiff: p.pair_zdiff,
        };
        let (_, res, reports) = gummiband_core::pairs::route_pairs(b, &sub_cfg, &router_cfg, &search_cfg, &cfg);
        (res, reports)
    } else {
        (
            gummiband_core::route_board(b, &sub_cfg, &router_cfg, &search_cfg).1,
            Vec::new(),
        )
    };
    let seconds = t0.elapsed().as_secs_f64();
    let viol = gummiband_core::check::check(b, &res.geometry);

    let mut items = Vec::new();
    for t in &res.geometry.tracks {
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
    }
    for v in &res.geometry.vias {
        items.push(write::Item::Via {
            at: v.at,
            diameter: v.diameter,
            drill: v.drill,
            net: v.net,
        });
    }
    let keep_all = p.keep_all == Some(true);
    let opts = write::StripOptions {
        unlocked: !keep_all,
        locked: !keep_locked && !keep_all,
        zone_layers: vec![],
        nets: reroute_nets(&p),
        items: Vec::new(),
    };
    let ranges = write::strip_ranges(&kb, &opts);
    std::fs::write(&output, write::patch(&kb, &ranges, &items))
        .with_context(|| format!("writing {}", output.display()))?;
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
    let st = &res.stats;
    let out = RouteOut {
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
    );
    Ok((summary, out))
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
    let png = match &p.output {
        Some(o) => abs_path(o),
        None => {
            let stem = path.file_stem().and_then(|s| s.to_str()).unwrap_or("board");
            external::temp_file(&format!("{stem}-render"), "png")?
        }
    };
    let r = external::render(
        &path,
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
        "Rendered {} ({}) region x {:.2}..{:.2}, y {:.2}..{:.2} mm at {}x{} px ({:.1} px/mm) to {}.",
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
        description = "Move a part to a new position (and rotation) in the working copy <input>.prerouted.kicad_pcb; the input file stays untouched. Use the returned board id from now on. Courtyards are not checked; use drc.",
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
        ServerConfig::new(ServerCapabilities::builder().enable_tools().build())
            .with_server_info(
                Implementation::new("gummiband-mcp", env!("CARGO_PKG_VERSION")).with_title("Gummiband autorouter"),
            )
            .with_instructions(INSTRUCTIONS)
    }
}
