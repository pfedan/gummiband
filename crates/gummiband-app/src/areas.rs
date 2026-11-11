//! Areas: what the copper fill of a net with a zone (a pour on an outer layer, a plane on an
//! inner one) reaches on the copper a board file has, as a report.
//!
//! An area is one model: the zone outlines of a net with the settings of their fill. The fill is
//! never stored. It is computed by `gummiband_core::fill` from the board and its copper each
//! time it is asked for, and this module puts the answer into words: the pieces of fill, the
//! islands with their pads, the pads the fill does not reach and why, the site of one via per
//! island, how far the pads are from a spot where a via lands in the area, and the pads whose
//! answer hangs on less than a raster cell.
//!
//! `gummiband areas` and the MCP tool `area_report` both call [`area_report`]; their options are
//! the table [`AREA_OPTIONS`].
//!
//! The model is the router's own. KiCad's filler has the last word, and at strips of about the
//! minimum width the two have not been compared yet (`bench/areas-kicad.sh`).

use std::collections::BTreeMap;
use std::path::Path;

use anyhow::{Context, Result, bail};
use gummiband_core::board::{Board, LayerId, LayerSet, NetId, Obstacle, ObstacleKind};
use gummiband_core::fill::{self, Area, Connect, NetFill, NoVia, Reach, edt, pieces_of};
use gummiband_core::geom::{Point, Polygon, Shape, pt};
use gummiband_core::region;
use gummiband_kicad::{KeepCopper, KicadBoard, ReadOptions, read_board};
use serde::Serialize;

use crate::options::{Kind, OptionDoc, opt};

/// The options of an area report. [`AREA_OPTIONS`] describes them.
#[derive(Debug, Clone, PartialEq)]
pub struct AreaOptions {
    pub nets: Vec<String>,
    pub pour: Vec<String>,
    pub zone_clearance: f64,
    pub zone_min_width: f64,
    pub thermal_gap: f64,
    pub thermal_spoke_width: f64,
    pub pad_connection: String,
    pub plane_layers: Vec<String>,
    pub step: f64,
    pub doubtful: bool,
}

impl Default for AreaOptions {
    fn default() -> Self {
        let p = Area::pour(0, 0, Vec::new());
        AreaOptions {
            nets: Vec::new(),
            pour: Vec::new(),
            zone_clearance: p.clearance,
            zone_min_width: p.min_width,
            thermal_gap: p.thermal_gap,
            thermal_spoke_width: p.spoke_width,
            pad_connection: "thermal".into(),
            plane_layers: Vec::new(),
            step: fill::Options::default().step,
            doubtful: true,
        }
    }
}

/// Every option of an area report, in the order of [`AreaOptions`].
pub const AREA_OPTIONS: &[OptionDoc] = &[
    opt("nets", Kind::List, "every net with a zone", "Report only these nets."),
    opt(
        "pour",
        Kind::List,
        "none",
        "Layers on which the nets given get an area for this report where they have none yet, as ground_pour would lay it: one net over the board outline, several nets each around its own pads.",
    ),
    opt(
        "zone_clearance",
        Kind::Number,
        "0.2",
        "Clearance in mm of the areas of pour to copper of other nets.",
    ),
    opt(
        "zone_min_width",
        Kind::Number,
        "0.2",
        "Minimum width in mm of the fill in the areas of pour.",
    ),
    opt(
        "thermal_gap",
        Kind::Number,
        "0.3",
        "Gap in mm of the thermal reliefs in the areas of pour.",
    ),
    opt(
        "thermal_spoke_width",
        Kind::Number,
        "0.3",
        "Width in mm of the thermal spokes in the areas of pour.",
    ),
    opt(
        "pad_connection",
        Kind::Text,
        "thermal",
        "How the pads of the net join the areas of pour: thermal, solid, thermal_tht or none.",
    ),
    opt(
        "plane_layers",
        Kind::List,
        "none",
        "Copper layers treated as planes besides those of KiCad type \"power\".",
    ),
    opt(
        "step",
        Kind::Number,
        "0.05",
        "Raster step wanted in mm; the step used divides the minimum width into an odd number of cells.",
    ),
    opt(
        "doubtful",
        Kind::OnOff,
        "on",
        "List the pads whose answer changes when every clearance moves by half a raster cell, which takes two more runs of the model per net.",
    ),
];

/// The entry of an option of the area report.
pub fn area_option_doc(name: &str) -> Option<&'static OptionDoc> {
    AREA_OPTIONS.iter().find(|o| o.name == name)
}

/// `pad_connection` in words to the model's kind.
pub fn parse_connect(s: &str) -> Result<Connect> {
    Ok(match s {
        "thermal" => Connect::Thermal,
        "solid" => Connect::Solid,
        "thermal_tht" => Connect::ThermalTht,
        "none" => Connect::None,
        o => bail!("pad_connection {o:?}: use thermal, solid, thermal_tht or none"),
    })
}

fn r3(x: f64) -> f64 {
    (x * 1000.0).round() / 1000.0
}

fn xy(p: Point) -> [f64; 2] {
    [r3(p.x), r3(p.y)]
}

/// The fill of one layer.
#[derive(Debug, Clone, Serialize)]
pub struct LayerFill {
    pub layer: String,
    /// Pieces of fill, of which so many belong to the area and so many hold nothing of the net.
    pub pieces: usize,
    pub of_the_area: usize,
    pub floating: usize,
    /// mm².
    pub filled: f64,
}

/// A pad the area does not reach.
#[derive(Debug, Clone, Serialize)]
pub struct Unreached {
    pub pad: String,
    pub at: [f64; 2],
    /// `island`, `walled`, `no_spoke`, `outside` or `other_layer`.
    pub reason: &'static str,
    /// The reason in plain words.
    pub why: String,
    /// For `island`: its index in `islands`.
    pub island: Option<usize>,
    /// Straight distance in mm to the nearest spot where a via lands in the area.
    pub site_distance: Option<f64>,
    /// The pad holds a via (an exposed pad), and at this spot a via in the pad lands in the
    /// area on another layer. A suggestion: a via in a pad is the user's decision.
    pub pad_via: Option<[f64; 2]>,
}

/// Where one via joins an island to the area.
#[derive(Debug, Clone, Serialize)]
pub struct ViaReport {
    pub at: [f64; 2],
    /// It stands in the island on `from` and lands in the area on `to`.
    pub from: String,
    pub to: String,
    /// Room beyond what the via needs, mm.
    pub spare: f64,
    /// The piece it lands on holds nothing of the net yet and needs this second via to hang on
    /// the area.
    pub second: Option<[f64; 2]>,
}

/// A piece of fill with pads that is cut off from the area.
#[derive(Debug, Clone, Serialize)]
pub struct IslandReport {
    pub pads: Vec<String>,
    pub layers: Vec<String>,
    /// Fill in mm² without what lies on its pads, and its widest spot in mm.
    pub filled: f64,
    pub widest: f64,
    pub via: Option<ViaReport>,
    /// Why it has no via site, in plain words.
    pub no_via: Option<String>,
}

/// How far the pads of the net are from a spot where a via lands in the area.
#[derive(Debug, Clone, Serialize)]
pub struct SiteDistance {
    pub mean: f64,
    pub max: f64,
    /// The pad with the longest way.
    pub farthest: String,
}

/// A pad whose answer changes when every clearance moves by half a raster cell.
#[derive(Debug, Clone, Serialize)]
pub struct Doubtful {
    pub pad: String,
    pub at: [f64; 2],
    /// Reached with the clearances as they are, with half a cell less, with half a cell more.
    pub reached: bool,
    pub with_less: bool,
    pub with_more: bool,
}

/// The model against the fill the file holds (KiCad's, as of its last fill), on one layer.
#[derive(Debug, Clone, Serialize)]
pub struct Against {
    pub layer: String,
    /// mm² in both, in the model only, in the file only.
    pub both: f64,
    pub model_only: f64,
    pub file_only: f64,
    /// Of those, more than one and a half cells away from the other fill: beyond what the
    /// raster explains.
    pub model_only_deep: f64,
    pub file_only_deep: f64,
    pub pieces_model: usize,
    pub pieces_file: usize,
    /// Pads of the net on the layer: touched by both fills, by none, by one only.
    pub pads_both: usize,
    pub pads_none: usize,
    pub pads_model_only: Vec<String>,
    pub pads_file_only: Vec<String>,
}

/// The area of one net.
#[derive(Debug, Clone, Serialize)]
pub struct NetReport {
    pub net: String,
    /// The finding in one sentence.
    pub summary: String,
    /// Raster step used, mm: a statement is good to about this.
    pub step: f64,
    pub layers: Vec<LayerFill>,
    /// Fill of the area in mm², and of the pieces apart from it.
    pub area: f64,
    pub apart: f64,
    /// Pads the area joins. Below two nothing is connected through it: its largest piece is
    /// then taken as the area.
    pub joins: usize,
    /// Groups of pads apart from the area (islands, and pads or routed pieces without fill),
    /// and the unconnected items KiCad's DRC would count for them: one each, one less where
    /// the area itself joins no pad.
    pub groups_apart: usize,
    pub expected_unconnected: usize,
    pub pads: usize,
    pub reached: usize,
    /// Of those: touched by the fill itself, and joined through tracks, vias or other pads.
    pub by_fill: usize,
    pub by_copper: usize,
    pub unreached: Vec<Unreached>,
    pub islands: Vec<IslandReport>,
    pub site_distance: Option<SiteDistance>,
    /// Left out when the option `doubtful` is off.
    pub doubtful: Option<Vec<Doubtful>>,
    /// Empty where the file holds no fill of the net.
    pub against_file: Vec<Against>,
    pub seconds: f64,
}

/// What [`area_report`] finds.
#[derive(Debug, Clone, Serialize)]
pub struct AreaReport {
    pub board: String,
    pub nets: Vec<NetReport>,
    /// What a reader has to know to weigh the report.
    pub notes: Vec<String>,
    /// The fill of each net as the model has it, in the order of `nets`, and the names of the
    /// copper layers its rasters are indexed by.
    #[serde(skip)]
    pub fills: Vec<NetFill>,
    #[serde(skip)]
    pub layer_names: Vec<String>,
}

/// The areas of a board file with the areas of `pour` added, and the settings of the model.
pub struct Model {
    pub areas: Vec<Area>,
    pub options: fill::Options,
    /// The fill the file holds for each of its areas (empty for those of `pour`).
    pub filled: Vec<Vec<Polygon>>,
    /// The nets asked for, or every net with an area.
    pub nets: Vec<NetId>,
    /// The areas that `pour` added, as indices into `areas`.
    pub poured: Vec<usize>,
    /// Pads that got no outline in a pour of several nets: alone among the pads of the others.
    pub alone: Vec<usize>,
}

/// Reads the areas of the file and lays those of `pour`.
pub fn model(kb: &KicadBoard, o: &AreaOptions) -> Result<Model> {
    let b = &kb.board;
    let from_file = gummiband_kicad::area::areas(kb);
    let mut areas = from_file.areas;
    let mut filled = from_file.filled;
    let asked: Vec<NetId> = o
        .nets
        .iter()
        .map(|n| b.net_id(n).with_context(|| format!("no net named {n:?}")))
        .collect::<Result<_>>()?;
    for (name, v) in [
        ("zone_clearance", o.zone_clearance),
        ("zone_min_width", o.zone_min_width),
        ("thermal_gap", o.thermal_gap),
    ] {
        if !(v >= 0.0) {
            bail!("{name} must not be negative");
        }
    }
    if !(o.thermal_spoke_width > 0.0) {
        bail!("thermal_spoke_width must be positive");
    }
    if !(o.step > 0.0) {
        bail!("step must be positive");
    }
    let settings = PourSettings {
        clearance: o.zone_clearance,
        min_width: o.zone_min_width,
        thermal_gap: o.thermal_gap,
        spoke_width: o.thermal_spoke_width,
        connect: parse_connect(&o.pad_connection)?,
        priority: 0,
    };
    let mut poured = Vec::new();
    let mut alone = Vec::new();
    if !o.pour.is_empty() {
        if asked.is_empty() {
            bail!("pour needs nets: name the nets that get an area");
        }
        let layers: Vec<LayerId> = o
            .pour
            .iter()
            .map(|l| b.layer_id(l).with_context(|| format!("no copper layer {l:?}")))
            .collect::<Result<_>>()?;
        let laid = lay_pours(b, &areas, &asked, &layers, &settings)?;
        for a in laid.areas {
            poured.push(areas.len());
            areas.push(a);
            filled.push(Vec::new());
        }
        alone = laid.alone;
    }
    let nets = if asked.is_empty() {
        let mut v: Vec<NetId> = areas.iter().map(|x| x.net).collect();
        v.sort_unstable();
        v.dedup();
        v
    } else {
        asked
    };
    Ok(Model {
        areas,
        options: fill::Options {
            step: o.step,
            pad_connect: from_file.pad_connect,
            no_pour: from_file.no_pour,
            ..Default::default()
        },
        filled,
        nets,
        poured,
        alone,
    })
}

/// The settings of the fill of a pour that is laid.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PourSettings {
    pub clearance: f64,
    pub min_width: f64,
    pub thermal_gap: f64,
    pub spoke_width: f64,
    pub connect: Connect,
    pub priority: i64,
}

impl Default for PourSettings {
    fn default() -> Self {
        let p = Area::pour(0, 0, Vec::new());
        PourSettings {
            clearance: p.clearance,
            min_width: p.min_width,
            thermal_gap: p.thermal_gap,
            spoke_width: p.spoke_width,
            connect: p.connect,
            priority: 0,
        }
    }
}

/// What [`lay_pours`] lays.
#[derive(Debug, Clone, Default)]
pub struct Laid {
    /// One area per net, layer and outline.
    pub areas: Vec<Area>,
    /// Pads that got no outline: alone among the pads of the other nets (several nets only).
    pub alone: Vec<usize>,
    /// Net and layer that stay as they are, because the net has a zone there already.
    pub kept: Vec<(NetId, LayerId)>,
}

/// The areas of a pour of `nets` on `layers`. One net gets the board outline. Several nets
/// share the board by nearest pad (`gummiband_core::region`): their outlines never overlap, and
/// a pad alone among the pads of another net gets none. A layer on which a net has a zone
/// already is left as it is; the small `gb:` pours of `power_copper` do not count.
pub fn lay_pours(b: &Board, existing: &[Area], nets: &[NetId], layers: &[LayerId], s: &PourSettings) -> Result<Laid> {
    let mut out = Laid::default();
    let outlines: Vec<(NetId, Polygon)> = if nets.len() == 1 {
        let outline = b
            .main_outline()
            .context("the board has no closed outline (Edge.Cuts)")?;
        vec![(nets[0], outline.clone())]
    } else {
        b.main_outline()
            .context("the board has no closed outline (Edge.Cuts)")?;
        let r = region::by_nearest_pad(b, nets, region::step_for(s.clearance));
        out.alone = r.alone;
        r.regions.into_iter().map(|x| (x.net, x.ring)).collect()
    };
    for &net in nets {
        for &layer in layers {
            let has = |a: &&Area| {
                a.net == net && a.layer == layer && !a.name.starts_with(gummiband_kicad::read::GB_ZONE_PREFIX)
            };
            if existing.iter().any(|a| has(&a)) {
                out.kept.push((net, layer));
                continue;
            }
            for (_, ring) in outlines.iter().filter(|(n, _)| *n == net) {
                let mut a = Area::pour(net, layer, vec![ring.clone()]);
                a.clearance = s.clearance;
                a.min_width = s.min_width;
                a.thermal_gap = s.thermal_gap;
                a.spoke_width = s.spoke_width;
                a.connect = s.connect;
                a.priority = s.priority;
                out.areas.push(a);
            }
        }
    }
    Ok(out)
}

/// Adds a via of `net` to the board as locked copper on every layer.
pub fn add_via(board: &mut Board, net: NetId, at: Point, diameter: f64, drill: f64) {
    let clearance = board.class_of(net).clearance;
    board.obstacles.push(Obstacle {
        shape: Shape::Circle {
            c: at,
            r: diameter / 2.0,
        },
        layers: LayerSet::all(board.layers.len()),
        net: Some(net),
        kind: ObstacleKind::Via,
        hole: Some(drill / 2.0),
        clearance,
        pin: None,
        locked: true,
    });
}

/// Stitches the islands of `net`: one via per island at the site the model finds (and the
/// second via where that site lands on a piece that is itself apart), then the model again on
/// the board with those vias, until no island has a site left. Every via joins an island to the
/// area; none is set where the model finds nothing to join. The vias are added to `board` and
/// returned.
pub fn stitch_islands(board: &mut Board, areas: &[Area], net: NetId, o: &fill::Options) -> Vec<Point> {
    let class = board.class_of(net).clone();
    let (diameter, drill) = o.via.unwrap_or((class.via_diameter, class.via_drill));
    let pitch = diameter + board.rules.hole_to_hole.max(class.clearance);
    let mut placed: Vec<Point> = Vec::new();
    for _round in 0..8 {
        let f = fill::fill_net(board, areas, net, o);
        let before = placed.len();
        let sites = f.islands.iter().filter_map(|i| i.via.as_ref());
        for at in sites.flat_map(|v| v.anchor.into_iter().chain([v.at])) {
            if placed.iter().any(|q| q.dist(at) < pitch) {
                continue;
            }
            placed.push(at);
            add_via(board, net, at, diameter, drill);
        }
        if placed.len() == before {
            break;
        }
    }
    placed
}

/// A via in every pad of `net` that nothing else reaches and that holds one (an exposed pad),
/// where it lands in the area on another layer, with the second via where it lands on a piece
/// that is itself apart. The vias are added to `board` and returned.
pub fn stitch_pads(board: &mut Board, areas: &[Area], net: NetId, o: &fill::Options) -> Vec<Point> {
    let class = board.class_of(net).clone();
    let (diameter, drill) = o.via.unwrap_or((class.via_diameter, class.via_drill));
    let pitch = diameter + board.rules.hole_to_hole.max(class.clearance);
    let f = fill::fill_net(board, areas, net, o);
    let mut placed: Vec<Point> = Vec::new();
    for site in f.unreached().filter_map(|p| p.pad_site.as_ref()) {
        for at in site.anchor.into_iter().chain([site.at]) {
            if placed.iter().any(|q| q.dist(at) < pitch) {
                continue;
            }
            placed.push(at);
            add_via(board, net, at, diameter, drill);
        }
    }
    placed
}

/// The reason a pad is not reached, as a word for programs and in plain words.
pub fn why_unreached(r: Reach) -> (&'static str, &'static str) {
    match r {
        Reach::Fill | Reach::Copper => ("reached", "reached"),
        Reach::Island(_) => (
            "island",
            "it lies on a piece of fill that copper of other nets cuts off from the area",
        ),
        Reach::Walled => (
            "walled",
            "it lies in the outline, but copper of other nets leaves no room for fill next to it (clearance plus minimum width)",
        ),
        Reach::NoSpoke => (
            "no_spoke",
            "it has a thermal relief and none of its spokes ends in fill",
        ),
        Reach::Outside => ("outside", "it lies outside the outline of the area on its layer"),
        Reach::OtherLayer => (
            "other_layer",
            "the net has no area on the layer of the pad: it needs a via or a track",
        ),
    }
}

fn no_via_words(w: &NoVia) -> String {
    match w {
        NoVia::TooNarrow { widest, need } => {
            format!("its widest spot is {widest:.2} mm and a via has {need:.2} mm")
        }
        NoVia::Blocked => {
            "copper of another net, a hole or the board edge is in the way on some layer under every spot".into()
        }
        NoVia::NothingOpposite => "no fill of the area lies above or below it".into(),
    }
}

/// The names of a list of pads, at most `limit` of them.
fn some(names: &[&str], limit: usize) -> String {
    if names.len() <= limit {
        names.join(" ")
    } else {
        format!("{} ... ({} more)", names[..limit].join(" "), names.len() - limit)
    }
}

fn against(b: &Board, f: &NetFill, layer: LayerId, file: &[Polygon]) -> Option<Against> {
    let model = f.fill[layer as usize].as_ref()?;
    let g = &f.raster;
    let k = g.rings(file);
    let cell = g.step * g.step;
    let inv = |m: &[bool]| m.iter().map(|&x| !x).collect::<Vec<bool>>();
    let to_k = edt(&inv(&k), g.nx, g.ny);
    let to_m = edt(&inv(model), g.nx, g.ny);
    let (mut both, mut mo, mut ko, mut mo_deep, mut ko_deep) = (0usize, 0usize, 0usize, 0usize, 0usize);
    for i in 0..g.n() {
        match (model[i], k[i]) {
            (true, true) => both += 1,
            (true, false) => {
                mo += 1;
                mo_deep += (to_k[i] > 1.5) as usize;
            }
            (false, true) => {
                ko += 1;
                ko_deep += (to_m[i] > 1.5) as usize;
            }
            _ => {}
        }
    }
    let touches = |m: &[bool], s: &Shape| {
        let bb = s.bbox().grow(g.step);
        let mut y = bb.min.y;
        while y <= bb.max.y {
            let mut x = bb.min.x;
            while x <= bb.max.x {
                let p = pt(x, y);
                if s.dist_to_point(p) <= g.step && g.cell(p).is_some_and(|i| m[i]) {
                    return true;
                }
                x += g.step;
            }
            y += g.step;
        }
        false
    };
    let mut c = Against {
        layer: b.layers[layer as usize].name.clone(),
        both: r3(both as f64 * cell),
        model_only: r3(mo as f64 * cell),
        file_only: r3(ko as f64 * cell),
        model_only_deep: r3(mo_deep as f64 * cell),
        file_only_deep: r3(ko_deep as f64 * cell),
        pieces_model: pieces_of(model, g.nx, g.ny).1,
        pieces_file: pieces_of(&k, g.nx, g.ny).1,
        pads_both: 0,
        pads_none: 0,
        pads_model_only: Vec::new(),
        pads_file_only: Vec::new(),
    };
    for pr in &f.pins {
        let pin = &b.pins[pr.pin];
        if !pin.layers.has(layer) {
            continue;
        }
        let shapes: Vec<&Shape> = pin.obstacles.iter().map(|&o| &b.obstacles[o].shape).collect();
        let m = shapes.iter().any(|s| touches(model, s));
        let kk = shapes.iter().any(|s| touches(&k, s));
        match (m, kk) {
            (true, true) => c.pads_both += 1,
            (false, false) => c.pads_none += 1,
            (true, false) => c.pads_model_only.push(pin.name.clone()),
            (false, true) => c.pads_file_only.push(pin.name.clone()),
        }
    }
    Some(c)
}

/// The fill of one net on `board` as a report, with the fill itself. `doubtful` runs the model
/// twice more, with every clearance half a raster cell smaller and larger.
pub fn net_report(
    board: &Board,
    areas: &[Area],
    net: NetId,
    o: &fill::Options,
    doubtful: bool,
) -> (NetReport, NetFill) {
    let t0 = std::time::Instant::now();
    let f = fill::fill_net(board, areas, net, o);
    let name = board.nets[net as usize].name.clone();
    let lname = |l: LayerId| board.layers[l as usize].name.clone();

    let mut layers = Vec::new();
    for l in 0..board.layers.len() {
        if f.fill[l].is_none() {
            continue;
        }
        let ps: Vec<_> = f.pieces.iter().filter(|p| p.layer as usize == l).collect();
        layers.push(LayerFill {
            layer: lname(l as LayerId),
            pieces: ps.len(),
            of_the_area: ps.iter().filter(|p| p.main).count(),
            floating: ps.iter().filter(|p| p.floating).count(),
            filled: r3(ps.iter().map(|p| p.area).sum()),
        });
    }
    let islands: Vec<IslandReport> = f
        .islands
        .iter()
        .map(|i| IslandReport {
            pads: i.names.clone(),
            layers: i.layers.iter().map(|&l| lname(l)).collect(),
            filled: r3(i.area),
            widest: r3(i.widest),
            via: i.via.as_ref().map(|v| ViaReport {
                at: xy(v.at),
                from: lname(v.from),
                to: lname(v.to),
                spare: r3(v.room),
                second: v.anchor.map(xy),
            }),
            no_via: i.no_via.as_ref().map(no_via_words),
        })
        .collect();
    let unreached: Vec<Unreached> = f
        .unreached()
        .map(|p| {
            let (reason, why) = why_unreached(p.reach);
            Unreached {
                pad: p.name.clone(),
                at: xy(board.pins[p.pin].pos),
                reason,
                why: why.to_string(),
                island: match p.reach {
                    Reach::Island(k) => Some(k),
                    _ => None,
                },
                site_distance: (p.site_dist >= 0.0).then(|| r3(p.site_dist)),
                pad_via: p.pad_site.as_ref().map(|v| xy(v.at)),
            }
        })
        .collect();
    let with_site: Vec<&fill::PinReach> = f.pins.iter().filter(|p| p.site_dist >= 0.0).collect();
    let site_distance = with_site
        .iter()
        .max_by(|a, b| a.site_dist.total_cmp(&b.site_dist))
        .map(|far| SiteDistance {
            mean: r3(with_site.iter().map(|p| p.site_dist).sum::<f64>() / with_site.len() as f64),
            max: r3(far.site_dist),
            farthest: far.name.clone(),
        });
    let doubt = doubtful.then(|| {
        let at = |slack: f64| {
            let o = fill::Options { slack, ..o.clone() };
            let f = fill::fill_net(board, areas, net, &o);
            f.pins.iter().map(|p| p.reached()).collect::<Vec<bool>>()
        };
        let half = f.raster.step / 2.0;
        let (less, more) = (at(-half), at(half));
        f.pins
            .iter()
            .enumerate()
            .filter(|(i, p)| less[*i] != p.reached() || more[*i] != p.reached())
            .map(|(i, p)| Doubtful {
                pad: p.name.clone(),
                at: xy(board.pins[p.pin].pos),
                reached: p.reached(),
                with_less: less[i],
                with_more: more[i],
            })
            .collect::<Vec<_>>()
    });

    let pads = f.pins.len();
    let reached = pads - unreached.len();
    let by_fill = f.pins.iter().filter(|p| p.reach == Reach::Fill).count();
    let on_layers: Vec<String> = {
        let mut ls: Vec<LayerId> = f.pieces.iter().filter(|p| p.main).map(|p| p.layer).collect();
        ls.sort_unstable();
        ls.dedup();
        ls.into_iter().map(lname).collect()
    };
    let summary = if f.pieces.is_empty() {
        if layers.is_empty() {
            format!("{name} has no area: no zone in the file and no layer of pour.")
        } else {
            format!(
                "{name} has no fill: copper of other nets, the board edge or the minimum width leave nothing of its outline, and none of its {pads} pads is reached."
            )
        }
    } else if f.main_pads < 2 {
        format!(
            "The fill of {name} joins no two pads: its largest piece ({:.1} mm2 on {}) is taken as the area, {} of {pads} pads hang on it and {} are not reached.",
            f.area_main,
            on_layers.join(", "),
            reached,
            unreached.len()
        )
    } else if unreached.is_empty() {
        format!(
            "The area of {name} ({:.1} mm2 on {}) reaches all {pads} pads.",
            f.area_main,
            on_layers.join(", ")
        )
    } else {
        let mut by: BTreeMap<&str, usize> = BTreeMap::new();
        for u in &unreached {
            *by.entry(u.reason).or_default() += 1;
        }
        let parts: Vec<String> = [
            ("island", "on islands"),
            ("walled", "walled in"),
            ("no_spoke", "without a spoke"),
            ("outside", "outside the outline"),
            ("other_layer", "on a layer without area"),
        ]
        .iter()
        .filter_map(|(k, words)| by.get(k).map(|n| format!("{n} {words}")))
        .collect();
        format!(
            "The area of {name} ({:.1} mm2 on {}) reaches {reached} of {pads} pads; {} are not reached: {}.",
            f.area_main,
            on_layers.join(", "),
            unreached.len(),
            parts.join(", ")
        )
    };
    let report = NetReport {
        net: name,
        summary,
        step: r3(f.raster.step * 100.0) / 100.0,
        layers,
        area: r3(f.area_main),
        apart: r3(f.area_rest),
        joins: f.main_pads,
        groups_apart: f.groups_apart,
        expected_unconnected: f.groups_apart - (f.main_pads == 0 && f.groups_apart > 0) as usize,
        pads,
        reached,
        by_fill,
        by_copper: reached - by_fill,
        unreached,
        islands,
        site_distance,
        doubtful: doubt,
        against_file: Vec::new(),
        seconds: (t0.elapsed().as_secs_f64() * 100.0).round() / 100.0,
    };
    (report, f)
}

/// The area report of a board that is read already (with all its copper kept).
pub fn area_report_of(kb: &KicadBoard, name: &str, o: &AreaOptions) -> Result<AreaReport> {
    let m = model(kb, o)?;
    let b = &kb.board;
    let mut out = AreaReport {
        board: name.to_string(),
        nets: Vec::new(),
        notes: vec![
            "The fill is the router's model on a raster, not KiCad's fill: a statement is good to about one step, and strips of about the minimum width have not been compared with KiCad yet.".into(),
        ],
        fills: Vec::new(),
        layer_names: b.layers.iter().map(|l| l.name.clone()).collect(),
    };
    if m.nets.is_empty() {
        out.notes
            .push("The board has no copper zone: name nets and layers with nets and pour.".into());
    }
    if !m.alone.is_empty() {
        let names: Vec<&str> = m.alone.iter().map(|&p| b.pins[p].name.as_str()).collect();
        out.notes.push(format!(
            "Pads that get no outline in a pour of several nets, because they lie alone among the pads of another net: {}.",
            some(&names, 40)
        ));
    }
    for &net in &m.nets {
        let (mut r, f) = net_report(b, &m.areas, net, &m.options, o.doubtful);
        for l in 0..b.layers.len() as LayerId {
            let file: Vec<Polygon> = m
                .areas
                .iter()
                .zip(&m.filled)
                .filter(|(a, _)| a.net == net && a.layer == l)
                .flat_map(|(_, p)| p.iter().cloned())
                .collect();
            if !file.is_empty()
                && let Some(c) = against(b, &f, l, &file)
            {
                r.against_file.push(c);
            }
        }
        out.nets.push(r);
        out.fills.push(f);
    }
    Ok(out)
}

/// The area report of a board file: the one function behind `gummiband areas` and the MCP tool
/// `area_report`. The file is read with all its tracks and vias as copper.
pub fn area_report(board: &Path, o: &AreaOptions) -> Result<AreaReport> {
    let kb = read_board(
        board,
        &ReadOptions {
            keep: KeepCopper::All,
            plane_layers: o.plane_layers.clone(),
            ..Default::default()
        },
    )?;
    area_report_of(&kb, &board.display().to_string(), o)
}

impl NetReport {
    /// The report of the net as text; `brief` gives the summary only.
    pub fn text(&self, brief: bool) -> String {
        let mut s = format!("{}\n", self.summary);
        if brief {
            return s;
        }
        s += &format!(
            "  {} pads: {} by fill, {} through other copper of the net, {} not reached in {} groups apart from the area; {:.1} mm2 of fill apart from the area; step {} mm, {} s\n",
            self.pads,
            self.by_fill,
            self.by_copper,
            self.unreached.len(),
            self.groups_apart,
            self.apart,
            self.step,
            self.seconds
        );
        for l in &self.layers {
            s += &format!(
                "  {}: {} pieces of fill ({} of the area, {} that hold nothing of the net), {:.1} mm2\n",
                l.layer, l.pieces, l.of_the_area, l.floating, l.filled
            );
        }
        for (k, i) in self.islands.iter().enumerate() {
            let pads: Vec<&str> = i.pads.iter().map(String::as_str).collect();
            let via = match (&i.via, &i.no_via) {
                (Some(v), _) => format!(
                    "one via at {} {} joins it ({} to {}, {:.2} mm to spare){}",
                    v.at[0],
                    v.at[1],
                    v.from,
                    v.to,
                    v.spare,
                    v.second.map_or(String::new(), |q| format!(
                        ", with a second via at {} {} for the piece it lands on",
                        q[0], q[1]
                    ))
                ),
                (_, Some(why)) => format!("no via joins it: {why}"),
                _ => String::new(),
            };
            s += &format!(
                "  island {k} on {}: {:.2} mm2, pads {}; {via}\n",
                i.layers.join("+"),
                i.filled,
                some(&pads, 12)
            );
        }
        for (reason, _) in [("walled", ()), ("no_spoke", ()), ("outside", ()), ("other_layer", ())] {
            let of: Vec<&Unreached> = self.unreached.iter().filter(|u| u.reason == reason).collect();
            if let Some(first) = of.first() {
                let pads: Vec<&str> = of.iter().map(|u| u.pad.as_str()).collect();
                s += &format!(
                    "  not reached, {}: {}\n",
                    first.why.replacen("it ", "", 1),
                    some(&pads, 40)
                );
            }
        }
        for u in self.unreached.iter().filter(|u| u.pad_via.is_some()) {
            let at = u.pad_via.unwrap();
            s += &format!(
                "  {} holds a via: one in the pad at {} {} would join it to the area\n",
                u.pad, at[0], at[1]
            );
        }
        if let Some(d) = &self.site_distance {
            s += &format!(
                "  from a pad to the nearest spot where a via lands in the area: mean {:.2} mm, at most {:.2} mm ({})\n",
                d.mean, d.max, d.farthest
            );
        } else if self.pads > 0 {
            s += "  no spot where a via lands in the area\n";
        }
        match &self.doubtful {
            Some(d) if d.is_empty() => {
                s += "  no pad changes its answer when every clearance moves by half a raster cell\n"
            }
            Some(d) => {
                let gained: Vec<&str> = d
                    .iter()
                    .filter(|x| !x.reached && (x.with_less || x.with_more))
                    .map(|x| x.pad.as_str())
                    .collect();
                let lost: Vec<&str> = d.iter().filter(|x| x.reached).map(|x| x.pad.as_str()).collect();
                s += &format!(
                    "  doubtful, the answer changes when every clearance moves by half a raster cell: {} pads\n",
                    d.len()
                );
                if !gained.is_empty() {
                    s += &format!("    not reached, but reached then: {}\n", some(&gained, 40));
                }
                if !lost.is_empty() {
                    s += &format!("    reached, but not then: {}\n", some(&lost, 40));
                }
            }
            None => {}
        }
        for c in &self.against_file {
            s += &format!(
                "  against the fill in the file on {}: both {:.1} mm2, model only {:.2} ({:.2} beyond the raster), file only {:.2} ({:.2}); pieces {} / {}; pads touched by both {}, by none {}, by the model only {} {:?}, by the file only {} {:?}\n",
                c.layer,
                c.both,
                c.model_only,
                c.model_only_deep,
                c.file_only,
                c.file_only_deep,
                c.pieces_model,
                c.pieces_file,
                c.pads_both,
                c.pads_none,
                c.pads_model_only.len(),
                c.pads_model_only,
                c.pads_file_only.len(),
                c.pads_file_only
            );
        }
        s
    }
}

impl AreaReport {
    /// The report as text; `brief` gives one sentence per net.
    pub fn text(&self, brief: bool) -> String {
        let mut s = String::new();
        for n in &self.nets {
            s += &n.text(brief);
        }
        for n in &self.notes {
            s += &format!("note: {n}\n");
        }
        s
    }
}

/// The fill of a net on one layer as a picture (PGM): white the fill of the area, grey other
/// fill of the net, black none. `None` where the net has no area on the layer.
pub fn fill_picture(f: &NetFill, layer: usize) -> Option<Vec<u8>> {
    let m = f.fill.get(layer)?.as_ref()?;
    let g = &f.raster;
    let (lab, _) = pieces_of(m, g.nx, g.ny);
    let main: Vec<u32> = f
        .pieces
        .iter()
        .filter(|p| p.main && p.layer as usize == layer)
        .filter_map(|p| g.cell(p.at).map(|i| lab[i]))
        .filter(|&l| l != 0)
        .collect();
    let mut out = format!("P5\n{} {}\n255\n", g.nx, g.ny).into_bytes();
    out.extend((0..g.n()).map(|i| {
        if lab[i] != 0 && main.contains(&lab[i]) {
            255u8
        } else if m[i] {
            128
        } else {
            0
        }
    }));
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use gummiband_kicad::read::parse_board;

    /// The names of the fields of `AreaOptions`. A field that is missing here does not compile.
    macro_rules! fields {
        ($($f:ident),* $(,)?) => {{
            let AreaOptions { $($f: _),* } = AreaOptions::default();
            vec![$(stringify!($f)),*]
        }};
    }

    #[test]
    fn the_table_has_every_option_of_the_report_once() {
        let fields = fields!(
            nets,
            pour,
            zone_clearance,
            zone_min_width,
            thermal_gap,
            thermal_spoke_width,
            pad_connection,
            plane_layers,
            step,
            doubtful,
        );
        let table: Vec<&str> = AREA_OPTIONS.iter().map(|o| o.name).collect();
        assert_eq!(table, fields, "AREA_OPTIONS and the fields of AreaOptions, in order");
        for o in AREA_OPTIONS {
            assert!(
                o.help.ends_with('.') && !o.help.trim_end_matches('.').contains(". "),
                "{}: one sentence",
                o.name
            );
            assert!(!o.default.is_empty(), "{}", o.name);
        }
    }

    #[test]
    fn the_defaults_in_words_are_the_defaults() {
        let d = AreaOptions::default();
        let plain = [
            ("zone_clearance", d.zone_clearance.to_string()),
            ("zone_min_width", d.zone_min_width.to_string()),
            ("thermal_gap", d.thermal_gap.to_string()),
            ("thermal_spoke_width", d.thermal_spoke_width.to_string()),
            ("pad_connection", d.pad_connection.clone()),
            ("step", d.step.to_string()),
            ("doubtful", if d.doubtful { "on" } else { "off" }.to_string()),
        ];
        for (name, value) in plain {
            assert_eq!(area_option_doc(name).unwrap().default, value, "{name}");
        }
        assert!(d.nets.is_empty() && d.pour.is_empty() && d.plane_layers.is_empty());
        assert_eq!(parse_connect(&d.pad_connection).unwrap(), Connect::Thermal);
        assert!(parse_connect("spokes").is_err());
    }

    /// GND with four pads and a pour of the file on the front; a track of S runs across the
    /// board and cuts the pad on the right off; a track pair leaves a strip of exactly the
    /// minimum width to the pad at the bottom.
    const BOARD: &str = r#"(kicad_pcb (version 20240108) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (net 0 "") (net 1 "GND") (net 2 "S") (net 3 "T")
  (gr_rect (start 0 0) (end 30 20) (layer "Edge.Cuts") (stroke (width 0.1) (type default)) (fill no))
  (footprint "R" (layer "F.Cu") (at 4 4) (property "Reference" "R1")
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "GND")))
  (footprint "R" (layer "F.Cu") (at 12 4) (property "Reference" "R2")
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "GND")))
  (footprint "R" (layer "F.Cu") (at 26 10) (property "Reference" "R3")
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "GND")))
  (footprint "R" (layer "B.Cu") (at 8 16) (property "Reference" "R4")
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "B.Cu") (net 1 "GND")))
  (footprint "R" (layer "F.Cu") (at 20 1) (property "Reference" "R5")
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 2 "S")))
  (footprint "R" (layer "F.Cu") (at 20 19) (property "Reference" "R6")
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 2 "S")))
  (segment (start 20 1) (end 20 19) (width 0.2) (layer "F.Cu") (net 2) (uuid "00000000-0000-4000-8000-000000000001"))
  (zone (net 1) (net_name "GND") (layer "F.Cu") (uuid "00000000-0000-4000-8000-000000000002") (name "") (hatch edge 0.5) (priority 0)
    (connect_pads (clearance 0.2)) (min_thickness 0.2) (fill yes (thermal_gap 0.3) (thermal_bridge_width 0.3))
    (polygon (pts (xy 0 0) (xy 30 0) (xy 30 20) (xy 0 20))))
)"#;

    fn report(o: &AreaOptions) -> AreaReport {
        let kb = parse_board(
            BOARD.to_string(),
            None,
            &ReadOptions {
                keep: KeepCopper::All,
                ..Default::default()
            },
        )
        .unwrap();
        area_report_of(&kb, "test", o).unwrap()
    }

    #[test]
    fn the_report_names_what_the_area_reaches_and_what_not() {
        let r = report(&AreaOptions::default());
        assert_eq!(r.nets.len(), 1);
        let n = &r.nets[0];
        assert_eq!((n.net.as_str(), n.pads, n.reached, n.joins), ("GND", 4, 2, 2));
        assert_eq!((n.by_fill, n.by_copper), (2, 0));
        assert_eq!((n.groups_apart, n.expected_unconnected), (2, 2));
        // the pad behind the track is on an island; one layer only, so no via helps
        let by: Vec<(&str, &str)> = n.unreached.iter().map(|u| (u.pad.as_str(), u.reason)).collect();
        assert_eq!(by, [("R3.1", "island"), ("R4.1", "other_layer")]);
        assert_eq!(n.islands.len(), 1);
        assert_eq!(n.islands[0].pads, ["R3.1"]);
        assert!(n.islands[0].via.is_none());
        assert!(n.islands[0].no_via.as_deref().unwrap().contains("above or below"));
        assert!(n.summary.starts_with("The area of GND"), "{}", n.summary);
        assert!(n.summary.contains("reaches 2 of 4 pads"), "{}", n.summary);
        assert_eq!(n.doubtful.as_ref().map(Vec::len), Some(0));
        assert!(n.against_file.is_empty());
        // with a pour on the back the island gets its via, and the pad on the back is reached
        let o = AreaOptions {
            nets: vec!["GND".into()],
            pour: vec!["B.Cu".into()],
            doubtful: false,
            ..Default::default()
        };
        let r = report(&o);
        let n = &r.nets[0];
        assert_eq!(n.unreached.len(), 2, "{:?}", n.unreached);
        assert!(n.doubtful.is_none());
        let via = n
            .islands
            .iter()
            .find(|i| i.pads == ["R3.1"])
            .unwrap()
            .via
            .as_ref()
            .unwrap();
        assert_eq!((via.from.as_str(), via.to.as_str()), ("F.Cu", "B.Cu"));
        assert!(via.at[0] > 20.0);
        // the back is an island of its own with the pad there: the via lands on it, and the
        // via that joins the back to the area is the second one
        assert!(via.second.is_some_and(|q| q[0] < 20.0));
        let text = r.text(false);
        assert!(text.contains("island 0 on F.Cu"), "{text}");
        assert!(text.contains("note: The fill is the router's model"), "{text}");
        // the JSON carries the same, without the rasters
        let json = serde_json::to_value(&r).unwrap();
        assert_eq!(json["nets"][0]["net"], "GND");
        assert!(json.get("fills").is_none());
    }

    #[test]
    fn a_fill_that_joins_no_two_pads_is_not_called_the_area_of_the_net() {
        // only the pad on the right and the one on the back are left: no piece joins two
        let board = BOARD.replacen("(net 1 \"GND\")))", "(net 3 \"T\")))", 2);
        let kb = parse_board(
            board,
            None,
            &ReadOptions {
                keep: KeepCopper::All,
                ..Default::default()
            },
        )
        .unwrap();
        let r = area_report_of(&kb, "test", &AreaOptions::default()).unwrap();
        let n = &r.nets[0];
        assert_eq!((n.pads, n.joins), (2, 0));
        assert!(
            n.summary.starts_with("The fill of GND joins no two pads"),
            "{}",
            n.summary
        );
    }

    #[test]
    fn a_fill_in_the_file_is_held_against_the_model() {
        // the file's fill covers the left half only, and leaves the pad at 12 4 out
        let board = BOARD.replace(
            "(polygon (pts (xy 0 0) (xy 30 0) (xy 30 20) (xy 0 20))))",
            "(polygon (pts (xy 0 0) (xy 30 0) (xy 30 20) (xy 0 20)))\n    \
             (filled_polygon (layer \"F.Cu\") (pts (xy 0.3 0.3) (xy 10 0.3) (xy 10 19.7) (xy 0.3 19.7))))",
        );
        let kb = parse_board(
            board,
            None,
            &ReadOptions {
                keep: KeepCopper::All,
                ..Default::default()
            },
        )
        .unwrap();
        let r = area_report_of(&kb, "test", &AreaOptions::default()).unwrap();
        let c = &r.nets[0].against_file;
        assert_eq!(c.len(), 1);
        assert_eq!((c[0].layer.as_str(), c[0].pieces_file), ("F.Cu", 1));
        // the model fills right of x = 10 as well, far beyond a raster cell
        assert!(c[0].both > 150.0 && c[0].model_only_deep > 200.0, "{:?}", c[0]);
        assert!(c[0].file_only < 20.0, "{:?}", c[0]);
        assert_eq!(c[0].pads_both, 1);
        assert_eq!(c[0].pads_model_only, ["R2.1", "R3.1"]);
        assert!(c[0].pads_file_only.is_empty());
        assert!(r.text(false).contains("against the fill in the file on F.Cu"));
    }

    #[test]
    fn what_cannot_be_reported_is_refused_in_words() {
        let kb = parse_board(BOARD.to_string(), None, &ReadOptions::default()).unwrap();
        let err = |o: AreaOptions| area_report_of(&kb, "test", &o).unwrap_err().to_string();
        let with = |f: &dyn Fn(&mut AreaOptions)| {
            let mut o = AreaOptions::default();
            f(&mut o);
            o
        };
        assert!(err(with(&|o| o.nets = vec!["NOPE".into()])).contains("no net named"));
        assert!(err(with(&|o| o.pour = vec!["B.Cu".into()])).contains("pour needs nets"));
        assert!(
            err(with(&|o| {
                o.nets = vec!["GND".into()];
                o.pour = vec!["In9.Cu".into()];
            }))
            .contains("no copper layer")
        );
        assert!(err(with(&|o| o.pad_connection = "spokes".into())).contains("thermal, solid"));
        assert!(err(with(&|o| o.thermal_spoke_width = 0.0)).contains("positive"));
    }
}
