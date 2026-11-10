//! Experimental: the fill model of `gummiband_core::fill` on a board file, as a report.
//!
//! For every net with a copper zone (or the nets given), on the copper the file has: the pieces
//! of fill, the pads the fill reaches, the pads it does not reach and why, the islands with
//! pads, and per island the site of one via that joins it to the area. Where KiCad has filled
//! the zones of the file, the model is held against that fill.
//!
//! Usage: area_fill BOARD.kicad_pcb [options]
//!   --net NAME            report this net (repeatable; default: every net with a zone)
//!   --pour LAYER[,LAYER]  also lay an area of the nets given over the board outline on these
//!                         layers, as `ground_pour` does, where the net has none yet
//!   --clearance MM, --min-width MM, --thermal-gap MM, --spoke-width MM,
//!   --connect thermal|solid|thermal_tht|none
//!                         settings of the areas of --pour (default 0.2, 0.2, 0.3, 0.3, thermal)
//!   --step MM             raster step wanted (default 0.05)
//!   --plane-layers L,..   read these layers as planes, as the router's option does
//!   --json FILE           everything as JSON
//!   --pgm PREFIX          the fill per net and layer as PREFIX.<net>.<layer>.pgm (white: fill
//!                         of the area, grey: other fill of the net, black: none)
//!   --stitch OUT.kicad_pcb
//!                         write the board with the areas of --pour and one locked via per
//!                         island that has a site, found in rounds until none is left
//!   --quiet               one line per net
use std::collections::BTreeMap;
use std::path::PathBuf;

use gummiband_core::board::{Board, LayerSet, NetId, Obstacle, ObstacleKind};
use gummiband_core::fill::{self, Area, Connect, NetFill, NoVia, Options, Reach, edt, pieces_of};
use gummiband_core::geom::Shape;
use gummiband_kicad::{KeepCopper, ReadOptions, read_board, write};

struct Args {
    board: PathBuf,
    nets: Vec<String>,
    pour: Vec<String>,
    clearance: f64,
    min_width: f64,
    thermal_gap: f64,
    spoke_width: f64,
    connect: Connect,
    step: f64,
    plane_layers: Vec<String>,
    json: Option<PathBuf>,
    pgm: Option<String>,
    stitch: Option<PathBuf>,
    quiet: bool,
}

fn args() -> Args {
    let mut a = Args {
        board: PathBuf::new(),
        nets: Vec::new(),
        pour: Vec::new(),
        clearance: 0.2,
        min_width: 0.2,
        thermal_gap: 0.3,
        spoke_width: 0.3,
        connect: Connect::Thermal,
        step: 0.05,
        plane_layers: Vec::new(),
        json: None,
        pgm: None,
        stitch: None,
        quiet: false,
    };
    let mut it = std::env::args().skip(1);
    let list = |s: String| {
        s.split(',')
            .map(str::to_string)
            .filter(|x| !x.is_empty())
            .collect::<Vec<_>>()
    };
    while let Some(x) = it.next() {
        let mut val = || it.next().unwrap_or_else(|| panic!("{x} needs a value"));
        let num = |s: String| s.parse::<f64>().unwrap_or_else(|_| panic!("{s}: not a number"));
        match x.as_str() {
            "--net" => a.nets.push(val()),
            "--pour" => a.pour = list(val()),
            "--clearance" => a.clearance = num(val()),
            "--min-width" => a.min_width = num(val()),
            "--thermal-gap" => a.thermal_gap = num(val()),
            "--spoke-width" => a.spoke_width = num(val()),
            "--connect" => {
                a.connect = match val().as_str() {
                    "thermal" => Connect::Thermal,
                    "solid" => Connect::Solid,
                    "thermal_tht" => Connect::ThermalTht,
                    "none" => Connect::None,
                    o => panic!("--connect {o}: use thermal, solid, thermal_tht or none"),
                }
            }
            "--step" => a.step = num(val()),
            "--plane-layers" => a.plane_layers = list(val()),
            "--json" => a.json = Some(val().into()),
            "--pgm" => a.pgm = Some(val()),
            "--stitch" => a.stitch = Some(val().into()),
            "--quiet" => a.quiet = true,
            s if s.starts_with("--") => panic!("unknown option {s}"),
            _ => a.board = x.into(),
        }
    }
    if a.board.as_os_str().is_empty() {
        panic!("usage: area_fill BOARD.kicad_pcb [--net NAME] [--pour LAYERS] ... (see the head of the source)");
    }
    a
}

/// The model against the fill KiCad wrote, for one net on one layer.
struct Compare {
    layer: String,
    /// mm² in both, in the model only, in KiCad's fill only.
    both: f64,
    model_only: f64,
    kicad_only: f64,
    /// Of those, more than one and a half cells away from the other fill: beyond what the
    /// raster explains.
    model_only_deep: f64,
    kicad_only_deep: f64,
    pieces_model: usize,
    pieces_kicad: usize,
    /// Pads of the net on the layer: touched by both fills, by none, by the model only, by
    /// KiCad's only (with their names).
    pads_both: usize,
    pads_none: usize,
    pads_model_only: Vec<String>,
    pads_kicad_only: Vec<String>,
}

impl Compare {
    fn json(&self) -> serde_json::Value {
        serde_json::json!({
            "layer": self.layer,
            "both": self.both,
            "model_only": self.model_only,
            "kicad_only": self.kicad_only,
            "model_only_deep": self.model_only_deep,
            "kicad_only_deep": self.kicad_only_deep,
            "pieces_model": self.pieces_model,
            "pieces_kicad": self.pieces_kicad,
            "pads_both": self.pads_both,
            "pads_none": self.pads_none,
            "pads_model_only": self.pads_model_only,
            "pads_kicad_only": self.pads_kicad_only,
        })
    }
}

fn compare(b: &Board, f: &NetFill, layer: u8, kicad: &[gummiband_core::geom::Polygon]) -> Option<Compare> {
    let model = f.fill[layer as usize].as_ref()?;
    let g = &f.raster;
    let k = g.rings(kicad);
    let cell = g.step * g.step;
    let inv = |m: &[bool]| m.iter().map(|&x| !x).collect::<Vec<bool>>();
    let to_k = edt(&inv(&k), g.nx, g.ny);
    let to_m = edt(&inv(model), g.nx, g.ny);
    let (mut both, mut mo, mut ko, mut mod_, mut kod) = (0usize, 0usize, 0usize, 0usize, 0usize);
    for i in 0..g.n() {
        match (model[i], k[i]) {
            (true, true) => both += 1,
            (true, false) => {
                mo += 1;
                mod_ += (to_k[i] > 1.5) as usize;
            }
            (false, true) => {
                ko += 1;
                kod += (to_m[i] > 1.5) as usize;
            }
            _ => {}
        }
    }
    let touches = |m: &[bool], s: &Shape| {
        let bb = s.bbox().grow(g.step);
        let mut hit = false;
        let (mut y, y1) = (bb.min.y, bb.max.y);
        while y <= y1 && !hit {
            let mut x = bb.min.x;
            while x <= bb.max.x {
                let p = gummiband_core::geom::pt(x, y);
                if s.dist_to_point(p) <= g.step && g.cell(p).is_some_and(|i| m[i]) {
                    hit = true;
                    break;
                }
                x += g.step;
            }
            y += g.step;
        }
        hit
    };
    let mut c = Compare {
        layer: b.layers[layer as usize].name.clone(),
        both: both as f64 * cell,
        model_only: mo as f64 * cell,
        kicad_only: ko as f64 * cell,
        model_only_deep: mod_ as f64 * cell,
        kicad_only_deep: kod as f64 * cell,
        pieces_model: pieces_of(model, g.nx, g.ny).1,
        pieces_kicad: pieces_of(&k, g.nx, g.ny).1,
        pads_both: 0,
        pads_none: 0,
        pads_model_only: Vec::new(),
        pads_kicad_only: Vec::new(),
    };
    for pr in &f.pins {
        let pin = &b.pins[pr.pin];
        if !pin.layers.has(layer) {
            continue;
        }
        let shapes: Vec<&Shape> = pin.obstacles.iter().map(|&o| &b.obstacles[o].shape).collect();
        let (m, kk) = (
            shapes.iter().any(|s| touches(model, s)),
            shapes.iter().any(|s| touches(&k, s)),
        );
        match (m, kk) {
            (true, true) => c.pads_both += 1,
            (false, false) => c.pads_none += 1,
            (true, false) => c.pads_model_only.push(pin.name.clone()),
            (false, true) => c.pads_kicad_only.push(pin.name.clone()),
        }
    }
    Some(c)
}

fn reach_name(r: Reach) -> &'static str {
    match r {
        Reach::Fill => "fill",
        Reach::Copper => "copper",
        Reach::Island(_) => "island",
        Reach::Walled => "walled",
        Reach::NoSpoke => "no-spoke",
        Reach::Outside => "outside",
        Reach::OtherLayer => "other-layer",
    }
}

fn pgm(path: &str, f: &NetFill, layer: usize) {
    let Some(m) = &f.fill[layer] else { return };
    let g = &f.raster;
    let mut main = vec![false; g.n()];
    for p in f.pieces.iter().filter(|p| p.main && p.layer as usize == layer) {
        // pieces are labelled again here: the report keeps one point per piece
        let (lab, _) = pieces_of(m, g.nx, g.ny);
        if let Some(i) = g.cell(p.at) {
            let l = lab[i];
            for j in 0..g.n() {
                if lab[j] == l && l != 0 {
                    main[j] = true;
                }
            }
        }
    }
    let mut out = format!("P5\n{} {}\n255\n", g.nx, g.ny).into_bytes();
    out.extend((0..g.n()).map(|i| {
        if main[i] {
            255u8
        } else if m[i] {
            128
        } else {
            0
        }
    }));
    std::fs::write(path, out).unwrap_or_else(|e| panic!("writing {path}: {e}"));
}

fn main() {
    let a = args();
    let kb = read_board(
        &a.board,
        &ReadOptions {
            keep: KeepCopper::All,
            plane_layers: a.plane_layers.clone(),
            ..Default::default()
        },
    )
    .unwrap_or_else(|e| panic!("{e:#}"));
    let mut board = kb.board.clone();
    let from_file = gummiband_kicad::area::areas(&kb);
    let mut areas = from_file.areas.clone();
    let net_id = |name: &str| board.net_id(name).unwrap_or_else(|| panic!("no net named {name:?}"));
    let asked: Vec<NetId> = a.nets.iter().map(|n| net_id(n)).collect();
    // the areas of --pour
    let mut poured: Vec<(NetId, u8)> = Vec::new();
    if !a.pour.is_empty() {
        let outline = board.main_outline().expect("the board has no closed outline").clone();
        for &net in &asked {
            for l in &a.pour {
                let layer = board.layer_id(l).unwrap_or_else(|| panic!("no layer {l:?}"));
                if areas.iter().any(|x| x.net == net && x.layer == layer) {
                    continue;
                }
                let mut ar = Area::pour(net, layer, vec![outline.clone()]);
                ar.clearance = a.clearance;
                ar.min_width = a.min_width;
                ar.thermal_gap = a.thermal_gap;
                ar.spoke_width = a.spoke_width;
                ar.connect = a.connect;
                areas.push(ar);
                poured.push((net, layer));
            }
        }
    }
    let nets: Vec<NetId> = if asked.is_empty() {
        let mut v: Vec<NetId> = areas.iter().map(|x| x.net).collect();
        v.sort_unstable();
        v.dedup();
        v
    } else {
        asked
    };
    let opts = Options {
        step: a.step,
        pad_connect: from_file.pad_connect.clone(),
        no_pour: from_file.no_pour.clone(),
        ..Default::default()
    };

    // stitching: one via per island with a site, in rounds
    let mut stitched: Vec<write::Item> = Vec::new();
    let mut rounds: BTreeMap<String, Vec<usize>> = BTreeMap::new();
    if a.stitch.is_some() {
        for &net in &nets {
            let c = board.class_of(net).clone();
            let pitch = c.via_diameter + board.rules.hole_to_hole.max(c.clearance);
            for _round in 0..8 {
                let f = fill::fill_net(&board, &areas, net, &opts);
                let mut added = 0usize;
                let mut placed: Vec<gummiband_core::geom::Point> = Vec::new();
                // the via of each island, and the via that anchors the piece it lands on
                let sites = f.islands.iter().filter_map(|i| i.via.as_ref());
                for at in sites.flat_map(|v| v.anchor.into_iter().chain([v.at])) {
                    if placed.iter().any(|q| q.dist(at) < pitch) {
                        continue;
                    }
                    placed.push(at);
                    board.obstacles.push(Obstacle {
                        shape: Shape::Circle {
                            c: at,
                            r: c.via_diameter / 2.0,
                        },
                        layers: LayerSet::all(board.layers.len()),
                        net: Some(net),
                        kind: ObstacleKind::Via,
                        hole: Some(c.via_drill / 2.0),
                        clearance: c.clearance,
                        pin: None,
                        locked: true,
                    });
                    stitched.push(write::Item::Via {
                        at,
                        diameter: c.via_diameter,
                        drill: c.via_drill,
                        net,
                    });
                    added += 1;
                }
                rounds
                    .entry(board.nets[net as usize].name.clone())
                    .or_default()
                    .push(added);
                if added == 0 {
                    break;
                }
            }
        }
    }

    let mut report = serde_json::Map::new();
    for &net in &nets {
        let name = board.nets[net as usize].name.clone();
        let t0 = std::time::Instant::now();
        let f = fill::fill_net(&board, &areas, net, &opts);
        let secs = t0.elapsed().as_secs_f64();
        let mut by: BTreeMap<&str, Vec<&str>> = BTreeMap::new();
        for p in &f.pins {
            by.entry(reach_name(p.reach)).or_default().push(&p.name);
        }
        let count = |k: &str| by.get(k).map_or(0, |v| v.len());
        let unreached = f.unreached().count();
        let sites: Vec<f64> = f.pins.iter().map(|p| p.site_dist).filter(|d| *d >= 0.0).collect();
        let (mean, max) = if sites.is_empty() {
            (-1.0, -1.0)
        } else {
            (
                sites.iter().sum::<f64>() / sites.len() as f64,
                sites.iter().copied().fold(0.0, f64::max),
            )
        };
        let with_via = f.islands.iter().filter(|i| i.via.is_some()).count();
        let why = |w: fn(&NoVia) -> bool| f.islands.iter().filter(|i| i.no_via.as_ref().is_some_and(w)).count();
        println!(
            "{name}: {} pads, {} reached ({} by fill, {} by other copper), {} not ({} on islands, {} walled, {} no spoke, {} outside, {} other layer); \
             {} pieces of fill, area {:.1} mm2 + {:.1} mm2 apart; {} islands with pads ({} with a via site, {} too narrow, {} blocked, {} nothing opposite); \
             pin to via site mean {:.2} max {:.2} mm; step {:.3} mm, {:.2} s",
            f.pins.len(),
            f.pins.len() - unreached,
            count("fill"),
            count("copper"),
            unreached,
            count("island"),
            count("walled"),
            count("no-spoke"),
            count("outside"),
            count("other-layer"),
            f.pieces.len(),
            f.area_main + 0.0,
            f.area_rest + 0.0,
            f.islands.len(),
            with_via,
            why(|w| matches!(w, NoVia::TooNarrow { .. })),
            why(|w| matches!(w, NoVia::Blocked)),
            why(|w| matches!(w, NoVia::NothingOpposite)),
            mean,
            max,
            f.raster.step,
            secs,
        );
        if !a.quiet {
            for (l, layer) in board.layers.iter().enumerate() {
                let ps: Vec<_> = f.pieces.iter().filter(|p| p.layer as usize == l).collect();
                if f.fill[l].is_some() {
                    println!(
                        "  {}: {} pieces ({} of the area, {} floating), {:.1} mm2",
                        layer.name,
                        ps.len(),
                        ps.iter().filter(|p| p.main).count(),
                        ps.iter().filter(|p| p.floating).count(),
                        ps.iter().map(|p| p.area).sum::<f64>()
                    );
                }
            }
            for (k, i) in f.islands.iter().enumerate() {
                let layers: Vec<&str> = i
                    .layers
                    .iter()
                    .map(|&l| board.layers[l as usize].name.as_str())
                    .collect();
                let via = match (&i.via, &i.no_via) {
                    (Some(v), _) => format!(
                        "via at {:.3} {:.3} ({} to {}, {:.2} mm to spare){}",
                        v.at.x,
                        v.at.y,
                        board.layers[v.from as usize].name,
                        board.layers[v.to as usize].name,
                        v.room,
                        v.anchor.map_or(String::new(), |q| format!(
                            ", with a via at {:.3} {:.3} that joins that piece to the area",
                            q.x, q.y
                        ))
                    ),
                    (_, Some(NoVia::TooNarrow { widest, need })) => {
                        format!("no via: widest spot {widest:.2} mm, a via has {need:.2} mm")
                    }
                    (_, Some(NoVia::Blocked)) => "no via: other copper, a hole or the edge under every spot".into(),
                    (_, Some(NoVia::NothingOpposite)) => "no via: no fill of the area on another layer there".into(),
                    _ => String::new(),
                };
                println!(
                    "  island {k} on {}: {:.2} mm2, pads {}; {via}",
                    layers.join("+"),
                    i.area,
                    i.names.join(" ")
                );
            }
            for key in ["walled", "no-spoke", "outside", "other-layer"] {
                if let Some(v) = by.get(key) {
                    let shown: Vec<&str> = v.iter().take(40).copied().collect();
                    println!("  {key}: {}{}", shown.join(" "), if v.len() > 40 { " ..." } else { "" });
                }
            }
        }
        // against KiCad's fill, where the file has one
        let mut compares = Vec::new();
        for (l, layer) in board.layers.iter().enumerate() {
            let polys: Vec<_> = from_file
                .areas
                .iter()
                .zip(&from_file.filled)
                .filter(|(ar, _)| ar.net == net && ar.layer as usize == l)
                .flat_map(|(_, p)| p.iter().cloned())
                .collect();
            if polys.is_empty() {
                continue;
            }
            if let Some(c) = compare(&board, &f, l as u8, &polys) {
                println!(
                    "  against KiCad's fill on {}: both {:.1} mm2, model only {:.2} ({:.2} beyond the raster), KiCad only {:.2} ({:.2}); \
                     pieces {} / {}; pads touched by both {}, by none {}, model only {} {:?}, KiCad only {} {:?}",
                    layer.name,
                    c.both,
                    c.model_only,
                    c.model_only_deep,
                    c.kicad_only,
                    c.kicad_only_deep,
                    c.pieces_model,
                    c.pieces_kicad,
                    c.pads_both,
                    c.pads_none,
                    c.pads_model_only.len(),
                    c.pads_model_only,
                    c.pads_kicad_only.len(),
                    c.pads_kicad_only,
                );
                compares.push(c.json());
            }
        }
        if let Some(prefix) = &a.pgm {
            for l in 0..board.layers.len() {
                let safe: String = name
                    .chars()
                    .map(|c| if c.is_alphanumeric() { c } else { '_' })
                    .collect();
                pgm(&format!("{prefix}.{safe}.{}.pgm", board.layers[l].name), &f, l);
            }
        }
        report.insert(
            name.clone(),
            serde_json::json!({
                "fill": f,
                "kicad": compares,
                "stitch_rounds": rounds.get(&name),
                "seconds": secs,
            }),
        );
    }
    if let Some(path) = &a.json {
        std::fs::write(path, serde_json::to_string(&report).unwrap()).unwrap_or_else(|e| panic!("{e}"));
    }
    if let Some(out) = &a.stitch {
        let mut text = if stitched.is_empty() {
            kb.text.clone()
        } else {
            write::add_locked(&kb, &stitched)
        };
        for (i, &(net, layer)) in poured.iter().enumerate() {
            let before = text.len();
            let spec = write::ZoneSpec {
                net,
                layer,
                priority: 0,
                name: "",
                connect: match a.connect {
                    Connect::Solid => write::PadConnect::Solid,
                    Connect::ThermalTht => write::PadConnect::ThermalTht,
                    _ => write::PadConnect::Thermal,
                },
                spoke: a.spoke_width,
            };
            let outline = board.main_outline().unwrap().pts.clone();
            let with = write::add_zone_named(&kb, &text, &spec, &outline, 0xa7ea + i as u64);
            // the writer has fixed settings: put the ones asked for into the zone it added
            let at = with.len() - (with.len() - before) - 2;
            let (head, zone) = with.split_at(at.min(with.len()));
            let zone = zone
                .replace("(clearance 0.2)", &format!("(clearance {})", a.clearance))
                .replace("(min_thickness 0.2)", &format!("(min_thickness {})", a.min_width))
                .replace("(thermal_gap 0.3)", &format!("(thermal_gap {})", a.thermal_gap));
            text = format!("{head}{zone}");
        }
        std::fs::write(out, text).unwrap_or_else(|e| panic!("{e}"));
        println!(
            "wrote {} with {} areas and {} stitching vias (per round: {:?})",
            out.display(),
            poured.len(),
            stitched.len(),
            rounds
        );
    }
}
