mod svg;

use std::path::PathBuf;

use anyhow::Result;
use clap::{Parser, Subcommand};
use gummiband_core::board::ObstacleKind;
use gummiband_core::realize::Elem;
use gummiband_core::route::RouterConfig;
use gummiband_core::topo::SubstrateConfig;
use gummiband_kicad::{KeepCopper, ReadOptions, read_board, write};

#[derive(Parser)]
#[command(name = "gummiband", about = "Topological rubber band autorouter for KiCad")]
struct Cli {
    /// Worker threads (default: RAYON_NUM_THREADS or all cores).
    #[arg(long, global = true)]
    threads: Option<usize>,
    /// Treat these copper layers as planes (not routed; their zones stay), comma separated.
    /// Layers of KiCad type "power" are planes anyway.
    #[arg(long, global = true, value_delimiter = ',')]
    plane_layers: Vec<String>,
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Print a summary of a board.
    Info {
        board: PathBuf,
        /// Keep all existing tracks and vias (default: locked ones only).
        #[arg(long)]
        keep_all: bool,
    },
    /// Route a board and write the result.
    Route {
        board: PathBuf,
        #[arg(short, long)]
        output: PathBuf,
        /// Keep only locked tracks and vias as fixed copper (default: remove all).
        #[arg(long)]
        keep_locked: bool,
        /// Keep all existing tracks and vias as fixed copper and add to them (for completing
        /// a partly routed board or finishing a started track).
        #[arg(long)]
        keep_all: bool,
        /// With --keep-all --nets: if a connection finds no way, let the unlocked tracks of other
        /// nets within this many mm of its pin give way (they are routed again), growing in
        /// steps up to this radius. 0: nothing moves.
        #[arg(long, default_value_t = 0.0)]
        give: f64,
        /// With --keep-all --nets: the unlocked tracks of other nets are threaded into the
        /// topology, so a connection passes between them and they shift aside. What it does not
        /// touch stays as it is in the file.
        #[arg(long)]
        shove: bool,
        /// Route only these nets, comma separated (the others stay as they are).
        #[arg(long, value_delimiter = ',')]
        nets: Vec<String>,
        /// Portfolio size (router runs with different orders). A constant, so that results do
        /// not depend on the machine.
        #[arg(long, default_value_t = gummiband_core::DEFAULT_RUNS)]
        runs: usize,
        /// Via site grid pitch in mm.
        #[arg(long, default_value_t = 0.5)]
        via_pitch: f64,
        /// Via cost in mm of track (the search varies it around this value).
        #[arg(long, default_value_t = 3.0)]
        via_cost: f64,
        /// Weight of the preferred layer directions (horizontal and vertical in turn) for
        /// every third run of the portfolio; 0 = off.
        #[arg(long, default_value_t = 0.5)]
        layer_dirs: f64,
        /// Factor on the via cost in the pass that routes connections with vias again and
        /// keeps what has fewer vias (0 = off).
        #[arg(long, default_value_t = 4.0)]
        via_pass: f64,
        /// Weight of the pull between the nets of differential pairs (0 = off).
        #[arg(long, default_value_t = 0.0)]
        pair_pull: f64,
        /// Route differential pairs as one wide track and split it into two parallel tracks.
        #[arg(long)]
        pairs: bool,
        /// With --pairs: length difference left within a pair in mm; the shorter track gets
        /// bumps down to this (a `skew` rule in the .kicad_dru may ask for less).
        #[arg(long, default_value_t = 0.1)]
        pair_skew: f64,
        /// With --pairs: target differential impedance in ohm. Sets the track width per layer
        /// from the board's stackup; the gap stays.
        #[arg(long)]
        pair_zdiff: Option<f64>,
        /// Generations of the evolutionary order search after the portfolio.
        #[arg(long, default_value_t = 3)]
        generations: usize,
        /// Generations that select by clearance violations after the geometry step (slower).
        #[arg(long, default_value_t = 0)]
        geometry_generations: usize,
        /// Write a debug SVG.
        #[arg(long)]
        svg: Option<PathBuf>,
        /// Draw the triangulation into the SVG.
        #[arg(long)]
        svg_tris: bool,
        /// Write one SVG per generation (best candidate) into this folder, and an animated GIF
        /// if ImageMagick is available.
        #[arg(long)]
        frames: Option<PathBuf>,
        /// Write a machine readable JSON report (stats, unrouted connections, violations,
        /// portfolio) for scripts and AI agents.
        #[arg(long)]
        report: Option<PathBuf>,
        /// Let the geometry generations shift parts next to clearance problems (locked
        /// footprints, connectors, modules and mounting holes stay).
        #[arg(long)]
        move_parts: bool,
        /// References that never move under --move-parts, comma separated.
        #[arg(long, value_delimiter = ',')]
        fixed: Vec<String>,
        /// Track width per net from a JSON file {"net name": width_mm, ...} (e.g. the widths
        /// of an existing routing, see bench/net_widths.py).
        #[arg(long)]
        net_widths: Option<PathBuf>,
        /// Switch on KiCad's teardrops for vias and pads in the output; KiCad builds them when
        /// it fills the zones.
        #[arg(long)]
        teardrops: bool,
        /// Routing priority per net from a JSON file {"net name": priority, ...}; lower routes
        /// first, unnamed nets keep their class priority.
        #[arg(long)]
        net_priorities: Option<PathBuf>,
        /// Print a "progress {json}" line after every phase of the search (best candidate so
        /// far, elapsed seconds), for the KiCad plugin and other front ends.
        #[arg(long)]
        progress: bool,
    },
    /// Remove tracks, vias and outer copper zones (benchmark preparation).
    Strip {
        board: PathBuf,
        #[arg(short, long)]
        output: PathBuf,
        /// Keep locked tracks and vias.
        #[arg(long)]
        keep_locked: bool,
        /// Keep copper zones on the routing layers.
        #[arg(long)]
        keep_zones: bool,
    },
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    if let Some(n) = cli.threads {
        rayon::ThreadPoolBuilder::new().num_threads(n.max(1)).build_global()?;
    }
    match cli.cmd {
        Cmd::Info { board, keep_all } => {
            let keep = if keep_all { KeepCopper::All } else { KeepCopper::Locked };
            let kb = read_board(
                &board,
                &ReadOptions {
                    keep,
                    plane_layers: cli.plane_layers.clone(),
                    ..Default::default()
                },
            )?;
            let b = &kb.board;
            let bb = b.bbox();
            println!(
                "outline: {} polygons, {:.2} x {:.2} mm",
                b.outline.len(),
                bb.width(),
                bb.height()
            );
            for (i, l) in b.layers.iter().enumerate() {
                println!("layer {i}: {}{}", l.name, if l.is_plane { " (plane)" } else { "" });
            }
            let count = |k: fn(&ObstacleKind) -> bool| b.obstacles.iter().filter(|o| k(&o.kind)).count();
            println!(
                "obstacles: {} (pads {}, tracks {}, vias {}, keepouts {}, holes {}, graphics {})",
                b.obstacles.len(),
                count(|k| matches!(k, ObstacleKind::Pad)),
                count(|k| matches!(k, ObstacleKind::Track)),
                count(|k| matches!(k, ObstacleKind::Via)),
                count(|k| matches!(k, ObstacleKind::Keepout { .. })),
                count(|k| matches!(k, ObstacleKind::Hole)),
                count(|k| matches!(k, ObstacleKind::Graphic)),
            );
            let routable = b.routable_nets().count();
            let conns: usize = b.routable_nets().map(|n| b.nets[n as usize].pins.len() - 1).sum();
            println!(
                "pins: {}, nets: {} ({} with 2+ pins, {} connections)",
                b.pins.len(),
                b.nets.len(),
                routable,
                conns
            );
            println!("planes:");
            for p in &b.planes {
                let r = p.outline.bbox();
                println!(
                    "  {} on {}: {:.1} x {:.1} mm",
                    b.nets[p.net as usize].name,
                    b.layers[p.layer as usize].name,
                    r.width(),
                    r.height()
                );
            }
            println!("classes:");
            for (i, c) in b.classes.iter().enumerate() {
                let nets: Vec<&str> = b
                    .nets
                    .iter()
                    .filter(|n| n.class as usize == i && n.pins.len() > 1)
                    .map(|n| n.name.as_str())
                    .collect();
                println!(
                    "  {} (prio {}, w {}, clr {}, via {}/{}): {} nets {}",
                    c.name,
                    c.priority,
                    c.track_width,
                    c.clearance,
                    c.via_diameter,
                    c.via_drill,
                    nets.len(),
                    if nets.len() <= 8 { nets.join(" ") } else { String::new() }
                );
            }
            println!("rules: {:?}", b.rules);
            println!("custom track clearances: {} net pairs", b.pair_clearance.len());
        }
        Cmd::Route {
            board,
            output,
            keep_locked,
            keep_all,
            give,
            shove,
            nets,
            runs,
            via_pitch,
            via_cost,
            layer_dirs,
            pair_pull,
            via_pass,
            pairs,
            pair_skew,
            pair_zdiff,
            generations,
            geometry_generations,
            svg,
            svg_tris,
            report,
            frames,
            move_parts,
            fixed,
            progress,
            net_widths,
            teardrops,
            net_priorities,
        } => {
            if shove && (!keep_all || nets.is_empty() || move_parts) {
                anyhow::bail!("--shove needs --keep-all and --nets, and fixed parts");
            }
            let keep = if keep_all {
                KeepCopper::All
            } else if keep_locked {
                KeepCopper::Locked
            } else {
                KeepCopper::None
            };
            let kb = read_board(
                &board,
                &ReadOptions {
                    keep,
                    plane_layers: cli.plane_layers.clone(),
                    move_parts,
                    fixed,
                    ..Default::default()
                },
            )?;
            let mut kb = kb;
            if !nets.is_empty() {
                for n in &nets {
                    if kb.board.net_id(n).is_none() {
                        anyhow::bail!("no net named {n:?}");
                    }
                }
            }
            if let Some(path) = &net_widths {
                let map: std::collections::HashMap<String, f64> =
                    serde_json::from_str(&std::fs::read_to_string(path)?)?;
                let mut n = 0;
                for (name, w) in map {
                    if let Some(net) = kb.board.net_id(&name) {
                        kb.board.set_net_width(net, w);
                        n += 1;
                    }
                }
                println!("net widths: {n} nets");
            }
            if let Some(path) = &net_priorities {
                let map: std::collections::HashMap<String, i32> =
                    serde_json::from_str(&std::fs::read_to_string(path)?)?;
                for n in kb.board.nets.iter_mut() {
                    if let Some(&p) = map.get(&n.name) {
                        n.priority = p;
                    }
                }
            }
            if let (true, Some(z)) = (pairs, pair_zdiff) {
                gummiband_core::pairs::apply_zdiff(&mut kb.board, z);
            }
            // debugging: GB_SHIFT="R8:0.3,0;C4:0,-0.2" routes the board with these parts shifted
            let shifted_in;
            let b = match std::env::var("GB_SHIFT") {
                Ok(spec) => {
                    let off: std::collections::HashMap<String, gummiband_core::geom::Point> = spec
                        .split(';')
                        .filter_map(|e| {
                            let (r, d) = e.split_once(':')?;
                            let (x, y) = d.split_once(',')?;
                            Some((
                                r.to_string(),
                                gummiband_core::geom::pt(x.parse().ok()?, y.parse().ok()?),
                            ))
                        })
                        .collect();
                    shifted_in = kb.board.with_offsets(&off);
                    &shifted_in
                }
                Err(_) => &kb.board,
            };
            if move_parts {
                println!("movable parts: {}", b.movable.len());
            }
            let t0 = std::time::Instant::now();
            let sub_cfg = SubstrateConfig {
                via_pitch,
                ..Default::default()
            };
            let router_cfg = RouterConfig {
                via_cost,
                layer_dirs,
                pair_pull,
                via_pass,
                ..Default::default()
            };
            let search_cfg = gummiband_core::SearchConfig {
                population: runs,
                generations,
                geometry_generations,
                frames: frames.is_some(),
                progress: progress.then(|| {
                    gummiband_core::ProgressFn(std::sync::Arc::new(move |p: &gummiband_core::Progress| {
                        use std::io::Write;
                        let mut v = serde_json::to_value(p).unwrap_or_default();
                        v["seconds"] = serde_json::json!((t0.elapsed().as_secs_f64() * 10.0).round() / 10.0);
                        let mut out = std::io::stdout().lock();
                        let _ = writeln!(out, "progress {v}");
                        let _ = out.flush();
                    }))
                }),
            };
            // the nets asked for, and the nets that give way when a connection finds no way
            let mut given: std::collections::BTreeSet<String> = Default::default();
            let mut radius = 0.0;
            let mut pair_reports = Vec::new();
            let (routed_board, sub, res) = loop {
                let mut board = b.only_nets(&nets, &given);
                if shove {
                    board.soften(&nets);
                }
                let (sub, res) = if pairs {
                    let pair_cfg = gummiband_core::pairs::PairConfig {
                        skew: pair_skew,
                        zdiff: pair_zdiff,
                    };
                    let (sub, res, reports) =
                        gummiband_core::pairs::route_pairs(&board, &sub_cfg, &router_cfg, &search_cfg, &pair_cfg);
                    pair_reports = reports;
                    (sub, res)
                } else {
                    gummiband_core::route_board(&board, &sub_cfg, &router_cfg, &search_cfg)
                };
                if res.stats.unrouted.is_empty() || nets.is_empty() || radius >= give {
                    break (board, sub, res);
                }
                radius = (radius * 2.0).max(2.0).min(give);
                // around what was asked for; a net that gave way must not widen the circle
                let asked: Vec<String> = res
                    .stats
                    .unrouted
                    .iter()
                    .filter(|u| nets.iter().any(|n| u.starts_with(&format!("{n} from "))))
                    .cloned()
                    .collect();
                if asked.is_empty() {
                    break (board, sub, res);
                }
                let more = b.nets_near_pins(&asked, radius);
                let before = given.len();
                given.extend(more.into_iter().filter(|n| !nets.contains(n)));
                println!("give way within {radius} mm: {} nets", given.len());
                if given.len() == before && radius >= give {
                    break (board, sub, res);
                }
            };
            let b = &routed_board;
            let dt = t0.elapsed();
            // the geometry belongs to the board with the shifted parts
            let shifted;
            let b = if res.offsets.is_empty() {
                b
            } else {
                let mut moved: Vec<_> = res.offsets.iter().collect();
                moved.sort_by(|a, b| a.0.cmp(b.0));
                for (f, d) in moved {
                    println!("moved {f} by {:.2}, {:.2} mm", d.x, d.y);
                }
                shifted = b.with_offsets(&res.offsets);
                &shifted
            };
            println!(
                "substrate: {} vertices, {} triangles, {} via sites",
                sub.m.iter().map(|m| m.verts.len()).sum::<usize>(),
                sub.m.iter().map(|m| m.tris.len()).sum::<usize>(),
                sub.shared.sites.len()
            );
            for r in &res.portfolio {
                println!(
                    "  gen {:2} seed {:2} fanout_first {:5} via cost {:.1}: {}/{} routed, {} vias, {:.0} mm{}",
                    r.generation,
                    r.seed,
                    r.fanout_first,
                    r.via_cost,
                    r.routed,
                    r.conns,
                    r.vias,
                    r.length,
                    r.violations
                        .map(|v| format!(", {v} violations ({} shorts)", r.shorts.unwrap_or(0)))
                        .unwrap_or_default()
                );
            }
            let s = &res.stats;
            println!(
                "best: seed {} {}/{} routed, {} vias, {:.0} mm, {:.2} s",
                s.seed,
                s.routed,
                s.conns,
                s.vias,
                s.length,
                dt.as_secs_f64()
            );
            let bound = gummiband_core::length_lower_bound(b);
            println!(
                "  length lower bound (signal nets, Steiner): {:.0} mm; signal length is at most {:.0}% above the optimum",
                bound,
                // plane fan-outs are not in the bound, so this ratio is an upper estimate
                (s.length / bound - 1.0) * 100.0
            );
            for u in s.unrouted.iter().take(40) {
                println!("  unrouted: {u}");
            }
            if let Ok(near) = std::env::var("GB_NEAR") {
                let v: Vec<f64> = near.split(',').filter_map(|x| x.parse().ok()).collect();
                let p = gummiband_core::geom::pt(v[0], v[1]);
                for t in &res.geometry.tracks {
                    if let Some(q) = t.pts.iter().find(|q| q.dist(p) < v.get(2).copied().unwrap_or(0.3)) {
                        println!(
                            "near: {} layer {} vertex {:.3},{:.3} ends {:.3},{:.3} -> {:.3},{:.3}",
                            b.nets[t.net as usize].name,
                            b.layers[t.layer as usize].name,
                            q.x,
                            q.y,
                            t.pts[0].x,
                            t.pts[0].y,
                            t.pts.last().unwrap().x,
                            t.pts.last().unwrap().y
                        );
                    }
                }
            }
            let pairs = b.diff_pairs().len();
            if pairs > 0 {
                let (c, t) = gummiband_core::pair_coupling(b, &res.geometry, 1.0);
                println!(
                    "diff pairs: {pairs}, {:.0} of {:.0} mm coupled ({:.0}%)",
                    c,
                    t,
                    100.0 * c / t.max(1e-9)
                );
            }
            for r in &pair_reports {
                println!(
                    "  pair {} / {}: {}, width {:.3} gap {:.3}, {:.0} ohm, {:.2} / {:.2} mm, skew {:.3} mm, {} vias, {} swaps {}",
                    r.p,
                    r.n,
                    if r.wide { "wide" } else { "single" },
                    r.width,
                    r.gap,
                    r.zdiff,
                    r.length_p,
                    r.length_n,
                    r.skew,
                    r.vias,
                    r.swaps,
                    r.note
                );
            }
            let viol = gummiband_core::check::check(b, &res.geometry);
            println!("clearance check: {} violations", viol.len());
            let mut kinds = std::collections::BTreeMap::new();
            for v in &viol {
                *kinds
                    .entry(v.what.split(' ').next().unwrap_or("").to_string())
                    .or_insert(0) += 1;
            }
            println!("  {kinds:?}");
            if std::env::var("GB_VIOL").is_ok() {
                let mut hist = [0usize; 6];
                for v in &viol {
                    let r = (v.actual / v.required).clamp(0.0, 0.999);
                    hist[(r * 6.0) as usize] += 1;
                }
                println!("  ratio histogram (0..1 in sixths): {hist:?}");
                let mut def = [0usize; 5];
                for v in &viol {
                    let d = v.required - v.actual;
                    def[[0.002, 0.01, 0.03, 0.1, f64::INFINITY]
                        .iter()
                        .position(|&x| d < x)
                        .unwrap()] += 1;
                }
                println!("  deficit <2um <10um <30um <100um more: {def:?}");
                let severe: Vec<_> = viol.iter().filter(|v| v.actual < v.required * 0.5).collect();
                for v in severe.iter().step_by((severe.len() / 12).max(1)) {
                    println!(
                        "  {} at {:.3},{:.3}: {:.4} < {:.4}",
                        v.what, v.at.x, v.at.y, v.actual, v.required
                    );
                }
            }
            if let Some(dir) = &frames {
                std::fs::create_dir_all(dir)?;
                let mut pngs = Vec::new();
                for (i, (label, geo, relaxed)) in res.frames.iter().enumerate() {
                    let v = if *relaxed {
                        gummiband_core::check::check(b, geo)
                    } else {
                        Vec::new()
                    };
                    let svg_text = svg::render(b, None, geo, &v, false).replace(
                        "</svg>",
                        &format!(
                            r##"<text x="{}" y="{}" font-size="2.2" font-family="Helvetica" fill="#eee">{}</text></svg>"##,
                            b.bbox().min.x,
                            b.bbox().min.y - 0.6,
                            label
                        ),
                    );
                    let f = dir.join(format!("frame-{i:03}.svg"));
                    std::fs::write(&f, svg_text)?;
                    let png = dir.join(format!("frame-{i:03}.png"));
                    let ok = std::process::Command::new("rsvg-convert")
                        .args(["-w", "1200", "-o"])
                        .arg(&png)
                        .arg(&f)
                        .status()
                        .map(|s| s.success())
                        .unwrap_or(false);
                    if ok {
                        pngs.push(png);
                    }
                }
                if !pngs.is_empty() {
                    let gif = dir.join("evolution.gif");
                    let _ = std::process::Command::new("magick")
                        .args(["-delay", "120", "-loop", "0"])
                        .args(&pngs)
                        .arg(&gif)
                        .status();
                    println!("frames: {} images, animation {}", pngs.len(), gif.display());
                }
            }
            if let Some(path) = svg {
                std::fs::write(&path, svg::render(b, Some(&sub), &res.geometry, &viol, svg_tris))?;
            }
            let changes = gummiband_core::import::soft_changes(b, &res.geometry);
            if shove {
                println!(
                    "soft copper: {} tracks, {} vias and branch points; moved {} tracks of {} nets, at most {:.3} mm",
                    b.soft.chains.len(),
                    b.soft.nodes.len(),
                    changes.chains.iter().filter(|c| **c).count(),
                    changes.nets.len(),
                    changes.shift
                );
            }
            let rests = |origin: Option<u32>, list: &[bool]| origin.is_some_and(|o| !list[o as usize]);
            let mut items = Vec::new();
            for t in &res.geometry.tracks {
                if rests(t.origin, &changes.chains) {
                    continue;
                }
                for e in &t.elems {
                    items.push(match *e {
                        Elem::Line(a, b) => write::Item::Track {
                            width: t.piece_width(a, b),
                            a,
                            b,
                            layer: t.layer,
                            net: t.net,
                        },
                        Elem::Arc(a, mid, b) => write::Item::Arc {
                            width: t.piece_width(a, b),
                            a,
                            mid,
                            b,
                            layer: t.layer,
                            net: t.net,
                        },
                    });
                }
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
            let opts = write::StripOptions {
                unlocked: !keep_all,
                locked: !keep_locked && !keep_all,
                zone_layers: vec![],
                nets: given.iter().cloned().collect(),
                items: changes.strip.clone(),
            };
            let ranges = write::strip_ranges(&kb, &opts);
            let text = write::patch(&kb, &ranges, &items);
            let text = if res.offsets.is_empty() {
                text
            } else {
                write::move_footprints(&text, &res.offsets, &kb.board.blocks)?
            };
            let text = if teardrops {
                write::enable_teardrops(&text)?
            } else {
                text
            };
            std::fs::write(&output, text)?;
            if teardrops {
                track_end_teardrops(&board, &output)?;
            }
            println!("wrote {} items to {}", items.len(), output.display());
            if let Some(path) = report {
                let st = &res.stats;
                let json = serde_json::json!({
                    "board": board.display().to_string(),
                    "output": output.display().to_string(),
                    "seconds": dt.as_secs_f64(),
                    "connections": st.conns,
                    "routed": st.routed,
                    "unrouted": st.unrouted,
                    "pairs": pair_reports,
                    "vias": st.vias,
                    "length_mm": st.length,
                    "length_lower_bound_mm": gummiband_core::length_lower_bound(b),
                    "items_written": items.len(),
                    "violations": viol.iter().map(|v| serde_json::json!({
                        "what": v.what,
                        "x": v.at.x,
                        "y": v.at.y,
                        "actual_mm": v.actual,
                        "required_mm": v.required,
                        "nets": [b.nets[v.nets.0 as usize].name.clone(), v.nets.1.map(|n| b.nets[n as usize].name.clone())],
                    })).collect::<Vec<_>>(),
                    "portfolio": res.portfolio.iter().map(|r| serde_json::json!({
                        "generation": r.generation,
                        "seed": r.seed,
                        "fanout_first": r.fanout_first,
                        "routed": r.routed,
                        "connections": r.conns,
                        "vias": r.vias,
                        "length_mm": r.length,
                        "violations": r.violations,
                        "shorts": r.shorts,
                    })).collect::<Vec<_>>(),
                });
                std::fs::write(&path, serde_json::to_string_pretty(&json)?)?;
            }
        }
        Cmd::Strip {
            board,
            output,
            keep_locked,
            keep_zones,
        } => {
            let kb = read_board(
                &board,
                &ReadOptions {
                    keep: KeepCopper::All,
                    plane_layers: cli.plane_layers.clone(),
                    ..Default::default()
                },
            )?;
            let zone_layers = if keep_zones {
                vec![]
            } else {
                kb.board
                    .layers
                    .iter()
                    .filter(|l| !l.is_plane)
                    .map(|l| l.name.clone())
                    .collect()
            };
            let opts = write::StripOptions {
                unlocked: true,
                locked: !keep_locked,
                zone_layers,
                nets: Vec::new(),
                items: Vec::new(),
            };
            let ranges = write::strip_ranges(&kb, &opts);
            let text = write::patch(&kb, &ranges, &[]);
            std::fs::write(&output, text)?;
            println!("removed {} items, wrote {}", ranges.len(), output.display());
        }
    }
    Ok(())
}

/// Teardrops at track width changes are a project setting: the output's .kicad_pro (a copy of
/// the input's if it has none yet) switches them on. The input's project stays as it is.
fn track_end_teardrops(input: &std::path::Path, output: &std::path::Path) -> Result<()> {
    let target = output.with_extension("kicad_pro");
    let source = if target.exists() {
        target.clone()
    } else {
        input.with_extension("kicad_pro")
    };
    if target == input.with_extension("kicad_pro") {
        return Ok(());
    }
    let Ok(text) = std::fs::read_to_string(&source) else {
        return Ok(());
    };
    let mut pro: serde_json::Value = serde_json::from_str(&text)?;
    let design = &mut pro["board"]["design_settings"];
    if let Some(o) = design["teardrop_options"].as_array_mut().and_then(|a| a.first_mut()) {
        o["td_ontrackend"] = serde_json::json!(true);
    }
    if let Some(list) = design["teardrop_parameters"].as_array_mut() {
        for p in list.iter_mut().filter(|p| p["td_target_name"] == "td_track_end") {
            p["td_enabled"] = serde_json::json!(true);
        }
    }
    std::fs::write(&target, serde_json::to_string_pretty(&pro)?)?;
    if let Ok(dru) = std::fs::read(input.with_extension("kicad_dru")) {
        let d = output.with_extension("kicad_dru");
        if !d.exists() {
            std::fs::write(d, dru)?;
        }
    }
    Ok(())
}
