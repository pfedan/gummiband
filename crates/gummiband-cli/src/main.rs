mod svg;

use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use gummiband_core::board::ObstacleKind;
use gummiband_core::realize::Elem;
use gummiband_core::route::RouterConfig;
use gummiband_core::topo::SubstrateConfig;
use gummiband_kicad::{KeepCopper, ReadOptions, read_board, write};

/// Track angles of a routed result.
#[derive(Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum Angles {
    /// Any-angle tracks with arcs, as the rubber bands lie.
    Any,
    /// Conventional 45 degree routing.
    #[value(name = "45")]
    Deg45,
}

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
#[allow(clippy::large_enum_variant)] // parsed once
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
        /// steps up to this radius; the try with the fewest open connections is kept. 0: nothing
        /// gives way [default: 8 with --keep-all --nets, else 0]
        #[arg(long)]
        give: Option<f64>,
        /// With --keep-all --nets: the unlocked tracks of other nets are threaded into the
        /// topology, so a connection passes between them and they shift aside. What it does not
        /// touch stays as it is in the file. On by default with --keep-all --nets (not with
        /// --move-parts); this flag insists on it.
        #[arg(long, conflicts_with = "no_shove")]
        shove: bool,
        /// With --keep-all --nets: the tracks of other nets stay where they are (no shove).
        #[arg(long)]
        no_shove: bool,
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
        /// Hints from local patterns of the board, comma separated: `shares` (pins of a plane net
        /// next to each other share one fan-out via), `decaps` (a decoupling capacitor is
        /// connected to its IC pin first, its via serves both), `rows` (the open end re-lays the
        /// connections of a pin row in pin order), `pairs` (differential pairs before the other
        /// signal connections, the longest first), `groups` (the connections between two parts
        /// as one block in pin order, the longest block first; `groups-short` puts the shortest
        /// first, `groups-inline` leaves each block at the place of its connections), `all` or
        /// `none`. `gummiband info` lists the patterns it finds. When to use which: see
        /// docs/guide/options.md.
        #[arg(long, default_value = "none")]
        patterns: String,
        /// Factor on the via cost in the pass that routes connections with vias again and
        /// keeps what has fewer vias (0 = off).
        #[arg(long, default_value_t = 4.0)]
        via_pass: f64,
        /// Weight of the pull between the nets of differential pairs (0 = off).
        #[arg(long, default_value_t = 0.0)]
        pair_pull: f64,
        /// Experimental: route differential pairs as one wide track and split it into two parallel
        /// tracks. Check the result with the KiCad DRC; on dense boards it costs connections.
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
        /// With --pairs: couple every pair recognised by name. Without it only pairs that a
        /// `diff_pair_gap` rule or a net class of their own marks as differential pairs.
        #[arg(long)]
        pair_all: bool,
        /// Experimental, with --pairs: let the wide track end at a point just outside the pads,
        /// with two single tracks to the pads (breakout zones).
        #[arg(long)]
        breakouts: bool,
        /// Experimental, with --pairs: two vias side by side are one via pair, a capsule that
        /// turns with the wide track, in place of the circle around both.
        #[arg(long)]
        via_pairs: bool,
        /// Rounds of negotiated routing from the start, in a second search next to the plain one.
        /// Connections may overfill edges and cross for a rising price, see docs/ARCHITECTURE.md.
        /// With 0 the search negotiates on its own when the plain one leaves a few connections
        /// open (GB_AUTO_NEGOTIATE=0 switches that off).
        #[arg(long, default_value_t = 0)]
        negotiate: usize,
        /// Generations of the evolutionary order search after the portfolio.
        #[arg(long, default_value_t = 3)]
        generations: usize,
        /// Generations that select by clearance violations after the geometry step. Each costs
        /// about one routing pass; 0 is fastest and leaves many more violations.
        #[arg(long, default_value_t = 3)]
        geometry_generations: usize,
        /// Seed of the search (default: GB_EVO_SEED, then a constant). The same seed gives the
        /// same result, another one a different try.
        #[arg(long)]
        seed: Option<u64>,
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
        /// Let the geometry generations shift parts next to clearance problems and open
        /// connections; the routed copper follows like rubber bands (locked footprints,
        /// connectors, modules and mounting holes stay). On by default when the whole board is
        /// routed (not with --keep-all); this flag insists on it.
        #[arg(long, conflicts_with = "no_move_parts")]
        move_parts: bool,
        /// Parts stay where they are.
        #[arg(long)]
        no_move_parts: bool,
        /// References that never move under --move-parts and --placements, comma separated.
        #[arg(long, value_delimiter = ',')]
        fixed: Vec<String>,
        /// Try this many placements from the netlist (see `gummiband place`), route each one with
        /// these settings and keep the best: fewest open connections, then clearance violations,
        /// vias and length. Parts that are not locked or mechanical get new positions.
        #[arg(long, default_value_t = 0)]
        placements: usize,
        /// First seed of --placements.
        #[arg(long, default_value_t = 1)]
        placement_seed: u64,
        /// Before the routing, parts that sit in the band of a group of tracks between two parts
        /// (ICs, connectors sharing at least two signal nets) and are not on its nets get new
        /// positions with `place --corridors`; everything else stays. Pairs with `--patterns
        /// groups`. Experimental.
        #[arg(long)]
        clear_corridors: bool,
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
        /// Keep improving after the normal search until a line arrives on stdin or stdin closes,
        /// then finish the best result. Mutants take a few connections around one spot out and
        /// lay them again, the rest of the result stays; when that stops helping, bolder side
        /// branches take over. Results that differ in vias and length are written next to the
        /// output as alt1, alt2, ... No placement rounds, and no plain search next to --negotiate.
        /// Finishing nets, the tries of --give run first and the one kept goes on open ended.
        #[arg(long)]
        until_stop: bool,
        /// With --until-stop: what a via is worth in mm of track when the open end judges its
        /// results. Lower gives more vias and shorter tracks, higher fewer vias and longer ones.
        #[arg(long, default_value_t = gummiband_core::DEFAULT_OPEN_VIA_COST)]
        open_via_cost: f64,
        /// Give the finished copper this reserve in mm on top of the clearances where there is
        /// room, instead of leaving it at the limit [default: 0.03 with --until-stop, but not
        /// when finishing nets with --keep-all --nets, else 0]
        #[arg(long)]
        air: Option<f64>,
        /// At the end tracks of the default net class push aside what can give way and grow up
        /// to this width in mm wherever there is room, with a taper at every change; routing
        /// still works with the class width. A class with a track width rule in the .kicad_dru
        /// that has an `opt` value grows to that one, also without this flag
        #[arg(long, default_value_t = 0.0)]
        prefer_width: f64,
        /// Track angles of the result: `any` keeps the tight any-angle tracks with arcs, `45`
        /// turns them into conventional 45 degree routing as a last step (straight pieces in
        /// eight directions, chamfered corners). The step never adds a clearance violation;
        /// where a 45 degree path does not fit, a piece keeps its angle.
        #[arg(long, value_enum, default_value_t = Angles::Any)]
        angles: Angles,
        /// Leave the lengths as routed: no meanders for the length and skew rules of the
        /// .kicad_dru (without such rules nothing is tuned anyway)
        #[arg(long)]
        no_tune: bool,
        /// Highest meander bump in mm (centre line above the original line)
        #[arg(long, default_value_t = 1.0)]
        tune_amplitude: f64,
        /// Air between neighbouring meander legs in track widths (at least the clearance)
        #[arg(long, default_value_t = 2.0)]
        tune_spacing: f64,
    },
    /// Place parts from the netlist: locked footprints, connectors, modules, mounting holes,
    /// buttons and test points stay, everything else gets a new position. Its current position
    /// does not matter. --only, --outside and --all-unlocked change which parts move. Route the
    /// result with `route`.
    Place {
        board: PathBuf,
        #[arg(short, long)]
        output: PathBuf,
        /// Place only these references, comma separated; everything else stays.
        #[arg(long, value_delimiter = ',', conflicts_with_all = ["outside", "all_unlocked"])]
        only: Vec<String>,
        /// Place only the parts outside the board outline.
        #[arg(long, conflicts_with = "all_unlocked")]
        outside: bool,
        /// Place every unlocked part, connectors, modules and mounting holes too.
        #[arg(long)]
        all_unlocked: bool,
        /// References that stay as well, comma separated.
        #[arg(long, value_delimiter = ',')]
        fixed: Vec<String>,
        /// References released although they count as mechanical, comma separated.
        #[arg(long, value_delimiter = ',')]
        free: Vec<String>,
        /// Another seed gives another set of starts.
        #[arg(long, default_value_t = 1)]
        seed: u64,
        /// Independent runs in parallel; the one with the fewest crossings is kept. The crossings
        /// hardly predict the routed result, so for another attempt change --seed instead.
        #[arg(long, default_value_t = 1)]
        starts: usize,
        /// Annealing moves per run.
        #[arg(long, default_value_t = 3_000_000)]
        iters: usize,
        /// Spacing between courtyards in mm.
        #[arg(long, default_value_t = 0.3)]
        gap: f64,
        /// Courtyards keep this distance from the outline in mm.
        #[arg(long, default_value_t = 0.5)]
        margin: f64,
        /// Extra room around parts with five or more pads in mm, for their escape.
        #[arg(long, default_value_t = 0.3)]
        halo: f64,
        /// Use the schematic beyond the netlist: sheets and clusters around ICs stay together,
        /// inductors sit at their converter, decoupling capacitors prefer an IC of their sheet,
        /// wide nets and switch nodes count their full length.
        #[arg(long)]
        semantic: bool,
        /// Keep a band free for every group of tracks between two parts (ICs and connectors that
        /// share at least two signal nets, a bus, SPI, I2C): other parts pay for the area of the
        /// band they cover. Experimental.
        #[arg(long)]
        corridors: bool,
        /// Room for the escape per side of a part with five or more pads, from its signal pins
        /// on that side (their tracks with clearance, shared by the routing layers, at most 3
        /// mm), instead of the same halo all round. Experimental.
        #[arg(long)]
        escape_halo: bool,
        /// Parts with 4 or more pads (ICs, connectors) stay within this distance of where they
        /// sit (mm, 0 = free): a local change instead of a new placement, e.g. with --only.
        #[arg(long, default_value_t = 0.0)]
        leash: f64,
        /// Place on both sides: each side has its own room; a through hole part takes its holes
        /// on the other side (the room under its body stays free). SMD parts with at most
        /// --back-max-pads pads may change sides; they pay a little for the
        /// back and for a net that then needs a via, so the back takes what does not fit on the
        /// front or sits better there (a decoupling capacitor under its IC). Off: one room for
        /// both sides and every part keeps its side.
        #[arg(long)]
        two_sided: bool,
        /// With --two-sided: parts with more pads keep their side.
        #[arg(long, default_value_t = gummiband_place::auto::BACK_MAX_PADS)]
        back_max_pads: usize,
        /// With --two-sided: cost of a part on the back in mm of wire. A part goes to the back
        /// if the front has no room for it or the back saves at least this much (0: both sides
        /// alike).
        #[arg(long, default_value_t = gummiband_place::auto::BACK_COST)]
        back_cost: f64,
        /// With --two-sided: these parts go on the front, whatever their pads (comma separated).
        #[arg(long, value_delimiter = ',')]
        on_front: Vec<String>,
        /// With --two-sided: these parts go on the back, whatever their pads (comma separated).
        #[arg(long, value_delimiter = ',')]
        on_back: Vec<String>,
        /// Write a machine readable JSON report.
        #[arg(long)]
        report: Option<PathBuf>,
    },
    /// Turn the unlocked tracks of a routed board into 45 degree tracks, without routing again.
    /// Vias, track ends, pads and locked tracks stay where they are; every new piece keeps the
    /// clearances, and a piece that finds no room on the grid keeps its angle.
    Angles45 {
        board: PathBuf,
        #[arg(short, long)]
        output: PathBuf,
        /// Only the tracks of these nets, comma separated (default: all nets).
        #[arg(long, value_delimiter = ',')]
        nets: Vec<String>,
        /// Write a machine readable JSON report.
        #[arg(long)]
        report: Option<PathBuf>,
    },
    /// Move a part on a routed board and let the tracks around it follow like rubber bands:
    /// its own tracks stretch, those of other nets give way within their clearances. The part
    /// moves in small steps and stops where the copper cannot give way any more, or where its
    /// courtyard meets another part or the board edge. Nothing is routed again.
    MovePart {
        board: PathBuf,
        /// Footprint reference, e.g. C12.
        reference: String,
        /// New position in mm (KiCad board coordinates).
        x: f64,
        y: f64,
        #[arg(short, long)]
        output: PathBuf,
        /// Write a machine readable JSON report.
        #[arg(long)]
        report: Option<PathBuf>,
        /// Draw the result with its clearance problems into an SVG.
        #[arg(long)]
        svg: Option<PathBuf>,
    },
    /// Measure a routed board as KiCad's DRC does: net lengths with vias and pad-to-die, the
    /// coupled length of differential pairs, and the length, skew and uncoupled rules of the
    /// `.kicad_dru`.
    Lengths {
        board: PathBuf,
        /// Nets to list with their lengths (glob patterns), in addition to the rules.
        #[arg(long, value_delimiter = ',')]
        nets: Vec<String>,
        /// Print JSON instead of text.
        #[arg(long)]
        json: bool,
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

/// Wildcard match as in KiCad rules (`*` and `?`).
fn glob_match(pat: &str, s: &str) -> bool {
    fn rec(p: &[u8], s: &[u8]) -> bool {
        match (p.first(), s.first()) {
            (None, None) => true,
            (Some(b'*'), _) => rec(&p[1..], s) || (!s.is_empty() && rec(p, &s[1..])),
            (Some(b'?'), Some(_)) => rec(&p[1..], &s[1..]),
            (Some(a), Some(b)) if a == b => rec(&p[1..], &s[1..]),
            _ => false,
        }
    }
    rec(pat.as_bytes(), s.as_bytes())
}

/// The lines of the length, skew and uncoupled rules: one per rule, and one per net out of range.
fn print_matching(rules: &[gummiband_core::matching::RuleMatch]) {
    use gummiband_core::board::MatchKind;
    if rules.is_empty() {
        return;
    }
    println!("length rules: {}", gummiband_core::matching::summary(rules));
    for r in rules {
        let kind = match (r.kind, r.within_pairs) {
            (MatchKind::Length, _) => "length",
            (MatchKind::Skew, false) => "skew",
            (MatchKind::Skew, true) => "skew within pairs",
            (MatchKind::Uncoupled, _) => "uncoupled",
            (MatchKind::Gap, _) => "gap",
        };
        let v = r.value;
        let range = [("min", v.min), ("opt", v.opt), ("max", v.max)]
            .iter()
            .filter_map(|(k, x)| x.map(|x| format!("{k} {x}")))
            .collect::<Vec<_>>()
            .join(" ");
        println!(
            "  rule {} ({kind} {range}): {} of {} ok",
            r.rule,
            r.nets.len() - r.failed(),
            r.nets.len()
        );
        for n in r.nets.iter().filter(|n| !n.ok) {
            match n.skew {
                Some(s) => println!("    {}: {:.4} mm, skew {:+.4}", n.net, n.length, s),
                None => println!("    {}: {:.4} mm", n.net, n.length),
            }
        }
    }
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
            println!("patterns:");
            for line in gummiband_core::patterns::find(b).summary(b) {
                println!("  {line}");
            }
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
                let width = match (c.min_width, c.target_width) {
                    (None, None) => format!("{}", c.track_width),
                    (min, target) => format!(
                        "{} (min {}, target {})",
                        c.track_width,
                        min.map_or("-".into(), |w| w.to_string()),
                        target.map_or("-".into(), |w| w.to_string())
                    ),
                };
                println!(
                    "  {} (prio {}, w {}, clr {}, via {}/{}): {} nets {}",
                    c.name,
                    c.priority,
                    width,
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
            no_shove,
            nets,
            runs,
            via_pitch,
            via_cost,
            layer_dirs,
            patterns,
            pair_pull,
            via_pass,
            negotiate,
            pairs,
            pair_skew,
            pair_zdiff,
            pair_all,
            breakouts,
            via_pairs,
            generations,
            geometry_generations,
            seed,
            svg,
            svg_tris,
            report,
            frames,
            move_parts,
            no_move_parts,
            fixed,
            placements,
            placement_seed,
            clear_corridors,
            progress,
            until_stop,
            open_via_cost,
            air,
            prefer_width,
            net_widths,
            teardrops,
            net_priorities,
            angles,
            no_tune,
            tune_amplitude,
            tune_spacing,
        } => {
            // parts move by default when the whole board is routed; on a board whose copper is
            // kept they stay unless asked
            let move_parts = move_parts || (!no_move_parts && !keep_all);
            if until_stop && pairs {
                anyhow::bail!("--until-stop does not work with --pairs");
            }
            if shove && (!keep_all || nets.is_empty() || move_parts) {
                anyhow::bail!("--shove needs --keep-all and --nets, and fixed parts");
            }
            // finishing nets on a routed board: the unlocked tracks of the other nets shift aside
            // and, if that is not enough, give way; --no-shove and --give 0 switch that off. With
            // --until-stop the tries run without the open end and the one kept runs on open ended.
            let finishing = keep_all && !nets.is_empty();
            let shove = shove || (finishing && !no_shove && !move_parts);
            let give = give.unwrap_or(if finishing { gummiband_core::DEFAULT_GIVE } else { 0.0 });
            if placements > 0 && (keep_all || !nets.is_empty() || pairs) {
                anyhow::bail!("--placements routes the whole board: not with --keep-all, --nets or --pairs");
            }
            let keep = if keep_all {
                KeepCopper::All
            } else if keep_locked {
                KeepCopper::Locked
            } else {
                KeepCopper::None
            };
            if std::env::var_os("GB_GPU").is_some() && !cfg!(feature = "gpu") {
                println!("warning: GB_GPU has no effect, this binary is built without the gpu feature");
            }
            let t0 = std::time::Instant::now();
            let sub_cfg = SubstrateConfig {
                via_pitch,
                ..Default::default()
            };
            let hints = gummiband_core::patterns::hints(&patterns).map_err(|e| anyhow::anyhow!("--patterns: {e}"))?;
            let router_cfg = RouterConfig {
                via_cost,
                layer_dirs,
                pair_pull,
                via_pass,
                negotiate,
                ..Default::default()
            }
            .with_hints(&hints);
            // a line on stdin ends the run; so does a closed stdin, nothing can come any more
            let open_end = until_stop.then(|| {
                let flag = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
                let set = flag.clone();
                std::thread::spawn(move || {
                    let _ = std::io::stdin().read_line(&mut String::new());
                    set.store(true, std::sync::atomic::Ordering::Relaxed);
                });
                flag
            });
            let search_cfg = gummiband_core::SearchConfig {
                population: runs,
                generations,
                geometry_generations,
                frames: frames.is_some(),
                seed,
                open_end: open_end.clone(),
                // finishing nets moves only what the new tracks touch, also open ended
                air: air.unwrap_or(if until_stop && !finishing { 0.03 } else { 0.0 }),
                prefer_width,
                open_via_cost,
                progress: progress.then(|| progress_lines(t0, String::new())),
            };
            let load = |path: &std::path::Path| -> Result<gummiband_kicad::KicadBoard> {
                let kb = read_board(
                    path,
                    &ReadOptions {
                        keep,
                        plane_layers: cli.plane_layers.clone(),
                        move_parts,
                        fixed: fixed.clone(),
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
                Ok(kb)
            };
            // parts in the corridors of track groups move out of the way first
            let cleared_dir = std::env::temp_dir().join(format!("gummiband-corridors-{}", std::process::id()));
            let board = if clear_corridors {
                use gummiband_place::auto::{PlaceOptions, Scope, corridor_intruders, place_board};
                let opts = PlaceOptions {
                    fixed: fixed.clone(),
                    seed: placement_seed,
                    plane_layers: cli.plane_layers.clone(),
                    ..Default::default()
                };
                let intruders = corridor_intruders(&board, &opts)?;
                if intruders.is_empty() {
                    println!("corridors: no part in the way");
                    board
                } else {
                    println!(
                        "corridors: {} parts in the way get new positions: {}",
                        intruders.len(),
                        intruders.join(" ")
                    );
                    let env = |k: &str| std::env::var(k).ok().and_then(|v| v.parse::<f64>().ok());
                    let placed = place_board(
                        &board,
                        &PlaceOptions {
                            scope: Scope::Only(intruders),
                            corridors: true,
                            gap: env("GB_CC_GAP").unwrap_or(opts.gap),
                            halo: env("GB_CC_HALO").unwrap_or(opts.halo),
                            escape_halo: env("GB_CC_ESCAPE").is_some_and(|v| v > 0.0),
                            semantic: env("GB_CC_SEMANTIC").is_some_and(|v| v > 0.0),
                            leash: env("GB_CC_LEASH").unwrap_or(gummiband_place::auto::CLEAR_CORRIDORS_LEASH),
                            ..opts
                        },
                    )?;
                    println!(
                        "corridors: smallest courtyard distance {:.2} mm",
                        placed.report.min_separation
                    );
                    std::fs::create_dir_all(&cleared_dir)?;
                    let path = cleared_dir.join(board.file_name().unwrap_or_default());
                    std::fs::write(&path, &placed.text)?;
                    for ext in ["kicad_pro", "kicad_dru"] {
                        let src = board.with_extension(ext);
                        if src.exists() {
                            std::fs::copy(&src, path.with_extension(ext))?;
                        }
                    }
                    path
                }
            } else {
                board
            };
            let input = board;
            let placed_dir = std::env::temp_dir().join(format!("gummiband-placements-{}", std::process::id()));
            let mut placement = None;
            let board = if placements > 0 {
                let opts = gummiband_place::auto::PlaceOptions {
                    fixed: fixed.clone(),
                    seed: placement_seed,
                    starts: 1,
                    plane_layers: cli.plane_layers.clone(),
                    ..Default::default()
                };
                let found = try_placements(&input, &placed_dir, placements, &opts, &|path, seed, i| {
                    let kb = load(path)?;
                    let search = gummiband_core::SearchConfig {
                        frames: false,
                        // only the winner runs open ended
                        open_end: None,
                        progress: progress.then(|| progress_lines(t0, format!("placement {i}/{placements}: "))),
                        ..search_cfg.clone()
                    };
                    let (_, res) = gummiband_core::route_board(&kb.board, &sub_cfg, &router_cfg, &search);
                    let violations = gummiband_core::check::check(&kb.board.with_offsets(&res.offsets), &res.geometry);
                    Ok(Trial {
                        seed,
                        routed: res.stats.routed,
                        connections: res.stats.conns,
                        violations: violations.len(),
                        vias: res.stats.vias,
                        length: res.stats.length,
                    })
                })
                .inspect_err(|_| {
                    let _ = std::fs::remove_dir_all(&placed_dir);
                })?;
                let path = found.path.clone();
                placement = Some(found);
                path
            } else {
                input.clone()
            };
            let kb = load(&board)?;
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
            let tune_cfg = gummiband_core::tune::TuneConfig {
                style: if angles == Angles::Deg45 {
                    gummiband_core::tune::Style::Chamfer
                } else {
                    gummiband_core::tune::Style::Round
                },
                amplitude: tune_amplitude,
                spacing: tune_spacing,
            };
            let tune_room = !no_tune && std::env::var("GB_NO_TUNE_ROOM").is_err();
            // the nets asked for, and the nets that give way when a connection finds no way
            let finished = gummiband_core::finish_nets(b, &nets, shove, give, &search_cfg, |board, search| {
                let out = if pairs {
                    let pair_cfg = gummiband_core::pairs::PairConfig {
                        skew: pair_skew,
                        zdiff: pair_zdiff,
                        all: pair_all,
                        room: tune_room.then(|| tune_cfg.clone()),
                        angles45: angles == Angles::Deg45,
                        breakouts,
                        via_pairs,
                    };
                    gummiband_core::pairs::route_pairs(board, &sub_cfg, &router_cfg, search, &pair_cfg)
                } else {
                    let (sub, res) = gummiband_core::route_board(board, &sub_cfg, &router_cfg, search);
                    (sub, res, Vec::new())
                };
                gummiband_core::prof::report();
                out
            });
            for t in finished.tries.iter().skip(1) {
                println!(
                    "give way within {} mm: {} nets, {} open, {} violations",
                    t.radius, t.given, t.open, t.violations
                );
            }
            if finished.tries.len() > 1 {
                let k = &finished.tries[finished.kept];
                println!(
                    "kept: {} nets gave way, {} open, {} violations",
                    k.given, k.open, k.violations
                );
            }
            let gummiband_core::Finished {
                board: routed_board,
                sub,
                res,
                extra: pair_reports,
                given,
                ..
            } = finished;
            let b = &routed_board;
            let mut res = res;
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
            // room for the meanders: the copper next to the nets that want length steps aside
            // (the pairs make it while they are one wide track)
            if !pairs && tune_room && !b.match_rules.is_empty() {
                let t1 = std::time::Instant::now();
                let want = gummiband_core::tune::room_wanted(b, &res.geometry, &tune_cfg, &[]);
                if !want.is_empty() {
                    let made = gummiband_core::tune::make_room(b, &sub, &res.geometry, &want);
                    println!(
                        "room for meanders: {} nets, {}, {:.1} s",
                        want.len(),
                        if made.is_some() {
                            "made"
                        } else {
                            "none without new violations"
                        },
                        t1.elapsed().as_secs_f64()
                    );
                    if let Some(g) = made {
                        res.geometry = g;
                    }
                }
            }
            let octo = (angles == Angles::Deg45).then(|| {
                let before = gummiband_core::check::check(b, &res.geometry).len();
                let t1 = std::time::Instant::now();
                // the two tracks of a pair that went onto the grid as one wide track stay as
                // they are; their ends and what was routed on its own afterwards do not
                let wide: Vec<gummiband_core::board::NetId> = pair_reports
                    .iter()
                    .filter(|r| r.wide)
                    .flat_map(|r| [b.net_id(&r.p), b.net_id(&r.n)])
                    .flatten()
                    .collect();
                let st = gummiband_core::octo::octilinear_only(b, &mut res.geometry, &|t| {
                    !(wide.contains(&t.net) && t.seg == u32::MAX)
                }, &|_, _| false);
                let secs = t1.elapsed().as_secs_f64();
                let after = gummiband_core::check::check(b, &res.geometry).len();
                println!(
                    "45 degrees: {} tracks, {:.0} -> {:.0} mm, {} pieces off the grid ({:.1} mm), {} sharp corners, violations {before} -> {after}, {secs:.1} s",
                    st.tracks, st.length_before, st.length_after, st.any_angle, st.any_angle_length, st.sharp_corners
                );
                st
            });
            let tuned = (!no_tune && !b.match_rules.is_empty()).then(|| {
                let t1 = std::time::Instant::now();
                let rep = gummiband_core::tune::tune(b, &mut res.geometry, &tune_cfg);
                let short: Vec<_> = rep.nets.iter().filter(|n| n.aim - n.after > 0.005).collect();
                println!(
                    "length tuning: {} nets, {} reached their length, {:.1} s",
                    rep.nets.len(),
                    rep.nets.len() - short.len(),
                    t1.elapsed().as_secs_f64()
                );
                for n in &short {
                    println!(
                        "  {}: {:.3} -> {:.3} mm, aim {:.3} mm{}",
                        n.net,
                        n.before,
                        n.after,
                        n.aim,
                        if n.coupled { " (as a pair)" } else { "" }
                    );
                }
                rep
            });
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
                    "  pair {} / {}: {}, width {:.3} gap {:.3}, {:.0} ohm, {:.2} / {:.2} mm, skew {:.3} mm, uncoupled {:.2} mm, {} vias, {} swaps {}",
                    r.p,
                    r.n,
                    if r.wide { "wide" } else { "single" },
                    r.width,
                    r.gap,
                    r.zdiff,
                    r.length_p,
                    r.length_n,
                    r.skew,
                    r.uncoupled,
                    r.vias,
                    r.swaps,
                    r.note
                );
            }
            let matching = gummiband_core::matching::check(b, &res.geometry);
            print_matching(&matching);
            let viol = gummiband_core::check::check(b, &res.geometry);
            let spots = gummiband_core::check::spots(&viol).len();
            println!("clearance check: {} violations at {spots} spots", viol.len());
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
            let write_result =
                |geometry: &gummiband_core::realize::Geometry, path: &std::path::Path| -> Result<usize> {
                    // an alternative has its own soft copper that moved
                    let changes = gummiband_core::import::soft_changes(b, geometry);
                    let mut items = Vec::new();
                    // the items of each track, for the tuning patterns
                    let mut of_track: Vec<std::ops::Range<usize>> = Vec::with_capacity(geometry.tracks.len());
                    for t in &geometry.tracks {
                        let start = items.len();
                        if rests(t.origin, &changes.chains) {
                            of_track.push(start..start);
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
                        of_track.push(start..items.len());
                    }
                    for p in tuned.iter().flat_map(|r| &r.patterns) {
                        let members: Vec<usize> = p
                            .tracks
                            .iter()
                            .filter_map(|&i| of_track.get(i))
                            .flat_map(|r| r.clone())
                            .collect();
                        if members.is_empty() {
                            continue;
                        }
                        items.push(write::Item::Pattern(Box::new(write::PatternItem {
                            layer: p.layer,
                            net: p.nets[0],
                            base: p.base[0],
                            base_coupled: p.base.get(1).copied(),
                            rounded: p.rounded,
                            width: p.width,
                            gap: p.gap,
                            amplitude: p.amplitude,
                            spacing: p.spacing,
                            target: p.aim,
                            length: gummiband_core::matching::net_length(b, geometry, p.nets[0]).total(),
                            members,
                        })));
                    }
                    for v in &geometry.vias {
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
                    std::fs::write(path, text)?;
                    if teardrops {
                        track_end_teardrops(&board, path)?;
                    }
                    Ok(items.len())
                };
            let items = write_result(&res.geometry, &output)?;
            // the results the open end leaves to choose from, next to the output
            for (i, (st, geometry)) in res.alternatives.iter().enumerate() {
                let path = output.with_extension(format!("alt{}.kicad_pcb", i + 1));
                write_result(geometry, &path)?;
                println!(
                    "alternative: {}: {}/{} routed, {} vias, {:.0} mm, {} clearance spots",
                    path.display(),
                    st.routed,
                    st.conns,
                    st.vias,
                    st.length,
                    st.violations.unwrap_or(0)
                );
            }
            println!("wrote {items} items to {}", output.display());
            if let Some(path) = report {
                let st = &res.stats;
                let json = serde_json::json!({
                    "board": input.display().to_string(),
                    "output": output.display().to_string(),
                    "seconds": dt.as_secs_f64(),
                    "placement": placement.as_ref().map(Placement::to_json),
                    "connections": st.conns,
                    "routed": st.routed,
                    "unrouted": st.unrouted,
                    "pairs": pair_reports,
                    "matching": matching,
                    "vias": st.vias,
                    "length_mm": st.length,
                    "length_lower_bound_mm": gummiband_core::length_lower_bound(b),
                    "patterns": gummiband_core::patterns::find(b).named(b),
                    "angles_45": octo,
                    "length_tuning": tuned,
                    "items_written": items,
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
                        "layer_dirs": r.layer_dirs,
                        "via_cost": r.via_cost,
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
            let _ = std::fs::remove_dir_all(&placed_dir);
        }
        Cmd::Place {
            board,
            output,
            only,
            outside,
            all_unlocked,
            fixed,
            free,
            seed,
            starts,
            iters,
            gap,
            margin,
            halo,
            semantic,
            corridors,
            escape_halo,
            leash,
            two_sided,
            back_max_pads,
            back_cost,
            on_front,
            on_back,
            report,
        } => {
            use gummiband_place::auto::{PlaceOptions, Scope};
            let scope = if !only.is_empty() {
                Scope::Only(only)
            } else if outside {
                Scope::Outside
            } else if all_unlocked {
                Scope::AllUnlocked
            } else {
                Scope::Unlocked
            };
            let opts = PlaceOptions {
                scope,
                fixed,
                free,
                seed,
                starts,
                iters,
                gap,
                margin,
                halo,
                plane_layers: cli.plane_layers.clone(),
                semantic,
                corridors,
                escape_halo,
                leash,
                two_sided,
                back_max_pads,
                back_cost,
                on_front,
                on_back,
            };
            let placed = gummiband_place::auto::place_board(&board, &opts)?;
            let r = &placed.report;
            println!(
                "{} parts placed, {} fixed; {:.0} mm2 of courtyards on {:.0} mm2 free",
                r.moved.len(),
                r.fixed.len(),
                r.movable_area,
                r.free_area
            );
            if two_sided {
                println!(
                    "two sided: {:.0} mm2 free on the front, {:.0} on the back; {} moved parts on the back, {} changed sides",
                    r.free_area - r.back_free_area,
                    r.back_free_area,
                    r.back.len(),
                    r.flipped.len()
                );
            }
            for w in &r.warnings {
                println!("warning: {w}");
            }
            for (s, w, c) in &r.starts {
                println!("  start {s}: wire {w:.0}, {c} crossings");
            }
            println!(
                "best: seed {} start {}, wire {:.0}, {} crossings, smallest courtyard distance {:.2} mm",
                r.seed, r.start, r.wire, r.crossings, r.min_separation
            );
            for (l, ic) in &r.coils {
                println!("inductor {l} at {ic}");
            }
            for (name, refs) in &r.groups {
                println!("group {name}: {}", refs.join(" "));
            }
            if !r.anchored.is_empty() {
                println!("stayed for their kept copper: {}", r.anchored.join(" "));
            }
            if !r.off_plane.is_empty() {
                println!("pins off their plane: {}", r.off_plane.join(" "));
            }
            if r.stale_copper > 0 {
                println!("{} tracks and vias in the file do not follow the parts", r.stale_copper);
            }
            if let Some(dir) = output.parent() {
                std::fs::create_dir_all(dir)?;
            }
            std::fs::write(&output, &placed.text)?;
            for ext in ["kicad_pro", "kicad_dru"] {
                let src = board.with_extension(ext);
                if src.exists() {
                    std::fs::copy(&src, output.with_extension(ext))?;
                }
            }
            if let Some(path) = report {
                let json = serde_json::json!({
                    "seed": r.seed,
                    "start": r.start,
                    "anchored": r.anchored,
                    "wire": r.wire,
                    "crossings": r.crossings,
                    "starts": r.starts,
                    "fixed": r.fixed,
                    "moved": r.moved,
                    "poses": r.poses,
                    "zones": r
                        .zones
                        .iter()
                        .map(|(id, pts)| {
                            let pts: Vec<[f64; 2]> = pts.iter().map(|p| [p.x, p.y]).collect();
                            serde_json::json!({ "id": id, "points": pts })
                        })
                        .collect::<Vec<_>>(),
                    "decoupling": r.decoupling,
                    "coils": r.coils,
                    "groups": r.groups,
                    "off_plane": r.off_plane,
                    "stale_copper": r.stale_copper,
                    "free_area": r.free_area,
                    "movable_area": r.movable_area,
                    "min_separation": r.min_separation,
                    "back_free_area": r.back_free_area,
                    "flipped": r.flipped,
                    "flipped_zones": r.flipped_zones,
                    "back": r.back,
                    "warnings": r.warnings,
                });
                std::fs::write(path, serde_json::to_string_pretty(&json)?)?;
            }
            println!("wrote {}", output.display());
        }
        Cmd::Angles45 {
            board: input,
            output,
            nets,
            report,
        } => {
            let t0 = std::time::Instant::now();
            let kb = read_board(
                &input,
                &ReadOptions {
                    keep: KeepCopper::All,
                    plane_layers: cli.plane_layers.clone(),
                    ..Default::default()
                },
            )?;
            let (text, r) = gummiband_kicad::octo_write::angles_45(&kb, &nets)?;
            std::fs::write(&output, text)?;
            let secs = t0.elapsed().as_secs_f64();
            let st = &r.stats;
            println!(
                "45 degrees: {} tracks, {} changed, {:.0} -> {:.0} mm, {} pieces off the grid ({:.1} mm), {} sharp corners, violations {} -> {}, {secs:.1} s",
                st.tracks,
                r.tracks_changed,
                st.length_before,
                st.length_after,
                st.any_angle,
                st.any_angle_length,
                st.sharp_corners,
                r.violations_before,
                r.violations_after
            );
            println!(
                "replaced {} items with {} items, wrote {}",
                r.items_removed,
                r.items_written,
                output.display()
            );
            if let Some(path) = report {
                let json = serde_json::json!({
                    "board": input.display().to_string(),
                    "output": output.display().to_string(),
                    "seconds": secs,
                    "angles_45": r,
                });
                std::fs::write(&path, serde_json::to_string_pretty(&json)?)?;
            }
        }
        Cmd::MovePart {
            board: input,
            reference,
            x,
            y,
            output,
            report,
            svg: svg_out,
        } => {
            let t0 = std::time::Instant::now();
            let kb = read_board(
                &input,
                &ReadOptions {
                    keep: KeepCopper::All,
                    plane_layers: cli.plane_layers.clone(),
                    ..Default::default()
                },
            )?;
            if !input.with_extension("kicad_pro").is_file() {
                println!(
                    "warning: no {} next to the board: default clearances and widths, the copper may give way to the wrong rules",
                    input.with_extension("kicad_pro").display()
                );
            }
            let (text, r) = gummiband_kicad::follow_write::move_part_following(&kb, &reference, x, y)?;
            std::fs::write(&output, text)?;
            let secs = t0.elapsed().as_secs_f64();
            let st = &r.stats;
            println!(
                "moved {} from ({:.3}, {:.3}) to ({:.3}, {:.3}), {} of {} steps; {} tracks and {} vias of {} nets followed, at most {:.3} mm; clearance spots {} -> {}, {secs:.1} s",
                r.reference,
                r.from.0,
                r.from.1,
                r.to.0,
                r.to.1,
                st.steps,
                st.steps_planned,
                st.tracks_moved,
                st.vias_moved,
                st.nets_moved,
                st.max_shift,
                st.spots_before,
                st.spots_after
            );
            if let Some(why) = &st.stopped {
                println!("stopped short: {why}");
            }
            println!(
                "replaced {} items with {} items, wrote {}",
                r.items_removed,
                r.items_written,
                output.display()
            );
            if let Some(path) = svg_out {
                let mut ob = read_board(
                    &output,
                    &ReadOptions {
                        keep: KeepCopper::All,
                        plane_layers: cli.plane_layers.clone(),
                        ..Default::default()
                    },
                )?
                .board;
                ob.soften(&[]);
                let geo = gummiband_core::follow::soft_geometry(&ob);
                let viol = gummiband_core::check::check(&ob, &geo);
                std::fs::write(&path, svg::render(&ob, None, &geo, &viol, false))?;
            }
            if let Some(path) = report {
                let json = serde_json::json!({
                    "board": input.display().to_string(),
                    "output": output.display().to_string(),
                    "seconds": secs,
                    "move_part": r,
                });
                std::fs::write(&path, serde_json::to_string_pretty(&json)?)?;
            }
        }
        Cmd::Lengths { board, nets, json } => {
            let kb = read_board(
                &board,
                &ReadOptions {
                    keep: KeepCopper::All,
                    plane_layers: cli.plane_layers.clone(),
                    ..Default::default()
                },
            )?;
            let b = &kb.board;
            let geo = gummiband_core::matching::kept_geometry(b);
            let rules = gummiband_core::matching::check(b, &geo);
            let glob = |pat: &str, s: &str| glob_match(pat, s);
            let listed: Vec<(String, gummiband_core::matching::NetLength)> = b
                .nets
                .iter()
                .enumerate()
                .filter(|(_, n)| nets.iter().any(|p| glob(p, &n.name)))
                .map(|(i, n)| (n.name.clone(), gummiband_core::matching::net_length(b, &geo, i as u32)))
                .collect();
            let pairs: Vec<serde_json::Value> = b
                .diff_pairs()
                .into_iter()
                .map(|(p, n)| {
                    let gap = gummiband_core::matching::gap_rule(b, p);
                    let (c, t) = gummiband_core::matching::coupling(b, &geo, p, n, gap);
                    let lp = gummiband_core::matching::net_length(b, &geo, p).total();
                    let ln = gummiband_core::matching::net_length(b, &geo, n).total();
                    serde_json::json!({
                        "p": b.nets[p as usize].name, "n": b.nets[n as usize].name,
                        "length_p": lp, "length_n": ln, "skew": lp - ln,
                        "coupled": c, "uncoupled": (t - c).max(0.0),
                    })
                })
                .collect();
            if json {
                let nets_json: Vec<serde_json::Value> = listed
                    .iter()
                    .map(|(n, l)| serde_json::json!({"net": n, "length": l.total(), "tracks": l.tracks, "vias": l.vias, "die": l.die}))
                    .collect();
                println!(
                    "{}",
                    serde_json::to_string_pretty(
                        &serde_json::json!({"rules": rules, "pairs": pairs, "nets": nets_json})
                    )?
                );
            } else {
                for (n, l) in &listed {
                    println!(
                        "net {n}: {:.4} mm (tracks {:.4}, vias {:.4}, pad-to-die {:.4})",
                        l.total(),
                        l.tracks,
                        l.vias,
                        l.die
                    );
                }
                for p in &pairs {
                    println!(
                        "pair {} / {}: {:.4} / {:.4} mm, skew {:.4}, uncoupled {:.4} mm",
                        p["p"].as_str().unwrap_or(""),
                        p["n"].as_str().unwrap_or(""),
                        p["length_p"].as_f64().unwrap_or(0.0),
                        p["length_n"].as_f64().unwrap_or(0.0),
                        p["skew"].as_f64().unwrap_or(0.0),
                        p["uncoupled"].as_f64().unwrap_or(0.0)
                    );
                }
                print_matching(&rules);
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

/// The `progress {json}` lines of a route, with `prefix` in front of each phase.
fn progress_lines(t0: std::time::Instant, prefix: String) -> gummiband_core::ProgressFn {
    gummiband_core::ProgressFn(std::sync::Arc::new(move |p: &gummiband_core::Progress| {
        use std::io::Write;
        let mut p = p.clone();
        p.phase = format!("{prefix}{}", p.phase);
        let mut v = serde_json::to_value(&p).unwrap_or_default();
        v["seconds"] = serde_json::json!((t0.elapsed().as_secs_f64() * 10.0).round() / 10.0);
        let mut out = std::io::stdout().lock();
        let _ = writeln!(out, "progress {v}");
        let _ = out.flush();
    }))
}

/// One placement of --placements, routed.
struct Trial {
    seed: u64,
    routed: usize,
    connections: usize,
    violations: usize,
    vias: usize,
    length: f64,
}

impl Trial {
    /// Lower is better: completion first, then clean copper, then vias and length.
    fn score(&self) -> (usize, usize, usize, f64) {
        (self.connections - self.routed, self.violations, self.vias, self.length)
    }
}

/// The winner of --placements.
struct Placement {
    /// The placed board, the final route reads it.
    path: PathBuf,
    report: gummiband_place::auto::PlaceReport,
    trials: Vec<Trial>,
    seed: u64,
}

impl Placement {
    fn to_json(&self) -> serde_json::Value {
        serde_json::json!({
            "seed": self.seed,
            "moved": self.report.moved,
            "zones": self.report.zones.iter().map(|(id, pts)| {
                let pts: Vec<[f64; 2]> = pts.iter().map(|p| [p.x, p.y]).collect();
                serde_json::json!({ "id": id, "points": pts })
            }).collect::<Vec<_>>(),
            "tried": self.trials.iter().map(|t| serde_json::json!({
                "seed": t.seed,
                "routed": t.routed,
                "connections": t.connections,
                "violations": t.violations,
                "vias": t.vias,
                "length_mm": t.length,
            })).collect::<Vec<_>>(),
        })
    }
}

/// Place the board `count` times and let `route` judge every result. The seeds run one after the
/// other: each route uses all threads itself.
fn try_placements(
    input: &std::path::Path,
    dir: &std::path::Path,
    count: usize,
    opts: &gummiband_place::auto::PlaceOptions,
    route: &dyn Fn(&std::path::Path, u64, usize) -> Result<Trial>,
) -> Result<Placement> {
    std::fs::create_dir_all(dir)?;
    let mut best: Option<Placement> = None;
    let mut trials: Vec<Trial> = Vec::new();
    for i in 0..count {
        let seed = opts.seed + i as u64;
        let placed =
            gummiband_place::auto::place_board(input, &gummiband_place::auto::PlaceOptions { seed, ..opts.clone() })?;
        let path = dir.join(format!("placement-{seed}.kicad_pcb"));
        std::fs::write(&path, &placed.text)?;
        for ext in ["kicad_pro", "kicad_dru"] {
            let src = input.with_extension(ext);
            if src.exists() {
                std::fs::copy(&src, path.with_extension(ext))?;
            }
        }
        let t = route(&path, seed, i + 1)?;
        println!(
            "placement {}/{count}, seed {seed}: {}/{} routed, {} violations, {} vias, {:.0} mm",
            i + 1,
            t.routed,
            t.connections,
            t.violations,
            t.vias,
            t.length
        );
        let better = trials.iter().all(|o| t.score() < o.score());
        trials.push(t);
        if better {
            best = Some(Placement {
                path,
                report: placed.report,
                trials: Vec::new(),
                seed,
            });
        }
    }
    let mut best = best.context("--placements needs at least one placement")?;
    println!("best placement: seed {}", best.seed);
    best.trials = trials;
    Ok(best)
}
