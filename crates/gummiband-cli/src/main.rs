use std::path::PathBuf;

use anyhow::Result;
use clap::{Parser, Subcommand};
use gummiband_app::{AreaOptions, Hooks, RouteOptions, defaults};
use gummiband_core::board::ObstacleKind;
use gummiband_kicad::{KeepCopper, ReadOptions, read_board, write};
use gummiband_place::auto::PlaceOptions;

/// Track angles of a routed result.
#[derive(Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum Angles {
    /// Any-angle tracks with arcs, as the rubber bands lie.
    Any,
    /// Conventional 45 degree routing.
    #[value(name = "45")]
    Deg45,
}

/// How the nets with an area on a routed layer are connected.
#[derive(Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum Areas {
    /// Like any net, as tracks.
    Tracks,
    /// By the fill of their area.
    Fill,
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

/// The arguments of `route`: the options of the pipeline ([`gummiband_app::OPTIONS`], each as
/// `--kebab-case`) and what the CLI itself does with the result.
#[derive(clap::Args)]
struct RouteArgs {
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
    /// With --keep-all --nets: remove the unlocked tracks and vias of those nets first and route
    /// them anew. Without it a net that is already connected stays as it is.
    #[arg(long)]
    reroute: bool,
    /// Portfolio size (router runs with different orders). A constant, so that results do
    /// not depend on the machine.
    #[arg(long, default_value_t = defaults::RUNS)]
    runs: usize,
    /// Via site grid pitch in mm.
    #[arg(long, default_value_t = defaults::VIA_PITCH)]
    via_pitch: f64,
    /// Via cost in mm of track (the search varies it around this value).
    #[arg(long, default_value_t = defaults::VIA_COST)]
    via_cost: f64,
    /// Weight of the preferred layer directions (horizontal and vertical in turn) for
    /// every third run of the portfolio; 0 = off.
    #[arg(long, default_value_t = defaults::LAYER_DIRS)]
    layer_dirs: f64,
    /// Hints from local patterns of the board, comma separated: `shares` (pins of a plane net
    /// next to each other share one fan-out via), `decaps` (a decoupling capacitor is
    /// connected to its IC pin first, its via serves both), `rows` (the open end re-lays the
    /// connections of a pin row in pin order), `pairs` (differential pairs before the other
    /// signal connections, the longest first), `groups` (the connections between two parts
    /// as one block in pin order, the longest block first; `groups-short` puts the shortest
    /// first), `all` or
    /// `none`. `gummiband info` lists the patterns it finds. When to use which: see
    /// docs/guide/options.md.
    #[arg(long, default_value = defaults::PATTERNS)]
    patterns: String,
    /// Factor on the via cost in the pass that routes connections with vias again and
    /// keeps what has fewer vias (0 = off).
    #[arg(long, default_value_t = defaults::VIA_PASS)]
    via_pass: f64,
    /// Weight of the pull between the nets of differential pairs (0 = off).
    #[arg(long, default_value_t = defaults::PAIR_PULL)]
    pair_pull: f64,
    /// Experimental: route differential pairs as one wide track and split it into two parallel
    /// tracks. Check the result with the KiCad DRC; on dense boards it costs connections.
    #[arg(long)]
    pairs: bool,
    /// With --pairs: length difference left within a pair in mm; the shorter track gets
    /// bumps down to this (a `skew` rule in the .kicad_dru may ask for less).
    #[arg(long, default_value_t = defaults::PAIR_SKEW)]
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
    #[arg(long, default_value_t = defaults::NEGOTIATE)]
    negotiate: usize,
    /// Generations of the evolutionary order search after the portfolio.
    #[arg(long, default_value_t = defaults::GENERATIONS)]
    generations: usize,
    /// Generations that select by clearance violations after the geometry step. Each costs
    /// about one routing pass; 0 is fastest and leaves many more violations.
    #[arg(long, default_value_t = defaults::GEOMETRY_GENERATIONS)]
    geometry_generations: usize,
    /// Seed of the search (default: GB_EVO_SEED, then a constant). The same seed gives the
    /// same result, another one a different try. The line "best: seed N" and the report
    /// name the seed that was used.
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
    /// portfolio) for scripts and AI agents. Its `seconds` are the wall time of the whole
    /// route, from reading the board to the checked result (writing the files not
    /// included), the same number the MCP tool route returns; `search_seconds` is the part
    /// up to the end of the search.
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
    /// References that never move under --move-parts, --placements and --clear-corridors,
    /// comma separated.
    #[arg(long, value_delimiter = ',')]
    fixed: Vec<String>,
    /// How nets with an area (a zone of the file, or a pour of --pour) on a routed layer are
    /// connected: `tracks` lays them like any net and does not look at the area, `fill` does
    /// not lay them as tracks: the fill connects them, and the report says which pads it does
    /// not reach.
    #[arg(long, value_enum, default_value_t = Areas::Tracks)]
    areas: Areas,
    /// With --areas fill: a net that gets a pour after the route, as NET or NET:LAYER,LAYER
    /// (the outer layers where none is given). The route takes the pour as an area of the
    /// net; it does not write the zone. Give the flag once per net.
    #[arg(long, value_name = "NET[:LAYERS]")]
    pour: Vec<String>,
    /// With --areas fill: an island of fill that holds pads gets one via where it joins the
    /// area, set on the finished copper where the fill model finds the spot. On by default
    /// with --areas fill; this flag insists on it.
    #[arg(long, conflicts_with = "no_area_stitch")]
    area_stitch: bool,
    /// No vias for islands of fill: a pad on an island gets a track to the area instead.
    #[arg(long)]
    no_area_stitch: bool,
    /// Try this many placements from the netlist (see `gummiband place`), route each one with
    /// these settings and keep the best: fewest open connections, then clearance violations,
    /// vias and length. Parts that are not locked or mechanical get new positions.
    #[arg(long, default_value_t = defaults::PLACEMENTS)]
    placements: usize,
    /// First seed of --placements, and the seed of --clear-corridors.
    #[arg(long, default_value_t = defaults::PLACEMENT_SEED)]
    placement_seed: u64,
    /// Before the routing, parts that sit in the band of a group of tracks between two parts
    /// (ICs, connectors sharing at least two signal nets) and are not on its nets get new
    /// positions with `place --corridors`; everything else stays. Pairs with `--patterns
    /// groups`. Experimental.
    #[arg(long)]
    clear_corridors: bool,
    /// Options per net from a JSON file, one entry per net name: a track width in mm
    /// ({"GND": 0.5}, e.g. the widths of an existing routing, see bench/net_widths.py) or
    /// an object with any of `width`, `min_width` (no necking below it), `neck_refs` (the
    /// parts in whose courtyard the track may neck all the same; left out: those with a
    /// narrower pad), `priority` (lower routes first) and `skip` (leave the net unrouted),
    /// the fields of the MCP tool set_net_options. Nets the board lacks are passed over.
    #[arg(long)]
    net_widths: Option<PathBuf>,
    /// Switch on KiCad's teardrops for vias and pads in the output; KiCad builds them when
    /// it fills the zones.
    #[arg(long)]
    teardrops: bool,
    /// Routing priority per net from a JSON file {"net name": priority, ...}; lower routes
    /// first, unnamed nets keep their class priority. Wins over a priority in --net-widths.
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
    /// Open end with a time budget: after the normal search keep improving for this many
    /// seconds, then finish the best result (0: off). The budget starts when the open end
    /// starts, so it never cuts the normal search short. With --until-stop as well, the
    /// run ends on the line on stdin or after the budget, whichever comes first.
    #[arg(long, default_value_t = defaults::OPEN_END_SECONDS)]
    open_end_seconds: f64,
    /// In the open end (--until-stop, --open-end-seconds): what a via is worth in mm of track when the open end judges its
    /// results. Lower gives more vias and shorter tracks, higher fewer vias and longer ones.
    #[arg(long, default_value_t = defaults::OPEN_VIA_COST)]
    open_via_cost: f64,
    /// Give the finished copper this reserve in mm on top of the clearances where there is
    /// room, instead of leaving it at the limit [default: 0.03 in an open ended run
    /// (--until-stop, --open-end-seconds), but not when finishing nets with --keep-all
    /// --nets, else 0]
    #[arg(long)]
    air: Option<f64>,
    /// At the end tracks of the default net class push aside what can give way and grow up
    /// to this width in mm wherever there is room, with a taper at every change; routing
    /// still works with the class width. A class with a track width rule in the .kicad_dru
    /// that has an `opt` value grows to that one, also without this flag
    #[arg(long, default_value_t = defaults::PREFER_WIDTH)]
    prefer_width: f64,
    /// Track angles of the result: `any` keeps the tight any-angle tracks with arcs, `45`
    /// turns them into conventional 45 degree routing as a last step (straight pieces in
    /// eight directions, chamfered corners). The step never adds a clearance violation;
    /// where a 45 degree path does not fit, a piece keeps its angle.
    #[arg(long, value_enum, default_value_t = Angles::Any)]
    angles: Angles,
    /// Meanders for the length and skew rules of the .kicad_dru; each is written as a KiCad
    /// tuning pattern. On by default (without such rules nothing is tuned anyway); this flag
    /// insists on it.
    #[arg(long, conflicts_with = "no_tune")]
    tune: bool,
    /// Leave the lengths as routed: no meanders for the length and skew rules of the
    /// .kicad_dru
    #[arg(long)]
    no_tune: bool,
    /// Highest meander bump in mm (centre line above the original line)
    #[arg(long, default_value_t = defaults::TUNE_AMPLITUDE)]
    tune_amplitude: f64,
    /// Air between neighbouring meander legs in track widths (at least the clearance)
    #[arg(long, default_value_t = defaults::TUNE_SPACING)]
    tune_spacing: f64,
}

impl RouteArgs {
    /// The options of the pipeline these arguments ask for. What was not given is the default
    /// of `RouteOptions`, through the clap defaults that name the same constants.
    fn options(&self, plane_layers: &[String]) -> RouteOptions {
        // `--name` insists, `--no-name` switches off, neither leaves the default to the pipeline
        let on_off = |on: bool, off: bool| match (on, off) {
            (true, _) => Some(true),
            (false, true) => Some(false),
            (false, false) => None,
        };
        RouteOptions {
            keep_locked: self.keep_locked,
            keep_all: self.keep_all,
            nets: self.nets.clone(),
            reroute: self.reroute,
            shove: on_off(self.shove, self.no_shove),
            give: self.give,
            move_parts: on_off(self.move_parts, self.no_move_parts),
            fixed: self.fixed.clone(),
            plane_layers: plane_layers.to_vec(),
            areas: match self.areas {
                Areas::Tracks => gummiband_app::Areas::Tracks,
                Areas::Fill => gummiband_app::Areas::Fill,
            },
            pour: self.pour.clone(),
            area_stitch: on_off(self.area_stitch, self.no_area_stitch),
            runs: self.runs,
            via_pitch: self.via_pitch,
            via_cost: self.via_cost,
            layer_dirs: self.layer_dirs,
            patterns: self.patterns.clone(),
            via_pass: self.via_pass,
            pair_pull: self.pair_pull,
            pairs: self.pairs,
            pair_skew: self.pair_skew,
            pair_zdiff: self.pair_zdiff,
            pair_all: self.pair_all,
            breakouts: self.breakouts,
            via_pairs: self.via_pairs,
            negotiate: self.negotiate,
            generations: self.generations,
            geometry_generations: self.geometry_generations,
            seed: self.seed,
            frames: self.frames.clone(),
            open_end_seconds: self.open_end_seconds,
            open_via_cost: self.open_via_cost,
            air: self.air,
            prefer_width: self.prefer_width,
            angles: match self.angles {
                Angles::Any => gummiband_app::Angles::Any,
                Angles::Deg45 => gummiband_app::Angles::Deg45,
            },
            tune: !self.no_tune,
            tune_amplitude: self.tune_amplitude,
            tune_spacing: self.tune_spacing,
            placements: self.placements,
            placement_seed: self.placement_seed,
            clear_corridors: self.clear_corridors,
            net_widths: self.net_widths.clone(),
            net_priorities: self.net_priorities.clone(),
            net_options: Default::default(),
            teardrops: self.teardrops,
        }
    }
}

/// The arguments of `areas`: the options of the report ([`gummiband_app::AREA_OPTIONS`], each as
/// `--kebab-case`) and how the CLI prints it.
#[derive(clap::Args)]
struct AreasArgs {
    board: PathBuf,
    /// Report only these nets, comma separated [default: every net with a zone]
    #[arg(long, alias = "net", value_delimiter = ',')]
    nets: Vec<String>,
    /// Layers, comma separated, on which the nets given get an area for this report where they
    /// have none yet, as `ground_pour` would lay it: one net over the board outline, several
    /// nets each around its own pads. The file stays as it is.
    #[arg(long, value_delimiter = ',')]
    pour: Vec<String>,
    /// Clearance in mm of the areas of --pour to copper of other nets.
    #[arg(long, default_value_t = AreaOptions::default().zone_clearance)]
    zone_clearance: f64,
    /// Minimum width in mm of the fill in the areas of --pour.
    #[arg(long, default_value_t = AreaOptions::default().zone_min_width)]
    zone_min_width: f64,
    /// Gap in mm of the thermal reliefs in the areas of --pour.
    #[arg(long, default_value_t = AreaOptions::default().thermal_gap)]
    thermal_gap: f64,
    /// Width in mm of the thermal spokes in the areas of --pour.
    #[arg(long, default_value_t = AreaOptions::default().thermal_spoke_width)]
    thermal_spoke_width: f64,
    /// How the pads of the net join the areas of --pour: thermal, solid, thermal_tht or none.
    #[arg(long, default_value_t = AreaOptions::default().pad_connection)]
    pad_connection: String,
    /// Raster step wanted in mm; the step used divides the minimum width into an odd number of
    /// cells.
    #[arg(long, default_value_t = AreaOptions::default().step)]
    step: f64,
    /// List the pads whose answer changes when every clearance moves by half a raster cell
    /// (two more runs of the model per net). On by default; this flag insists on it.
    #[arg(long, conflicts_with = "no_doubtful")]
    doubtful: bool,
    /// Leave the doubtful pads out: one run of the model per net.
    #[arg(long)]
    no_doubtful: bool,
    /// Print JSON instead of text.
    #[arg(long)]
    json: bool,
    /// One sentence per net.
    #[arg(long)]
    brief: bool,
    /// Write the fill per net and layer as PREFIX.<net>.<layer>.pgm (white: fill of the area,
    /// grey: other fill of the net, black: none).
    #[arg(long, value_name = "PREFIX")]
    pgm: Option<String>,
}

impl AreasArgs {
    /// The options of the report as the arguments give them.
    fn options(&self, plane_layers: &[String]) -> AreaOptions {
        AreaOptions {
            nets: self.nets.clone(),
            pour: self.pour.clone(),
            zone_clearance: self.zone_clearance,
            zone_min_width: self.zone_min_width,
            thermal_gap: self.thermal_gap,
            thermal_spoke_width: self.thermal_spoke_width,
            pad_connection: self.pad_connection.clone(),
            plane_layers: plane_layers.to_vec(),
            step: self.step,
            doubtful: self.doubtful || !self.no_doubtful,
        }
    }
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
    Route(Box<RouteArgs>),
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
        #[arg(long, default_value_t = PlaceOptions::default().seed)]
        seed: u64,
        /// Independent runs in parallel; the one with the fewest crossings is kept. The crossings
        /// hardly predict the routed result, so for another attempt change --seed instead.
        #[arg(long, default_value_t = PlaceOptions::default().starts)]
        starts: usize,
        /// Annealing moves per run.
        #[arg(long, default_value_t = PlaceOptions::default().iters)]
        iters: usize,
        /// Spacing between courtyards in mm.
        #[arg(long, default_value_t = PlaceOptions::default().gap)]
        gap: f64,
        /// Courtyards keep this distance from the outline in mm.
        #[arg(long, default_value_t = PlaceOptions::default().margin)]
        margin: f64,
        /// Extra room around parts with five or more pads in mm, for their escape.
        #[arg(long, default_value_t = PlaceOptions::default().halo)]
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
        #[arg(long, default_value_t = PlaceOptions::default().leash)]
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
        #[arg(long, default_value_t = PlaceOptions::default().back_max_pads)]
        back_max_pads: usize,
        /// With --two-sided: cost of a part on the back in mm of wire. A part goes to the back
        /// if the front has no room for it or the back saves at least this much (0: both sides
        /// alike).
        #[arg(long, default_value_t = PlaceOptions::default().back_cost)]
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
    /// What the copper fill of the nets with a zone (pours, planes) reaches on the copper the
    /// board has: pieces of fill, islands with their pads and the site of one via for each,
    /// the pads the fill does not reach and why. The fill is the router's own model on a
    /// raster; KiCad's fill has the last word.
    Areas(AreasArgs),
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
        Cmd::Route(args) => {
            use gummiband_app::Event;
            let opts = args.options(&cli.plane_layers);
            let RouteArgs {
                board,
                output,
                svg,
                svg_tris,
                report,
                progress,
                until_stop,
                ..
            } = *args;
            // the options against each other, before a word is printed
            opts.validate(until_stop).map_err(anyhow::Error::msg)?;
            if gummiband_core::knobs::GPU.on() && !cfg!(feature = "gpu") {
                println!("warning: GB_GPU has no effect, this binary is built without the gpu feature");
            }
            let t0 = std::time::Instant::now();
            gummiband_core::patterns::hints(&opts.patterns).map_err(|e| anyhow::anyhow!("--patterns: {e}"))?;
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
            let progress_fn = |prefix: &str| progress_lines(t0, prefix.to_string());
            let event = |e: Event<'_>| match e {
                Event::NetWidths { nets } => println!("net widths: {nets} nets"),
                Event::CorridorsFree => println!("corridors: no part in the way"),
                Event::CorridorParts { parts } => println!(
                    "corridors: {} parts in the way get new positions: {}",
                    parts.len(),
                    parts.join(" ")
                ),
                Event::CorridorsCleared { min_separation } => {
                    println!("corridors: smallest courtyard distance {min_separation:.2} mm")
                }
                Event::PlacementTried { index, count, trial: t } => println!(
                    "placement {index}/{count}, seed {}: {}/{} routed, {} violations, {} vias, {:.0} mm",
                    t.seed, t.routed, t.connections, t.violations, t.vias, t.length
                ),
                Event::PlacementKept { seed } => println!("best placement: seed {seed}"),
                Event::MovableParts { count } => println!("movable parts: {count}"),
            };
            let hooks = Hooks {
                stop: open_end.as_ref(),
                progress: if progress { Some(&progress_fn) } else { None },
                event: Some(&event),
            };
            let outcome = gummiband_app::route_file(&board, &output, &opts, &hooks)?;
            let r = &outcome.routed;
            let (b, sub, res) = (&r.board, &r.sub, &r.res);
            if !outcome.skipped_nets.is_empty() {
                println!("nets left out: {}", outcome.skipped_nets.len());
            }
            for a in &r.areas {
                println!("areas: {}", gummiband_app::areas::route_line(a));
            }
            if !r.areas.is_empty() {
                println!(
                    "areas: {} vias set for islands of fill; the fill model ran {} times in the search, {:.1} s of CPU",
                    r.area_vias, r.area_model_runs, r.area_model_seconds
                );
            }
            for t in r.tries.iter().skip(1) {
                println!(
                    "give way within {} mm: {} nets, {} open, {} violations",
                    t.radius, t.given, t.open, t.violations
                );
            }
            if r.tries.len() > 1 {
                let k = &r.tries[r.kept];
                println!(
                    "kept: {} nets gave way, {} open, {} violations",
                    k.given, k.open, k.violations
                );
            }
            let mut moved: Vec<_> = res.offsets.iter().collect();
            moved.sort_by(|a, b| a.0.cmp(b.0));
            for (f, d) in moved {
                println!("moved {f} by {:.2}, {:.2} mm", d.x, d.y);
            }
            if let Some(room) = &r.room {
                println!(
                    "room for meanders: {} nets, {}, {:.1} s",
                    room.nets,
                    if room.made {
                        "made"
                    } else {
                        "none without new violations"
                    },
                    room.seconds
                );
            }
            if let Some(a) = &r.angles_45 {
                let st = &a.stats;
                println!(
                    "45 degrees: {} tracks, {:.0} -> {:.0} mm, {} pieces off the grid ({:.1} mm), {} sharp corners, violations {} -> {}, {:.1} s",
                    st.tracks,
                    st.length_before,
                    st.length_after,
                    st.any_angle,
                    st.any_angle_length,
                    st.sharp_corners,
                    a.violations_before,
                    a.violations_after,
                    a.seconds
                );
            }
            if let Some(t) = &r.tuned {
                let rep = &t.report;
                let short: Vec<_> = rep.nets.iter().filter(|n| n.aim - n.after > 0.005).collect();
                println!(
                    "length tuning: {} nets, {} reached their length, {:.1} s",
                    rep.nets.len(),
                    rep.nets.len() - short.len(),
                    t.seconds
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
            }
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
            // the seed of the search: the same one on the same input gives this result again
            println!(
                "best: seed {} {}/{} routed, {} vias, {:.0} mm, {:.2} s",
                r.seed, s.routed, s.conns, s.vias, s.length, r.seconds
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
            if let Some(near) = gummiband_core::knobs::NEAR.text() {
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
            for r in &r.pairs {
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
            let matching = &r.matching;
            print_matching(matching);
            let viol = &r.violations;
            let spots = gummiband_core::check::spots(&viol).len();
            println!("clearance check: {} violations at {spots} spots", viol.len());
            let mut kinds = std::collections::BTreeMap::new();
            for v in viol {
                *kinds
                    .entry(v.what.split(' ').next().unwrap_or("").to_string())
                    .or_insert(0) += 1;
            }
            println!("  {kinds:?}");
            if gummiband_core::knobs::VIOL.on() {
                let mut hist = [0usize; 6];
                for v in viol {
                    let r = (v.actual / v.required).clamp(0.0, 0.999);
                    hist[(r * 6.0) as usize] += 1;
                }
                println!("  ratio histogram (0..1 in sixths): {hist:?}");
                let mut def = [0usize; 5];
                for v in viol {
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
            if let Some(path) = svg {
                std::fs::write(
                    &path,
                    gummiband_app::svg::render(b, Some(sub), &res.geometry, viol, svg_tris),
                )?;
            }
            let changes = &r.soft;
            if r.shove {
                println!(
                    "soft copper: {} tracks, {} vias and branch points; moved {} tracks of {} nets, at most {:.3} mm",
                    b.soft.chains.len(),
                    b.soft.nodes.len(),
                    changes.chains.iter().filter(|c| **c).count(),
                    changes.nets.len(),
                    changes.shift
                );
            }
            let written = outcome.write()?;
            // the results the open end leaves to choose from, next to the output
            for a in &written.alternatives {
                let st = &a.stats;
                println!(
                    "alternative: {}: {}/{} routed, {} vias, {:.0} mm, {} clearance spots",
                    a.path.display(),
                    st.routed,
                    st.conns,
                    st.vias,
                    st.length,
                    st.violations.unwrap_or(0)
                );
            }
            if let Some(f) = &written.frames {
                match &f.animation {
                    Some(gif) => println!("frames: {} images, animation {}", f.pngs, gif.display()),
                    None => println!("frames: {} pictures in {}", f.svgs, f.dir.display()),
                }
            }
            for path in &written.copied {
                println!("copied {}", path.display());
            }
            let items = written.items;
            println!("wrote {items} items to {}", output.display());
            if let Some(path) = report {
                let st = &res.stats;
                let json = serde_json::json!({
                    "board": outcome.input.display().to_string(),
                    "output": output.display().to_string(),
                    "seed": r.seed,
                    "seconds": r.seconds,
                    "search_seconds": r.search_seconds,
                    "placement": outcome.placement.as_ref().map(placement_json),
                    "connections": st.conns,
                    "routed": st.routed,
                    "unrouted": st.unrouted,
                    "pairs": r.pairs,
                    "matching": matching,
                    "vias": st.vias,
                    "length_mm": st.length,
                    "length_lower_bound_mm": gummiband_core::length_lower_bound(b),
                    "patterns": gummiband_core::patterns::find(b).named(b),
                    "angles_45": r.angles_45.as_ref().map(|a| a.stats),
                    "length_tuning": r.tuned.as_ref().map(|t| &t.report),
                    "items_written": items,
                    "alternatives": written.alternatives.iter().map(|a| serde_json::json!({
                        "path": a.path.display().to_string(),
                        "routed": a.stats.routed,
                        "connections": a.stats.conns,
                        "vias": a.stats.vias,
                        "length_mm": a.stats.length,
                        "clearance_spots": a.stats.violations,
                    })).collect::<Vec<_>>(),
                    "copied": written.copied.iter().map(|p| p.display().to_string()).collect::<Vec<_>>(),
                    "skipped_nets": outcome.skipped_nets,
                    "cleared_corridors": outcome.cleared_corridors,
                    "pours": r.legal.pours,
                    "areas": r.areas,
                    "area_vias": r.area_vias,
                    "area_model_seconds": r.area_model_seconds,
                    "area_model_runs": r.area_model_runs,
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
            use gummiband_place::auto::Scope;
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
                std::fs::write(&path, gummiband_app::svg::render(&ob, None, &geo, &viol, false))?;
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
        Cmd::Areas(args) => {
            let report = gummiband_app::area_report(&args.board, &args.options(&cli.plane_layers))?;
            if args.json {
                println!("{}", serde_json::to_string_pretty(&report)?);
            } else {
                print!("{}", report.text(args.brief));
            }
            if let Some(prefix) = &args.pgm {
                for (n, f) in report.nets.iter().zip(&report.fills) {
                    let safe: String = n
                        .net
                        .chars()
                        .map(|c| if c.is_alphanumeric() { c } else { '_' })
                        .collect();
                    for l in &n.layers {
                        let layer =
                            f.fill.iter().enumerate().position(|(i, m)| {
                                m.is_some() && report.layer_names.get(i).is_some_and(|x| *x == l.layer)
                            });
                        if let Some(picture) = layer.and_then(|i| gummiband_app::areas::fill_picture(f, i)) {
                            std::fs::write(format!("{prefix}.{safe}.{}.pgm", l.layer), picture)?;
                        }
                    }
                }
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

/// The winner of --placements with every trial, for the report.
fn placement_json(p: &gummiband_app::Placement) -> serde_json::Value {
    serde_json::json!({
        "seed": p.seed,
        "moved": p.report.moved,
        "zones": p.report.zones.iter().map(|(id, pts)| {
            let pts: Vec<[f64; 2]> = pts.iter().map(|p| [p.x, p.y]).collect();
            serde_json::json!({ "id": id, "points": pts })
        }).collect::<Vec<_>>(),
        "tried": p.trials.iter().map(|t| serde_json::json!({
            "seed": t.seed,
            "routed": t.routed,
            "connections": t.connections,
            "violations": t.violations,
            "vias": t.vias,
            "length_mm": t.length,
        })).collect::<Vec<_>>(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::CommandFactory;
    use gummiband_app::{Kind, OPTIONS};

    /// Arguments of `route` that are the CLI's own: where the result and its reports go, and
    /// the stop line of the open end. Everything else has to be an option of the pipeline.
    const OWN: &[&str] = &["board", "output", "svg", "svg-tris", "report", "progress", "until-stop"];

    /// The long names of `route`, with the global ones.
    fn route_flags() -> Vec<(String, Vec<String>)> {
        flags_of("route")
    }

    /// The long names of a subcommand, with the global ones.
    fn flags_of(sub: &str) -> Vec<(String, Vec<String>)> {
        let cli = Cli::command();
        let route = cli.find_subcommand(sub).expect(sub);
        cli.get_arguments()
            .chain(route.get_arguments())
            .filter(|a| a.get_id() != "threads")
            .map(|a| {
                let name = a.get_long().map(str::to_string).unwrap_or(a.get_id().to_string());
                let defaults = a
                    .get_default_values()
                    .iter()
                    .map(|v| v.to_string_lossy().into_owned())
                    .collect();
                (name, defaults)
            })
            .collect()
    }

    /// Every option of the pipeline is a flag of `route` under its name, with the default of
    /// the table, and `route` has no option of the pipeline that the table lacks. The MCP server
    /// has the same test against the same table, so the two front ends offer the same.
    #[test]
    fn route_offers_the_options_of_the_pipeline() {
        let flags = route_flags();
        let has = |name: &str| flags.iter().any(|(f, _)| f == name);
        for o in OPTIONS {
            let flag = o.flag();
            let Some((_, defaults)) = flags.iter().find(|(f, _)| *f == flag) else {
                panic!("route lacks --{flag}");
            };
            if o.kind == Kind::OnOff {
                assert!(has(&format!("no-{flag}")), "route lacks --no-{flag}");
            }
            // a default clap prints is the default of the table
            if let [d] = defaults.as_slice()
                && o.kind != Kind::Switch
                && o.kind != Kind::OnOff
            {
                assert_eq!(d, o.default, "default of --{flag}");
            }
        }
        for (flag, _) in &flags {
            let name = flag.strip_prefix("no-").unwrap_or(flag).replace('-', "_");
            assert!(
                OWN.contains(&flag.as_str()) || gummiband_app::option_doc(&name).is_some(),
                "--{flag} is neither in gummiband_app::OPTIONS nor one of the CLI's own arguments"
            );
        }
    }

    /// The same for `areas` and the options of the area report: every entry of the table is a
    /// flag with the table's default, and `areas` has no option that the table lacks. The MCP
    /// tool `area_report` has the same test.
    #[test]
    fn areas_offers_the_options_of_the_report() {
        const OWN: &[&str] = &["board", "json", "brief", "pgm"];
        let flags = flags_of("areas");
        let has = |name: &str| flags.iter().any(|(f, _)| f == name);
        for o in gummiband_app::AREA_OPTIONS {
            let flag = o.flag();
            let Some((_, defaults)) = flags.iter().find(|(f, _)| *f == flag) else {
                panic!("areas lacks --{flag}");
            };
            if o.kind == Kind::OnOff {
                assert!(has(&format!("no-{flag}")), "areas lacks --no-{flag}");
            }
            if let [d] = defaults.as_slice()
                && o.kind != Kind::Switch
                && o.kind != Kind::OnOff
            {
                assert_eq!(d, o.default, "default of --{flag}");
            }
        }
        for (flag, _) in &flags {
            let name = flag.strip_prefix("no-").unwrap_or(flag).replace('-', "_");
            assert!(
                OWN.contains(&flag.as_str()) || gummiband_app::area_option_doc(&name).is_some(),
                "--{flag} is neither in gummiband_app::AREA_OPTIONS nor one of the CLI's own arguments"
            );
        }
        // with nothing given the report runs with the defaults of the table
        let args = |extra: &[&str]| {
            let mut argv = vec!["gummiband", "areas", "in.kicad_pcb"];
            argv.extend(extra);
            let cli = Cli::try_parse_from(argv).unwrap();
            let Cmd::Areas(a) = cli.cmd else { panic!("areas") };
            a.options(&cli.plane_layers)
        };
        assert_eq!(args(&[]), AreaOptions::default());
        let o = args(&[
            "--net",
            "GND,GNDR",
            "--pour",
            "F.Cu,B.Cu",
            "--zone-min-width",
            "0.15",
            "--pad-connection",
            "solid",
            "--no-doubtful",
            "--plane-layers",
            "In1.Cu",
        ]);
        assert_eq!(
            (o.nets, o.pour),
            (
                vec!["GND".to_string(), "GNDR".into()],
                vec!["F.Cu".to_string(), "B.Cu".into()]
            )
        );
        assert_eq!(
            (o.zone_min_width, o.pad_connection.as_str(), o.doubtful),
            (0.15, "solid", false)
        );
        assert_eq!(o.plane_layers, ["In1.Cu"]);
    }

    fn route_args(extra: &[&str]) -> (RouteOptions, bool) {
        let mut argv = vec!["gummiband", "route", "in.kicad_pcb", "-o", "out.kicad_pcb"];
        argv.extend(extra);
        let cli = Cli::try_parse_from(argv).unwrap();
        let Cmd::Route(args) = cli.cmd else {
            panic!("route");
        };
        (args.options(&cli.plane_layers), args.until_stop)
    }

    /// With nothing given the CLI routes with the defaults of the pipeline, and what is given
    /// arrives under its name.
    #[test]
    fn arguments_become_the_options_of_the_pipeline() {
        let (opts, until_stop) = route_args(&[]);
        assert_eq!(format!("{opts:?}"), format!("{:?}", RouteOptions::default()));
        assert!(!until_stop);
        let (opts, _) = route_args(&[
            "--keep-all",
            "--nets",
            "A,B",
            "--reroute",
            "--no-shove",
            "--give",
            "2",
            "--no-tune",
            "--open-end-seconds",
            "30",
            "--plane-layers",
            "In1.Cu",
            "--frames",
            "pictures",
        ]);
        assert!(opts.keep_all && opts.reroute && !opts.tune);
        assert_eq!(opts.nets, ["A", "B"]);
        assert_eq!((opts.shove, opts.give, opts.move_parts), (Some(false), Some(2.0), None));
        assert_eq!(opts.open_end_seconds, 30.0);
        assert_eq!(opts.plane_layers, ["In1.Cu"]);
        assert_eq!(opts.frames, Some(PathBuf::from("pictures")));
        assert_eq!(opts.validate(false), Ok(()));
        // a pour is one entry per flag: the comma belongs to its layers
        let (opts, _) = route_args(&["--areas", "fill", "--pour", "GND:F.Cu,B.Cu", "--pour", "A"]);
        assert!(opts.areas == gummiband_app::Areas::Fill);
        assert_eq!(opts.pour, ["GND:F.Cu,B.Cu", "A"]);
        assert_eq!(opts.validate(false), Ok(()));
        let (opts, until_stop) = route_args(&["--areas", "fill", "--until-stop"]);
        assert!(opts.validate(until_stop).unwrap_err().contains("open ended"));
        // contradictions are the pipeline's to refuse, in its words
        let (opts, until_stop) = route_args(&["--until-stop", "--pairs"]);
        assert!(opts.validate(until_stop).unwrap_err().contains("pairs"));
        let (opts, _) = route_args(&["--runs", "0"]);
        assert_eq!(opts.validate(false), Err("runs must be at least 1".to_string()));
    }
}
