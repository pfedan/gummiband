//! Route a board with a set of options: from the loaded board to the result that is ready to be
//! written and reported, with the steps before the search (parts out of track corridors, a search
//! over placements) and after it (room for meanders, 45 degree angles, length tuning, checks).
//!
//! Nothing here prints. What a front end shows while the pipeline runs comes through [`Hooks`]
//! (progress of the search, [`Event`]s of the steps before it), everything else is data in
//! [`Routed`] and [`RouteOutcome`].

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::AtomicBool;
use std::time::Instant;

use anyhow::{Context, Result};
use gummiband_core::board::{Board, NetId};
use gummiband_core::realize::{Elem, Geometry};
use gummiband_core::route::RouterConfig;
use gummiband_core::topo::{Substrate, SubstrateConfig};
use gummiband_core::{GiveTry, ProgressFn, RouteResult, SearchConfig};
use gummiband_kicad::{KeepCopper, KicadBoard, ReadOptions, read_board, write};
use gummiband_place::auto::{PlaceOptions, PlaceReport};

use crate::nets::{NetOptions, apply_net_options, read_net_options, read_net_priorities};

/// The defaults of [`RouteOptions`], each defined once: `RouteOptions::default()` is built from
/// them, the CLI names them as its clap defaults and the MCP server falls back to them.
pub mod defaults {
    /// Portfolio size.
    pub const RUNS: usize = gummiband_core::DEFAULT_RUNS;
    /// Via site grid pitch in mm.
    pub const VIA_PITCH: f64 = 0.5;
    /// Via cost in mm of track.
    pub const VIA_COST: f64 = 3.0;
    /// Weight of the preferred layer directions.
    pub const LAYER_DIRS: f64 = 0.5;
    /// Hints from local patterns.
    pub const PATTERNS: &str = "none";
    /// Factor on the via cost in the via pass.
    pub const VIA_PASS: f64 = 4.0;
    /// Weight of the pull between the two nets of a differential pair, with `pairs` pull.
    pub const PAIR_PULL: f64 = 1.0;
    /// Length difference left within a pair in mm.
    pub const PAIR_SKEW: f64 = 0.1;
    /// Rounds of negotiated routing from the start.
    pub const NEGOTIATE: usize = 0;
    /// Generations of the order search.
    pub const GENERATIONS: usize = 3;
    /// Generations that select by clearance violations.
    pub const GEOMETRY_GENERATIONS: usize = 3;
    /// Placements to try before routing.
    pub const PLACEMENTS: usize = 0;
    /// First placement seed.
    pub const PLACEMENT_SEED: u64 = 1;
    /// What a via is worth in mm of track when the open end judges its results.
    pub const OPEN_VIA_COST: f64 = gummiband_core::DEFAULT_OPEN_VIA_COST;
    /// Width in mm that tracks of the default class grow to (0: off).
    pub const PREFER_WIDTH: f64 = 0.0;
    /// Highest meander bump in mm.
    pub const TUNE_AMPLITUDE: f64 = 1.0;
    /// Air between neighbouring meander legs in track widths.
    pub const TUNE_SPACING: f64 = 2.0;
    /// Give way radius in mm when finishing nets.
    pub const GIVE: f64 = gummiband_core::DEFAULT_GIVE;
    /// Reserve in mm on top of the clearances in an open ended run that routes the board.
    pub const OPEN_END_AIR: f64 = 0.03;
    /// Seconds the open end keeps improving (0: no open end with a time budget).
    pub const OPEN_END_SECONDS: f64 = 0.0;
}

/// Track angles of a routed result.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Angles {
    /// Any-angle tracks with arcs, as the rubber bands lie.
    #[default]
    Any,
    /// Conventional 45 degree routing as a last step.
    Deg45,
}

/// What of the copper on the board stays, and what is laid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Scope {
    /// No copper stays, locked or not; the board is laid anew.
    #[default]
    Fresh,
    /// Locked tracks and vias stay as fixed copper, all unlocked copper goes and is laid anew.
    KeepLocked,
    /// All copper stays and what is still open is laid; with `nets` only those nets.
    Finish,
    /// The unlocked copper of `nets` goes and they are laid anew; everything else stays.
    Reroute,
}

impl Scope {
    pub const ALL: [Scope; 4] = [Scope::Fresh, Scope::KeepLocked, Scope::Finish, Scope::Reroute];

    /// The word of the option `scope` ([`crate::OPTIONS`]).
    pub fn word(self) -> &'static str {
        match self {
            Scope::Fresh => "fresh",
            Scope::KeepLocked => "keep_locked",
            Scope::Finish => "finish",
            Scope::Reroute => "reroute",
        }
    }

    /// The value of a word; the error names the words there are.
    pub fn parse(word: &str) -> Result<Scope, String> {
        let word = crate::option_doc("scope").expect("scope").choice(word)?;
        Ok(*Scope::ALL
            .iter()
            .find(|s| s.word() == word)
            .expect("a word of the table"))
    }

    /// Whether the copper of the board stays, unlocked copper too: the router adds to it.
    pub fn keeps_all(self) -> bool {
        matches!(self, Scope::Finish | Scope::Reroute)
    }
}

/// How much search a route gets: a named bundle of `runs`, `generations`,
/// `geometry_generations` and `open_end_seconds`. Each of the four still has its own option,
/// and an option that is given wins over the preset. Every value here was measured or is a
/// default of old; a preset tunes nothing that the four options do not.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Preset {
    /// The settings of the benchmark suite and the hash lists: a look at whether the board
    /// routes at all.
    Draft,
    /// The whole order search without the geometry generations: what the plugin's Finish action
    /// runs.
    Quick,
    /// The defaults.
    #[default]
    Standard,
    /// The defaults, then the open end for ten minutes.
    Final,
}

/// What a [`Preset`] sets.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PresetValues {
    pub runs: usize,
    pub generations: usize,
    pub geometry_generations: usize,
    pub open_end_seconds: f64,
}

impl Preset {
    pub const ALL: [Preset; 4] = [Preset::Draft, Preset::Quick, Preset::Standard, Preset::Final];

    /// The word of the option `preset` ([`crate::OPTIONS`]).
    pub fn word(self) -> &'static str {
        match self {
            Preset::Draft => "draft",
            Preset::Quick => "quick",
            Preset::Standard => "standard",
            Preset::Final => "final",
        }
    }

    /// The value of a word; the error names the words there are.
    pub fn parse(word: &str) -> Result<Preset, String> {
        let word = crate::option_doc("preset").expect("preset").choice(word)?;
        Ok(*Preset::ALL
            .iter()
            .find(|p| p.word() == word)
            .expect("a word of the table"))
    }

    /// The four values of the preset.
    pub fn values(self) -> PresetValues {
        let standard = PresetValues {
            runs: defaults::RUNS,
            generations: defaults::GENERATIONS,
            geometry_generations: defaults::GEOMETRY_GENERATIONS,
            open_end_seconds: defaults::OPEN_END_SECONDS,
        };
        match self {
            // bench/run_suite.sh and bench/hashes.sh route every board with these
            Preset::Draft => PresetValues {
                runs: 4,
                generations: 1,
                geometry_generations: 0,
                ..standard
            },
            // about a third of the time of the defaults, and many more clearance violations
            // (Arty 32 instead of 1, ULX3S 54 instead of 4, LED controller 18 instead of 4)
            Preset::Quick => PresetValues {
                geometry_generations: 0,
                ..standard
            },
            Preset::Standard => standard,
            // the open end of the LED controller had 93 % of its saving after 10 minutes, and
            // two runs of it converged in 9 (docs/MILESTONES.md)
            Preset::Final => PresetValues {
                open_end_seconds: 600.0,
                ..standard
            },
        }
    }
}

/// How the two nets of a differential pair are laid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Pairs {
    /// Each net on its own, like any other net.
    #[default]
    Off,
    /// Two nets, the second pulled to the way of the first ([`RouteOptions::pair_pull`]); the
    /// finished tracks close up to the pair gap where nothing lies between.
    Pull,
    /// One wide track per pair that is split into two parallel tracks.
    Band,
}

impl Pairs {
    pub const ALL: [Pairs; 3] = [Pairs::Off, Pairs::Pull, Pairs::Band];

    /// The word of the option `pairs` ([`crate::OPTIONS`]).
    pub fn word(self) -> &'static str {
        match self {
            Pairs::Off => "off",
            Pairs::Pull => "pull",
            Pairs::Band => "band",
        }
    }

    /// The value of a word; the error names the words there are.
    pub fn parse(word: &str) -> Result<Pairs, String> {
        let word = crate::option_doc("pairs").expect("pairs").choice(word)?;
        Ok(*Pairs::ALL
            .iter()
            .find(|p| p.word() == word)
            .expect("a word of the table"))
    }
}

/// Every option of the route pipeline. `Default` gives the documented defaults ([`defaults`]).
/// Options that are `None` follow from the others, see the methods of the same name.
#[derive(Debug, Clone)]
pub struct RouteOptions {
    /// What of the copper on the board stays, and what is laid.
    pub scope: Scope,
    /// Lay only these nets (empty: all). [`Scope::Reroute`] needs them.
    pub nets: Vec<String>,
    /// Single nets on kept copper ([`RouteOptions::finishing`]): the unlocked tracks of the
    /// other nets are part of the topology and shift aside. None: on there unless parts move, see
    /// [`RouteOptions::shove`].
    pub shove: Option<bool>,
    /// Single nets on kept copper: radius in mm within which other nets give way. None:
    /// [`defaults::GIVE`] there, else 0, see [`RouteOptions::give`].
    pub give: Option<f64>,
    /// Let the geometry generations shift parts. None: on with the scopes that lay the board
    /// anew, off with those that keep its copper, see [`RouteOptions::move_parts`].
    pub move_parts: Option<bool>,
    /// References that never move under `move_parts`, `placements` and `clear_corridors`.
    pub fixed: Vec<String>,
    /// Copper layers treated as planes besides those of KiCad type "power".
    pub plane_layers: Vec<String>,
    /// How much search the route gets: the values of `runs`, `generations`,
    /// `geometry_generations` and `open_end_seconds` where those are None.
    pub preset: Preset,
    /// Portfolio size. None: what the preset says, see [`RouteOptions::runs`].
    pub runs: Option<usize>,
    pub via_pitch: f64,
    pub via_cost: f64,
    pub layer_dirs: f64,
    /// Hints from local patterns, comma separated (`gummiband_core::patterns::hints`).
    pub patterns: String,
    pub via_pass: f64,
    /// How differential pairs are laid.
    pub pairs: Pairs,
    /// Weight of the pull between the two nets of a pair. None: [`defaults::PAIR_PULL`] with
    /// `pairs` pull, and 0 with band, where it is the pull for the pairs that get no wide track,
    /// see [`RouteOptions::pair_pull`].
    pub pair_pull: Option<f64>,
    /// With `pairs` band, as are the four below.
    pub pair_skew: f64,
    pub pair_zdiff: Option<f64>,
    pub pair_all: bool,
    pub breakouts: bool,
    pub via_pairs: bool,
    pub negotiate: usize,
    /// Generations of the order search. None: what the preset says, see
    /// [`RouteOptions::generations`].
    pub generations: Option<usize>,
    /// Generations that select by clearance violations. None: what the preset says, see
    /// [`RouteOptions::geometry_generations`].
    pub geometry_generations: Option<usize>,
    pub seed: Option<u64>,
    /// Keep the best candidate of every generation, and [`RouteOutcome::write`] draws them into
    /// this folder ([`Routed::write_frames`]).
    pub frames: Option<PathBuf>,
    /// Open end with a time budget: after the normal search the result keeps improving for
    /// this many seconds, then the best one is finished (0: off). The budget starts when the
    /// open end starts, so it never cuts the normal search short. [`Hooks::stop`] is the open
    /// end without a budget; with both, whichever comes first ends the run. None: what the
    /// preset says, see [`RouteOptions::open_end_seconds`].
    pub open_end_seconds: Option<f64>,
    pub open_via_cost: f64,
    /// Reserve in mm on top of the clearances. None: [`defaults::OPEN_END_AIR`] in an open ended
    /// run that does not finish nets, else 0, see [`RouteOptions::air`].
    pub air: Option<f64>,
    pub prefer_width: f64,
    pub angles: Angles,
    /// Meanders for the length and skew rules.
    pub tune: bool,
    pub tune_amplitude: f64,
    pub tune_spacing: f64,
    /// [`route_file`]: try this many placements, route each and go on with the best.
    pub placements: usize,
    /// First seed of `placements`, and the seed of `clear_corridors`.
    pub placement_seed: u64,
    /// [`route_file`]: parts in the band of a group of tracks get new positions first.
    pub clear_corridors: bool,
    /// [`load_board`]: options per net from a JSON file, `{"net name": width_mm}` or
    /// `{"net name": {"width": .., "min_width": .., "neck_refs": [..], "priority": .., "skip": ..}}`
    /// ([`NetOptions`]).
    pub net_widths: Option<PathBuf>,
    /// [`load_board`]: routing priority per net from a JSON file {"net name": priority}; it
    /// wins over a priority in `net_widths`.
    pub net_priorities: Option<PathBuf>,
    /// [`load_board`]: options per net that the front end holds itself (the session of the MCP
    /// server). Field by field they win over the two files.
    pub net_options: BTreeMap<String, NetOptions>,
    /// Switch on KiCad's teardrops in the written board.
    pub teardrops: bool,
}

impl Default for RouteOptions {
    fn default() -> Self {
        RouteOptions {
            scope: Scope::Fresh,
            nets: Vec::new(),
            shove: None,
            give: None,
            move_parts: None,
            fixed: Vec::new(),
            plane_layers: Vec::new(),
            preset: Preset::Standard,
            runs: None,
            via_pitch: defaults::VIA_PITCH,
            via_cost: defaults::VIA_COST,
            layer_dirs: defaults::LAYER_DIRS,
            patterns: defaults::PATTERNS.to_string(),
            via_pass: defaults::VIA_PASS,
            pairs: Pairs::Off,
            pair_pull: None,
            pair_skew: defaults::PAIR_SKEW,
            pair_zdiff: None,
            pair_all: false,
            breakouts: false,
            via_pairs: false,
            negotiate: defaults::NEGOTIATE,
            generations: None,
            geometry_generations: None,
            seed: None,
            frames: None,
            open_end_seconds: None,
            open_via_cost: defaults::OPEN_VIA_COST,
            air: None,
            prefer_width: defaults::PREFER_WIDTH,
            angles: Angles::Any,
            tune: true,
            tune_amplitude: defaults::TUNE_AMPLITUDE,
            tune_spacing: defaults::TUNE_SPACING,
            placements: defaults::PLACEMENTS,
            placement_seed: defaults::PLACEMENT_SEED,
            clear_corridors: false,
            net_widths: None,
            net_priorities: None,
            net_options: BTreeMap::new(),
            teardrops: false,
        }
    }
}

impl RouteOptions {
    /// Single nets are laid on a board whose copper stays: scope reroute, or scope finish with
    /// `nets`. The defaults of `shove`, `give` and `air` hang on it.
    pub fn finishing(&self) -> bool {
        self.scope.keeps_all() && !self.nets.is_empty()
    }

    /// Parts move by default where the board is laid anew (scope fresh and keep_locked); on a
    /// board whose copper is kept (finish, reroute) they stay unless asked.
    pub fn move_parts(&self) -> bool {
        self.move_parts.unwrap_or(!self.scope.keeps_all())
    }

    /// Where single nets are laid on kept copper, the unlocked tracks of the other nets shift
    /// aside unless switched off or parts move.
    pub fn shove(&self) -> bool {
        self.shove.unwrap_or(self.finishing() && !self.move_parts())
    }

    /// Where single nets are laid on kept copper, other nets give way within [`defaults::GIVE`]
    /// mm unless told otherwise.
    pub fn give(&self) -> f64 {
        self.give.unwrap_or(if self.finishing() { defaults::GIVE } else { 0.0 })
    }

    /// The reserve on the clearances. Single nets on kept copper move only what the new tracks
    /// touch, also open ended, so every other open ended route gets one by default.
    pub fn air(&self, open_end: bool) -> f64 {
        self.air.unwrap_or(if open_end && !self.finishing() {
            defaults::OPEN_END_AIR
        } else {
            0.0
        })
    }

    /// The weight of the pull between the two nets of a pair: with `pairs` pull
    /// [`defaults::PAIR_PULL`] unless told otherwise, with band what was given (there it pulls the
    /// pairs that get no wide track together), without pairs none.
    pub fn pair_pull(&self) -> f64 {
        match self.pairs {
            Pairs::Off => 0.0,
            Pairs::Pull => self.pair_pull.unwrap_or(defaults::PAIR_PULL),
            Pairs::Band => self.pair_pull.unwrap_or(0.0),
        }
    }

    /// Portfolio size: what was given, else what the preset says.
    pub fn runs(&self) -> usize {
        self.runs.unwrap_or(self.preset.values().runs)
    }

    /// Generations of the order search: what was given, else what the preset says.
    pub fn generations(&self) -> usize {
        self.generations.unwrap_or(self.preset.values().generations)
    }

    /// Generations that select by clearance violations: what was given, else what the preset
    /// says.
    pub fn geometry_generations(&self) -> usize {
        self.geometry_generations
            .unwrap_or(self.preset.values().geometry_generations)
    }

    /// Seconds of the open end (0: none): what was given, else what the preset says.
    pub fn open_end_seconds(&self) -> f64 {
        self.open_end_seconds.unwrap_or(self.preset.values().open_end_seconds)
    }

    /// Whether the run is open ended: by its time budget, or because the front end may stop it
    /// (`until_stop`, [`Hooks::stop`]).
    pub fn open_end(&self, until_stop: bool) -> bool {
        until_stop || self.open_end_seconds() > 0.0
    }

    /// Check the options against each other; `until_stop` says that the front end may stop the
    /// run from outside, which makes it open ended like `open_end_seconds`. Options that
    /// contradict each other are an error that says what to change; nothing is adjusted
    /// silently. An option that was left out never contradicts: its default follows from the
    /// others. [`route_board`] and [`route_file`] call this first, so every front end gets the
    /// same message. It names the options as the fields here; the CLI spells them `--kebab-case`.
    pub fn validate(&self, until_stop: bool) -> Result<(), String> {
        let finishing = self.finishing();
        let open_end = self.open_end(until_stop);
        let err = |m: &str| Err(m.to_string());
        if self.runs() == 0 {
            return err("runs must be at least 1");
        }
        if self.via_pitch.is_nan() || self.via_pitch <= 0.0 {
            return err("via_pitch must be above 0");
        }
        if self.give.is_some_and(|g| g.is_nan() || g < 0.0) {
            return err("give must not be negative");
        }
        if self.open_end_seconds().is_nan() || self.open_end_seconds() < 0.0 {
            return err("open_end_seconds must not be negative");
        }
        if self.scope == Scope::Reroute && self.nets.is_empty() {
            return err(
                "scope reroute needs nets: name the nets to lay again, or write scope finish to lay what is open",
            );
        }
        // shove and give act where single nets are laid on kept copper
        let single = "write scope finish with nets, or scope reroute";
        if self.shove == Some(true) && !finishing {
            return Err(format!(
                "shove needs single nets on kept copper: {single}, or leave shove out"
            ));
        }
        if self.give.is_some_and(|g| g > 0.0) && !finishing {
            return Err(format!(
                "give needs single nets on kept copper: {single}, or leave give out"
            ));
        }
        if finishing && self.move_parts == Some(true) && self.shove != Some(false) {
            return err("move_parts does not work with shove: switch shove off, or leave move_parts out");
        }
        if self.pair_pull.is_some_and(|w| w.is_nan() || w < 0.0) {
            return err("pair_pull must not be negative");
        }
        let band = self.pairs == Pairs::Band;
        if self.pairs == Pairs::Off && self.pair_pull.is_some_and(|w| w > 0.0) {
            return err("pair_pull needs pairs pull: write pairs pull, or leave pair_pull out");
        }
        if self.pairs == Pairs::Pull && self.pair_pull == Some(0.0) {
            return err(
                "pairs pull with pair_pull 0 pulls nothing: write pairs off, or leave pair_pull out (the weight is then 1)",
            );
        }
        if !band {
            let given = [
                ("pair_zdiff", self.pair_zdiff.is_some()),
                ("pair_all", self.pair_all),
                ("breakouts", self.breakouts),
                ("via_pairs", self.via_pairs),
            ];
            if let Some((name, _)) = given.iter().find(|(_, on)| *on) {
                return Err(format!(
                    "{name} acts on the wide track of pairs band only: write pairs band, or leave {name} out"
                ));
            }
        }
        if open_end && band {
            return err(
                "an open ended run does not work with pairs band: write pairs pull or off, or leave out the open end (until_stop, open_end_seconds, preset final)",
            );
        }
        if self.placements > 0 && (self.scope.keeps_all() || !self.nets.is_empty() || band) {
            return err("placements routes the whole board: not with scope finish or reroute, nets or pairs band");
        }
        if self.clear_corridors && self.scope.keeps_all() {
            return err(
                "clear_corridors places parts before the whole board is routed: not with scope finish or reroute",
            );
        }
        Ok(())
    }

    /// The copper of the input that stays.
    pub fn keep(&self) -> KeepCopper {
        match self.scope {
            Scope::Fresh => KeepCopper::None,
            Scope::KeepLocked => KeepCopper::Locked,
            Scope::Finish | Scope::Reroute => KeepCopper::All,
        }
    }

    /// The nets whose unlocked copper goes before routing.
    pub fn reroute_nets(&self) -> Vec<String> {
        if self.scope == Scope::Reroute {
            self.nets.clone()
        } else {
            Vec::new()
        }
    }

    /// How the board file is read for these options.
    pub fn read_options(&self) -> ReadOptions {
        ReadOptions {
            keep: self.keep(),
            plane_layers: self.plane_layers.clone(),
            move_parts: self.move_parts(),
            fixed: self.fixed.clone(),
            ignore_nets: self.reroute_nets(),
            ..Default::default()
        }
    }

    /// How parts are placed for these options (`clear_corridors`, `placements`): the references
    /// that stay, the seed and the plane layers.
    pub fn place_options(&self) -> PlaceOptions {
        PlaceOptions {
            fixed: self.fixed.clone(),
            seed: self.placement_seed,
            plane_layers: self.plane_layers.clone(),
            ..Default::default()
        }
    }

    fn substrate_config(&self) -> SubstrateConfig {
        SubstrateConfig {
            via_pitch: self.via_pitch,
            ..Default::default()
        }
    }

    fn router_config(&self) -> Result<RouterConfig> {
        let hints = gummiband_core::patterns::hints(&self.patterns).map_err(anyhow::Error::msg)?;
        Ok(RouterConfig {
            via_cost: self.via_cost,
            layer_dirs: self.layer_dirs,
            pair_pull: self.pair_pull(),
            via_pass: self.via_pass,
            negotiate: self.negotiate,
            ..Default::default()
        }
        .with_hints(&hints))
    }

    /// The search of the core for these options. `prefix` goes in front of each phase the
    /// front end is told.
    fn search_config(&self, hooks: &Hooks, prefix: &str) -> SearchConfig {
        let open_end = self.open_end(hooks.stop.is_some());
        // the flag that ends the open end: the front end's, set by its user, or one for the
        // time budget alone
        let stop = open_end.then(|| hooks.stop.cloned().unwrap_or_default());
        let told = hooks.progress.map(|p| p(prefix));
        let progress = match &stop {
            Some(stop) if self.open_end_seconds() > 0.0 => {
                Some(with_budget(told, stop.clone(), self.open_end_seconds()))
            }
            _ => told,
        };
        SearchConfig {
            population: self.runs(),
            generations: self.generations(),
            geometry_generations: self.geometry_generations(),
            frames: self.frames.is_some(),
            seed: self.seed,
            open_end: stop,
            air: self.air(open_end),
            prefer_width: self.prefer_width,
            open_via_cost: self.open_via_cost,
            progress,
        }
    }

    fn tune_config(&self) -> gummiband_core::tune::TuneConfig {
        gummiband_core::tune::TuneConfig {
            style: if self.angles == Angles::Deg45 {
                gummiband_core::tune::Style::Chamfer
            } else {
                gummiband_core::tune::Style::Round
            },
            amplitude: self.tune_amplitude,
            spacing: self.tune_spacing,
        }
    }
}

/// The progress callback of a search whose open end has a time budget: when the search says
/// that its open end begins, the clock starts, and `seconds` later `stop` is set. Everything is
/// passed on to `told`.
fn with_budget(told: Option<ProgressFn>, stop: Arc<AtomicBool>, seconds: f64) -> ProgressFn {
    use std::sync::atomic::Ordering;
    let started = AtomicBool::new(false);
    ProgressFn(Arc::new(move |p: &gummiband_core::Progress| {
        if p.phase == gummiband_core::OPEN_END_PHASE && !started.swap(true, Ordering::Relaxed) {
            let stop = stop.clone();
            std::thread::spawn(move || {
                std::thread::sleep(std::time::Duration::from_secs_f64(seconds));
                stop.store(true, Ordering::Relaxed);
            });
        }
        if let Some(f) = &told {
            (f.0)(p);
        }
    }))
}

/// What a front end hooks into a run. All of it is optional.
#[derive(Default, Clone, Copy)]
pub struct Hooks<'a> {
    /// Open end without a time budget: after the normal search the result keeps improving
    /// until the front end sets this flag (a stop line of the user). None: the run ends on its
    /// own, after `open_end_seconds` if that is set.
    pub stop: Option<&'a Arc<AtomicBool>>,
    /// Makes the progress callback of a search; the argument goes in front of each phase name
    /// (the routes of `placements` name their placement there).
    pub progress: Option<&'a dyn Fn(&str) -> ProgressFn>,
    /// Called for the steps before the search, as they happen.
    pub event: Option<&'a dyn Fn(Event<'_>)>,
}

impl Hooks<'_> {
    fn emit(&self, e: Event<'_>) {
        if let Some(f) = self.event {
            f(e);
        }
    }
}

/// A step before the search, told to the front end when it happens.
#[derive(Debug, Clone, Copy)]
pub enum Event<'a> {
    /// `net_widths` gave this many nets of the board their width.
    NetWidths { nets: usize },
    /// `clear_corridors`: no part sits in a track corridor.
    CorridorsFree,
    /// `clear_corridors`: these parts are in the way and get new positions.
    CorridorParts { parts: &'a [String] },
    /// `clear_corridors`: they are placed.
    CorridorsCleared { min_separation: f64 },
    /// `placements`: placement `index` of `count` (from 1) is routed.
    PlacementTried {
        index: usize,
        count: usize,
        trial: &'a Trial,
    },
    /// `placements`: the placement of this seed is routed for the result.
    PlacementKept { seed: u64 },
    /// `move_parts`: so many parts may move.
    MovableParts { count: usize },
}

/// Room made for the meanders of the length tuning.
#[derive(Debug, Clone, Copy)]
pub struct Room {
    /// Nets that want room.
    pub nets: usize,
    /// False: none without new violations, the geometry stayed.
    pub made: bool,
    pub seconds: f64,
}

/// The 45 degree step.
#[derive(Debug, Clone, Copy)]
pub struct Angles45 {
    pub stats: gummiband_core::octo::OctoStats,
    /// Clearance violations before and after the step.
    pub violations_before: usize,
    pub violations_after: usize,
    pub seconds: f64,
}

/// The length tuning.
#[derive(Debug, Clone)]
pub struct Tuned {
    pub report: gummiband_core::tune::TuneReport,
    pub seconds: f64,
}

/// A routed board: the result of the search with the steps after it, and what the reports of the
/// front ends are built from.
pub struct Routed {
    /// The board the geometry belongs to: only the nets routed have connections, and moved
    /// parts are where they ended up.
    pub board: Board,
    pub sub: Substrate,
    /// Geometry, stats, portfolio, frames, part offsets and the alternatives of the open end.
    pub res: RouteResult,
    /// One report per differential pair (`pairs` band).
    pub pairs: Vec<gummiband_core::pairs::PairReport>,
    /// The nets whose unlocked copper gave way: the result replaces it.
    pub given: std::collections::BTreeSet<String>,
    /// Every try of giving way in order, and the one kept.
    pub tries: Vec<GiveTry>,
    pub kept: usize,
    /// Whether the unlocked tracks of other nets were part of the topology.
    pub shove: bool,
    /// What moved of that soft copper in the result.
    pub soft: gummiband_core::import::SoftChanges,
    /// The seed the search ran with: `seed` of the options, else `GB_EVO_SEED`, else the
    /// constant of the core. The same seed on the same input gives the same result.
    pub seed: u64,
    /// How long the route took, in seconds of wall time: from the call to the result with every
    /// step after the search done and checked. For [`route_file`] that includes reading the
    /// board, clearing corridors and trying placements; writing the files is not included.
    /// Every front end reports this number as `seconds`.
    pub seconds: f64,
    /// The part of `seconds` up to the end of the search: without room for meanders, the 45
    /// degree step, length tuning and the checks.
    pub search_seconds: f64,
    pub room: Option<Room>,
    pub angles_45: Option<Angles45>,
    pub tuned: Option<Tuned>,
    /// The length, skew and uncoupled rules against the result.
    pub matching: Vec<gummiband_core::matching::RuleMatch>,
    /// Clearance violations of the result.
    pub violations: Vec<gummiband_core::check::Violation>,
    /// What the last check on the finished copper took out or found open (its open connections
    /// are in `res.stats.unrouted` as well).
    pub legal: gummiband_core::legal::Legal,
    /// The same for every alternative of the open end, in their order.
    pub legal_alternatives: Vec<gummiband_core::legal::Legal>,
    write: WriteOptions,
}

/// What of the options the writer needs.
#[derive(Debug, Clone)]
struct WriteOptions {
    scope: Scope,
    reroute: Vec<String>,
    teardrops: bool,
}

/// Route `board` with `opts`: the search, then room for meanders, 45 degree angles, length
/// tuning, and the checks. `board` is the board model as the front end prepared it (net widths
/// and priorities, skipped nets); [`Routed::write`] puts the result into its file. Options about files
/// (`placements`, `clear_corridors`, `net_widths`, `net_priorities`) belong to [`route_file`]
/// and [`load_board`] and do nothing here.
///
/// Debugging switches of the environment: `GB_SHIFT="R8:0.3,0;C4:0,-0.2"` routes the board with
/// these parts shifted, `GB_NO_TUNE_ROOM` skips the room for meanders, `GB_TIMING` prints the
/// phase timers to stderr.
pub fn route_board(board: &Board, opts: &RouteOptions, hooks: &Hooks) -> Result<Routed> {
    route_from(Instant::now(), board, opts, hooks)
}

fn route_from(t0: Instant, board: &Board, opts: &RouteOptions, hooks: &Hooks) -> Result<Routed> {
    opts.validate(hooks.stop.is_some()).map_err(anyhow::Error::msg)?;
    let sub_cfg = opts.substrate_config();
    let router_cfg = opts.router_config()?;
    let search_cfg = opts.search_config(hooks, "");
    let seed = search_cfg.seed_used();
    let with_zdiff;
    let band = opts.pairs == Pairs::Band;
    let board = match (band, opts.pair_zdiff) {
        (true, Some(z)) => {
            let mut b = board.clone();
            gummiband_core::pairs::apply_zdiff(&mut b, z);
            with_zdiff = b;
            &with_zdiff
        }
        _ => board,
    };
    let shifted_in;
    let b = match gummiband_core::knobs::SHIFT.text() {
        Some(spec) => {
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
            shifted_in = board.with_offsets(&off);
            &shifted_in
        }
        None => board,
    };
    if opts.move_parts() {
        hooks.emit(Event::MovableParts { count: b.movable.len() });
    }
    let tune_cfg = opts.tune_config();
    let deg45 = opts.angles == Angles::Deg45;
    let tune_room = opts.tune && !gummiband_core::knobs::NO_TUNE_ROOM.on();
    let shove = opts.shove();
    // the nets asked for, and the nets that give way when a connection finds no way
    let finished = gummiband_core::finish_nets(b, &opts.nets, shove, opts.give(), &search_cfg, |board, search| {
        let out = if band {
            let pair_cfg = gummiband_core::pairs::PairConfig {
                skew: opts.pair_skew,
                zdiff: opts.pair_zdiff,
                all: opts.pair_all,
                room: tune_room.then(|| tune_cfg.clone()),
                angles45: deg45,
                breakouts: opts.breakouts,
                via_pairs: opts.via_pairs,
            };
            gummiband_core::pairs::route_pairs(board, &sub_cfg, &router_cfg, search, &pair_cfg)
        } else {
            let (sub, res) = gummiband_core::route_board(board, &sub_cfg, &router_cfg, search);
            (sub, res, Vec::new())
        };
        gummiband_core::prof::report();
        out
    });
    let gummiband_core::Finished {
        board: routed_board,
        sub,
        mut res,
        extra: pair_reports,
        given,
        tries,
        kept,
    } = finished;
    let search_seconds = t0.elapsed().as_secs_f64();
    // the geometry belongs to the board with the shifted parts
    let final_board = if res.offsets.is_empty() {
        routed_board
    } else {
        routed_board.with_offsets(&res.offsets)
    };
    // an exit piece that no track took is no copper: the steps from here on see the pads
    // without those pieces, as the last steps of the search did
    let released = gummiband_core::exit::release_unused(&final_board, &res.geometry);
    let final_board = released.unwrap_or(final_board);
    let b = &final_board;
    // room for the meanders: the copper next to the nets that want length steps aside
    // (the pairs make it while they are one wide track)
    let mut room = None;
    if !band && tune_room && !b.match_rules.is_empty() {
        let t1 = Instant::now();
        let want = gummiband_core::tune::room_wanted(b, &res.geometry, &tune_cfg, &[]);
        if !want.is_empty() {
            let made = gummiband_core::tune::make_room(b, &sub, &res.geometry, &want);
            room = Some(Room {
                nets: want.len(),
                made: made.is_some(),
                seconds: t1.elapsed().as_secs_f64(),
            });
            if let Some(g) = made {
                res.geometry = g;
            }
        }
    }
    let angles_45 = deg45.then(|| {
        let violations_before = gummiband_core::check::check(b, &res.geometry).len();
        let t1 = Instant::now();
        // the two tracks of a pair that went onto the grid as one wide track stay as
        // they are; their ends and what was routed on its own afterwards do not
        let wide: Vec<NetId> = pair_reports
            .iter()
            .filter(|r| r.wide)
            .flat_map(|r| [b.net_id(&r.p), b.net_id(&r.n)])
            .flatten()
            .collect();
        let stats = gummiband_core::octo::octilinear_only(
            b,
            &mut res.geometry,
            &|t| !(wide.contains(&t.net) && t.seg == u32::MAX),
            &|_, _| false,
        );
        let seconds = t1.elapsed().as_secs_f64();
        let violations_after = gummiband_core::check::check(b, &res.geometry).len();
        Angles45 {
            stats,
            violations_before,
            violations_after,
            seconds,
        }
    });
    let tuned = (opts.tune && !b.match_rules.is_empty()).then(|| {
        let t1 = Instant::now();
        let report = gummiband_core::tune::tune(b, &mut res.geometry, &tune_cfg);
        Tuned {
            report,
            seconds: t1.elapsed().as_secs_f64(),
        }
    });
    // up to here the exit pieces of the pads were part of their pads (`exit.rs`); from here on
    // they are tracks of the route, and the pads are the pads of the file
    let plain = (!b.exits.is_empty()).then(|| {
        let stubs = |g: &mut Geometry| {
            let s = gummiband_core::exit::stub_tracks(b, g);
            g.tracks.extend(s);
        };
        stubs(&mut res.geometry);
        res.alternatives.iter_mut().for_each(|(_, g)| stubs(g));
        res.frames.iter_mut().for_each(|(_, g, _)| stubs(g));
        b.without_exits()
    });
    let final_board = plain.unwrap_or(final_board);
    let b = &final_board;
    // the last word, on the finished copper: no short between two nets is written, and a
    // connection counts as routed only if the copper holds together
    let legal = gummiband_core::legal::enforce(b, &mut res.geometry, &mut res.stats);
    // the alternatives of the open end are written as boards too
    let legal_alternatives = res
        .alternatives
        .iter_mut()
        .map(|(stats, geometry)| gummiband_core::legal::enforce(b, geometry, stats))
        .collect();
    let matching = gummiband_core::matching::check(b, &res.geometry);
    let violations = gummiband_core::check::check(b, &res.geometry);
    let soft = gummiband_core::import::soft_changes(b, &res.geometry);
    Ok(Routed {
        board: final_board,
        sub,
        res,
        pairs: pair_reports,
        given,
        tries,
        kept,
        shove,
        soft,
        seed,
        seconds: t0.elapsed().as_secs_f64(),
        search_seconds,
        room,
        angles_45,
        tuned,
        matching,
        violations,
        legal,
        legal_alternatives,
        write: WriteOptions {
            scope: opts.scope,
            reroute: opts.reroute_nets(),
            teardrops: opts.teardrops,
        },
    })
}

/// A pad whose row leaves a track no legal way out, see [`Routed::tight_pads`].
#[derive(Debug, Clone, serde::Serialize)]
pub struct TightPad {
    /// "U3.12"
    pub pad: String,
    pub net: String,
    /// Width of the piece that leaves the pad, in mm.
    pub width_mm: f64,
    /// How much the footprint lacks for that width and the clearance, in µm.
    pub missing_um: f64,
}

impl std::fmt::Display for TightPad {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "pad {} ({}): the row leaves a track of {:.3} mm {:.0} µm less than its clearance",
            self.pad, self.net, self.width_mm, self.missing_um
        )
    }
}

impl Routed {
    /// The pads of the routed nets where the footprint itself leaves a track no legal way out:
    /// on the middle line of the pad, at the narrowest width the rules allow there, it is nearer
    /// to the next pad of another net than its clearance. No route can mend that; the rules or
    /// the footprint can.
    pub fn tight_pads(&self) -> Vec<TightPad> {
        let b = &self.board;
        b.tight_pads
            .iter()
            .map(|t| {
                let pin = &b.pins[t.pin];
                TightPad {
                    pad: pin.name.clone(),
                    net: b.nets[pin.net as usize].name.clone(),
                    width_mm: t.width,
                    missing_um: t.missing * 1000.0,
                }
            })
            .collect()
    }

    /// The board file with `geometry` (the result's own, or one of its alternatives) in place
    /// of the copper that goes, and the number of items written. Soft copper that rests stays
    /// in the file as it is; the meanders of the tuning become KiCad tuning patterns.
    ///
    /// Every board of a route is written here, so the writer looks once more: a track of the
    /// route that shorts another net is not written, whatever way the geometry came
    /// (`gummiband_core::legal::without_shorts`). After [`route_board`] there is none.
    pub fn board_text(&self, kb: &KicadBoard, geometry: &Geometry) -> Result<(String, usize)> {
        let b = &self.board;
        let without = gummiband_core::legal::without_shorts(b, geometry);
        let geometry = without.as_ref().unwrap_or(geometry);
        // an alternative has its own soft copper that moved
        let changes = gummiband_core::import::soft_changes(b, geometry);
        let rests = |origin: Option<u32>, list: &[bool]| origin.is_some_and(|o| !list[o as usize]);
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
        for p in self.tuned.iter().flat_map(|r| &r.report.patterns) {
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
        let w = &self.write;
        let mut nets = w.reroute.clone();
        nets.extend(self.given.iter().cloned());
        let opts = write::StripOptions {
            unlocked: !w.scope.keeps_all(),
            locked: w.scope == Scope::Fresh,
            zone_layers: vec![],
            nets,
            items: changes.strip.clone(),
        };
        let ranges = write::strip_ranges(kb, &opts);
        let text = write::patch(kb, &ranges, &items);
        let text = if self.res.offsets.is_empty() {
            text
        } else {
            write::move_footprints(&text, &self.res.offsets, &kb.board.blocks)?
        };
        let text = if w.teardrops {
            write::enable_teardrops(&text)?
        } else {
            text
        };
        Ok((text, items.len()))
    }

    /// Write [`Routed::board_text`] to `path`; with `teardrops` the project next to it gets
    /// them at track ends too (`input` is the board file whose project it starts from). Returns
    /// the number of items.
    pub fn write(&self, kb: &KicadBoard, geometry: &Geometry, input: &Path, path: &Path) -> Result<usize> {
        let (text, items) = self.board_text(kb, geometry)?;
        std::fs::write(path, text)?;
        if self.write.teardrops {
            track_end_teardrops(input, path)?;
        }
        Ok(items)
    }
}

/// The pictures of a search, see [`Routed::write_frames`].
#[derive(Debug, Clone)]
pub struct Frames {
    pub dir: PathBuf,
    /// One SVG per generation, `frame-NNN.svg`.
    pub svgs: usize,
    /// As many PNGs next to them, if `rsvg-convert` is there.
    pub pngs: usize,
    /// `evolution.gif` of the PNGs, if ImageMagick's `magick` is there.
    pub animation: Option<PathBuf>,
}

impl Routed {
    /// Draw the best candidate of every generation into `dir` (the search kept them with
    /// `frames` set): `frame-NNN.svg`, a PNG of each if `rsvg-convert` is there, and
    /// `evolution.gif` if ImageMagick's `magick` is.
    pub fn write_frames(&self, dir: &Path) -> Result<Frames> {
        use std::process::{Command, Stdio};
        let b = &self.board;
        std::fs::create_dir_all(dir)?;
        let mut pngs = Vec::new();
        for (i, (label, geo, relaxed)) in self.res.frames.iter().enumerate() {
            let v = if *relaxed {
                gummiband_core::check::check(b, geo)
            } else {
                Vec::new()
            };
            let svg_text = crate::svg::render(b, None, geo, &v, false).replace(
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
            // a front end may speak a protocol on its standard output: the tools stay silent
            let ok = Command::new("rsvg-convert")
                .args(["-w", "1200", "-o"])
                .arg(&png)
                .arg(&f)
                .stdin(Stdio::null())
                .output()
                .map(|o| o.status.success())
                .unwrap_or(false);
            if ok {
                pngs.push(png);
            }
        }
        let mut animation = None;
        if !pngs.is_empty() {
            let gif = dir.join("evolution.gif");
            let made = Command::new("magick")
                .args(["-delay", "120", "-loop", "0"])
                .args(&pngs)
                .arg(&gif)
                .stdin(Stdio::null())
                .output()
                .map(|o| o.status.success())
                .unwrap_or(false);
            if made {
                animation = Some(gif);
            }
        }
        Ok(Frames {
            dir: dir.to_path_buf(),
            svgs: self.res.frames.len(),
            pngs: pngs.len(),
            animation,
        })
    }
}

/// Teardrops at track width changes are a project setting: the output's .kicad_pro (a copy of
/// the input's if it has none yet) switches them on. The input's project stays as it is.
fn track_end_teardrops(input: &Path, output: &Path) -> Result<()> {
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
    copy_project_files(input, output);
    Ok(())
}

/// Copy `<from>.kicad_pro` and `<from>.kicad_dru` next to the board file `to` (same base name)
/// where they are missing: KiCad and the reader take net classes and custom rules from them. A
/// file that is already there stays as it is. Returns the files copied.
pub fn copy_project_files(from: &Path, to: &Path) -> Vec<PathBuf> {
    let mut copied = Vec::new();
    for ext in ["kicad_pro", "kicad_dru"] {
        let s = from.with_extension(ext);
        let d = to.with_extension(ext);
        if s != d && s.is_file() && !d.exists() && std::fs::copy(&s, &d).is_ok() {
            copied.push(d);
        }
    }
    copied
}

/// Whether two paths name the same file, also when one of them does not exist yet.
fn same_file(a: &Path, b: &Path) -> bool {
    let full = |p: &Path| {
        p.canonicalize().unwrap_or_else(|_| {
            let dir = p
                .parent()
                .filter(|d| !d.as_os_str().is_empty())
                .unwrap_or(Path::new("."));
            dir.canonicalize()
                .unwrap_or_else(|_| dir.to_path_buf())
                .join(p.file_name().unwrap_or_default())
        })
    };
    full(a) == full(b)
}

/// Read a board file as `opts` ask: the copper to keep, planes, movable parts, then the options
/// per net (`net_widths`, `net_priorities`, `net_options`). Fails for a net of `opts.nets` the
/// board lacks.
pub fn load_board(path: &Path, opts: &RouteOptions, hooks: &Hooks) -> Result<KicadBoard> {
    Ok(load(path, opts, hooks)?.0)
}

/// [`load_board`], and the nets left out of the route.
fn load(path: &Path, opts: &RouteOptions, hooks: &Hooks) -> Result<(KicadBoard, Vec<String>)> {
    let mut kb = read_board(path, &opts.read_options())?;
    for n in &opts.nets {
        if kb.board.net_id(n).is_none() {
            anyhow::bail!("no net named {n:?}");
        }
    }
    let mut per_net: BTreeMap<String, NetOptions> = BTreeMap::new();
    let mut add = |more: &BTreeMap<String, NetOptions>| {
        for (net, o) in more {
            per_net.entry(net.clone()).or_default().merge(o);
        }
    };
    if let Some(path) = &opts.net_widths {
        let file = read_net_options(path)?;
        let on_board = file.keys().filter(|n| kb.board.net_id(n).is_some()).count();
        hooks.emit(Event::NetWidths { nets: on_board });
        add(&file);
    }
    if let Some(path) = &opts.net_priorities {
        add(&read_net_priorities(path)?);
    }
    add(&opts.net_options);
    let applied = apply_net_options(&mut kb.board, &per_net, &opts.nets)?;
    Ok((kb, applied.skipped))
}

/// The parts that [`clear_corridors`] placed again.
pub struct Cleared {
    pub parts: Vec<String>,
    /// The board file with the parts at their new positions.
    pub text: String,
    /// Smallest courtyard distance of the placement in mm.
    pub min_separation: f64,
}

/// Parts in the band of a group of tracks between two parts, and not on its nets, get new
/// positions; everything else stays. None: no part is in the way. `place` gives the fixed
/// references, the seed and the plane layers. The environment may override the placement for
/// experiments: `GB_CC_GAP`, `GB_CC_HALO`, `GB_CC_LEASH` (mm), `GB_CC_ESCAPE` and
/// `GB_CC_SEMANTIC` (above 0: on).
pub fn clear_corridors(from: &Path, place: &PlaceOptions, hooks: &Hooks) -> Result<Option<Cleared>> {
    use gummiband_place::auto::{CLEAR_CORRIDORS_LEASH, Scope, corridor_intruders, place_board};
    let intruders = corridor_intruders(from, place)?;
    if intruders.is_empty() {
        hooks.emit(Event::CorridorsFree);
        return Ok(None);
    }
    hooks.emit(Event::CorridorParts { parts: &intruders });
    use gummiband_core::knobs;
    let placed = place_board(
        from,
        &PlaceOptions {
            scope: Scope::Only(intruders.clone()),
            corridors: true,
            gap: knobs::CC_GAP.get::<f64>().unwrap_or(place.gap),
            halo: knobs::CC_HALO.get::<f64>().unwrap_or(place.halo),
            escape_halo: knobs::CC_ESCAPE.get::<f64>().is_some_and(|v| v > 0.0),
            semantic: knobs::CC_SEMANTIC.get::<f64>().is_some_and(|v| v > 0.0),
            leash: knobs::CC_LEASH.get::<f64>().unwrap_or(CLEAR_CORRIDORS_LEASH),
            ..place.clone()
        },
    )?;
    hooks.emit(Event::CorridorsCleared {
        min_separation: placed.report.min_separation,
    });
    Ok(Some(Cleared {
        parts: intruders,
        text: placed.text,
        min_separation: placed.report.min_separation,
    }))
}

/// One placement of `placements`, routed.
#[derive(Debug, Clone)]
pub struct Trial {
    pub seed: u64,
    pub routed: usize,
    pub connections: usize,
    pub violations: usize,
    pub vias: usize,
    pub length: f64,
}

impl Trial {
    /// Lower is better: completion first, then clean copper, then vias and length.
    fn score(&self) -> (usize, usize, usize, f64) {
        (self.connections - self.routed, self.violations, self.vias, self.length)
    }
}

/// The winner of `placements`.
pub struct Placement {
    /// The placed board, the final route reads it.
    pub path: PathBuf,
    pub report: PlaceReport,
    pub trials: Vec<Trial>,
    pub seed: u64,
}

/// Place the board `count` times and let `route` judge every result. The seeds run one after the
/// other: each route uses all threads itself.
fn try_placements(
    input: &Path,
    dir: &Path,
    count: usize,
    opts: &PlaceOptions,
    hooks: &Hooks,
    route: &dyn Fn(&Path, u64, usize) -> Result<Trial>,
) -> Result<Placement> {
    std::fs::create_dir_all(dir)?;
    let mut best: Option<Placement> = None;
    let mut trials: Vec<Trial> = Vec::new();
    for i in 0..count {
        let seed = opts.seed + i as u64;
        let placed = gummiband_place::auto::place_board(input, &PlaceOptions { seed, ..opts.clone() })?;
        let path = dir.join(format!("placement-{seed}.kicad_pcb"));
        std::fs::write(&path, &placed.text)?;
        copy_rules(input, &path)?;
        let t = route(&path, seed, i + 1)?;
        hooks.emit(Event::PlacementTried {
            index: i + 1,
            count,
            trial: &t,
        });
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
    hooks.emit(Event::PlacementKept { seed: best.seed });
    best.trials = trials;
    Ok(best)
}

/// The router reads the .kicad_pro and .kicad_dru next to a board: copy them next to `to`.
fn copy_rules(from: &Path, to: &Path) -> Result<()> {
    for ext in ["kicad_pro", "kicad_dru"] {
        let src = from.with_extension(ext);
        if src.exists() {
            std::fs::copy(&src, to.with_extension(ext))?;
        }
    }
    Ok(())
}

/// A routed board file, see [`route_file`].
pub struct RouteOutcome {
    /// The board file that was given.
    pub input: PathBuf,
    /// The file [`RouteOutcome::write`] writes.
    pub output: PathBuf,
    /// The file that was routed: `input`, its copy with the track corridors cleared, or the
    /// placement kept.
    pub source: PathBuf,
    /// That file, read.
    pub kb: KicadBoard,
    /// Nets left out of the route: skipped by their options, or not among `nets`.
    pub skipped_nets: Vec<String>,
    /// `clear_corridors`: the parts that got new positions before the routing.
    pub cleared_corridors: Vec<String>,
    /// The winner of `placements` with every trial.
    pub placement: Option<Placement>,
    pub routed: Routed,
    /// The folder of `frames`.
    frames: Option<PathBuf>,
    /// The copies with cleared corridors and the placements live here until the result is
    /// written and dropped.
    _temp: TempDirs,
}

/// Folders that go when the route is over, however it ends.
struct TempDirs(Vec<PathBuf>);

impl Drop for TempDirs {
    fn drop(&mut self) {
        for dir in &self.0 {
            let _ = std::fs::remove_dir_all(dir);
        }
    }
}

/// Another result of the open end, written next to the output.
#[derive(Debug, Clone)]
pub struct Alternative {
    /// `<output>.altN.kicad_pcb`, N from 1.
    pub path: PathBuf,
    pub stats: gummiband_core::RunStats,
}

/// What [`RouteOutcome::write`] wrote.
#[derive(Debug, Clone)]
pub struct Written {
    pub output: PathBuf,
    /// Tracks, arcs, vias and tuning patterns in the output.
    pub items: usize,
    /// The results the open end leaves to choose from, those that differ from the output in
    /// vias and length.
    pub alternatives: Vec<Alternative>,
    /// The project files copied next to the output ([`copy_project_files`]).
    pub copied: Vec<PathBuf>,
    /// The pictures of the search, with `frames`.
    pub frames: Option<Frames>,
}

impl RouteOutcome {
    /// Write the result: the routed board to `output`, the project files of the input next to
    /// it where they are missing, and every alternative of the open end next to it as
    /// `<output>.altN.kicad_pcb`.
    pub fn write(&self) -> Result<Written> {
        let geometry = &self.routed.res.geometry;
        let copied = copy_project_files(&self.input, &self.output);
        let items = self.write_geometry(geometry, &self.output)?;
        let mut alternatives = Vec::new();
        for (i, (stats, geometry)) in self.routed.res.alternatives.iter().enumerate() {
            let path = self.output.with_extension(format!("alt{}.kicad_pcb", i + 1));
            self.write_geometry(geometry, &path)?;
            alternatives.push(Alternative {
                path,
                stats: stats.clone(),
            });
        }
        let frames = match &self.frames {
            Some(dir) => Some(self.routed.write_frames(dir)?),
            None => None,
        };
        Ok(Written {
            output: self.output.clone(),
            items,
            alternatives,
            copied,
            frames,
        })
    }

    /// Write the board with `geometry` (the result's own or one of the alternatives) to `path`.
    pub fn write_geometry(&self, geometry: &Geometry, path: &Path) -> Result<usize> {
        self.routed
            .write(&self.kb, geometry, &self.input, path)
            .with_context(|| format!("writing {}", path.display()))
    }
}

/// Route the board file `board` with `opts`, everything from the file to the result that is
/// ready to write to `output` ([`RouteOutcome::write`]): the checks of the options, parts out of
/// the track corridors (`clear_corridors`), the search over placements (`placements`),
/// [`load_board`], then [`route_board`]. The input stays as it is: `output` must be another file.
pub fn route_file(board: &Path, output: &Path, opts: &RouteOptions, hooks: &Hooks) -> Result<RouteOutcome> {
    let t0 = Instant::now();
    // fail on the options and the hints before any work
    opts.validate(hooks.stop.is_some()).map_err(anyhow::Error::msg)?;
    if same_file(board, output) {
        anyhow::bail!("the output must differ from the input file: the input stays as it is");
    }
    let router_cfg = opts.router_config()?;
    let sub_cfg = opts.substrate_config();
    let place = opts.place_options();
    let mut temp = TempDirs(Vec::new());
    // parts in the corridors of track groups move out of the way first
    let mut placed_from = board.to_path_buf();
    let mut cleared_corridors = Vec::new();
    if opts.clear_corridors
        && let Some(cleared) = clear_corridors(board, &place, hooks)?
    {
        let dir = std::env::temp_dir().join(format!("gummiband-corridors-{}", std::process::id()));
        temp.0.push(dir.clone());
        std::fs::create_dir_all(&dir)?;
        let path = dir.join(board.file_name().unwrap_or_default());
        std::fs::write(&path, &cleared.text)?;
        copy_rules(board, &path)?;
        placed_from = path;
        cleared_corridors = cleared.parts;
    }
    let mut placement = None;
    let source = if opts.placements > 0 {
        let placed_dir = std::env::temp_dir().join(format!("gummiband-placements-{}", std::process::id()));
        temp.0.push(placed_dir.clone());
        let count = opts.placements;
        let place = PlaceOptions { starts: 1, ..place };
        let found = try_placements(&placed_from, &placed_dir, count, &place, hooks, &|path, seed, i| {
            let kb = load_board(path, opts, hooks)?;
            let search = SearchConfig {
                frames: false,
                // only the winner runs open ended
                open_end: None,
                progress: hooks.progress.map(|p| p(&format!("placement {i}/{count}: "))),
                ..opts.search_config(hooks, "")
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
        })?;
        let path = found.path.clone();
        placement = Some(found);
        path
    } else {
        placed_from
    };
    let (kb, skipped_nets) = load(&source, opts, hooks)?;
    let routed = route_from(t0, &kb.board, opts, hooks)?;
    Ok(RouteOutcome {
        input: board.to_path_buf(),
        output: output.to_path_buf(),
        source,
        kb,
        skipped_nets,
        cleared_corridors,
        placement,
        routed,
        frames: opts.frames.clone(),
        _temp: temp,
    })
}
