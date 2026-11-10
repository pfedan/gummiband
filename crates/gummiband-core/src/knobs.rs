//! Every environment variable the router reads, in one table, with the accessors the code uses.
//!
//! The variables are for development and measurements: logs, switches that turn a default
//! behaviour off for a comparison, and knobs of experiments. They are no stable interface and
//! may change or go with any commit. `docs/ENVIRONMENT.md` is printed from this table (see
//! [`markdown`]); a test keeps the two the same.
//!
//! A variable is read once, the first time its accessor is called, and stays as it was for the
//! life of the process: the reads are cheap in hot paths, and a run cannot change under its own
//! feet. A program that sets a variable for itself (`examples/relax_bench.rs`) has to do so
//! before the first route. Nothing else in the core, kicad and place crates reads the
//! environment; variables that are not the router's own (`HOME`, KiCad's path variables) go
//! through [`foreign`], which does not cache.

use std::str::FromStr;
use std::sync::OnceLock;

/// What a variable is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// Output for the developer on stderr or into a file; the routed board stays the same.
    Log,
    /// Turns off (or back on) a behaviour that is on by default, for comparisons.
    Switch,
    /// A knob of an experiment or a measurement: a constant under test, an idea that is off by
    /// default, a seed or a limit for a benchmark.
    Experiment,
}

impl Kind {
    pub fn name(self) -> &'static str {
        match self {
            Kind::Log => "log",
            Kind::Switch => "switch",
            Kind::Experiment => "experiment",
        }
    }
}

/// How the value of a variable is understood. The rules differ from variable to variable for
/// historical reasons; each keeps the one it always had.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Read {
    /// On when the variable is set, to any value (also `0` or the empty string).
    Set,
    /// On unless the variable is exactly `0`; on when unset.
    NotZero,
    /// A number; the default of the table holds when the variable is unset or does not parse.
    Num,
    /// A value the reading code interprets (a name, a path, a list, a number whose default
    /// depends on the run).
    Value,
}

impl Read {
    /// The rule in words, for the documentation.
    pub fn words(self) -> &'static str {
        match self {
            Read::Set => "set to anything",
            Read::NotZero => "`0` turns it off",
            Read::Num => "number",
            Read::Value => "value",
        }
    }
}

/// One environment variable.
#[derive(Debug)]
pub struct Knob {
    pub name: &'static str,
    pub kind: Kind,
    pub read: Read,
    /// The state without the variable, in words or as the number that then holds.
    pub default: &'static str,
    /// One line on what it does.
    pub doc: &'static str,
    cell: OnceLock<Option<String>>,
}

/// Whether a flag is on for this value of its variable.
fn flag(read: Read, raw: Option<&str>) -> bool {
    match read {
        Read::Set => raw.is_some(),
        Read::NotZero => raw != Some("0"),
        Read::Num | Read::Value => unreachable!("not a flag"),
    }
}

impl Knob {
    const fn new(name: &'static str, kind: Kind, read: Read, default: &'static str, doc: &'static str) -> Knob {
        Knob {
            name,
            kind,
            read,
            default,
            doc,
            cell: OnceLock::new(),
        }
    }

    /// The value as the process was started with it; a value that is not Unicode counts as unset.
    pub fn text(&'static self) -> Option<&'static str> {
        self.cell.get_or_init(|| std::env::var(self.name).ok()).as_deref()
    }

    /// For a flag ([`Read::Set`], [`Read::NotZero`]): whether it is on.
    pub fn on(&'static self) -> bool {
        flag(self.read, self.text())
    }

    /// The value as a number, if the variable is set and parses as one.
    pub fn get<T: FromStr>(&'static self) -> Option<T> {
        self.text().and_then(|v| v.parse().ok())
    }

    /// For a number with a fixed default ([`Read::Num`]): the value, or the default of the table.
    pub fn num<T: FromStr>(&'static self) -> T {
        debug_assert_eq!(self.read, Read::Num, "{}", self.name);
        self.get().unwrap_or_else(|| {
            self.default
                .parse()
                .unwrap_or_else(|_| panic!("the default of {} is no number of this type", self.name))
        })
    }
}

/// A variable that is not the router's own (`HOME`, `APPDATA`, a path variable of KiCad named in
/// a library table). Read from the environment at every call.
pub fn foreign(name: &str) -> Option<String> {
    std::env::var(name).ok()
}

macro_rules! knobs {
    ($($id:ident = $name:literal, $kind:ident, $read:ident, $default:literal, $doc:literal;)*) => {
        $(
            #[doc = $doc]
            pub static $id: Knob = Knob::new($name, Kind::$kind, Read::$read, $default, $doc);
        )*
        /// All variables, in the order of the documentation.
        pub static TABLE: &[&Knob] = &[$(&$id),*];
    };
}

knobs! {
    // logs
    OPEN_LOG = "GB_OPEN_LOG", Log, Set, "off",
        "The open end reports its generations, side branches and room rounds on stderr.";
    PAIR_LOG = "GB_PAIR_LOG", Log, Set, "off",
        "Differential pairs: joins of the two tracks that failed, and why.";
    TUNE_LOG = "GB_TUNE_LOG", Log, Set, "off",
        "Length tuning: what each meander attempt reached.";
    NEG_LOG = "GB_NEG_LOG", Log, Set, "off",
        "Negotiated routing: conflicts and prices round by round.";
    ZIP_LOG = "GB_ZIP_LOG", Log, Set, "off",
        "What closing up the pairs at the end (`pairs::zip`) did.";
    PLANE_LOG = "GB_PLANE_LOG", Log, Set, "off",
        "Says when a new via of the open end would leave the plane of its net.";
    OCTO_DEBUG = "GB_OCTO_DEBUG", Log, Set, "off",
        "`--angles 45`: the pieces that stay at their angle, and what is in their way.";
    DEBUG_VIAS = "GB_DEBUG_VIAS", Log, Set, "off",
        "After a router run: via pairs close to each other and the vias with the least use.";
    DEBUG_CORRIDOR = "GB_DEBUG_CORRIDOR", Log, Set, "off",
        "After a router run: the fullest corridors of the topology.";
    DEBUG_X = "GB_DEBUG_X", Log, Set, "off",
        "After a router run: intersections left in the topology.";
    TIMING = "GB_TIMING", Log, Set, "off",
        "Cumulative phase timers and work counters at the end of a route, and the split of a relaxation.";
    VALIDATE = "GB_VALIDATE", Log, Set, "off",
        "After a router run: check the inner state of the router and print the problems.";
    VALIDATE_EACH = "GB_VALIDATE_EACH", Log, Set, "off",
        "Check the inner state after every routed connection and print the one that broke it (slow).";
    WHY = "GB_WHY", Log, Value, "unset",
        "A pin name (`U1.3`): trace the search of the first open connection from this pin into `/tmp/gb-why.txt`; `--svg` draws it.";
    MUTANT_LOG = "GB_MUTANT_LOG", Log, Value, "unset",
        "A file path: the open end writes one line of JSON per mutant (why it ended, what it changed).";
    NEAR = "GB_NEAR", Log, Value, "unset",
        "CLI `route`: `x,y[,radius]` in mm lists the tracks with a vertex near this point.";
    VIOL = "GB_VIOL", Log, Set, "off",
        "CLI `route`: a histogram of how much clearance the violations miss, and the worst ones.";

    // switches for behaviour that is on by default
    AUTO_NEGOTIATE = "GB_AUTO_NEGOTIATE", Switch, NotZero, "on",
        "A plain search that leaves at most 15 % of the connections open starts a negotiated search on its own.";
    TAPER = "GB_TAPER", Switch, NotZero, "on",
        "The last step tapers a track at a spot that misses its clearance by a little.";
    STITCH = "GB_STITCH", Switch, NotZero, "on",
        "Layer changes of current paths get the vias that carry their current (`relax::stitch_vias`).";
    ZIP = "GB_ZIP", Switch, NotZero, "on",
        "Pairs the search pulled together close up to their gap at the end (`pairs::zip`).";
    GOAL_CHECK = "GB_GOAL_CHECK", Switch, NotZero, "on",
        "A search asks early whether its goal is shut in. The results are the same without, only slower.";
    BANDIT = "GB_BANDIT", Switch, NotZero, "on",
        "Open end, combination A: the via price of a move is drawn by what has won on this board (Thompson sampling).";
    AIM_COST = "GB_AIM_COST", Switch, Num, "0.25",
        "Open end, combination A: this share of the moves aims at an expensive routed connection; `0` turns it off.";
    PRUNE = "GB_PRUNE", Switch, NotZero, "on",
        "Open end, combination A: a mutant that cannot beat its parent is dropped before its relaxation.";
    PLANE_HOLD = "GB_PLANE_HOLD", Switch, NotZero, "on",
        "Open end: a new via that reaches the plane of its net stays in it when it is moved.";
    NO_RELAX = "GB_NO_RELAX", Switch, Set, "off",
        "The search returns its best topology without the geometry stage (for `examples/relax_bench.rs`).";
    NO_POLISH = "GB_NO_POLISH", Switch, Set, "off",
        "Skips the polish relaxation of a winner that has clearance spots left.";
    NO_ARCS = "GB_NO_ARCS", Switch, Set, "off",
        "Tracks are realised as polylines, without arcs around the corners.";
    NO_TUNE_ROOM = "GB_NO_TUNE_ROOM", Switch, Set, "off",
        "CLI `route`: length tuning does not make room for its meanders first.";

    // seeds, limits and knobs of experiments
    EVO_SEED = "GB_EVO_SEED", Experiment, Value, "0x5eed",
        "Seed of the search when `--seed` (MCP: `seed`) is not given. The bench scripts use it.";
    GPU = "GB_GPU", Experiment, Set, "off",
        "Relaxation rounds on the GPU; needs a binary built with the `gpu` feature.";
    OPEN_GENERATIONS = "GB_OPEN_GENERATIONS", Experiment, Value, "no limit",
        "Benchmarks: the open end stops by itself after this many generations.";
    OPEN_POP = "GB_OPEN_POP", Experiment, Value, "the size of the portfolio",
        "Open end: mutants per generation.";
    LOCAL_IT = "GB_LOCAL_IT", Experiment, Num, "160",
        "Open end: relaxation rounds of a mutant.";
    TIGHTEN = "GB_TIGHTEN", Experiment, Num, "24",
        "Open end: every this many generations the whole board is relaxed again.";
    BRANCH_AFTER = "GB_BRANCH_AFTER", Experiment, Num, "24",
        "Open end: generations without a gain before a side branch starts.";
    BRANCH_LIFE = "GB_BRANCH_LIFE", Experiment, Num, "32",
        "Open end: generations a side branch gets.";
    BOLD_AIM = "GB_BOLD_AIM", Experiment, Num, "0.8",
        "Open end: share of the bold moves that aim at an open connection while there is one.";
    VIA_PRICE = "GB_VIA_PRICE", Experiment, Set, "off",
        "Open end idea: the via price of a move reaches up to what a via is worth in the cost of the result.";
    MERGE = "GB_MERGE", Experiment, Set, "off",
        "Open end idea: all mutants of a generation that win in places apart from each other count.";
    SPOT_FADE = "GB_SPOT_FADE", Experiment, Set, "off",
        "Open end idea: a clearance spot that many moves failed to clear gets fewer moves.";
    WINDOW = "GB_WINDOW", Experiment, Num, "0",
        "Open end idea: a mutant relaxes only the copper within this many mm of its new copper (0: the whole board).";
    AIR_IT = "GB_AIR_IT", Experiment, Num, "120",
        "Relaxation rounds of the steps that give copper air or make room (`--air`, room for tuning).";
    ROOM_ROUNDS = "GB_ROOM_ROUNDS", Experiment, Num, "6",
        "Rounds of making room for meanders before length tuning.";
    GEO_SCREEN = "GB_GEO_SCREEN", Experiment, Value, "off",
        "`k`: a geometry generation relaxes all candidates briefly and only the best k fully.";
    FINALISTS = "GB_FINALISTS", Experiment, Num, "24",
        "How many candidates of the search go on to the geometry stage.";
    FULL = "GB_FULL", Experiment, Num, "6",
        "How many finalists get the full relaxation after the short one.";
    DIRS_ALL = "GB_DIRS_ALL", Experiment, Set, "off",
        "Every run of the portfolio uses the preferred layer directions, not every third.";
    PLACE_ROUNDS = "GB_PLACE_ROUNDS", Experiment, Num, "2",
        "Rounds in which parts give way at hot spots (`--move-parts`).";
    PLACE_AFTER_STOP = "GB_PLACE_AFTER_STOP", Experiment, Set, "off",
        "An open end that was stopped still gets the placement rounds.";
    NEG_ONLY = "GB_NEG_ONLY", Experiment, Set, "off",
        "With `--negotiate`: only the negotiated search, without the plain one to compare with.";
    NEG_GROWTH = "GB_NEG_GROWTH", Experiment, Num, "1.5",
        "Negotiated routing: factor by which the price of a conflict grows per round.";
    NEG_HISTORY = "GB_NEG_HISTORY", Experiment, Num, "1.0",
        "Negotiated routing: weight of the history of conflicts on an edge.";
    NEG_OVERFLOW = "GB_NEG_OVERFLOW", Experiment, Num, "2.0",
        "Negotiated routing: starting price of an overfilled channel.";
    NEG_CROSSING = "GB_NEG_CROSSING", Experiment, Num, "1.5",
        "Negotiated routing: starting price of a crossing.";
    NEG_TIGHT = "GB_NEG_TIGHT", Experiment, Num, "3.0",
        "Negotiated routing: price of a channel that is nearly full.";
    NEG_RESERVE = "GB_NEG_RESERVE", Experiment, Num, "0.05",
        "Negotiated routing: reserve in mm a channel keeps.";
    MAXEXP = "GB_MAXEXP", Experiment, Num, "400000",
        "Most expansions of one search for a connection.";
    RIPUP = "GB_RIPUP", Experiment, Num, "3",
        "Rip-up passes of a router run.";
    ESCAPE = "GB_ESCAPE", Experiment, Num, "0",
        "Extra cost in mm for passing right next to a pad of another net that still waits for its route.";
    SHARE_KIND = "GB_SHARE_KIND", Experiment, Value, "all kinds",
        "`--patterns shares`: only the shares of this kind (`neighbours` or `decap`).";
    SKEW_WEIGHT = "GB_SKEW_WEIGHT", Experiment, Num, "0.5",
        "Weight of a mm of skew need in the cost of a candidate.";
    REFINE = "GB_REFINE", Experiment, Num, "0.6",
        "Triangulation: long obstacle sides get a vertex every this many mm.";
    RELAX_IT = "GB_RELAX_IT", Experiment, Num, "400",
        "Rounds of a full relaxation.";
    SPACING = "GB_SPACING", Experiment, Num, "0.05",
        "Relaxation: distance of the points on a track in mm.";
    COARSE = "GB_COARSE", Experiment, Value, "twice the spacing, at least 0.1",
        "Relaxation: distance of the points in mm while the bands still move a lot.";
    SHIFT = "GB_SHIFT", Experiment, Value, "unset",
        "CLI `route`, debugging: `R8:0.3,0;C4:0,-0.2` routes the board with these parts shifted by mm.";
    CC_GAP = "GB_CC_GAP", Experiment, Value, "the gap of `place`",
        "CLI `route --clear-corridors`: gap between the parts placed again, in mm.";
    CC_HALO = "GB_CC_HALO", Experiment, Value, "the halo of `place`",
        "CLI `route --clear-corridors`: halo around the parts placed again, in mm.";
    CC_ESCAPE = "GB_CC_ESCAPE", Experiment, Value, "off",
        "CLI `route --clear-corridors`: a number above 0 turns the escape halo on.";
    CC_SEMANTIC = "GB_CC_SEMANTIC", Experiment, Value, "off",
        "CLI `route --clear-corridors`: a number above 0 turns the semantic placement on.";
    CC_LEASH = "GB_CC_LEASH", Experiment, Value, "1.5",
        "CLI `route --clear-corridors`: how far ICs and connectors in a band may move, in mm.";
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flags_keep_their_rules() {
        let on = |read, raw: Option<&str>| flag(read, raw);
        // (rule, unset, "", "0", "1", "x")
        for (read, want) in [
            (Read::Set, [false, true, true, true, true]),
            (Read::NotZero, [true, true, false, true, true]),
        ] {
            let got = [None, Some(""), Some("0"), Some("1"), Some("x")].map(|raw| on(read, raw));
            assert_eq!(got, want, "{read:?}");
        }
    }

    #[test]
    fn the_table_is_sound() {
        let mut names: Vec<&str> = TABLE.iter().map(|k| k.name).collect();
        names.sort_unstable();
        let n = names.len();
        names.dedup();
        assert_eq!(names.len(), n, "a variable is in the table twice");
        for k in TABLE {
            assert!(k.name.starts_with("GB_"), "{}", k.name);
            assert!(!k.doc.is_empty() && !k.doc.contains('|'), "{}", k.name);
            if k.read == Read::Num {
                assert!(k.default.parse::<f64>().is_ok(), "{}: default {}", k.name, k.default);
            }
        }
    }
}
