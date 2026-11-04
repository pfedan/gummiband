//! Gummiband core: geometry, board model and the topological router.

pub mod board;
pub mod check;
pub mod follow;
pub mod geom;
pub mod import;
mod improve;
pub mod index;
pub mod matching;
pub mod octo;
pub mod pairs;
pub mod patterns;
pub mod prof;
pub mod realize;
pub mod relax;
pub mod route;
pub mod topo;
pub mod tune;

use std::collections::HashMap;

use rayon::prelude::*;

use board::{Board, NetId};
use realize::Geometry;
use route::{Router, RouterConfig};
use topo::{Substrate, SubstrateConfig};

/// Default portfolio size. Fixed and not the core count: the same input gives the same result
/// on every machine.
pub const DEFAULT_RUNS: usize = 14;

#[derive(Debug, Clone, Default)]
pub struct RunStats {
    pub seed: u64,
    pub fanout_first: bool,
    pub via_cost: f64,
    /// Weight of the layer directions in this run (see [`RouterConfig`]).
    pub layer_dirs: f64,
    /// Via pass of this run (see [`RouterConfig`]).
    pub via_pass: f64,
    /// Rounds of negotiation of this run (see [`RouterConfig`], 0 = plain).
    pub negotiate: usize,
    /// Generation of the evolutionary search (0 = initial portfolio).
    pub generation: usize,
    pub conns: usize,
    pub routed: usize,
    pub vias: usize,
    pub length: f64,
    pub unrouted: Vec<String>,
    /// For each unrouted connection the nets that run in its way.
    pub blocked: Vec<(String, Vec<String>)>,
    /// Clearance violations after the geometry step (None if not relaxed).
    pub violations: Option<usize>,
    /// Of those, the ones where copper of two nets touches or crosses (shorts).
    pub shorts: Option<usize>,
    /// mm the nets of the skew rules lack to their longest net, beyond what the rule allows
    /// (see [`skew_need`]).
    pub skew_need: f64,
}

/// Weight of a via in [`RunStats::cost`].
pub(crate) const VIA_WEIGHT: f64 = 2.0;
/// Weight of a mm of track in [`RunStats::cost`]: a via weighs 40 mm there.
pub(crate) const LENGTH_WEIGHT: f64 = 0.05;

impl RunStats {
    /// Lower is better: completion first, then vias, then length. One unrouted connection
    /// weighs as much as ten clearance violations (see the geometry stage).
    pub fn cost(&self) -> f64 {
        (self.conns - self.routed) as f64 * 1e4
            + self.vias as f64 * VIA_WEIGHT
            + self.length * LENGTH_WEIGHT
            + self.skew_need * skew_weight()
    }
}

/// Weight of a mm of [`RunStats::skew_need`]: a group of nets with a skew rule whose routes
/// differ a lot needs meanders the board may have no room for, so the search prefers routes of
/// similar length. 0.5 measured best on the DDR5 testbed (0.2 too weak, 1.0 the same as 0.5);
/// `GB_SKEW_WEIGHT` sets another one.
fn skew_weight() -> f64 {
    static W: std::sync::OnceLock<f64> = std::sync::OnceLock::new();
    *W.get_or_init(|| {
        std::env::var("GB_SKEW_WEIGHT")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(0.5)
    })
}

/// What length tuning would have to add: per skew rule (not within pairs), for each net with
/// tracks, how much shorter than the longest net of the rule it is beyond the allowed skew. A
/// single long way round raises this for the whole group. Track lengths only, no vias.
pub fn skew_need(board: &Board, geo: &Geometry) -> f64 {
    if board.match_rules.is_empty() {
        return 0.0;
    }
    let mut len: HashMap<NetId, f64> = HashMap::new();
    for t in &geo.tracks {
        *len.entry(t.net).or_insert(0.0) += t.pts.windows(2).map(|w| w[0].dist(w[1])).sum::<f64>();
    }
    let mut need = 0.0;
    for r in &board.match_rules {
        if r.kind != board::MatchKind::Skew || r.within_pairs {
            continue;
        }
        let tol = r.value.max.unwrap_or(0.0);
        let ls: Vec<f64> = r.nets.iter().filter_map(|n| len.get(n).copied()).collect();
        let top = ls.iter().copied().fold(0.0, f64::max);
        need += ls.iter().map(|&l| (top - l - tol).max(0.0)).sum::<f64>();
    }
    need
}

/// Share of the differential pair track length that runs next to its partner on the same
/// layer, at most `near` mm away: (coupled mm, total mm).
pub fn pair_coupling(board: &Board, geo: &Geometry, near: f64) -> (f64, f64) {
    board.diff_pairs().into_iter().fold((0.0, 0.0), |(c, t), (a, b)| {
        let (x, y) = pair_coupling_of(geo, a, b, near);
        (c + x, t + y)
    })
}

/// [`pair_coupling`] of one pair `a`, `b`: (coupled mm, total mm of both nets).
pub fn pair_coupling_of(geo: &Geometry, a: NetId, b: NetId, near: f64) -> (f64, f64) {
    let (mut coupled, mut total) = (0.0, 0.0);
    for (x, y) in [(a, b), (b, a)] {
        for t in geo.tracks.iter().filter(|t| t.net == x) {
            for w in t.pts.windows(2) {
                let (l, mid) = (w[0].dist(w[1]), (w[0] + w[1]) * 0.5);
                total += l;
                let close = geo
                    .tracks
                    .iter()
                    .filter(|o| o.net == y && o.layer == t.layer)
                    .any(|o| o.pts.windows(2).any(|v| geom::point_seg_dist(mid, v[0], v[1]) < near));
                if close {
                    coupled += l;
                }
            }
        }
    }
    (coupled, total)
}

/// A stretch of a pair net that does not run beside its partner (see [`uncoupled_stretches`]).
#[derive(Debug, Clone, serde::Serialize)]
pub struct Uncoupled {
    pub net: NetId,
    pub layer: board::LayerId,
    /// Middle of the stretch and its length in mm.
    pub at: geom::Point,
    pub length: f64,
    /// Why, as far as the geometry tells: "partner on another layer", "partner farther away"
    /// (a breakout to the pins, or a way of its own), "something between" (copper, a pad or a
    /// via within [`pairs::ZIP_REACH`] that keeps the two apart).
    pub why: &'static str,
}

/// The stretches of the tracks of pair `a`, `b` that are not within `near` of the partner's
/// track on the same layer, longest first; pieces shorter than `min` are left out.
pub fn uncoupled_stretches(geo: &Geometry, a: NetId, b: NetId, near: f64, min: f64) -> Vec<Uncoupled> {
    const STEP: f64 = 0.1;
    let dist = |q: geom::Point, net: NetId, layer: Option<board::LayerId>| {
        geo.tracks
            .iter()
            .filter(|o| o.net == net && layer.is_none_or(|l| o.layer == l))
            .flat_map(|o| o.pts.windows(2).map(move |v| geom::point_seg_dist(q, v[0], v[1])))
            .fold(f64::INFINITY, f64::min)
    };
    let mut out = Vec::new();
    for (x, y) in [(a, b), (b, a)] {
        for t in geo.tracks.iter().filter(|t| t.net == x) {
            // (start, length, points) of the open stretch
            let mut run: Option<(f64, Vec<geom::Point>)> = None;
            let mut close_run = |run: &mut Option<(f64, Vec<geom::Point>)>| {
                if let Some((len, pts)) = run.take()
                    && len >= min
                {
                    let at = pts[pts.len() / 2];
                    let same = dist(at, y, Some(t.layer));
                    let why = if same <= pairs::ZIP_REACH {
                        "something between"
                    } else if dist(at, y, None) <= pairs::ZIP_REACH {
                        "partner on another layer"
                    } else {
                        "partner farther away"
                    };
                    out.push(Uncoupled {
                        net: x,
                        layer: t.layer,
                        at,
                        length: len,
                        why,
                    });
                }
            };
            for w in t.pts.windows(2) {
                let l = w[0].dist(w[1]);
                let k = (l / STEP).ceil().max(1.0) as usize;
                for i in 0..k {
                    let q = w[0].lerp(w[1], (i as f64 + 0.5) / k as f64);
                    if dist(q, y, Some(t.layer)) < near {
                        close_run(&mut run);
                    } else {
                        let r = run.get_or_insert((0.0, Vec::new()));
                        r.0 += l / k as f64;
                        r.1.push(q);
                    }
                }
            }
            close_run(&mut run);
        }
    }
    out.sort_by(|p, q| q.length.total_cmp(&p.length));
    out
}

/// Lower bound of the total track length: per net without plane the minimum spanning tree of
/// its pins times sqrt(3)/2 (no Steiner tree is shorter than that). Plane nets count zero.
pub fn length_lower_bound(board: &Board) -> f64 {
    let mut total = 0.0;
    for net in board.routable_nets() {
        let n = &board.nets[net as usize];
        if board.planes.iter().any(|p| p.net == net) {
            continue;
        }
        let pts: Vec<geom::Point> = n.pins.iter().map(|&p| board.pins[p].pos).collect();
        let mut in_tree = vec![false; pts.len()];
        let mut dist = vec![f64::INFINITY; pts.len()];
        dist[0] = 0.0;
        for _ in 0..pts.len() {
            let k = (0..pts.len())
                .filter(|&k| !in_tree[k])
                .min_by(|&a, &b| dist[a].total_cmp(&dist[b]))
                .unwrap();
            in_tree[k] = true;
            total += dist[k];
            for j in 0..pts.len() {
                if !in_tree[j] {
                    dist[j] = dist[j].min(pts[j].dist(pts[k]));
                }
            }
        }
    }
    total * 3f64.sqrt() / 2.0
}

pub struct RouteResult {
    pub geometry: Geometry,
    pub stats: RunStats,
    pub portfolio: Vec<RunStats>,
    /// The best candidate after every generation (label, geometry, relaxed), if requested.
    pub frames: Vec<(String, Geometry, bool)>,
    /// Footprints the router shifted (reference, offset in mm); the geometry belongs to the
    /// shifted board, see [`Board::with_offsets`].
    pub offsets: HashMap<String, geom::Point>,
    /// Results of the open end that are as complete and as clean as this one but differ in vias
    /// and length, for the user to choose.
    pub alternatives: Vec<(RunStats, Geometry)>,
}

/// A progress report of the search: the best candidate after a phase.
#[derive(Debug, Clone, serde::Serialize)]
pub struct Progress {
    /// "portfolio", "generation 2", "geometry", "geometry generation 1", "polish",
    /// "placement round 1"
    pub phase: String,
    pub routed: usize,
    pub conns: usize,
    pub vias: usize,
    pub length: f64,
    /// Clearance violations (None before the geometry step).
    pub violations: Option<usize>,
    /// Where a search with generations stands (see [`Search`]).
    #[serde(flatten)]
    pub search: Search,
}

/// The state of the geometry generations and the open end.
#[derive(Debug, Clone, Copy, Default, serde::Serialize)]
pub struct Search {
    /// Geometry generation (0 before them).
    pub generation: usize,
    /// Generations of the open end since the result last got noticeably better.
    pub stale: usize,
    /// Side branches of the open end that failed since then; each one is bolder.
    pub level: usize,
    /// As many as have to fail for the search to count as converged.
    pub top: usize,
    /// Even bold changes do not help any more: taking the result now loses little.
    pub converged: bool,
}

/// Receives progress reports (from the search thread).
#[derive(Clone)]
pub struct ProgressFn(pub std::sync::Arc<dyn Fn(&Progress) + Send + Sync>);

impl std::fmt::Debug for ProgressFn {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("ProgressFn")
    }
}

#[derive(Debug, Clone)]
pub struct SearchConfig {
    /// Parallel runs per generation.
    pub population: usize,
    /// Generations of the evolutionary search after the initial portfolio (0 = portfolio only).
    pub generations: usize,
    /// Generations that also run the geometry step and select by clearance violations.
    pub geometry_generations: usize,
    /// Keep the best candidate of every generation (for an animation).
    pub frames: bool,
    /// Called with the best candidate after every phase.
    pub progress: Option<ProgressFn>,
    /// Seed of the evolutionary search (default: GB_EVO_SEED, then a constant). The same seed
    /// gives the same result.
    pub seed: Option<u64>,
    /// After the search keep improving the result until this flag is set (`improve.rs`).
    pub open_end: Option<std::sync::Arc<std::sync::atomic::AtomicBool>>,
    /// Reserve in mm on top of the clearances that the finished copper gets where there is room
    /// (0 = off, the copper stays at the limit).
    pub air: f64,
    /// Tracks of the default net class push their neighbours aside and grow up to this width in
    /// mm where there is room (0 = off). A class with a target width of its own (`opt` of a
    /// track width rule) grows to that one instead, also with this off.
    pub prefer_width: f64,
    /// What a via is worth in mm of track when the open end judges its results
    /// ([`DEFAULT_OPEN_VIA_COST`]). Lower gives more vias and shorter tracks.
    pub open_via_cost: f64,
}

/// The via price of the open end in mm of track: half of what a via weighs in the search
/// before, since connections that may end on a track of their net would pay for fewer vias
/// with long ways.
pub const DEFAULT_OPEN_VIA_COST: f64 = 20.0;

impl SearchConfig {
    fn stopped(&self) -> bool {
        self.open_end
            .as_ref()
            .is_some_and(|f| f.load(std::sync::atomic::Ordering::Relaxed))
    }

    fn report(&self, phase: &str, s: &RunStats) {
        self.report_at(phase, s, Search::default());
    }

    fn report_at(&self, phase: &str, s: &RunStats, search: Search) {
        if let Some(p) = &self.progress {
            (p.0)(&Progress {
                phase: phase.to_string(),
                routed: s.routed,
                conns: s.conns,
                vias: s.vias,
                length: s.length,
                violations: s.violations,
                search,
            });
        }
    }
}

/// Routing order genome: a key per connection, identified by (net, from pin). Lower routes first.
type Genome = HashMap<(NetId, usize), f64>;

#[derive(Clone)]
struct Candidate {
    stats: RunStats,
    geo: Geometry,
    /// Final connection order and the unrouted connections, for the next genome.
    order: Vec<(NetId, usize)>,
    failed: Vec<(NetId, usize)>,
    /// Nets with clearance violations after the geometry step.
    bad_nets: Vec<NetId>,
    /// Where the geometry step left clearance violations.
    hot: Vec<geom::Point>,
    /// The open end's measure of them, see `improve::judge`.
    flaw: f64,
    /// The configuration of this run.
    cfg: RouterConfig,
}

fn evaluate(
    board: &Board,
    sub: &Substrate,
    cfg: RouterConfig,
    genome: Option<&Genome>,
    generation: usize,
) -> Candidate {
    prof::time(prof::Slot::Evaluate, || {
        // a run that trips over an inconsistency of its own topology is lost, not the search
        let run = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            evaluate_inner(board, sub, cfg.clone(), genome, generation)
        }));
        run.unwrap_or_else(|e| {
            let what = e
                .downcast_ref::<&str>()
                .map(|s| s.to_string())
                .or_else(|| e.downcast_ref::<String>().cloned())
                .unwrap_or_default();
            eprintln!("run of seed {} in generation {generation} lost: {what}", cfg.seed);
            lost_run(board, sub, cfg, generation)
        })
    })
}

/// A run that did not finish counts as one that routed nothing.
fn lost_run(board: &Board, sub: &Substrate, cfg: RouterConfig, generation: usize) -> Candidate {
    let mut r = Router::new(board, sub, cfg.clone());
    r.plan();
    let order: Vec<(NetId, usize)> = r.conns.iter().map(|c| (c.net, c.from)).collect();
    let stats = RunStats {
        seed: cfg.seed,
        fanout_first: cfg.fanout_first,
        via_cost: cfg.via_cost,
        layer_dirs: cfg.layer_dirs,
        via_pass: cfg.via_pass,
        negotiate: cfg.negotiate,
        generation,
        conns: order.len(),
        routed: 0,
        vias: 0,
        length: 0.0,
        unrouted: order
            .iter()
            .map(|&(n, p)| format!("{} from {}", board.nets[n as usize].name, board.pins[p].name))
            .collect(),
        blocked: Vec::new(),
        violations: None,
        shorts: None,
        skew_need: 0.0,
    };
    Candidate {
        stats,
        geo: Geometry::default(),
        failed: order.clone(),
        order,
        bad_nets: Vec::new(),
        hot: Vec::new(),
        flaw: 0.0,
        cfg,
    }
}

fn evaluate_inner(
    board: &Board,
    sub: &Substrate,
    cfg: RouterConfig,
    genome: Option<&Genome>,
    generation: usize,
) -> Candidate {
    let cfg_used = cfg.clone();
    let mut r = Router::new(board, sub, cfg.clone());
    r.plan();
    if let Some(g) = genome {
        r.reorder(|c| g.get(&(c.net, c.from)).copied().unwrap_or(f64::MAX));
    }
    let routed = r.run();
    debug_hooks(board, &mut r);
    let geo = r.geometry();
    candidate(board, &r, geo, routed, cfg_used, generation)
}

/// What a finished router run is worth, with `geo` as its geometry.
fn candidate(
    board: &Board,
    r: &Router,
    geo: Geometry,
    routed: usize,
    cfg: RouterConfig,
    generation: usize,
) -> Candidate {
    let length = geo
        .tracks
        .iter()
        .map(|t| t.pts.windows(2).map(|w| w[0].dist(w[1])).sum::<f64>())
        .sum();
    let unrouted_ids = r.unrouted();
    let unrouted = unrouted_ids
        .iter()
        .map(|&ci| {
            let c = &r.conns[ci];
            format!("{} from {}", board.nets[c.net as usize].name, board.pins[c.from].name)
        })
        .collect();
    // what is in the way of each unrouted connection, for the report
    let blocked = unrouted_ids
        .iter()
        .map(|&ci| {
            let c = &r.conns[ci];
            let mut nets: Vec<&str> = r
                .blockers(ci)
                .into_iter()
                .map(|v| board.nets[r.conns[v].net as usize].name.as_str())
                .collect();
            nets.sort_unstable();
            nets.dedup();
            let name = format!("{} from {}", board.nets[c.net as usize].name, board.pins[c.from].name);
            (name, nets.into_iter().map(str::to_string).collect())
        })
        .collect();
    let stats = RunStats {
        blocked,
        seed: cfg.seed,
        fanout_first: cfg.fanout_first,
        via_cost: cfg.via_cost,
        layer_dirs: cfg.layer_dirs,
        via_pass: cfg.via_pass,
        negotiate: cfg.negotiate,
        generation,
        conns: r.conns.len(),
        routed,
        vias: geo.vias.iter().filter(|v| v.drill > 0.0).count(),
        length,
        unrouted,
        violations: None,
        shorts: None,
        skew_need: skew_need(board, &geo),
    };
    Candidate {
        stats,
        geo,
        order: r.conns.iter().map(|c| (c.net, c.from)).collect(),
        failed: unrouted_ids
            .iter()
            .map(|&ci| (r.conns[ci].net, r.conns[ci].from))
            .collect(),
        bad_nets: Vec::new(),
        hot: Vec::new(),
        flaw: 0.0,
        cfg,
    }
}

/// Environment controlled debugging output of a finished router run.
fn debug_hooks(board: &Board, r: &mut Router) {
    if let Ok(pin) = std::env::var("GB_WHY") {
        // trace the search of the first unrouted connection from this pin
        if let Some(ci) = r
            .unrouted()
            .into_iter()
            .find(|&ci| board.pins[r.conns[ci].from].name == pin)
        {
            *r.trace.lock().unwrap() = Some(Vec::new());
            r.route_one(ci);
            let t = r.trace.lock().unwrap().take().unwrap_or_default();
            eprintln!("why {pin}: {} expansions", t.len());
            let _ = std::fs::write(
                "/tmp/gb-why.txt",
                t.iter()
                    .map(|(p, l)| format!("{} {} {}", p.x, p.y, l))
                    .collect::<Vec<_>>()
                    .join("\n"),
            );
        }
    }
    if std::env::var("GB_VALIDATE").is_ok() {
        let v = r.validate();
        eprintln!("validate: {} problems", v.len());
        for l in v.iter().take(15) {
            eprintln!("  {l}");
        }
    }
    if std::env::var("GB_DEBUG_VIAS").is_ok() {
        for l in r.debug_via_pairs() {
            eprintln!("{l}");
        }
        for l in r.debug_vias(12) {
            eprintln!("{l}");
        }
    }
    if std::env::var("GB_DEBUG_CORRIDOR").is_ok() {
        for l in r.debug_corridors(15) {
            eprintln!("{l}");
        }
    }
    if std::env::var("GB_DEBUG_X").is_ok() {
        for l in r.debug_intersections(12) {
            eprintln!("{l}");
        }
    }
}

/// Small deterministic random source (splitmix64).
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// A mutant of the best candidate's order: failed connections move forward by a random
/// amount, a few random connections are shuffled locally.
fn mutate(best: &Candidate, rng: &mut Rng) -> Genome {
    let n = best.order.len().max(1) as f64;
    let mut g: Genome = best.order.iter().enumerate().map(|(i, &k)| (k, i as f64)).collect();
    for k in &best.failed {
        if let Some(v) = g.get_mut(k) {
            *v -= n * (0.2 + 0.8 * rng.unit());
        }
    }
    let shuffles = 1 + (rng.next() % (n as u64 / 20 + 2)) as usize;
    let keys: Vec<(NetId, usize)> = best.order.clone();
    if keys.is_empty() {
        return g;
    }
    for _ in 0..shuffles {
        let k = keys[(rng.next() % keys.len() as u64) as usize];
        if let Some(v) = g.get_mut(&k) {
            *v += (rng.unit() - 0.5) * n * 0.3;
        }
    }
    g
}

/// Give way radius of [`finish_nets`] in mm when nothing else is asked for: the same as the
/// plugin's *Finish selected nets*.
pub const DEFAULT_GIVE: f64 = 8.0;

/// One try of [`finish_nets`].
#[derive(Debug, Clone, PartialEq)]
pub struct GiveTry {
    /// Radius in mm around the pins of the open connections (0: the first try, nothing gave way).
    pub radius: f64,
    /// Nets that gave way.
    pub given: usize,
    /// Connections left open, and clearance violations.
    pub open: usize,
    pub violations: usize,
}

/// The try that [`finish_nets`] keeps.
pub struct Finished<T> {
    /// The board of the result: the nets asked for and those that gave way have connections,
    /// the others are copper.
    pub board: Board,
    pub sub: Substrate,
    pub res: RouteResult,
    /// What the route of that try returned besides.
    pub extra: T,
    /// The nets whose unlocked copper gave way: the result replaces it.
    pub given: std::collections::BTreeSet<String>,
    /// Every try in order, and the one kept.
    pub tries: Vec<GiveTry>,
    pub kept: usize,
}

/// Route only `nets` on a board with kept copper (all nets if it is empty); the rest stays. With
/// `shove` the unlocked tracks of the other nets are part of the topology and shift aside where a
/// new connection needs the room ([`Board::soften`]). If a connection of `nets` still finds no
/// way, the unlocked tracks of other nets near its pins give way and are routed again, within 2,
/// 4, 8 ... mm up to `give` (0: nothing gives way). The try with the fewest open connections,
/// then the fewest clearance violations is kept; on a tie the earlier one, it moves less. So
/// giving way can only help: a try that breaks a net that gave way loses against the one before.
///
/// `route` runs a try with the search it is given. With an open end in `search` and something
/// that may give way, the tries run without it and the one kept runs again open ended: the same
/// search first, so the open end goes on from where that try ended. A stop before that keeps the
/// try, and so does an open end that ends worse (a stop in its search part). With nothing to give
/// way the only try runs open ended right away.
pub fn finish_nets<T>(
    board: &Board,
    nets: &[String],
    shove: bool,
    give: f64,
    search: &SearchConfig,
    route: impl Fn(&Board, &SearchConfig) -> (Substrate, RouteResult, T),
) -> Finished<T> {
    let mut given = std::collections::BTreeSet::new();
    let mut radius = 0.0;
    let mut tries: Vec<GiveTry> = Vec::new();
    let mut kept: Option<Finished<T>> = None;
    let open_later = search.open_end.is_some() && give > 0.0;
    let without = SearchConfig {
        open_end: None,
        ..search.clone()
    };
    let try_search = if open_later { &without } else { search };
    let measure = |b: &Board, res: &RouteResult| (res.stats.unrouted.len(), check::check(b, &res.geometry).len());
    loop {
        let mut b = board.only_nets(nets, &given);
        if shove {
            b.soften(nets);
        }
        let (sub, res, extra) = route(&b, try_search);
        let (open, violations) = measure(&b, &res);
        let t = GiveTry {
            radius,
            given: given.len(),
            open,
            violations,
        };
        // around what was asked for; a net that gave way must not widen the circle
        let asked: Vec<String> = res
            .stats
            .unrouted
            .iter()
            .filter(|u| nets.iter().any(|n| u.starts_with(&format!("{n} from "))))
            .cloned()
            .collect();
        let better = kept.as_ref().is_none_or(|k| {
            let best = &tries[k.kept];
            (t.open, t.violations) < (best.open, best.violations)
        });
        tries.push(t);
        if better {
            kept = Some(Finished {
                board: b,
                sub,
                res,
                extra,
                given: given.clone(),
                tries: Vec::new(),
                kept: tries.len() - 1,
            });
        }
        if asked.is_empty() || give <= 0.0 {
            break;
        }
        // the circle grows until it takes in nets that can give way: a net without two pins
        // could not be routed again
        let before = given.len();
        while given.len() == before && radius < give {
            radius = (radius * 2.0).max(2.0).min(give);
            given.extend(board.nets_near_pins(&asked, radius).into_iter().filter(|n| {
                !nets.contains(n)
                    && board
                        .net_id(n)
                        .is_some_and(|id| board.nets[id as usize].pins.len() >= 2)
            }));
        }
        if given.len() == before {
            break;
        }
    }
    let mut k = kept.expect("one try at least");
    if open_later && !search.stopped() {
        let (sub, res, extra) = route(&k.board, search);
        let best = &tries[k.kept];
        if measure(&k.board, &res) <= (best.open, best.violations) {
            (k.sub, k.res, k.extra) = (sub, res, extra);
        }
    }
    k.tries = tries;
    k
}

/// Route a board (see [`route_fixed`]). With movable parts and clearance problems left, parts
/// next to the problems are shifted and the whole search runs again on the shifted board, for
/// a few placement rounds; the best result over all rounds wins. At the end the layer changes
/// of current paths get the vias they need ([`relax::stitch_vias`]; `GB_STITCH=0`: off).
pub fn route_board(
    board: &Board,
    sub_cfg: &SubstrateConfig,
    base: &RouterConfig,
    search: &SearchConfig,
) -> (Substrate, RouteResult) {
    let (sub, mut res) = place_and_route(board, sub_cfg, base, search);
    if std::env::var("GB_STITCH").map_or(true, |v| v != "0") {
        let moved;
        let b = if res.offsets.is_empty() {
            board
        } else {
            moved = board.with_offsets(&res.offsets);
            &moved
        };
        // nothing moves any more; what makes a new clearance problem is not kept
        let mut geo = res.geometry.clone();
        let added = relax::stitch_vias(b, &mut geo);
        if added > 0 {
            let clearances = |g: &Geometry| check::check(b, g).iter().filter(|v| !v.is_width()).count();
            if clearances(&geo) <= clearances(&res.geometry) {
                eprintln!("current paths: {added} vias added next to their layer changes");
                res.geometry = geo;
                res.stats.vias += added;
            }
        }
    }
    (sub, res)
}

/// [`route_board`] up to the vias of the current paths.
fn place_and_route(
    board: &Board,
    sub_cfg: &SubstrateConfig,
    base: &RouterConfig,
    search: &SearchConfig,
) -> (Substrate, RouteResult) {
    let score = |b: &Board, r: &RouteResult| -> (usize, f64, Vec<geom::Point>) {
        let v = check::check(b, &r.geometry);
        let shorts = v.iter().filter(|x| x.actual <= 0.0).count();
        let cost = r.stats.cost() + v.len() as f64 * 1000.0 + shorts as f64 * 5000.0;
        // places that ask for room: clearance problems, and the pins of open connections,
        // where the parts around may step aside to let a track out
        let mut hot: Vec<geom::Point> = v.iter().map(|x| x.at).collect();
        hot.extend(open_pins(b, &r.stats.unrouted));
        (r.stats.routed, cost, hot)
    };
    let (sub, res) = route_search(board, sub_cfg, base, search);
    if board.movable.is_empty() || search.geometry_generations == 0 {
        return (sub, res);
    }
    let rounds = std::env::var("GB_PLACE_ROUNDS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(2);
    let mut best_score = score(board, &res);
    let mut best = (sub, res);
    let mut offsets: HashMap<String, geom::Point> = HashMap::new();
    let mut hot = best_score.2.clone();
    let mut rng = Rng(0x91ace);
    for round in 1..=rounds {
        // GB_PLACE_AFTER_STOP: an open end that was stopped still gets the fast rounds below
        let after_stop = search.open_end.is_some() && std::env::var("GB_PLACE_AFTER_STOP").is_ok();
        if (hot.is_empty() && best.1.stats.unrouted.is_empty()) || (search.stopped() && !after_stop) {
            break;
        }
        let cur = board.with_offsets(&offsets);
        let Some(next) = shift_parts(board, &cur, &offsets, &hot, &mut rng) else {
            break;
        };
        // fast path, the rubber band way: keep the topology of the best routing and move the
        // parts in small steps; after each the pads take their track ends along, and the tracks
        // they press against give way within their clearances. The parts stop where the copper
        // cannot give way any more, so the shift may end short of `next`.
        let quick = {
            // the routed copper stays awake: the relaxation of each step may also settle the
            // spots the shift is meant to free
            let f = follow::follow_steps(
                board,
                &offsets,
                &next,
                &best.1.geometry,
                sub_cfg,
                &relax::RelaxConfig::default(),
            );
            let geometry = f.geo;
            eprintln!(
                "placement round {round}: followed {} of {} steps{}",
                f.stats.steps,
                f.stats.steps_planned,
                f.stats
                    .stopped
                    .as_ref()
                    .map_or(String::new(), |w| format!(", stopped: {w}"))
            );
            let moved = board.with_offsets(&f.offsets);
            let sub = Substrate::build(&moved, sub_cfg);
            let res = RouteResult {
                geometry,
                stats: best.1.stats.clone(),
                portfolio: Vec::new(),
                frames: Vec::new(),
                offsets: f.offsets.clone(),
                alternatives: Vec::new(),
            };
            (sub, res, moved, f.offsets)
        };
        let (quick, moved_quick, reached) = ((quick.0, quick.1), quick.2, quick.3);
        let qs = score(&moved_quick, &quick.1);
        eprintln!(
            "placement round {round}: {} parts shifted, kept topology: cost {:.0} (best {:.0})",
            next.len(),
            qs.1,
            best_score.1
        );
        if qs.0 == best_score.0 && qs.1 < best_score.1 {
            hot = qs.2.clone();
            best_score = qs;
            best = quick;
            offsets = reached;
            let mut st = best.1.stats.clone();
            st.violations = Some(best_score.2.len());
            search.report(&format!("placement round {round}"), &st);
            continue;
        }
        // a full new search pays off only for a handful of local problems, and only on boards
        // where a search is not expensive; otherwise the fast path above is all there is.
        // By board size and not by the clock: the result must not depend on the machine.
        if hot.len() > 20 || best.1.stats.conns > 400 || search.stopped() {
            continue;
        }
        let moved = board.with_offsets(&next);
        // where the search took up a negotiation on its own, the rounds negotiate right away
        let (sub, mut res) = if base.negotiate == 0 && best.1.stats.negotiate > 0 {
            let cfg = RouterConfig {
                negotiate: best.1.stats.negotiate,
                ..base.clone()
            };
            search_fixed(&moved, sub_cfg, &cfg, search)
        } else {
            route_search(&moved, sub_cfg, base, search)
        };
        res.offsets = next.clone();
        let sc = score(&moved, &res);
        eprintln!(
            "placement round {round}: {} parts shifted, {}/{} routed, cost {:.0} (best {:.0})",
            next.len(),
            res.stats.routed,
            res.stats.conns,
            sc.1,
            best_score.1
        );
        // the next round starts from the best result, with its placement: the fast path moves
        // the copper of that result and needs the offsets it was routed with
        if sc.0 > best_score.0 || (sc.0 == best_score.0 && sc.1 < best_score.1) {
            hot = sc.2.clone();
            offsets = next;
            best_score = sc;
            best = (sub, res);
        }
        let mut st = best.1.stats.clone();
        st.violations = Some(best_score.2.len());
        search.report(&format!("placement round {round}"), &st);
    }
    best
}

/// Cost of a relaxed candidate. A short is much worse than a clearance a little too small: it
/// weighs six violations.
fn full_cost(c: &Candidate) -> f64 {
    c.stats.cost() + c.stats.violations.unwrap_or(0) as f64 * 1000.0 + c.stats.shorts.unwrap_or(0) as f64 * 5000.0
}

/// One full search on a fixed placement ([`route_fixed`]). A negotiation is a second search next
/// to the plain one and not a replacement: it completes the dense boards, but on a loose board
/// the routing order alone may leave the geometry more room. The better result counts,
/// completion first.
fn route_search(
    board: &Board,
    sub_cfg: &SubstrateConfig,
    base: &RouterConfig,
    search: &SearchConfig,
) -> (Substrate, RouteResult) {
    if base.negotiate == 0 {
        return route_fixed(board, sub_cfg, base, search);
    }
    let negotiated = search_fixed(board, sub_cfg, base, search);
    if std::env::var("GB_NEG_ONLY").is_ok() || search.stopped() {
        return negotiated;
    }
    let plain = RouterConfig {
        negotiate: 0,
        ..base.clone()
    };
    let plain = search_fixed(board, sub_cfg, &plain, search);
    better_search(board, negotiated, plain)
}

/// The better of a negotiated and a plain search on one placement: completion first, then the
/// cost with the clearance violations of the finished copper; on a tie the plain one.
fn better_search(
    board: &Board,
    negotiated: (Substrate, RouteResult),
    plain: (Substrate, RouteResult),
) -> (Substrate, RouteResult) {
    let key = |r: &RouteResult| {
        let v = check::check(board, &r.geometry);
        let shorts = v.iter().filter(|x| x.actual <= 0.0).count();
        (
            r.stats.routed,
            r.stats.cost() + v.len() as f64 * 1000.0 + shorts as f64 * 5000.0,
        )
    };
    let (a, b) = (key(&negotiated.1), key(&plain.1));
    eprintln!(
        "negotiated: {}/{} routed, cost {:.0}; plain: {}/{} routed, cost {:.0}",
        a.0, negotiated.1.stats.conns, a.1, b.0, plain.1.stats.conns, b.1
    );
    if a.0 > b.0 || (a.0 == b.0 && a.1 < b.1) {
        return negotiated;
    }
    let (sub, mut res) = plain;
    res.portfolio.extend(negotiated.1.portfolio);
    (sub, res)
}

/// Rounds of the negotiation that [`route_fixed`] takes up on its own.
pub const AUTO_NEGOTIATE: usize = 20;

/// Whether a search with these results takes up a negotiation on its own: none was asked for,
/// and it leaves connections open, at most 15 % of them. More than that means more connections
/// than room on the layers (ULX3S leaves about a fifth open), which no price settles.
/// `GB_AUTO_NEGOTIATE=0` switches it off.
fn auto_negotiate(base: &RouterConfig, stats: &RunStats) -> bool {
    let open = stats.conns - stats.routed;
    base.negotiate == 0
        && std::env::var("GB_AUTO_NEGOTIATE").map_or(true, |v| v != "0")
        && open > 0
        && open * 20 <= stats.conns * 3
}

/// One full search on a fixed placement, see `search_fixed`. Where it leaves a few connections
/// open and no negotiation was asked for ([`auto_negotiate`]), a second search negotiates with
/// [`AUTO_NEGOTIATE`] rounds and the better result counts, as with `--negotiate`. A search that
/// completes the board, or leaves more open, stays as it is, and so does an open ended one: its
/// open end has worked on what was open (also when `GB_OPEN_GENERATIONS` ends it, not a stop).
/// Nor does a board with soft copper (finishing nets with shove): its router does not negotiate.
pub fn route_fixed(
    board: &Board,
    sub_cfg: &SubstrateConfig,
    base: &RouterConfig,
    search: &SearchConfig,
) -> (Substrate, RouteResult) {
    let plain = search_fixed(board, sub_cfg, base, search);
    if !auto_negotiate(base, &plain.1.stats) || search.open_end.is_some() || !board.soft.chains.is_empty() {
        return plain;
    }
    eprintln!(
        "negotiation: the search leaves {} connections open, a second search negotiates",
        plain.1.stats.conns - plain.1.stats.routed
    );
    let cfg = RouterConfig {
        negotiate: AUTO_NEGOTIATE,
        ..base.clone()
    };
    let negotiated = search_fixed(board, sub_cfg, &cfg, search);
    better_search(board, negotiated, plain)
}

/// One full search on a fixed placement: build the substrate once, run a portfolio of router
/// configurations in parallel, then improve the best routing order in generations of mutants
/// (completion first, then vias, then length). The winner is relaxed into its final geometry.
fn search_fixed(
    board: &Board,
    sub_cfg: &SubstrateConfig,
    base: &RouterConfig,
    search: &SearchConfig,
) -> (Substrate, RouteResult) {
    let mut with_patterns;
    let base = if base.patterns.is_none() && (base.share_fanouts || base.decaps_first || base.row_moves > 0.0) {
        with_patterns = base.clone();
        with_patterns.patterns = Some(std::sync::Arc::new(patterns::find(board)));
        &with_patterns
    } else {
        base
    };
    let sub = Substrate::build(board, sub_cfg);
    // experiment `GB_VIP_EARLY`: a via in a pad at any time, decided before the generations, for
    // the nets that need one; `=empty` asks the empty board (see `Router::pocket_nets`), `=1` the
    // best of the portfolio (below)
    let early_mode = std::env::var("GB_VIP_EARLY").unwrap_or_default();
    let pocket;
    let base = if early_mode == "empty" && base.vip_nets.is_empty() {
        let mut r = Router::new(board, &sub, base.clone());
        r.plan();
        let nets = r.pocket_nets();
        let names: Vec<&str> = nets.iter().map(|&n| board.nets[n as usize].name.as_str()).collect();
        eprintln!("via in pad from the start: {} nets {:?}", nets.len(), names);
        pocket = RouterConfig {
            vip_nets: std::sync::Arc::new(nets),
            ..base.clone()
        };
        &pocket
    } else {
        base
    };
    let pop = search.population.max(1);
    let configs: Vec<RouterConfig> = (0..pop)
        .map(|i| RouterConfig {
            seed: i as u64,
            fanout_first: i % 2 == 0,
            // the via cost is part of the search: spread it around the configured value
            via_cost: base.via_cost * [1.0, 1.0, 0.6, 1.6, 0.8, 1.3][(i / 2) % 6],
            // the via pass comes at the very end, see below
            via_pass: 0.0,
            // and so are the layer directions: every third run uses them, if there is a
            // choice of layers
            layer_dirs: if (i % 3 == 2 || std::env::var("GB_DIRS_ALL").is_ok())
                && board.routing_layers().iter().count() > 1
            {
                base.layer_dirs
            } else {
                0.0
            },
            ..base.clone()
        })
        .collect();
    let first: Vec<Candidate> = configs
        .into_par_iter()
        .map(|cfg| evaluate(board, &sub, cfg, None, 0))
        .collect();
    let mut portfolio: Vec<RunStats> = first.iter().map(|c| c.stats.clone()).collect();
    let mut frames: Vec<(String, Geometry, bool)> = Vec::new();
    let label = |c: &Candidate, what: &str| {
        format!(
            "{what}: {}/{} routed, {} vias, {:.0} mm{}",
            c.stats.routed,
            c.stats.conns,
            c.stats.vias,
            c.stats.length,
            c.stats
                .violations
                .map(|v| format!(", {v} violations"))
                .unwrap_or_default()
        )
    };
    // the best few candidates go through the geometry step at the end
    let keep = std::env::var("GB_FINALISTS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(24);
    let mut finalists: Vec<Candidate> = Vec::new();
    let offer = |finalists: &mut Vec<Candidate>, c: Candidate| {
        finalists.push(c);
        finalists.sort_by(|a, b| a.stats.cost().total_cmp(&b.stats.cost()));
        finalists.truncate(keep);
    };
    let mut first = first;
    first.sort_by(|a, b| a.stats.cost().total_cmp(&b.stats.cost()));
    let rest = first.split_off(1);
    let mut best = first.pop().unwrap();
    for c in rest {
        offer(&mut finalists, c);
    }
    // `GB_VIP_EARLY=1`: the best of the portfolio is routed again in its order once per net it
    // left open, with a via in a pad for that net alone; the nets that then route more get one
    // at any time for the rest of the search, as if `GB_VIP_NETS` named them
    let early;
    let base = if early_mode == "1" && base.vip_nets.is_empty() && !best.failed.is_empty() {
        let mut nets: Vec<NetId> = best.failed.iter().map(|&(n, _)| n).collect();
        nets.sort_unstable();
        nets.dedup();
        let identity: Genome = best.order.iter().enumerate().map(|(i, &k)| (k, i as f64)).collect();
        let runs: Vec<Vec<NetId>> = std::iter::once(Vec::new())
            .chain(nets.iter().map(|&n| vec![n]))
            .collect();
        let routed: Vec<usize> = runs
            .par_iter()
            .map(|v| {
                let cfg = RouterConfig {
                    vip_nets: std::sync::Arc::new(v.clone()),
                    ..best.cfg.clone()
                };
                evaluate(board, &sub, cfg, Some(&identity), 0).stats.routed
            })
            .collect();
        let tried = nets.len();
        let open: Vec<&str> = nets.iter().map(|&n| board.nets[n as usize].name.as_str()).collect();
        eprintln!("via in pad early: the portfolio leaves {open:?} open");
        nets = nets
            .iter()
            .zip(&routed[1..])
            .filter(|&(_, &r)| r > routed[0])
            .map(|(&n, _)| n)
            .collect();
        let names: Vec<&str> = nets.iter().map(|&n| board.nets[n as usize].name.as_str()).collect();
        eprintln!(
            "via in pad early: {} of {tried} nets route more on their own {:?}",
            nets.len(),
            names
        );
        if nets.is_empty() {
            base
        } else {
            early = RouterConfig {
                vip_nets: std::sync::Arc::new(nets),
                ..base.clone()
            };
            let cfg = RouterConfig {
                vip_nets: early.vip_nets.clone(),
                ..best.cfg.clone()
            };
            let c = evaluate(board, &sub, cfg, Some(&identity), 0);
            if c.stats.cost() < best.stats.cost() {
                let old = std::mem::replace(&mut best, c);
                offer(&mut finalists, old);
            } else {
                offer(&mut finalists, c);
            }
            &early
        }
    } else {
        base
    };
    search.report("portfolio", &best.stats);
    if search.frames {
        frames.push((label(&best, "portfolio"), best.geo.clone(), false));
    }
    let mut rng = Rng(search
        .seed
        .or_else(|| std::env::var("GB_EVO_SEED").ok().and_then(|v| v.parse().ok()))
        .unwrap_or(0x5eed));
    for generation in 1..=search.generations {
        if best.failed.is_empty() && generation > 2 {
            // complete: a few generations for vias and length are enough
            break;
        }
        let genomes: Vec<Genome> = (0..pop).map(|_| mutate(&best, &mut rng)).collect();
        let cfg = RouterConfig {
            seed: 0,
            fanout_first: best.stats.fanout_first,
            via_cost: best.stats.via_cost,
            layer_dirs: best.stats.layer_dirs,
            via_pass: best.stats.via_pass,
            ..base.clone()
        };
        let next: Vec<Candidate> = genomes
            .par_iter()
            .map(|g| evaluate(board, &sub, cfg.clone(), Some(g), generation))
            .collect();
        portfolio.extend(next.iter().map(|c| c.stats.clone()));
        for cand in next {
            if cand.stats.cost() < best.stats.cost() {
                let old = std::mem::replace(&mut best, cand);
                offer(&mut finalists, old);
            } else {
                offer(&mut finalists, cand);
            }
        }
        search.report(&format!("generation {generation}"), &best.stats);
        if search.frames {
            frames.push((
                label(&best, &format!("generation {generation}")),
                best.geo.clone(),
                false,
            ));
        }
    }
    offer(&mut finalists, best);
    if std::env::var("GB_NO_RELAX").is_ok() {
        let best = finalists.swap_remove(0);
        return (
            sub,
            RouteResult {
                geometry: best.geo,
                stats: best.stats,
                portfolio,
                frames,
                offsets: HashMap::new(),
                alternatives: Vec::new(),
            },
        );
    }
    // relax the finalists of the best completion and keep the cleanest geometry; then, if
    // asked, run generations that select by the clearance violations after the geometry step
    let full_relax = relax::RelaxConfig::default();
    let geo_eval_with = |mut c: Candidate, cfg: &relax::RelaxConfig| -> Candidate {
        relax::relax(board, &sub, &mut c.geo, cfg);
        let v = prof::time(prof::Slot::Check, || check::check(board, &c.geo));
        let mut nets: Vec<NetId> = v.iter().flat_map(|x| [Some(x.nets.0), x.nets.1]).flatten().collect();
        nets.sort_unstable();
        nets.dedup();
        c.stats.violations = Some(v.len());
        c.stats.shorts = Some(v.iter().filter(|x| x.actual <= 0.0).count());
        c.bad_nets = nets;
        c.hot = v.iter().map(|x| x.at).collect();
        c
    };
    let geo_eval = |c: Candidate| geo_eval_with(c, &full_relax);
    // successive halving: the topology cost predicts the finished geometry only weakly, so
    // many finalists get a short relaxation first and only the best quarter the full one
    let top = finalists[0].stats.routed;
    let full_n = std::env::var("GB_FULL")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(6usize);
    let pool: Vec<Candidate> = finalists.into_iter().filter(|c| c.stats.routed == top).collect();
    let pool = if pool.len() > full_n {
        let screen = relax::RelaxConfig {
            iterations: (full_relax.iterations / 4).max(40),
            ..full_relax.clone()
        };
        let mut screened: Vec<(Candidate, Geometry)> = pool
            .into_par_iter()
            .map(|c| {
                let geo = c.geo.clone();
                (geo_eval_with(c, &screen), geo)
            })
            .collect();
        screened.sort_by(|a, b| full_cost(&a.0).total_cmp(&full_cost(&b.0)));
        screened.truncate(full_n);
        screened
            .into_iter()
            .map(|(mut c, geo)| {
                c.geo = geo;
                c
            })
            .collect()
    } else {
        pool
    };
    let judged: Vec<Candidate> = pool.into_par_iter().map(geo_eval).collect();
    for c in &judged {
        portfolio.push(c.stats.clone());
    }
    let mut best = judged
        .into_iter()
        .min_by(|a, b| full_cost(a).total_cmp(&full_cost(b)))
        .unwrap();
    search.report("geometry", &best.stats);
    if search.frames {
        frames.push((label(&best, "geometry"), best.geo.clone(), true));
    }
    let open = search.open_end.is_some();
    let mut hotspots: Vec<(geom::Point, f64)> = Vec::new();
    let mut done = 0;
    for generation in 1..=search.geometry_generations {
        if search.stopped() || (best.stats.violations == Some(0) && best.failed.is_empty()) {
            break;
        }
        done = generation;
        // hot spots: violation places of the best so far ask for more room; repeated places
        // grow (negotiation in the spirit of PathFinder, driven by the geometry)
        for &p in &best.hot {
            match hotspots.iter_mut().find(|(c, _)| c.dist(p) < 0.6) {
                Some(h) => h.1 = (h.1 + 0.06).min(0.3),
                None => hotspots.push((p, 0.06)),
            }
        }
        let identity: Genome = best.order.iter().enumerate().map(|(i, &k)| (k, i as f64)).collect();
        // half the candidates keep the order and feel the hot spots at different strengths,
        // the other half also mutate the order
        let variants: Vec<(Genome, f64)> = (0..pop)
            .map(|i| {
                if i % 2 == 0 {
                    (identity.clone(), 0.5 + 0.5 * (i / 2) as f64)
                } else {
                    (mutate_geo(&best, &mut rng), 1.0)
                }
            })
            .collect();
        let next: Vec<Candidate> = variants
            .par_iter()
            .map(|(g, scale)| {
                let cfg = RouterConfig {
                    seed: 0,
                    fanout_first: best.stats.fanout_first,
                    via_cost: best.stats.via_cost,
                    layer_dirs: best.stats.layer_dirs,
                    via_pass: 0.0,
                    hotspots: std::sync::Arc::new(hotspots.iter().map(|&(p, x)| (p, x * scale)).collect()),
                    ..base.clone()
                };
                geo_eval(evaluate(board, &sub, cfg, Some(g), search.generations + generation))
            })
            .collect();
        for c in next {
            portfolio.push(c.stats.clone());
            // completion first: a generation never trades a connection for cleaner geometry
            let better = c.stats.routed > best.stats.routed
                || (c.stats.routed == best.stats.routed && full_cost(&c) < full_cost(&best));
            if better {
                best = c;
            }
        }
        search.report_at(
            &format!("geometry generation {generation}"),
            &best.stats,
            Search {
                generation,
                ..Default::default()
            },
        );
        if search.frames {
            frames.push((
                label(&best, &format!("geometry generation {generation}")),
                best.geo.clone(),
                true,
            ));
        }
    }
    // experiment `GB_VIP_RETRY`: the winner again, with a via in a pad at any time for the nets
    // it left open, as if `GB_VIP_NETS` named them; it stays only if it routes more, or as many
    // for less. The nets stay in its configuration, so the open end keeps them and repeats it.
    // `GB_VIP_RETRY=each` first routes the winner once per such net, with a via in a pad for that
    // net alone, and keeps only the nets that then route more: a net that does not need a via in
    // a pad does not get one.
    let retry = std::env::var("GB_VIP_RETRY").unwrap_or_default();
    if (retry == "1" || retry == "each") && !best.failed.is_empty() {
        let mut nets: Vec<NetId> = best.failed.iter().map(|&(n, _)| n).collect();
        nets.sort_unstable();
        nets.dedup();
        let identity: Genome = best.order.iter().enumerate().map(|(i, &k)| (k, i as f64)).collect();
        if retry == "each" {
            let generation = best.stats.generation;
            let plain = evaluate(board, &sub, best.cfg.clone(), Some(&identity), generation)
                .stats
                .routed;
            let alone: Vec<_> = nets
                .par_iter()
                .map(|&n| {
                    let cfg = RouterConfig {
                        vip_nets: std::sync::Arc::new(vec![n]),
                        ..best.cfg.clone()
                    };
                    evaluate(board, &sub, cfg, Some(&identity), generation).stats.routed
                })
                .collect();
            let tried = nets.len();
            nets = nets
                .iter()
                .zip(alone)
                .filter(|&(_, r)| r > plain)
                .map(|(&n, _)| n)
                .collect();
            eprintln!(
                "via in pad retry: {} of {tried} nets route more on their own",
                nets.len()
            );
        }
        if !nets.is_empty() {
            let cfg = RouterConfig {
                vip_nets: std::sync::Arc::new(nets),
                ..best.cfg.clone()
            };
            let c = geo_eval(evaluate(board, &sub, cfg, Some(&identity), best.stats.generation));
            let better = c.stats.routed > best.stats.routed
                || (c.stats.routed == best.stats.routed && full_cost(&c) < full_cost(&best));
            eprintln!(
                "via in pad retry: {} nets, {} instead of {} connections{}",
                c.cfg.vip_nets.len(),
                c.stats.routed,
                best.stats.routed,
                if better { ", taken" } else { "" }
            );
            if better {
                best = c;
                search.report("via in pad retry", &best.stats);
            }
        }
    }
    // polish: a few clearance spots are often left that more rounds without tension resolve.
    // The via pass routes the winner again in its order and configuration, which the polish
    // does not change: both run side by side, and the via pass is judged against the polished
    // winner as before.
    let do_polish = best.stats.violations.unwrap_or(0) > 0 && std::env::var("GB_NO_POLISH").is_err();
    let do_via_pass = base.via_pass > 0.0 && best.stats.vias > 0;
    let (polished, via_passed) = rayon::join(
        || {
            do_polish.then(|| {
                let polish = relax::RelaxConfig {
                    tension: 0.0,
                    ..full_relax.clone()
                };
                geo_eval_with(best.clone(), &polish)
            })
        },
        || {
            do_via_pass.then(|| {
                let identity: Genome = best.order.iter().enumerate().map(|(i, &k)| (k, i as f64)).collect();
                let cfg = RouterConfig {
                    via_pass: base.via_pass,
                    ..best.cfg.clone()
                };
                geo_eval(evaluate(board, &sub, cfg, Some(&identity), best.stats.generation))
            })
        },
    );
    if let Some(c) = polished {
        if full_cost(&c) < full_cost(&best) {
            best = c;
        }
        search.report("polish", &best.stats);
    }
    // the winner again with the via pass: it stays only if nothing else gets worse
    if let Some(c) = via_passed {
        let fewer = c.stats.routed >= best.stats.routed
            && c.stats.violations <= best.stats.violations
            && c.stats.shorts <= best.stats.shorts
            && c.stats.vias < best.stats.vias;
        if fewer {
            best = c;
            search.report("via pass", &best.stats);
        }
    }
    // the open end: from here on the result only changes where a mutant makes it better
    let mut others = Vec::new();
    if open {
        (best, others) = improve::run(board, &sub, search, best, done, &mut rng, &mut |c, generation| {
            if search.frames {
                frames.push((label(c, &format!("generation {generation}")), c.geo.clone(), true));
            }
        });
    }
    // a class of its own grows only to the target its rules give it, and the width of a pair
    // is its impedance
    let pairs: Vec<NetId> = board.diff_pairs().into_iter().flat_map(|(a, b)| [a, b]).collect();
    let target = |n: NetId| board.target_width(n, search.prefer_width, &pairs);
    let grows = (0..board.nets.len() as NetId).any(|n| target(n) > 0.0);
    let finish = |mut best: Candidate, report: bool| -> Candidate {
        if search.air > 0.0 {
            if let Some(c) = improve::give_air(board, &sub, &best, search.air) {
                best = c;
                if report {
                    search.report("air", &best.stats);
                }
            }
        }
        if grows {
            if let Some(c) = improve::make_room(board, &sub, &best, target, search.air / 2.0) {
                best = c;
                if report {
                    search.report("room", &best.stats);
                }
            }
        }
        // tracks as wide as the surroundings allow; a net that gets a new violation by it falls back
        let plain = best.geo.clone();
        let known: std::collections::HashSet<(NetId, Option<NetId>)> =
            check::check(board, &plain).iter().map(|v| v.nets).collect();
        relax::widen_necks(board, &sub, &mut best.geo);
        for _ in 0..4 {
            let bad: Vec<NetId> = check::check(board, &best.geo)
                .iter()
                .filter(|v| !known.contains(&v.nets))
                .flat_map(|v| [Some(v.nets.0), v.nets.1])
                .flatten()
                .collect();
            if bad.is_empty() {
                break;
            }
            for (t, p) in best
                .geo
                .tracks
                .iter_mut()
                .zip(&plain.tracks)
                .filter(|(t, _)| bad.contains(&t.net))
            {
                if board.class_of(t.net).min_width.is_some() {
                    // a cut piece out of the zone may not be narrow: the track as it was
                    *t = p.clone();
                } else {
                    let neck = t.neck_w;
                    t.fitted.iter_mut().for_each(|f| f.1 = neck);
                }
            }
        }
        // the known pairs above may have got more spots; the open end does not give back what it won
        if open && check::check(board, &best.geo).len() > check::check(board, &plain).len() {
            best.geo = plain;
        }
        if grows {
            let plain = best.geo.clone();
            relax::prefer_width(board, &sub, &mut best.geo, target, search.air / 2.0);
            if check::check(board, &best.geo).len() > check::check(board, &plain).len() {
                best.geo = plain;
            }
        }
        // the pairs the search pulled together close up to their gap where nothing is between
        if base.pair_pull > 0.0 && std::env::var("GB_ZIP").map_or(true, |v| v != "0") {
            let zipped = pairs::zip(board, &mut best.geo);
            if std::env::var("GB_ZIP_LOG").is_ok() {
                eprintln!("zip {zipped:?}");
            }
        }
        if std::env::var("GB_TAPER").map_or(true, |v| v != "0")
            && let Some(g) = improve::taper_spots(board, &best.geo)
        {
            best.geo = g;
        }
        best
    };
    let mut best = finish(best, true);
    // tracks with a clearance problem at their pad go back onto the middle of their fine pitch
    // pad, one by one, where that leaves fewer problems
    let mut now = check::check(board, &best.geo);
    let mut straightened = 0;
    for ti in 0..best.geo.tracks.len() {
        let t = &best.geo.tracks[ti];
        let (Some(&first), Some(&last)) = (t.pts.first(), t.pts.last()) else {
            continue;
        };
        let near = |v: &check::Violation| {
            (v.nets.0 == t.net || v.nets.1 == Some(t.net)) && (v.at.dist(first) < 1.5 || v.at.dist(last) < 1.5)
        };
        if !now.iter().any(near) {
            continue;
        }
        let mut g = best.geo.clone();
        if relax::straighten_exits(board, &mut g, Some(ti)) > 0 {
            let after = check::check(board, &g);
            if after.len() < now.len() {
                best.stats.violations = Some(after.len());
                best.geo = g;
                now = after;
                straightened += 1;
            }
        }
    }
    if straightened > 0 {
        eprintln!(
            "pad exits: {straightened} tracks back on the middle of their pads, {} violations left",
            now.len()
        );
    }
    let alternatives = others
        .into_iter()
        .map(|c| {
            let c = finish(c, false);
            (c.stats, c.geo)
        })
        .collect();
    (
        sub,
        RouteResult {
            geometry: best.geo,
            stats: best.stats,
            portfolio,
            frames,
            offsets: HashMap::new(),
            alternatives,
        },
    )
}

/// The geometry of `geo` (routed on `from`) with every track end and via that sits on a pad
/// moved along with that pad's footprint on `to` (same board, other offsets).
pub(crate) fn follow_parts(from: &Board, to: &Board, geo: &Geometry) -> Geometry {
    // the moved pad under p (its obstacles on `from`) and where p goes with it
    let pad_at = |p: geom::Point, net: NetId| -> Option<(&[usize], geom::Point)> {
        from.pins.iter().zip(&to.pins).find_map(|(a, b)| {
            (a.net == net
                && a.pos.dist(b.pos) >= 1e-9
                && a.obstacles
                    .iter()
                    .any(|&o| from.obstacles[o].shape.dist_to_point(p) < 1e-6))
            .then(|| (a.obstacles.as_slice(), p + (b.pos - a.pos)))
        })
    };
    let on = |obs: &[usize], p: geom::Point| obs.iter().any(|&o| from.obstacles[o].shape.dist_to_point(p) < 1e-6);
    let shift = |p: geom::Point, net: NetId| pad_at(p, net).map_or(p, |x| x.1);
    let mut g = geo.clone();
    for t in &mut g.tracks {
        // neck zones go with the part they belong to
        for nk in &mut t.necks {
            if let Some((a, b)) = from
                .pins
                .iter()
                .zip(&to.pins)
                .find(|(a, b)| a.pos.dist(b.pos) > 1e-9 && nk.contains(a.pos))
            {
                let d = b.pos - a.pos;
                *nk = geom::Rect {
                    min: nk.min + d,
                    max: nk.max + d,
                };
            }
        }
        let n = t.pts.len();
        if n == 0 {
            continue;
        }
        let (pa, pb) = (pad_at(t.pts[0], t.net), pad_at(t.pts[n - 1], t.net));
        let da = pa.map_or(geom::Point::default(), |x| x.1 - t.pts[0]);
        let db = pb.map_or(geom::Point::default(), |x| x.1 - t.pts[n - 1]);
        if da.len() + db.len() == 0.0 {
            continue;
        }
        let pts = t.pts.clone();
        let mut at = vec![0.0; n];
        for i in 1..n {
            at[i] = at[i - 1] + pts[i].dist(pts[i - 1]);
        }
        let len = at[n - 1].max(1e-9);
        // the part of the track on the pad moves with the pad as a whole: a narrow pad holds a
        // track as wide as itself only on its middle line
        let a0 = pa.map_or(0.0, |(obs, _)| {
            (0..n).take_while(|&i| on(obs, pts[i])).last().map_or(0.0, |i| at[i])
        });
        let b0 = pb.map_or(len, |(obs, _)| {
            (0..n)
                .rev()
                .take_while(|&i| on(obs, pts[i]))
                .last()
                .map_or(len, |i| at[i])
        });
        let (a0, b0) = if a0 < b0 { (a0, b0) } else { (0.0, len) };
        // spread the rest of the end moves along the track so that it does not kink at the pad
        let reach = (b0 - a0).min(1.0).max(1e-9);
        let fade = |x: f64| if x <= 0.0 { 1.0 } else { (1.0 - x / reach).max(0.0) };
        for i in 0..n {
            t.pts[i] = pts[i] + da * fade(at[i] - a0) + db * fade(b0 - at[i]);
        }
        t.elems.clear();
    }
    for v in &mut g.vias {
        v.at = shift(v.at, v.net);
    }
    g
}

/// The pins open connections start from (`unrouted` as [`RunStats`] names them), on `b`.
fn open_pins(b: &Board, unrouted: &[String]) -> Vec<geom::Point> {
    unrouted
        .iter()
        .filter_map(|u| u.rsplit_once(" from ").map(|x| x.1))
        .filter_map(|pin| b.pins.iter().find(|p| p.name == pin).map(|p| p.pos))
        .collect()
}

/// New footprint offsets: movable parts with pads next to clearance problems step away from
/// them (boldly, 0.15 to 0.5 mm per step), as long as the courtyards stay inside the board and
/// apart from each other. `cur` is the board with the current `offsets` applied.
fn shift_parts(
    board: &Board,
    cur: &Board,
    offsets: &HashMap<String, geom::Point>,
    hot: &[geom::Point],
    rng: &mut Rng,
) -> Option<HashMap<String, geom::Point>> {
    let mut offsets = offsets.clone();
    let mut courtyards = cur.courtyards.clone();
    let mut changed = false;
    let mut spots: Vec<geom::Point> = Vec::new();
    for &h in hot {
        if spots.iter().all(|s| s.dist(h) > 1.0) {
            spots.push(h);
        }
    }
    for h in spots {
        // the part whose pin is the spot itself (an open connection) stays: its neighbours make
        // the room
        let near: Vec<&String> = board
            .movable
            .iter()
            .filter(|f| cur.pins.iter().any(|p| &p.footprint == *f && p.pos.dist(h) < 1.5))
            .filter(|f| !cur.pins.iter().any(|p| &p.footprint == *f && p.pos.dist(h) < 1e-6))
            .collect();
        if near.is_empty() {
            continue;
        }
        let fp = near[(rng.next() % near.len() as u64) as usize];
        let Some(r) = courtyards.get(fp).copied() else {
            continue;
        };
        let centre = (r.min + r.max) * 0.5;
        let away = centre - h;
        if away.len() < 1e-6 {
            continue;
        }
        let step = away.norm() * (0.15 + 0.35 * rng.unit());
        for scale in [1.0, 0.5] {
            // the part shoves the movable parts in its way along (a chain, as far as needed)
            let mut trial_off = offsets.clone();
            let mut trial_cy = courtyards.clone();
            if shove(
                board,
                fp,
                step * scale,
                &mut trial_cy,
                &mut trial_off,
                &mut Vec::new(),
                0,
            ) {
                offsets = trial_off;
                courtyards = trial_cy;
                changed = true;
                break;
            }
        }
    }
    changed.then_some(offsets)
}

/// Shift footprint `fp` by `d`, and with it the rest of its block; movable parts whose
/// courtyards it would run into are shifted by the same amount first (up to a chain of four).
/// False if a fixed part or the board edge is in the way.
fn shove(
    board: &Board,
    fp: &str,
    d: geom::Point,
    courtyards: &mut HashMap<String, geom::Rect>,
    offsets: &mut HashMap<String, geom::Point>,
    done: &mut Vec<String>,
    depth: usize,
) -> bool {
    if done.iter().any(|x| x == fp) {
        return true;
    }
    done.push(fp.to_string());
    let Some(r) = courtyards.get(fp).copied() else {
        return false;
    };
    let moved = geom::Rect {
        min: r.min + d,
        max: r.max + d,
    };
    let inside = [
        moved.min,
        moved.max,
        geom::pt(moved.min.x, moved.max.y),
        geom::pt(moved.max.x, moved.min.y),
    ]
    .iter()
    .all(|&q| board.inside(q));
    if !inside {
        return false;
    }
    courtyards.insert(fp.to_string(), moved);
    let e = offsets.entry(fp.to_string()).or_insert(geom::pt(0.0, 0.0));
    *e = *e + d;
    // the block moves as a whole (a member without courtyard only takes the offset)
    for block in board.blocks.iter().filter(|b| b.refs.iter().any(|x| x == fp)) {
        for other in &block.refs {
            if !courtyards.contains_key(other) {
                if !done.contains(other) {
                    done.push(other.clone());
                    let e = offsets.entry(other.clone()).or_insert(geom::pt(0.0, 0.0));
                    *e = *e + d;
                }
            } else if !shove(board, other, d, courtyards, offsets, done, depth) {
                return false;
            }
        }
    }
    // parts that already overlapped may keep overlapping as much as before
    let hit: Vec<String> = courtyards
        .iter()
        .filter(|(other, o)| other.as_str() != fp && overlap(o, &moved) > overlap(o, &r) + 1e-9)
        .map(|(other, _)| other.clone())
        .collect();
    for other in hit {
        if done.contains(&other) {
            continue;
        }
        if depth >= 4
            || !board.movable.contains(&other)
            || !shove(board, &other, d, courtyards, offsets, done, depth + 1)
        {
            return false;
        }
    }
    true
}

fn overlap(a: &geom::Rect, b: &geom::Rect) -> f64 {
    let w = a.max.x.min(b.max.x) - a.min.x.max(b.min.x);
    let h = a.max.y.min(b.max.y) - a.min.y.max(b.min.y);
    if w > 0.0 && h > 0.0 { w * h } else { 0.0 }
}

/// A mutant that moves the connections of nets with clearance violations (and failed ones)
/// earlier, so that they get their room before their neighbours.
fn mutate_geo(best: &Candidate, rng: &mut Rng) -> Genome {
    let n = best.order.len().max(1) as f64;
    let mut g = mutate(best, rng);
    for k in &best.order {
        if best.bad_nets.contains(&k.0) && rng.unit() < 0.6 {
            if let Some(v) = g.get_mut(k) {
                *v -= n * (0.05 + 0.5 * rng.unit());
            }
        }
    }
    g
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::Block;
    use crate::geom::{Polygon, Rect, pt};

    fn rect(x: f64, y: f64, w: f64, h: f64) -> Rect {
        Rect {
            min: pt(x, y),
            max: pt(x + w, y + h),
        }
    }

    /// A part moves sideways: the piece of its track on the pad goes along as a whole and stays
    /// on the middle line of the pad, only the rest of the track bends.
    #[test]
    fn the_track_on_a_moved_pad_stays_on_its_middle() {
        let from = crate::topo::tests::two_pad_board();
        let mut to = from.clone();
        to.pins[0].pos = to.pins[0].pos + pt(0.0, -0.05);
        let mut t = crate::check::tests::line((2.0, 5.0), (8.0, 5.0), 0);
        t.pts = (0..=60).map(|i| pt(2.0 + 0.1 * i as f64, 5.0)).collect();
        let geo = Geometry {
            tracks: vec![t],
            vias: Vec::new(),
        };
        let g = follow_parts(&from, &to, &geo);
        let pts = &g.tracks[0].pts;
        // the pad reaches to x 2.5
        for p in pts.iter().filter(|p| p.x <= 2.5 + 1e-9) {
            assert!((p.y - 4.95).abs() < 1e-9, "{p:?}");
        }
        assert!(pts.iter().all(|p| p.y <= 5.0 + 1e-12 && p.y >= 4.95 - 1e-12));
        assert!((pts[30].y - 5.0).abs() < 1e-9, "the far part stays: {:?}", pts[30]);
        assert!((pts[60].y - 5.0).abs() < 1e-9);
    }

    /// The search negotiates on its own only where the plain one left a few connections open.
    #[test]
    fn the_search_negotiates_when_a_few_stay_open() {
        let stats = |routed: usize| RunStats {
            conns: 100,
            routed,
            ..Default::default()
        };
        let plain = RouterConfig::default();
        assert!(!auto_negotiate(&plain, &stats(100)), "complete");
        assert!(auto_negotiate(&plain, &stats(99)) && auto_negotiate(&plain, &stats(85)));
        assert!(!auto_negotiate(&plain, &stats(84)), "more connections than room");
        let asked = RouterConfig {
            negotiate: 20,
            ..Default::default()
        };
        assert!(!auto_negotiate(&asked, &stats(90)), "asked for already");
    }

    fn board() -> Board {
        let mut b = Board::default();
        b.outline.push(Polygon::new(vec![
            pt(0.0, 0.0),
            pt(50.0, 0.0),
            pt(50.0, 50.0),
            pt(0.0, 50.0),
        ]));
        b.courtyards.insert("A".into(), rect(10.0, 10.0, 2.0, 2.0));
        b.courtyards.insert("B".into(), rect(13.0, 10.0, 2.0, 2.0));
        b.courtyards.insert("C".into(), rect(20.0, 20.0, 2.0, 2.0));
        b.movable = vec!["A".into(), "B".into(), "C".into()];
        b
    }

    #[test]
    fn a_block_moves_as_a_whole() {
        let mut b = board();
        b.blocks.push(Block {
            refs: vec!["A".into(), "C".into()],
            ..Default::default()
        });
        let (mut cy, mut off) = (b.courtyards.clone(), HashMap::new());
        assert!(shove(&b, "A", pt(0.5, 0.0), &mut cy, &mut off, &mut Vec::new(), 0));
        assert_eq!(off.len(), 2);
        assert_eq!(off["C"].x, 0.5);
        assert_eq!(cy["C"].min.x, 20.5);
    }

    #[test]
    fn shoving_pushes_movable_neighbours_and_stops_at_fixed_ones() {
        let mut b = board();
        let (mut cy, mut off) = (b.courtyards.clone(), HashMap::new());
        assert!(shove(&b, "A", pt(1.5, 0.0), &mut cy, &mut off, &mut Vec::new(), 0));
        assert_eq!(off["B"].x, 1.5);
        b.movable.retain(|f| f != "B");
        let (mut cy, mut off) = (b.courtyards.clone(), HashMap::new());
        assert!(!shove(&b, "A", pt(1.5, 0.0), &mut cy, &mut off, &mut Vec::new(), 0));
    }
}
