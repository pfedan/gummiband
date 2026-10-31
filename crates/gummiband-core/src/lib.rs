//! Gummiband core: geometry, board model and the topological router.

pub mod board;
pub mod check;
pub mod geom;
pub mod import;
mod improve;
pub mod index;
pub mod pairs;
pub mod prof;
pub mod realize;
pub mod relax;
pub mod route;
pub mod topo;

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
}

impl RunStats {
    /// Lower is better: completion first, then vias, then length. One unrouted connection
    /// weighs as much as ten clearance violations (see the geometry stage).
    pub fn cost(&self) -> f64 {
        (self.conns - self.routed) as f64 * 1e4 + self.vias as f64 * 2.0 + self.length * 0.05
    }
}

/// Share of the differential pair track length that runs next to its partner on the same
/// layer, at most `near` mm away: (coupled mm, total mm).
pub fn pair_coupling(board: &Board, geo: &Geometry, near: f64) -> (f64, f64) {
    let (mut coupled, mut total) = (0.0, 0.0);
    for (a, b) in board.diff_pairs() {
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
    }
    (coupled, total)
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
    /// The open end has found its level: taking the result now loses little.
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
}

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
        evaluate_inner(board, sub, cfg, genome, generation)
    })
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
        generation,
        conns: r.conns.len(),
        routed,
        vias: geo.vias.iter().filter(|v| v.drill > 0.0).count(),
        length,
        unrouted,
        violations: None,
        shorts: None,
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

/// Route a board (see [`route_fixed`]). With movable parts and clearance problems left, parts
/// next to the problems are shifted and the whole search runs again on the shifted board, for
/// a few placement rounds; the best result over all rounds wins.
pub fn route_board(
    board: &Board,
    sub_cfg: &SubstrateConfig,
    base: &RouterConfig,
    search: &SearchConfig,
) -> (Substrate, RouteResult) {
    let score = |b: &Board, r: &RouteResult| -> (usize, f64, Vec<geom::Point>) {
        let v = check::check(b, &r.geometry);
        let shorts = v.iter().filter(|x| x.actual <= 0.0).count();
        let cost = r.stats.cost() + v.len() as f64 * 1000.0 + shorts as f64 * 5000.0;
        (r.stats.routed, cost, v.iter().map(|x| x.at).collect())
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
        if (hot.is_empty() && best.1.stats.unrouted.is_empty()) || search.stopped() {
            break;
        }
        let cur = board.with_offsets(&offsets);
        let Some(next) = shift_parts(board, &cur, &offsets, &hot, &mut rng) else {
            break;
        };
        let moved = board.with_offsets(&next);
        // fast path, the rubber band way: keep the topology of the best routing, let the pads
        // take their track ends along and the relaxation push the tracks aside
        let quick = {
            let sub = Substrate::build(&moved, sub_cfg);
            let mut res = RouteResult {
                geometry: follow_parts(&cur, &moved, &best.1.geometry),
                stats: best.1.stats.clone(),
                portfolio: Vec::new(),
                frames: Vec::new(),
                offsets: next.clone(),
            };
            relax::relax(&moved, &sub, &mut res.geometry, &relax::RelaxConfig::default());
            (sub, res)
        };
        let qs = score(&moved, &quick.1);
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
            offsets = next;
            let mut st = best.1.stats.clone();
            st.violations = Some(best_score.2.len());
            search.report(&format!("placement round {round}"), &st);
            continue;
        }
        // a full new search pays off only for a handful of local problems, and only on boards
        // where a search is not expensive; otherwise the fast path above is all there is.
        // By board size and not by the clock: the result must not depend on the machine.
        if hot.len() > 20 || best.1.stats.conns > 400 {
            continue;
        }
        let (sub, mut res) = route_search(&moved, sub_cfg, base, search);
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
        hot = sc.2.clone();
        offsets = next;
        if sc.0 > best_score.0 || (sc.0 == best_score.0 && sc.1 < best_score.1) {
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
    let negotiated = route_fixed(board, sub_cfg, base, search);
    if base.negotiate == 0 || std::env::var("GB_NEG_ONLY").is_ok() || search.stopped() {
        return negotiated;
    }
    let plain = RouterConfig {
        negotiate: 0,
        ..base.clone()
    };
    let (sub, mut res) = route_fixed(board, sub_cfg, &plain, search);
    let key = |r: &RouteResult| {
        let v = check::check(board, &r.geometry);
        let shorts = v.iter().filter(|x| x.actual <= 0.0).count();
        (
            r.stats.routed,
            r.stats.cost() + v.len() as f64 * 1000.0 + shorts as f64 * 5000.0,
        )
    };
    let (a, b) = (key(&negotiated.1), key(&res));
    eprintln!(
        "negotiated: {}/{} routed, cost {:.0}; plain: {}/{} routed, cost {:.0}",
        a.0, negotiated.1.stats.conns, a.1, b.0, res.stats.conns, b.1
    );
    if a.0 > b.0 || (a.0 == b.0 && a.1 < b.1) {
        return negotiated;
    }
    res.portfolio.extend(negotiated.1.portfolio);
    (sub, res)
}

/// One full search on a fixed placement: build the substrate once, run a portfolio of router
/// configurations in parallel, then improve the best routing order in generations of mutants
/// (completion first, then vias, then length). The winner is relaxed into its final geometry.
pub fn route_fixed(
    board: &Board,
    sub_cfg: &SubstrateConfig,
    base: &RouterConfig,
    search: &SearchConfig,
) -> (Substrate, RouteResult) {
    let sub = Substrate::build(board, sub_cfg);
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
    // polish: a few clearance spots are often left that more rounds without tension resolve
    if best.stats.violations.unwrap_or(0) > 0 && std::env::var("GB_NO_POLISH").is_err() {
        let polish = relax::RelaxConfig {
            tension: 0.0,
            ..full_relax.clone()
        };
        let c = geo_eval_with(best.clone(), &polish);
        if full_cost(&c) < full_cost(&best) {
            best = c;
        }
        search.report("polish", &best.stats);
    }
    // the winner again with the via pass: it stays only if nothing else gets worse
    if base.via_pass > 0.0 && best.stats.vias > 0 {
        let identity: Genome = best.order.iter().enumerate().map(|(i, &k)| (k, i as f64)).collect();
        let cfg = RouterConfig {
            via_pass: base.via_pass,
            ..best.cfg.clone()
        };
        let c = geo_eval(evaluate(board, &sub, cfg, Some(&identity), best.stats.generation));
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
    if open {
        best = improve::run(board, &sub, search, best, done, &mut rng, &mut |c, generation| {
            if search.frames {
                frames.push((
                    label(c, &format!("generation {generation}")),
                    c.geo.clone(),
                    true,
                ));
            }
        });
    }
    if search.air > 0.0 {
        if let Some(c) = improve::give_air(board, &sub, &best, search.air) {
            best = c;
            search.report("air", &best.stats);
        }
    }
    // a class of its own grows only to the target its rules give it, and the width of a pair
    // is its impedance
    let pairs: Vec<NetId> = board.diff_pairs().into_iter().flat_map(|(a, b)| [a, b]).collect();
    let target = |n: NetId| board.target_width(n, search.prefer_width, &pairs);
    let grows = (0..board.nets.len() as NetId).any(|n| target(n) > 0.0);
    if grows {
        if let Some(c) = improve::make_room(board, &sub, &best, target, search.air / 2.0) {
            best = c;
            search.report("room", &best.stats);
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
        for t in best.geo.tracks.iter_mut().filter(|t| bad.contains(&t.net)) {
            let neck = t.neck_w;
            t.fitted.iter_mut().for_each(|f| f.1 = neck);
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
    (
        sub,
        RouteResult {
            geometry: best.geo,
            stats: best.stats,
            portfolio,
            frames,
            offsets: HashMap::new(),
        },
    )
}

/// The geometry of `geo` (routed on `from`) with every track end and via that sits on a pad
/// moved along with that pad's footprint on `to` (same board, other offsets).
fn follow_parts(from: &Board, to: &Board, geo: &Geometry) -> Geometry {
    let shift = |p: geom::Point, net: NetId| -> geom::Point {
        for (a, b) in from.pins.iter().zip(&to.pins) {
            if a.net != net || a.pos.dist(b.pos) < 1e-9 {
                continue;
            }
            if a.obstacles
                .iter()
                .any(|&o| from.obstacles[o].shape.dist_to_point(p) < 1e-6)
            {
                return p + (b.pos - a.pos);
            }
        }
        p
    };
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
        let (a, b) = (shift(t.pts[0], t.net), shift(t.pts[n - 1], t.net));
        let (da, db) = (a - t.pts[0], b - t.pts[n - 1]);
        if da.len() + db.len() == 0.0 {
            continue;
        }
        // spread the end moves along the track so that it does not kink at the pad
        let len: f64 = t.pts.windows(2).map(|w| w[0].dist(w[1])).sum::<f64>().max(1e-9);
        let reach = len.min(1.0);
        let fade = |x: f64| (1.0 - x / reach).max(0.0);
        let mut s = 0.0;
        let pts = t.pts.clone();
        for i in 0..n {
            if i > 0 {
                s += pts[i].dist(pts[i - 1]);
            }
            t.pts[i] = pts[i] + da * fade(s) + db * fade(len - s);
        }
        t.elems.clear();
    }
    for v in &mut g.vias {
        v.at = shift(v.at, v.net);
    }
    g
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
        let near: Vec<&String> = board
            .movable
            .iter()
            .filter(|f| cur.pins.iter().any(|p| &p.footprint == *f && p.pos.dist(h) < 1.5))
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
