//! Gummiband core: geometry, board model and the topological router.

pub mod board;
pub mod check;
pub mod geom;
pub mod index;
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

#[derive(Debug, Clone, Default)]
pub struct RunStats {
    pub seed: u64,
    pub fanout_first: bool,
    pub via_cost: f64,
    /// Generation of the evolutionary search (0 = initial portfolio).
    pub generation: usize,
    pub conns: usize,
    pub routed: usize,
    pub vias: usize,
    pub length: f64,
    pub unrouted: Vec<String>,
    /// Clearance violations after the geometry step (None if not relaxed).
    pub violations: Option<usize>,
}

impl RunStats {
    /// Lower is better: completion first, then vias, then length.
    pub fn cost(&self) -> f64 {
        (self.conns - self.routed) as f64 * 1e6 + self.vias as f64 * 2.0 + self.length * 0.05
    }
}

pub struct RouteResult {
    pub geometry: Geometry,
    pub stats: RunStats,
    pub portfolio: Vec<RunStats>,
}

#[derive(Debug, Clone)]
pub struct SearchConfig {
    /// Parallel runs per generation.
    pub population: usize,
    /// Generations of the evolutionary search after the initial portfolio (0 = portfolio only).
    pub generations: usize,
    /// Generations that also run the geometry step and select by clearance violations.
    pub geometry_generations: usize,
}

/// Routing order genome: a key per connection, identified by (net, from pin). Lower routes first.
type Genome = HashMap<(NetId, usize), f64>;

struct Candidate {
    stats: RunStats,
    geo: Geometry,
    /// Final connection order and the unrouted connections, for the next genome.
    order: Vec<(NetId, usize)>,
    failed: Vec<(NetId, usize)>,
    /// Nets with clearance violations after the geometry step.
    bad_nets: Vec<NetId>,
}

fn evaluate(
    board: &Board,
    sub: &Substrate,
    cfg: RouterConfig,
    genome: Option<&Genome>,
    generation: usize,
) -> Candidate {
    let mut r = Router::new(board, sub, cfg.clone());
    r.plan();
    if let Some(g) = genome {
        r.reorder(|c| g.get(&(c.net, c.from)).copied().unwrap_or(f64::MAX));
    }
    let routed = r.run();
    debug_hooks(board, &mut r);
    let geo = r.geometry();
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
    let stats = RunStats {
        seed: cfg.seed,
        fanout_first: cfg.fanout_first,
        via_cost: cfg.via_cost,
        generation,
        conns: r.conns.len(),
        routed,
        vias: geo.vias.len(),
        length,
        unrouted,
        violations: None,
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
    for _ in 0..shuffles {
        let k = keys[(rng.next() % keys.len().max(1) as u64) as usize];
        if let Some(v) = g.get_mut(&k) {
            *v += (rng.unit() - 0.5) * n * 0.3;
        }
    }
    g
}

/// Route a board: build the substrate once, run a portfolio of router configurations in
/// parallel, then improve the best routing order in generations of mutants (completion first,
/// then vias, then length). The winner is relaxed into its final geometry.
pub fn route_board(
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
            ..base.clone()
        })
        .collect();
    let first: Vec<Candidate> = configs
        .into_par_iter()
        .map(|cfg| evaluate(board, &sub, cfg, None, 0))
        .collect();
    let mut portfolio: Vec<RunStats> = first.iter().map(|c| c.stats.clone()).collect();
    // the best few candidates go through the geometry step at the end
    let keep = std::env::var("GB_FINALISTS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(8);
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
    let mut rng = Rng(0x5eed);
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
            },
        );
    }
    // relax the finalists of the best completion and keep the cleanest geometry; then, if
    // asked, run generations that select by the clearance violations after the geometry step
    let geo_eval = |mut c: Candidate| -> Candidate {
        relax::relax(board, &sub, &mut c.geo, &relax::RelaxConfig::default());
        let v = check::check(board, &c.geo);
        let mut nets: Vec<NetId> = v.iter().flat_map(|x| [Some(x.nets.0), x.nets.1]).flatten().collect();
        nets.sort_unstable();
        nets.dedup();
        c.stats.violations = Some(v.len());
        c.bad_nets = nets;
        c
    };
    let full_cost = |c: &Candidate| c.stats.cost() + c.stats.violations.unwrap_or(0) as f64 * 1000.0;
    let top = finalists[0].stats.routed;
    let judged: Vec<Candidate> = finalists
        .into_iter()
        .filter(|c| c.stats.routed == top)
        .collect::<Vec<_>>()
        .into_par_iter()
        .map(geo_eval)
        .collect();
    for c in &judged {
        portfolio.push(c.stats.clone());
    }
    let mut best = judged
        .into_iter()
        .min_by(|a, b| full_cost(a).total_cmp(&full_cost(b)))
        .unwrap();
    for generation in 1..=search.geometry_generations {
        if best.stats.violations == Some(0) && best.failed.is_empty() {
            break;
        }
        let genomes: Vec<Genome> = (0..pop).map(|_| mutate_geo(&best, &mut rng)).collect();
        let cfg = RouterConfig {
            seed: 0,
            fanout_first: best.stats.fanout_first,
            via_cost: best.stats.via_cost,
            ..base.clone()
        };
        let next: Vec<Candidate> = genomes
            .par_iter()
            .map(|g| {
                geo_eval(evaluate(
                    board,
                    &sub,
                    cfg.clone(),
                    Some(g),
                    search.generations + generation,
                ))
            })
            .collect();
        for c in next {
            portfolio.push(c.stats.clone());
            if full_cost(&c) < full_cost(&best) {
                best = c;
            }
        }
    }
    (
        sub,
        RouteResult {
            geometry: best.geo,
            stats: best.stats,
            portfolio,
        },
    )
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
