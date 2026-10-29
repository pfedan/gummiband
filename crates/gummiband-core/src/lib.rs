//! Gummiband core: geometry, board model and the topological router.

pub mod board;
pub mod check;
pub mod geom;
pub mod index;
pub mod realize;
pub mod relax;
pub mod route;
pub mod topo;

use rayon::prelude::*;

use board::Board;
use realize::Geometry;
use route::{Router, RouterConfig};
use topo::{Substrate, SubstrateConfig};

#[derive(Debug, Clone, Default)]
pub struct RunStats {
    pub seed: u64,
    pub fanout_first: bool,
    pub conns: usize,
    pub routed: usize,
    pub vias: usize,
    pub length: f64,
    pub unrouted: Vec<String>,
}

pub struct RouteResult {
    pub geometry: Geometry,
    pub stats: RunStats,
    pub portfolio: Vec<RunStats>,
}

/// Route a board: build the substrate once, run a portfolio of router configurations in
/// parallel and keep the best result (completion, then vias, then length).
pub fn route_board(
    board: &Board,
    sub_cfg: &SubstrateConfig,
    base: &RouterConfig,
    runs: usize,
) -> (Substrate, RouteResult) {
    let sub = Substrate::build(board, sub_cfg);
    let configs: Vec<RouterConfig> = (0..runs.max(1))
        .map(|i| RouterConfig {
            seed: i as u64,
            fanout_first: i % 2 == 0,
            ..base.clone()
        })
        .collect();
    let results: Vec<(RunStats, Geometry)> = configs
        .into_par_iter()
        .map(|cfg| {
            let mut r = Router::new(board, &sub, cfg.clone());
            r.plan();
            let routed = r.run();
            let geo = r.geometry();
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
            if std::env::var("GB_DEBUG_X").is_ok() {
                for l in r.debug_intersections(12) {
                    eprintln!("{l}");
                }
            }
            let length = geo
                .tracks
                .iter()
                .map(|t| t.pts.windows(2).map(|w| w[0].dist(w[1])).sum::<f64>())
                .sum();
            let unrouted = r
                .unrouted()
                .into_iter()
                .map(|ci| {
                    let c = &r.conns[ci];
                    format!("{} from {}", board.nets[c.net as usize].name, board.pins[c.from].name)
                })
                .collect();
            let stats = RunStats {
                seed: cfg.seed,
                fanout_first: cfg.fanout_first,
                conns: r.conns.len(),
                routed,
                vias: geo.vias.len(),
                length,
                unrouted,
            };
            (stats, geo)
        })
        .collect();
    let portfolio: Vec<RunStats> = results.iter().map(|r| r.0.clone()).collect();
    let (stats, mut geometry) = results
        .into_iter()
        .max_by(|a, b| {
            a.0.routed
                .cmp(&b.0.routed)
                .then(b.0.vias.cmp(&a.0.vias))
                .then(b.0.length.total_cmp(&a.0.length))
        })
        .unwrap();
    if std::env::var("GB_NO_RELAX").is_err() {
        relax::relax(board, &sub, &mut geometry, &relax::RelaxConfig::default());
    }
    (
        sub,
        RouteResult {
            geometry,
            stats,
            portfolio,
        },
    )
}
