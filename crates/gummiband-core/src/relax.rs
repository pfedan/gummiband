//! Elastic optimizer: every track is a chain of points under tension; obstacles and tracks of
//! other nets push with their clearance. All tracks relax in parallel (Jacobi style: each reads
//! the previous state of the others). The topology comes from the router, the relaxation only
//! moves points a little per round, so tracks do not jump over each other; it shortens the bands,
//! spreads them and removes clearance violations.

use rayon::prelude::*;

use crate::board::{Board, LayerId, ObstacleKind};
use crate::geom::{Point, Rect, point_seg_dist, pt};
use crate::index::{GridIndex, PointGrid};
use crate::realize::{Elem, Geometry, Track};
use crate::topo::Substrate;

#[derive(Debug, Clone)]
pub struct RelaxConfig {
    pub iterations: usize,
    /// Target point spacing along a track in mm.
    pub spacing: f64,
    /// Fraction of the way to the neighbour midpoint per round.
    pub tension: f64,
    /// Extra clearance kept for numerical safety in mm.
    pub margin: f64,
    /// Largest move of a point per round in mm.
    pub max_step: f64,
}

impl Default for RelaxConfig {
    fn default() -> Self {
        RelaxConfig {
            iterations: env_usize("GB_RELAX_IT", 400),
            spacing: std::env::var("GB_SPACING")
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(0.05),
            tension: 0.25,
            margin: 0.008,
            max_step: 0.03,
        }
    }
}

struct Piece {
    a: Point,
    b: Point,
    track: u32,
}

/// Squared distances are compared with this much slack, so that a rejection is certain and
/// only the rare remaining candidates need the exact test.
const SURE: f64 = 1.0 + 1e-9;

/// True if the point at offset (dx, dy) is certainly at least `r` away.
#[inline]
fn surely_outside(dx: f64, dy: f64, r: f64) -> bool {
    dx * dx + dy * dy >= r * r * SURE
}

/// What the point loop needs to know of a piece, kept in grid order for cheap sweeps.
#[derive(Clone, Copy)]
struct NearPiece {
    mid: Point,
    /// Half the track width plus half the piece length: bounds the piece around `mid`.
    extent: f64,
    net: u32,
    id: u32,
}

/// The pieces of one layer in a grid.
struct LayerPieces {
    grid: PointGrid,
    /// In grid order.
    near: Vec<NearPiece>,
    /// Pieces and their keys in id order while building, kept to reuse the buffers.
    raw: Vec<NearPiece>,
    mids: Vec<Point>,
}

impl LayerPieces {
    fn new(bounds: Rect, cell: f64) -> LayerPieces {
        LayerPieces {
            grid: PointGrid::new(bounds, cell, &[]),
            near: Vec::new(),
            raw: Vec::new(),
            mids: Vec::new(),
        }
    }

    fn rebuild(&mut self) {
        self.mids.clear();
        self.mids.extend(self.raw.iter().map(|r| r.mid));
        self.grid.rebuild(&self.mids);
        self.near.clear();
        let raw = &self.raw;
        self.near.extend(self.grid.ids().iter().map(|&i| raw[i as usize]));
    }

    /// Calls `f` for all pieces near `p`, by cell and within a cell by ascending id.
    #[inline]
    fn for_each_near(&self, p: Point, radius: f64, mut f: impl FnMut(&NearPiece)) {
        self.grid.for_each_run(p, radius, |lo, hi| {
            for r in &self.near[lo..hi] {
                f(r);
            }
        });
    }
}

/// What the point loop needs to know of an obstacle.
#[derive(Clone, Copy)]
struct NearObstacle {
    bbox: Rect,
    /// Tracks of this net ignore the obstacle.
    own_net: u32,
    /// Clearance it demands; `floor` means at least the clearance of the track's net.
    clr: f64,
    floor: bool,
}

fn resample(pts: &[Point], spacing: f64) -> Vec<Point> {
    let mut out = vec![pts[0]];
    for w in pts.windows(2) {
        let l = w[0].dist(w[1]);
        let n = (l / spacing).ceil().max(1.0) as usize;
        for i in 1..=n {
            out.push(w[0].lerp(w[1], i as f64 / n as f64));
        }
    }
    out.dedup_by(|a, b| a.dist(*b) < 1e-6);
    if out.len() < 2 {
        out.push(*pts.last().unwrap());
    }
    out
}

/// Drop points that lie on the line of their neighbours (within `tol`).
pub fn simplify(pts: &[Point], tol: f64) -> Vec<Point> {
    if pts.len() <= 2 {
        return pts.to_vec();
    }
    let mut keep = vec![false; pts.len()];
    keep[0] = true;
    keep[pts.len() - 1] = true;
    let mut stack = vec![(0usize, pts.len() - 1)];
    while let Some((a, b)) = stack.pop() {
        let mut best = (0.0, 0usize);
        for i in a + 1..b {
            let d = point_seg_dist(pts[i], pts[a], pts[b]);
            if d > best.0 {
                best = (d, i);
            }
        }
        if best.0 > tol {
            keep[best.1] = true;
            stack.push((a, best.1));
            stack.push((best.1, b));
        }
    }
    pts.iter().zip(keep).filter(|(_, k)| *k).map(|(p, _)| *p).collect()
}

pub fn relax(board: &Board, sub: &Substrate, geo: &mut Geometry, cfg: &RelaxConfig) {
    let clr = |n: u32| board.class_of(n).clearance.max(board.rules.min_clearance);
    let t_start = std::time::Instant::now();
    // coarse points while the bands move a lot, fine points for the final clearance work
    let coarse = std::env::var("GB_COARSE")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or((cfg.spacing * 2.0).max(0.1));
    let mut tracks: Vec<Vec<Point>> = geo.tracks.iter().map(|t| resample(&t.pts, coarse)).collect();
    // how far any push can reach: largest clearance plus the widest track
    let max_clr = board
        .classes
        .iter()
        .map(|c| c.clearance)
        .fold(board.rules.min_clearance, f64::max)
        .max(board.pair_clearance.values().copied().fold(0.0, f64::max))
        .max(board.obstacles.iter().map(|o| o.clearance).fold(0.0, f64::max))
        .max(board.rules.hole_clearance);
    let max_w = geo.tracks.iter().map(|t| t.width).fold(0.0, f64::max);
    // the same reach for track pairs only: obstacles often demand much more
    let max_track_clr = board
        .classes
        .iter()
        .map(|c| c.clearance)
        .fold(board.rules.min_clearance, f64::max)
        .max(board.pair_clearance.values().copied().fold(0.0, f64::max));
    let n_layers = geo.tracks.iter().map(|t| t.layer as usize + 1).max().unwrap_or(0);
    let half_w = |t: &Track| if t.necks.is_empty() { t.width } else { t.width.max(t.neck_w) } / 2.0;
    let mut max_hw = vec![0.0f64; n_layers];
    for t in &geo.tracks {
        max_hw[t.layer as usize] = max_hw[t.layer as usize].max(half_w(t));
    }
    let obs_boxes: Vec<Rect> = sub.shared.obs_polys.iter().map(|p| p.bbox()).collect();
    let bounds = board.bbox().grow(5.0);
    // every point query below looks at one cell only: the cell lists hold all obstacles within
    // the largest reach of any point in it, and the edge mask marks cells near the board edge
    let reach_max = max_w + max_clr + cfg.margin + 0.01;
    let cells = CellLists::new(bounds, 0.5, &obs_boxes, reach_max, |_| true);
    // the same per layer, without the obstacles tracks there never see
    let track_obstacle = |o: &crate::board::Obstacle, l: LayerId| {
        o.layers.has(l) && !matches!(o.kind, ObstacleKind::Keepout { tracks: false, .. })
    };
    let layer_cells: Vec<CellLists> = (0..n_layers)
        .map(|l| {
            let l = l as LayerId;
            CellLists::new(bounds, 0.5, &obs_boxes, reach_max, |i| {
                track_obstacle(&board.obstacles[i], l)
            })
        })
        .collect();
    let near_obs: Vec<NearObstacle> = board
        .obstacles
        .iter()
        .zip(&obs_boxes)
        .map(|(o, b)| {
            let (clr, floor) = match o.kind {
                ObstacleKind::Keepout { .. } => (crate::board::KEEPOUT_GAP, false),
                ObstacleKind::Hole => (board.rules.hole_clearance, false),
                _ => (o.clearance, true),
            };
            NearObstacle {
                bbox: *b,
                own_net: if o.kind == ObstacleKind::Hole {
                    u32::MAX
                } else {
                    o.net.unwrap_or(u32::MAX)
                },
                clr,
                floor,
            }
        })
        .collect();
    let near_edge: Vec<bool> = (0..cells.len())
        .map(|c| {
            let (r, half) = cells.cell_rect(c);
            let mid = (r.min + r.max) * 0.5;
            let d = board
                .outline
                .iter()
                .map(|o| nearest_on_poly(o, mid).0)
                .fold(f64::INFINITY, f64::min);
            d < board.rules.edge_clearance + max_w / 2.0 + cfg.margin + half + 0.01 || !board.inside(mid)
        })
        .collect();
    // vias as static obstacles: (centre, radius, net, clearance)
    let mut vias: Vec<(Point, f64, u32, f64)> = geo
        .vias
        .iter()
        .map(|v| (v.at, v.diameter / 2.0, v.net, clr(v.net)))
        .collect();
    let max_vr = vias.iter().map(|v| v.1).fold(0.0, f64::max);
    let max_drill = drills_max(geo);
    let edge_clr = board.rules.edge_clearance;
    let routing = board.routing_layers();
    let drills: Vec<f64> = geo.vias.iter().map(|v| v.drill).collect();
    // track ends that sit on a via of their net follow the via
    let attach = |p: Point, net: u32, vias: &[(Point, f64, u32, f64)]| {
        vias.iter().position(|v| v.2 == net && v.0.dist(p) < 1e-6)
    };
    let ends: Vec<(Option<usize>, Option<usize>)> = tracks
        .iter()
        .zip(&geo.tracks)
        .map(|(p, t)| (attach(p[0], t.net, &vias), attach(*p.last().unwrap(), t.net, &vias)))
        .collect();
    // per via the layers and widths of the tracks that end on it
    let mut via_tracks: Vec<Vec<(_, f64)>> = vec![Vec::new(); vias.len()];
    for (t, (a, b)) in geo.tracks.iter().zip(&ends) {
        for v in [a, b].into_iter().flatten() {
            via_tracks[*v].push((t.layer, t.width_at(vias[*v].0)));
        }
    }

    // while annealing only a few spots still move: cells near the points and vias that moved
    // in the previous round are the only ones worth visiting (None: visit everything)
    let mut dirty: Option<Vec<bool>> = None;
    let mut grids: Vec<LayerPieces> = (0..n_layers).map(|_| LayerPieces::new(bounds, 0.25)).collect();
    let mut via_at: Vec<Point> = Vec::with_capacity(vias.len());
    let mut via_grid = PointGrid::new(bounds, 1.0, &[]);
    let mut pieces: Vec<Piece> = Vec::new();
    let annealing = |it: usize| it * 4 >= cfg.iterations * 3;
    for it in 0..cfg.iterations {
        // anneal: the last quarter only resolves clearances
        let tension = if annealing(it) { 0.0 } else { cfg.tension };
        let before = annealing(it).then(|| (tracks.clone(), vias.iter().map(|v| v.0).collect::<Vec<_>>()));
        if it % 10 == 9 {
            tracks = tracks
                .par_iter()
                .map(|p| {
                    resample(
                        &simplify(p, 0.0005),
                        if it * 4 >= cfg.iterations * 3 {
                            cfg.spacing
                        } else {
                            coarse
                        },
                    )
                })
                .collect();
        }
        pieces.clear();
        for g in &mut grids {
            g.raw.clear();
        }
        // pieces by their midpoints per layer; queries grow by the longest half piece
        let mut half = 0.0f64;
        for (ti, p) in tracks.iter().enumerate() {
            let t = &geo.tracks[ti];
            let (hw, raw) = (half_w(t), &mut grids[t.layer as usize].raw);
            for w in p.windows(2) {
                let id = pieces.len() as u32;
                pieces.push(Piece {
                    a: w[0],
                    b: w[1],
                    track: ti as u32,
                });
                let hl = (w[1] - w[0]).len2().sqrt() * 0.5 * SURE;
                half = half.max(hl);
                raw.push(NearPiece {
                    mid: (w[0] + w[1]) * 0.5,
                    extent: hw + hl,
                    net: t.net,
                    id,
                });
            }
        }
        grids.iter_mut().for_each(LayerPieces::rebuild);
        let (pieces, grids) = (&pieces, &grids);
        via_at.clear();
        via_at.extend(vias.iter().map(|v| v.0));
        via_grid.rebuild(&via_at);
        let via_grid = &via_grid;
        // vias move as well: pushed by obstacles, other vias and tracks of other nets
        let moved: Vec<Point> = vias
            .par_iter()
            .enumerate()
            .map(|(vi, &(vc, vr, vn, vclr))| {
                let mut push = pt(0.0, 0.0);
                for oi in sub.shared.obs_index.query_point(vc, vr + 1.0) {
                    let o = &board.obstacles[oi as usize];
                    if !o.layers.intersects(routing) && o.hole.is_none() {
                        continue;
                    }
                    if o.net == Some(vn) && o.kind != ObstacleKind::Hole {
                        continue;
                    }
                    let need = match o.kind {
                        ObstacleKind::Keepout { vias: no_vias, tracks } => {
                            // a via outside a track keepout still needs room for the ends of
                            // its tracks on that layer (they end at the via centre)
                            let ends = via_tracks[vi]
                                .iter()
                                .filter(|(l, _)| tracks && o.layers.has(*l))
                                .map(|(_, w)| w / 2.0)
                                .fold(f64::NEG_INFINITY, f64::max);
                            let r = if no_vias { vr.max(ends) } else { ends };
                            if r == f64::NEG_INFINITY {
                                continue;
                            }
                            crate::board::KEEPOUT_GAP + r
                        }
                        ObstacleKind::Hole => board.rules.hole_clearance + vr,
                        _ => o.clearance.max(vclr) + vr,
                    } + cfg.margin;
                    let poly = &sub.shared.obs_polys[oi as usize];
                    let inside = poly.contains(vc);
                    let (d, q) = nearest_on_poly(poly, vc);
                    let d = if inside { -d } else { d };
                    if d < need {
                        let dir = if inside { (q - vc).norm() } else { (vc - q).norm() };
                        push = push + dir * (need - d);
                    }
                }
                // sums run in id order: collect the few candidates first
                let mut hits: Vec<u32> = Vec::new();
                let via_reach = (vr + max_vr + max_clr).max(drills[vi] / 2.0 + max_drill / 2.0 + board.rules.hole_to_hole)
                    + cfg.margin
                    + 0.01;
                via_grid.for_each_near(vc, via_reach, |vj| {
                    if vj as usize != vi {
                        hits.push(vj);
                    }
                });
                hits.sort_unstable();
                for &vj in &hits {
                    let vj = vj as usize;
                    let (c2, r2, n2, clr2) = vias[vj];
                    let holes = drills[vi] / 2.0 + drills[vj] / 2.0 + board.rules.hole_to_hole + cfg.margin;
                    let need = if n2 == vn {
                        holes
                    } else {
                        (vr + r2 + vclr.max(clr2) + cfg.margin).max(holes)
                    };
                    let dv = vc - c2;
                    if surely_outside(dv.x, dv.y, need) {
                        continue;
                    }
                    let d = vc.dist(c2);
                    if d < need {
                        let dir = if d > 1e-9 {
                            (vc - c2).norm()
                        } else {
                            pt(1.0, 0.0).rotate(vi as f64)
                        };
                        push = push + dir * ((need - d) * 0.5);
                    }
                }
                // candidates of all layers, back in the order of one grid over all of them
                let mut near: Vec<(usize, u32)> = Vec::new();
                let base = vr + max_clr + cfg.margin;
                for g in grids {
                    g.for_each_near(vc, base + max_w / 2.0 + 0.1 + half, |r| {
                        let (dm, lim) = (vc - r.mid, base + r.extent);
                        if r.net != vn && !surely_outside(dm.x, dm.y, lim) {
                            near.push((g.grid.cell_of(r.mid), r.id));
                        }
                    });
                }
                near.sort_unstable();
                for &(_, pi) in &near {
                    let pc = &pieces[pi as usize];
                    let o = &geo.tracks[pc.track as usize];
                    let need = vr + o.piece_width(pc.a, pc.b) / 2.0 + vclr.max(clr(o.net)) + cfg.margin;
                    let ab = pc.b - pc.a;
                    let l2 = ab.len2();
                    let tt = if l2 > 0.0 {
                        ((vc - pc.a).dot(ab) / l2).clamp(0.0, 1.0)
                    } else {
                        0.0
                    };
                    let q = pc.a + ab * tt;
                    let d = vc.dist(q);
                    if d < need {
                        let dir = if d > 1e-9 { (vc - q).norm() } else { ab.perp().norm() };
                        push = push + dir * ((need - d) * 0.5);
                    }
                }
                let l = push.len();
                let mv = if l > cfg.max_step {
                    push * (cfg.max_step / l)
                } else {
                    push
                };
                vc + mv
            })
            .collect();
        let snapshot = &tracks;
        tracks = snapshot
            .par_iter()
            .enumerate()
            .map(|(ti, pts)| {
                let t = &geo.tracks[ti];
                let (c, net) = (clr(t.net), t.net);
                let (grid, obs_cells) = (&grids[t.layer as usize], &layer_cells[t.layer as usize]);
                let mut out = pts.clone();
                let n = pts.len();
                // points in parallel too: a few long tracks would otherwise keep one core busy
                let inner: Vec<Point> = (1..n.saturating_sub(1))
                    .into_par_iter()
                    .with_min_len(32)
                    .map(|i| {
                        let p = pts[i];
                        if dirty.as_ref().is_some_and(|d| !d[cells.cell_of(p)]) {
                            return p;
                        }
                        let w = t.width_at(p);
                        // tension
                        let mid = (pts[i - 1] + pts[i + 1]) * 0.5;
                        let mut mv = (mid - p) * tension;
                        let mut push = pt(0.0, 0.0);
                        // static obstacles
                        let cell = cells.cell_of(p);
                        for &oi in obs_cells.get(cell) {
                            let o = &near_obs[oi as usize];
                            if o.own_net == net {
                                continue;
                            }
                            let need = if o.floor { o.clr.max(c) } else { o.clr } + w / 2.0 + cfg.margin;
                            // cheap reject on the bounding box before the polygon distance
                            let dx = (o.bbox.min.x - p.x).max(0.0).max(p.x - o.bbox.max.x);
                            let dy = (o.bbox.min.y - p.y).max(0.0).max(p.y - o.bbox.max.y);
                            if surely_outside(dx, dy, need) || dx.hypot(dy) >= need {
                                continue;
                            }
                            let poly = &sub.shared.obs_polys[oi as usize];
                            let inside = poly.contains(p);
                            let (d, q) = nearest_on_poly(poly, p);
                            let d = if inside { -d } else { d };
                            if d < need {
                                let dir = if inside { (q - p).norm() } else { (p - q).norm() };
                                push = push + dir * (need - d);
                            }
                        }
                        // board edge (only near it)
                        let inside_board = !near_edge[cell] || board.inside(p);
                        for o in board.outline.iter().filter(|_| near_edge[cell]) {
                            let (d, q) = nearest_on_poly(o, p);
                            let need = edge_clr + w / 2.0 + cfg.margin;
                            // a point outside the board always goes back in
                            if d < need || !inside_board {
                                let dir = if inside_board { (p - q).norm() } else { (q - p).norm() };
                                let d = if inside_board { d } else { -d };
                                push = push + dir * (need - d);
                            }
                        }
                        // vias of other nets
                        let mut hits: Vec<u32> = Vec::new();
                        via_grid.for_each_near(p, w / 2.0 + max_clr + max_vr + cfg.margin + 0.01, |vi| {
                            let (vc, vr, vn, vclr) = vias[vi as usize];
                            let dv = p - vc;
                            if vn != net && !surely_outside(dv.x, dv.y, vr + vclr.max(c) + w / 2.0 + cfg.margin) {
                                hits.push(vi);
                            }
                        });
                        hits.sort_unstable();
                        for &vi in &hits {
                            let (vc, vr, _, vclr) = vias[vi as usize];
                            let need = vr + vclr.max(c) + w / 2.0 + cfg.margin;
                            let d = p.dist(vc);
                            if d < need {
                                push = push + (p - vc).norm() * ((need - d) * 0.5);
                            }
                        }
                        // other tracks (they push back from their side, so take half)
                        let clr_reach = w / 2.0 + max_track_clr + cfg.margin;
                        grid.for_each_near(p, clr_reach + max_hw[t.layer as usize] + half, |r| {
                            let (dm, lim) = (p - r.mid, clr_reach + r.extent);
                            if r.net == net || surely_outside(dm.x, dm.y, lim) {
                                return;
                            }
                            let pc = &pieces[r.id as usize];
                            let o = &geo.tracks[pc.track as usize];
                            let need = w / 2.0
                                + o.piece_width(pc.a, pc.b) / 2.0
                                + board.track_clearance(net, o.net)
                                + cfg.margin;
                            let ab = pc.b - pc.a;
                            let l2 = ab.len2();
                            let tt = if l2 > 0.0 {
                                ((p - pc.a).dot(ab) / l2).clamp(0.0, 1.0)
                            } else {
                                0.0
                            };
                            let q = pc.a + ab * tt;
                            let d = p.dist(q);
                            if d < need {
                                let dir = if d > 1e-9 { (p - q).norm() } else { ab.perp().norm() };
                                push = push + dir * ((need - d) * 0.5);
                            }
                        });
                        // constraints win over tension: drop the tension part that points into the push
                        if push.len2() > 0.0 {
                            let pn = push.norm();
                            let into = mv.dot(pn);
                            if into < 0.0 {
                                mv = mv - pn * into;
                            }
                            mv = mv + push;
                        }
                        let l = mv.len();
                        if l > cfg.max_step {
                            mv = mv * (cfg.max_step / l);
                        }
                        p + mv
                    })
                    .collect();
                out[1..1 + inner.len()].copy_from_slice(&inner);
                out
            })
            .collect();
        for (v, p) in vias.iter_mut().zip(moved) {
            v.0 = p;
        }
        for (pts, (a, b)) in tracks.iter_mut().zip(&ends) {
            if let Some(a) = a {
                pts[0] = vias[*a].0;
            }
            if let Some(b) = b {
                let n = pts.len();
                pts[n - 1] = vias[*b].0;
            }
        }
        dirty = before.map(|(old_tracks, old_vias)| {
            let mut d = vec![false; cells.len()];
            let k = ((reach_max + cfg.max_step) / cells.cell).ceil() as isize + 1;
            let mut mark = |p: Point| {
                let c = cells.cell_of(p) as isize;
                let (cx, cy) = (c % cells.nx as isize, c / cells.nx as isize);
                for y in (cy - k).max(0)..=(cy + k).min(cells.ny as isize - 1) {
                    for x in (cx - k).max(0)..=(cx + k).min(cells.nx as isize - 1) {
                        d[(y * cells.nx as isize + x) as usize] = true;
                    }
                }
            };
            for (a, b) in old_tracks.iter().zip(&tracks) {
                if a.len() != b.len() {
                    a.iter().chain(b).for_each(|p| mark(*p));
                } else {
                    for (p, q) in a.iter().zip(b) {
                        if p != q && p.dist(*q) > 1e-9 {
                            mark(*p);
                            mark(*q);
                        }
                    }
                }
            }
            for (p, v) in old_vias.iter().zip(&vias) {
                if *p != v.0 && p.dist(v.0) > 1e-9 {
                    mark(*p);
                    mark(v.0);
                }
            }
            d
        });
    }
    for (v, (p, ..)) in geo.vias.iter_mut().zip(&vias) {
        v.at = *p;
    }
    let t_taut = std::time::Instant::now();
    let tracks = taut(board, sub, geo, tracks, cfg);
    if std::env::var("GB_TIMING").is_ok() {
        eprintln!(
            "relax rounds {:.2} s, taut {:.2} s",
            t_taut.duration_since(t_start).as_secs_f64(),
            t_taut.elapsed().as_secs_f64()
        );
    }
    for (t, (pts, elems)) in geo.tracks.iter_mut().zip(tracks) {
        *t = Track {
            elems,
            pts,
            width: t.width,
            layer: t.layer,
            net: t.net,
            necks: std::mem::take(&mut t.necks),
            neck_w: t.neck_w,
        };
    }
}

/// Final pull: replace runs of points by straight shortcuts wherever the shortcut keeps all
/// clearances (to obstacles, vias and the current geometry of the other tracks). This removes
/// the small waves the relaxation leaves and straightens the bands. Tracks are processed one by
/// one, each against the already shortened ones.
fn taut(
    board: &Board,
    sub: &Substrate,
    geo: &Geometry,
    mut tracks: Vec<Vec<Point>>,
    cfg: &RelaxConfig,
) -> Vec<(Vec<Point>, Vec<Elem>)> {
    let clr = |n: u32| board.class_of(n).clearance.max(board.rules.min_clearance);
    let bounds = board.bbox().grow(5.0);
    let mut result: Vec<(Vec<Point>, Vec<Elem>)> = Vec::with_capacity(tracks.len());
    let mut idx = GridIndex::new(bounds, 0.5);
    // pieces: (a, b, track); a removed piece gets track = u32::MAX
    let mut pieces: Vec<(Point, Point, u32)> = Vec::new();
    let mut owned: Vec<Vec<u32>> = vec![Vec::new(); tracks.len()];
    let add = |idx: &mut GridIndex,
               pieces: &mut Vec<(Point, Point, u32)>,
               owned: &mut Vec<Vec<u32>>,
               ti: usize,
               p: &[Point]| {
        for w in p.windows(2) {
            let id = pieces.len() as u32;
            pieces.push((w[0], w[1], ti as u32));
            let mut r = Rect::empty();
            r.add(w[0]);
            r.add(w[1]);
            idx.insert(id, &r);
            owned[ti].push(id);
        }
    };
    for (ti, p) in tracks.iter().enumerate() {
        add(&mut idx, &mut pieces, &mut owned, ti, p);
    }
    let routing_vias: Vec<(Point, f64, u32)> = geo.vias.iter().map(|v| (v.at, v.diameter / 2.0, v.net)).collect();
    let m = cfg.margin * 0.5;
    for ti in 0..tracks.len() {
        let t = &geo.tracks[ti];
        let (net, layer, c) = (t.net, t.layer, clr(t.net));
        let clear = |a: Point, b: Point, pieces: &[(Point, Point, u32)], idx: &GridIndex| -> bool {
            let w = t.piece_width(a, b);
            let mut r = Rect::empty();
            r.add(a);
            r.add(b);
            let rr = r.grow(w / 2.0 + 1.0);
            for oi in sub.shared.obs_index.query(&rr) {
                let o = &board.obstacles[oi as usize];
                if !o.layers.has(layer) || (o.net == Some(net) && o.kind != ObstacleKind::Hole) {
                    continue;
                }
                let need = match o.kind {
                    ObstacleKind::Keepout { tracks, .. } => {
                        if !tracks {
                            continue;
                        }
                        crate::board::KEEPOUT_GAP
                    }
                    ObstacleKind::Hole => board.rules.hole_clearance,
                    _ => o.clearance.max(c),
                } + w / 2.0
                    + m;
                if sub.shared.obs_polys[oi as usize].dist_to_seg(a, b) < need {
                    return false;
                }
            }
            for o in &board.outline {
                if o.edges()
                    .map(|(p, q)| crate::geom::seg_seg_dist(a, b, p, q))
                    .fold(f64::INFINITY, f64::min)
                    < board.rules.edge_clearance + w / 2.0 + m
                {
                    return false;
                }
            }
            for &(vc, vr, vn) in &routing_vias {
                if vn != net && point_seg_dist(vc, a, b) < vr + c.max(clr(vn)) + w / 2.0 + m {
                    return false;
                }
            }
            for pi in idx.query(&rr) {
                let (p, q, tj) = pieces[pi as usize];
                if tj == u32::MAX || tj as usize == ti {
                    continue;
                }
                let o = &geo.tracks[tj as usize];
                if o.net == net || o.layer != layer {
                    continue;
                }
                let need = w / 2.0 + o.piece_width(p, q) / 2.0 + board.track_clearance(net, o.net) + m;
                if crate::geom::seg_seg_dist(a, b, p, q) < need {
                    return false;
                }
            }
            true
        };
        let pts = &tracks[ti];
        let n = pts.len();
        if n <= 2 {
            let p = simplify(pts, 0.0005);
            result.push((p.clone(), p.windows(2).map(|w| Elem::Line(w[0], w[1])).collect()));
            continue;
        }
        let mut out = vec![pts[0]];
        let mut i = 0;
        while i < n - 1 {
            // farthest visible point: exponential search, then bisection (visibility along a
            // band is close to monotone, so this needs a handful of checks instead of a scan)
            let mut good = i + 1;
            let mut step = 2;
            let mut bad = n;
            while good + step < n {
                let k = good + step;
                if clear(pts[i], pts[k], &pieces, &idx) {
                    good = k;
                    step *= 2;
                } else {
                    bad = k;
                    break;
                }
            }
            if bad == n && good + 1 < n && clear(pts[i], pts[n - 1], &pieces, &idx) {
                good = n - 1;
            } else {
                let mut hi = bad.min(n);
                while hi - good > 1 {
                    let mid = (good + hi) / 2;
                    if clear(pts[i], pts[mid], &pieces, &idx) {
                        good = mid;
                    } else {
                        hi = mid;
                    }
                }
            }
            let j = good;
            out.push(pts[j]);
            i = j;
        }
        let out = simplify(&out, 0.0005);
        // round the corners with the largest arcs that keep all clearances
        let mut elems: Vec<Elem> = Vec::new();
        let mut flat: Vec<Point> = vec![out[0]];
        let mut cur = out[0];
        for i in 1..out.len() {
            let b = out[i];
            if i + 1 < out.len() {
                let (a, c) = (out[i - 1], out[i + 1]);
                let (u, v) = ((a - b).norm(), (c - b).norm());
                let cos = u.dot(v).clamp(-1.0, 1.0);
                let theta = cos.acos(); // interior angle
                if theta < std::f64::consts::PI - 0.03 && theta > 0.05 {
                    let half = theta / 2.0;
                    let t_max = (a.dist(b) * 0.45).min(c.dist(b) * 0.45).min(b.dist(cur));
                    let mut r = t_max * half.tan();
                    let mut done = false;
                    while r >= 0.1 && !done {
                        let t = r / half.tan();
                        let (p1, p2) = (b + u * t, b + v * t);
                        let bis = (u + v).norm();
                        let center = b + bis * (r / half.sin());
                        let mid = center - bis * r;
                        let arc = crate::geom::arc_polyline(p1, mid, p2, 0.002);
                        if arc.windows(2).all(|w| clear(w[0], w[1], &pieces, &idx)) {
                            if cur.dist(p1) > 1e-6 {
                                elems.push(Elem::Line(cur, p1));
                            }
                            elems.push(Elem::Arc(p1, mid, p2));
                            flat.extend_from_slice(&arc);
                            cur = p2;
                            done = true;
                        }
                        r *= 0.5;
                    }
                    if done {
                        continue;
                    }
                }
            }
            elems.push(Elem::Line(cur, b));
            flat.push(b);
            cur = b;
        }
        flat.dedup_by(|a, b| a.dist(*b) < 1e-9);
        for &id in &owned[ti] {
            pieces[id as usize].2 = u32::MAX;
        }
        owned[ti].clear();
        add(&mut idx, &mut pieces, &mut owned, ti, &flat);
        tracks[ti] = flat.clone();
        result.push((flat, elems));
    }
    result
}

/// Distance from p to the polygon boundary and the nearest boundary point.
fn nearest_on_poly(poly: &crate::geom::Polygon, p: Point) -> (f64, Point) {
    let mut best = (f64::INFINITY, p);
    for (a, b) in poly.edges() {
        let ab = b - a;
        let l2 = ab.len2();
        let t = if l2 > 0.0 {
            ((p - a).dot(ab) / l2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let q = a + ab * t;
        let d2 = (p - q).len2();
        if d2 < best.0 {
            best = (d2, q);
        }
    }
    (best.0.sqrt(), best.1)
}

fn drills_max(geo: &Geometry) -> f64 {
    geo.vias.iter().map(|v| v.drill).fold(0.0, f64::max)
}

fn env_usize(k: &str, d: usize) -> usize {
    std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(d)
}

/// Uniform grid where each cell lists the boxes within `reach` of it, so a point query is a
/// single slice lookup without deduplication.
struct CellLists {
    origin: Point,
    cell: f64,
    nx: usize,
    ny: usize,
    start: Vec<u32>,
    ids: Vec<u32>,
}

impl CellLists {
    fn new(bounds: Rect, cell: f64, boxes: &[Rect], reach: f64, keep: impl Fn(usize) -> bool) -> CellLists {
        let nx = ((bounds.width() / cell).ceil() as usize).max(1);
        let ny = ((bounds.height() / cell).ceil() as usize).max(1);
        let mut lists: Vec<Vec<u32>> = vec![Vec::new(); nx * ny];
        let f = |v: f64, o: f64, n: usize| (((v - o) / cell).floor().max(0.0) as usize).min(n - 1);
        for (i, b) in boxes.iter().enumerate().filter(|(i, _)| keep(*i)) {
            let g = b.grow(reach);
            for y in f(g.min.y, bounds.min.y, ny)..=f(g.max.y, bounds.min.y, ny) {
                for x in f(g.min.x, bounds.min.x, nx)..=f(g.max.x, bounds.min.x, nx) {
                    lists[y * nx + x].push(i as u32);
                }
            }
        }
        let mut start = vec![0u32];
        let mut ids = Vec::new();
        for l in lists {
            ids.extend(l);
            start.push(ids.len() as u32);
        }
        CellLists {
            origin: bounds.min,
            cell,
            nx,
            ny,
            start,
            ids,
        }
    }

    fn len(&self) -> usize {
        self.nx * self.ny
    }

    fn cell_of(&self, p: Point) -> usize {
        let f = |v: f64, o: f64, n: usize| (((v - o) / self.cell).floor().max(0.0) as usize).min(n - 1);
        f(p.y, self.origin.y, self.ny) * self.nx + f(p.x, self.origin.x, self.nx)
    }

    /// The cell's rectangle and half its diagonal.
    fn cell_rect(&self, c: usize) -> (Rect, f64) {
        let (x, y) = ((c % self.nx) as f64, (c / self.nx) as f64);
        let min = pt(self.origin.x + x * self.cell, self.origin.y + y * self.cell);
        let mut r = Rect::empty();
        r.add(min);
        r.add(pt(min.x + self.cell, min.y + self.cell));
        (r, self.cell * std::f64::consts::FRAC_1_SQRT_2)
    }

    fn get(&self, c: usize) -> &[u32] {
        &self.ids[self.start[c] as usize..self.start[c + 1] as usize]
    }
}
