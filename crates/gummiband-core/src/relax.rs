//! Elastic optimizer: every track is a chain of points under tension; obstacles and tracks of
//! other nets push with their clearance. All tracks relax in parallel (Jacobi style: each reads
//! the previous state of the others). The topology comes from the router, the relaxation only
//! moves points a little per round, so tracks do not jump over each other; it shortens the bands,
//! spreads them and removes clearance violations.

use rayon::prelude::*;

use crate::board::{Board, LayerId, LayerSet, ObstacleKind};
use crate::geom::{Point, Rect, Shape, point_seg_dist, point_seg_offset, pt, seg_seg_dist, seg_seg_nearest};
use crate::index::{GridIndex, PointGrid, axis_cell};
use crate::knobs;
use crate::prof::{self, Slot};
use crate::realize::{Elem, Geometry, Track, Via};
use crate::topo::Substrate;

#[cfg(feature = "gpu")]
#[path = "relax_gpu.rs"]
mod gpu;

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
    /// Experimental: run the tension rounds on the GPU (needs the `gpu` feature).
    pub gpu: bool,
    /// The board edge pushes vias as well, a via stays in the plane it reaches, and tracks leave
    /// fine pitch pads straight (see [`pin_rails`]). Off for the search itself, where vias stay
    /// near the place the router gave them; passes that push hard can shove a via over the edge
    /// rule, or out of the zone it hangs on.
    pub via_edge: bool,
    /// Finish with the shortcuts of [`taut`]. Off, the tracks stay chains of short pieces where
    /// the rounds left them, and a clearance check can say where it hurts.
    pub taut: bool,
}

impl Default for RelaxConfig {
    fn default() -> Self {
        RelaxConfig {
            iterations: knobs::RELAX_IT.num(),
            spacing: knobs::SPACING.num(),
            tension: 0.25,
            margin: 0.008,
            max_step: 0.03,
            gpu: knobs::GPU.on(),
            via_edge: false,
            taut: true,
        }
    }
}

struct Piece {
    a: Point,
    b: Point,
    track: u32,
}

/// The way a track leaves a fine pitch pad: on the middle of the pad, up to `len` from its centre.
/// Next to a pin at 0.4 mm pitch there is room for the track and hardly more, and neighbours push
/// each other sideways: a track that lies 20 um off the middle of its own pad breaks the clearance
/// to the next one.
#[derive(Debug, Clone, Copy)]
struct Rail {
    at: Point,
    dir: Point,
    len: f64,
}

/// The tracks of `tracks` that end on a track of their net, each with the track it ends on.
fn tap_targets(tracks: &[Track]) -> Vec<(usize, Vec<usize>)> {
    let mut out = Vec::new();
    for (ti, t) in tracks.iter().enumerate().filter(|(_, t)| t.tap.is_some()) {
        let on = |(_, o): &(usize, &Track)| Some(o.seg) == t.tap;
        let others: Vec<usize> = tracks.iter().enumerate().filter(on).map(|x| x.0).collect();
        if !others.is_empty() {
            out.push((ti, others));
        }
    }
    out
}

/// Where the chains `others` come closest to `p`.
fn nearest_on_tracks(chains: &[Vec<Point>], others: &[usize], p: Point) -> Option<Point> {
    let mut best: Option<(f64, Point)> = None;
    for &o in others {
        for w in chains[o].windows(2) {
            let ab = w[1] - w[0];
            let x = ((p - w[0]).dot(ab) / ab.len2().max(1e-12)).clamp(0.0, 1.0);
            let q = w[0] + ab * x;
            let d = (q - p).len2();
            if best.is_none_or(|b| d < b.0) {
                best = Some((d, q));
            }
        }
    }
    best.map(|b| b.1)
}

/// Put the ends of the tapping tracks on the tracks they end on.
pub(crate) fn snap_taps(tracks: &mut [Track]) {
    for (ti, others) in tap_targets(tracks) {
        let chains: Vec<Vec<Point>> = others.iter().map(|&o| tracks[o].pts.clone()).collect();
        let index: Vec<usize> = (0..chains.len()).collect();
        let n = tracks[ti].pts.len();
        if let Some(q) = nearest_on_tracks(&chains, &index, tracks[ti].pts[n - 1]) {
            tracks[ti].pts[n - 1] = q;
        }
    }
}

/// Pads count as fine pitch up to this distance to the next pad of their part.
const RAIL_PITCH: f64 = 0.65;
/// How far a rail goes on beyond the end of the pad. Longer rails take the room to fan out.
const RAIL_OUT: f64 = 0.1;

fn pin_rails(board: &Board) -> crate::index::FastMap<u32, Vec<Rail>> {
    let mut parts: crate::index::FastMap<&str, Vec<usize>> = Default::default();
    for (i, p) in board.pins.iter().enumerate() {
        parts.entry(p.footprint.as_str()).or_default().push(i);
    }
    let mut out: crate::index::FastMap<u32, Vec<Rail>> = Default::default();
    for pins in parts.values().filter(|p| p.len() >= 4) {
        let centre =
            pins.iter().map(|&p| board.pins[p].pos).fold(pt(0.0, 0.0), |a, b| a + b) * (1.0 / pins.len() as f64);
        for &i in pins {
            let pin = &board.pins[i];
            let pitch = pins
                .iter()
                .filter(|&&j| j != i)
                .map(|&j| board.pins[j].pos.dist(pin.pos))
                .fold(f64::INFINITY, f64::min);
            let Some(&o) = pin.obstacles.first().filter(|_| pitch <= RAIL_PITCH) else {
                continue;
            };
            let bb = board.obstacles[o].shape.bbox();
            let (w, h) = (bb.width(), bb.height());
            let out_of_part = pin.pos - centre;
            let (axis, off, along) = if w >= h {
                (pt(1.0, 0.0), out_of_part.x, w)
            } else {
                (pt(0.0, 1.0), out_of_part.y, h)
            };
            // A pad across the middle of its part (the centre strip of a DFN, say) can be left
            // at either end; its side of the centre is only rounding then, so both ends get a rail.
            let dirs: &[f64] = if off.abs() < along / 4.0 {
                &[1.0, -1.0]
            } else {
                &[off.signum()]
            };
            for &sg in dirs {
                out.entry(pin.net).or_default().push(Rail {
                    at: pin.pos,
                    dir: axis * sg,
                    len: along / 2.0 + RAIL_OUT,
                });
            }
        }
    }
    out
}

/// The rail a track end at index 0 of `pts` follows: a pad of its net, and the track goes that way.
fn rail_of<'a>(rails: &[Rail], pts: impl Iterator<Item = &'a Point> + Clone) -> Option<Rail> {
    let start = *pts.clone().next()?;
    rails.iter().filter(|r| r.at.dist(start) < 0.35).find_map(|r| {
        let far = pts.clone().map(|p| *p - r.at).find(|d| d.len() >= r.len)?;
        (far.dot(r.dir) >= 0.5 * far.len()).then_some(*r)
    })
}

impl Rail {
    /// Still on the rail: a track that turns away has left it, wherever it goes then.
    fn holds(&self, p: Point) -> bool {
        let d = p - self.at;
        d.dot(self.dir) <= self.len && d.len() <= self.len + 0.05
    }
}

/// Pull the points of a track end onto its rail, up to where it leaves it. The far end stays.
fn follow(r: &Rail, pts: &mut [Point], from_end: bool) {
    let last = pts.len() - 1;
    let mut pull = |p: &mut Point| {
        let on = r.holds(*p);
        if on {
            *p = r.at + r.dir * (*p - r.at).dot(r.dir).max(0.0);
        }
        on
    };
    if from_end {
        pts[1..].iter_mut().rev().all(&mut pull);
    } else {
        pts[..last].iter_mut().all(&mut pull);
    }
}

/// Put the ends of the finished tracks back on the rails of their fine pitch pads (see
/// [`pin_rails`]). The main relaxation runs without rails, and its tension pulls a track that
/// turns right behind a long narrow pad a little off the middle line: as wide as the pad, it then
/// sticks out of it towards the neighbour. The pieces on the rail become straight lines on it.
/// Returns the number of tracks that changed.
/// `only`: just that track.
pub(crate) fn straighten_exits(board: &Board, geo: &mut Geometry, only: Option<usize>) -> usize {
    let by_net = pin_rails(board);
    let mut changed = 0;
    for (ti, t) in geo.tracks.iter_mut().enumerate() {
        if only.is_some_and(|o| o != ti) {
            continue;
        }
        let Some(rails) = by_net.get(&t.net) else {
            continue;
        };
        if t.elems.is_empty() || t.pts.len() < 2 {
            continue;
        }
        let ends = [rail_of(rails, t.pts.iter()), rail_of(rails, t.pts.iter().rev())];
        let mut elems = t.elems.clone();
        for (from_end, r) in ends.iter().enumerate() {
            let Some(r) = r else {
                continue;
            };
            let proj = |p: Point| r.at + r.dir * (p - r.at).dot(r.dir).max(0.0);
            let n = elems.len();
            for k in 0..n {
                let i = if from_end == 1 { n - 1 - k } else { k };
                let (a, b) = match elems[i] {
                    Elem::Line(a, b) | Elem::Arc(a, _, b) => (a, b),
                };
                // a and b in the order seen from the pad
                let (near, far) = if from_end == 1 { (b, a) } else { (a, b) };
                if !r.holds(near) {
                    break;
                }
                let far_on = r.holds(far);
                let (near, far) = (proj(near), if far_on { proj(far) } else { far });
                elems[i] = if from_end == 1 {
                    Elem::Line(far, near)
                } else {
                    Elem::Line(near, far)
                };
                if !far_on {
                    break;
                }
            }
        }
        // the neighbours of a replaced piece share its ends
        for i in 1..elems.len() {
            let end = match elems[i - 1] {
                Elem::Line(_, b) | Elem::Arc(_, _, b) => b,
            };
            match &mut elems[i] {
                Elem::Line(a, _) | Elem::Arc(a, _, _) => *a = end,
            }
        }
        elems.retain(|e| !matches!(e, Elem::Line(a, b) if a.dist(*b) < 1e-9));
        if elems.is_empty() {
            continue;
        }
        let pts = crate::realize::flatten(&elems);
        if pts.len() != t.pts.len() || pts.iter().zip(&t.pts).any(|(p, q)| p.dist(*q) > 1e-9) {
            t.elems = elems;
            t.pts = pts;
            changed += 1;
        }
    }
    changed
}

/// Squared distances are compared with this much slack, so that a rejection is certain and
/// only the rare remaining candidates need the exact test.
const SURE: f64 = 1.0 + 1e-9;

/// True if the point at offset (dx, dy) is certainly at least `r` away.
#[inline]
fn surely_outside(dx: f64, dy: f64, r: f64) -> bool {
    dx * dx + dy * dy >= r * r * SURE
}

/// True if the point at offset (dx, dy) is certainly no farther than `r`.
#[inline]
fn surely_within(dx: f64, dy: f64, r: f64) -> bool {
    dx * dx + dy * dy <= r * r * (1.0 / SURE)
}

/// True if the offset `v` is longer than `r`; the root only where the squares cannot tell.
#[inline]
fn longer(v: Point, r: f64) -> bool {
    if surely_outside(v.x, v.y, r) {
        true
    } else if surely_within(v.x, v.y, r) {
        false
    } else {
        v.len() > r
    }
}

/// `a.max(b)` for an `a` that is not NaN, in one instruction instead of the NaN handling of
/// `f64::max` (the result is the same).
#[inline]
fn max_of(a: f64, b: f64) -> f64 {
    if b > a { b } else { a }
}

/// What the point loop needs to know of a piece, kept in grid order for cheap sweeps.
#[derive(Clone, Copy)]
struct NearPiece {
    mid: Point,
    /// Half the track width plus half the piece length: bounds the piece around `mid`.
    extent: f32,
    /// The largest track clearance its net can ask for against any other net.
    clr: f32,
    net: u32,
    id: u32,
}

/// Pieces in a grid, with the bounds that size a query.
struct Tier {
    grid: PointGrid,
    /// In grid order.
    near: Vec<NearPiece>,
    /// The order key of each piece in `near`, see `LayerPieces`.
    keys: Vec<u64>,
    /// Pieces and their keys in id order while building, kept to reuse the buffers.
    raw: Vec<NearPiece>,
    /// `raw` changed since the last build.
    moved: bool,
    mids: Vec<Point>,
    max_extent: f64,
    max_clr: f64,
}

impl Tier {
    fn new(bounds: Rect, cell: f64) -> Tier {
        Tier {
            grid: PointGrid::new(bounds, cell, &[]),
            near: Vec::new(),
            keys: Vec::new(),
            raw: Vec::new(),
            moved: false,
            mids: Vec::new(),
            max_extent: 0.0,
            max_clr: 0.0,
        }
    }

    fn rebuild(&mut self) {
        self.mids.clear();
        self.mids.extend(self.raw.iter().map(|r| r.mid));
        self.grid.rebuild(&self.mids);
        self.near.clear();
        let raw = &self.raw;
        self.near.extend(self.grid.ids().iter().map(|&i| raw[i as usize]));
        let (e, c) = raw
            .iter()
            .fold((0.0f32, 0.0f32), |(e, c), r| (e.max(r.extent), c.max(r.clr)));
        (self.max_extent, self.max_clr) = (e as f64, c as f64);
    }

    #[inline]
    fn for_each_near(&self, p: Point, radius: f64, mut f: impl FnMut(&NearPiece)) {
        self.grid.for_each_run(p, radius, |lo, hi| {
            for r in &self.near[lo..hi] {
                f(r);
            }
        });
    }

    /// Like `for_each_near`, with the order key of each piece.
    #[inline]
    fn for_each_keyed(&self, p: Point, radius: f64, mut f: impl FnMut(&NearPiece, u64)) {
        self.grid.for_each_run(p, radius, |lo, hi| {
            for (r, &k) in self.near[lo..hi].iter().zip(&self.keys[lo..hi]) {
                f(r, k);
            }
        });
    }
}

/// How far a point may lie from the point of the last grid sweep and reuse its result.
const SWEEP_SKIN: f64 = 0.4;

/// Scratch of the point loop. Its buffers come from a pool of the thread and go back to it:
/// a relaxation makes a few for every track in every round.
struct Sweep {
    /// Pieces and vias of other nets within the skin of `anchor`, and the candidates of the
    /// current point (pieces by their order keys, vias by id).
    anchor: Point,
    /// In key order, so that the candidates of a point are in key order as well.
    list: Vec<Swept>,
    /// Scratch of the merge of the tiers.
    merge: Vec<Swept>,
    via_list: Vec<SweptVia>,
    near: Vec<u64>,
    vias: Vec<u32>,
}

/// The buffers of a `Sweep`, kept for the next one.
#[derive(Default)]
struct SweepBufs {
    list: Vec<Swept>,
    merge: Vec<Swept>,
    via_list: Vec<SweptVia>,
    near: Vec<u64>,
    vias: Vec<u32>,
}

thread_local! {
    static SWEEP_POOL: std::cell::RefCell<Vec<SweepBufs>> = const { std::cell::RefCell::new(Vec::new()) };
}

impl Default for Sweep {
    fn default() -> Sweep {
        let mut b = SWEEP_POOL.with(|p| p.borrow_mut().pop()).unwrap_or_default();
        b.list.clear();
        b.merge.clear();
        b.via_list.clear();
        b.near.clear();
        b.vias.clear();
        Sweep {
            anchor: pt(f64::INFINITY, f64::INFINITY),
            list: b.list,
            merge: b.merge,
            via_list: b.via_list,
            near: b.near,
            vias: b.vias,
        }
    }
}

impl Drop for Sweep {
    fn drop(&mut self) {
        let b = SweepBufs {
            list: std::mem::take(&mut self.list),
            merge: std::mem::take(&mut self.merge),
            via_list: std::mem::take(&mut self.via_list),
            near: std::mem::take(&mut self.near),
            vias: std::mem::take(&mut self.vias),
        };
        // a thread holds few sweeps at a time (nested jobs); more would only keep memory
        let _ = SWEEP_POOL.try_with(|p| {
            let mut p = p.borrow_mut();
            if p.len() < 16 {
                p.push(b);
            }
        });
    }
}

/// A via in the sweep of a track: its centre, the radius and clearance the point filter needs,
/// and its id.
#[derive(Clone, Copy)]
struct SweptVia {
    at: Point,
    /// Radius plus the larger of its clearance and the track's; the custom rules come on top.
    reach: f64,
    id: u32,
}

/// A piece in the sweep of a track: what the point filter needs, and the order key.
#[derive(Clone, Copy)]
struct Swept {
    mid: Point,
    extent: f32,
    clr: f32,
    key: u64,
}

/// The pieces of one layer. A single wide or high clearance net would size the query of every
/// point on the board, so the few tracks that reach far sit in a coarse grid of their own.
/// Pushes must add up in the same order whatever the grids: by the cell of the fine grid and
/// within a cell by id. The order key holds both, the cell above the id.
struct LayerPieces {
    fine: Tier,
    far: Tier,
}

impl LayerPieces {
    fn new(bounds: Rect) -> LayerPieces {
        LayerPieces {
            fine: Tier::new(bounds, 0.25),
            far: Tier::new(bounds, 1.0),
        }
    }

    /// The grids of the tiers whose pieces moved.
    fn rebuild(&mut self) {
        let key = |cell: u32, r: &NearPiece| (cell as u64) << 32 | r.id as u64;
        if std::mem::take(&mut self.fine.moved) {
            self.fine.rebuild();
            let Tier { grid, near, keys, .. } = &mut self.fine;
            let cells = grid.key_cells();
            keys.clear();
            keys.extend(
                grid.ids()
                    .iter()
                    .zip(near.iter())
                    .map(|(&i, r)| key(cells[i as usize], r)),
            );
        }
        if std::mem::take(&mut self.far.moved) {
            self.far.rebuild();
            let grid = &self.fine.grid;
            self.far.keys.clear();
            self.far
                .keys
                .extend(self.far.near.iter().map(|r| key(grid.cell_of(r.mid) as u32, r)));
        }
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
    hole: bool,
}

pub(crate) fn resample(pts: &[Point], spacing: f64) -> Vec<Point> {
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

/// What a relaxation needs of the board alone, for one margin, widest track and layer count.
/// The open end relaxes thousands of mutants of one board, and on a board with many cutouts
/// the edge mask alone took seconds per call, so it is built once and kept with the substrate.
pub struct BoardStatic {
    bounds: Rect,
    cells: CellLists,
    layer_cells: Vec<CellLists>,
    near_obs: Vec<NearObstacle>,
    near_edge: Vec<bool>,
    /// The board edge by cells, for a board of many outlines or outlines of many edges (a
    /// panel with mouse bites has hundreds). `edge_segs` are the edges of all outlines, outline
    /// by outline, each with its outline; `edge_cells` holds per cell the edges whose box comes
    /// within the reach of the edge clearance of a track as wide as the widest (ascending):
    /// the nearest point of an outline that is near enough to push is on one of them.
    edge_segs: Vec<(u32, Point, Point)>,
    edge_cells: CellLists,
    /// Per row of cells the edges (as the even-odd test takes them) that reach into the row:
    /// only they can cross the line of a point in the row, so they alone decide whether the
    /// point is on the board.
    row_start: Vec<u32>,
    row_edges: Vec<(Point, Point)>,
}

impl BoardStatic {
    /// `Board::inside` for a point in row `row` of the cells: the same answer from the edges
    /// that can tell (the parity of all outlines is the parity of all their crossings).
    #[inline]
    fn inside(&self, row: usize, p: Point) -> bool {
        let mut inside = false;
        for &(a, b) in &self.row_edges[self.row_start[row] as usize..self.row_start[row + 1] as usize] {
            if (a.y > p.y) != (b.y > p.y) {
                let x = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
                if p.x < x {
                    inside = !inside;
                }
            }
        }
        inside
    }
}

/// The board-static part of a relaxation, from the cache of `sub` or built (and kept) now. It
/// is built without holding the lock, because building runs on rayon and a thread that waits
/// for its parallel work may pick up another relaxation that asks for the same entry.
fn board_static(
    board: &Board,
    sub: &Substrate,
    max_w: f64,
    reach_max: f64,
    margin: f64,
    n_layers: usize,
) -> std::sync::Arc<BoardStatic> {
    let key = (max_w.to_bits(), margin.to_bits(), reach_max.to_bits(), n_layers);
    if let Some(s) = sub.shared.relax_cache.lock().unwrap().iter().find(|e| e.0 == key) {
        return s.1.clone();
    }
    let built = std::sync::Arc::new(build_board_static(board, sub, max_w, reach_max, margin, n_layers));
    let mut cache = sub.shared.relax_cache.lock().unwrap();
    if let Some(s) = cache.iter().find(|e| e.0 == key) {
        return s.1.clone();
    }
    if cache.len() >= 4 {
        cache.remove(0);
    }
    cache.push((key, built.clone()));
    built
}

fn build_board_static(
    board: &Board,
    sub: &Substrate,
    max_w: f64,
    reach_max: f64,
    margin: f64,
    n_layers: usize,
) -> BoardStatic {
    let obs_boxes: Vec<Rect> = sub.shared.obs_polys.iter().map(|p| p.bbox()).collect();
    let bounds = board.bbox().grow(5.0);
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
                hole: o.kind == ObstacleKind::Hole,
            }
        })
        .collect();
    let near_edge: Vec<bool> = (0..cells.len())
        .into_par_iter()
        .map(|c| {
            let (r, half) = cells.cell_rect(c);
            let mid = (r.min + r.max) * 0.5;
            let d = board
                .outline
                .iter()
                .map(|o| nearest_on_poly(o, mid).0)
                .fold(f64::INFINITY, f64::min);
            d < board.rules.edge_clearance + max_w / 2.0 + margin + half + 0.01 || !board.inside(mid)
        })
        .collect();
    let edge_segs: Vec<(u32, Point, Point)> = board
        .outline
        .iter()
        .enumerate()
        .flat_map(|(oi, o)| o.edges().map(move |(a, b)| (oi as u32, a, b)))
        .collect();
    let edge_boxes: Vec<Rect> = edge_segs
        .iter()
        .map(|&(_, a, b)| {
            let mut r = Rect::empty();
            r.add(a);
            r.add(b);
            r
        })
        .collect();
    let edge_reach = board.rules.edge_clearance + max_w / 2.0 + margin + 0.01;
    let edge_cells = CellLists::new(bounds, 0.5, &edge_boxes, edge_reach, |_| true);
    // the rows of the even-odd test, with a little room at the borders of a row
    let row = |y: f64| axis_cell(y, cells.origin.y, cells.inv, cells.ny);
    let mut rows: Vec<Vec<(Point, Point)>> = vec![Vec::new(); cells.ny];
    for o in &board.outline {
        let n = o.pts.len();
        let mut j = n - 1;
        for i in 0..n {
            let (a, b) = (o.pts[i], o.pts[j]);
            for r in &mut rows[row(a.y.min(b.y) - 1e-6)..=row(a.y.max(b.y) + 1e-6)] {
                r.push((a, b));
            }
            j = i;
        }
    }
    let mut row_start = vec![0u32];
    let mut row_edges = Vec::new();
    for r in rows {
        row_edges.extend(r);
        row_start.push(row_edges.len() as u32);
    }
    BoardStatic {
        bounds,
        cells,
        layer_cells,
        near_obs,
        near_edge,
        edge_segs,
        edge_cells,
        row_start,
        row_edges,
    }
}

pub fn relax(board: &Board, sub: &Substrate, geo: &mut Geometry, cfg: &RelaxConfig) {
    relax_while(board, sub, geo, cfg, &|| true);
}

/// `relax`, as long as `go_on` says so before each round. False once it says no, and then `geo`
/// is left half way: for a mutant of the open end that is no longer needed.
pub fn relax_while(
    board: &Board,
    sub: &Substrate,
    geo: &mut Geometry,
    cfg: &RelaxConfig,
    go_on: &dyn Fn() -> bool,
) -> bool {
    let clr = |n: u32| board.class_of(n).clearance.max(board.rules.min_clearance);
    let t_start = std::time::Instant::now();
    // coarse points while the bands move a lot, fine points for the final clearance work
    let coarse = knobs::COARSE.get().unwrap_or((cfg.spacing * 2.0).max(0.1));
    let mut tracks: Vec<Vec<Point>> = geo.tracks.iter().map(|t| resample(&t.pts, coarse)).collect();
    let taps = tap_targets(&geo.tracks);
    let rails: Vec<[Option<Rail>; 2]> = if cfg.via_edge {
        let by_net = pin_rails(board);
        geo.tracks
            .iter()
            .zip(&tracks)
            .map(|(t, p)| match by_net.get(&t.net) {
                Some(r) => [rail_of(r, p.iter()), rail_of(r, p.iter().rev())],
                None => [None, None],
            })
            .collect()
    } else {
        Vec::new()
    };
    // custom rules between items of two nets are rare: most lookups below are cut short by this
    let item_max = board
        .item_clearance
        .values()
        .chain(board.net_hole_clearance.values())
        .copied()
        .fold(0.0, f64::max);
    // how far any push can reach: largest clearance plus the widest track
    let max_clr = board
        .classes
        .iter()
        .map(|c| c.clearance)
        .fold(board.rules.min_clearance, f64::max)
        .max(board.pair_clearance.values().copied().fold(0.0, f64::max))
        .max(board.obstacles.iter().map(|o| o.clearance).fold(0.0, f64::max))
        .max(board.rules.hole_clearance)
        .max(item_max);
    let max_w = geo.tracks.iter().map(|t| t.width).fold(0.0, f64::max);
    let n_layers = geo.tracks.iter().map(|t| t.layer as usize + 1).max().unwrap_or(0);
    let half_w = |t: &Track| if t.necks.is_empty() { t.width } else { t.width.max(t.neck_w) } / 2.0;
    // how much more than that a fitted width can give a piece, none before the widths are fitted
    let fitted_over = geo
        .tracks
        .iter()
        .flat_map(|t| t.fitted.iter().map(move |f| f.1 / 2.0 - half_w(t)))
        .fold(0.0, f64::max);
    // per net the largest clearance it can ask for against any other net
    let mut net_clr: Vec<f64> = (0..board.nets.len() as u32).map(clr).collect();
    for (&(a, b), &c) in &board.pair_clearance {
        net_clr[a as usize] = net_clr[a as usize].max(c);
        net_clr[b as usize] = net_clr[b as usize].max(c);
    }
    // per layer the split that makes the fewest grid cells to visit: a query looks at about
    // the square of its reach, weighted by the pieces it sweeps
    let far_track: Vec<bool> = {
        let reach = |ti: usize| half_w(&geo.tracks[ti]) + net_clr[geo.tracks[ti].net as usize];
        let mut limit = vec![f64::INFINITY; n_layers];
        for (l, limit) in limit.iter_mut().enumerate() {
            let mut by: Vec<(f64, f64, f64, f64)> = (0..geo.tracks.len())
                .filter(|&ti| geo.tracks[ti].layer as usize == l)
                .map(|ti| {
                    let t = &geo.tracks[ti];
                    (reach(ti), half_w(t), net_clr[t.net as usize], tracks[ti].len() as f64)
                })
                .collect();
            by.sort_unstable_by(|a, b| a.0.total_cmp(&b.0));
            let span = |part: &[(f64, f64, f64, f64)], cell: f64| {
                let hw = part.iter().map(|b| b.1).fold(0.0, f64::max);
                let c = part.iter().map(|b| b.2).fold(0.0, f64::max);
                let side = 2.0 * (hw + c + hw + 0.1) + cell;
                part.iter().map(|b| b.3).sum::<f64>() * side * side
            };
            let mut best = f64::INFINITY;
            for i in 0..by.len() {
                if i + 1 == by.len() || by[i + 1].0 > by[i].0 {
                    let cost = span(&by[..=i], 0.25) + span(&by[i + 1..], 1.0);
                    if cost < best {
                        best = cost;
                        *limit = by[i].0;
                    }
                }
            }
        }
        (0..geo.tracks.len())
            .map(|ti| reach(ti) > limit[geo.tracks[ti].layer as usize])
            .collect()
    };
    // every point query below looks at one cell only: the cell lists hold all obstacles within
    // the largest reach of any point in it, and the edge mask marks cells near the board edge
    let reach_max = max_w + max_clr + cfg.margin + 0.01;
    let st = board_static(board, sub, max_w, reach_max, cfg.margin, n_layers);
    let BoardStatic {
        bounds,
        cells,
        layer_cells,
        near_obs,
        near_edge,
        edge_segs,
        edge_cells,
        ..
    } = &*st;
    let bounds = *bounds;
    // vias as static obstacles: (centre, radius, net, clearance)
    let mut vias: Vec<(Point, f64, u32, f64)> = geo
        .vias
        .iter()
        .map(|v| (v.at, v.diameter / 2.0, v.net, clr(v.net)))
        .collect();
    let max_vr = vias.iter().map(|v| v.1).fold(0.0, f64::max);
    // a via pair is a capsule: its line turns with the tracks that end on it, its vias are as
    // wide as the circle around it less the line (round vias: no line, the circle)
    let mut halves: Vec<Point> = geo.vias.iter().map(|v| v.half).collect();
    let any_pair = geo.vias.iter().any(|v| v.is_pair());
    let pair_r: Vec<f64> = geo
        .vias
        .iter()
        .map(|v| {
            if v.is_pair() {
                v.diameter / 2.0 - v.half.len()
            } else {
                v.diameter / 2.0
            }
        })
        .collect();
    let max_drill = drills_max(geo);
    let edge_clr = board.rules.edge_clearance;
    let routing = board.routing_layers();
    let drills: Vec<f64> = geo.vias.iter().map(|v| v.drill).collect();
    let pair_drills: Vec<f64> = geo
        .vias
        .iter()
        .map(|v| {
            if v.is_pair() {
                v.drill - 2.0 * v.half.len()
            } else {
                v.drill
            }
        })
        .collect();
    // the planes each via reaches, and how near to their border it may come
    let via_planes: Vec<Vec<(&crate::board::Plane, f64)>> = geo
        .vias
        .iter()
        .map(|v| {
            board
                .planes
                .iter()
                .filter(|p| cfg.via_edge && p.net == v.net && p.covers(v.at))
                .map(|p| (p, p.edge_dist(v.at).min(v.diameter / 2.0)))
                .collect()
        })
        .collect();
    // track ends that sit on a via of their net follow the via
    let attach = |p: Point, net: u32, vias: &[(Point, f64, u32, f64)]| {
        vias.iter().position(|v| v.2 == net && v.0.dist(p) < 1e-6)
    };
    let ends: Vec<(Option<usize>, Option<usize>)> = tracks
        .iter()
        .zip(&geo.tracks)
        .map(|(p, t)| (attach(p[0], t.net, &vias), attach(*p.last().unwrap(), t.net, &vias)))
        .collect();
    // soft copper sleeps until something comes too close: it wakes up with the first contact and
    // takes its neighbours along, everything else stays as it is
    let mut awake: Vec<bool> = geo.tracks.iter().map(|t| !t.resting).collect();
    let mut via_awake: Vec<bool> = geo.vias.iter().map(|v| !v.resting).collect();
    let lazy = awake.iter().chain(&via_awake).any(|a| !a);
    let via_layer: Vec<Option<LayerId>> = geo.vias.iter().map(|v| v.layer).collect();
    // per via pair the tracks that end on it (and whether with their start)
    let mut via_legs: Vec<Vec<(usize, bool)>> = vec![Vec::new(); vias.len()];
    if any_pair {
        for (ti, (a, b)) in ends.iter().enumerate() {
            for (v, start) in [(a, true), (b, false)] {
                if let Some(v) = *v
                    && geo.vias[v].is_pair()
                {
                    via_legs[v].push((ti, start));
                }
            }
        }
    }
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
    // the points and vias at the begin of an annealing round, to find those that moved
    let (mut old_tracks, mut old_vias): (Vec<Vec<Point>>, Vec<Point>) = (Vec::new(), Vec::new());
    prof::add(Slot::RelaxSetup, t_start.elapsed().as_nanos() as u64);
    prof::add(Slot::Calls, 1);
    let mut grids: Vec<LayerPieces> = (0..n_layers).map(|_| LayerPieces::new(bounds)).collect();
    let mut via_at: Vec<Point> = Vec::with_capacity(vias.len());
    let mut via_grid = PointGrid::new(bounds, 1.0, &[]);
    let mut pieces: Vec<Piece> = Vec::new();
    // per track where its pieces begin in `pieces` and in the list of its tier
    let mut track_at: Vec<(usize, usize)> = Vec::new();
    let annealing = |it: usize| it * 4 >= cfg.iterations * 3;
    // sleeping copper, vias of one layer and fitted widths stay on the CPU
    #[cfg(feature = "gpu")]
    let mut on_gpu = (cfg.gpu
        && !lazy
        && !any_pair
        && via_layer.iter().all(Option::is_none)
        && geo.tracks.iter().all(|t| t.fitted.is_empty()))
    .then(|| {
        let half: Vec<f64> = geo.tracks.iter().map(half_w).collect();
        let clrs: Vec<f64> = (0..board.nets.len() as u32).map(clr).collect();
        gpu::GpuRelax::new(gpu::Setup {
            board,
            geo,
            cfg,
            origin: (bounds.min + bounds.max) * 0.5,
            cells: &cells,
            layer_cells: &layer_cells,
            near_obs: &near_obs,
            obs_polys: &sub.shared.obs_polys,
            near_edge: &near_edge,
            net_clr: &net_clr,
            half_w: &half,
            clr: &clrs,
            max_clr,
            max_vr,
            via_grid: &via_grid,
        })
    })
    .flatten();
    // the last quarter resolves clearances on the CPU: there tracks and vias have to answer each
    // other round by round, and f64 and the dirty cells count
    #[cfg(feature = "gpu")]
    let gpu_end = if on_gpu.is_some() {
        (cfg.iterations * 3 / 4 / 10 * 10).saturating_sub(1)
    } else {
        0
    };
    #[cfg(not(feature = "gpu"))]
    let gpu_end = 0;
    // during a batch the tracks do not answer, so a via moves once per batch, but a batch far
    let via_gain = 10.0;
    let starts_batch = |it: usize| it == 0 || it % 10 == 9;
    for it in 0..cfg.iterations {
        if !go_on() {
            return false;
        }
        let on_gpu_now = it < gpu_end;
        // anneal: the last quarter only resolves clearances
        let tension = if annealing(it) { 0.0 } else { cfg.tension };
        let t_round = std::time::Instant::now();
        let before = (annealing(it) && !on_gpu_now).then(|| {
            old_tracks.clone_from(&tracks);
            old_vias.clear();
            old_vias.extend(vias.iter().map(|v| v.0));
        });
        if it % 10 == 9 {
            tracks = tracks
                .par_iter()
                .enumerate()
                .map(|(ti, p)| {
                    if !awake[ti] {
                        return p.clone();
                    }
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
        let keep_pieces = on_gpu_now && !starts_batch(it);
        // from one resampling to the next the tracks keep their points: then only the pieces of
        // copper that can have moved are new, all keep their ids and places
        let in_place = !keep_pieces && it > gpu_end && it % 10 != 9;
        if !keep_pieces && !in_place {
            pieces.clear();
            track_at.clear();
            for g in grids.iter_mut() {
                for tier in [&mut g.fine, &mut g.far] {
                    tier.raw.clear();
                    tier.moved = true;
                }
            }
        }
        // asleep, a track only moves with the vias at its ends
        let can_move = |ti: usize| {
            let (a, b) = ends[ti];
            awake[ti] || a.is_some_and(|v| via_awake[v]) || b.is_some_and(|v| via_awake[v])
        };
        // pieces by their midpoints per layer; queries grow by the largest extent
        for (ti, p) in tracks.iter().enumerate().filter(|_| !keep_pieces) {
            if in_place && !can_move(ti) {
                continue;
            }
            let t = &geo.tracks[ti];
            let g = &mut grids[t.layer as usize];
            let (hw, nc) = (half_w(t), (net_clr[t.net as usize] as f32).next_up());
            let tier = if far_track[ti] { &mut g.far } else { &mut g.fine };
            tier.moved = true;
            if !in_place {
                track_at.push((pieces.len(), tier.raw.len()));
            }
            let (first, at) = track_at[ti];
            for (k, w) in p.windows(2).enumerate() {
                let id = (first + k) as u32;
                let piece = Piece {
                    a: w[0],
                    b: w[1],
                    track: ti as u32,
                };
                let hl = (w[1] - w[0]).len2().sqrt() * 0.5 * SURE;
                let near = NearPiece {
                    mid: (w[0] + w[1]) * 0.5,
                    extent: ((hw + hl) as f32).next_up(),
                    clr: nc,
                    net: t.net,
                    id,
                };
                if in_place {
                    pieces[id as usize] = piece;
                    tier.raw[at + k] = near;
                } else {
                    pieces.push(piece);
                    tier.raw.push(near);
                }
            }
        }
        if !keep_pieces {
            grids.par_iter_mut().for_each(LayerPieces::rebuild);
        }
        let (pieces, grids) = (&pieces, &grids);
        via_at.clear();
        via_at.extend(vias.iter().map(|v| v.0));
        via_grid.rebuild(&via_at);
        let via_grid = &via_grid;
        if lazy {
            wake(
                board,
                &geo.tracks,
                &tracks,
                pieces,
                grids,
                (&vias, &via_layer, via_grid),
                &ends,
                (&mut awake, &mut via_awake),
                (max_clr, max_vr),
                (&net_clr, fitted_over),
            );
            // a track that ends on a moving track goes with it
            for (ti, others) in &taps {
                if others.iter().any(|&o| awake[o]) {
                    awake[*ti] = true;
                }
            }
        }
        let (awake, via_awake) = (&awake, &via_awake);
        prof::add(Slot::RelaxPieces, t_round.elapsed().as_nanos() as u64);
        let t_vias = std::time::Instant::now();
        // vias move as well: pushed by obstacles, other vias and tracks of other nets
        let moved: Vec<Point> = vias
            .par_iter()
            .enumerate()
            .map_init(Sweep::default, |sweep: &mut Sweep, (vi, &(vc, vr, vn, vclr))| {
                if !via_awake[vi] || (on_gpu_now && !starts_batch(it)) {
                    return vc;
                }
                let half = halves[vi];
                let pair = half != Point::default();
                // the radius of the copper around the centre (a pair: around its line)
                let cr = if pair { pair_r[vi] } else { vr };
                let mut push = pt(0.0, 0.0);
                for oi in sub.shared.obs_index.query_point(vc, vr + 1.0) {
                    let o = &board.obstacles[oi as usize];
                    let keeps_vias = matches!(o.kind, ObstacleKind::Keepout { vias: true, .. });
                    if !o.layers.intersects(routing) && o.hole.is_none() && !keeps_vias {
                        continue;
                    }
                    if via_layer[vi].is_some_and(|l| !o.layers.has(l)) {
                        continue;
                    }
                    if o.net == Some(vn) && o.kind != ObstacleKind::Hole {
                        continue;
                    }
                    let need = match o.kind {
                        ObstacleKind::Keepout {
                            vias: no_vias, tracks, ..
                        } => {
                            // a via outside a track keepout still needs room for the ends of
                            // its tracks on that layer (they end at the via centre)
                            let ends = via_tracks[vi]
                                .iter()
                                .filter(|(l, _)| tracks && o.layers.has(*l))
                                .map(|(_, w)| w / 2.0)
                                .fold(f64::NEG_INFINITY, f64::max);
                            let r = if no_vias { cr.max(ends) } else { ends };
                            if r == f64::NEG_INFINITY {
                                continue;
                            }
                            crate::board::KEEPOUT_GAP + r
                        }
                        ObstacleKind::Hole => board.hole_gap(vn) + cr,
                        _ => o.clearance.max(vclr).max(board.item_gap(vn, o.net)) + cr,
                    } + cfg.margin;
                    let poly = &sub.shared.obs_polys[oi as usize];
                    let (p, d, q, inside) = if pair {
                        pair_near_poly(poly, vc, half, false)
                    } else {
                        let inside = poly.contains(vc);
                        let (d, q) = nearest_on_poly(poly, vc);
                        (vc, d, q, inside)
                    };
                    let d = if inside { -d } else { d };
                    if d < need {
                        let dir = if inside { (q - p).norm() } else { (p - q).norm() };
                        push = push + dir * (need - d);
                    }
                }
                if cfg.via_edge {
                    let inside_board = board.inside(vc);
                    for o in &board.outline {
                        let (p, d, q, inside_board) = if pair {
                            pair_near_poly(o, vc, half, true)
                        } else {
                            let (d, q) = nearest_on_poly(o, vc);
                            (vc, d, q, inside_board)
                        };
                        let need = edge_clr + cr + cfg.margin;
                        if d < need || !inside_board {
                            let dir = if inside_board { (p - q).norm() } else { (q - p).norm() };
                            let d = if inside_board { d } else { -d };
                            push = push + dir * (need - d);
                        }
                    }
                }
                // sums run in id order: collect the few candidates first
                let Sweep { near, vias: hits, .. } = sweep;
                hits.clear();
                let via_reach = (vr + max_vr + max_clr)
                    .max(drills[vi] / 2.0 + max_drill / 2.0 + board.rules.hole_to_hole)
                    + cfg.margin
                    + 0.01;
                via_grid.for_each_near(vc, via_reach, |vj| {
                    if vj as usize != vi {
                        hits.push(vj);
                    }
                });
                hits.sort_unstable();
                for &vj in hits.iter() {
                    let vj = vj as usize;
                    let (c2, r2, n2, clr2) = vias[vj];
                    if matches!((via_layer[vi], via_layer[vj]), (Some(a), Some(b)) if a != b) {
                        continue;
                    }
                    let (half2, pair2) = (halves[vj], halves[vj] != Point::default());
                    let (dri, drj) = if pair || pair2 {
                        (pair_drills[vi], pair_drills[vj])
                    } else {
                        (drills[vi], drills[vj])
                    };
                    let holes = if drills[vi] == 0.0 || drills[vj] == 0.0 {
                        0.0
                    } else {
                        dri / 2.0 + drj / 2.0 + board.rules.hole_to_hole + cfg.margin
                    };
                    let r2 = if pair2 { pair_r[vj] } else { r2 };
                    let need = if n2 == vn {
                        holes
                    } else {
                        (cr + r2 + vclr.max(clr2).max(board.item_gap(vn, Some(n2))) + cfg.margin).max(holes)
                    };
                    let dv = if pair || pair2 {
                        let (p, q) = seg_seg_nearest(vc - half, vc + half, c2 - half2, c2 + half2);
                        p - q
                    } else {
                        vc - c2
                    };
                    if surely_outside(dv.x, dv.y, need) {
                        continue;
                    }
                    let d = dv.len();
                    if d < need {
                        let dir = if d > 1e-9 {
                            dv * (1.0 / d)
                        } else {
                            pt(1.0, 0.0).rotate(vi as f64)
                        };
                        push = push + dir * ((need - d) * 0.5);
                    }
                }
                // candidates of all layers, back in the order of one grid over all of them
                near.clear();
                for (l, g) in grids.iter().enumerate() {
                    if via_layer[vi].is_some_and(|x| x as usize != l) {
                        continue;
                    }
                    for tier in [&g.fine, &g.far] {
                        let radius = vr + vclr.max(tier.max_clr).max(item_max) + cfg.margin + tier.max_extent;
                        tier.for_each_keyed(vc, radius, |r, key| {
                            let dm = vc - r.mid;
                            let lim = vr + max_of(max_of(vclr, r.clr as f64), item_max) + cfg.margin + r.extent as f64;
                            if r.net != vn && !surely_outside(dm.x, dm.y, lim) {
                                near.push(key);
                            }
                        });
                    }
                }
                near.sort_unstable();
                for &key in near.iter() {
                    let pc = &pieces[key as u32 as usize];
                    let o = &geo.tracks[pc.track as usize];
                    let need = cr
                        + o.piece_width(pc.a, pc.b) / 2.0
                        + vclr.max(clr(o.net)).max(board.item_gap(vn, Some(o.net)))
                        + cfg.margin;
                    let ab = pc.b - pc.a;
                    let v = if pair {
                        let (p, q) = seg_seg_nearest(vc - half, vc + half, pc.a, pc.b);
                        p - q
                    } else {
                        let l2 = ab.len2();
                        let tt = if l2 > 0.0 {
                            ((vc - pc.a).dot(ab) / l2).clamp(0.0, 1.0)
                        } else {
                            0.0
                        };
                        let q = pc.a + ab * tt;
                        vc - q
                    };
                    if surely_outside(v.x, v.y, need) {
                        continue;
                    }
                    let d = v.len();
                    if d < need {
                        let dir = if d > 1e-9 { v * (1.0 / d) } else { ab.perp().norm() };
                        push = push + dir * ((need - d) * 0.5);
                    }
                }
                let mv = if surely_within(push.x, push.y, cfg.max_step) {
                    push
                } else {
                    let l = push.len();
                    if l > cfg.max_step {
                        push * (cfg.max_step / l)
                    } else {
                        push
                    }
                };
                let to = vc + mv * if on_gpu_now { via_gain } else { 1.0 };
                // the plane copper, cut by the higher zones of other nets
                let stays = |&(pl, floor): &(&crate::board::Plane, f64)| pl.holds(to, floor);
                if via_planes[vi].is_empty() || via_planes[vi].iter().any(stays) {
                    to
                } else {
                    vc
                }
            })
            .collect();
        prof::add(Slot::RelaxVias, t_vias.elapsed().as_nanos() as u64);
        let t_tracks = std::time::Instant::now();
        // the GPU runs from one resampling to the next, the tracks stay as they are until then
        #[cfg(feature = "gpu")]
        if let Some(g) = on_gpu.as_mut().filter(|_| on_gpu_now) {
            if starts_batch(it) {
                let end = (if it == 0 { 9 } else { it + 10 }).min(gpu_end);
                g.start(
                    &gpu::Round {
                        tracks: &tracks,
                        grids,
                        pieces,
                        vias: &vias,
                        via_grid,
                        tension,
                        rounds: (end - it) as u32,
                    },
                    geo,
                );
            }
        }
        #[cfg(feature = "gpu")]
        let gpu_moves = on_gpu
            .as_mut()
            .filter(|_| on_gpu_now && it % 10 == 8)
            .map(|g| g.finish());
        #[cfg(not(feature = "gpu"))]
        let gpu_moves: Option<Vec<[f32; 2]>> = None;
        // each track reads only its own points, the others it sees through the pieces
        tracks
            .par_iter_mut()
            .enumerate()
            .filter(|(ti, _)| awake[*ti] && !on_gpu_now)
            .for_each(|(ti, track)| {
                let t = &geo.tracks[ti];
                let (c, net, nc) = (clr(t.net), t.net, net_clr[t.net as usize]);
                let (grid, obs_cells) = (&grids[t.layer as usize], &layer_cells[t.layer as usize]);
                let pts: &[Point] = track;
                let n = pts.len();
                // points in parallel too: a few long tracks would otherwise keep one core busy
                let inner: Vec<Point> = (1..n.saturating_sub(1))
                    .into_par_iter()
                    .with_min_len(32)
                    .map_init(Sweep::default, |sweep: &mut Sweep, i| {
                        let p = pts[i];
                        let cell = cells.cell_of(p);
                        if dirty.as_ref().is_some_and(|d| !d[cell]) {
                            return p;
                        }
                        let w = t.width_at(p);
                        // tension
                        let mid = (pts[i - 1] + pts[i + 1]) * 0.5;
                        let mut mv = (mid - p) * tension;
                        let mut push = pt(0.0, 0.0);
                        // static obstacles
                        for &oi in obs_cells.get(cell) {
                            let o = &near_obs[oi as usize];
                            if o.own_net == net {
                                continue;
                            }
                            let need = if o.floor {
                                max_of(max_of(o.clr, c), board.item_gap(net, Some(o.own_net)))
                            } else if o.hole {
                                board.hole_gap(net)
                            } else {
                                o.clr
                            } + w / 2.0
                                + cfg.margin;
                            // cheap reject on the bounding box before the polygon distance
                            let dx = max_of(max_of(0.0, o.bbox.min.x - p.x), p.x - o.bbox.max.x);
                            let dy = max_of(max_of(0.0, o.bbox.min.y - p.y), p.y - o.bbox.max.y);
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
                        let inside_board = !near_edge[cell] || st.inside(cell / cells.nx, p);
                        let need = edge_clr + w / 2.0 + cfg.margin;
                        if near_edge[cell] && inside_board && w <= max_w {
                            // the nearest point of each outline from the edges near the cell:
                            // an outline without such an edge is too far to push
                            let near = edge_cells.get(cell);
                            let mut k = 0;
                            while k < near.len() {
                                let o = edge_segs[near[k] as usize].0;
                                let mut best = (f64::INFINITY, p);
                                while k < near.len() && edge_segs[near[k] as usize].0 == o {
                                    let (_, a, b) = edge_segs[near[k] as usize];
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
                                    k += 1;
                                }
                                let (d, q) = (best.0.sqrt(), best.1);
                                if d < need {
                                    push = push + (p - q).norm() * (need - d);
                                }
                            }
                        } else if near_edge[cell] {
                            for o in &board.outline {
                                let (d, q) = nearest_on_poly(o, p);
                                // a point outside the board always goes back in
                                if d < need || !inside_board {
                                    let dir = if inside_board { (p - q).norm() } else { (q - p).norm() };
                                    let d = if inside_board { d } else { -d };
                                    push = push + dir * (need - d);
                                }
                            }
                        }
                        // vias of other nets
                        let Sweep {
                            anchor,
                            list,
                            merge,
                            via_list,
                            near,
                            vias: hits,
                        } = sweep;
                        // the pieces and vias near one point serve the next points along the
                        // track too: within the skin of the anchor every candidate of a point is
                        // in the lists, and the filters below pick exactly those of the point
                        let sweep_again = (p - *anchor).len2() >= SWEEP_SKIN * SWEEP_SKIN;
                        if sweep_again {
                            via_list.clear();
                            let radius = half_w(t) + max_clr + max_vr + cfg.margin + 0.01 + SWEEP_SKIN;
                            via_grid.for_each_near(p, radius, |vi| {
                                let (vc, vr, vn, vclr) = vias[vi as usize];
                                if vn != net && via_layer[vi as usize].is_none_or(|l| l == t.layer) {
                                    via_list.push(SweptVia {
                                        at: vc,
                                        reach: vr + max_of(max_of(vclr, c), item_max),
                                        id: vi,
                                    });
                                }
                            });
                        }
                        hits.clear();
                        let wm = w / 2.0 + cfg.margin;
                        for v in via_list.iter() {
                            let dv = p - v.at;
                            if !surely_outside(dv.x, dv.y, v.reach + wm) {
                                hits.push(v.id);
                            }
                        }
                        hits.sort_unstable();
                        for &vi in hits.iter() {
                            let (vc, vr, vn, vclr) = vias[vi as usize];
                            let half = halves[vi as usize];
                            let (vr, v) = if half != Point::default() {
                                (pair_r[vi as usize], point_seg_offset(p, vc - half, vc + half))
                            } else {
                                (vr, p - vc)
                            };
                            let need =
                                vr + max_of(max_of(vclr, c), board.item_gap(net, Some(vn))) + w / 2.0 + cfg.margin;
                            if surely_outside(v.x, v.y, need) {
                                continue;
                            }
                            let d = v.len();
                            if d < need {
                                let dir = if d > 0.0 { v * (1.0 / d) } else { v };
                                push = push + dir * ((need - d) * 0.5);
                            }
                        }
                        // other tracks (they push back from their side, so take half)
                        if sweep_again {
                            *anchor = p;
                            list.clear();
                            let base = half_w(t) + cfg.margin + SWEEP_SKIN + 1e-6;
                            let mut far_at = 0;
                            for tier in [&grid.fine, &grid.far] {
                                far_at = list.len();
                                let radius = base + nc.max(tier.max_clr) + tier.max_extent;
                                tier.for_each_keyed(p, radius, |r, key| {
                                    let dm = p - r.mid;
                                    let lim = base + max_of(nc, r.clr as f64) + r.extent as f64;
                                    if r.net != net && !surely_outside(dm.x, dm.y, lim) {
                                        list.push(Swept {
                                            mid: r.mid,
                                            extent: r.extent,
                                            clr: r.clr,
                                            key,
                                        });
                                    }
                                });
                            }
                            // the fine grid hands out its pieces in key order already (rows of
                            // cells, ids within a cell); the far ones go in between
                            if !list[..far_at].is_sorted_by_key(|r| r.key) {
                                list[..far_at].sort_unstable_by_key(|r| r.key);
                            }
                            if far_at < list.len() {
                                list[far_at..].sort_unstable_by_key(|r| r.key);
                                merge.clear();
                                let (a, b) = list.split_at(far_at);
                                let (mut i, mut j) = (0, 0);
                                while i < a.len() && j < b.len() {
                                    if a[i].key < b[j].key {
                                        merge.push(a[i]);
                                        i += 1;
                                    } else {
                                        merge.push(b[j]);
                                        j += 1;
                                    }
                                }
                                merge.extend_from_slice(&a[i..]);
                                merge.extend_from_slice(&b[j..]);
                                std::mem::swap(list, merge);
                            }
                        }
                        near.clear();
                        let base = w / 2.0 + cfg.margin;
                        for r in list.iter() {
                            let dm = p - r.mid;
                            let lim = base + max_of(nc, r.clr as f64) + r.extent as f64;
                            if !surely_outside(dm.x, dm.y, lim) {
                                near.push(r.key);
                            }
                        }
                        for &key in near.iter() {
                            let pc = &pieces[key as u32 as usize];
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
                            let v = p - q;
                            if surely_outside(v.x, v.y, need) {
                                continue;
                            }
                            let d = v.len();
                            if d < need {
                                let dir = if d > 1e-9 { v * (1.0 / d) } else { ab.perp().norm() };
                                push = push + dir * ((need - d) * 0.5);
                            }
                        }
                        // constraints win over tension: drop the tension part that points into the push
                        if push.len2() > 0.0 {
                            let pn = push.norm();
                            let into = mv.dot(pn);
                            if into < 0.0 {
                                mv = mv - pn * into;
                            }
                            mv = mv + push;
                        }
                        // most points move less than a step: then the length needs no root
                        if !surely_within(mv.x, mv.y, cfg.max_step) {
                            let l = mv.len();
                            if l > cfg.max_step {
                                mv = mv * (cfg.max_step / l);
                            }
                        }
                        p + mv
                    })
                    .collect();
                track[1..1 + inner.len()].copy_from_slice(&inner);
            });
        for (ti, ends) in rails.iter().enumerate().filter(|(ti, _)| awake[*ti]) {
            if let Some(r) = &ends[0] {
                follow(r, &mut tracks[ti], false);
            }
            if let Some(r) = &ends[1] {
                follow(r, &mut tracks[ti], true);
            }
        }
        // the end of a tap slides along the track it ends on, to where it has the shortest way
        for (ti, others) in taps.iter().filter(|(ti, _)| awake[*ti]) {
            let n = tracks[*ti].len();
            if n >= 2
                && let Some(q) = nearest_on_tracks(&tracks, others, tracks[*ti][n - 2])
            {
                tracks[*ti][n - 1] = q;
            }
        }
        if let Some(moves) = gpu_moves {
            let mut at = 0;
            for p in &mut tracks {
                for (q, m) in p.iter_mut().zip(&moves[at..]) {
                    *q = pt(q.x + m[0] as f64, q.y + m[1] as f64);
                }
                at += p.len();
            }
        }
        prof::add(Slot::RelaxTracks, t_tracks.elapsed().as_nanos() as u64);
        prof::add(Slot::Rounds, 1);
        let t_dirty = std::time::Instant::now();
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
        // via pairs turn with their tracks
        let mut turned: Vec<usize> = Vec::new();
        for (vi, legs) in via_legs.iter().enumerate() {
            if let [(t1, s1), (t2, s2)] = legs[..]
                && let (Some(e1), Some(e2)) = (
                    crate::realize::leaving(&tracks[t1], s1),
                    crate::realize::leaving(&tracks[t2], s2),
                )
            {
                let h = halves[vi].len();
                let new = crate::realize::pair_half([e1, e2], h, halves[vi]);
                if (new - halves[vi]).len2() > 1e-18 {
                    halves[vi] = new;
                    turned.push(vi);
                }
            }
        }
        dirty = before.map(|()| {
            let mut d = vec![false; cells.len()];
            // the points that moved come in runs along their tracks: one mark per cell will do
            let mut centre = vec![false; cells.len()];
            let k = ((reach_max + cfg.max_step) / cells.cell).ceil() as isize + 1;
            let mut mark = |p: Point| {
                let c = cells.cell_of(p);
                if std::mem::replace(&mut centre[c], true) {
                    return;
                }
                let c = c as isize;
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
                        if p != q && longer(*p - *q, 1e-9) {
                            mark(*p);
                            mark(*q);
                        }
                    }
                }
            }
            for (p, v) in old_vias.iter().zip(&vias) {
                if *p != v.0 && longer(*p - v.0, 1e-9) {
                    mark(*p);
                    mark(v.0);
                }
            }
            for &vi in &turned {
                mark(vias[vi].0 - halves[vi]);
                mark(vias[vi].0 + halves[vi]);
            }
            d
        });
        prof::add(Slot::RelaxDirty, t_dirty.elapsed().as_nanos() as u64);
    }
    for (i, (v, (p, ..))) in geo.vias.iter_mut().zip(&vias).enumerate() {
        v.at = *p;
        v.resting = !via_awake[i];
        v.half = halves[i];
    }
    let t_taut = std::time::Instant::now();
    let tracks: Vec<(Vec<Point>, Vec<Elem>)> = if cfg.taut {
        prof::time(Slot::Taut, || taut(board, sub, geo, tracks, cfg, &awake, &rails))
    } else {
        tracks
            .into_iter()
            .map(|p| {
                let elems = p.windows(2).map(|w| Elem::Line(w[0], w[1])).collect();
                (p, elems)
            })
            .collect()
    };
    if knobs::TIMING.on() {
        eprintln!(
            "relax rounds {:.2} s, taut {:.2} s",
            t_taut.duration_since(t_start).as_secs_f64(),
            t_taut.elapsed().as_secs_f64()
        );
    }
    // the tracks they end on have their last shape now
    let mut tracks = tracks;
    let chains: Vec<Vec<Point>> = tracks.iter().map(|t| t.0.clone()).collect();
    for (ti, others) in &taps {
        let (pts, elems) = &mut tracks[*ti];
        let Some(&end) = pts.last() else { continue };
        if let Some(q) = nearest_on_tracks(&chains, others, end).filter(|q| q.dist(end) > 1e-9) {
            *pts.last_mut().unwrap() = q;
            match elems.last_mut() {
                Some(Elem::Line(_, b)) => *b = q,
                _ => elems.push(Elem::Line(end, q)),
            }
        }
    }
    for (ti, (t, (pts, elems))) in geo.tracks.iter_mut().zip(tracks).enumerate() {
        *t = Track {
            elems,
            pts,
            width: t.width,
            layer: t.layer,
            net: t.net,
            necks: std::mem::take(&mut t.necks),
            neck_w: t.neck_w,
            origin: t.origin,
            resting: !awake[ti],
            fitted: std::mem::take(&mut t.fitted),
            pattern: t.pattern,
            seg: t.seg,
            tap: t.tap,
        };
    }
    if any_pair {
        crate::realize::turn_pairs(geo);
    }
    true
}

/// The point of a via pair's line (c - half to c + half) nearest to a polygon, its distance, the
/// nearest point of the polygon and whether the point lies inside: a few rounds of nearest
/// points both ways from the middle and the ends of the line. The free side is outside the
/// polygon, or inside with `free_inside` (the board outline): the line point the least far
/// into the other side or the nearest to it wins.
fn pair_near_poly(poly: &crate::geom::Polygon, c: Point, half: Point, free_inside: bool) -> (Point, f64, Point, bool) {
    let (a, b) = (c - half, c + half);
    let mut best: Option<(f64, (Point, f64, Point, bool))> = None;
    for start in [c, a, b] {
        let mut p = start;
        for _ in 0..3 {
            let inside = poly.contains(p);
            let (d, q) = nearest_on_poly(poly, p);
            let free = inside == free_inside;
            let signed = if free { d } else { -d };
            if best.is_none_or(|(s, _)| signed < s) {
                best = Some((signed, (p, d, q, inside)));
            }
            if !free {
                break;
            }
            let np = q - point_seg_offset(q, a, b);
            if np.dist(p) < 1e-9 {
                break;
            }
            p = np;
        }
    }
    best.unwrap().1
}

/// Wake the sleeping copper that the waking copper violates. A sleeping track that only has
/// less than the relaxation's margin to spare stays asleep, or the wave would never end.
#[allow(clippy::too_many_arguments)]
fn wake(
    board: &Board,
    infos: &[Track],
    tracks: &[Vec<Point>],
    pieces: &[Piece],
    grids: &[LayerPieces],
    (vias, via_layer, via_grid): (&[(Point, f64, u32, f64)], &[Option<LayerId>], &PointGrid),
    ends: &[(Option<usize>, Option<usize>)],
    (awake, via_awake): (&mut [bool], &mut [bool]),
    (max_clr, max_vr): (f64, f64),
    (net_clr, fitted_over): (&[f64], f64),
) {
    let clr = |n: u32| board.class_of(n).clearance.max(board.rules.min_clearance);
    // within `need`, from the offset to the nearest point; the root only where the squares
    // cannot tell
    let within = |v: Point, need: f64| !surely_outside(v.x, v.y, need) && v.len() < need;
    let mut woken: Vec<u32> = Vec::new();
    let mut woken_vias: Vec<u32> = Vec::new();
    for (ti, pts) in tracks.iter().enumerate() {
        if !awake[ti] {
            continue;
        }
        let t = &infos[ti];
        let c = clr(t.net);
        let nc = net_clr[t.net as usize];
        let hw = t.width.max(t.neck_w) / 2.0;
        let g = &grids[t.layer as usize];
        for &p in pts {
            let wp = t.width_at(p) / 2.0;
            for tier in [&g.fine, &g.far] {
                let radius = hw + max_clr + tier.max_extent;
                tier.for_each_near(p, radius, |r| {
                    // no part of a piece comes nearer than its bounds allow
                    let dm = p - r.mid;
                    let reach = wp + max_of(nc, r.clr as f64) + r.extent as f64 + fitted_over;
                    if r.net == t.net || surely_outside(dm.x, dm.y, reach) {
                        return;
                    }
                    let pc = &pieces[r.id as usize];
                    if awake[pc.track as usize] {
                        return;
                    }
                    let o = &infos[pc.track as usize];
                    let need = wp + o.piece_width(pc.a, pc.b) / 2.0 + board.track_clearance(t.net, o.net);
                    if within(point_seg_offset(p, pc.a, pc.b), need) {
                        woken.push(pc.track);
                    }
                });
            }
            via_grid.for_each_near(p, hw + max_clr + max_vr, |vi| {
                let (vc, vr, vn, vclr) = vias[vi as usize];
                if !via_awake[vi as usize]
                    && vn != t.net
                    && via_layer[vi as usize].is_none_or(|l| l == t.layer)
                    && within(p - vc, vr + vclr.max(c) + wp)
                {
                    woken_vias.push(vi);
                }
            });
        }
    }
    for (vi, &(vc, vr, vn, vclr)) in vias.iter().enumerate() {
        if !via_awake[vi] {
            continue;
        }
        for (l, g) in grids.iter().enumerate() {
            if via_layer[vi].is_some_and(|x| x as usize != l) {
                continue;
            }
            for tier in [&g.fine, &g.far] {
                tier.for_each_near(vc, vr + max_clr + tier.max_extent, |r| {
                    let dm = vc - r.mid;
                    let reach = vr + max_of(vclr, r.clr as f64) + r.extent as f64 + fitted_over;
                    if r.net == vn || surely_outside(dm.x, dm.y, reach) {
                        return;
                    }
                    let pc = &pieces[r.id as usize];
                    if awake[pc.track as usize] {
                        return;
                    }
                    let o = &infos[pc.track as usize];
                    let need = vr + o.piece_width(pc.a, pc.b) / 2.0 + vclr.max(clr(o.net));
                    if within(point_seg_offset(vc, pc.a, pc.b), need) {
                        woken.push(pc.track);
                    }
                });
            }
        }
        via_grid.for_each_near(vc, vr + max_vr + max_clr, |vj| {
            let (c2, r2, n2, clr2) = vias[vj as usize];
            if !via_awake[vj as usize]
                && n2 != vn
                && !matches!((via_layer[vi], via_layer[vj as usize]), (Some(a), Some(b)) if a != b)
                && within(vc - c2, vr + r2 + vclr.max(clr2))
            {
                woken_vias.push(vj);
            }
        });
    }
    for t in woken {
        awake[t as usize] = true;
    }
    if !woken_vias.is_empty() {
        for &v in &woken_vias {
            via_awake[v as usize] = true;
        }
        for (ti, (a, b)) in ends.iter().enumerate() {
            if a.is_some_and(|v| via_awake[v]) || b.is_some_and(|v| via_awake[v]) {
                awake[ti] = true;
            }
        }
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
    awake: &[bool],
    rails: &[[Option<Rail>; 2]],
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
    // (centre, radius, net, half line of a pair)
    let routing_vias: Vec<(Point, f64, u32, Point)> = geo
        .vias
        .iter()
        .map(|v| {
            let r = if v.is_pair() {
                v.diameter / 2.0 - v.half.len()
            } else {
                v.diameter / 2.0
            };
            (v.at, r, v.net, v.half)
        })
        .collect();
    let m = cfg.margin * 0.5;
    for ti in 0..tracks.len() {
        let t = &geo.tracks[ti];
        if !awake[ti] {
            result.push((t.pts.clone(), t.elems.clone()));
            continue;
        }
        let (net, layer, c) = (t.net, t.layer, clr(t.net));
        // under a minimum width rule a line that leaves its neck zone is cut where its full
        // width would reach into the zone, and keeps that width outside (`realize::zone_cuts`)
        let ruled = board.class_of(net).min_width.is_some() && !t.necks.is_empty();
        let clear_piece = |a: Point, b: Point, pieces: &[(Point, Point, u32)], idx: &GridIndex| -> bool {
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
                    ObstacleKind::Hole => board.hole_gap(net),
                    _ => o.clearance.max(c).max(board.item_gap(net, o.net)),
                } + w / 2.0
                    + m;
                if sub.shared.obs_polys[oi as usize].dist_to_seg(a, b) < need {
                    return false;
                }
            }
            // only what lies within reach of the box of a-b can be too close
            let need = board.rules.edge_clearance + w / 2.0 + m;
            let edge_box = r.grow(need + 1e-9);
            for o in &board.outline {
                let near = |(p, q): &(Point, Point)| {
                    p.x.max(q.x) >= edge_box.min.x
                        && p.x.min(q.x) <= edge_box.max.x
                        && p.y.max(q.y) >= edge_box.min.y
                        && p.y.min(q.y) <= edge_box.max.y
                };
                if o.edges()
                    .filter(near)
                    .any(|(p, q)| crate::geom::seg_seg_dist(a, b, p, q) < need)
                {
                    return false;
                }
            }
            for &(vc, vr, vn, half) in &routing_vias {
                if vn == net {
                    continue;
                }
                let need = vr + c.max(clr(vn)).max(board.item_gap(net, Some(vn))) + w / 2.0 + m;
                let pair = half != Point::default();
                let reach = need + 1e-9 + if pair { half.len() } else { 0.0 };
                if vc.x >= r.min.x - reach
                    && vc.x <= r.max.x + reach
                    && vc.y >= r.min.y - reach
                    && vc.y <= r.max.y + reach
                    && if pair {
                        seg_seg_dist(vc - half, vc + half, a, b) < need
                    } else {
                        point_seg_dist(vc, a, b) < need
                    }
                {
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
        let clear = |a: Point, b: Point, pieces: &[(Point, Point, u32)], idx: &GridIndex| -> bool {
            if !ruled {
                return clear_piece(a, b, pieces, idx);
            }
            let mut from = a;
            for to in crate::realize::zone_cuts(a, b, &t.necks, t.width)
                .into_iter()
                .chain([b])
            {
                if !clear_piece(from, to, pieces, idx) {
                    return false;
                }
                from = to;
            }
            true
        };
        let cut = |p: Vec<Point>| {
            if ruled {
                crate::realize::cut_at_zones(&p, &t.necks, t.width)
            } else {
                p
            }
        };
        let pts = &tracks[ti];
        let n = pts.len();
        if n <= 2 {
            let p = cut(simplify(pts, 0.0005));
            result.push((p.clone(), p.windows(2).map(|w| Elem::Line(w[0], w[1])).collect()));
            continue;
        }
        // a shortcut must not leave a rail before its end
        let mut stops: Vec<usize> = Vec::new();
        if let Some(ends) = rails.get(ti) {
            if let Some(r) = &ends[0] {
                stops.push(pts.iter().take_while(|p| r.holds(**p)).count().saturating_sub(1));
            }
            if let Some(r) = &ends[1] {
                stops.push(n - pts.iter().rev().take_while(|p| r.holds(**p)).count().max(1));
            }
            stops.sort_unstable();
        }
        let mut out = vec![pts[0]];
        let mut i = 0;
        while i < n - 1 {
            let n = stops.iter().find(|&&k| k > i).map_or(n, |&k| k + 1);
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
        let out = cut(simplify(&out, 0.0005));
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

/// Uniform grid where each cell lists the boxes within `reach` of it, so a point query is a
/// single slice lookup without deduplication.
struct CellLists {
    origin: Point,
    cell: f64,
    inv: f64,
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
        let f = |v: f64, o: f64, n: usize| axis_cell(v, o, 1.0 / cell, n);
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
            inv: 1.0 / cell,
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
        axis_cell(p.y, self.origin.y, self.inv, self.ny) * self.nx + axis_cell(p.x, self.origin.x, self.inv, self.nx)
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

/// Tracks stay narrow only where they must: a piece inside a neck zone takes the full width if
/// that keeps all clearances to obstacles, vias and other tracks. The zones shrink to the pieces
/// that stay narrow. A line that leaves a zone is cut as well, so that it is wide where it is
/// out of the zone; under a minimum width rule it is cut where the full width would reach into
/// the zone, and the piece outside keeps the width of the track ([`crate::realize::zone_cuts`]).
/// Only for the finished geometry: the track shapes do not change.
pub fn widen_necks(board: &Board, sub: &Substrate, geo: &mut Geometry) {
    let clr = |n: u32| board.class_of(n).clearance.max(board.rules.min_clearance);
    let bounds = board.bbox().grow(5.0);
    // a straight piece has one width: long ones are cut, so that one tight spot stays local
    for t in geo.tracks.iter_mut().filter(|t| !t.necks.is_empty()) {
        let ruled = board.class_of(t.net).min_width.is_some();
        let mut elems = Vec::with_capacity(t.elems.len());
        for e in &t.elems {
            match *e {
                Elem::Line(a, b) if ruled && !crate::realize::zone_cuts(a, b, &t.necks, t.width).is_empty() => {
                    let mut pts = vec![a];
                    pts.extend(crate::realize::zone_cuts(a, b, &t.necks, t.width));
                    pts.push(b);
                    elems.extend(pts.windows(2).map(|w| Elem::Line(w[0], w[1])));
                }
                e => elems.push(e),
            }
        }
        let mut cut = Vec::with_capacity(elems.len());
        for e in elems {
            match e {
                Elem::Line(a, b) if a.dist(b) > 0.5 && t.necks.iter().any(|r| r.touches_seg(a, b)) => {
                    let n = (a.dist(b) / 0.3).ceil() as usize;
                    let at = |i: usize| a + (b - a) * (i as f64 / n as f64);
                    cut.extend((0..n).map(|i| Elem::Line(at(i), at(i + 1))));
                    // all of it was narrow so far, and stays so until it is found to fit
                    for i in 0..n {
                        let mut r = Rect::empty();
                        r.add((at(i) + at(i + 1)) * 0.5);
                        t.fitted.push((r.grow(1e-6), t.neck_w));
                    }
                }
                e => cut.push(e),
            }
        }
        t.pts = crate::realize::flatten(&cut);
        t.elems = cut;
    }
    let mut idx = GridIndex::new(bounds, 1.0);
    let mut pieces: Vec<(Point, Point, u32)> = Vec::new();
    for (ti, t) in geo.tracks.iter().enumerate() {
        for w in t.pts.windows(2) {
            let mut r = Rect::empty();
            r.add(w[0]);
            r.add(w[1]);
            idx.insert(pieces.len() as u32, &r);
            pieces.push((w[0], w[1], ti as u32));
        }
    }
    let margin = RelaxConfig::default().margin;
    for ti in 0..geo.tracks.len() {
        if geo.tracks[ti].necks.is_empty() {
            continue;
        }
        let t = &geo.tracks[ti];
        let (net, layer, c) = (t.net, t.layer, clr(t.net));
        let fits = |a: Point, b: Point, w: f64| -> bool {
            let mut r = Rect::empty();
            r.add(a);
            r.add(b);
            let rr = r.grow(w / 2.0 + c + 1.0);
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
                    ObstacleKind::Hole => board.hole_gap(net),
                    _ => o.clearance.max(c).max(board.item_gap(net, o.net)),
                };
                if sub.shared.obs_polys[oi as usize].dist_to_seg(a, b) < need + w / 2.0 + margin {
                    return false;
                }
            }
            for v in &geo.vias {
                if v.net != net
                    && point_seg_dist(v.at, a, b)
                        < v.diameter / 2.0 + w / 2.0 + c.max(clr(v.net)).max(board.item_gap(net, Some(v.net))) + margin
                {
                    return false;
                }
            }
            for pi in idx.query(&rr) {
                let (p, q, tj) = pieces[pi as usize];
                let o = &geo.tracks[tj as usize];
                if o.net == net || o.layer != layer {
                    continue;
                }
                let need = w / 2.0 + o.piece_width(p, q) / 2.0 + board.track_clearance(net, o.net) + margin;
                if crate::geom::seg_seg_dist(a, b, p, q) < need {
                    return false;
                }
            }
            true
        };
        // the widest step that fits, piece by piece; the minimum of a width rule is a step of
        // its own
        let min = board.class_of(net).min_width;
        let fitted: Vec<(Rect, f64)> = t
            .pts
            .windows(2)
            .filter(|p| t.piece_width(p[0], p[1]) < t.width)
            .map(|p| {
                let neck = t.piece_width(p[0], p[1]);
                let steps = ((t.width - neck) / 0.05).floor() as usize;
                let mut widths: Vec<f64> = (0..=steps).map(|i| t.width - i as f64 * 0.05).collect();
                if let Some(m) = min.filter(|&m| m > neck && m < t.width && !widths.contains(&m)) {
                    let at = widths.iter().position(|&w| w < m).unwrap_or(widths.len());
                    widths.insert(at, m);
                }
                let best = widths.into_iter().find(|&w| fits(p[0], p[1], w)).unwrap_or(neck);
                let mut r = Rect::empty();
                r.add((p[0] + p[1]) * 0.5);
                (r.grow(1e-6), best)
            })
            .collect();
        geo.tracks[ti].fitted = fitted;
    }
}

/// A row of [`stitch_vias`]: the pieces of copper from the via (layer and end) and the new vias.
type StitchRow = (Vec<(LayerId, Point)>, Vec<Point>);

/// A layer change on a current path carries what its tracks carry: a via that joins copper of
/// the minimum width of its class on two layers gets as many more vias next to it as that takes
/// ([`Board::vias_for`]), in a row on the straight end of one of its tracks with a piece of the
/// other layer's copper over them, or where those have no room around the via, with a piece of
/// copper from it on each layer. Only where they keep the clearances of the check, the hole to
/// hole distance, and stay out of the pads; kept copper of the file stays as it is. Returns how
/// many vias it added.
pub fn stitch_vias(board: &Board, geo: &mut Geometry) -> usize {
    let near = 1e-3;
    let mut added = 0;
    for vi in 0..geo.vias.len() {
        let v = geo.vias[vi].clone();
        if v.drill <= 0.0 || v.resting {
            continue;
        }
        let cls = board.class_of(v.net);
        let Some(min) = board.current_min(v.net) else {
            continue;
        };
        let need = board.vias_for(min, v.drill);
        if need <= 1 {
            continue;
        }
        // the wide copper at the via: per track that ends there its layer, direction and how
        // far it runs on straight and wide
        let mut arms: Vec<(LayerId, Point, f64)> = Vec::new();
        let mut wide = LayerSet::default();
        for t in geo.tracks.iter().filter(|t| t.net == v.net && t.pts.len() > 1) {
            let pts: Vec<Point> = if t.pts[0].dist(v.at) < near {
                t.pts.clone()
            } else if t.pts[t.pts.len() - 1].dist(v.at) < near {
                t.pts.iter().rev().copied().collect()
            } else {
                continue;
            };
            let carries = |a: Point, b: Point| board.current_width(t.layer, t.piece_width(a, b)) >= min - 1e-9;
            if !carries(pts[0], pts[1]) {
                continue;
            }
            wide.insert(t.layer);
            let u = (pts[1] - pts[0]).norm();
            let mut len = 0.0;
            for w in pts.windows(2) {
                let d = w[1] - w[0];
                if u.cross(d).abs() > 1e-6 * d.len() || u.dot(d) <= 0.0 || !carries(w[0], w[1]) {
                    break;
                }
                len += d.len();
            }
            arms.push((t.layer, u, len));
        }
        // a wide pad of the net under the via is wide copper as well
        for &p in board.nets[v.net as usize]
            .pins
            .iter()
            .filter(|&&p| !board.narrow_pin(p))
        {
            for &o in &board.pins[p].obstacles {
                let o = &board.obstacles[o];
                if o.shape.dist_to_point(v.at) < near {
                    o.layers.iter().for_each(|l| wide.insert(l));
                }
            }
        }
        if wide.iter().count() < 2 {
            continue;
        }
        let pitch = v.drill + board.rules.hole_to_hole + 0.02;
        let on_arm = |q: Point, &(_, u, len): &(LayerId, Point, f64)| {
            let d = q - v.at;
            let along = d.dot(u);
            along > near && along <= len + near && (d - u * along).len() < near
        };
        // the vias of an earlier row
        let have = 1 + geo
            .vias
            .iter()
            .enumerate()
            .filter(|&(j, w)| j != vi && w.net == v.net && w.drill > 0.0 && arms.iter().any(|a| on_arm(w.at, a)))
            .count();
        if have >= need {
            continue;
        }
        let clr = cls.clearance.max(board.rules.min_clearance);
        let hole_room = |at: Point, placed: &[Point]| -> bool {
            let gap = board.rules.hole_to_hole;
            geo.vias
                .iter()
                .filter(|w| w.drill > 0.0)
                .all(|w| w.at.dist(at) >= (w.drill + v.drill) / 2.0 + gap - 1e-9)
                && placed.iter().all(|p| p.dist(at) >= v.drill + gap - 1e-9)
                && board
                    .obstacles
                    .iter()
                    .filter(|o| o.hole.is_some())
                    .all(|o| o.shape.bbox().center().dist(at) >= o.hole.unwrap() + v.drill / 2.0 + gap - 1e-9)
        };
        let pad = Shape::Circle {
            c: v.at,
            r: v.diameter / 2.0,
        };
        let off_pads = |at: Point| {
            let pad = match pad {
                Shape::Circle { r, .. } => Shape::Circle { c: at, r },
                _ => unreachable!(),
            };
            board.nets[v.net as usize].pins.iter().all(|&p| {
                board.pins[p]
                    .obstacles
                    .iter()
                    .all(|&o| board.obstacles[o].shape.dist(&pad) >= clr - 1e-9)
            })
        };
        let copper = crate::check::Copper::new(board, geo);
        let width = cls.track_width.max(min);
        arms.sort_by(|a, b| b.2.total_cmp(&a.2));
        // where the vias go: in a row on the straight end of a track, the longest first, else
        // around the via; a via off the copper of a wide layer gets a piece of it from the via
        let mut dirs: Vec<(Point, f64)> = arms.iter().map(|a| (a.1, a.2)).collect();
        dirs.extend((0..8).map(|i| {
            let a = i as f64 * std::f64::consts::FRAC_PI_4;
            (crate::geom::pt(a.cos(), a.sin()), f64::INFINITY)
        }));
        // per direction: the pieces of copper from the via (layer, end) and the vias
        let mut rows: Vec<StitchRow> = Vec::new();
        let mut placed: Vec<Point> = Vec::new();
        for (u, len) in dirs {
            let mut pieces: Vec<(LayerId, Point)> = Vec::new();
            let mut row = Vec::new();
            for k in 1.. {
                if have + placed.len() >= need || pitch * k as f64 > len + near {
                    break;
                }
                let at = v.at + u * (pitch * k as f64);
                let segs: Vec<(Point, Point, f64, LayerId)> = wide
                    .iter()
                    .filter(|&l| !arms.iter().any(|a| a.0 == l && on_arm(at, a)))
                    .map(|l| (v.at, at, width, l))
                    .collect();
                if !hole_room(at, &placed) || !off_pads(at) || !copper.clear(v.net, &segs, &[(at, v.diameter)]) {
                    break;
                }
                for &(_, _, _, l) in &segs {
                    match pieces.iter_mut().find(|p| p.0 == l) {
                        Some(p) => p.1 = at,
                        None => pieces.push((l, at)),
                    }
                }
                row.push(at);
                placed.push(at);
            }
            if !row.is_empty() {
                rows.push((pieces, row));
            }
        }
        for (pieces, row) in rows {
            for (layer, end) in pieces {
                let elems = vec![Elem::Line(v.at, end)];
                geo.tracks.push(Track {
                    pts: crate::realize::flatten(&elems),
                    elems,
                    width,
                    layer,
                    net: v.net,
                    necks: Vec::new(),
                    neck_w: width,
                    origin: None,
                    resting: false,
                    fitted: Vec::new(),
                    pattern: None,
                    seg: u32::MAX,
                    tap: None,
                });
            }
            for at in row {
                geo.vias.push(Via {
                    at,
                    origin: None,
                    resting: false,
                    ..v.clone()
                });
                added += 1;
            }
        }
    }
    added
}

/// A piece of track of one width while tracks grow: an element of the track.
struct Grown {
    track: u32,
    chords: Vec<(Point, Point)>,
    len: f64,
    w: f64,
    max: f64,
}

/// Width steps of [`prefer_width`] in mm, and how short a wider stretch may be: a little more
/// width over a fraction of a mm is only a bump in the track.
const GROW_STEP: f64 = 0.025;
const GROW_MIN_LEN: f64 = 1.5;

/// Let the tracks grow up to `target` of their net (0: the net does not grow) wherever there is
/// room, keeping
/// `reserve` on top of the clearances; tracks in a bundle share the room. A track never gets
/// narrower than it is, and it changes width in steps of [`GROW_STEP`] per piece, which gives
/// a taper that needs no special copper.
pub fn prefer_width(board: &Board, sub: &Substrate, geo: &mut Geometry, target: impl Fn(u32) -> f64, reserve: f64) {
    let clr = |n: u32| board.class_of(n).clearance.max(board.rules.min_clearance);
    let margin = RelaxConfig::default().margin + reserve;
    let mut grown: Vec<Grown> = Vec::new();
    let mut spans: Vec<std::ops::Range<usize>> = Vec::new();
    for (ti, t) in geo.tracks.iter_mut().enumerate() {
        let from = grown.len();
        let target = target(t.net);
        let grows = t.width < target;
        let mut elems = Vec::with_capacity(t.elems.len());
        for e in &t.elems {
            let (w, parts) = match *e {
                Elem::Line(a, b) => {
                    let n = if grows {
                        (a.dist(b) / 0.3).ceil().max(1.0) as usize
                    } else {
                        1
                    };
                    let at = |i: usize| a + (b - a) * (i as f64 / n as f64);
                    (
                        t.piece_width(a, b),
                        (0..n).map(|i| Elem::Line(at(i), at(i + 1))).collect(),
                    )
                }
                Elem::Arc(a, _, b) => {
                    let pts = crate::realize::flatten(std::slice::from_ref(e));
                    let w = pts
                        .windows(2)
                        .map(|c| t.piece_width(c[0], c[1]))
                        .fold(t.piece_width(a, b), f64::min);
                    (w, vec![*e])
                }
            };
            for part in parts {
                let pts = crate::realize::flatten(std::slice::from_ref(&part));
                let chords: Vec<(Point, Point)> = pts.windows(2).map(|c| (c[0], c[1])).collect();
                grown.push(Grown {
                    track: ti as u32,
                    len: chords.iter().map(|c| c.0.dist(c.1)).sum(),
                    chords,
                    w,
                    max: if grows { target.max(w) } else { w },
                });
                elems.push(part);
            }
        }
        t.pts = crate::realize::flatten(&elems);
        t.elems = elems;
        spans.push(from..grown.len());
    }
    let mut idx = GridIndex::new(board.bbox().grow(5.0), 1.0);
    let mut pieces: Vec<(Point, Point, u32)> = Vec::new();
    for (ui, u) in grown.iter().enumerate() {
        for &(a, b) in &u.chords {
            let mut r = Rect::empty();
            r.add(a);
            r.add(b);
            idx.insert(pieces.len() as u32, &r);
            pieces.push((a, b, ui as u32));
        }
    }
    let fits = |grown: &[Grown], ui: usize, w: f64| -> bool {
        let u = &grown[ui];
        let t = &geo.tracks[u.track as usize];
        let (net, layer, c) = (t.net, t.layer, clr(t.net));
        u.chords.iter().all(|&(a, b)| {
            let mut r = Rect::empty();
            r.add(a);
            r.add(b);
            let rr = r.grow(w / 2.0 + c + 1.0);
            let on_board = board.outline.iter().all(|o| {
                o.edges()
                    .map(|(p, q)| crate::geom::seg_seg_dist(a, b, p, q))
                    .fold(f64::INFINITY, f64::min)
                    >= board.rules.edge_clearance + w / 2.0 + margin
            });
            on_board
                && sub.shared.obs_index.query(&rr).into_iter().all(|oi| {
                    let o = &board.obstacles[oi as usize];
                    if !o.layers.has(layer) || (o.net == Some(net) && o.kind != ObstacleKind::Hole) {
                        return true;
                    }
                    let need = match o.kind {
                        ObstacleKind::Keepout { tracks, .. } => {
                            if !tracks {
                                return true;
                            }
                            crate::board::KEEPOUT_GAP
                        }
                        ObstacleKind::Hole => board.hole_gap(net),
                        _ => o.clearance.max(c).max(board.item_gap(net, o.net)),
                    };
                    sub.shared.obs_polys[oi as usize].dist_to_seg(a, b) >= need + w / 2.0 + margin
                })
                && geo.vias.iter().all(|v| {
                    v.net == net
                        || v.layer.is_some_and(|l| l != layer)
                        || point_seg_dist(v.at, a, b)
                            >= v.diameter / 2.0
                                + w / 2.0
                                + c.max(clr(v.net)).max(board.item_gap(net, Some(v.net)))
                                + margin
                })
                && idx.query(&rr).into_iter().all(|pi| {
                    let (p, q, uj) = pieces[pi as usize];
                    let o = &geo.tracks[grown[uj as usize].track as usize];
                    o.net == net
                        || o.layer != layer
                        || crate::geom::seg_seg_dist(a, b, p, q)
                            >= w / 2.0 + grown[uj as usize].w / 2.0 + board.track_clearance(net, o.net) + margin
                })
        })
    };
    // round by round, so that neighbours share what is between them
    loop {
        let mut any = false;
        for ui in 0..grown.len() {
            let w = (grown[ui].w + GROW_STEP).min(grown[ui].max);
            if w > grown[ui].w && fits(&grown, ui, w) {
                grown[ui].w = w;
                any = true;
            }
        }
        if !any {
            break;
        }
    }
    for span in spans {
        let units = &mut grown[span];
        // short wide stretches go back to their neighbours
        loop {
            let mut changed = false;
            let mut i = 0;
            while i < units.len() {
                let mut j = i + 1;
                while j < units.len() && units[j].w == units[i].w {
                    j += 1;
                }
                let before = i.checked_sub(1).map(|k| units[k].w);
                let after = units.get(j).map(|u| u.w);
                let edge = before.into_iter().chain(after).fold(f64::NEG_INFINITY, f64::max);
                let len: f64 = units[i..j].iter().map(|u| u.len).sum();
                if edge.is_finite() && units[i].w > edge && len < GROW_MIN_LEN {
                    units[i..j].iter_mut().for_each(|u| u.w = edge);
                    changed = true;
                }
                i = j;
            }
            if !changed {
                break;
            }
        }
        // and every change of width gets a taper
        for i in 1..units.len() {
            units[i].w = units[i].w.min(units[i - 1].w + GROW_STEP);
        }
        for i in (1..units.len()).rev() {
            units[i - 1].w = units[i - 1].w.min(units[i].w + GROW_STEP);
        }
    }
    for t in geo.tracks.iter_mut() {
        t.fitted.clear();
    }
    for u in &grown {
        // the class width stays the width of the track: anything not listed here is at it
        let t = &mut geo.tracks[u.track as usize];
        let mids = u
            .chords
            .iter()
            .map(|c| (c.0 + c.1) * 0.5)
            .chain([(u.chords[0].0 + u.chords[u.chords.len() - 1].1) * 0.5]);
        for m in mids {
            let mut r = Rect::empty();
            r.add(m);
            t.fitted.push((r.grow(1e-6), u.w));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_rail_lets_go_of_a_track_that_turns_away() {
        let r = Rail {
            at: pt(0.0, 0.0),
            dir: pt(0.0, -1.0),
            len: 0.6,
        };
        // out of the pad a little off the middle, then far to the side at the height of the rail
        let mut pts = vec![
            pt(0.0, 0.0),
            pt(0.02, -0.1),
            pt(0.02, -0.2),
            pt(-1.0, -0.2),
            pt(-4.0, -0.2),
        ];
        follow(&r, &mut pts, false);
        assert_eq!(pts[2], pt(0.0, -0.2));
        assert_eq!(pts[3], pt(-1.0, -0.2));
        assert_eq!(pts[4], pt(-4.0, -0.2));
    }

    /// A track that leaves a long centre strip a little off its middle line comes back onto it;
    /// past the rail it stays as it was.
    #[test]
    fn a_finished_track_goes_back_onto_its_rail() {
        use crate::board::{Obstacle, Pin};
        use crate::geom::roundrect_polygon;
        let mut b = crate::topo::tests::two_pad_board();
        for (k, x) in [4.5, 5.0, 5.5, 6.0].into_iter().enumerate() {
            let o = b.obstacles.len();
            b.obstacles.push(Obstacle {
                shape: Shape::Polygon(roundrect_polygon(pt(x, 5.0), 0.3, 2.4, 0.0, 0.0, 0.005)),
                layers: LayerSet::single(0),
                net: Some(0),
                kind: ObstacleKind::Pad,
                hole: None,
                clearance: 0.2,
                pin: Some(b.pins.len()),
                locked: true,
            });
            b.pins.push(Pin {
                name: format!("U.{}", k + 1),
                footprint: "U".into(),
                net: 0,
                pos: pt(x, 5.0),
                layers: LayerSet::single(0),
                obstacles: vec![o],
                size: (0.3, 2.4),
            });
        }
        let mut t = crate::check::tests::line((5.0, 5.0), (5.04, 3.98), 0);
        t.elems.push(Elem::Line(pt(5.04, 3.98), pt(5.6, 3.0)));
        t.elems.push(Elem::Line(pt(5.6, 3.0), pt(9.0, 3.0)));
        t.pts = crate::realize::flatten(&t.elems);
        let mut geo = Geometry {
            tracks: vec![t],
            vias: Vec::new(),
        };
        assert_eq!(straighten_exits(&b, &mut geo, None), 1);
        let t = &geo.tracks[0];
        assert!(t.pts.iter().any(|p| p.dist(pt(5.0, 3.98)) < 1e-9), "{:?}", t.pts);
        assert!(t.pts.iter().all(|p| p.dist(pt(5.04, 3.98)) > 1e-6));
        assert!(t.pts.iter().any(|p| p.dist(pt(5.6, 3.0)) < 1e-9));
        assert_eq!(*t.pts.last().unwrap(), pt(9.0, 3.0));
        assert_eq!(straighten_exits(&b, &mut geo, None), 0, "a second run changes nothing");
    }

    /// A centre pad has a rail at each end; the track takes the one it leaves by.
    #[test]
    fn a_centre_pad_rails_both_ways() {
        let rail = |y: f64| Rail {
            at: pt(0.0, 0.0),
            dir: pt(0.0, y),
            len: 1.3,
        };
        let rails = [rail(1.0), rail(-1.0)];
        let north = [pt(0.0, 0.0), pt(0.04, -1.0), pt(0.3, -1.7), pt(2.0, -3.0)];
        assert_eq!(rail_of(&rails, north.iter()).map(|r| r.dir), Some(pt(0.0, -1.0)));
        let south = [pt(0.0, 0.0), pt(0.0, 1.2), pt(0.1, 2.0)];
        assert_eq!(rail_of(&rails, south.iter()).map(|r| r.dir), Some(pt(0.0, 1.0)));
        let mut pts = north.to_vec();
        follow(&rail(-1.0), &mut pts, false);
        assert_eq!(pts[1], pt(0.0, -1.0));
    }

    #[test]
    fn the_far_end_of_a_short_track_stays() {
        let r = Rail {
            at: pt(0.0, 0.0),
            dir: pt(0.0, -1.0),
            len: 0.6,
        };
        let mut pts = vec![pt(0.0, 0.0), pt(0.1, -0.2)];
        follow(&r, &mut pts, false);
        assert_eq!(pts[1], pt(0.1, -0.2));
    }

    /// A layer change between two tracks of a 4.1 A rule gets the two vias it lacks, in a row on
    /// one of the tracks with a piece of the other layer over them; where that piece would come
    /// too close to other copper, the row goes the other way.
    #[test]
    fn a_layer_change_on_a_current_path_gets_its_vias() {
        use crate::check::tests::{current_board, line, via};
        let mut b = current_board();
        let geo = || Geometry {
            tracks: vec![line((2., 5.), (5., 5.), 0), line((5., 5.), (8., 5.), 1)],
            vias: vec![via(5.)],
        };
        let mut g = geo();
        assert_eq!(stitch_vias(&b, &mut g), 2);
        assert!(
            crate::check::check(&b, &g).is_empty(),
            "{:?}",
            crate::check::check(&b, &g)
        );
        assert_eq!(stitch_vias(&b, &mut g), 0, "a second run adds nothing");
        // a pad of another net under the top track, on the bottom
        b.nets.push(crate::board::Net {
            name: "B".into(),
            class: 0,
            pins: Vec::new(),
            planes: LayerSet::default(),
            priority: 0,
        });
        let mut o = b.obstacles[1].clone();
        o.shape = Shape::Polygon(crate::geom::Polygon::new(vec![
            pt(3.6, 5.5),
            pt(4.4, 5.5),
            pt(4.4, 6.0),
            pt(3.6, 6.0),
        ]));
        o.net = Some(1);
        o.pin = None;
        b.obstacles.push(o);
        let mut g = geo();
        assert_eq!(stitch_vias(&b, &mut g), 2);
        assert!(
            crate::check::check(&b, &g).is_empty(),
            "{:?}",
            crate::check::check(&b, &g)
        );
        assert!(g.vias.iter().all(|v| v.at.x >= 5.0 - 1e-9), "{:?}", g.vias);
    }

    /// Under a minimum width rule a line that leaves its neck zone is cut at the zone: in the zone
    /// it takes what fits there, outside it keeps the width of the track, also where that is too
    /// wide (the check reports it: a neck outside the courtyard is no way out).
    #[test]
    fn a_ruled_neck_ends_at_its_zone() {
        use crate::board::{Layer, LayerSet, Net, NetClass, Obstacle};
        use crate::geom::{Polygon, Shape};
        let mut b = Board::default();
        b.rules.min_clearance = 0.1;
        b.layers = vec![Layer {
            name: "F.Cu".into(),
            is_plane: false,
        }];
        b.outline = vec![Polygon::new(vec![pt(0., 0.), pt(20., 0.), pt(20., 10.), pt(0., 10.)])];
        b.classes = vec![NetClass {
            name: "Input".into(),
            track_width: 0.75,
            clearance: 0.2,
            via_diameter: 0.8,
            via_drill: 0.4,
            min_width: Some(0.69),
            neck_refs: vec!["U1".into()],
            ..Default::default()
        }];
        for name in ["VBUS", "OTHER"] {
            b.nets.push(Net {
                name: name.into(),
                class: 0,
                pins: Vec::new(),
                planes: LayerSet::default(),
                priority: 0,
            });
        }
        let mut zone = Rect::empty();
        zone.add(pt(4., 4.));
        zone.add(pt(6., 6.));
        // the courtyard of U1
        b.courtyards.insert("U1".into(), zone);
        let track = |elems: Vec<Elem>| Track {
            pts: crate::realize::flatten(&elems),
            elems,
            width: 0.75,
            layer: 0,
            net: 0,
            necks: vec![zone],
            neck_w: 0.25,
            origin: None,
            resting: false,
            fitted: Vec::new(),
            pattern: None,
            seg: u32::MAX,
            tap: None,
        };
        let widen = |b: &Board| {
            let sub = Substrate::build(b, &crate::topo::SubstrateConfig::default());
            let mut geo = Geometry {
                tracks: vec![track(vec![Elem::Line(pt(5., 5.), pt(12., 5.))])],
                vias: Vec::new(),
            };
            widen_necks(b, &sub, &mut geo);
            geo
        };
        let widths = |geo: &Geometry| -> Vec<(Point, Point, f64)> {
            let t = &geo.tracks[0];
            t.elems
                .iter()
                .map(|e| match *e {
                    Elem::Line(a, b) | Elem::Arc(a, _, b) => (a, b, t.piece_width(a, b)),
                })
                .collect()
        };
        // with room all of it is wide, also the end in the zone
        let open = widen(&b);
        assert!(widths(&open).iter().all(|w| w.2 == 0.75), "{:?}", widths(&open));
        let pad = |x: f64, y: f64| Obstacle {
            shape: Shape::Polygon(Polygon::new(vec![pt(x, y), pt(x + 1., y), pt(x + 1., 6.), pt(x, 6.)])),
            layers: LayerSet::single(0),
            net: Some(1),
            kind: ObstacleKind::Pad,
            hole: None,
            clearance: 0.2,
            pin: None,
            locked: false,
        };
        // a pad next to the line outside the zone: the line stays wide there and the check
        // reports the clearance; the narrow piece ends where the round end of the wide one
        // stays out of the zone
        for y in [5.555, 5.5] {
            b.obstacles = vec![pad(8., y)];
            let geo = widen(&b);
            let w = widths(&geo);
            for &(a, c, wd) in &w {
                assert!(zone.touches_seg(a, c) || wd == 0.75, "{w:?}");
                assert!(!zone.touches_seg(a, c) || a.x.max(c.x) < 6.0 + 0.375 + 2e-3, "{w:?}");
            }
            assert!(!crate::check::check(&b, &geo).is_empty());
        }
        // a pad next to the end in the zone: there the line narrows to what fits
        b.obstacles = vec![pad(4.6, 5.5)];
        let geo = widen(&b);
        let w = widths(&geo);
        assert!(crate::check::check(&b, &geo).is_empty(), "{w:?}");
        assert!(w.iter().any(|x| x.2 < 0.75 && x.2 >= 0.25), "{w:?}");
        for &(a, c, wd) in &w {
            assert!(zone.touches_seg(a, c) || wd == 0.75, "{w:?}");
        }
    }

    /// A via pair is a capsule across its band: a track of another net passes its long side
    /// where the circle around both vias would push it away; and it turns with its tracks.
    #[test]
    fn a_via_pair_takes_less_room_than_its_circle_and_turns_with_its_tracks() {
        use crate::board::{Layer, LayerSet, Net, NetClass};
        use crate::geom::Polygon;
        let mut b = Board::default();
        b.rules.min_clearance = 0.1;
        b.layers = ["F.Cu", "B.Cu"]
            .iter()
            .map(|n| Layer {
                name: n.to_string(),
                is_plane: false,
            })
            .collect();
        b.outline = vec![Polygon::new(vec![pt(0., 0.), pt(20., 0.), pt(20., 10.), pt(0., 10.)])];
        b.classes = vec![NetClass {
            name: "Default".into(),
            track_width: 0.2,
            clearance: 0.1,
            via_diameter: 0.6,
            via_drill: 0.3,
            ..Default::default()
        }];
        for name in ["A", "B"] {
            b.nets.push(Net {
                name: name.into(),
                class: 0,
                pins: Vec::new(),
                planes: LayerSet::default(),
                priority: 0,
            });
        }
        let track = |a: Point, c: Point, layer: u8, net: u32| {
            let elems = vec![Elem::Line(a, c)];
            Track {
                pts: crate::realize::flatten(&elems),
                elems,
                width: 0.2,
                layer,
                net,
                necks: Vec::new(),
                neck_w: 0.2,
                origin: None,
                resting: false,
                fitted: Vec::new(),
                pattern: None,
                seg: u32::MAX,
                tap: None,
            }
        };
        // the band comes in on the top and goes back on the bottom: the vias stand across it
        let c = pt(10., 5.);
        let geo = |half: Point, neighbour: bool| {
            let mut tracks = vec![track(pt(2., 5.), c, 0, 0), track(c, pt(2., 5.5), 1, 0)];
            if neighbour {
                tracks.push(track(pt(10.75, 1.), pt(10.75, 9.), 0, 1));
            }
            Geometry {
                tracks,
                vias: vec![Via {
                    at: c,
                    diameter: 1.5,
                    drill: 1.2,
                    net: 0,
                    layer: None,
                    origin: None,
                    resting: false,
                    half,
                }],
            }
        };
        let sub = Substrate::build(&b, &crate::topo::SubstrateConfig::default());
        let gap = |half: Point| {
            let mut g = geo(half, true);
            relax(&b, &sub, &mut g, &RelaxConfig::default());
            let x = g.tracks[2].pts.iter().map(|p| p.x).fold(f64::INFINITY, f64::min);
            (x - g.vias[0].at.x, g)
        };
        // two vias of 0.6 mm 0.9 mm apart: 0.3 mm copper and 0.1 mm clearance to a track of 0.2 mm
        let (pair, g) = gap(pt(0., 0.45));
        assert!(pair < 0.8, "{pair}");
        assert!(
            crate::check::check(&b, &g).is_empty(),
            "{:?}",
            crate::check::check(&b, &g)
        );
        // the circle around them: 0.75 mm, the track goes away
        let (round, _) = gap(pt(0., 0.));
        assert!(round > 0.85, "{round}");
        // a pair that stands along the band turns across it
        let mut g = geo(pt(0.45, 0.), false);
        relax(&b, &sub, &mut g, &RelaxConfig::default());
        let h = g.vias[0].half;
        assert!(h.x.abs() < 0.1 && (h.len() - 0.45).abs() < 1e-9, "{h:?}");
    }
}
