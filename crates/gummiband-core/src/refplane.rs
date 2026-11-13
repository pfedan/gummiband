//! Reference planes of the routing layers, for the price of a step that leaves its return path.
//!
//! The return current of a signal flows in the plane right under the track. Where the plane
//! next to a routing layer has no copper (outside its outline, in the cut of a zone that KiCad
//! fills first) the current has to detour, and where the track passes from the zone of one net
//! into that of another (a split supply plane) it has to find a way across the split. Both add
//! loop inductance, ringing and emission. The router pays for such stretches
//! ([`crate::route::RouterConfig::ref_cost`]), so that a signal keeps over unbroken copper where
//! it can, on the layer next to ground or around a split.
//!
//! The planes are rastered once per board ([`RefMap::build`]), at [`STEP`]. Only zone outlines
//! count: the antipads of vias are not in this map, `reference_check` of the MCP server sees
//! them in the filled zones after routing.

use crate::board::{Board, LayerId, NetId};
use crate::geom::{Point, pt};

/// Raster step in mm.
pub const STEP: f64 = 0.25;
/// A split costs as much as this many mm of track over no copper.
pub const SPLIT_MM: f64 = 2.0;
/// No plane copper in a cell.
const GAP: u32 = 0;

/// The copper of one plane layer: per cell the net of the plane there plus one, [`GAP`] for none.
struct Raster {
    cells: Vec<u32>,
}

/// The planes next to each routing layer, rastered, and which nets pay for leaving them.
pub struct RefMap {
    x0: f64,
    y0: f64,
    w: usize,
    h: usize,
    /// Per routing layer (index into `Substrate::layers`): the rasters of its reference layers,
    /// the plane layers right above and below it in the stack. Empty: nothing to pay for.
    refs: Vec<Vec<usize>>,
    rasters: Vec<Raster>,
    /// Per net: a step of this net pays. Nets with a plane of their own and nets of clearly wide
    /// (power) classes do not: their current returns through the planes anyway.
    pays: Vec<bool>,
}

/// Width from which a class counts as power: twice the default width and at least 0.5 mm (as in
/// the `reference_check` of the MCP server).
fn wide(board: &Board, w: f64) -> bool {
    let default = board
        .classes
        .iter()
        .find(|c| c.name == "Default")
        .or_else(|| {
            board
                .classes
                .iter()
                .min_by(|a, b| a.track_width.total_cmp(&b.track_width))
        })
        .map_or(0.2, |c| c.track_width);
    w >= (2.0 * default).max(0.5)
}

impl RefMap {
    /// None if no routing layer has a plane layer next to it.
    pub fn build(board: &Board, routing: &[LayerId]) -> Option<RefMap> {
        let with_planes = |l: LayerId| board.planes.iter().any(|p| p.layer == l);
        let mut plane_layers: Vec<LayerId> = Vec::new();
        let refs_of: Vec<Vec<LayerId>> = routing
            .iter()
            .map(|&l| {
                let mut v = Vec::new();
                if l > 0 && board.layers[l as usize - 1].is_plane && with_planes(l - 1) {
                    v.push(l - 1);
                }
                if (l as usize) + 1 < board.layers.len() && board.layers[l as usize + 1].is_plane && with_planes(l + 1)
                {
                    v.push(l + 1);
                }
                for &r in &v {
                    if !plane_layers.contains(&r) {
                        plane_layers.push(r);
                    }
                }
                v
            })
            .collect();
        if plane_layers.is_empty() {
            return None;
        }
        let bb = board.bbox();
        if bb.width() <= 0.0 || bb.height() <= 0.0 {
            return None;
        }
        let (x0, y0) = (bb.min.x, bb.min.y);
        let w = (bb.width() / STEP).ceil() as usize + 1;
        let h = (bb.height() / STEP).ceil() as usize + 1;
        let rasters: Vec<Raster> = plane_layers
            .iter()
            .map(|&l| {
                // the highest priority first: KiCad fills it first, `cut` says the same
                let mut planes: Vec<_> = board.planes.iter().filter(|p| p.layer == l).collect();
                planes.sort_by_key(|p| std::cmp::Reverse(p.priority));
                // Not in the rayon pool: the map is built inside a `OnceLock` that the searches
                // of the pool wait on. A pool worker that waited here for its subtasks would
                // steal another search, which blocks on the same lock for good.
                let cells = (0..h)
                    .flat_map(|j| {
                        let planes = &planes;
                        (0..w).map(move |i| {
                            let p = pt(x0 + (i as f64 + 0.5) * STEP, y0 + (j as f64 + 0.5) * STEP);
                            planes.iter().find(|pl| pl.covers(p)).map_or(GAP, |pl| pl.net + 1)
                        })
                    })
                    .collect();
                Raster { cells }
            })
            .collect();
        let refs = refs_of
            .iter()
            .map(|v| {
                v.iter()
                    .map(|r| plane_layers.iter().position(|x| x == r).unwrap())
                    .collect()
            })
            .collect();
        let pays = (0..board.nets.len())
            .map(|n| {
                let n = n as NetId;
                board.nets[n as usize].planes.is_empty() && !wide(board, board.class_of(n).track_width)
            })
            .collect();
        Some(RefMap {
            x0,
            y0,
            w,
            h,
            refs,
            rasters,
            pays,
        })
    }

    fn cell(&self, p: Point) -> Option<usize> {
        let i = ((p.x - self.x0) / STEP).floor();
        let j = ((p.y - self.y0) / STEP).floor();
        if i < 0.0 || j < 0.0 || i as usize >= self.w || j as usize >= self.h {
            return None;
        }
        Some(j as usize * self.w + i as usize)
    }

    /// Whether a step of `net` on routing layer `li` can pay at all.
    pub fn applies(&self, li: usize, net: NetId) -> bool {
        self.pays.get(net as usize).copied().unwrap_or(false) && self.refs.get(li).is_some_and(|r| !r.is_empty())
    }

    /// Price of the step a-b on routing layer `li` in mm: its length over no copper of any of
    /// its reference planes plus [`SPLIT_MM`] for every split.
    pub fn penalty(&self, li: usize, a: Point, b: Point) -> f64 {
        let (gap, splits) = self.leaves(li, a, b);
        gap + SPLIT_MM * splits as f64
    }

    /// Length in mm of the step a-b on routing layer `li` over no copper of any of its reference
    /// planes, and the number of splits: the track passes from the plane of one net into that of
    /// another on each of its reference layers. Off the board raster nothing counts.
    pub fn leaves(&self, li: usize, a: Point, b: Point) -> (f64, u32) {
        let refs = &self.refs[li];
        let len = a.dist(b);
        let n = ((len / STEP).ceil() as usize).max(1);
        let piece = len / n as f64;
        let mut gap = 0.0;
        // last copper net seen and splits so far, per reference layer
        let mut last = [GAP; 2];
        let mut splits = [0u32; 2];
        for s in 0..=n {
            let p = a + (b - a) * (s as f64 / n as f64);
            let Some(c) = self.cell(p) else { continue };
            let mut covered = false;
            for (k, &r) in refs.iter().enumerate().take(2) {
                let v = self.rasters[r].cells[c];
                if v != GAP {
                    covered = true;
                    if last[k] != GAP && last[k] != v {
                        splits[k] += 1;
                    }
                    last[k] = v;
                }
            }
            // the end points count half: the next step starts where this one ends
            if !covered {
                gap += if s == 0 || s == n { piece * 0.5 } else { piece };
            }
        }
        let split = splits.iter().take(refs.len().min(2)).copied().min().unwrap_or(0);
        (gap, split)
    }
}

/// How the signal tracks of a routed board keep to their reference planes.
#[derive(Debug, Clone, Copy, Default)]
pub struct Measure {
    /// Track length checked (signal nets on layers next to a plane), mm.
    pub checked_mm: f64,
    /// Track length over no copper of any reference plane, mm.
    pub gap_mm: f64,
    /// Passes from the plane of one net into that of another.
    pub splits: u32,
}

/// Measures the tracks of a routed board against the reference planes of their layers; None
/// when no routing layer has a plane next to it. Zone outlines only, like the price.
pub fn measure(board: &Board, tracks: &[crate::realize::Track]) -> Option<Measure> {
    let routing: Vec<LayerId> = board.routing_layers().iter().collect();
    let m = RefMap::build(board, &routing)?;
    let mut out = Measure::default();
    for t in tracks {
        let Some(li) = routing.iter().position(|&l| l == t.layer) else {
            continue;
        };
        if !m.applies(li, t.net) {
            continue;
        }
        for w in t.pts.windows(2) {
            let (gap, splits) = m.leaves(li, w[0], w[1]);
            out.checked_mm += w[0].dist(w[1]);
            out.gap_mm += gap;
            out.splits += splits;
        }
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::Plane;
    use crate::geom::Polygon;

    fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Polygon {
        Polygon::new(vec![pt(x0, y0), pt(x1, y0), pt(x1, y1), pt(x0, y1)])
    }

    /// Two layers, the second a plane: net 0 left of x = 10, net 1 right of it, a hole in the
    /// middle of net 0's half.
    fn board() -> Board {
        let mut b = crate::topo::tests::two_pad_board();
        b.layers[1].is_plane = true;
        b.outline = vec![rect(0.0, 0.0, 20.0, 10.0)];
        let mut other = b.nets[0].clone();
        other.name = "B".into();
        other.pins.clear();
        b.nets.push(other);
        let mut left = Plane::new(0, 1, rect(0.0, 0.0, 10.0, 10.0));
        left.cut = vec![rect(3.0, 4.0, 5.0, 6.0)];
        b.planes = vec![left, Plane::new(1, 1, rect(10.0, 0.0, 20.0, 10.0))];
        b
    }

    #[test]
    fn a_step_over_a_hole_pays_its_length_and_one_over_a_split_two_mm() {
        let b = board();
        let m = RefMap::build(&b, &[0]).unwrap();
        // in the copper of net 0
        assert!(m.penalty(0, pt(1.0, 2.0), pt(8.0, 2.0)) < 1e-9);
        // across the 2 mm hole
        let hole = m.penalty(0, pt(1.0, 5.0), pt(8.0, 5.0));
        assert!((hole - 2.0).abs() < 0.3, "{hole}");
        // from net 0 into net 1
        let split = m.penalty(0, pt(8.0, 2.0), pt(12.0, 2.0));
        assert!((split - SPLIT_MM).abs() < 1e-9, "{split}");
    }

    /// The map is built inside a `OnceLock` that all searches of the rayon pool read. Built
    /// with the pool itself, a worker waiting for its subtasks took another search, which
    /// waited on the same lock for good.
    #[test]
    fn the_map_builds_under_a_lock_the_whole_pool_waits_on() {
        use rayon::prelude::*;
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            // a large board, so that the build is still at work when the other searches arrive
            let mut b = board();
            b.outline = vec![rect(0.0, 0.0, 200.0, 100.0)];
            b.planes[0] = Plane::new(0, 1, rect(0.0, 0.0, 100.0, 100.0));
            b.planes[1] = Plane::new(1, 1, rect(100.0, 0.0, 200.0, 100.0));
            let pool = rayon::ThreadPoolBuilder::new().num_threads(8).build().unwrap();
            for _ in 0..10 {
                let lock: std::sync::OnceLock<Option<RefMap>> = std::sync::OnceLock::new();
                pool.install(|| {
                    (0..4096u64).into_par_iter().for_each(|i| {
                        // most of the pool is busy with other work and free to take a part
                        // of the build; now and then a search reaches the map
                        if i % 64 == 0 {
                            assert!(lock.get_or_init(|| RefMap::build(&b, &[0])).is_some());
                        } else {
                            std::hint::black_box((0..20_000u64).fold(i, |a, x| a.wrapping_mul(31) ^ x));
                        }
                    })
                });
            }
            tx.send(()).ok();
        });
        assert!(
            rx.recv_timeout(std::time::Duration::from_secs(60)).is_ok(),
            "the searches block each other on the lock of the map"
        );
    }
}
