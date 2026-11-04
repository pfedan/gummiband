//! Buses as bundles. A bus is a run of at least [`MIN_MEMBERS`] connections between the same
//! two parts (the blocks of the `groups` hint, see [`crate::route::Router::plan`]) that are
//! neighbours at both parts, in the same order; a swap ends the run. The router lays the members
//! one after the other in their order across the bus and keeps them together in two ways
//! (`route --patterns buses`):
//!
//! - Pull: a member pays for every step away from the ways of its neighbours across the bus,
//!   as the second net of a differential pair does towards the first ([`RouterConfig::bus_pull`]).
//!   On a neighbour's way, one track pitch from it, a step costs nothing; on the other layer the
//!   neighbour counts as 1 mm farther. So the members follow the first one through the same
//!   gaps, round the same side of every part and change layers where it does.
//! - Room: the members still to come need room next to it. Where the room beside its way, up to
//!   the next fixed obstacle of another net or the board edge on their side, is less than the
//!   rest of the bus takes, it pays per mm of way and mm missing ([`RouterConfig::bus_room`]).
//!   Near the pins of the bus the members fan out to their own pads, so the room is not asked
//!   for there.
//!
//! [`RouterConfig::bus_pull`]: crate::route::RouterConfig::bus_pull
//! [`RouterConfig::bus_room`]: crate::route::RouterConfig::bus_room

use crate::board::{Board, NetId};
use crate::geom::Point;
use crate::index::FastMap;

/// A block needs at least this many connections to be a bus.
pub const MIN_MEMBERS: usize = 3;

/// The buses of a routing plan. Members are named by the pin their connection starts from,
/// which is unique per connection and stays the same when the order of the connections changes.
#[derive(Debug, Clone, Default)]
pub struct Buses {
    /// Start pin of a connection -> (bus, place across the bus).
    pub of: FastMap<usize, (u32, u32)>,
    /// Start pins of the members of each bus, in order across the bus.
    pub members: Vec<Vec<usize>>,
    /// The pins at both ends of every member of each bus.
    pub pins: Vec<Vec<Point>>,
    /// Track pitch (width and clearance) of each bus, the widest of its members.
    pub pitch: Vec<f64>,
    /// Per bus and member: whether the members after it across the bus lie to the left of its
    /// line from start to end (the members before it lie on the other side).
    pub after_left: Vec<Vec<bool>>,
}

impl Buses {
    /// Buses from the blocks of a plan: per block the members as (place across the block, start
    /// pin, pins at both ends, net). Every block gives its runs that keep their order
    /// ([`planar_runs`]) of at least [`MIN_MEMBERS`].
    pub fn from_blocks(board: &Board, blocks: Vec<Vec<(f64, usize, Vec<usize>, NetId)>>) -> Buses {
        let mut out = Buses::default();
        for mut block in blocks {
            if block.len() < MIN_MEMBERS {
                continue;
            }
            block.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
            for run in planar_runs(board, block) {
                out.push(board, run);
            }
        }
        out
    }

    /// Adds one bus, its members in order across it.
    fn push(&mut self, board: &Board, block: Vec<(f64, usize, Vec<usize>, NetId)>) {
        let bi = self.members.len() as u32;
        let mut pitch: f64 = 0.0;
        let mut pins = Vec::new();
        let mut members = Vec::new();
        // the side of the next member (or away from the one before, for the last)
        let mid = |e: &[usize]| {
            let n = e.len().max(1) as f64;
            e.iter().fold(Point::default(), |s, &p| s + board.pins[p].pos) * (1.0 / n)
        };
        let line = |e: &[usize]| (board.pins[e[0]].pos, board.pins[*e.last().unwrap()].pos);
        let after_left: Vec<bool> = (0..block.len())
            .map(|k| {
                let (a, z) = line(&block[k].2);
                let left = |o: &[usize]| (z - a).cross(mid(o) - a) > 0.0;
                match (block.get(k + 1), k.checked_sub(1).map(|j| &block[j])) {
                    (Some(next), _) => left(&next.2),
                    (None, Some(prev)) => !left(&prev.2),
                    (None, None) => true,
                }
            })
            .collect();
        self.after_left.push(after_left);
        for (k, (_, from, ends, net)) in block.into_iter().enumerate() {
            self.of.insert(from, (bi, k as u32));
            members.push(from);
            pins.extend(ends.iter().map(|&p| board.pins[p].pos));
            let cls = board.class_of(net);
            pitch = pitch.max(cls.track_width + cls.clearance.max(board.rules.min_clearance));
        }
        self.members.push(members);
        self.pins.push(pins);
        self.pitch.push(pitch);
    }

    pub fn is_empty(&self) -> bool {
        self.members.is_empty()
    }

    /// The neighbours across the bus of the member that starts at `from`.
    pub fn neighbours(&self, from: usize) -> impl Iterator<Item = usize> + '_ {
        let found = self.of.get(&from).copied();
        found.into_iter().flat_map(move |(b, k)| {
            let m = &self.members[b as usize];
            let k = k as usize;
            [k.checked_sub(1), Some(k + 1)]
                .into_iter()
                .flatten()
                .filter_map(move |i| m.get(i).copied())
        })
    }

    /// Whether q lies farther than `fan` from every pin of bus b.
    pub fn away_from_pins(&self, b: usize, q: Point, fan: f64) -> bool {
        let f2 = fan * fan;
        self.pins[b].iter().all(|p| (*p - q).len2() > f2)
    }
}

/// The runs of a block (in order across it at its first part) whose lines keep their order at
/// the other part as well: next to each other at both ends, in the same direction, along a row
/// of pins (each step more across the bus than along it). A block with
/// swapped lines crosses itself, and pulling its members together would pull them across each
/// other; only the runs between the swaps are bundles. Runs shorter than [`MIN_MEMBERS`] are
/// left out.
fn planar_runs(
    board: &Board,
    block: Vec<(f64, usize, Vec<usize>, NetId)>,
) -> Vec<Vec<(f64, usize, Vec<usize>, NetId)>> {
    let fp = |p: usize| board.pins[p].footprint.as_str();
    let first = &block[0].2;
    let part_a = fp(first[0]).min(fp(*first.last().unwrap()));
    // the pin of every member on each of the two parts
    let ends: Vec<(Point, Point)> = block
        .iter()
        .map(|(_, _, e, _)| {
            let (p, q) = (e[0], *e.last().unwrap());
            let (a, z) = if fp(p) == part_a { (p, q) } else { (q, p) };
            (board.pins[a].pos, board.pins[z].pos)
        })
        .collect();
    let n = ends.len() as f64;
    let ca = ends.iter().fold(Point::default(), |s, e| s + e.0) * (1.0 / n);
    let cb = ends.iter().fold(Point::default(), |s, e| s + e.1) * (1.0 / n);
    let axis = cb - ca;
    let len = axis.dist(Point::default()).max(1e-9);
    let normal = Point {
        x: -axis.y / len,
        y: axis.x / len,
    };
    // places across the bus at both parts; the block comes in the order of the first part
    let mut by_b: Vec<usize> = (0..block.len()).collect();
    by_b.sort_by(|&i, &j| (ends[i].1 - cb).dot(normal).total_cmp(&(ends[j].1 - cb).dot(normal)));
    let mut rank_b = vec![0i64; block.len()];
    for (r, &i) in by_b.iter().enumerate() {
        rank_b[i] = r as i64;
    }
    let mut by_a: Vec<usize> = (0..block.len()).collect();
    by_a.sort_by(|&i, &j| (ends[i].0 - ca).dot(normal).total_cmp(&(ends[j].0 - ca).dot(normal)));
    let mut rank_a = vec![0i64; block.len()];
    for (r, &i) in by_a.iter().enumerate() {
        rank_a[i] = r as i64;
    }
    // runs along the block order: the next member is the next one at both parts, in the
    // direction of the run
    let mut runs: Vec<Vec<usize>> = Vec::new();
    let mut cur: Vec<usize> = vec![0];
    let mut dir: Option<(i64, i64)> = None;
    for k in 1..block.len() {
        let j = *cur.last().unwrap();
        let step = (rank_a[k] - rank_a[j], rank_b[k] - rank_b[j]);
        // and a step along a row at both parts: across the bus more than along it (pins of a
        // pad grid next to each other across the bus may lie in different rows)
        let across = |p: Point, q: Point| 2.0 * (q - p).dot(normal).abs() >= (q - p).dist(Point::default());
        let ok = step.0.abs() == 1
            && step.1.abs() == 1
            && dir.is_none_or(|d| d == step)
            && across(ends[j].0, ends[k].0)
            && across(ends[j].1, ends[k].1);
        if ok {
            dir = Some(step);
            cur.push(k);
        } else {
            runs.push(std::mem::replace(&mut cur, vec![k]));
            dir = None;
        }
    }
    runs.push(cur);
    let mut slots: Vec<Option<_>> = block.into_iter().map(Some).collect();
    runs.into_iter()
        .filter(|r| r.len() >= MIN_MEMBERS)
        .map(|r| r.into_iter().map(|k| slots[k].take().unwrap()).collect())
        .collect()
}
