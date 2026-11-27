//! Parts as ice floes: the rubber bands push the movable parts (`--move-parts`).
//!
//! A floe is what moves as one: a movable part, or a block with all its parts and copper. The
//! routed copper acts on it in three ways ([`forces`]):
//!
//! - **pressure**: a clearance problem or the pin of an open connection next to a pad asks for
//!   room, and the floes around it step away from it;
//! - **wrap**: a track of another net that bends around a pad of the floe presses on it with the
//!   turn of its direction there, as a taut band does on the post it runs around;
//! - **pull**: a track that ends on a pad of the floe pulls along the way it leaves.
//!
//! Wrap and pull together are the exact slope of the track length over the place of the floe
//! (the rest of the copper is taut already), so a floe that follows them makes the copper
//! shorter. [`targets`] turns the forces into new places, with the courtyards kept apart: a floe
//! shoves the floes in its way along, a fixed part or the board edge stops it. [`drift`] then
//! moves the floes there in steps no longer than the smallest clearance, with the topology kept
//! and the copper following and giving way after every step. Where the copper cannot give way
//! any more, the push goes through it: the floes behind the squeezed copper are shoved along,
//! and where that does not help, only the floe that is stuck stops. The others go on.

use std::collections::HashMap;

use crate::board::Board;
use crate::check;
use crate::geom::{Point, Rect, pt};
use crate::realize::Geometry;
use crate::relax::{self, RelaxConfig};
use crate::topo::{Substrate, SubstrateConfig};

/// What moves as one: a movable part alone, or the parts and the copper of a block.
#[derive(Debug, Clone)]
pub struct Floe {
    pub refs: Vec<String>,
    /// The obstacles that move with it (pads, holes, graphics, the copper of the block).
    obstacles: Vec<usize>,
}

/// The floes of a board: every movable part, the members of a block together.
pub fn floes(board: &Board) -> Vec<Floe> {
    let mut out: Vec<Floe> = Vec::new();
    let mut names: Vec<&String> = board.movable.iter().collect();
    names.sort();
    for name in names {
        if out.iter().any(|f| f.refs.contains(name)) {
            continue;
        }
        let block = board.blocks.iter().find(|b| b.refs.contains(name));
        let refs = block.map_or_else(|| vec![name.clone()], |b| b.refs.clone());
        let mut obstacles: Vec<usize> = refs
            .iter()
            .filter_map(|r| board.footprint_obstacles.get(r))
            .flatten()
            .copied()
            .collect();
        if let Some(b) = block {
            obstacles.extend(&b.copper);
        }
        obstacles.sort_unstable();
        obstacles.dedup();
        out.push(Floe { refs, obstacles });
    }
    out
}

impl Floe {
    /// The box around its obstacles on `cur`, or around its courtyards where it has none.
    fn bbox(&self, cur: &Board) -> Rect {
        let mut r = Rect::empty();
        for &o in &self.obstacles {
            let b = cur.obstacles[o].shape.bbox();
            r.add(b.min);
            r.add(b.max);
        }
        for c in self.refs.iter().filter_map(|f| cur.courtyards.get(f)) {
            r.add(c.min);
            r.add(c.max);
        }
        r
    }

    fn centre(&self, cur: &Board) -> Point {
        let r = self.bbox(cur);
        (r.min + r.max) * 0.5
    }

    /// The obstacle nearest to `p` within `reach`, with its distance.
    fn nearest(&self, cur: &Board, p: Point, reach: f64) -> Option<(usize, f64)> {
        self.obstacles
            .iter()
            .map(|&o| (o, cur.obstacles[o].shape.dist_to_point(p)))
            .filter(|x| x.1 <= reach)
            .min_by(|a, b| a.1.total_cmp(&b.1))
    }

    /// The offset of the floe in `offsets` (all its parts share it).
    fn offset(&self, offsets: &HashMap<String, Point>) -> Point {
        self.refs
            .iter()
            .find_map(|r| offsets.get(r))
            .copied()
            .unwrap_or_default()
    }
}

/// What the copper does to each floe, see the module.
#[derive(Debug, Clone, Default)]
pub struct Forces {
    /// Sum of the unit directions away from the spots that ask for room.
    pub press: Vec<Point>,
    /// Sum of the turns of the tracks of other nets around the pads, in units of their tension.
    pub wrap: Vec<Point>,
    /// Sum of the unit directions in which the own tracks leave the pads.
    pub pull: Vec<Point>,
}

/// How near a spot that asks for room has to be to copper of a floe to push it.
const PRESS_REACH: f64 = 1.0;
/// A track this much farther from a pad than its clearance still counts as wrapped around it.
const WRAP_SLACK: f64 = 0.12;
/// The direction of a track at its pad is taken over this length of it.
const PULL_LENGTH: f64 = 1.0;

/// A clearance problem: where, and between which nets (None: copper without a net).
#[derive(Debug, Clone, Copy)]
pub struct Squeeze {
    pub at: Point,
    pub nets: (u32, Option<u32>),
}

/// The forces of the copper `geo` on the floes, on the board `cur` (the offsets applied).
/// `spots` are the clearance problems, `open` the pins of open connections.
pub fn forces(cur: &Board, floes: &[Floe], geo: &Geometry, spots: &[Squeeze], open: &[Point]) -> Forces {
    let n = floes.len();
    let mut f = Forces {
        press: vec![Point::default(); n],
        wrap: vec![Point::default(); n],
        pull: vec![Point::default(); n],
    };
    let boxes: Vec<Rect> = floes.iter().map(|fl| fl.bbox(cur)).collect();
    // pressure: the floes around a spot step away from it; the floe whose pin is the spot (an
    // open connection) stays, its neighbours make the room
    let mut seen: Vec<Point> = Vec::new();
    for (h, nets) in spots
        .iter()
        .map(|s| (s.at, Some(s.nets)))
        .chain(open.iter().map(|&h| (h, None)))
    {
        let is_pin = nets.is_none();
        if seen.iter().any(|s| s.dist(h) < 1.0) {
            continue;
        }
        seen.push(h);
        // a problem between two pads of one part, or between a pad and the track on the pad next
        // to it, moves with the part: no room to make
        if let Some((a, Some(b))) = nets.filter(|n| n.1 != Some(n.0)) {
            let at_spot: Vec<(&String, u32)> = cur
                .pins
                .iter()
                .filter(|p| p.net == a || p.net == b)
                .filter(|p| {
                    p.obstacles
                        .iter()
                        .any(|&o| cur.obstacles[o].shape.dist_to_point(h) <= PRESS_REACH)
                })
                .map(|p| (&p.footprint, p.net))
                .collect();
            if at_spot
                .iter()
                .any(|x| x.1 == a && at_spot.iter().any(|y| y.1 == b && y.0 == x.0))
            {
                continue;
            }
        }
        for (i, fl) in floes.iter().enumerate() {
            if !boxes[i].grow(PRESS_REACH).contains(h) {
                continue;
            }
            let Some((o, d)) = fl.nearest(cur, h, PRESS_REACH) else {
                continue;
            };
            if is_pin && d < 1e-6 {
                continue;
            }
            let ob = cur.obstacles[o].shape.bbox();
            let mut away = (ob.min + ob.max) * 0.5 - h;
            if away.len() < 1e-6 {
                away = fl.centre(cur) - h;
            }
            if away.len() > 1e-6 {
                f.press[i] = f.press[i] + away.norm();
            }
        }
    }
    // the floes by the pins they carry, for the track ends
    let mut pins_of: Vec<Vec<usize>> = vec![Vec::new(); n];
    for (pi, p) in cur.pins.iter().enumerate() {
        if let Some(i) = floes.iter().position(|fl| fl.refs.contains(&p.footprint)) {
            pins_of[i].push(pi);
        }
    }
    let end_on = |p: Point, net: u32| -> Option<usize> {
        (0..n).find(|&i| {
            boxes[i].grow(1e-6).contains(p)
                && pins_of[i].iter().any(|&pi| {
                    let pin = &cur.pins[pi];
                    pin.net == net
                        && pin
                            .obstacles
                            .iter()
                            .any(|&o| cur.obstacles[o].shape.dist_to_point(p) < 1e-6)
                })
        })
    };
    for t in &geo.tracks {
        let m = t.pts.len();
        if m < 2 {
            continue;
        }
        // pull: where the track ends on a pad of a floe (not from a pad to a pad of one floe)
        let (a, b) = (end_on(t.pts[0], t.net), end_on(t.pts[m - 1], t.net));
        if a != b {
            let along = |from_start: bool| -> Point {
                let at = |k: usize| if from_start { t.pts[k] } else { t.pts[m - 1 - k] };
                let mut len = 0.0;
                let mut k = 1;
                while k + 1 < m && len + at(k).dist(at(k - 1)) < PULL_LENGTH {
                    len += at(k).dist(at(k - 1));
                    k += 1;
                }
                at(k) - at(0)
            };
            for (i, from_start) in [(a, true), (b, false)] {
                let d = along(from_start);
                if let Some(i) = i.filter(|_| d.len() > 1e-9) {
                    f.pull[i] = f.pull[i] + d.norm();
                }
            }
        }
        // wrap: every bend of the track next to a pad of another net, the pad inside the bend
        let class_clr = cur.class_of(t.net).clearance;
        for k in 1..m - 1 {
            let (p, q, r) = (t.pts[k - 1], t.pts[k], t.pts[k + 1]);
            if p.dist(q) < 1e-9 || q.dist(r) < 1e-9 {
                continue;
            }
            let turn = (r - q).norm() - (q - p).norm();
            if turn.len() < 1e-4 {
                continue;
            }
            let half = t.width_at(q) / 2.0;
            let mut best: Option<(usize, f64)> = None;
            for i in 0..n {
                let reach = half + class_clr.max(cur.rules.min_clearance) + WRAP_SLACK + 0.5;
                if !boxes[i].grow(reach).contains(q) {
                    continue;
                }
                for &o in &floes[i].obstacles {
                    let ob = &cur.obstacles[o];
                    if !ob.layers.has(t.layer) || ob.net == Some(t.net) {
                        continue;
                    }
                    let d = ob.shape.dist_to_point(q) - half - ob.clearance.max(class_clr);
                    let bb = ob.shape.bbox();
                    let inside = ((bb.min + bb.max) * 0.5 - q).dot(turn) > 0.0;
                    if d <= WRAP_SLACK && inside && best.is_none_or(|x| d < x.1) {
                        best = Some((i, d));
                    }
                }
            }
            if let Some((i, _)) = best {
                f.wrap[i] = f.wrap[i] + turn;
            }
        }
    }
    f
}

/// How the forces become moves ([`targets`]).
#[derive(Debug, Clone)]
pub struct Push {
    /// How far a floe steps away per round from the spots that ask for room, in mm.
    pub press_step: f64,
    /// mm of move per round for a band force of one track (wrap and pull together), 0: the
    /// bands alone move nothing.
    pub band_step: f64,
    /// Band forces below this move nothing: a part that sits well stays where it was placed.
    pub band_min: f64,
    /// Longest move of a floe per round in mm.
    pub max_step: f64,
    /// How far a floe may end up from where the file has it, in mm.
    pub reach: f64,
}

impl Push {
    /// As the board asks: its reach, and the bands at work where its parts may drift.
    pub fn of(board: &Board) -> Push {
        Push {
            press_step: 0.3,
            band_step: if board.drift { crate::knobs::FLOE_BAND.num() } else { 0.0 },
            band_min: 0.5,
            max_step: 0.5,
            reach: board.move_reach,
        }
    }
}

/// New footprint offsets from the forces: every floe moves as its forces say, at most
/// `push.max_step` and never farther than `push.reach` from its place in the file. A floe shoves
/// the movable floes in its way along and stops at fixed parts and at the board edge; it then
/// tries half the move. None if nothing moves. `cur` is `board` with `offsets` applied.
pub fn targets(
    board: &Board,
    cur: &Board,
    floes: &[Floe],
    offsets: &HashMap<String, Point>,
    f: &Forces,
    push: &Push,
) -> Option<HashMap<String, Point>> {
    let mut wishes: Vec<(usize, Point)> = Vec::new();
    for (i, fl) in floes.iter().enumerate() {
        let mut d = Point::default();
        if f.press[i].len() > 0.1 {
            d = d + f.press[i].norm() * push.press_step;
        }
        let band = f.wrap[i] + f.pull[i];
        if push.band_step > 0.0 && band.len() >= push.band_min {
            d = d + band * push.band_step;
        }
        if d.len() > push.max_step {
            d = d.norm() * push.max_step;
        }
        // not farther than the reach from the place in the file
        let at = fl.offset(offsets);
        let to = at + d;
        let to = if to.len() > push.reach { to.norm() * push.reach } else { to };
        let d = to - at;
        if d.len() > 0.01 {
            wishes.push((i, d));
        }
    }
    // the strongest first: it is the one the others give way to
    wishes.sort_by(|a, b| {
        b.1.len()
            .total_cmp(&a.1.len())
            .then_with(|| floes[a.0].refs.cmp(&floes[b.0].refs))
    });
    let mut offsets = offsets.clone();
    let mut courtyards = cur.courtyards.clone();
    let mut changed = false;
    for (i, d) in wishes {
        let fp = &floes[i].refs[0];
        for scale in [1.0, 0.5] {
            let (mut off, mut cy) = (offsets.clone(), courtyards.clone());
            if shove(board, fp, d * scale, &mut cy, &mut off, &mut Vec::new(), 0) {
                // a shoved floe stays within its reach as well
                if off.values().all(|o| o.len() <= push.reach + 1e-9) {
                    offsets = off;
                    courtyards = cy;
                    changed = true;
                    break;
                }
            }
        }
    }
    changed.then_some(offsets)
}

/// Shift footprint `fp` by `d`, and with it the rest of its block; movable parts whose
/// courtyards it would run into are shifted by the same amount first (up to a chain of four).
/// False if a fixed part or the board edge is in the way.
pub(crate) fn shove(
    board: &Board,
    fp: &str,
    d: Point,
    courtyards: &mut HashMap<String, Rect>,
    offsets: &mut HashMap<String, Point>,
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
    let moved = Rect {
        min: r.min + d,
        max: r.max + d,
    };
    if !corners(&moved).iter().all(|&q| board.inside(q)) {
        return false;
    }
    courtyards.insert(fp.to_string(), moved);
    let e = offsets.entry(fp.to_string()).or_insert(pt(0.0, 0.0));
    *e = *e + d;
    // the block moves as a whole (a member without courtyard only takes the offset)
    for block in board.blocks.iter().filter(|b| b.refs.iter().any(|x| x == fp)) {
        for other in &block.refs {
            if !courtyards.contains_key(other) {
                if !done.contains(other) {
                    done.push(other.clone());
                    let e = offsets.entry(other.clone()).or_insert(pt(0.0, 0.0));
                    *e = *e + d;
                }
            } else if !shove(board, other, d, courtyards, offsets, done, depth) {
                return false;
            }
        }
    }
    // parts that already overlapped may keep overlapping as much as before
    let mut hit: Vec<String> = courtyards
        .iter()
        .filter(|(other, o)| other.as_str() != fp && overlap(o, &moved) > overlap(o, &r) + 1e-9)
        .map(|(other, _)| other.clone())
        .collect();
    hit.sort();
    for other in hit {
        if done.contains(&other) {
            continue;
        }
        if depth >= 4 || !board.movable.contains(&other) || !shove(board, &other, d, courtyards, offsets, done, depth + 1)
        {
            return false;
        }
    }
    true
}

fn corners(r: &Rect) -> [Point; 4] {
    [r.min, r.max, pt(r.min.x, r.max.y), pt(r.max.x, r.min.y)]
}

fn overlap(a: &Rect, b: &Rect) -> f64 {
    let w = a.max.x.min(b.max.x) - a.min.x.max(b.min.x);
    let h = a.max.y.min(b.max.y) - a.min.y.max(b.min.y);
    if w > 0.0 && h > 0.0 { w * h } else { 0.0 }
}

/// What [`drift`] reached.
#[derive(Debug, Clone)]
pub struct Drifted {
    /// The geometry after the last step.
    pub geo: Geometry,
    /// The footprint offsets reached (all of `from`, updated where parts moved).
    pub offsets: HashMap<String, Point>,
    /// Steps taken, of how many planned.
    pub steps: usize,
    pub planned: usize,
    /// Floes that stopped short, each with the reason.
    pub stuck: Vec<(String, String)>,
    /// Floes that were shoved along through the copper between them and the floe that came.
    pub shoved: Vec<String>,
    /// How many floes moved at all, and the longest way of one.
    pub moved: usize,
    pub longest: f64,
}

/// Copper farther than this from every moving floe and every spot sleeps during a [`drift`].
/// The whole board awake settles a few micrometres differently far away from the move, and
/// that is not what a step is judged on.
const AWAKE: f64 = 5.0;
/// A new clearance problem this near to copper of a moving floe is the floe's doing.
const BLAME: f64 = 1.0;
/// A push goes through the copper to at most this many floes in a row.
const CHAIN: usize = 3;

/// Move the floes from the offsets `from` towards `to` in steps no longer than the smallest
/// clearance (a pad cannot pass a track within one step). After every step the copper of `geo`
/// (on `board` with `from` applied) follows: track ends and vias on moved pads go along and the
/// relaxation lets the rest give way, with the topology kept. A step that leaves a clearance
/// spot that was not there or got worse is taken back and the floes next to the spot are held
/// to account: the floes on the other side of the squeezed copper are shoved along (within
/// `reach` of their place in the file), and a floe that still makes the spot stops for good. So
/// does a floe whose courtyard would run into another part or leave the board. The others go on.
/// `spots` are places whose copper should be awake as well.
#[allow(clippy::too_many_arguments)]
pub fn drift(
    board: &Board,
    floes: &[Floe],
    from: &HashMap<String, Point>,
    to: &HashMap<String, Point>,
    geo: &Geometry,
    spots: &[Point],
    reach: f64,
    sub_cfg: &SubstrateConfig,
    cfg: &RelaxConfig,
) -> Drifted {
    let n = floes.len();
    // what is left of the way of each floe
    let mut left: Vec<Point> = floes.iter().map(|f| f.offset(to) - f.offset(from)).collect();
    let longest = left.iter().map(|d| d.len()).fold(0.0, f64::max);
    let step = board.rules.min_clearance.clamp(0.02, 0.1);
    let planned = (longest / step).ceil() as usize;
    let mut cur = board.with_offsets(from);
    let base_viol = check::check(&cur, geo);
    let base_open: Vec<u32> = {
        let mut nets: Vec<u32> = geo.tracks.iter().map(|t| t.net).collect();
        nets.sort_unstable();
        nets.dedup();
        crate::legal::open_in_copper(&cur, geo, &nets)
            .into_iter()
            .map(|x| x.0)
            .collect()
    };
    let mut out = Drifted {
        geo: geo.clone(),
        offsets: from.clone(),
        steps: 0,
        planned,
        stuck: Vec::new(),
        shoved: Vec::new(),
        moved: 0,
        longest: 0.0,
    };
    let track_rest: Vec<bool> = geo.tracks.iter().map(|t| t.resting).collect();
    let via_rest: Vec<bool> = geo.vias.iter().map(|v| v.resting).collect();
    // far from the floes and the spots the copper sleeps
    let near_spots: Vec<Rect> = spots
        .iter()
        .map(|&s| Rect { min: s, max: s }.grow(AWAKE))
        .collect();
    for t in out.geo.tracks.iter_mut() {
        t.resting = t.resting || !t.pts.iter().any(|&p| near_spots.iter().any(|b| b.contains(p)));
    }
    for v in out.geo.vias.iter_mut() {
        v.resting = v.resting || !near_spots.iter().any(|b| b.contains(v.at));
    }
    let mut frozen = vec![false; n];
    let mut depth = vec![0usize; n];
    let mut tried: Vec<(usize, usize)> = Vec::new();
    let mut orphans: Vec<Point> = Vec::new();
    // floes that wait a step for the floe they shoved
    let mut hold = vec![false; n];
    let mut way = vec![0.0f64; n];
    let mut total = 0.0;
    let owner: HashMap<&String, usize> = floes
        .iter()
        .enumerate()
        .flat_map(|(i, f)| f.refs.iter().map(move |r| (r, i)))
        .collect();
    let net_name = |x: Option<u32>| x.map_or("fixed copper".to_string(), |x| board.nets[x as usize].name.clone());
    // every failed step shoves one floe more or stops one, so this ends; the bound is a net
    for _ in 0..planned * 2 + n * (CHAIN + 2) + 8 {
        let far = (0..n)
            .filter(|&i| !frozen[i] && !hold[i])
            .map(|i| left[i].len())
            .fold(0.0, f64::max);
        if far < 1e-9 {
            if hold.iter().any(|&h| h) {
                hold.fill(false);
                continue;
            }
            break;
        }
        // all floes arrive together: each takes the same share of what is left of its way
        let share = (step / far).min(1.0);
        let mut d: Vec<Point> = (0..n)
            .map(|i| {
                if frozen[i] || hold[i] {
                    Point::default()
                } else {
                    left[i] * share.min(step / left[i].len().max(1e-12))
                }
            })
            .collect();
        // courtyards: no more overlap than there is, and inside the board
        loop {
            let at = |r: &String, c: &Rect| {
                let m = owner.get(r).map_or(Point::default(), |&i| d[i]);
                Rect {
                    min: c.min + m,
                    max: c.max + m,
                }
            };
            let mut stop: Option<(usize, String)> = None;
            'floes: for i in (0..n).filter(|&i| d[i].len() > 0.0) {
                for r in &floes[i].refs {
                    let Some(c0) = cur.courtyards.get(r) else {
                        continue;
                    };
                    let c1 = at(r, c0);
                    if !corners(&c1).iter().all(|&q| board.inside(q)) {
                        stop = Some((i, "courtyard leaves the board".into()));
                        break 'floes;
                    }
                    let mut others: Vec<(&String, &Rect)> = cur
                        .courtyards
                        .iter()
                        .filter(|(o, _)| !floes[i].refs.contains(o))
                        .collect();
                    others.sort_by(|a, b| a.0.cmp(b.0));
                    if let Some((o, _)) = others
                        .into_iter()
                        .find(|(o, o0)| overlap(&c1, &at(o, o0)) > overlap(c0, o0) + 1e-9)
                    {
                        stop = Some((i, format!("courtyard runs into {o}")));
                        break 'floes;
                    }
                }
            }
            let Some((i, why)) = stop else {
                break;
            };
            frozen[i] = true;
            left[i] = Point::default();
            d[i] = Point::default();
            out.stuck.push((floes[i].refs[0].clone(), why));
        }
        let moving: Vec<usize> = (0..n).filter(|&i| d[i].len() > 0.0).collect();
        if moving.is_empty() {
            continue;
        }
        let mut next_off = out.offsets.clone();
        for &i in &moving {
            for r in &floes[i].refs {
                let e = next_off.entry(r.clone()).or_default();
                *e = *e + d[i];
            }
        }
        let dbg = std::env::var("GB_FLOE_DEBUG").is_ok();
        let t0 = std::time::Instant::now();
        let next = board.with_offsets(&next_off);
        let t1 = t0.elapsed().as_secs_f64();
        let mut g = crate::follow_parts(&cur, &next, &out.geo);
        let t2 = t0.elapsed().as_secs_f64();
        // what the step touches wakes up: the copper around the floes that move
        let boxes: Vec<Rect> = moving.iter().map(|&i| floes[i].bbox(&next).grow(AWAKE)).collect();
        for (k, t) in g.tracks.iter_mut().enumerate() {
            if t.resting
                && (t.pts != out.geo.tracks[k].pts
                    || (!track_rest[k] && t.pts.iter().any(|&p| boxes.iter().any(|b| b.contains(p)))))
            {
                t.resting = false;
            }
        }
        for (k, v) in g.vias.iter_mut().enumerate() {
            if v.resting
                && (v.at.dist(out.geo.vias[k].at) > 1e-12 || (!via_rest[k] && boxes.iter().any(|b| b.contains(v.at))))
            {
                v.resting = false;
            }
        }
        let t3 = t0.elapsed().as_secs_f64();
        let sub = Substrate::build(&next, sub_cfg);
        let t4 = t0.elapsed().as_secs_f64();
        relax::relax(&next, &sub, &mut g, cfg);
        let t5 = t0.elapsed().as_secs_f64();
        let viol = check::check(&next, &g);
        if dbg {
            eprintln!(
                "step: offsets {:.3} follow {:.3} wake {:.3} substrate {:.3} relax {:.3} check {:.3}, awake {} of {}",
                t1,
                t2 - t1,
                t3 - t2,
                t4 - t3,
                t5 - t4,
                t0.elapsed().as_secs_f64() - t5,
                g.tracks.iter().filter(|t| !t.resting).count(),
                g.tracks.len()
            );
        }
        let longest_move = moving.iter().map(|&i| d[i].len()).fold(0.0, f64::max);
        let mut fresh = crate::follow::new_problems(&base_viol, &viol, total + longest_move);
        // copper that came apart: a track end that lost its pad, a via that left its track
        let mut nets: Vec<u32> = g.tracks.iter().filter(|t| !t.resting).map(|t| t.net).collect();
        nets.sort_unstable();
        nets.dedup();
        for (net, pins) in crate::legal::open_in_copper(&next, &g, &nets) {
            if base_open.contains(&net) {
                continue;
            }
            if std::env::var("GB_FLOE_DEBUG").is_ok() {
                eprintln!(
                    "apart: {} pins {:?}",
                    board.nets[net as usize].name,
                    pins.iter().map(|&p| &next.pins[p].name).collect::<Vec<_>>()
                );
                for (k, t) in g.tracks.iter().enumerate().filter(|(_, t)| t.net == net) {
                    let o = &out.geo.tracks[k];
                    eprintln!(
                        "  track {k} layer {} w {} {:?}..{:?} was {:?}..{:?} tap {:?}",
                        t.layer,
                        t.width,
                        t.pts[0],
                        t.pts[t.pts.len() - 1],
                        o.pts[0],
                        o.pts[o.pts.len() - 1],
                        t.tap
                    );
                }
                for (k, v) in g.vias.iter().enumerate().filter(|(_, v)| v.net == net) {
                    eprintln!("  via {k} {:?} was {:?} d {} half {:?}", v.at, out.geo.vias[k].at, v.diameter, v.half);
                }
                for pn in next.pins.iter().filter(|p| p.net == net) {
                    eprintln!("  pin {} {:?}", pn.name, pn.pos);
                }
            }
            fresh.push(check::Spot {
                at: pins.first().map_or(Point::default(), |&p| next.pins[p].pos),
                nets: (net, Some(net)),
                missing: 0.0,
                short: false,
            });
        }
        if fresh.is_empty() {
            for &i in &moving {
                left[i] = left[i] - d[i];
                way[i] += d[i].len();
            }
            total += longest_move;
            hold.fill(false);
            out.steps += 1;
            out.offsets = next_off;
            out.geo = g;
            cur = next;
            continue;
        }
        // the step is not taken; who made the spots answers for them
        // a floe answers once per step, however many spots it made
        let mut answered = vec![false; n];
        for s in &fresh {
            let mut blamed: Vec<usize> = moving
                .iter()
                .copied()
                .filter(|&i| floes[i].nearest(&next, s.at, BLAME).is_some())
                .collect();
            if blamed.is_empty() {
                // pushed copper made it farther away: the floe nearest to it answers once. A
                // spot that comes again is nobody's in particular, and the drift ends there
                if orphans.iter().any(|o| o.dist(s.at) < 1.0) {
                    for &i in &moving {
                        if !frozen[i] {
                            frozen[i] = true;
                            left[i] = Point::default();
                        }
                    }
                    out.stuck.push((
                        "all parts".into(),
                        format!(
                            "a problem far from every part that moves, at ({:.2}, {:.2}): {} and {}",
                            s.at.x,
                            s.at.y,
                            net_name(Some(s.nets.0)),
                            net_name(s.nets.1)
                        ),
                    ));
                    break;
                }
                orphans.push(s.at);
                let dist = |i: usize| {
                    let b = floes[i].bbox(&next);
                    pt(
                        (b.min.x - s.at.x).max(s.at.x - b.max.x).max(0.0),
                        (b.min.y - s.at.y).max(s.at.y - b.max.y).max(0.0),
                    )
                    .len()
                };
                blamed.extend(moving.iter().copied().min_by(|&a, &b| dist(a).total_cmp(&dist(b))));
            }
            for u in blamed {
                if frozen[u] || left[u].len() < 1e-9 || answered[u] {
                    continue;
                }
                answered[u] = true;
                let dir = left[u].norm();
                let cu = floes[u].centre(&next);
                let mut shoved = false;
                if depth[u] < CHAIN {
                    for w in 0..n {
                        if w == u
                            || frozen[w]
                            || tried.contains(&(u, w))
                            || floes[w].nearest(&next, s.at, BLAME).is_none()
                            || (floes[w].centre(&next) - cu).dot(dir) <= 0.0
                        {
                            continue;
                        }
                        tried.push((u, w));
                        // as far along as the floe that comes still wants to go, and a step
                        // more: the copper between the two needs a little room to move in
                        let need = left[u].len() + step - left[w].dot(dir).max(0.0);
                        let to = floes[w].offset(&out.offsets) + left[w] + dir * need;
                        if need < 1e-6 || to.len() > reach {
                            continue;
                        }
                        left[w] = left[w] + dir * need;
                        depth[w] = depth[w].max(depth[u] + 1);
                        if !out.shoved.contains(&floes[w].refs[0]) {
                            out.shoved.push(floes[w].refs[0].clone());
                        }
                        shoved = true;
                    }
                }
                // the floe that was shoved goes first, for one step
                hold[u] = shoved;
                if !shoved {
                    frozen[u] = true;
                    left[u] = Point::default();
                    out.stuck.push((
                        floes[u].refs[0].clone(),
                        format!(
                            "no more room at ({:.2}, {:.2}): {} and {}, {:.3} mm missing",
                            s.at.x,
                            s.at.y,
                            net_name(Some(s.nets.0)),
                            net_name(s.nets.1),
                            s.missing
                        ),
                    ));
                }
            }
        }
    }
    // the copper is as awake as it came
    for (t, &r) in out.geo.tracks.iter_mut().zip(&track_rest) {
        t.resting = r;
    }
    for (v, &r) in out.geo.vias.iter_mut().zip(&via_rest) {
        v.resting = r;
    }
    out.moved = way.iter().filter(|w| **w > 1e-9).count();
    out.longest = way.iter().copied().fold(0.0, f64::max);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::*;
    use crate::geom::{Polygon, Shape, roundrect_polygon};

    fn rect(x: f64, y: f64, w: f64, h: f64) -> Rect {
        Rect {
            min: pt(x, y),
            max: pt(x + w, y + h),
        }
    }

    fn yards() -> Board {
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
        let mut b = yards();
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
        let mut b = yards();
        let (mut cy, mut off) = (b.courtyards.clone(), HashMap::new());
        assert!(shove(&b, "A", pt(1.5, 0.0), &mut cy, &mut off, &mut Vec::new(), 0));
        assert_eq!(off["B"].x, 1.5);
        b.movable.retain(|f| f != "B");
        let (mut cy, mut off) = (b.courtyards.clone(), HashMap::new());
        assert!(!shove(&b, "A", pt(1.5, 0.0), &mut cy, &mut off, &mut Vec::new(), 0));
    }

    /// The board of the tests of [`crate::follow`]: net A as a track from the pad P0 at (2, 5)
    /// to P1 at (8, 5), part Q (net B, one 0.6 mm pad) above it at (5, 3.5). Here the track is
    /// awake, and Q and P1 may move.
    fn board() -> (Board, Geometry) {
        let mut b = crate::follow::tests::board();
        b.soften(&[]);
        b.movable = vec!["Q".into(), "P1".into()];
        b.move_reach = 2.0;
        let mut geo = crate::follow::soft_geometry(&b);
        for t in &mut geo.tracks {
            t.resting = false;
        }
        (b, geo)
    }

    /// One more part with one 0.6 mm pad of a net of its own at `at`.
    fn add_part(b: &mut Board, name: &str, at: Point) {
        let net = b.nets.len() as u32;
        let pin = b.pins.len();
        b.nets.push(Net {
            name: name.into(),
            class: 0,
            pins: vec![pin],
            planes: LayerSet::default(),
            priority: 0,
        });
        let o = b.obstacles.len();
        b.obstacles.push(Obstacle {
            shape: Shape::Polygon(roundrect_polygon(at, 0.6, 0.6, 0.0, 0.0, 0.005)),
            layers: LayerSet::single(0),
            net: Some(net),
            kind: ObstacleKind::Pad,
            hole: None,
            clearance: 0.2,
            pin: Some(pin),
            locked: true,
        });
        b.pins.push(Pin {
            name: format!("{name}.1"),
            footprint: name.into(),
            net,
            pos: at,
            layers: LayerSet::single(0),
            obstacles: vec![o],
            size: (0.6, 0.6),
        });
        b.footprint_obstacles.insert(name.into(), vec![o]);
        b.courtyards.insert(name.into(), rect(at.x - 0.4, at.y - 0.4, 0.8, 0.8));
    }

    fn index(fl: &[Floe], name: &str) -> usize {
        fl.iter().position(|f| f.refs[0] == name).unwrap()
    }

    fn go(b: &Board, geo: &Geometry, to: &[(&str, Point)]) -> Drifted {
        let to: HashMap<String, Point> = to.iter().map(|(r, d)| (r.to_string(), *d)).collect();
        drift(
            b,
            &floes(b),
            &HashMap::new(),
            &to,
            geo,
            &[],
            b.move_reach,
            &SubstrateConfig::default(),
            &RelaxConfig::default(),
        )
    }

    #[test]
    fn a_track_pulls_at_the_part_it_ends_on() {
        let (b, geo) = board();
        let fl = floes(&b);
        let f = forces(&b, &fl, &geo, &[], &[]);
        // P1 carries the right end: the track leaves it to the left
        let p1 = f.pull[index(&fl, "P1")];
        assert!(p1.dist(pt(-1.0, 0.0)) < 1e-9, "{p1:?}");
        let q = index(&fl, "Q");
        assert!(f.pull[q].len() + f.wrap[q].len() + f.press[q].len() < 1e-12);
    }

    #[test]
    fn a_track_that_bends_around_a_pad_presses_on_it() {
        let (mut b, mut geo) = board();
        // Q sits on the line between the pads, the track runs around below it: pad bottom
        // 5.3, clearance 0.2, half the track 0.1
        let down = HashMap::from([("Q".to_string(), pt(0.0, 1.5))]);
        b = b.with_offsets(&down);
        geo.tracks[0].pts = vec![pt(2.0, 5.0), pt(4.6, 5.6), pt(5.4, 5.6), pt(8.0, 5.0)];
        let fl = floes(&b);
        let f = forces(&b, &fl, &geo, &[], &[]);
        let w = f.wrap[index(&fl, "Q")];
        // away from the track, as hard as the track turns: twice the sine of its slope
        assert!(w.x.abs() < 1e-9 && w.y < -0.4 && w.y > -0.5, "{w:?}");
        // and that is how the length of the track changes with the place of Q
        let len = |y: f64| pt(2.0, 5.0).dist(pt(4.6, y)) * 2.0 + 0.8;
        let slope = (len(5.6) - len(5.599)) / 0.001;
        assert!((slope + w.y).abs() < 0.01, "{slope} against {w:?}");
    }

    #[test]
    fn a_spot_pushes_the_parts_around_it_but_not_a_part_against_itself() {
        let (b, geo) = board();
        let fl = floes(&b);
        let q = index(&fl, "Q");
        // a problem between the track of A and the pad of Q, just below the pad
        let spot = Squeeze {
            at: pt(5.0, 4.0),
            nets: (0, Some(1)),
        };
        let f = forces(&b, &fl, &geo, &[spot], &[]);
        assert!(f.press[q].dist(pt(0.0, -1.0)) < 1e-9, "{:?}", f.press[q]);
        // the open pin of Q itself: Q stays, it is its neighbours that make room
        let f = forces(&b, &fl, &geo, &[], &[pt(5.0, 3.5)]);
        assert!(f.press[q].len() < 1e-12);
        // a problem between two pads of one part goes along with the part
        let mut two = b.clone();
        let pin = two.pins.len();
        let o = two.obstacles.len();
        let mut pad = two.obstacles[two.pins[2].obstacles[0]].clone();
        pad.shape = pad.shape.translated(pt(0.7, 0.0));
        pad.net = Some(0);
        pad.pin = Some(pin);
        two.obstacles.push(pad);
        let mut p = two.pins[2].clone();
        p.net = 0;
        p.pos = p.pos + pt(0.7, 0.0);
        p.obstacles = vec![o];
        two.pins.push(p);
        two.footprint_obstacles.get_mut("Q").unwrap().push(o);
        let inner = Squeeze {
            at: pt(5.35, 3.5),
            nets: (0, Some(1)),
        };
        let fl = floes(&two);
        let f = forces(&two, &fl, &geo, &[inner], &[]);
        assert!(f.press.iter().all(|p| p.len() < 1e-12), "{:?}", f.press);
    }

    #[test]
    fn the_forces_become_moves_within_the_reach() {
        let (mut b, _) = board();
        let fl = floes(&b);
        let q = index(&fl, "Q");
        let mut f = Forces {
            press: vec![Point::default(); fl.len()],
            wrap: vec![Point::default(); fl.len()],
            pull: vec![Point::default(); fl.len()],
        };
        f.press[q] = pt(0.0, -2.0);
        let push = Push::of(&b);
        let to = targets(&b, &b, &fl, &HashMap::new(), &f, &push).unwrap();
        assert!(to["Q"].dist(pt(0.0, -push.press_step)) < 1e-9, "{to:?}");
        // a band force below the threshold moves nothing, one above it does, and no farther
        // than the reach
        b.drift = true;
        let push = Push::of(&b);
        f.press[q] = Point::default();
        f.pull[q] = pt(push.band_min * 0.9, 0.0);
        assert!(targets(&b, &b, &fl, &HashMap::new(), &f, &push).is_none());
        f.pull[q] = pt(1.0, 0.0);
        let to = targets(&b, &b, &fl, &HashMap::new(), &f, &push).unwrap();
        assert!(to["Q"].dist(pt(push.band_step, 0.0)) < 1e-9, "{to:?}");
        let at = HashMap::from([("Q".to_string(), pt(b.move_reach - 0.1, 0.0))]);
        let to = targets(&b, &b.with_offsets(&at), &fl, &at, &f, &push).unwrap();
        assert!((to["Q"].x - b.move_reach).abs() < 1e-9, "{to:?}");
        // without drift the bands move nothing
        b.drift = false;
        assert!(targets(&b, &b, &fl, &HashMap::new(), &f, &Push::of(&b)).is_none());
    }

    #[test]
    fn the_copper_gives_way_to_a_drifting_part() {
        let (b, geo) = board();
        let d = go(&b, &geo, &[("Q", pt(0.0, 1.5))]);
        assert!(d.stuck.is_empty(), "{:?}", d.stuck);
        assert!(d.offsets["Q"].dist(pt(0.0, 1.5)) < 1e-9);
        assert_eq!((d.steps, d.moved), (d.planned, 1));
        assert!(check::check(&b.with_offsets(&d.offsets), &d.geo).is_empty());
        assert!(!d.geo.tracks[0].resting, "as awake as it came");
    }

    /// Fixed copper right below the track: Q runs out of room and stops. P1, which moves at the
    /// same time, goes all its way.
    #[test]
    fn only_the_part_that_is_stuck_stops() {
        let (mut b, geo) = board();
        b.obstacles.push(Obstacle {
            shape: Shape::Polygon(Polygon::new(vec![
                pt(3.0, 5.35),
                pt(7.0, 5.35),
                pt(7.0, 6.0),
                pt(3.0, 6.0),
            ])),
            layers: LayerSet::single(0),
            net: None,
            kind: ObstacleKind::Graphic,
            hole: None,
            clearance: 0.2,
            pin: None,
            locked: true,
        });
        let d = go(&b, &geo, &[("Q", pt(0.0, 1.5)), ("P1", pt(0.6, 0.0))]);
        assert!(d.offsets["P1"].dist(pt(0.6, 0.0)) < 1e-9, "{:?}", d.offsets);
        let q = d.offsets["Q"];
        assert!(q.y > 0.3 && q.y < 1.4, "{q:?}");
        assert_eq!(d.stuck.len(), 1, "{:?}", d.stuck);
        assert!(d.stuck[0].0 == "Q" && d.stuck[0].1.contains("no more room"), "{:?}", d.stuck);
        assert!(check::check(&b.with_offsets(&d.offsets), &d.geo).is_empty());
        // the track end went along with P1
        assert!(d.geo.tracks[0].pts.last().unwrap().dist(pt(8.6, 5.0)) < 1e-9);
    }

    /// Part W below the track: where Q comes down, the track is squeezed between the two. A W
    /// that may move is shoved along through the track; a fixed one stops Q.
    #[test]
    fn a_part_shoves_the_part_behind_the_copper() {
        let (mut b, geo) = board();
        add_part(&mut b, "W", pt(5.0, 6.0));
        let fixed = go(&b, &geo, &[("Q", pt(0.0, 1.5))]);
        assert!(fixed.offsets["Q"].y < 1.35, "{:?}", fixed.offsets);
        assert!(fixed.shoved.is_empty() && fixed.stuck.len() == 1);
        b.movable.push("W".into());
        let d = go(&b, &geo, &[("Q", pt(0.0, 1.5))]);
        assert!(d.offsets["Q"].dist(pt(0.0, 1.5)) < 1e-9, "{:?} {:?}", d.offsets, d.stuck);
        assert_eq!(d.shoved, vec!["W".to_string()]);
        let w = d.offsets["W"];
        assert!(w.x.abs() < 1e-9 && w.y > 0.15 && w.y < 0.5, "{w:?}");
        assert!(check::check(&b.with_offsets(&d.offsets), &d.geo).is_empty());
        // not beyond its reach
        b.move_reach = 0.1;
        let d = go(&b, &geo, &[("Q", pt(0.0, 1.5))]);
        assert!(!d.offsets.contains_key("W") && d.offsets["Q"].y < 1.35, "{:?}", d.offsets);
    }

    #[test]
    fn a_courtyard_stops_its_part_alone() {
        let (b, geo) = board();
        // Q down and left onto P0, whose courtyard is in the way; P1 goes on
        let d = go(&b, &geo, &[("Q", pt(-3.0, 1.2)), ("P1", pt(0.6, 0.0))]);
        assert!(d.offsets["P1"].dist(pt(0.6, 0.0)) < 1e-9);
        assert!(d.offsets["Q"].x > -3.0 && d.offsets["Q"].x < -1.5, "{:?}", d.offsets);
        assert!(d.stuck.iter().any(|s| s.0 == "Q" && s.1.contains("P0")), "{:?}", d.stuck);
    }
}
