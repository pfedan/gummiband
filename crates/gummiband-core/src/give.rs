//! The neighbours give way: the last findings of the check, on the finished copper, are pushed
//! off exactly, and what that pushes into goes along.
//!
//! The relaxation balances forces: where a passage is a few micrometres too narrow, it shares
//! what is missing among the gaps, and every gap ends a little short. Here a finding is taken as
//! a distance that must grow by what is missing. The item at the spot (a via, or a track at its
//! nearest point) tries a move in sixteen directions by what is missing and by twice that. A via
//! takes the ends of its tracks along; a track bends by the move at the spot and less and less
//! up to [`REACH`] along it. Then every movable track or via that the moved copper is now too
//! near to gives way in turn (by what it misses, away from the copper that pressed it), up to
//! [`DEPTH`] steps away from the spot. A move is taken only if the findings of everything it
//! touched, measured by the check itself, add up to less than before; shorts weigh more than
//! any distance. So the pass never makes a board worse by its own measure.
//!
//! What never moves: pads, kept copper, the board, soft copper still as the file has it, the
//! tracks of differential pairs and of tuning patterns, vias in pads, via pairs, and the ends of
//! a track (but the end on a via that moves).

use std::collections::{HashMap, HashSet};

use crate::board::{Board, NetId};
use crate::check::{Copper, MARGIN, Violation};
use crate::geom::{Point, Rect, arc_from_three, point_seg_dist, pt};
use crate::realize::{Elem, Geometry, flatten};

/// How far along a track a bend reaches from where it is pushed, in mm.
pub const REACH: f64 = 0.6;
/// How many steps of giving way a move may set off.
pub const DEPTH: usize = 3;
/// The most a move goes in one step, in mm.
const MAX_STEP: f64 = 0.15;
/// Rounds over the findings.
const ROUNDS: usize = 6;
/// Room around a spot in which the findings are weighed, beyond the reach of a bend.
const AROUND: f64 = 1.0;

/// What the pass did.
#[derive(Debug, Clone, Default, serde::Serialize)]
pub struct Report {
    /// Clearance findings before and after.
    pub before: usize,
    pub after: usize,
    /// Moves taken, and the vias and tracks they moved.
    pub moves: usize,
    pub vias: usize,
    pub tracks: usize,
    pub rounds: usize,
}

/// The weight of a set of findings: what is missing, in mm, a short as 1 mm, and 5 µm for every
/// finding, so that a move that turns one finding into two small ones must save more than that.
fn weight(f: &[Violation]) -> f64 {
    // the pieces of one track near a via are many findings of one problem: per other net and
    // kind the worst counts
    let mut worst: HashMap<(Option<NetId>, &str), f64> = HashMap::new();
    for v in f.iter().filter(|v| !v.is_width()) {
        let w = (v.required - v.actual).max(0.0) + if v.actual <= 0.0 { 1.0 } else { 0.0 } + 0.005;
        let kind = v.what.split(' ').next().unwrap_or("");
        let e = worst.entry((v.nets.1, kind)).or_insert(0.0);
        *e = e.max(w);
    }
    worst.values().sum()
}

/// Something that can move.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum Item {
    Via(usize),
    Track(usize),
}

struct Pass<'a> {
    _board: &'a Board,
    geo: &'a mut Geometry,
    copper: Copper<'a>,
    /// Per via the tracks that end on it: (track, at its start).
    ends: Vec<Vec<(usize, bool)>>,
    /// Movable tracks and vias by net.
    tracks_of: HashMap<NetId, Vec<usize>>,
    vias_of: HashMap<NetId, Vec<usize>>,
    via_ok: Vec<bool>,
    track_ok: Vec<bool>,
    /// The box around every track, kept with its bends.
    boxes: Vec<Rect>,
}

fn bbox(pts: &[Point]) -> Rect {
    let mut r = Rect::empty();
    for &p in pts {
        r.add(p);
    }
    r
}

/// A change of one item, to take back.
enum Undo {
    Via(usize, Point),
    Track(usize, Vec<Point>, Vec<Elem>),
}

impl<'a> Pass<'a> {
    fn new(board: &'a Board, geo: &'a mut Geometry) -> Pass<'a> {
        let copper = Copper::new(board, geo);
        let track_ok: Vec<bool> = geo
            .tracks
            .iter()
            .map(|t| !t.resting && t.pattern.is_none() && t.pts.len() >= 2 && !board.in_diff_pair(t.net))
            .collect();
        let mut ends = vec![Vec::new(); geo.vias.len()];
        let mut at: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
        let key = |p: Point| ((p.x * 1e4).round() as i64, (p.y * 1e4).round() as i64);
        for (vi, v) in geo.vias.iter().enumerate() {
            at.entry(key(v.at)).or_default().push(vi);
        }
        for (ti, t) in geo.tracks.iter().enumerate() {
            for (p, start) in [(t.pts[0], true), (*t.pts.last().unwrap(), false)] {
                for &vi in at.get(&key(p)).into_iter().flatten() {
                    if geo.vias[vi].net == t.net && geo.vias[vi].at.dist(p) < 1e-4 {
                        ends[vi].push((ti, start));
                    }
                }
            }
        }
        let via_ok: Vec<bool> = geo
            .vias
            .iter()
            .enumerate()
            .map(|(vi, v)| {
                v.drill > 0.0
                    && !v.resting
                    // a via the board holds as well is kept copper
                    && !board.obstacles.iter().any(|o| {
                        o.hole.is_some() && o.net == Some(v.net) && o.shape.bbox().center().dist(v.at) < 1e-3
                    })
                    && !v.is_pair()
                    && !board.in_diff_pair(v.net)
                    && ends[vi].iter().all(|&(ti, _)| track_ok[ti])
                    // a via in a pad of its net stays in the pad
                    && !board.nets[v.net as usize]
                        .pins
                        .iter()
                        .any(|&p| board.pins[p].pos.dist(v.at) < v.diameter / 2.0)
            })
            .collect();
        let mut tracks_of: HashMap<NetId, Vec<usize>> = HashMap::new();
        for (ti, t) in geo.tracks.iter().enumerate() {
            if track_ok[ti] {
                tracks_of.entry(t.net).or_default().push(ti);
            }
        }
        let mut vias_of: HashMap<NetId, Vec<usize>> = HashMap::new();
        for (vi, v) in geo.vias.iter().enumerate() {
            if via_ok[vi] {
                vias_of.entry(v.net).or_default().push(vi);
            }
        }
        let boxes = geo.tracks.iter().map(|t| bbox(&t.pts)).collect();
        Pass {
            boxes,
            _board: board,
            geo,
            copper,
            ends,
            tracks_of,
            vias_of,
            via_ok,
            track_ok,
        }
    }

    /// The findings of an item within a region.
    fn findings(&self, it: Item, within: &Rect) -> Vec<Violation> {
        match it {
            Item::Via(vi) => self.copper.via_findings(vi),
            Item::Track(ti) => self.copper.track_findings(ti, within),
        }
    }

    fn weigh(&self, items: &HashSet<Item>, within: &Rect) -> f64 {
        items.iter().map(|&it| weight(&self.findings(it, within))).sum()
    }

    /// Bend track ti by d at arc length s0 (falling off to nothing at REACH along it); with
    /// `free_end` that end goes along by d in full. The ends stay otherwise.
    fn bend(&mut self, ti: usize, s0: f64, d: Point, free_end: Option<bool>, undo: &mut Vec<Undo>) -> bool {
        let t = &self.geo.tracks[ti];
        let Some(elems) = bent(&t.elems, s0, d, REACH, free_end) else {
            return false;
        };
        let pts = flatten(&elems);
        if pts.len() < 2 {
            return false;
        }
        undo.push(Undo::Track(ti, t.pts.clone(), t.elems.clone()));
        self.boxes[ti] = bbox(&pts);
        let t = &mut self.geo.tracks[ti];
        t.pts = pts;
        t.elems = elems;
        let t = &self.geo.tracks[ti];
        self.copper.set_track(ti, t, &t.pts, &t.elems);
        true
    }

    /// Move via vi by d with the ends of its tracks, and with the vias of its net at the same
    /// place (the geometry may hold one via twice, for two pairs of layers).
    fn shift_via(&mut self, vi: usize, d: Point, undo: &mut Vec<Undo>) {
        let at = self.geo.vias[vi].at;
        let net = self.geo.vias[vi].net;
        let twins: Vec<usize> = (0..self.geo.vias.len())
            .filter(|&k| self.geo.vias[k].net == net && self.geo.vias[k].at.dist(at) < 1e-3)
            .collect();
        for k in twins {
            undo.push(Undo::Via(k, self.geo.vias[k].at));
            self.geo.vias[k].at = self.geo.vias[k].at + d;
            if self.geo.vias[k].drill > 0.0 {
                let v = self.geo.vias[k].clone();
                self.copper.set_via(k, &v);
            }
            for (ti, start) in self.ends[k].clone() {
                let s0 = if start {
                    0.0
                } else {
                    arc_length(&self.geo.tracks[ti].elems)
                };
                self.bend(ti, s0, d, Some(start), undo);
            }
        }
    }

    /// Take back a tried move: the geometry by its undo list, the copper by its journal.
    fn take_back(&mut self, undo: Vec<Undo>) {
        for u in undo.into_iter().rev() {
            match u {
                Undo::Via(vi, at) => self.geo.vias[vi].at = at,
                Undo::Track(ti, pts, elems) => {
                    self.boxes[ti] = bbox(&pts);
                    let t = &mut self.geo.tracks[ti];
                    t.pts = pts;
                    t.elems = elems;
                }
            }
        }
        self.copper.rollback();
    }

    /// The movable items of net `net` near p: vias whose copper is within `r`, tracks of
    /// `layer` (any layer: None) within `r`.
    fn near(&self, net: NetId, p: Point, r: f64, layer: Option<u8>) -> Vec<Item> {
        let mut out = Vec::new();
        for &vi in self.vias_of.get(&net).into_iter().flatten() {
            let v = &self.geo.vias[vi];
            if v.at.dist(p) < v.diameter / 2.0 + r {
                out.push(Item::Via(vi));
            }
        }
        for &ti in self.tracks_of.get(&net).into_iter().flatten() {
            let t = &self.geo.tracks[ti];
            if layer.is_some_and(|l| l != t.layer) || !self.boxes[ti].grow(t.width / 2.0 + r).contains(p) {
                continue;
            }
            if nearest(&t.pts, p).1 < t.width / 2.0 + r {
                out.push(Item::Track(ti));
            }
        }
        out
    }

    /// Push item `it` away from p by `amount` (a via straight away, a track at its point nearest
    /// to p).
    fn push(&mut self, it: Item, p: Point, amount: f64, fallback: Point, undo: &mut Vec<Undo>) -> bool {
        match it {
            Item::Via(vi) => {
                let c = self.geo.vias[vi].at;
                let dir = if c.dist(p) > 1e-6 { (c - p).norm() } else { fallback };
                self.shift_via(vi, dir * amount, undo);
                true
            }
            Item::Track(ti) => {
                let (q, _, s) = nearest(&self.geo.tracks[ti].pts, p);
                let dir = if q.dist(p) > 1e-6 { (q - p).norm() } else { fallback };
                self.bend(ti, s, dir * amount, None, undo)
            }
        }
    }

    /// One move of `first` by d and the giving way it sets off; the items it touched.
    fn try_move(&mut self, first: Item, at: Point, d: Point, within: &Rect, undo: &mut Vec<Undo>) -> HashSet<Item> {
        let mut touched: HashSet<Item> = HashSet::new();
        match first {
            Item::Via(vi) => self.shift_via(vi, d, undo),
            Item::Track(ti) => {
                let s = nearest(&self.geo.tracks[ti].pts, at).2;
                if !self.bend(ti, s, d, None, undo) {
                    return touched;
                }
            }
        }
        touched.insert(first);
        if let Item::Via(vi) = first {
            for &(ti, _) in &self.ends[vi] {
                touched.insert(Item::Track(ti));
            }
        }
        let mut frontier: Vec<Item> = touched.iter().copied().collect();
        let fallback = if d.len() > 0.0 { d.norm() } else { pt(1.0, 0.0) };
        for _ in 0..DEPTH {
            let mut next = Vec::new();
            for &it in &frontier {
                for f in self.findings(it, within) {
                    let Some(other) = f.nets.1 else { continue };
                    if f.is_width() || other == f.nets.0 {
                        continue;
                    }
                    let layer = match it {
                        Item::Track(ti) => Some(self.geo.tracks[ti].layer),
                        Item::Via(_) => None,
                    };
                    let miss = (f.required - f.actual).max(0.0) + MARGIN;
                    for o in self.near(other, f.at, f.required + 0.3, layer) {
                        if touched.contains(&o) {
                            continue;
                        }
                        if self.push(o, f.at, miss.min(MAX_STEP), fallback, undo) {
                            touched.insert(o);
                            if let Item::Via(vi) = o {
                                for &(ti, _) in &self.ends[vi] {
                                    touched.insert(Item::Track(ti));
                                }
                            }
                            next.push(o);
                        }
                    }
                }
            }
            if next.is_empty() {
                break;
            }
            frontier = next;
        }
        touched
    }

    /// The best move for the spot of a finding of `first` (missing `miss`), taken if it pays.
    fn settle(&mut self, first: Item, at: Point, miss: f64) -> bool {
        let within = Rect { min: at, max: at }.grow(REACH + AROUND);
        let step = (miss + MARGIN).min(MAX_STEP);
        let mut best: Option<(f64, Point)> = None;
        for k in 0..16 {
            let a = k as f64 * std::f64::consts::PI / 8.0;
            let (s, c) = crate::math::sin_cos(a);
            let dir = pt(c, s);
            for f in [1.0, 2.0] {
                let d = dir * (step * f).min(MAX_STEP);
                let mut undo = Vec::new();
                self.copper.begin();
                let touched = self.try_move(first, at, d, &within, &mut undo);
                if touched.is_empty() {
                    self.take_back(undo);
                    continue;
                }
                let after = self.weigh(&touched, &within);
                self.take_back(undo);
                let before = self.weigh(&touched, &within);
                let gain = before - after;
                if gain > 1e-5 && best.is_none_or(|b| gain > b.0) {
                    best = Some((gain, d));
                }
            }
        }
        let Some((_, d)) = best else { return false };
        let mut undo = Vec::new();
        self.try_move(first, at, d, &within, &mut undo);
        self.copper.commit();
        true
    }
}

/// Let the neighbours give way at the findings of the check (see the module).
pub fn give_way(board: &Board, geo: &mut Geometry) -> Report {
    let clearance = |v: &Violation| !v.is_width();
    let before = crate::check::check(board, geo).iter().filter(|v| clearance(v)).count();
    let mut report = Report {
        before,
        after: before,
        ..Default::default()
    };
    if before == 0 {
        return report;
    }
    let mut moved_vias: HashSet<usize> = HashSet::new();
    let mut moved_tracks: HashSet<usize> = HashSet::new();
    for round in 0..ROUNDS {
        let found: Vec<Violation> = crate::check::check(board, geo)
            .into_iter()
            .filter(|v| clearance(v))
            .collect();
        if found.is_empty() {
            break;
        }
        report.rounds = round + 1;
        // the worst finding of each spot
        let spots = crate::check::spots(&found);
        let mut pass = Pass::new(board, geo);
        let mut moves = 0;
        for s in &spots {
            if s.short {
                continue;
            }
            // what can move at the spot: a via first, then a track, of either net
            let nets = [Some(s.nets.0), s.nets.1];
            let mut cands: Vec<Item> = Vec::new();
            for n in nets.into_iter().flatten() {
                cands.extend(pass.near(n, s.at, 0.3, None));
            }
            cands.sort_by_key(|it| match it {
                Item::Via(_) => 0,
                Item::Track(_) => 1,
            });
            cands.dedup();
            for it in cands.into_iter().take(3) {
                let ok = match it {
                    Item::Via(vi) => pass.via_ok[vi],
                    Item::Track(ti) => pass.track_ok[ti],
                };
                if ok && pass.settle(it, s.at, s.missing) {
                    moves += 1;
                    match it {
                        Item::Via(vi) => {
                            moved_vias.insert(vi);
                        }
                        Item::Track(ti) => {
                            moved_tracks.insert(ti);
                        }
                    }
                    break;
                }
            }
        }
        report.moves += moves;
        if moves == 0 {
            break;
        }
    }
    report.vias = moved_vias.len();
    report.tracks = moved_tracks.len();
    report.after = crate::check::check(board, geo).iter().filter(|v| clearance(v)).count();
    report
}

/// The point of the polyline nearest to p, its distance and its arc length.
fn nearest(pts: &[Point], p: Point) -> (Point, f64, f64) {
    let mut best = (pts[0], pts[0].dist(p), 0.0);
    let mut s = 0.0;
    for w in pts.windows(2) {
        let (a, b) = (w[0], w[1]);
        let l = a.dist(b);
        let t = if l > 1e-12 {
            ((p - a).dot(b - a) / (l * l)).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let q = a.lerp(b, t);
        let d = point_seg_dist(p, a, b);
        if d < best.1 {
            best = (q, d, s + t * l);
        }
        s += l;
    }
    best
}

fn elem_len(e: &Elem) -> f64 {
    match *e {
        Elem::Line(a, b) => a.dist(b),
        Elem::Arc(..) => flatten(std::slice::from_ref(e))
            .windows(2)
            .map(|w| w[0].dist(w[1]))
            .sum(),
    }
}

fn arc_length(elems: &[Elem]) -> f64 {
    elems.iter().map(elem_len).sum()
}

/// The elements of a track bent by d at arc length s0, by less and less up to `reach` along it
/// on both sides. Lines are cut where the bend starts, peaks and ends; an arc moves its three
/// points (and stays an arc where it can). The ends do not move, but the one `free_end` names
/// (true: the start), which goes along in full. None if nothing moves.
fn bent(elems: &[Elem], s0: f64, d: Point, reach: f64, free_end: Option<bool>) -> Option<Vec<Elem>> {
    let total = arc_length(elems);
    // cut the lines at the arc lengths of the bend
    let cuts = [s0 - reach, s0, s0 + reach];
    let mut cut: Vec<Elem> = Vec::with_capacity(elems.len() + 3);
    let mut s = 0.0;
    for e in elems {
        let l = elem_len(e);
        match *e {
            Elem::Line(a, b) if l > 1e-9 => {
                let mut from = a;
                for &c in &cuts {
                    if c > s + 1e-6 && c < s + l - 1e-6 {
                        let p = a.lerp(b, (c - s) / l);
                        cut.push(Elem::Line(from, p));
                        from = p;
                    }
                }
                cut.push(Elem::Line(from, b));
            }
            _ => cut.push(*e),
        }
        s += l;
    }
    let w = |s: f64| -> f64 {
        if s < 1e-9 {
            return if free_end == Some(true) { 1.0 } else { 0.0 };
        }
        if s > total - 1e-9 {
            return if free_end == Some(false) { 1.0 } else { 0.0 };
        }
        (1.0 - (s - s0).abs() / reach).max(0.0)
    };
    let mut out = Vec::with_capacity(cut.len());
    let mut s = 0.0;
    let mut any = false;
    for e in &cut {
        let l = elem_len(e);
        match *e {
            Elem::Line(a, b) => {
                let (wa, wb) = (w(s), w(s + l));
                any |= wa > 0.0 || wb > 0.0;
                out.push(Elem::Line(a + d * wa, b + d * wb));
            }
            Elem::Arc(a, m, b) => {
                let (wa, wm, wb) = (w(s), w(s + l / 2.0), w(s + l));
                any |= wa > 0.0 || wm > 0.0 || wb > 0.0;
                let (a2, m2, b2) = (a + d * wa, m + d * wm, b + d * wb);
                if arc_from_three(a2, m2, b2).is_some() {
                    out.push(Elem::Arc(a2, m2, b2));
                } else {
                    out.push(Elem::Line(a2, m2));
                    out.push(Elem::Line(m2, b2));
                }
            }
        }
        s += l;
    }
    // (a line that the cut left without length goes)
    out.retain(|e| !matches!(e, Elem::Line(a, b) if a.dist(*b) < 1e-9));
    any.then_some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_bend_moves_the_middle_and_keeps_the_ends() {
        let elems = vec![Elem::Line(pt(0.0, 0.0), pt(4.0, 0.0))];
        let e = bent(&elems, 2.0, pt(0.0, 0.1), 0.6, None).unwrap();
        let pts = flatten(&e);
        assert_eq!(pts[0], pt(0.0, 0.0));
        assert_eq!(*pts.last().unwrap(), pt(4.0, 0.0));
        assert!(pts.iter().any(|p| (p.x - 2.0).abs() < 1e-9 && (p.y - 0.1).abs() < 1e-9));
        assert!(pts.iter().all(|p| p.y >= 0.0 && p.y <= 0.1 + 1e-12));
    }

    #[test]
    fn a_free_end_goes_along_in_full() {
        let elems = vec![Elem::Line(pt(0.0, 0.0), pt(4.0, 0.0))];
        let e = bent(&elems, 0.0, pt(0.05, 0.05), 0.6, Some(true)).unwrap();
        let pts = flatten(&e);
        assert!(pts[0].dist(pt(0.05, 0.05)) < 1e-12);
        assert_eq!(*pts.last().unwrap(), pt(4.0, 0.0));
    }
}
