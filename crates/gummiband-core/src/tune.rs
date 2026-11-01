//! Length tuning: meanders that bring nets to the lengths their `length` and `skew` rules ask for.
//!
//! Works on a finished routing. The aims come from the rules of the board as KiCad groups them
//! ([`crate::board::MatchRule`]): a `length` rule lifts a net that is too short into its window,
//! a `skew` rule lifts every net of its group that lies more than the limit below the longest
//! one. Nets are only lengthened, never shortened.
//!
//! A meander is a row of bumps on a straight line of the track, each one up, across and down
//! again, round (semicircles, as KiCad's round trombone) or with 45 degree corners (for
//! `--angles 45`). Neighbouring legs keep `spacing` track widths of air (at least the clearance).
//! Each bump goes to the side with more room and as high as its clearance allows, up to
//! `amplitude`; the heights are then lowered evenly until the gain is what the net needs.
//! A differential pair whose two nets both need length is meandered as one band: the bumps are
//! laid out for the centre line between the two tracks and both tracks follow them at their
//! distance, so the pair stays coupled and its skew does not change.

use std::collections::HashMap;

use crate::board::{Board, LayerId, MatchKind, NetId};
use crate::geom::{Point, Rect};
use crate::matching::{self, elem_len};
use crate::realize::{Elem, Geometry, flatten};

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Style {
    /// Semicircle turns.
    Round,
    /// 45 degree corners: every piece at a multiple of 45 degrees to the original line.
    Chamfer,
}

#[derive(Debug, Clone)]
pub struct TuneConfig {
    pub style: Style,
    /// Highest bump: centre line above the original line, mm.
    pub amplitude: f64,
    /// Air between neighbouring legs of a meander in track widths (at least the clearance).
    pub spacing: f64,
}

impl Default for TuneConfig {
    fn default() -> Self {
        TuneConfig {
            style: Style::Round,
            amplitude: 1.0,
            spacing: 2.0,
        }
    }
}

/// What the tuning did for one net.
#[derive(Debug, Clone, Default, serde::Serialize)]
pub struct NetTune {
    pub net: String,
    /// Length before and after (as KiCad measures it), and the length aimed at.
    pub before: f64,
    pub after: f64,
    pub aim: f64,
    /// Tuned together with its pair partner.
    pub coupled: bool,
}

/// One meander, written as a KiCad `tuning_pattern` group: KiCad shows it as a tuning pattern
/// that can be edited, and leaves its tracks out of the diff pair coupling check.
#[derive(Debug, Clone, serde::Serialize)]
pub struct Pattern {
    /// The net, or the P and N net of a pair.
    pub nets: Vec<NetId>,
    pub layer: LayerId,
    /// The straight line the meander replaces, per net.
    pub base: Vec<(Point, Point)>,
    /// The meander tracks in the geometry, per net.
    pub tracks: Vec<usize>,
    pub rounded: bool,
    pub width: f64,
    /// Gap of the pair (0 for a single net).
    pub gap: f64,
    pub amplitude: f64,
    pub spacing: f64,
    /// Length aimed at.
    pub aim: f64,
}

#[derive(Debug, Clone, Default, serde::Serialize)]
pub struct TuneReport {
    pub nets: Vec<NetTune>,
    pub patterns: Vec<Pattern>,
}

/// Remaining length below which a net counts as tuned.
const TOL: f64 = 0.005;

/// Lengths the rules ask for: per net the length to reach (only nets that are too short).
pub fn aims(board: &Board, geo: &Geometry) -> HashMap<NetId, f64> {
    let mut len: HashMap<NetId, f64> = HashMap::new();
    for r in &board.match_rules {
        for &n in &r.nets {
            len.entry(n)
                .or_insert_with(|| matching::net_length(board, geo, n).total());
        }
    }
    let mut aim: HashMap<NetId, f64> = HashMap::new();
    // length windows first: they decide how long the longest net of a group is
    for r in board.match_rules.iter().filter(|r| r.kind == MatchKind::Length) {
        let Some(min) = r.value.min.or(r.value.opt) else {
            continue;
        };
        let want = match (r.value.opt, r.value.max) {
            (Some(o), _) if o >= min => o,
            (_, Some(max)) => (min + max) / 2.0,
            _ => min + 0.02,
        };
        for &n in &r.nets {
            if len[&n] < min {
                let a = aim.entry(n).or_insert(0.0);
                *a = a.max(want);
            }
        }
    }
    let after = |n: NetId, aim: &HashMap<NetId, f64>| len[&n].max(aim.get(&n).copied().unwrap_or(0.0));
    for r in board.match_rules.iter().filter(|r| r.kind == MatchKind::Skew) {
        let Some(max) = r.value.max else { continue };
        let groups: Vec<Vec<NetId>> = if r.within_pairs {
            board
                .diff_pairs()
                .into_iter()
                .filter(|(p, n)| r.nets.contains(p) && r.nets.contains(n))
                .map(|(p, n)| vec![p, n])
                .collect()
        } else {
            vec![r.nets.clone()]
        };
        for g in groups {
            let target = g.iter().map(|&n| after(n, &aim)).fold(0.0, f64::max);
            for &n in &g {
                if target - after(n, &aim) > max - matching_eps() {
                    let a = aim.entry(n).or_insert(0.0);
                    *a = a.max(target - max * 0.25);
                }
            }
        }
    }
    aim
}

/// Longest a net with an aim may become: below the `max` of its length windows and not past the
/// longest net of its skew groups (within-pair skew rules aside, a band does not change those).
/// A pair tuned as a band may lengthen its two nets by different amounts; this is how far.
pub fn ceiling(board: &Board, geo: &Geometry, aims: &HashMap<NetId, f64>, net: NetId) -> f64 {
    let mut cap = f64::INFINITY;
    for r in board.match_rules.iter().filter(|r| r.nets.contains(&net)) {
        match r.kind {
            MatchKind::Length => {
                if let Some(max) = r.value.max {
                    cap = cap.min(max - matching_eps());
                }
            }
            MatchKind::Skew if !r.within_pairs => {
                let target = r
                    .nets
                    .iter()
                    .map(|&n| {
                        let l = matching::net_length(board, geo, n).total();
                        l.max(aims.get(&n).copied().unwrap_or(0.0))
                    })
                    .fold(0.0, f64::max);
                cap = cap.min(target);
            }
            _ => {}
        }
    }
    cap.max(aims.get(&net).copied().unwrap_or(0.0))
}

fn matching_eps() -> f64 {
    0.0005
}

/// Turn radius of a meander of a track of width `w`: legs `2r` apart.
fn turn_radius(board: &Board, net: NetId, w: f64, cfg: &TuneConfig) -> f64 {
    (w + (cfg.spacing * w).max(board.class_of(net).clearance)) / 2.0
}

/// How much wider the tracks of each net that wants length should push, so that a meander finds
/// room next to them: twice the bump height that gives the missing length on the straight lines
/// the net has (a bump goes to one side, the push makes room on both). A pair in `bands` (both
/// nets want length, the band is one track while the room is made) gets it under its P net;
/// other pair nets are left out, their two tracks would push each other apart.
pub fn room_wanted(board: &Board, geo: &Geometry, cfg: &TuneConfig, bands: &[(NetId, NetId)]) -> HashMap<NetId, f64> {
    let aims = aims(board, geo);
    let in_pair: Vec<NetId> = board.diff_pairs().into_iter().flat_map(|(p, n)| [p, n]).collect();
    let need_of = |n: NetId| aims.get(&n).map(|&a| a - matching::net_length(board, geo, n).total());
    // height of the bumps that gain `need` on `room` mm of slots with turn radius r
    let height = |need: f64, room: f64, r: f64| {
        let h = (4.0 * r * need / room - (2.0 * std::f64::consts::PI - 8.0) * r) / 2.0;
        h.clamp(2.0 * r, cfg.amplitude.max(2.0 * r))
    };
    let slots = |len: f64, r: f64| {
        let margin = (2.0 * r).max(0.3);
        let k = ((len - 2.0 * margin) / (4.0 * r)).floor();
        if k >= 1.0 { k * 4.0 * r } else { 0.0 }
    };
    let mut out = HashMap::new();
    let mut nets: Vec<NetId> = aims.keys().copied().collect();
    nets.sort();
    for net in nets {
        if in_pair.contains(&net) {
            continue;
        }
        let Some(need) = need_of(net).filter(|&x| x > TOL) else {
            continue;
        };
        let Some(w) = geo.tracks.iter().find(|t| t.net == net).map(|t| t.width) else {
            continue;
        };
        let r = turn_radius(board, net, w, cfg);
        let room: f64 = geo
            .tracks
            .iter()
            .filter(|t| t.net == net)
            .flat_map(|t| &t.elems)
            .map(|e| match *e {
                Elem::Line(a, b) => slots(a.dist(b), r),
                Elem::Arc(..) => 0.0,
            })
            .sum();
        if room > 0.0 {
            out.insert(net, 2.0 * height(need, room, r));
        }
    }
    for &(p, n) in bands {
        let (Some(np), Some(nn)) = (need_of(p), need_of(n)) else {
            continue;
        };
        let need = (np + nn) / 2.0;
        if need <= TOL {
            continue;
        }
        let sides = sides(geo, p, n);
        let Some(first) = sides.first() else { continue };
        let w = geo.tracks[first.tp].width.max(geo.tracks[first.tn].width);
        let d = first.off.abs();
        let r = (d + w + (cfg.spacing * w).max(board.class_of(p).clearance)) / 2.0;
        let room: f64 = sides.iter().map(|s| slots(s.s1 - s.s0, r)).sum();
        if room > 0.0 {
            out.insert(p, 2.0 * height(need, room, r));
        }
    }
    out
}

/// Push the copper around the tracks of the nets in `extra` aside, as if those tracks were wider
/// by `extra`: room for their meanders. Pushes without tension, in chains as far as that goes; a
/// spot without room keeps the old width (a neck). Nothing may get worse at the real widths:
/// None if the clearance check finds more spots or shorts than before.
pub fn make_room(
    board: &Board,
    sub: &crate::topo::Substrate,
    geo: &Geometry,
    extra: &HashMap<NetId, f64>,
) -> Option<Geometry> {
    make_room_judged(board, sub, geo, extra, &|g| {
        crate::check::spots(&crate::check::check(board, g))
    })
}

/// [`make_room`] with the clearance spots of a geometry as `judge` finds them (for a board
/// whose pairs are one wide track: the spots of the two tracks they become).
pub fn make_room_judged(
    board: &Board,
    sub: &crate::topo::Substrate,
    geo: &Geometry,
    extra: &HashMap<NetId, f64>,
    judge: &dyn Fn(&Geometry) -> Vec<crate::check::Spot>,
) -> Option<Geometry> {
    if extra.is_empty() {
        return None;
    }
    let spots = judge;
    let count = |s: &[crate::check::Spot]| (s.len(), s.iter().filter(|s| s.short).count());
    let before = count(&spots(geo));
    // all nets at once; where that leaves a new spot, each half on its own on top of what
    // the other half got, down to single nets, within a budget of pushes
    let mut nets: Vec<NetId> = extra.keys().copied().collect();
    nets.sort();
    let mut todo: Vec<Vec<NetId>> = vec![nets];
    let mut cur = geo.clone();
    let mut made = false;
    let mut budget = 12;
    while let Some(set) = todo.pop() {
        if budget == 0 {
            break;
        }
        budget -= 1;
        let part: HashMap<NetId, f64> = set.iter().map(|n| (*n, extra[n])).collect();
        let g = push_aside(board, sub, &cur, &part);
        let after = count(&spots(&g));
        if std::env::var("GB_TUNE_LOG").is_ok() {
            eprintln!("room: {} nets, spots {before:?} before, {after:?} after", set.len());
        }
        if after.0 <= before.0 && after.1 <= before.1 {
            cur = g;
            made = true;
        } else if set.len() > 1 {
            let (x, y) = set.split_at(set.len() / 2);
            todo.push(y.to_vec());
            todo.push(x.to_vec());
        }
    }
    made.then_some(cur)
}

fn push_aside(board: &Board, sub: &crate::topo::Substrate, geo: &Geometry, extra: &HashMap<NetId, f64>) -> Geometry {
    let count = |g: &Geometry| crate::check::spots(&crate::check::check(board, g)).len();
    let before = count(geo);
    let mut g = geo.clone();
    let plain: Vec<(f64, Vec<Rect>, f64)> = g.tracks.iter().map(|t| (t.width, t.necks.clone(), t.neck_w)).collect();
    for t in &mut g.tracks {
        if let Some(&e) = extra.get(&t.net) {
            t.neck_w = t.neck_w.min(t.width);
            // the ends stay narrow: they hang on pads and vias between other pins
            let reach = e + 0.5;
            for p in [t.pts[0], t.pts[t.pts.len() - 1]] {
                let mut zone = Rect::empty();
                zone.add(p);
                t.necks.push(zone.grow(reach));
            }
            // and so do the stretches next to pads: those do not step aside, and the
            // neighbours that pass them have nowhere to go
            let near = e / 2.0 + t.width / 2.0 + board.class_of(t.net).clearance + 0.2;
            let mut zones = Vec::new();
            for w in t.pts.windows(2) {
                let mut bb = Rect::empty();
                bb.add(w[0]);
                bb.add(w[1]);
                let hit = board.obstacles.iter().any(|o| {
                    o.layers.has(t.layer)
                        && o.net != Some(t.net)
                        && o.shape.bbox().overlaps(&bb.grow(near))
                        && o.shape.dist_to_seg(w[0], w[1]) < near
                });
                if hit {
                    zones.push(bb.grow(0.3));
                }
            }
            t.necks.extend(zones);
        }
    }
    let base = crate::relax::RelaxConfig::default();
    let cfg = crate::relax::RelaxConfig {
        iterations: 120,
        tension: 0.0,
        via_edge: false,
        taut: false,
        ..base.clone()
    };
    let wake = |g: &mut Geometry| {
        // the rest sleeps until a push reaches it
        g.tracks
            .iter_mut()
            .for_each(|t| t.resting = !extra.contains_key(&t.net));
        g.vias.iter_mut().for_each(|v| v.resting = !extra.contains_key(&v.net));
    };
    // the tracks grow in small steps: a neighbour that a wide track overlaps by more than
    // the clearance could be pushed through to the other side
    let most = extra.values().copied().fold(0.0, f64::max);
    let steps = (most / 0.15).ceil().max(1.0) as usize;
    for k in 1..=steps {
        for (t, p) in g.tracks.iter_mut().zip(&plain) {
            if let Some(&e) = extra.get(&t.net) {
                t.width = p.0 + e * k as f64 / steps as f64;
            }
        }
        wake(&mut g);
        let cfg = crate::relax::RelaxConfig {
            iterations: 20,
            ..cfg.clone()
        };
        crate::relax::relax(board, sub, &mut g, &cfg);
    }
    for _ in 0..4 {
        wake(&mut g);
        let t0 = std::time::Instant::now();
        crate::relax::relax(board, sub, &mut g, &cfg);
        let left = crate::check::spots(&crate::check::check(board, &g));
        if std::env::var("GB_TUNE_LOG").is_ok() {
            eprintln!("room round: {} spots, {:.1} s", left.len(), t0.elapsed().as_secs_f64());
        }
        if left.is_empty() {
            break;
        }
        for v in &left {
            let mut zone = Rect::empty();
            zone.add(v.at);
            let zone = zone.grow(0.6);
            let nets = [Some(v.nets.0), v.nets.1];
            for t in g
                .tracks
                .iter_mut()
                .filter(|t| nets.contains(&Some(t.net)) && extra.contains_key(&t.net))
            {
                t.necks.push(zone);
            }
        }
    }
    // the pushed copper as rubber bands again around the wide tracks: a push in a chain
    // leaves bulges that only make other nets longer, and the longest net of a group sets
    // the length for all of it
    // (only what moved: the rest stays exactly as the router left it)
    for (t, o) in g.tracks.iter_mut().zip(&geo.tracks) {
        t.resting = t.pts == o.pts && !extra.contains_key(&t.net);
    }
    for (v, o) in g.vias.iter_mut().zip(&geo.vias) {
        v.resting = v.at == o.at;
    }
    let tight = crate::relax::RelaxConfig {
        tension: base.tension,
        ..cfg.clone()
    };
    crate::relax::relax(board, sub, &mut g, &tight);
    // the shortcuts at the widths reached
    let cfg = crate::relax::RelaxConfig {
        iterations: 0,
        margin: 2.0 * base.margin,
        taut: true,
        ..cfg
    };
    crate::relax::relax(board, sub, &mut g, &cfg);
    g.tracks.iter_mut().for_each(|t| t.resting = false);
    g.vias.iter_mut().for_each(|v| v.resting = false);
    for (t, (w, necks, neck_w)) in g.tracks.iter_mut().zip(plain) {
        t.width = w;
        t.necks = necks;
        t.neck_w = neck_w;
    }
    // what overlaps now settles at the real widths: first without tension, then the nets
    // still in trouble as rubber bands again
    for k in 0..3 {
        let left = crate::check::spots(&crate::check::check(board, &g));
        if left.len() <= before {
            break;
        }
        let bad: Vec<NetId> = left.iter().flat_map(|v| [Some(v.nets.0), v.nets.1]).flatten().collect();
        g.tracks.iter_mut().for_each(|t| t.resting = !bad.contains(&t.net));
        g.vias.iter_mut().for_each(|v| v.resting = !bad.contains(&v.net));
        let cfg = crate::relax::RelaxConfig {
            iterations: 120,
            tension: if k == 0 { 0.0 } else { base.tension },
            via_edge: false,
            taut: true,
            ..base.clone()
        };
        crate::relax::relax(board, sub, &mut g, &cfg);
    }
    g.tracks.iter_mut().for_each(|t| t.resting = false);
    g.vias.iter_mut().for_each(|v| v.resting = false);
    g
}

/// Meander the nets that the rules want longer. Returns what was done per net.
pub fn tune(board: &Board, geo: &mut Geometry, cfg: &TuneConfig) -> TuneReport {
    let aims = aims(board, geo);
    if aims.is_empty() {
        return TuneReport::default();
    }
    let before: HashMap<NetId, f64> = aims
        .keys()
        .map(|&n| (n, matching::net_length(board, geo, n).total()))
        .collect();
    // units: pairs whose two nets both want length go as one band
    let mut units: Vec<(Vec<NetId>, f64)> = Vec::new();
    let mut done: Vec<NetId> = Vec::new();
    for (p, n) in board.diff_pairs() {
        if let (Some(&ap), Some(&an)) = (aims.get(&p), aims.get(&n)) {
            let need = ((ap - before[&p]) + (an - before[&n])) / 2.0;
            units.push((vec![p, n], need));
            done.extend([p, n]);
        }
    }
    let mut singles: Vec<NetId> = aims.keys().copied().filter(|n| !done.contains(n)).collect();
    singles.sort();
    for n in singles {
        units.push((vec![n], aims[&n] - before[&n]));
    }
    // the largest needs first: they need the most room
    units.sort_by(|a, b| b.1.total_cmp(&a.1).then(a.0.cmp(&b.0)));
    let mut coupled_nets: Vec<NetId> = Vec::new();
    let mut patterns: Vec<Pattern> = Vec::new();
    for (nets, need) in &units {
        if *need <= TOL {
            continue;
        }
        let mut left = *need;
        if nets.len() == 2 {
            let aim = aims[&nets[0]].max(aims[&nets[1]]);
            let caps = (ceiling(board, geo, &aims, nets[0]), ceiling(board, geo, &aims, nets[1]));
            let coupled = !sides(geo, nets[0], nets[1]).is_empty();
            left -= tune_pair(board, geo, nets[0], nets[1], left, aim, caps, cfg, &mut patterns);
            coupled_nets.extend(nets.iter().copied());
            // a pair that does not run side by side anywhere: each net on its own (a coupled
            // pair stays as it is, meanders of one track would break its coupling and skew)
            if !coupled && left > TOL {
                for &n in nets {
                    let l = aims[&n] - matching::net_length(board, geo, n).total();
                    if l > TOL {
                        tune_net(board, geo, n, l, aims[&n], cfg, &mut patterns);
                    }
                }
            }
        } else {
            tune_net(board, geo, nets[0], left, aims[&nets[0]], cfg, &mut patterns);
        }
    }
    let mut nets: Vec<NetTune> = aims
        .iter()
        .map(|(&n, &aim)| NetTune {
            net: board.nets[n as usize].name.clone(),
            before: before[&n],
            after: matching::net_length(board, geo, n).total(),
            aim,
            coupled: coupled_nets.contains(&n),
        })
        .collect();
    nets.sort_by(|a, b| a.net.cmp(&b.net));
    TuneReport { nets, patterns }
}

/// Put `mid` in place of element `ei` of track `ti` as a track of its own: the track keeps the
/// elements before it (and `pre`), a new track takes `post` and the elements after it. Returns
/// the index of the new middle track.
fn split_in(
    geo: &mut Geometry,
    ti: usize,
    ei: usize,
    pre: Vec<Elem>,
    mid: Vec<Elem>,
    post: Vec<Elem>,
    pattern: u32,
) -> usize {
    let t = &mut geo.tracks[ti];
    let mut rest = post;
    rest.extend_from_slice(&t.elems[ei + 1..]);
    t.elems.truncate(ei);
    t.elems.extend(pre);
    t.pts = flatten(&t.elems);
    let mut m = t.clone();
    m.pts = flatten(&mid);
    m.elems = mid;
    m.pattern = Some(pattern);
    let mut r = t.clone();
    r.pts = flatten(&rest);
    r.elems = rest;
    geo.tracks.push(m);
    let at = geo.tracks.len() - 1;
    if !r.elems.is_empty() {
        geo.tracks.push(r);
    }
    at
}

/// Copper a meander must keep away from: everything but the nets being tuned.
struct Room<'a> {
    board: &'a Board,
    tracks: Vec<(Rect, usize)>,
    nets: Vec<NetId>,
}

impl<'a> Room<'a> {
    fn new(board: &'a Board, geo: &Geometry, nets: &[NetId]) -> Room<'a> {
        let tracks = geo
            .tracks
            .iter()
            .enumerate()
            .filter(|(_, t)| !nets.contains(&t.net))
            .map(|(i, t)| {
                let mut r = Rect::empty();
                for &p in &t.pts {
                    r.add(p);
                }
                (r.grow(t.width / 2.0), i)
            })
            .collect();
        Room {
            board,
            tracks,
            nets: nets.to_vec(),
        }
    }

    /// Whether a polyline of half width `hw` on `layer` keeps its clearance to everything.
    fn free(&self, geo: &Geometry, pts: &[Point], hw: f64, layer: LayerId) -> bool {
        let net = self.nets[0];
        // a little more: arcs bulge out of their flattened chords, and the file rounds to nm
        let hw = hw + 0.005;
        let clr = self.board.class_of(net).clearance;
        let edge = self.board.rules.edge_clearance + hw;
        if pts.iter().any(|&p| self.board.edge_dist(p) < edge) {
            return false;
        }
        let mut bb = Rect::empty();
        for &p in pts {
            bb.add(p);
        }
        let bb = bb.grow(hw + 1.0);
        for o in &self.board.obstacles {
            if !o.layers.has(layer) || o.net.is_some_and(|n| self.nets.contains(&n)) || !o.shape.bbox().overlaps(&bb) {
                continue;
            }
            let need = o.clearance.max(clr) + hw;
            if pts.windows(2).any(|w| o.shape.dist_to_seg(w[0], w[1]) < need) {
                return false;
            }
        }
        for &(r, i) in &self.tracks {
            let t = &geo.tracks[i];
            if t.layer != layer || !r.overlaps(&bb) {
                continue;
            }
            let need = self.board.track_clearance(net, t.net) + hw + t.width / 2.0;
            for w in pts.windows(2) {
                for v in t.pts.windows(2) {
                    if crate::geom::seg_seg_dist(w[0], w[1], v[0], v[1]) < need {
                        return false;
                    }
                }
            }
        }
        for v in &geo.vias {
            if self.nets.contains(&v.net) || !bb.grow(v.diameter).contains(v.at) {
                continue;
            }
            if v.drill <= 0.0 && v.layer.is_some_and(|l| l != layer) {
                continue;
            }
            let need = self.board.track_clearance(net, v.net) + hw + v.diameter / 2.0;
            if pts
                .windows(2)
                .any(|w| crate::geom::point_seg_dist(v.at, w[0], w[1]) < need)
            {
                return false;
            }
        }
        true
    }
}

/// One bump in local coordinates (x along the line, y to the side), from (0, 0) to (4r, 0).
fn bump(style: Style, r: f64, h: f64) -> Vec<(Point, Option<Point>, Point)> {
    let p = |x: f64, y: f64| Point { x, y };
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let mut out = Vec::new();
    match style {
        Style::Round => {
            out.push((p(0.0, 0.0), Some(p(r * s, r * (1.0 - s))), p(r, r)));
            if h - 2.0 * r > 1e-9 {
                out.push((p(r, r), None, p(r, h - r)));
            }
            out.push((p(r, h - r), Some(p(2.0 * r, h)), p(3.0 * r, h - r)));
            if h - 2.0 * r > 1e-9 {
                out.push((p(3.0 * r, h - r), None, p(3.0 * r, r)));
            }
            out.push((p(3.0 * r, r), Some(p(4.0 * r - r * s, r * (1.0 - s))), p(4.0 * r, 0.0)));
        }
        Style::Chamfer => {
            let q = 0.8 * r;
            let pts = [
                p(0.0, 0.0),
                p(r - q, 0.0),
                p(r, q),
                p(r, h - q),
                p(r + q, h),
                p(3.0 * r - q, h),
                p(3.0 * r, h - q),
                p(3.0 * r, q),
                p(3.0 * r + q, 0.0),
                p(4.0 * r, 0.0),
            ];
            for w in pts.windows(2) {
                if w[0].dist(w[1]) > 1e-9 {
                    out.push((w[0], None, w[1]));
                }
            }
        }
    }
    out
}

/// Extra length of one bump of height h.
fn bump_gain(style: Style, r: f64, h: f64) -> f64 {
    bump(style, r, h)
        .iter()
        .map(|e| local_elem(e, |q| q))
        .map(|e| elem_len(&e))
        .sum::<f64>()
        - 4.0 * r
}

fn local_elem(e: &(Point, Option<Point>, Point), f: impl Fn(Point) -> Point) -> Elem {
    match e.1 {
        Some(m) => Elem::Arc(f(e.0), f(m), f(e.2)),
        None => Elem::Line(f(e.0), f(e.2)),
    }
}

/// A planned meander on the line from `a` to `b`: per slot (start along the line, side, height).
struct Plan {
    r: f64,
    bumps: Vec<(f64, f64, f64)>,
    gain: f64,
}

/// The meander for a centre line from a to b with half width `hw`, legs `2r` apart, that gains
/// as close to `need` as the room allows (never more).
#[allow(clippy::too_many_arguments)]
fn plan(
    room: &Room,
    geo: &Geometry,
    a: Point,
    b: Point,
    layer: LayerId,
    hw: f64,
    r: f64,
    need: f64,
    cfg: &TuneConfig,
) -> Option<Plan> {
    let len = a.dist(b);
    let u = (b - a).norm();
    let left = Point { x: -u.y, y: u.x };
    let margin = (2.0 * r).max(0.3);
    let slots = ((len - 2.0 * margin) / (4.0 * r)).floor();
    if slots < 1.0 {
        return None;
    }
    let slots = slots as usize;
    let x0 = margin + ((len - 2.0 * margin) - slots as f64 * 4.0 * r) / 2.0;
    let hmax = cfg.amplitude.max(2.0 * r);
    let pts_of = |x: f64, side: f64, r: f64, h: f64| -> Vec<Point> {
        let o = a + u * x;
        let f = |q: Point| o + u * q.x + left * (side * q.y);
        let elems: Vec<Elem> = bump(cfg.style, r, h).iter().map(|e| local_elem(e, f)).collect();
        flatten(&elems)
    };
    // per slot the side with more room and the height it allows
    let mut free_h: Vec<(f64, f64)> = Vec::new();
    for k in 0..slots {
        let x = x0 + k as f64 * 4.0 * r;
        let mut best = (0.0, 0.0);
        for side in [1.0, -1.0] {
            if !room.free(geo, &pts_of(x, side, r, 2.0 * r), hw, layer) {
                continue;
            }
            let (mut lo, mut hi) = (2.0 * r, hmax);
            if room.free(geo, &pts_of(x, side, r, hi), hw, layer) {
                lo = hi;
            } else {
                for _ in 0..8 {
                    let mid = (lo + hi) / 2.0;
                    if room.free(geo, &pts_of(x, side, r, mid), hw, layer) {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
            }
            if lo > best.1 {
                best = (side, lo);
            }
        }
        free_h.push(best);
    }
    let gmin = bump_gain(cfg.style, r, 2.0 * r);
    let mut usable: Vec<usize> = (0..slots).filter(|&k| free_h[k].1 > 0.0).collect();
    if usable.is_empty() {
        return None;
    }
    // the roomiest slots first
    usable.sort_by(|&i, &j| free_h[j].1.total_cmp(&free_h[i].1).then(i.cmp(&j)));
    let total =
        |ks: &[usize], cap: f64| -> f64 { ks.iter().map(|&k| bump_gain(cfg.style, r, free_h[k].1.min(cap))).sum() };
    let x_of = |k: usize| x0 + k as f64 * 4.0 * r;
    if need < gmin {
        // one small bump: a smaller turn radius, at its lowest
        let k = usable[0];
        let r2 = (r * need / gmin).max(0.005);
        let x = x_of(k) + 2.0 * r - 2.0 * r2;
        let (side, _) = free_h[k];
        if !room.free(geo, &pts_of(x, side, r2, 2.0 * r2), hw, layer) {
            return None;
        }
        return Some(Plan {
            r: r2,
            bumps: vec![(x, side, 2.0 * r2)],
            gain: bump_gain(cfg.style, r2, 2.0 * r2),
        });
    }
    let m = ((need / gmin).floor() as usize).clamp(1, usable.len());
    let chosen: Vec<usize> = usable[..m].to_vec();
    let full = total(&chosen, hmax);
    let cap = if full <= need {
        hmax
    } else {
        let (mut lo, mut hi) = (2.0 * r, hmax);
        for _ in 0..40 {
            let mid = (lo + hi) / 2.0;
            if total(&chosen, mid) <= need {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo
    };
    let mut bumps: Vec<(f64, f64, f64)> = chosen
        .iter()
        .map(|&k| (x_of(k), free_h[k].0, free_h[k].1.min(cap)))
        .collect();
    bumps.sort_by(|p, q| p.0.total_cmp(&q.0));
    Some(Plan {
        r,
        gain: total(&chosen, cap),
        bumps,
    })
}

/// The centre line from a to b with the planned bumps.
fn realize_plan(a: Point, b: Point, p: &Plan, style: Style) -> Vec<Elem> {
    let u = (b - a).norm();
    let left = Point { x: -u.y, y: u.x };
    let mut out = Vec::new();
    let mut cur = a;
    for &(x, side, h) in &p.bumps {
        let o = a + u * x;
        if cur.dist(o) > 1e-9 {
            out.push(Elem::Line(cur, o));
        }
        let f = |q: Point| o + u * q.x + left * (side * q.y);
        out.extend(bump(style, p.r, h).iter().map(|e| local_elem(e, f)));
        cur = o + u * (4.0 * p.r);
    }
    if cur.dist(b) > 1e-9 {
        out.push(Elem::Line(cur, b));
    }
    out
}

/// Lengthen one net by about `need` mm with meanders on its longest straight lines. Returns the
/// length gained.
#[allow(clippy::too_many_arguments)]
pub fn tune_net(
    board: &Board,
    geo: &mut Geometry,
    net: NetId,
    need: f64,
    aim: f64,
    cfg: &TuneConfig,
    patterns: &mut Vec<Pattern>,
) -> f64 {
    let mut left = need;
    let mut tried: Vec<(Point, Point)> = Vec::new();
    while left > TOL {
        let room = Room::new(board, geo, &[net]);
        let mut lines: Vec<(f64, usize, usize)> = Vec::new();
        for (ti, t) in geo
            .tracks
            .iter()
            .enumerate()
            .filter(|(_, t)| t.net == net && t.pattern.is_none())
        {
            for (ei, e) in t.elems.iter().enumerate() {
                if let Elem::Line(a, b) = *e
                    && a.dist(b) >= 1.0
                    && !tried.iter().any(|&(p, q)| p.dist(a) < 1e-9 && q.dist(b) < 1e-9)
                {
                    lines.push((a.dist(b), ti, ei));
                }
            }
        }
        lines.sort_by(|x, y| y.0.total_cmp(&x.0).then(x.1.cmp(&y.1)).then(x.2.cmp(&y.2)));
        let mut placed = false;
        for &(_, ti, ei) in &lines {
            let t = &geo.tracks[ti];
            let Elem::Line(a, b) = t.elems[ei] else { continue };
            tried.push((a, b));
            let w = t.width;
            let clr = board.class_of(net).clearance;
            let r = (w + (cfg.spacing * w).max(clr)) / 2.0;
            let Some(p) = plan(&room, geo, a, b, t.layer, w / 2.0, r, left, cfg) else {
                continue;
            };
            if p.gain <= 1e-6 {
                continue;
            }
            let new = realize_plan(a, b, &p, cfg.style);
            let layer = t.layer;
            let id = patterns.len() as u32;
            let m = split_in(geo, ti, ei, Vec::new(), new, Vec::new(), id);
            patterns.push(Pattern {
                nets: vec![net],
                layer,
                base: vec![(a, b)],
                tracks: vec![m],
                rounded: cfg.style == Style::Round,
                width: w,
                gap: 0.0,
                amplitude: cfg.amplitude,
                spacing: 2.0 * r - w,
                aim,
            });
            left -= p.gain;
            placed = true;
            break;
        }
        if !placed {
            break;
        }
    }
    need - left
}

/// The two curves at distance d/2 to both sides of a centre line of lines and arcs whose
/// pieces meet at a tangent (round meanders) or at 45 degrees (chamfered ones).
fn offset(elems: &[Elem], d: f64) -> Vec<Elem> {
    let mut out: Vec<Elem> = Vec::new();
    for e in elems {
        let next = match *e {
            Elem::Line(a, b) => {
                let u = (b - a).norm();
                let n = Point { x: -u.y, y: u.x } * d;
                Elem::Line(a + n, b + n)
            }
            Elem::Arc(a, m, b) => match crate::geom::arc_from_three(a, m, b) {
                Some((c, r, _, sweep)) => {
                    let r2 = if sweep > 0.0 { r - d } else { r + d };
                    let f = r2 / r;
                    let q = |x: Point| c + (x - c) * f;
                    Elem::Arc(q(a), q(m), q(b))
                }
                None => {
                    let u = (b - a).norm();
                    let n = Point { x: -u.y, y: u.x } * d;
                    Elem::Line(a + n, b + n)
                }
            },
        };
        // corners between lines: meet where the moved lines cross
        if let (Some(Elem::Line(pa, pb)), Elem::Line(na, nb)) = (out.last().copied(), next)
            && pb.dist(na) > 1e-9
        {
            let (u, v) = (pb - pa, nb - na);
            let den = u.x * v.y - u.y * v.x;
            if den.abs() > 1e-12 {
                let w = na - pa;
                let s = (w.x * v.y - w.y * v.x) / den;
                let x = pa + u * s;
                *out.last_mut().unwrap() = Elem::Line(pa, x);
                out.push(Elem::Line(x, nb));
                continue;
            }
        }
        out.push(next);
    }
    out
}

/// A line of `p` and a line of `n` on the same layer that run side by side.
struct Side {
    tp: usize,
    ep: usize,
    tn: usize,
    en: usize,
    /// Overlap along the P line, from its start.
    s0: f64,
    s1: f64,
    /// Signed distance of the N line to the left of the P line.
    off: f64,
    reversed: bool,
}

fn sides(geo: &Geometry, p: NetId, n: NetId) -> Vec<Side> {
    let mut out = Vec::new();
    for (tp, a) in geo.tracks.iter().enumerate().filter(|(_, t)| t.net == p) {
        for (tn, b) in geo
            .tracks
            .iter()
            .enumerate()
            .filter(|(_, t)| t.net == n && t.layer == a.layer)
        {
            let pitch_max = 3.0 * (a.width + b.width);
            for (ep, e) in a.elems.iter().enumerate() {
                let Elem::Line(pa, pb) = *e else { continue };
                let lp = pa.dist(pb);
                if lp < 1.0 {
                    continue;
                }
                let u = (pb - pa).norm();
                let left = Point { x: -u.y, y: u.x };
                for (en, f) in b.elems.iter().enumerate() {
                    let Elem::Line(na, nb) = *f else { continue };
                    let v = (nb - na).norm();
                    let dot = u.x * v.x + u.y * v.y;
                    if dot.abs() < 1.0 - 1e-6 {
                        continue;
                    }
                    let off = (na - pa).x * left.x + (na - pa).y * left.y;
                    let off2 = (nb - pa).x * left.x + (nb - pa).y * left.y;
                    if (off - off2).abs() > 1e-4 || off.abs() > pitch_max || off.abs() < 1e-3 {
                        continue;
                    }
                    let (sa, sb) = (
                        (na - pa).x * u.x + (na - pa).y * u.y,
                        (nb - pa).x * u.x + (nb - pa).y * u.y,
                    );
                    let (s0, s1) = (sa.min(sb).max(0.0), sa.max(sb).min(lp));
                    if s1 - s0 < 1.0 {
                        continue;
                    }
                    out.push(Side {
                        tp,
                        ep,
                        tn,
                        en,
                        s0,
                        s1,
                        off,
                        reversed: dot < 0.0,
                    });
                }
            }
        }
    }
    out
}

/// Lengthen a pair by about `need` mm with meanders of the band, on the longest lines where
/// the two tracks run side by side. Returns the length gained (per net).
#[allow(clippy::too_many_arguments)]
pub fn tune_pair(
    board: &Board,
    geo: &mut Geometry,
    p: NetId,
    n: NetId,
    need: f64,
    aim: f64,
    caps: (f64, f64),
    cfg: &TuneConfig,
    patterns: &mut Vec<Pattern>,
) -> f64 {
    let mut left = need;
    let mut tried: Vec<(Point, Point)> = Vec::new();
    while left > TOL {
        let room = Room::new(board, geo, &[p, n]);
        let mut cands = sides(geo, p, n);
        cands.retain(|s| geo.tracks[s.tp].pattern.is_none() && geo.tracks[s.tn].pattern.is_none());
        cands.retain(|s| {
            let Elem::Line(a, b) = geo.tracks[s.tp].elems[s.ep] else {
                return false;
            };
            !tried.iter().any(|&(x, y)| x.dist(a) < 1e-9 && y.dist(b) < 1e-9)
        });
        cands.sort_by(|x, y| {
            (y.s1 - y.s0)
                .total_cmp(&(x.s1 - x.s0))
                .then(x.tp.cmp(&y.tp))
                .then(x.ep.cmp(&y.ep))
        });
        let mut placed = false;
        for s in &cands {
            let tp = &geo.tracks[s.tp];
            let Elem::Line(pa, pb) = tp.elems[s.ep] else { continue };
            tried.push((pa, pb));
            let u = (pb - pa).norm();
            let left_n = Point { x: -u.y, y: u.x };
            let w = tp.width.max(geo.tracks[s.tn].width);
            let d = s.off.abs();
            let clr = board.class_of(p).clearance;
            let r = (d + w + (cfg.spacing * w).max(clr)) / 2.0;
            let c0 = pa + u * s.s0 + left_n * (s.off / 2.0);
            let c1 = pa + u * s.s1 + left_n * (s.off / 2.0);
            let hw = d / 2.0 + w / 2.0;
            // the outer track of a turn gains more than the inner one: neither may get past
            // its ceiling (the window's max, the longest net of its group)
            let lp = caps.0 - matching::net_length(board, geo, p).total();
            let ln = caps.1 - matching::net_length(board, geo, n).total();
            let mut want = left;
            let mut found = None;
            for _ in 0..4 {
                let Some(plan) = plan(&room, geo, c0, c1, tp.layer, hw, r, want, cfg) else {
                    break;
                };
                if plan.gain <= 1e-6 {
                    break;
                }
                let centre = realize_plan(c0, c1, &plan, cfg.style);
                // P lies to the right of the centre line if N is on the left of P
                let on_p = offset(&centre, -s.off / 2.0);
                let on_n = offset(&centre, s.off / 2.0);
                let gain_p = on_p.iter().map(elem_len).sum::<f64>() - (s.s1 - s.s0);
                let gain_n = on_n.iter().map(elem_len).sum::<f64>() - (s.s1 - s.s0);
                let over = (gain_p - lp).max(gain_n - ln);
                found = Some((on_p, on_n, gain_p, gain_n));
                if over <= TOL {
                    break;
                }
                want -= over;
                found = None;
                if want <= TOL {
                    break;
                }
            }
            let Some((on_p, on_n, gain_p, gain_n)) = found else {
                continue;
            };
            // P: start of its line, the meander, the rest of its line
            let p0 = pa + u * s.s0;
            let p1 = pa + u * s.s1;
            let pre_p: Vec<Elem> = if pa.dist(p0) > 1e-9 {
                vec![Elem::Line(pa, p0)]
            } else {
                Vec::new()
            };
            let post_p: Vec<Elem> = if p1.dist(pb) > 1e-9 {
                vec![Elem::Line(p1, pb)]
            } else {
                Vec::new()
            };
            let Elem::Line(na, nb) = geo.tracks[s.tn].elems[s.en] else {
                continue;
            };
            let (n0, n1) = (mid_n_start(&on_n), mid_n_end(&on_n));
            let rev = |v: &[Elem]| -> Vec<Elem> {
                v.iter()
                    .rev()
                    .map(|e| match *e {
                        Elem::Line(a, b) => Elem::Line(b, a),
                        Elem::Arc(a, m, b) => Elem::Arc(b, m, a),
                    })
                    .collect()
            };
            let (pre_n, mid_n, post_n) = if !s.reversed {
                (
                    if na.dist(n0) > 1e-9 {
                        vec![Elem::Line(na, n0)]
                    } else {
                        Vec::new()
                    },
                    on_n,
                    if n1.dist(nb) > 1e-9 {
                        vec![Elem::Line(n1, nb)]
                    } else {
                        Vec::new()
                    },
                )
            } else {
                (
                    if na.dist(n1) > 1e-9 {
                        vec![Elem::Line(na, n1)]
                    } else {
                        Vec::new()
                    },
                    rev(&on_n),
                    if n0.dist(nb) > 1e-9 {
                        vec![Elem::Line(n0, nb)]
                    } else {
                        Vec::new()
                    },
                )
            };
            let layer = tp.layer;
            let id = patterns.len() as u32;
            let (tpi, tni, ep, en) = (s.tp, s.tn, s.ep, s.en);
            let mp = split_in(geo, tpi, ep, pre_p, on_p, post_p, id);
            let mn = split_in(geo, tni, en, pre_n, mid_n, post_n, id);
            patterns.push(Pattern {
                nets: vec![p, n],
                layer,
                base: vec![(p0, p1), (n0, n1)],
                tracks: vec![mp, mn],
                rounded: cfg.style == Style::Round,
                width: w,
                gap: d - w,
                amplitude: cfg.amplitude,
                spacing: 2.0 * r - d - w,
                aim,
            });
            left -= (gain_p + gain_n) / 2.0;
            placed = true;
            break;
        }
        if !placed {
            break;
        }
    }
    need - left
}

fn mid_n_start(e: &[Elem]) -> Point {
    match e[0] {
        Elem::Line(a, _) | Elem::Arc(a, _, _) => a,
    }
}

fn mid_n_end(e: &[Elem]) -> Point {
    match e[e.len() - 1] {
        Elem::Line(_, b) | Elem::Arc(_, _, b) => b,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{Layer, LayerSet, MatchRule, MinOptMax, Net, NetClass, Obstacle, ObstacleKind, Pin};
    use crate::geom::{Polygon, Shape, pt};
    use crate::realize::Track;

    /// Two nets from x = 4 to x = 26 at y = 5 and y = 5 + pitch, straight tracks between their pads.
    fn board(names: [&str; 2], pitch: f64) -> (Board, Geometry) {
        let mut b = Board::default();
        b.rules.min_clearance = 0.1;
        b.layers = ["F.Cu", "B.Cu"]
            .iter()
            .map(|n| Layer {
                name: n.to_string(),
                is_plane: false,
            })
            .collect();
        b.outline = vec![Polygon::new(vec![pt(0., 0.), pt(30., 0.), pt(30., 12.), pt(0., 12.)])];
        b.classes = vec![NetClass {
            name: "Default".into(),
            track_width: 0.15,
            clearance: 0.1,
            via_diameter: 0.6,
            via_drill: 0.3,
            diff_gap: 0.15,
            ..Default::default()
        }];
        let mut geo = Geometry::default();
        for (i, name) in names.iter().enumerate() {
            let mut net = Net {
                name: name.to_string(),
                class: 0,
                pins: Vec::new(),
                planes: LayerSet::default(),
                priority: 0,
            };
            let y = 5.0 + pitch * i as f64;
            for (j, x) in [4.0, 26.0].into_iter().enumerate() {
                let (id, o) = (b.pins.len(), b.obstacles.len());
                let at = pt(x, y);
                let (lo, hi) = (at + pt(-0.3, -0.07), at + pt(0.3, 0.07));
                b.obstacles.push(Obstacle {
                    shape: Shape::Polygon(Polygon::new(vec![lo, pt(hi.x, lo.y), hi, pt(lo.x, hi.y)])),
                    layers: LayerSet::single(0),
                    net: Some(i as NetId),
                    kind: ObstacleKind::Pad,
                    hole: None,
                    clearance: 0.1,
                    pin: Some(id),
                    locked: false,
                });
                b.pins.push(Pin {
                    name: format!("J{j}.{i}"),
                    footprint: format!("J{j}"),
                    net: i as NetId,
                    pos: at,
                    layers: LayerSet::single(0),
                    obstacles: vec![o],
                    size: (0.6, 0.14),
                });
                net.pins.push(id);
            }
            b.nets.push(net);
            let elems = vec![Elem::Line(pt(4.0, y), pt(26.0, y))];
            geo.tracks.push(Track {
                pts: flatten(&elems),
                elems,
                width: 0.15,
                layer: 0,
                net: i as NetId,
                necks: Vec::new(),
                neck_w: 0.15,
                origin: None,
                resting: false,
                fitted: Vec::new(),
                pattern: None,
                seg: u32::MAX,
                tap: None,
            });
        }
        (b, geo)
    }

    fn length_rule(b: &mut Board, nets: Vec<NetId>, min: f64, max: f64) {
        b.match_rules.push(MatchRule {
            name: "len".into(),
            kind: MatchKind::Length,
            value: MinOptMax {
                min: Some(min),
                opt: None,
                max: Some(max),
            },
            within_pairs: false,
            nets,
        });
    }

    #[test]
    fn a_net_gets_into_its_length_window_with_clean_meanders() {
        for style in [Style::Round, Style::Chamfer] {
            let (mut b, mut geo) = board(["A", "B"], 4.0);
            length_rule(&mut b, vec![0], 27.0, 27.2);
            let cfg = TuneConfig {
                style,
                ..TuneConfig::default()
            };
            let rep = tune(&b, &mut geo, &cfg);
            let l = matching::net_length(&b, &geo, 0).total();
            assert!((27.0..=27.2).contains(&l), "{style:?}: {l}");
            assert!((matching::net_length(&b, &geo, 1).total() - 22.0).abs() < 1e-9);
            assert_eq!(rep.patterns.len(), 1);
            assert!(crate::check::check(&b, &geo).is_empty(), "{style:?}");
            assert!(matching::check(&b, &geo).iter().all(|r| r.failed() == 0));
        }
    }

    #[test]
    fn a_pair_is_meandered_as_a_band_and_keeps_its_skew() {
        let (mut b, mut geo) = board(["D_P", "D_N"], 0.3);
        length_rule(&mut b, vec![0, 1], 26.0, 26.4);
        let rep = tune(&b, &mut geo, &TuneConfig::default());
        let (lp, ln) = (
            matching::net_length(&b, &geo, 0).total(),
            matching::net_length(&b, &geo, 1).total(),
        );
        assert!((26.0..=26.4).contains(&lp) && (26.0..=26.4).contains(&ln), "{lp} {ln}");
        assert!((lp - ln).abs() < 1e-6, "{lp} {ln}");
        assert_eq!(rep.patterns.len(), 1);
        assert_eq!(rep.patterns[0].nets, vec![0, 1]);
        assert!(crate::check::check(&b, &geo).is_empty());
        // the meander tracks are the pattern's, and KiCad leaves them out of the coupling
        assert!(rep.patterns[0].tracks.iter().all(|&t| geo.tracks[t].pattern == Some(0)));
    }
}
