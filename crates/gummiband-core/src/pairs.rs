//! Differential pairs as one wide track. The two nets of a pair become one net with a track as
//! wide as both tracks and their gap, and vias large enough for two vias side by side; the pads
//! of the two nets merge into one pin per end. The normal search and geometry steps route that
//! net, so the pair stays together and nothing gets between its tracks. Afterwards the wide
//! track is split into the two real tracks, parallel to its centre line.

use std::collections::HashMap;

use crate::board::{Board, ClassId, LayerId, LayerSet, NetId, Obstacle, ObstacleKind, Pin};
use crate::geom::{Point, Polygon, Shape, arc_from_three, pt};
use crate::realize::{Elem, Geometry, Track, Via, flatten};
use crate::route::RouterConfig;
use crate::topo::{Substrate, SubstrateConfig};
use crate::{RouteResult, SearchConfig, route_board};

/// Pads of a pair farther apart than this do not merge into one end of the wide track.
const MAX_PAD_DIST: f64 = 3.0;

/// The wide track is this much wider than the two tracks and their gap: the links from the
/// pads to the band leave it by a few micrometres.
const BAND_MARGIN: f64 = 0.03;

/// One end of a pair: a pad of each net.
#[derive(Debug, Clone, Copy)]
struct PairEnd {
    p: usize,
    n: usize,
    /// Breakout: the wide track starts here, outside the pads; single tracks connect the pads.
    out: Option<Point>,
}

#[derive(Debug, Clone)]
struct MergedPair {
    /// The net that stands for the pair (the P net) and its partner.
    p: NetId,
    n: NetId,
    ends: Vec<PairEnd>,
    /// Net of the P pins without partner while the pair is merged.
    rest: Option<NetId>,
}

/// A board with its pairs merged into wide nets. Net and pin indices stay valid.
struct Merged {
    board: Board,
    pairs: Vec<MergedPair>,
    /// Pairs that stay two nets, and why.
    skipped: Vec<(String, String)>,
    /// Pads of one net that the last pass joins on their own, short and at their part: the
    /// second pad row of a USB-C socket to the pads where the wide track ends (see
    /// [`second_row`]).
    joins: Vec<(usize, usize)>,
    /// The wide tracks run on the 45 degree grid: the outer side of a corner of the two
    /// tracks is a corner as well, not an arc.
    miter: bool,
}

/// Result of a pair for reports.
#[derive(Debug, Clone, Default, serde::Serialize)]
pub struct PairReport {
    pub p: String,
    pub n: String,
    /// False if the pair was routed as two separate nets.
    pub wide: bool,
    /// Why the pair or a part of it was not routed as one wide track.
    #[serde(skip_serializing_if = "String::is_empty")]
    pub note: String,
    pub width: f64,
    pub gap: f64,
    pub length_p: f64,
    pub length_n: f64,
    pub skew: f64,
    /// Length that does not run coupled, as KiCad's DRC measures it.
    pub uncoupled: f64,
    pub vias: usize,
    /// Polarity swaps on the way (through the pair's vias).
    pub swaps: usize,
    /// Differential impedance from the stackup in ohm (worst layer of the board), 0 if unknown.
    pub zdiff: f64,
}

#[derive(Debug, Clone)]
pub struct PairConfig {
    /// Length difference within a pair that is left alone, in mm (custom rules may ask for less).
    pub skew: f64,
    /// Target differential impedance in ohm: sets the track width per layer from the stackup,
    /// the gap stays.
    pub zdiff: Option<f64>,
    /// Couple every pair recognised by name, also those without a pair rule or class of their
    /// own (see [`Board::coupled_pairs`]).
    pub all: bool,
    /// Make room for the meanders of the length tuning while the pairs are still one wide
    /// track (see [`crate::tune::make_room`]), with the settings the tuning will use.
    pub room: Option<crate::tune::TuneConfig>,
    /// `--angles 45`: the wide tracks go onto the 45 degree grid before they are split, so the
    /// two tracks of a pair stay parallel (see [`crate::octo`]).
    pub angles45: bool,
    /// Breakout zones: the wide track starts outside the pads of a pair end, two single
    /// tracks connect the pads (see [`breakout`]).
    pub breakouts: bool,
    /// Vias side by side as one via pair: a capsule that turns with the wide track in the
    /// relaxation, instead of the circle that holds the vias in any arrangement (see
    /// [`pair_vias`]).
    pub via_pairs: bool,
}

impl Default for PairConfig {
    fn default() -> Self {
        PairConfig {
            skew: 0.1,
            zdiff: None,
            all: false,
            room: None,
            angles45: false,
            breakouts: false,
            via_pairs: false,
        }
    }
}

fn cross(a: Point, b: Point) -> f64 {
    a.x * b.y - a.y * b.x
}

/// Unit normal on the left of direction u.
fn left_of(u: Point) -> Point {
    pt(-u.y, u.x)
}

/// Convex hull of a point set, counter-clockwise.
fn hull(mut pts: Vec<Point>) -> Vec<Point> {
    pts.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    pts.dedup_by(|a, b| a.dist(*b) < 1e-9);
    if pts.len() < 3 {
        return pts;
    }
    let half = |it: &mut dyn Iterator<Item = &Point>| {
        let mut h: Vec<Point> = Vec::new();
        for &p in it {
            while h.len() >= 2 && cross(h[h.len() - 1] - h[h.len() - 2], p - h[h.len() - 1]) <= 0.0 {
                h.pop();
            }
            h.push(p);
        }
        h.pop();
        h
    };
    let mut lower = half(&mut pts.iter());
    lower.extend(half(&mut pts.iter().rev()));
    lower
}

/// The copper hull of two pads, if it keeps its distance from everything foreign.
fn bridge(b: &Board, p: usize, n: usize, carrier_clr: f64) -> Option<Obstacle> {
    zone(b, p, n, &[], carrier_clr)
}

/// A breakout zone for the pads `p` and `n`: the point outside the pads where the wide track
/// of width `wide` starts, and the hull of the pads and that start as the end of the wide net,
/// so that nothing else runs where the two single tracks will go. The point lies on the side of
/// the pads that faces the other end of the pair (else on the other side), as close as the hull
/// keeps its distance from everything foreign.
fn breakout(b: &Board, p: usize, n: usize, toward: Point, wide: f64, carrier_clr: f64) -> Option<(Point, Obstacle)> {
    let (pp, pn) = (&b.pins[p], &b.pins[n]);
    let mid = (pp.pos + pn.pos) * 0.5;
    let axis = (pp.pos - pn.pos).norm();
    let x = left_of(axis);
    // towards the other end of the pair first, else to the other side
    let first = if (toward - mid).dot(x) >= 0.0 { x } else { x * -1.0 };
    // the wide track starts outside the part: between the balls of a BGA or the pad rows of a
    // package it would find no way out past the pads
    let mut part = crate::geom::Rect::empty();
    for pin in [pp, pn] {
        for q in b
            .pins
            .iter()
            .filter(|q| !pin.footprint.is_empty() && q.footprint == pin.footprint)
        {
            for &o in &q.obstacles {
                for v in b.obstacles[o].shape.to_polygon(b.rules.max_error).pts {
                    part.add(v);
                }
            }
        }
    }
    [first, first * -1.0]
        .into_iter()
        .find_map(|x| breakout_towards(b, p, n, x, &part, wide, carrier_clr))
}

fn breakout_towards(
    b: &Board,
    p: usize,
    n: usize,
    x: Point,
    part: &crate::geom::Rect,
    wide: f64,
    carrier_clr: f64,
) -> Option<(Point, Obstacle)> {
    let (pp, pn) = (&b.pins[p], &b.pins[n]);
    let mid = (pp.pos + pn.pos) * 0.5;
    let reach = [pp, pn]
        .iter()
        .flat_map(|pin| pin.obstacles.iter())
        .flat_map(|&o| b.obstacles[o].shape.to_polygon(b.rules.max_error).pts)
        .map(|q| (q - mid).dot(x))
        .fold(0.0, f64::max);
    let r = wide / 2.0;
    let mut d = reach + r + carrier_clr + 0.02;
    while d <= reach + 3.0 {
        let c = mid + x * d;
        if part.contains(c) {
            d += 0.1;
            continue;
        }
        let disc: Vec<Point> = (0..16)
            .map(|i| {
                let a = i as f64 * std::f64::consts::PI / 8.0;
                c + pt(a.cos(), a.sin()) * r
            })
            .collect();
        if let Some(mut o) = zone(b, p, n, &disc, carrier_clr) {
            o.clearance = o.clearance.max(carrier_clr);
            return Some((c, o));
        }
        d += 0.1;
    }
    None
}

/// The hull of two pads and the points `extra`, if it keeps its distance from everything
/// foreign.
fn zone(b: &Board, p: usize, n: usize, extra: &[Point], carrier_clr: f64) -> Option<Obstacle> {
    let (pp, pn) = (&b.pins[p], &b.pins[n]);
    let layers = LayerSet(pp.layers.0 & pn.layers.0 & b.routing_layers().0);
    if layers.is_empty() {
        return None;
    }
    let polys = |pin: &Pin| {
        pin.obstacles
            .iter()
            .flat_map(|&o| b.obstacles[o].shape.to_polygon(b.rules.max_error).pts)
            .collect::<Vec<_>>()
    };
    let mut pts = polys(pp);
    pts.extend(polys(pn));
    pts.extend_from_slice(extra);
    let poly = Polygon::new(hull(pts));
    let bb = poly.bbox();
    let foreign = b.obstacles.iter().any(|o| {
        if !o.layers.intersects(layers) || (o.net.is_some() && (o.net == Some(pp.net) || o.net == Some(pn.net))) {
            return false;
        }
        let need = match o.kind {
            ObstacleKind::Keepout { tracks, .. } => {
                if !tracks {
                    return false;
                }
                0.0
            }
            _ => o.clearance.max(carrier_clr),
        };
        o.shape.bbox().overlaps(&bb.grow(need)) && o.shape.to_polygon(b.rules.max_error).dist_to_poly(&poly) < need
    });
    (!foreign).then(|| Obstacle {
        shape: Shape::Polygon(poly),
        layers,
        net: Some(pp.net),
        kind: ObstacleKind::Pad,
        hole: None,
        clearance: b.obstacles[pp.obstacles[0]]
            .clearance
            .max(b.obstacles[pn.obstacles[0]].clearance),
        pin: Some(p),
        locked: false,
    })
}

/// Pads of the two nets that belong together: nearest first, the same part before two parts.
fn match_pads(b: &Board, a: NetId, c: NetId) -> Vec<PairEnd> {
    let mut cand: Vec<(bool, f64, usize, usize)> = Vec::new();
    for &p in &b.nets[a as usize].pins {
        for &n in &b.nets[c as usize].pins {
            let d = b.pins[p].pos.dist(b.pins[n].pos);
            if d <= MAX_PAD_DIST {
                cand.push((b.pins[p].footprint != b.pins[n].footprint, d, p, n));
            }
        }
    }
    cand.sort_by(|x, y| {
        x.0.cmp(&y.0)
            .then(x.1.total_cmp(&y.1))
            .then(x.2.cmp(&y.2))
            .then(x.3.cmp(&y.3))
    });
    let mut ends: Vec<PairEnd> = Vec::new();
    for (_, _, p, n) in cand {
        if ends.iter().all(|e| e.p != p && e.n != n) {
            ends.push(PairEnd { p, n, out: None });
        }
    }
    ends
}

/// The class of the wide net: as wide as the widest pair spacing of any routing layer, with vias
/// that hold two vias of the pair.
fn carrier_class(b: &Board, p: NetId, n: NetId) -> crate::board::NetClass {
    let (cp, cn) = (b.class_of(p), b.class_of(n));
    let mut c = cp.clone();
    let width = b
        .routing_layers()
        .iter()
        .map(|l| {
            let (w, g, _) = b.pair_shape(p, n, l);
            2.0 * w + g
        })
        .fold(0.0, f64::max)
        + BAND_MARGIN;
    let (_, _, via_gap) = b.pair_shape(p, n, 0);
    let d = cp.via_diameter.max(cn.via_diameter);
    let drill = cp.via_drill.max(cn.via_drill);
    let radius = via_disc(d, via_gap, width / 2.0, cp.clearance.max(cn.clearance));
    c.name = format!("pair:{}", b.nets[p as usize].name);
    c.track_width = width;
    c.clearance = cp.clearance.max(cn.clearance);
    c.via_diameter = 2.0 * radius;
    c.via_drill = 2.0 * radius - (d - drill);
    c.priority = cp.priority.min(cn.priority);
    // no necks: two tracks and their gap do not get narrower near a pin
    c.min_width = Some(width);
    c.target_width = None;
    c.neck_refs = Vec::new();
    c
}

/// Radius of the circle that holds both vias of a pair, parallel or in line, and the tracks that
/// lead to them.
fn via_disc(d: f64, via_gap: f64, half_band: f64, clr: f64) -> f64 {
    let h = (d + via_gap) / 2.0;
    // in line the vias sit on one of the tracks, a quarter of the band off the centre
    let side = half_band / 2.0;
    ((h * h + side * side).sqrt() + d / 2.0).max(side + d / 2.0 + clr + half_band / 2.0)
}

/// The classes of the pairs with track widths that reach `zdiff` ohm on every layer of the stackup.
pub fn apply_zdiff(b: &mut Board, zdiff: f64) {
    let min_w = b.rules.min_track_width;
    for (p, n) in b.diff_pairs() {
        let (mut outer, mut inner) = (None, None);
        for l in b.routing_layers().iter() {
            let (_, g, _) = b.pair_shape(p, n, l);
            let Some(w) = b.width_for_zdiff(l, g, zdiff, min_w) else {
                continue;
            };
            if l == 0 || l as usize + 1 == b.layers.len() {
                outer = Some((w, g));
            } else {
                inner = Some((w, g));
            }
        }
        let c = b.nets[p as usize].class as usize;
        if outer.is_some() {
            b.classes[c].diff_outer = outer;
        }
        if inner.is_some() {
            b.classes[c].diff_inner = inner;
        }
    }
}

/// A pair end at a second row of pins, its bridge, and the joins of the other pads to it.
type Row = (PairEnd, Obstacle, Vec<(usize, usize)>);

/// The end `e` of a pair at a second row of the same pins: if its part has one more pad of
/// each net and nothing foreign lies within the hull of the four pads (the double row of a
/// USB-C socket), the end on two of them that can be bridged, keeps the polarity towards the
/// other end of the pair (`other`, seen along the straight line) and has its pads closest, its
/// bridge, and the joins of the other two pads to it.
fn second_row(b: &Board, e: PairEnd, other: PairEnd, carrier_clr: f64) -> Option<Row> {
    let fp = &b.pins[e.p].footprint;
    if fp.is_empty() || *fp != b.pins[e.n].footprint {
        return None;
    }
    let (np, nn) = (b.pins[e.p].net, b.pins[e.n].net);
    let more = |net: NetId, not: usize| -> Vec<usize> {
        b.nets[net as usize]
            .pins
            .iter()
            .copied()
            .filter(|&x| x != not && b.pins[x].footprint == *fp)
            .collect()
    };
    let (xp, xn) = (more(np, e.p), more(nn, e.n));
    let (&[xp], &[xn]) = (&xp[..], &xn[..]) else {
        return None;
    };
    let pads: Vec<Point> = [e.p, e.n, xp, xn]
        .iter()
        .flat_map(|&q| b.pins[q].obstacles.iter())
        .flat_map(|&o| b.obstacles[o].shape.to_polygon(b.rules.max_error).pts)
        .collect();
    let row = Polygon::new(hull(pads));
    // its neighbours may be as close to the row as its pads to each other
    let foreign = b.obstacles.iter().any(|o| {
        o.net.is_none_or(|x| x != np && x != nn)
            && o.layers.intersects(b.pins[e.p].layers)
            && !matches!(o.kind, ObstacleKind::Keepout { .. })
            && o.shape.bbox().overlaps(&row.bbox())
            && o.shape.to_polygon(b.rules.max_error).dist_to_poly(&row) <= 0.0
    });
    if foreign {
        return None;
    }
    let mid = |e: &PairEnd| (b.pins[e.p].pos + b.pins[e.n].pos) * 0.5;
    let twisted = |e: &PairEnd| {
        let u = (mid(&other) - mid(e)).norm();
        cross(u, b.pins[e.p].pos - b.pins[e.n].pos) * cross(u, b.pins[other.p].pos - b.pins[other.n].pos) < 0.0
    };
    let gap = |e: &PairEnd| b.pins[e.p].pos.dist(b.pins[e.n].pos);
    let (keep, o) = [(e.p, e.n), (e.p, xn), (xp, e.n), (xp, xn)]
        .into_iter()
        .map(|(p, n)| PairEnd { p, n, out: None })
        .filter_map(|c| bridge(b, c.p, c.n, carrier_clr).map(|o| (c, o)))
        .min_by(|(x, _), (y, _)| twisted(x).cmp(&twisted(y)).then(gap(x).total_cmp(&gap(y))))?;
    let (op, on) = (
        if keep.p == e.p { xp } else { e.p },
        if keep.n == e.n { xn } else { e.n },
    );
    Some((keep, o, vec![(op, keep.p), (on, keep.n)]))
}

fn merge(board: &Board, all: bool, breakouts: bool) -> Merged {
    let mut b = board.clone();
    let mut pairs = Vec::new();
    let mut skipped = Vec::new();
    let mut all_joins = Vec::new();
    let coupled = board.coupled_pairs(all);
    for (p, n) in board.diff_pairs() {
        if !coupled.contains(&(p, n)) {
            skipped.push((
                board.nets[p as usize].name.clone(),
                "recognised by name only: no diff_pair_gap rule or net class of its own".to_string(),
            ));
            continue;
        }
        let name = |x: NetId| board.nets[x as usize].name.clone();
        let kept = board.obstacles.iter().any(|o| {
            matches!(o.kind, ObstacleKind::Track | ObstacleKind::Via) && (o.net == Some(p) || o.net == Some(n))
        });
        if kept {
            skipped.push((name(p), "kept copper".to_string()));
            continue;
        }
        if board.planes.iter().any(|x| x.net == p || x.net == n) {
            continue;
        }
        let class = carrier_class(&b, p, n);
        let mut ends: Vec<(PairEnd, Obstacle)> = Vec::new();
        for e in match_pads(&b, p, n) {
            if let Some(o) = bridge(&b, e.p, e.n, class.clearance) {
                ends.push((e, o));
            }
        }
        if ends.len() < 2 {
            skipped.push((
                name(p),
                "pads of the two nets do not sit next to each other".to_string(),
            ));
            continue;
        }
        // one wide track, between the two ends farthest apart: wide tracks of one net could run
        // on top of each other, and they cannot branch. The other pins get single tracks later.
        if ends.len() > 2 {
            let mid = |e: &PairEnd| (b.pins[e.p].pos + b.pins[e.n].pos) * 0.5;
            let mut far = (0.0, 0, 1);
            for i in 0..ends.len() {
                for j in i + 1..ends.len() {
                    let d = mid(&ends[i].0).dist(mid(&ends[j].0));
                    if d > far.0 {
                        far = (d, i, j);
                    }
                }
            }
            let second = ends.swap_remove(far.2);
            let first = ends.swap_remove(far.1);
            ends = vec![first, second];
        }
        // an end at a second row of the same pins (a USB-C socket): the pads of the other row
        // join it right at the part
        let mut joins = Vec::new();
        for i in 0..2 {
            let other = ends[1 - i].0;
            if let Some((e, o, j)) = second_row(&b, ends[i].0, other, class.clearance) {
                ends[i] = (e, o);
                joins.extend(j);
            }
        }
        if std::env::var("GB_PAIR_LOG").is_ok() && !joins.is_empty() {
            eprintln!(
                "second row {}: joins {:?}",
                name(p),
                joins
                    .iter()
                    .map(|&(x, y): &(usize, usize)| format!("{}-{}", board.pins[x].name, board.pins[y].name))
                    .collect::<Vec<_>>()
            );
        }
        all_joins.extend(joins);
        if breakouts {
            let mids: Vec<Point> = ends
                .iter()
                .map(|(e, _)| (b.pins[e.p].pos + b.pins[e.n].pos) * 0.5)
                .collect();
            let ours: Vec<usize> = ends.iter().flat_map(|(e, _)| [e.p, e.n]).collect();
            for (i, (e, o)) in ends.iter_mut().enumerate() {
                // a part with more pins of the pair (a USB-C socket) keeps the end at its pads:
                // the single tracks of those pins join the pair there
                let (fp, fn_) = (&b.pins[e.p].footprint, &b.pins[e.n].footprint);
                let crowded = b.nets[p as usize]
                    .pins
                    .iter()
                    .chain(&b.nets[n as usize].pins)
                    .any(|&x| !ours.contains(&x) && (&b.pins[x].footprint == fp || &b.pins[x].footprint == fn_));
                if crowded {
                    continue;
                }
                // towards the other end of the pair
                if let Some((c, z)) = breakout(&b, e.p, e.n, mids[1 - i], class.track_width, class.clearance) {
                    e.out = Some(c);
                    *o = z;
                    if std::env::var("GB_PAIR_LOG").is_ok() {
                        eprintln!("breakout {} at {:.2},{:.2}", board.nets[p as usize].name, c.x, c.y);
                    }
                }
            }
        }
        // the class first: from here on the pair is one net
        b.classes.push(class);
        let cid = (b.classes.len() - 1) as ClassId;
        let (pins_p, pins_n) = (b.nets[p as usize].pins.clone(), b.nets[n as usize].pins.clone());
        let matched_p: Vec<usize> = ends.iter().map(|(e, _)| e.p).collect();
        let matched_n: Vec<usize> = ends.iter().map(|(e, _)| e.n).collect();
        let free_p: Vec<usize> = pins_p.iter().copied().filter(|x| !matched_p.contains(x)).collect();
        // P pins without partner must not count as the wide net's own copper
        let rest = (!free_p.is_empty()).then(|| {
            let id = b.nets.len() as NetId;
            let mut net = b.nets[p as usize].clone();
            net.name = format!("{}~rest", net.name);
            net.pins = free_p.clone();
            b.nets.push(net);
            for &x in &free_p {
                b.pins[x].net = id;
                for i in b.pins[x].obstacles.clone() {
                    b.obstacles[i].net = Some(id);
                }
            }
            id
        });
        b.nets[p as usize].pins = matched_p;
        b.nets[p as usize].class = cid;
        b.nets[p as usize].priority = b.nets[p as usize].priority.min(b.nets[n as usize].priority);
        b.nets[n as usize].pins = pins_n.iter().copied().filter(|x| !matched_n.contains(x)).collect();
        for (e, o) in &ends {
            let mid = e.out.unwrap_or((b.pins[e.p].pos + b.pins[e.n].pos) * 0.5);
            if e.out.is_some() {
                // at a breakout the wide track ends outside the pads and must not run over
                // them: they block it like copper of no net
                let mut pads = std::mem::take(&mut b.pins[e.p].obstacles);
                pads.extend(b.pins[e.n].obstacles.iter().copied());
                for i in pads {
                    b.obstacles[i].net = None;
                    b.obstacles[i].pin = None;
                }
            }
            for i in b.pins[e.n].obstacles.clone() {
                if e.out.is_some() {
                    break;
                }
                b.obstacles[i].net = Some(p);
                b.obstacles[i].pin = Some(e.p);
                b.pins[e.p].obstacles.push(i);
            }
            let (fp, fn_) = (b.pins[e.p].footprint.clone(), b.pins[e.n].footprint.clone());
            let id = b.obstacles.len();
            b.obstacles.push(o.clone());
            b.pins[e.p].obstacles.push(id);
            b.pins[e.p].pos = mid;
            b.pins[e.p].layers = LayerSet(b.pins[e.p].layers.0 & b.pins[e.n].layers.0);
            b.footprint_obstacles.entry(fp.clone()).or_default().push(id);
            if fp != fn_ {
                // a pair end across two parts stays where it is
                b.movable.retain(|f| *f != fp && *f != fn_);
            }
        }
        // custom clearances of the partner hold for the wide net
        let extra: Vec<((NetId, NetId), f64)> = b
            .pair_clearance
            .iter()
            .filter(|((x, y), _)| *x == n || *y == n)
            .map(|(&k, &v)| (k, v))
            .collect();
        for ((x, y), c) in extra {
            let other = if x == n { y } else { x };
            if other != p {
                let k = (p.min(other), p.max(other));
                let e = b.pair_clearance.entry(k).or_insert(c);
                *e = e.max(c);
            }
        }
        pairs.push(MergedPair {
            p,
            n,
            ends: ends.into_iter().map(|(e, _)| e).collect(),
            rest,
        });
    }
    Merged {
        board: b,
        pairs,
        skipped,
        joins: all_joins,
        miter: false,
    }
}

fn elem_ends(e: &Elem) -> (Point, Point) {
    match *e {
        Elem::Line(a, b) | Elem::Arc(a, _, b) => (a, b),
    }
}

/// Direction of travel at the end of an element (`at_end`) or at its start.
fn tangent(e: &Elem, at_end: bool) -> Point {
    match *e {
        Elem::Line(a, b) => (b - a).norm(),
        Elem::Arc(a, m, b) => match arc_from_three(a, m, b) {
            Some((c, _, _, sweep)) => {
                let p = if at_end { b } else { a };
                let n = left_of((p - c).norm());
                if sweep > 0.0 { n } else { n * -1.0 }
            }
            None => (b - a).norm(),
        },
    }
}

fn elem_len(e: &Elem) -> f64 {
    match *e {
        Elem::Line(a, b) => a.dist(b),
        Elem::Arc(a, m, b) => arc_from_three(a, m, b).map_or(a.dist(b), |(_, r, _, s)| r * s.abs()),
    }
}

pub fn elems_len(elems: &[Elem]) -> f64 {
    elems.iter().map(elem_len).sum()
}

fn reversed(elems: &[Elem]) -> Vec<Elem> {
    elems
        .iter()
        .rev()
        .map(|e| match *e {
            Elem::Line(a, b) => Elem::Line(b, a),
            Elem::Arc(a, m, b) => Elem::Arc(b, m, a),
        })
        .collect()
}

/// The element from arc length `s` on.
fn cut_elem(e: &Elem, s: f64) -> Elem {
    match *e {
        Elem::Line(a, b) => Elem::Line(a.lerp(b, s / a.dist(b)), b),
        Elem::Arc(a, m, b) => match arc_from_three(a, m, b) {
            Some((c, r, sa, sweep)) => {
                let (from, rest) = (s / r * sweep.signum(), sweep - s / r * sweep.signum());
                let at = |x: f64| c + pt(r, 0.0).rotate(sa + x);
                Elem::Arc(at(from), at(from + rest / 2.0), b)
            }
            None => Elem::Line(a, b),
        },
    }
}

/// The curve without its first `s` mm.
fn cut_start(elems: &[Elem], mut s: f64) -> Vec<Elem> {
    let mut out = Vec::new();
    for e in elems {
        let l = elem_len(e);
        if !out.is_empty() {
            out.push(*e);
        } else if s < l - 1e-6 {
            out.push(if s > 1e-9 { cut_elem(e, s) } else { *e });
        } else {
            s -= l;
        }
    }
    out
}

fn cut_end(elems: &[Elem], s: f64) -> Vec<Elem> {
    reversed(&cut_start(&reversed(elems), s))
}

/// Direction from the start of a curve to its point `len` mm on (or its end if shorter).
fn chord(elems: &[Elem], len: f64) -> Point {
    let a = point_at_start(elems);
    let total = elems_len(elems);
    let b = if total > len {
        point_at_end(&cut_end(elems, total - len))
    } else {
        point_at_end(elems)
    };
    if b.dist(a) < 1e-9 {
        tangent(&elems[0], false)
    } else {
        (b - a).norm()
    }
}

fn point_at_start(elems: &[Elem]) -> Point {
    elem_ends(&elems[0]).0
}

fn point_at_end(elems: &[Elem]) -> Point {
    elem_ends(&elems[elems.len() - 1]).1
}

/// Intersection of the lines a-b and c-d: parameters on both.
fn line_cross(a: Point, b: Point, c: Point, d: Point) -> Option<(f64, f64)> {
    let (u, v) = (b - a, d - c);
    let den = cross(u, v);
    if den.abs() < 1e-12 {
        return None;
    }
    Some((cross(c - a, v) / den, cross(c - a, u) / den))
}

/// The curve moved `d` mm to the left of travel (right if negative): arcs stay arcs, the outer
/// side of a sharp corner gets an arc of radius |d| around it, the inner side a corner.
pub fn offset_curve(elems: &[Elem], d: f64) -> Vec<Elem> {
    offset_curve_with(elems, d, false)
}

/// [`offset_curve`]; with `miter` the outer side of a corner between two lines is a corner too:
/// both lines go on to where they meet (on the 45 degree grid they stay on it).
fn offset_curve_with(elems: &[Elem], d: f64, miter: bool) -> Vec<Elem> {
    let mut out: Vec<Elem> = Vec::new();
    for e in elems {
        let (ea, eb) = elem_ends(e);
        let next = match *e {
            Elem::Line(a, b) => {
                if a.dist(b) < 1e-9 {
                    continue;
                }
                let n = left_of((b - a).norm()) * d;
                Elem::Line(a + n, b + n)
            }
            Elem::Arc(a, m, b) => match arc_from_three(a, m, b) {
                Some((c, r, _, sweep)) => {
                    // the centre is on the left of a counter-clockwise arc
                    let r2 = if sweep > 0.0 { r - d } else { r + d };
                    if r2 < 0.02 {
                        continue;
                    }
                    let f = r2 / r;
                    let q = |x: Point| c + (x - c) * f;
                    Elem::Arc(q(a), q(m), q(b))
                }
                None => {
                    let n = left_of((b - a).norm()) * d;
                    Elem::Line(a + n, b + n)
                }
            },
        };
        let start = elem_ends(&next).0;
        let Some(prev) = out.last().copied() else {
            out.push(next);
            continue;
        };
        let end = elem_ends(&prev).1;
        if end.dist(start) < 1e-6 {
            out.push(next);
            continue;
        }
        let (u, v) = (tangent(&prev, true), tangent(&next, false));
        let turn = cross(u, v);
        // a collapsed arc leaves its two neighbours to meet on the inner side
        let inner = (turn > 0.0) == (d > 0.0);
        if inner {
            if let (Elem::Line(a, b), Elem::Line(c, f)) = (prev, next)
                && let Some((s, t)) = line_cross(a, b, c, f)
                && s > 0.0
                && t < 1.0
            {
                let x = a.lerp(b, s);
                *out.last_mut().unwrap() = Elem::Line(a, x);
                out.push(Elem::Line(x, f));
                continue;
            }
            out.push(Elem::Line(end, start));
        } else {
            if miter
                && let (Elem::Line(a, b), Elem::Line(c, f)) = (prev, next)
                && let Some((s, t)) = line_cross(a, b, c, f)
                && s > 0.0
                && t < 1.0
                && turn.abs() < 0.75
            {
                // at most a 45 degree turn: the corner sticks out by d tan(22.5 deg)
                let x = a.lerp(b, s);
                *out.last_mut().unwrap() = Elem::Line(a, x);
                out.push(Elem::Line(x, f));
                continue;
            }
            let bis = (end - ea) + (start - ea);
            if bis.len() > 1e-9 && ea.dist(end) > 1e-9 {
                out.push(Elem::Arc(end, ea + bis.norm() * d.abs(), start));
            } else {
                out.push(Elem::Line(end, start));
            }
        }
        out.push(next);
        let _ = eb;
    }
    out
}

/// The element ending in `p` instead (`p` lies on it, up to the flattening error).
fn with_end(e: Elem, p: Point) -> Elem {
    match e {
        Elem::Line(a, _) => Elem::Line(a, p),
        Elem::Arc(a, m, _) => Elem::Arc(a, m, p),
    }
}

fn with_start(e: Elem, p: Point) -> Elem {
    match e {
        Elem::Line(_, b) => Elem::Line(p, b),
        Elem::Arc(_, m, b) => Elem::Arc(p, m, b),
    }
}

/// Where an offset curve runs into itself within `window` mm of its length, cut out the loop.
/// The inner side of bends tighter than the offset folds over like that.
fn remove_loops(mut elems: Vec<Elem>, window: f64) -> Vec<Elem> {
    'again: loop {
        // pieces: ends, element, arc length in the element at the start
        let mut pieces: Vec<(Point, Point, usize, f64)> = Vec::new();
        for (ei, e) in elems.iter().enumerate() {
            let pts = flatten(std::slice::from_ref(e));
            let mut s = 0.0;
            for w in pts.windows(2) {
                pieces.push((w[0], w[1], ei, s));
                s += w[0].dist(w[1]);
            }
        }
        for i in 0..pieces.len() {
            let (a, b, ei, si) = pieces[i];
            let mut walked = 0.0;
            for &(c, d, ej, sj) in pieces.iter().skip(i + 2) {
                walked += c.dist(d);
                if walked > window {
                    break;
                }
                if ei == ej {
                    continue;
                }
                let Some((t, u)) = line_cross(a, b, c, d) else { continue };
                if !(0.0..=1.0).contains(&t) || !(0.0..=1.0).contains(&u) {
                    continue;
                }
                let x = a.lerp(b, t);
                let head = elem_len(&elems[ei]) - (si + a.dist(x));
                let mut out: Vec<Elem> = elems[..ei].to_vec();
                if let Some(&e) = cut_end(&elems[ei..=ei], head).first() {
                    out.push(with_end(e, x));
                }
                if let Some(&e) = cut_start(&elems[ej..=ej], sj + c.dist(x)).first() {
                    out.push(with_start(e, x));
                }
                out.extend_from_slice(&elems[ej + 1..]);
                elems = out;
                continue 'again;
            }
        }
        return elems;
    }
}

/// The offset of a curve without loops, at least a line between the moved ends.
fn offset_side(elems: &[Elem], d: f64, miter: bool) -> Vec<Elem> {
    let c = remove_loops(offset_curve_with(elems, d, miter), 6.0 * d.abs() + 0.5);
    if !c.is_empty() {
        return c;
    }
    let (a, b) = (point_at_start(elems), point_at_end(elems));
    let n = left_of((b - a).norm()) * d;
    vec![Elem::Line(a + n, b + n)]
}

/// The curve without its ends; at least its middle fifth stays.
fn cut_both(elems: &[Elem], start: f64, end: f64) -> Vec<Elem> {
    let l = elems_len(elems);
    cut_end(&cut_start(elems, start.min(l * 0.4)), end.min(l * 0.4))
}

fn make_track(elems: Vec<Elem>, width: f64, layer: LayerId, net: NetId) -> Track {
    Track {
        pts: flatten(&elems),
        elems,
        width,
        layer,
        net,
        necks: Vec::new(),
        neck_w: width,
        origin: None,
        resting: false,
        fitted: Vec::new(),
        pattern: None,
        seg: u32::MAX,
        tap: None,
    }
}

/// A track of the path, travelled from `start` on.
#[derive(Clone)]
struct Leg {
    elems: Vec<Elem>,
    layer: LayerId,
}

/// The legs of the wide tracks of one net as paths from pin to pin; layer changes between legs.
fn paths_of(tracks: &[&Track], vias: &[&Via], pins: &[Point]) -> Vec<(usize, Vec<Leg>, usize, Vec<Point>)> {
    let close = |a: Point, b: Point| a.dist(b) < 1e-5;
    let pin_at = |p: Point| pins.iter().position(|&q| close(p, q));
    let mut used = vec![false; tracks.len()];
    let mut out = Vec::new();
    for (pi, &pin) in pins.iter().enumerate() {
        loop {
            let first = (0..tracks.len())
                .find(|&i| !used[i] && (close(tracks[i].pts[0], pin) || close(*tracks[i].pts.last().unwrap(), pin)));
            let Some(mut ti) = first else { break };
            let mut legs: Vec<Leg> = Vec::new();
            let mut joints: Vec<Point> = Vec::new();
            let mut at = pin;
            let end = loop {
                used[ti] = true;
                let t = tracks[ti];
                let forward = close(t.pts[0], at);
                let elems = if forward { t.elems.clone() } else { reversed(&t.elems) };
                let other = if forward { *t.pts.last().unwrap() } else { t.pts[0] };
                legs.push(Leg { elems, layer: t.layer });
                if let Some(k) = pin_at(other) {
                    break Some(k);
                }
                at = other;
                let next = (0..tracks.len())
                    .find(|&i| !used[i] && (close(tracks[i].pts[0], at) || close(*tracks[i].pts.last().unwrap(), at)));
                match next {
                    Some(n) => {
                        // the same layer means a joint, not a layer change
                        if vias.iter().any(|v| close(v.at, at)) {
                            joints.push(at);
                        } else {
                            joints.push(pt(f64::NAN, f64::NAN));
                        }
                        ti = n;
                    }
                    None => break None,
                }
            };
            if let Some(k) = end {
                out.push((pi, legs, k, joints));
            }
        }
    }
    out
}

/// Make the vias of the wide tracks that the split puts side by side a via pair each: the
/// capsule around the two vias, which the relaxation turns with the band, in place of the
/// circle around them that leaves room for vias in line as well. The joint where a twisted
/// pair changes sides keeps the circle. Returns how many.
fn pair_vias(real: &Board, m: &Merged, geo: &mut Geometry) -> usize {
    let probe = split(real, m, geo);
    let mut made = 0;
    for (net, c, half) in probe.side {
        let Some(mp) = m.pairs.iter().find(|p| p.p == net) else {
            continue;
        };
        let (cp, cn) = (real.class_of(mp.p), real.class_of(mp.n));
        let (d, drill) = (cp.via_diameter.max(cn.via_diameter), cp.via_drill.max(cn.via_drill));
        if let Some(v) = geo.vias.iter_mut().find(|v| v.net == net && v.at.dist(c) < 1e-6) {
            let h = half.len();
            v.half = half;
            v.diameter = 2.0 * h + d;
            v.drill = 2.0 * h + drill;
            made += 1;
        }
    }
    made
}

/// Where the tracks of a pair change sides, per via joint (turns back, via pair, length of the
/// shorter leg): vias side by side change them where the band turns back, vias in line where it
/// goes on. In line the far via's track passes the near via, which takes room, so all joints
/// are side by side where that gives the polarity the ends ask for (`twisted`: they ask for a
/// change), else one joint is in line: where the legs are longest, best where the band goes on,
/// not at a via pair unless all are.
fn side_changes(joints: &[(bool, bool, f64)], twisted: bool) -> Vec<bool> {
    let backs = joints.iter().filter(|j| j.0).count();
    let pairs_only = joints.iter().all(|j| j.1);
    let room = |j: &(bool, bool, f64)| j.2 + if j.0 { 0.0 } else { 1.0 };
    let in_line = ((backs % 2 == 1) != twisted)
        .then(|| {
            (0..joints.len())
                .filter(|&k| pairs_only || !joints[k].1)
                .max_by(|&a, &b| room(&joints[a]).total_cmp(&room(&joints[b])))
        })
        .flatten();
    joints
        .iter()
        .enumerate()
        .map(|(k, j)| if in_line == Some(k) { !j.0 } else { j.0 })
        .collect()
}

/// Replace the wide tracks in the geometry with the two tracks of each pair.
fn split(real: &Board, m: &Merged, geo: &Geometry) -> Split {
    let mut out = Geometry {
        tracks: Vec::new(),
        vias: Vec::new(),
    };
    let carriers: HashMap<NetId, &MergedPair> = m.pairs.iter().map(|p| (p.p, p)).collect();
    let rest: HashMap<NetId, NetId> = m.pairs.iter().filter_map(|p| p.rest.map(|r| (r, p.p))).collect();
    for t in &geo.tracks {
        if !carriers.contains_key(&t.net) {
            let mut c = t.clone();
            c.net = rest.get(&t.net).copied().unwrap_or(t.net);
            out.tracks.push(c);
        }
    }
    for v in &geo.vias {
        if !carriers.contains_key(&v.net) {
            let mut c = v.clone();
            c.net = rest.get(&v.net).copied().unwrap_or(v.net);
            out.vias.push(c);
        }
    }
    let around = Around {
        board: real,
        obstacles: (0..real.obstacles.len()).collect(),
        tracks: out.tracks.iter().map(|t| (track_box(t), t.clone())).collect(),
        vias: out.vias.clone(),
    };
    let mut reports = Vec::new();
    let mut unfinished = Vec::new();
    let mut tuned = Vec::new();
    let mut side = Vec::new();
    for (pi, pair) in m.pairs.iter().enumerate() {
        let mut rep = PairReport {
            p: real.nets[pair.p as usize].name.clone(),
            n: real.nets[pair.n as usize].name.clone(),
            wide: true,
            ..Default::default()
        };
        let tracks: Vec<&Track> = geo.tracks.iter().filter(|t| t.net == pair.p).collect();
        let vias: Vec<&Via> = geo.vias.iter().filter(|v| v.net == pair.p).collect();
        let mids: Vec<Point> = pair
            .ends
            .iter()
            .map(|e| e.out.unwrap_or((real.pins[e.p].pos + real.pins[e.n].pos) * 0.5))
            .collect();
        let paths = paths_of(&tracks, &vias, &mids);
        let (cp, cn) = (real.class_of(pair.p), real.class_of(pair.n));
        let (via_d, via_drill) = (cp.via_diameter.max(cn.via_diameter), cp.via_drill.max(cn.via_drill));
        let clr = cp.clearance.max(cn.clearance);
        for (a, legs, b, joints) in &paths {
            let (ea, eb) = (pair.ends[*a], pair.ends[*b]);
            let u0 = tangent(&legs[0].elems[0], false);
            let last = legs.last().unwrap();
            let u1 = tangent(last.elems.last().unwrap(), true);
            // at the far end the pads lie ahead: seen from them the tracks arrive. On the 45
            // degree grid the first piece may run along the row of pads: the side of the pads
            // the band leaves to is where it is a millimetre on
            let (side_a, side_b) = if m.miter {
                let all: Vec<Elem> = legs.iter().flat_map(|l| l.elems.iter().copied()).collect();
                (chord(&all, 1.0), chord(&reversed(&all), 1.0) * -1.0)
            } else {
                (u0, u1)
            };
            let mut exit_a = exit_of(real, ea, side_a);
            let mut exit_b = exit_of(real, eb, side_b * -1.0);
            let (mut left_p, mut left_p_end) = (exit_a.p_left, !exit_b.p_left);
            // at a breakout the band may leave the start point in any direction: the tracks
            // take the sides that do not cross, and where either side would do, the one that
            // spares the pair a change of sides
            let (mut free_a, mut free_b) = (false, false);
            if let Some(c) = ea.out {
                let (w, g, _) = real.pair_shape(pair.p, pair.n, legs[0].layer);
                let s = (w + g) / 2.0;
                (exit_a, left_p, free_a) = breakout_exit(real, ea, c, c + left_of(u0) * s, c - left_of(u0) * s, u0);
            }
            if let Some(c) = eb.out {
                let (w, g, _) = real.pair_shape(pair.p, pair.n, last.layer);
                let s = (w + g) / 2.0;
                (exit_b, left_p_end, free_b) =
                    breakout_exit(real, eb, c, c + left_of(u1) * s, c - left_of(u1) * s, u1 * -1.0);
            }
            if left_p != left_p_end {
                if free_a {
                    left_p = left_p_end;
                } else if free_b {
                    left_p_end = left_p;
                }
            }
            let twisted = left_p != left_p_end;
            let via_joints: Vec<usize> = (0..joints.len()).filter(|&j| joints[j].x.is_finite()).collect();
            if twisted && via_joints.is_empty() {
                rep.note = "polarity is crossed and the wide track has no via".to_string();
                unfinished.push(rep.p.clone());
                unfinished.push(rep.n.clone());
                continue;
            }
            // which joints change the sides of the tracks (`side_changes`)
            let turns_back = |j: usize| {
                let (a, b) = (&legs[j].elems, &legs[j + 1].elems);
                tangent(a.last().unwrap(), true).dot(tangent(&b[0], false)) < 0.0
            };
            let backs: Vec<bool> = (0..joints.len())
                .map(|j| joints[j].x.is_finite() && turns_back(j))
                .collect();
            let capsule = |j: usize| vias.iter().any(|v| v.is_pair() && v.at.dist(joints[j]) < 1e-6);
            let plan: Vec<(bool, bool, f64)> = via_joints
                .iter()
                .map(|&j| {
                    let len = elems_len(&legs[j].elems).min(elems_len(&legs[j + 1].elems));
                    (backs[j], capsule(j), len)
                })
                .collect();
            let mut swaps = vec![false; joints.len()];
            for (&j, s) in via_joints.iter().zip(side_changes(&plan, twisted)) {
                swaps[j] = s;
            }
            let first_track = out.tracks.len();
            let mut joint_vias: Vec<Option<[usize; 2]>> = Vec::new();
            // walk the legs: which net runs on the left, and where each track starts and ends
            let mut left_is_p = left_p;
            let mut ln_start = pad_target(real, ea, left_is_p);
            let mut rn_start = pad_target(real, ea, !left_is_p);
            let (mut lead_l, mut lead_r): (Vec<Point>, Vec<Point>) = (Vec::new(), Vec::new());
            let mut reach_start = None;
            for (k, leg) in legs.iter().enumerate() {
                let (w, g, via_gap) = real.pair_shape(pair.p, pair.n, leg.layer);
                rep.width = rep.width.max(w);
                rep.gap = rep.gap.max(g);
                let s = (w + g) / 2.0;
                let net_l = if left_is_p { pair.p } else { pair.n };
                let net_r = if left_is_p { pair.n } else { pair.p };
                let disc = via_disc(via_d, via_gap, s + w / 2.0, clr);
                let (full_l, full_r) = (
                    offset_side(&leg.elems, s, m.miter),
                    offset_side(&leg.elems, -s, m.miter),
                );
                let mut start_trim = reach_start.unwrap_or(0.0);
                if reach_start.is_none() {
                    // a breakout starts the band past the tips of the pads already
                    let base = if ea.out.is_some() {
                        w + clr
                    } else {
                        exit_trim(&exit_a, u0, s, w, clr)
                    };
                    let sides = [
                        (ln_start, net_l, full_l.as_slice()),
                        (rn_start, net_r, full_r.as_slice()),
                    ];
                    let (more, t) = best_exit(&around, &exit_a, sides, base, ea.out.is_some(), w, clr, leg.layer);
                    let out = exit_a.x * (exit_a.ext + more);
                    lead_l = vec![ln_start + out];
                    lead_r = vec![rn_start + out];
                    start_trim = t;
                }
                // a via pair side by side: the tracks go to their vias within its capsule
                let tight = k + 1 < legs.len() && joints[k].x.is_finite() && capsule(k) && {
                    let (ua, ub) = (
                        tangent(leg.elems.last().unwrap(), true),
                        tangent(&legs[k + 1].elems[0], false),
                    );
                    (ua.dot(ub) < 0.0) == swaps[k]
                };
                let joint_trim = if tight { via_d / 2.0 + 0.02 } else { disc + 0.02 };
                let mut end_trim = joint_trim;
                let mut exits = None;
                if k + 1 == legs.len() {
                    let base = if eb.out.is_some() {
                        w + clr
                    } else {
                        exit_trim(&exit_b, u1, s, w, clr)
                    };
                    let (tl, tr) = (pad_target(real, eb, left_is_p), pad_target(real, eb, !left_is_p));
                    let (bl, br) = (reversed(&full_l), reversed(&full_r));
                    let sides = [(tl, net_l, bl.as_slice()), (tr, net_r, br.as_slice())];
                    let (more, t) = best_exit(&around, &exit_b, sides, base, eb.out.is_some(), w, clr, leg.layer);
                    let out = exit_b.x * (exit_b.ext + more);
                    end_trim = t;
                    exits = Some((tl + out, tr + out));
                }
                let cl = cut_both(&full_l, start_trim, end_trim);
                let cr = cut_both(&full_r, start_trim, end_trim);
                let start_l = |c: &[Elem]| {
                    let mut v: Vec<Elem> = Vec::new();
                    let mut from = ln_start;
                    for &q in &lead_l {
                        v.push(Elem::Line(from, q));
                        from = q;
                    }
                    v.push(Elem::Line(from, point_at_start(c)));
                    v
                };
                let start_r = |c: &[Elem]| {
                    let mut v: Vec<Elem> = Vec::new();
                    let mut from = rn_start;
                    for &q in &lead_r {
                        v.push(Elem::Line(from, q));
                        from = q;
                    }
                    v.push(Elem::Line(from, point_at_start(c)));
                    v
                };
                let (mut el, mut er) = (start_l(&cl), start_r(&cr));
                el.extend(cl.iter().copied());
                er.extend(cr.iter().copied());
                if k + 1 == legs.len() {
                    let (tl, tr) = (pad_target(real, eb, left_is_p), pad_target(real, eb, !left_is_p));
                    let (ql, qr) = exits.unwrap();
                    el.push(Elem::Line(point_at_end(&cl), ql));
                    el.push(Elem::Line(ql, tl));
                    er.push(Elem::Line(point_at_end(&cr), qr));
                    er.push(Elem::Line(qr, tr));
                    out.tracks.push(make_track(el, w, leg.layer, net_l));
                    out.tracks.push(make_track(er, w, leg.layer, net_r));
                    break;
                }
                // the joint to the next leg
                let c = joints[k];
                let next = &legs[k + 1];
                let (ua, ub) = (tangent(leg.elems.last().unwrap(), true), tangent(&next.elems[0], false));
                if !c.x.is_finite() {
                    // two tracks of one layer in a row
                    let (jl, jr) = (point_at_end(&cl), point_at_end(&cr));
                    out.tracks.push(make_track(el, w, leg.layer, net_l));
                    out.tracks.push(make_track(er, w, leg.layer, net_r));
                    joint_vias.push(None);
                    ln_start = jl;
                    rn_start = jr;
                    lead_l.clear();
                    lead_r.clear();
                    reach_start = Some(0.0);
                    continue;
                }
                // a band that turns back at the via comes out on the other side of itself: there
                // vias side by side swap the tracks and vias in line keep them
                let back = ua.dot(ub) < 0.0;
                if std::env::var("GB_PAIR_LOG").is_ok() {
                    eprintln!(
                        "joint {} at {:.2},{:.2}: capsule {} back {} swap {} turn {:.2}",
                        rep.p,
                        c.x,
                        c.y,
                        capsule(k),
                        back,
                        swaps[k],
                        ua.dot(ub)
                    );
                }
                let t = if back { (ua - ub).norm() } else { (ua + ub).norm() };
                let n = left_of(t);
                let h = (via_d + via_gap) / 2.0;
                let swap = swaps[k];
                let (near, far);
                if swap != back {
                    // vias in line on the left track: it ends on the near via, the right track
                    // passes that via and ends on the far one
                    let q = via_d / 2.0 + clr + w / 2.0 + 0.01;
                    let side = c + n * (s - q);
                    let along = h + 0.42 * q;
                    (near, far) = (c - t * h + n * s, c + t * h + n * s);
                    let (w1, w2) = (side - t * along, side);
                    el.push(Elem::Line(point_at_end(&cl), near));
                    er.push(Elem::Line(point_at_end(&cr), w1));
                    er.push(Elem::Line(w1, w2));
                    er.push(Elem::Line(w2, far));
                    lead_l.clear();
                    if back {
                        // on the way back the far via's track passes the near via outside
                        let other = c + n * (s + q);
                        lead_r = vec![other, other - t * along];
                    } else {
                        // the near via's track passes the far via on the right
                        lead_r = vec![w2, side + t * along];
                    }
                } else {
                    (near, far) = (c + n * h, c - n * h);
                    // (a band that turns nearly square could turn the other way in the end)
                    if ua.dot(ub).abs() >= 0.2 {
                        side.push((pair.p, c, n * h));
                    }
                    el.push(Elem::Line(point_at_end(&cl), near));
                    er.push(Elem::Line(point_at_end(&cr), far));
                    lead_l.clear();
                    lead_r.clear();
                }
                out.tracks.push(make_track(el, w, leg.layer, net_l));
                out.tracks.push(make_track(er, w, leg.layer, net_r));
                joint_vias.push(Some([out.vias.len(), out.vias.len() + 1]));
                for (at, net) in [(near, net_l), (far, net_r)] {
                    out.vias.push(Via {
                        at,
                        diameter: via_d,
                        drill: via_drill,
                        net,
                        layer: None,
                        origin: None,
                        resting: false,
                        half: Default::default(),
                    });
                }
                if swap {
                    left_is_p = !left_is_p;
                    rep.swaps += 1;
                }
                // the track of the net that runs on the left from now on starts on its via
                (ln_start, rn_start) = if swap { (far, near) } else { (near, far) };
                if swap && back {
                    std::mem::swap(&mut lead_l, &mut lead_r);
                }
                rep.vias += 1;
                reach_start = Some(joint_trim);
            }
            tuned.push(PairPath {
                report: pi,
                p: pair.p,
                n: pair.n,
                tracks: (first_track..out.tracks.len()).collect(),
                vias: joint_vias,
                whole: pair.ends.len() == real.nets[pair.p as usize].pins.len()
                    && pair.ends.len() == real.nets[pair.n as usize].pins.len(),
                ends: [ea, eb],
            });
        }
        // ends that no wide track reached, and pins without partner
        let reached: Vec<usize> = paths.iter().flat_map(|(a, _, b, _)| [*a, *b]).collect();
        let open = (0..pair.ends.len()).any(|i| !reached.contains(&i));
        let free = real.nets[pair.p as usize].pins.len() > pair.ends.len()
            || real.nets[pair.n as usize].pins.len() > pair.ends.len();
        if open || free {
            if open {
                rep.note = "no way for the wide track".to_string();
            } else if rep.note.is_empty() {
                rep.note = "pins without partner are connected as single tracks".to_string();
            }
            unfinished.push(rep.p.clone());
            unfinished.push(rep.n.clone());
        }
        reports.push(rep);
    }
    Split {
        geo: out,
        reports,
        unfinished,
        paths: tuned,
        side,
    }
}

/// The tracks of one path from pad to pad.
struct PairPath {
    report: usize,
    p: NetId,
    n: NetId,
    /// Left and right track per leg, in the order of the path; finished tracks join at the end.
    tracks: Vec<usize>,
    /// The two vias behind each leg but the last, if it ends on vias.
    vias: Vec<Option<[usize; 2]>>,
    /// The nets have no other pins: their whole length counts.
    whole: bool,
    /// The pair ends at the start and the end of the path.
    ends: [PairEnd; 2],
}

/// A track of a pair that lost its way to the pad: pad, and where the track starts now.
struct Cut {
    /// Index of the path, and whether the cut is at its start.
    path: usize,
    at_start: bool,
    pad: usize,
    net: NetId,
    at: Point,
    layer: LayerId,
    width: f64,
}

struct Split {
    geo: Geometry,
    reports: Vec<PairReport>,
    unfinished: Vec<String>,
    paths: Vec<PairPath>,
    /// The joints with vias side by side: wide track, centre and the half line to the via of
    /// the left track.
    side: Vec<(NetId, Point, Point)>,
}

fn track_len(t: &Track) -> f64 {
    t.pts.windows(2).map(|w| w[0].dist(w[1])).sum()
}

/// How far from the ends of a path a clearance problem is worth giving up the wide track.
const REPAIR_REACH: f64 = 6.0;

/// Next to the pads and vias the two tracks do not always fit, the wide track knew nothing of
/// them. Where a path has clearance problems near one of its ends, take its tracks and vias
/// away up to the last problem: single tracks connect the rest. The paths in `given_up` go
/// completely (their nets are routed as single tracks); the ends in `kept` (path, start or
/// end) stay as they are. Returns the nets concerned.
fn repair(
    real: &Board,
    geo: &mut Geometry,
    paths: &[PairPath],
    given_up: &[usize],
    kept: &[(usize, bool)],
) -> (Vec<String>, Vec<Cut>) {
    let viol = crate::check::check(real, geo);
    let mut names = Vec::new();
    let mut cuts = Vec::new();
    let mut gone_vias: Vec<usize> = Vec::new();
    for (pi, path) in paths.iter().enumerate() {
        let give_up = given_up.contains(&pi);
        let hits: Vec<Point> = viol
            .iter()
            .filter(|v| [path.p, path.n].iter().any(|&x| v.nets.0 == x || v.nets.1 == Some(x)))
            .map(|v| v.at)
            .collect();
        if hits.is_empty() && !give_up {
            continue;
        }
        let legs: Vec<[usize; 2]> = path.tracks.chunks(2).map(|c| [c[0], c[1]]).collect();
        let lens: Vec<f64> = legs
            .iter()
            .map(|l| (track_len(&geo.tracks[l[0]]) + track_len(&geo.tracks[l[1]])) / 2.0)
            .collect();
        let total: f64 = lens.iter().sum();
        let (mut from_start, mut from_end) = (0.0f64, 0.0f64);
        for h in hits {
            let mut best = (0.7, None);
            let mut before = 0.0;
            for (k, l) in legs.iter().enumerate() {
                for &ti in l {
                    let mut along = before;
                    for w in geo.tracks[ti].pts.windows(2) {
                        let d = crate::geom::point_seg_dist(h, w[0], w[1]);
                        if d < best.0 {
                            best = (d, Some(along));
                        }
                        along += w[0].dist(w[1]);
                    }
                }
                before += lens[k];
            }
            let Some(s) = best.1 else { continue };
            if s <= REPAIR_REACH && s <= total - s {
                from_start = from_start.max(s + 0.5);
            } else if total - s <= REPAIR_REACH {
                from_end = from_end.max(total - s + 0.5);
            }
        }
        if give_up {
            (from_start, from_end) = (total + 1.0, 0.0);
        }
        if kept.contains(&(pi, true)) {
            from_start = 0.0;
        }
        if kept.contains(&(pi, false)) {
            from_end = 0.0;
        }
        if from_start == 0.0 && from_end == 0.0 {
            continue;
        }
        names.push(real.nets[path.p as usize].name.clone());
        names.push(real.nets[path.n as usize].name.clone());
        let mut cut = |geo: &mut Geometry, legs_in_order: Vec<usize>, mut reach: f64, at_start: bool| {
            for k in legs_in_order {
                if reach <= 0.0 {
                    break;
                }
                let via = if at_start {
                    path.vias.get(k).copied().flatten()
                } else {
                    k.checked_sub(1).and_then(|j| path.vias[j])
                };
                if lens[k] <= reach {
                    for &ti in &legs[k] {
                        geo.tracks[ti].elems.clear();
                        geo.tracks[ti].pts.clear();
                    }
                    gone_vias.extend(via.into_iter().flatten());
                    reach -= lens[k];
                } else {
                    for &ti in &legs[k] {
                        let t = &mut geo.tracks[ti];
                        t.elems = if at_start {
                            cut_start(&t.elems, reach)
                        } else {
                            cut_end(&t.elems, reach)
                        };
                        t.pts = flatten(&t.elems);
                    }
                    reach = 0.0;
                }
            }
        };
        if from_start > 0.0 {
            cut(geo, (0..legs.len()).collect(), from_start, true);
        }
        if from_end > 0.0 {
            cut(geo, (0..legs.len()).rev().collect(), from_end, false);
        }
        // where the rest of each track starts now, and the pad it has to reach
        for (at_start, reach) in [(true, from_start), (false, from_end)] {
            if reach == 0.0 {
                continue;
            }
            let order: Vec<usize> = if at_start {
                (0..legs.len()).collect()
            } else {
                (0..legs.len()).rev().collect()
            };
            let whole_leg = |g: &Geometry, k: usize| legs[k].iter().all(|&ti| g.tracks[ti].pts.len() >= 2);
            let Some(&k) = order.iter().find(|&&k| whole_leg(geo, k)) else {
                continue;
            };
            // a leg that lost only one of its tracks is gone as well
            for &j in order.iter().take_while(|&&j| j != k) {
                for &ti in &legs[j] {
                    geo.tracks[ti].elems.clear();
                    geo.tracks[ti].pts.clear();
                }
            }
            let end = path.ends[if at_start { 0 } else { 1 }];
            for &ti in &legs[k] {
                let t = &geo.tracks[ti];
                let at = if at_start { t.pts[0] } else { *t.pts.last().unwrap() };
                cuts.push(Cut {
                    path: pi,
                    at_start,
                    pad: if t.net == path.p { end.p } else { end.n },
                    net: t.net,
                    at,
                    layer: t.layer,
                    width: t.width,
                });
            }
        }
    }
    gone_vias.sort_unstable();
    gone_vias.dedup();
    for &v in gone_vias.iter().rev() {
        geo.vias[v].diameter = 0.0;
    }
    geo.vias.retain(|v| v.diameter > 0.0);
    (names, cuts)
}

/// In the board of the last pass every cut becomes a net of its own, from the pad to the end of
/// the track, so that the search goes there and does not ride the pair's other copper. Returns
/// the real net of each new net.
fn cut_nets(b: &mut Board, cuts: &[Cut], kept_from: usize) -> HashMap<NetId, NetId> {
    let mut map = HashMap::new();
    for (i, c) in cuts.iter().enumerate() {
        let id = b.nets.len() as NetId;
        map.insert(id, c.net);
        let class = b.nets[c.net as usize].class;
        b.nets[c.net as usize].pins.retain(|&x| x != c.pad);
        for o in b.pins[c.pad].obstacles.clone() {
            b.obstacles[o].net = Some(id);
        }
        b.pins[c.pad].net = id;
        // the last bit of the track belongs to the new net, the end must not repel its own net
        for o in b.obstacles[kept_from..].iter_mut() {
            if o.net == Some(c.net) && o.shape.dist_to_point(c.at) < 1.5 {
                o.net = Some(id);
            }
        }
        let (obs, pin) = (b.obstacles.len(), b.pins.len());
        let clearance = b.classes[class as usize].clearance.max(b.rules.min_clearance);
        b.obstacles.push(Obstacle {
            shape: Shape::Circle {
                c: c.at,
                r: c.width / 2.0,
            },
            layers: LayerSet::single(c.layer),
            net: Some(id),
            kind: ObstacleKind::Pad,
            hole: None,
            clearance,
            pin: Some(pin),
            locked: true,
        });
        b.pins.push(Pin {
            name: format!("cut{i}"),
            footprint: String::new(),
            net: id,
            pos: c.at,
            layers: LayerSet::single(c.layer),
            obstacles: vec![obs],
            size: (c.width, c.width),
        });
        b.nets.push(crate::board::Net {
            name: format!("{}~cut{i}", b.nets[c.net as usize].name),
            class,
            pins: vec![c.pad, pin],
            planes: LayerSet::default(),
            priority: 0,
        });
    }
    map
}

/// Around the pads of a join, the copper of their net there belongs to the join (mm).
const JOIN_REACH: f64 = 1.0;

/// A join longer than this many times the distance of its pads, plus [`JOIN_SLACK`] mm, went a
/// long way round: its pins join their net in the last pass instead.
const JOIN_DETOUR: f64 = 3.0;
const JOIN_SLACK: f64 = 6.0;

/// In the board of the last pass every join (see [`Merged::joins`]) becomes a net of its own,
/// from pad to pad with the copper of their net right at the pads, so that the search joins
/// them there and does not ride the pair's other copper. A pad that is in a net of a cut or an
/// earlier join already (from `first_new` on) brings the other pad into that net. Returns the
/// real net of each new net.
fn join_nets(b: &mut Board, joins: &[(usize, usize)], kept_from: usize, first_new: usize) -> HashMap<NetId, NetId> {
    let mut map = HashMap::new();
    let move_pin = |b: &mut Board, pin: usize, to: NetId| {
        let from = b.pins[pin].net;
        b.nets[from as usize].pins.retain(|&x| x != pin);
        b.nets[to as usize].pins.push(pin);
        b.pins[pin].net = to;
        for o in b.pins[pin].obstacles.clone() {
            b.obstacles[o].net = Some(to);
        }
    };
    for (i, &(a, c)) in joins.iter().enumerate() {
        let (na, nc) = (b.pins[a].net, b.pins[c].net);
        if nc as usize >= first_new {
            move_pin(b, a, nc);
            continue;
        }
        if na as usize >= first_new {
            move_pin(b, c, na);
            continue;
        }
        let id = b.nets.len() as NetId;
        map.insert(id, na);
        b.nets.push(crate::board::Net {
            name: format!("{}~join{i}", b.nets[na as usize].name),
            class: b.nets[na as usize].class,
            pins: Vec::new(),
            planes: LayerSet::default(),
            priority: 0,
        });
        for pin in [a, c] {
            let at = b.pins[pin].pos;
            move_pin(b, pin, id);
            for o in b.obstacles[kept_from..].iter_mut() {
                if o.net == Some(na) && o.shape.dist_to_point(at) < JOIN_REACH {
                    o.net = Some(id);
                }
            }
        }
    }
    map
}

/// Extra length of a bump of height h with 45 degree flanks.
const BUMP_GAIN: f64 = 2.0 * (std::f64::consts::SQRT_2 - 1.0);

/// Make the two nets of each path equally long: the shorter one gets bumps on its outer side,
/// where a straight piece has room for them. Returns the skew left per path.
fn tune(real: &Board, geo: &mut Geometry, paths: &[PairPath], limit: f64) -> Vec<(usize, f64)> {
    let mut around = Around {
        board: real,
        obstacles: (0..real.obstacles.len()).collect(),
        tracks: geo.tracks.iter().map(|t| (track_box(t), t.clone())).collect(),
        vias: geo.vias.clone(),
    };
    let mut left = Vec::new();
    for path in paths {
        let length = |g: &Geometry, net: NetId| {
            path.tracks
                .iter()
                .map(|&i| &g.tracks[i])
                .filter(|t| t.net == net)
                .map(track_len)
                .sum::<f64>()
        };
        let (lp, ln) = (length(geo, path.p), length(geo, path.n));
        let (short, other) = if lp < ln { (path.p, path.n) } else { (path.n, path.p) };
        let mut need = (lp - ln).abs();
        let cls = real.class_of(path.p);
        let limit = real.pair_skew_limit(path.p, path.n).map_or(limit, |s| s.min(limit));
        if need > limit {
            let clr = cls.clearance.max(real.class_of(path.n).clearance);
            // the longest straight pieces first
            let mut lines: Vec<(usize, usize, f64)> = Vec::new();
            for &ti in &path.tracks {
                let t = &geo.tracks[ti];
                if t.net == short {
                    for (ei, e) in t.elems.iter().enumerate() {
                        if matches!(e, Elem::Line(..)) && elem_len(e) >= 1.0 {
                            lines.push((ti, ei, elem_len(e)));
                        }
                    }
                }
            }
            lines.sort_by(|a, b| b.2.total_cmp(&a.2).then(a.0.cmp(&b.0)).then(a.1.cmp(&b.1)));
            // indices shift when an element is replaced: from the back of a track to the front
            let mut replace: Vec<(usize, usize, Vec<Elem>)> = Vec::new();
            for (ti, ei, _) in lines {
                if need <= limit * 0.5 {
                    break;
                }
                let t = &geo.tracks[ti];
                let Elem::Line(a, b) = t.elems[ei] else { continue };
                let (w, layer) = (t.width, t.layer);
                let partner: Vec<Point> = path
                    .tracks
                    .iter()
                    .map(|&i| &geo.tracks[i])
                    .filter(|x| x.net == other && x.layer == layer)
                    .flat_map(|x| x.pts.iter().copied())
                    .collect();
                let u = (b - a).norm();
                let mid = a.lerp(b, 0.5);
                let near = |q: Point| partner.iter().map(|p| p.dist(q)).fold(f64::INFINITY, f64::min);
                // away from the partner
                let probe = left_of(u) * (2.0 * w);
                let out = if near(mid + probe) < near(mid - probe) {
                    probe * -0.5
                } else {
                    probe * 0.5
                } * (1.0 / w);
                let (new, gained) = bumps(&around, short, a, b, u, out.norm(), w, clr, layer, need - limit * 0.25);
                if gained > 0.0 {
                    need -= gained;
                    let t2 = make_track(new.clone(), w, layer, short);
                    around.tracks.push((track_box(&t2), t2));
                    replace.push((ti, ei, new));
                }
            }
            replace.sort_by(|a, b| b.0.cmp(&a.0).then(b.1.cmp(&a.1)));
            for (ti, ei, new) in replace {
                let t = &mut geo.tracks[ti];
                t.elems.splice(ei..=ei, new);
                t.pts = flatten(&t.elems);
            }
        }
        left.push((path.report, need));
    }
    left
}

/// The line a-b with bumps towards `out` that add up to at most `want` mm, as far as they keep
/// their clearance: the elements and the length gained.
#[allow(clippy::too_many_arguments)]
fn bumps(
    around: &Around,
    net: NetId,
    a: Point,
    b: Point,
    u: Point,
    out: Point,
    w: f64,
    clr: f64,
    layer: LayerId,
    want: f64,
) -> (Vec<Elem>, f64) {
    let len = a.dist(b);
    // a bump is a point without a flat top: no piece of it runs parallel to the other track at
    // a wider gap (KiCad's DRC counts such a piece as coupled at the wrong gap), its two slopes
    // count as uncoupled
    let mut from = 0.3;
    let mut cur = a;
    let mut elems = Vec::new();
    let mut gained = 0.0;
    let edge = around.board.rules.edge_clearance + w / 2.0;
    while gained < want {
        let rest = want - gained;
        let mut placed = false;
        for h in [4.0 * w, 2.5 * w, 1.5 * w] {
            let h = h.min(rest / BUMP_GAIN);
            if h < 0.02 {
                break;
            }
            let span = 2.0 * h;
            if from + span + 0.3 > len {
                continue;
            }
            let p0 = a + u * from;
            let p1 = p0 + u * h + out * h;
            let p3 = p1 + u * h - out * h;
            // around the bump: its copper and the clearance on the far side
            let free = [(p0, p1), (p1, p3)]
                .iter()
                .all(|&(x, y)| around.slack(net, x, y, w, clr, layer) >= 0.02)
                && around.board.edge_dist(p1) >= edge;
            if free {
                elems.push(Elem::Line(cur, p0));
                elems.push(Elem::Line(p0, p1));
                elems.push(Elem::Line(p1, p3));
                cur = p3;
                gained += h * BUMP_GAIN;
                from += span + 4.0 * w;
                placed = true;
                break;
            }
        }
        if !placed {
            from += 2.0 * w;
            if from + 0.6 > len {
                break;
            }
        }
    }
    elems.push(Elem::Line(cur, b));
    (elems, gained)
}

fn pad_target(b: &Board, e: PairEnd, p: bool) -> Point {
    b.pins[if p { e.p } else { e.n }].pos
}

/// How the two tracks leave an end of a pair: straight along the pads, towards the wide track,
/// and only past the tips of the pads over to the wide track.
struct Exit {
    /// Unit direction away from the pads.
    x: Point,
    /// Distance from the pad centres to just past the tips of the pads, where the tracks may turn.
    ext: f64,
    /// True if the P pad lies on the left when looking along `x`.
    p_left: bool,
}

fn exit_of(b: &Board, e: PairEnd, u: Point) -> Exit {
    let (pp, pn) = (b.pins[e.p].pos, b.pins[e.n].pos);
    let a = (pp - pn).norm();
    let mut x = left_of(a);
    if x.dot(u) < 0.0 {
        x = x * -1.0;
    }
    exit_along(b, e, x)
}

fn exit_along(b: &Board, e: PairEnd, x: Point) -> Exit {
    let (pp, pn) = (b.pins[e.p].pos, b.pins[e.n].pos);
    let reach = |pin: usize| {
        b.pins[pin]
            .obstacles
            .iter()
            .flat_map(|&o| b.obstacles[o].shape.to_polygon(b.rules.max_error).pts)
            .map(|q| (q - b.pins[pin].pos).dot(x))
            .fold(0.0, f64::max)
    };
    Exit {
        x,
        ext: reach(e.p).max(reach(e.n)) + 0.06,
        p_left: cross(x, pp - pn) > 0.0,
    }
}

/// At a breakout the wide track starts at `c`, outside the pads, and may leave in any
/// direction `v`. The tracks go straight out of the pads and then over to their curves, which
/// begin some way along the band; which of them takes the left track of the band (`left` and
/// `right` are where the two tracks start) is what keeps the two links from crossing, else the
/// shorter. Returns the exit, whether P takes `left`, and whether the other choice would not
/// cross either.
fn breakout_exit(b: &Board, e: PairEnd, c: Point, left: Point, right: Point, v: Point) -> (Exit, bool, bool) {
    let (left, right) = (left + v, right + v);
    let (pp, pn) = (b.pins[e.p].pos, b.pins[e.n].pos);
    let ex = exit_along(b, e, (c - (pp + pn) * 0.5).norm());
    let link = |pad: Point, to: Point| (pad + ex.x * ex.ext, to);
    let crossed = |p_left: bool| {
        let (tp, tn) = if p_left { (left, right) } else { (right, left) };
        let (a, b) = (link(pp, tp), link(pn, tn));
        crate::geom::seg_seg_dist(a.0, a.1, b.0, b.1) < 1e-6
    };
    let length = |p_left: bool| {
        let (tp, tn) = if p_left { (left, right) } else { (right, left) };
        (pp + ex.x * ex.ext).dist(tp) + (pn + ex.x * ex.ext).dist(tn)
    };
    let (a, b) = (crossed(true), crossed(false));
    let p_left = match (a, b) {
        (false, true) => true,
        (true, false) => false,
        _ => length(true) <= length(false),
    };
    (ex, p_left, a == b)
}

/// Arc length to walk along a band of direction u until both tracks are past the tips of the
/// pads by a track and its clearance, so that no track runs over a tip of the other net's pad.
fn exit_trim(ex: &Exit, u: Point, s: f64, w: f64, clr: f64) -> f64 {
    let cos = ex.x.dot(u).abs().max(0.3);
    let sin = (1.0 - cos * cos).sqrt();
    (ex.ext + w / 2.0 + clr + s * sin + 0.03) / cos
}

/// Room left around the line a-b of a track of `net`: negative if it comes too close to copper
/// of another net.
struct Around<'a> {
    board: &'a Board,
    /// Indices of the obstacles.
    obstacles: Vec<usize>,
    /// Finished tracks with their bounding boxes.
    tracks: Vec<(crate::geom::Rect, Track)>,
    vias: Vec<Via>,
}

fn track_box(t: &Track) -> crate::geom::Rect {
    let mut r = crate::geom::Rect::empty();
    for &q in &t.pts {
        r.add(q);
    }
    r.grow(t.width / 2.0)
}

impl<'a> Around<'a> {
    /// What lies within r of a point.
    fn near(&self, at: Point, r: f64) -> Around<'a> {
        let mut bb = crate::geom::Rect::empty();
        bb.add(at);
        let bb = bb.grow(r);
        Around {
            board: self.board,
            obstacles: self
                .obstacles
                .iter()
                .copied()
                .filter(|&i| self.board.obstacles[i].shape.bbox().overlaps(&bb))
                .collect(),
            tracks: self.tracks.iter().filter(|(b, _)| b.overlaps(&bb)).cloned().collect(),
            vias: self.vias.iter().filter(|v| bb.contains(v.at)).cloned().collect(),
        }
    }

    fn slack(&self, net: NetId, p: Point, q: Point, w: f64, clr: f64, layer: LayerId) -> f64 {
        let mut bb = crate::geom::Rect::empty();
        bb.add(p);
        bb.add(q);
        let bb = bb.grow(1.0);
        let pads = self
            .obstacles
            .iter()
            .map(|&i| &self.board.obstacles[i])
            .filter(|o| o.layers.has(layer) && o.net != Some(net) && o.shape.bbox().overlaps(&bb))
            .map(|o| o.shape.dist_to_seg(p, q) - o.clearance.max(clr) - w / 2.0)
            .fold(f64::INFINITY, f64::min);
        self.tracks
            .iter()
            .filter(|(r, t)| t.layer == layer && t.net != net && r.overlaps(&bb))
            .flat_map(|(_, t)| {
                t.pts
                    .windows(2)
                    .map(|v| crate::geom::seg_seg_dist(p, q, v[0], v[1]) - clr - (w + t.width) / 2.0)
            })
            .fold(pads, f64::min)
            .min(
                self.vias
                    .iter()
                    .filter(|v| v.net != net && bb.grow(v.diameter).contains(v.at))
                    .map(|v| crate::geom::point_seg_dist(v.at, p, q) - clr - (w + v.diameter) / 2.0)
                    .fold(f64::INFINITY, f64::min),
            )
    }
}

/// How to go from the pads of an end to the two curves (which start at that end): how far
/// straight out of the pads and how much of the curves to skip, the same for both tracks, so
/// that they keep their clearance on the links and on the next 3 mm of the curves. The first
/// choice that does, else the one with the most room.
#[allow(clippy::too_many_arguments)]
fn best_exit(
    around: &Around,
    ex: &Exit,
    sides: [(Point, NetId, &[Elem]); 2],
    base: f64,
    breakout: bool,
    w: f64,
    clr: f64,
    layer: LayerId,
) -> (f64, f64) {
    let near = around.near(sides[0].0, 8.0);
    let mut best = (0.0, base);
    let mut room = f64::NEG_INFINITY;
    // a breakout starts with a short trim and may need longer ones
    let factors: &[f64] = if breakout {
        &[1.0, 1.3, 1.7, 2.2, 3.0, 4.0, 5.5, 7.0]
    } else {
        &[1.0, 1.3, 1.7, 2.2, 3.0]
    };
    for &f in factors {
        for more in [0.0, 0.1, 0.2] {
            let t = base * f;
            let mut r = f64::INFINITY;
            let mut links = Vec::new();
            for &(pad, net, curve) in &sides {
                let q = pad + ex.x * (ex.ext + more);
                let cut = cut_start(curve, t.min(elems_len(curve) * 0.4));
                if cut.is_empty() {
                    continue;
                }
                let to = point_at_start(&cut);
                links.push((pad, q, to));
                r = r.min(near.slack(net, q, to, w, clr, layer));
                r = r.min(near.slack(net, pad + ex.x * ex.ext, q, w, clr, layer));
                let shown = cut_end(&cut, (elems_len(&cut) - 3.0).max(0.0));
                for v in flatten(&shown).windows(2) {
                    r = r.min(near.slack(net, v[0], v[1], w, clr, layer));
                }
            }
            if let [a, b] = links[..] {
                r = r.min(crate::geom::seg_seg_dist(a.1, a.2, b.1, b.2) - clr - w);
            }
            if r > room {
                (best, room) = ((more, t), r);
            }
            if r >= 0.01 {
                return (more, t);
            }
        }
    }
    best
}

/// Routed geometry as locked copper, for a later pass around it.
pub fn lock_geometry(b: &mut Board, geo: &Geometry) {
    let routing = b.routing_layers();
    for t in &geo.tracks {
        let clearance = b.class_of(t.net).clearance.max(b.rules.min_clearance);
        for w in t.pts.windows(2) {
            b.obstacles.push(Obstacle {
                shape: Shape::Capsule {
                    a: w[0],
                    b: w[1],
                    r: t.width / 2.0,
                },
                layers: LayerSet::single(t.layer),
                net: Some(t.net),
                kind: ObstacleKind::Track,
                hole: None,
                clearance,
                pin: None,
                locked: true,
            });
        }
    }
    for v in &geo.vias {
        b.obstacles.push(Obstacle {
            shape: Shape::Circle {
                c: v.at,
                r: v.diameter / 2.0,
            },
            layers: LayerSet(routing.0 | b.all_layers().0),
            net: Some(v.net),
            kind: ObstacleKind::Via,
            hole: Some(v.drill / 2.0),
            clearance: b.class_of(v.net).clearance.max(b.rules.min_clearance),
            pin: None,
            locked: true,
        });
    }
}

/// Route a board with its differential pairs as wide tracks. Pairs that cannot be merged and
/// connections the wide tracks leave open are routed as single tracks.
pub fn route_pairs(
    board: &Board,
    sub_cfg: &SubstrateConfig,
    base: &RouterConfig,
    search: &SearchConfig,
    cfg: &PairConfig,
) -> (Substrate, RouteResult, Vec<PairReport>) {
    let mut merged = merge(board, cfg.all, cfg.breakouts);
    merged.miter = cfg.angles45;
    let (sub, mut res) = route_board(&merged.board, sub_cfg, base, search);
    let real = if res.offsets.is_empty() {
        board.clone()
    } else {
        board.with_offsets(&res.offsets)
    };
    if cfg.via_pairs {
        let t0 = std::time::Instant::now();
        let made = pair_vias(&real, &merged, &mut res.geometry);
        if made > 0 {
            let moved;
            let mb = if res.offsets.is_empty() {
                &merged.board
            } else {
                moved = merged.board.with_offsets(&res.offsets);
                &moved
            };
            // the wide tracks with via pairs take the room the circles leave; the rest sleeps
            // until they come too close
            let with: Vec<NetId> = res
                .geometry
                .vias
                .iter()
                .filter(|v| v.is_pair())
                .map(|v| v.net)
                .collect();
            for t in res.geometry.tracks.iter_mut() {
                t.resting = !with.contains(&t.net);
            }
            for v in res.geometry.vias.iter_mut() {
                v.resting = !with.contains(&v.net);
            }
            crate::relax::relax(mb, &sub, &mut res.geometry, &crate::relax::RelaxConfig::default());
        }
        eprintln!("via pairs: {made} side by side, {:.1} s", t0.elapsed().as_secs_f64());
    }
    if let Some(tc) = cfg.room.as_ref().filter(|_| !board.match_rules.is_empty()) {
        let t0 = std::time::Instant::now();
        // the lengths of the real tracks decide who needs room, the band makes it
        let probe = split(&real, &merged, &res.geometry);
        let bands: Vec<(NetId, NetId)> = merged.pairs.iter().map(|p| (p.p, p.n)).collect();
        let want = crate::tune::room_wanted(&real, &probe.geo, tc, &bands);
        if !want.is_empty() {
            let moved;
            let mb = if res.offsets.is_empty() {
                &merged.board
            } else {
                moved = merged.board.with_offsets(&res.offsets);
                &moved
            };
            // the two tracks of a moved band may not fit next to vias and pads where the band
            // did: judged on the split tracks
            let judge = |g: &Geometry| crate::check::spots(&crate::check::check(&real, &split(&real, &merged, g).geo));
            let made = crate::tune::make_room_judged(mb, &sub, &res.geometry, &want, &judge);
            eprintln!(
                "room for meanders: {} nets and pairs, {}, {:.1} s",
                want.len(),
                if made.is_some() {
                    "made"
                } else {
                    "none without new violations"
                },
                t0.elapsed().as_secs_f64()
            );
            if let Some(g) = made {
                res.geometry = g;
            }
        }
    }
    // the 45 degree grid on the wide tracks: the two tracks of a pair follow it side by side
    if cfg.angles45 {
        let moved;
        let mb = if res.offsets.is_empty() {
            &merged.board
        } else {
            moved = merged.board.with_offsets(&res.offsets);
            &moved
        };
        let wide: Vec<NetId> = merged.pairs.iter().map(|p| p.p).collect();
        // the split decides at the pads and at the vias how the two tracks leave the wide one
        // (exit, via pair, polarity): the wide tracks keep their direction there
        let mut held: HashMap<usize, (usize, usize)> = HashMap::new();
        for (ti, t) in res.geometry.tracks.iter_mut().enumerate() {
            if !wide.contains(&t.net) || t.elems.is_empty() {
                continue;
            }
            let r = (3.0 * t.width).max(1.0);
            let total = elems_len(&t.elems);
            if total <= 2.0 * r + 0.01 {
                held.insert(ti, (usize::MAX, 0));
                continue;
            }
            let head = cut_end(&t.elems, total - r);
            let mid = cut_end(&cut_start(&t.elems, r), r);
            let tail = cut_start(&t.elems, total - r);
            let (h, m) = (head.len(), mid.len());
            t.elems = head.into_iter().chain(mid).chain(tail).collect();
            t.pts = flatten(&t.elems);
            held.insert(ti, (h, h + m));
        }
        // the outer track of a 45 degree corner is mitred: its corner sticks out of the round
        // corner of the wide track by s (1 / cos 22.5 deg - 1), which the wide track carries
        // while it goes onto the grid
        let widths: Vec<f64> = res.geometry.tracks.iter().map(|t| t.width).collect();
        for t in res.geometry.tracks.iter_mut() {
            if let Some(mp) = merged.pairs.iter().find(|mp| mp.p == t.net) {
                let (w, g, _) = real.pair_shape(mp.p, mp.n, t.layer);
                let s = (w + g) / 2.0;
                let out = s * (1.0 / (std::f64::consts::PI / 8.0).cos() - 1.0);
                t.width = t.width.max(2.0 * (s + out + w / 2.0) + 0.002);
            }
        }
        let hold = |ti: usize, ei: usize| {
            held.get(&ti)
                .is_some_and(|&(h, m)| h == usize::MAX || ei < h || ei >= m)
        };
        // all tracks: a bundle makes room for its wide tracks only when its other tracks go
        // onto the grid as well
        let st = crate::octo::octilinear_only(mb, &mut res.geometry, &|_| true, &hold);
        for (t, w) in res.geometry.tracks.iter_mut().zip(widths) {
            t.width = w;
        }
        eprintln!(
            "45 degrees on the wide tracks: {} tracks, {} pieces off the grid ({:.1} mm)",
            st.tracks, st.any_angle, st.any_angle_length
        );
    }
    let Split {
        geo,
        mut reports,
        unfinished,
        paths,
        ..
    } = split(&real, &merged, &res.geometry);
    for (p, why) in &merged.skipped {
        reports.push(PairReport {
            p: p.clone(),
            note: why.clone(),
            ..Default::default()
        });
    }
    // what the wide tracks left open, around everything that is there now
    let carriers: Vec<String> = merged
        .pairs
        .iter()
        .flat_map(|p| {
            [
                real.nets[p.p as usize].name.clone(),
                real.nets[p.n as usize].name.clone(),
            ]
        })
        .collect();
    let mut unrouted: Vec<String> = res
        .stats
        .unrouted
        .iter()
        .filter(|u| !carriers.iter().any(|c| u.starts_with(&format!("{c} from "))))
        .cloned()
        .collect();
    let mut extra_conns = merged.pairs.len();
    let finish = SearchConfig {
        population: search.population.min(6),
        generations: 1,
        geometry_generations: 0,
        frames: false,
        progress: None,
        seed: search.seed,
        open_end: None,
        air: 0.0,
        prefer_width: 0.0,
        open_via_cost: crate::DEFAULT_OPEN_VIA_COST,
    };
    // a repaired end that finds no way gives up its pair: its two nets are routed as single
    // tracks, so that no crossed or touching tracks of the pair stay. If they find no way
    // either, the ends that found no way keep the two tracks they had: completion comes first
    let split_geo = geo;
    let mut given_up: Vec<usize> = Vec::new();
    let mut failed_ends: Vec<(usize, bool)> = Vec::new();
    let mut keep: Vec<(usize, bool)> = Vec::new();
    let mut worse_tried: Vec<usize> = Vec::new();
    // the joins of the pairs (see [`Merged::joins`]) but those that found no short way: the
    // last pass connects their pins along with the rest of their nets
    let mut joins_off: Vec<(usize, usize)> = Vec::new();
    let (geo_done, fin) = loop {
        let mut geo = split_geo.clone();
        let (repaired, cuts) = repair(&real, &mut geo, &paths, &given_up, &keep);
        let mut names = unfinished.clone();
        names.extend(repaired);
        // a pair that is given up gets no joins: its single tracks need those pads
        let gone: Vec<NetId> = given_up.iter().flat_map(|&pi| [paths[pi].p, paths[pi].n]).collect();
        let joins: Vec<(usize, usize)> = merged
            .joins
            .iter()
            .copied()
            .filter(|&(a, c)| !joins_off.contains(&(a, c)) && !gone.contains(&real.pins[a].net))
            .collect();
        names.extend(
            joins
                .iter()
                .map(|&(a, _)| real.nets[real.pins[a].net as usize].name.clone()),
        );
        names.sort();
        names.dedup();
        if names.is_empty() {
            break (geo, None);
        }
        let mut b = real.only_nets(&names, &Default::default());
        let kept_from = b.obstacles.len();
        lock_geometry(&mut b, &geo);
        let first_cut = b.nets.len();
        let mut cut_map = cut_nets(&mut b, &cuts, kept_from);
        let first_join = b.nets.len();
        let join_map = join_nets(&mut b, &joins, kept_from, first_cut);
        let (_, mut fin) = route_board(&b, sub_cfg, base, &finish);
        // a join that finds no way, or only a long way round, is left to its net
        let failed_joins: Vec<(usize, usize)> = (first_join..b.nets.len())
            .filter(|&id| {
                let nm = &b.nets[id].name;
                let routed: f64 = fin
                    .geometry
                    .tracks
                    .iter()
                    .filter(|t| t.net == id as NetId)
                    .map(track_len)
                    .sum();
                let (a, c) = (b.nets[id].pins[0], b.nets[id].pins[1]);
                fin.stats.unrouted.iter().any(|u| u.starts_with(&format!("{nm} from ")))
                    || routed > JOIN_DETOUR * b.pins[a].pos.dist(b.pins[c].pos) + JOIN_SLACK
            })
            .filter_map(|id| joins.iter().copied().find(|&(a, c)| b.nets[id].pins == [a, c]))
            .collect();
        if std::env::var("GB_PAIR_LOG").is_ok() && !failed_joins.is_empty() {
            eprintln!("joins without a short way: {failed_joins:?}");
        }
        if !failed_joins.is_empty() {
            joins_off.extend(failed_joins);
            continue;
        }
        cut_map.extend(join_map);
        // a cut that found no way, or only a long way round (which would set the aim of its
        // length group), counts as failed
        let detour = |i: usize, c: &Cut| {
            let id = (first_cut + i) as NetId;
            let routed: f64 = fin.geometry.tracks.iter().filter(|t| t.net == id).map(track_len).sum();
            routed > 3.0 * real.pins[c.pad].pos.dist(c.at) + 3.0
        };
        let failed: Vec<(usize, bool)> = cuts
            .iter()
            .enumerate()
            .filter(|&(i, c)| fin.stats.unrouted.iter().any(|u| u.contains(&format!("~cut{i} from "))) || detour(i, c))
            .map(|(_, c)| (c.path, c.at_start))
            .collect();
        if !failed.is_empty() {
            for (pi, at_start) in failed {
                if keep.iter().any(|&(k, _)| k == pi) {
                    // a path that already keeps an end: the other end keeps its tracks too
                    keep.push((pi, at_start));
                } else {
                    given_up.push(pi);
                }
                failed_ends.push((pi, at_start));
            }
            continue;
        }
        let lost: Vec<usize> = given_up
            .iter()
            .copied()
            .filter(|&pi| {
                let names = [
                    &real.nets[paths[pi].p as usize].name,
                    &real.nets[paths[pi].n as usize].name,
                ];
                fin.stats
                    .unrouted
                    .iter()
                    .any(|u| names.iter().any(|n| u.starts_with(&format!("{n} from "))))
            })
            .collect();
        if !lost.is_empty() {
            given_up.retain(|pi| !lost.contains(pi));
            keep.extend(failed_ends.iter().filter(|(pi, _)| lost.contains(pi)));
            continue;
        }
        for t in fin.geometry.tracks.iter_mut() {
            t.net = cut_map.get(&t.net).copied().unwrap_or(t.net);
        }
        for v in fin.geometry.vias.iter_mut() {
            v.net = cut_map.get(&v.net).copied().unwrap_or(v.net);
        }
        // the search completes before it keeps its distance: where it joined the pins without
        // partner (the second pads of a USB-C socket) along the pair, onto the partner or
        // foreign copper, the pair gives up its wide track
        // (once: a pair whose single tracks find no way either keeps it)
        let worse = worse_paths(
            &real,
            &geo,
            &fin.geometry,
            &paths,
            &[&given_up[..], &worse_tried].concat(),
        );
        if !worse.is_empty() {
            worse_tried.extend(&worse);
            given_up.extend(worse);
            continue;
        }
        break (geo, Some(fin));
    };
    let mut geo = geo_done;
    given_up.sort_unstable();
    given_up.dedup();
    for &pi in &given_up {
        let r = &mut reports[paths[pi].report];
        r.wide = false;
        r.note = if worse_tried.contains(&pi) {
            "routed as single tracks: the pins without partner found no clean way to the pair"
        } else {
            "routed as single tracks: an end of the wide track found no way to its pads"
        }
        .to_string();
    }
    let mut paths: Vec<PairPath> = paths
        .into_iter()
        .enumerate()
        .filter(|(i, _)| !given_up.contains(i))
        .map(|(_, p)| p)
        .collect();
    if let Some(fin) = fin {
        for path in paths.iter_mut().filter(|p| p.whole) {
            let start = geo.tracks.len();
            path.tracks.extend(
                fin.geometry
                    .tracks
                    .iter()
                    .enumerate()
                    .filter(|(_, t)| t.net == path.p || t.net == path.n)
                    .map(|(i, _)| start + i),
            );
        }
        geo.tracks.extend(fin.geometry.tracks);
        geo.vias.extend(fin.geometry.vias);
        extra_conns += fin.stats.conns;
        unrouted.extend(fin.stats.unrouted.iter().cloned());
    }
    for (r, skew) in tune(&real, &mut geo, &paths, cfg.skew) {
        reports[r].skew = reports[r].skew.max(skew);
    }
    geo.tracks.retain(|t| t.pts.len() >= 2);
    for rep in reports.iter_mut().filter(|r| r.wide) {
        if let (Some(p), Some(n)) = (real.net_id(&rep.p), real.net_id(&rep.n)) {
            rep.length_p = crate::matching::net_length(&real, &geo, p).total();
            rep.length_n = crate::matching::net_length(&real, &geo, n).total();
            rep.skew = rep.length_p - rep.length_n;
            let (c, t) = crate::matching::coupling(&real, &geo, p, n, crate::matching::gap_rule(&real, p));
            rep.uncoupled = (t - c).max(0.0);
            if let Some(z) = real.pair_impedance(p, n) {
                rep.zdiff = (z * 10.0).round() / 10.0;
            }
        }
    }
    res.stats.conns += extra_conns;
    res.stats.routed = res.stats.conns - unrouted.len();
    res.stats.unrouted = unrouted;
    res.stats.vias = geo.vias.len();
    res.stats.length = geo
        .tracks
        .iter()
        .map(|t| t.pts.windows(2).map(|w| w[0].dist(w[1])).sum::<f64>())
        .sum();
    res.geometry = geo;
    (sub, res, reports)
}

/// The paths (but those in `skip`) of pairs with pins without partner whose nets have more
/// clearance spots with the tracks of the last pass `fin` than without them.
fn worse_paths(real: &Board, geo: &Geometry, fin: &Geometry, paths: &[PairPath], skip: &[usize]) -> Vec<usize> {
    // only pairs with pins without partner: a pass that only repairs the ends of a pair is
    // better than the pair given up
    let extra = |path: &PairPath| real.nets[path.p as usize].pins.len() + real.nets[path.n as usize].pins.len() > 4;
    let open: Vec<usize> = (0..paths.len())
        .filter(|&pi| !skip.contains(&pi) && extra(&paths[pi]))
        .collect();
    if open.is_empty() {
        return open;
    }
    let mut all = geo.clone();
    all.tracks.extend(fin.tracks.iter().cloned());
    all.vias.extend(fin.vias.iter().cloned());
    let (before, after) = (crate::check::check(real, geo), crate::check::check(real, &all));
    let count = |vs: &[crate::check::Violation], path: &PairPath| {
        let of: Vec<crate::check::Violation> = vs
            .iter()
            .filter(|v| [path.p, path.n].iter().any(|&x| v.nets.0 == x || v.nets.1 == Some(x)))
            .cloned()
            .collect();
        crate::check::spots(&of).len()
    };
    open.into_iter()
        .filter(|&pi| count(&after, &paths[pi]) > count(&before, &paths[pi]))
        .collect()
}

/// Farthest the tracks of a pair may be apart for [`zip`] to close them up, in mm.
pub const ZIP_REACH: f64 = 1.5;

/// Point spacing of the tracks [`zip`] reshapes, in mm.
const ZIP_STEP: f64 = 0.05;

/// Around a clearance problem that closing up made, the track of [`zip`] stays where it was
/// within this distance in mm (and comes over in a 45 degree ramp beyond).
const ZIP_KEEP: f64 = 0.3;

/// Shortest stretch in mm that [`zip`] moves, ramps included.
const ZIP_MIN_RUN: f64 = 1.0;

/// Tries of [`zip`] per track: each one keeps the track where it was around the clearance
/// problems of the try before, twice as far around a place that has one again.
const ZIP_TRIES: usize = 10;

/// What [`zip`] did to a pair.
#[derive(Debug, Clone, Default, serde::Serialize)]
pub struct ZipReport {
    pub p: String,
    pub n: String,
    /// Tracks that now run at the pair's gap beside their partner (all or part of the way).
    pub zipped: usize,
    /// Where a track stays apart from its partner because closing up would break a clearance
    /// (other copper between the two, a via or pad of the partner): a point of each place.
    pub kept_apart: Vec<Point>,
}

/// From q to the nearest point of the polyline, if that point lies inside it (not on one of its
/// two ends, where the track turns off to a pad or via).
fn inner_offset(poly: &[Point], q: Point) -> Option<(f64, Point)> {
    let mut best: Option<(f64, Point, bool)> = None;
    let last = poly.len().checked_sub(2)?;
    for (i, w) in poly.windows(2).enumerate() {
        let ab = w[1] - w[0];
        let l2 = ab.len2();
        let t = if l2 > 0.0 {
            ((q - w[0]).dot(ab) / l2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let o = q - (w[0] + ab * t);
        let d = o.len();
        let end = (i == 0 && t <= 0.0) || (i == last && t >= 1.0);
        if best.is_none_or(|b| d < b.0) {
            best = Some((d, o, end));
        }
    }
    best.filter(|b| !b.2).map(|b| (b.0, b.1))
}

/// Shift wanted per point of `pts` (spaced [`ZIP_STEP`]) to run at `pitch` beside the anchors,
/// zero where no anchor is within [`ZIP_REACH`] and where the point or its place beside the anchor
/// lies within the radius of a `keep` point. The
/// ends stay where they are, a shift grows by at most the way along the track from a point
/// that stays (45 degrees at most), and stretches shorter than [`ZIP_MIN_RUN`] stay as well.
pub(crate) fn zip_shifts(pts: &[Point], anchors: &[&[Point]], pitch: f64, keep: &[(Point, f64)]) -> Vec<Point> {
    let mut want: Vec<Point> = pts
        .iter()
        .map(|&q| {
            let v = anchors
                .iter()
                .filter_map(|a| inner_offset(a, q))
                .min_by(|a, b| a.0.total_cmp(&b.0))
                .filter(|&(d, _)| d > pitch + 1e-3 && d <= ZIP_REACH)
                .map_or(pt(0.0, 0.0), |(d, o)| o * ((pitch - d) / d));
            // a place to keep counts where the point is and where it would go
            if keep.iter().any(|&(k, r)| k.dist(q) < r || k.dist(q + v) < r) {
                return pt(0.0, 0.0);
            }
            v
        })
        .collect();
    for end in [0, want.len().saturating_sub(1)] {
        if let Some(v) = want.get_mut(end) {
            *v = pt(0.0, 0.0);
        }
    }
    let mut cap: Vec<f64> = want.iter().map(|v| v.len()).collect();
    for i in 1..pts.len() {
        cap[i] = cap[i].min(cap[i - 1] + pts[i].dist(pts[i - 1]));
    }
    for i in (0..pts.len().saturating_sub(1)).rev() {
        cap[i] = cap[i].min(cap[i + 1] + pts[i].dist(pts[i + 1]));
    }
    for (v, c) in want.iter_mut().zip(cap) {
        let l = v.len();
        if l > c {
            *v = if c > 0.0 { *v * (c / l) } else { pt(0.0, 0.0) };
        }
    }
    // a short move between places that stay is a dent, not a coupled stretch
    let mut i = 0;
    while i < want.len() {
        if want[i].len() < 1e-6 {
            i += 1;
            continue;
        }
        let j = (i..want.len()).find(|&k| want[k].len() < 1e-6).unwrap_or(want.len());
        let len: f64 = pts[i.saturating_sub(1)..j.min(pts.len() - 1) + 1]
            .windows(2)
            .map(|w| w[0].dist(w[1]))
            .sum();
        if len < ZIP_MIN_RUN {
            want[i..j].fill(pt(0.0, 0.0));
        }
        i = j;
    }
    want
}

/// Where the violations that involve net n are.
fn spots_of(board: &Board, geo: &Geometry, n: NetId) -> Vec<Point> {
    crate::check::check(board, geo)
        .iter()
        .filter(|v| v.nets.0 == n || v.nets.1 == Some(n))
        .map(|v| v.at)
        .collect()
}

/// Close up the differential pairs routed as two nets with `pair_pull`: the search keeps the
/// two nets in one channel on one layer, but each track is a band of its own and lies where its
/// own obstacles leave it, often half a millimetre or more from its partner. Where a track runs
/// within [`ZIP_REACH`] of its partner on the same layer, it moves over to the pair's gap, with
/// 45 degree ramps to where it stays: its ends at pads and vias, where other tracks of its net
/// end on it, where the partner is farther, and around every place where the move would give
/// its net a clearance violation more (other copper between the two, a via or pad of the
/// partner). Such places are reported in [`ZipReport::kept_apart`].
pub fn zip(board: &Board, geo: &mut Geometry) -> Vec<ZipReport> {
    let mut out = Vec::new();
    for (a, b) in board.diff_pairs() {
        let mut rep = ZipReport {
            p: board.nets[a as usize].name.clone(),
            n: board.nets[b as usize].name.clone(),
            ..Default::default()
        };
        // where a track moved, it is the partner's anchor no more: only the pieces it kept
        // pull the partner over (None: the whole track stays as it was)
        let mut kept: Vec<Option<Vec<Vec<Point>>>> = vec![None; geo.tracks.len()];
        for (mover, anchor) in [(b, a), (a, b)] {
            for ti in 0..geo.tracks.len() {
                let t = &geo.tracks[ti];
                if t.net != mover || t.resting || t.pattern.is_some() || t.pts.len() < 2 {
                    continue;
                }
                let (layer, width) = (t.layer, t.width);
                let others: Vec<usize> = (0..geo.tracks.len())
                    .filter(|&oi| geo.tracks[oi].net == anchor && geo.tracks[oi].layer == layer)
                    .collect();
                let Some(w_anchor) = others.first().map(|&oi| geo.tracks[oi].width) else {
                    continue;
                };
                let anchors: Vec<Vec<Point>> = others
                    .iter()
                    .flat_map(|&oi| match &kept[oi] {
                        Some(pieces) => pieces.clone(),
                        None => vec![geo.tracks[oi].pts.clone()],
                    })
                    .collect();
                let refs: Vec<&[Point]> = anchors.iter().map(|v| v.as_slice()).collect();
                let (_, gap, _) = board.pair_shape(a, b, layer);
                let pitch = (width + w_anchor) / 2.0 + gap;
                // where other tracks of the net end on this one
                let mut keep: Vec<(Point, f64)> = geo
                    .tracks
                    .iter()
                    .enumerate()
                    .filter(|&(oi, o)| oi != ti && o.net == mover)
                    .flat_map(|(_, o)| [o.pts[0], o.pts[o.pts.len() - 1]])
                    .map(|p| (p, ZIP_STEP))
                    .collect();
                let old = geo.tracks[ti].clone();
                let base = crate::relax::resample(&old.pts, ZIP_STEP);
                let before = spots_of(board, geo, mover);
                let mut done: Option<Vec<Vec<Point>>> = None;
                for _ in 0..ZIP_TRIES {
                    let shifts = zip_shifts(&base, &refs, pitch, &keep);
                    if shifts.iter().all(|v| v.len() < 1e-6) {
                        break;
                    }
                    let moved: Vec<Point> = base.iter().zip(&shifts).map(|(&p, &v)| p + v).collect();
                    let pts = crate::relax::simplify(&moved, 0.0005);
                    let t = &mut geo.tracks[ti];
                    t.elems = pts.windows(2).map(|w| Elem::Line(w[0], w[1])).collect();
                    t.pts = pts;
                    let after = spots_of(board, geo, mover);
                    let new: Vec<Point> = after
                        .iter()
                        .copied()
                        .filter(|p| !before.iter().any(|q| q.dist(*p) < ZIP_STEP))
                        .collect();
                    if after.len() <= before.len() && new.is_empty() {
                        let mut pieces: Vec<Vec<Point>> = vec![Vec::new()];
                        for (&p, v) in base.iter().zip(&shifts) {
                            if v.len() < 1e-6 {
                                pieces.last_mut().unwrap().push(p);
                            } else if !pieces.last().unwrap().is_empty() {
                                pieces.push(Vec::new());
                            }
                        }
                        pieces.retain(|p| p.len() >= 2);
                        done = Some(pieces);
                        break;
                    }
                    geo.tracks[ti] = old.clone();
                    if new.is_empty() {
                        break;
                    }
                    for p in new {
                        // the same place again: the ramp beside it was too steep, keep more
                        match keep
                            .iter_mut()
                            .find(|k| k.1 >= ZIP_KEEP && k.0.dist(p) < k.1 + ZIP_KEEP)
                        {
                            Some(k) => k.1 = (k.1 * 2.0).min(ZIP_REACH),
                            None => {
                                rep.kept_apart.push(p);
                                keep.push((p, ZIP_KEEP));
                            }
                        }
                    }
                }
                if done.is_some() {
                    kept[ti] = done;
                    rep.zipped += 1;
                }
            }
        }
        out.push(rep);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{Layer, Net, NetClass};

    fn close(a: Point, b: Point) -> bool {
        a.dist(b) < 1e-9
    }

    /// Two nets with a pad each at both ends of a free board.
    fn pair_board() -> Board {
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
            track_width: 0.2,
            clearance: 0.1,
            via_diameter: 0.6,
            via_drill: 0.3,
            diff_gap: 0.15,
            ..Default::default()
        }];
        for (i, name) in ["D_P", "D_N"].iter().enumerate() {
            let mut net = Net {
                name: name.to_string(),
                class: 0,
                pins: Vec::new(),
                planes: LayerSet::default(),
                priority: 0,
            };
            for (j, x) in [4.0, 26.0].into_iter().enumerate() {
                let (id, o) = (b.pins.len(), b.obstacles.len());
                let at = pt(x, 5.0 + 0.8 * i as f64);
                let (lo, hi) = (at + pt(-0.6, -0.15), at + pt(0.6, 0.15));
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
                    size: (0.3, 1.2),
                });
                net.pins.push(id);
            }
            b.nets.push(net);
        }
        b
    }

    fn polyline(pts: &[Point], net: NetId) -> Track {
        make_track(pts.windows(2).map(|w| Elem::Line(w[0], w[1])).collect(), 0.2, 0, net)
    }

    /// A USB 2.0 pair from the double pad row of a USB-C socket (B7 A6 A7 B6, between the
    /// pads of two other nets) to the two pins of a controller. Returns the board and the pins
    /// by name.
    fn usb_board() -> (Board, HashMap<String, usize>) {
        let mut b = pair_board();
        b.obstacles.clear();
        b.pins.clear();
        b.nets.clear();
        b.outline = vec![Polygon::new(vec![pt(0., 0.), pt(40., 0.), pt(40., 16.), pt(0., 16.)])];
        let mut usb = b.classes[0].clone();
        usb.name = "USB".into();
        b.classes.push(usb);
        let mut names = HashMap::new();
        let mut pad = |b: &mut Board, name: &str, net: &str, at: Point, size: (f64, f64)| {
            let ni = match b.net_id(net) {
                Some(i) => i,
                None => {
                    b.nets.push(Net {
                        name: net.to_string(),
                        class: if net.starts_with("USB") { 1 } else { 0 },
                        pins: Vec::new(),
                        planes: LayerSet::default(),
                        priority: 0,
                    });
                    (b.nets.len() - 1) as NetId
                }
            };
            let (id, o) = (b.pins.len(), b.obstacles.len());
            let (lo, hi) = (at - pt(size.0, size.1) * 0.5, at + pt(size.0, size.1) * 0.5);
            b.obstacles.push(Obstacle {
                shape: Shape::Polygon(Polygon::new(vec![lo, pt(hi.x, lo.y), hi, pt(lo.x, hi.y)])),
                layers: LayerSet::single(0),
                net: Some(ni),
                kind: ObstacleKind::Pad,
                hole: None,
                clearance: 0.1,
                pin: Some(id),
                locked: false,
            });
            b.pins.push(Pin {
                name: name.to_string(),
                footprint: name.split('.').next().unwrap().to_string(),
                net: ni,
                pos: at,
                layers: LayerSet::single(0),
                obstacles: vec![o],
                size: (size.1, size.0),
            });
            b.nets[ni as usize].pins.push(id);
            names.insert(name.to_string(), id);
        };
        let row = (1.2, 0.3);
        for (name, net, y) in [
            ("J1.A5", "CC1", 6.75),
            ("J1.B7", "USB_N", 7.25),
            ("J1.A6", "USB_P", 7.75),
            ("J1.A7", "USB_N", 8.25),
            ("J1.B6", "USB_P", 8.75),
            ("J1.A8", "SBU1", 9.25),
        ] {
            pad(&mut b, name, net, pt(3.0, y), row);
        }
        pad(&mut b, "U10.13", "USB_P", pt(34.0, 7.8), row);
        pad(&mut b, "U10.14", "USB_N", pt(34.0, 8.6), row);
        (b, names)
    }

    #[test]
    fn the_second_row_of_a_usb_c_socket_joins_the_pair_at_the_socket() {
        let (b, pin) = usb_board();
        let m = merge(&b, true, false);
        assert_eq!(m.pairs.len(), 1, "{:?}", m.skipped);
        // the band ends on the pads of the row that keep its polarity towards the controller
        let e = m.pairs[0].ends[0];
        assert_eq!([e.p, e.n], [pin["J1.A6"], pin["J1.A7"]]);
        let mut joins = m.joins.clone();
        joins.sort_unstable();
        let mut want = vec![(pin["J1.B6"], pin["J1.A6"]), (pin["J1.B7"], pin["J1.A7"])];
        want.sort_unstable();
        assert_eq!(joins, want);
        let search = SearchConfig {
            population: 2,
            generations: 0,
            geometry_generations: 0,
            frames: false,
            progress: None,
            seed: None,
            open_end: None,
            air: 0.0,
            prefer_width: 0.0,
            open_via_cost: crate::DEFAULT_OPEN_VIA_COST,
        };
        let (_, res, rep) = route_pairs(
            &b,
            &SubstrateConfig::default(),
            &RouterConfig::default(),
            &search,
            &PairConfig {
                all: true,
                ..PairConfig::default()
            },
        );
        assert!(res.stats.unrouted.is_empty(), "{:?}", res.stats.unrouted);
        assert_eq!(rep.len(), 1, "{rep:?}");
        assert!(rep[0].wide, "{:?}", rep[0]);
        let viol = crate::check::check(&b, &res.geometry);
        assert!(viol.is_empty(), "{viol:?}");
        // the joins stay at the socket: no single track of a pair net reaches far from it
        for t in &res.geometry.tracks {
            let at_socket = t.pts.iter().all(|q| q.x < 5.0);
            let long = track_len(t) > 20.0;
            assert!(at_socket || long, "{:?}", t.pts);
        }
        // coupled from the socket on: not split as crossed polarity (the other pairing of the
        // row), which leaves the whole pair uncoupled
        assert!(rep[0].width > 0.0 && rep[0].uncoupled < 10.0, "{:?}", rep[0]);
    }

    #[test]
    fn a_pair_whose_extra_pins_are_joined_onto_its_partner_gives_up() {
        let mut b = pair_board();
        let path = PairPath {
            report: 0,
            p: 0,
            n: 1,
            tracks: Vec::new(),
            vias: Vec::new(),
            whole: false,
            ends: [PairEnd { p: 0, n: 2, out: None }, PairEnd { p: 1, n: 3, out: None }],
        };
        // the pair's two tracks, and a last pass that runs a track of D_P on top of D_N
        let geo = Geometry {
            tracks: vec![
                polyline(&[pt(4., 5.), pt(26., 5.)], 0),
                polyline(&[pt(4., 5.8), pt(26., 5.8)], 1),
            ],
            vias: Vec::new(),
        };
        let fin = Geometry {
            tracks: vec![polyline(&[pt(8., 5.9), pt(20., 5.9)], 0)],
            vias: Vec::new(),
        };
        // two pins each: the pass only repaired the ends, the pair stays
        assert!(worse_paths(&b, &geo, &fin, std::slice::from_ref(&path), &[]).is_empty());
        // a third pin of D_P (the second pad of a USB-C socket)
        b.nets[0].pins.push(b.pins.len());
        b.pins.push(b.pins[0].clone());
        assert_eq!(worse_paths(&b, &geo, &fin, std::slice::from_ref(&path), &[]), vec![0]);
        assert!(worse_paths(&b, &geo, &fin, std::slice::from_ref(&path), &[0]).is_empty());
    }

    #[test]
    fn zip_closes_up_a_pair_to_its_gap_except_where_copper_is_between() {
        let mut b = pair_board();
        // a keep-out of no net between the two tracks around x = 15
        let (lo, hi) = (pt(14.8, 5.45), pt(15.2, 6.05));
        b.obstacles.push(Obstacle {
            shape: Shape::Polygon(Polygon::new(vec![lo, pt(hi.x, lo.y), hi, pt(lo.x, hi.y)])),
            layers: LayerSet::single(0),
            net: None,
            kind: ObstacleKind::Pad,
            hole: None,
            clearance: 0.1,
            pin: None,
            locked: false,
        });
        // D_P straight, D_N 1.4 mm away from it in the middle, as two separate bands leave them
        let mut geo = Geometry {
            tracks: vec![
                polyline(&[pt(4.0, 5.0), pt(26.0, 5.0)], 0),
                polyline(&[pt(4.0, 5.8), pt(5.0, 6.4), pt(25.0, 6.4), pt(26.0, 5.8)], 1),
            ],
            vias: Vec::new(),
        };
        assert!(crate::check::check(&b, &geo).is_empty());
        let before = crate::pair_coupling_of(&geo, 0, 1, 0.5).0;
        let rep = zip(&b, &mut geo);
        assert!(
            crate::check::check(&b, &geo).is_empty(),
            "{:?}",
            crate::check::check(&b, &geo)
        );
        assert_eq!(rep.len(), 1);
        assert_eq!(rep[0].zipped, 1, "{:?}", rep[0]);
        // stays apart at the keep-out, and says where
        assert!(
            rep[0].kept_apart.iter().any(|p| p.dist(pt(15.0, 6.0)) < 1.0),
            "{:?}",
            rep[0]
        );
        let n = &geo.tracks[1];
        let y_at = |x: f64| {
            n.pts
                .windows(2)
                .find(|w| (w[0].x - x) * (w[1].x - x) <= 0.0 && w[0].x != w[1].x)
                .map(|w| w[0].y + (w[1].y - w[0].y) * (x - w[0].x) / (w[1].x - w[0].x))
                .unwrap()
        };
        // at the pair's pitch (track width and the class gap) elsewhere
        let pitch = 0.2 + 0.15;
        for x in [8.0, 11.0, 19.0, 22.0] {
            assert!((y_at(x) - 5.0 - pitch).abs() < 0.01, "{x}: {}", y_at(x));
        }
        assert!(y_at(15.0) > 6.05 + 0.1 + 0.1 - 1e-6, "{}", y_at(15.0));
        // the ends stay on their pads
        assert!(close(n.pts[0], pt(4.0, 5.8)) && close(*n.pts.last().unwrap(), pt(26.0, 5.8)));
        // D_N runs coupled for most of its 22 mm (D_P is one piece, counted by its middle)
        let after = crate::pair_coupling_of(&geo, 0, 1, 0.5).0;
        assert!(before == 0.0 && after > 15.0, "{before} -> {after}");
    }

    #[test]
    fn a_pair_becomes_two_parallel_tracks_of_equal_length() {
        let b = pair_board();
        assert_eq!(merge(&b, true, false).pairs.len(), 1);
        for breakouts in [true, false] {
            let search = SearchConfig {
                population: 2,
                generations: 0,
                geometry_generations: 0,
                frames: false,
                progress: None,
                seed: None,
                open_end: None,
                air: 0.0,
                prefer_width: 0.0,
                open_via_cost: crate::DEFAULT_OPEN_VIA_COST,
            };
            let (_, res, rep) = route_pairs(
                &b,
                &SubstrateConfig::default(),
                &RouterConfig::default(),
                &search,
                &PairConfig {
                    all: true,
                    breakouts,
                    ..PairConfig::default()
                },
            );
            assert!(res.stats.unrouted.is_empty(), "{:?}", res.stats.unrouted);
            assert_eq!(rep.len(), 1);
            assert!(rep[0].wide && rep[0].note.is_empty(), "{:?}", rep[0]);
            assert!((rep[0].length_p - rep[0].length_n).abs() <= 0.1, "{:?}", rep[0]);
            assert_eq!(res.geometry.tracks.iter().filter(|t| t.net == 0).count(), 1);
            assert_eq!(res.geometry.tracks.iter().filter(|t| t.net == 1).count(), 1);
            assert!(crate::check::check(&b, &res.geometry).is_empty());
        }
    }

    /// With the pads of one end on the bottom the band changes layers: the vias stand side by
    /// side, and as a via pair they take the capsule around them instead of the circle.
    #[test]
    fn vias_side_by_side_become_a_via_pair() {
        let mut b = pair_board();
        for pin in [1, 3] {
            b.pins[pin].layers = LayerSet::single(1);
            let o = b.pins[pin].obstacles[0];
            b.obstacles[o].layers = LayerSet::single(1);
        }
        let search = SearchConfig {
            population: 2,
            generations: 0,
            geometry_generations: 0,
            frames: false,
            progress: None,
            seed: None,
            open_end: None,
            air: 0.0,
            prefer_width: 0.0,
            open_via_cost: crate::DEFAULT_OPEN_VIA_COST,
        };
        let m = merge(&b, true, false);
        let (_, mut res) = route_board(&m.board, &SubstrateConfig::default(), &RouterConfig::default(), &search);
        let circle = res
            .geometry
            .vias
            .iter()
            .find(|v| v.net == m.pairs[0].p)
            .unwrap()
            .diameter;
        assert_eq!(pair_vias(&b, &m, &mut res.geometry), 1);
        let v = res.geometry.vias.iter().find(|v| v.net == m.pairs[0].p).unwrap();
        // two vias of 0.6 mm with their gap: 0.6 + 0.15 + 0.6 long, 0.6 wide
        assert!(v.is_pair() && v.diameter < circle, "{v:?} {circle}");
        let (_, res, rep) = route_pairs(
            &b,
            &SubstrateConfig::default(),
            &RouterConfig::default(),
            &search,
            &PairConfig {
                all: true,
                via_pairs: true,
                ..PairConfig::default()
            },
        );
        assert!(res.stats.unrouted.is_empty(), "{:?}", res.stats.unrouted);
        assert!(rep[0].wide && rep[0].vias == 1 && rep[0].swaps == 0, "{:?}", rep[0]);
        assert!(
            crate::check::check(&b, &res.geometry).is_empty(),
            "{:?}",
            crate::check::check(&b, &res.geometry)
        );
    }

    #[test]
    fn the_tracks_change_sides_where_the_band_turns_back() {
        // (turns back, via pair, shorter leg)
        let u = (true, false, 5.0);
        let on = (false, false, 5.0);
        // two turns back: both side by side, twice a change, none in line
        assert_eq!(side_changes(&[u, on, u], false), vec![true, false, true]);
        // a crossed pair with one turn back changes there
        assert_eq!(side_changes(&[on, u], true), vec![false, true]);
        // three turns back and no change wanted: one in line, at the longer legs, or where
        // the band goes on if it can
        assert_eq!(
            side_changes(&[u, u, (true, false, 9.0)], false),
            vec![true, true, false]
        );
        assert_eq!(side_changes(&[u, on, u], true), vec![true, true, true]);
        // the longer legs take the vias in line, a via pair does not
        let long = (false, false, 9.0);
        assert_eq!(side_changes(&[on, long], true), vec![false, true]);
        assert_eq!(side_changes(&[on, (false, true, 9.0)], true), vec![true, false]);
    }

    #[test]
    fn a_breakout_starts_the_wide_track_past_the_pads() {
        let b = pair_board();
        let m = merge(&b, true, true);
        assert_eq!(m.pairs[0].ends.len(), 2);
        for e in &m.pairs[0].ends {
            let c = e.out.expect("a breakout");
            let mid = (b.pins[e.p].pos + b.pins[e.n].pos) * 0.5;
            // past the tips of the pads, 0.6 mm from their centres, towards the other end
            assert!(
                (c.x - mid.x).abs() > 0.6 && (c.x - mid.x) * (15.0 - mid.x) > 0.0,
                "{c:?}"
            );
            assert!((c.y - mid.y).abs() < 1e-9, "{c:?}");
            // the wide net does not own the pads there
            assert!(
                b.pins[e.p]
                    .obstacles
                    .iter()
                    .all(|&i| m.board.obstacles[i].net.is_none())
            );
        }
        let off = merge(&b, true, false);
        assert!(off.pairs[0].ends.iter().all(|e| e.out.is_none()));
    }

    #[test]
    fn crossed_polarity_goes_through_a_via_or_stays_two_nets() {
        // the second pair end upside down: only a swap or single tracks can connect it
        let mut b = pair_board();
        let (p1, n1) = (1, 3);
        let (a, c) = (b.pins[p1].pos, b.pins[n1].pos);
        b.pins[p1].pos = c;
        b.pins[n1].pos = a;
        let (o1, o2) = (b.pins[p1].obstacles[0], b.pins[n1].obstacles[0]);
        let (s1, s2) = (b.obstacles[o1].shape.clone(), b.obstacles[o2].shape.clone());
        b.obstacles[o1].shape = s2;
        b.obstacles[o2].shape = s1;
        let search = SearchConfig {
            population: 2,
            generations: 0,
            geometry_generations: 0,
            frames: false,
            progress: None,
            seed: None,
            open_end: None,
            air: 0.0,
            prefer_width: 0.0,
            open_via_cost: crate::DEFAULT_OPEN_VIA_COST,
        };
        let (_, res, _) = route_pairs(
            &b,
            &SubstrateConfig::default(),
            &RouterConfig::default(),
            &search,
            &PairConfig {
                all: true,
                ..PairConfig::default()
            },
        );
        assert!(res.stats.unrouted.is_empty(), "{:?}", res.stats.unrouted);
    }

    #[test]
    fn impedance_falls_with_width_and_rises_with_gap() {
        let mut b = pair_board();
        b.stackup = vec![
            crate::board::StackItem {
                copper: Some(0),
                thickness: 0.035,
                epsilon_r: 1.0,
            },
            crate::board::StackItem {
                copper: None,
                thickness: 0.2,
                epsilon_r: 4.4,
            },
            crate::board::StackItem {
                copper: Some(1),
                thickness: 0.035,
                epsilon_r: 1.0,
            },
        ];
        let z = b.zdiff(0, 0.2, 0.2).unwrap();
        assert!((80.0..120.0).contains(&z), "{z}");
        assert!(b.zdiff(0, 0.3, 0.2).unwrap() < z);
        assert!(b.zdiff(0, 0.2, 0.3).unwrap() > z);
        let w = b.width_for_zdiff(0, 0.2, 90.0, 0.1).unwrap();
        assert!((b.zdiff(0, w, 0.2).unwrap() - 90.0).abs() < 1.0);
    }

    #[test]
    fn offset_of_a_line_and_an_arc_stays_parallel() {
        let (a, b) = (pt(0.0, 0.0), pt(10.0, 0.0));
        // a left turn around (10, 5)
        let m = pt(10.0, 5.0) + pt(0.0, -5.0).rotate(std::f64::consts::FRAC_PI_4);
        let c = pt(15.0, 5.0);
        let e = vec![Elem::Line(a, b), Elem::Arc(b, m, c)];
        let l = offset_curve(&e, 1.0);
        assert_eq!(l.len(), 2);
        assert!(close(elem_ends(&l[0]).0, pt(0.0, 1.0)));
        assert!(close(elem_ends(&l[1]).1, pt(14.0, 5.0)));
        assert!((elems_len(&l) - (10.0 + 4.0 * std::f64::consts::FRAC_PI_2)).abs() < 1e-9);
        let r = offset_curve(&e, -1.0);
        assert!((elems_len(&r) - (10.0 + 6.0 * std::f64::consts::FRAC_PI_2)).abs() < 1e-9);
    }

    #[test]
    fn a_sharp_corner_gets_an_arc_outside_and_a_corner_inside() {
        let e = vec![
            Elem::Line(pt(0.0, 0.0), pt(10.0, 0.0)),
            Elem::Line(pt(10.0, 0.0), pt(10.0, 10.0)),
        ];
        let inner = offset_curve(&e, 1.0);
        assert_eq!(inner.len(), 2);
        assert!(close(elem_ends(&inner[0]).1, pt(9.0, 1.0)));
        let outer = offset_curve(&e, -1.0);
        assert_eq!(outer.len(), 3);
        assert!(matches!(outer[1], Elem::Arc(..)));
    }

    #[test]
    fn a_fold_on_the_inner_side_is_cut_out() {
        // a tight zigzag: the left offset folds over itself
        let e = vec![
            Elem::Line(pt(0.0, 0.0), pt(5.0, 0.0)),
            Elem::Line(pt(5.0, 0.0), pt(5.1, 0.1)),
            Elem::Line(pt(5.1, 0.1), pt(5.2, 0.0)),
            Elem::Line(pt(5.2, 0.0), pt(10.0, 0.0)),
        ];
        let c = remove_loops(offset_curve(&e, -0.5), 3.5);
        let pts = flatten(&c);
        for i in 0..pts.len() - 1 {
            for j in i + 2..pts.len() - 1 {
                assert!(
                    line_cross(pts[i], pts[i + 1], pts[j], pts[j + 1])
                        .is_none_or(|(t, u)| { !(0.0..=1.0).contains(&t) || !(0.0..=1.0).contains(&u) })
                );
            }
        }
    }

    #[test]
    fn a_mitred_offset_stays_on_the_45_degree_grid() {
        // a 45 degree turn to the right: the left side is the outer one
        let e = vec![
            Elem::Line(pt(0.0, 0.0), pt(2.0, 0.0)),
            Elem::Line(pt(2.0, 0.0), pt(4.0, -2.0)),
        ];
        let round = offset_curve(&e, 0.2);
        assert!(round.iter().any(|x| matches!(x, Elem::Arc(..))));
        let mitred = offset_curve_with(&e, 0.2, true);
        assert_eq!(mitred.len(), 2);
        let dirs: Vec<f64> = mitred
            .iter()
            .map(|x| {
                let (a, b) = elem_ends(x);
                (b.y - a.y).atan2(b.x - a.x).to_degrees()
            })
            .collect();
        assert!(dirs[0].abs() < 1e-9 && (dirs[1] + 45.0).abs() < 1e-9);
        // both pieces keep their distance 0.2 from the centre line
        let (a, _) = elem_ends(&mitred[0]);
        assert!((a.y - 0.2).abs() < 1e-9);
    }

    #[test]
    fn a_chord_looks_past_the_first_piece() {
        let e = vec![
            Elem::Line(pt(0.0, 0.0), pt(0.1, 0.0)),
            Elem::Line(pt(0.1, 0.0), pt(0.1, 2.0)),
        ];
        let u = chord(&e, 1.0);
        assert!(u.y > 0.9);
    }

    #[test]
    fn cutting_a_curve_keeps_the_rest() {
        let e = vec![
            Elem::Line(pt(0.0, 0.0), pt(1.0, 0.0)),
            Elem::Line(pt(1.0, 0.0), pt(3.0, 0.0)),
        ];
        let c = cut_start(&e, 1.5);
        assert_eq!(c.len(), 1);
        assert!(close(point_at_start(&c), pt(1.5, 0.0)));
        let c = cut_end(&e, 0.5);
        assert!(close(point_at_end(&c), pt(2.5, 0.0)));
    }
}
